#include "sbc_in.h"

#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "sync_proto.h"
#include "sbc_link.h"
#include "sbc_decoder.h"
#include "streamer.h"

#define UART_PORT   UART_NUM_1
#define UART_RX_PIN 23          /* reuses the old I2S DATA wire */
#define UART_TX_PIN 22          /* unused: the link is one-way */

static const char *TAG = "sbc_in";

/* Statistics, reported every 5 s. These replace the rate/loss instrumentation
 * that audio_in.c needed -- with no shared clock there is no rate to measure,
 * so what matters now is whether packets arrive intact. */
static uint32_t s_packets, s_bad_sync, s_bad_crc, s_gaps, s_decode_err;
static uint64_t s_pcm_samples;

/*
 * Bulk-read into a local buffer and parse in memory.
 *
 * The first version called uart_read_bytes() one byte at a time to hunt for the
 * sync word. After a CRC error that meant rescanning ~660 bytes of a corrupt
 * packet with 660 separate driver calls, tens of thousands per second -- which
 * backed up the RX buffer and caused further corruption. A self-sustaining
 * failure, and largely why half of all packets were being lost.
 */
#define PARSE_BUF (SBC_LINK_MAX_PAYLOAD * 4)

static uint8_t s_buf[PARSE_BUF];
static size_t s_have;          /* valid bytes in s_buf */

/* Drop `n` bytes from the front. */
static void consume(size_t n)
{
    if (n >= s_have) {
        s_have = 0;
    } else {
        memmove(s_buf, s_buf + n, s_have - n);
        s_have -= n;
    }
}

/* Index of the next sync pair at or after `from`, or -1. */
static int find_sync_at(size_t from)
{
    for (size_t i = from; i + 1 < s_have; i++) {
        if (s_buf[i] == SBC_LINK_SYNC0 && s_buf[i + 1] == SBC_LINK_SYNC1) {
            return (int)i;
        }
    }
    return -1;
}

static void rx_task(void *arg)
{
    /* No staging buffer: the parser decodes in place from s_buf. */
    static int16_t pcm[SBC_MAX_PCM_SAMPLES];
    uint32_t expect_seq = 0;
    bool have_seq = false;
    int64_t next_report = 0;

    while (1) {
        /* Top up from the driver in bulk. */
        if (s_have < sizeof(s_buf)) {
            int r = uart_read_bytes(UART_PORT, s_buf + s_have, sizeof(s_buf) - s_have,
                                    pdMS_TO_TICKS(20));
            if (r > 0) {
                s_have += r;
            }
        }

        /* Drain every complete packet the buffer holds. */
        while (1) {
            int sync = find_sync_at(0);
            if (sync < 0) {
                /* Keep the last byte: a sync pair may straddle the boundary. */
                if (s_have > 1) consume(s_have - 1);
                break;
            }
            if (sync > 0) {
                s_bad_sync++;
                consume((size_t)sync);
                continue;
            }
            if (s_have < sizeof(sbc_link_hdr_t)) {
                break;                      /* header still arriving */
            }

            uint8_t kind = s_buf[2];
            uint16_t len;
            uint32_t seq;
            memcpy(&len, s_buf + 4, 2);
            memcpy(&seq, s_buf + 6, 4);
            uint8_t crc = s_buf[10];

            if (len == 0 || len > SBC_LINK_MAX_PAYLOAD) {
                s_bad_sync++;
                consume(2);                 /* false sync, skip past it */
                continue;
            }
            if (s_have < sizeof(sbc_link_hdr_t) + len) {
                break;                      /* payload still arriving */
            }

            const uint8_t *payload = s_buf + sizeof(sbc_link_hdr_t);
            if (sbc_link_checksum(payload, len) != crc) {
                s_bad_crc++;
                consume(2);                 /* resync from just after this sync */
                continue;
            }

            if (have_seq && seq != expect_seq) {
                s_gaps++;
            }
            expect_seq = seq + 1;
            have_seq = true;

            if (kind == LINK_KIND_META) {
                if (len == sizeof(link_meta_t)) {
                    const link_meta_t *m = (const link_meta_t *)payload;
                    ESP_LOGW(TAG, "TRACK #%" PRIu32 ": \"%s\" - %s [%s]",
                             m->track_id, m->title, m->artist, m->album);
                    streamer_send_meta(payload, len);

                    /* A new track is the one moment a splice is inaudible, so
                     * take it: flag the next audio packet and let every unit
                     * null its phase error when playback reaches it. Metadata is
                     * resent for the same track, so act on the id, not arrival. */
                    static uint32_t last_track_id;
                    static bool have_track;
                    if (have_track && m->track_id != last_track_id) {
                        streamer_request_restart();
                    }
                    last_track_id = m->track_id;
                    have_track = true;
                }
                consume(sizeof(sbc_link_hdr_t) + len);
                continue;
            }
            s_packets++;

            /* One A2DP packet holds several SBC frames back to back. Decode for
             * this unit's own speaker, and count frames so the streamer knows
             * how far to advance the timeline for the copy it sends on. */
            /* Tag before feeding, so the mark lands at the start of this
             * packet's audio in the ring. */
            streamer_begin_packet();

            static uint32_t mark_count;
            bool tagged = (mark_count++ % MARKER_EVERY_PKTS) == 0;
            if (tagged) {
                streamer_mark_here();
            }

            uint32_t frames_here = 0;
            size_t off = 0;
            while (off < len) {
                size_t consumed = 0, samples = 0;
                if (!sbc_decode_frame(payload + off, len - off, &consumed, pcm, &samples)) {
                    s_decode_err++;
                    sbc_decoder_init();
                    break;
                }
                if (consumed == 0) {
                    break;
                }
                off += consumed;
                s_pcm_samples += samples;
                frames_here += samples / AUDIO_CHANNELS;

                const uint8_t *bytes = (const uint8_t *)pcm;
                size_t nbytes = samples * sizeof(int16_t);
                /* The visualiser is deliberately NOT fed here. This is the
                 * arrival path, and ~200 ms of buffer sits between it and the
                 * speaker -- lights driven from here run that far ahead of the
                 * sound. streamer.c feeds it from the playback path instead. */
                streamer_feed(bytes, nbytes);
            }

            /* Satellites get the SBC itself, not what we decoded from it. */
            streamer_send_sbc(payload, len, frames_here, tagged);

            consume(sizeof(sbc_link_hdr_t) + len);
        }

        int64_t now = esp_timer_get_time();
        if (next_report == 0) {
            next_report = now + 5000000;
        } else if (now >= next_report) {
            sbc_stream_info_t info;
            sbc_decoder_get_info(&info);
            /*
             * Effective rate is the number that matters: how many PCM samples
             * per second actually reach the playback path. The link can be
             * perfectly clean and this still fall short of 44100 if frames are
             * lost after decoding.
             */
            uint32_t eff = (uint32_t)(s_pcm_samples * 1000000ULL /
                                      (uint64_t)(now - (next_report - 5000000)) / 2);
            ESP_LOGI(TAG, "pkts %" PRIu32 " | %" PRIu32 " Hz x%u | eff %" PRIu32 " Hz | "
                          "sync %" PRIu32 " crc %" PRIu32 " gaps %" PRIu32
                          " dec %" PRIu32 " | fed-drop %" PRIu32 " B",
                     s_packets, info.sample_rate, info.channels, eff,
                     s_bad_sync, s_bad_crc, s_gaps, s_decode_err,
                     streamer_take_dropped());
            s_pcm_samples = 0;
            /* Rate the decoder actually produced, which is what the DAC must match. */
            streamer_set_sample_rate(info.sample_rate ? info.sample_rate : 44100);
            /* Per-window, not cumulative: a rising total tells you far less than
             * a rate, and cumulative counters made 500 k look worse than it was. */
            s_packets = s_bad_sync = s_bad_crc = s_gaps = s_decode_err = 0;
            next_report = now + 5000000;
        }
    }
}

void sbc_in_start(void)
{
    uart_config_t cfg = {
        .baud_rate = SBC_LINK_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    /* Generous RX buffer: the bridge sends in bursts as A2DP packets arrive. */
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 8192, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    if (!sbc_decoder_init()) {
        ESP_LOGE(TAG, "SBC decoder init failed");
    }
    xTaskCreatePinnedToCore(rx_task, "sbc_in", 4096, NULL, 9, NULL, 1);
    ESP_LOGI(TAG, "SBC link listening on GPIO %d at %d baud", UART_RX_PIN, SBC_LINK_BAUD);
}
