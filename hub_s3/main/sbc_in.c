#include "sbc_in.h"

#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "sync_proto.h"
#include "sbc_link.h"
#include "sbc_decoder.h"
#include "streamer.h"

#define SPI_LINK_HOST SPI3_HOST
#define PIN_SCK  CONFIG_DANCEFLOOR_SBC_SPI_SCK_PIN
#define PIN_MOSI CONFIG_DANCEFLOOR_SBC_SPI_MOSI_PIN
#define PIN_CS   CONFIG_DANCEFLOOR_SBC_SPI_CS_PIN
#define PIN_HS   CONFIG_DANCEFLOOR_SBC_SPI_HS_PIN

#define NFRAMES 2

static const char *TAG = "sbc_in";

static uint8_t *s_frame[NFRAMES];
static spi_slave_transaction_t s_trans[NFRAMES];

static uint32_t s_packets, s_bad_hdr, s_bad_crc, s_short, s_gaps, s_decode_err, s_dec_crc;

static uint32_t s_lost;

static uint32_t s_vol;
static uint64_t s_pcm_samples;

static int64_t s_last_pkt_us;
static uint32_t s_max_gap_us;

#define GAP_ALARM_US 150000

static void IRAM_ATTR spi_post_setup(spi_slave_transaction_t *t)
{
    gpio_set_level(PIN_HS, 1);
}

static void IRAM_ATTR spi_post_trans(spi_slave_transaction_t *t)
{
    gpio_set_level(PIN_HS, 0);
}

static void rx_task(void *arg)
{
    uint32_t expect_seq = 0;
    bool have_seq = false;
    int64_t next_report = 0;

    for (int i = 0; i < NFRAMES; i++) {
        ESP_ERROR_CHECK(spi_slave_queue_trans(SPI_LINK_HOST, &s_trans[i], portMAX_DELAY));
    }

    while (1) {

        spi_slave_transaction_t *done = NULL;
        if (spi_slave_get_trans_result(SPI_LINK_HOST, &done,
                                       pdMS_TO_TICKS(20)) == ESP_OK) {

            if (done->trans_len != SBC_LINK_FRAME_BYTES * 8) {
                s_short++;
                goto rearm;
            }

            const uint8_t *frame = (const uint8_t *)done->rx_buffer;
            spi_link_hdr_t hdr;
            memcpy(&hdr, frame, sizeof(hdr));
            const uint8_t *payload = frame + sizeof(spi_link_hdr_t);

            if (hdr.len == 0 || hdr.len > SBC_LINK_MAX_PAYLOAD ||
                (hdr.kind != LINK_KIND_SBC && hdr.kind != LINK_KIND_META &&
                 hdr.kind != LINK_KIND_VOL)) {
                s_bad_hdr++;
                goto rearm;
            }
            if (sbc_link_crc16(&hdr, payload, hdr.len) != hdr.crc) {
                s_bad_crc++;
                goto rearm;
            }

            if (have_seq && hdr.seq != expect_seq) {
                s_gaps++;

                const int32_t ahead = (int32_t)(hdr.seq - expect_seq);
                if (ahead > 0) {
                    s_lost += (uint32_t)ahead;
                }
            }
            expect_seq = hdr.seq + 1;
            have_seq = true;

            if (hdr.kind == LINK_KIND_META) {
                if (hdr.len == sizeof(link_meta_t)) {
                    const link_meta_t *m = (const link_meta_t *)payload;
                    ESP_LOGW(TAG, "TRACK #%" PRIu32 ": \"%s\" - %s [%s]",
                             m->track_id, m->title, m->artist, m->album);
                    streamer_send_meta(payload, hdr.len);

                    static uint32_t last_track_id;
                    static bool have_track;
                    if (have_track && m->track_id != last_track_id) {
                        streamer_request_restart();
                    }
                    last_track_id = m->track_id;
                    have_track = true;
                }
                goto rearm;
            }

            if (hdr.kind == LINK_KIND_VOL) {
                if (hdr.len == sizeof(link_vol_t)) {
                    const link_vol_t *v = (const link_vol_t *)payload;

                    s_vol++;

                    streamer_set_volume(v->volume);
                }
                goto rearm;
            }
            s_packets++;

            {
                const int64_t t = esp_timer_get_time();
                if (s_last_pkt_us) {
                    const uint32_t gap = (uint32_t)(t - s_last_pkt_us);
                    if (gap > s_max_gap_us) {
                        s_max_gap_us = gap;
                    }
                }
                s_last_pkt_us = t;
            }

            streamer_begin_packet();

            static uint32_t mark_count;
            bool tagged = (mark_count++ % MARKER_EVERY_PKTS) == 0;
            if (tagged) {
                streamer_mark_here();
            }

            static int16_t pcm[SBC_MAX_PCM_SAMPLES];

            size_t off = 0;
            size_t span_off = 0;
            uint32_t span_frames = 0;
            bool first_span = true;
            while (off < hdr.len) {
                size_t consumed = 0, samples = 0;
                if (!sbc_decode_frame(payload + off, hdr.len - off, &consumed, pcm, &samples)) {

                    if (sbc_decoder_last_result() == SBC_DECODE_CRC) {
                        s_dec_crc++;
                    } else {
                        s_decode_err++;
                    }
                    sbc_decoder_init();
                    break;
                }
                if (consumed == 0) {
                    break;
                }

                if (span_frames > 0 &&
                    (off - span_off) + consumed > AUDIO_TX_PAYLOAD_MTU_MAX) {
                    streamer_send_sbc(payload + span_off,
                                      (uint16_t)(off - span_off),
                                      span_frames, first_span && tagged);
                    first_span = false;
                    span_off = off;
                    span_frames = 0;
                    streamer_begin_packet();
                }

                off += consumed;
                s_pcm_samples += samples;
                span_frames += samples / AUDIO_CHANNELS;

                const uint8_t *bytes = (const uint8_t *)pcm;
                size_t nbytes = samples * sizeof(int16_t);

                streamer_feed(bytes, nbytes);
            }

            if (span_frames > 0) {
                streamer_send_sbc(payload + span_off, (uint16_t)(off - span_off),
                                  span_frames, first_span && tagged);
            }

rearm:
            ESP_ERROR_CHECK(spi_slave_queue_trans(SPI_LINK_HOST, done, portMAX_DELAY));
        }

        int64_t now = esp_timer_get_time();
        if (next_report == 0) {
            next_report = now + 5000000;
        } else if (now >= next_report) {
            sbc_stream_info_t info;
            sbc_decoder_get_info(&info);

            uint32_t eff = (uint32_t)(s_pcm_samples * 1000000ULL /
                                      (uint64_t)(now - (next_report - 5000000)) / 2);
            const uint32_t dropped = streamer_take_dropped();

            static int quiet_left;
            const bool bad = s_bad_hdr || s_bad_crc || s_short || s_gaps
                             || s_decode_err || s_dec_crc || dropped
                             || s_max_gap_us > GAP_ALARM_US;
            if (bad || --quiet_left <= 0) {
                if (!bad) {
                    quiet_left = CONFIG_DANCEFLOOR_LOG_PERIOD_S / 5;
                }
                ESP_LOGI(TAG, "pkts %" PRIu32 " | %" PRIu32 " Hz x%u | eff %" PRIu32 " Hz | "
                              "hdr %" PRIu32 " crc %" PRIu32 " short %" PRIu32
                              " gaps %" PRIu32 " lost %" PRIu32
                              " dec %" PRIu32 " dcrc %" PRIu32
                              " | vol %" PRIu32
                              " | fed-drop %" PRIu32 " B | max gap %" PRIu32 " us",
                         s_packets, info.sample_rate, info.channels, eff,
                         s_bad_hdr, s_bad_crc, s_short, s_gaps, s_lost,
                         s_decode_err, s_dec_crc, s_vol, dropped, s_max_gap_us);
            }
            s_pcm_samples = 0;

            streamer_set_sample_rate(info.sample_rate ? info.sample_rate : 44100);

            s_packets = s_bad_hdr = s_bad_crc = s_short = s_gaps = s_decode_err = s_dec_crc = 0;
            s_lost = 0;
            s_vol = 0;
            s_max_gap_us = 0;
            next_report = now + 5000000;
        }
    }
}

void sbc_in_start(void)
{
    const gpio_config_t hs = {
        .pin_bit_mask = 1ULL << PIN_HS,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&hs));
    gpio_set_level(PIN_HS, 0);

    const spi_bus_config_t bus = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    const spi_slave_interface_config_t slv = {
        .spics_io_num = PIN_CS,
        .flags = 0,
        .queue_size = NFRAMES,
        .mode = 0,
        .post_setup_cb = spi_post_setup,
        .post_trans_cb = spi_post_trans,
    };
    ESP_ERROR_CHECK(spi_slave_initialize(SPI_LINK_HOST, &bus, &slv, SPI_DMA_CH_AUTO));

    for (int i = 0; i < NFRAMES; i++) {

        s_frame[i] = heap_caps_aligned_alloc(4, SBC_LINK_FRAME_BYTES, MALLOC_CAP_DMA);
        assert(s_frame[i]);
        s_trans[i].length = SBC_LINK_FRAME_BYTES * 8;
        s_trans[i].rx_buffer = s_frame[i];
        s_trans[i].tx_buffer = NULL;
    }

    if (!sbc_decoder_init()) {
        ESP_LOGE(TAG, "SBC decoder init failed");
    }

    task_start(rx_task, "sbc_in", 4096, 9, 1);
    ESP_LOGI(TAG, "SBC link listening: SPI slave at %d Hz, sck %d mosi %d cs %d, "
                  "handshake out on %d", SBC_LINK_SPI_HZ, PIN_SCK, PIN_MOSI,
             PIN_CS, PIN_HS);
}
