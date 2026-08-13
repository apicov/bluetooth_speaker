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
#include "streamer.h"   /* task_start() -- see the note where the task starts */

/*
 * SPI3. SPI2 is the LED strip -- led_strip_wrapper.cpp asks for SPI2_HOST and
 * the WS2812 encoding needs a whole bus to use one pin -- so this link takes
 * the other one. The S3 has no IOMUX pins for SPI3, meaning everything here
 * goes through the GPIO matrix; that costs a little propagation delay and is
 * irrelevant at the clock this link starts at. It is the first thing to suspect
 * if the crc counter starts moving as the clock is raised.
 *
 * Pins are Kconfig, like every other pin in this firmware, and the four of them
 * take all but one of the free pads on a XIAO ESP32-S3. GPIO 44 was the UART RX
 * pin this link used to arrive on, so the board keeps one wire it already had.
 *
 * WHAT THIS COST. GPIO 5 was DANCEFLOOR_MONITOR_GPIO, the input that watches a
 * satellite's marker pulse, and taking it kills the marker/monitor instrument
 * for good -- it needs both 4 and 5, and only one of them survives. That was
 * the cheapest of the available prices: the instrument is off
 * (DANCEFLOOR_ENABLE_MARKER defaults to n), needs a wire between two boards
 * that a deployed floor cannot have, and nothing corrects on it. Track-boundary
 * divergence is still reported over WiFi, for every satellite rather than the
 * wired one.
 *
 * NOT GPIO 43. The console lives there, deliberately moved off USB so
 * ESP_PHY_ENABLE_USB could be disabled -- see hub_s3/sdkconfig.defaults.
 */
#define SPI_LINK_HOST SPI3_HOST
#define PIN_SCK  CONFIG_DANCEFLOOR_SBC_SPI_SCK_PIN
#define PIN_MOSI CONFIG_DANCEFLOOR_SBC_SPI_MOSI_PIN
#define PIN_CS   CONFIG_DANCEFLOOR_SBC_SPI_CS_PIN
#define PIN_HS   CONFIG_DANCEFLOOR_SBC_SPI_HS_PIN

/*
 * Two frame buffers, and the reason is the decode.
 *
 * The bridge will not clock a transfer until the handshake says a buffer is
 * armed, so anything that leaves this chip with none queued stalls the link.
 * Decoding a packet takes far longer than the transfer does, so a single buffer
 * would leave the bridge waiting through every decode. With two, one is always
 * armed while the other is being decoded in place -- there is no staging copy,
 * which is also why a buffer cannot be re-queued until its packet is finished
 * with.
 */
#define NFRAMES 2

static const char *TAG = "sbc_in";

static uint8_t *s_frame[NFRAMES];
static spi_slave_transaction_t s_trans[NFRAMES];

/* Statistics, reported every 5 s. These replace the rate/loss instrumentation
 * that audio_in.c needed -- with no shared clock there is no rate to measure,
 * so what matters now is whether packets arrive intact. */
static uint32_t s_packets, s_bad_hdr, s_bad_crc, s_short, s_gaps, s_decode_err, s_dec_crc;
static uint64_t s_pcm_samples;

/*
 * Longest silence between two audio packets in the window.
 *
 * The counters above all describe packets that ARRIVED and were wrong. Nothing
 * described packets that never came, and a source that simply stops sending
 * produces no bad header, no CRC failure, no sequence gap and no decode error
 * -- it is invisible to every one of them.
 *
 * That blindness cost a real diagnosis: the hub's timeline was seen jumping
 * -126734 us, and drift accounted for 7% of it. The remaining ~118 ms was a
 * pause in delivery, and no instrument on this board could see one.
 *
 * A2DP packets arrive ~23/s, so ~43 ms apart -- but that is the AVERAGE, and
 * the first run with this counter showed the truth: every 5 s window contains a
 * gap of 79 to 112 ms, median 100. The source has always delivered in bursts,
 * and nothing here could see it.
 *
 * GAP_ALARM_US therefore sits above that, not at it. 100 ms was the first guess
 * and it alarmed on every window, which is no alarm at all.
 */
static int64_t s_last_pkt_us;
static uint32_t s_max_gap_us;

#define GAP_ALARM_US 150000

/*
 * The handshake: "a buffer is armed, you may clock a frame."
 *
 * ESP-IDF's spi_slave loses any transfer that arrives with no transaction
 * queued -- the peripheral has nowhere to put it -- so this is what makes the
 * link reliable rather than what makes it fast. post_setup fires when the
 * hardware has loaded a transaction, post_trans when that transfer ends, which
 * is exactly the window during which the bridge may start one.
 *
 * Both run in interrupt context, hence IRAM_ATTR -- and IRAM_ATTR on the
 * callback alone would be decoration, because gpio_set_level() lives in flash
 * unless GPIO_CTRL_FUNC_IN_IRAM is set, which sdkconfig.defaults now sets for
 * this reason.
 *
 * What that does NOT buy is running with the cache disabled. The spi_slave ISR
 * is registered without ESP_INTR_FLAG_IRAM, so during a flash write the
 * interrupt is deferred rather than taken, and the handshake drops late instead
 * of dangerously. The bridge's 100 ms timeout is what covers that, and a stall
 * there is counted rather than silent.
 */
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

    /* Arm every buffer before the bridge can clock anything at us. */
    for (int i = 0; i < NFRAMES; i++) {
        ESP_ERROR_CHECK(spi_slave_queue_trans(SPI_LINK_HOST, &s_trans[i], portMAX_DELAY));
    }

    while (1) {
        /*
         * One completed transfer is one packet: CS framed it, so there is no
         * sync word to hunt for, no partial header to wait on and no resync
         * scan. The 20 ms timeout is not idleness -- it is what keeps the
         * report below running when the source has stopped, which is the case
         * `max gap` exists to catch.
         */
        spi_slave_transaction_t *done = NULL;
        if (spi_slave_get_trans_result(SPI_LINK_HOST, &done,
                                       pdMS_TO_TICKS(20)) == ESP_OK) {
            /*
             * CS framed this transfer, but framing is not the same thing as
             * delivering it whole. trans_len is how many bits the master
             * actually clocked before CS rose again; anything short of a full
             * frame means CS glitched and split the transfer, so the buffer
             * holds a prefix of the new packet on top of the stale tail of the
             * last one. Nothing in there is to be trusted -- not the header,
             * not the length that would scope the CRC -- so count it and re-arm
             * without reading either.
             *
             * This does not contradict the fixed-frame-size design in
             * sbc_link.h: that refuses to TRUST trans_len to say where the
             * payload ends (the fixed size and len do that). This uses trans_len
             * only as an alarm that the frame was not delivered whole.
             */
            if (done->trans_len != SBC_LINK_FRAME_BYTES * 8) {
                s_short++;
                goto rearm;
            }

            const uint8_t *frame = (const uint8_t *)done->rx_buffer;
            spi_link_hdr_t hdr;
            memcpy(&hdr, frame, sizeof(hdr));
            const uint8_t *payload = frame + sizeof(spi_link_hdr_t);

            /*
             * Header first, because a wrong length would decide how much of the
             * frame the CRC covers. Past this point len is known sane and the
             * CRC is checking content rather than deciding where content ends.
             */
            if (hdr.len == 0 || hdr.len > SBC_LINK_MAX_PAYLOAD ||
                (hdr.kind != LINK_KIND_SBC && hdr.kind != LINK_KIND_META)) {
                s_bad_hdr++;
                goto rearm;
            }
            if (sbc_link_crc16(&hdr, payload, hdr.len) != hdr.crc) {
                s_bad_crc++;
                goto rearm;
            }

            if (have_seq && hdr.seq != expect_seq) {
                s_gaps++;
            }
            expect_seq = hdr.seq + 1;
            have_seq = true;

            if (hdr.kind == LINK_KIND_META) {
                if (hdr.len == sizeof(link_meta_t)) {
                    const link_meta_t *m = (const link_meta_t *)payload;
                    ESP_LOGW(TAG, "TRACK #%" PRIu32 ": \"%s\" - %s [%s]",
                             m->track_id, m->title, m->artist, m->album);
                    streamer_send_meta(payload, hdr.len);

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
                goto rearm;
            }
            s_packets++;

            /* Time since the previous audio packet. Skipped on the first one,
             * which has nothing to measure against. */
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

            /* Decoded straight out of the DMA buffer -- no staging copy, which
             * is why this buffer is not re-armed until the loop below is done
             * with it, and why there are two of them. */
            static int16_t pcm[SBC_MAX_PCM_SAMPLES];
            /*
             * Normally one packet per A2DP payload, exactly as before. The span
             * machinery exists for the one case that could never have worked: a
             * payload too large for one datagram, which is AUDIO_TX_PAYLOAD_MTU_MAX
             * at 1446 bytes and which a phone sending ~825 does not reach. It
             * does not bind, so the packet rate is what every captured log shows.
             *
             * IT WAS 721 -- the cap a whole redundant copy needs at depth 1 --
             * and that binds on every payload, so every one became two datagrams
             * and the audio packet rate doubled: 15% of the hub's own audio
             * discarded at the socket, and a timeline slewing at twice the rate
             * the satellites could follow. DANCEFLOOR_AUDIO_FEC_DEPTH's Kconfig
             * help has the run. The threshold is the whole difference between
             * the two behaviours, which is why it is named for what it protects
             * rather than for a number.
             *
             * The split is free where it is. This loop already walks SBC frames
             * and knows each one's length, so nothing parses SBC twice to find a
             * boundary it is safe to cut on.
             */
            size_t off = 0;
            size_t span_off = 0;        /* first byte not yet sent */
            uint32_t span_frames = 0;   /* PCM frames in [span_off, off) */
            bool first_span = true;     /* only the first carries the marker */
            while (off < hdr.len) {
                size_t consumed = 0, samples = 0;
                if (!sbc_decode_frame(payload + off, hdr.len - off, &consumed, pcm, &samples)) {
                    /* CRC here is the SBC frame's own, not the link's: the link
                     * CRC already passed, so a CRC result means corruption the
                     * link check did not catch. Either way the decoder state is
                     * suspect, so reset it and drop the rest of this packet. */
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

                /*
                 * Flush before taking this frame if it would push the span past
                 * the cap. Never flushes an empty span: a single SBC frame
                 * larger than the cap cannot be cut, so it goes on its own and
                 * the attach path truncates its copy and counts n_fec_truncated.
                 *
                 * streamer_begin_packet() again for the new span -- it snapshots
                 * the ring position this packet's audio starts at, which is the
                 * servo's input, and a span that inherited the previous span's
                 * position would be dated to audio already fed.
                 */
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
                /* The visualiser is deliberately NOT fed here. This is the
                 * arrival path, and ~200 ms of buffer sits between it and the
                 * speaker -- lights driven from here run that far ahead of the
                 * sound. streamer.c feeds it from the playback path instead. */
                streamer_feed(bytes, nbytes);
            }

            /*
             * Satellites get the SBC itself, not what we decoded from it.
             *
             * Whatever is left, which includes the case where a decode error
             * broke the loop early. Sending only the span that decoded cleanly
             * is a correction in its own right: this used to send the whole
             * payload with a frame count covering only the good prefix, so a
             * satellite decoded more audio than the packet claimed to carry and
             * every gap length computed from msg->frames after it was wrong.
             */
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
            /*
             * Effective rate is the number that matters: how many PCM samples
             * per second actually reach the playback path. The link can be
             * perfectly clean and this still fall short of 44100 if frames are
             * lost after decoding.
             */
            uint32_t eff = (uint32_t)(s_pcm_samples * 1000000ULL /
                                      (uint64_t)(now - (next_report - 5000000)) / 2);
            const uint32_t dropped = streamer_take_dropped();

            /*
             * Quiet when the link is clean, immediate when it is not.
             *
             * The window stays 5 s -- streamer_set_sample_rate() below feeds the
             * servo's rate estimate and must keep its cadence -- but a healthy
             * window says the same thing every time, so only one window per
             * log period gets printed. Any window with a bad header, a CRC failure, a
             * sequence gap, a decode error or a dropped feed prints regardless,
             * because those are the windows worth seeing and waiting 20 s to
             * hear about a fault is how faults get missed.
             *
             * `hdr` used to be `sync`, and counted bytes skipped while hunting
             * for the UART link's sync word. CS frames a packet now, so there is
             * no such thing to count and a column that could only ever print 0
             * would be an instrument with nothing behind it. Same position, same
             * job -- a frame refused before its contents were trusted -- and it
             * now counts an impossible kind or length.
             */
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
                              " gaps %" PRIu32 " dec %" PRIu32 " dcrc %" PRIu32
                              " | fed-drop %" PRIu32 " B | max gap %" PRIu32 " us",
                         s_packets, info.sample_rate, info.channels, eff,
                         s_bad_hdr, s_bad_crc, s_short, s_gaps, s_decode_err,
                         s_dec_crc, dropped, s_max_gap_us);
            }
            s_pcm_samples = 0;
            /* Rate the decoder actually produced, which is what the DAC must match. */
            streamer_set_sample_rate(info.sample_rate ? info.sample_rate : 44100);
            /* Per-window, not cumulative: a rising total tells you far less than
             * a rate, and cumulative counters made 500 k look worse than it was. */
            s_packets = s_bad_hdr = s_bad_crc = s_short = s_gaps = s_decode_err = s_dec_crc = 0;
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
        .miso_io_num = -1,          /* one-way link */
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
        /* DMA-capable and 4-byte aligned, with a length that is a multiple of
         * 4 -- the driver requires all three, and SBC_LINK_FRAME_BYTES is sized
         * in sbc_link.h so the last one comes for free. */
        s_frame[i] = heap_caps_aligned_alloc(4, SBC_LINK_FRAME_BYTES, MALLOC_CAP_DMA);
        assert(s_frame[i]);
        s_trans[i].length = SBC_LINK_FRAME_BYTES * 8;
        s_trans[i].rx_buffer = s_frame[i];
        s_trans[i].tx_buffer = NULL;
    }

    if (!sbc_decoder_init()) {
        ESP_LOGE(TAG, "SBC decoder init failed");
    }
    /* Checked, unlike every xTaskCreate in this tree used to be: without this
     * task the SPI link is wired, logged as listening, and silently receiving
     * nothing.
     *
     * Through task_start() rather than its own copy of the check. That helper
     * was static to streamer.c, so this file open-coded a near-identical message
     * and incremented nothing -- meaning a failure HERE, the one that costs all
     * the audio, was the one failure n_task_fail could not count. */
    task_start(rx_task, "sbc_in", 4096, 9, 1);
    ESP_LOGI(TAG, "SBC link listening: SPI slave at %d Hz, sck %d mosi %d cs %d, "
                  "handshake out on %d", SBC_LINK_SPI_HZ, PIN_SCK, PIN_MOSI,
             PIN_CS, PIN_HS);
}
