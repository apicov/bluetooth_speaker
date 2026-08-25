/**
 * @file sbc_in.c
 * @brief The SPI slave link from the bridge: raw SBC in, decoded audio out
 *        to this unit's speaker and undecoded SBC out to the satellites.
 *
 * One CS assertion is one packet -- sbc_link.h carries the framing -- so
 * rx_task never hunts for a sync word. What it does with a whole packet is
 * validate it, decode it, and hand every product to the streamer: the PCM
 * for the local ring, the SBC itself for the fan-out, the timeline
 * position, the marker tag. This file owns no playback state.
 */
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
#include "streamer.h"   /* task_start(), whose docs live there */

/**
 * @name The SPI link from the bridge
 * SPI3, because SPI2 is the LED strip (led_strip_wrapper.cpp takes SPI2_HOST for the
 * WS2812 encoding), so this link takes the other controller. The S3 has no
 * IOMUX pins for SPI3, so everything here goes through the GPIO matrix --
 * a little propagation delay, irrelevant at SBC_LINK_SPI_HZ, and the first
 * thing to suspect if the crc counter starts moving as that clock is raised.
 * The four pins are Kconfig, like every pin in this firmware.
 * @{
 */
#define SPI_LINK_HOST SPI3_HOST                        /**< Controller, not SPI2. */
#define PIN_SCK  CONFIG_DANCEFLOOR_SBC_SPI_SCK_PIN     /**< Clock, driven by the bridge. */
#define PIN_MOSI CONFIG_DANCEFLOOR_SBC_SPI_MOSI_PIN    /**< Data in from the bridge. */
#define PIN_CS   CONFIG_DANCEFLOOR_SBC_SPI_CS_PIN      /**< Chip select. */
#define PIN_HS   CONFIG_DANCEFLOOR_SBC_SPI_HS_PIN      /**< Handshake out. @see spi_post_setup */
/** @} */

/**
 * @def NFRAMES
 * @brief Two frame buffers, because of the decode.
 * The bridge will not clock a
 * transfer until the handshake says a buffer is armed, so a chip left with
 * none queued stalls the link -- and decoding a packet takes far longer
 * than the transfer does. With two, one is always armed while the other is
 * being decoded in place; that is also why a buffer cannot be re-queued
 * until its packet is finished with.
 */
#define NFRAMES 2

/** @brief ESP_LOG tag for the link. */
static const char *TAG = "sbc_in";

/** @brief The DMA frame buffers, one per queued transaction. */
static uint8_t *s_frame[NFRAMES];
/** @brief The queued transactions; armed once at start and re-armed in place. */
static spi_slave_transaction_t s_trans[NFRAMES];

/**
 * @name Per-window link statistics, printed every 5 s below
 * One declaration each, because a doc block cannot cover several declarators.
 * @{
 */
static uint32_t s_packets;    /**< Audio packets accepted. */
static uint32_t s_bad_hdr;    /**< Frames whose header did not validate. */
static uint32_t s_bad_crc;    /**< Frames the link CRC rejected. */
static uint32_t s_short;      /**< Transfers shorter than the frame claims. */
static uint32_t s_gaps;       /**< Sequence discontinuities, as EVENTS. @see s_lost */
static uint32_t s_decode_err; /**< SBC frames that would not decode. */
static uint32_t s_dec_crc;    /**< Decoder-reported CRC failures within a frame. */
/** @} */

/**
 * @brief Packets the gaps actually cost.
 *
 * @ref s_gaps counts EVENTS: the receive loop resyncs its expectation to
 * whatever arrived, so one lost packet and a thousand lost packets both read
 * as one gap. This is the number that can tell a steady trickle of single
 * losses from a burst of hundreds, which @ref s_gaps cannot.
 */
static uint32_t s_lost;

/**
 * @brief Volume frames taken, REPEATS INCLUDED.
 *
 * The bridge re-states the level on a heartbeat, so a link that is dropping
 * them reads as this sitting still while the phone is plainly being used. It
 * lives here rather than beside the streamer's transmit counter because this
 * module reports its own half of that exchange.
 */
static uint32_t s_vol;
/** @brief PCM samples decoded this window -- the `eff` rate's numerator. */
static uint64_t s_pcm_samples;

/** @brief Arrival instant of the previous audio packet; 0 before the first. */
static int64_t s_last_pkt_us;
/**
 * @brief Longest silence between two audio packets this window.
 *
 * Every other counter here describes a packet that ARRIVED and was wrong. A
 * source that simply stops sending produces no bad header, no CRC failure, no
 * sequence gap and no decode error, and is invisible to all of them.
 *
 * Delivery is bursty by construction, so an ordinary window still contains a
 * gap of around a hundred milliseconds against a ~50 packets/s average.
 * @ref GAP_ALARM_US therefore sits above that spread rather than above the
 * mean spacing: set near the spread itself it alarms on every window, which
 * is no alarm at all.
 */
static uint32_t s_max_gap_us;

#define GAP_ALARM_US 150000   /**< A max gap past this alarms the 5 s report. */

/*
 * The handshake: "a buffer is armed, you may clock a frame." ESP-IDF's
 * spi_slave drops any transfer that arrives with no transaction queued --
 * the peripheral has nowhere to put it -- so this pair is what makes the
 * link reliable. post_setup fires when the hardware has loaded a
 * transaction and post_trans when that transfer ends: exactly the window
 * during which the bridge may start one.
 *
 * Both run in interrupt context, hence IRAM_ATTR -- and IRAM_ATTR on the
 * callbacks alone would be decoration, because gpio_set_level() lives in
 * flash unless CONFIG_GPIO_CTRL_FUNC_IN_IRAM is set, which
 * sdkconfig.defaults sets for this reason. What that does NOT buy is
 * running through a flash write: the spi_slave ISR is registered without
 * ESP_INTR_FLAG_IRAM, so the interrupt is deferred, the handshake drops
 * late, and the bridge's HANDSHAKE_TIMEOUT_MS covers the stall.
 */
/**
 * @brief Raise the handshake: a buffer is armed, the bridge may clock a frame.
 * @param t The transaction the hardware just loaded. Unused.
 */
static void IRAM_ATTR spi_post_setup(spi_slave_transaction_t *t)
{
    gpio_set_level(PIN_HS, 1);
}

/**
 * @brief Drop the handshake: that transfer has ended.
 * @param t The transaction that just completed. Unused.
 */
static void IRAM_ATTR spi_post_trans(spi_slave_transaction_t *t)
{
    gpio_set_level(PIN_HS, 0);
}

/**
 * @brief Read the link forever: validate, decode, hand over, re-arm.
 * @param arg  Unused (the task body contract).
 *
 * Pinned beside the decoder at priority 9: nothing on this core outranks
 * feeding the playback ring, and out.c's dma-starve counter is what shows
 * it if anything ever holds this task long enough to matter.
 */
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
        /* The 20 ms timeout is not idleness -- it is what keeps the report
         * below running when the source has stopped, which is the case
         * s_max_gap_us exists to catch. */
        spi_slave_transaction_t *done = NULL;
        if (spi_slave_get_trans_result(SPI_LINK_HOST, &done,
                                       pdMS_TO_TICKS(20)) == ESP_OK) {
            /*
             * CS framed this transfer, but framing is not delivering it
             * whole: trans_len is how many bits the master clocked before
             * CS rose again, and anything short of a full frame means CS
             * glitched and split it -- the buffer holds a prefix of the new
             * packet on top of the stale tail of the last one. Not even the
             * length is trustworthy there (it would scope the CRC), so
             * count it and re-arm without reading either.
             */
            if (done->trans_len != SBC_LINK_FRAME_BYTES * 8) {
                s_short++;
                goto rearm;
            }

            const uint8_t *frame = (const uint8_t *)done->rx_buffer;
            spi_link_hdr_t hdr;
            memcpy(&hdr, frame, sizeof(hdr));
            const uint8_t *payload = frame + sizeof(spi_link_hdr_t);

            /* Header before CRC: a wrong length would decide how much of
             * the frame the CRC covers. Past this point len is sane and the
             * CRC is checking content, not deciding where content ends. */
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
                /* SIGNED, and that is the subtlety: a seq that went
                 * backwards -- repeated or reordered -- reads as four
                 * billion lost on unsigned arithmetic and swamps s_lost
                 * for the rest of the window. The bridge can produce one;
                 * its seq counter is reachable from two tasks. Only a
                 * forward jump is loss. */
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

                    /* A new track is the one moment a splice is inaudible,
                     * so take it: flag the next audio packet and let every
                     * unit null its phase error when playback reaches it.
                     * Metadata is re-sent for the same track, so act on the
                     * id rather than on arrival. */
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
                    /* Before the streamer decides whether it changed
                     * anything -- s_vol counts re-statements too. */
                    s_vol++;
                    /* Through the streamer's interface, like the metadata
                     * and the restart request: this module reads the link
                     * and hands over. */
                    streamer_set_volume(v->volume);
                }
                goto rearm;
            }
            s_packets++;

            /* Time since the previous audio packet; the first has nothing
             * to measure against. */
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

            /* Tag before feeding, so the mark lands at the start of this
             * packet's audio in the ring. Every MARKER_EVERY_PKTS-th packet
             * carries it -- a marker every ~2 s at the input rate. */
            streamer_begin_packet();

            static uint32_t mark_count;
            bool tagged = (mark_count++ % MARKER_EVERY_PKTS) == 0;
            if (tagged) {
                streamer_mark_here();
            }

            /* Decoded straight out of the DMA buffer -- no staging copy,
             * which is why the buffer is not re-armed until the loop below
             * is done with it, and why there are two of them. */
            static int16_t pcm[SBC_MAX_PCM_SAMPLES];
            /*
             * The span loop walks SBC frames and sends on at
             * AUDIO_TX_PAYLOAD_MTU_MAX, the most one audio datagram may
             * carry. A phone's ~825-byte payload never reaches it, so in
             * practice one packet is one datagram; the split exists for the
             * payload that would not fit, and this loop already knows each
             * frame's length, so nothing parses SBC twice to find a safe
             * place to cut.
             */
            size_t off = 0;
            size_t span_off = 0;        /* first byte not yet sent */
            uint32_t span_frames = 0;   /* PCM frames in [span_off, off) */
            bool first_span = true;     /* only the first carries the marker */
            while (off < hdr.len) {
                size_t consumed = 0, samples = 0;
                if (!sbc_decode_frame(payload + off, hdr.len - off, &consumed, pcm, &samples)) {
                    /* A CRC failure here is the SBC frame's own, not the
                     * link's -- the link CRC already passed, so this is
                     * corruption that check did not catch. Either way the
                     * decoder state is suspect: reset it and drop the rest
                     * of this packet. */
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
                 * Flush before taking this frame if it would push the span
                 * past the cap. Never flushes an empty span: a single SBC
                 * frame larger than the cap cannot be cut, so it goes on
                 * its own. begin_packet() again for the new span -- it
                 * snapshots the ring position the packet's audio starts at,
                 * which is the servo's input, and a span that inherited the
                 * previous span's position would be dated to audio already
                 * fed.
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
                /* The visualiser is deliberately not fed here: this is the
                 * arrival path, and ~200 ms of ring sits between it and the
                 * speaker -- lights driven from here run that far ahead of
                 * the sound. streamer.c's play side feeds it instead. */
                streamer_feed(bytes, nbytes);
            }

            /* Whatever is left, including the case where a decode error
             * broke the loop early: span_frames covers only the prefix that
             * decoded cleanly, so a satellite is never told to decode more
             * audio than the packet claims to carry. */
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
            /* Effective rate: PCM samples per second actually reaching the
             * playback path, in frames (hence /2 for stereo). A clean link
             * can still fall short of 44100 if frames die after decode. */
            uint32_t eff = (uint32_t)(s_pcm_samples * 1000000ULL /
                                      (uint64_t)(now - (next_report - 5000000)) / 2);
            const uint32_t dropped = streamer_take_dropped();

            /*
             * Quiet when clean, immediate when not. A healthy window says
             * the same thing every time, so only one clean window per log
             * period prints; a window with any fault prints regardless,
             * because waiting out the period to hear about a fault is how
             * faults get missed.
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
                              " gaps %" PRIu32 " lost %" PRIu32
                              " dec %" PRIu32 " dcrc %" PRIu32
                              " | vol %" PRIu32
                              " | fed-drop %" PRIu32 " B | max gap %" PRIu32 " us",
                         s_packets, info.sample_rate, info.channels, eff,
                         s_bad_hdr, s_bad_crc, s_short, s_gaps, s_lost,
                         s_decode_err, s_dec_crc, s_vol, dropped, s_max_gap_us);
            }
            s_pcm_samples = 0;

            /*
             * The DECLARED rate from the SBC header -- what the DAC, the
             * timeline and the LED conversion are all keyed to. Not `eff`
             * above, and it must not be: one SBC packet is ~882 frames
             * against the 220500 in five seconds, so whether the window
             * boundary falls before or after a packet is ±0.4%, and feeding
             * that here would retune the DAC on quantisation noise. A real
             * source-rate measurement would have to count frames between
             * packet arrivals over minutes; nothing needs it yet.
             */
            streamer_set_sample_rate(info.sample_rate ? info.sample_rate : 44100);
            /* Per-window, not cumulative: a rate says more than a rising
             * total, and the cumulative form made a healthy link read
             * worse with every capture. */
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
        /* DMA-capable, 4-byte aligned, and a length that is a multiple of
         * 4 -- the driver requires all three, and SBC_LINK_FRAME_BYTES is
         * sized in sbc_link.h so the last one comes free. */
        s_frame[i] = heap_caps_aligned_alloc(4, SBC_LINK_FRAME_BYTES, MALLOC_CAP_DMA);
        assert(s_frame[i]);
        s_trans[i].length = SBC_LINK_FRAME_BYTES * 8;
        s_trans[i].rx_buffer = s_frame[i];
        s_trans[i].tx_buffer = NULL;
    }

    if (!sbc_decoder_init()) {
        ESP_LOGE(TAG, "SBC decoder init failed");
    }
    /* The task that reads the link, through task_start() so a failure is
     * counted and named: without it the link is wired, logged as listening,
     * and silently receiving nothing. */
    task_start(rx_task, "sbc_in", 4096, 9, 1);
    ESP_LOGI(TAG, "SBC link listening: SPI slave at %d Hz, sck %d mosi %d cs %d, "
                  "handshake out on %d", SBC_LINK_SPI_HZ, PIN_SCK, PIN_MOSI,
             PIN_CS, PIN_HS);
}
