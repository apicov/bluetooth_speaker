/**
 * @file rx.c
 * @brief Datagram demux, anchor and gap policy, XOR parity, SBC decode, and
 *        the feed into the ring.
 *
 * @ref rx_task() is one recvfrom() loop that dispatches on the message type.
 * Audio takes the longest road: @ref handle_audio() applies the policies in a
 * fixed order — trust the clock, gauge the arrival, offer the packet to a
 * parity hold, anchor if there is no stream, upgrade a provisional anchor,
 * conceal a sequence gap — and only then delivers.
 *
 * @section fec The parity layer
 *
 * Guarded by `CONFIG_DANCEFLOOR_AUDIO_FEC_K` and absent when it is zero. Each
 * group of K packets is XORed into one accumulator as it arrives; when one
 * packet of a group is missing, the packets BEHIND the hole are held rather
 * than delivered, because delivering them would advance the ring past the gap
 * and there would be nowhere to put the repair. The parity datagram then
 * rebuilds the missing packet, or the hold is abandoned and the hole becomes
 * concealment.
 *
 * The cost is bounded: a hold lasts at most (K-2) packet times, which at K=4
 * and ~50 packets/s is about 40 ms against a @ref RING_TARGET_MS of 350. It
 * degrades to plain concealment whenever the group cannot be repaired.
 *
 * @warning This task is the only thing draining a UDP mailbox a few datagrams
 * deep against ~50 audio datagrams a second, and the console is a 115200-baud
 * UART. A log line per lost packet closes a loop — loss makes lines, lines
 * block this task, a blocked task overflows the mailbox, and the overflow is
 * more loss. Faults are counted here and narrated by telemetry.c.
 */
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_app_desc.h"
#include "nvs_flash.h"

#include "audio_out.h"
#include "sync_proto.h"
#include "sbc_link.h"
#include "sbc_decoder.h"
#include "visualiser.h"
#include "wifi_log.h"

#include "sat.h"

/** @brief The sequence number the next audio packet should carry. */
static uint32_t expect_seq;
/** @brief Whether @ref expect_seq means anything yet. Cleared whenever the
 *  stream is about to be re-anchored, so the next packet is taken as a fresh
 *  start rather than a gap. */
static bool have_seq;

/** @brief When the previous audio datagram arrived, for the cadence gauges. */
static int64_t s_prev_audio_at;
/** @brief Length of the run of datagrams arriving less than @ref RX_BURST_US
 *  apart. */
static uint32_t s_burst_run;

/**
 * @name Packet loss concealment
 *
 * Most of what a dropout sounds like is the two transients at its edges, not
 * the silence between them. So a gap is filled with the last real audio faded
 * down rather than an abrupt cut, and the audio that resumes afterwards is
 * faded back up — and the fill must be exactly the length of what was lost, or
 * every position recorded after it is wrong.
 * @{
 */
/** @brief How much of the last real audio is kept to fade out from. One SBC
 *  frame's worth. */
#define PLC_TAIL_FRAMES 128
/** @brief How long the fade out of, and back into, a gap takes. */
#define PLC_FADE_FRAMES 128
static int16_t s_plc_tail[PLC_TAIL_FRAMES * AUDIO_CHANNELS]; /**< The kept tail. */
static uint32_t s_plc_have;    /**< Frames actually in @ref s_plc_tail. */
static uint32_t s_plc_fade_in; /**< Frames of fade-in still owed after a gap. */
/** @} */

/**
 * @brief Keep the tail of some just-queued audio, to fade out from if the next
 *        packet is lost.
 *
 * @param pcm    Interleaved PCM just sent to the ring.
 * @param frames How many frames it holds.
 *
 * Taken AFTER any fade-in has been applied, so the tail is the audio as the
 * speaker will actually hear it.
 */
static void plc_note(const int16_t *pcm, uint32_t frames)
{
    if (frames == 0) {
        return;
    }
    const uint32_t take = frames > PLC_TAIL_FRAMES ? PLC_TAIL_FRAMES : frames;
    memcpy(s_plc_tail, pcm + (frames - take) * AUDIO_CHANNELS,
           (size_t)take * AUDIO_CHANNELS * sizeof(int16_t));
    s_plc_have = take;
}

/**
 * @brief Start or restart playback on this packet's timeline.
 *
 * @param msg    The packet to anchor on.
 * @param offset Local-to-master offset to seed the stream with.
 * @param by_tsf True if @p offset came from TSF.
 * @return true if the stream was anchored.
 *
 * The anchor is the only moment playback timing is decided: `play_at` is
 * consulted once, so an error in @p offset here is baked in for the life of
 * the stream. Everything this function refuses, it refuses for that reason.
 */
static bool anchor_stream(const audio_msg_t *msg, int64_t offset, bool by_tsf)
{
    /* The settling wait applies to the estimator only. TSF has no round trip,
     * so there is nothing to average and nothing to wait for. */
    if (!by_tsf && !sync_est_settled(&est)) {
        static bool told;
        if (!told) {
            told = true;
            ESP_LOGI(TAG, "holding playback until the clock estimate settles");
        }
        return false;
    }

    /* A resync is pending, so the playback task is on its way to parking and
     * has not yet released the ring. Anchoring now would reset a stream buffer
     * another task is blocked on. It is bounded and cannot deadlock: playback
     * clears the flag within its receive timeout whatever else happens. */
    if (resync_request) {
        return false;
    }

    const int64_t now_local = esp_timer_get_time();
    const int64_t start_local = sync_to_local(msg->play_at, offset);
    static int64_t refuse_since;   /* first refusal of this streak */
    static int64_t last_anchor;

    /* Two refusals, and a deadline on both. A packet that is already late is
     * evidence about the link, not a timeline worth starting on; and a
     * re-anchor that does not stick is worse than no re-anchor, so at most one
     * per interval. Past ANCHOR_GIVE_UP_US, take a bad anchor rather than
     * leave the speaker silent, and mark it provisional so a better packet can
     * replace it. */
    if (start_local - now_local < ANCHOR_MIN_LEAD_US ||
        (last_anchor && now_local - last_anchor < ANCHOR_MIN_INTERVAL_US)) {
        const bool late = start_local - now_local < ANCHOR_MIN_LEAD_US;
        if (late) n_anchor_late++; else n_anchor_soon++;
        if (refuse_since == 0) {
            refuse_since = now_local;
        }
        if (now_local - refuse_since < ANCHOR_GIVE_UP_US) {
            return false;
        }
        ESP_LOGE(TAG, "no anchorable packet for %lld ms (lead %+lld ms) -- "
                      "anchoring on a bad one rather than staying silent",
                 (now_local - refuse_since) / 1000,
                 (start_local - now_local) / 1000);
        anchor_provisional = true;
    } else {
        anchor_provisional = false;   /* this one had the lead it needed */
    }
    refuse_since = 0;
    last_anchor = now_local;
    anchor_at = now_local;
    resync_request = false;

    stream_rate = msg->sample_rate ? msg->sample_rate : 44100;
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
    /* The strip must date audio with the same rate playback does, or its
     * frames drift against the sound by the ratio between the two. */
    visualiser_set_rate(stream_rate);

    /* A new origin: whatever the analysis pipeline still holds belongs to the
     * previous stream. */
    visualiser_flush();
#endif
    sbc_decoder_init();

    /* Drop the concealment tail: fading a new stream up out of the previous
     * one's last frames would invent a discontinuity that was never there. */
    s_plc_have = 0;
    s_plc_fade_in = 0;
    stream_start_local = start_local;
    stream_offset = offset;
    offset_slew_last = 0;   /* re-seed the slew for this stream */

    samples_in = 0;
    marker_sample = -1;
    phase_head = phase_tail = 0;
    phase_valid = false;
    restart_pos = -1;
    expect_seq = msg->seq;
    have_seq = true;
    xStreamBufferReset(ring);
    n_reanchors++;
    if (by_tsf) n_tsf_used++; else n_tsf_fallback++;
    ESP_LOGI(TAG, "stream start: play_at %lld -> local %lld (in %lld ms) [%s]",
             msg->play_at, stream_start_local,
             (stream_start_local - esp_timer_get_time()) / 1000,
             by_tsf ? "TSF" : "probe estimator");

    /* The first anchor after a rejoin says which clock it used and how stale
     * the estimator's newest sample was: nothing invalidates the estimator's
     * window on a disconnect, so it may be describing the link from before
     * the outage. */
    if (rejoined_at) {
        const int64_t age = est_newest_at ? (now_local - est_newest_at) / 1000 : -1;
        ESP_LOGW(TAG, "first anchor after a rejoin: %s, %lld ms since the "
                      "rejoin, newest probe %lld ms old",
                 by_tsf ? "TSF" : "PROBE ESTIMATOR",
                 (now_local - rejoined_at) / 1000, age);
        rejoined_at = 0;
    }
    return true;
}

/**
 * @brief Replace a provisional anchor once a properly-led packet turns up.
 *
 * @param msg    The candidate packet.
 * @param offset Local-to-master offset.
 * @return true if a re-anchor was requested, in which case the caller must
 *         stop processing this packet.
 *
 * A provisional anchor is playing at a position known to be wrong, and the
 * servo cannot walk off an error that large. Requesting a resync rather than
 * anchoring inline lets playback park and release the ring first.
 */
static bool upgrade_provisional_anchor(const audio_msg_t *msg, int64_t offset)
{
    if (anchor_provisional &&
        sync_to_local(msg->play_at, offset) - esp_timer_get_time() >= ANCHOR_MIN_LEAD_US) {
        anchor_provisional = false;
        n_anchor_upgrades++;
        resync_request = true;
        have_seq = false;
        return true;
    }
    return false;
}

/**
 * @brief Write one packet's worth of concealment into the ring.
 *
 * @param want_frames Frames to write.
 * @return Frames the ring actually took, which may be fewer.
 *
 * The kept tail faded to zero over @ref PLC_FADE_FRAMES, then true silence.
 * Arms the matching fade-in so the audio that resumes is not an edge either.
 */
static uint32_t fill_gap_silence(uint32_t want_frames)
{
    static const int16_t silence[128 * AUDIO_CHANNELS] = {0};
    uint32_t filled = 0;

    static int16_t conceal[128 * AUDIO_CHANNELS];
    uint32_t faded = 0;             /* frames of fade emitted so far */

    while (filled < want_frames) {
        uint32_t left = want_frames - filled;
        uint32_t n = left > 128 ? 128 : left;
        const int16_t *src = silence;

        if (s_plc_have && faded < PLC_FADE_FRAMES) {
            for (uint32_t i = 0; i < n; i++) {
                const uint32_t pos = faded + i;

                const int32_t gain = pos < PLC_FADE_FRAMES
                                   ? (int32_t)(PLC_FADE_FRAMES - pos) : 0;
                const int16_t *s = &s_plc_tail[(pos % s_plc_have) * AUDIO_CHANNELS];
                for (int c = 0; c < AUDIO_CHANNELS; c++) {
                    conceal[i * AUDIO_CHANNELS + c] =
                        (int16_t)((int32_t)s[c] * gain / (int32_t)PLC_FADE_FRAMES);
                }
            }
            src = conceal;
        }

        size_t want = (size_t)n * AUDIO_CHANNELS * sizeof(int16_t);
        size_t sent = xStreamBufferSend(ring, src, want, 0);
        uint32_t got = (uint32_t)(sent / (AUDIO_CHANNELS * sizeof(int16_t)));
        filled += got;
        faded += got;
        /* samples_in MUST count this silence: it is time the DAC will spend,
         * so every position recorded after it is displaced by exactly this
         * much if it is not counted. */
        samples_in += (int32_t)got;
        if (sent < want) {
            break;              /* raced the playback task; caller resyncs */
        }
    }

    s_plc_fade_in = PLC_FADE_FRAMES;
    return filled;
}

/**
 * @brief Conceal a run of lost packets, or ask for a re-anchor if it is too
 *        big to conceal.
 *
 * @param missing    How many packets were lost.
 * @param per_frames Frames each packet carried.
 * @return true if the gap was filled; false if a resync was requested
 *         instead, in which case the caller must stop.
 *
 * Past @ref GAP_RESYNC_MS this is an outage rather than jitter, and the
 * silence would have to go somewhere the ring is not sized for — re-anchoring
 * is cheaper than filling. The ring's room is checked up front rather than
 * discovered part-way through, so a fill either happens whole or does not
 * start.
 */
static bool fill_gap(uint32_t missing, uint32_t per_frames)
{
    const uint32_t frames_missing = missing * per_frames;

    /* Counted HERE and not at detection: this is silence actually written, so
     * a gap that parity repairs adds to n_gaps and nothing to this. The two
     * together say how much was lost and how much of it was heard. */
    n_gap_frames += frames_missing;
    {

        if (frames_missing > (uint32_t)((uint64_t)GAP_RESYNC_MS * stream_rate / 1000)) {
            n_gap_resyncs++;
            resync_request = true;
            have_seq = false;
            return false;
        }

        /* A snapshot -- the playback task is draining the ring concurrently
         * -- so the per-packet loop below still checks what it got. */
        size_t room = xStreamBufferSpacesAvailable(ring);
        uint32_t can_take = (uint32_t)(room / (AUDIO_CHANNELS * sizeof(int16_t)));
        if (can_take < frames_missing) {
            n_gap_short++;
            n_gap_short_frames += frames_missing - can_take;
            n_gap_short_resyncs++;
            resync_request = true;
            have_seq = false;
            return false;
        }

        uint32_t filled = 0;
        for (uint32_t i = 0; i < missing && filled < frames_missing; i++) {
            uint32_t per = frames_missing - filled;
            if (per > per_frames) {
                per = per_frames;
            }
            uint32_t got = fill_gap_silence(per);
            filled += got;
            if (got < per) {

                n_gap_short++;
                n_gap_short_frames += per - got;
                n_gap_short_resyncs++;
                resync_request = true;
                have_seq = false;
                return false;
            }
        }
    }
    return true;
}

#if CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0
static bool fec_hold_begin(const audio_msg_t *msg, uint32_t missing);
#endif

/**
 * @brief Deal with a sequence discontinuity before this packet is delivered.
 *
 * @param msg The arriving packet.
 * @return true if the packet should be delivered; false if it was held for
 *         parity, dropped, or a resync was requested.
 *
 * A forward jump is loss: offered to parity first, and concealed if parity
 * cannot take it. A backward one should be impossible, since the hub sends
 * each packet once and the link layer retries below the socket rather than
 * above it — and it matters because a reorder means the "gap" before it was
 * never a loss, so silence was inserted against a packet that did arrive.
 */
static bool absorb_sequence_gap(const audio_msg_t *msg)
{
    if (have_seq && msg->seq > expect_seq) {
        const uint32_t missing = msg->seq - expect_seq;
        /* Counted at DETECTION, before any repair, so this measures the link
         * rather than the concealment -- which is what makes a parity build
         * and a bare build comparable on it. */
        n_gaps++;

#if CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0
        if (fec_hold_begin(msg, missing)) {
            return false;
        }

        /* Not held, so parity will not cover this one. Every detected gap
         * lands in exactly one of recovered or lost, which is the identity
         * telemetry reads them against. */
        n_fec_lost++;
#endif
        return fill_gap(missing, msg->frames);
    } else if (have_seq && msg->seq < expect_seq) {
        n_seq_dropped++;
        return false;
    }
    return true;
}

/**
 * @brief Note where this packet's audio lands in the ring, and when it is due.
 *
 * @param msg The packet about to be decoded.
 *
 * Called BEFORE the audio is queued, so the recorded position is the start of
 * this packet rather than its end.
 */
static void record_packet_positions(const audio_msg_t *msg)
{
    if (msg->marker && marker_sample < 0) {
        marker_sample = samples_in;
    }

    if (msg->restart && restart_pos < 0) {
        restart_pos = samples_in;
    }

    uint32_t next = (phase_head + 1) % PHASE_Q_LEN;
    if (next != phase_tail) {
        phase_q[phase_head].pos = samples_in;
        phase_q[phase_head].play_at = msg->play_at;
        phase_head = next;
    } else {
        n_phase_drop++;
    }
}

/**
 * @brief Turn the packet's SBC into PCM and hand it to the ring.
 *
 * @param msg The packet to decode.
 *
 * A frame that will not decode ends this packet: the rest of it is dropped and
 * the decoder is re-initialised so the next packet resyncs rather than
 * wedging. The timeline is then short by whatever that packet held, which is
 * what @ref n_decode_err exists to say.
 */
static void decode_into_ring(const audio_msg_t *msg)
{
    static int16_t pcm[SBC_MAX_PCM_SAMPLES];

#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
    uint32_t pkt_frames = 0;
#endif
    size_t off = 0;
    while (off < msg->payload_len) {
        size_t consumed = 0, samples = 0;
        if (!sbc_decode_frame(msg->payload + off, msg->payload_len - off,
                              &consumed, pcm, &samples)) {

            n_decode_err++;
            sbc_decoder_init();
            break;
        }
        if (consumed == 0) {
            break;
        }
        off += consumed;
        size_t want = samples * sizeof(int16_t);

        /* Faded in place, BEFORE the ring and before the visualiser, so both
         * see the audio the speaker will actually produce. */
        if (s_plc_fade_in) {
            const uint32_t frames = (uint32_t)(samples / AUDIO_CHANNELS);
            for (uint32_t i = 0; i < frames && s_plc_fade_in; i++) {
                const int32_t gain =
                    (int32_t)(PLC_FADE_FRAMES - s_plc_fade_in) + 1;
                for (int c = 0; c < AUDIO_CHANNELS; c++) {
                    int16_t *v = &pcm[i * AUDIO_CHANNELS + c];
                    *v = (int16_t)((int32_t)*v * gain / (int32_t)PLC_FADE_FRAMES);
                }
                s_plc_fade_in--;
            }
        }

        plc_note(pcm, (uint32_t)(samples / AUDIO_CHANNELS));

        size_t sent = xStreamBufferSend(ring, pcm, want, 0);
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER

        visualiser_feed((const uint8_t *)pcm, (uint32_t)sent,
                        msg->play_at + (int64_t)pkt_frames * 1000000 / stream_rate);
        pkt_frames += (uint32_t)(sent / (AUDIO_CHANNELS * sizeof(int16_t)));
#endif
        if (sent < want) {
            n_ring_full++;
        }
        /* By what the ring TOOK, not by what was offered: a full ring drops
         * the remainder, and counting it would put every later position ahead
         * of the audio. */
        samples_in += (int32_t)(sent / (AUDIO_CHANNELS * sizeof(int16_t)));
    }
}

static void audio_deliver(const audio_msg_t *msg);
#if CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0

_Static_assert(CONFIG_DANCEFLOOR_AUDIO_FEC_K >= 2 &&
               CONFIG_DANCEFLOOR_AUDIO_FEC_K <= AUDIO_FEC_K_MAX,
               "DANCEFLOOR_AUDIO_FEC_K must be 0 (off) or 2..AUDIO_FEC_K_MAX");
/** @brief Packets per parity group. */
#define FEC_K        CONFIG_DANCEFLOOR_AUDIO_FEC_K
/** @brief Most packets that can sit behind a hole: everything in the group
 *  except the lost one. */
#define FEC_HOLD_MAX (FEC_K - 1)

/**
 * @name The parity accumulator
 * ONE accumulator, not a window of K buffers. Each packet is XORed in as it
 * arrives, so a group costs one codeword of memory rather than K of them, and
 * the parity datagram completes the sum.
 * @{
 */
static uint8_t  s_fec_acc[AUDIO_FEC_CODEWORD_MAX]; /**< Running XOR of the group. */
static uint8_t  s_fec_rec[AUDIO_FEC_CODEWORD_MAX]; /**< A rebuilt packet, as wire bytes. */
static uint16_t s_fec_span;        /**< Bytes of @ref s_fec_acc in use. */
static uint32_t s_fec_base;        /**< First sequence number of the open group. */
static uint8_t  s_fec_seen;        /**< Bitmask of group members folded in. */
static bool     s_fec_open;        /**< A group is accumulating. */
static bool     s_fec_ok;          /**< Nothing has spoiled it (an oversized payload). */
static uint32_t s_fec_closed_base; /**< The group just closed. */
static bool     s_fec_closed;      /**< @ref s_fec_closed_base is meaningful. */
/** @} */

/**
 * @brief Packets held behind a hole, waiting for the parity that would fill
 *        it.
 *
 * Stored as raw WIRE bytes rather than decoded PCM: they have to be XORed and
 * possibly replayed in arrival order, and decoding them before the hole is
 * resolved would advance the ring past the gap.
 */

static struct {
    bool     active;
    uint32_t seq;
    uint32_t frames;
    uint32_t last_seq;
    int64_t  began_at;
    int      n;
    uint16_t len[FEC_HOLD_MAX];
    uint8_t  buf[FEC_HOLD_MAX][AUDIO_FEC_CODEWORD_MAX];
} s_hold;

/**
 * @brief Discard the accumulator and open a new group.
 * @param base First sequence number of the new group.
 */
static void fec_group_reset(uint32_t base)
{
    memset(s_fec_acc, 0, s_fec_span);
    s_fec_span = 0;
    s_fec_base = base;
    s_fec_seen = 0;
    s_fec_open = true;
    s_fec_ok = true;
}

/** @brief Close the open group, remembering which it was so late members do
 *  not reopen it. */
static void fec_group_close(void)
{
    if (s_fec_open) {
        s_fec_closed_base = s_fec_base;
        s_fec_closed = true;
    }
    s_fec_open = false;
}

/**
 * @brief Fold a delivered or held packet into its group's XOR.
 * @param m The packet.
 *
 * Called for every packet that reaches the ring, held or not — the sum has to
 * include them all for the parity to complete it.
 */
static void fec_note_arrival(const audio_msg_t *m)
{
    const uint32_t base = m->seq - (m->seq % FEC_K);

    /* A group that has already been decided must not be reopened by a late
     * member arriving after it. */
    if (s_fec_closed && base == s_fec_closed_base) {
        return;
    }
    if (!s_fec_open || base != s_fec_base) {
        fec_group_reset(base);
    }
    if (!audio_fec_xor_in(s_fec_acc, &s_fec_span, m)) {
        s_fec_ok = false;
        return;
    }
    s_fec_seen |= (uint8_t)(1u << (m->seq % FEC_K));
}

/**
 * @brief Append a packet to the hold, and fold it into the group XOR.
 * @param m The packet.
 * @return false if the hold is full or the packet will not fit a codeword.
 */
static bool fec_hold_push(const audio_msg_t *m)
{
    const uint16_t len = (uint16_t)AUDIO_MSG_BYTES(m->payload_len);

    if (s_hold.n >= FEC_HOLD_MAX || len > (uint16_t)AUDIO_FEC_CODEWORD_MAX) {
        return false;
    }
    memcpy(s_hold.buf[s_hold.n], m, len);
    s_hold.len[s_hold.n] = len;
    s_hold.n++;
    s_hold.last_seq = m->seq;
    fec_note_arrival(m);
    return true;
}

/** @brief Record how long the hold lasted into @ref fec_hold_max_us, whichever
 *  way it ended — the figure the whole parity design rests on. */
static void fec_hold_note_duration(void)
{
    const int64_t held = esp_timer_get_time() - s_hold.began_at;

    if (held > 0 && held < INT32_MAX && (int32_t)held > fec_hold_max_us) {
        fec_hold_max_us = (int32_t)held;
    }
}

/** @brief Deliver the held packets, in arrival order. */
static void fec_hold_flush(void)
{
    for (int i = 0; i < s_hold.n; i++) {
        audio_deliver((const audio_msg_t *)s_hold.buf[i]);
    }
    s_hold.n = 0;
}

/**
 * @brief Give up on a hold: conceal the hole, then release what was behind it.
 *
 * This is the fallback, and it is exactly what the receiver would have done
 * with no parity at all. If the fill could not happen the whole hold is
 * dropped, because a resync is coming and those packets belong to a timeline
 * that is about to be replaced.
 */
static void fec_hold_abandon(void)
{
    if (!s_hold.active) {
        return;
    }
    n_fec_lost++;
    fec_hold_note_duration();
    s_hold.active = false;
    fec_group_close();

    if (fill_gap(1, s_hold.frames)) {
        fec_hold_flush();
    } else {

        s_hold.n = 0;
    }
}

/**
 * @brief Whether there is a stream for parity to be about.
 * @return true if the sequence is known AND playback is running.
 *
 * Both halves are needed: rx.c clears the first, but play.c zeroes the second
 * on its own, so testing either alone would miss a stream that had ended.
 */
static bool fec_stream_ready(void)
{
    return have_seq && stream_start_local != 0;
}

/** @brief Throw away the hold and the accumulator, counting an active hold as
 *  a loss. Used when there is no stream for them to belong to. */
static void fec_reset(void)
{
    if (s_hold.active) {
        n_fec_lost++;
        fec_hold_note_duration();
        s_hold.active = false;
        s_hold.n = 0;
    }
    s_fec_open = false;
    s_fec_closed = false;
    memset(s_fec_acc, 0, s_fec_span);
    s_fec_span = 0;
}

/**
 * @brief Start holding, if this loss is one parity could actually repair.
 *
 * @param msg     The packet that arrived after the hole.
 * @param missing How many packets are missing.
 * @return true if the hold started, in which case @p msg is not delivered yet.
 *
 * Every refusal below is a case parity cannot cover, and each one costs
 * nothing: the caller falls through to plain concealment.
 */
static bool fec_hold_begin(const audio_msg_t *msg, uint32_t missing)
{
    const uint32_t want = expect_seq;

    if (missing != 1) {
        return false;           /* parity repairs one loss, not two */
    }
    if (anchor_provisional) {
        /* The anchor is already known to be wrong and a re-anchor is being
         * looked for; holding audio behind a hole would only delay it. */
        return false;
    }
    if (msg->payload_len > AUDIO_FEC_PAYLOAD_MAX) {
        return false;           /* would not fit a codeword */
    }
    if (want / FEC_K != msg->seq / FEC_K) {
        return false;           /* the loss is not in this packet's group */
    }
    if (!s_fec_open || !s_fec_ok || s_fec_base != want - (want % FEC_K)) {
        return false;           /* nothing usable accumulated to repair from */
    }

    s_hold.n = 0;
    if (!fec_hold_push(msg)) {
        return false;
    }
    s_hold.active = true;
    s_hold.seq = want;
    s_hold.frames = msg->frames;
    s_hold.began_at = esp_timer_get_time();
    n_fec_holds++;
    return true;
}

/**
 * @brief Offer a further packet to an open hold.
 * @param m The arriving packet.
 * @return true if it was taken; false if the caller must abandon the hold.
 *
 * Only a packet contiguous with the hold and in the same group can join it: a
 * second discontinuity means parity can no longer close the first.
 */
static bool fec_hold_offer(const audio_msg_t *m)
{
    if (m->seq != s_hold.last_seq + 1) {
        return false;
    }
    if (m->seq / FEC_K != s_hold.seq / FEC_K) {
        return false;
    }
    if (m->payload_len > AUDIO_FEC_PAYLOAD_MAX) {
        return false;
    }
    return fec_hold_push(m);
}

/**
 * @brief Complete the group's XOR with the parity and rebuild the lost packet.
 *
 * @param fm       The parity datagram.
 * @param want_seq Sequence number of the packet to rebuild.
 * @return true if the packet was rebuilt and delivered.
 *
 * The rebuilt packet is delivered first, then the hold behind it, so the ring
 * receives the group in sequence order.
 */
static bool fec_repair(const audio_fec_msg_t *fm, uint32_t want_seq)
{
    for (uint16_t i = 0; i < fm->span; i++) {
        s_fec_acc[i] ^= fm->parity[i];
    }

    s_fec_span = fm->span;

    if (!audio_fec_extract(s_fec_acc, fm->span, want_seq,
                           (audio_msg_t *)s_fec_rec)) {
        n_fec_bad++;
        return false;
    }

    /* No hold means the loss was the group's LAST member, so nothing arrived
     * after it to notice the discontinuity -- absorb_sequence_gap() never ran
     * and never counted it. Counted here so the recovered/lost identity still
     * holds. */
    if (!s_hold.active) {
        n_gaps++;
    }
    n_fec_recovered++;
    if (s_hold.active) {
        fec_hold_note_duration();
        s_hold.active = false;
    }
    fec_group_close();
    audio_deliver((const audio_msg_t *)s_fec_rec);
    fec_hold_flush();
    return true;
}

/**
 * @brief Handle a parity datagram: repair the group, or give up on it.
 *
 * @param fm The parity datagram.
 * @param n  Datagram length in bytes, checked against the span it claims.
 *
 * A parity can arrive with no audio datagram between it and the last check, so
 * the stream test here is not reachable from the one in @ref handle_audio().
 */
static void fec_parity_rx(const audio_fec_msg_t *fm, int n)
{
    n_fec_parity_rx++;

    if (!fec_stream_ready()) {
        fec_reset();
        return;
    }

    const bool usable =
        fm->count == FEC_K &&
        fm->span >= (uint16_t)AUDIO_MSG_BYTES(0) &&
        fm->span <= (uint16_t)AUDIO_FEC_CODEWORD_MAX &&
        fm->span >= s_fec_span &&
        n >= (int)AUDIO_FEC_MSG_BYTES(fm->span) &&
        s_fec_open && s_fec_ok && fm->base_seq == s_fec_base;

    if (usable) {
        const uint8_t full = (uint8_t)((1u << FEC_K) - 1u);
        const uint8_t lost = (uint8_t)(full & ~s_fec_seen);

        if (lost == 0) {
            fec_group_close();      /* a clean group: nothing to repair */
            return;
        }

        /* Exactly one bit set, i.e. exactly one member missing. */
        if ((lost & (uint8_t)(lost - 1u)) == 0) {
            const uint32_t want = s_fec_base + (uint32_t)__builtin_ctz(lost);
            if (want == expect_seq && fec_repair(fm, want)) {
                return;
            }
        }
    } else if (fm->count != FEC_K) {
        /* The hub and this build disagree about the group size, which is a
         * firmware mismatch and not a radio problem. */
        n_fec_bad++;
    }

    fec_hold_abandon();
    fec_group_close();
}
#endif

/**
 * @brief Accept a packet: advance the sequence, record its positions, decode
 *        it into the ring, and fold it into the parity group.
 *
 * @param msg The packet, which may be one rebuilt by parity rather than one
 *            that arrived.
 */
static void audio_deliver(const audio_msg_t *msg)
{
    expect_seq = msg->seq + 1;
    have_seq = true;

    record_packet_positions(msg);
    decode_into_ring(msg);
#if CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0
    fec_note_arrival(msg);
#endif
}

/**
 * @brief The audio path: every policy, in the order they have to be applied.
 *
 * @param msg        The arriving packet.
 * @param arrived_at Local time recvfrom() returned it.
 */
static void handle_audio(const audio_msg_t *msg, int64_t arrived_at)
{
    int64_t offset;
    bool by_tsf = false;
    if (!clock_offset(&offset, &by_tsf)) {
        return;
    }

    /* Gauge the lead here: after the offset is trusted, and before every
     * refusal below, so a packet that is rejected still reports how it
     * arrived. @see rx_lead_min_us */
    const int64_t lead = msg->play_at - (arrived_at + offset);
    if (lead > LEAD_INSANE_US || lead < -LEAD_INSANE_US) {
        n_lead_insane++;
    } else if (lead < rx_lead_min_us) {
        rx_lead_min_us = (int32_t)lead;
    }
    if (msg->format != AUDIO_FMT_SBC) {
        ESP_LOGW(TAG, "unexpected audio format %u -- hub and satellite disagree",
                 msg->format);
        return;
    }

#if CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0
    /* The hold is decided BEFORE the anchor, and the order is load-bearing:
     * anchoring resets the ring, and a hold whose packets outlived that reset
     * would be replayed onto a stream they do not belong to. */
    if (!fec_stream_ready()) {
        fec_reset();
    }
    if (s_hold.active) {
        if (fec_hold_offer(msg)) {
            return;
        }
        fec_hold_abandon();
    }
#endif

    if ((!have_seq || stream_start_local == 0) &&
        !anchor_stream(msg, offset, by_tsf)) {
        return;
    }
    if (upgrade_provisional_anchor(msg, offset)) {
        return;
    }
    if (!absorb_sequence_gap(msg)) {
        return;
    }

    audio_deliver(msg);
}

/* rx_task() is documented at its declaration in sat.h. */
void rx_task(void *arg)
{
    (void)arg;

    /* Sized to the largest message this unit can be sent, and asserted rather
     * than assumed. */
    static uint8_t buf[sizeof(audio_msg_t)];
    _Static_assert(sizeof(buf) >= AUDIO_UDP_MTU, "a full datagram would not fit");

    while (1) {
        int n = recvfrom(sock, buf, sizeof(buf), 0, NULL, NULL);
        int64_t t4 = esp_timer_get_time();
        if (n < 1) {
            /* Counted and briefly yielded, rather than spun on at priority 7
             * with nothing recorded. */
            if (n < 0) {
                n_recv_err++;
                vTaskDelay(1);
            }
            continue;
        }

        if (buf[0] == MSG_TIME_RSP && n >= (int)sizeof(time_msg_t)) {
            time_msg_t msg;
            memcpy(&msg, buf, sizeof(msg));
            sync_est_add(&est, msg.t1, msg.t2, msg.t3, t4);

            /* Dates the newest sample, so a first anchor after a rejoin can
             * say how stale the estimator's window was. */
            est_newest_at = t4;
        } else if (buf[0] == MSG_VOL && n >= (int)sizeof(vol_msg_t)) {
            const vol_msg_t *v = (const vol_msg_t *)buf;

            const uint8_t vol = v->volume > AUDIO_VOL_MAX ? AUDIO_VOL_MAX
                                                          : v->volume;

            /* Counted before the change test, so a unit that hears audio and
             * never hears a level shows silence here rather than looking the
             * same as one whose level nobody has touched. */
            n_vol_rx++;

            if (vol != audio_volume || !audio_vol_known) {
                ESP_LOGW(TAG, "VOLUME %u/%d", vol, AUDIO_VOL_MAX);
            }

            /* The level THEN the flag. Both are single-byte volatile stores,
             * so a reader that sees the flag has necessarily seen the level
             * that goes with it. */
            audio_volume = vol;
            audio_vol_known = true;
        } else if (buf[0] == MSG_META && n >= (int)sizeof(meta_msg_t)) {
            const link_meta_t *m = (const link_meta_t *)((const meta_msg_t *)buf)->payload;
            ESP_LOGW(TAG, "TRACK #%" PRIu32 ": \"%s\" - %s [%s]",
                     m->track_id, m->title, m->artist, m->album);
        } else if (buf[0] == MSG_FRAME && n >= (int)FRAME_MSG_BYTES(0)) {

#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER && CONFIG_DANCEFLOOR_LED_SOURCE_REMOTE

            /* The batch shape is checked rather than assumed, and complained
             * about once: a mismatch means the hub and this unit are not the
             * same build, which will not fix itself. */
            const frame_msg_t *fm = (const frame_msg_t *)buf;
            if (fm->len == sizeof(vis_frame_t) && fm->count >= 1 &&
                (size_t)fm->count * fm->len <= FRAME_PAYLOAD_MAX &&
                n >= (int)FRAME_MSG_BYTES((size_t)fm->count * fm->len)) {
                for (unsigned i = 0; i < fm->count; i++) {
                    vis_frame_t f;
                    memcpy(&f, fm->payload + i * sizeof(f), sizeof(f));
                    visualiser_submit_frame(&f);
                }

                /* Counted in FRAMES, not datagrams, so the rate is comparable
                 * with the rate the hub says it sent. */
                n_frames_rx += fm->count;
            } else {
                static bool told;
                if (!told) {
                    told = true;
                    ESP_LOGE(TAG, "frame batch of %u x %u bytes in %d -- hub and "
                                  "satellite are not the same build (frame is %u)",
                             fm->count, fm->len, n, (unsigned)sizeof(vis_frame_t));
                }
                n_frames_bad++;
            }
#endif
        } else if (buf[0] == MSG_TSF && n >= (int)sizeof(tsf_msg_t)) {

            tsf_msg_t m;
            memcpy(&m, buf, sizeof(m));

            /* The local clock read is BRACKETED between two TSF reads, so the
             * sample carries its own error bar: `span` is how much could have
             * landed between them, and the midpoint is the best estimate of
             * the TSF value at `my_local`. */
            const int64_t tsf_a = esp_wifi_get_tsf_time(WIFI_IF_STA);
            const int64_t my_local = esp_timer_get_time();
            const int64_t tsf_b = esp_wifi_get_tsf_time(WIFI_IF_STA);
            const int64_t span = tsf_b - tsf_a;
            const int64_t my_tsf = (tsf_a + tsf_b) / 2;

            static bool announced;
            if (!announced) {
                announced = true;
                ESP_LOGW(TAG, "TSF first message: hub %lld us, ours %lld us%s",
                         m.tsf, my_tsf,
                         (m.tsf && my_tsf) ? "" : "  <-- zero means not available");
            }

            int64_t est_offset;
            if (m.tsf == 0 || my_tsf == 0) {
                static bool told_zero;
                if (!told_zero) {
                    told_zero = true;
                    ESP_LOGW(TAG, "TSF unavailable (hub %lld, ours %lld) -- "
                                  "no comparison possible", m.tsf, my_tsf);
                }
                continue;
            }
            if (!sync_est_offset(&est, &est_offset)) {
                continue;
            }

            const int64_t tsf_offset = (m.local - m.tsf) - (my_local - my_tsf);

            /* Counted, not enforced. TSF is the anchor clock source, and a
             * threshold chosen blind would silently demote it to the probe
             * estimator -- a regression wearing no log line. @see n_tsf_wide */
            if (span > TSF_SPAN_MAX_US) {
                n_tsf_wide++;
            }

            tsf_publish(tsf_offset, my_local);

            static int64_t last_log_us;
            static int64_t prev_tsf, prev_est;
            static int64_t span_max;
            if (span > span_max) {
                span_max = span;
            }
            if (my_local - last_log_us < (int64_t)CONFIG_DANCEFLOOR_LOG_PERIOD_S * 1000000) {
                continue;
            }
            const int64_t tsf_step = last_log_us ? tsf_offset - prev_tsf : 0;
            const int64_t est_step = last_log_us ? est_offset - prev_est : 0;
            last_log_us = my_local;
            prev_tsf = tsf_offset;
            prev_est = est_offset;

            ESP_LOGW(TAG, "TSF: est %+lld us | tsf %+lld us | diff %+lld us | "
                          "steps tsf %+lld us, est %+lld us | span %lld us "
                          "(max %lld)",
                     est_offset, tsf_offset, tsf_offset - est_offset,
                     tsf_step, est_step, span, span_max);
            span_max = 0;
        } else if (buf[0] == MSG_AUDIO && n >= (int)AUDIO_MSG_BYTES(0)) {

            /* Cadence gauges first, before any refusal in handle_audio():
             * these describe how audio ARRIVED, which is a separate question
             * from whether it was usable. Parity datagrams are deliberately
             * excluded, so the instrument does not report a delivery that
             * carried no audio. */
            n_audio_rx++;
            if (s_prev_audio_at) {
                const int64_t gap = t4 - s_prev_audio_at;
                if (gap > rx_gap_max_us) {
                    rx_gap_max_us = (int32_t)(gap > INT32_MAX ? INT32_MAX : gap);
                }
                s_burst_run = gap < RX_BURST_US ? s_burst_run + 1 : 1;
            } else {
                s_burst_run = 1;    /* the first packet is a run of one */
            }
            if (s_burst_run > rx_burst_max) {
                rx_burst_max = s_burst_run;
            }
            s_prev_audio_at = t4;
            handle_audio((const audio_msg_t *)buf, t4);
#if CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0
        } else if (buf[0] == MSG_AUDIO_FEC &&
                   n >= (int)AUDIO_FEC_MSG_BYTES(0)) {

            fec_parity_rx((const audio_fec_msg_t *)buf, n);
#endif
        }
    }
}
