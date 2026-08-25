/**
 * @file sat.h
 * @brief The satellite's shared state, and which task is allowed to write it.
 *
 * Every value that crosses between the receive task, the playback task, the
 * probe task and the reporting task is declared here and defined once in
 * sat_state.c. Nothing is documented at its definition: the declaration is the
 * single place a field's meaning and its owner are written down.
 *
 * @section ownership Who writes what
 *
 * `volatile` here means "another task writes this", not "this is atomic".
 *
 * - **rx_task** writes the stream anchor and everything describing arrival:
 *   @ref stream_start_local, @ref anchor_at, @ref stream_offset,
 *   @ref stream_rate, @ref samples_in, @ref marker_sample, @ref restart_pos,
 *   the @ref phase_q head, the @ref tsf pair, @ref resync_request,
 *   @ref anchor_provisional, @ref audio_volume and the arrival gauges.
 * - **play_task** writes what playback has reached: the @ref phase_q tail,
 *   @ref phase_err_us, @ref phase_stepped, @ref phase_hist and the
 *   @ref phase_med_us pair it publishes from it, the
 *   `splice_report_*` and `step_report_*` sets, @ref playing,
 *   @ref ring_low_ms and the refill pair. It reads @ref rate_trim_hz.
 * - **drift_task** runs @ref telemetry_tick() then @ref servo_tick(). The
 *   servo writes @ref rate_trim_hz and the `retune_*` set; telemetry writes
 *   the windowed heap figures and **clears the arrival gauges as it reads
 *   them**.
 * - **probe_task** writes the self-mute set — @ref self_muted,
 *   @ref n_self_mutes, @ref n_self_retries — and carries
 *   @ref wifi_retry_tick().
 * - **the WiFi event handler** writes @ref n_wifi_drops, @ref wifi_down_at,
 *   @ref rejoined_at and net.c's own reconnect state.
 *
 * Several fields have two writers by design, and each says so where it is
 * declared: @ref stream_start_local, @ref marker_sample, @ref restart_pos,
 * @ref resync_request, @ref catchup_frames, @ref phase_valid,
 * @ref phase_stepped and @ref est_newest_at.
 *
 * @section tearing What a torn read costs
 *
 * A 64-bit load is two instructions on this CPU, so a reader can catch half of
 * a write. It only matters when the write changes the HIGH word: for a
 * monotonic microsecond clock that is once per 71.6 minutes, but for anything
 * set to zero, or crossing zero, it is every time.
 *
 * - @ref tsf is under a sequence lock — an offset hovers near zero and changes
 *   sign, and its two halves only mean anything together.
 * - @ref marker_sample and @ref samples_in are 32-bit to avoid the problem.
 * - @ref stream_start_local, @ref anchor_at, @ref wifi_down_at and
 *   @ref rejoined_at are exposed and deliberately left so; each says what a
 *   torn read costs there.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "driver/i2s_std.h"
#include "esp_attr.h"
#include "sdkconfig.h"

#include "sync_proto.h"
#include "audio_shift.h"

/** @brief ESP_LOG tag for the whole firmware. `"sat"`. */
extern const char *TAG;

/**
 * @def AP_SSID
 * @brief The hub's SoftAP name.
 *
 * Declared once in components/dancefloor_sync/Kconfig, because it is one link
 * seen from both ends and nothing would check two copies agreed.
 */
#define AP_SSID    CONFIG_DANCEFLOOR_AP_SSID
/** @brief The hub's SoftAP passphrase. Shared with the hub via Kconfig. */
#define AP_PASS    CONFIG_DANCEFLOOR_AP_PASS
/** @brief The hub's address: the esp_netif SoftAP default. */
#define MASTER_IP  "192.168.4.1"

/** @brief How often @ref probe_task() runs one tick: a clock probe, the WiFi
 *  retry check and one slot of the self-mute window. */
#define PROBE_PERIOD_MS 250

/**
 * @def RING_BYTES
 * @brief Capacity of the decoded-audio ring, from `DANCEFLOOR_RING_KB`.
 *
 * It must hold the hub's lead plus room for the servo to move around it. At
 * the default 96 kB that is 557 ms of 44.1 kHz stereo, against a
 * @ref RING_TARGET_MS of 350 and a depth-net excursion reaching 470 ms.
 *
 * The ceiling is a CONTIGUOUS block on a classic ESP32 with no PSRAM, taken at
 * boot when the largest block is at its biggest. There is headroom, but not
 * much: this is the first thing to suspect if the unit stops booting after the
 * ring or the lead is raised.
 */
#define RING_BYTES  (CONFIG_DANCEFLOOR_RING_KB * 1024)

/** @brief The one UDP socket, bound to `SYNC_PORT`. Every message in and out
 *  of this unit rides on it. `-1` until socket_start(). */
extern int sock;
/** @brief The probe clock estimator: round-trip samples, lowest-RTT selection.
 *  The fallback when TSF is unavailable. */
extern sync_est_t est;
/** @brief Decoded PCM between the receive task and the DAC. Written by rx_task,
 *  drained by play_task, sized @ref RING_BYTES. */
extern StreamBufferHandle_t ring;
/** @brief The I2S transmit channel. */
extern i2s_chan_handle_t i2s_tx;

/**
 * @brief Local-clock instant at which the next byte entering the ring should
 *        reach the DAC. Zero means playback has not started.
 *
 * @warning TWO WRITERS, and a seqlock would be the wrong fix for it. rx_task
 * writes the anchor instant; play_task writes 0 when it parks. The exposure is
 * small — play's write of 0 can be seen by the servo as a large nonzero value,
 * costing one 5 s window in which it believes a stream is running, and rx's
 * write only tears harmfully across a 71.6-minute boundary. Giving it one
 * owner is a design change, not an annotation.
 */
extern volatile int64_t stream_start_local;

/**
 * @brief When the current stream was anchored.
 *
 * Read by the servo, which holds its depth safety net off for
 * @ref DEPTH_NET_HOLD_US after it. One writer, and the only reader uses it as
 * `now - anchor_at < DEPTH_NET_HOLD_US`, so a torn read costs one window of
 * the net being wrongly held or released.
 */
extern volatile int64_t anchor_at;
/** @brief Sample rate of the stream being received, from the anchor packet.
 *  44100 until a stream says otherwise. */
extern volatile uint32_t stream_rate;
/** @brief What the output clock is actually set to. Moved only by
 *  @ref retune_output(); starts at 44100 before any stream exists. */
extern uint32_t tx_rate;

/**
 * @brief Local-to-master conversion used by playback.
 *
 * Seeded at anchoring and then slewed towards the live estimate — see
 * @ref track_offset(). Owned by the playback task once a stream is running;
 * rx_task writes it only while playback is parked waiting for one.
 */
extern int64_t stream_offset;
/** @brief When @ref track_offset() last moved @ref stream_offset. Zero means
 *  this stream has not been looked at yet. */
extern int64_t offset_slew_last;

/**
 * @brief Ring frame position of a tagged packet, or -1 for none.
 *
 * @note 32-bit, not 64, deliberately: play_task reads this while rx_task
 * writes it, and a 64-bit load is two instructions here — a torn read would
 * yield a garbage position and a wild marker. An `int32` holds over 13 hours
 * of frames at 44.1 kHz.
 *
 * rx_task sets it; play_task clears it back to -1 once the marker has fired.
 */
extern volatile int32_t marker_sample;
/** @brief Frames written into the ring since the anchor. 32-bit for the reason
 *  @ref marker_sample is. Silence inserted for a lost packet counts here, or
 *  every later position would be displaced by it. */
extern volatile int32_t samples_in;

/** @brief Slots in @ref phase_q. */
#define PHASE_Q_LEN 32
/**
 * @brief One recorded packet boundary: where its audio lands, and when it is
 *        due.
 *
 * Servoing on buffer depth matches the playback RATE to the arrival rate but
 * says nothing about POSITION, and depth moves with network jitter — so two
 * units seeing different jitter end up at slightly different rates and drift
 * apart while each one's own buffer looks perfectly stable. Every packet says
 * exactly when its first sample should play, so recording that against the
 * ring position it lands at gives a direct position measurement when playback
 * reaches it.
 */
typedef struct {
    int32_t pos;        /**< Ring frame position where this packet's audio starts. */
    int64_t play_at;    /**< Master-clock instant that sample should be heard. */
} phase_pt_t;

/** @brief The recorded boundaries. Single producer (rx_task), single consumer
 *  (play_task), 32-bit indices, so no lock is needed. */
extern phase_pt_t phase_q[PHASE_Q_LEN];
/** @brief Write index into @ref phase_q. rx_task only. */
extern volatile uint32_t phase_head;
/** @brief Read index into @ref phase_q. play_task only. */
extern volatile uint32_t phase_tail;
/** @brief Position error at the last crossing, in microseconds. Positive means
 *  playing late. Written by play_task, read by the servo. */
extern volatile int32_t phase_err_us;
/** @brief Whether @ref phase_err_us describes anything yet. play_task sets it
 *  at the first crossing; both it and rx_task clear it when a stream ends or a
 *  new one is anchored. */
extern volatile bool phase_valid;

/**
 * @brief The last few raw phase readings, for the boundary splice.
 *
 * The splice acts on `sync_phase_median()` of this, falling back to the raw
 * reading when the history is too short — a boundary correction is the largest
 * single move this unit makes, and one noisy reading should not size it.
 *
 * @warning PLAY TASK ONLY. It is a plain non-`volatile` struct and it is reset
 * under its own task's feet at every splice, so nothing on another task may
 * read it — the servo takes @ref phase_med_us instead.
 */
extern sync_phase_hist_t phase_hist;

/**
 * @brief The median of @ref phase_hist, published for the servo.
 *
 * The servo runs on drift_task and cannot read @ref phase_hist itself, so
 * play_task computes the median where it pushes the reading and publishes it
 * here. Computed on the play task rather than in the servo because it needs
 * the packet-cadence history — the readings it summarises span a couple of
 * hundred milliseconds, which is far shorter than a servo window.
 *
 * Single writer (play_task), single reader (the servo), 32-bit, so no lock is
 * needed.
 */
extern volatile int32_t phase_med_us;
/** @brief Whether @ref phase_med_us summarises this stream's current position.
 *  Cleared wherever @ref phase_hist is reset, because a median taken before a
 *  splice or a re-anchor describes a position the unit has left. */
extern volatile bool phase_med_valid;
/** @brief Ring frame position of a track boundary, or -1 for none. rx_task
 *  sets it, play_task clears it once the splice is done. */
extern volatile int32_t restart_pos;

/**
 * @brief Set when the phase genuinely stepped, so the servo re-seeds its
 *        smoothing instead of averaging across the step.
 *
 * A splice moves the unit at a stroke, so the running average from before it
 * describes a situation that no longer exists. Set by play_task at the events
 * that move it, cleared by the servo when it acts on it. Re-seeding on every
 * correction rather than only on a step would destroy the smoothing entirely.
 */
extern volatile bool phase_stepped;

/** @brief Never splice more than this in one go. A larger error means
 *  something a splice will not fix, and a 150 ms jump is very audible even at
 *  a track change. */
#define MAX_SPLICE_MS 150

/**
 * @def SPLICE_INSERT_HEADROOM_MS
 * @brief An insert may not push the ring past this far below capacity.
 *
 * The insert takes DAC time and consumes nothing from the ring, so while its
 * zeros play the receive path keeps pushing — inserting into an already-deep
 * ring drops audio at the far end as ring-full. The skip side needs no such
 * clamp: its discard loop reads with a zero timeout and stops on an empty
 * ring by construction.
 */
#define SPLICE_INSERT_HEADROOM_MS 50

/**
 * @def PHASE_INSANE_US
 * @brief Beyond this, the reading is not describing our playback at all.
 *
 * Drift is under a millisecond a minute and delivery jitter is a few
 * milliseconds, so a whole second can be neither. What it means is that the
 * stamps arriving now were issued against a different clock origin than the
 * one playback anchored to. Servoing on that is meaningless; re-anchoring is
 * the only thing that helps.
 */
#define PHASE_INSANE_US 1000000

/**
 * @def ANCHOR_MIN_LEAD_US
 * @brief How much of the hub's lead must survive the trip for a packet to be
 *        worth anchoring on.
 *
 * The scheduled wait before playback starts is the ONLY thing that prefills
 * the ring, so whatever lead reaches here IS the prefill. Anchoring on a
 * packet that has almost none starts the stream with a buffer it cannot keep,
 * and the servo then spends minutes walking the error back.
 *
 * @note Tied by convention to the hub's `LEAD_US`, which this unit cannot see
 * — it is compiled into a different image. If the lead changes, this is the
 * second place to look.
 */
#define ANCHOR_MIN_LEAD_US     125000
/** @brief At most one anchor per second. Against a hub supplying roughly fifty
 *  packets in that time, if none of them anchors then the link is not in a
 *  state a re-anchor can fix. */
#define ANCHOR_MIN_INTERVAL_US 1000000
/** @brief After this long refusing, anchor anyway and set
 *  @ref anchor_provisional. Long enough that no plausible burst of lateness
 *  reaches it, short enough that a lead mismatch does not leave a speaker
 *  silent for a whole track. */
#define ANCHOR_GIVE_UP_US      5000000

/**
 * @def GAP_RESYNC_MS
 * @brief A gap beyond this is an outage, not jitter: re-anchor rather than
 *        fill it with silence.
 *
 * 150 ms is about seven packets at the ~50/s the audio lane runs at. Normal
 * loss on a healthy link is one to three, so this sits well clear of anything
 * that should be concealed, while staying below the @ref RING_TARGET_MS a fill
 * that size would otherwise push the ring past.
 */
#define GAP_RESYNC_MS 150

/**
 * @def DEPTH_NET_HOLD_US
 * @brief How long after an anchor the servo ignores buffer depth and servos on
 *        phase alone.
 *
 * A fresh stream starts below target by construction — the prefill is at most
 * the lead minus transit, and playback begins consuming while the rest of the
 * stream is still in flight. Without the hold the depth net fires on that and
 * drags the rate to rescue a ring that was never in trouble.
 *
 * 20 s is four servo windows and matches the retune cooldown, by which point
 * the phase measurement is trustworthy and is the better input anyway. No
 * underrun protection is given up: an actually empty ring is caught by the
 * playback task's 500 ms receive timeout, which is a different mechanism and
 * still armed.
 */
#define DEPTH_NET_HOLD_US      20000000

/**
 * @brief Largest phase step seen this window, for @ref telemetry_tick() to
 *        narrate. Written by play_task.
 *
 * The play task is the audio path and must not log: at 115200 baud a status
 * line is milliseconds of blocking UART against a chunk that is 5.8 ms of
 * audio, and steps arrive in bursts. Largest-per-window, because a burst of
 * ten says the same thing as its biggest member.
 */
extern volatile int32_t step_report_mag;
/** @brief Phase before the step being reported, in microseconds. */
extern volatile int32_t step_report_from;
/** @brief Phase after the step being reported, in microseconds. */
extern volatile int32_t step_report_to;
/** @brief Ring depth in ms when the step happened — a step with an empty ring
 *  is a different fault from one with a full ring. */
extern volatile int32_t step_report_ring;
/** @brief @ref rate_trim_hz at the moment of the step. */
extern volatile int32_t step_report_trim;
/** @brief Cumulative padded frames at the moment of the step. Telemetry prints
 *  the delta against what it last said, not this total. */
extern volatile uint32_t step_report_pad;
/** @brief Set by play_task when a step is worth narrating; cleared by
 *  telemetry once it has. */
extern volatile bool    step_report_pending;

/** @brief The correction a track-boundary splice applied, in microseconds. */
extern volatile int32_t splice_report_us;
/** @brief The phase reading the splice was computed from. */
extern volatile int32_t splice_report_phase;

/**
 * @brief The counterfactual: what the RAW reading would have spliced by.
 *
 * The splice itself acts on the median. This carries the unsmoothed value so
 * the hub can print both at the same boundary and the two units' figures are
 * comparable. Acted on by nothing here.
 */
extern volatile int32_t splice_report_alt;
/** @brief A splice waiting to be sent to the hub. Written by play_task, sent
 *  by probe_task — a sendto() in the audio path is exactly the kind of thing
 *  that costs a buffer. */
extern volatile bool    splice_report_pending;

/**
 * @def TSF_READ_TRIES
 * @brief How many times @ref tsf_read() retries before giving up.
 *
 * Real contention needs one retry: the writer's critical section is four
 * stores and two fences, far shorter than a reader's pass. Eight is well past
 * any legitimate collision, so reaching the end of them means the writer is
 * not running — which is the pathological case this bound exists for, not a
 * busy one to be waited out.
 */
#define TSF_READ_TRIES 8
/**
 * @brief The TSF-derived clock offset and when it was taken, under a sequence
 *        lock.
 *
 * Read and written as a PAIR, and never directly — use @ref tsf_publish() and
 * @ref tsf_fresh(). Two things go wrong with a plain pair of volatiles:
 *
 * - **Tearing.** Both are 64-bit and a 64-bit load is two instructions here.
 *   An offset hovers near zero and changes sign, which flips the high word
 *   between 0 and 0xFFFFFFFF — exactly the case where the halves do not belong
 *   together and the result is close to neither value.
 * - **Pairing**, which no amount of per-field atomicity would fix. A reader
 *   that checks `at`, decides the reading is fresh, and then loads the offset
 *   can be interrupted between the two and get a fresh timestamp with the
 *   offset from before it. That offset is what a stream anchors on, where an
 *   error is baked in for the life of the stream and every log line downstream
 *   is derived from the same wrong number.
 *
 * @warning A seqlock is correct here because there is exactly ONE writer
 * (rx_task, in the MSG_TSF arm). Do not add a second without replacing this: a
 * seqlock with two writers is silently broken.
 */
typedef struct {
    volatile uint32_t seq;      /**< Odd while a write is in progress. */
    volatile int64_t  offset_us; /**< Local-to-master offset, in microseconds. */
    volatile int64_t  at;       /**< Local time the offset was derived; 0 = never. */
} tsf_reading_t;

/** @brief The current TSF reading. Use the accessors, not the fields. */
extern tsf_reading_t tsf;

/**
 * @brief Publish a TSF offset. Writer side of the seqlock; rx_task only.
 *
 * @param offset_us Local-to-master offset in microseconds.
 * @param at        Local time the offset was derived.
 */
static inline void tsf_publish(int64_t offset_us, int64_t at)
{
    tsf.seq++;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    tsf.offset_us = offset_us;
    tsf.at = at;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    tsf.seq++;
}

/**
 * @brief Reads that gave up after @ref TSF_READ_TRIES and fell back to the
 *        estimator.
 *
 * @warning MUST BE ZERO. The writer's critical section is a handful of stores,
 * so losing eight races in a row does not happen to a healthy system. Any
 * movement means the writer was unable to run while a reader wanted it. The
 * fix for a rate here is to stop pinning playback above the writer, not to
 * raise @ref TSF_READ_TRIES.
 */
extern volatile uint32_t n_tsf_read_fail;

/**
 * @brief Read the offset and its timestamp as one consistent pair.
 *
 * @param[out] offset_us The published offset, untouched on failure.
 * @param[out] at        When it was derived, untouched on failure.
 * @return true if a pair that was published together was caught.
 *
 * @note THE RETRY IS BOUNDED, and that is load-bearing rather than
 * belt-and-braces. `seq` is odd for exactly the window between the writer's
 * two increments, and an unbounded spin waiting for it to go even deadlocks:
 * play_task is priority 8 pinned to core 1 and rx_task, the writer, is
 * priority 7 and unpinned, so a publish executing on core 1 when playback
 * becomes ready is preempted by it — and the reader spinning is what prevents
 * the writer from ever completing. Giving up instead costs one chunk's worth
 * of the estimator, which is the fallback that already exists for TSF being
 * unavailable and which every caller already handles.
 */
static inline bool tsf_read(int64_t *offset_us, int64_t *at)
{
    for (int t = 0; t < TSF_READ_TRIES; t++) {
        const uint32_t s0 = tsf.seq;
        if (s0 & 1u) {
            continue;
        }
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        const int64_t us = tsf.offset_us;
        const int64_t a  = tsf.at;
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        if (s0 == tsf.seq) {
            *offset_us = us;
            *at = a;
            return true;
        }
    }
    n_tsf_read_fail++;
    return false;
}

/** @brief Anchors that used a TSF offset. */
extern volatile uint32_t n_tsf_used;
/** @brief Anchors that fell back to the probe estimator. */
extern volatile uint32_t n_tsf_fallback;

/** @brief How stale a TSF offset may be and still beat the estimator. Messages
 *  arrive with every probe reply, 4/s, so a second means several have been
 *  missed and the link is not healthy enough to trust the last one. */
#define TSF_MAX_AGE_US 1000000

/**
 * @brief True if a TSF reading exists and is fresh enough to prefer over the
 *        estimator, and if so what it is.
 *
 * @param now              Current local time.
 * @param[out] offset_us   The offset, when the return is true. May be NULL.
 * @return true if @p offset_us was set from a fresh reading.
 *
 * The freshness test and the value it vouches for come out of ONE
 * @ref tsf_read(), which is the whole point of the pair.
 */
static inline bool tsf_fresh(int64_t now, int64_t *offset_us)
{
    int64_t us, at;
    if (!tsf_read(&us, &at)) {
        return false;
    }
    if (at && now - at < TSF_MAX_AGE_US) {
        if (offset_us) *offset_us = us;
        return true;
    }
    return false;
}

/** @brief Playback ran dry. Cumulative for the whole run, never reset — the
 *  5 s lines answer "what is happening now", only a total answers "has this
 *  been happening slowly for an hour". */
extern volatile uint32_t n_underruns;
/** @brief Streams anchored, the first one included. */
extern volatile uint32_t n_reanchors;
/** @brief Track-boundary corrections applied. */
extern volatile uint32_t n_splices;
/** @brief Output-clock retunes performed. */
extern volatile uint32_t n_retunes;
/** @brief Retunes refused because the rate asked for was outside the trim's
 *  range. Nothing the servo computes may panic the speaker, so an absurd rate
 *  is counted and dropped rather than checked with ESP_ERROR_CHECK. */
extern volatile uint32_t n_retunes_bad;

/**
 * @brief Gaps the air made, counted at DETECTION, before anything tries to
 *        repair them.
 *
 * This measures the LINK, not the concealment — which is what makes a parity
 * build and a bare build comparable on it. A gap that FEC repairs still counts
 * here and adds nothing to @ref n_gap_frames, so the printed line says both
 * how much was lost and how much of it was heard.
 *
 * @see n_fec_recovered for the identity these satisfy.
 */
extern volatile uint32_t n_gaps;

/**
 * @brief Packets rebuilt WHOLE by XOR parity.
 *
 * "Whole" is the point: `audio_fec_extract()` either returns the packet that
 * went missing or refuses, so there is no partial recovery and
 * `n_gaps - n_fec_recovered` is exactly the silence that reached the room.
 *
 * @note @ref n_gaps == @ref n_fec_recovered + @ref n_fec_lost across any
 * window, by construction: every detected gap is offered to parity exactly
 * once and lands in one column or the other. That identity is the point — a
 * burst loss parity cannot touch would otherwise appear in neither column and
 * the repair rate would read better than it was.
 */
extern volatile uint32_t n_fec_recovered;
/** @brief Holes parity could not close: two losses in one group, a gap of more
 *  than one packet, a parity that never arrived, or a resync that threw the
 *  group away. Not a fault — the fraction of losses the scheme does not
 *  cover. */
extern volatile uint32_t n_fec_lost;
/** @brief How often packets were held behind a hole waiting for a parity. Read
 *  against @ref n_fec_recovered — holds that did not become recoveries are
 *  @ref n_fec_lost, and a hold costs ring depth whichever way it ends. */
extern volatile uint32_t n_fec_holds;
/** @brief Parity datagrams taken. Arrival proof for the parity lane itself: it
 *  should sit at the audio rate divided by K, and a zero here with audio
 *  flowing says the hub is not sending parity at all — which no other counter
 *  would distinguish from a channel that never lost anything. */
extern volatile uint32_t n_fec_parity_rx;
/** @brief A parity that arrived and could not be trusted: a count field
 *  disagreeing with this build's K, or a rebuilt header that is not the packet
 *  that went missing. Must stay zero; both mean the two firmwares disagree
 *  about the wire, and neither is a radio problem. */
extern volatile uint32_t n_fec_bad;

/** @brief Re-anchors forced because a gap fill did not fit the ring. Distinct
 *  from @ref n_gap_resyncs — that says the air was bad, this says the ring
 *  could not absorb a burst arriving faster than it plays. */
extern volatile uint32_t n_gap_short_resyncs;
/** @brief Packets older than expected. Should be 0: the hub sends each once.
 *  Non-zero means duplication or reordering — and a reorder means the "gap"
 *  before it was never a loss, so silence was inserted against a packet that
 *  did arrive. */
extern volatile uint32_t n_seq_dropped;
/** @brief Live-stream SBC frames that would not decode. The rest of that
 *  packet is dropped, so the timeline is short by whatever it held and no
 *  other counter sees it. */
extern volatile uint32_t n_decode_err;
/** @brief recvfrom() returning an error rather than a datagram. */
extern volatile uint32_t n_recv_err;
/** @brief Disconnects from the hub's AP. */
extern volatile uint32_t n_wifi_drops;

/**
 * @brief Associations that never produced a DHCP lease and were torn down.
 *
 * Separate from @ref n_wifi_drops because the two say opposite things about
 * where to look. A drop is the radio losing a link it had; this is the radio
 * perfectly happy with a link that is useless — associated, no address, probes
 * going nowhere. Each one is followed by a disconnect this unit asked for, so
 * @ref n_wifi_drops counts it too: drops well above lease failures is an
 * ordinary flaky link, drops tracking them one for one is this.
 */
extern volatile uint32_t n_wifi_lease_fail;

/**
 * @brief Frames of silence actually written into the ring. A repaired gap adds
 *        none.
 *
 * These counters exist instead of a log line in the receive path. Each event
 * used to be an ESP_LOGW from rx_task, which is the only thing draining a UDP
 * mailbox six datagrams deep — and the console is a 115200-baud UART, so a
 * line is milliseconds of blocking write. That closes a loop: loss makes
 * lines, lines block the receive task, a blocked receive task overflows the
 * mailbox, and the overflow is more loss. So the audio path increments and
 * @ref telemetry_tick() talks, from a task that can afford to wait on a UART.
 */
extern volatile uint32_t n_gap_frames;
/** @brief Gap fills the ring could not take. */
extern volatile uint32_t n_gap_short;
/** @brief Frames the short fills came up short by. */
extern volatile uint32_t n_gap_short_frames;
/** @brief Decoded blocks dropped because the ring was full. */
extern volatile uint32_t n_ring_full;
/** @brief Gaps too large to fill, re-anchored instead. @see GAP_RESYNC_MS */
extern volatile uint32_t n_gap_resyncs;
/** @brief Provisional anchors later replaced by a proper one. */
extern volatile uint32_t n_anchor_upgrades;

/** @brief Set by rx_task when a gap is too large to fill; cleared by play_task
 *  when it parks. Two writers, and deliberately so — the consumer clearing it
 *  is what stops the reset racing a blocked reader. @see GAP_RESYNC_MS */
extern volatile bool resync_request;

/** @brief Set when @ref ANCHOR_GIVE_UP_US forced an anchor onto a packet that
 *  was already late. Playback runs, but its position is known to be wrong, so
 *  the receive path keeps watching for a packet it could have anchored on
 *  properly. */
extern volatile bool anchor_provisional;
/** @brief Anchors refused because `play_at` was already past. */
extern volatile uint32_t n_anchor_late;
/** @brief Anchors refused because one had just happened.
 *  @see ANCHOR_MIN_INTERVAL_US */
extern volatile uint32_t n_anchor_soon;

/** @brief Phase points dropped because @ref phase_q was full. A full queue
 *  means playback is not consuming points as fast as the receive path records
 *  them, so the servo silently stops getting fresh input while every log line
 *  still reads normally. */
extern volatile uint32_t n_phase_drop;

/** @brief Ring reads that came back short of a full chunk. The ring's trigger
 *  level is one chunk, so a short read means the receive timeout expired on a
 *  partly-filled ring — a near-underrun. */
extern volatile uint32_t n_short_reads;
/** @brief Frames of silence padded in to cover those short reads. Playback
 *  advances by what came out of the ring, so this is what the bias WOULD have
 *  been had it advanced by a whole chunk regardless. */
extern volatile uint32_t n_short_frames;

/** @brief Sentinel for the two minima below: "nothing measured this window",
 *  which is distinct from a measured zero. The line prints text rather than a
 *  number nothing stands behind. */
#define ARRIVAL_UNSEEN INT32_MAX
/** @brief Audio datagrams taken. A plain cumulative counter, differenced for
 *  the window. */
extern volatile uint32_t n_audio_rx;
/**
 * @brief Longest silence between two audio datagrams, this window.
 *
 * @note A GAUGE, NOT A COUNTER: it is the extreme seen since telemetry last
 * read it, and telemetry CLEARS it as it reads — a maximum summed across
 * windows is not a maximum of anything. rx_task is the only other writer, so
 * the race is one-sided: a clear landing between a reader's load and its store
 * costs one window one sample, a price a diagnostic can pay and a counter
 * cannot.
 */
extern volatile int32_t  rx_gap_max_us;
/** @brief Longest run of datagrams arriving less than @ref RX_BURST_US apart.
 *  A gauge, cleared by telemetry. */
extern volatile uint32_t rx_burst_max;
/**
 * @brief Least of `play_at - arrival`, in master microseconds: how much of the
 *        hub's lead survived the trip. A gauge, cleared by telemetry.
 *
 * This is the discriminator for a gap in arrivals. If the gap comes with a
 * COLLAPSED lead, the hub stamped those packets on time and the transport held
 * them; if the lead is still near the full lead, they were stamped late and
 * the fault is upstream of the air. Nothing else on this unit tells those
 * apart. @ref ARRIVAL_UNSEEN means nothing was measured.
 */
extern volatile int32_t  rx_lead_min_us;
/** @brief Lead readings refused as not-a-lead. @see LEAD_INSANE_US */
extern volatile uint32_t n_lead_insane;
/** @brief Shallowest the play task found the ring, in ms. A gauge, cleared by
 *  telemetry; @ref ARRIVAL_UNSEEN means nothing was measured. */
extern volatile int32_t  ring_low_ms;

/**
 * @brief Longest a packet waited behind a hole for its parity, this window.
 *
 * The number the parity design rests on. A loss can only be repaired once the
 * group's parity arrives, so the receiver holds the packets behind the hole
 * until it does — and every claim about whether that is affordable is a claim
 * about this figure. The arithmetic says (K-2) packet times, which at K=4 and
 * ~50 packets/s is about 40 ms against a 350 ms @ref RING_TARGET_MS.
 *
 * A gauge rather than an assertion, because the arithmetic assumes the hub
 * sends parity promptly after the group's last packet and nothing here can
 * know that it did. Read beside @ref ring_low_ms — the hold is the one
 * mechanism in the receive path that deliberately stops writing to the ring.
 * Zero is the normal reading — a hold only starts when a packet is lost.
 */
extern volatile int32_t  fec_hold_max_us;

/** @brief Close enough together to be one release rather than two arrivals.
 *  Packets are stamped ~20 ms apart and paced by the hub's DAC, so anything
 *  under a millisecond or two came out of a queue that had been holding it. */
#define RX_BURST_US 2000

/**
 * @def LEAD_INSANE_US
 * @brief Beyond this a lead reading is REFUSED rather than clamped.
 *
 * At that size the number is not describing delivery, it is describing a
 * timeline this unit is no longer on — the same condition
 * @ref PHASE_INSANE_US names, which is why it is the same bound. Refusing
 * matters because a gauge that clamps lies about its own worst case: one
 * absurd sample would otherwise own the window's minimum and the whole run's
 * summary. @see n_lead_insane
 */
#define LEAD_INSANE_US PHASE_INSANE_US

/** @brief How long a TSF read pair may take before the sample is suspect —
 *  something preempted between the two counter reads, so the offset is off by
 *  whatever landed in the gap. @see n_tsf_wide */
#define TSF_SPAN_MAX_US 100
/** @brief TSF samples whose read pair exceeded @ref TSF_SPAN_MAX_US. COUNTED,
 *  NOT ENFORCED: TSF is the anchor clock source, and a threshold chosen blind
 *  could silently demote it to the probe estimator, which is worse. This says
 *  what the reject rate WOULD be, so a threshold can be chosen from the
 *  distribution instead. */
extern volatile uint32_t n_tsf_wide;

/** @brief When this unit went off the air. Only the first drop of a streak
 *  dates it: measuring the last retry is not measuring time off the air.
 *  Feeds log lines only, so a torn read prints one wrong number and no servo
 *  acts on it. */
extern volatile int64_t wifi_down_at;
/** @brief When it came back; 0 means the next anchor is not the first after a
 *  rejoin. Nothing invalidates the probe estimator's window on a disconnect,
 *  so the first anchor after a rejoin reports which clock it used and how
 *  stale the estimator's newest sample was. */
extern volatile int64_t rejoined_at;
/** @brief When the newest probe reply landed. Written by both clock.c (cleared
 *  on un-mute) and rx.c (set on each reply). */
extern volatile int64_t est_newest_at;
/** @brief Analysis frames taken from the hub. Counted in frames, not
 *  datagrams, so the rate is comparable with the hub's own. */
extern volatile uint32_t n_frames_rx;
/** @brief Analysis frames rejected for being the wrong size. */
extern volatile uint32_t n_frames_bad;
/** @brief Stack headroom of the play task, in bytes. Sampled in-task, because
 *  that is the only place the figure is valid. */
extern volatile uint32_t hw_play;
/** @brief Stack headroom of the drift task, in bytes. */
extern volatile uint32_t hw_drift;

/** @brief Least total free heap seen this window; `UINT32_MAX` until sampled.
 *  A windowed minimum says nothing, every minute, until the minute it does. */
extern volatile uint32_t heap_min_window;

/**
 * @def CAP_USABLE_INTERNAL
 * @brief The pool ordinary allocations actually draw from.
 *
 * `MALLOC_CAP_INTERNAL` alone is NOT that pool. On the classic ESP32 the IRAM
 * heap is registered as INTERNAL|EXEC|32BIT — internal, but neither 8-bit
 * accessible nor DEFAULT, so nothing needing byte access can touch it. A task
 * stack cannot; malloc() cannot. Adding 8BIT is what makes the figure describe
 * memory a stack can be cut from, and it is exactly the mask of the requests
 * that fail.
 */
#define CAP_USABLE_INTERNAL (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)

/** @brief Least free @ref CAP_USABLE_INTERNAL seen this window;
 *  `UINT32_MAX` until sampled. */
extern volatile uint32_t heap_int_window;
/** @brief Allocations that failed, via the heap's failure hook. */
extern volatile uint32_t n_alloc_fail;
/** @brief The largest request that failed. */
extern volatile uint32_t alloc_fail_size;
/** @brief The capability mask that request asked for. */
extern volatile uint32_t alloc_fail_caps;

/**
 * @brief Tasks that did not start.
 *
 * Every xTaskCreate here reads its return value, because discarding it made a
 * crippled unit indistinguishable from a quiet one: still associated, still
 * holding a lease, no audio, no LEDs, and — if the missing task was the drift
 * task — no HEALTH line to say otherwise.
 */
extern volatile uint32_t n_task_fail;
/** @brief Names of the tasks that failed, space separated. Kept so the GOT_IP
 *  handler can say them once the radio can carry them: app_main runs about a
 *  second before DHCP completes, so anything logged at the failure itself is
 *  dropped for want of a route. */
extern char s_task_fail_names[64];

/**
 * @def RING_TARGET_MS
 * @brief Target buffer depth: the hub stamps audio this far ahead, so in
 *        steady state that much should be sitting here waiting.
 *
 * @warning IT IS THE HUB'S `LEAD_US` AND MUST TRACK IT. This unit cannot see
 * that constant — it is compiled into a different image — so the two are held
 * equal by convention, and a mismatch does not fail loudly: the depth net
 * would simply hold this unit at the wrong depth, which reads as a standing
 * phase error nothing explains.
 *
 * It also drags @ref RING_BYTES with it. The depth net clamps at ±120 ms, so
 * an ordinary excursion sits at 470 ms — which must fit inside the ring, or
 * normal running drops audio as ring-full.
 */
#define RING_TARGET_MS 350

/** @brief Below this, an i2s_channel_write() did not block, so the DAC was not
 *  pacing it. @see s_refill_active */
#define REFILL_FAST_US 1000
/**
 * @brief Whether the refill instrument is armed. Play task only, so no
 *        volatile.
 *
 * i2s_channel_write() does not block until the descriptors are full, so the
 * first writes after the channel has been idle return at memory speed:
 * playback's position advances by the whole DMA depth against a write time
 * that has barely moved, and every phase reading dated inside that window is
 * measured against a reference the DAC is not pacing. This sizes how much
 * audio a playback START puts in before pacing begins.
 */
extern bool    s_refill_active;
/** @brief Frames written unpaced since the instrument was armed. Play task
 *  only. */
extern int32_t s_refill_frames;

/**
 * @brief The FINE rate correction, in Hz. Written by the servo, read by
 *        playback.
 *
 * It names a rate the DAC is NOT running at: the clock stays put and playback
 * consumes the ring at an effective `tx_rate + rate_trim_hz` instead, dropping
 * one frame when it needs to get through the stream faster and duplicating one
 * when it needs to get through it slower. Positive means playing late, so
 * consume faster.
 *
 * Worth the trouble because the alternative is a clock retune, which takes the
 * I2S channel down for milliseconds to apply a few Hz against a few ppm of
 * real drift. One frame in tens of thousands is inaudible, and unlike a retune
 * it is continuous rather than stepped.
 *
 * Identical to the hub's, deliberately and to the arithmetic: unequal
 * correction between the two units is a cross-unit sync error by construction.
 * Persists across a playback restart, as @ref tx_rate does; zeroed only when a
 * coarse retune moves the clock, because the clock then carries what this was
 * carrying.
 */
extern volatile int32_t rate_trim_hz;

/**
 * @brief Playback volume, 0-`AUDIO_VOL_MAX`, written by rx_task and read by
 *        playback.
 *
 * Meaningless until @ref audio_vol_known. The hub sends full-scale audio and
 * this level separately, and both units run the same integer taper on it, so
 * two speakers told the same number produce the same level.
 *
 * @warning WRITE ORDER IS LOAD-BEARING: rx_task stores the level and only then
 * sets @ref audio_vol_known. Both are single-byte volatile stores, so a reader
 * that observes the flag has necessarily observed the level that goes with it,
 * and no lock is needed for a pair this shape.
 */
extern volatile uint8_t audio_volume;

/** @brief Whether anything has ever said how loud, this boot. Sticky: a hub
 *  going away does not make the last level wrong, it takes the audio with it.
 *  The fallback for a hub that never speaks at all is a deadline in the play
 *  task, not a reset of this. */
extern volatile bool audio_vol_known;

/** @brief MSG_VOL messages taken, REPEATS INCLUDED. Counted before the change
 *  test, not after: the fault this makes visible is a unit that hears audio
 *  and never hears a level, whose symptom is silence on this counter while the
 *  level it plays at is stale. A counter that only moved on change could not
 *  tell that from a level nobody has touched. */
extern volatile uint32_t n_vol_rx;

/** @brief Frames the fine trim has dropped from the stream. The rate is
 *  `|rate_trim_hz|` frames per second by construction, so flat while the trim
 *  is non-zero means the trim is not running. */
extern volatile uint32_t n_trim_drops;
/** @brief Frames the fine trim has duplicated into the stream. Climbing
 *  together with @ref n_trim_drops means the servo is hunting across zero. */
extern volatile uint32_t n_trim_dups;

/**
 * @brief The catch-up debt, in signed FRAMES: positive means skip that many
 *        (playing late), negative means replay that many (early).
 *
 * The servo ARMS it — raises it toward the measured error, or clears it under
 * the threshold — and play_task is the only thing that shrinks it, one chunk's
 * shift at a time as the crossfade actually spends it. Two writers, and benign
 * in both directions: the next window re-arms from a fresh median either way.
 */
extern volatile int32_t  catchup_frames;
/** @brief Frames the catch-up has dropped. Kept apart from @ref n_trim_drops
 *  because it measures a different mechanism with a different expected rate:
 *  the trim runs at a few frames a second in normal service, the catch-up at
 *  up to a thousand for the few seconds a large error takes to drain. */
extern volatile uint32_t n_catchup_drops;
/** @brief Frames the catch-up has duplicated. */
extern volatile uint32_t n_catchup_dups;

/** @brief Phase reading taken immediately before a retune, so its cost can be
 *  reported as one number rather than a difference of two 5 s ticks. */
extern volatile int32_t retune_phase_before;
/** @brief Asks playback to report the next phase reading as a retune cost, and
 *  to withhold it from the servo. */
extern volatile bool    retune_watch;
/** @brief How long the channel was down across disable/reconfig/enable. Real
 *  time passes with no audio playing, so playback returns that far behind the
 *  timeline; the step IS the outage. Feeds log lines only. */
extern volatile int64_t retune_outage_us;

/** @brief When the retune finished. Feeds log lines only. */
extern volatile int64_t retune_done_at;
/** @brief How many further crossings to narrate after a retune, to show how
 *  far its tail reaches. Measurement only — the servo withholds exactly one
 *  reading regardless. Must be armed after @ref retune_phase_before and
 *  @ref retune_watch, not before. */
extern volatile uint8_t retune_tail_left;

/**
 * @brief Held across a retune; the playback task parks on it.
 *
 * i2s_channel_write() returns IMMEDIATELY once the channel is disabled — it
 * does not block — so without this the play task runs flat out for the whole
 * outage: pulling chunks from the ring, counting them as played, feeding them
 * to the visualiser, and throwing them at a channel that is not running.
 * Milliseconds of outage cost tens of milliseconds of buffer.
 */
extern volatile bool retuning;

/**
 * @brief True while the play task is inside its write loop, i.e. while
 *        something is supposed to be feeding the DAC.
 *
 * Read from the I2S starve callback, which is the only reason it exists: a
 * starved channel is a fault only if a writer was meant to be keeping up with
 * it. Set by the play task on either side of that loop and by nothing else, so
 * it needs no lock — a bool cannot tear, and the worst a stale one costs is
 * one counted or uncounted starve at a park boundary.
 */
extern volatile bool playing;

/**
 * @def MUTE_WINDOW_US
 * @brief How long a unit must hear nothing before it takes itself off the
 *        hub's send list.
 *
 * @par Why a satellite mutes itself at all
 * On a shared radio a deaf satellite's failure is not private. A unit that can
 * no longer hear the hub may still be heard BY it — the hub has the better
 * antenna and receiver — so it stays registered and the hub goes on unicasting
 * audio to a station that will never acknowledge any of it. Every one of those
 * frames is retried to the limit, and on a half-duplex medium that airtime
 * comes out of every other speaker's share. ESP-IDF exposes no retry limit, so
 * the only remedy is to stop being sent to.
 *
 * @par Why the satellite decides and not the hub
 * The link is asymmetric: from the hub's side a deaf satellite looks perfectly
 * alive, because its probes keep arriving. Only the satellite can see that
 * nothing is coming back. Registration already supplies the mechanism — the
 * hub keeps a client for `CLIENT_TIMEOUT_US` after its last probe, so "stop
 * probing" IS "leave the send list", with no protocol change.
 *
 * Three seconds is long enough that a burst of loss or one re-anchor does not
 * reach it, and short enough that the floor is not held under while it decides.
 */
#define MUTE_WINDOW_US   3000000
/** @brief Audio datagrams in one @ref MUTE_WINDOW_US that count as "hearing
 *  something". About a tenth of what the window should deliver, not zero: the
 *  state being caught is not silence but a trickle, and a threshold of zero
 *  waits for the one window in four that happens to reach it. */
#define MUTE_AUDIO_MIN   15

/**
 * @def MUTE_RSSI_FLOOR
 * @brief The signal below which "no audio" means deaf rather than idle.
 *
 * This is the whole of the idle-floor defence. Between tracks nothing arrives
 * at ANY satellite, and muting then would cost up to @ref MUTE_RETRY_US of
 * silence at the start of every track on every speaker. A unit that hears the
 * AP well and receives nothing is idle; one that hears it barely is deaf. The
 * value sits in the gap between those two populations rather than near either
 * — it is not a sensitivity threshold and should not be tuned like one.
 */
#define MUTE_RSSI_FLOOR  (-85)

/**
 * @def MUTE_RSSI_REJOIN
 * @brief The signal at which a muted unit tries coming back.
 *
 * A muted unit is still associated, so it can read the AP's beacon —
 * `esp_wifi_sta_get_ap_info()` needs association, not a place on the send list
 * — and can watch for the antenna coming back without rejoining to find out.
 * While the signal stays down that costs the floor nothing at all, where a
 * blind timer would re-register the unit and resume the airtime theft for the
 * length of every trial.
 *
 * Set above @ref MUTE_RSSI_FLOOR deliberately: hysteresis, so a unit hovering
 * at the threshold cannot chatter across it.
 */
#define MUTE_RSSI_REJOIN  (-80)
/** @brief Consecutive ticks the signal must hold above @ref MUTE_RSSI_REJOIN,
 *  so one lucky beacon does not count. About 2 s at
 *  @ref PROBE_PERIOD_MS. */
#define MUTE_REJOIN_TICKS 8

/** @brief The fallback, and only that: if the signal never recovers, try
 *  anyway once a minute. An RSSI that cannot be read — or one that lies — must
 *  not be able to wedge a working speaker off the floor forever. Long, because
 *  this is the case where retrying is known to be futile. */
#define MUTE_RETRY_US   60000000

/** @brief Grace after un-muting, before starvation counts again. A rejoining
 *  unit is starving BY CONSTRUCTION — the hub has to notice its probe, the
 *  stream has to be re-anchored, and the ring has to fill — and judging it
 *  during that would re-mute every trial immediately. */
#define MUTE_TRIAL_US    5000000

/** @brief Ticks of @ref probe_task() in one @ref MUTE_WINDOW_US, which is the
 *  width of the arrival history the mute keeps. */
#define MUTE_SLOTS       (MUTE_WINDOW_US / (PROBE_PERIOD_MS * 1000))

/** @brief True while this unit is off the hub's send list. */
extern volatile bool     self_muted;
/** @brief Times it took itself off. */
extern volatile uint32_t n_self_mutes;
/** @brief Times it tried coming back. */
extern volatile uint32_t n_self_retries;

/**
 * @name Module entry points
 * One prototype per thing another module calls. Anything absent from this list
 * is private to its file and stays `static` there. These are documented here,
 * at the declaration, and not again at the definition.
 * @{
 */

/** @brief Bring up the WiFi station and join the hub's SoftAP. Registers the
 *  event handler that owns the reconnect state. */
void wifi_start_sta(void);
/** @brief Create and bind @ref sock. No multicast join is needed: audio
 *  arrives by unicast, and the probes this unit sends are what put it on the
 *  hub's send list. */
void socket_start(void);

/**
 * @brief Re-ask for the association when the backoff since the last drop is
 *        up, and tear down an association that never produced a lease.
 *
 * Must be called periodically or a disconnected unit never comes back;
 * @ref probe_task() carries it. It is not a task of its own because a
 * satellite that cannot probe cannot anchor and has no audio, so folding the
 * retry into that task adds no failure this unit could otherwise survive — and
 * it is not the vTaskDelay in the event handler it replaced, because that ran
 * on the default event loop's task and stopped esp_event dead for its
 * duration.
 */
void wifi_retry_tick(void);

/** @brief Open the I2S channel at @p rate and register the starve callback.
 *  @param rate Output sample rate in Hz. */
void i2s_start(uint32_t rate);

/** @brief How many times the DMA has run out of audio to send.
 *  @return A running total, not a rate; a retune contributes by construction. */
uint32_t dma_starve_count(void);
/**
 * @brief Apply the volume, widen to the output word, and write to the DAC.
 *
 * @param frames   Interleaved stereo PCM.
 * @param n_frames Frame count — NOT bytes. This is the one place the sample
 *                 width changes, so frame counts cross this boundary and byte
 *                 counts do not.
 * @param vol      Level to play at, 0-`AUDIO_VOL_MAX`.
 */
void write_audio(const int16_t *frames, size_t n_frames, uint8_t vol);

/** @brief Put the output gain back to silence, so a stream starts with a fade
 *  in rather than an edge. Called by playback when it begins feeding. */
void write_audio_reset_ramp(void);
/**
 * @brief Move the output clock, taking the channel down to do it.
 *
 * @param hz The rate to run at. Refused and counted as @ref n_retunes_bad if
 *           it is outside the trim's range of @ref stream_rate — nothing the
 *           servo computes may panic the speaker.
 *
 * The channel is DOWN across disable/reconfig/enable and real time passes with
 * no audio playing, so playback returns that far behind the timeline. Callers
 * must hold @ref retuning across it.
 */
void retune_output(uint32_t hz);
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
/** @brief Convert a master-clock instant to local time, for the strip.
 *  @param master_us The instant a frame names.
 *  @return The local time to draw it at; the identity before the first anchor. */
int64_t vis_master_to_local(int64_t master_us);
#endif

/** @brief Send a clock probe every @ref PROBE_PERIOD_MS, carry
 *  @ref wifi_retry_tick(), run the self-mute state machine, and forward
 *  finished splice reports to the hub. @param arg Unused. */
void probe_task(void *arg);
/**
 * @brief The local-to-master offset to use, and which clock it came from.
 *
 * @param[out] out      The offset in microseconds.
 * @param[out] used_tsf Set true if it came from TSF rather than the estimator.
 * @return false if no clock is usable yet.
 *
 * TSF is preferred because it needs no round trip and so carries no path
 * asymmetry. The probe estimator stands in when TSF reads zero — unassociated,
 * no beacon yet, or a hub that does not send MSG_TSF at all.
 */
bool clock_offset(int64_t *out, bool *used_tsf);
/** @brief Slew @ref stream_offset towards the live estimate. Held fixed, it
 *  would bias every phase measurement by exactly the drift the servo exists to
 *  correct; stepped, it would jump the timeline whenever min-RTT selection
 *  rotated a new sample into the window. */
void track_offset(void);

/** @brief Receive datagrams: demux, anchor and gap policy, parity, decode, and
 *  the feed into the ring. @param arg Unused. */
void rx_task(void *arg);

/** @brief The playback timeline: hold the first sample for its instant, then
 *  pace the ring against the DAC, measuring and correcting position.
 *  @param arg Unused. */
void play_task(void *arg);

/** @brief One 5 s window of rate control. Reads the phase, hands it to the
 *  shared servo loop, and applies the result as either a trim or a retune. */
void servo_tick(void);

/** @brief One 5 s window of reporting: the heap windows, the arrival gauges,
 *  the RX counters, and the periodic HEALTH, TRIM and MEM lines. */
void telemetry_tick(void);

/**
 * @brief Heap allocation-failure hook. Records; does not log.
 *
 * @param size          Bytes requested.
 * @param caps          Capability mask requested.
 * @param function_name Caller, unused here.
 *
 * @note IRAM_ATTR is on the definition only — repeating the section attribute
 * on this declaration makes GCC complain that .iram1.1 conflicts with
 * .iram1.0.
 */
void on_alloc_failed(size_t size, uint32_t caps, const char *function_name);

/** @} */
