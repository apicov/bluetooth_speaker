/*
 * The satellite's shared state, and who owns each piece of it.
 *
 * main.c was one 2437-line file until 2026-08-12. Splitting it into modules
 * made a question explicit that had always been there: sixty-odd values cross
 * between the receive task, the playback task, the servo and the reporting, and
 * nothing said which task was allowed to write which. The comments below are
 * the originals, and several of them answer exactly that question -- the
 * 32-bit-not-64 rule on marker_sample and samples_in is the clearest.
 *
 * OWNERSHIP, stated once so it can be checked:
 *
 *   rx_task     writes the stream anchor and everything describing arrival:
 *               stream_start_local, anchor_at, stream_rate, samples_in,
 *               marker_sample, restart_pos, the phase queue head, the tsf_*
 *               pair, resync_request, anchor_provisional, and the n_gap*,
 *               n_ring_full, n_anchor_* counters.
 *   play_task   writes what playback has reached: the phase queue tail,
 *               phase_err_us, phase_valid, phase_stepped, phase_hist, the
 *               splice_report_* set, n_underruns, n_splices, hw_play, and the
 *               refill pair. It owns stream_offset once a stream is running.
 *   drift_task  writes the servo's own state, the retune_* set and the windowed
 *               heap figures, and READS everything else in order to report it.
 *   wifi_event  writes n_wifi_drops, wifi_down_at, rejoined_at.
 *
 * `volatile` here means "another task writes this", not "this is atomic". A
 * 64-bit load is two instructions on this CPU, so a reader can catch half of a
 * write -- which is why marker_sample and samples_in are deliberately 32-bit,
 * as their own comment explains. The 64-bit values that cross tasks were left
 * unexamined for a long time; docs/satellite-audit.md F3 is the audit of them,
 * and this is where the conclusions live.
 *
 * WHAT TEARING ACTUALLY COSTS, per value, because it is not uniform. A torn
 * read only differs from both the old and the new value when the write changes
 * the HIGH word. For a monotonic microsecond clock that happens every 2^32 us
 * -- once per 71.6 minutes -- and for anything that is set to 0, or crosses
 * zero, every time.
 *
 *   tsf.offset_us / tsf.at   FIXED, and it needed fixing twice over. An offset
 *                            hovers near zero and changes sign, so the high
 *                            word flips constantly; and the two were read
 *                            separately when they only mean anything together.
 *                            Both are now under one sequence lock. See
 *                            tsf_reading_t.
 *
 *   stream_start_local       NOT fixed, and a seqlock is the WRONG fix: it has
 *                            two writers. rx_task writes the anchor instant,
 *                            play_task writes 0 when it parks, and a seqlock
 *                            with two writers is silently broken. The exposure
 *                            is real but small -- play's write of 0 can be seen
 *                            by the servo as a large nonzero value, costing one
 *                            5 s window in which it believes a stream is
 *                            running; and rx's anchor write tears harmfully
 *                            only across a 71.6-minute boundary. The honest fix
 *                            is to give it one owner, which is a design change
 *                            and wants a bench run behind it.
 *
 *   anchor_at                NOT fixed. One writer, but the only reader uses it
 *                            as `now - anchor_at < DEPTH_NET_HOLD_US`, so a
 *                            torn read costs one window of the depth net being
 *                            wrongly held or released. Cheap to fix with a
 *                            seqlock if it ever earns one.
 *
 *   wifi_down_at,            NOT fixed, deliberately. These feed log lines and
 *   rejoined_at,             nothing else. A torn read prints one wrong number
 *   retune_outage_us,        and no servo acts on it. Fixing them would add
 *   retune_done_at           machinery to the diagnostics for no behaviour.
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

extern const char *TAG;

/* The SSID and password are the hub's, and are declared once in
 * components/dancefloor_sync/Kconfig -- they are one link seen from both ends,
 * and were previously a #define here and another in the hub with nothing
 * checking they agreed. */
#define AP_SSID    CONFIG_DANCEFLOOR_AP_SSID
#define AP_PASS    CONFIG_DANCEFLOOR_AP_PASS
#define MASTER_IP  "192.168.4.1"        /* esp_netif SoftAP default */

#define PROBE_PERIOD_MS 250             /* see docs/clock-sync.md §3 */

/*
 * Must hold the master's lead time plus headroom for jitter -- specifically
 * LEAD_US + RESYNC_US, which is how far the hub's timeline can legitimately be
 * from real time when a chunk is stamped.
 *
 * 80 kB, up from 64: 464 ms at 44.1 kHz stereo against 371 ms.
 *
 * The 16 kB buys the hub's RESYNC_US the headroom its own comment says it
 * wants and cannot have. Delivery from the Bluetooth bridge is bursty by
 * construction -- A2DP packets arrive ~23/s carrying ~43 ms each -- and the
 * measured swing of the hub's timeline against real time reaches +-132 ms
 * against a 120 ms threshold. So it trips about seven times a minute on
 * entirely normal delivery. Raising the threshold past the swing was blocked by
 * this constant: 200 + 120 = 320 already sat close enough to 371 that 150 was
 * not affordable.
 *
 * It is affordable here rather than on the hub because the hub is not the unit
 * that has to hold it. This one is, and it is the classic ESP32 -- 117 kB free
 * with a largest block of 106 kB, so 80 kB fits and 107 kB (which is what a
 * 500 ms lead would need) would not allocate at all. That asymmetry is worth
 * knowing before anyone proposes a longer lead on the strength of the S3 hub's
 * PSRAM: the buffer a lead has to fit in is on the other board.
 *
 * Set through DANCEFLOOR_RING_KB now rather than fixed here, because with a
 * second target the "117 kB free" above stops being the only answer -- an S3
 * satellite has 512 kB of internal SRAM. The default is unchanged at 80 on both
 * targets; the Kconfig help says why raising it on the S3 is a measurement
 * nobody has taken rather than a free win.
 */
#define RING_BYTES  (CONFIG_DANCEFLOOR_RING_KB * 1024)

extern int sock;
extern sync_est_t est;
extern StreamBufferHandle_t ring;
extern i2s_chan_handle_t i2s_tx;

/* Local-clock instant the next byte entering the ring should reach the DAC.
 * Zero means playback has not started. */
extern volatile int64_t stream_start_local;
/*
 * When the current stream was anchored. Written by the receive task, read by
 * the drift servo, which holds its depth safety net off for a while after --
 * see DEPTH_NET_HOLD_US.
 */
extern volatile int64_t anchor_at;
extern volatile uint32_t stream_rate;
extern uint32_t tx_rate;  /* what the output clock is actually set to */
/*
 * Local -> master conversion used by playback. Seeded at anchoring and then
 * slewed towards the live estimate -- see track_offset(). Owned by the playback
 * task once a stream is running; the receive task only writes it while playback
 * is parked waiting for one.
 */
extern int64_t stream_offset;
extern int64_t offset_slew_last;  /* when the slew last moved it */
/*
 * 32-bit, not 64: these are read by the playback task while the receive task
 * writes them, and a 64-bit load is two instructions on this CPU -- a torn read
 * yields a garbage position and a wild marker. int32 holds 13 hours of frames
 * at 44.1 kHz, which is longer than any party.
 */
extern volatile int32_t marker_sample;  /* ring position of a tagged packet */
extern volatile int32_t samples_in;  /* frames written into the ring */

/*
 * Phase tracking.
 *
 * Servoing on buffer depth matches the playback RATE to the arrival rate, but
 * says nothing about POSITION. Depth also moves with network jitter, so the
 * servo nudges the rate in response to noise -- and two units seeing different
 * jitter end up with rates differing by ~0.03% at any instant, which is
 * several ms of relative movement between markers. Observed as 10-25 ms of
 * wander between hub and satellite, with each unit's own buffer perfectly
 * stable.
 *
 * Every packet says exactly when its first sample should play. Recording that
 * against the ring position it lands at gives a direct phase measurement when
 * playback reaches it: where we are, versus where the timeline says we should
 * be. Correcting that holds position rather than merely matching rates.
 *
 * Single producer (receive task), single consumer (playback), 32-bit indices,
 * so no locking is needed.
 */
#define PHASE_Q_LEN 32
typedef struct {
    int32_t pos;        /* ring frame position where this packet's audio starts */
    int64_t play_at;    /* master-clock instant that sample should be heard */
} phase_pt_t;

extern phase_pt_t phase_q[PHASE_Q_LEN];
extern volatile uint32_t phase_head, phase_tail;
extern volatile int32_t phase_err_us;  /* + = playing late */
extern volatile bool phase_valid;
/*
 * The last few raw readings, for the splice alone. Play task only -- pushed in
 * the crossing loop, read and reset in the splice, reset at the top of the
 * outer loop -- so no volatile and no lock, unlike everything around it.
 *
 * The servo has smoothed its input since it was measured triggering on noise;
 * the splice never did, and it is the larger correction of the two. See
 * sync_phase_hist_t. Nothing acts on this yet: it is measured against the raw
 * value first, on the same boundaries, and only then does the splice move.
 */
extern sync_phase_hist_t phase_hist;
extern volatile int32_t restart_pos;  /* ring position of a track boundary */
/*
 * Set after a splice. The phase genuinely steps at that instant, so the running
 * average from before it describes a situation that no longer exists -- seen as
 * "phase -2153 us (smoothed +26992 us)", with the servo acting on the stale
 * figure. Splices are rare, so re-seeding here costs nothing; re-seeding on
 * every correction, as an earlier version did, destroys the smoothing entirely.
 */
extern volatile bool phase_stepped;

/* Never splice more than this in one go. A larger error means something is
 * wrong that a splice will not fix, and a 150 ms jump is very audible even at a
 * track change. */
#define MAX_SPLICE_MS 150

/*
 * Beyond this, the phase reading is not describing our playback at all.
 *
 * Drift is ~0.8 ms per minute and delivery jitter is a few ms, so a whole
 * second cannot be either. What it does mean is that the stamps arriving now
 * were issued against a different clock origin than the one playback anchored
 * to. Servoing on that is meaningless; re-anchoring is the only thing that
 * helps.
 */
#define PHASE_INSANE_US 1000000

/*
 * What an anchorable packet looks like. See the refusals in handle_audio().
 *
 * The hub stamps every chunk LEAD_US = 200 ms ahead, so a healthy packet
 * arrives with most of that still in front of it -- a good anchor was measured
 * at "in 154 ms". This is half of that lead, and the number is not arbitrary
 * caution: the scheduled wait below is the ONLY thing that prefills the ring,
 * so whatever lead survives to here is the prefill, and half the design depth
 * is the least worth starting on.
 *
 * It was 20 ms, chosen as "the floor below which prefill is not worth having",
 * and that was the wrong test. A run cleared it by four milliseconds: 144
 * packets refused, then one accepted at +24 ms, anchoring with `buffer 0 ms`.
 * The ring then overfilled to 400 ms behind it (ring-full 59), phase reached
 * +118 ms, and the servo spent over 200 seconds walking it back. The guard
 * fired 144 times and still let through the one that mattered.
 *
 * There is a real tension in the value, and it is not resolved so much as
 * chosen. The hub's RESYNC_US allows its timeline to wander 150 ms from real
 * time before slewing, so a perfectly healthy packet arriving during a trough
 * can show as little as ~50 ms of lead -- below this floor. Such a packet WILL
 * be refused. That is deliberate: anchoring mid-trough is how a stream starts
 * with a lead it cannot keep, and troughs recover within a second or two, so
 * refusing costs a second and buys an anchor taken on the recovery instead.
 * ANCHOR_GIVE_UP_US bounds the cost if the trough is not a trough.
 *
 * Tied by convention to the hub's LEAD_US, which this unit cannot see. If the
 * lead ever changes, this is the second place to look.
 *
 * A second between anchors, against a hub that would supply forty-nine packets
 * in that time: if none of them anchors, the link is not in a state a re-anchor
 * can fix.
 *
 * Five seconds before giving up and anchoring anyway. Long enough that no
 * plausible burst of lateness reaches it, short enough that a genuine
 * lead/path mismatch does not leave a speaker silent for a whole track.
 */
#define ANCHOR_MIN_LEAD_US     100000
#define ANCHOR_MIN_INTERVAL_US 1000000
#define ANCHOR_GIVE_UP_US      5000000

/*
 * A gap beyond this is an outage, not jitter, and is re-anchored rather than
 * filled with silence. See the reasoning at the fill in handle_audio().
 *
 * 150 ms is about seven packets. Normal loss on a healthy link is one to three
 * -- 20 to 60 ms -- so this sits well clear of anything that should be filled,
 * while staying below the 200 ms RING_TARGET_MS that a fill this size would
 * otherwise push the ring past.
 */
#define GAP_RESYNC_MS 150

/* How long after an anchor the drift servo ignores buffer depth and servos on
 * phase alone. See the safety net in drift_task() for what it was doing to a
 * ring that had simply not finished filling yet. */
#define DEPTH_NET_HOLD_US      20000000

/*
 * A track-boundary correction waiting to be reported to the hub, so it can
 * print how far apart the units had drifted -- see splice_msg_t. Written by
 * playback, sent by the probe task, because a sendto() in the audio path is
 * exactly the kind of thing that costs a buffer.
 */
extern volatile int32_t splice_report_us;
extern volatile int32_t splice_report_phase;
/* SHADOW: the correction the median would have produced instead. Acted on by
 * nothing here; the hub prints it beside the real one so both units' figures
 * are compared at the same boundary. See splice_msg_t.applied_alt_us.
 *
 * Three boundaries have been captured so far and the median agreed with the raw
 * reading at every one of them -- sat -3/-3, -1/-1, +4/+4 ms, and the hub the
 * same. That is the shadow doing its job and finding nothing: at a track
 * boundary the phase is evidently stable enough that smoothing changes no
 * decision. Three is too few to retire it on, so it stays; if it is still
 * three-for-three after a long evening, the median machinery can come out of
 * the audio path and phase_hist with it. */
extern volatile int32_t splice_report_alt;
extern volatile bool    splice_report_pending;

/*
 * The TSF-derived clock offset, and when it was last updated.
 *
 * Written by the receive task, read by playback. Zero `at` means no usable TSF
 * message has arrived, in which case everything falls back to the probe
 * estimator exactly as before -- both units may have TSF unavailable, the
 * satellite may not have associated yet, or the hub may be an older build that
 * does not send MSG_TSF at all.
 */
/*
 * Read and written as a PAIR, under a sequence lock, and not directly.
 *
 * Two things were wrong with the plain pair of volatiles these replace, and the
 * second is the one that matters.
 *
 * TEARING: both are 64-bit and a 64-bit load is two instructions on this CPU,
 * so a reader can catch one half of a write. An offset hovers near zero and
 * changes sign, which flips the high word between 0 and 0xFFFFFFFF -- exactly
 * the case where the two halves do not belong together and the result is not
 * close to either value.
 *
 * PAIRING, which no amount of per-field atomicity would fix: clock_offset()
 * reads `at`, decides the reading is fresh, and then reads `us`. A publish
 * landing between those two lines gives it a fresh timestamp and the offset
 * from before it -- and that offset is what the stream anchors on, where an
 * error is baked in for the life of the stream. It is the same shape as the
 * bug docs/clock-sync.md section 9 records: invisible, because every log line
 * downstream is derived from the same wrong number.
 *
 * A sequence lock is the right tool because there is exactly ONE writer
 * (rx_task, in the MSG_TSF arm) and several readers. The writer never blocks;
 * a reader retries only if a publish overlapped it. Do not add a second writer
 * without replacing this -- a seqlock with two writers is silently broken.
 */
typedef struct {
    volatile uint32_t seq;      /* odd while a write is in progress */
    volatile int64_t  offset_us;
    volatile int64_t  at;       /* local time the offset was derived; 0 = never */
} tsf_reading_t;

extern tsf_reading_t tsf;

/* Writer side. rx_task only. */
static inline void tsf_publish(int64_t offset_us, int64_t at)
{
    tsf.seq++;                              /* now odd: a write is in progress */
    __atomic_thread_fence(__ATOMIC_RELEASE);
    tsf.offset_us = offset_us;
    tsf.at = at;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    tsf.seq++;                              /* even again: consistent */
}

/* Reader side. Returns a pair that was published together. */
static inline void tsf_read(int64_t *offset_us, int64_t *at)
{
    uint32_t s0;
    do {
        s0 = tsf.seq;
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        *offset_us = tsf.offset_us;
        *at = tsf.at;
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
    } while ((s0 & 1u) || s0 != tsf.seq);
}

/* tsf_fresh() is below, after TSF_MAX_AGE_US, which it needs. */
extern volatile uint32_t n_tsf_used;  /* anchors that used TSF */
extern volatile uint32_t n_tsf_fallback;  /* anchors that fell back */

/*
 * How stale a TSF offset may be and still be preferred over the estimator.
 * Messages arrive with every probe reply, 4/s, so a second means several have
 * been missed and the link is not healthy enough to trust the last one.
 */
#define TSF_MAX_AGE_US 1000000

/*
 * True if a TSF reading exists and is fresh enough to prefer over the
 * estimator, and if so what it is.
 *
 * The freshness test and the value it vouches for come out of ONE tsf_read(),
 * which is the whole point of the pair -- see tsf_reading_t. Every caller that
 * used to read tsf_offset_at, decide, and then read tsf_offset_us goes through
 * here instead.
 */
static inline bool tsf_fresh(int64_t now, int64_t *offset_us)
{
    int64_t us, at;
    tsf_read(&us, &at);
    if (at && now - at < TSF_MAX_AGE_US) {
        if (offset_us) *offset_us = us;
        return true;
    }
    return false;
}

/*
 * Cumulative totals for a long run, never reset. The 5 s lines answer "what is
 * happening now"; only a total answers "has this been happening slowly for an
 * hour", and nothing here had one. See the hub's copy.
 */
extern volatile uint32_t n_underruns;  /* playback ran dry */
extern volatile uint32_t n_reanchors;  /* streams anchored, first included */
extern volatile uint32_t n_splices;  /* track-boundary corrections applied */
extern volatile uint32_t n_retunes;
extern volatile uint32_t n_retunes_bad;
extern volatile uint32_t n_gaps;  /* lost-packet gaps filled with silence */
extern volatile uint32_t n_fec_recovered;  /* lost packets decoded from FEC redundancy */
/*
 * Frames a recovery was SHORT by, padded with silence -- the honest half of
 * n_fec_recovered.
 *
 * These two together are the instrument that was missing. n_fec_recovered
 * counts packets a redundant copy covered; it said nothing about how MUCH of
 * each one the copy actually carried, and the copy is truncated to ~3/4 by the
 * hub's MTU guard. So the log read `gaps 1 (20 ms silence) | fec 1` -- a gap,
 * and a recovery, and the reader is left to assume they cancel. They did not:
 * ~6 ms of that 20 was still silence, every time, once every ~2.7 s.
 *
 * Now the shortfall is its own number, and it reads 0 only because redundancy
 * is off by default. Turn DANCEFLOOR_AUDIO_FEC_DEPTH up and it will be roughly a
 * quarter of every recovery; n_fec_truncated at the hub end is the same fact
 * seen from the sender. Between them they are what any future attempt at
 * redundancy has to drive to zero before it can claim to have recovered
 * anything.
 */
extern volatile uint32_t n_fec_short_frames;
/*
 * Decodes of a redundant copy that failed part-way.
 *
 * Distinct from a live-stream decode error because the response differs: this
 * path deliberately does NOT reinitialise the decoder. See the note in
 * fill_recovered_then_silence().
 */
extern volatile uint32_t n_fec_decode_err;
/*
 * Three faults that used to happen silently. Each was a `continue`, a `break` or
 * a bare `return false` with nothing recorded, so a run in which any of them
 * fired looked exactly like a clean one.
 *
 * n_seq_dropped -- a packet older than expected. Should be 0: the hub sends
 *   each once and a group frame is not retried. Non-zero means either something
 *   is duplicating, or packets are arriving out of order -- and a reorder means
 *   the "gap" before it was never a loss, so the silence filled for it was
 *   inserted against a packet that did arrive.
 * n_decode_err -- a live-stream SBC frame that would not decode. The rest of
 *   that packet is dropped, so the timeline is short by whatever it held, and
 *   no other counter sees it.
 * n_recv_err -- recvfrom() returning an error rather than a datagram. The old
 *   code spun on this at priority 7 without counting it.
 */
/*
 * Re-anchors forced because a gap fill did not fit the ring.
 *
 * Distinct from n_gap_resyncs, which is a gap longer than GAP_RESYNC_MS: that
 * one says the air was bad, this one says the ring could not absorb a burst
 * arriving faster than it plays. Both end in a reset and a re-anchor, and
 * telling them apart is what says whether to spend the next effort on loss or
 * on buffering.
 */
extern volatile uint32_t n_gap_short_resyncs;
extern volatile uint32_t n_seq_dropped;
extern volatile uint32_t n_decode_err;
extern volatile uint32_t n_recv_err;
extern volatile uint32_t n_wifi_drops;  /* disconnects from the hub's AP */
/*
 * The receive path's own instruments, counted here rather than logged there.
 *
 * These three used to be an ESP_LOGW each, per event, from inside
 * handle_audio() -- which runs in rx_task, which is the only thing draining a
 * UDP mailbox six datagrams deep against ~136 datagrams a second. The console
 * is a 115200-baud UART, so a ~60-character line is ~5 ms of blocking write.
 *
 * That closes a loop: packet loss makes lines, lines block the receive task,
 * a blocked receive task overflows the mailbox, and the overflow is more loss.
 * A run of it printed several hundred lines across six seconds and the loss
 * outlived the disturbance that started it by about that much.
 *
 * So the audio path increments and drift_task talks, within 5 s, from a task
 * that can afford to wait on a UART. Cumulative, like every other counter here;
 * the narration below prints the window by subtracting what it said last time.
 */
extern volatile uint32_t n_gap_frames;  /* silence inserted for lost packets */
extern volatile uint32_t n_gap_short;  /* gap fills the ring could not take */
extern volatile uint32_t n_gap_short_frames;
extern volatile uint32_t n_ring_full;  /* decoded blocks dropped, ring full */
extern volatile uint32_t n_gap_resyncs;  /* gaps too large to fill, re-anchored */
extern volatile uint32_t n_anchor_upgrades;  /* provisional anchors replaced */
/*
 * Set by the receive task when a gap is too large to fill, cleared by the
 * playback task when it parks. See GAP_RESYNC_MS.
 */
extern volatile bool resync_request;
/*
 * Set when ANCHOR_GIVE_UP_US forced an anchor onto a packet that was already
 * late. Playback is running but its position is known to be wrong, so the
 * receive path keeps watching for a packet it could have anchored on properly.
 */
extern volatile bool anchor_provisional;
extern volatile uint32_t n_anchor_late;  /* anchors refused, play_at already past */
extern volatile uint32_t n_anchor_soon;  /* anchors refused, one just happened */
/*
 * Phase points dropped because phase_q was full. The only loss path in this
 * file that had no counter, which is the one thing the rest of this system is
 * built not to allow: every real fault here was invisible until something
 * counted it. A full queue means playback is not consuming points as fast as
 * the receive path records them, and the servo silently stops getting fresh
 * input while every log line still reads normally.
 */
extern volatile uint32_t n_phase_drop;
/*
 * Ring reads that came back short of a full chunk, and the frames of silence
 * padded in to cover them.
 *
 * Suspected, not established, which is why this is a counter and not a fix. The
 * pad is played but was never in the ring, while samples_played advances by a
 * whole chunk regardless -- so if it happens, every later phase point is
 * displaced by the pad and the servo's only input carries a permanent bias.
 * That is the exact shape of the "silence inserted for a lost packet was not
 * counted in samples_in" bug, which put this unit ~20 ms out per loss and
 * stayed hidden because the marker was derived from the same count.
 *
 * The ring's trigger level is one chunk, so a short read means the 500 ms
 * timeout expired on a partly-filled ring -- a near-underrun. If these stay
 * zero over a long session the concern is latent and the fix can ride along
 * with anything; if they do not, n_short_frames IS the bias, in frames.
 */
extern volatile uint32_t n_short_reads;
extern volatile uint32_t n_short_frames;
/*
 * TSF samples whose read pair took longer than TSF_SPAN_MAX_US -- i.e. samples
 * something preempted between the two counter reads, so the offset they carry
 * is off by whatever landed in the gap.
 *
 * COUNTED, NOT ENFORCED. TSF is the anchor clock source now, and a threshold
 * chosen blind could silently demote it to the probe estimator, which is worse
 * -- that is a regression wearing no log line. This says what the reject rate
 * WOULD be, so the threshold can be chosen from the distribution instead.
 *
 * ENFORCING IT AT 100 us WOULD NOT BE FREE, which is the opposite of what the
 * earlier logs suggested. Those all read `wide-span 0`; the run of 2026-08-12
 * 16:39 read 1 in the first five seconds and 21 by 65 s, against roughly 240
 * samples a minute -- call it 9% -- with the reported `span max` reaching 210
 * and 214 us in ordinary windows.
 *
 * So the counter did its job: a threshold picked from the old logs would have
 * looked free and then quietly demoted TSF to the probe estimator for one
 * sample in eleven, which is exactly the regression-wearing-no-log-line this
 * was written to prevent. What changed between the runs is not known -- more
 * traffic, a different board, the split moving where the read pair sits
 * relative to other work in rx_task -- and that is worth establishing before
 * any threshold is chosen, because it decides whether 100 us is too tight or
 * the spans are a symptom.
 */
#define TSF_SPAN_MAX_US 100
extern volatile uint32_t n_tsf_wide;
/*
 * When this unit went off the air, and when it came back.
 *
 * The suspicion being measured: nothing invalidates the probe estimator's
 * window on a disconnect. sync_est_offset() selects the lowest-RTT sample in a
 * 10-sample window and neither it nor sync_est_settled() decays with time, so
 * after an outage of any length the unit may anchor on an offset measured
 * before the drop -- and an offset error at the anchor is baked in for the life
 * of the stream, since play_at is consulted once. The first anchor after a
 * rejoin therefore says which clock it used and how stale the estimator's
 * newest sample was. If it reads "TSF" the concern does not arise, because TSF
 * is re-derived from a fresh beacon; if it reads "probe" with an age spanning
 * the outage, it does.
 */
extern volatile int64_t wifi_down_at;
extern volatile int64_t rejoined_at;  /* 0 = the next anchor is not the first */
extern volatile int64_t est_newest_at;  /* when the newest probe landed */
extern volatile uint32_t n_frames_rx;  /* analysis frames taken from the hub */
extern volatile uint32_t n_frames_bad;  /* ... and rejected, wrong size */
extern volatile uint32_t n_ml_rx;  /* analyser results taken from the hub */
extern volatile uint32_t n_ml_bad;  /* ... and rejected, wrong size */
extern volatile uint32_t hw_play;  /* stack headroom, sampled in-task */
extern volatile uint32_t hw_drift;

/*
 * Heap, dated, and allocation failures made audible. The hub's copy carries the
 * reasoning; this is the same instrument on the other unit.
 *
 * It is here despite no pressure ever having been observed on a satellite --
 * 52 kB free analysing locally, 118 kB being given its frames, against a hub
 * that reached 2040 bytes. Which is the point: the value of a windowed minimum
 * is that it says nothing, every minute, until the minute it does. A counter
 * that only exists on the unit already known to be sick cannot tell you the
 * other one just got sick too, and these two units do not have the same job or
 * the same failure.
 */
extern volatile uint32_t heap_min_window;
/*
 * The pool ordinary allocations actually draw from.
 *
 * MALLOC_CAP_INTERNAL ALONE IS NOT THAT POOL, and getting this wrong hid a dead
 * satellite for an evening. On the classic ESP32 the IRAM heap is registered as
 * INTERNAL|EXEC|32BIT (heap/port/esp32/memory_layout.c) -- internal, but neither
 * 8-bit accessible nor DEFAULT, so nothing that needs byte access can touch it.
 * A task stack cannot. malloc() cannot. This unit reported
 *
 *   MEM: internal 31760 free (min 31424, ..., largest 30720) | total 396 (largest 208)
 *
 * while three of its four tasks were failing to start on 4096-byte requests. The
 * 31 kB was real and entirely useless; 396 bytes was the truth. Adding 8BIT is
 * what makes the figure describe the memory a stack can be cut from -- and it is
 * exactly the mask of the request that was failing, caps 0x804.
 *
 * The hub's copy carries the same constant for the same reason, though it is the
 * S3 where the two masks nearly agree: no IRAM-only region is registered there.
 */
#define CAP_USABLE_INTERNAL (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)

extern volatile uint32_t heap_int_window;
extern volatile uint32_t n_alloc_fail;
extern volatile uint32_t alloc_fail_size;  /* the largest request that failed */
extern volatile uint32_t alloc_fail_caps;

/*
 * Tasks that did not start, which used to be unsayable.
 *
 * Every xTaskCreate here passed NULL for the handle and threw the return value
 * away, so a unit that could not start its play task ran on without one and
 * looked from the outside exactly like a satellite that had gone quiet: still
 * associated, still holding a lease, no audio, no LEDs, and -- because the
 * missing task was often drift_task -- no HEALTH line to say otherwise. That is
 * the same silent-failure class the allocation hook above exists to end, left in
 * the one place it could take the whole unit down.
 *
 * Reported on the join line rather than only where it happens: see task_start().
 */
extern volatile uint32_t n_task_fail;
extern char s_task_fail_names[64];


/* Target buffer depth: the hub stamps audio ~200 ms ahead, so in steady state
 * that much should be sitting here waiting. */
#define RING_TARGET_MS 200

/*
 * How much unpaced audio a playback START puts in before the DAC begins pacing.
 *
 * i2s_channel_write() does not block until the descriptors are full, so the
 * first writes after the channel has been idle return at memory speed:
 * samples_played advances by the whole DMA depth against a wrote_at that has
 * barely moved, and every phase reading dated inside that window is measured
 * against a reference the DAC is not pacing.
 *
 * THE RETUNE HALF OF THIS QUESTION IS ANSWERED AND HAS BEEN REMOVED. It asked
 * whether i2s_channel_disable() discards the DMA descriptors or drains them,
 * because the two answers predict opposite signs for the phase step a retune
 * causes. Across the collected bench logs, 25 of 26 post-retune measurements
 * read `0 frames` and the twenty-sixth read 256 -- so it DRAINS, there is
 * nothing to refill, and the first write after a retune blocks immediately.
 * retune_output()'s closing comment already relies on that result. What used to
 * arm this after a retune, and the `s_refill_why` that told the two cases
 * apart, are gone with the question.
 *
 * The START case is not answered and is kept. Two measurements, both `1536
 * frames (34 ms)`, against packets arriving ~20 ms apart -- so roughly the
 * first one or two crossings of a fresh stream are dated against an unpaced
 * reference. Whether to withhold them is the open question; this is the
 * instrument that sizes it, and two samples is not yet enough to act on.
 *
 * Play task only -- it arms and clears these, so no volatile.
 */
#define REFILL_FAST_US 1000     /* below this, the write did not block */
extern bool    s_refill_active;
extern int32_t s_refill_frames;

/* ----------------------------------------------------------------- drift */

/*
 * Widest trim the servo may ever ask for. Real drift is ~14 ppm and the buffer
 * safety net asks for 20 Hz, so 100 Hz is already absurd -- anything beyond it
 * is a broken measurement, not a correction.
 */
#define RATE_TRIM_MAX_HZ 100

/*
 * What a retune costs, which nobody has measured.
 *
 * Two effects pull opposite ways and neither is visible to the servo. The
 * channel is DOWN across disable/reconfig/enable, and real time passes with no
 * audio playing, so playback returns that far behind the timeline. Against
 * that, the disable discards the DMA buffer -- audio already counted in
 * samples_played and already fed to the visualiser -- which skips content and
 * pushes the other way.
 *
 * Software can see the first and, by construction, never the second: those
 * frames were counted as played, so every reading derived from samples_played
 * agrees that they were. Only the marker GPIO, which fires when a sample
 * physically reaches the output, can close that gap.
 *
 * These record the first effect directly and the NET at the writer, which is
 * what the servo has to correct. Four retunes scraped from a session put the
 * net somewhere between +5 and +22 ms; that is a range, not a number, because
 * phase wanders by several ms on its own between the 5 s log ticks.
 */
extern volatile int32_t retune_phase_before;
extern volatile bool    retune_watch;  /* playback reports the next reading */
extern volatile int64_t retune_outage_us;

/*
 * When the retune finished, and how many crossings have been narrated since.
 *
 * MEASUREMENT ONLY -- the servo still withholds exactly one reading, so this
 * build behaves identically to the last and merely says more.
 *
 * The bench numbers behind the one-shot withholding were taken here: 19
 * same-rate retunes, net +4.4 ms against a 3.6 ms outage, every one positive,
 * the crossing landing 1-22 ms after the retune and inside the refill every
 * time. Crossings arrive one per packet, ~20 ms apart, so one withheld reading
 * covers perhaps the first of a disturbance that reaches 22 ms -- and whatever
 * is left goes to the servo as position error, so each retune injects what the
 * next one corrects. These lines say how far the tail actually reaches, which
 * is what sizes a settle window instead of guessing one.
 *
 * STILL OPEN, AND THE COLLECTED LOGS SAY IT REACHES FURTHER THAN THAT. Across
 * 78 narrated tail crossings the crossing lands 17 to 74 ms after the retune,
 * median 43 ms -- so the three narrated here do not reach the end of it, and
 * the single withheld RETUNE COST reading (which crossed 2.8 to 14 ms after)
 * covers only its beginning. The withheld reading is doing less than it looks.
 *
 * What the logs cannot settle is how much of that is the retune. `net` is
 * measured from the phase before the retune, so it carries whatever drift and
 * delivery jitter happened in the intervening 43 ms as well, and the largest
 * |net| seen (11.4 ms) is not attributable to the retune on this evidence.
 * Sizing a settle window needs the disturbance separated from the background,
 * which needs the bench retune (CONFIG_DANCEFLOOR_RETUNE_BENCH_S) on one unit
 * against an undisturbed reference -- that is what that option is for.
 */
extern volatile int64_t retune_done_at;
extern volatile uint8_t retune_tail_left;

/*
 * Held across a retune, and the playback task parks on it.
 *
 * i2s_channel_write() returns IMMEDIATELY once the channel is disabled -- it
 * does not block, and dac_write() did not look at the return value -- so for
 * the whole outage the play task ran flat out: pulling chunks from the ring,
 * counting them in samples_played, feeding them to the visualiser, and throwing
 * them at a channel that was not running. Milliseconds of outage cost tens of
 * milliseconds of buffer.
 *
 * Measured on hardware: a 7.7 ms outage produced a +42 ms phase step, 5432
 * bytes overflowed the visualiser's buffer, and it re-aligned nine times in one
 * window. The short outages, where the task had less time to spin, cost +4 ms.
 *
 * The hub has had this since "a measured 54 ms correction cost 177 ms of
 * buffer". It was never ported here, and every satellite retune has been paying
 * for it since.
 */
extern volatile bool retuning;


/* ------------------------------------------------------------- module entry */
/*
 * One prototype per thing another module calls. Anything absent from this list
 * is private to its file and should stay `static` there.
 */

/* net.c -- joining the AP, and the socket everything rides on */
void wifi_start_sta(void);
void socket_start(void);

/* out.c -- the I2S channel, the write path, and retuning its clock */
void i2s_start(uint32_t rate);
/* How many times the DMA has run out of audio to send -- see on_tx_starved().
 * A running total, not a rate; a retune contributes by construction. */
uint32_t dma_starve_count(void);
void write_audio(const uint8_t *pcm, size_t bytes);
void retune_output(uint32_t hz);
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
int64_t vis_master_to_local(int64_t master_us);
#endif

/* clock.c -- which offset to believe, and keeping it current */
void probe_task(void *arg);
bool clock_offset(int64_t *out, bool *used_tsf);
void track_offset(void);

/* rx.c -- demux, anchor and gap policy, decode, ring feed */
void rx_task(void *arg);

/* play.c -- the playback timeline */
void play_task(void *arg);

/* servo.c -- one 5 s window of rate control */
void servo_tick(void);

/* telemetry.c -- one 5 s window of reporting, and the allocator hook */
void telemetry_tick(void);
/* IRAM_ATTR is on the definition only: repeating the section attribute here
 * makes GCC complain that .iram1.1 conflicts with .iram1.0. */
void on_alloc_failed(size_t size, uint32_t caps, const char *function_name);
