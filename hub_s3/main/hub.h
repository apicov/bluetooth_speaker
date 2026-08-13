/*
 * The hub's shared state, and who owns each piece of it.
 *
 * streamer.c was one 2686-line file until 2026-08-12, when docs/hub-audit.md
 * asked whether a reviewer could follow it. Splitting it into modules made a
 * question explicit that had always been there: ninety-odd values cross between
 * the timeline publisher, the playback task, the servo, the probe server and the
 * reporting, and nothing said which task was allowed to write which. The
 * comments below are the ORIGINALS, moved here unedited -- several of them
 * answer exactly that question, and the 32-bit-not-64 rule on s_marker_sample
 * and s_samples_in is the clearest.
 *
 * OWNERSHIP, stated once so it can be checked:
 *
 *   sbc_in's rx_task   calls streamer_feed / _begin_packet / _mark_here /
 *                      _send_sbc / _send_meta / _request_restart, so it writes
 *                      s_samples_in, s_pending_pos, s_marker_sample,
 *                      s_restart_pending, s_restart_pos, the phase queue HEAD,
 *                      the s_vis_anchor pair, local_start + local_epoch, the
 *                      slew trio and
 *                      n_restarts / n_phase_drop / n_wifi_oversize.
 *   local_play_task    writes what playback has reached: the phase queue TAIL,
 *                      s_phase_err_us, s_phase_valid, s_phase_stepped,
 *                      s_phase_hist, the s_hub_splice_* set, the refill trio,
 *                      s_marker_at, n_underruns, n_splices, n_short_*, hw_play.
 *                      and s_playing. It no longer writes local_start.
 *   ring_monitor_task  writes the servo's own state, the retune_* set and the
 *                      windowed heap figures, and READS everything else in order
 *                      to report it. retune_dac() runs on this task.
 *   probe_task         writes only the client list (via client_seen).
 *   the WiFi event task writes n_sta_left and the client list.
 *   monitor_task       writes s_sync_err_us / s_sync_at (marker builds only).
 *
 * `volatile` here means "another task writes this", not "this is atomic". A
 * 64-bit load is two instructions on this CPU, so a reader can catch half of a
 * write -- which is why s_marker_sample and s_samples_in are deliberately
 * 32-bit, as their own comment explains.
 *
 * WHAT TEARING ACTUALLY COSTS. A torn read only differs from both the old and
 * the new value when the write changes the HIGH word. For a monotonic
 * microsecond clock that happens once per 71.6 minutes; for anything set to 0,
 * or crossing zero, every time.
 *
 *   local_start   FIXED, by single ownership rather than by a lock. It had TWO
 *                 writers -- streamer_send_sbc() assigned the instant,
 *                 local_play_task() zeroed it to park -- and a seqlock with two
 *                 writers is silently broken, so the exposure was accepted for
 *                 as long as that was true. The play task now parks on
 *                 local_epoch and writes neither, leaving one writer and a
 *                 32-bit handshake that cannot tear. See the declaration.
 *
 *                 The satellite's stream_start_local is the same shape and still
 *                 has two writers; this is the worked example for fixing it.
 *
 *                 Worth knowing that this part is DUAL-CORE and "play" is pinned
 *                 to core 1 while the others float, so these tasks genuinely run
 *                 at the same time rather than interleaving on one core.
 *   s_marker_at   accepted. Written by play, read by monitor_task, and only in
 *                 marker builds -- a bench instrument nothing corrects on.
 *   s_sync_at / s_hub_splice_at / s_retune_done_at / s_retune_outage_us
 *                 accepted. All are compared against a threshold or printed;
 *                 a torn read costs one wrong log line, not a wrong decision.
 *   s_vis_anchor_due
 *                 accepted. Interpolation anchor for the analysis; a torn read
 *                 mislabels one frame's due time and the next packet replaces
 *                 it.
 *
 * See docs/hub-audit.md H2 for the audit these conclusions came from.
 */
#pragma once

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "esp_attr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "sdkconfig.h"

#include "audio_out.h"
#include "sync_proto.h"
#include "visualiser.h"
#include "wifi_log.h"

#include "streamer.h"

/* Declared in components/dancefloor_sync/Kconfig, not here: the satellite needs
 * the same two values to join, and it used to carry its own #define of each. */
#define AP_SSID   CONFIG_DANCEFLOOR_AP_SSID
#define AP_PASS   CONFIG_DANCEFLOOR_AP_PASS

/*
 * How far ahead of playback each chunk is stamped. Must exceed worst-case
 * network delivery -- measured RTT peaked at 28 ms, so this is a ~7x margin.
 *
 * Reduced from 250 ms so that LEAD + RESYNC stays inside the satellite's ring:
 * with bursty input the actual lead swings around this value rather than
 * sitting on it. That ring is 80 kB / 464 ms now and the bound is 350, so there
 * is room here again -- but do not spend it on the lead without a reason the
 * lead itself can serve. It buys tolerance for LATE delivery, and the delivery
 * fault this system has actually suffered was the transmit path refusing sends
 * outright, which no lead recovers. The 30 ms went to RESYNC_US below, which
 * had a measured seven-times-a-minute misfire to its name.
 *
 * The same answers "the S3 hub has PSRAM, can the lead not be 500 ms": the ring
 * a lead must fit in belongs to the SATELLITE, which is a classic ESP32 with no
 * PSRAM and a largest free block of 106 kB against the ~107 kB a 500 ms lead
 * would need. Memory on this board buys the lead nothing.
 */
#define LEAD_US   200000

/*
 * How far the presentation timeline may wander from real time before the slew
 * starts walking it back.
 *
 * Now measured rather than estimated. sbc_in tracks the longest silence between
 * audio packets and every 5 s window contains one of 79 to 112 ms, median 100 --
 * so `next_play_at - target` does not drift, it OSCILLATES: a burst arrives and
 * the timeline races ahead of real time, a gap follows and it falls behind. The
 * trough reaches -132 ms, which is past this threshold, so it trips on entirely
 * normal delivery about seven times a minute and recovers on its own within a
 * second or two.
 *
 * That is survivable now and was not before. The old code JUMPED to `target` on
 * crossing this line, writing a transient trough permanently into the timeline
 * and stepping every unit's phase by the whole amount; the slew moves 1 ms/s,
 * so a trip that lasts a second costs 1 ms and the burst refill does the rest.
 *
 * Now raised past the swing, which is what this comment used to say it wanted
 * and could not have. The blocker was never this constant: LEAD + RESYNC bounds
 * how much a satellite must buffer, and at 200 + 120 = 320 against a 372 ms
 * ring there was no room to add the 30 ms that would clear a 132 ms trough.
 *
 * The satellite's ring is 80 kB now -- 464 ms -- so the bound is 200 + 150 =
 * 350 against 464, and a swing that reaches 132 no longer touches it. What
 * should disappear from the log is the "timeline off by ... slewing back" line
 * arriving about seven times a minute on entirely normal delivery. If it still
 * does, the swing is larger than 132 on this hardware and the number to look at
 * is sbc_in's max gap, not this one.
 *
 * NOTE the ordering: a satellite still running a 64 kB ring against this must
 * buffer 350 ms of a 372 ms ring, which fits but leaves little. Flash the
 * satellite first, or together.
 *
 * The original note, still true: SBC delivery is bursty, and the wire is not why.
 * A2DP packets arrive ~23/s, each
 * carrying ~43 ms of audio that decodes in one go, so the timeline legitimately
 * races ahead while a burst is consumed and falls behind while waiting for the
 * next -- measured swings of +-75 ms. The old 50 ms threshold fired several
 * times a second.
 *
 * The I2S link used to hide this by pacing the audio. Neither the UART that
 * replaced it nor the SPI link that replaced that one paces anything: both
 * deliver a packet as soon as the bridge has one.
 *
 * This does not affect the local ring, which is governed by rate rather than by
 * the timeline. It does set how far a satellite's start time can be off, so
 * LEAD + RESYNC must fit the satellite ring: 200 + 150 = 350 ms against 464 ms.
 */
#define RESYNC_US 150000

/*
 * Past this the timeline is not merely off, it is wrong, and no gradual
 * correction closes it in useful time. Jump instead.
 *
 * 300 ms, down from 1 s, and the number that matters is LEAD_US rather than
 * anything about the slew.
 *
 * A satellite anchors on a packet whose play_at is still ahead of it, and the
 * lead it sees is LEAD_US plus this error. So at err = -100 ms the lead has
 * already fallen to the 100 ms a satellite insists on before it will anchor at
 * all, and past roughly -200 ms packets arrive with their play_at already gone.
 * Beyond that the timeline is not merely inaccurate, it is unusable: no
 * satellite can start, and any satellite already playing is being asked to
 * absorb an error far outside the +-100 Hz its servo can trim.
 *
 * Which is what made 1 s wrong. A displacement of ~380 ms was measured sitting
 * below it and therefore slewing: the hub's own phase read +281 ms and walked
 * back at exactly the 0.9 ms/s the slew delivers, taking about five minutes,
 * with its local ring starved to 46 ms and its servo alternating 44200 / 44080
 * the whole time because a starved ring and a late phase are the same fact seen
 * twice. The satellite meanwhile refused 237 consecutive packets, gave up, and
 * anchored 317 ms late. Nothing was being protected by correcting that
 * gradually, because nothing downstream was still working.
 *
 * The slew keeps everything it was right about. Between RESYNC_US and this, the
 * error is small enough that satellites can still anchor and still follow, so
 * moving smoothly is worth more than moving fast -- see the long note above
 * TIMELINE_SLEW_US, and the -126 ms excursion that argued for it. This only
 * changes where "gradual" stops being a kindness.
 */
#define RESYNC_HARD_US 300000

/*
 * How steadily the source must be delivering before a timeline may start.
 *
 * A timeline start publishes an origin every unit anchors to, and then playback
 * begins LEAD_US later. That only works if the source can hand over LEAD_US of
 * audio in LEAD_US of wall clock. Nothing checked whether it could.
 *
 * Measured, at a track start: `timeline start` fired on `pkts 12 | eff 2070 Hz`
 * with a 1.76 SECOND hole in the SBC input behind it. Playback began, drained,
 * and underran 700 ms later; the recovery restarted the timeline, and what came
 * out the far side was displaced 338 ms -- which every satellite then saw as
 * packets arriving past their play_at, refusing 237 in a row and anchoring 317
 * ms late. The whole cascade came from starting a timeline on a source that was
 * not yet running.
 *
 * STALL is what separates a real hole from the ordinary burst pattern. A2DP
 * arrives in lumps of ~43 ms and sbc_in's `max gap` sits at 110-150 ms in every
 * healthy window, so 300 ms is clear of normal and far below the 1.76 s that
 * mattered.
 *
 * STEADY is how long it must go without one. 500 ms is more than twice LEAD_US,
 * so a source that manages it can fill the ring before playback reaches it.
 *
 * GIVE_UP bounds the wait, for the same reason the satellite's anchor guard has
 * one: a source that stalls forever is not fixed by refusing to play it, and a
 * hub that stays silent through a whole track is the worse failure. Say so and
 * start anyway.
 */
#define SOURCE_STALL_US    300000
#define SOURCE_STEADY_US   500000
#define SOURCE_GIVE_UP_US 5000000

/*
 * How far the timeline moves per packet while slewing back to real time.
 *
 * At ~50 packets/s this is 1 ms/s. The bound that matters is the servo's: it
 * trims at most +-100 Hz, which is 2.27 ms/s at 44.1 kHz, so a slew near that
 * outruns the units it is supposed to be leading.
 */
#define TIMELINE_SLEW_US 20

/*
 * Throttles on the non-audio downlink lanes. Audio is the priority: a dropped
 * audio datagram costs a gap, where a dropped frame or ML result costs only a
 * slightly-stale LED. publish_frame and publish_ml can EACH run at the ~86 Hz
 * analysis rate (Kconfig.projbuild), so together they add ~172 datagrams/s on
 * top of ~50 audio and overflow the 26 static TX buffers (tx-fail ENOMEM,
 * measured -- see the errno tally tx_fail_note() keeps).
 *
 * ML_PUBLISH_PERIOD_US caps the ML lane well under the analysis rate. TX_BACKOFF_US
 * is how long BOTH non-audio lanes stay silent after ANY sendto() returns ENOMEM:
 * the instant the pool is exhausted, non-audio yields, leaving the buffers audio
 * was being refused for audio. fan_out() -- the audio path -- is never gated.
 *
 * Falsified by tx-fail on the servo line: throttling is working while that drops.
 */
#define ML_PUBLISH_PERIOD_US   100000   /* 10/s, down from up to 86/s */
#define TX_BACKOFF_US           40000   /* ~2 audio packets: yield, then probe again */

/*
 * Local playback ring. The master delays its own audio by LEAD_US exactly like a
 * satellite, otherwise it would play ahead of every other speaker.
 *
 * THE LEAD IS ~35 kB, not the ~21 kB this said until 2026-08-12. LEAD_US is
 * 200 ms and 200 ms at 44.1 kHz stereo is 35,280 bytes; the old figure described
 * a shorter lead and was never updated when it grew. It mattered, because it made
 * this ring look far better provisioned than it was: 64 kB reads as three times
 * the lead against 21 kB and is only 1.86x against the real one.
 *
 * 48 kB is 279 ms, DOWN FROM 64 kB / 371 ms, and the 16 kB it releases is spent
 * on WiFi static TX buffers -- see ESP_WIFI_STATIC_TX_BUFFER_NUM in
 * sdkconfig.defaults, which is the other half of this change. This ring was 48%
 * of the internal heap and the only place a WiFi-sized block existed: internal
 * ran 7,760 bytes free with a 3,584 largest block, so nothing could grow until
 * something here shrank.
 *
 * WHY 48 AND NOT LESS. Measured occupancy over a full run was 31,744-39,424
 * bytes (179-223 ms), so 48 kB leaves 9,728 bytes above the observed peak. The
 * constraint is not the steady state but the FEED BURST: sbc_in reports
 * `max gap` of 34.8-42.0 ms, so the decoder pauses and then delivers in a lump,
 * and the ring has to swallow the lump or drop it. 9,728 bytes is 55 ms, which
 * clears the worst gap measured (42 ms) and not by much. 44 kB would leave 26 ms
 * and lose to a gap this unit has already produced.
 *
 * FALSIFIED BY fed-drop, which is why that counter exists. Non-zero here means
 * the burst no longer fits and the ring was the wrong donor; put it back to 64 kB
 * and take the TX buffers from somewhere else.
 */
#define LOCAL_RING_BYTES (48 * 1024)

extern const char *TAG;

extern StreamBufferHandle_t local_ring;
extern i2s_chan_handle_t i2s_tx;
extern int sock;
extern volatile uint32_t sample_rate;
extern uint32_t tx_rate;   /* what the DAC clock is actually set to */
extern uint32_t rate_ema;   /* smoothed measured input rate */

#if CONFIG_DANCEFLOOR_ENABLE_MARKER
extern volatile int64_t s_marker_at;   /* local time we last pulsed */
#endif

/*
 * Ring position at which a tagged packet's audio begins, or -1 for none
 * pending. Playback pulses when it reaches this, so the pulse tracks the audio
 * through the buffer rather than being predicted from a clock.
 */
/*
 * 32-bit, not 64: read by the playback task while the receive task writes them,
 * and a 64-bit load is two instructions here -- a torn read gives a garbage
 * position and a wild marker. int32 holds 13 hours of frames at 44.1 kHz.
 */
extern volatile int32_t s_marker_sample;
extern volatile int32_t s_samples_in;   /* frames written into local_ring */

/*
 * Phase tracking, matching the satellite.
 *
 * This unit publishes the timeline, so it should hold itself to it. Servoing on
 * buffer depth alone matches its playback RATE to its input but lets its
 * POSITION wander -- and since it wanders independently of every satellite, the
 * speakers separate even though each one's buffer looks perfectly stable.
 */
#define PHASE_Q_LEN 32
typedef struct {
    int32_t pos;
    int64_t play_at;
} phase_pt_t;

extern phase_pt_t s_phase_q[PHASE_Q_LEN];
extern volatile uint32_t s_phase_head, s_phase_tail;
extern volatile int32_t s_phase_err_us;   /* + = playing late */
extern volatile bool s_phase_valid;
/*
 * The last few raw readings, for the splice. Play task only -- pushed in the
 * crossing loop, read and reset in the splice, reset at the top of the outer
 * loop -- so no volatile and no lock. See sync_phase_hist_t for why the splice
 * needs its own filter and cannot borrow the servo's.
 */
extern sync_phase_hist_t s_phase_hist;
/*
 * The median of that history, published for the servo.
 *
 * The servo runs on ring_monitor_task and s_phase_hist is play-task-only, so
 * the servo cannot take the median itself -- and taking one across its own 5 s
 * samples would filter at the wrong cadence, adding tens of seconds of lag to
 * reject scatter that lives inside 180 ms. The play task already has the
 * history at packet cadence, so it publishes the answer instead.
 *
 * Cleared, not just stale, when the history is reset: a median that survived a
 * splice would describe the phase the splice just removed. Valid stays false
 * until SYNC_PHASE_MIN readings have arrived, which is what the servo's fallback
 * to the raw reading is for.
 */
extern volatile int32_t s_phase_med_us;
extern volatile bool s_phase_med_valid;
/* Set when a splice steps the phase, so the shadow average in ring_monitor_task
 * forgets a history that describes the situation before it -- the satellite has
 * done this since "Forget the phase average after a splice". */
extern volatile bool s_phase_stepped;
extern volatile bool s_restart_pending;   /* flag the next packet */
/*
 * Packets left before a post-hard-jump boundary is flagged, or 0 for none armed.
 *
 * A jump at RESYNC_HARD_US used to be announced to nobody: satellites took a
 * step of up to 300 ms with no splice hint and, being below PHASE_INSANE_US, no
 * re-anchor either, then walked it off through their servos at 2.27 ms/s. The
 * flag cannot fire ON the jump, because every unit's phase reading still
 * describes the timeline that just ended, so it is delayed SYNC_PHASE_HIST
 * packets -- long enough for the splice's median window to be clear of the
 * discontinuity. Timeline task only; no other task touches it.
 */
extern uint8_t s_jump_arm;

/*
 * Set when local playback underruns. The play task then waits for local_start,
 * which is only assigned at a timeline start -- so without this the hub stays
 * silent until the stream stops and restarts, discarding every incoming byte in
 * the meantime (seen as fed-drop climbing to the full stream rate).
 *
 * Recovery restarts the timeline, which also re-anchors every satellite, so the
 * system comes back aligned rather than merely audible.
 */
extern volatile bool s_underrun_recover;
extern volatile int32_t s_restart_pos;

/* Never splice more than this in one go -- a larger error means something a
 * splice will not fix, and 150 ms is audible even at a track change. */
#define MAX_SPLICE_MS 150

/* Held across a DAC retune, and the play task parks on it.
 * i2s_channel_write() returns immediately once the channel is disabled, so
 * without this the play task spins through the ring at memory speed -- a
 * measured 54 ms correction cost 177 ms of buffer. */
/* True while the timeline is being walked back to real time -- see the slew in
 * streamer_send_sbc(). */
extern bool s_slewing;
extern bool s_slew_told;   /* whether this episode has been announced */
extern int64_t s_slew_since;   /* when it started, for the 5 s filter */

extern volatile bool retuning;
/*
 * The instant local playback begins, and the handshake that publishes it.
 *
 * ONE WRITER: the timeline path, at a start. That is the fix H2 named and it is
 * why the pair below can be read safely at all.
 *
 * It used to have two -- streamer_send_sbc() assigned the instant, and
 * local_play_task() zeroed it on an underrun to make itself park. Two writers is
 * what made a seqlock impossible, so the 64-bit tearing exposure was documented
 * and accepted rather than fixed. The play task now parks on the EPOCH instead
 * and writes neither, which removes the second writer and the exposure together.
 *
 * local_epoch is incremented AFTER local_start is written, and the play task
 * reads it BEFORE reading local_start. A reader that sees a new epoch therefore
 * sees the value that belongs to it: both are volatile, so the compiler may not
 * reorder the two stores or the two loads, and this core orders stores in
 * program order. 32-bit, so the epoch itself cannot tear -- the same rule
 * s_marker_sample and s_samples_in are written down for.
 *
 * The epoch wrapping after 2^32 starts is harmless: the test is inequality
 * against the last one seen, not ordering.
 */
extern volatile int64_t local_start;
extern volatile uint32_t local_epoch;

/*
 * Whether the play task is actually playing.
 *
 * Written only by the play task. The servo used to ask `local_start == 0` and
 * that stopped being the question the moment local_start gained a single owner:
 * it now holds the last start instant for ever rather than being zeroed at an
 * underrun, so it can no longer answer "is anything playing". This can, and it
 * is owned by the task that knows.
 */
extern volatile bool s_playing;

/* Bytes dropped because pcm_stream was full. Silent loss here looks exactly
 * like a starving ring, which is why it needs a counter. */
extern volatile uint32_t s_feed_dropped;
extern volatile uint32_t s_tx_fail;   /* sendto() rejections */
/*
 * Audio datagrams handed to fan_out() this window, printed as a rate.
 *
 * The packet RATE is a load-bearing number that nothing reported. Two constants
 * are sized against it -- TIMELINE_SLEW_US is per packet and assumes ~50/s, and
 * the TX buffer pool is consumed per packet -- so a change to packetisation
 * moves the timeline slew and the transmit pressure together, silently. A commit
 * that doubled it got through review, a build and two test suites, and was only
 * caught by a floor that stopped working. `pkts/s` on the status line is what
 * would have caught it in the first log window.
 *
 * Note this is NOT sbc_in's `pkts`, which counts SPI frames from the bridge:
 * that stayed at ~250 per window throughout, because the fault was downstream of
 * it. The two numbers were equal until packetisation stopped being one-to-one,
 * which is exactly when it mattered that they are different questions.
 */
extern volatile uint32_t s_audio_pkts;
/* ... and the subset of them that were AUDIO, which is the only subset that is
 * audible. See tx_fail_note_audio(). */
extern volatile uint32_t s_tx_fail_audio;

/*
 * Non-audio publish throttling, paired with ML_PUBLISH_PERIOD_US / TX_BACKOFF_US.
 * tx_fail_note() raises s_tx_congested_until on ENOMEM; publish_frame/publish_ml
 * skip while now < it and count the skip in n_tx_cong_skip. publish_ml additionally
 * rate-limits to the period and counts drops in n_ml_throttled. fan_out() ignores
 * both -- audio always sends. Racy exactly as s_tx_fail is (net.c): a torn read of
 * the 64-bit deadline at worst sends or skips one non-audio datagram wrongly.
 */
extern volatile uint32_t n_ml_throttled;            /* ML sends dropped to the period */
extern volatile uint32_t n_tx_cong_skip;            /* non-audio sends skipped under ENOMEM backoff */
extern volatile int64_t s_tx_congested_until;       /* esp_timer deadline; non-audio yields until it */

/*
 * Every failed sendto() goes through this rather than incrementing s_tx_fail
 * directly, so the reason is kept alongside the count. Call it with errno, at
 * the failure, before anything else can overwrite it. tx_fail_summary() renders
 * the tally for the status line and clears it. Both live in net.c, which owns
 * the socket; the rationale for keeping the reason at all is there.
 */
void tx_fail_note(int err);
/* The audio downlink's own entry point, which also counts s_tx_fail_audio. */
void tx_fail_note_audio(int err);
void tx_fail_summary(char *buf, size_t len);
/* Both units count the DMA running dry; see on_tx_starved() in out.c. */
uint32_t dma_starve_count(void);

/*
 * Cumulative totals for a long run, never reset -- deliberately separate from
 * the per-window counters above, which are cleared every 5 s.
 *
 * A rate tells you what is happening now; a total tells you whether something
 * has been happening slowly for an hour. Nothing here had one, so the longest
 * evidenced session was seven minutes and a leak or a slow decay would have
 * been invisible. This project's own lesson: every real fault was invisible
 * until something counted it.
 */
extern volatile uint32_t n_underruns;   /* local playback ran dry */
extern volatile uint32_t n_restarts;   /* timeline restarted */
extern volatile uint32_t n_splices;   /* track-boundary corrections applied */
extern volatile uint32_t n_retunes;   /* DAC clock changes that succeeded */
extern volatile uint32_t n_retunes_bad;   /* refused or failed */
extern volatile uint32_t n_sta_left;   /* satellites disassociating */
extern volatile uint32_t hw_play;   /* stack headroom, sampled in-task */
extern volatile uint32_t hw_mon;

/*
 * Heap, with a timestamp -- because the all-time figure could not provide one.
 *
 * esp_get_minimum_free_heap_size() is a watermark since boot. A run that ended
 * at 2040 bytes free, against 21892 on two clean runs before it, said only that
 * the hub had nearly died at some point in five minutes: not when, not during
 * what, and not whether it was the same moment as the underrun on the same line.
 * The heap is one large block plus scraps -- `largest` reads 26624 in every log
 * ever captured here -- so reaching 2040 means that block went in one piece, and
 * the only thing on this unit that takes that much on demand is WiFi's 32
 * dynamic TX and 32 dynamic RX buffers. Attributing it needs a window, not a
 * watermark.
 *
 * Sampled every 5 s and cleared by each HEALTH line, so a dip lands in a named
 * minute beside sta-left, restarts and underruns. A dip shorter than the sample
 * interval is still missed; that is what the allocation hook below is for.
 */
extern volatile uint32_t heap_min_window;

/*
 * The pool that actually says yes or no to a WiFi buffer.
 *
 * Everything above measures the whole heap, and on this board that is mostly
 * PSRAM. CONFIG_SPIRAM_USE_CAPS_ALLOC means ordinary malloc() never returns
 * PSRAM, so the ring, the DMA buffers, the frame queue, the WiFi buffers and
 * every stack live in internal SRAM -- and internal SRAM is what runs out. The
 * whole-heap figure cannot see it: this unit has reported 8407580 bytes free in
 * the same second that a 1700-byte MALLOC_CAP_INTERNAL request failed.
 *
 * Same window discipline as heap_min_window above. No since-boot twin is needed
 * because heap_caps_get_minimum_free_size() already keeps a per-capability
 * watermark, which esp_get_minimum_free_heap_size() does not.
 *
 * 8BIT is part of the mask and is not decoration. MALLOC_CAP_INTERNAL alone also
 * matches regions that are internal but 32-bit-access-only, which nothing that
 * needs byte access -- malloc(), a task stack -- can use. It costs nothing here,
 * because the S3 registers no IRAM-only region with this build's cache setting,
 * but it cost an evening on the satellite, where the IRAM heap is registered as
 * INTERNAL|EXEC|32BIT and this figure read 31 kB free while the pool a stack
 * comes from had 396 bytes. The satellite's copy carries that story in full.
 * It is also exactly the mask of the requests that fail: caps 0x804.
 */
#define CAP_USABLE_INTERNAL (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)

extern volatile uint32_t heap_int_window;

/*
 * Allocation failures, which until now were silent.
 *
 * At 2040 bytes free something very likely failed to allocate, and nothing said
 * so -- it surfaced as an underrun, which names the symptom and not the cause.
 * CONFIG_HEAP_ABORT_WHEN_ALLOCATION_FAILS would panic instead, which is too
 * violent for a dance floor; this counts and reports.
 *
 * The hook records and does not log; see the constraints where it is defined.
 * ring_monitor_task notices within 5 s from a context where logging is safe.
 */
extern volatile uint32_t n_alloc_fail;
extern volatile uint32_t alloc_fail_size;   /* the largest request that failed */
extern volatile uint32_t alloc_fail_caps;

/*
 * Tasks that did not start, which used to be unsayable.
 *
 * Every xTaskCreate in this tree passed NULL for the handle and discarded the
 * return value, so a unit that could not start a task ran on without it and
 * looked like one that had simply gone quiet. On this unit that would cost the
 * timeline, the local speaker or the monitor line depending on which one lost;
 * on the satellite it cost all three at once and there was no HEALTH line left
 * to say so. Same silent-failure class as the allocation hook above, left in the
 * one place that can take a whole unit down.
 */
extern volatile uint32_t n_task_fail;
extern char s_task_fail_names[64];

/*
 * Phase points dropped because s_phase_q was full -- the only loss path in this
 * file with no counter. It is worth one, because a wedged phase queue is not a
 * degradation here but a stop: the ring servo runs on nothing but points coming
 * off this queue, and it has already been wedged once for a whole session by a
 * stale entry queued across a timeline restart. Every log line read normally
 * throughout.
 */
extern volatile uint32_t n_phase_drop;

/*
 * Ring reads that came back short of a chunk, and the frames of silence padded
 * in to cover them. Suspected, not established -- see the satellite's copy for
 * the full argument. The pad is played but was never in the ring, while
 * samples_played advances by a whole chunk regardless, so if this fires at all
 * it is a permanent displacement of every later phase point.
 */
/*
 * Phase readings discarded because the DMA was still filling at memory speed.
 *
 * Non-zero at every playback start is EXPECTED and is the guard working: the
 * channel drained while the play task was parked, so the first writes return
 * without blocking and anything measured against them is not a phase error. A
 * figure that keeps climbing mid-stream is not expected and means the channel is
 * repeatedly running empty, which is an underrun by another name.
 */
extern volatile uint32_t n_refill_withheld;
extern volatile uint32_t n_short_reads;
extern volatile uint32_t n_short_frames;

/*
 * How much audio goes into the DMA before a write first blocks.
 *
 * The satellite has had this since a retune was found to be costing tens of
 * milliseconds; the hub never did, and the hub is the unit with the larger
 * startup offset. i2s_channel_write() does not block while descriptors are
 * free, so on an empty channel the first writes return at memory speed:
 * samples_played advances by the whole DMA depth (6 x AUDIO_FRAMES = 34.8 ms
 * at 44.1 kHz) against a wrote_at that has barely moved, and every phase
 * reading dated inside that window is measured against a reference the DAC is
 * not pacing.
 *
 * The channel is empty at every playback START as well as after a retune -- the
 * play task was parked, so it drained. Nothing guards it there on either unit.
 * This is very likely the "-42 ms (hub), -26 ms (satellite)" startup phase in
 * clock-sync.md §8: a 16 ms difference between two units, on a cold start, on
 * both at once, taking ~45 s to walk off. On a reconnect only the satellite
 * restarts, against a hub already servoed to zero, which is why that case
 * behaves so much better.
 *
 * MEASUREMENT ONLY. Nothing is withheld and nothing is corrected; this says how
 * big the window is so a guard can be sized rather than guessed.
 */
#define REFILL_FAST_US 1000     /* below this, the write did not block */
extern bool s_refill_active;
extern int32_t s_refill_frames;
/* s_refill_why is gone with the probe's retune arm: the window only ever
 * happens at a start now, so nothing needs to say which. */

/*
 * How many satellites this hub will carry.
 *
 * 15 is the ceiling the radio imposes, not a guess: ESP_WIFI_MAX_CONN_NUM in
 * esp_wifi_ap_get_sta_list.h. Three separate limits have to agree or the count
 * is whichever is smallest -- this one, wifi_config.ap.max_connection in
 * wifi_start_ap(), and CONFIG_LWIP_DHCPS_MAX_STATION_NUM, which decides how many
 * leases the DHCP server has to give out. A satellite refused by the third
 * associates and then has no address, which reports as a unit that joined and
 * never probed rather than as a floor that is full.
 *
 * Was 8, from when audio and analysis frames were both unicast and airtime
 * scaled with speaker count -- 50 + ~96xN packets a second, which 8 already
 * strained. Both are group-addressed now, so the hub's transmit rate is ~146
 * packets a second flat and what scales with N is only the probe traffic: 4
 * probes a second per satellite, replied to individually, which at 15 is ~180
 * small packets a second. That is the arithmetic that makes 15 affordable; see
 * the airtime table in the audit.
 *
 * MEASURED AT TWO. Nothing past one satellite had been run when this was 8, and
 * nothing past two has been run now. The number says what the design intends to
 * carry, not what has been demonstrated.
 *
 * Registration is implicit: satellites already send time probes every 250 ms, so
 * anything that has probed recently is listening.
 */
#define MAX_CLIENTS 15

/*
 * How long a satellite that has stopped probing stays on the send list.
 *
 * Was 10 s, which is 40 probes at PROBE_PERIOD_MS -- far more tolerance than
 * losing probes needs, and the window during which this unit keeps unicasting
 * audio and 86 analysis frames a second at a station that is not there. Each of
 * those sends takes a 1152-byte DMA buffer the driver cannot deliver and will
 * not free until it gives up, and the pool is finite: pulling a satellite mid-
 * track produced 124 failed allocations over exactly ten seconds, free heap down
 * to 4580 bytes and the largest block from 26624 to 1216, recovering the instant
 * this timeout expired. An earlier instance of the same thing cost an underrun.
 *
 * 2 s is 8 consecutive probes, which is still generous against a link measured
 * essentially clean, and cuts that window five-fold. Not shorter: a satellite
 * forgotten in error gets no audio until its next probe re-registers it, and at
 * ~150 ms of ring against a 250 ms probe period that is an underrun rather than
 * a glitch. The event handler below is what makes a clean disassociation instant
 * regardless; this bound is for the satellite that vanishes without saying so.
 *
 * What remains after both is 3 to 5 failed allocations per disconnect taken MID-
 * TRACK, and that is the floor rather than a leftover to chase. Those are frames
 * handed to the WiFi driver before client_gone() ran -- about 20 buffers
 * outstanding by the same heap arithmetic that gave the 124 above -- and nothing
 * above the driver can reach a buffer it has already taken. Audio is unaffected:
 * underruns stay 0 across it. A run showing 3-5 here is this working, not
 * regressing.
 *
 * Mid-track is load-bearing in that sentence. A satellite that leaves while
 * nothing is playing costs zero, because the residual IS the in-flight sends and
 * there are none: a disassociation observed 110 s before the first audio came
 * through reported alloc-fail 0, with the heap window never below 25 kB against
 * the 1748 B the exhaustion left. Zero here is an idle disconnect, not a better
 * floor, and reading it as one would hide the case that matters.
 */
#define CLIENT_TIMEOUT_US 2000000

typedef struct {
    struct sockaddr_in addr;
    int64_t last_seen;
} client_t;

extern client_t s_clients[MAX_CLIENTS];
extern portMUX_TYPE s_clients_lock;
extern esp_netif_t *s_ap_netif;   /* for the MAC -> IP lookup below */
extern volatile uint32_t n_sta_dropped;   /* forgotten on the event, not the timeout */
extern volatile uint32_t n_sta_nolease;   /* ... and the times the lookup could not say who */
/*
 * ... and the ones the timeout got, which nothing counted.
 *
 * This is the only path by which a satellite leaves the send list without
 * incrementing anything, and it is the path an ungraceful departure takes: the
 * AP notices inactivity far later than CLIENT_TIMEOUT_US, so a unit that loses
 * power or walks out of range is dropped here and raises no event. A HEALTH line
 * therefore read identically whether a satellite had vanished mid-track and come
 * back or nothing had happened at all, which made a disconnect test
 * uninterpretable after the fact -- the counters could not say whether a quiet
 * run meant the departure was handled well or was never seen.
 */
extern volatile uint32_t n_sta_timeout;

/* SBC the SPI link delivered but the WiFi ceiling refused. Should be 0 while
 * AUDIO_MAX_PAYLOAD tracks SBC_LINK_MAX_PAYLOAD; counted, not silent, because a
 * ceiling drift here would otherwise read as satellite-side gaps. */
extern uint32_t n_wifi_oversize;

/*
 * Redundant copies the MTU forced short.
 *
 * Zero while DANCEFLOOR_AUDIO_FEC_DEPTH is 0, which is the default and why this
 * is quiet today. With redundancy on it counts almost every packet, and that is
 * the honest reading rather than a fault: an ~825-byte payload leaves ~618 bytes
 * for a copy, so ~1/4 of each one is missing and the satellite pads it with
 * silence -- about 6 ms per "recovered" packet. The satellite's
 * n_fec_short_frames is the same fact seen from the receiving end.
 *
 * A cap that made copies fit whole was tried and reverted; see
 * DANCEFLOOR_AUDIO_FEC_DEPTH's Kconfig help. So this counter is what any future
 * attempt at redundancy has to drive to zero, and what says immediately whether
 * a given payload size and depth actually fit each other.
 */
extern uint32_t n_fec_truncated;

/* Ring position and scheduled instant of the last packet sent, so the analysis
 * -- fed on arrival, before this packet's stamp exists -- can date what it is
 * given. See where they are set. */
extern volatile int32_t s_vis_anchor_pos;
extern volatile int64_t s_vis_anchor_due;

/*
 * Capture where the next packet's audio will start, before it is fed.
 *
 * Needed because a packet's play_at is not known until streamer_send_sbc()
 * computes it, by which point s_samples_in has already advanced past that
 * packet's audio. The pair only means anything if both halves refer to the
 * same instant in the stream.
 */
extern int32_t s_pending_pos;

/*
 * Last cross-unit measurement, for the per-track summary below.
 *
 * A track boundary nulls phase on every unit, so cross-unit error resets there
 * and grows until the next one. That makes the reading taken just BEFORE a
 * boundary the one worth keeping: it is how far apart the speakers had drifted
 * over a whole track, which is the number to compare sessions and builds on.
 * Any other sample depends on where in the track cycle it was taken, and
 * comparing two of those produced three confident wrong diagnoses in a row.
 */
extern volatile int64_t s_sync_err_us;
extern volatile int64_t s_sync_at;   /* 0 = never measured */

/* This unit's own last boundary correction, for satellites to be compared
 * against when they report theirs. */
extern volatile int32_t s_hub_splice_us;
extern volatile int64_t s_hub_splice_at;   /* 0 = no boundary yet */

/*
 * SHADOW, acted on by nothing: what this unit's boundary correction would have
 * been had the splice used sync_phase_median() instead of the single most
 * recent phase reading. See sync_phase_hist_t.
 *
 * Reported beside the real figure on the same line so the two can be compared
 * at the same boundary, which is the only place they are comparable. Judging
 * the filter by flashing it and reading a log window is exactly the mistake
 * that produced three wrong diagnoses here; this makes the comparison
 * simultaneous instead.
 */
extern volatile int32_t s_hub_splice_alt_us;

/*
 * Widest DRIFT correction the servo may ask for, in Hz. Deliberately not
 * applied inside retune_dac(): the initial match to the measured input rate is
 * a different thing and is legitimately several percent (44100 nominal against
 * ~42600 measured), so a bound tight enough to be useful here would refuse it.
 */
#define RATE_TRIM_MAX_HZ 100

/* What could conceivably be an audio sample rate at all. Anything outside this
 * is a broken calculation, whoever asked for it. */
#define RATE_SANE_MIN 8000
#define RATE_SANE_MAX 192000

/* What a retune costs -- see the note on the satellite's copy. The channel down
 * is measurable here; the discarded DMA buffer is not measurable anywhere in
 * software, because those frames were counted as played. */
extern volatile int32_t s_retune_phase_before;
extern volatile bool    s_retune_watch;
extern volatile int64_t s_retune_outage_us;

/*
 * When the retune finished, and how many crossings have been narrated since.
 *
 * MEASUREMENT ONLY. The servo still withholds exactly one reading, as before;
 * these only add ages to the lines it already prints and narrate the three
 * crossings after it without withholding them.
 *
 * The question they answer is whether withholding one crossing is enough.
 * Crossings arrive one per audio packet, ~20 ms apart, and the note in the
 * crossing loop records the transient landing 1-22 ms after the retune with the
 * buffer refilling "over the following few writes" -- so a one-shot flag may be
 * covering the first third of a disturbance and handing the rest to the servo
 * as position error. Every retune then injects what the next one corrects,
 * which is the loop that pinned the trim at RATE_TRIM_MAX_HZ and ran phase to
 * +500 ms before the `retuning` park existed.
 *
 * That comment says "if the transient turns out to outlast it, the REFILL line
 * says so". It does not, for the hub -- REFILL is a satellite instrument. This
 * is the hub's version, and it sizes RETUNE_SETTLE_US directly instead of by
 * inference.
 */
extern volatile int64_t s_retune_done_at;
extern volatile uint8_t s_retune_tail_left;

/* ------------------------------------------------------------------ across
 * the module boundary. Everything below was a static function in streamer.c
 * and is now called from another file; nothing new was introduced.
 */

/* out.c -- the output clock. */
void i2s_start(uint32_t rate);
void retune_dac(uint32_t hz);

/* net.c -- SoftAP and the sync socket. */
void wifi_start_ap(void);
void socket_start(void);
#if CONFIG_DANCEFLOOR_AUDIO_MCAST
/* The group address, resolved once and owned by net.c. Both the audio path and
 * the analysis frames send to it; see the note beside the definition. */
const struct sockaddr_in *mcast_addr(void);
#endif

/* clients.c -- the send list, and every fan-out over it.
 *
 * client_joined() and client_gone() are called from net.c's WiFi event handler:
 * the events arrive on the radio, the bookkeeping they drive is the send list's.
 * client_seen() is called from probe.c, since probing implies listening. */
void client_seen(const struct sockaddr_in *from);
void client_joined(const uint8_t mac[6], const esp_ip4_addr_t *ip);
void client_gone(const uint8_t mac[6]);

/*
 * Snapshot the send list under the spinlock.
 *
 * Four call sites open-coded this identically -- streamer_send_meta,
 * publish_frame, publish_ml and streamer_send_sbc. The duplication publish_ml's
 * comment defends is the MESSAGE BUILDING, which is untouched: those stay
 * separate because they are on different cadences and each is likely to grow
 * its own rate limit. This is only the copy, which was byte-identical in all
 * four and is where MAX_CLIENTS scaling lands.
 *
 * Returns how many slots were copied (always MAX_CLIENTS); the caller still
 * tests last_seen, because a zeroed slot is how "not listening" is spelled.
 */
void clients_snapshot(client_t *dst);

/* Drop satellites past CLIENT_TIMEOUT_US. Called before each audio send AND from
 * the 5 s tick -- it used to run only in the send loop, so nothing aged out while
 * audio was stopped. */
void clients_age(int64_t now);

/* Guards match the definitions in clients.c exactly, which are the guards these
 * two were written under in streamer.c: the frame publisher exists whenever the
 * visualiser does, and PUBLISH_FRAMES gates only whether streamer_start()
 * registers it. */
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
void publish_frame(const vis_frame_t *f);
#endif
#if CONFIG_DANCEFLOOR_PUBLISH_ML
void publish_ml(const ml_result_t *r);
#endif

/* play.c, probe.c -- tasks. */
void local_play_task(void *arg);
void probe_task(void *arg);
void ring_monitor_task(void *arg);

/* servo.c / telemetry.c -- the two halves of what ring_monitor_task used to be.
 * Called in this order once every 5 s, which is the order they ran in when they
 * were one function: the heap and the counters matter most when audio has
 * stopped, so reporting deliberately precedes the streaming check. */
void telemetry_tick(void);
void servo_tick(void);
void telemetry_register_alloc_hook(void);

#if CONFIG_DANCEFLOOR_ENABLE_MARKER
/* marker.c -- the bench instrument. */
void marker_start(void);
#endif

/* task_start() is declared in streamer.h, not here, so that sbc_in.c can reach
 * it without pulling in this header's `extern const char *TAG` and colliding
 * with its own. */
