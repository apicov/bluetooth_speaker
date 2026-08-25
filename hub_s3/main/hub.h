/**
 * @file hub.h
 * @brief The hub's shared state, and who is allowed to write each piece of it.
 *
 * Roughly ninety values cross between the input task, the playback task, the
 * servo, the probe server and the reporting. The table below states the
 * ownership once so it can be checked; every entry is a statement about which
 * task's execution performs the write, wherever the line of code sits.
 *
 *   sbc_in's rx_task   The streamer_send_* / streamer_feed path in timeline.c
 *                      runs on this task, and with it the input side of the
 *                      timeline: s_samples_in, s_pending_pos, s_marker_sample,
 *                      s_restart_pending, s_restart_pos, the phase queue HEAD,
 *                      the s_vis_anchor pair, local_start + local_epoch, the
 *                      slew trio, s_jump_arm, sample_rate, rate_ema,
 *                      audio_volume, and n_restarts / n_phase_drop /
 *                      n_wifi_oversize / s_feed_dropped / s_audio_pkts.
 *   local_play_task    What playback has reached: the phase queue TAIL,
 *                      s_phase_err_us, s_phase_valid, s_phase_stepped,
 *                      s_phase_hist, the published s_phase_med_us pair, the
 *                      s_hub_splice_* set, the refill pair, s_marker_at,
 *                      n_underruns, n_splices, n_short_*, n_trim_drops /
 *                      n_trim_dups, hw_play and s_playing. Reads rate_trim_hz.
 *   ring_monitor_task  servo_tick() and telemetry_tick() run here: the servo
 *                      owns rate_trim_hz and arms catchup_frames and the
 *                      retune_* watch set; telemetry reads everything else to
 *                      report it. retune_dac() runs on this task and, on a
 *                      gross rate mismatch, on the rx task too -- the shared
 *                      path is why retune_dac() sanity-bounds its argument.
 *   probe_task         The client list, via client_seen().
 *   the WiFi event task  n_sta_left, n_join_churn, n_sta_dropped /
 *                      n_sta_nolease, and the client list via client_joined()
 *                      and client_gone().
 *   the analysis task  publish_frame() in clients.c: s_tx_frame_sent_us.
 *   monitor_task       s_sync_err_us / s_sync_at, marker builds only.
 *
 * `volatile` here means "another task writes this", not "this is atomic".
 * A 64-bit load is two instructions on this CPU, so a reader can catch half
 * of a write; a torn read differs from both the old and the new value only
 * when the HIGH word changes, which for a monotonic microsecond clock happens
 * once per 71.6 minutes -- and for anything set to, or crossing, zero, every
 * time.
 *
 *   local_start   SAFE, by ownership and a handshake rather than a lock: one
 *                 writer, and local_epoch incremented AFTER local_start is
 *                 written while readers read the epoch BEFORE the value. Both
 *                 volatile, stores and loads in program order on this core,
 *                 and the epoch is 32-bit so it cannot tear. See below.
 *   s_marker_sample / s_samples_in   32-bit on purpose -- a 64-bit load tears
 *                 here, and int32 still holds 13 hours of frames at 44.1 kHz.
 *   s_sync_at / s_hub_splice_at / s_retune_done_at / s_retune_outage_us /
 *   s_vis_anchor_due   Accepted: each is printed or compared against a
 *                 threshold, so a torn read costs one wrong log line or one
 *                 mislabelled frame, never a wrong decision.
 *   s_marker_at   Accepted: marker builds only, a bench instrument.
 *
 * The chip is dual-core and local_play_task is pinned to core 1 while the
 * others float, so these tasks genuinely run at the same time rather than
 * interleaving on one core. The satellite's sat.h is the counterpart of this
 * table for the other unit.
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
#include "audio_shift.h"
#include "visualiser.h"
#include "wifi_log.h"

#include "streamer.h"

/** @brief SoftAP credentials, declared in components/dancefloor_sync/Kconfig
 *         so the satellite joins the same network the hub advertises. */
#define AP_SSID   CONFIG_DANCEFLOOR_AP_SSID
#define AP_PASS   CONFIG_DANCEFLOOR_AP_PASS   /**< Same deal: one Kconfig for both units. */

/**
 * @brief How far ahead of playback each chunk is stamped.
 *
 * The margin must clear three things: the satellite's anchor minimum
 * (ANCHOR_MIN_LEAD_US in satellite/main/sat.h), a frame held a whole DTIM
 * period, and the worst transit a loss-free packet can take. LEAD_US minus
 * RESYNC_US is the worst lead a healthy packet can carry, and it must stay
 * above all three.
 *
 * The ceiling is the satellite's ring, a classic ESP32 with no PSRAM: 96 kB
 * is 557 ms and must hold LEAD_US plus RESYNC_US plus the ring-depth swing.
 * Walk this back toward 300 ms if refill-withheld, short-reads or a hub
 * underrun appears on the HEALTH line, and move LOCAL_RING_BYTES here and
 * RING_TARGET_MS / ANCHOR_MIN_LEAD_US on the satellite in step with it.
 */
#define LEAD_US   350000

/**
 * @brief How far the presentation timeline may wander from real time before
 *        the slew walks it back.
 *
 * err is `next_play_at - (now + LEAD_US)`, i.e. the actual lead minus
 * LEAD_US, so this deadband licenses the lead to sit anywhere in
 * LEAD_US +- RESYNC_US. The floor -- LEAD_US - RESYNC_US, 280 ms -- is what
 * must clear the three numbers in the LEAD_US note above; the two constants
 * decide that floor together.
 *
 * SBC delivery is bursty (A2DP arrives in ~43 ms lumps), so err oscillates
 * +-100 ms on normal delivery and crosses this band several times a minute.
 * That is the design, not a failure: the oscillation averages out and the
 * slew is proportional control on the MEAN offset, which is the controller
 * this timeline needs. steer_timeline() reports only 5 s of persistent
 * slewing, so the normal crossings say nothing.
 */
#define RESYNC_US  70000

/**
 * @brief Past this the timeline is not merely off, it is wrong: no satellite
 *        can anchor, and one already playing cannot trim fast enough. Jump.
 *
 * The landmarks sit below it, which is the right way round: at LEAD_US 350
 * the lead reaches the 125 ms anchor minimum around err = -225 ms and packets
 * start arriving past their play_at near -300 ms, so the jump happens only
 * once anchoring is already impossible. A displacement of this size corrected
 * gradually, at the slew's 1 ms/s, is five minutes of a system where nothing
 * downstream is still working; see TIMELINE_HOLD_STARVE_MS for when the jump
 * is itself refused.
 */
#define RESYNC_HARD_US 300000

/**
 * @brief A hard jump is refused while the local ring is under this, because a
 *        starved ring drives err past RESYNC_HARD_US with the clock healthy.
 *
 * Normal delivery jitter swings the ring +-40 ms around the lead; 150 ms is
 * two and a half times the deepest normal trough. Below it, next_play_at
 * stops advancing while `now` keeps moving, so the error IS the starvation.
 * Jumping anyway re-stamps every packet with an error the refill burst will
 * unwind -- a source wandering +-10% swings err across this line several
 * times a minute, and each jump costs audible capped inserts on every unit.
 * Held, the same excursion slews back on its own.
 *
 * TIMELINE_HOLD_GIVE_UP_US bounds a hold that never ends. The primary escape
 * is cleaner: a ring starved all the way to empty underruns, and
 * s_underrun_recover restarts the timeline wholesale with every satellite
 * re-anchoring.
 */
#define TIMELINE_HOLD_STARVE_MS   150
#define TIMELINE_HOLD_GIVE_UP_US  30000000   /**< 30 s: bound on a hold that never ends. */

/**
 * @brief How steadily the source must deliver before a timeline may start.
 *
 * A start publishes an origin every unit anchors to, and playback begins
 * LEAD_US later -- which only works if the source can hand over LEAD_US of
 * audio in LEAD_US of wall clock. STALL separates a real hole from the A2DP
 * burst pattern (healthy 5 s windows carry a 79-112 ms gap, median 100);
 * STEADY is twice LEAD_US, so a source that passes it fills the ring before
 * playback arrives. GIVE_UP bounds the wait: a source that stalls forever is
 * not fixed by staying silent through a whole track.
 */
#define SOURCE_STALL_US    300000   /**< A gap this long is a hole, not a burst. */
#define SOURCE_STEADY_US   500000   /**< Twice LEAD_US: fills the ring before playback. */
#define SOURCE_GIVE_UP_US 5000000   /**< Then say so and start anyway. */

/**
 * @brief Timeline movement per packet while slewing back to real time.
 *
 * ~50 packets/s makes this 1 ms/s. The bound that matters is the servo's:
 * +-100 Hz is 2.27 ms/s at 44.1 kHz, so a slew much faster than this
 * outruns the units it is supposed to be leading.
 */
#define TIMELINE_SLEW_US 20

/**
 * @brief How long the non-audio lanes stay silent after a sendto() returns
 *        ENOMEM: the instant the pool is exhausted, non-audio yields and
 *        leaves what is left to audio. fan_out() -- the audio path -- is
 *        never gated by it.
 */
#define TX_BACKOFF_US           40000   /**< ~2 audio packets: yield, then probe again */

/**
 * @brief 100 TU x 1024 us: how long a group-addressed frame can occupy a
 *        static TX buffer while it waits for a DTIM beacon. Every lane is
 *        unicast now, so nothing waits on the beacon any more; the value
 *        survives as the derivation of TX_FRAME_PACE_US.
 */
#define DTIM_HOLD_US 102400

/**
 * @brief The frame lane's pace: the minimum spacing between two frame
 *        datagrams, holding the lane to ~10 datagrams/s per satellite
 *        instead of the analysis rate of 86/s. Airtime scales with speaker
 *        count, so this is what keeps the hub's transmit rate near flat.
 *
 * Batching latency and lane rate trade against each other, so neither can be
 * changed on arithmetic alone -- the cost of an unpaced lane is per
 * satellite, and tools/satsim is what loads the fan-out to show it.
 */
#define TX_FRAME_PACE_US       DTIM_HOLD_US

/**
 * @brief Frames carried in one frame datagram.
 *
 * 12 against the 8.8 a pace period holds at analysis hop 512; the spare
 * three absorb the analysis task's own lumpiness. Frames append as they
 * arrive and the batch goes out when the pace elapses, so a quiet period
 * sends a short batch rather than a late one. clients.c static-asserts the
 * product against FRAME_PAYLOAD_MAX.
 *
 * This is both the loss-granularity and the batching-delay knob: one lost
 * datagram is TX_FRAME_BATCH consecutive frames, and a frame appended just
 * after a flush waits a whole batch period before it is even offered to the
 * radio. If either shows -- a stutter on the floor, or `late` climbing on
 * the satellite's LED line -- halve this; do not shorten the pace.
 */
#define TX_FRAME_BATCH         12

/**
 * @brief The local playback ring. The master delays its own audio by LEAD_US
 *        exactly like a satellite; playing at the lead is what keeps it in
 *        time with every other speaker.
 *
 * 80 kB is 464 ms. The lead occupies most of it, and what is left is the
 * headroom that has to absorb a gap on the input -- it must stay comfortably
 * wider than the worst gap the source can produce. xStreamBufferCreate
 * allocates from internal SRAM, which is why the ring cannot follow the
 * PCM buffers into PSRAM. fed-drop non-zero means the source stalled longer
 * than the ring holds, and memory is not the answer.
 */
#define LOCAL_RING_BYTES (80 * 1024)

extern const char *TAG;               /**< Log tag, defined per .c file that includes this. */
extern StreamBufferHandle_t local_ring;   /**< The local playback ring, LOCAL_RING_BYTES. */
extern i2s_chan_handle_t i2s_tx;      /**< This unit's I2S TX channel. */
extern int sock;                      /**< The sync socket (owned by net.c). */
extern volatile uint32_t sample_rate; /**< Rate the input reports, stamped onto every packet. */
extern uint32_t tx_rate;              /**< What the DAC clock is actually set to. */
extern uint32_t rate_ema;             /**< Smoothed measured input rate; the servo's baseline. */

#if CONFIG_DANCEFLOOR_ENABLE_MARKER
extern volatile int64_t s_marker_at;  /**< Local time of the last marker pulse (marker builds). */
#endif

extern volatile int32_t s_marker_sample;  /**< Ring position a tagged packet's audio begins, -1 none. */
extern volatile int32_t s_samples_in;     /**< Frames written into local_ring; 32-bit, see the file header. */

/** @brief Phase queue depth: how many stamped points may sit between the
 *         rx task and the play task before the queue drops one. */
#define PHASE_Q_LEN 32

/**
 * @brief One point on the phase queue: a ring position and the master-clock
 *        instant it was scheduled to reach.
 */
typedef struct {
    int32_t pos;       /**< Ring position in frames. */
    int64_t play_at;   /**< Master-clock instant this position is due, us. */
} phase_pt_t;

/** @brief The phase queue: written at the head by the rx task, drained at the
 *         tail by the play task. Full queue drops a point and counts it in
 *         n_phase_drop. */
extern phase_pt_t s_phase_q[PHASE_Q_LEN];
/** head written by the rx task as it stamps, tail advanced by the play task. */
extern volatile uint32_t s_phase_head, s_phase_tail;   /**< Head: rx writes. Tail: play writes. */
extern volatile int32_t s_phase_err_us;   /**< + = playing late, us. */
extern volatile bool s_phase_valid;       /**< Whether s_phase_err_us describes a real crossing. */

/** @brief Raw crossing history for the splice. Play-task only -- pushed in
 *         the crossing loop, read and reset in the splice, no lock. Why the
 *         splice cannot borrow the servo's filter: sync_phase_hist_t. */
extern sync_phase_hist_t s_phase_hist;

/** @brief The median of s_phase_hist, published by the play task for the
 *         servo: the servo runs on ring_monitor_task and sampling the median
 *         itself, at its own 5 s cadence, would filter at the wrong rate.
 *         Cleared -- not left stale -- when the history resets, and valid
 *         only after SYNC_PHASE_MIN readings. */
extern volatile int32_t s_phase_med_us;
extern volatile bool s_phase_med_valid;   /**< Whether the median is usable yet. */

/** @brief Set when a splice steps the phase, so the servo's shadow average
 *         forgets a history describing the situation before it. */
extern volatile bool s_phase_stepped;
extern volatile bool s_restart_pending;   /**< Flag the next audio packet as a track boundary. */

/** @brief Packets left before a post-hard-jump boundary is flagged, 0 none
 *         armed. The flag cannot fire on the jump itself -- every unit's
 *         phase still describes the timeline that just ended -- so it waits
 *         SYNC_PHASE_HIST packets for the splice's window to clear. Timeline
 *         path only. */
extern uint8_t s_jump_arm;

/** @brief Set by the play task on an underrun; the next packet through
 *         streamer_send_sbc() restarts the timeline, which also re-anchors
 *         every satellite. Without it the hub parks until the stream stops
 *         and restarts, discarding every incoming byte (fed-drop climbing to
 *         the full stream rate). */
extern volatile bool s_underrun_recover;
extern volatile int32_t s_restart_pos;    /**< Ring position the restart boundary lands at, -1 none. */

/** @brief Splice ceiling: a larger error is something a splice will not fix,
 *         and 150 ms is audible even at a track change. */
#define MAX_SPLICE_MS 150

/** @brief An insert may not push the ring this far below capacity -- the same
 *         50 ms that the catch-up drain treats as too deep to add to. While
 *         an insert's zeros play, receive keeps pushing; unclamped, 150 ms
 *         inserts into brimming rings push satellites to the ring ceiling
 *         and blocks get dropped at rx. The skip side needs no clamp: its
 *         discard loop reads with a zero timeout. */
#define SPLICE_INSERT_HEADROOM_MS 50

/** @brief The slew state: true while the timeline is being walked back to
 *         real time; s_slewing, and whether this episode was announced, and
 *         when it started for the 5 s persistence filter. */
extern bool s_slewing;
extern bool s_slew_told;      /**< Whether this episode has been announced. */
extern int64_t s_slew_since;  /**< When it started, for the 5 s filter. */

/** @brief True across a DAC retune. The play task parks on it:
 *         i2s_channel_write() returns immediately once the channel is
 *         disabled, so without this the play task spins through the ring at
 *         memory speed. Held across both retune_dac() callers. */
extern volatile bool retuning;

/**
 * @brief The instant local playback begins, and the handshake that publishes
 *        it.
 *
 * ONE WRITER: the timeline path, at a start. local_epoch is incremented
 * AFTER local_start is written; readers read the epoch BEFORE the value, so
 * a reader that sees a new epoch sees the value that belongs to it. Both are
 * volatile and this core keeps stores and loads in program order, and the
 * epoch is 32-bit so it cannot tear. Epoch wrap after 2^32 starts is
 * harmless -- the test is inequality against the last one seen, not
 * ordering.
 *
 * local_start is never zeroed to park; local_play_task parks on the epoch.
 * It therefore holds the last start instant for ever and cannot answer "is
 * anything playing" -- s_playing, owned by the play task, is that flag.
 */
extern volatile int64_t local_start;
/** @brief The handshake around local_start: incremented after the value is
 *         written, read before it; 32-bit so it cannot tear. */
extern volatile uint32_t local_epoch;

/** @brief Whether the play task is actually playing; written only by it. */
extern volatile bool s_playing;

extern volatile uint32_t s_feed_dropped;  /**< Bytes dropped because local_ring was full. */
extern volatile uint32_t s_tx_fail;       /**< sendto() rejections. */

/** @brief Audio datagrams handed to fan_out() this window. The rate is
 *         load-bearing -- TIMELINE_SLEW_US is per packet against ~50/s, and
 *         the TX pool is consumed per packet -- so it is printed, because a
 *         packetisation change moves both silently. Not sbc_in's `pkts`,
 *         which counts SPI frames from the bridge; the two split the moment
 *         packetisation stops being one-to-one. */
extern volatile uint32_t s_audio_pkts;

/** @brief sendto() failures split by lane, because how many and whose are
 *         different questions and only the second picks a fix. Ordered
 *         audible-first: a refused audio packet is a hole on every satellite
 *         at once, a refused frame is one repaint, a refused level is
 *         covered by the 1 Hz repeat. */
typedef enum {
    TX_LANE_AUDIO = 0,   /**< fan_out(): the only refusal the room can hear. */
    TX_LANE_FRAME,       /**< publish_frame(): analysis frames. */
    TX_LANE_VOL,         /**< streamer_send_vol(). */
    TX_LANE_META,        /**< streamer_send_meta(): track metadata. */
    TX_LANE_PROBE,       /**< probe_task(): time and TSF replies. */
    TX_LANE_FEC,         /**< send_fec_to_clients(): XOR parity for a group. */
    TX_LANE_N,
} tx_lane_t;

extern volatile uint32_t s_tx_lane_fail[TX_LANE_N];   /**< Per-lane refusal counts. */

/** @brief "In flight" grace for the refuse-near-frame instrument below:
 *         ~4 audio packets' worth of time. */
#define TX_NEAR_US              5000
/** @brief esp_timer stamp of the last frame fan-out, from publish_frame(). */
extern volatile int64_t  s_tx_frame_sent_us;
/** @brief Audio ENOMEMs within TX_NEAR_US of that stamp: non-zero near the
 *         window's audio refusals means the frame lane is still the
 *         competitor; near zero means the pool is being drained by something
 *         that is not us. The first reading that can tell those apart. */
extern volatile uint32_t n_refuse_near_frame;

/** @brief A refused audio packet gets one immediate second sendto(): the
 *         pool frees buffers as frames complete, on a timescale far below
 *         one audio period. Deliberately not a queue -- a deferred resend
 *         arrives after a newer packet and the satellite bins it as
 *         seq-drop. One attempt, ENOMEM only; any other errno is a real
 *         error and retrying hides it. Judge on the pair: retry-ok tracking
 *         retry means a transiently empty pool; retry-ok near zero means
 *         the retry should come out again. */
extern volatile uint32_t n_audio_retry;        /**< Audio ENOMEMs that got a second sendto. */
extern volatile uint32_t n_audio_retry_ok;     /**< Of which the second one went. */

/** @brief Stations arriving or leaving in this window, counted together:
 *         they come in pairs on a re-join and the question is whether the AP
 *         was doing association work. Read beside tx-fail -- refusals
 *         sharing a window with churn points at the join, refusals in
 *         churn-0 windows kill that suspicion. */
extern volatile uint32_t n_join_churn;

/** @brief Non-audio publish throttling, paired with TX_BACKOFF_US:
 *         tx_fail_note() raises s_tx_congested_until on ENOMEM, non-audio
 *         lanes skip past it and count the skip. fan_out() ignores it. A
 *         torn read of the 64-bit deadline costs one wrongly-sent or
 *         wrongly-skipped non-audio datagram. */
extern volatile uint32_t n_tx_cong_skip;            /**< Non-audio sends skipped under ENOMEM backoff. */
extern volatile int64_t s_tx_congested_until;       /**< esp_timer deadline; non-audio yields until it. */

/** @brief Frames dropped with a stranded batch: a stream that stopped before
 *         the next frame arrived to flush what was pending. A handful per
 *         track boundary and zero between; a standing rate means frames are
 *         being computed with due times already in the past, which is a
 *         timeline fault, not a transport one. Nothing is discarded for
 *         arriving early any more -- frames are batched. */
extern volatile uint32_t n_tx_pace_skip;

/** @brief The widest interval between two audio packets reaching the air
 *         this window. Timed on successful sends only, so a refused packet
 *         leaves the previous stamp standing and the next success measures
 *         the whole hole. Read beside the satellite's arrival gauges: this
 *         one spiking with the satellite's `gap max` means the packets left
 *         late; this flat while that spikes means they left on time. A
 *         gauge, cleared by the window that prints it. */
extern volatile int32_t n_fanout_gap_max_us;

/** @brief Sentinel for "no packet was stamped this window"; distinct from a
 *         measured zero and must not print as one. */
#define LEAD_UNSEEN INT32_MAX

/** @brief Beyond this a reading is not a lead and is refused rather than
 *         clamped. The satellite's LEAD_INSANE_US uses the same 1 s, because
 *         the two ends' lead gauges subtract to a transit time and must
 *         agree on what they are subtracting. */
#define LEAD_INSANE_US 1000000
/** @brief The least lead any packet carried when STAMPED this window. The
 *         satellite reports the same quantity at ARRIVAL, so
 *         transit ~= this window's lead-min minus the satellite's lead-min.
 *         This flat while the satellite's collapses means the packets left
 *         on time and were held after sendto(); both collapsing together
 *         means they were stamped late. A gauge, cleared by the window that
 *         prints it. */
extern volatile int32_t n_lead_min_us;

/**
 * @brief Route every failed sendto() through this, with errno, at the
 *        failure -- so the reason is kept beside the count.
 * @param lane  Which lane was refused.
 * @param err   The errno sendto() returned.
 */
void tx_fail_note(tx_lane_t lane, int err);
/** @brief The audio downlink's entry point, kept as a name for the two send
 *         paths in timeline.c. */
/** @brief The audio downlink's entry point, kept as a name for the two send
 *         paths in timeline.c.
 *  @param err  The errno sendto() returned. */
void tx_fail_note_audio(int err);
/** @brief Render the errno tally for the status line and clear it. */
/** @brief Render the errno tally for the status line and clear it.
 *  @param buf  Output buffer.
 *  @param len  Its size. */
void tx_fail_summary(char *buf, size_t len);
/** @brief Render the per-lane breakdown; rendered and cleared with the errno
 *         tally so the two cannot describe different windows. */
/** @brief Render the per-lane breakdown; rendered and cleared with the errno
 *         tally so the two cannot describe different windows.
 *  @param buf  Output buffer.
 *  @param len  Its size. */
void tx_fail_lanes(char *buf, size_t len);
/** @brief Render the shape of an ENOMEM storm -- beacon-spaced clusters
 *         against one unbroken stall; empty string when nothing was
 *         refused. */
/** @brief Render the shape of an ENOMEM storm -- beacon-spaced clusters
 *         against one unbroken stall; empty string when nothing was refused.
 *  @param buf  Output buffer.
 *  @param len  Its size. */
void tx_burst_summary(char *buf, size_t len);
/** @brief The air's own reading, from tx_done_cb() in net.c, which survives
 *         a cache queue in front of the driver when every other stall signal
 *         does not. Always writes a non-empty string. */
/** @brief The air's own reading, from tx_done_cb() in net.c, which survives
 *         a cache queue in front of the driver when every other stall signal
 *         does not. Always writes a non-empty string.
 *  @param buf  Output buffer.
 *  @param len  Its size. */
void tx_air_summary(char *buf, size_t len);
/** @brief An audio packet got through, so the pool freed a buffer: close any
 *         open refusal stretch. Only audio may close one -- see net.c. */
void tx_send_ok(void);
/** @brief DMA-starvation count; the HEALTH line's window on the output path. */
uint32_t dma_starve_count(void);

/** @brief Cumulative totals for a long run, never reset -- a rate says what
 *         is happening now, a total says whether something has been
 *         happening slowly for an hour. */
extern volatile uint32_t n_underruns;    /**< Local playback ran dry. */
extern volatile uint32_t n_restarts;     /**< Timeline restarted. */
extern volatile uint32_t n_splices;      /**< Track-boundary corrections applied. */
extern volatile uint32_t n_retunes;      /**< DAC clock changes that succeeded. */
extern volatile uint32_t n_retunes_bad;  /**< Retunes refused or failed. */
extern volatile uint32_t n_sta_left;     /**< Satellites disassociating. */
extern volatile uint32_t hw_play;        /**< Play-task stack headroom, sampled in-task. */
extern volatile uint32_t hw_mon;         /**< Monitor-task stack headroom. */

/** @brief Heap low-water for the window, with a timestamp because the
 *         since-boot watermark cannot say when a dip landed or what it
 *         shared a window with. Sampled every 5 s, cleared by each HEALTH
 *         line. */
extern volatile uint32_t heap_min_window;

/** @brief The pool that actually says yes or no to a WiFi buffer. With
 *         SPIRAM_USE_CAPS_ALLOC, ordinary malloc never returns PSRAM, so
 *         the ring, DMA buffers, frame queue, WiFi buffers and every stack
 *         live in internal SRAM -- and the whole-heap figure cannot see
 *         that pool run out. 8BIT is in the mask because INTERNAL alone
 *         also matches 32-bit-only regions nothing byte-addressable can
 *         use; it is also exactly the mask (caps 0x804) of the requests
 *         that fail. */
#define CAP_USABLE_INTERNAL (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)

/** @brief Internal-SRAM low-water for the window; same discipline as
 *         heap_min_window. No since-boot twin is needed:
 *         heap_caps_get_minimum_free_size() already keeps one. */
extern volatile uint32_t heap_int_window;

/** @brief Allocation failures, counted and reported rather than aborted:
 *         CONFIG_HEAP_ABORT_WHEN_ALLOCATION_FAILS is too violent for a
 *         dance floor, and silent failure surfaces as an underrun, which
 *         names the symptom. The hook records and does not log; telemetry
 *         notices within 5 s from a context where logging is safe. */
extern volatile uint32_t n_alloc_fail;
extern volatile uint32_t alloc_fail_size;   /**< The largest request that failed. */
extern volatile uint32_t alloc_fail_caps;   /**< The caps of that request. */

/** @brief Tasks that did not start. A unit that loses a task runs on
 *         without it and looks like one that has gone quiet; task_start()
 *         counts the failure, names the task, and says so. */
extern volatile uint32_t n_task_fail;
extern char s_task_fail_names[64];          /**< Names of the tasks that failed. */

/** @brief Phase points dropped because the queue was full. A wedged phase
 *         queue is not a degradation but a stop: the servo runs on nothing
 *         but points coming off this queue. */
extern volatile uint32_t n_phase_drop;

/** @brief Refill instrument: phase readings withheld because the DMA was
 *         still filling at memory speed (the play task was parked, the
 *         channel drained, early writes return without blocking). Non-zero
 *         at every playback start is the guard working; climbing
 *         mid-stream is an underrun by another name. */
extern volatile uint32_t n_refill_withheld;
/** @brief Ring reads that came back short of a chunk, and the frames of
 *         silence padded to cover them. The pad is played but was never in
 *         the ring while samples_played advances by a whole chunk, so any
 *         firing permanently displaces every later phase point. */
extern volatile uint32_t n_short_reads;
extern volatile uint32_t n_short_frames;  /**< Silence frames padded over them. */

/** @brief Frames the fine rate trim has dropped from and duplicated into
 *         the stream -- the instrument for rate_trim_hz, because a
 *         correction you cannot see is a correction you cannot attribute.
 *         The counts track |rate_trim_hz| by construction; flat while the
 *         trim is non-zero means the trim is not running, both climbing
 *         together means the servo is hunting. */
extern volatile uint32_t n_trim_drops;
extern volatile uint32_t n_trim_dups;   /**< Frames duplicated for the same. */

/** @brief The faded catch-up, audio_shift.h's debt and its counters. The
 *         servo (ring_monitor_task) arms the debt; playback is the only
 *         thing that shrinks it, one chunk's shift at a time as the
 *         crossfade spends it. The knock being paid usually arrives on both
 *         units at once, and paying it at the same rate from the same code
 *         is what keeps them from diverging while it is paid. */
extern volatile int32_t  catchup_frames;   /**< Signed frames; positive = skip, running late. */
extern volatile uint32_t n_catchup_drops;  /**< Frames the drain dropped. */
extern volatile uint32_t n_catchup_dups;   /**< Frames the drain duplicated. */

/** @brief Below this a DMA write did not block -- the channel was still
 *         draining. MEASUREMENT ONLY: nothing is withheld; this sizes the
 *         window a guard would need. i2s_channel_write() does not block
 *         while descriptors are free, so on an empty channel the first
 *         writes return at memory speed and phase readings dated inside
 *         that window are measured against a reference the DAC is not
 *         pacing. */
#define REFILL_FAST_US 1000
extern bool s_refill_active;     /**< True while the DMA is filling unblocked. */
extern int32_t s_refill_frames;  /**< Frames written during that window. */

/** @brief How many satellites this hub will carry. 15 is the radio's ceiling
 *         (ESP_WIFI_MAX_CONN_NUM), and three limits must agree -- this,
 *         wifi_config.ap.max_connection in net.c, and
 *         CONFIG_LWIP_DHCPS_MAX_STATION_NUM: a satellite refused by the
 *         third associates and then has no address, which reports as a unit
 *         that joined and never probed. The number says what the design
 *         intends to carry, not what has been exercised. */
#define MAX_CLIENTS 15

/** @brief How long a satellite that stops probing stays on the send list:
 *         8 probes at the satellite's rate. Shorter risks an underrun on a
 *         wrongly forgotten satellite, which gets no audio until its next
 *         probe re-registers it; longer holds TX buffers for a station that
 *         is gone -- each send takes a DMA buffer the driver will not free
 *         until it gives up. A clean disassociation is instant through the
 *         event handler; this bound is for the satellite that vanishes
 *         without saying so. 3-5 alloc-fails per mid-track disconnect is
 *         the floor -- buffers already handed to the driver -- and is this
 *         working, not regressing. */
#define CLIENT_TIMEOUT_US 2000000

/** @brief The volume repeat. VOL_REPEAT_US is the standing repeat's period
 *         and, under the silent-until-told rule, the worst-case silence a
 *         joining satellite sits through when the join push is lost.
 *         VOL_CHANGE_REPEATS covers a change being lost with the link
 *         otherwise healthy: group frames are not retried, so a level that
 *         must arrive is sent three times rather than trusted once. */
#define VOL_REPEAT_US     1000000
#define VOL_CHANGE_REPEATS 3   /**< A change is sent this many times, not trusted once. */

/** @brief One registered satellite: where to send, and when it last gave
 *        sign of listening. A zeroed slot is how "not listening" is
 *        spelled -- callers test last_seen after clients_snapshot(). */
typedef struct {
    struct sockaddr_in addr;    /**< Unicast destination. */
    int64_t last_seen;          /**< esp_timer stamp of the last probe, 0 = empty slot. */
} client_t;

extern client_t s_clients[MAX_CLIENTS];  /**< The send list; guard with s_clients_lock. */
extern portMUX_TYPE s_clients_lock;      /**< Spinlock over s_clients. */
extern esp_netif_t *s_ap_netif;          /**< For the MAC -> IP lookup on disassociation. */
extern volatile uint32_t n_sta_dropped;  /**< Satellites dropped on the disassociation event. */
extern volatile uint32_t n_sta_nolease;  /**< ...and the times the lookup could not say who. */

/** @brief Satellites the timeout got, which raises no event: a unit that
 *         loses power or leaves range is dropped here, silently. Counted so
 *         a HEALTH line can tell a handled departure from an unseen one. */
extern volatile uint32_t n_sta_timeout;

/** @brief SBC the SPI link delivered but the WiFi ceiling refused. Should be
 *         0 while AUDIO_MAX_PAYLOAD tracks SBC_LINK_MAX_PAYLOAD; a ceiling
 *         drift here would otherwise read as satellite-side gaps. */
extern uint32_t n_wifi_oversize;

/** @brief XOR parity from the sending end, per window. n_fec_sent should be
 *         the audio rate over K; at zero with audio flowing the scheme is
 *         not running. n_fec_cong_skip is parity WITHHELD under ENOMEM
 *         backoff -- redundancy standing down so it does not displace the
 *         audio it protects; read it beside tx-fail. n_fec_skipped is a
 *         group that could not produce parity at all (oversize payload,
 *         non-contiguous seq); flat at zero on this hardware, any movement
 *         is a wire-format question. */
extern volatile uint32_t n_fec_sent;
extern volatile uint32_t n_fec_skipped;    /**< Groups that could not produce parity at all. */
extern volatile uint32_t n_fec_cong_skip;  /**< Parity withheld under ENOMEM backoff. */

/** @brief Where the next packet's audio starts and when it is due, so the
 *         analysis -- fed on arrival, before the stamp exists -- can date
 *         what it is given. Only meaningful as a pair referring to the same
 *         instant in the stream. */
extern volatile int32_t s_vis_anchor_pos;
extern volatile int64_t s_vis_anchor_due;   /**< Master-clock instant it is due, us. */

/** @brief Ring position the audio being fed RIGHT NOW will start at,
 *         captured before the feed because play_at is not known until
 *         streamer_send_sbc() computes it, by which point s_samples_in has
 *         advanced. */
extern int32_t s_pending_pos;

/** @brief Last cross-unit measurement, kept for the per-track summary: a
 *         track boundary nulls phase on every unit, so the reading just
 *         BEFORE a boundary is how far the speakers had drifted over a
 *         whole track -- the one sample worth comparing sessions on. */
extern volatile int64_t s_sync_err_us;
extern volatile int64_t s_sync_at;         /**< 0 = never measured. */

/** @brief This unit's own last boundary correction, for satellites to be
 *         compared against when they report theirs. */
extern volatile int32_t s_hub_splice_us;
extern volatile int64_t s_hub_splice_at;   /**< 0 = no boundary yet. */

/** @brief SHADOW, acted on by nothing: what the boundary correction would
 *         have been had the splice used sync_phase_median() instead of the
 *         single most recent reading. Reported beside the real figure at
 *         the same boundary -- the only place the two are comparable. */
extern volatile int32_t s_hub_splice_alt_us;

/** @brief What could conceivably be an audio sample rate at all; anything
 *         outside is a broken calculation, whoever asked for it. The bound
 *         retune_dac() refuses on. */
#define RATE_SANE_MIN 8000
#define RATE_SANE_MAX 192000   /**< The upper sane bound; the same ceiling retune_dac() refuses on. */

/** @brief The fine rate correction in Hz, written by the servo and read by
 *         playback. The DAC clock stays put; playback consumes the ring at
 *         an effective (tx_rate + rate_trim_hz) by dropping a frame to get
 *         through the stream faster and duplicating one to get through it
 *         slower. Positive means playing late, so consume faster. Whole Hz
 *         because that is what the servo computes; the software path has no
 *         clock-quantisation floor, so the deadband can tighten without
 *         touching this. Persists across a playback restart; zeroed only
 *         when a coarse retune moves the clock, because the clock then
 *         carries what this was carrying. RATE_TRIM_MAX_HZ lives in
 *         audio_shift.h so both units agree on the boundary between this
 *         and a retune. */
extern volatile int32_t rate_trim_hz;

/** @brief Playback volume, 0-127, AUDIO_VOL_MAX being unity. Meaningless
 *         until audio_vol_known. Applied at the DAC write only: what goes
 *         to the satellites stays full scale and each unit attenuates its
 *         own output, so the air's dynamic range is not spent on a level
 *         decision. A torn read is impossible on a byte; a stale one costs
 *         one chunk at the previous level. */
extern volatile uint8_t audio_volume;

/** @brief Whether the bridge has ever said how loud, this boot. Sticky.
 *         streamer_send_vol() is gated on it: a hub must not relay a level
 *         it invented, or a hub whose bridge died would blast satellites
 *         sitting correctly at -50 dB. */
extern volatile bool audio_vol_known;

/** @brief MSG_VOL datagrams sent, repeats included -- counted before any
 *         change test, because the fault it exposes is a unit playing at a
 *         level nobody told it, and that reads as a satellite whose vol-rx
 *         sits still while this climbs. */
extern volatile uint32_t n_vol_tx;

/** @brief What a retune costs, on the coarse-only path the servo reaches
 *         when the correction exceeds RATE_TRIM_MAX_HZ. The channel-down
 *         time is measured here; the discarded DMA buffer is not measurable
 *         anywhere, because those frames counted as played. */
extern volatile int32_t s_retune_phase_before;  /**< Phase reading just before the retune. */
extern volatile bool    s_retune_watch;         /**< Whether that reading is being watched. */
extern volatile int64_t s_retune_outage_us;     /**< Channel-down time, us. */

/** @brief When the retune finished, and how many crossings have been
 *         narrated since -- MEASUREMENT ONLY, sizing whether the servo's
 *         single withheld crossing covers the transient. Crossings arrive
 *         one per packet, ~20 ms apart, and the transient lands 1-22 ms
 *         after the retune, so a one-shot flag may be covering the first
 *         third of a disturbance and handing the rest to the servo as
 *         position error. */
extern volatile int64_t s_retune_done_at;
extern volatile uint8_t s_retune_tail_left;     /**< Crossings left to narrate, 0 none. */

/** @brief Bring up the I2S TX channel and its DMA (out.c). */
void i2s_start(uint32_t rate);
/** @brief Move the DAC clock, parking playback for the switch (out.c). */
void retune_dac(uint32_t hz);

/** @brief Bring up the SoftAP (net.c). */
void wifi_start_ap(void);
/** @brief Bring up the sync socket (net.c). */
void socket_start(void);

/** @brief Register or refresh a satellite on the send list; called from
 *         probe.c, since probing implies listening. */
/** @brief Register or refresh a satellite on the send list; called from
 *         probe.c, since probing implies listening.
 *  @param from  The probe's source address. */
void client_seen(const struct sockaddr_in *from);
/** @brief A satellite associated: called from net.c's WiFi event handler. */
/** @brief A satellite associated: called from net.c's WiFi event handler.
 *  @param mac  The satellite's MAC, as the event carries it.
 *  @param ip   The address the DHCP server gave it. */
void client_joined(const uint8_t mac[6], const esp_ip4_addr_t *ip);
/** @brief A satellite disassociated: called from net.c's event handler. */
/** @brief A satellite disassociated: called from net.c's event handler.
 *  @param mac  The satellite's MAC. */
void client_gone(const uint8_t mac[6]);

/** @brief Copy the send list under the spinlock; returns MAX_CLIENTS slots,
 *         caller tests last_seen (a zeroed slot is "not listening"). Only
 *         the copy is shared -- message building stays per-caller, because
 *         each lane is on its own cadence and likely to grow its own rate
 *         limit. */
/** @brief Copy the send list under the spinlock; returns MAX_CLIENTS slots,
 *         caller tests last_seen (a zeroed slot is "not listening"). Only
 *         the copy is shared -- message building stays per-caller, because
 *         each lane is on its own cadence and likely to grow its own rate
 *         limit.
 *  @param dst  Destination array, MAX_CLIENTS slots. */
void clients_snapshot(client_t *dst);

/** @brief Drop satellites past CLIENT_TIMEOUT_US. Runs before each audio
 *         send and on the 5 s tick, so clients age out while audio is
 *         stopped too. */
/** @brief Drop satellites past CLIENT_TIMEOUT_US. Runs before each audio
 *         send and on the 5 s tick, so clients age out while audio is
 *         stopped too.
 *  @param now  Current esp_timer reading, us. */
void clients_age(int64_t now);

#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
/** @brief Batch analysis frames and send them to every listener, paced by
 *         TX_FRAME_PACE_US (clients.c). */
/** @brief Batch analysis frames and send them to every listener, paced by
 *         TX_FRAME_PACE_US (clients.c).
 *  @param f  The frame the analysis lane produced. */
void publish_frame(const vis_frame_t *f);
#endif

/** @brief The playback task (play.c). */
/** @brief The playback task (play.c).
 *  @param arg  Unused; the FreeRTOS task signature. */
void local_play_task(void *arg);
/** @brief The time-probe server task (probe.c). */
/** @brief The time-probe server task (probe.c).
 *  @param arg  Unused; the FreeRTOS task signature. */
void probe_task(void *arg);
/** @brief The 5 s monitor task: telemetry_tick() then servo_tick(), in that
 *         order, because the counters matter most when audio has stopped. */
/** @brief The 5 s monitor task: telemetry_tick() then servo_tick(), in that
 *         order, because the counters matter most when audio has stopped.
 *  @param arg  Unused; the FreeRTOS task signature. */
void ring_monitor_task(void *arg);

/** @brief Report the window: print and clear the counters and gauges. */
void telemetry_tick(void);
/** @brief Run the ring servo for one window. */
void servo_tick(void);
/** @brief Install the allocation-failure hook (records, does not log). */
void telemetry_register_alloc_hook(void);

#if CONFIG_DANCEFLOOR_ENABLE_MARKER
/** @brief Start the cross-unit sync marker instrument (marker.c). */
void marker_start(void);
#endif
