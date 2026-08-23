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
 *                      s_marker_at, n_underruns, n_splices, n_short_*,
 *                      n_trim_drops / n_trim_dups, hw_play and s_playing. It
 *                      no longer writes local_start. It READS rate_trim_hz.
 *   ring_monitor_task  writes the servo's own state, rate_trim_hz, the retune_*
 *                      set and the windowed heap figures, and READS everything
 *                      else in order to report it. retune_dac() runs on this
 *                      task.
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
#include "audio_shift.h"
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
 * BACK TO 250 ms, and this time with a reason the lead itself serves. It was cut
 * to 200 so LEAD + RESYNC stayed inside the satellite's ring, and the note here
 * said the room had returned -- that ring is 80 kB / 464 ms -- "but do not spend
 * it on the lead without a reason the lead itself can serve". There are two now:
 *
 *   - THE SATELLITE RUNS MODELS. The S3 satellite runs the analyser lane, and
 *     every frame names the instant it is drawn at. The lead is the whole budget
 *     between a frame existing and being due, so it is what an inference has to
 *     finish inside. At 200 ms that budget is shared with delivery; at 250 it is
 *     not as tight.
 *
 *     THIS BULLET NO LONGER ARGUES FROM THIS LANE, and the difference does not
 *     change the number. It used to read "takes MSG_FRAME and runs the analyser
 *     lane on the spectrum it carries" -- that unit is LED_SOURCE_LOCAL now and
 *     computes its own spectrum, because spec[] came off the wire (see
 *     vis_frame_t). So the inference budget is no longer bounded by DELIVERY at
 *     all; it is bounded by the lead between analysis and render, which is the
 *     same lead and the same figure. The second bullet is what actually holds
 *     this constant up regardless.
 *   - ANCHORS ARE BEING REFUSED. 251 in three hours. A satellite will not anchor
 *     on a packet with less than ANCHOR_MIN_LEAD_US in front of it, RESYNC_US
 *     lets this timeline wander 150 ms below target, and the measured mean lead
 *     was 146 ms -- so a healthy packet in a trough shows under 100 ms and is
 *     refused, exactly as the note on ANCHOR_MIN_LEAD_US predicts. Raising the
 *     centre lifts the whole distribution off that floor.
 *
 * NOW 350 ms, and the "ANCHORS ARE BEING REFUSED" bullet above turned out to be
 * the whole fault rather than one symptom of it. Raising the centre from 200 to
 * 250 lifted the distribution but left the MECHANISM in place, and the mechanism
 * is RESYNC_US: see the note below it, which is the change that matters. This
 * line widens the base margin on top.
 *
 * WHAT THE MARGIN HAS TO COVER, measured this session rather than assumed:
 *
 *   satellite anchor minimum   125 ms   ANCHOR_MIN_LEAD_US (satellite/main/sat.h)
 *   worst-case DTIM hold       102 ms   beacon read-back in net.c: 100 TU
 *   worst measured transit     153 ms   2026-08-20 soak, one packet, tx-fail 0
 *
 * The third is the one that decides it. A group frame that misses its beacon
 * window waits a whole DTIM period, and two in a row is 205 ms -- so a floor
 * anywhere near 100 ms is a floor below which ordinary, loss-free delivery puts
 * packets past their play_at. With RESYNC_US at 70 the floor is 280 ms, which
 * clears all three.
 *
 * THE PRICE, AND IT IS ON THIS UNIT'S OWN RING. LOCAL_RING_BYTES is 80 kB and
 * holds the lead before any feed burst arrives: at 250 ms that left 214 ms of
 * headroom, at 350 it leaves 114 ms (20,180 bytes). The worst `max gap` this
 * unit has recorded is ~95 ms, so it still clears -- by 19 ms rather than by
 * 119. It CANNOT be bought back by growing the ring: xStreamBufferCreate takes
 * the default heap, SPIRAM_USE_CAPS_ALLOC means plain malloc never returns
 * PSRAM, and this unit runs at `internal 12868 free`. So the hub ring stays at
 * 80 kB and this is the constant to walk back if `refill-withheld`,
 * `short-reads` or a hub underrun ever appears on the HEALTH line. 300 ms would
 * restore 164 ms of headroom and still give a 230 ms floor.
 *
 * Three things move with this and must stay in step -- RING_TARGET_MS on the
 * satellite (which forces CONFIG_DANCEFLOOR_RING_KB to 96, see the note there),
 * ANCHOR_MIN_LEAD_US beside it, and LOCAL_RING_BYTES below.
 *
 * What it does NOT buy is tolerance for the transmit path refusing sends
 * outright. No lead recovers a packet that was never transmitted -- but note
 * that the 2026-08-20 soak showed the refusals FOLLOWING the starvation by ~30
 * seconds, not causing it, so that caveat is smaller than it used to read.
 *
 * The ceiling is the SATELLITE's ring, a classic ESP32 with no PSRAM: 96 kB is
 * 557 ms and holds 350 + the depth net's 120 ms swing with 87 ms to spare. A
 * 500 ms lead would need ~107 kB against a ~106 kB largest free block, so that
 * is still the wall.
 */
#define LEAD_US   350000

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
 * BACK DOWN TO 70, AND THIS IS THE FAULT FIX. Raising it past the swing was the
 * wrong lesson from the right measurement, and it cost three soaks to see why.
 *
 * `err` is `next_play_at - target` and `target` is `now + LEAD_US`, so err IS
 * `lead - LEAD_US` -- nothing else. A deadband on err is therefore a licence for
 * the lead to sit anywhere in `LEAD_US +- RESYNC_US`, and at 150 that meant
 * NOTHING CORRECTED THE LEAD UNTIL IT HAD FALLEN TO 100 ms. Below three
 * separately measured numbers: the satellite refuses to anchor under
 * ANCHOR_MIN_LEAD_US (125 ms), a group frame can be held a whole DTIM period
 * (102 ms, read back in net.c), and the worst transit measured on a loss-free
 * packet was 153 ms. The 2026-08-20 soak sat in exactly that trap -- the
 * sawtooth troughs read 96, 99 and 99 ms, the threshold itself, and each one
 * that met a delayed packet emptied both satellite rings and ended in a
 * re-anchor storm. `anchors refused 90 late` is that, stated by the satellite.
 *
 * At 70 the floor is 280 ms and all three are cleared.
 *
 * YES, IT NOW TRIPS ON NORMAL DELIVERY, and that is the point rather than a
 * cost. The -132 ms trough described above is bigger than 70, so the slew is
 * active much of the time -- but err OSCILLATES around its mean, so the nudges
 * above and below cancel and what survives is a correction proportional to the
 * MEAN offset. That is exactly the controller this needs, and a 150 ms deadband
 * gave it nothing to work with until the mean was already 100 ms wrong.
 *
 * The reason 150 looked necessary was the log line arriving seven times a
 * minute. That is fixed where it belongs: steer_timeline() only reports after
 * 5 s of PERSISTENT slewing, so an oscillation that recovers within a second
 * or two says nothing at all. Tuning a control deadband to quieten a log was
 * the mistake worth remembering here.
 *
 * The rate is unchanged and still bounded by what the servo can follow:
 * 20 us/packet at ~50 packets/s is 1 ms/s against the +-100 Hz (2.27 ms/s) a
 * unit can trim, so being in the slew more often costs nothing downstream.
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
 * LEAD + RESYNC must fit the satellite ring: now 350 + 70 = 420 ms against the
 * 557 ms a 96 kB ring holds. (That arithmetic read "200 + 150 = 350 against
 * 464" for a while after LEAD_US had already moved to 250 -- worth a glance
 * whenever either constant changes, since nothing checks it.)
 */
#define RESYNC_US  70000

/*
 * Past this the timeline is not merely off, it is wrong, and no gradual
 * correction closes it in useful time. Jump instead.
 *
 * 300 ms, down from 1 s, and the number that matters is LEAD_US rather than
 * anything about the slew.
 *
 * A satellite anchors on a packet whose play_at is still ahead of it, and the
 * lead it sees is LEAD_US plus this error. The arithmetic below was written when
 * LEAD_US was 250 and ANCHOR_MIN_LEAD_US read 100; at LEAD_US 350 the same two
 * landmarks sit at err = -225 ms (lead down to the 125 ms a satellite insists on
 * before it will anchor at all) and roughly -300 ms (packets arriving with their
 * play_at already gone). Both are now BELOW this threshold rather than above it,
 * which is the right way round: the jump is reached only once anchoring is
 * already impossible. RESYNC_US, not this, is what keeps the lead off that floor
 * in normal running -- see the note there.
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
 * A hard jump is refused while the local ring is under this, since 2026-08-18.
 *
 * Normal delivery jitter swings the ring +-40 ms around the 250 ms lead, so
 * 150 ms is 100 below target -- two and a half times the deepest normal
 * trough. Below it, delivery is starving, and a starved ring makes `err` run
 * past RESYNC_HARD_US with nothing wrong with the clock: next_play_at stops
 * advancing while target keeps moving with `now`, so the error IS the
 * starvation, measured twice -- the same fact the RESYNC_HARD_US note above
 * caught from the other side ("a starved ring and a late phase are the same
 * fact seen twice").
 *
 * Jumping anyway re-stamps every packet with an error the burst will unwind,
 * and each jump arms a post-jump boundary: on the 2026-08-18 soak the source
 * wandered +-10% for a minute (sbc_in eff 40106..48743 Hz), the ring sat at
 * 27 ms, err swung +-310 ms across this threshold five times in 45 s, and
 * every unit spliced four capped 150 ms inserts mid-track -- audible holes,
 * then replay drains on top, then rings ballooned to the ceiling. Held
 * instead, the same excursion recovered by itself: +241427 us slewed back
 * within 13 ms of the resync threshold with no help from anyone.
 *
 * TIMELINE_HOLD_GIVE_UP_US bounds a hold that never ends (a wedged state in
 * which new satellites cannot anchor on a play_at stuck in the past). The
 * primary escape is cleaner and already exists: a ring starved all the way
 * to empty ends in CHUNK_UNDERRUN -> s_underrun_recover -> start_timeline(),
 * which restarts the timeline wholesale with every satellite re-anchoring,
 * rather than stepping the one that exists.
 */
#define TIMELINE_HOLD_STARVE_MS   150
#define TIMELINE_HOLD_GIVE_UP_US  30000000

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
 * STEADY is how long it must go without one. 500 ms is exactly twice LEAD_US
 * since that became 250 ms -- it was "more than twice" at 200 -- so a source
 * that manages it can still fill the ring before playback reaches it, with the
 * margin now equal to the lead rather than half again. If LEAD_US grows any
 * further this is the next constant to move, and the test is whether a stream
 * still starts with a full ring rather than filling one behind playback.
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
 * WHAT A REAL SOURCE STALL LOOKS LIKE, measured, so the next person reading a
 * multi-second `max gap` on the sbc_in line does not go looking for a fault in
 * the floor.
 *
 * 2026-08-20 19:15 soak, at a track boundary (TRACK #2 -> #3):
 *
 *   sbc_in: pkts 177 | eff 31160 Hz | fed-drop 16384 B | max gap 1443818 us
 *
 * 1.44 SECONDS with nothing arriving from the phone, across a track change.
 * That is four times LEAD_US and nothing in this system can absorb it: the hub
 * ran its own ring dry, took `local underrun, restarting timeline`, and both
 * satellites re-anchored off the fresh timeline. It cost 470 ms of starvation
 * on each and was over inside one 5 s window -- `lead-min +318 ms, starved 0`
 * in the very next one, with anchors-refused still 0 for the whole session.
 *
 * So the recovery path handled it exactly as designed, and the number to check
 * before suspecting anything here is sbc_in's `max gap`. It was the ONLY audible
 * event in 53 minutes of that run, and it was not ours.
 */

/*
 * How far the timeline moves per packet while slewing back to real time.
 *
 * At ~50 packets/s this is 1 ms/s. The bound that matters is the servo's: it
 * trims at most +-100 Hz, which is 2.27 ms/s at 44.1 kHz, so a slew near that
 * outruns the units it is supposed to be leading.
 */
#define TIMELINE_SLEW_US 20

/*
 * Throttle on the non-audio downlink. Audio is the priority: a dropped audio
 * datagram costs a gap, where a dropped frame costs only a slightly-stale LED.
 * publish_frame runs at the ~86 Hz analysis rate (Kconfig.projbuild), on top of
 * ~50 audio, and overflowed the 26 static TX buffers this was written against
 * (tx-fail ENOMEM, measured -- see the errno tally tx_fail_note() keeps).
 *
 * There was a second lane here, publish_ml, capped by its own
 * ML_PUBLISH_PERIOD_US at 10/s against a possible 86. It is gone: analysers read
 * a spectrum, and a unit that wants one computes it, so results are computed
 * where they are needed rather than shipped. That halves what this backoff is
 * protecting. (The spectrum itself stopped being shipped too, later and for a
 * different reason -- see vis_frame_t -- which is what makes "computes it" a
 * build rule rather than a preference.)
 *
 * TX_BACKOFF_US is how long the non-audio lane stays silent after ANY sendto()
 * returns ENOMEM: the instant the pool is exhausted, non-audio yields, leaving
 * the buffers audio was being refused for audio. fan_out() -- the audio path --
 * is never gated.
 *
 * Falsified by tx-fail on the servo line: throttling is working while that drops.
 */
#define TX_BACKOFF_US           40000   /* ~2 audio packets: yield, then probe again */

/*
 * The frame lane's PROACTIVE pace, beside TX_BACKOFF_US's reactive one: the
 * minimum spacing between two frame sends, so the burst never forms in the
 * first place.
 *
 * THE FAULT THIS IS FOR, from the 2026-08-17 soak. The analysis lane and the
 * audio lane share one static TX pool, and the analysis task does not run at
 * a steady 86/s: whenever the decoder hands it a lump -- a source stall
 * unwinding, an A2DP catch-up -- it publishes back-to-back frames, and a burst
 * of them crowds the pool badly enough that the AUDIO's sendto comes back
 * ENOMEM. Measured as exactly the audible glitch moments: `tx-fail 22 (19
 * audio) -- Not enough space 22 | cong-skip 38`, with both satellites
 * gap-filling +40..+150 ms in the same second. TX_BACKOFF_US fires only after
 * a send has already FAILED -- it protects the pool's recovery, not the audio
 * that was refused while the burst was still going in.
 *
 * 3/4 of the frame period. In steady state the analysis already produces one
 * frame per period, so this changes nothing at all -- a paced lane and an
 * unpaced one are the same lane when the producer is on cadence. It binds
 * only during a burst, where it caps the lane at 4/3 of its normal rate and
 * staggers the frames a quarter-period apart (~2.9 ms at the 512 hop): the
 * pool keeps a frame's worth of headroom, and the cost to the floor is a few
 * frames skipped out of 86/s -- the same loss a failed send already was, and
 * the visualiser is built to take it (frames are snapshots, not a protocol).
 *
 * ALL OF THE ABOVE IS THE RECORD OF A PACE THAT NO LONGER APPLIES, and it is
 * kept because its reasoning was sound and its PREMISE was not. It assumed the
 * lane's constraint is the rate the analysis produces at -- one frame per hop,
 * 86/s at hop 512 -- so it derived the pace from the hop. The constraint is
 * actually the rate the radio RELEASES group frames at, which is the DTIM
 * beacon and nothing to do with the analysis. See TX_FRAME_PACE_US below.
 *
 * The hop rates it derived from, kept for the arithmetic elsewhere in this
 * file: at 44.1 kHz, hop 1024 -> 23220 us, 512 -> 11610, 256 -> 5805. Not a
 * macro any more, because an unused TX_FRAME_* constant sitting beside the pace
 * reads as though the pace still comes from it.
 */
/*
 * THE DTIM HOLD, and the pace that is actually derived from it.
 *
 * 100 TU x 1024 us. A SoftAP releases group-addressed frames only after a DTIM
 * beacon, so this is how long a group frame occupies a static TX buffer in the
 * worst case -- and it is a FIXED cost: 100 TU is the bottom of IDF's
 * documented range and dtim_period is already 1. net.c carries the measurement
 * that proved the field had never held anything else.
 */
#define DTIM_HOLD_US 102400

/*
 * One analysis frame per beacon, not three per four analysis periods.
 *
 * THE OLD PACE WAS DERIVED FROM THE WRONG CLOCK. (TX_FRAME_PERIOD_US * 3) / 4
 * is ~8.7 ms at hop 512, which permits ~115 frames/s into a queue that opens
 * 9.8 times a second. The extra 105 do not go out sooner; they sit in the pool
 * holding buffers audio also needs, and the 2026-08-20 soak measured what that
 * costs -- `cong-skip` in the hundreds per window right through the join
 * transient, which is the frame lane discovering by failure what the beacon
 * rate could have told it in advance.
 *
 * Pacing to the beacon makes the group burst DETERMINISTIC: audio's ~5 packets
 * plus exactly one frame, every time, instead of audio's 5 plus however many
 * frames the analysis task happened to emit since the last release.
 *
 * WHAT IT COSTS IS SMALLER THAN IT LOOKS -- AND THAT WAS WRONG. Kept because
 * the mistake is instructive. The paragraph read: the satellites were already
 * only receiving 26.6 frames/s (88,237 frames over 3,312 s, counted by the
 * satellite itself), because the old pace gate was already discarding ~60% of
 * what the analysis lane offered -- `pace-skip` ~52/s in every soak on file --
 * so taking a rate that was 26.6 and landing it at 9.8 costs little on a lane
 * whose frames are SNAPSHOTS.
 *
 * FRAMES ARE SNAPSHOTS; THE DETECTOR THAT EATS THEM IS NOT. df::RemoteDetect
 * derives flux from the DIFFERENCE between consecutive frames and builds its
 * threshold from a history measured in FRAMES (BEAT_HIST, 43 of them). Decimate
 * the series and every one of those changes: flux spans 102 ms instead of 11.6,
 * so a kick's attack falls between two samples; the adaptive window stretches
 * from 0.5 s to 4.4 s; and the pattern's envelope, which decays per frame drawn,
 * turns a swell into a ten-hertz staircase. It was visible from across the field
 * on the first evening -- the hub following the music and the satellites
 * lurching -- which is what TX_FRAME_BATCH below is for.
 *
 * The pace itself survived that discovery unchanged, because it was never the
 * part that was wrong: a frame sent more often than the beacon releases is a
 * frame that arrives no sooner and costs a buffer meanwhile. What was wrong was
 * sending one frame in the datagram the beacon does release.
 *
 * WHY IT STILL MATTERS AFTER THE LEAD FIX. This was written believing the TX
 * pool was the fault; it was not (see LEAD_US). What it does is shorten the TAIL
 * of delivery latency -- the occasional packet held for one or two DTIM periods
 * because the queue ahead of it was deep. That tail is exactly what the lead
 * margin has to absorb, and the 2026-08-20 soak caught its worst case at 153 ms
 * on a packet the hub had transmitted cleanly. A smaller burst is a shorter
 * tail, so the two changes work on the same failure from opposite ends: this one
 * makes the worst case rarer, LEAD_US makes it survivable.
 *
 * ITS PREMISE IS GONE AND THE NUMBER HAS NOT BEEN RE-MEASURED. Everything above
 * reasons about the rate a SoftAP releases GROUP-ADDRESSED frames. Multicast was
 * removed -- this lane is unicast to each satellite now, it goes out immediately
 * with rate adaptation, and it waits for no beacon. So DTIM_HOLD_US is a number
 * here rather than a reason.
 *
 * It is left exactly as it was, deliberately, because the pace is doing a second
 * job the beacon argument never named: it is what holds the frame lane to ~9.8
 * datagrams a second per satellite instead of 86, and with airtime scaling in N
 * again that is the difference between a hub sending 50 + 96xN packets and one
 * sending far more. That job is real whatever the transport.
 *
 * WHAT TO MEASURE IF THIS IS REVISITED. The open question is the delivery TAIL:
 * batching up to 102.4 ms of frames may now be ADDING the latency the pace was
 * introduced to remove, since there is no longer a DTIM queue in front of it.
 * Soak with the pace here and at one analysis hop (~11.6 ms at 512), reading
 * cong-skip, pace-skip and tx-fail (N audio) on the hub against gaps and
 * frames/s on the satellite -- and load the fan-out with tools/satsim, because
 * the cost of an unpaced lane is per satellite. Do not change it on the
 * arithmetic alone; the last time this lane was retuned on arithmetic it
 * starved the satellites' detector (see the paragraphs above).
 */
#define TX_FRAME_PACE_US       DTIM_HOLD_US

/*
 * How many frames ride in the one datagram the beacon releases.
 *
 * The pace above says WHEN the frame lane may transmit; this says what it takes
 * with it. Every frame the analysis produces is now sent -- the satellites are
 * back on the hub's own 86/s series, and the detector they run is back on the
 * cadence its constants were swept at -- while the group burst stays what pacing
 * made it: audio's ~5 packets plus exactly one frame packet, every beacon.
 *
 * 12, against the 8.8 a beacon holds at hop 512 (102400 / 11610). The spare
 * three are for the analysis task's own lumpiness: it does not run at a
 * metronomic 86/s, and a decoder lump hands it several windows at once, which
 * without headroom would fill the batch early and cut it short. Frames are
 * appended as they arrive and the batch goes out when the pace elapses, so a
 * quiet period sends a short batch rather than a late one.
 *
 * Raising it past what FRAME_PAYLOAD_MAX holds is a compile error, not a
 * fragmented packet -- clients.c static-asserts the product against the cap.
 *
 * IT IS ALSO THE LOSS-GRANULARITY KNOB. One lost group packet is now this many
 * consecutive frames rather than one: ~102 ms of them, a hole every 30-50 s at
 * the 0.2-0.3% loss measured on this link. If that ever reads as a stutter on
 * the floor, halve this and send two batches per beacon -- two buffers instead
 * of one, still deterministic, still nothing like the ~26 the unpaced lane put
 * in the pool. Do not answer it by shortening the pace; that is the change this
 * lane has already made twice and paid for twice.
 *
 * AND IT IS THE DELIVERY-MARGIN KNOB, which is the one to watch first. A frame
 * appended just after a flush waits a whole batch period before it is even
 * offered to the radio, and then however long the beacon phase costs it: call it
 * ~200 ms worst case against the LEAD_US of 350 that the analysis runs ahead of
 * playback. The remainder is what absorbs the delivery tail -- 153 ms at its
 * worst on the 2026-08-20 soak -- so the margin is real but not large. The
 * satellite already measures the outcome directly: `late` on its LED line counts
 * frames that came due before the render task reached them. If that climbs,
 * halving this halves the batching delay as well as the hole, because a batch
 * that fills early is sent early. Zero is what it should read.
 */
#define TX_FRAME_BATCH         12

/*
 * Local playback ring. The master delays its own audio by LEAD_US exactly like a
 * satellite, otherwise it would play ahead of every other speaker.
 *
 * THE LEAD IS ~35 kB, not the ~21 kB this said until 2026-08-12. LEAD_US was
 * 200 ms then and 200 ms at 44.1 kHz stereo is 35,280 bytes; the old figure
 * described a shorter lead and was never updated when it grew. It mattered,
 * because it made this ring look far better provisioned than it was: 64 kB reads
 * as three times the lead against 21 kB and is only 1.86x against the real one.
 * (The lead is 250 ms now -- 44,100 bytes -- which the paragraph on 80 kB below
 * is sized against. Same trap, one size along.)
 *
 * 80 kB is 464 ms. It was 48 kB / 279 ms, and before that 64 kB / 371 ms; the
 * cut to 48 released 16 kB for WiFi static TX buffers, because this ring was 48%
 * of the internal heap and the only place a WiFi-sized block existed.
 *
 * FED-DROP FALSIFIED IT, which is what that counter is for, and the note here
 * said what to do: "non-zero means the burst no longer fits and the ring was the
 * wrong donor; put it back to 64 kB and take the TX buffers from somewhere
 * else." Measured 2026-08-16, twice in three hours -- 46,080 B and 42,496 B --
 * both immediately after the A2DP source stalled (11.8 s and 2.4 s, on sbc_in's
 * `max gap`) and the decoder caught up in a lump.
 *
 * PAST 64 kB, TO 80, because the burst it is sized against has grown and the
 * lead it must hold has too. `max gap` now sits at 51-80 ms in 523 of 597
 * windows and tops out at ~95 ms, against the 34.8-42.0 ms this was sized on;
 * and LEAD_US is 250 ms, which is 44,100 bytes of the ring before any burst
 * arrives. 80 kB leaves 37,820 bytes -- 214 ms -- above the lead, which clears
 * the worst normal gap twice over and would have swallowed both of the
 * catch-up lumps above.
 *
 * "SOMEWHERE ELSE" IS PSRAM, and that is what makes this affordable now rather
 * than a trade against the transmit path. pcm_stream's 32 kB moved to SPIRAM
 * (visualiser.cpp), measured as hub internal min 30592 -> 37704 with the buffer
 * count unchanged, so this ring can grow 32 kB without the TX pool giving
 * anything back. Nothing else in this tree may follow it there without the same
 * argument: SPIRAM_USE_CAPS_ALLOC exists so that things stay internal unless
 * they ask, and the DMA buffers and every task stack must never ask.
 *
 * Still falsified by fed-drop. Non-zero again means the source stalls longer
 * than this holds, and the answer stops being memory.
 */
#define LOCAL_RING_BYTES (80 * 1024)

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

/*
 * An insert may not push the ring past this far below capacity, since
 * 2026-08-18. The same 50 ms as the catch-up drain's "level >= target + 50
 * refuses inserts" -- the system's existing definition of "too deep to add
 * to". The splice's insert takes DAC time and consumes nothing from the
 * ring, so while its zeros play, receive keeps pushing: on the 2026-08-18
 * soak, 150 ms inserts into already-brimming rings took both satellites to
 * the 464 ms ceiling and 121/89 decoded blocks were dropped at rx. The skip
 * side needs no such clamp -- its discard loop reads with a zero timeout
 * and stops on an empty ring by construction.
 */
#define SPLICE_INSERT_HEADROOM_MS 50

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
/*
 * ... and the same total split by WHICH LANE was refused, because "how many"
 * and "whose" are different questions and only the second one picks a fix.
 *
 * AUDIO is the subset that is audible, and was the only split this had. The
 * 2026-08-20 soak is why the rest exist: `tx-fail 534 (454 audio)` left 80
 * failures unexplained on a line where the whole argument was about which lane
 * was holding the WiFi driver's transmit buffers. They were all publish_frame,
 * as it happens -- but nothing on the line said so, and VOL, META and the probe
 * replies were not counted AT ALL, so the printed total was not even the true
 * total. A lane that cannot be seen cannot be exonerated.
 *
 * Ordered audible-first, which is also roughly worst-first: a refused audio
 * packet is a hole in the sound on every satellite at once, a refused frame is
 * one repaint, a refused level is covered by the 1 Hz repeat, and a refused
 * probe reply is one clock sample out of four a second.
 *
 * Racy exactly as s_tx_fail is, and for the same reason -- see net.c.
 */
typedef enum {
    TX_LANE_AUDIO = 0,   /* fan_out(): the only refusal the room can hear */
    TX_LANE_FRAME,       /* publish_frame(): analysis frames */
    TX_LANE_VOL,         /* streamer_send_vol() */
    TX_LANE_META,        /* streamer_send_meta(): track metadata */
    TX_LANE_PROBE,       /* probe_task(): time and TSF replies */
    TX_LANE_N,
} tx_lane_t;

extern volatile uint32_t s_tx_lane_fail[TX_LANE_N];

/*
 * WHO WAS ON THE POOL WHEN AUDIO WAS REFUSED -- the instrument eight dead
 * hypotheses have needed and none of them had.
 *
 * Every ENOMEM diagnosis so far has been inferred from counters that do not
 * move. Ruled out, each against a soak: A-MPDU TX off (worse -- see
 * sdkconfig.defaults), the channel (the boot survey picks 11 correctly), the
 * SPI link (hdr/crc/short 0 over ~3M packets), heap (internal free flat at
 * ~12,288 B through every refusal), the source (the hub never starves during a
 * satellite dropout), the pool size (36->48 bought bufferbloat and
 * anchors-refused; 38 is the settled value), a 2.4 GHz blackout (the 2026-08-23
 * 01:38 run had two major episodes with the band wide open -- see analyse.py's
 * section 7b), DTIM group buffering (fixed: every lane is unicast now, and the
 * beacon-rate signature is gone -- 0 of 41 and 0 of 32 burst gaps in the
 * 75-150 ms bucket during that run's two episodes, against a median 40 bursts
 * per window measured on 2026-08-20).
 *
 * What none of that could see is the inside of the pool. IDF exposes no numeric
 * count of free static TX buffers -- esp_wifi_statis_dump() prints to the
 * console and cannot be sampled at the millisecond a burst lives on -- so the
 * pool's own occupancy is not measurable from here.
 *
 * This measures the next best thing and the only mechanism still standing: hub.h
 * above documents the frame lane crowding the pool until AUDIO's sendto comes
 * back ENOMEM, measured at the audible glitch moments of the 2026-08-17 soak.
 * TX_FRAME_PACE_US was the fix. Whether it is ENOUGH has never been tested,
 * because nothing recorded whether a frame send was in flight at the instant
 * audio was refused. Now it does: publish_frame stamps s_tx_frame_sent_us, and
 * an audio ENOMEM within TX_NEAR_US of that stamp increments n_refuse_near_frame.
 *
 * HOW TO READ IT. `refuse-near-frame` close to the window's audio refusals says
 * the frame lane is still the competitor and the pace is not enough. Close to
 * zero says the pool is being drained by something that is not us -- the
 * driver's own retries, which is the air -- and that is a different repair
 * entirely. It is the first reading that can tell those two apart.
 */
#define TX_NEAR_US              5000    /* "in flight": ~4 audio packets' grace */
extern volatile int64_t  s_tx_frame_sent_us;   /* esp_timer stamp of the last frame fan-out */
extern volatile uint32_t n_refuse_near_frame;  /* audio ENOMEMs with a frame send just before */

/*
 * The refused audio packet's second chance, and the counters that judge it.
 *
 * fan_out() used to count a refusal and drop the packet, which is the hole the
 * room hears -- and under FEC a hole is recoverable only if the NEXT packet
 * gets through, which during a burst is exactly what does not happen. The pool
 * frees buffers as frames complete, on a timescale far below one audio period,
 * so an immediate second sendto costs one syscall and may well find room.
 *
 * DELIBERATELY NOT A QUEUE. A deferred resend would arrive after a newer packet
 * and the satellite would count it seq-drop and bin it, so the retry has to be
 * now or never. One attempt, ENOMEM only: any other errno is a real error and
 * retrying it would just hide it.
 *
 * JUDGE IT ON THE PAIR. `audio-retry N | audio-retry-ok M` -- if M tracks N
 * the pool is transiently empty and this recovers most of the fault; if M
 * stays near zero the pool is empty for longer than a syscall and the retry
 * should come out again. Either reading is worth more than the drop it
 * replaces.
 */
extern volatile uint32_t n_audio_retry;        /* audio ENOMEMs that got a second sendto */
extern volatile uint32_t n_audio_retry_ok;     /* ...of which the second one went */

/*
 * Stations arriving or leaving in this window, both counted together.
 *
 * The 2026-08-20 soak put its whole ENOMEM storm in the first five minutes,
 * starting the minute the stream did, with both satellites re-joining after the
 * hub was flashed -- and then ran eleven minutes clean. sdkconfig.defaults
 * describes the same shape ("a satellite joining a playing stream measured 1016
 * sendto() rejections in one 20 s window"), so a join is the leading suspect
 * for what holds the transmit pool.
 *
 * That is still an INFERENCE from two timestamps lining up. This makes it a
 * reading: if churn and refusals share a window, the join is the mechanism; if
 * refusals arrive in windows with churn 0, it is not, and the suspicion dies
 * instead of being carried forward.
 *
 * Arrivals and departures in one counter deliberately. They come in pairs on a
 * re-join, the question is "was the AP doing association work", and two numbers
 * that always move together are one number with extra width. `stations` on the
 * same line already says what the floor settled at.
 */
extern volatile uint32_t n_join_churn;

/*
 * Non-audio publish throttling, paired with TX_BACKOFF_US. tx_fail_note() raises
 * s_tx_congested_until on ENOMEM; publish_frame skips while now < it and counts
 * the skip in n_tx_cong_skip. fan_out() ignores it -- audio always sends. Racy
 * exactly as s_tx_fail is (net.c): a torn read of the 64-bit deadline at worst
 * sends or skips one non-audio datagram wrongly.
 */
extern volatile uint32_t n_tx_cong_skip;            /* non-audio sends skipped under ENOMEM backoff */
extern volatile int64_t s_tx_congested_until;       /* esp_timer deadline; non-audio yields until it */

/* ...and the proactive sibling, WHICH NOW MEANS SOMETHING ELSE. It used to
 * count the frames the pace gate discarded between sends, which at one frame per
 * datagram was ~76 of every 86 -- the decimation TX_FRAME_BATCH exists to end.
 * Frames are batched now, so nothing is dropped for arriving early, and this
 * counts the one case that still throws frames away: a batch stranded by a
 * stream that stopped, whose instants have passed before the next frame arrives
 * to flush it. Expect a handful per track boundary and zero in between. A
 * standing rate here means frames are being computed with due times already in
 * the past, which is a timeline fault rather than a transport one. Read beside
 * cong-skip, which says the pool was exhausted; cleared in the same window. */
extern volatile uint32_t n_tx_pace_skip;            /* frames dropped with a stranded batch */

/*
 * The widest interval between two audio packets REACHING THE AIR this window.
 *
 * THE HUB'S HALF OF A CROSS-UNIT QUESTION. On the 2026-08-19 soak a satellite
 * took nine phase steps of +35 to +205 ms, every one of them with its ring at
 * zero and its DAC playing auto_clear silence for the duration, while this unit
 * logged a ring of 230-295 ms, phase within +-6 ms and tx-fail 0 in 57 of 59
 * windows. Nothing was lost on the way -- the satellite's seq-drop, decode-err,
 * recv-err, wifi-drops and fec-err all read zero -- so packets were delayed and
 * released in a lump, and no counter anywhere said whether the lump formed here
 * or on the air.
 *
 * Steady state is ~20 ms of SBC packet plus whatever lumpiness the source
 * hands over: the 2026-08-19 18:36 soak read 54-77 ms per window against a
 * satellite seeing 39-95, which is the two ends agreeing that the air adds
 * nothing. This reading spiking at the same instant as the satellite's `gap
 * max` means the packets left late; this one flat while that one spikes means
 * they left on time. Read it beside the satellite's ARRIVAL line, which
 * carries the other three numbers.
 *
 * ON SUCCESSFUL SENDS ONLY, since the first version of this gauge timed
 * fan_out() CALLS and was therefore blind to the one fault it most needed to
 * catch -- see fanout_result_t in timeline.c. A refused packet leaves the
 * previous stamp standing so the next success measures the whole hole. It is
 * still not a substitute for reading tx-fail beside it: this says how long the
 * hole was, tx-fail says the hub made it.
 *
 * A GAUGE, not a counter: cleared by the window that prints it, exactly as the
 * satellite's arrival gauges are, because a maximum accumulated across windows
 * is not a maximum of anything.
 */
extern volatile int32_t n_fanout_gap_max_us;

/*
 * The least lead any packet carried when it was STAMPED this window, in us.
 *
 * THE OTHER HALF OF THE TRANSIT TIME, and the one number that separates a hole
 * before sendto() from a hole after it. The satellite already reports the lead
 * every packet had when it ARRIVED (sat.h, rx_lead_min_us): play_at minus the
 * arrival instant on the master clock. This is the same quantity measured where
 * the stamp is written, so
 *
 *     transit ~= this window's lead-min  minus  the satellite's lead-min
 *
 * with no protocol change, because both ends were already computing their half
 * and only this one went unrecorded.
 *
 * WHY IT IS NEEDED. On the 2026-08-19 20:04 soak the satellite saw a 386 ms
 * hole in arrivals at 20:49:56 with its lead collapsed to -169 ms, while the
 * source read a healthy 50-77 ms all window, the hub's fan-out gap read 67-72
 * ms on successful sends, and 248 of 250 packets arrived with one 20 ms
 * sequence gap. The audio existed, this unit emitted it evenly, almost none was
 * lost -- and it still did not turn up for 386 ms. Nothing measured whether
 * those packets were still here or already on the air, because n_fanout_gap_max_us
 * times when lwIP ACCEPTED a frame, not when the radio sent it.
 *
 * So: this flat while the satellite's collapses means the packets left on time
 * and were held after sendto() -- the driver's TX queue, the SoftAP's group
 * buffering, or the air. Both collapsing together means they were stamped late,
 * and the fault is upstream of the radio entirely.
 *
 * A 20 s WINDOW against the satellite's 5 s, because this rides the status line
 * and that is its cadence. Coarser, and it does not matter for the question: a
 * hole drives the satellite's reading hundreds of ms negative, which no 20 s
 * minimum of a healthy sender can imitate.
 *
 * Measured at the stamp rather than after the send, so a refused packet is
 * included. That is the right way round for this question -- it asks when the
 * timeline said the audio was due, not whether the radio took it -- and tx-fail
 * beside it says whether any were refused at all.
 *
 * A GAUGE, cleared by the window that prints it, exactly as the two beside it.
 * LEAD_UNSEEN means no packet was stamped this window, which is distinct from a
 * measured zero and must not print as one.
 */
#define LEAD_UNSEEN INT32_MAX

/*
 * Beyond this a reading is not a lead, and is REFUSED rather than clamped.
 *
 * One second, the same number the satellite's LEAD_INSANE_US uses, because the
 * two ends subtract to a transit time and must agree on what they are
 * subtracting. Normal operation cannot approach it: the lead is LEAD_US plus at
 * most RESYNC_HARD_US of timeline wander before the jump takes over, so 550 ms
 * one way and a few hundred ms the other.
 *
 * This end has no known way to produce an insane one -- both terms are this
 * board's own clock. The satellite's copy does, and the soak that found it has
 * the story: see sat.h. The bound is here so a gauge cannot report its own rail
 * as a measurement, which is a property worth having on both ends whether or
 * not this one has ever needed it.
 */
#define LEAD_INSANE_US 1000000
extern volatile int32_t n_lead_min_us;

/*
 * Every failed sendto() goes through this rather than incrementing s_tx_fail
 * directly, so the reason is kept alongside the count. Call it with errno, at
 * the failure, before anything else can overwrite it. tx_fail_summary() renders
 * the tally for the status line and clears it. Both live in net.c, which owns
 * the socket; the rationale for keeping the reason at all is there.
 */
void tx_fail_note(tx_lane_t lane, int err);
/* The audio downlink's own entry point. Kept as a name rather than folded into
 * the call above because timeline.c's two send paths are the only callers and
 * three of its comments refer to it by name. */
void tx_fail_note_audio(int err);
void tx_fail_summary(char *buf, size_t len);
/* The lane breakdown for the status line, rendered and cleared together with
 * the errno tally above so the two cannot describe different windows. */
void tx_fail_lanes(char *buf, size_t len);
/* ...and the shape of the ENOMEM storm behind them: whether refusals arrive in
 * beacon-spaced clusters (the queue is deep) or one unbroken stall (the release
 * has stopped). Writes an empty string when nothing was refused. See net.c. */
void tx_burst_summary(char *buf, size_t len);
/* Closes an open refusal stretch: an audio packet got through, so the pool
 * freed a buffer. Called from fan_out() on FANOUT_SENT. See net.c for why
 * audio, and only audio, is what may close one. */
void tx_send_ok(void);
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
 * Frames the fine rate trim has dropped from, and duplicated into, the stream.
 *
 * The instrument for rate_trim_hz, and the reason it went in before the trim
 * was ever flashed: a correction you cannot see in a log is a correction you
 * cannot attribute. On this branch a doubled packet rate went unnoticed through
 * a build, two test suites and a review because the one counter that would have
 * shown it was added afterwards.
 *
 * The rate is |rate_trim_hz| frames per second, by construction: a trim of N Hz
 * against a rate of `rate` needs rate * N/rate = N extra frames each second.
 * Measured on the first run that played: 720 frames in the 60 s window where
 * the trim was -14 then -10 Hz, and 300 in the window where it was -10 then
 * -6. That is the arithmetic, and it is what these are here to keep honest.
 *
 * What that means in practice, which is NOT one number:
 *
 *   converging   ~14/s for the first minute or two. Startup phase is ~-32 ms
 *                (the DMA refill, see s_refill_active) and the loop spreads it
 *                over ~100 s, so it asks for ~320 ppm -- twenty times the
 *                crystals' own difference. Heard on the bench as nothing.
 *   steady       ~1/s. Real drift is ~14 ppm, which is 0.6 Hz, and the trim is
 *                whole Hz, so it sits at 0 or 1 and the deadband holds it
 *                there.
 *   depth net    20/s, the +-20 Hz rescue. The loudest this gets.
 *
 * Flat when rate_trim_hz is non-zero means the trim is not running. Both
 * climbing together means the servo is hunting across zero.
 */
extern volatile uint32_t n_trim_drops;
extern volatile uint32_t n_trim_dups;

/*
 * The faded catch-up: audio_shift.h is the mechanism, and these are its state.
 * Same meanings, same ownership split, same counters as the satellite's copy
 * in sat.h: servo_tick arms the debt (signed FRAMES; positive = skip, late),
 * playback is the only thing that shrinks it, one chunk's shift at a time as
 * the crossfade spends it.
 *
 * On this unit the drain is not only for its own speaker: the knock it exists
 * to pay off usually arrives on BOTH units at once -- the tx-fail burst that
 * starves the satellites starves this ring too -- and paying it off at the
 * same rate from the same code is what keeps the two from diverging while
 * they do.
 */
extern volatile int32_t  catchup_frames;
extern volatile uint32_t n_catchup_drops;
extern volatile uint32_t n_catchup_dups;

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

/*
 * How the volume is repeated, and how often.
 *
 * VOL_REPEAT_US is the interval of the standing repeat, and under the
 * silent-until-told rule it is also the worst-case silence a joining satellite
 * sits through when client_joined()'s push is lost. One second of two bytes is
 * nothing to send and a short thing to hear.
 *
 * VOL_CHANGE_REPEATS covers the moment the level is provably wrong everywhere
 * else. Group frames are not retried, so a change is sent more than once rather
 * than trusted once; three is enough that losing all of them needs the link to
 * be failing at a rate the audio would already be reporting.
 */
#define VOL_REPEAT_US     1000000
#define VOL_CHANGE_REPEATS 3

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
 * RATE_TRIM_MAX_HZ -- the widest DRIFT correction the servo may ask for, and
 * the boundary between its two actuators -- now lives in audio_shift.h, so that
 * this unit and the satellite cannot disagree about how fast either may
 * correct. The general reasoning is there; what is this unit's own is that the
 * bound is deliberately NOT applied inside retune_dac(). A coarse correction is
 * routed to the clock precisely so that it arrives in front of
 * RATE_SANE_MIN/MAX below and is refused there if it is nonsense.
 */

/* What could conceivably be an audio sample rate at all. Anything outside this
 * is a broken calculation, whoever asked for it. */
#define RATE_SANE_MIN 8000
#define RATE_SANE_MAX 192000

/*
 * The FINE rate correction, in Hz, written by the servo and read by playback.
 *
 * This is what the servo used to hand to retune_dac(). It now names a rate the
 * DAC is NOT running at: the clock stays put and playback consumes the ring at
 * an effective (tx_rate + rate_trim_hz) instead, by dropping one frame when it
 * needs to get through the stream faster and duplicating one when it needs to
 * get through it slower. Positive means playing late, so consume faster.
 *
 * Why it is worth the trouble: a clock retune takes the I2S channel down for
 * 1.7-6.2 ms, measured, mean ~3.6 ms, once every 20-45 s per unit -- to apply
 * +-4 Hz against ~14 ppm of real drift. It was the last self-inflicted
 * interruption in the audio path. One frame in ~71000 is inaudible and, more to
 * the point, continuous where the clock was stepped.
 *
 * In whole Hz because that is what the servo already computes and what the
 * deadband already quantises to, NOT because anything here needs it: the
 * whole-Hz floor under PHASE_DEADBAND_US was a property of the CLOCK, and the
 * software path has no such floor. That is what makes tightening the deadband a
 * later one-line experiment -- see docs/clock-sync.md.
 *
 * Persists across a playback restart, exactly as tx_rate does. Zeroed only when
 * a coarse retune moves the clock, because the clock then carries what this was
 * carrying.
 */
extern volatile int32_t rate_trim_hz;

/*
 * Playback volume, 0-127, AUDIO_VOL_MAX being unity.
 *
 * MEANINGLESS UNTIL audio_vol_known. It used to default to AUDIO_VOL_MAX; see
 * audio_vol_effective() in audio_out.h for why an untold level is now silence
 * with a bounded fallback instead. The hub reaches that state the same way a
 * satellite does -- it reboots, and the phone is not going to send a command it
 * has no reason to send -- which is what the bridge's heartbeat now covers.
 *
 * Written by the SBC input task when the phone moves the slider, read by
 * playback. Applied at the DAC write only, so what is transmitted to the
 * satellites stays FULL SCALE and each unit attenuates its own output.
 * Attenuating before the send would spend the air's dynamic range on a level
 * decision and leave the satellites unable to differ, which is the same mistake
 * the phone was making before the bridge advertised an AVRCP target.
 *
 * A torn read is not possible on a byte, and a stale one costs one chunk at the
 * previous level -- 5.8 ms, at a step a human just asked for. The flag is stored
 * after the level for the same reason the satellite's is; see sat.h.
 */
extern volatile uint8_t audio_volume;

/*
 * Whether the bridge has ever said how loud, this boot. Sticky; see the
 * satellite's copy in sat.h for why nothing clears it.
 *
 * streamer_send_vol() is gated on this. A hub must not relay a level it invented
 * -- if the local fallback fired and were broadcast, a hub whose bridge had died
 * would blast satellites that were sitting correctly at -50 dB.
 */
extern volatile bool audio_vol_known;

/*
 * MSG_VOL datagrams sent to listeners, REPEATS INCLUDED.
 *
 * Counted before any change test, which is what makes it useful: the fault it
 * exists to expose is a unit playing audio at a level nobody told it, and that
 * reads as a satellite whose own vol-rx sits still while this counter climbs. A
 * counter that only moved on change could not tell that apart from a slider
 * nobody touched.
 *
 * The other direction is counted in sbc_in.c, on that module's own line, with
 * its siblings -- it does not read hub.h and is not going to start.
 *
 * On the TRIM line as `vol-tx`, which tools/soak/capture.py's KEYNUM pass turns
 * into a metrics column with no parser change.
 */
extern volatile uint32_t n_vol_tx;

/* What a retune costs -- see the note on the satellite's copy. The channel down
 * is measurable here; the discarded DMA buffer is not measurable anywhere in
 * software, because those frames were counted as played.
 *
 * Since 2026-08-14 this is a COARSE-ONLY path: the servo reaches it when the
 * correction exceeds RATE_TRIM_MAX_HZ, which in steady state it never does. The
 * measurements below stand; what changed is how often they are paid. */
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
 * publish_frame, publish_ml and streamer_send_sbc. (publish_ml has since gone
 * with the rest of the result-distribution path; three remain.) What was
 * deliberately NOT shared is the MESSAGE BUILDING: those stay separate because
 * they are on different cadences and each is likely to grow its own rate limit.
 * This is only the copy, which was byte-identical in all of them and is where
 * MAX_CLIENTS scaling lands.
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
