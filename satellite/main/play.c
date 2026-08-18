/*
 * The playback timeline: holding the first sample for its instant, measuring
 * phase at each packet boundary, splicing at a track change, and the marker.
 *
 * Split out of main.c on 2026-08-12; the bodies are unchanged.
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
#include "audio_shift.h"
#include "sbc_link.h"
#include "sbc_decoder.h"
#include "visualiser.h"
#include "wifi_log.h"

#include "sat.h"

/*
 * Frames the fine rate trim owes, in 1/TRIM_ONE_FRAME of a frame.
 *
 * rate_trim_hz names a rate the DAC is not running at, so playback has to
 * consume the ring at that rate instead: over one chunk of output that is
 * AUDIO_FRAMES * rate_trim_hz / tx_rate extra frames of input, which is a
 * fraction far below one. Accumulate it and spend a whole frame when a whole
 * frame has been earned.
 *
 * At 1 Hz and 44.1 kHz the step is 380, so a frame comes due every 172 chunks
 * -- one second, which is 22.7 ppm, exactly the granularity a whole-Hz clock
 * retune had. At the RATE_TRIM_MAX_HZ clamp the step is 38036, still under
 * TRIM_ONE_FRAME, so ONE frame per chunk is always enough headroom and no pass
 * ever needs two.
 *
 * Identical to the hub's copy, to the arithmetic. Unequal correction between
 * the two units is a cross-unit sync error by construction.
 */
#define TRIM_ONE_FRAME 65536
static int32_t s_trim_owed;

/*
 * The previous accepted phase reading, for the step detector in the crossing
 * loop. Play task only, and reset at every start -- the first reading of a new
 * stream is not a step from the last reading of the old one, and calling it one
 * would put a PHASE STEP line under every re-anchor.
 */
static int32_t s_phase_prev;
static bool    s_phase_prev_valid;

/*
 * How far two consecutive readings must differ to be worth a line.
 *
 * Drift is ~0.8 ms per minute and the deadband is 7 ms, so 20 ms is clear of
 * everything the system does on purpose and well under the 40-270 ms steps the
 * soak recorded. A splice trips it too, which is correct: a splice IS a step,
 * and it already announces itself on the line above.
 */
#define PHASE_STEP_LOG_US 20000

/* +1 to drop a frame this pass, -1 to duplicate one, 0 for neither. */
static int trim_due(void)
{
    const int32_t hz = rate_trim_hz;
    if (hz == 0) {
        /* Not just an optimisation: it stops a stale fraction sitting in the
         * accumulator for as long as the trim is off. */
        s_trim_owed = 0;
        return 0;
    }
    s_trim_owed += (int32_t)((int64_t)AUDIO_FRAMES * hz * TRIM_ONE_FRAME
                             / (int64_t)tx_rate);
    /* Bounded before it is spent, so that a spell where the drop could not be
     * taken -- the ring was momentarily empty -- cannot wind up an arbitrary
     * debt and then pay it off in a burst of consecutive drops. */
    if (s_trim_owed >  2 * TRIM_ONE_FRAME) s_trim_owed =  2 * TRIM_ONE_FRAME;
    if (s_trim_owed < -2 * TRIM_ONE_FRAME) s_trim_owed = -2 * TRIM_ONE_FRAME;
    if (s_trim_owed >= TRIM_ONE_FRAME) {
        s_trim_owed -= TRIM_ONE_FRAME;
        return 1;
    }
    if (s_trim_owed <= -TRIM_ONE_FRAME) {
        s_trim_owed += TRIM_ONE_FRAME;
        return -1;
    }
    return 0;
}

/*
 * The catch-up read buffer. A pass that shifts k frames reads AUDIO_FRAMES + k
 * in and crosses it down to AUDIO_FRAMES out (audio_shift_chunk reads both
 * strands from the input while writing chunk), so it cannot land in chunk.
 * Sized for the largest shift the constants allow plus the fine trim folded
 * in: CATCHUP_SHIFT_MAX + 1.
 */
static int16_t s_cu_in[(AUDIO_FRAMES + CATCHUP_SHIFT_MAX + 1) * AUDIO_CHANNELS];

/*
 * How much catch-up this pass's shift is carrying, set by chunk_shift() and
 * spent by the caller once the pass lands. Play task only.
 */
static int s_catchup_take;

/*
 * Frames of catch-up applied since the phase history was last aged. Each one
 * moves this unit 23 us silently; after PHASE_STEP_LOG_US worth, every reading
 * still in the history describes a unit that has moved, and it is dropped --
 * the same call, for the same reason, as after a splice.
 */
static int32_t s_catchup_moved;

/*
 * The shift for this pass: the fine trim's 0/+-1, plus the armed catch-up debt
 * while one is draining.
 *
 * While a debt is draining, the trim is FOLDED INTO the shift rather than
 * taken separately, so one crossfaded pass carries both and the two
 * mechanisms cannot each take a frame out of the same chunk. Outside a drain
 * the return is the trim's 0/+-1 and the plain paths handle it exactly as
 * they always did.
 *
 * Depth guard, because debt and depth are the same quantity on this unit:
 * ring level is RING_TARGET_MS + lateness, so a unit 40 ms late legitimately
 * holds 40 ms past target, and draining a debt drains the ring toward target
 * -- which is what a late unit wants. The guard only says no when the ring
 * does not have the margin the shift would spend: no dropping through a ring
 * already below target (that is a unit refilling after trouble, not a late
 * one), no inserting into one already past target + 50 ms.
 */
static int chunk_shift(void)
{
    s_catchup_take = 0;
    const int trim = trim_due();

    const int32_t debt = catchup_frames;
    if (debt == 0) {
        return trim;
    }

    const int32_t level_ms = (int32_t)((int64_t)(RING_BYTES -
            (int64_t)xStreamBufferSpacesAvailable(ring)) * 1000
            / ((int64_t)stream_rate * AUDIO_CHANNELS * 2));
    if (debt > 0 && level_ms <= RING_TARGET_MS - 30) {
        return trim;    /* dropping now would push a thin ring at the DAC */
    }
    if (debt < 0 && level_ms >= RING_TARGET_MS + 50) {
        return trim;    /* inserting would stack onto an already-deep ring */
    }

    int32_t k = debt;
    if (k >  CATCHUP_SHIFT_MAX_DROP) k =  CATCHUP_SHIFT_MAX_DROP;
    if (k < -CATCHUP_SHIFT_MAX_DUP)  k = -CATCHUP_SHIFT_MAX_DUP;
    s_catchup_take = (int)k;
    return (int)k + trim;
}

/*
 * The level to actually play this chunk with.
 *
 * Kept out of the ring and out of every count, exactly like the level itself:
 * this decides what the DAC hears, and nothing upstream may be able to see it.
 *
 * The deadline is a local of this task because this task is its only reader --
 * no shared 64-bit field, so nothing to tear. It is measured from boot rather
 * than from playback start; audio_vol_effective() has the argument.
 *
 * Says so once, loudly, if it ever fires. A unit inventing its own loudness is
 * not a thing to discover by ear.
 */
static uint8_t vol_now(void)
{
    static bool told;
    const bool due = esp_timer_get_time() >= AUDIO_VOL_UNKNOWN_HOLD_US;
    if (due && !audio_vol_known && !told) {
        told = true;
        ESP_LOGE(TAG, "NO VOLUME in %d s -- nothing has told this unit a level. "
                      "Falling back to FULL SCALE.",
                 (int)(AUDIO_VOL_UNKNOWN_HOLD_US / 1000000));
    }
    return audio_vol_effective(audio_volume, audio_vol_known, due);
}

void play_task(void *arg)
{
    (void)arg;
    static uint8_t chunk[AUDIO_CHUNK_BYTES];

    while (1) {
        if (stream_start_local == 0) {
            /* Parked, by whichever route -- resync, underrun or a changed
             * timeline. The flag has been served either way, and clearing it
             * here rather than only where it is consumed is what lets the
             * anchor path below wait on it without being able to deadlock. */
            resync_request = false;
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        /* Hold the first sample until its moment arrives. After this, I2S paces
         * everything: i2s_channel_write blocks once the DMA buffers are full. */
        int64_t wait = stream_start_local - esp_timer_get_time();
        if (wait > 2000) {
            vTaskDelay(pdMS_TO_TICKS((wait - 2000) / 1000));
        }
        while (esp_timer_get_time() < stream_start_local) {
            /* spin the last stretch, as in the M4 blink task */
        }
        int64_t actual_master = esp_timer_get_time() + stream_offset;
        int64_t sched_master = stream_start_local + stream_offset;
        ESP_LOGI(TAG, "playback started: scheduled %lld, actual %lld (%+lld us) [master]",
                 sched_master, actual_master, actual_master - sched_master);
        /*
         * samples_played counts from the first sample played, which is the
         * first sample queued after the ring was reset on anchoring. The two
         * counters therefore share an origin -- do NOT reset samples_in here.
         * It has legitimately been counting the audio buffered during the wait,
         * and zeroing it shifts every marker by the buffer depth.
         */
        int32_t samples_played = 0;
        /* The fraction of a frame owed described the stream this anchor
         * replaces. rate_trim_hz itself is NOT reset: it is this unit's
         * standing rate offset against the source, exactly as tx_rate is, and
         * it survives a restart for the same reason. */
        s_trim_owed = 0;
        /* Every reading in it was measured against the stream this anchor
         * replaces, so none of them describes where this unit now is. */
        sync_phase_reset(&phase_hist);
        /* Same reason: an armed debt described a position on the stream this
         * anchor replaces, and a replay debt draining into the freshly-reset
         * thin ring is the audible stretch again. The servo re-arms from
         * fresh readings if the error survives the re-anchor; the restart
         * hold (CATCHUP_HOLD_US) gives them time to be medians first. */
        catchup_frames = 0;
        /* Same reason, and it stops the first reading of this stream being
         * reported as a step from the last reading of the previous one. */
        s_phase_prev_valid = false;
        /*
         * Measure the DMA prefill at every playback START, not only after a
         * retune. The channel has been draining while this task was parked, so
         * it is empty here for exactly the same reason it is empty after a
         * disable -- and i2s_channel_write() does not block until the
         * descriptors are full, so the first writes return at memory speed and
         * samples_played advances by the whole DMA depth against a wrote_at
         * that has barely moved. Every phase reading dated from that window is
         * measured against a reference the DAC is not pacing.
         *
         * This is the same mechanism the `retuning` park exists to prevent, and
         * clock-sync.md records it costing +42, +43 and +50 ms there before the
         * park existed. Nothing guards it here.
         *
         * It is very likely the "-42 ms (hub), -26 ms (satellite)" startup phase
         * in clock-sync.md §8 -- a 16 ms difference between two units that then
         * takes ~45 s to walk off, on a cold start, on both units at once. On a
         * reconnect only this unit restarts, against a hub already servoed to
         * zero, which is a different and much easier situation.
         *
         * MEASUREMENT ONLY: the REFILL line reports, nothing withholds. Whether
         * to withhold is the next question and this is what sizes it.
         */
        s_refill_active = true;
        s_refill_frames = 0;
        /* When the DAC last accepted a chunk -- what the phase reading is dated
         * against. See the hub's copy for why it is not a clock read taken in
         * the phase loop. Seeded so the first pass has a sane value. */
        int64_t wrote_at = esp_timer_get_time();

        /* Something is now meant to be keeping the DAC fed, which is the only
         * condition under which a starved channel is a fault. See `playing`. */
        playing = true;

        while (1) {
            if (retuning) {
                /* Do not pull from the ring while the channel is down -- the
                 * writes would return instantly and drain it. */
                vTaskDelay(pdMS_TO_TICKS(2));
                continue;
            }
            /*
             * The receive task found a gap too large to fill and wants a clean
             * restart -- see GAP_RESYNC_MS. Leave by the same door as an
             * underrun, so the anchor path owns the ring reset rather than
             * racing this loop for it. Noticed within one chunk (~5.8 ms),
             * comfortably inside the ~20 ms until the next packet arrives.
             *
             * Not logged here. This task is the audio path, and putting an
             * ESP_LOGW on it is the mistake the RX counters exist to undo;
             * drift_task narrates n_gap_resyncs within 5 s instead.
             */
            if (resync_request) {
                resync_request = false;
                stream_start_local = 0;
                phase_valid = false;
                phase_stepped = true;
                break;
            }
            /*
             * The trim varies how much is read FROM THE RING; it never varies
             * what is written to the DAC. write_audio() below still hands over
             * exactly sizeof(chunk), which is exactly one DMA descriptor -- see
             * the dma_frame_num note in i2s_start(), which both units must
             * carry because it also sets the output pipeline latency the servo
             * absorbs at startup.
             *
             * A CATCH-UP pass (|shift| > 1) is the same statement at a larger
             * size: it reads AUDIO_FRAMES + shift in, crosses the two strands
             * under the fade, and still hands the DAC exactly one chunk. See
             * audio_shift.h; the shift itself comes from chunk_shift() above.
             */
            const int shift = chunk_shift();
            const bool cu = (shift > 1 || shift < -1);
            const size_t frame_bytes = AUDIO_CHANNELS * sizeof(int16_t);
            const size_t want = cu
                ? (size_t)((int)AUDIO_FRAMES + shift) * frame_bytes
                : (shift < 0) ? sizeof(chunk) - frame_bytes : sizeof(chunk);

            hw_play = uxTaskGetStackHighWaterMark(NULL);   /* only valid in-task */
            uint8_t *const dest = cu ? (uint8_t *)s_cu_in : chunk;
            size_t got = xStreamBufferReceive(ring, dest, want, pdMS_TO_TICKS(500));
            if (got == 0) {
                n_underruns++;
                ESP_LOGW(TAG, "underrun, waiting for a new stream");
                stream_start_local = 0;
                break;
            }
            /*
             * A short read is padded to a full chunk so the DAC gets one, but
             * samples_played must NOT count the pad. See where it is advanced
             * below for why; the hub has counted it this way since it was
             * written and this unit did not until 2026-08-14.
             *
             * Measured against `want`, not sizeof(chunk): on a duplicating pass
             * a read of one frame less than the chunk is what was ASKED for and
             * is not short. The pad still fills to sizeof(chunk), because that
             * is what the DAC is given. True of a catch-up want as well -- an
             * insert's want is a frame count the plain path would happily
             * have played.
             */
            int32_t consumed = (int32_t)(got / frame_bytes);
            if (got < want) {
                n_short_reads++;
                n_short_frames += (uint32_t)((want - got) / frame_bytes);
            }

            size_t have = got;      /* bytes of real audio now sitting in chunk */
            if (cu && got == want) {
                /*
                 * The catch-up pass proper. Shift, count the input exactly as
                 * consumed, and retire the debt this pass carried. The plain
                 * trim block below does not run: this pass's shift already
                 * includes whatever trim_due() asked for.
                 *
                 * The counters count the whole shift, trim included, so the
                 * TRIM line's frames/s arithmetic keeps holding across a
                 * drain: what was spent is what was counted.
                 */
                audio_shift_chunk((int16_t *)chunk, s_cu_in, AUDIO_FRAMES,
                                  shift, CATCHUP_FADE_FRAMES, AUDIO_CHANNELS);
                consumed = (int32_t)AUDIO_FRAMES + shift;
                have = sizeof(chunk);
                catchup_frames -= s_catchup_take;
                if (shift > 0) n_catchup_drops += (uint32_t)shift;
                else           n_catchup_dups  += (uint32_t)(-shift);
                /*
                 * Age the phase history as the drain moves this unit: every
                 * reading older than CATCHUP_HIST_RESET_US worth of movement
                 * describes a position this unit has left. Resetting here
                 * keeps the splice median and the servo's next arming
                 * decision dated from where we are, and phase_stepped drops
                 * the servo's smoothing for the same reason it does at a
                 * re-anchor: the EMA is describing a different world.
                 */
                s_catchup_moved += s_catchup_take < 0 ? -s_catchup_take
                                                      : s_catchup_take;
                if (s_catchup_moved >= (int32_t)((int64_t)CATCHUP_HIST_RESET_US *
                                                 stream_rate / 1000000)) {
                    s_catchup_moved = 0;
                    sync_phase_reset(&phase_hist);
                    phase_stepped = true;
                }
            } else if (cu) {
                /* Short of the crossfade's want: play what arrived, plainly.
                 * The debt stands and is spent by a later pass -- spending it
                 * on a partial buffer would crossfade into the pad. */
                const size_t copy = got < sizeof(chunk) ? got : sizeof(chunk);
                memcpy(chunk, s_cu_in, copy);
                have = copy;
            }
            if (have < sizeof(chunk)) {
                memset(chunk + have, 0, sizeof(chunk) - have);
            }

            if (!cu && shift > 0) {
                /*
                 * Drop one frame: take it out of the ring and throw it away. A
                 * second receive rather than one oversized read, so that
                 * `chunk` stays exactly AUDIO_CHUNK_BYTES and every
                 * sizeof(chunk) here stays correct.
                 *
                 * Non-blocking, and credited only if it actually returned a
                 * frame: if the ring is momentarily empty the trim waits for
                 * the next pass rather than stalling the DAC for a correction
                 * worth 23 us.
                 *
                 * At the chunk boundary rather than mid-chunk, which is the
                 * same thing: the ring is a byte stream and the boundary is an
                 * artefact of how much is read at a time.
                 */
                static uint8_t dropped[AUDIO_CHANNELS * sizeof(int16_t)];
                if (xStreamBufferReceive(ring, dropped, sizeof(dropped), 0)
                    == sizeof(dropped)) {
                    consumed++;
                    n_trim_drops++;
                } else {
                    s_trim_owed += TRIM_ONE_FRAME;  /* still owed; retry next pass */
                }
            } else if (!cu && shift < 0) {
                /* Duplicate one frame into the slot the shorter read left.
                 * Zero-order hold on a single sample, at 0.6 Hz in normal
                 * service. */
                memcpy(chunk + want, chunk + want - frame_bytes, frame_bytes);
                n_trim_dups++;
            }
            /* Before measuring anything against the master clock, make sure the
             * conversion still describes it. */
            track_offset();

            /* Has playback reached a recorded packet boundary? If so, compare
             * now against when that sample was due. */
            bool timeline_changed = false;
            while (phase_tail != phase_head && samples_played >= phase_q[phase_tail].pos) {
                int64_t due = phase_q[phase_tail].play_at;
                /*
                 * Dated by where the crossing HAPPENED, not where the loop
                 * noticed it. samples_played moves AUDIO_FRAMES at a time --
                 * 5.8 ms at 44.1 kHz -- so the crossing is up to a chunk in the
                 * past by an amount that depends on where pos falls on the
                 * chunk grid, which is uncorrelated noise straight into the
                 * servo's only input. The overshoot is known, and writes are
                 * paced by the DAC, so the correction is exact rather than a
                 * filter. See the hub's copy for what this cost there, and for
                 * the sub-microsecond error the fine rate trim adds by leaving
                 * an input-frame count converted at the output rate.
                 */
                int32_t overshoot = samples_played - phase_q[phase_tail].pos;
                /* Capped at one chunk: a splice steps samples_played by up to
                 * MAX_SPLICE_MS at once and those frames were skipped rather
                 * than played, so anything the jump carried us past cannot be
                 * dated this way. A dropping pass steps it by AUDIO_FRAMES + 1
                 * and is clipped by one frame, worth 23 us. Same guard as the
                 * hub. */
                if (overshoot > (int32_t)AUDIO_FRAMES) {
                    overshoot = (int32_t)AUDIO_FRAMES;
                }
                /* Dated from when the DAC last took a chunk, not from a clock
                 * read here -- the write is the only DAC-paced event in this
                 * loop, and everything between it and this line is unpaced.
                 * See the hub's copy, which is where that mattered. */
                /* The crossing instant on THIS unit's clock. now_master is the
                 * same instant converted; the local form is what dates the
                 * crossing against a retune, which is a local event. */
                int64_t crossed_at = wrote_at
                                   - (int64_t)overshoot * 1000000 / stream_rate;
                int64_t now_master = crossed_at + stream_offset;
                int64_t err = now_master - due;
                /*
                 * Seconds of error is not drift and not jitter. It means the
                 * stamps are being issued against a different clock origin than
                 * the one playback anchored to -- the hub rebooted or was
                 * reflashed while this unit kept playing -- and no servo can
                 * correct that, because there is nothing wrong with the rate.
                 *
                 * It used to be cast straight into an int32: an hour of error
                 * wrapped to -699 seconds, the smoothing overflowed on top of
                 * it, and the servo asked for a 4.29 GHz sample rate, which
                 * aborted the board. Re-anchor instead, which is the one action
                 * that actually fixes it -- it re-reads the offset against the
                 * clock the hub is really using now.
                 */
                if (err > PHASE_INSANE_US || err < -PHASE_INSANE_US) {
                    ESP_LOGE(TAG, "phase %lld us -- not the timeline we anchored to, "
                                  "re-anchoring", err);
                    timeline_changed = true;
                    break;
                }
                /* The first reading after a retune is a transient -- logged,
                 * then thrown away rather than handed to the servo. See the
                 * hub's copy for what the outage figure actually covers and
                 * for the bench numbers that came off this unit. */
                const int64_t since_retune = retune_done_at
                                           ? crossed_at - retune_done_at : -1;
                if (retune_watch) {
                    retune_watch = false;
                    ESP_LOGW(TAG, "RETUNE COST: phase %+ld -> %+lld us (net %+lld), "
                                  "channel was down %lld us -- withheld from the "
                                  "servo, crossed %lld us after the retune",
                             (long)retune_phase_before, err, err - retune_phase_before,
                             retune_outage_us, since_retune);
                } else {
                    /*
                     * Narrated but NOT withheld -- these reach the servo exactly
                     * as they did before, so behaviour is unchanged and only the
                     * log says more. Whether they SHOULD be withheld is the
                     * question these lines exist to answer.
                     */
                    if (retune_tail_left) {
                        retune_tail_left--;
                        ESP_LOGW(TAG, "RETUNE TAIL: phase %+lld us at %lld us after "
                                      "the retune (net %+lld from before it)",
                                 err, since_retune, err - retune_phase_before);
                    }
                    /*
                     * A STEP, RECORDED here and narrated by drift_task.
                     *
                     * The soak that found the delivery-burst fault
                     * (tools/soak/logs-soak-20260815-224002) had to be
                     * reconstructed from 20 s status samples, which is how a
                     * +12 ms reading and a +268 ms one came to sit in adjacent
                     * lines with nothing between them. A step is a
                     * packet-cadence event -- ~50/s -- and nothing recorded it
                     * at that resolution.
                     *
                     * IT MUST NOT LOG FROM HERE, and the first version of this
                     * did. See the note on resync_request above, which says the
                     * same thing about the same task: this is the audio path.
                     * This console is 115200 baud, so a ~140-character line is
                     * ~12 ms of blocking UART against a 5.8 ms chunk and 34.8 ms
                     * of DMA -- and the steps arrive in bursts, three inside one
                     * millisecond in the 0116 soak, which is more than the whole
                     * DMA depth. It starved the DAC, and starvation causes short
                     * reads, which cause steps, which logged again. It was
                     * audible.
                     *
                     * So: store, and let the 5 s window print it, exactly as
                     * splice_report_* is written here and sent by the probe
                     * task. Only the largest step per window survives, which is
                     * the one worth seeing.
                     *
                     * Ring depth and the short-read count ride along because
                     * they are the two things that move a unit permanently
                     * later and neither is visible in a phase number: depth is
                     * lateness by definition, and a padded frame takes DAC time
                     * that samples_played does not count.
                     *
                     * In this arm only, so a reading the retune watch withheld
                     * does not become the baseline the next one is a step from.
                     */
                    if (s_phase_prev_valid) {
                        const int32_t step = (int32_t)err - s_phase_prev;
                        const int32_t mag = step < 0 ? -step : step;
                        if (mag > PHASE_STEP_LOG_US && mag > step_report_mag) {
                            step_report_mag  = mag;
                            step_report_from = s_phase_prev;
                            step_report_to   = (int32_t)err;
                            step_report_ring = (int32_t)((RING_BYTES -
                                xStreamBufferSpacesAvailable(ring)) * 1000
                                / (stream_rate * AUDIO_CHANNELS * 2));
                            step_report_pad  = n_short_frames;
                            step_report_trim = rate_trim_hz;
                            step_report_pending = true;
                        }
                    }
                    s_phase_prev = (int32_t)err;
                    s_phase_prev_valid = true;

                    phase_err_us = (int32_t)err;
                    phase_valid = true;
                    sync_phase_push(&phase_hist, (int32_t)err);
                }
                phase_tail = (phase_tail + 1) % PHASE_Q_LEN;
            }
            if (timeline_changed) {
                /* Same exit as an underrun: handle_audio() re-anchors on the
                 * next packet, which re-seeds stream_offset from a current
                 * estimate rather than one describing a hub that no longer
                 * exists. Drop the smoothing with it -- every sample in it was
                 * measured against the old origin. */
                stream_start_local = 0;
                phase_valid = false;
                phase_stepped = true;
                break;
            }

            /*
             * Track boundary reached: snap phase to zero rather than letting the
             * servo walk it off over ~45 s. Skipping or inserting audio is
             * inaudible here and nowhere else.
             */
            int32_t rp = restart_pos;
            if (rp >= 0 && samples_played >= rp) {
                restart_pos = -1;
                int32_t max_frames = (int32_t)stream_rate * MAX_SPLICE_MS / 1000;
                /*
                 * THE MEDIAN, not the newest reading. Was the shadow, is now the
                 * decision; the raw value is reported beside it.
                 *
                 * A splice on a single reading lands wherever that reading's
                 * noise put it, and the hub's carries ~15.7 ms of scatter. Two
                 * units each splicing on their own noisy sample therefore land in
                 * DIFFERENT places at the same boundary -- which is why a track
                 * change sometimes improved cross-unit sync and sometimes made it
                 * worse. This unit's readings are the quieter of the two, and
                 * that does not help: the divergence is set by the noisier one.
                 *
                 * LANDS WITH THE HUB, NOT BEFORE IT. hub_s3/main/play.c carries
                 * the same change in the same commit. One-sided is strictly worse
                 * than neither side, because it guarantees the two splice by
                 * different estimators.
                 *
                 * Same guard as the hub: phase_err_us survives a re-anchor, so a
                 * boundary reached before the first measurement of the new stream
                 * would splice on a number describing the old one. And the same
                 * fallback: with fewer than SYNC_PHASE_MIN readings no median is
                 * offered, so use the raw value rather than decline to splice.
                 */
                int32_t med_us = 0;
                const bool have_med = phase_valid &&
                                      sync_phase_median(&phase_hist, &med_us);
                const int32_t splice_us = have_med ? med_us : (int32_t)phase_err_us;

                int32_t adj = phase_valid
                    ? (int32_t)((int64_t)splice_us * stream_rate / 1000000) : 0;
                if (adj > max_frames)  adj = max_frames;
                if (adj < -max_frames) adj = -max_frames;
                int32_t applied = 0;      /* what the splice actually moved */

                /*
                 * The RAW value is the shadow now: what this unit would have
                 * corrected on the newest reading alone, clamped identically so
                 * it subtracts meaningfully against the hub's. Reported, acted on
                 * by nothing. See splice_msg_t.applied_alt_us.
                 */
                int32_t raw_adj = phase_valid
                    ? (int32_t)((int64_t)phase_err_us * stream_rate / 1000000) : 0;
                if (raw_adj > max_frames)  raw_adj = max_frames;
                if (raw_adj < -max_frames) raw_adj = -max_frames;
                splice_report_alt = (int32_t)((int64_t)raw_adj * 1000000 / stream_rate);

                if (adj > 0) {
                    /* Late: discard input so playback jumps forward in content.
                     * samples_played tracks input consumed, so it advances too.
                     *
                     * Into its own buffer, not into chunk: chunk is holding the
                     * audio read at the top of this pass, which has not been
                     * played yet. Discarding into it threw that away and played
                     * the tail of the skipped region in its place -- 5.8 ms of
                     * the wrong audio at every boundary. Nothing drifted, since
                     * both reads are counted, but it is not what the splice is
                     * supposed to do. */
                    static uint8_t discard[AUDIO_CHUNK_BYTES];
                    int32_t left = adj;
                    while (left > 0) {
                        size_t want = (size_t)(left > (int32_t)AUDIO_FRAMES ? AUDIO_FRAMES : left)
                                      * AUDIO_CHANNELS * sizeof(int16_t);
                        size_t got2 = xStreamBufferReceive(ring, discard, want, 0);
                        if (got2 == 0) break;
                        left -= got2 / (AUDIO_CHANNELS * sizeof(int16_t));
                    }
                    samples_played += (adj - left);
                    applied = adj - left;
                    ESP_LOGW(TAG, "track boundary: skipped %ld ms to null phase",
                             (long)(applied * 1000 / (int32_t)stream_rate));
                    phase_stepped = true;
                } else if (adj < 0) {
                    /*
                     * Early: emit silence so the timeline catches up with us.
                     *
                     * HEADROOM, since 2026-08-18: the zeros take DAC time and
                     * consume nothing from the ring, so receive keeps pushing
                     * while they play -- the insert is the one splice that can
                     * overflow the ring it is fixing. Clamp to what fits below
                     * capacity minus SPLICE_INSERT_HEADROOM_MS; whatever the
                     * clamp eats is left standing for the catch-up, which
                     * re-arms from the median this splice is about to reset.
                     * On the 2026-08-18 soak, 150 ms inserts into brimming
                     * rings took both satellites to the 464 ms ceiling and
                     * 121/89 decoded blocks were dropped at rx as ring-full.
                     */
                    const int32_t level_frames = (int32_t)((RING_BYTES -
                            xStreamBufferSpacesAvailable(ring))
                            / (AUDIO_CHANNELS * (int)sizeof(int16_t)));
                    const int32_t room = (int32_t)(RING_BYTES /
                            (AUDIO_CHANNELS * (int)sizeof(int16_t)))
                            - (int32_t)stream_rate * SPLICE_INSERT_HEADROOM_MS / 1000
                            - level_frames;
                    if (-adj > room) {
                        ESP_LOGW(TAG, "insert clamped: phase asked %ld ms, ring "
                                      "has room for %ld ms",
                                 (long)(-adj * 1000 / (int32_t)stream_rate),
                                 (long)((room > 0 ? room : 0) * 1000
                                        / (int32_t)stream_rate));
                        adj = room > 0 ? -room : 0;
                    }
                    static const uint8_t quiet[AUDIO_CHUNK_BYTES] = {0};
                    int32_t left = -adj;
                    while (left > 0) {
                        int32_t n = left > (int32_t)AUDIO_FRAMES ? (int32_t)AUDIO_FRAMES : left;
                        write_audio(quiet, (size_t)n * AUDIO_CHANNELS * sizeof(int16_t));
                        left -= n;
                    }
                    applied = adj;
                    ESP_LOGW(TAG, "track boundary: inserted %ld ms to null phase",
                             (long)(-applied * 1000 / (int32_t)stream_rate));
                    phase_stepped = true;
                }

                /*
                 * Hand the correction to the probe task to report. Not sent
                 * from here: a sendto() in the playback path is exactly the
                 * kind of thing that costs a buffer, and this is not urgent --
                 * the hub only wants it to print one line per track.
                 */
                if (applied != 0) {
                    n_splices++;
                    /* Same reason phase_stepped is set above: every reading in
                     * the history was taken before this unit moved. */
                    sync_phase_reset(&phase_hist);
                    /*
                     * THE SPLICE IS THE PAYER AND THE ONLY PAYER, since
                     * 2026-08-18. It moved this unit by `applied`, so any
                     * debt the servo armed against that same error is now
                     * double-counted -- on that soak, each capped 150 ms
                     * insert was followed by ~20 s of the pre-armed debt
                     * replaying on top. Zeroed here, the servo re-arms
                     * whatever error REMAINS once a fresh median exists:
                     * 150 of 250 ms paid by the splice, the rest re-armed
                     * for the drain. Mirrored in hub_s3/main/play.c.
                     */
                    catchup_frames = 0;
                }
                splice_report_us = (int32_t)((int64_t)applied * 1000000 / stream_rate);
                splice_report_phase = phase_valid ? phase_err_us : 0;
                splice_report_pending = true;
                /*
                 * The visualiser is not told either. A splice moves audio around
                 * WITHIN the timeline to correct this unit's position in it; the
                 * timeline itself, which is what every frame is dated against and
                 * drawn on, does not move. Arrival is untouched.
                 */
            }

            int32_t mark = marker_sample;
            if (mark >= 0 && samples_played >= mark) {
#if CONFIG_DANCEFLOOR_ENABLE_MARKER
                /* 200 us of busy-wait in the playback path. Worth it while a
                 * hub is wired to the other end and nothing else can measure
                 * what reaches the speaker; pure cost once the wire is off. */
                gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 1);
                esp_rom_delay_us(MARKER_PULSE_US);
                gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 0);
#endif
                marker_sample = -1;
            }
            /*
             * By what came OUT OF THE RING, not by the chunk size.
             *
             * samples_played is a position in the ring stream: it is compared
             * against phase_q[].pos, marker_sample and restart_pos, all of
             * which come from samples_in, which counts only frames actually
             * written to the ring. A short read's pad was never in the ring, so
             * advancing by a whole chunk regardless displaced samples_played
             * permanently against every position it is compared with -- the
             * same shape as the "silence inserted for a lost packet was not
             * counted in samples_in" bug that once put this unit ~20 ms out per
             * loss and stayed hidden because the marker came from the same
             * count.
             *
             * The pad does take DAC time, so the timing reference shifts by
             * that much for one pass. That is a one-off of at most a chunk
             * (5.8 ms) at the moment of the short read, against a displacement
             * that was permanent and cumulative.
             */
            samples_played += consumed;

            /* Last thing before the output, and deliberately after every count
             * above: it rewrites slots within frames that already exist, so
             * samples_played and the phase queue are looking at the same
             * timeline whatever this unit's speaker is placed as.
             *
             * Here rather than inside write_audio() because that is also called
             * with the const `quiet` buffer a splice inserts, which must not be
             * written to -- and needs nothing doing to it, since every mode
             * maps silence to silence. */
            audio_apply_channel_mode((int16_t *)chunk, AUDIO_FRAMES);
            /* Beside the channel mode and for the same reasons: in place, on
             * the last buffer before the output, frame count untouched. The hub
             * sends full scale and this level separately, and both units run the
             * same integer taper, so the two speakers match. Not inside
             * write_audio() because a splice calls that with the const `quiet`
             * buffer -- and silence needs no attenuating. */
            audio_apply_volume((int16_t *)chunk, AUDIO_FRAMES, vol_now());

            write_audio(chunk, sizeof(chunk));
            /* Immediately: it is the instant the next pass dates its phase
             * reading from. */
            wrote_at = esp_timer_get_time();
        }
        /* The inner loop only ends by parking -- underrun, resync, or a changed
         * timeline -- and from here until it starts again nothing is feeding the
         * DAC, so its running dry is expected rather than a fault. */
        playing = false;
    }
}
