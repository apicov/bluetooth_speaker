/**
 * @file play.c
 * @brief The playback timeline: hold the first sample for its instant, then
 *        pace the ring against the DAC, measuring and correcting position.
 *
 * @section shape The shape of play_task()
 *
 * An outer loop that parks until a stream is anchored, waits out the scheduled
 * start, and resets everything that described the previous stream. Then an
 * inner loop, one DMA chunk per pass:
 *
 * 1. Decide this pass's frame shift — the fine trim, plus any catch-up debt.
 * 2. Read that many frames from the ring, blocking with a timeout.
 * 3. Apply the shift: a crossfaded skip or replay for a catch-up, or a single
 *    dropped or duplicated frame for the trim alone.
 * 4. Cross any packet boundaries playback has now reached, and measure the
 *    position error at each.
 * 5. Splice at a track boundary, if one has come up.
 * 6. Write the chunk to the DAC.
 *
 * The inner loop only ends by parking: an underrun, a resync request, or a
 * timeline this unit is no longer on.
 *
 * @warning This task IS the audio path and must not log from inside the chunk
 * loop. At 115200 baud a status line is milliseconds of blocking UART against
 * a chunk that is 5.8 ms of audio. Everything worth saying is recorded into
 * the `step_report_*` and `splice_report_*` fields for telemetry.c to narrate.
 */

#include <string.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "audio_out.h"
#include "audio_shift.h"
#include "sbc_link.h"
#include "sync_proto.h"

#include "sat.h"

/**
 * @def TRIM_ONE_FRAME
 * @brief Fixed-point scale for @ref s_trim_owed — one whole frame.
 *
 * @ref rate_trim_hz names a rate the DAC is not running at, so playback has to
 * consume the ring at that rate instead — over one chunk of output that is
 * `AUDIO_FRAMES * rate_trim_hz / tx_rate` extra frames of input, a fraction
 * far below one. Accumulate it and spend a whole frame once one has been
 * earned.
 *
 * The scale is large enough that even at the trim's clamp one pass earns well
 * under a whole frame, so ONE frame per chunk is always enough headroom and no
 * pass ever needs two.
 *
 * Identical to the hub's, to the arithmetic: unequal correction between the
 * two units is a cross-unit sync error by construction.
 */
#define TRIM_ONE_FRAME 65536
/** @brief Frames the fine trim owes, in 1/@ref TRIM_ONE_FRAME of a frame. */
static int32_t s_trim_owed;

/** @brief The previous accepted phase reading, for the step detector. Play
 *  task only, and reset at every start: the first reading of a new stream is
 *  not a step from the last reading of the old one, and calling it one would
 *  put a step report under every re-anchor. */
static int32_t s_phase_prev;
/** @brief Whether @ref s_phase_prev holds a reading from this stream. */
static bool    s_phase_prev_valid;

/** @brief How far two consecutive readings must differ to be worth reporting.
 *  Clear of everything the system does on purpose — drift is well under a
 *  millisecond a minute and the servo deadband is a few milliseconds — so what
 *  trips it is a real discontinuity. A splice trips it too, which is correct:
 *  a splice IS a step, and it announces itself as well. */
#define PHASE_STEP_LOG_US 20000

/**
 * @brief Whether the fine trim owes a frame this pass.
 *
 * @return +1 to drop a frame, -1 to duplicate one, 0 for neither.
 */
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

    /* Bounded before it is spent, so a spell where the drop could not be taken
     * -- the ring was momentarily empty -- cannot wind up an arbitrary debt
     * and then pay it off in a burst of consecutive drops. */
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

/** @brief The catch-up read buffer. A pass that shifts k frames reads
 *  `AUDIO_FRAMES + k` in and crossfades it down to `AUDIO_FRAMES` out, reading
 *  both strands from the input while writing the output, so it cannot be done
 *  in place. Sized for the largest shift the constants allow, plus the fine
 *  trim folded in. */
static int16_t s_cu_in[(AUDIO_FRAMES + CATCHUP_SHIFT_MAX + 1) * AUDIO_CHANNELS];

/** @brief How much catch-up this pass's shift is carrying, set by
 *  @ref chunk_shift() and retired by the caller once the pass lands. Play task
 *  only. */
static int s_catchup_take;

/** @brief Frames of catch-up applied since @ref phase_hist was last aged. Each
 *  one moves this unit silently, so past a threshold every reading still in
 *  the history describes a unit that has moved and the history is dropped —
 *  the same call, for the same reason, as after a splice. */
static int32_t s_catchup_moved;

/**
 * @brief The frame shift for this pass: the fine trim, plus any armed
 *        catch-up debt while one is draining.
 *
 * @return Frames to shift by; positive skips, negative replays.
 *
 * While a debt is draining the trim is FOLDED INTO the shift rather than taken
 * separately, so one crossfaded pass carries both and the two mechanisms
 * cannot each take a frame out of the same chunk. Outside a drain the return
 * is the trim's 0/±1 and the plain paths handle it as they always do.
 *
 * The depth guard exists because debt and depth are the same quantity here:
 * ring level is @ref RING_TARGET_MS plus lateness, so draining a debt drains
 * the ring towards target, which is what a late unit wants. It only says no
 * when the ring lacks the margin the shift would spend — no dropping through a
 * ring already below target, which is a unit refilling after trouble rather
 * than a late one, and no inserting into one already past it.
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

/**
 * @brief The level to actually play this chunk at.
 *
 * @return 0-`AUDIO_VOL_MAX`.
 *
 * Kept out of the ring and out of every count, exactly like the level itself:
 * this decides what the DAC hears, and nothing upstream may be able to see it.
 * The deadline is a local of this task because this task is its only reader,
 * and it is measured from boot rather than from playback start.
 *
 * Says so once, loudly, if the fallback ever fires. A unit inventing its own
 * loudness is not a thing to discover by ear.
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

/* play_task() is documented at its declaration in sat.h. */
void play_task(void *arg)
{
    (void)arg;
    static uint8_t chunk[AUDIO_CHUNK_BYTES];

    while (1) {
        if (stream_start_local == 0) {
            /* Cleared here, in the park, rather than at its producer: the
             * receive task resets the ring on a resync, and doing that while
             * this task is blocked on a read is what the handshake avoids. */
            resync_request = false;
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        /* Hold the first sample until its moment arrives: sleep the bulk of
         * the wait, then spin the last two milliseconds, because a tick is
         * coarser than the accuracy this start is worth. After this, I2S paces
         * everything. */
        int64_t wait = stream_start_local - esp_timer_get_time();
        if (wait > 2000) {
            vTaskDelay(pdMS_TO_TICKS((wait - 2000) / 1000));
        }
        while (esp_timer_get_time() < stream_start_local) {
        }
        int64_t actual_master = esp_timer_get_time() + stream_offset;
        int64_t sched_master = stream_start_local + stream_offset;
        ESP_LOGI(TAG, "playback started: scheduled %lld, actual %lld (%+lld us) [master]",
                 sched_master, actual_master, actual_master - sched_master);

        /* NOTE: samples_in is deliberately NOT reset here. The receive task
         * owns it and has been filling the ring since the anchor; zeroing it
         * would put every recorded position behind the audio already queued. */
        int32_t samples_played = 0;

        /* The fraction of a frame owed described the stream this anchor
         * replaces. */
        s_trim_owed = 0;

        /* Every reading in the history was measured against that stream too. */
        sync_phase_reset(&phase_hist);
        phase_med_valid = false;

        /* Same reason: an armed debt described a position on the old stream. */
        catchup_frames = 0;

        /* And so the first reading of this stream is not called a step from
         * the last reading of the previous one. */
        s_phase_prev_valid = false;

        /* Arm the refill instrument: the first writes after an idle channel
         * return at memory speed, so readings dated inside that window are not
         * measured against a DAC that was pacing them. */
        s_refill_active = true;
        s_refill_frames = 0;

        /* When the DAC last accepted a chunk -- what a phase reading is dated
         * from. */
        int64_t wrote_at = esp_timer_get_time();

        /* Out of silence, not out of whatever the last stream ended at. */
        write_audio_reset_ramp();

        /* Something is now meant to be keeping the DAC fed, which is the only
         * condition under which a starved channel is a fault. */
        playing = true;

        while (1) {
            if (retuning) {
                /* i2s_channel_write() returns immediately on a disabled
                 * channel, so without this park the task would spin through
                 * the ring at memory speed for the whole outage. */
                vTaskDelay(pdMS_TO_TICKS(2));
                continue;
            }

            if (resync_request) {
                resync_request = false;
                stream_start_local = 0;
                phase_valid = false;
                phase_stepped = true;
                break;
            }

            const int shift = chunk_shift();
            const bool cu = (shift > 1 || shift < -1);
            const size_t frame_bytes = AUDIO_CHANNELS * sizeof(int16_t);
            const size_t want = cu
                ? (size_t)((int)AUDIO_FRAMES + shift) * frame_bytes
                : (shift < 0) ? sizeof(chunk) - frame_bytes : sizeof(chunk);

            hw_play = uxTaskGetStackHighWaterMark(NULL);  /* only valid in-task */

            /* Taken BEFORE the read, which is the whole point: after it, the
             * shallowest the ring ever got has already been drained away and
             * the sample would never show a near-underrun. */
            {
                const int32_t level_ms = (int32_t)((int64_t)(RING_BYTES -
                        (int64_t)xStreamBufferSpacesAvailable(ring)) * 1000
                        / ((int64_t)stream_rate * AUDIO_CHANNELS * 2));
                if (level_ms < ring_low_ms) {
                    ring_low_ms = level_ms;
                }
            }
            uint8_t *const dest = cu ? (uint8_t *)s_cu_in : chunk;
            size_t got = xStreamBufferReceive(ring, dest, want, pdMS_TO_TICKS(500));
            if (got == 0) {
                /* Leave by the same door a resync does, so the anchor path
                 * owns the ring reset. */
                n_underruns++;
                ESP_LOGW(TAG, "underrun, waiting for a new stream");
                stream_start_local = 0;
                break;
            }

            /* Measured against `want`, not the chunk size: a catch-up pass
             * legitimately asks for more or less than a chunk. */
            int32_t consumed = (int32_t)(got / frame_bytes);
            if (got < want) {
                n_short_reads++;
                n_short_frames += (uint32_t)((want - got) / frame_bytes);
            }

            size_t have = got;
            if (cu && got == want) {

                audio_shift_chunk((int16_t *)chunk, s_cu_in, AUDIO_FRAMES,
                                  shift, CATCHUP_FADE_FRAMES, AUDIO_CHANNELS);
                consumed = (int32_t)AUDIO_FRAMES + shift;
                have = sizeof(chunk);
                catchup_frames -= s_catchup_take;
                if (shift > 0) n_catchup_drops += (uint32_t)shift;
                else           n_catchup_dups  += (uint32_t)(-shift);

                s_catchup_moved += s_catchup_take < 0 ? -s_catchup_take
                                                      : s_catchup_take;
                if (s_catchup_moved >= (int32_t)((int64_t)CATCHUP_HIST_RESET_US *
                                                 stream_rate / 1000000)) {
                    s_catchup_moved = 0;
                    sync_phase_reset(&phase_hist);
                    phase_med_valid = false;
                    phase_stepped = true;
                }
            } else if (cu) {
                /* A short catch-up read: copy plainly and leave the debt
                 * standing. Crossfading a partial chunk would spend the debt
                 * against audio that is not there. */
                const size_t copy = got < sizeof(chunk) ? got : sizeof(chunk);
                memcpy(chunk, s_cu_in, copy);
                have = copy;
            }
            if (have < sizeof(chunk)) {
                memset(chunk + have, 0, sizeof(chunk) - have);
            }

            if (!cu && shift > 0) {
                /* A second, non-blocking read of one frame rather than one
                 * oversized read: this pass has already committed to a whole
                 * chunk, and blocking here would turn a trim into a stall. If
                 * the ring cannot spare it, refund the debt. */
                static uint8_t dropped[AUDIO_CHANNELS * sizeof(int16_t)];
                if (xStreamBufferReceive(ring, dropped, sizeof(dropped), 0)
                    == sizeof(dropped)) {
                    consumed++;
                    n_trim_drops++;
                } else {
                    s_trim_owed += TRIM_ONE_FRAME;
                }
            } else if (!cu && shift < 0) {
                /* Duplicate one frame into the slot the shorter read left. */
                memcpy(chunk + want, chunk + want - frame_bytes, frame_bytes);
                n_trim_dups++;
            }

            /* Refresh the conversion before measuring anything against it. */
            track_offset();

            bool timeline_changed = false;
            /* Cross every packet boundary playback has now reached, and
             * measure position at each: where we are, against where the
             * timeline says we should be. */
            while (phase_tail != phase_head && samples_played >= phase_q[phase_tail].pos) {
                int64_t due = phase_q[phase_tail].play_at;

                /* Date the crossing by where it actually happened. Playback
                 * advances a chunk at a time, so the boundary was passed
                 * somewhere inside the last one and `wrote_at` is that chunk's
                 * end. */
                int32_t overshoot = samples_played - phase_q[phase_tail].pos;

                /* Capped at one chunk: a larger overshoot means several
                 * boundaries were crossed in one pass, and only the last chunk
                 * is what `wrote_at` dates. */
                if (overshoot > (int32_t)AUDIO_FRAMES) {
                    overshoot = (int32_t)AUDIO_FRAMES;
                }

                int64_t crossed_at = wrote_at
                                   - (int64_t)overshoot * 1000000 / stream_rate;
                int64_t now_master = crossed_at + stream_offset;
                int64_t err = now_master - due;

                /* Not our timeline any more -- re-anchor rather than servo on
                 * it. @see PHASE_INSANE_US */
                if (err > PHASE_INSANE_US || err < -PHASE_INSANE_US) {
                    ESP_LOGE(TAG, "phase %lld us -- not the timeline we anchored to, "
                                  "re-anchoring", err);
                    timeline_changed = true;
                    break;
                }

                const int64_t since_retune = retune_done_at
                                           ? crossed_at - retune_done_at : -1;
                /* The reading immediately after a retune is withheld from the
                 * servo: it carries the channel-down outage, which the servo
                 * would otherwise correct as though it were drift, injecting
                 * exactly what the next retune has to undo. */
                if (retune_watch) {
                    retune_watch = false;
                    ESP_LOGW(TAG, "RETUNE COST: phase %+ld -> %+lld us (net %+lld), "
                                  "channel was down %lld us -- withheld from the "
                                  "servo, crossed %lld us after the retune",
                             (long)retune_phase_before, err, err - retune_phase_before,
                             retune_outage_us, since_retune);
                } else {

                    /* Narrated but NOT withheld: these say how far a retune's
                     * disturbance actually reaches, which is what would size a
                     * settle window if one were ever added. */
                    if (retune_tail_left) {
                        retune_tail_left--;
                        ESP_LOGW(TAG, "RETUNE TAIL: phase %+lld us at %lld us after "
                                      "the retune (net %+lld from before it)",
                                 err, since_retune, err - retune_phase_before);
                    }

                    if (s_phase_prev_valid) {
                        const int32_t step = (int32_t)err - s_phase_prev;
                        const int32_t mag = step < 0 ? -step : step;
                        /* Largest per window only, and recorded rather than
                         * logged. Ring depth and the pad count ride along
                         * because a step with an empty ring and a step with a
                         * full one are different faults. */
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

                    /* Publish the median for the servo, which runs on another
                     * task and must not touch phase_hist -- that history is
                     * play-task-only and is reset under this task's feet. */
                    int32_t med;
                    if (sync_phase_median(&phase_hist, &med)) {
                        phase_med_us = med;
                        phase_med_valid = true;
                    }
                }
                phase_tail = (phase_tail + 1) % PHASE_Q_LEN;
            }
            if (timeline_changed) {

                stream_start_local = 0;
                phase_valid = false;
                phase_stepped = true;
                break;
            }

            /* A track boundary is the one place a correction this large is
             * inaudible: the audio either side of it is unrelated anyway. */
            int32_t rp = restart_pos;
            if (rp >= 0 && samples_played >= rp) {
                restart_pos = -1;
                int32_t max_frames = (int32_t)stream_rate * MAX_SPLICE_MS / 1000;

                /* THE MEDIAN, not the newest reading. This is the largest
                 * single move the unit makes, so one noisy sample must not
                 * size it; the raw value stands in when the history is too
                 * short. */
                int32_t med_us = 0;
                const bool have_med = phase_valid &&
                                      sync_phase_median(&phase_hist, &med_us);
                const int32_t splice_us = have_med ? med_us : (int32_t)phase_err_us;

                int32_t adj = phase_valid
                    ? (int32_t)((int64_t)splice_us * stream_rate / 1000000) : 0;
                if (adj > max_frames)  adj = max_frames;
                if (adj < -max_frames) adj = -max_frames;
                int32_t applied = 0;

                /* The counterfactual, reported and acted on by nothing: what
                 * the raw reading would have spliced by, so the hub can print
                 * both at the same boundary. */
                int32_t raw_adj = phase_valid
                    ? (int32_t)((int64_t)phase_err_us * stream_rate / 1000000) : 0;
                if (raw_adj > max_frames)  raw_adj = max_frames;
                if (raw_adj < -max_frames) raw_adj = -max_frames;
                splice_report_alt = (int32_t)((int64_t)raw_adj * 1000000 / stream_rate);

                if (adj > 0) {
                    /* Playing late: throw audio away. Into its OWN buffer, not
                     * into `chunk` -- discarding through the chunk about to be
                     * written would play the tail of the skipped region. */
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
                    /* Playing early: insert silence. It takes DAC time and
                     * consumes nothing from the ring, so the receive path
                     * keeps pushing while it plays -- hence the headroom
                     * clamp. @see SPLICE_INSERT_HEADROOM_MS */
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

                    /* write_audio() takes a FRAME count, so the silence goes
                     * out in frames and the byte-width trap does not arise.
                     * `quiet` is const, so it also cannot be modified in place
                     * the way the live chunk is. */
                    static const int16_t quiet[AUDIO_FRAMES * AUDIO_CHANNELS] = {0};
                    _Static_assert(sizeof(quiet) == AUDIO_CHUNK_BYTES,
                                   "the splice's silence is one ring-domain chunk");
                    int32_t left = -adj;
                    while (left > 0) {
                        int32_t n = left > (int32_t)AUDIO_FRAMES ? (int32_t)AUDIO_FRAMES : left;

                        write_audio(quiet, (size_t)n, AUDIO_VOL_MAX);
                        left -= n;
                    }
                    applied = adj;
                    ESP_LOGW(TAG, "track boundary: inserted %ld ms to null phase",
                             (long)(-applied * 1000 / (int32_t)stream_rate));
                    phase_stepped = true;
                }

                if (applied != 0) {
                    n_splices++;
                    /* The unit has just moved, so every reading in the history
                     * describes where it used to be. */
                    sync_phase_reset(&phase_hist);
                    phase_med_valid = false;
                    /* And the splice is the payer: an armed debt would replay
                     * the same correction on top of the one just applied. */
                    catchup_frames = 0;
                }
                /* Recorded for probe_task to send. No sendto() from the audio
                 * path. */
                splice_report_us = (int32_t)((int64_t)applied * 1000000 / stream_rate);
                splice_report_phase = phase_valid ? phase_err_us : 0;
                splice_report_pending = true;

            }

            int32_t mark = marker_sample;
            if (mark >= 0 && samples_played >= mark) {
#if CONFIG_DANCEFLOOR_ENABLE_MARKER
                /* A bench instrument: nothing corrects on it. The busy-wait is
                 * a knowing cost, paid only on a build that asked for it. */
                gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 1);
                esp_rom_delay_us(MARKER_PULSE_US);
                gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 0);
#endif
                marker_sample = -1;
            }

            /* By what came OUT of the ring, not by a whole chunk. A short
             * read that was padded played audio that was never in the ring, so
             * advancing by the chunk size would displace every later phase
             * point and put a permanent bias in the servo's only input. */
            samples_played += consumed;

            /* Outside write_audio(), because it works IN PLACE and the
             * splice's silence buffer above is const. */
            audio_apply_channel_mode((int16_t *)chunk, AUDIO_FRAMES);

            write_audio((const int16_t *)chunk, AUDIO_FRAMES, vol_now());

            wrote_at = esp_timer_get_time();
        }

        /* Nothing is feeding the DAC now, so a dry channel from here on is
         * expected rather than a fault. */
        playing = false;
    }
}
