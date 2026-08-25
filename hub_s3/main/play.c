/**
 * @file play.c
 * @brief This unit's own speaker.
 *
 * The hub delays its own audio by LEAD_US exactly as a satellite does --
 * otherwise it would play ahead of every other speaker on the floor. Same
 * shape as the satellite's play task, minus the clock conversion: here master
 * time IS local time, so the phase reading is a direct measure of how far this
 * unit has slipped from the timeline it is itself publishing.
 *
 * One function per decision, in the order local_play_task() takes them: park
 * for a retune, read a chunk, absorb the phase crossings it passed, splice at
 * a track boundary, pulse the bench marker, and write to the DAC.
 */
#include "hub.h"
#include "audio_shift.h"

/**
 * @brief Position in the ring stream: frames handed to the DAC since the
 *        timeline started.
 *
 * Compared against s_phase_q[].pos, s_marker_sample and s_restart_pos, all of
 * which are positions in the same stream. Play-task-only, like the two below,
 * which is why they are statics here rather than declarations in hub.h.
 */
static int32_t s_samples_played;
/** @brief When the DAC last accepted a chunk -- the only DAC-paced instant in
 *         the loop, and what every phase reading is dated from. */
static int64_t s_wrote_at;

/**
 * @brief The chunk in flight, at file scope and deliberately not a parameter.
 *
 * read_chunk() and write_chunk() both ask it `sizeof(chunk)`, and that has to
 * keep meaning AUDIO_CHUNK_BYTES. Passed as `uint8_t *` it would silently mean
 * the size of the pointer, so the ring would be read four bytes at a time and
 * the DAC written four bytes at a time. Array-to-pointer decay is invisible at
 * the call and draws no warning, so the fix is to leave nothing to decay.
 */
static uint8_t chunk[AUDIO_CHUNK_BYTES];

/** @brief The timeline generation this task last started on, compared against
 *         local_epoch to decide whether a new origin has been published. */
static uint32_t s_play_epoch;

/** @brief What read_chunk() found in the ring. */
typedef enum {
    CHUNK_OK,        /**< A chunk is in the buffer; play it. */
    CHUNK_UNDERRUN,  /**< The ring ran dry; the timeline restarts. */
} chunk_result_t;

/**
 * @brief Park while the output clock is being retuned.
 *
 * @return true if this pass should be abandoned.
 */
static bool park_for_retune(void)
{
    if (retuning) {
        /* Do not pull from the ring while the channel is down -- writes would
         * return instantly and drain it. */
        vTaskDelay(pdMS_TO_TICKS(2));
        return true;
    }
    return false;
}

/**
 * @brief Fixed-point unit of the trim accumulator: one whole frame.
 *
 * rate_trim_hz names a rate the DAC is not running at, so playback consumes
 * the ring at that rate instead: over one chunk of output that is
 * AUDIO_FRAMES * rate_trim_hz / tx_rate extra frames of input, a fraction far
 * below one. It is accumulated in these units and spent when a whole frame has
 * been earned. At the RATE_TRIM_MAX_HZ clamp the per-chunk step is still under
 * one unit, so one frame per pass is always enough headroom and no pass ever
 * needs two.
 */
#define TRIM_ONE_FRAME 65536
/** @brief Fraction of a frame the fine trim owes, in 1/TRIM_ONE_FRAME. */
static int32_t s_trim_owed;

/**
 * @brief Bank this chunk's share of the fine trim and spend a whole frame if
 *        one has come due.
 *
 * @return +1 to drop a frame this pass, -1 to duplicate one, 0 for neither.
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

/** @brief Read buffer for a catch-up pass: one chunk plus the largest shift
 *         audio_shift_chunk() may be asked to cross. */
static int16_t s_cu_in[(AUDIO_FRAMES + CATCHUP_SHIFT_MAX + 1) * AUDIO_CHANNELS];
/** @brief Frames of catchup_frames this pass is taking, retired once the
 *         crossfade has actually happened. */
static int s_catchup_take;
/** @brief Frames the drain has moved this unit since the phase history was
 *         last reset. See CATCHUP_HIST_RESET_US. */
static int32_t s_catchup_moved;

/**
 * @brief The shift for this pass: the fine trim's 0/+-1, plus the armed
 *        catch-up debt while one is draining.
 *
 * The trim is folded in so that one crossfaded pass carries both. The depth
 * guard is against the local ring, whose target here is the LEAD this unit
 * publishes -- the ring IS the lead, held -- so a unit legitimately late holds
 * more than target. The guard only refuses when the ring lacks the margin the
 * shift would spend.
 *
 * @return Frames to add to (negative) or take from (positive) this pass's read.
 */
static int chunk_shift(void)
{
    s_catchup_take = 0;
    const int trim = trim_due();

    const int32_t debt = catchup_frames;
    if (debt == 0) {
        return trim;
    }

    const int32_t level_ms = (int32_t)((int64_t)(LOCAL_RING_BYTES -
            (int64_t)xStreamBufferSpacesAvailable(local_ring)) * 1000
            / ((int64_t)sample_rate * AUDIO_CHANNELS * 2));
    if (debt > 0 && level_ms <= (int32_t)(LEAD_US / 1000) - 30) {
        return trim;    /* dropping now would push a thin ring at the DAC */
    }
    if (debt < 0 && level_ms >= (int32_t)(LEAD_US / 1000) + 50) {
        return trim;    /* inserting would stack onto an already-deep ring */
    }

    int32_t k = debt;
    if (k >  CATCHUP_SHIFT_MAX_DROP) k =  CATCHUP_SHIFT_MAX_DROP;
    if (k < -CATCHUP_SHIFT_MAX_DUP)  k = -CATCHUP_SHIFT_MAX_DUP;
    s_catchup_take = (int)k;
    return (int)k + trim;
}

/**
 * @brief Pull one chunk out of the ring, applying this pass's shift.
 *
 * @param[out] got_frames  Frames actually taken from the RING -- not the
 *                         padded chunk size, since s_samples_played must count
 *                         only audio that was in the ring.
 * @return CHUNK_OK, or CHUNK_UNDERRUN if the ring ran dry.
 */
static chunk_result_t read_chunk(uint32_t *got_frames)
{
    /*
     * The shift varies how much is read FROM THE RING; it never varies what is
     * written to the DAC. write_chunk() always hands over one chunk, which is
     * exactly one DMA descriptor -- see the dma_frame_num note in i2s_start().
     *
     * A catch-up pass (|shift| > 1) is the same statement at a larger size: it
     * reads AUDIO_FRAMES + shift in, crosses the two strands under the fade,
     * and still hands the DAC one chunk. audio_shift.c is shared with the
     * satellite, so the two units cannot correct at different rates -- which
     * would itself be a cross-unit sync error.
     */
    const int shift = chunk_shift();
    const bool cu = (shift > 1 || shift < -1);
    const size_t frame_bytes = AUDIO_CHANNELS * sizeof(int16_t);
    const size_t want = cu
        ? (size_t)((int)AUDIO_FRAMES + shift) * frame_bytes
        : (shift < 0) ? sizeof(chunk) - frame_bytes : sizeof(chunk);

    hw_play = uxTaskGetStackHighWaterMark(NULL);   /* only valid in-task */
    uint8_t *const dest = cu ? (uint8_t *)s_cu_in : chunk;
    size_t got = xStreamBufferReceive(local_ring, dest, want, pdMS_TO_TICKS(500));
    if (got == 0) {
        n_underruns++;
        ESP_LOGW(TAG, "local underrun, restarting timeline");
        /* local_start is NOT zeroed here: it has one owner and it is not this
         * task. s_underrun_recover is how the timeline is told, and parking is
         * handled by the epoch test in the outer loop. */
        s_underrun_recover = true;
        return CHUNK_UNDERRUN;
    }

    /*
     * A short read is padded to a full chunk so the DAC gets one, but
     * got_frames must NOT count the pad.
     *
     * s_samples_played is a position in the RING STREAM, compared against
     * positions derived from s_samples_in, which counts only frames actually
     * written to the ring. The pad was never in the ring, so advancing by a
     * whole chunk regardless would displace s_samples_played permanently
     * against every position it is compared with.
     *
     * The pad does take DAC time, so the timing reference shifts by that much
     * for one pass -- a one-off of at most a chunk, against a displacement
     * that would be permanent and cumulative. n_short_reads is what says
     * whether this fires at all.
     *
     * Measured against `want`, not sizeof(chunk): on a duplicating pass a read
     * of one frame less than the chunk is what was ASKED for and is not short.
     */
    *got_frames = (uint32_t)(got / frame_bytes);
    if (got < want) {
        n_short_reads++;
        n_short_frames += (uint32_t)((want - got) / frame_bytes);
    }

    size_t have = got;      /* bytes of real audio now sitting in chunk */
    if (cu && got == want) {
        /* The catch-up pass proper: shift, count the input exactly, retire the
         * debt it carried. The plain trim block below does not run -- this
         * pass's shift already includes whatever trim_due() asked for, and the
         * counters count the whole shift, so the TRIM line's frames/s
         * arithmetic keeps holding across a drain. */
        audio_shift_chunk((int16_t *)chunk, s_cu_in, AUDIO_FRAMES,
                          shift, CATCHUP_FADE_FRAMES, AUDIO_CHANNELS);
        *got_frames = (uint32_t)((int)AUDIO_FRAMES + shift);
        have = sizeof(chunk);
        catchup_frames -= s_catchup_take;
        if (shift > 0) n_catchup_drops += (uint32_t)shift;
        else           n_catchup_dups  += (uint32_t)(-shift);

        /* Age the history as the drain moves this unit: readings older than
         * CATCHUP_HIST_RESET_US worth of movement describe a position this
         * unit has left. The history is play-task-only here (the servo reads
         * the published median), so the reset and the invalidation of that
         * median belong together, exactly as the splice leaves them. */
        s_catchup_moved += s_catchup_take < 0 ? -s_catchup_take
                                              : s_catchup_take;
        if (s_catchup_moved >= (int32_t)((int64_t)CATCHUP_HIST_RESET_US *
                                         sample_rate / 1000000)) {
            s_catchup_moved = 0;
            sync_phase_reset(&s_phase_hist);
            s_phase_med_valid = false;
            s_phase_stepped = true;
        }
    } else if (cu) {
        /* Short of the crossfade's want: play what arrived, plainly. The debt
         * stands and is spent by a later pass -- spending it on a partial
         * buffer would crossfade into the pad. */
        const size_t copy = got < sizeof(chunk) ? got : sizeof(chunk);
        memcpy(chunk, s_cu_in, copy);
        have = copy;
    }
    if (have < sizeof(chunk)) {
        memset(chunk + have, 0, sizeof(chunk) - have);
    }

    if (!cu && shift > 0) {
        /*
         * Drop one frame: take it out of the ring and throw it away. A second
         * receive rather than one oversized read, so `chunk` stays exactly
         * AUDIO_CHUNK_BYTES and every sizeof(chunk) in this file stays
         * correct -- see the note on its declaration.
         *
         * Non-blocking, and credited only if it actually returned a frame: if
         * the ring is momentarily empty the trim waits for the next pass
         * rather than stalling the DAC for a correction worth a single frame.
         *
         * At the chunk boundary rather than mid-chunk, which is the same
         * thing: the ring is a byte stream and the boundary is an artefact of
         * how much is read at a time.
         */
        static uint8_t dropped[AUDIO_CHANNELS * sizeof(int16_t)];
        if (xStreamBufferReceive(local_ring, dropped, sizeof(dropped), 0)
            == sizeof(dropped)) {
            *got_frames += 1;
            n_trim_drops++;
        } else {
            s_trim_owed += TRIM_ONE_FRAME;   /* still owed; try again next pass */
        }
    } else if (!cu && shift < 0) {
        /* Duplicate one frame into the slot the short read deliberately left:
         * a zero-order hold on a single sample. */
        memcpy(chunk + want, chunk + want - frame_bytes, frame_bytes);
        n_trim_dups++;
    }
    return CHUNK_OK;
}

/**
 * @brief Drain the phase queue: every packet boundary playback has now passed.
 *
 * This is the servo's only input, and everything in it is about making one
 * reading trustworthy -- dated where the crossing happened rather than where
 * it was noticed, measured from the DAC rather than from a clock read here,
 * and withheld entirely when the reference is not DAC-paced.
 */
static void absorb_phase_crossings(void)
{
    /* Local time IS master time here, so this is a direct read of how far
     * playback has slipped from the published timeline. */
    while (s_phase_tail != s_phase_head && s_samples_played >= s_phase_q[s_phase_tail].pos) {

        /*
         * Correct for WHERE the crossing was noticed against where it
         * happened, which is most of this unit's phase noise.
         *
         * s_samples_played advances a whole chunk per iteration, so by the
         * time the loop sees it has passed `pos` it passed it up to a chunk
         * ago, by an amount that depends on where pos falls on the chunk grid
         * and is therefore uncorrelated sample to sample. Reading the clock
         * here would date the crossing at "when I noticed", and the difference
         * is pure quantisation noise on the servo's only input.
         *
         * The overshoot is known exactly, so this is arithmetic rather than a
         * filter: writes are paced by the DAC, so the instant s_samples_played
         * was `pos` is `overshoot / rate` ago.
         *
         * Strictly the overshoot is in INPUT frames while `rate` is the output
         * rate, and the fine trim is the difference between them. That error
         * is overshoot x trim -- microseconds at the RATE_TRIM_MAX_HZ clamp,
         * against a deadband in milliseconds. Deliberately not corrected:
         * converting it would mean tracking a second rate through this
         * arithmetic to move the answer by less than the clock's own
         * quantisation.
         */
        int32_t overshoot = s_samples_played - s_phase_q[s_phase_tail].pos;
        /*
         * Capped at one chunk, because beyond that the pacing assumption is
         * false. A splice advances s_samples_played by up to MAX_SPLICE_MS in
         * a single step, and those frames were discarded rather than played
         * over time, so any point the jump carried us past cannot be dated
         * this way. Capping leaves those readings no worse than they were
         * before this correction existed.
         *
         * A dropping pass advances it by AUDIO_FRAMES + 1, so the cap also
         * clips that by one frame -- a single frame, once per correction.
         */
        if (overshoot > AUDIO_FRAMES) {
            overshoot = AUDIO_FRAMES;
        }
        /*
         * Dated from when the DAC last took a chunk, not from a clock read
         * here.
         *
         * s_samples_played describes audio handed to the DAC by the write at
         * the END of the previous pass, and that write is the only DAC-paced
         * event in this loop -- the same pacing the overshoot correction above
         * rests on. Everything between it and this line is paced by nothing:
         * the ring receive, and whatever preemption a board also running a
         * SoftAP, SBC decode and the bridge SPI link hands out. Reading the
         * clock here would fold all of it into the measurement, uncorrelated
         * pass to pass.
         *
         * The satellite carries the same change for symmetry, but it is not
         * where the problem was: its load is a fraction of this one's.
         */
        const int64_t crossed_at = s_wrote_at
                                 - (int64_t)overshoot * 1000000 / sample_rate;
        const int32_t err = (int32_t)(crossed_at - s_phase_q[s_phase_tail].play_at);

        /*
         * The first reading after a retune is a transient, so it is logged and
         * thrown away rather than handed to the servo.
         *
         * What a retune costs is not what the outage figure suggests.
         * i2s_channel_disable() sets the channel state, then blocks on the
         * same binary semaphore i2s_channel_write() holds across its wait for
         * a DMA descriptor, and only calls handle->stop() after that -- so the
         * audio keeps playing for most of the reported outage. What stops is
         * this task: s_samples_played freezes while the DMA drains and real
         * time advances, and the next crossing reads that gap as position
         * error. It is not one; the buffer refills over the following writes
         * and the position comes back on its own.
         *
         * Left in, the servo takes each of those as a real error and trims the
         * rate for it, so every retune injects the disturbance the next one
         * would correct.
         *
         * Only the first crossing is withheld. If the transient outlasts it,
         * the REFILL line says so -- a reading taken before the refill
         * completes is the one to distrust.
         */
        const int64_t since_retune = s_retune_done_at
                                   ? crossed_at - s_retune_done_at : -1;
        if (s_retune_watch) {
            s_retune_watch = false;
            ESP_LOGW(TAG, "RETUNE COST: phase %+ld -> %+ld us (net %+ld), "
                          "channel was down %lld us -- withheld from the "
                          "servo, crossed %lld us after the retune",
                     (long)s_retune_phase_before, (long)err,
                     (long)(err - s_retune_phase_before),
                     s_retune_outage_us, since_retune);
        } else {
            /* Narrated but NOT withheld: these reach the servo exactly as they
             * did before, so the tail is measurable without changing what the
             * servo acts on. */
            if (s_retune_tail_left) {
                s_retune_tail_left--;
                ESP_LOGW(TAG, "RETUNE TAIL: phase %+ld us at %lld us after "
                              "the retune (net %+ld from before it)",
                         (long)err, since_retune,
                         (long)(err - s_retune_phase_before));
            }

            /*
             * Withheld while the DMA is still filling.
             *
             * i2s_channel_write() does not block while descriptors are free,
             * so on an empty channel the first writes return at memory speed:
             * s_samples_played advances by the whole DMA depth against an
             * s_wrote_at that has barely moved. Every reading dated inside
             * that window is measured against a reference the DAC is not
             * pacing, and is not a phase error at all.
             *
             * The channel is empty at every playback START -- the task was
             * parked, so it drained -- which is the case this guards. It is
             * the same mechanism the `retuning` park prevents mid-stream.
             *
             * The REFILL line still reports the window; this stops the servo
             * acting on what is inside it.
             */
            if (s_refill_active) {
                n_refill_withheld++;
            } else {
                s_phase_err_us = err;
                s_phase_valid = true;
                sync_phase_push(&s_phase_hist, err);

                /*
                 * Publish the median for the servo, which runs on
                 * ring_monitor_task and cannot read s_phase_hist -- that
                 * history is play-task-only and is reset under this task's
                 * feet at every splice.
                 *
                 * Computed here rather than in the servo because it needs the
                 * packet-cadence history: SYNC_PHASE_HIST readings span a
                 * fraction of a second, so this removes the scatter without
                 * adding lag the servo would have to wait out.
                 */
                int32_t med;
                if (sync_phase_median(&s_phase_hist, &med)) {
                    s_phase_med_us = med;
                    s_phase_med_valid = true;
                }
            }
        }
        s_phase_tail = (s_phase_tail + 1) % PHASE_Q_LEN;
    }
}

/**
 * @brief A track boundary: snap this unit's phase to zero rather than let the
 *        servo walk it off.
 *
 * The one moment a splice is inaudible, which is the whole reason the boundary
 * is taken from track metadata rather than picked by a timer.
 */
static void apply_track_boundary(void)
{
    int32_t rp = s_restart_pos;
    if (rp >= 0 && s_samples_played >= rp) {
        s_restart_pos = -1;
        int32_t max_frames = (int32_t)sample_rate * MAX_SPLICE_MS / 1000;
        /*
         * The MEDIAN, not the newest reading, with the raw value reported
         * beside it.
         *
         * The raw reading carries this unit's full phase scatter, and the
         * servo has smoothed its own input since it was caught triggering on
         * that noise. A splice taken on a single sample lands several ms wrong
         * in a direction nothing predicts, while the satellite -- a fraction
         * of the load and a fraction of the scatter -- lands closer. The two
         * would splice to DIFFERENT places, which is why this must land on
         * both units at once, and does: satellite/main/play.c carries the same
         * arithmetic.
         *
         * Falls back to the raw reading when the history is too short to offer
         * a median (fewer than SYNC_PHASE_MIN readings, i.e. a boundary soon
         * after a start or a previous splice). That is better than declining
         * to splice.
         *
         * s_phase_valid gates the whole thing: with nothing measured since the
         * last re-anchor, s_phase_err_us still describes the previous
         * timeline, and splicing on it would cut up to MAX_SPLICE_MS of real
         * audio to correct an error that no longer exists. The servo takes out
         * anything genuine instead.
         */
        int32_t med_us = 0;
        const bool have_med = s_phase_valid &&
                              sync_phase_median(&s_phase_hist, &med_us);
        const int32_t splice_us = have_med ? med_us : (int32_t)s_phase_err_us;

        int32_t adj = s_phase_valid
            ? (int32_t)((int64_t)splice_us * sample_rate / 1000000) : 0;
        if (adj > max_frames)  adj = max_frames;
        if (adj < -max_frames) adj = -max_frames;
        int32_t applied = 0;      /* what the splice actually moved */

        /* The counterfactual: what the correction would have been on the
         * newest reading alone. Reported on the same line, so the comparison
         * that chose the median survives the change and a revert has something
         * to check against. */
        int32_t raw_adj = s_phase_valid
            ? (int32_t)((int64_t)s_phase_err_us * sample_rate / 1000000) : 0;
        if (raw_adj > max_frames)  raw_adj = max_frames;
        if (raw_adj < -max_frames) raw_adj = -max_frames;
        s_hub_splice_alt_us = (int32_t)((int64_t)raw_adj * 1000000 / sample_rate);

        if (adj > 0) {
            /* Its own buffer, not `chunk`: chunk holds the audio read at the
             * top of this pass and not yet written to the DAC, so discarding
             * into it would drop that and play the tail of the skipped region
             * instead. Same buffer on the satellite. */
            static uint8_t discard[AUDIO_CHUNK_BYTES];
            int32_t left = adj;
            while (left > 0) {
                size_t want = (size_t)(left > AUDIO_FRAMES ? AUDIO_FRAMES : left)
                              * AUDIO_CHANNELS * sizeof(int16_t);
                size_t g = xStreamBufferReceive(local_ring, discard, want, 0);
                if (g == 0) break;
                left -= g / (AUDIO_CHANNELS * sizeof(int16_t));
            }
            s_samples_played += (adj - left);
            applied = adj - left;
            ESP_LOGW(TAG, "track boundary: skipped %ld ms to null phase",
                     (long)(applied * 1000 / (int32_t)sample_rate));
        } else if (adj < 0) {
            /*
             * Headroom. The zeros below take DAC time and consume nothing from
             * the ring, so streamer_feed() keeps pushing while they play --
             * the insert is the one splice that can overflow the ring it is
             * fixing. Clamped to what fits below capacity minus
             * SPLICE_INSERT_HEADROOM_MS; whatever the clamp eats is left
             * standing for the catch-up drain, which re-arms from the median
             * this splice is about to reset.
             */
            const int32_t level_frames = (int32_t)((LOCAL_RING_BYTES -
                    xStreamBufferSpacesAvailable(local_ring))
                    / (AUDIO_CHANNELS * (int)sizeof(int16_t)));
            const int32_t room = (int32_t)(LOCAL_RING_BYTES /
                    (AUDIO_CHANNELS * (int)sizeof(int16_t)))
                    - (int32_t)sample_rate * SPLICE_INSERT_HEADROOM_MS / 1000
                    - level_frames;
            if (-adj > room) {
                ESP_LOGW(TAG, "insert clamped: phase asked %ld ms, ring has "
                              "room for %ld ms",
                         (long)(-adj * 1000 / (int32_t)sample_rate),
                         (long)((room > 0 ? room : 0) * 1000
                                / (int32_t)sample_rate));
                adj = room > 0 ? -room : 0;
            }

            /* Sized in the DAC domain, because that is what it is written to.
             * An insert is a TIMING correction, so a byte count computed from
             * sizeof(int16_t) would silently halve every insert while sounding
             * exactly the same -- silence is silence at any width. The assert
             * is the guard, since no ear could be. */
            static const audio_out_sample_t quiet[AUDIO_FRAMES * AUDIO_CHANNELS] = {0};
            _Static_assert(sizeof(quiet) == AUDIO_OUT_CHUNK_BYTES,
                           "the splice's silence must be sized in output samples");
            int32_t left = -adj;
            size_t w = 0;
            while (left > 0) {
                int32_t n = left > AUDIO_FRAMES ? AUDIO_FRAMES : left;
                size_t bytes = (size_t)n * AUDIO_OUT_FRAME_BYTES;
                i2s_channel_write(i2s_tx, quiet, bytes, &w, portMAX_DELAY);
                left -= n;
            }
            applied = adj;
            ESP_LOGW(TAG, "track boundary: inserted %ld ms to null phase",
                     (long)(-applied * 1000 / (int32_t)sample_rate));
        }
        if (adj != 0) {
            n_splices++;
            s_phase_stepped = true;   /* the average before it is stale */
            /* And so is the splice's own history, for the same reason: every
             * reading in it was taken before this unit moved. */
            sync_phase_reset(&s_phase_hist);
            s_phase_med_valid = false;   /* it summarised the phase just removed */
            /*
             * The splice is the payer and the only payer. It moved this unit
             * by `applied`, so any debt the servo armed against that same
             * error is now double-counted -- the drain would replay it on top
             * of a correction already made. Zeroed here, the servo re-arms
             * whatever error REMAINS once a fresh median exists, which is the
             * capped-splice case handled right: part paid by the splice, the
             * rest re-armed for the drain.
             *
             * On `applied`, not `adj`: a skip that hit an empty ring paid
             * nothing, and the debt it encodes is still real.
             */
            if (applied != 0) {
                catchup_frames = 0;
            }
        }

        /*
         * One line per track for how far apart the speakers had drifted by the
         * end of it -- the figure to compare sessions and builds on, because
         * it is taken at the same point of every track cycle rather than
         * wherever a log window happened to fall.
         *
         * The satellite figure is the marker: a physical measurement of when a
         * sample reached the output, so it sees what no software reading can.
         * The hub's splice is how much of its own error it had accumulated.
         * They answer different questions and both belong here.
         */
        s_hub_splice_us = (int32_t)((int64_t)applied * 1000000 / sample_rate);
        s_hub_splice_at = esp_timer_get_time();

        if (s_sync_at) {
            ESP_LOGW(TAG, "TRACK DIVERGENCE: satellite %+lld us "
                          "(marker, %lld ms before this boundary) | "
                          "hub spliced %+ld ms | hub phase %+ld us "
                          "(median %+ld us; raw would have spliced %+ld ms)",
                     s_sync_err_us,
                     (s_hub_splice_at - s_sync_at) / 1000,
                     (long)(s_hub_splice_us / 1000),
                     (long)s_phase_err_us,
                     (long)med_us, (long)(s_hub_splice_alt_us / 1000));
        } else {
            /* No marker wire -- the normal deployed case, since it is a bench
             * instrument. Satellites report over WiFi instead, and their line
             * arrives within PROBE_PERIOD_MS of this one. See probe.c. */
            ESP_LOGW(TAG, "TRACK BOUNDARY: hub spliced %+ld ms | "
                          "hub phase %+ld us (median %+ld us; raw would "
                          "have spliced %+ld ms) | no marker fitted",
                     (long)(s_hub_splice_us / 1000),
                     (long)s_phase_err_us,
                     (long)med_us, (long)(s_hub_splice_alt_us / 1000));
        }
        /* The visualiser is not told. A splice moves audio WITHIN the timeline
         * to correct this unit's position in it; the timeline every frame is
         * dated against does not move, and arrival is untouched. */
    }
}

/** @brief Pulse the bench marker if playback has reached a tagged sample. */
static void pulse_marker(void)
{
    int32_t mark = s_marker_sample;
    if (mark >= 0 && s_samples_played >= mark) {
#if CONFIG_DANCEFLOOR_ENABLE_MARKER
        /* MARKER_PULSE_US of busy-wait in the playback path -- see the Kconfig
         * help. Nothing corrects on what it measures. */
        gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 1);
        s_marker_at = esp_timer_get_time();
        esp_rom_delay_us(MARKER_PULSE_US);
        gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 0);
#endif
        s_marker_sample = -1;
    }
}

/**
 * @brief The level to actually play this chunk with.
 *
 * Kept out of the ring and out of every count, exactly like the level itself:
 * this decides what the DAC hears, and nothing upstream may be able to see it.
 * The deadline is a local of this task because this task is its only reader --
 * no shared 64-bit field, so nothing to tear. It is measured from boot rather
 * than from playback start; audio_vol_effective() carries that argument.
 *
 * @return The 0..AUDIO_VOL_MAX level to hand audio_volume_write_i32().
 */
static uint8_t vol_now(void)
{
    /* Said once, loudly, if it ever fires: a unit inventing its own loudness
     * is not a thing to discover by ear. */
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

/** @brief The output fade-in, zeroed at every playback start so a stream comes
 *         up out of silence rather than on an edge. Playback-task only; see
 *         audio_ramp_t. */
static audio_ramp_t s_out_ramp;

/**
 * @brief Hand the chunk to the DAC, and measure the refill while doing it.
 *
 * The write is the only DAC-paced event in the whole loop, which is why the
 * instant after it is what every phase reading is dated from.
 */
static void write_chunk(void)
{
    /* Last thing before the DMA buffer, and deliberately after every count
     * above: it rewrites slots within frames that already exist, so
     * s_samples_played and the phase queue look at the same timeline whatever
     * this unit's speaker is placed as. */
    audio_apply_channel_mode((int16_t *)chunk, AUDIO_FRAMES);

    /* Widened to the DAC's 32 bits with the level folded in -- out of place,
     * into a static staging buffer, so the ring-domain chunk is left as it is
     * and the frame count is untouched. What went to the satellites was full
     * scale; each unit attenuates its own. audio_out.h has the exactness
     * argument and the reason the ring stays 16-bit. */
    static audio_out_sample_t out32[AUDIO_FRAMES * AUDIO_CHANNELS];
    audio_volume_write_i32(out32, (const int16_t *)chunk, AUDIO_FRAMES, vol_now(),
                           &s_out_ramp);
    size_t written = 0;
    const int64_t w0 = s_refill_active ? esp_timer_get_time() : 0;
    if (i2s_channel_write(i2s_tx, out32, sizeof(out32), &written,
                          portMAX_DELAY) != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (s_refill_active) {
        if (esp_timer_get_time() - w0 < REFILL_FAST_US) {
            s_refill_frames += (int32_t)(written / AUDIO_OUT_FRAME_BYTES);
        } else {
            s_refill_active = false;
            ESP_LOGW(TAG, "REFILL after start: %ld frames (%ld ms) before a "
                          "write blocked -- phase readings inside this "
                          "window are not DAC-paced",
                     (long)s_refill_frames,
                     (long)(s_refill_frames * 1000 / (int32_t)sample_rate));
        }
    }
    /* Immediately, and before anything else can delay this task: it is the
     * instant the next pass dates its phase reading from. */
    s_wrote_at = esp_timer_get_time();
}

/** @brief Reset the per-stream state a fresh timeline invalidates. */
static void begin_playback(void)
{
    /* Out of silence, not out of whatever the last stream ended at. */
    s_out_ramp.cur = 0;
    /* s_samples_played counts from the first sample played, which is the first
     * sample fed after the ring was reset at timeline start. Both counters
     * therefore share an origin -- do NOT reset s_samples_in here, it has
     * legitimately been counting the audio buffered during the wait. */
    s_samples_played = 0;
    /* The fraction of a frame owed described the stream that just ended.
     * rate_trim_hz itself is NOT reset: it is this unit's standing rate offset
     * against the source, exactly as tx_rate is, and survives a restart for
     * the same reason. */
    s_trim_owed = 0;
    /* Every reading in it was measured against the timeline this start
     * replaces, so none of them describes where this unit now is. */
    sync_phase_reset(&s_phase_hist);
    s_phase_med_valid = false;
    /* Same reason: an armed debt described a position on the timeline this
     * start replaces, and a replay debt draining into the freshly-reset thin
     * ring is an audible stretch. The servo re-arms from fresh readings if the
     * error survives the restart, and CATCHUP_HOLD_US gives those readings
     * time to be medians first. */
    catchup_frames = 0;
    /* The channel drained while this task was parked, so it is empty here for
     * the same reason it is empty after a disable. See s_refill_active. */
    s_refill_active = true;
    s_refill_frames = 0;
    /* Seeded so the first pass, before any write has happened, dates its phase
     * reading against a sane value rather than zero. */
    s_wrote_at = esp_timer_get_time();
}

void local_play_task(void *arg)
{
    (void)arg;

    while (1) {
        /*
         * Park until the timeline publishes a NEW origin.
         *
         * Waiting on the epoch rather than on `local_start == 0` leaves the
         * timeline as the only writer of both, and reading the epoch BEFORE
         * the instant is what makes the pair safe against a torn 64-bit read.
         * See hub.h.
         */
        if (local_epoch == s_play_epoch) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        s_play_epoch = local_epoch;
        const int64_t start_at = local_start;

        int64_t wait = start_at - esp_timer_get_time();
        if (wait > 2000) {
            vTaskDelay(pdMS_TO_TICKS((wait - 2000) / 1000));
        }
        while (esp_timer_get_time() < start_at) {
            /* spin the last stretch */
        }
        /* On the hub local time IS master time, so this is directly comparable
         * with the satellite's figure. A difference here is a difference in
         * the audio each unit is playing, which no amount of clock accuracy
         * fixes. */
        ESP_LOGI(TAG, "local playback started: scheduled %lld, actual %lld (%+lld us)",
                 start_at, esp_timer_get_time(), esp_timer_get_time() - start_at);
        s_playing = true;
        begin_playback();

        while (1) {
            uint32_t got_frames = 0;

            if (park_for_retune()) {
                continue;
            }
            if (read_chunk(&got_frames) == CHUNK_UNDERRUN) {
                break;
            }
            absorb_phase_crossings();
            apply_track_boundary();
            pulse_marker();
            /* Frames that came out of the ring, not the padded chunk size --
             * see the short-read note in read_chunk(). Equal to AUDIO_FRAMES
             * in every pass that did not come up short. */
            s_samples_played += (int32_t)got_frames;
            write_chunk();
        }
        /* The inner loop only ends on an underrun, and the servo should stop
         * treating this unit as playing the moment it does. */
        s_playing = false;
    }
}
