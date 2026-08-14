/*
 * This unit's own speaker.
 *
 * The hub delays its own audio by LEAD_US exactly like a satellite -- otherwise
 * it would play ahead of every other speaker on the floor. Same shape as the
 * satellite's play task, minus the clock conversion: here master time IS local
 * time, so the phase reading is a direct measure of how far this unit has
 * slipped from the timeline it is itself publishing.
 *
 * ONE FUNCTION PER DECISION. local_play_task was 363 lines in which the phase
 * queue, splice policy, the marker, the refill instrument and the DAC write
 * interleaved, so splice policy could not be read without reading the crossing
 * loop. The order those decisions are taken in is now the whole of the task; each
 * one is a function below, in the order it runs. Same treatment handle_audio got
 * on the satellite, and for the same reason.
 *
 * The bodies are unchanged. What changed is control flow, which the file split
 * deliberately did not touch: three loop locals became file statics because a
 * loop body became a function, and the underrun `break` became a typed result.
 */
#include "hub.h"

/*
 * Loop state that outlived its loop.
 *
 * These were locals of local_play_task's inner loop and have to persist across
 * the calls that loop body is now made of. They are NOT shared with another task
 * -- the play task is the only reader and the only writer, which is why they are
 * plain statics here rather than declarations in hub.h. Reset by begin_playback()
 * at every start, which is what a fresh loop entry used to do.
 */
static int32_t s_samples_played;   /* position in the ring stream */
static int64_t s_wrote_at;         /* when the DAC last accepted a chunk */
/*
 * The chunk in flight, at FILE SCOPE and deliberately not a parameter.
 *
 * Both bodies below ask it `sizeof(chunk)`, and that has to keep meaning
 * AUDIO_CHUNK_BYTES. Passed as `uint8_t *` it silently means 4 -- the pointer --
 * so the ring was read 4 bytes at a time and the DAC written 4 bytes at a time,
 * one frame per pass instead of 256. The bodies still read exactly as they did
 * inside the loop; only their meaning had changed, which is the one thing the
 * split promised not to do. Array-to-pointer decay is invisible at the call and
 * the compiler does not warn, so the fix is to leave nothing to decay.
 */
static uint8_t chunk[AUDIO_CHUNK_BYTES];
/* The timeline generation this task last started on. Compared against
 * local_epoch to decide whether a new origin has been published. */
static uint32_t s_play_epoch;

typedef enum {
    CHUNK_OK,        /* a chunk is in the buffer, play it */
    CHUNK_UNDERRUN,  /* the ring ran dry; the timeline restarts */
} chunk_result_t;


/*
 * Park while the output clock is being retuned, and note that we did.
 *
 * Returns true if this pass should be abandoned. The re-arm of the refill
 * instrument belongs here rather than at the top of the next pass because the
 * two facts -- "we parked" and "the channel is therefore empty" -- are the same
 * fact, and separating them is what let the START case go unguarded for so long.
 */
static bool park_for_retune(void)
{
    if (retuning) {
        /* Do not pull from the ring while the channel is down -- writes
         * would return instantly and drain it. */
        vTaskDelay(pdMS_TO_TICKS(2));
        return true;
    }
    /*
     * The RETUNE arm of the refill probe is RETIRED, 2026-08-12, on measurement.
     *
     * Re-arming here assumed the DMA is empty after a retune, on the reasoning
     * that i2s_channel_disable() discards the descriptors. It does not -- it
     * drains them. The satellite retired its copy of this probe on 25 of 26
     * samples reading `0 frames`, and the hub's 18:09 run read `0 frames` on all
     * three of its retunes independently. There is no refill window here to
     * measure or to withhold readings inside.
     *
     * The transient a retune DOES cause is a step, not a fill, and s_retune_watch
     * already withholds exactly one reading for it -- which the same run showed
     * is the right number, the tail being flat across ~70 ms rather than decaying.
     *
     * The START arm is untouched and is load-bearing: the channel really is empty
     * there, because this task was parked and it drained.
     */
    return false;
}


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
 * ever needs two. The clamp below is insurance against that stopping being
 * true, not something that fires.
 */
#define TRIM_ONE_FRAME 65536
static int32_t s_trim_owed;

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

/* Pull one chunk out of the ring, or report that it ran dry. */
static chunk_result_t read_chunk(uint32_t *got_frames)
{
    /*
     * The trim varies how much is read FROM THE RING; it never varies what is
     * written to the DAC. write_chunk() still hands over exactly sizeof(chunk),
     * which is exactly one DMA descriptor -- see the dma_frame_num note in
     * i2s_start(), where writing across two descriptors is most of why this
     * unit's retunes cost 2-18 ms against the satellite's 2-6.
     */
    const int trim = trim_due();
    const size_t frame_bytes = AUDIO_CHANNELS * sizeof(int16_t);
    const size_t want = (trim < 0) ? sizeof(chunk) - frame_bytes : sizeof(chunk);

    hw_play = uxTaskGetStackHighWaterMark(NULL);   /* only valid in-task */
    size_t got = xStreamBufferReceive(local_ring, chunk, want, pdMS_TO_TICKS(500));
    if (got == 0) {
        n_underruns++;
        ESP_LOGW(TAG, "local underrun, restarting timeline");
        /* Does NOT zero local_start any more: it has one owner now, and it is
         * not this task. s_underrun_recover is how the timeline is told, which it
         * always was; parking is handled by the epoch test in the outer loop,
         * which will not fire again until a new origin is published. */
        s_underrun_recover = true;
        return CHUNK_UNDERRUN;
    }
    /*
     * A short read is padded to a full chunk so the DAC gets one, but
     * samples_played must NOT count the pad.
     *
     * samples_played is a position in the RING STREAM: it is compared
     * against s_phase_q[].pos, s_marker_sample and s_restart_pos, all of
     * which come from s_samples_in, which counts only frames actually
     * written to the ring. The pad was never in the ring. Advancing by a
     * whole chunk regardless therefore displaced samples_played
     * permanently against every position it is compared with -- the same
     * shape as the "silence inserted for a lost packet was not counted in
     * samples_in" bug that once put a unit 20 ms out per loss.
     *
     * The pad does take DAC time, so the timing reference shifts by that
     * much for one pass. That is a one-off of at most a chunk (5.8 ms) at
     * the moment of the short read, against a displacement that was
     * permanent and cumulative. n_short_reads is what says whether this
     * fires at all; it has read 0 on every run so far.
     *
     * Measured against `want`, not sizeof(chunk): on a duplicating pass a
     * read of one frame less than the chunk is what was ASKED for and is not
     * short. The pad still fills to sizeof(chunk) because that is what the
     * DAC is given.
     */
    *got_frames = (uint32_t)(got / frame_bytes);
    if (got < want) {
        n_short_reads++;
        n_short_frames += (uint32_t)((want - got) / frame_bytes);
    }
    if (got < sizeof(chunk)) {
        memset(chunk + got, 0, sizeof(chunk) - got);
    }

    if (trim > 0) {
        /*
         * Drop one frame: take it out of the ring and throw it away. A second
         * receive rather than one oversized read, so that `chunk` stays exactly
         * AUDIO_CHUNK_BYTES and every sizeof(chunk) in this file stays correct
         * -- see the note on the declaration for what happened when it did not.
         *
         * Non-blocking, and credited only if it actually returned a frame: if
         * the ring is momentarily empty the trim simply waits for the next
         * pass rather than stalling the DAC for a correction worth 23 us.
         *
         * At the chunk boundary rather than mid-chunk, which is the same thing:
         * the ring is a byte stream and the boundary is an artefact of how much
         * is read at a time.
         */
        static uint8_t dropped[AUDIO_CHANNELS * sizeof(int16_t)];
        if (xStreamBufferReceive(local_ring, dropped, sizeof(dropped), 0)
            == sizeof(dropped)) {
            *got_frames += 1;
            n_trim_drops++;
        } else {
            s_trim_owed += TRIM_ONE_FRAME;   /* still owed; try again next pass */
        }
    } else if (trim < 0) {
        /* Duplicate one frame into the slot the short read deliberately left.
         * Zero-order hold on a single sample, at 0.6 Hz in normal service. */
        memcpy(chunk + want, chunk + want - frame_bytes, frame_bytes);
        n_trim_dups++;
    }
    return CHUNK_OK;
}


/*
 * Drain the phase queue: every packet boundary playback has now passed.
 *
 * This is the servo's only input, and everything in it is about making one
 * reading trustworthy -- where the crossing happened rather than where it was
 * noticed, dated from the DAC rather than from a clock read here, and withheld
 * entirely when the reference is not DAC-paced.
 */
static void absorb_phase_crossings(void)
{
    /* Local time IS master time here, so this is a direct read of how
     * far playback has slipped from the published timeline. */
    while (s_phase_tail != s_phase_head && s_samples_played >= s_phase_q[s_phase_tail].pos) {
        /*
         * Correct for WHERE the crossing was noticed versus where it
         * happened, which is most of this unit's phase noise.
         *
         * s_samples_played advances by AUDIO_FRAMES per iteration -- 5.8
         * ms at 44.1 kHz -- so by the time the loop sees it has passed
         * `pos`, it passed it up to a chunk ago, by an amount that
         * depends on where pos falls on the chunk grid and is therefore
         * uncorrelated sample to sample. Reading the clock here dates
         * the crossing at "when I noticed", and the difference is pure
         * quantisation noise on the servo's only input.
         *
         * Measured before this: two reads of s_phase_err_us in adjacent
         * log lines, a millisecond apart, differing by 15.7 ms. That
         * noise made the hub's own retune bench unmeasurable (scatter
         * 2.9x the effect), produced a false 23 ms alarm, and is the
         * "hub absolute phase does not settle" wart in clock-sync.md.
         *
         * The overshoot is known exactly, so this is arithmetic rather
         * than a filter: writes are paced by the DAC, so the instant
         * s_samples_played was `pos` is `overshoot / rate` ago.
         *
         * Strictly the overshoot is in INPUT frames while `rate` is the
         * output rate, and the fine trim is the difference between them.
         * The error is overshoot x trim: 13 us at the RATE_TRIM_MAX_HZ
         * clamp and 0.08 us at real drift, against a 7 ms deadband.
         * Deliberately not corrected -- converting it would mean tracking
         * a second rate through this arithmetic to move the answer by
         * less than the clock's own quantisation.
         */
        int32_t overshoot = s_samples_played - s_phase_q[s_phase_tail].pos;
        /*
         * Capped at one chunk, because beyond that the pacing
         * assumption is false. A splice advances s_samples_played by up
         * to MAX_SPLICE_MS in a single step and those frames were
         * discarded rather than played over time, so any point the jump
         * carried us past cannot be dated this way. Capping leaves
         * those readings no worse than they were before this
         * correction existed.
         *
         * A dropping pass advances it by AUDIO_FRAMES + 1, so the cap
         * also clips that by one frame. 23 us, once per correction.
         */
        if (overshoot > AUDIO_FRAMES) {
            overshoot = AUDIO_FRAMES;
        }
        /*
         * Dated from when the DAC last took a chunk, not from a clock
         * read here.
         *
         * s_samples_played describes audio handed to the DAC by the write
         * at the END of the previous pass, and that write is the only
         * DAC-paced event in this loop -- the same pacing the overshoot
         * correction above already rests on. Everything between it and
         * this line is not paced by anything: the ring receive, and
         * whatever preemption a board also running a SoftAP, SBC decode
         * and the bridge SPI link hands out. (This said "also running
         * Bluetooth" until the two-chip split moved A2DP to its own
         * chip; the load it describes is real, that item was not.)
         * Reading the clock
         * here folds all of it into the measurement, uncorrelated pass
         * to pass, which is the shape of the +-20 ms scatter
         * docs/clock-sync.md §9 lists as unexplained -- two reads of
         * s_phase_err_us a millisecond apart differing by 15.7 ms.
         *
         * The satellite carries the same change for symmetry, but it is
         * not where the problem was: its load is a fraction of this
         * one's and its readings were always the quiet ones.
         */
        const int64_t crossed_at = s_wrote_at
                                 - (int64_t)overshoot * 1000000 / sample_rate;
        const int32_t err = (int32_t)(crossed_at - s_phase_q[s_phase_tail].play_at);
        /*
         * The first reading after a retune is a transient, so it is
         * logged and thrown away rather than handed to the servo.
         *
         * What a retune actually costs is not what the outage figure
         * suggests. i2s_channel_disable() sets the channel state, then
         * blocks on the same binary semaphore i2s_channel_write() holds
         * across a portMAX_DELAY wait for a DMA descriptor -- and only
         * calls handle->stop() after that. So the audio keeps playing
         * for most of the reported outage. What stops is this task:
         * s_samples_played freezes while the DMA drains and real time
         * advances, and the next crossing reads that gap as position
         * error. It is not one. The buffer refills over the following
         * few writes and the position comes back on its own.
         *
         * Measured on the satellite bench, 19 same-rate retunes: net
         * +4.4 ms against a 3.6 ms outage, every one positive, and the
         * crossing landing 1-22 ms after the retune -- inside the
         * refill every time. The servo took each of those as a real
         * error and trimmed the rate for it, so every retune injected
         * the disturbance the next one would correct. That is what
         * pinned the trim at RATE_TRIM_MAX_HZ and ran phase to +500 ms.
         *
         * Only the first crossing is withheld. If the transient turns
         * out to outlast it, the REFILL line says so -- a reading taken
         * before the refill completes is the one to distrust.
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
            /*
             * Narrated but NOT withheld -- these still reach the servo
             * exactly as they did before, so this build behaves
             * identically and only says more. Whether they should be
             * withheld is the question; answering it first is the point.
             */
            if (s_retune_tail_left) {
                s_retune_tail_left--;
                ESP_LOGW(TAG, "RETUNE TAIL: phase %+ld us at %lld us after "
                              "the retune (net %+ld from before it)",
                         (long)err, since_retune,
                         (long)(err - s_retune_phase_before));
            }
            /*
             * WITHHELD while the DMA is still filling.
             *
             * i2s_channel_write() does not block while descriptors are
             * free, so on an empty channel the first writes return at
             * memory speed: s_samples_played advances by the whole DMA
             * depth (6 x AUDIO_FRAMES = 34.8 ms at 44.1 kHz) against a
             * s_wrote_at that has barely moved. Every reading dated inside
             * that window is measured against a reference the DAC is not
             * pacing, and is not a phase error at all.
             *
             * The channel is empty at every playback START -- the task
             * was parked, so it drained -- and this is the guard that was
             * missing there. It is the same mechanism the `retuning` park
             * exists to prevent mid-stream, where clock-sync.md records
             * it costing +42, +43 and +50 ms.
             *
             * Very likely the "-42 ms (hub), -26 ms (satellite)" startup
             * phase in clock-sync.md §8: a 16 ms cross-unit difference on
             * a cold start taking ~45 s to walk off, which that document
             * says "nothing accounts for at anchor time". A reconnect
             * restarts only the satellite, against a hub already servoed
             * to zero, which is why that case always behaved better.
             *
             * The REFILL line still reports the window; this stops the
             * servo acting on what is inside it.
             */
            if (s_refill_active) {
                n_refill_withheld++;
            } else {
                s_phase_err_us = err;
                s_phase_valid = true;
                sync_phase_push(&s_phase_hist, err);
                /*
                 * Publish the median for the servo, which runs on another task
                 * and cannot read s_phase_hist -- that history is play-task-only
                 * and is reset under this task's feet at every splice.
                 *
                 * The splice has filtered its input this way since it stopped
                 * splicing on one raw reading; the servo had not, and it is the
                 * same 15.7 ms of scatter feeding both. Computed here rather
                 * than in the servo because it needs the packet-cadence history:
                 * nine readings span ~180 ms, so this removes the scatter
                 * without adding lag the servo would have to wait out.
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


/*
 * A track boundary: snap this unit's phase to zero rather than let the servo
 * walk it off.
 *
 * The one moment a splice is inaudible, which is the whole reason the boundary
 * is taken from track metadata rather than picked by a timer.
 */
static void apply_track_boundary(void)
{
    /* Track boundary: snap phase to zero instead of letting the servo
     * walk it off over ~45 s. Only inaudible here. */
    int32_t rp = s_restart_pos;
    if (rp >= 0 && s_samples_played >= rp) {
        s_restart_pos = -1;
        int32_t max_frames = (int32_t)sample_rate * MAX_SPLICE_MS / 1000;
        /*
         * Nothing measured since the last re-anchor: s_phase_err_us is
         * whatever it read against the previous timeline, and it is not
         * cleared. Splicing on it would cut up to MAX_SPLICE_MS of real
         * audio to correct an error that no longer exists. Drop the
         * boundary instead -- the servo will take out anything genuine.
         */
        /*
         * THE MEDIAN, not the newest reading. This line was the shadow
         * and is now the decision; the raw value is reported beside it.
         *
         * The raw reading carries ~15.7 ms of scatter -- measured here,
         * two reads of s_phase_err_us a millisecond apart differing by
         * that much -- and the servo has used an average since it was
         * caught triggering on that noise. The splice was still using the
         * single newest sample, so at every boundary this unit jumped to
         * a position several ms wrong in a direction nothing predicts,
         * while the satellite -- a third the scatter, a fraction the load
         * -- landed closer. The two spliced to DIFFERENT places, which is
         * why a track change sometimes improved cross-unit sync and
         * sometimes degraded it.
         *
         * MUST LAND ON BOTH UNITS AT ONCE, and does: satellite/main/play.c
         * carries the same change. One-sided would guarantee they splice
         * to different places, which is the bug itself.
         *
         * Falls back to the raw reading when the history is too short to
         * offer a median (fewer than SYNC_PHASE_MIN readings, i.e. a
         * boundary very soon after a start or a previous splice). That is
         * the old behaviour, and it is better than declining to splice.
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

        /*
         * The RAW value is the shadow now -- what the correction would
         * have been on the newest reading alone. Kept, and reported on
         * the same line, so the comparison that justified this change
         * survives it and so a revert has something to check against.
         */
        int32_t raw_adj = s_phase_valid
            ? (int32_t)((int64_t)s_phase_err_us * sample_rate / 1000000) : 0;
        if (raw_adj > max_frames)  raw_adj = max_frames;
        if (raw_adj < -max_frames) raw_adj = -max_frames;
        s_hub_splice_alt_us = (int32_t)((int64_t)raw_adj * 1000000 / sample_rate);

        if (adj > 0) {
            /* Its own buffer, not chunk: chunk holds the audio read at
             * the top of this pass and not yet written to the DAC, so
             * discarding into it dropped that and played the tail of the
             * skipped region instead. Counted correctly either way, so
             * nothing drifted -- it just played 5.8 ms of the wrong
             * audio at every boundary. Same fix on the satellite. */
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
            static const uint8_t quiet[AUDIO_CHUNK_BYTES] = {0};
            int32_t left = -adj;
            size_t w = 0;
            while (left > 0) {
                int32_t n = left > AUDIO_FRAMES ? AUDIO_FRAMES : left;
                size_t bytes = (size_t)n * AUDIO_CHANNELS * sizeof(int16_t);
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
            /* And so is the splice's own history, for the same reason:
             * every reading in it was taken before this unit moved. */
            sync_phase_reset(&s_phase_hist);
            s_phase_med_valid = false;   /* it summarised the phase just removed */
        }

        /*
         * One line per track for how far apart the speakers had drifted
         * by the end of it -- the figure to compare sessions and builds
         * on, because it is taken at the same point of every track
         * cycle rather than wherever a log window happened to fall.
         *
         * The satellite figure is the marker: a physical measurement of
         * when a sample reached the output, so it sees things no
         * software reading can. The hub's splice is how much of its own
         * error it had accumulated. They answer different questions and
         * both belong here.
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
            /* No marker wire -- the normal deployed case, since it is a
             * bench instrument. Satellites report over WiFi instead, and
             * their line arrives within PROBE_PERIOD_MS of this one. */
            ESP_LOGW(TAG, "TRACK BOUNDARY: hub spliced %+ld ms | "
                          "hub phase %+ld us (median %+ld us; raw would "
                          "have spliced %+ld ms) | no marker fitted",
                     (long)(s_hub_splice_us / 1000),
                     (long)s_phase_err_us,
                     (long)med_us, (long)(s_hub_splice_alt_us / 1000));
        }
        /*
         * The visualiser is not told. A splice moves audio WITHIN the
         * timeline to correct this unit's position in it; the timeline
         * every frame is dated against and drawn on does not move, and
         * arrival is untouched.
         */
    }
}


/* The bench marker, if playback has reached a tagged sample. */
static void pulse_marker(void)
{
    int32_t mark = s_marker_sample;
    if (mark >= 0 && s_samples_played >= mark) {
#if CONFIG_DANCEFLOOR_ENABLE_MARKER
        /* 200 us of busy-wait in the playback path -- see the Kconfig
         * help. Nothing corrects on what it measures. */
        gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 1);
        s_marker_at = esp_timer_get_time();
        esp_rom_delay_us(MARKER_PULSE_US);
        gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 0);
#endif
        s_marker_sample = -1;
    }
}


/*
 * Hand the chunk to the DAC, and measure the refill while doing it.
 *
 * The write is the only DAC-paced event in the whole loop, which is why the
 * instant after it is what every phase reading is dated from.
 */
static void write_chunk(void)
{
    /* Last thing before the DMA buffer, and deliberately after every
     * count above: it rewrites slots within frames that already exist,
     * so samples_played and the phase queue are looking at the same
     * timeline whatever this unit's speaker is placed as. */
    audio_apply_channel_mode((int16_t *)chunk, AUDIO_FRAMES);
    size_t written = 0;
    const int64_t w0 = s_refill_active ? esp_timer_get_time() : 0;
    if (i2s_channel_write(i2s_tx, chunk, sizeof(chunk), &written,
                          portMAX_DELAY) != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (s_refill_active) {
        if (esp_timer_get_time() - w0 < REFILL_FAST_US) {
            s_refill_frames += (int32_t)(written / (AUDIO_CHANNELS * sizeof(int16_t)));
        } else {
            s_refill_active = false;
            ESP_LOGW(TAG, "REFILL after start: %ld frames (%ld ms) before a "
                          "write blocked -- phase readings inside this "
                          "window are not DAC-paced",
                     (long)s_refill_frames,
                     (long)(s_refill_frames * 1000 / (int32_t)sample_rate));
        }
    }
    /* Immediately, and before anything else can delay this task: it is
     * the instant the next pass dates its phase reading from. */
    s_wrote_at = esp_timer_get_time();
}


/*
 * The order those decisions are taken in, and nothing else.
 *
 * Same shape as the satellite's play task, minus the clock conversion: here
 * master time is local time. Holding the first sample until its scheduled
 * instant is what puts this speaker on the same timeline as the rest.
 */
static void begin_playback(void)
{
/* samples_played counts from the first sample played, which is the
 * first sample fed after the ring was reset at timeline start. Both
 * counters therefore share an origin -- do NOT reset s_samples_in here,
 * it has legitimately been counting the audio buffered during the wait. */
s_samples_played = 0;
/* The fraction of a frame owed described the stream that just ended.
 * rate_trim_hz itself is NOT reset: it is this unit's standing rate offset
 * against the source, exactly as tx_rate is, and it survives a restart for
 * the same reason. */
s_trim_owed = 0;
/* Every reading in it was measured against the timeline this start
 * replaces, so none of them describes where this unit now is. */
sync_phase_reset(&s_phase_hist);
s_phase_med_valid = false;
/* The channel drained while this task was parked, so it is empty here
 * for the same reason it is empty after a disable. See s_refill_active. */
s_refill_active = true;
s_refill_frames = 0;
/*
 * When the DAC last accepted a chunk -- the reference the phase reading
 * is dated against. See the note in the phase loop for why it is not a
 * clock read taken there. Seeded here so the first pass, before any
 * write has happened, has a sane value rather than zero.
 */
s_wrote_at = esp_timer_get_time();
}

void local_play_task(void *arg)
{
    (void)arg;

    while (1) {
        /*
         * Park until the timeline publishes a NEW origin.
         *
         * The test used to be `local_start == 0`, which worked because this task
         * zeroed it itself on an underrun -- and that second writer is exactly
         * what made local_start's 64-bit tearing unfixable. Waiting on the epoch
         * instead leaves the timeline as the only writer of both, and reading the
         * epoch BEFORE the instant is what makes the pair safe. See hub.h.
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
         * with the satellite's figure. A difference here is a difference in the
         * audio each unit is playing, which no amount of clock accuracy fixes. */
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
            /* Frames that came out of the ring, not the padded chunk size -- see
             * the short-read note above. Equal to AUDIO_FRAMES in every pass that
             * did not come up short. */
            s_samples_played += (int32_t)got_frames;
            write_chunk();
        }
        /* The inner loop only ends on an underrun, and the servo should stop
         * treating this unit as playing the moment it does. */
        s_playing = false;
    }
}
