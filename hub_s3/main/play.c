/*
 * This unit's own speaker.
 *
 * The hub delays its own audio by LEAD_US exactly like a satellite -- otherwise
 * it would play ahead of every other speaker on the floor. Same shape as the
 * satellite's play task, minus the clock conversion: here master time IS local
 * time, so the phase reading is a direct measure of how far this unit has
 * slipped from the timeline it is itself publishing.
 */
#include "hub.h"

/*
 * Same shape as the satellite's play task, minus the clock conversion: here
 * master time is local time. Holding the first sample until its scheduled
 * instant is what puts this speaker on the same timeline as the rest.
 */
void local_play_task(void *arg)
{
    (void)arg;
    static uint8_t chunk[AUDIO_CHUNK_BYTES];

    while (1) {
        if (local_start == 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        int64_t wait = local_start - esp_timer_get_time();
        if (wait > 2000) {
            vTaskDelay(pdMS_TO_TICKS((wait - 2000) / 1000));
        }
        while (esp_timer_get_time() < local_start) {
            /* spin the last stretch */
        }
        /* On the hub local time IS master time, so this is directly comparable
         * with the satellite's figure. A difference here is a difference in the
         * audio each unit is playing, which no amount of clock accuracy fixes. */
        ESP_LOGI(TAG, "local playback started: scheduled %lld, actual %lld (%+lld us)",
                 local_start, esp_timer_get_time(), esp_timer_get_time() - local_start);
        /* samples_played counts from the first sample played, which is the
         * first sample fed after the ring was reset at timeline start. Both
         * counters therefore share an origin -- do NOT reset s_samples_in here,
         * it has legitimately been counting the audio buffered during the wait. */
        int32_t samples_played = 0;
        bool was_retuning = false;    /* armed by the park below, see s_refill_active */
        /* Every reading in it was measured against the timeline this start
         * replaces, so none of them describes where this unit now is. */
        sync_phase_reset(&s_phase_hist);
        /* The channel drained while this task was parked, so it is empty here
         * for the same reason it is empty after a disable. See s_refill_active. */
        s_refill_active = true;
        s_refill_frames = 0;
        s_refill_why = "start";
        /*
         * When the DAC last accepted a chunk -- the reference the phase reading
         * is dated against. See the note in the phase loop for why it is not a
         * clock read taken there. Seeded here so the first pass, before any
         * write has happened, has a sane value rather than zero.
         */
        int64_t wrote_at = esp_timer_get_time();

        while (1) {
            if (retuning) {
                /* Do not pull from the ring while the channel is down -- writes
                 * would return instantly and drain it. */
                was_retuning = true;
                vTaskDelay(pdMS_TO_TICKS(2));
                continue;
            }
            if (was_retuning) {
                was_retuning = false;
                s_refill_active = true;
                s_refill_frames = 0;
                s_refill_why = "retune";
            }
            hw_play = uxTaskGetStackHighWaterMark(NULL);   /* only valid in-task */
            size_t got = xStreamBufferReceive(local_ring, chunk, sizeof(chunk), pdMS_TO_TICKS(500));
            if (got == 0) {
                n_underruns++;
                ESP_LOGW(TAG, "local underrun, restarting timeline");
                local_start = 0;
                s_underrun_recover = true;
                break;
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
             */
            uint32_t got_frames = (uint32_t)(got / (AUDIO_CHANNELS * sizeof(int16_t)));
            if (got < sizeof(chunk)) {
                memset(chunk + got, 0, sizeof(chunk) - got);
                n_short_reads++;
                n_short_frames += (uint32_t)((sizeof(chunk) - got)
                                             / (AUDIO_CHANNELS * sizeof(int16_t)));
            }
            /* Local time IS master time here, so this is a direct read of how
             * far playback has slipped from the published timeline. */
            while (s_phase_tail != s_phase_head && samples_played >= s_phase_q[s_phase_tail].pos) {
                /*
                 * Correct for WHERE the crossing was noticed versus where it
                 * happened, which is most of this unit's phase noise.
                 *
                 * samples_played advances by AUDIO_FRAMES per iteration -- 5.8
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
                 * samples_played was `pos` is `overshoot / rate` ago.
                 */
                int32_t overshoot = samples_played - s_phase_q[s_phase_tail].pos;
                /*
                 * Capped at one chunk, because beyond that the pacing
                 * assumption is false. A splice advances samples_played by up
                 * to MAX_SPLICE_MS in a single step and those frames were
                 * discarded rather than played over time, so any point the jump
                 * carried us past cannot be dated this way. Capping leaves
                 * those readings no worse than they were before this
                 * correction existed.
                 */
                if (overshoot > AUDIO_FRAMES) {
                    overshoot = AUDIO_FRAMES;
                }
                /*
                 * Dated from when the DAC last took a chunk, not from a clock
                 * read here.
                 *
                 * samples_played describes audio handed to the DAC by the write
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
                const int64_t crossed_at = wrote_at
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
                 * samples_played freezes while the DMA drains and real time
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
                     * memory speed: samples_played advances by the whole DMA
                     * depth (6 x AUDIO_FRAMES = 34.8 ms at 44.1 kHz) against a
                     * wrote_at that has barely moved. Every reading dated inside
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
                    }
                }
                s_phase_tail = (s_phase_tail + 1) % PHASE_Q_LEN;
            }

            /* Track boundary: snap phase to zero instead of letting the servo
             * walk it off over ~45 s. Only inaudible here. */
            int32_t rp = s_restart_pos;
            if (rp >= 0 && samples_played >= rp) {
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
                    samples_played += (adj - left);
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

            int32_t mark = s_marker_sample;
            if (mark >= 0 && samples_played >= mark) {
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
            /* Frames that came out of the ring, not the padded chunk size -- see
             * the short-read note above. Equal to AUDIO_FRAMES in every pass that
             * did not come up short. */
            samples_played += (int32_t)got_frames;

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
                    ESP_LOGW(TAG, "REFILL after %s: %ld frames (%ld ms) before a "
                                  "write blocked -- phase readings inside this "
                                  "window are not DAC-paced",
                             s_refill_why, (long)s_refill_frames,
                             (long)(s_refill_frames * 1000 / (int32_t)sample_rate));
                }
            }
            /* Immediately, and before anything else can delay this task: it is
             * the instant the next pass dates its phase reading from. */
            wrote_at = esp_timer_get_time();
        }
    }
}
