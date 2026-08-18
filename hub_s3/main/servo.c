/*
 * The output-rate servo, and the 5 s task that drives it.
 *
 * The comment below is the original from streamer.c, where it sat -- orphaned --
 * immediately above the allocation-failure hook rather than above the loop it
 * describes. It is back where it belongs.
 */
#include "hub.h"

/*
 * Servo the output rate on the buffer level, not on the measured rate.
 *
 * Chasing the measured rate cannot work: it carries ~0.3% noise, and whatever
 * error is left integrates straight into this buffer until it overflows or
 * empties. The level itself IS that integral, so nulling it removes the
 * accumulated error rather than the instantaneous one. Correction is spread over
 * ~40 s, well below the ~1% pitch shift a listener would notice.
 *
 * "The output rate", not "the DAC clock", since 2026-08-14. The loop is
 * unchanged -- same input, same gain, same deadband, same cooldown -- but its
 * output now normally goes to rate_trim_hz, which playback applies by dropping
 * or duplicating one frame at a time, rather than to retune_dac(), which takes
 * the I2S channel down for several milliseconds to do it. The clock is still
 * what a COARSE rate match uses; see RATE_TRIM_MAX_HZ for the boundary.
 */

/*
 * NOTE for anyone comparing this against the pre-split file: s_err_ema,
 * s_err_ema_valid, status_left, cooldown and bench_left are still declared where
 * they always were, as function-local statics inside the loop body below. They
 * were static in ring_monitor_task too, so moving that body into a function
 * called once per tick preserves their storage, their zero-initialisation and
 * their lifetime exactly. They MUST survive between 5 s windows -- s_err_ema is
 * the smoothed phase the loop runs on and cooldown is what stops it retuning
 * every window and chasing its own last correction -- and nothing here changes
 * whether they do.
 */

void servo_tick(void)
{
    /* s_playing, not `local_start == 0`: local_start has a single owner now and
     * keeps the last start instant rather than being zeroed at an underrun, so it
     * can no longer answer this. See hub.h. */
    if (!s_playing || rate_ema == 0) {
        return;
    }

#if CONFIG_DANCEFLOOR_RETUNE_BENCH_S > 0
    /* Bench: retune to the rate already set, so the RETUNE COST line below
     * reports the cost of retuning and nothing else. One unit at a time. */
    static int bench_left;
    if (--bench_left <= 0) {
        bench_left = (CONFIG_DANCEFLOOR_RETUNE_BENCH_S + 4) / 5;
        ESP_LOGW(TAG, "BENCH: forcing a same-rate retune at %" PRIu32 " Hz", tx_rate);
        retune_dac(tx_rate);
        return;
    }
#endif

    /*
     * The servo input, matching the satellite: a 4-sample EMA of the phase,
     * forgotten after a splice because the average from before it describes
     * a situation that no longer exists.
     *
     * This unit used to act on the raw reading, and the raw reading is far
     * noisier than anyone had established. Measured here, two reads of
     * s_phase_err_us one millisecond apart:
     *
     *   local ring ... | phase +26786 us (smoothed +8996 us)
     *   servo: phase +11108 us (smoothed +8996), ...
     *
     * 15.7 ms of swing between consecutive samples, with the average
     * sitting still at +9 ms through it. That is the "hub absolute phase
     * does not settle" wart in docs/clock-sync.md, quantified: the servo was
     * substantially triggering on measurement noise. A shadow run put two
     * of six retunes at the deadband edge, both of which the average would
     * have held.
     *
     * Honest caveat: cross-unit audio measured 0.5 to 2.5 ms with the raw
     * input, which is already the best this project has recorded, so this
     * is expected to reduce pointless retunes rather than to move that
     * number. If it moves it the wrong way, revert this commit -- the raw
     * value is still computed below and still logged.
     *
     * THE EMA WAS THE RIGHT FILTER ON THE WRONG INPUT. It averages across
     * servo ticks, 5 s apart, so rejecting scatter that lives inside 180 ms
     * costs it tens of seconds of memory -- and it still only averages the
     * outliers in rather than discarding them, which is what let 10 retunes
     * happen in 365 s against a real drift of ~14 ppm. Its input is now the
     * median the play task publishes, taken over the same nine packet-cadence
     * readings the splice has used since it stopped splicing on one sample.
     * One outlier in nine cannot move a median at all; it moves a 4-sample
     * EMA by a quarter of itself.
     *
     * The EMA is kept on top rather than replaced. It is what carries state
     * across the deadband and cooldown, and the two filters answer different
     * questions: the median says where this unit is now, the average says
     * whether it has been there long enough to act on. Falling back to the raw
     * reading while the median is invalid keeps the servo working through the
     * first ~100 ms after a splice or a start, which is exactly when it must
     * not be idle.
     */
    static int32_t s_err_ema;
    static bool    s_err_ema_valid;
    if (s_phase_stepped || !s_phase_valid) {
        s_phase_stepped = false;
        s_err_ema_valid = false;   /* history describes a different world */
    }
    const int32_t err_in = s_phase_med_valid ? s_phase_med_us : s_phase_err_us;
    s_err_ema = s_err_ema_valid ? (s_err_ema * 3 + err_in) / 4 : err_in;
    s_err_ema_valid = true;

    size_t filled = LOCAL_RING_BYTES - xStreamBufferSpacesAvailable(local_ring);
    /* Printed once per log period, not every window. tx-fail accumulates
     * across the quiet windows so nothing is lost by not printing it. */
    static int status_left;
    if (--status_left <= 0) {
        status_left = CONFIG_DANCEFLOOR_LOG_PERIOD_S / 5;
        /* Empty unless something failed, so a clean line is unchanged from
         * every log captured before this instrument existed. */
        char why[128];
        tx_fail_summary(why, sizeof(why));
        /* Raw, median and average side by side, because the change from feeding
         * the servo the raw reading to feeding it the median is only
         * falsifiable if a log shows both. Raw minus median IS the scatter this
         * was written to reject: if they track each other, the 15.7 ms wart has
         * gone somewhere else and this filter is not earning its place. */
        ESP_LOGI(TAG, "local ring %u bytes (%lu ms) | phase %+ld us "
                      "(median %+ld%s, smoothed %+ld us) | "
                      "tx-fail %" PRIu32 " (%" PRIu32 " audio)%s"
                      " | cong-skip %" PRIu32
                      " | pace-skip %" PRIu32
                      " | %lu pkts/s"
                      " | xport %s fec %d frames %s",
                 (unsigned)filled,
                 (unsigned long)(filled * 1000 / (sample_rate * AUDIO_CHANNELS * 2)),
                 (long)s_phase_err_us,
                 (long)(s_phase_med_valid ? s_phase_med_us : 0),
                 s_phase_med_valid ? "" : " n/a",
                 (long)s_err_ema, s_tx_fail, s_tx_fail_audio, why,
                 n_tx_cong_skip, n_tx_pace_skip,
                 (unsigned long)(s_audio_pkts / (uint32_t)CONFIG_DANCEFLOOR_LOG_PERIOD_S),
                 AUDIO_TRANSPORT_TAG, (int)CONFIG_DANCEFLOOR_AUDIO_FEC_DEPTH,
                 FRAMES_TRANSPORT_TAG);
        s_tx_fail = 0;
        s_tx_fail_audio = 0;   /* same window as the total it is a subset of */
        s_audio_pkts = 0;
        n_tx_cong_skip = 0;
        n_tx_pace_skip = 0;    /* same window as the cong-skip it accompanies */
    }

    const int32_t target = (int32_t)(LEAD_US / 1000) *
                           (int32_t)(rate_ema * AUDIO_CHANNELS * 2 / 1000);
    int32_t err_frames = ((int32_t)filled - target) / (AUDIO_CHANNELS * 2);
    int32_t depth_ms = err_frames * 1000 / (int32_t)rate_ema;

    if (!s_phase_valid) {
        return;
    }
    /*
     * Phase drives the correction; buffer depth is only a guard against
     * running empty or overflowing, which phase control would not see
     * coming. Late means behind the timeline, so play faster.
     *
     * Spread over ~100 s: at 40 s the loop was still correcting after the
     * error had gone and overshot to +8 ms. Real drift is ~0.8 ms/minute,
     * far slower than the correction needs to be.
     */
    int32_t adj = (int32_t)((int64_t)s_err_ema * rate_ema / 100000000LL);
    /* The drift correction is small by nature -- real drift is ~14 ppm.
     * Anything larger is a bad phase reading, not a rate error. */
    if (adj >  RATE_TRIM_MAX_HZ) adj =  RATE_TRIM_MAX_HZ;
    if (adj < -RATE_TRIM_MAX_HZ) adj = -RATE_TRIM_MAX_HZ;
    /*
     * A FLOOR, NOT A REPLACEMENT, since 2026-08-15. It used to assign adj
     * outright, so it could only ever WEAKEN a phase correction that already
     * agreed with it -- and a ring past target IS playing late, so the two
     * normally do agree. The satellite's copy carries the measurement that
     * argued for this; this unit's own ring is fed over a stream buffer rather
     * than the radio, so it never showed the fault. Landed on both units in one
     * commit all the same: the two servos are the same loop, and a correction
     * rate that differs between them is a cross-unit sync error by
     * construction.
     *
     * Depth still wins when the two disagree, which is the case it was written
     * for. No steady-state effect -- depth only leaves +-120 ms during the
     * events this is about.
     */
    const int32_t adj_phase = adj;       /* what phase alone asked for */
    if (depth_ms < -120) {
        if (adj > -20) adj = -20;
    } else if (depth_ms > 120) {
        if (adj < 20) adj = 20;
    }
    /* Same instrument as the satellite's, for the same reason. */
    if (adj != adj_phase) {
        ESP_LOGW(TAG, "depth net: buffer %+ld ms, phase asked %+ld Hz -> %+ld Hz",
                 (long)depth_ms, (long)adj_phase, (long)adj);
    }
    /*
     * THE LARGE-ERROR CATCH-UP, armed here, spent by playback.
     *
     * Same mechanism, same constants, same arithmetic as the satellite's copy
     * -- audio_shift.c is shared, and a correction rate that differs between
     * the units is itself a cross-unit sync error. The situation it exists
     * for was this unit's to cause: a tx-fail burst on the radio below starves
     * the satellites' audio AND this unit's own ring at once, and both sides
     * then carry the same tens-of-ms knock that the 2.27 ms/s trim ceiling
     * takes a minute or more to walk off.
     *
     * Armed from `err_in` for the same reason the servo itself reads it: the
     * EMA's settling (~20 s) outlives the whole drain, so arming from the EMA
     * would re-arm error the drain had already paid and overshoot to the other
     * side. But since 2026-08-18 the ARM requires the published median to be
     * VALID at the moment of the write -- the raw fallback no longer arms
     * anything, for two reasons measured on that day's soak:
     *
     *   a splice zeroes the debt and invalidates the median precisely because
     *   the readings before it described the error just paid, so arming on the
     *   raw survivor is arming on the paid error itself; and between the read
     *   of err_in at the top of this tick and the write below sits the status
     *   print -- ~13 ms of UART at this log rate, ample for a splice to land
     *   in -- so validity is re-tested at the write, not just at the read.
     *
     * The debt only grows here and only playback shrinks it; a sign flip
     * replaces it outright. See the satellite's copy for the race note --
     * writes race the play task's spend at 5 s against ~6 ms cadence, benign
     * both ways.
     */
    {
        /*
         * The startup hold: a fresh timeline's first-minute phase (the DMA
         * refill transient, -30285 us median on the 2026-08-18 soak) is past
         * CATCHUP_ARM_US with nothing wrong, and armed 1347 frames of replay
         * here at boot for an error the trim was already walking off. local_start
         * rewrites at every timeline start, so the hold re-engages after an
         * underrun restart too -- correct, the transient exists at every
         * start. Only the ARM waits; the clear keeps working, because
         * standing a stale debt down early is safe in a way arming one is not.
         */
        const bool held = esp_timer_get_time() - local_start < CATCHUP_HOLD_US;
        const int32_t med = err_in;
        if (!held && s_phase_med_valid &&
            (med >= CATCHUP_ARM_US || med <= -CATCHUP_ARM_US)) {
            const int32_t cap = (int32_t)((int64_t)CATCHUP_MAX_US * rate_ema
                                          / 1000000);
            int32_t want = (int32_t)((int64_t)med * rate_ema / 1000000);
            if (want >  cap) want =  cap;
            if (want < -cap) want = -cap;
            const int32_t now = catchup_frames;
            if ((want > 0) != (now > 0)) {
                catchup_frames = want;              /* overshot: reversed */
            } else if (want > 0 ? want > now : want < now) {
                catchup_frames = want;              /* same side, deeper */
            }
        } else if (med < CATCHUP_CLEAR_US && med > -CATCHUP_CLEAR_US) {
            catchup_frames = 0;   /* remainder is the fine trim's to finish */
        }
    }
    uint32_t desired = (uint32_t)((int32_t)rate_ema + adj);

    /*
     * Deadband in phase error, not in rate -- see PHASE_DEADBAND_US. The
     * old tx_rate/5000 was documented as ~8 ms and is really ~20 ms. This
     * unit has always parked its playback across a retune, so its retunes
     * were never the expensive kind; the satellite's were, until it got the
     * same guard.
     */
    int32_t deadband = (int32_t)((int64_t)PHASE_DEADBAND_US * rate_ema / 100000000LL);
    if (deadband < 1) {
        deadband = 1;
    }

    /* Wait for the buffer to respond before correcting again -- the hub
     * had no cooldown at all, so it retuned every window and chased its own
     * previous correction. */
    /*
     * RETIRED 2026-08-12, on the bench logs rather than on opinion.
     *
     * A shadow used to compute what the RAW phase reading would have asked for,
     * and log SERVO DIVERGES whenever it disagreed with the average. Its question
     * was whether smoothing the servo's input declines retunes that the raw value
     * would have made. The 18:09 run answered it: SERVO DIVERGES fired repeatedly
     * across the session and EVERY firing was the same way round -- raw would
     * retune, smoothed held -- while phase converged from -26.9 ms to -1.9 ms
     * with retunes 30 and 45 s apart.
     *
     * That is the averaging doing exactly what it was added for, and a shadow
     * that only ever says so is an instrument with nothing left to find. The raw
     * value is still printed on the servo line beside the smoothed one, so the
     * two remain comparable at every retune.
     */
    /*
     * The correction, as an offset from the rate the CLOCK is actually running
     * at. This is the number that used to be handed to retune_dac() as an
     * absolute rate; splitting it out is what lets the two actuators be chosen
     * between, because their boundary is a size and not a kind.
     */
    const int32_t trim_hz = (int32_t)desired - (int32_t)tx_rate;

    static int cooldown;
    if (cooldown > 0) {
        cooldown--;
    } else {
        /*
         * Deadband against the trim ALREADY APPLIED, which is where tx_rate
         * used to be read: the servo tolerates PHASE_DEADBAND_US of its own
         * error before moving, and that meaning is unchanged. Only what moves
         * has changed.
         */
        const int32_t step = trim_hz - rate_trim_hz;
        if (step > deadband || step < -deadband) {
            if (trim_hz > RATE_TRIM_MAX_HZ || trim_hz < -RATE_TRIM_MAX_HZ) {
                /*
                 * COARSE: too big for software to absorb without shredding the
                 * audio, so the clock has to move. The case this exists for is
                 * a source measured at ~42600 against a 44100 output -- 40000
                 * ppm, which drains a 250 ms buffer in five seconds. Costs the
                 * channel-down that the fine path was written to stop paying,
                 * and in steady state never happens.
                 *
                 * The trim is cleared because the clock now carries what it was
                 * carrying; leaving it set would apply the correction twice.
                 */
                ESP_LOGI(TAG, "servo: smoothed %+ld us (raw %+ld), buffer %+ld ms "
                              "-> COARSE, DAC %" PRIu32 " Hz",
                         (long)s_err_ema, (long)s_phase_err_us, (long)depth_ms,
                         desired);
                rate_trim_hz = 0;
                retune_dac(desired);
            } else {
                /* FINE: playback drops or duplicates one frame at a time. No
                 * channel-down, and continuous rather than stepped. */
                /*
                 * Frames per second, which IS |trim_hz|: a trim of N Hz against
                 * a rate of `rate` needs rate * N/rate = N extra frames every
                 * second. Printed anyway, because it is the figure the
                 * n_trim_drops / n_trim_dups deltas have to match, and reading
                 * that off a Hz value is one inference more than a log should
                 * ask for. It was briefly printed as `tx_rate / |trim_hz|`
                 * labelled milliseconds, which is frames-between-corrections
                 * with the wrong unit on it -- 3150 ms where the truth was 71.
                 */
                ESP_LOGI(TAG, "servo: smoothed %+ld us (raw %+ld), buffer %+ld ms "
                              "-> trim %+ld Hz (%ld frames/s)",
                         (long)s_err_ema, (long)s_phase_err_us, (long)depth_ms,
                         (long)trim_hz, (long)(trim_hz < 0 ? -trim_hz : trim_hz));
                rate_trim_hz = trim_hz;
            }
            cooldown = 4;          /* ~20 s against a 100 s correction */
        }
    }
}

/*
 * The 5 s tick both halves hang off.
 *
 * telemetry_tick() first, then servo_tick(), which is the order they ran in as
 * one function -- the reporting deliberately preceded the "is anything playing"
 * check, because a stopped stream is when the counters matter most.
 */
void ring_monitor_task(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        telemetry_tick();
        servo_tick();
    }
}
