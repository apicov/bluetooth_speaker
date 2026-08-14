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
                      " | %lu pkts/s"
                      " | xport %s fec %d frames %s",
                 (unsigned)filled,
                 (unsigned long)(filled * 1000 / (sample_rate * AUDIO_CHANNELS * 2)),
                 (long)s_phase_err_us,
                 (long)(s_phase_med_valid ? s_phase_med_us : 0),
                 s_phase_med_valid ? "" : " n/a",
                 (long)s_err_ema, s_tx_fail, s_tx_fail_audio, why,
                 n_tx_cong_skip,
                 (unsigned long)(s_audio_pkts / (uint32_t)CONFIG_DANCEFLOOR_LOG_PERIOD_S),
                 AUDIO_TRANSPORT_TAG, (int)CONFIG_DANCEFLOOR_AUDIO_FEC_DEPTH,
                 FRAMES_TRANSPORT_TAG);
        s_tx_fail = 0;
        s_tx_fail_audio = 0;   /* same window as the total it is a subset of */
        s_audio_pkts = 0;
        n_tx_cong_skip = 0;
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
    if (depth_ms < -120) {
        adj = -20;
    } else if (depth_ms > 120) {
        adj = 20;
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
