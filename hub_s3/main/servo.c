/*
 * The output-rate servo, and the 5 s task that drives it.
 *
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
 *
 * SINCE 2026-08-19 THE LOOP ITSELF LIVES IN components/dancefloor_sync/
 * df_servo.c, which the satellite runs too. It was two copies of the same
 * arithmetic, and both copies said in as many words that they must never
 * disagree -- a correction rate that differs between the units is a cross-unit
 * sync error by construction. df_servo.h says why that is now structural rather
 * than remembered, and test_servo.c pins the branches that soaks found the hard
 * way, including this unit's own 15.7 ms of raw-reading scatter.
 *
 * What is left here is what is genuinely this unit's: when a stream counts as
 * playing, how deep the local ring is, that a coarse correction reaches
 * retune_dac(), and every log line. s_err_ema and cooldown are still carried
 * between windows -- only their storage moved into the shared struct.
 */
#include "hub.h"

#include "df_servo.h"

static df_servo_t s_servo;

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
     * The servo input: THE MEDIAN the play task publishes, falling back to the
     * raw reading while it is invalid -- which is where this unit differs from
     * the satellite, deliberately. See the note at the top of df_servo.h.
     *
     * This unit used to act on the raw reading, and the raw reading is far
     * noisier than anyone had established. Measured here, two reads of
     * s_phase_err_us one millisecond apart:
     *
     *   local ring ... | phase +26786 us (smoothed +8996 us)
     *   servo: phase +11108 us (smoothed +8996), ...
     *
     * 15.7 ms of swing between consecutive samples, with the average sitting
     * still at +9 ms through it. That is the "hub absolute phase does not
     * settle" wart in docs/clock-sync.md, quantified: the servo was
     * substantially triggering on measurement noise. A shadow run put two of six
     * retunes at the deadband edge, both of which the average would have held.
     *
     * Honest caveat: cross-unit audio measured 0.5 to 2.5 ms with the raw input,
     * which is already the best this project has recorded, so this is expected
     * to reduce pointless retunes rather than to move that number.
     *
     * THE EMA WAS THE RIGHT FILTER ON THE WRONG INPUT. It averages across servo
     * ticks, 5 s apart, so rejecting scatter that lives inside 180 ms costs it
     * tens of seconds of memory -- and it still only averages the outliers in
     * rather than discarding them, which is what let 10 retunes happen in 365 s
     * against a real drift of ~14 ppm. Its input is now the median the play task
     * publishes, taken over the same nine packet-cadence readings the splice has
     * used since it stopped splicing on one sample. One outlier in nine cannot
     * move a median at all; it moves a 4-sample EMA by a quarter of itself.
     *
     * The EMA is kept on top rather than replaced. It is what carries state
     * across the deadband and cooldown, and the two filters answer different
     * questions: the median says where this unit is now, the average says
     * whether it has been there long enough to act on. Falling back to the raw
     * reading while the median is invalid keeps the servo working through the
     * first ~100 ms after a splice or a start, which is exactly when it must not
     * be idle.
     */
    const bool reset = (s_phase_stepped || !s_phase_valid);
    if (s_phase_stepped) {
        s_phase_stepped = false;
    }
    const int32_t err_in = s_phase_med_valid ? s_phase_med_us : s_phase_err_us;
    const int32_t err_ema = df_servo_ema(&s_servo, err_in, reset);

    size_t filled = LOCAL_RING_BYTES - xStreamBufferSpacesAvailable(local_ring);
    /* Printed once per log period, not every window. tx-fail accumulates
     * across the quiet windows so nothing is lost by not printing it. */
    static int status_left;
    /*
     * ...EXCEPT WHILE THE TRANSMIT PATH IS IN TROUBLE, when it prints every 5 s.
     *
     * The 20 s window is why the 2026-08-20 soak cannot be read. The satellites
     * lost their lead at 16:20:31 and the first large tx-fail was reported at
     * 16:20:47 -- both inside ONE window, so nothing in the capture says which
     * came first, and "the queue filled and then refused" versus "something
     * refused and the queue backed up behind it" are the two candidate stories.
     * A 5 s cadence separates them; 20 s cannot, however long the run is.
     *
     * Armed by the failure itself rather than by a mode, so it costs nothing on
     * a clean run and needs no way to be turned on: the interesting window is
     * always the one after the one that went wrong. Held for ~60 s past the
     * last refusal so the recovery is captured at the same resolution as the
     * onset -- a storm that ends is as informative as one that starts, and this
     * hub has now had both.
     */
    static int fast_left;
    /* 5 s passes since the last print. The window is no longer a constant, and
     * every RATE on this line is a count divided by it -- so it has to be
     * measured rather than assumed, or `pkts/s` reads a quarter of the truth in
     * exactly the windows anyone will be looking at. */
    static int ticks;
    ticks++;
    if (s_tx_fail) {
        fast_left = 60 / 5;
    }
    if (--status_left <= 0 || fast_left > 0) {
        status_left = (fast_left > 0) ? 1 : CONFIG_DANCEFLOOR_LOG_PERIOD_S / 5;
        if (fast_left > 0) {
            fast_left--;
        }
        const uint32_t window_s = (uint32_t)ticks * 5;
        ticks = 0;
        /* Empty unless something failed, so a clean line is unchanged from
         * every log captured before this instrument existed. */
        /*
         * SIZED FOR THE WHOLE LINE, for the reason burst[] below is: 128 was a
         * typical-case number, not a worst-case one. tx_fail_summary() renders
         * up to TX_ERR_SLOTS (4) distinct errno names plus "other", and
         * strerror() strings run to ~32 characters -- " -- Software caused
         * connection abort 4294967295" is 47 bytes on its own, so four of them
         * and a tail is ~200. Worse, it CLEARS each slot as it renders it, so a
         * truncated tally is lost rather than deferred: the window would under-
         * report and no later window would make it up.
         */
        char why[224];
        tx_fail_summary(why, sizeof(why));
        /* Which lanes were refused, and -- only when something was -- whether
         * the refusals were beacon-spaced or one unbroken stall. See net.c. */
        char lanes[96];
        tx_fail_lanes(lanes, sizeof(lanes));
        /*
         * SIZED FOR THE WHOLE LINE, NOT FOR A TYPICAL ONE. At 96 this silently
         * truncated: the 2026-08-23 15:56 soak logged `audio-retry 21 |  |`
         * with the audio-retry-ok value cut clean off, and snprintf reports a
         * truncation nobody was checking. Worst case is every counter at
         * UINT32_MAX -- 6 fields, ~190 bytes -- so anything added here has to
         * be added to this number too.
         */
        char burst[224];
        tx_burst_summary(burst, sizeof(burst));
        /*
         * What the RADIO did, as opposed to what sendto() accepted. Always
         * non-empty -- see tx_air_summary() in net.c for why a clean window
         * still has to print here. Worst case is a 10-digit ms figure and two
         * UINT32s, ~64 bytes; 96 leaves room for the labels.
         */
        char air[96];
        tx_air_summary(air, sizeof(air));
        /*
         * The station census, taken here rather than kept as a counter because
         * association is a fact the driver already holds and a counter would
         * only be a stale copy of it.
         *
         * `stations` is the first thing to read on a bad line. Every soak this
         * project has on file with ONE satellite ran tx-fail at essentially
         * zero, including a 3h11m one; the run that fell apart was the first
         * with two. The hub's downlink packet rate is 50 + 96xN, so a second
         * satellite roughly triples it -- which makes satellite count and
         * transmit load the same variable, and the first thing to establish is
         * that both units were actually associated the whole time rather than
         * flapping. (This read the other way while the downlink was multicast
         * and the rate was flat in N; that is no longer the build.)
         *
         * rssi-min and phy-11n come free from the same call. The first says
         * whether the air changed under the fault; the second whether a station
         * associated without 11n, which would mean no aggregation to it and a
         * transmit queue that drains a frame at a time.
         */
        wifi_sta_list_t sta = {0};
        int8_t rssi_min = 0;
        int n_11n = 0;
        if (esp_wifi_ap_get_sta_list(&sta) != ESP_OK) {
            sta.num = -1;   /* says "not measured", which is not "none joined" */
        }
        for (int i = 0; i < sta.num; i++) {
            if (i == 0 || sta.sta[i].rssi < rssi_min) {
                rssi_min = sta.sta[i].rssi;
            }
            n_11n += sta.sta[i].phy_11n ? 1 : 0;
        }
        /* Text, not a number, so "nothing stamped this window" cannot be read
         * as a lead of zero -- the same reason the satellite's ARRIVAL line
         * spells its two minima out. See n_lead_min_us. */
        char lead_s[16];
        const int32_t lead_min = n_lead_min_us;
        if (lead_min == LEAD_UNSEEN) {
            snprintf(lead_s, sizeof(lead_s), "none");
        } else {
            snprintf(lead_s, sizeof(lead_s), "%ld ms", (long)(lead_min / 1000));
        }
        /* Raw, median and average side by side, because the change from feeding
         * the servo the raw reading to feeding it the median is only
         * falsifiable if a log shows both. Raw minus median IS the scatter this
         * was written to reject: if they track each other, the 15.7 ms wart has
         * gone somewhere else and this filter is not earning its place. */
        ESP_LOGI(TAG, "local ring %u bytes (%lu ms) | phase %+ld us "
                      "(median %+ld%s, smoothed %+ld us) | "
                      "tx-fail %" PRIu32 " (%s)%s%s%s"
                      " | cong-skip %" PRIu32
                      " | stale-skip %" PRIu32
                      " | %lu pkts/s | fanout-gap-max %ld ms | lead-min %s"
                      " | stations %d | rssi-min %d | phy-11n %d | churn %" PRIu32
                      " | fec %d",
                 (unsigned)filled,
                 (unsigned long)(filled * 1000 / (sample_rate * AUDIO_CHANNELS * 2)),
                 (long)s_phase_err_us,
                 (long)(s_phase_med_valid ? s_phase_med_us : 0),
                 s_phase_med_valid ? "" : " n/a",
                 (long)err_ema, s_tx_fail, lanes, why, burst, air,
                 n_tx_cong_skip, n_tx_pace_skip,
                 (unsigned long)(s_audio_pkts / window_s),
                 (long)(n_fanout_gap_max_us / 1000), lead_s,
                 (int)sta.num, (int)rssi_min, n_11n, n_join_churn,
                 (int)CONFIG_DANCEFLOOR_AUDIO_FEC_DEPTH);
        s_tx_fail = 0;
        /* The lane counters are cleared inside tx_fail_lanes(), on the same
         * pass that rendered them, for the reason tx_fail_summary() is: the
         * total and its breakdown must not come to describe different windows. */
        s_audio_pkts = 0;
        n_tx_cong_skip = 0;
        /* Same window as the cong-skip it accompanies. It reads `stale-skip`
         * on the line because that is what it counts now -- frames stranded
         * in a batch a stopped stream never flushed. See hub.h. */
        n_tx_pace_skip = 0;
        /* Beside the packet rate it qualifies: the rate says how many went, this
         * says whether they went evenly. See hub.h for what the pair answers
         * against the satellite's ARRIVAL line. */
        n_fanout_gap_max_us = 0;
        n_lead_min_us = LEAD_UNSEEN;
        /* Same window as the refusals it is there to be correlated with. */
        n_join_churn = 0;
    }

    const int32_t target = (int32_t)(LEAD_US / 1000) *
                           (int32_t)(rate_ema * AUDIO_CHANNELS * 2 / 1000);
    int32_t err_frames = ((int32_t)filled - target) / (AUDIO_CHANNELS * 2);
    int32_t depth_ms = err_frames * 1000 / (int32_t)rate_ema;

    /*
     * This unit gives up here rather than carrying on to the depth net, which is
     * why df_servo_ema() is a separate call from df_servo_step(): the average is
     * still folded forward every window, and the early-out stays visible in the
     * unit it belongs to.
     */
    if (!s_phase_valid) {
        return;
    }

    /*
     * The catch-up arm requires the published median to be VALID at the moment
     * of the write, which is re-tested here rather than at the read above --
     * between the two sits the status print, ~13 ms of UART at this log rate,
     * ample for a splice to land in. The satellite takes its median inline and
     * needs no such re-test.
     *
     * The startup hold: a fresh timeline's first-minute phase (the DMA refill
     * transient, -30285 us median on the 2026-08-18 soak) is past CATCHUP_ARM_US
     * with nothing wrong, and armed 1347 frames of replay here at boot for an
     * error the trim was already walking off. local_start rewrites at every
     * timeline start, so the hold re-engages after an underrun restart too --
     * correct, the transient exists at every start.
     */
    const df_servo_in_t in = {
        .phase_valid    = true,
        .med_us         = err_in,
        .have_med       = s_phase_med_valid,
        .catchup_held   = esp_timer_get_time() - local_start < CATCHUP_HOLD_US,
        .depth_ms       = depth_ms,
        /* This unit's ring is fed over a stream buffer rather than the radio, so
         * it never showed the fault the satellite's post-anchor hold exists for. */
        .depth_net_held = false,
        .rate           = rate_ema,
        .tx_rate        = (int32_t)tx_rate,
        .trim_hz_now    = rate_trim_hz,
        .catchup_now    = catchup_frames,
    };
    df_servo_out_t out;
    df_servo_step(&s_servo, &in, &out);

    /* Same instrument as the satellite's, for the same reason. */
    if (out.depth_net_fired) {
        ESP_LOGW(TAG, "depth net: buffer %+ld ms, phase asked %+ld Hz -> %+ld Hz",
                 (long)depth_ms, (long)out.adj_phase, (long)out.adj);
    }

    /* The debt only grows in the servo and only playback shrinks it. See the
     * satellite's copy for the race note -- writes race the play task's spend at
     * 5 s against ~6 ms cadence, benign both ways. */
    if (out.catchup_write) {
        catchup_frames = out.catchup_frames_new;
    }

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
    if (!out.act) {
        return;
    }
    if (out.coarse) {
        /*
         * COARSE: too big for software to absorb without shredding the audio, so
         * the clock has to move. The case this exists for is a source measured
         * at ~42600 against a 44100 output -- 40000 ppm, which drains a 250 ms
         * buffer in five seconds. Costs the channel-down that the fine path was
         * written to stop paying, and in steady state never happens.
         *
         * The trim is cleared because the clock now carries what it was
         * carrying; leaving it set would apply the correction twice.
         */
        ESP_LOGI(TAG, "servo: smoothed %+ld us (raw %+ld), buffer %+ld ms "
                      "-> COARSE, DAC %" PRIu32 " Hz",
                 (long)out.err_ema, (long)s_phase_err_us, (long)depth_ms,
                 out.desired_rate);
        rate_trim_hz = 0;
        retune_dac(out.desired_rate);
    } else {
        /* FINE: playback drops or duplicates one frame at a time. No
         * channel-down, and continuous rather than stepped. */
        /*
         * Frames per second, which IS |trim_hz|: a trim of N Hz against a rate
         * of `rate` needs rate * N/rate = N extra frames every second. Printed
         * anyway, because it is the figure the n_trim_drops / n_trim_dups deltas
         * have to match, and reading that off a Hz value is one inference more
         * than a log should ask for. It was briefly printed as
         * `tx_rate / |trim_hz|` labelled milliseconds, which is
         * frames-between-corrections with the wrong unit on it -- 3150 ms where
         * the truth was 71.
         */
        ESP_LOGI(TAG, "servo: smoothed %+ld us (raw %+ld), buffer %+ld ms "
                      "-> trim %+ld Hz (%ld frames/s)",
                 (long)out.err_ema, (long)s_phase_err_us, (long)depth_ms,
                 (long)out.trim_hz,
                 (long)(out.trim_hz < 0 ? -out.trim_hz : out.trim_hz));
        rate_trim_hz = out.trim_hz;
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
