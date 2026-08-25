/**
 * @file servo.c
 * @brief The output-rate servo, and the 5 s task that drives it.
 *
 * The servo acts on the local ring's BUFFER LEVEL, not on the measured source
 * rate. Chasing the measured rate cannot work -- it carries ~0.3% noise, and
 * whatever error is left integrates into this buffer until it overflows or
 * empties. The level is that integral, so nulling it removes the accumulated
 * error rather than the instantaneous one.
 *
 * The loop arithmetic itself lives in components/dancefloor_sync/df_servo.c,
 * which the satellite runs too: a correction rate that differs between the
 * units is a cross-unit sync error by construction, so the two cannot be
 * allowed to hold separate copies of it. df_servo.h carries the loop's own
 * reasoning and test_servo.c pins its branches.
 *
 * What is left here is what is genuinely this unit's: when a stream counts as
 * playing, how deep the local ring is, that a coarse correction reaches
 * retune_dac(), and every log line -- including the 5 s status line, which is
 * the widest instrument either firmware has.
 */
#include "hub.h"

#include "df_servo.h"

/** @brief The loop's carried state: the smoothed error and the cooldown.
 *         Owned by ring_monitor_task, which is its only caller. */
static df_servo_t s_servo;

void servo_tick(void)
{
    /* s_playing, not `local_start == 0`: local_start has a single owner and
     * keeps the last start instant rather than being zeroed at an underrun,
     * so it can no longer answer this. See hub.h. */
    if (!s_playing || rate_ema == 0) {
        return;
    }

#if CONFIG_DANCEFLOOR_RETUNE_BENCH_S > 0
    /* Bench: retune to the rate already set, so the RETUNE COST line play.c
     * prints reports the cost of retuning and nothing else. One unit at a
     * time. */
    static int bench_left;
    if (--bench_left <= 0) {
        bench_left = (CONFIG_DANCEFLOOR_RETUNE_BENCH_S + 4) / 5;
        ESP_LOGW(TAG, "BENCH: forcing a same-rate retune at %" PRIu32 " Hz", tx_rate);
        retune_dac(tx_rate);
        return;
    }
#endif

    /*
     * The servo's input is the MEDIAN the play task publishes, falling back to
     * the raw reading while no median is valid.
     *
     * The raw reading is noisy at this unit's load -- far noisier than the
     * satellite's, which is why the two differ here; see the note at the top
     * of df_servo.h. The median is taken over SYNC_PHASE_HIST readings at
     * packet cadence, so one outlier cannot move it at all, where it moves an
     * EMA by a share of itself.
     *
     * The EMA is kept on top rather than replaced. It is what carries state
     * across the deadband and the cooldown, and the two filters answer
     * different questions: the median says where this unit is now, the average
     * says whether it has been there long enough to act on. Falling back to
     * the raw reading keeps the servo working through the first readings after
     * a splice or a start, which is exactly when it must not be idle.
     */
    const bool reset = (s_phase_stepped || !s_phase_valid);
    if (s_phase_stepped) {
        s_phase_stepped = false;
    }
    const int32_t err_in = s_phase_med_valid ? s_phase_med_us : s_phase_err_us;
    const int32_t err_ema = df_servo_ema(&s_servo, err_in, reset);

    size_t filled = LOCAL_RING_BYTES - xStreamBufferSpacesAvailable(local_ring);

    /* Printed once per CONFIG_DANCEFLOOR_LOG_PERIOD_S, not every window;
     * tx-fail accumulates across the quiet windows, so nothing is lost. */
    static int status_left;
    /*
     * ...except while the transmit path is in trouble, when it prints every
     * 5 s. A long window cannot order the two candidate stories for a transmit
     * stall -- "the queue filled and then refused" against "something refused
     * and the queue backed up behind it" -- if both events fall inside one of
     * them.
     *
     * Armed by the failure itself rather than by a build mode, so it costs
     * nothing on a clean run and needs no way to be turned on: the interesting
     * window is always the one after the one that went wrong. Held for ~60 s
     * past the last refusal, so the recovery is captured at the same
     * resolution as the onset.
     */
    static int fast_left;
    /* 5 s passes since the last print. The window is not a constant, and every
     * RATE on this line is a count divided by it, so it has to be measured
     * rather than assumed. */
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
         * every log captured before this instrument existed.
         *
         * Sized for the whole line, not a typical one. tx_fail_summary()
         * renders up to TX_ERR_SLOTS distinct errno names plus "other", and
         * strerror() strings run to ~32 characters -- four of them and a tail
         * is ~200. It CLEARS each slot as it renders it, so a truncated tally
         * is lost rather than deferred: the window would under-report and no
         * later window would make it up. */
        char why[224];
        tx_fail_summary(why, sizeof(why));

        /* Which lanes were refused, and -- only when something was -- whether
         * the refusals were beacon-spaced or one unbroken stall. See net.c. */
        char lanes[96];
        tx_fail_lanes(lanes, sizeof(lanes));

        /* Sized for the whole line for the same reason `why` is: worst case is
         * every counter at UINT32_MAX, six fields, ~190 bytes. Anything added
         * to tx_burst_summary() has to be added to this number too. */
        char burst[224];
        tx_burst_summary(burst, sizeof(burst));

        /* What the RADIO did, as opposed to what sendto() accepted -- see
         * tx_air_summary() in net.c for why a clean window still prints here.
         * Worst case is a 10-digit ms figure and two UINT32s, ~64 bytes. */
        char air[96];
        tx_air_summary(air, sizeof(air));

        /*
         * The station census, taken here rather than kept as a counter,
         * because association is a fact the driver already holds and a counter
         * would only be a stale copy of it.
         *
         * `stations` is the first thing to read on a bad line. The hub's
         * downlink packet rate scales with satellite count, so satellite count
         * and transmit load are the same variable, and the first thing to
         * establish about a transmit fault is that every unit was actually
         * associated throughout rather than flapping.
         *
         * rssi-min and phy-11n come free from the same call: the first says
         * whether the air changed under the fault, the second whether a
         * station associated without 11n, which would mean no aggregation to
         * it and a transmit queue that drains a frame at a time.
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
         * as a lead of zero. See n_lead_min_us and LEAD_UNSEEN. */
        char lead_s[16];
        const int32_t lead_min = n_lead_min_us;
        if (lead_min == LEAD_UNSEEN) {
            snprintf(lead_s, sizeof(lead_s), "none");
        } else {
            snprintf(lead_s, sizeof(lead_s), "%ld ms", (long)(lead_min / 1000));
        }

        /* Raw, median and average side by side: raw minus median IS the scatter
         * the median was added to reject, so a log carrying both is what says
         * whether this filter is earning its place. */
        ESP_LOGI(TAG, "local ring %u bytes (%lu ms) | phase %+ld us "
                      "(median %+ld%s, smoothed %+ld us) | "
                      "tx-fail %" PRIu32 " (%s)%s%s%s"
                      " | cong-skip %" PRIu32
                      " | stale-skip %" PRIu32
                      " | %lu pkts/s | fanout-gap-max %ld ms | lead-min %s"
                      " | stations %d | rssi-min %d | phy-11n %d | churn %" PRIu32
                      " | fec-k %d fec-tx %" PRIu32 " fec-cong %" PRIu32
                      " fec-skip %" PRIu32,
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
                 (int)CONFIG_DANCEFLOOR_AUDIO_FEC_K,
                 n_fec_sent, n_fec_cong_skip, n_fec_skipped);
        s_tx_fail = 0;
        /* The lane counters are cleared inside tx_fail_lanes(), on the pass
         * that rendered them, for the reason tx_fail_summary() is: the total
         * and its breakdown must not come to describe different windows. */
        s_audio_pkts = 0;
        n_tx_cong_skip = 0;
        /* Same window as the cong-skip it accompanies. It reads `stale-skip`
         * on the line because that is what it counts now -- frames stranded in
         * a batch a stopped stream never flushed. See hub.h. */
        n_tx_pace_skip = 0;
        /* Beside the packet rate they qualify: the rate says how many went,
         * these say whether they went evenly and with what margin. */
        n_fanout_gap_max_us = 0;
        n_lead_min_us = LEAD_UNSEEN;
        /* Same window as the refusals it is there to be correlated with. */
        n_join_churn = 0;
        /* The parity trio, cleared with the tx-fail they are read against: a
         * window where cong-skip rose and tx-fail did not is the trade
         * working, and that comparison only holds while both describe the same
         * 5 s. */
        n_fec_sent = 0;
        n_fec_cong_skip = 0;
        n_fec_skipped = 0;
    }

    const int32_t target = (int32_t)(LEAD_US / 1000) *
                           (int32_t)(rate_ema * AUDIO_CHANNELS * 2 / 1000);
    int32_t err_frames = ((int32_t)filled - target) / (AUDIO_CHANNELS * 2);
    int32_t depth_ms = err_frames * 1000 / (int32_t)rate_ema;

    /* This unit gives up here rather than carrying on to the depth net, which
     * is why df_servo_ema() is a separate call from df_servo_step(): the
     * average is still folded forward every window, and the early-out stays
     * visible in the unit it belongs to. */
    if (!s_phase_valid) {
        return;
    }

    /*
     * The catch-up arm requires the published median to be valid at the moment
     * of the write, which is re-tested here rather than taken from the read
     * above -- between the two sits the status print, milliseconds of UART,
     * ample for a splice to land in. The satellite takes its median inline and
     * needs no such re-test.
     *
     * catchup_held is the startup hold: a fresh timeline's first-minute phase
     * is past CATCHUP_ARM_US with nothing wrong, because the DMA refill
     * transient is in it. local_start is rewritten at every timeline start, so
     * the hold re-engages after an underrun restart too -- correct, since the
     * transient exists at every start.
     */
    const df_servo_in_t in = {
        .phase_valid    = true,
        .med_us         = err_in,
        .have_med       = s_phase_med_valid,
        .catchup_held   = esp_timer_get_time() - local_start < CATCHUP_HOLD_US,
        .depth_ms       = depth_ms,
        /* This unit's ring is fed over a stream buffer rather than the radio,
         * so it never showed the fault the satellite's post-anchor hold exists
         * for. */
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

    /* The debt only grows in the servo and only playback shrinks it. The write
     * races the play task's spend at 5 s against ~6 ms cadence, benign both
     * ways -- see the satellite's copy. */
    if (out.catchup_write) {
        catchup_frames = out.catchup_frames_new;
    }

    if (!out.act) {
        return;
    }
    if (out.coarse) {
        /*
         * COARSE: too big for software to absorb without shredding the audio,
         * so the clock has to move. The case it exists for is a source rate
         * far enough from the output that the ring drains in seconds; it costs
         * the channel-down the fine path was written to stop paying, and in
         * steady state never happens.
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
        /*
         * FINE: playback drops or duplicates one frame at a time -- no
         * channel-down, and continuous rather than stepped.
         *
         * Frames per second, which IS |trim_hz|: a trim of N Hz against a rate
         * of `rate` needs rate * N/rate = N extra frames every second. Printed
         * anyway, because it is the figure the n_trim_drops / n_trim_dups
         * deltas have to match, and reading that off a Hz value is one
         * inference more than a log should ask for.
         */
        ESP_LOGI(TAG, "servo: smoothed %+ld us (raw %+ld), buffer %+ld ms "
                      "-> trim %+ld Hz (%ld frames/s)",
                 (long)out.err_ema, (long)s_phase_err_us, (long)depth_ms,
                 (long)out.trim_hz,
                 (long)(out.trim_hz < 0 ? -out.trim_hz : out.trim_hz));
        rate_trim_hz = out.trim_hz;
    }
}

/* telemetry_tick() first, then servo_tick(): the reporting deliberately
 * precedes the "is anything playing" check above, because a stopped stream is
 * when the counters matter most. Declared in hub.h. */
void ring_monitor_task(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        telemetry_tick();
        servo_tick();
    }
}
