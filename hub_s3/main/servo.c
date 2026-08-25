
#include "hub.h"

#include "df_servo.h"

static df_servo_t s_servo;

void servo_tick(void)
{

    if (!s_playing || rate_ema == 0) {
        return;
    }

#if CONFIG_DANCEFLOOR_RETUNE_BENCH_S > 0

    static int bench_left;
    if (--bench_left <= 0) {
        bench_left = (CONFIG_DANCEFLOOR_RETUNE_BENCH_S + 4) / 5;
        ESP_LOGW(TAG, "BENCH: forcing a same-rate retune at %" PRIu32 " Hz", tx_rate);
        retune_dac(tx_rate);
        return;
    }
#endif

    const bool reset = (s_phase_stepped || !s_phase_valid);
    if (s_phase_stepped) {
        s_phase_stepped = false;
    }
    const int32_t err_in = s_phase_med_valid ? s_phase_med_us : s_phase_err_us;
    const int32_t err_ema = df_servo_ema(&s_servo, err_in, reset);

    size_t filled = LOCAL_RING_BYTES - xStreamBufferSpacesAvailable(local_ring);

    static int status_left;

    static int fast_left;

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

        char why[224];
        tx_fail_summary(why, sizeof(why));

        char lanes[96];
        tx_fail_lanes(lanes, sizeof(lanes));

        char burst[224];
        tx_burst_summary(burst, sizeof(burst));

        char air[96];
        tx_air_summary(air, sizeof(air));

        wifi_sta_list_t sta = {0};
        int8_t rssi_min = 0;
        int n_11n = 0;
        if (esp_wifi_ap_get_sta_list(&sta) != ESP_OK) {
            sta.num = -1;
        }
        for (int i = 0; i < sta.num; i++) {
            if (i == 0 || sta.sta[i].rssi < rssi_min) {
                rssi_min = sta.sta[i].rssi;
            }
            n_11n += sta.sta[i].phy_11n ? 1 : 0;
        }

        char lead_s[16];
        const int32_t lead_min = n_lead_min_us;
        if (lead_min == LEAD_UNSEEN) {
            snprintf(lead_s, sizeof(lead_s), "none");
        } else {
            snprintf(lead_s, sizeof(lead_s), "%ld ms", (long)(lead_min / 1000));
        }

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

        s_audio_pkts = 0;
        n_tx_cong_skip = 0;

        n_tx_pace_skip = 0;

        n_fanout_gap_max_us = 0;
        n_lead_min_us = LEAD_UNSEEN;

        n_join_churn = 0;

        n_fec_sent = 0;
        n_fec_cong_skip = 0;
        n_fec_skipped = 0;
    }

    const int32_t target = (int32_t)(LEAD_US / 1000) *
                           (int32_t)(rate_ema * AUDIO_CHANNELS * 2 / 1000);
    int32_t err_frames = ((int32_t)filled - target) / (AUDIO_CHANNELS * 2);
    int32_t depth_ms = err_frames * 1000 / (int32_t)rate_ema;

    if (!s_phase_valid) {
        return;
    }

    const df_servo_in_t in = {
        .phase_valid    = true,
        .med_us         = err_in,
        .have_med       = s_phase_med_valid,
        .catchup_held   = esp_timer_get_time() - local_start < CATCHUP_HOLD_US,
        .depth_ms       = depth_ms,

        .depth_net_held = false,
        .rate           = rate_ema,
        .tx_rate        = (int32_t)tx_rate,
        .trim_hz_now    = rate_trim_hz,
        .catchup_now    = catchup_frames,
    };
    df_servo_out_t out;
    df_servo_step(&s_servo, &in, &out);

    if (out.depth_net_fired) {
        ESP_LOGW(TAG, "depth net: buffer %+ld ms, phase asked %+ld Hz -> %+ld Hz",
                 (long)depth_ms, (long)out.adj_phase, (long)out.adj);
    }

    if (out.catchup_write) {
        catchup_frames = out.catchup_frames_new;
    }

    if (!out.act) {
        return;
    }
    if (out.coarse) {

        ESP_LOGI(TAG, "servo: smoothed %+ld us (raw %+ld), buffer %+ld ms "
                      "-> COARSE, DAC %" PRIu32 " Hz",
                 (long)out.err_ema, (long)s_phase_err_us, (long)depth_ms,
                 out.desired_rate);
        rate_trim_hz = 0;
        retune_dac(out.desired_rate);
    } else {

        ESP_LOGI(TAG, "servo: smoothed %+ld us (raw %+ld), buffer %+ld ms "
                      "-> trim %+ld Hz (%ld frames/s)",
                 (long)out.err_ema, (long)s_phase_err_us, (long)depth_ms,
                 (long)out.trim_hz,
                 (long)(out.trim_hz < 0 ? -out.trim_hz : out.trim_hz));
        rate_trim_hz = out.trim_hz;
    }
}

void ring_monitor_task(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        telemetry_tick();
        servo_tick();
    }
}
