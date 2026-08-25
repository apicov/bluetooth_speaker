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
#include "df_servo.h"
#include "sbc_link.h"
#include "sbc_decoder.h"
#include "visualiser.h"
#include "wifi_log.h"

#include "sat.h"

static df_servo_t s_servo;

void servo_tick(void)
{
    if (stream_start_local == 0) {
        return;
    }

#if CONFIG_DANCEFLOOR_RETUNE_BENCH_S > 0

    static int bench_left;
    if (--bench_left <= 0) {
        bench_left = (CONFIG_DANCEFLOOR_RETUNE_BENCH_S + 4) / 5;
        ESP_LOGW(TAG, "BENCH: forcing a same-rate retune at %" PRIu32 " Hz", tx_rate);
        retune_output(tx_rate);
        return;
    }
#endif

    size_t filled = RING_BYTES - xStreamBufferSpacesAvailable(ring);
    int32_t target = (int32_t)(RING_TARGET_MS *
                               (stream_rate * AUDIO_CHANNELS * 2 / 1000));
    int32_t err_frames = ((int32_t)filled - target) / (AUDIO_CHANNELS * 2);

    int32_t ph = phase_valid ? phase_err_us : 0;
    const bool stepped = phase_stepped;
    if (phase_stepped) {
        phase_stepped = false;
    }
    const int32_t err_ema = df_servo_ema(&s_servo, ph, stepped);

    int32_t med = phase_err_us;
    const bool have_med = sync_phase_median(&phase_hist, &med);

    static int status_left;
    if (--status_left <= 0) {
        status_left = CONFIG_DANCEFLOOR_LOG_PERIOD_S / 5;
        ESP_LOGI(TAG, "buffer %lu ms | phase %+ld us (median %+ld%s, smoothed %+ld us)"
                      " | fec-k %d",
                 (unsigned long)(filled * 1000 / (stream_rate * AUDIO_CHANNELS * 2)),
                 (long)ph, (long)(have_med ? med : 0), have_med ? "" : " n/a",
                 (long)err_ema,
                 (int)CONFIG_DANCEFLOOR_AUDIO_FEC_K);
    }

    const int64_t since_anchor = anchor_at ? esp_timer_get_time() - anchor_at
                                           : INT64_MAX;

    const df_servo_in_t in = {
        .phase_valid    = phase_valid,
        .med_us         = med,
        .have_med       = have_med,
        .catchup_held   = esp_timer_get_time() - stream_start_local
                          < CATCHUP_HOLD_US,
        .depth_ms       = err_frames * 1000 / (int32_t)stream_rate,
        .depth_net_held = since_anchor < DEPTH_NET_HOLD_US,
        .rate           = stream_rate,
        .tx_rate        = (int32_t)tx_rate,
        .trim_hz_now    = rate_trim_hz,
        .catchup_now    = catchup_frames,
    };
    df_servo_out_t out;
    df_servo_step(&s_servo, &in, &out);

    if (out.depth_net_fired) {
        ESP_LOGW(TAG, "depth net: buffer %+ld ms, phase asked %+ld Hz -> %+ld Hz",
                 (long)in.depth_ms, (long)out.adj_phase, (long)out.adj);
    }

    if (out.catchup_write) {
        catchup_frames = out.catchup_frames_new;
    }

    if (!out.act) {
        return;
    }
    if (out.coarse) {

        ESP_LOGI(TAG, "servo: smoothed %+ld us -> COARSE, output %" PRIu32 " Hz",
                 (long)out.err_ema, out.desired_rate);
        rate_trim_hz = 0;
        retune_output(out.desired_rate);
    } else {

        ESP_LOGI(TAG, "servo: smoothed %+ld us -> trim %+ld Hz "
                      "(%ld frames/s)",
                 (long)out.err_ema, (long)out.trim_hz,
                 (long)(out.trim_hz < 0 ? -out.trim_hz : out.trim_hz));
        rate_trim_hz = out.trim_hz;
    }
}
