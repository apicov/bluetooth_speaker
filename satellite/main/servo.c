/**
 * @file servo.c
 * @brief The output-rate servo: hold this speaker's position in the timeline
 *        by trimming the rate audio is consumed at.
 *
 * The loop arithmetic is not here. It lives in
 * components/dancefloor_sync/df_servo.c, which the hub runs too, because a
 * correction rate that differs between the units is a cross-unit sync error by
 * construction — sharing the code makes that structural rather than
 * remembered.
 *
 * What is left here is what is genuinely this unit's: when a stream counts as
 * running, how deep the ring is, which actuator a correction reaches, and
 * every log line.
 */
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

/** @brief The smoothed error and the cooldown, which must survive between 5 s
 *  windows. */
static df_servo_t s_servo;

/* servo_tick() is documented at its declaration in sat.h. */
void servo_tick(void)
{
    if (stream_start_local == 0) {
        return;
    }

#if CONFIG_DANCEFLOOR_RETUNE_BENCH_S > 0
    /* Bench: retune to the rate we are already at. Nothing about the audio
     * changes, so whatever the retune cost line then reports is the cost of
     * retuning alone -- no rate change, no drift, no track boundary in the
     * way. Run it on one unit and leave the other as the reference. */
    static int bench_left;
    if (--bench_left <= 0) {
        bench_left = (CONFIG_DANCEFLOOR_RETUNE_BENCH_S + 4) / 5;
        ESP_LOGW(TAG, "BENCH: forcing a same-rate retune at %" PRIu32 " Hz", tx_rate);
        retune_output(tx_rate);
        return;                        /* do not also servo this window */
    }
#endif

    size_t filled = RING_BYTES - xStreamBufferSpacesAvailable(ring);
    int32_t target = (int32_t)(RING_TARGET_MS *
                               (stream_rate * AUDIO_CHANNELS * 2 / 1000));
    int32_t err_frames = ((int32_t)filled - target) / (AUDIO_CHANNELS * 2);

    /* Smoothed phase error is what the servo acts on. Buffer depth is kept
     * only as a safety net against underrun or overflow, which phase control
     * alone would not notice until it was too late.
     *
     * The RAW reading, not the median -- which is where this unit differs from
     * the hub. Whether it should also smooth is a question with an answer in
     * the soak logs, and not one to settle as a side effect of sharing code. */
    int32_t ph = phase_valid ? phase_err_us : 0;
    const bool stepped = phase_stepped;
    if (phase_stepped) {
        phase_stepped = false;
    }
    const int32_t err_ema = df_servo_ema(&s_servo, ph, stepped);

    /* The catch-up arm's input, taken inline just above the decision it gates,
     * so there is no window between the read and the write for a splice to
     * land in and no re-test is needed. sync_phase_median() leaves med alone
     * when the history is too short, so the raw reading survives as the
     * fallback. */
    int32_t med = phase_err_us;
    const bool have_med = sync_phase_median(&phase_hist, &med);

    /* Once per log period, not every window: the servo below still runs at 5 s
     * and still sees every sample, it just stops narrating. Raw, median and
     * smoothed side by side, in the hub's format -- the median is printed here
     * and not acted on, which is what makes the two units' inputs comparable
     * in a single log. */
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

    /* The depth net is held off for DEPTH_NET_HOLD_US after an anchor, because
     * for that stretch the depth is not evidence of anything the net exists to
     * catch: a fresh stream starts below target by construction, and the net
     * would drag the rate to rescue a ring that is merely still filling. */
    const int64_t since_anchor = anchor_at ? esp_timer_get_time() - anchor_at
                                           : INT64_MAX;

    /* The catch-up arm is also held for CATCHUP_HOLD_US after a stream starts:
     * an anchored stream's startup phase is large with nothing wrong. It keys
     * off stream_start_local rather than anchor_at so the hold re-engages on a
     * re-anchor, which is when the transient recurs. */
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

    /* Say so when the net is in play: a log window has no other way to show
     * that the phase correction was capped rather than agreed with. */
    if (out.depth_net_fired) {
        ESP_LOGW(TAG, "depth net: buffer %+ld ms, phase asked %+ld Hz -> %+ld Hz",
                 (long)in.depth_ms, (long)out.adj_phase, (long)out.adj);
    }

    /* The debt only grows here and only playback shrinks it, as it spends.
     * These writes race the play task's decrements at 5 s against a ~6 ms
     * cadence, benign in both directions: the next window re-arms from a fresh
     * median either way. */
    if (out.catchup_write) {
        catchup_frames = out.catchup_frames_new;
    }

    if (!out.act) {
        return;
    }
    if (out.coarse) {
        /* COARSE: too big for software to absorb without shredding the audio,
         * so the clock has to move. On this unit that is how a stream is first
         * matched at all -- i2s_start() runs at a hardcoded 44100 before any
         * stream exists, so a source at a different rate arrives here as one
         * large step. The trim is cleared because the clock now carries what
         * it was carrying; leaving it set would apply the correction twice. */
        ESP_LOGI(TAG, "servo: smoothed %+ld us -> COARSE, output %" PRIu32 " Hz",
                 (long)out.err_ema, out.desired_rate);
        rate_trim_hz = 0;
        retune_output(out.desired_rate);
    } else {
        /* FINE: playback drops or duplicates one frame at a time. No
         * channel-down, and continuous rather than stepped. The frames per
         * second it costs IS |trim_hz|, by construction. */
        ESP_LOGI(TAG, "servo: smoothed %+ld us -> trim %+ld Hz "
                      "(%ld frames/s)",
                 (long)out.err_ema, (long)out.trim_hz,
                 (long)(out.trim_hz < 0 ? -out.trim_hz : out.trim_hz));
        rate_trim_hz = out.trim_hz;
    }
}
