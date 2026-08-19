/*
 * The output-rate servo: hold this speaker's position in the timeline by
 * trimming the rate audio is consumed at.
 *
 * "Consumed at", not "the DAC runs at", since 2026-08-14. The loop is unchanged
 * -- same input, same gain, same deadband, same cooldown -- but its output now
 * normally goes to rate_trim_hz, which playback applies by dropping or
 * duplicating one frame at a time, rather than to retune_output(), which takes
 * the I2S channel down for several milliseconds to do it. The clock is still
 * what a COARSE rate match uses; see RATE_TRIM_MAX_HZ for the boundary.
 *
 * Split out of main.c on 2026-08-12, and split again on 2026-08-19: THE LOOP
 * ITSELF NOW LIVES IN components/dancefloor_sync/df_servo.c, which the hub runs
 * too. It was two copies of the same arithmetic, and both copies said in as many
 * words that they must never disagree -- a correction rate that differs between
 * the units is a cross-unit sync error by construction. df_servo.h says why that
 * is now structural rather than remembered, and test_servo.c pins the branches
 * that soaks found the hard way.
 *
 * What is left here is what is genuinely this unit's: when a stream counts as
 * running, how deep the ring is, which actuator a coarse correction reaches, and
 * every log line. The measurements that used to sit above each branch went with
 * the branch; the ones below stayed because they are about this unit.
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

/*
 * The smoothed error and the cooldown, which must survive between 5 s windows.
 * They were file-scope locals of this translation unit for exactly that reason
 * and still are -- only their storage moved into the shared struct.
 */
static df_servo_t s_servo;

void servo_tick(void)
{
    if (stream_start_local == 0) {
        return;
    }

#if CONFIG_DANCEFLOOR_RETUNE_BENCH_S > 0
    /*
     * Bench: retune to the rate we are already at. Nothing about the audio
     * changes, so whatever the RETUNE COST line then reports is the cost of
     * retuning alone -- no rate change, no drift, no track boundary in the
     * way. Run it on one unit and leave the other as the reference.
     */
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

    /*
     * Smoothed phase error is what the servo acts on. Buffer depth is kept only
     * as a safety net against underrun or overflow, which phase control alone
     * would not notice until it was too late.
     *
     * THE RAW READING, not the median -- which is where this unit differs from
     * the hub, deliberately. See the note at the top of df_servo.h: whether this
     * unit should also smooth the median is a real question with an answer in
     * the soak logs, and not one to settle as a side effect of sharing code.
     */
    int32_t ph = phase_valid ? phase_err_us : 0;
    const bool stepped = phase_stepped;
    if (phase_stepped) {
        phase_stepped = false;
    }
    const int32_t err_ema = df_servo_ema(&s_servo, ph, stepped);

    /*
     * The catch-up arm's input. Taken inline, microns above the decision it
     * gates, so unlike the hub's copy there is no window between the read and
     * the write for a splice to land in and no re-test is needed.
     *
     * sync_phase_median() leaves `med` alone when the history is too short, so
     * the raw reading survives as the STAND-DOWN's input -- which is what
     * `have_med` gating the arm alone preserves.
     *
     * Read here rather than below the status print only so the print can show
     * it. The call takes a const pointer and has no side effects, so where it
     * sits makes no difference to anything but the log.
     */
    int32_t med = phase_err_us;
    const bool have_med = sync_phase_median(&phase_hist, &med);

    /*
     * Once per log period, not every window. The servo below still runs at
     * 5 s and still sees every sample; it just stops narrating.
     *
     * RAW, MEDIAN AND SMOOTHED SIDE BY SIDE, in the hub's own format and for
     * the hub's own reason. The median is PRINTED here and not acted on: this
     * unit's servo runs on the raw reading and the hub's runs on the median,
     * and until 2026-08-19 no soak could say whether that divergence matters,
     * because only one of the two units reported both numbers.
     *
     * Raw minus median IS the scatter the hub's filter exists to reject. The
     * hub's 15.7 ms of swing is what put it there; on the 2026-08-19 soak the
     * hub's own raw and median tracked to 56 us mean, and this unit's phase was
     * a clean ~100 us/window ramp -- but that run carried no tx-fail burst, and
     * a burst is the condition the median was added for. So the question stays
     * open and this line is what closes it: if this unit scatters like the hub
     * under load it should take the median too, and if neither does the hub
     * should drop it. Either way the divergence ends on a measurement.
     */
    static int status_left;
    if (--status_left <= 0) {
        status_left = CONFIG_DANCEFLOOR_LOG_PERIOD_S / 5;
        ESP_LOGI(TAG, "buffer %lu ms | phase %+ld us (median %+ld%s, smoothed %+ld us)"
                      " | xport %s fec %d",
                 (unsigned long)(filled * 1000 / (stream_rate * AUDIO_CHANNELS * 2)),
                 (long)ph, (long)(have_med ? med : 0), have_med ? "" : " n/a",
                 (long)err_ema,
                 AUDIO_TRANSPORT_TAG, (int)CONFIG_DANCEFLOOR_AUDIO_FEC_DEPTH);
    }

    /*
     * The depth net is held off for DEPTH_NET_HOLD_US after an anchor, because
     * for that stretch the depth is not evidence of anything the net exists to
     * catch.
     *
     * A fresh stream starts below target by construction. RING_TARGET_MS is 250,
     * matching the hub's LEAD_US -- both were 200 when the measurements were
     * taken, and both moved together -- but the deepest prefill an anchor can
     * ever buy is that lead MINUS transit; the scheduled wait is the only thing
     * that fills the ring before playback begins, so the best measured start was
     * 154 ms and a 107 ms one is unremarkable. The reading is lower still in the
     * first moments, because playback has begun consuming while the rest of the
     * stream is in flight: a run read `buffer 40 ms` 100 ms after playback
     * started, from a 107 ms prefill.
     *
     * The net fired on exactly that, asking for -20 Hz to rescue a ring that was
     * not in trouble. It moved the clock in those days; it moves the software
     * trim now, but the hold is unchanged and so is the reason for it. The
     * stream then spent 110 s and six retunes walking off the phase excursion it
     * caused -- peak +48 ms, with the visualiser rendering 7% of its frames late
     * while the sound ran behind the timeline the lights were drawn on.
     *
     * 20 s is four servo windows and matches the retune cooldown, by which point
     * the phase measurement is trustworthy and is the better input anyway.
     * Nothing about underrun protection is given up here: an actually empty ring
     * is caught by the playback task's 500 ms receive timeout, which is a
     * different mechanism and still armed.
     */
    const int64_t since_anchor = anchor_at ? esp_timer_get_time() - anchor_at
                                           : INT64_MAX;
    /*
     * The arm is also held for CATCHUP_HOLD_US after a stream starts: an
     * anchored stream's startup phase is past CATCHUP_ARM_US with nothing wrong
     * -- this unit armed 1448 frames of replay in the first window of the
     * 2026-08-18 soak -- and stream_start_local rewrites at every anchor, so the
     * hold re-engages on a re-anchor, which is when the transient recurs.
     */
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

    /*
     * Say so when the net is in play, because a log window has no other way to
     * show it. Under the pre-2026-08-15 code the two numbers were the whole
     * story: `phase 36 -> net 20` was the cap that commit exists to stop.
     */
    if (out.depth_net_fired) {
        ESP_LOGW(TAG, "depth net: buffer %+ld ms, phase asked %+ld Hz -> %+ld Hz",
                 (long)in.depth_ms, (long)out.adj_phase, (long)out.adj);
    }

    /*
     * The debt only grows in the servo and only playback shrinks it, as it
     * spends. Writes race the play task's decrements at 5 s against ~6 ms
     * cadence, benign in both directions by the same reasoning as rate_trim_hz:
     * the next window re-arms from a fresh median either way.
     */
    if (out.catchup_write) {
        catchup_frames = out.catchup_frames_new;
    }

    if (!out.act) {
        return;
    }
    if (out.coarse) {
        /*
         * COARSE: too big for software to absorb without shredding the audio, so
         * the clock has to move. On this unit that is how a stream is first
         * matched at all -- i2s_start() ran with a hardcoded 44100 before any
         * stream existed, so a source measured at ~42600 arrives here as a
         * ~1500 Hz step.
         *
         * The trim is cleared because the clock now carries what it was
         * carrying; leaving it set would apply the correction twice.
         */
        ESP_LOGI(TAG, "servo: smoothed %+ld us -> COARSE, output %" PRIu32 " Hz",
                 (long)out.err_ema, out.desired_rate);
        rate_trim_hz = 0;
        retune_output(out.desired_rate);
    } else {
        /* FINE: playback drops or duplicates one frame at a time. No
         * channel-down, and continuous rather than stepped. */
        /* Frames per second, which IS |trim_hz| -- see the hub's copy,
         * including what this printed before it was right. */
        ESP_LOGI(TAG, "servo: smoothed %+ld us -> trim %+ld Hz "
                      "(%ld frames/s)",
                 (long)out.err_ema, (long)out.trim_hz,
                 (long)(out.trim_hz < 0 ? -out.trim_hz : out.trim_hz));
        rate_trim_hz = out.trim_hz;
    }
}
