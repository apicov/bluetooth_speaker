/*
 * The output-rate servo: hold this speaker's position in the timeline by
 * trimming the rate audio is consumed at.
 *
 * "Consumed at", not "the DAC runs at", since 2026-08-14. The loop below is
 * unchanged -- same input, same gain, same deadband, same cooldown -- but its
 * output now normally goes to rate_trim_hz, which playback applies by dropping
 * or duplicating one frame at a time, rather than to retune_output(), which
 * takes the I2S channel down for several milliseconds to do it. The clock is
 * still what a COARSE rate match uses; see RATE_TRIM_MAX_HZ for the boundary.
 *
 * Split out of main.c on 2026-08-12. The body is unchanged apart from one level
 * of indentation and the `continue` statements, which skipped to the next 5 s
 * window and are `return` now that the window is a function call. The three
 * values the loop carried between windows were locals of drift_task and are
 * file-scope here for the same reason: they must survive the call.
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
#include "sbc_link.h"
#include "sbc_decoder.h"
#include "visualiser.h"
#include "wifi_log.h"

#include "sat.h"

static int32_t err_ema;
static bool    err_ema_valid;
/*
 * Windows to wait after a retune before considering another. The buffer takes
 * tens of seconds to respond, and acting again before it has is how a servo
 * ends up chasing its own corrections.
 */
static int cooldown;

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
     * Smooth before acting, and separate two things that look identical in a
     * single reading:
     *
     *   jitter -- delivery is bursty, so the level swings +-40 ms with no
     *             trend. Correcting for it would retune constantly, and each
     *             retune is an audible click.
     *   drift  -- the crystals genuinely differ (~14 ppm measured), so the
     *             level walks steadily in one direction.
     *
     * Averaging kills the first and leaves the second. The old code coped by
     * using a wide deadband instead, which meant real drift could reach
     * ~60 ms before anything happened -- clearly audible echo, and about 75
     * minutes away at 14 ppm. Fine in a short test, wrong over an evening.
     */
    /* Smoothed phase error is what the servo acts on now. Buffer depth is
     * kept only as a safety net against underrun or overflow, which phase
     * control alone would not notice until it was too late. */
    int32_t ph = phase_valid ? phase_err_us : 0;
    if (phase_stepped) {
        phase_stepped = false;
        err_ema_valid = false;       /* history describes a different world */
    }
    err_ema = err_ema_valid ? (err_ema * 3 + ph) / 4 : ph;
    err_ema_valid = true;

    /* Once per log period, not every window. The servo above still runs at
     * 5 s and still sees every sample; it just stops narrating. */
    static int status_left;
    if (--status_left <= 0) {
        status_left = CONFIG_DANCEFLOOR_LOG_PERIOD_S / 5;
        ESP_LOGI(TAG, "buffer %lu ms | phase %+ld us (smoothed %+ld us) | xport %s fec %d",
                 (unsigned long)(filled * 1000 / (stream_rate * AUDIO_CHANNELS * 2)),
                 (long)ph, (long)err_ema,
                 AUDIO_TRANSPORT_TAG, (int)CONFIG_DANCEFLOOR_AUDIO_FEC_DEPTH);
    }

    /*
     * Late (positive error) means we are behind the timeline, so play
     * faster.
     *
     * Spread over ~100 s, not 40. The buffer takes tens of seconds to
     * respond, so a 40 s loop was still correcting after the error had gone
     * and sailed past it -- both units converged to near zero then
     * overshot to +10 ms and oscillated. Real drift is only ~0.8 ms per
     * minute, so the loop can afford to be much gentler than the
     * disturbance it corrects.
     */
    int32_t adj = (int32_t)((int64_t)err_ema * stream_rate / 100000000LL);
    /* Belt and braces against the arithmetic above, which has produced
     * -138000 once already. retune_output() refuses anything wilder, but
     * the number should not get that far in the first place. */
    if (adj >  RATE_TRIM_MAX_HZ) adj =  RATE_TRIM_MAX_HZ;
    if (adj < -RATE_TRIM_MAX_HZ) adj = -RATE_TRIM_MAX_HZ;

    /*
     * Safety net: if the buffer is heading for empty or full, that matters
     * more than phase.
     *
     * Held off for DEPTH_NET_HOLD_US after an anchor, because for that
     * stretch the depth is not evidence of anything the net exists to
     * catch.
     *
     * A fresh stream starts below target by construction. RING_TARGET_MS is
     * 200, matching the hub's LEAD_US, but the deepest prefill an anchor can
     * ever buy is that lead MINUS transit -- the scheduled wait is the only
     * thing that fills the ring before playback begins, so the best measured
     * start was 154 ms and a 107 ms one is unremarkable. The reading is
     * lower still in the first moments, because playback has begun consuming
     * while the rest of the stream is in flight: a run read `buffer 40 ms`
     * 100 ms after playback started, from a 107 ms prefill.
     *
     * The net fired on exactly that, asking for -20 Hz to rescue a ring that
     * was not in trouble. It moved the clock in those days; it moves the
     * software trim now, but the hold is unchanged and so is the reason for
     * it. The stream then spent 110 s and six retunes walking off the phase
     * excursion it caused -- peak +48 ms, with the visualiser rendering 7% of
     * its frames late while the sound ran behind the timeline the lights were
     * drawn on.
     *
     * 20 s is four servo windows and matches the retune cooldown, by which
     * point the phase measurement is trustworthy and is the better input
     * anyway. Nothing about underrun protection is given up here: an
     * actually empty ring is caught by the playback task's 500 ms receive
     * timeout, which is a different mechanism and still armed.
     */
    /*
     * A FLOOR, NOT A REPLACEMENT, since 2026-08-15.
     *
     * This used to assign adj outright, which meant it could only ever WEAKEN a
     * phase correction that already agreed with it. Measured on the soak in
     * tools/soak/logs-soak-20260815-224002, in this unit's own words:
     *
     *   buffer 449 ms | phase +268516 us
     *   servo: smoothed +83432 us -> trim +20 Hz (20 frames/s)
     *
     * The phase term asked for 83432 * 44100 / 1e8 = 36 Hz and got 20, because
     * the ring was 249 ms past target and this branch overwrote it. So at the
     * exact moment the ring was deepest, the guard that exists to protect the
     * ring cut the recovery to 0.45 ms/s -- and a ring that deep IS playing
     * that late, so the two were asking for the same thing and the guard won
     * anyway. Recovery then took minutes and a track boundary ended it first.
     *
     * Depth still WINS when the two disagree, which is the case this was
     * written for: a ring heading for empty or full while phase reads fine.
     * When they agree, the larger correction stands.
     *
     * No steady-state effect. Depth only leaves +-120 ms during exactly the
     * events this is about; every log window that ever read `buffer 165-250 ms`
     * never reached this branch and still does not.
     */
    const int64_t since_anchor = anchor_at ? esp_timer_get_time() - anchor_at
                                           : INT64_MAX;
    const int32_t depth_ms = err_frames * 1000 / (int32_t)stream_rate;
    const int32_t adj_phase = adj;       /* what phase alone asked for */
    if (since_anchor < DEPTH_NET_HOLD_US) {
        /* say nothing; the buffer line above already prints the depth */
    } else if (depth_ms < -120) {
        if (adj > -20) adj = -20;        /* nearly empty: slow down, at least */
    } else if (depth_ms > 120) {
        if (adj < 20) adj = 20;          /* nearly full: speed up, at least */
    }
    /*
     * Say so when the net is in play, because this is the branch that changed
     * and a log window has no other way to show it. Under the old code the two
     * numbers were the whole story: `phase 36 -> net 20` was the cap, and is
     * what this commit exists to stop. Rare by construction -- depth only
     * leaves +-120 ms during a delivery burst.
     */
    if (adj != adj_phase) {
        ESP_LOGW(TAG, "depth net: buffer %+ld ms, phase asked %+ld Hz -> %+ld Hz",
                 (long)depth_ms, (long)adj_phase, (long)adj);
    }
    uint32_t desired = (uint32_t)((int32_t)stream_rate + adj);
    /*
     * Deadband, stated in phase error rather than in rate.
     *
     * It used to be tx_rate/5000, described in a comment as "~8 ms of
     * accumulated drift before a correction". That read 0.02% of the sample
     * rate as if it were milliseconds: 8 Hz of threshold is really 8e8/44100
     * = ~20 ms of phase, per unit and in either direction.
     *
     * Affordable at 7 ms because a retune is cheap now -- measured at 1 to 7
     * ms, essentially just the channel outage. It was not while this task
     * spun through the ring during that outage; see PHASE_DEADBAND_US.
     */
    if (!phase_valid) {
        return;                        /* nothing measured yet */
    }
    int32_t deadband = (int32_t)((int64_t)PHASE_DEADBAND_US * stream_rate / 100000000LL);
    if (deadband < 1) {
        deadband = 1;
    }
    /*
     * The correction, as an offset from the rate the CLOCK is actually running
     * at. This is the number that used to be handed to retune_output() as an
     * absolute rate; splitting it out is what lets the two actuators be chosen
     * between, because their boundary is a size and not a kind.
     */
    const int32_t trim_hz = (int32_t)desired - (int32_t)tx_rate;

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
                 * audio, so the clock has to move. On this unit that is how a
                 * stream is first matched at all -- i2s_start() ran with a
                 * hardcoded 44100 before any stream existed, so a source
                 * measured at ~42600 arrives here as a ~1500 Hz step.
                 *
                 * The trim is cleared because the clock now carries what it was
                 * carrying; leaving it set would apply the correction twice.
                 */
                ESP_LOGI(TAG, "servo: smoothed %+ld us -> COARSE, output %" PRIu32 " Hz",
                         (long)err_ema, desired);
                rate_trim_hz = 0;
                retune_output(desired);
            } else {
                /* FINE: playback drops or duplicates one frame at a time. No
                 * channel-down, and continuous rather than stepped. */
                /* Frames per second, which IS |trim_hz| -- see the hub's copy,
                 * including what this printed before it was right. */
                ESP_LOGI(TAG, "servo: smoothed %+ld us -> trim %+ld Hz "
                              "(%ld frames/s)",
                         (long)err_ema, (long)trim_hz,
                         (long)(trim_hz < 0 ? -trim_hz : trim_hz));
                rate_trim_hz = trim_hz;
            }
            cooldown = 4;        /* ~20 s, against a 40 s correction time */
        }
    }
}
