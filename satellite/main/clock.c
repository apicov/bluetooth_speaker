/*
 * Which clock this unit believes, and keeping the conversion current.
 *
 * The probe task that feeds the estimator, the TSF-or-estimator choice every
 * anchor makes, and the slew that stops the offset going stale mid-stream.
 *
 * Split out of main.c on 2026-08-12; the bodies are unchanged.
 */
#include <inttypes.h>
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

/* --------------------------------------------------------------- receiving */

/*
 * One tick of the self-mute state machine. True means "stay off the air".
 *
 * WHAT IT WATCHES, AND WHY IT IS NOT STARVATION. The first version keyed on the
 * DAC starving while `playing`, and on the 2026-08-25 antenna tests it never
 * fired once -- because a satellite that has gone deaf does not starve. It
 * PARKS. The play task gives up, `playing` goes false, and out.c stops counting
 * starvation with it, so the detector was blind by construction in exactly the
 * state it existed for:
 *
 *     sat_s3   pkts 3 | starved 667 ms | lead-min -191 ms
 *              pkts 1 | starved   0 ms | lead-min none | ring-low none
 *              pkts 0 | starved   0 ms | hub-rssi -96 dBm
 *
 * `lead-min none` and `ring-low none` are the gauges the playback loop sets, so
 * their absence is the park. Meanwhile probe_task kept probing -- uplink still
 * works fine at -96 dBm -- so the hub held it as a client (`stations 3`) and
 * went on unicasting audio at it: `tx-fail 521 (470 audio)` with txdone
 * collapsed from ~3080 to 34, and sat_classic down to `pkts 9`.
 *
 * So the signal is AUDIO NOT ARRIVING, which is true whether the unit is
 * playing, parked, or somewhere between.
 *
 * RSSI IS THE SECOND HALF, and it is what stops an idle floor muting itself.
 * "No audio" alone cannot tell "the hub is not sending" from "I cannot hear the
 * hub", and the first is every gap between tracks. A unit that is associated
 * with a workable signal and receiving nothing is simply idle and must stay
 * registered; one receiving nothing at MUTE_RSSI_FLOOR is deaf. Measured, the
 * two are 30 dB apart -- healthy units read -44 and -66, the deaf one -96 --
 * so the floor is nowhere near either.
 */
static bool mute_tick(void)
{
    static uint32_t prev_rx;
    static uint16_t hist[MUTE_SLOTS];   /* audio datagrams per tick, sliding */
    static uint32_t hist_sum;           /* their total, kept incrementally */
    static int      slot;
    static int64_t  s_muted_at;         /* when it went off the air; 0 = on */
    static int64_t  s_trial_until;      /* grace after coming back */

    const int64_t now = esp_timer_get_time();

    /* Slide the window: this tick's arrivals in, the oldest tick's out. */
    const uint32_t rx = n_audio_rx;
    const uint32_t delta = rx - prev_rx;
    prev_rx = rx;
    hist_sum -= hist[slot];
    hist[slot] = (uint16_t)(delta > UINT16_MAX ? UINT16_MAX : delta);
    hist_sum += hist[slot];
    slot = (slot + 1) % MUTE_SLOTS;

    if (s_muted_at) {
        if (now - s_muted_at < MUTE_RETRY_US) {
            return true;
        }
        /*
         * Try again. The history is cleared with the timers: while muted this
         * unit is off the send list and receiving nothing BY DESIGN, so
         * carrying that forward would re-mute the trial on its first tick.
         */
        memset(hist, 0, sizeof(hist));
        hist_sum = 0;
        s_muted_at = 0;
        s_trial_until = now + MUTE_TRIAL_US;
        self_muted = false;
        n_self_retries++;
        ESP_LOGW(TAG, "trying the floor again after %d s off it",
                 (int)(MUTE_RETRY_US / 1000000));
        return false;
    }

    if (now < s_trial_until || hist_sum >= MUTE_AUDIO_MIN) {
        return false;                   /* being given its chance, or receiving */
    }

    /*
     * Nothing is arriving. Deaf, or just an idle floor? Only the radio can say,
     * and it is asked HERE rather than every tick because this point is reached
     * only after MUTE_WINDOW_US of silence -- rare, where the tick is not.
     */
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
        return false;                   /* not associated: already off the list */
    }
    if (ap.rssi > MUTE_RSSI_FLOOR) {
        return false;                   /* heard well enough; the hub is quiet */
    }

    /*
     * Off. The hub drops this unit CLIENT_TIMEOUT_US after the probe that is
     * not about to be sent, and stops spending airtime on frames it cannot
     * acknowledge -- which is airtime every other speaker gets back.
     */
    memset(hist, 0, sizeof(hist));
    hist_sum = 0;
    s_muted_at = now;
    self_muted = true;
    n_self_mutes++;
    ESP_LOGE(TAG, "MUTING: %" PRIu32 " audio packets in %d s at %d dBm -- deaf, "
                  "not idle. Leaving the hub's send list so it stops retrying "
                  "frames I cannot acknowledge; trying again every %d s.",
             hist_sum, (int)(MUTE_WINDOW_US / 1000000), (int)ap.rssi,
             (int)(MUTE_RETRY_US / 1000000));
    return true;
}

void probe_task(void *arg)
{
    (void)arg;
    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port = htons(SYNC_PORT),
        .sin_addr.s_addr = inet_addr(MASTER_IP),
    };
    uint32_t seq = 0;

    while (1) {
        /*
         * The reconnect, riding this loop's period.
         *
         * First, before the probe: while the unit is off the air the sendto
         * below goes nowhere, so getting the association back is the only thing
         * this task can usefully do with the tick. It is a compare and a return
         * on a unit that is joined, which is every tick but the ones that matter.
         *
         * net.c owns the policy -- when to try and what to wait -- and this only
         * supplies the heartbeat. See wifi_retry_tick() for why the wait is no
         * longer taken on the event loop's task.
         */
        wifi_retry_tick();

        /*
         * Whether this unit is fit to be on the floor at all, decided on this
         * task's tick because this task is what puts it there -- see self_muted
         * in sat.h for the measurement that made it necessary.
         *
         * Both sends are gated, not just the probe. MSG_SPLICE reaches
         * client_seen() on the hub exactly as MSG_TIME_REQ does, so a track
         * boundary landing during a mute would re-register the unit for another
         * CLIENT_TIMEOUT_US and undo it, once per track, for as long as the
         * fault lasted.
         */
        if (mute_tick()) {
            vTaskDelay(pdMS_TO_TICKS(PROBE_PERIOD_MS));
            continue;
        }

        time_msg_t msg = { .type = MSG_TIME_REQ, .seq = seq++, .t1 = esp_timer_get_time() };
        sendto(sock, &msg, sizeof(msg), 0, (struct sockaddr *)&dest, sizeof(dest));

        /* Piggyback any track-boundary correction on the same socket. Cleared
         * before sending, so a report landing while this runs is kept rather
         * than overwritten by the clear. */
        if (splice_report_pending) {
            splice_report_pending = false;
            splice_msg_t s = {
                .type = MSG_SPLICE,
                .applied_us = splice_report_us,
                .phase_us = splice_report_phase,
                .applied_alt_us = splice_report_alt,
            };
            sendto(sock, &s, sizeof(s), 0, (struct sockaddr *)&dest, sizeof(dest));
        }

        vTaskDelay(pdMS_TO_TICKS(PROBE_PERIOD_MS));
    }
}

/*
 * Which clock offset to believe.
 *
 * TSF when it is fresh, the probe estimator otherwise. TSF has no round trip in
 * it, so it carries none of the path asymmetry that is the estimator's error
 * floor -- measured on this hardware the two agree within ~450 us with a stable
 * bias, and TSF's sample-to-sample step is 1-80 us against the estimator's
 * 50-200 us.
 *
 * The estimator stays as the fallback rather than being removed. It works when
 * TSF reads 0 (not associated, no beacon yet, or a hub that does not send
 * MSG_TSF), and losing both at once would leave nothing to anchor on.
 */
bool clock_offset(int64_t *out, bool *used_tsf)
{
    if (tsf_fresh(esp_timer_get_time(), out)) {
        if (used_tsf) *used_tsf = true;
        return true;
    }
    if (used_tsf) *used_tsf = false;
    return sync_est_offset(&est, out);
}

/* --------------------------------------------------------------- playback */

/*
 * Keep the local -> master conversion current.
 *
 * The offset measured at anchoring goes stale at whatever the two crystals
 * differ by -- 10.6 ppm on our boards, so 38 ms in an hour (docs/clock-sync.md
 * §9). The phase measurement below converts local time to master time with it,
 * so holding it fixed biases that measurement by exactly the drift, and the
 * servo then faithfully parks the speaker at the growing error instead of
 * removing it. The drift the servo exists to correct was being fed back in as
 * its own reference.
 *
 * Invisible in the logs, which is why it survived: the marker pulse is derived
 * from the same conversion, so the cross-unit measurement reads correct while
 * the sound and the lights slide apart.
 *
 * Slewed, never stepped. Minimum-RTT selection moves the raw estimate by a few
 * ms as one probe replaces another in the window, and handing that straight to
 * the servo looks exactly like a real position error. The limit below is ~15x
 * the drift it has to follow, so drift is tracked with room to spare while
 * estimator noise averages out over tens of seconds.
 *
 * Nothing jumps as a result. This moves only where the servo believes it is;
 * the servo answers in sample rate, over ~100 s, as it always did.
 */
#define OFFSET_SLEW_PPM 200

void track_offset(void)
{
    int64_t measured;
    if (!clock_offset(&measured, NULL)) {
        return;
    }

    const int64_t now = esp_timer_get_time();
    if (offset_slew_last == 0) {
        offset_slew_last = now;              /* first look at this stream */
        return;
    }
    const int64_t limit = (now - offset_slew_last) * OFFSET_SLEW_PPM / 1000000;
    if (limit == 0) {
        return;                              /* too soon to move; let dt build */
    }
    offset_slew_last = now;

    int64_t diff = measured - stream_offset;
    if (diff >  limit) diff =  limit;
    if (diff < -limit) diff = -limit;
    stream_offset += diff;
}
