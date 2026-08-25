/**
 * @file clock.c
 * @brief The probe task, choosing between TSF and the estimator, the offset
 *        slew, and the self-mute state machine.
 *
 * The probe task is this unit's heartbeat. Its clock probes are also its
 * registration: the hub keeps a client for as long as its probes keep
 * arriving, so stopping them is how a satellite leaves the send list.
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

/**
 * @brief One tick of the self-mute state machine.
 *
 * @return true if this unit should stay off the air this tick.
 *
 * Keeps a sliding @ref MUTE_WINDOW_US history of audio arrivals and, when it
 * has heard almost nothing for that long AND the AP's beacon is weak, takes
 * the unit off the hub's send list. Both halves are needed: between tracks
 * nothing arrives at ANY satellite, so "no audio" alone would mute an idle
 * floor. A unit that hears the AP well and receives nothing is idle; one that
 * barely hears it is deaf.
 *
 * Once muted it watches for the signal to come back rather than retrying on a
 * timer, because each retry re-registers the unit and resumes the airtime cost
 * for the length of the trial. @ref MUTE_RETRY_US remains as the fallback for
 * an RSSI that cannot be read, or one that lies.
 *
 * @see MUTE_WINDOW_US for why a satellite mutes itself at all.
 */
static bool mute_tick(void)
{
    static uint32_t prev_rx;        /* audio datagrams at the last tick */
    static uint16_t hist[MUTE_SLOTS]; /* arrivals per tick, sliding */
    static uint32_t hist_sum;
    static int      slot;
    static int64_t  s_muted_at;     /* 0 = on the air */
    static int      s_good_ticks;
    static int64_t  s_trial_until;  /* grace after coming back */

    const int64_t now = esp_timer_get_time();

    const uint32_t rx = n_audio_rx;
    const uint32_t delta = rx - prev_rx;
    prev_rx = rx;
    hist_sum -= hist[slot];
    hist[slot] = (uint16_t)(delta > UINT16_MAX ? UINT16_MAX : delta);
    hist_sum += hist[slot];
    slot = (slot + 1) % MUTE_SLOTS;

    if (s_muted_at) {
        /* A muted unit is still ASSOCIATED, so it can read the AP's beacon --
         * esp_wifi_sta_get_ap_info() needs association, not a place on the
         * send list. Watching costs the floor nothing while the signal is
         * down. */
        wifi_ap_record_t back;
        if (esp_wifi_sta_get_ap_info(&back) == ESP_OK &&
            back.rssi >= MUTE_RSSI_REJOIN) {
            s_good_ticks++;
        } else {
            s_good_ticks = 0;
        }
        if (s_good_ticks < MUTE_REJOIN_TICKS &&
            now - s_muted_at < MUTE_RETRY_US) {
            return true;
        }
        s_good_ticks = 0;

        /* Clear the history before the trial: the unit has been receiving
         * nothing BY DESIGN while muted, so carrying that forward would
         * re-mute it on the trial's first tick. */
        memset(hist, 0, sizeof(hist));
        hist_sum = 0;
        s_muted_at = 0;
        s_trial_until = now + MUTE_TRIAL_US;
        self_muted = false;
        n_self_retries++;

        /* Throw the clock away with the mute. Probes carry clock sync as well
         * as registration, so the estimator's window is now as old as the
         * outage -- and an offset error at the anchor is baked in for the life
         * of the stream. sync_est_settled() then refuses to anchor until a
         * fresh window exists. Publishing an `at` of zero says "never seen",
         * which is how tsf_fresh() reports TSF as unavailable. */
        sync_est_init(&est);
        est_newest_at = 0;
        tsf_publish(0, 0);
        ESP_LOGW(TAG, "signal back -- rejoining the floor");
        return false;
    }

    if (now < s_trial_until || hist_sum >= MUTE_AUDIO_MIN) {
        return false;
    }

    /* Read here rather than every tick: this point is only reached after a
     * whole MUTE_WINDOW_US of silence. */
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
        return false;       /* not associated: already off the send list */
    }
    if (ap.rssi > MUTE_RSSI_FLOOR) {
        return false;
    }

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

/* probe_task() is documented at its declaration in sat.h. */
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

        /* net.c owns the retry policy; this only supplies the heartbeat. */
        wifi_retry_tick();

        /* Gate BOTH sends, not just the probe. MSG_SPLICE reaches the hub's
         * client table exactly as MSG_TIME_REQ does, so a track boundary
         * during a mute would re-register this unit once per track. */
        if (mute_tick()) {
            vTaskDelay(pdMS_TO_TICKS(PROBE_PERIOD_MS));
            continue;
        }

        time_msg_t msg = { .type = MSG_TIME_REQ, .seq = seq++, .t1 = esp_timer_get_time() };
        sendto(sock, &msg, sizeof(msg), 0, (struct sockaddr *)&dest, sizeof(dest));

        if (splice_report_pending) {
            /* Cleared before the send, so a report landing while this one is
             * in flight is kept rather than swallowed by the clear. */
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

/* clock_offset() is documented at its declaration in sat.h. */
bool clock_offset(int64_t *out, bool *used_tsf)
{
    if (tsf_fresh(esp_timer_get_time(), out)) {
        if (used_tsf) *used_tsf = true;
        return true;
    }
    if (used_tsf) *used_tsf = false;
    return sync_est_offset(&est, out);
}

/** @brief How fast @ref track_offset() may move @ref stream_offset, in ppm.
 *  Comfortably above the crystal difference it has to follow, and slow enough
 *  that nothing audible jumps. */
#define OFFSET_SLEW_PPM 200

/* track_offset() is documented at its declaration in sat.h.
 *
 * Slewed and never stepped: min-RTT selection moves the raw estimate by
 * milliseconds as probes rotate through its window, and stepping the timeline
 * by that would be audible. */
void track_offset(void)
{
    int64_t measured;
    if (!clock_offset(&measured, NULL)) {
        return;
    }

    const int64_t now = esp_timer_get_time();
    if (offset_slew_last == 0) {
        offset_slew_last = now;     /* first look at this stream */
        return;
    }
    const int64_t limit = (now - offset_slew_last) * OFFSET_SLEW_PPM / 1000000;
    if (limit == 0) {
        return;
    }
    offset_slew_last = now;

    int64_t diff = measured - stream_offset;
    if (diff >  limit) diff =  limit;
    if (diff < -limit) diff = -limit;
    stream_offset += diff;
}
