/*
 * Which clock this unit believes, and keeping the conversion current.
 *
 * The probe task that feeds the estimator, the TSF-or-estimator choice every
 * anchor makes, and the slew that stops the offset going stale mid-stream.
 *
 * Split out of main.c on 2026-08-12; the bodies are unchanged.
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

/* --------------------------------------------------------------- receiving */

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
                .applied_med_us = splice_report_med,
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
