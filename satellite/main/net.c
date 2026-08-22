/*
 * Joining the hub's SoftAP, and the one UDP socket everything rides on.
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

/* ------------------------------------------------------------------- wifi */

/*
 * Without this a failed association is completely silent: esp_wifi_connect() is
 * called once, and if it does not succeed nothing logs it and nothing retries.
 * A satellite that quietly never joins is far worse than one that says so.
 */
static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *d = data;
        n_wifi_drops++;
        /* Only the first drop of a streak dates the outage: repeated
         * association failures raise this repeatedly, and taking the latest
         * would measure the last retry rather than how long the unit was off
         * the air, which is the number that matters to playback. */
        if (wifi_down_at == 0) {
            wifi_down_at = esp_timer_get_time();
        }
        /*
         * Off the floor, so the marker LED stops claiming otherwise.
         *
         * Said on every drop of a streak rather than only the first, unlike
         * wifi_down_at above: that one is dating an outage and must not be
         * restarted by a retry, while this is asserting a level and repeating
         * it costs nothing.
         *
         * Nothing here waits for playback to notice. Audio outlives a drop by
         * roughly the ring's depth, so the LED goes dark a few hundred ms
         * before the flashes stop -- and it is the flashes that stop late, not
         * this that goes early. During those few hundred ms the render task is
         * still drawing, so the marker is still flashing and this level is not
         * shown until it falls idle. That is deliberate: audio still flowing IS
         * the link working, whatever the association says.
         */
        visualiser_marker_set_link(false);
        ESP_LOGW(TAG, "disconnected from \"%s\" (reason %d), retrying",
                 AP_SSID, d->reason);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *e = data;
        if (wifi_down_at) {
            /*
             * How long this unit was actually off the air. Playback survives
             * roughly the ring's worth of it and then underruns, so this is the
             * number that says whether a drop cost a glitch or a re-anchor --
             * and it is the baseline any change to the reconnect path has to
             * beat. rejoined_at arms the report on the next anchor.
             */
            ESP_LOGW(TAG, "rejoined \"%s\" after %lld ms",
                     AP_SSID, (esp_timer_get_time() - wifi_down_at) / 1000);
            wifi_down_at = 0;
            rejoined_at = esp_timer_get_time();
        }
        /*
         * On the floor. The marker LED goes solid until audio starts, which is
         * the first thing this unit can say from across a dark field without a
         * console -- and the thing it could not say before, when a satellite
         * that never found the hub and a satellite waiting for music were both
         * simply dark.
         *
         * Here rather than at association, because a lease is the point at
         * which the unit can actually be sent audio. It also repeats on every
         * rejoin, which is what re-lights the LED after a drop.
         *
         * Safe this early: the setter only stores a flag, and the render task
         * picks it up whenever it starts -- visualiser_start() runs after
         * wifi_start_sta() and a fast lease can beat it here.
         */
        visualiser_marker_set_link(true);
        ESP_LOGI(TAG, "joined \"%s\", IP " IPSTR, AP_SSID, IP2STR(&e->ip_info.ip));
        /*
         * Said here, not where it happened. task_start() logs the failure at
         * boot, roughly a second before this unit has a route, so wifi_log drops
         * that line for want of a destination and the collector never sees it.
         * This is the first moment the radio can carry the news.
         *
         * Silent on a healthy unit, like every other counter here. If it ever
         * prints, nothing else in this log means what it usually means.
         *
         * Ordering is assumed, not enforced: wifi_start_sta() runs about a
         * second before the tasks are created, so a DHCP lease arriving that
         * fast would find n_task_fail still zero and say nothing. The console
         * line in task_start() is unconditional, and this one repeats on every
         * rejoin, so the news is only lost if both the console is dead AND the
         * unit never reconnects. Worth knowing before trusting a silent join.
         */
        if (n_task_fail) {
            ESP_LOGE(TAG, "CRIPPLED: %" PRIu32 " task(s) failed to start: %s",
                     n_task_fail, s_task_fail_names);
        }
    }
}

void wifi_start_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wc = {0};
    strcpy((char *)wc.sta.ssid, AP_SSID);
#if CONFIG_DANCEFLOOR_AP_OPEN
    wc.sta.password[0] = '\0';
    wc.sta.threshold.authmode = WIFI_AUTH_OPEN;
#else
    strcpy((char *)wc.sta.password, AP_PASS);
#endif
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                       wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                       wifi_event, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));

#if CONFIG_DANCEFLOOR_DISABLE_PMF
    /*
     * Both ends, so neither advertises the capability -- see the hub's copy for
     * what PMF was doing to this link. This unit is the one that was failing to
     * answer the SA Query and being thrown off for it, and it is also the one
     * that never noticed: the hub counted two disassociations while this
     * counted zero.
     *
     * Must sit between esp_wifi_set_config() and esp_wifi_start().
     */
    const esp_err_t pmf = esp_wifi_disable_pmf_config(WIFI_IF_STA);
    ESP_LOGW(TAG, "PMF disabled on the station: %s", esp_err_to_name(pmf));
#endif

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    /* STA_START triggers the first connect; disconnects retry from the handler. */
}

void socket_start(void)
{
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    assert(sock >= 0);

    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SYNC_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    assert(bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) == 0);
    /* Nothing else to do: audio arrives by unicast, and the time probes this
     * unit sends are what put it on the hub's send list. */
}
