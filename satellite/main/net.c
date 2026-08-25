/**
 * @file net.c
 * @brief Joining the hub's SoftAP, the reconnect policy, and the UDP socket.
 *
 * @section states The three states this unit can be in
 *
 * - **Not associated.** Retry the connect when the backoff is up.
 * - **Associated with no lease.** The radio is happy and the link is useless:
 *   probes go nowhere and the hub never hears of this unit. Watched by a
 *   deadline, because nothing else will end it.
 * - **Leased.** The link level that matters — a lease is the point at which
 *   this unit can actually be sent audio.
 *
 * The middle state exists because a successful association raises
 * `STA_CONNECTED` and nothing else, and a DHCP exchange that silently fails
 * raises no `STA_DISCONNECTED` to react to.
 *
 * The event handler only ARMS the deadlines; the attempts themselves happen in
 * @ref wifi_retry_tick(), which the probe task calls. Blocking inside the
 * handler would stop the default event loop for the duration.
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

/** @brief Wait this long after a drop before asking for the association
 *  again. */
#define WIFI_RETRY_BACKOFF_US 1000000

/** @brief How long an association may go without a DHCP lease before it is
 *  torn down. Sized against lwIP's DHCP retry backoff, so roughly three
 *  attempts get to happen first: shorter would abandon a slow but working
 *  exchange, longer leaves the unit stranded and inaudible for no gain. */
#define WIFI_LEASE_TIMEOUT_US 10000000

/**
 * @name Reconnect state
 * Written by the event handler, and the two deadlines also by
 * @ref wifi_retry_tick() on the probe task. `volatile`, not atomic: the bools
 * cannot tear, and the int64s feed nothing but a compare against the current
 * time or one log line, so the worst a stale or torn read costs is one 250 ms
 * tick of latency or one wrong number printed.
 * @{
 */
static volatile bool    s_assoc;    /**< Associated to the AP. */
static volatile int64_t s_assoc_at; /**< When that association was made; handler only. */
static volatile bool    s_link_up;  /**< Associated AND holding a lease. */
static volatile int64_t s_retry_at; /**< When to re-attempt the connect. */
static volatile int64_t s_lease_at; /**< When to give up waiting for DHCP. */
/** @} */

/* wifi_retry_tick() is documented at its declaration in sat.h. */
void wifi_retry_tick(void)
{
    if (s_link_up) {
        return;
    }
    const int64_t now = esp_timer_get_time();

    if (s_assoc) {
        const int64_t due = s_lease_at;
        if (due == 0 || now < due) {
            return;
        }
        s_lease_at = now + WIFI_LEASE_TIMEOUT_US;

        /* Re-read after pushing the deadline forward. The deadline being acted
         * on is ten seconds old, and the lease may have arrived while this
         * tick was deciding -- dropping a link that just came up would be
         * worse than waiting one more window. */
        if (s_link_up) {
            return;
        }
        n_wifi_lease_fail++;
        /* Measured from the association, not assumed from the timeout. The
         * deadline is pushed forward rather than cleared, so a link that keeps
         * failing arrives here repeatedly and the elapsed time is a multiple of
         * WIFI_LEASE_TIMEOUT_US -- printing that constant would report the
         * first ten seconds every time and hide how long this has gone on. */
        ESP_LOGE(TAG, "associated to \"%s\" for %lld ms with no DHCP lease -- "
                      "dropping the association to start over",
                 AP_SSID, s_assoc_at ? (now - s_assoc_at) / 1000 : -1);
        /* disconnect() rather than a bare retry: it raises the
         * STA_DISCONNECTED the handler already deals with, and re-association
         * runs DHCP from the start. */
        esp_wifi_disconnect();
        return;
    }

    const int64_t due = s_retry_at;
    if (due == 0 || now < due) {
        return;
    }

    /* Arm BEFORE attempting, not clear after. If the connect fails and no
     * event ever arrives, a cleared deadline would leave nothing to retry
     * from and the unit would sit off the air forever. */
    s_retry_at = now + WIFI_RETRY_BACKOFF_US;
    esp_wifi_connect();
}

/**
 * @brief WiFi and IP event handler: maintains the reconnect state.
 *
 * @param arg  Unused.
 * @param base Event base, WIFI_EVENT or IP_EVENT.
 * @param id   Event id within @p base.
 * @param data Event payload.
 *
 * Arms deadlines and records what happened; the retries themselves belong to
 * @ref wifi_retry_tick(). This runs on the default event loop's task, so
 * anything that blocks here stops esp_event for every other subscriber.
 */
static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        /* Armed as well as attempted: if this first connect returns an error
         * no event follows it, and the deadline is what recovers. */
        s_retry_at = esp_timer_get_time() + WIFI_RETRY_BACKOFF_US;
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        /* Not logged on its own -- an association without a lease is not a
         * join. What it starts is the DHCP watchdog. */
        s_assoc = true;
        s_assoc_at = esp_timer_get_time();
        s_lease_at = esp_timer_get_time() + WIFI_LEASE_TIMEOUT_US;
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *d = data;
        n_wifi_drops++;

        /* Both flags down before the deadline is armed, so a tick that sees
         * the deadline cannot also see a stale state. */
        s_assoc = false;
        s_assoc_at = 0;
        s_link_up = false;
        s_lease_at = 0;
        s_retry_at = esp_timer_get_time() + WIFI_RETRY_BACKOFF_US;

        /* Only the first drop of a streak dates the outage: measuring from the
         * last retry is not measuring time off the air. */
        if (wifi_down_at == 0) {
            wifi_down_at = esp_timer_get_time();
        }

        /* Asserted on every drop, because this is a level and not an event.
         * Audio outlives the drop by a ring's depth, so the LED goes dark
         * before the flashes stop -- which is the intended reading. */
        visualiser_marker_set_link(false);
        ESP_LOGW(TAG, "disconnected from \"%s\" (reason %d), retrying in %d ms",
                 AP_SSID, d->reason, WIFI_RETRY_BACKOFF_US / 1000);

    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *e = data;

        /* Stops the retry, and disarms the DHCP watchdog. */
        s_link_up = true;
        s_lease_at = 0;
        if (wifi_down_at) {
            /* How long playback was without a hub. rejoined_at arms the next
             * anchor to report which clock it used and how stale it was. */
            ESP_LOGW(TAG, "rejoined \"%s\" after %lld ms",
                     AP_SSID, (esp_timer_get_time() - wifi_down_at) / 1000);
            wifi_down_at = 0;
            rejoined_at = esp_timer_get_time();
        }

        /* The link LED is lit at the lease, not at the association. Safe this
         * early even if visualiser_start() has not run: the setter only stores
         * a flag. */
        visualiser_marker_set_link(true);
        ESP_LOGI(TAG, "joined \"%s\", IP " IPSTR, AP_SSID, IP2STR(&e->ip_info.ip));

        /* Said here rather than only where it happens: task_start() runs about
         * a second before this unit has a route, so its own log line is
         * dropped by wifi_log and never reaches the collector. */
        if (n_task_fail) {
            ESP_LOGE(TAG, "CRIPPLED: %" PRIu32 " task(s) failed to start: %s",
                     n_task_fail, s_task_fail_names);
        }
    }
}

/* wifi_start_sta() is documented at its declaration in sat.h. */
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
    /* MUST sit between esp_wifi_set_config() and esp_wifi_start(). */
    const esp_err_t pmf = esp_wifi_disable_pmf_config(WIFI_IF_STA);
    ESP_LOGW(TAG, "PMF disabled on the station: %s", esp_err_to_name(pmf));
#endif

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
}

/* socket_start() is documented at its declaration in sat.h. */
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
}
