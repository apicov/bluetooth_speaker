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

#define WIFI_RETRY_BACKOFF_US 1000000

#define WIFI_LEASE_TIMEOUT_US 10000000

static volatile bool    s_assoc;
static volatile bool    s_link_up;
static volatile int64_t s_retry_at;
static volatile int64_t s_lease_at;

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

        if (s_link_up) {
            return;
        }
        n_wifi_lease_fail++;
        ESP_LOGE(TAG, "associated to \"%s\" for %d ms with no DHCP lease -- "
                      "dropping the association to start over",
                 AP_SSID, WIFI_LEASE_TIMEOUT_US / 1000);
        esp_wifi_disconnect();
        return;
    }

    const int64_t due = s_retry_at;
    if (due == 0 || now < due) {
        return;
    }

    s_retry_at = now + WIFI_RETRY_BACKOFF_US;
    esp_wifi_connect();
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {

        s_retry_at = esp_timer_get_time() + WIFI_RETRY_BACKOFF_US;
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {

        s_assoc = true;
        s_lease_at = esp_timer_get_time() + WIFI_LEASE_TIMEOUT_US;
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *d = data;
        n_wifi_drops++;

        s_assoc = false;
        s_link_up = false;
        s_lease_at = 0;
        s_retry_at = esp_timer_get_time() + WIFI_RETRY_BACKOFF_US;

        if (wifi_down_at == 0) {
            wifi_down_at = esp_timer_get_time();
        }

        visualiser_marker_set_link(false);
        ESP_LOGW(TAG, "disconnected from \"%s\" (reason %d), retrying in %d ms",
                 AP_SSID, d->reason, WIFI_RETRY_BACKOFF_US / 1000);

    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *e = data;

        s_link_up = true;
        s_lease_at = 0;
        if (wifi_down_at) {

            ESP_LOGW(TAG, "rejoined \"%s\" after %lld ms",
                     AP_SSID, (esp_timer_get_time() - wifi_down_at) / 1000);
            wifi_down_at = 0;
            rejoined_at = esp_timer_get_time();
        }

        visualiser_marker_set_link(true);
        ESP_LOGI(TAG, "joined \"%s\", IP " IPSTR, AP_SSID, IP2STR(&e->ip_info.ip));

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

    const esp_err_t pmf = esp_wifi_disable_pmf_config(WIFI_IF_STA);
    ESP_LOGW(TAG, "PMF disabled on the station: %s", esp_err_to_name(pmf));
#endif

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
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

}
