/*
 * M4 -- two-unit clock synchronisation, no audio.
 *
 * Master runs a SoftAP (there is no router in a field), answers time probes, and
 * multicasts "toggle a pin at time T". Satellite joins, probes once a second, and
 * schedules the same toggle on its own clock. Put a scope on GPIO_BLINK of both
 * boards: the delta between edges is the sync error, and it must be under 1 ms.
 *
 * Role is selected in menuconfig -> Dancefloor.
 */
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

#include "sync_proto.h"

/* A Kconfig bool set to 'n' is left *undefined*, not defined as 0, so it cannot
 * be used directly in a C expression -- the satellite build fails to compile. */
#ifdef CONFIG_DANCEFLOOR_ROLE_MASTER
#define ROLE_MASTER true
#else
#define ROLE_MASTER false
#endif

#define AP_SSID      "dancefloor"
#define AP_PASS      "dancefloor"
#define MASTER_IP    "192.168.4.1"   /* esp_netif SoftAP default */
#define GPIO_BLINK   ((gpio_num_t)CONFIG_DANCEFLOOR_BLINK_GPIO)
#define GPIO_MONITOR ((gpio_num_t)CONFIG_DANCEFLOOR_MONITOR_GPIO)
#define BLINK_US     10000           /* pulse width, comfortably visible */
/*
 * 250 ms, not 1 s. Minimum-RTT selection holds its best sample until a faster
 * one arrives, so the estimate is only as fresh as the window is short: at 1 s
 * intervals a held sample can be 10 s old, and at the ~14 ppm drift measured
 * between our boards that alone is ~140 us of error, growing the whole time.
 * Measured sync error slid linearly at 14 us/s with a 1 s period.
 *
 * Quartering the period keeps the same ten samples to choose from while cutting
 * the window to 2.5 s. Cost is four tiny packets a second, which is nothing.
 */
#define PROBE_PERIOD_MS  250
#define BLINK_PERIOD_MS  2000
#define BLINK_LEAD_US    500000      /* schedule this far ahead of now */
#define SPIN_GUARD_US    2000        /* busy-wait the last stretch */

static const char *TAG = "sync";

static int sock = -1;
static sync_est_t est;
static QueueHandle_t blink_q;        /* of int64_t, local-clock deadlines */

/* ---------------------------------------------------------------- blinking */

/*
 * Sleeping straight to the deadline would fold FreeRTOS tick jitter (~1 tick,
 * 10 ms by default) into the very number we are trying to measure. Wake early,
 * then spin. The spin is short and this task exists only for M4.
 */
static void blink_task(void *arg)
{
    (void)arg;
    int64_t deadline;
    while (xQueueReceive(blink_q, &deadline, portMAX_DELAY) == pdTRUE) {
        int64_t late = esp_timer_get_time() - deadline;
        if (late > 0) {
            /* Deadline already gone: a bad offset or a very delayed packet. The
             * pulse still fires, but it will read as a huge delta on the scope,
             * so say so rather than leaving it a mystery. */
            ESP_LOGW(TAG, "blink deadline missed by %lld us", late);
        }

        int64_t lead = deadline - esp_timer_get_time() - SPIN_GUARD_US;
        if (lead > 1000) {
            vTaskDelay(pdMS_TO_TICKS(lead / 1000));
        }
        while (esp_timer_get_time() < deadline) {
            /* spin */
        }
        gpio_set_level(GPIO_BLINK, 1);
        esp_rom_delay_us(BLINK_US);
        gpio_set_level(GPIO_BLINK, 0);
    }
}

static void schedule_blink(int64_t local_deadline)
{
    if (xQueueSend(blink_q, &local_deadline, 0) != pdTRUE) {
        ESP_LOGW(TAG, "blink queue full, dropped");
    }
}

/* ---------------------------------------------------------------- monitor */

/*
 * Measuring the sync error without an oscilloscope.
 *
 * The satellite's blink output is wired to an input pin on the master (plus a
 * common ground). The master already knows the master-clock instant it told
 * everyone to fire at, so the difference between that and the observed edge IS
 * the sync error, in the master's own time base.
 *
 * Signal propagation down the wire is nanoseconds and GPIO interrupt latency is
 * a few microseconds -- both negligible against a 1 ms budget.
 */
static QueueHandle_t edge_q;             /* of int64_t, master-clock edge times */
static volatile int64_t expected_blink;  /* master-clock instant last announced */

static void IRAM_ATTR monitor_isr(void *arg)
{
    (void)arg;
    int64_t now = esp_timer_get_time();
    BaseType_t hp_woken = pdFALSE;
    xQueueSendFromISR(edge_q, &now, &hp_woken);
    if (hp_woken) {
        portYIELD_FROM_ISR();
    }
}

static void monitor_task(void *arg)
{
    (void)arg;
    int64_t edge;
    while (xQueueReceive(edge_q, &edge, portMAX_DELAY) == pdTRUE) {
        int64_t target = expected_blink;
        if (target == 0) {
            continue;                    /* nothing announced yet */
        }
        int64_t err = edge - target;
        /* A pulse is 10 ms wide; anything beyond that is a missed announcement
         * rather than a sync error, and reporting it as one would mislead. */
        if (err > 100000 || err < -100000) {
            ESP_LOGW(TAG, "edge %lld us from target -- not a sync measurement", err);
            continue;
        }
        ESP_LOGI(TAG, "SYNC ERROR: %+lld us   (satellite %s)",
                 err, err >= 0 ? "late" : "early");
    }
}

static void monitor_start(void)
{
    edge_q = xQueueCreate(4, sizeof(int64_t));
    assert(edge_q);

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << GPIO_MONITOR,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,   /* defined level before the satellite boots */
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_MONITOR, monitor_isr, NULL));

    xTaskCreate(monitor_task, "mon", 3072, NULL, 9, NULL);
    ESP_LOGI(TAG, "monitoring satellite pulse on GPIO %d", CONFIG_DANCEFLOOR_MONITOR_GPIO);
}

/* ------------------------------------------------------------------- wifi */

static void wifi_start(bool master)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    if (master) {
        esp_netif_create_default_wifi_ap();
    } else {
        esp_netif_create_default_wifi_sta();
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wc = {0};
    if (master) {
        strcpy((char *)wc.ap.ssid, AP_SSID);
        strcpy((char *)wc.ap.password, AP_PASS);
        wc.ap.ssid_len = strlen(AP_SSID);
        wc.ap.max_connection = 8;
        wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    } else {
        strcpy((char *)wc.sta.ssid, AP_SSID);
        strcpy((char *)wc.sta.password, AP_PASS);
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    }
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Power save parks the radio between beacons and adds tens of ms of latency
     * to exactly the packets we are timing. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    if (!master) {
        ESP_ERROR_CHECK(esp_wifi_connect());
    }
}

static void socket_start(bool master)
{
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    assert(sock >= 0);

    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SYNC_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    assert(bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) == 0);

    if (!master) {
        struct ip_mreq mreq = {
            .imr_multiaddr.s_addr = inet_addr(SYNC_MCAST_ADDR),
            .imr_interface.s_addr = htonl(INADDR_ANY),
        };
        assert(setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) == 0);
    }
}

/* ------------------------------------------------------------------ master */

static void master_rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[64];
    struct sockaddr_in from;

    while (1) {
        /* recvfrom writes the actual address length back, so this must be reset
         * every iteration rather than hoisted out of the loop. */
        socklen_t from_len = sizeof(from);
        int n = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&from, &from_len);
        int64_t t2 = esp_timer_get_time();          /* stamp on arrival */
        if (n < (int)sizeof(time_msg_t) || buf[0] != MSG_TIME_REQ) {
            continue;
        }
        time_msg_t msg;
        memcpy(&msg, buf, sizeof(msg));
        msg.type = MSG_TIME_RSP;
        msg.t2 = t2;
        msg.t3 = esp_timer_get_time();              /* stamp immediately before send */
        sendto(sock, &msg, sizeof(msg), 0, (struct sockaddr *)&from, from_len);
    }
}

static void master_announce_task(void *arg)
{
    (void)arg;
    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port = htons(SYNC_PORT),
        .sin_addr.s_addr = inet_addr(SYNC_MCAST_ADDR),
    };

    while (1) {
        blink_msg_t msg = { .type = MSG_BLINK, .play_at = esp_timer_get_time() + BLINK_LEAD_US };
        sendto(sock, &msg, sizeof(msg), 0, (struct sockaddr *)&dest, sizeof(dest));
        schedule_blink(msg.play_at);   /* master clock == local clock */
        expected_blink = msg.play_at;  /* what the monitor compares the edge against */
        vTaskDelay(pdMS_TO_TICKS(BLINK_PERIOD_MS));
    }
}

/* --------------------------------------------------------------- satellite */

static void satellite_rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[64];

    while (1) {
        int n = recvfrom(sock, buf, sizeof(buf), 0, NULL, NULL);
        int64_t t4 = esp_timer_get_time();
        if (n < 1) {
            continue;
        }

        if (buf[0] == MSG_TIME_RSP && n >= (int)sizeof(time_msg_t)) {
            time_msg_t msg;
            memcpy(&msg, buf, sizeof(msg));
            sync_est_add(&est, msg.t1, msg.t2, msg.t3, t4);

            int64_t offset;
            if (sync_est_offset(&est, &offset)) {
                ESP_LOGI(TAG, "offset %lld us (rtt %lld us)",
                         offset, (t4 - msg.t1) - (msg.t3 - msg.t2));
            }
        } else if (buf[0] == MSG_BLINK && n >= (int)sizeof(blink_msg_t)) {
            blink_msg_t msg;
            memcpy(&msg, buf, sizeof(msg));

            int64_t offset;
            if (!sync_est_offset(&est, &offset)) {
                ESP_LOGW(TAG, "blink ignored, clock not yet synced");
                continue;
            }
            schedule_blink(sync_to_local(msg.play_at, offset));
        }
    }
}

static void satellite_probe_task(void *arg)
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
        vTaskDelay(pdMS_TO_TICKS(PROBE_PERIOD_MS));
    }
}

/* -------------------------------------------------------------------- main */

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << GPIO_BLINK,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    sync_est_init(&est);
    blink_q = xQueueCreate(4, sizeof(int64_t));
    assert(blink_q);

    const bool master = ROLE_MASTER;
    ESP_LOGI(TAG, "role: %s", master ? "master" : "satellite");

    wifi_start(master);
    socket_start(master);

    /* Priority 10: above the default task, below the WiFi/timer internals. */
    xTaskCreate(blink_task, "blink", 3072, NULL, 10, NULL);

    if (master) {
        monitor_start();
        xTaskCreate(master_rx_task, "m_rx", 4096, NULL, 5, NULL);
        xTaskCreate(master_announce_task, "m_ann", 4096, NULL, 5, NULL);
    } else {
        xTaskCreate(satellite_rx_task, "s_rx", 4096, NULL, 5, NULL);
        xTaskCreate(satellite_probe_task, "s_probe", 4096, NULL, 5, NULL);
    }
}
