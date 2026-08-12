/*
 * The marker/monitor instrument: a wire between two boards that reports how far
 * apart their audio actually is.
 *
 * Bench only, off by default, and nothing corrects on what it measures. On this
 * board it is also finished -- it needs GPIO 4 and GPIO 5, and GPIO 5 became the
 * SBC link's chip select. Kept because the measurement it makes is the one the
 * whole design exists to deliver, and because a board with pins to spare could
 * still take it. TRACK DIVERGENCE over WiFi is what covers the deployed case.
 */
#include "hub.h"

/*
 * These four sit OUTSIDE the marker guard although only the marker writes
 * s_sync_*, because the splice path and probe_task read all four
 * unconditionally -- a satellite reporting its own splice over WiFi is compared
 * against them, and that path exists precisely when no marker is fitted.
 *
 * They were inside it, which meant the hub did not compile from its tracked
 * config: CONFIG_DANCEFLOOR_ENABLE_MARKER defaults to n, and the "no marker
 * fitted" branch below says in as many words that this is the normal deployed
 * case. Every build that had ever been run carried a local sdkconfig with the
 * marker switched on, so nothing showed it.
 */
#if CONFIG_DANCEFLOOR_ENABLE_MARKER
static QueueHandle_t s_edge_q;            /* satellite edge timestamps */

static void IRAM_ATTR monitor_isr(void *arg)
{
    (void)arg;
    int64_t now = esp_timer_get_time();
    BaseType_t hp = pdFALSE;
    xQueueSendFromISR(s_edge_q, &now, &hp);
    if (hp) {
        portYIELD_FROM_ISR();
    }
}

/*
 * Reports how far a satellite's audio is from this unit's, by comparing when
 * each pulsed for the same master-clock instant. This is the end-to-end number
 * the whole design exists to deliver -- everything else (clock offset, buffer
 * level, packet loss) is a means to it.
 */
static void monitor_task(void *arg)
{
    (void)arg;
    int64_t edge;
    while (xQueueReceive(s_edge_q, &edge, portMAX_DELAY) == pdTRUE) {
        int64_t mine = s_marker_at;
        if (mine == 0) {
            continue;
        }
        int64_t err = edge - mine;
        /* Markers are 2 s apart; anything near that is a missed pulse rather
         * than a sync error, and reporting it as one would mislead. */
        if (err > 500000 || err < -500000) {
            continue;
        }
        /* Kept for the track-boundary summary, which wants the last reading
         * before the splice rather than a scroll of them. */
        s_sync_err_us = err;
        s_sync_at = esp_timer_get_time();

        /* Every ~2 s is more than anyone reads. The value is kept for the
         * track-boundary summary regardless of whether this prints. */
        static int64_t last_sync_log;
        if (s_sync_at - last_sync_log >= (int64_t)CONFIG_DANCEFLOOR_LOG_PERIOD_S * 1000000) {
            last_sync_log = s_sync_at;
            ESP_LOGW(TAG, "AUDIO SYNC: satellite %+lld us (%s)", err,
                     err >= 0 ? "late" : "early");
        }
    }
}

void marker_start(void)
{
    gpio_config_t out = {
        .pin_bit_mask = 1ULL << CONFIG_DANCEFLOOR_MARKER_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&out));

    gpio_config_t in = {
        .pin_bit_mask = 1ULL << CONFIG_DANCEFLOOR_MONITOR_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&in));

    s_edge_q = xQueueCreate(4, sizeof(int64_t));
    assert(s_edge_q);
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_DANCEFLOOR_MONITOR_GPIO, monitor_isr, NULL));
    task_start(monitor_task, "syncmon", 3072, 9, TASK_ANY_CORE);

    ESP_LOGI(TAG, "sync markers on GPIO %d, watching GPIO %d -- bench instrument, "
                  "nothing corrects on it",
             CONFIG_DANCEFLOOR_MARKER_GPIO, CONFIG_DANCEFLOOR_MONITOR_GPIO);
}
#endif  /* CONFIG_DANCEFLOOR_ENABLE_MARKER */
