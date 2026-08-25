/**
 * @file marker.c
 * @brief The wired sync instrument: pulse a GPIO as playback crosses a
 *        marker instant, and time a satellite's answering pulse against it.
 *
 * A bench tool, off by default (CONFIG_DANCEFLOOR_ENABLE_MARKER): it needs a
 * wire from a satellite's marker pin to this board's monitor pin, which a
 * deployed floor cannot have, and nothing in the control loop corrects on
 * the result -- the TRACK DIVERGENCE line in play.c is its entire output.
 * The deployed case needs no wire: every satellite reports its divergence
 * over WiFi at each track boundary, covering the whole floor rather than
 * the one board that happens to be wired up.
 *
 * The two pins default into a corner: DANCEFLOOR_MONITOR_GPIO defaults to
 * the same pad as DANCEFLOOR_SBC_SPI_CS_PIN, so a build with both the
 * instrument and the bridge link must move one of them.
 */
#include "hub.h"

#if CONFIG_DANCEFLOOR_ENABLE_MARKER
/** @brief Edge timestamps from the ISR to the monitor task. */
static QueueHandle_t s_edge_q;

/**
 * @brief Timestamp the monitor edge from ISR context and queue it.
 * @param arg  Unused (the GPIO ISR contract).
 */
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

/**
 * @brief Compare each monitor edge against this unit's own marker pulse and
 *        record the gap in s_sync_err_us, for the TRACK DIVERGENCE line.
 * @param arg  Unused (the task body contract).
 */
static void monitor_task(void *arg)
{
    (void)arg;
    int64_t edge;
    while (xQueueReceive(s_edge_q, &edge, portMAX_DELAY) == pdTRUE) {
        int64_t mine = s_marker_at;
        if (mine == 0) {
            continue;   /* no pulse of ours out yet to measure against */
        }
        int64_t err = edge - mine;

        /* Both units tag a marker every MARKER_EVERY_PKTS input packets --
         * roughly 2 s apart at the A2DP rate -- so a genuine pair of pulses
         * lands within tens of ms even badly out of sync. Past half a
         * second, one side has missed its tag and this edge belongs to the
         * next marker; keeping it would log a 2 s "error". */
        if (err > 500000 || err < -500000) {
            continue;
        }

        s_sync_err_us = err;
        s_sync_at = esp_timer_get_time();

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

    s_edge_q = xQueueCreate(4, sizeof(int64_t));   /* edges are ~2 s apart; deeper would only stack latency */
    assert(s_edge_q);
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_DANCEFLOOR_MONITOR_GPIO, monitor_isr, NULL));
    task_start(monitor_task, "syncmon", 3072, 9, TASK_ANY_CORE);

    ESP_LOGI(TAG, "sync markers on GPIO %d, watching GPIO %d -- bench instrument, "
                  "nothing corrects on it",
             CONFIG_DANCEFLOOR_MARKER_GPIO, CONFIG_DANCEFLOOR_MONITOR_GPIO);
}
#endif
