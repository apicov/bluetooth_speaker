
#include "hub.h"

#if CONFIG_DANCEFLOOR_ENABLE_MARKER
static QueueHandle_t s_edge_q;

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

    s_edge_q = xQueueCreate(4, sizeof(int64_t));
    assert(s_edge_q);
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_DANCEFLOOR_MONITOR_GPIO, monitor_isr, NULL));
    task_start(monitor_task, "syncmon", 3072, 9, TASK_ANY_CORE);

    ESP_LOGI(TAG, "sync markers on GPIO %d, watching GPIO %d -- bench instrument, "
                  "nothing corrects on it",
             CONFIG_DANCEFLOOR_MARKER_GPIO, CONFIG_DANCEFLOOR_MONITOR_GPIO);
}
#endif
