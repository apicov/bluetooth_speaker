
#include "hub.h"

#include "nvs_flash.h"

void task_start(TaskFunction_t fn, const char *name, uint32_t stack,
                UBaseType_t prio, int core)
{
    TaskHandle_t h = NULL;
    const BaseType_t ok = (core == TASK_ANY_CORE)
        ? xTaskCreate(fn, name, stack, NULL, prio, &h)
        : xTaskCreatePinnedToCore(fn, name, stack, NULL, prio, &h, core);

    if (ok == pdPASS && h != NULL) {
        return;
    }
    n_task_fail++;
    if (strlen(s_task_fail_names) + strlen(name) + 2 < sizeof s_task_fail_names) {
        if (s_task_fail_names[0]) {
            strcat(s_task_fail_names, " ");
        }
        strcat(s_task_fail_names, name);
    }
    ESP_LOGE(TAG, "TASK \"%s\" FAILED TO START (%" PRIu32 " B stack) -- internal "
                  "heap %u free, largest block %u. This unit is crippled.",
             name, stack,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
}

void streamer_start(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    telemetry_register_alloc_hook();

    local_ring = xStreamBufferCreate(LOCAL_RING_BYTES, AUDIO_CHUNK_BYTES);
    assert(local_ring);

    streamer_fec_start();

    wifi_start_ap();
    socket_start();

    wifi_log_init(LOG_ROLE_HUB, NULL);
    i2s_start(sample_rate);
#if CONFIG_DANCEFLOOR_ENABLE_MARKER
    marker_start();
#endif

    ESP_LOGI(TAG, "free heap after WiFi init: %" PRIu32 " bytes", esp_get_free_heap_size());

    vol_repeat_start();

    task_start(probe_task, "probe", 4096, 6, TASK_ANY_CORE);
    task_start(local_play_task, "play", 4096, 8, 1);
    task_start(ring_monitor_task, "ringmon", 3072, 3, TASK_ANY_CORE);
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER

#if CONFIG_DANCEFLOOR_PUBLISH_FRAMES
    visualiser_set_publish(publish_frame);
#endif
#endif
    ESP_LOGI(TAG, "streaming on port %d, unicast to registered listeners", SYNC_PORT);
}
