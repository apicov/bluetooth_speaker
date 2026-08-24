/*
 * The hub's entry point, and the one helper every module's startup goes through.
 *
 * This file was 2686 lines and nine concerns until 2026-08-12. What is left is
 * the order things are brought up in, which is the thing a reviewer most often
 * wants and could least easily find. Everything else moved out; hub.h says where
 * and who owns what. See docs/hub-audit.md.
 */
#include "hub.h"

#include "nvs_flash.h"

/*
 * xTaskCreate with the return value actually read.
 *
 * Not a panic, for the same reason CONFIG_HEAP_ABORT_WHEN_ALLOCATION_FAILS is
 * off above: a reboot loop on a dance floor is worse than a unit that comes up
 * crippled and says which part is missing.
 *
 * This unit needs no equivalent of the satellite's join-line repeat. Its console
 * is a dedicated UART wire at 921600 rather than a shared USB bridge -- see the
 * long note in sdkconfig.defaults -- so a boot-time ESP_LOGE here actually gets
 * out. The satellite's does not, which is why its copy defers the news to
 * GOT_IP.
 */
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

    /* Before anything else allocates, so a failure during WiFi or socket setup
     * is caught too -- that is the phase with the largest single requests. The
     * hook itself stays private to telemetry.c, which is the only thing that
     * reads what it records. */
    telemetry_register_alloc_hook();

    local_ring = xStreamBufferCreate(LOCAL_RING_BYTES, AUDIO_CHUNK_BYTES);
    assert(local_ring);

    /* Before wifi_start_ap(), so the PSRAM request is made while the heap is
     * least fragmented -- and so the "parity disabled" warning, if it comes,
     * prints ahead of the WiFi banner rather than buried in it. */
    streamer_fec_start();

    wifi_start_ap();
    socket_start();
    /* Mirror this hub's own logs and relay every satellite's to a collector.
     * Destination starts unset -- the collector announces itself with
     * MSG_LOG_SUB (handled in probe_task) and wifi_log_note_collector() points
     * this at it. No-op unless CONFIG_DANCEFLOOR_WIFI_LOGS is set. */
    wifi_log_init(LOG_ROLE_HUB, NULL);
    i2s_start(sample_rate);
#if CONFIG_DANCEFLOOR_ENABLE_MARKER
    marker_start();
#endif

    ESP_LOGI(TAG, "free heap after WiFi init: %" PRIu32 " bytes", esp_get_free_heap_size());

    /* After socket_start(), which is what it sends on, and before the tasks, so
     * a satellite that joins during startup is told a level by the first tick. */
    vol_repeat_start();

    task_start(probe_task, "probe", 4096, 6, TASK_ANY_CORE);
    task_start(local_play_task, "play", 4096, 8, 1);
    task_start(ring_monitor_task, "ringmon", 3072, 3, TASK_ANY_CORE);
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
    /*
     * Safe before visualiser_start(): this only stores a pointer the analysis
     * task reads, and that task does not exist yet.
     *
     * Registered only when DANCEFLOOR_PUBLISH_FRAMES is on. The hub's own strip
     * renders from the analysis either way; this gates only whether those frames
     * go out to satellites. A floor where every satellite is LED_SOURCE_LOCAL
     * analyses its own audio and gains nothing from the ~5 kB/s per listener this
     * costs -- turn it off there to recover the airtime at scale. The per-satellite
     * case (some of each) wants a subscribe message, not a build switch.
     */
#if CONFIG_DANCEFLOOR_PUBLISH_FRAMES
    visualiser_set_publish(publish_frame);
#endif
#endif
    ESP_LOGI(TAG, "streaming on port %d, unicast to registered listeners", SYNC_PORT);
}
