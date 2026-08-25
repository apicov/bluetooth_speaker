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

static void drift_task(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        telemetry_tick();
        servo_tick();
    }
}

static void log_build_stamp(const char *tag)
{
    const esp_app_desc_t *d = esp_app_get_description();
    char sha[17] = {0};
    for (int i = 0; i < 8; i++) {
        sprintf(sha + i * 2, "%02x", d->app_elf_sha256[i]);
    }
    ESP_LOGW(tag, "BUILD %s %s  %s  elf:%s",
             d->date, d->time, CONFIG_IDF_TARGET, sha);
}

#define TASK_ANY_CORE (-1)

static void task_start(TaskFunction_t fn, const char *name, uint32_t stack,
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

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(heap_caps_register_failed_alloc_callback(on_alloc_failed));

    sync_est_init(&est);
    ESP_LOGI(TAG, "SBC decoder init: %s", sbc_decoder_init() ? "ok" : "FAILED");

    const size_t ring_largest_before =
            heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    ring = xStreamBufferCreate(RING_BYTES, AUDIO_CHUNK_BYTES);
    if (!ring) {
        ESP_LOGE(TAG, "PLAYBACK RING FAILED: wanted %u bytes (%lu ms), largest "
                      "free block was %u -- lower DANCEFLOOR_RING_KB, and note "
                      "it must still cover the hub's LEAD_US + RESYNC_US",
                 (unsigned)RING_BYTES,
                 (unsigned long)(RING_BYTES * 1000ULL /
                                 (44100ULL * AUDIO_CHANNELS * 2)),
                 (unsigned)ring_largest_before);
    } else {
        ESP_LOGI(TAG, "playback ring: %u bytes (%lu ms), largest free block "
                      "%u -> %u",
                 (unsigned)RING_BYTES,
                 (unsigned long)(RING_BYTES * 1000ULL /
                                 (44100ULL * AUDIO_CHANNELS * 2)),
                 (unsigned)ring_largest_before,
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    }
    assert(ring);

    wifi_start_sta();
    socket_start();

    wifi_log_init(LOG_ROLE_SAT, MASTER_IP);
    i2s_start(44100);
    tx_rate = 44100;

#if CONFIG_DANCEFLOOR_ENABLE_MARKER
    gpio_config_t marker = {
        .pin_bit_mask = 1ULL << CONFIG_DANCEFLOOR_MARKER_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&marker));
    ESP_LOGI(TAG, "sync marker on GPIO %d -- bench instrument, nothing corrects on it",
             CONFIG_DANCEFLOOR_MARKER_GPIO);
#endif

#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER

    const size_t vis_largest_before =
            heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    visualiser_start();
    ESP_LOGI(TAG, "visualiser started (%s): largest free internal block "
                  "%u -> %u",
             visualiser_source_name(), (unsigned)vis_largest_before,
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

    visualiser_set_clock(vis_master_to_local);
#else
    ESP_LOGW(TAG, "visualiser DISABLED (menuconfig) -- LEDs will stay dark");
#endif

    task_start(probe_task, "probe", 4096, 6, TASK_ANY_CORE);
    task_start(rx_task, "rx", 4096, 7, TASK_ANY_CORE);
    task_start(play_task, "play", 4096, 8, 1);
    task_start(drift_task, "drift", 3072, 3, TASK_ANY_CORE);

    log_build_stamp(TAG);
    ESP_LOGI(TAG, "satellite up, joining \"%s\"", AP_SSID);
}
