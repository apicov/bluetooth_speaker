/**
 * @file main.c
 * @brief Startup and task wiring: bring the peripherals up, then start the
 *        four tasks.
 *
 * The satellite joins the master's SoftAP, keeps its clock aligned with the
 * master's, receives SBC packets addressed to it, decodes them, and plays each
 * at the instant it was stamped for. No Bluetooth here — the master owns the
 * phone connection; this board only listens on WiFi and drives a DAC.
 *
 * The behaviour lives in the modules beside this one: net.c, clock.c, rx.c,
 * play.c, out.c, servo.c and telemetry.c. The state they share, and the rules
 * about who may write what, are in sat.h — read that first.
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

/**
 * @brief The 5 s window the two slow loops share.
 *
 * @param arg Unused.
 *
 * One task, because they must not interleave: the health line is meant to be
 * read against the servo decision taken in the SAME window, and two tasks
 * would make that ordering a coincidence. Telemetry first — if audio has
 * stopped, the heap and the counters matter more than the trim, and the
 * servo's first act is to give up when there is no stream.
 */
static void drift_task(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        telemetry_tick();
        servo_tick();
    }
}

/**
 * @brief Say which build this is, at boot.
 *
 * @param tag ESP_LOG tag to print under.
 *
 * Compile time and ELF hash both change on every rebuild, so this settles
 * whether a reflash actually landed. The target is on it because one tree
 * builds two different units — the S3 runs the analyser lane and the classic
 * does not — so without it the line says a reflash landed but not which kind.
 */
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

/** @brief Core sentinel for @ref task_start(): let the scheduler choose. */
#define TASK_ANY_CORE (-1)

/**
 * @brief xTaskCreate with the return value actually read.
 *
 * @param fn    Task entry point.
 * @param name  Task name, also what a failure is reported as.
 * @param stack Stack size in bytes.
 * @param prio  FreeRTOS priority.
 * @param core  Core to pin to, or @ref TASK_ANY_CORE.
 *
 * Not a panic, for the same reason the heap is not configured to abort on a
 * failed allocation: a reboot loop on a dance floor is worse than a unit that
 * comes up crippled and says so. A satellite missing its play task can still
 * join, still answer probes, and still tell the collector what is wrong with
 * it, which is strictly more than a board stuck in the bootloader can do.
 *
 * The failure is logged here AND recorded in @ref s_task_fail_names, because
 * on this unit the console is the channel most likely to be missing: app_main
 * runs about a second before DHCP completes, so anything logged here is
 * dropped by wifi_log for want of a route. The names let the GOT_IP handler
 * say them once the radio can carry them.
 */
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

/** @brief Entry point: NVS, the allocation hook, the ring, WiFi, the socket,
 *  I2S, the optional marker and visualiser, then the four tasks. */
void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Before anything else allocates, so a failure during WiFi or socket
     * setup is caught too -- that is the phase with the largest requests. */
    ESP_ERROR_CHECK(heap_caps_register_failed_alloc_callback(on_alloc_failed));

    sync_est_init(&est);
    ESP_LOGI(TAG, "SBC decoder init: %s", sbc_decoder_init() ? "ok" : "FAILED");

    /* Say the size and the room either side of it. This is the allocation most
     * likely to fail on a classic ESP32, where the ceiling is a contiguous
     * block, and a bare assert() fails as an unexplained reboot loop. Logged on
     * success too: the successful number is what tells the next person how much
     * room the next increase has. */
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

    /* Mirror ESP_LOG lines to the hub. Compiles to nothing unless
     * CONFIG_DANCEFLOOR_WIFI_LOGS is set. */
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
    /* The second big contiguous allocation, and logged either side for the
     * reason the ring is. On a local-analysis build the analysis stream is
     * another single block taken after the ring has already come out of the
     * largest one; on the S3 it is asked for in PSRAM and this pair barely
     * moves. Printed for both sources deliberately -- a remote unit allocates
     * no stream, so its two numbers should be equal, and a pair that is not
     * equal on a build claiming to be remote is worth seeing. */
    const size_t vis_largest_before =
            heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    visualiser_start();
    ESP_LOGI(TAG, "visualiser started (%s): largest free internal block "
                  "%u -> %u",
             visualiser_source_name(), (unsigned)vis_largest_before,
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

    /* The strip draws each frame when the instant it names comes round, and
     * that instant has to be converted out of master time first. Passed as a
     * function rather than a value because stream_offset is slewed towards the
     * estimate, so a copy taken once would go stale at exactly the crystal
     * difference. */
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
