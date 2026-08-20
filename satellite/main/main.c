/*
 * Dancefloor satellite -- startup and task wiring.
 *
 * Joins the master's SoftAP, keeps its clock aligned with the master's, receives
 * SBC packets -- from a multicast group by default, per-unit unicast behind the
 * same switch (DANCEFLOOR_AUDIO_MCAST) -- decodes them, and plays each at the
 * instant it was stamped for.
 *
 * No Bluetooth here: the master owns the phone connection. This board only
 * listens on WiFi and drives a DAC.
 *
 * This file is app_main and nothing else: bring the peripherals up, then start
 * the four tasks. The behaviour is in the modules beside it --
 *
 *     net.c        joining the AP, reconnecting, the UDP socket
 *     clock.c      the probe task, TSF-or-estimator, the offset slew
 *     rx.c         demux, anchor policy, gap policy, decode, ring feed
 *     play.c       the scheduled start, phase measurement, splice, marker
 *     out.c        the I2S channel, the write path, retuning its clock
 *     servo.c      one 5 s window of rate control
 *     telemetry.c  one 5 s window of heap figures, HEALTH and MEM
 *
 * -- and the state they share, with the rules about who may write what, is in
 * sat.h. Read that first.
 *
 * It used to say this was milestone M5, which "aligns the *start* of playback"
 * and leaves the ~14 ppm crystal difference as "M6's problem". Both halves have
 * been untrue for a long time: position is held, not just started, by the phase
 * servo in servo.c, and the buffer-level log this comment pointed at as the
 * future signal has been driving that servo for most of the project's life.
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

/*
 * The 5 s window both slow loops run on.
 *
 * One task, because they must not interleave: the health line is meant to be
 * read against the servo decision taken in the same window, and two tasks would
 * make that ordering a coincidence. Telemetry first, as it always ran -- if
 * audio has stopped, the heap and the counters matter more than the trim, and
 * the servo's first act is to give up when there is no stream.
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

/* Printed at boot so a log immediately identifies which build produced it --
 * compile time and ELF hash both change on every rebuild. Saves guessing
 * whether a reflash actually landed.
 *
 * The TARGET is on it because this app is now built two ways from one tree, and
 * the two images are different units rather than the same one with different
 * pins: the S3 runs the analyser lane and the classic does not. Without it the
 * line says a reflash landed but not WHICH kind landed, which on a bench with
 * both boards attached is the question actually being asked. What the unit does
 * with its frames is already on the telemetry line -- see
 * visualiser_source_name() -- so it is not repeated here. */
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

/*
 * xTaskCreate, with the return value actually read. See n_task_fail above for
 * what discarding it cost.
 *
 * Not a panic, for the same reason CONFIG_HEAP_ABORT_WHEN_ALLOCATION_FAILS is
 * off: a reboot loop on a dance floor is worse than a unit that comes up
 * crippled and says so. A satellite missing its play task can still join, still
 * answer probes, and still tell the collector what is wrong with it -- which is
 * strictly more than a board stuck in the bootloader can do.
 *
 * The failure is logged here for the console AND recorded for the join line,
 * because on this unit the console is the channel most likely to be missing.
 * app_main runs about a second before DHCP completes, so anything logged here
 * is dropped by wifi_log for want of a route (s_no_dest) and never reaches the
 * collector. The names are kept so the GOT_IP handler can say them once the
 * radio can carry them.
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

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Before anything else allocates, so a failure during WiFi or socket setup
     * is caught too -- that is the phase with the largest single requests. */
    ESP_ERROR_CHECK(heap_caps_register_failed_alloc_callback(on_alloc_failed));

    sync_est_init(&est);
    /* Link check only for now -- the SBC receive path lands in the next stage. */
    ESP_LOGI(TAG, "SBC decoder init: %s", sbc_decoder_init() ? "ok" : "FAILED");
    /*
     * SAY THE SIZE AND THE ROOM, BEFORE AND AFTER, because this allocation is
     * the one most likely to fail on a classic ESP32 and the bare assert() that
     * used to stand here failed as an unexplained reboot loop.
     *
     * RING_BYTES follows the hub's LEAD_US (see RING_TARGET_MS in sat.h) and
     * grew 80 -> 96 kB when the lead went 250 -> 350. The ceiling is a
     * CONTIGUOUS block on a board whose largest has been measured at ~106 kB, so
     * the margin is real but thin, and it is the kind of thing that only a boot
     * can settle -- exactly the trap hub_s3/sdkconfig.defaults records for the
     * 40th TX buffer, which boot-looped for the same reason.
     *
     * Logged unconditionally, not only on failure: the successful number is what
     * tells the next person how much room the next increase has.
     */
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
    /* Mirror ESP_LOG lines to the hub (and a structured HEALTH, below). No-op
     * and compiles to nothing unless CONFIG_DANCEFLOOR_WIFI_LOGS is set. */
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
    visualiser_start();
    /*
     * The strip draws each frame when the instant it names comes round, and on
     * this board that instant has to be converted out of master time first.
     *
     * Read live rather than captured: stream_offset is slewed toward the
     * estimate at 200 ppm, so a copy taken once would go stale at exactly the
     * crystal difference -- which is the bug docs/clock-sync.md section 9
     * records in the audio path, where the servo was fed its own drift as its
     * reference and faithfully parked the speaker at the growing error.
     */
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
