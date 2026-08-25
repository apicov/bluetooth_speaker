/**
 * @file main.c
 * @brief Dancefloor hub -- chip B of the two-chip master.
 *
 *   phone --A2DP--> [bt_bridge ESP32] --SPI--> [this chip] --WiFi--> satellites
 *                                     raw SBC                --I2S--> its DAC
 *                                                            --SPI--> its LEDs
 *
 * This chip runs no Bluetooth. It owns the clock the whole floor syncs to,
 * sends the audio on to the satellites, and is a full speaker in its own
 * right. The split is what makes that possible: on a single chip Bluedroid
 * and the WiFi stack together left 164 bytes of DRAM free, which forced the
 * FFT down to 512 points and the buffers down to a quarter size. Apart, each
 * chip has ~200 kB spare and neither radio contends with the other.
 */
#include <stdio.h>

#include "esp_app_desc.h"
#include "esp_log.h"

#include "sbc_in.h"
#include "streamer.h"
#include "visualiser.h"

/**
 * @brief Print the compile time and ELF hash. Both change on every rebuild,
 *        so a log names the build that produced it and a reflash that did
 *        not land shows up immediately.
 * @param tag  Log tag to prefix the line with.
 */
static void log_build_stamp(const char *tag)
{
    const esp_app_desc_t *d = esp_app_get_description();
    char sha[17] = {0};
    for (int i = 0; i < 8; i++) {
        sprintf(sha + i * 2, "%02x", d->app_elf_sha256[i]);
    }
    ESP_LOGW(tag, "BUILD %s %s  elf:%s", d->date, d->time, sha);
}

/** @brief IDF entry point: log the build, bring the output side up, then the
 *         input side. */
void app_main(void)
{
    log_build_stamp("hub");
    ESP_LOGI("hub", "dancefloor hub starting");

    /* Streamer first: it brings up NVS, the AP and the sockets the rest
     * assume exist, and it owns the DAC. */
    streamer_start();
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
    visualiser_start();
#else
    ESP_LOGW("hub", "visualiser DISABLED (menuconfig) -- LEDs will stay dark");
#endif
    sbc_in_start();
}
