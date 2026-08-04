/*
 * Dancefloor hub -- chip B of the two-chip master.
 *
 *   phone --A2DP--> [bt_bridge ESP32] --UART--> [this chip] --WiFi--> satellites
 *                                     raw SBC              --I2S---> its own DAC
 *                                                          --SPI---> its own LEDs
 *
 * This chip runs no Bluetooth. It owns the clock the whole system syncs to,
 * sends the audio on to the satellites, and is a full speaker in its own right.
 *
 * Splitting the master in two is what makes that possible: Bluedroid and the
 * WiFi stack together left 164 bytes of DRAM free on one chip, which forced the
 * FFT down to 512 points and the buffers down to a quarter of their proper size.
 * Apart, each chip has ~200 kB spare and neither radio contends with the other.
 */
#include <stdio.h>

#include "esp_app_desc.h"
#include "esp_log.h"

#include "sbc_in.h"
#include "streamer.h"
#include "visualiser.h"


/* Printed at boot so a log immediately identifies which build produced it --
 * compile time and ELF hash both change on every rebuild. Saves guessing
 * whether a reflash actually landed. */
static void log_build_stamp(const char *tag)
{
    const esp_app_desc_t *d = esp_app_get_description();
    char sha[17] = {0};
    for (int i = 0; i < 8; i++) {
        sprintf(sha + i * 2, "%02x", d->app_elf_sha256[i]);
    }
    ESP_LOGW(tag, "BUILD %s %s  elf:%s", d->date, d->time, sha);
}

void app_main(void)
{
    log_build_stamp("hub");
    ESP_LOGI("hub", "dancefloor hub starting");

    /* Streamer first: it brings up NVS, WiFi and the sockets that the others
     * assume exist, and it owns the DAC. */
    streamer_start();
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
    visualiser_start();
#else
    ESP_LOGW("hub", "visualiser DISABLED (menuconfig) -- LEDs will stay dark");
#endif
    sbc_in_start();
}
