
#include <stdio.h>

#include "esp_app_desc.h"
#include "esp_log.h"

#include "sbc_in.h"
#include "streamer.h"
#include "visualiser.h"

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

    streamer_start();
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
    visualiser_start();
#else
    ESP_LOGW("hub", "visualiser DISABLED (menuconfig) -- LEDs will stay dark");
#endif
    sbc_in_start();
}
