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

static volatile uint32_t s_dma_starve;

static bool IRAM_ATTR on_tx_starved(i2s_chan_handle_t h, i2s_event_data_t *e, void *ctx)
{
    (void)h; (void)e; (void)ctx;
    if (playing && !retuning) {
        s_dma_starve++;
    }
    return false;
}

uint32_t dma_starve_count(void)
{
    return s_dma_starve;
}

void i2s_start(uint32_t rate)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

    chan_cfg.dma_frame_num = AUDIO_FRAMES;

    chan_cfg.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_tx, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(rate),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = CONFIG_DANCEFLOOR_I2S_BCK_PIN,
            .ws   = CONFIG_DANCEFLOOR_I2S_LRCK_PIN,
            .dout = CONFIG_DANCEFLOOR_I2S_DATA_PIN,
            .din  = I2S_GPIO_UNUSED,
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_tx, &std_cfg));

    const i2s_event_callbacks_t cbs = { .on_send_q_ovf = on_tx_starved };
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(i2s_tx, &cbs, NULL));
    ESP_ERROR_CHECK(i2s_channel_enable(i2s_tx));
    ESP_LOGW(TAG, "OUTPUT: I2S external DAC, buffer %d x %d frames = %d ms, "
                  "channels=%s, silence on starve",
             chan_cfg.dma_desc_num, chan_cfg.dma_frame_num,
             (int)(chan_cfg.dma_desc_num * chan_cfg.dma_frame_num * 1000 / rate),
             AUDIO_CHANNEL_MODE_NAME);
}

static void dac_write(const uint8_t *pcm, size_t bytes)
{
    size_t written = 0;
    const int64_t w0 = s_refill_active ? esp_timer_get_time() : 0;

    if (i2s_channel_write(i2s_tx, pcm, bytes, &written, portMAX_DELAY) != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (s_refill_active) {
        if (esp_timer_get_time() - w0 < REFILL_FAST_US) {
            s_refill_frames += (int32_t)(written / AUDIO_OUT_FRAME_BYTES);
        } else {
            s_refill_active = false;
            ESP_LOGW(TAG, "REFILL after start: %ld frames (%ld ms) before a write "
                          "blocked -- phase readings inside this window are not "
                          "DAC-paced",
                     (long)s_refill_frames,
                     (long)(s_refill_frames * 1000 / (int32_t)stream_rate));
        }
    }
}

static audio_ramp_t s_ramp;

void write_audio(const int16_t *frames, size_t n_frames, uint8_t vol)
{
    static audio_out_sample_t out[AUDIO_FRAMES * AUDIO_CHANNELS];
    audio_volume_write_i32(out, frames, n_frames, vol, &s_ramp);
    dac_write((const uint8_t *)out, n_frames * AUDIO_OUT_FRAME_BYTES);
}

void write_audio_reset_ramp(void)
{
    s_ramp.cur = 0;
}

#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER

int64_t vis_master_to_local(int64_t master_us)
{
    return sync_to_local(master_us, stream_offset);
}
#endif

void retune_output(uint32_t hz)
{

    const int64_t low  = (int64_t)stream_rate - RATE_TRIM_MAX_HZ;
    const int64_t high = (int64_t)stream_rate + RATE_TRIM_MAX_HZ;
    if ((int64_t)hz < low || (int64_t)hz > high) {
        n_retunes_bad++;
        ESP_LOGE(TAG, "refusing a %" PRIu32 " Hz retune (nominal %" PRIu32
                      ") -- the servo input is not trustworthy", hz, stream_rate);
        return;
    }

    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(hz);
    const int64_t down_at = esp_timer_get_time();

    retuning = true;
    vTaskDelay(pdMS_TO_TICKS(2));

    esp_err_t err = i2s_channel_disable(i2s_tx);
    if (err == ESP_OK) {
        err = i2s_channel_reconfig_std_clock(i2s_tx, &clk);

        const esp_err_t on = i2s_channel_enable(i2s_tx);
        if (err == ESP_OK) {
            err = on;
        }
    }
    retuning = false;

    if (err != ESP_OK) {
        n_retunes_bad++;
        ESP_LOGE(TAG, "retune to %" PRIu32 " Hz failed (%s), staying at %" PRIu32,
                 hz, esp_err_to_name(err), tx_rate);
        return;
    }
    retune_outage_us = esp_timer_get_time() - down_at;
    n_retunes++;
    ESP_LOGW(TAG, "output clock retuned %" PRIu32 " -> %" PRIu32 " Hz, channel down %lld us",
             tx_rate, hz, retune_outage_us);
    tx_rate = hz;

    retune_phase_before = phase_err_us;
    retune_watch = true;

    retune_tail_left = 3;
    retune_done_at = esp_timer_get_time();

}
