#include "hub.h"

void streamer_set_sample_rate(uint32_t hz)
{
    if (!hz) {
        return;
    }
    sample_rate = hz;
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
    visualiser_set_rate(hz);
#endif
    rate_ema = rate_ema ? (rate_ema * 3 + hz) / 4 : hz;

    if (i2s_tx && (hz > tx_rate + tx_rate / 100 || hz < tx_rate - tx_rate / 100)) {
        retune_dac(hz);
    } else {
        static uint32_t told_hz;
        if (hz != told_hz) {
            told_hz = hz;
            ESP_LOGI(TAG, "sample rate %" PRIu32 " Hz", hz);
        }
    }
}

static volatile uint32_t s_dma_starve;

static bool IRAM_ATTR on_tx_starved(i2s_chan_handle_t h, i2s_event_data_t *e, void *ctx)
{
    (void)h; (void)e; (void)ctx;
    if (s_playing && !retuning) {
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
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
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
    tx_rate = rate;
    ESP_LOGW(TAG, "OUTPUT: I2S external DAC, buffer %d x %d frames = %d ms, "
                  "channels=%s, silence on starve",
             chan_cfg.dma_desc_num, chan_cfg.dma_frame_num,
             (int)(chan_cfg.dma_desc_num * chan_cfg.dma_frame_num * 1000 / rate),
             AUDIO_CHANNEL_MODE_NAME);
}

void retune_dac(uint32_t hz)
{
    if (hz < RATE_SANE_MIN || hz > RATE_SANE_MAX) {
        n_retunes_bad++;
        ESP_LOGE(TAG, "refusing a %" PRIu32 " Hz retune -- not a sample rate", hz);
        return;
    }

    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(hz);
    retuning = true;
    const int64_t down_at = esp_timer_get_time();
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
    s_retune_outage_us = esp_timer_get_time() - down_at;
    n_retunes++;
    ESP_LOGW(TAG, "DAC clock retuned %" PRIu32 " -> %" PRIu32 " Hz, channel down %lld us",
             tx_rate, hz, s_retune_outage_us);
    tx_rate = hz;

    s_retune_phase_before = s_phase_err_us;
    s_retune_watch = true;
    s_retune_tail_left = 3;
    s_retune_done_at = esp_timer_get_time();
}
