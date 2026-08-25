/**
 * @file out.c
 * @brief The output clock: the I2S channel this unit's DAC hangs off, and
 * every change to its rate.
 *
 * Two callers feed it: sbc_in reports the rate the decoder is actually
 * producing (a coarse, once-per-5-s match) and the servo in servo.c asks for
 * fine trims. Both paths meet at retune_dac(), whose sanity bound is what
 * keeps a wrapped phase error from asking the driver for a 4.29 GHz sample
 * rate.
 */
#include "hub.h"

void streamer_set_sample_rate(uint32_t hz)
{
    if (!hz) {
        return;
    }
    sample_rate = hz;
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
    /* The LEDs convert between the timeline and a sample position; both
     * halves of that conversion must use this same rate. */
    visualiser_set_rate(hz);
#endif
    /* Smoothed: single windows carry ~0.3% noise, and every retune glitches
     * audio -- the servo trims against a stable baseline, not the latest
     * reading. */
    rate_ema = rate_ema ? (rate_ema * 3 + hz) / 4 : hz;

    /* A gross mismatch -- nominal 44100 against a measured ~42600 -- is
     * corrected once, immediately: playing at the wrong rate drains the
     * playback buffer at ~7 kB/s, which no sample-level correction can
     * absorb. Anything finer belongs to the servo, which corrects phase;
     * buffer depth only clamps it. */
    if (i2s_tx && (hz > tx_rate + tx_rate / 100 || hz < tx_rate - tx_rate / 100)) {
        retune_dac(hz);
    } else {
        /* Logged only on change. */
        static uint32_t told_hz;
        if (hz != told_hz) {
            told_hz = hz;
            ESP_LOGI(TAG, "sample rate %" PRIu32 " Hz", hz);
        }
    }
}

/** @brief DMA-starvation events counted while playback owns the channel. */
/*
 * The I2S driver raises on_send_q_ovf when the DMA
 * has been all the way round its descriptor ring without the writer taking a
 * buffer back; with auto_clear on, that is exactly when the output goes
 * silent.
 *
 * Counted only while playback is supposed to be feeding the channel
 * (s_playing, and not mid-retune): the channel runs from boot, and while
 * nothing feeds it, every descriptor completion overflows -- a healthy unit
 * would accrue ~172/s of counts that say nothing about playback.
 *
 * The hub loses no packets, so anything this counts it did to itself --
 * typically sbc_in (priority 9, above play at 8 on the same core) holding
 * the CPU for longer than the 34.8 ms of audio the DMA ring holds.
 */
static volatile uint32_t s_dma_starve;

/**
 * @brief I2S event callback: count one starvation event.
 * @param h   Channel handle (unused).
 * @param e   Event data (unused).
 * @param ctx Callback context (unused).
 * @return false -- the event needs no handling beyond the counter.
 */
static bool IRAM_ATTR on_tx_starved(i2s_chan_handle_t h, i2s_event_data_t *e, void *ctx)
{
    (void)h; (void)e; (void)ctx;
    if (s_playing && !retuning) {
        s_dma_starve++;
    }
    return false;
}

/**
 * @brief DMA-starvation count for the HEALTH line.
 * @return Events counted while playing, since boot.
 */
uint32_t dma_starve_count(void)
{
    return s_dma_starve;
}

/**
 * @brief Bring up the I2S TX channel and its DMA for this unit's DAC.
 * @param rate  Initial sample rate in Hz.
 */
void i2s_start(uint32_t rate)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    /* One descriptor per chunk, so a write never spans two. A write that
     * needs a second descriptor makes i2s_channel_disable() wait a further
     * period for it; against the driver default of 240 frames per
     * descriptor, AUDIO_FRAMES of 256 meant every write needed two, and the
     * retune outage paid it. satellite/main/out.c sets the same value:
     * unequal depths park the units at different standing offsets. */
    chan_cfg.dma_frame_num = AUDIO_FRAMES;
    /* A starved channel must go silent, not repeat the last 34.8 ms of audio
     * forever: without auto_clear, the circular TX descriptors replay their
     * contents while the play task is parked in the underrun branch. The
     * satellite carries the same setting. */
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
    /* Registered before enable, so the first traversal is covered. */
    const i2s_event_callbacks_t cbs = { .on_send_q_ovf = on_tx_starved };
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(i2s_tx, &cbs, NULL));
    ESP_ERROR_CHECK(i2s_channel_enable(i2s_tx));
    tx_rate = rate;
    /* The sync marker fires when a chunk is written, not when it is heard, so
     * unequal output buffering between the units reads as a fixed offset on
     * the marker, unrelated to clock sync. */
    ESP_LOGW(TAG, "OUTPUT: I2S external DAC, buffer %d x %d frames = %d ms, "
                  "channels=%s, silence on starve",
             chan_cfg.dma_desc_num, chan_cfg.dma_frame_num,
             (int)(chan_cfg.dma_desc_num * chan_cfg.dma_frame_num * 1000 / rate),
             AUDIO_CHANNEL_MODE_NAME);
}

/**
 * @brief Move the DAC clock to a new rate, parking playback for the switch.
 * @param hz  Target rate in Hz. Refused outside RATE_SANE_MIN..RATE_SANE_MAX:
 *            a refused retune costs sync, which is recoverable -- a garbage
 *            rate handed to the driver aborts the unit, which is not.
 */
void retune_dac(uint32_t hz)
{
    if (hz < RATE_SANE_MIN || hz > RATE_SANE_MAX) {
        n_retunes_bad++;
        ESP_LOGE(TAG, "refusing a %" PRIu32 " Hz retune -- not a sample rate", hz);
        return;
    }

    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(hz);
    retuning = true;
    /* Timed from before the park: the play task is stopped for the delay
     * just as surely as for the disable itself. */
    const int64_t down_at = esp_timer_get_time();
    vTaskDelay(pdMS_TO_TICKS(2));        /* let the play task notice and park */

    esp_err_t err = i2s_channel_disable(i2s_tx);
    if (err == ESP_OK) {
        err = i2s_channel_reconfig_std_clock(i2s_tx, &clk);
        /* Re-enable either way: a channel left down stalls playback silently
         * and reads as a dead board. */
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
    /* Set last: the play task reads these together, and this is the one that
     * arms the post-retune log tail. */
    s_retune_tail_left = 3;
    s_retune_done_at = esp_timer_get_time();

    /* No visualiser notification: it counts arrivals, and a retune disturbs
     * playback, not arrival. */
}
