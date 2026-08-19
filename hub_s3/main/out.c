/*
 * The output clock: the I2S channel this unit's own DAC hangs off, and every
 * change to its rate.
 *
 * Two callers and they want different things. sbc_in reports the rate the
 * decoder is actually producing (streamer_set_sample_rate), which is a coarse,
 * once-per-5s match; the servo in servo.c asks for fine trims. retune_dac() is
 * the one path both go through, and the sanity bound it applies is why a
 * wrapped phase error cannot ask for a 4.29 GHz sample rate again.
 */
#include "hub.h"

void streamer_set_sample_rate(uint32_t hz)
{
    if (!hz) {
        return;
    }
    sample_rate = hz;
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
    /* The LEDs convert between the timeline and a sample position, and this is
     * the rate the other half of that conversion uses: `sample_rate` is what
     * stamps every packet and what the playback task interpolates due_us with.
     * They have to be the same number. */
    visualiser_set_rate(hz);
#endif
    /* Smooth it: single windows carry ~0.3% noise, and every retune glitches
     * audio. The servo below wants a stable baseline, not the latest sample. */
    rate_ema = rate_ema ? (rate_ema * 3 + hz) / 4 : hz;

    /*
     * Match the DAC clock to the measured input rate.
     *
     * Transmitting at 44100 while receiving 42400 drains the playback buffer at
     * ~7 kB/s -- the 250 ms of audio is gone in five seconds and never recovers.
     * That is a 4% mismatch: 40000 ppm, against the ~14 ppm crystal drift M6 is
     * designed for. No sample-level correction can absorb it; the clocks have to
     * agree.
     *
     * Matching is right whether the deficit is a genuinely slower source or lost
     * frames: either way this board only receives `hz` frames per second, so
     * playing them at `hz` is what keeps real time.
     *
     * 1% threshold: measurement noise is ~0.3%, and retuning glitches audio.
     */
    /* Big initial mismatch (44100 nominal vs ~42600 actual) is corrected once,
     * immediately. Everything finer is left to the servo.
     *
     * WHICH SERVOES PHASE, not the buffer level -- this said "the buffer level
     * rather than the noisy rate estimate" and neither half was true of the
     * code. df_servo_step() derives its correction purely from the phase EMA and
     * consults depth only as a clamp past +-120 ms. That matters to a reader
     * here because it is exactly why a slow drift inside that band went
     * uncorrected for 45 minutes on the 2026-08-19 soak: nothing was watching
     * depth, and the thing that was watching -- phase -- was fine. */
    if (i2s_tx && (hz > tx_rate + tx_rate / 100 || hz < tx_rate - tx_rate / 100)) {
        retune_dac(hz);
    } else {
        /* Only when it moves. This ran every 5 s and printed the same 44100
         * every time -- a twelfth of the console spent saying nothing changed. */
        static uint32_t told_hz;
        if (hz != told_hz) {
            told_hz = hz;
            ESP_LOGI(TAG, "sample rate %" PRIu32 " Hz", hz);
        }
    }
}

/* I2S_NUM_1 by history: port 0 used to be the slave receiver from the bridge.
 * That link is SPI now, but there is no reason to move this. */
/*
 * The moment silence starts, counted. Both units carry this; see the satellite's
 * copy for the full reasoning.
 *
 * In short: auto_clear makes a starved channel emit zeroes rather than repeat
 * itself, which made every stall shorter than the underrun timeout invisible --
 * no counter, no log, and samples_played not advancing, so the only trace was a
 * phase error the servo then chased with a retune. The driver raises
 * on_send_q_ovf when the DMA has been all the way round its descriptor ring
 * without the writer taking a buffer back, which with auto_clear on is exactly
 * when the output goes quiet.
 *
 * This end is where it matters most for diagnosis: the hub loses no packets, so
 * anything it silences it did to itself -- and sbc_in at priority 9 sits above
 * play at 8 on the same core, decoding, feeding two stream buffers and calling
 * sendto in one block. If that block ever exceeds the 34.8 ms the DMA holds,
 * this is the counter that says so.
 *
 * ONLY COUNTED WHILE SOMETHING IS SUPPOSED TO BE FEEDING IT. Counting every
 * overflow made this read ~20000 on a healthy hub: the channel is enabled from
 * boot, so before the bridge delivers a first packet -- and through every
 * underrun park -- nothing writes, every descriptor completion overflows, and it
 * accrues at ~172/s for as long as that lasts. The total then said nothing about
 * playback and looked like a catastrophe; it was frozen across consecutive
 * HEALTH lines, which is what gave it away.
 *
 * s_playing is the play task's own flag, already used by the servo for the same
 * question, and `retuning` covers the re-enable after a clock change where the
 * descriptors are empty by construction -- that cost is reported as
 * `channel down` and does not belong here twice.
 */
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
    /*
     * One descriptor per chunk, so a write never spans two.
     *
     * i2s_channel_disable() waits for the in-flight write to release the DMA
     * queue, and a write that needs a second descriptor waits a second
     * descriptor period for it. At the default 240 against AUDIO_FRAMES of 256
     * every single write needed two, which is most of why this unit's retunes
     * were down 2-18 ms against the satellite's 2-6. Matching them makes the
     * worst case one period.
     *
     * Both units must carry this: it also sets the output pipeline latency the
     * servo absorbs at startup, and unequal depths park them at different
     * standing offsets.
     */
    chan_cfg.dma_frame_num = AUDIO_FRAMES;
    /*
     * A starved channel must go SILENT, not repeat itself. Without this, the
     * circular TX descriptors replay the last 34.8 ms forever once
     * local_play_task takes the underrun branch and parks -- see the
     * satellite's copy for the full mechanism. Both units must carry it.
     */
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
    /* Compare this against the satellite's line. The sync marker fires when a
     * chunk is written, not when it is heard, so unequal output buffering shows
     * up as a fixed offset unrelated to clock sync. */
    ESP_LOGW(TAG, "OUTPUT: I2S external DAC, buffer %d x %d frames = %d ms, "
                  "channels=%s, silence on starve",
             chan_cfg.dma_desc_num, chan_cfg.dma_frame_num,
             (int)(chan_cfg.dma_desc_num * chan_cfg.dma_frame_num * 1000 / rate),
             AUDIO_CHANNEL_MODE_NAME);
}

void retune_dac(uint32_t hz)
{
    /*
     * Nothing computed may panic the speaker. The satellite aborted on exactly
     * this path when a wrapped phase error asked for a 4.29 GHz sample rate:
     * ESP_ERROR_CHECK turned a bad number into a dead unit. A refused retune
     * costs sync, which is recoverable; an abort is not.
     */
    if (hz < RATE_SANE_MIN || hz > RATE_SANE_MAX) {
        n_retunes_bad++;
        ESP_LOGE(TAG, "refusing a %" PRIu32 " Hz retune -- not a sample rate", hz);
        return;
    }

    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(hz);
    retuning = true;
    /* Timed from here, so the 2 ms park counts: playback is stopped for it just
     * as surely as for the disable itself. */
    const int64_t down_at = esp_timer_get_time();
    vTaskDelay(pdMS_TO_TICKS(2));        /* let the play task notice and park */

    esp_err_t err = i2s_channel_disable(i2s_tx);
    if (err == ESP_OK) {
        err = i2s_channel_reconfig_std_clock(i2s_tx, &clk);
        /* Re-enable either way: leaving the channel down stalls playback
         * silently, which looks like a dead board rather than a failed trim. */
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
    /* Ordered after the two above: the play task reads them together and this
     * is what arms the narration. */
    s_retune_tail_left = 3;
    s_retune_done_at = esp_timer_get_time();

    /*
     * Nothing to tell the visualiser: it counts what ARRIVES, and a retune
     * disturbs playback rather than arrival. It used to need telling, on the
     * reasoning that the disable discarded the DMA buffer this task had already
     * counted as played -- retired both by moving the analysis off the playback
     * path and by the satellite's REFILL instrument showing the descriptors are
     * not discarded at all.
     */
}
