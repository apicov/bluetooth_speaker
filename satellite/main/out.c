/*
 * The audio output: the I2S channel, the write path, and retuning its clock.
 *
 * Split out of main.c on 2026-08-12; the bodies are unchanged. The retune_*
 * state the servo and the play task share moved to sat.h with everything else
 * that crosses a task boundary.
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

/* -------------------------------------------------------------------- i2s */

void i2s_start(uint32_t rate)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    /* One descriptor per chunk, so a write never spans two and the disable
     * waits at most one descriptor period for it. See the hub's copy for the
     * mechanism; both units must carry it, because this also sets the output
     * pipeline latency the servo absorbs at startup. */
    chan_cfg.dma_frame_num = AUDIO_FRAMES;
    /*
     * A starved channel must go SILENT, not repeat itself.
     *
     * The TX descriptors are a circular list, so a channel left enabled with
     * nobody writing replays its last dma_desc_num x AUDIO_FRAMES -- 34.8 ms --
     * forever, at 28.7 Hz. That is what the room heard when Bluetooth dropped
     * mid-track: play_task takes the underrun branch, parks, and stops writing,
     * but nothing disables the channel, so the DMA carries on with whatever it
     * was holding until the phone comes back.
     *
     * This makes the driver zero each buffer at its own EOF -- after it has
     * played, before the writer is handed it back -- so a stall drains to
     * digital zero within one traversal and stays there. It cannot race the
     * writer: the clear happens before the pointer reaches the queue
     * i2s_channel_write() pops from.
     *
     * No sync cost, and it must be on BOTH units. The number of buffers in
     * flight is unchanged, so the pipeline latency the servo absorbs at startup
     * is unchanged, and so is the REFILL figure that measures it. What would be
     * unequal, if only one unit carried it, is what the floor sounds like.
     */
    chan_cfg.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_tx, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(rate),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = CONFIG_DANCEFLOOR_I2S_BCK_PIN,
            .ws   = CONFIG_DANCEFLOOR_I2S_LRCK_PIN,
            .dout = CONFIG_DANCEFLOOR_I2S_DATA_PIN,
            .din  = I2S_GPIO_UNUSED,
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_tx, &std_cfg));
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
    /* Second line of defence behind the `retuning` park: a failed write does
     * NOT block, so ignoring it lets this task spin through the ring at memory
     * speed. That is what a retune used to cost. */
    if (i2s_channel_write(i2s_tx, pcm, bytes, &written, portMAX_DELAY) != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (s_refill_active) {
        if (esp_timer_get_time() - w0 < REFILL_FAST_US) {
            s_refill_frames += (int32_t)(written / (AUDIO_CHANNELS * sizeof(int16_t)));
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

/*
 * The LEDs used to be fed from here, at the DAC, so that they reacted to what
 * this speaker was actually playing rather than to what had merely arrived.
 *
 * They are fed from the receive path now. The objection to that was real -- ~200
 * ms of ring sits between arrival and the speaker, so lights driven from
 * arrival ran that far ahead of the sound -- and it stopped applying when
 * rendering became scheduled: a frame is drawn when the instant it names comes
 * round, not when it was computed, so where it was computed no longer decides
 * when it is seen. What moving it buys is those 200 ms as processing headroom,
 * which is what lets an algorithm cost more than one frame period.
 *
 * Anyone putting this back must put the scheduling back too, or the lights lead
 * the sound by the whole buffer again.
 */
void write_audio(const uint8_t *pcm, size_t bytes)
{
    dac_write(pcm, bytes);
}

#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
/*
 * When a master-clock instant falls on this board's clock.
 *
 * The same conversion playback uses, against the same slewed offset, so the
 * strip and the speaker are answering to one timeline rather than two. Before
 * the first anchor stream_offset is 0 and this is the identity, which dates the
 * handful of frames produced before a timeline exists into the past -- they are
 * drawn at once, which is the right thing to do with a frame that has no
 * schedule to keep.
 */
int64_t vis_master_to_local(int64_t master_us)
{
    return sync_to_local(master_us, stream_offset);
}
#endif

void retune_output(uint32_t hz)
{
    /*
     * Nothing the servo computes may panic the speaker.
     *
     * This aborted the board with ESP_ERROR_CHECK when a wrapped phase error
     * produced a rate of ~4.29e9 Hz: the servo arithmetic went wrong, and what
     * the room heard was a satellite rebooting. A refused retune costs sync; an
     * abort costs the unit. Clamp, log, carry on playing at the current rate.
     */
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

    /* Park playback before taking the channel down, and count the park as part
     * of the outage -- audio is stopped for it just as surely. */
    retuning = true;
    vTaskDelay(pdMS_TO_TICKS(2));

    esp_err_t err = i2s_channel_disable(i2s_tx);
    if (err == ESP_OK) {
        err = i2s_channel_reconfig_std_clock(i2s_tx, &clk);
        /* Re-enable whatever happened: leaving the channel down stalls playback
         * silently, which reads as a dead board rather than a failed trim. */
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

    /* Ask playback to report the first phase it measures after this, so the net
     * cost is one printed number rather than a difference between two 5 s log
     * ticks with several ms of wander in each. */
    retune_phase_before = phase_err_us;
    retune_watch = true;
    /* Ordered after the two above: the play task reads them together and this
     * is what arms the narration. See retune_done_at. */
    retune_tail_left = 3;
    retune_done_at = esp_timer_get_time();

    /*
     * Nothing to tell the visualiser. It counts what ARRIVES now, and a retune
     * disturbs playback rather than arrival -- so the block grid it rides on is
     * untouched by anything that happens here.
     *
     * It used to need telling, on the reasoning that disabling the channel
     * discarded the DMA buffer, which the playback task had already counted as
     * played and already fed onward. Two things retired that: the analysis is no
     * longer on the playback path at all, and the REFILL instrument showed the
     * descriptors are not discarded in the first place.
     */
}
