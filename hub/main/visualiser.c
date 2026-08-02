#include "visualiser.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_dsp.h"
#include "led_strip.h"
#include "led_strip_spi.h"

#include "beat_detect.h"

/* 43 Hz bins at 44.1 kHz. 512 is too coarse to resolve a kick fundamental --
 * it was forced on us briefly when Bluedroid shared this chip, and the two-chip
 * split bought the ~20 kB back. */
#define FFT_N       1024
#define SAMPLE_RATE 44100
#define STREAM_BYTES (FFT_N * 4 * 2)      /* ~2 analysis frames of stereo int16 */

static const char *TAG = "vis";

static StreamBufferHandle_t pcm_stream;
static led_strip_handle_t strip;
static beat_det_t beat;

/* FFT scratch. Static rather than stack: 12 KB would blow a default task stack. */
static __attribute__((aligned(16))) float fft_buf[FFT_N * 2];   /* complex interleaved */
static float window[FFT_N];
static float mono[FFT_N];

/*
 * Band edges in FFT bins at 44.1 kHz / 1024 (43.07 Hz per bin):
 *   0: 43-129 Hz    kick
 *   1: 172-990 Hz   low-mid / snare body
 *   2: 1.0-5.0 kHz  presence
 *   3: 5.0-22 kHz   air / hats
 * Bin 0 is DC and deliberately excluded.
 */
static const int BAND_LO[BEAT_BANDS] = { 1,  4,  24, 117 };
static const int BAND_HI[BEAT_BANDS] = { 3, 23, 116, FFT_N / 2 - 1 };

/* Empirical. Real music will need this retuned on hardware -- see README. */
#define BAND_GAIN 12.0f

void visualiser_feed(const uint8_t *pcm, uint32_t len)
{
    if (pcm_stream) {
        xStreamBufferSend(pcm_stream, pcm, len, 0);   /* 0 ticks: never block the BT task */
    }
}

static void compute_bands(float band[BEAT_BANDS])
{
    for (int i = 0; i < FFT_N; i++) {
        fft_buf[2 * i]     = mono[i] * window[i];
        fft_buf[2 * i + 1] = 0.0f;
    }

    dsps_fft2r_fc32(fft_buf, FFT_N);
    dsps_bit_rev_fc32(fft_buf, FFT_N);

    for (int b = 0; b < BEAT_BANDS; b++) {
        float sum = 0.0f;
        for (int k = BAND_LO[b]; k <= BAND_HI[b]; k++) {
            float re = fft_buf[2 * k], im = fft_buf[2 * k + 1];
            sum += sqrtf(re * re + im * im);
        }
        float v = (sum / (float)(BAND_HI[b] - BAND_LO[b] + 1)) * BAND_GAIN / (float)FFT_N * 2.0f;
        band[b] = v > 1.0f ? 1.0f : v;
    }
}

/* led_strip v3 dropped the hsv2rgb helper that older versions shipped.
 * h 0..360, s and v 0..1. */
static void hsv2rgb(float h, float s, float v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    float c = v * s;
    float x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
    float m = v - c;
    float rf, gf, bf;

    if      (h <  60.0f) { rf = c; gf = x; bf = 0; }
    else if (h < 120.0f) { rf = x; gf = c; bf = 0; }
    else if (h < 180.0f) { rf = 0; gf = c; bf = x; }
    else if (h < 240.0f) { rf = 0; gf = x; bf = c; }
    else if (h < 300.0f) { rf = x; gf = 0; bf = c; }
    else                 { rf = c; gf = 0; bf = x; }

    *r = (uint32_t)((rf + m) * 255.0f);
    *g = (uint32_t)((gf + m) * 255.0f);
    *b = (uint32_t)((bf + m) * 255.0f);
}

static void render(float level, float bass)
{
    /* One pattern for now: brightness tracks the beat envelope, hue drifts slowly
     * so a static room does not look frozen between hits. Chase effects across
     * multiple units belong with M5, once units share a clock. */
    static float hue = 0.0f;
    hue += 0.3f;
    if (hue >= 360.0f) hue -= 360.0f;

    uint32_t r, g, b;
    hsv2rgb(hue, 1.0f, level, &r, &g, &b);

    for (int i = 0; i < CONFIG_DANCEFLOOR_LED_COUNT; i++) {
        /* Bass pushes colour outward from the centre of the strip. */
        float pos = fabsf((float)i / CONFIG_DANCEFLOOR_LED_COUNT - 0.5f) * 2.0f;
        float k = bass > pos ? 1.0f : 0.25f;
        led_strip_set_pixel(strip, i, (uint32_t)(r * k), (uint32_t)(g * k), (uint32_t)(b * k));
    }

    /* Refresh failures are silent otherwise, and a wedged strip driver fails every
     * single frame -- log the first one rather than flooding the console. */
    esp_err_t err = led_strip_refresh(strip);
    if (err != ESP_OK) {
        static bool reported = false;
        if (!reported) {
            reported = true;
            ESP_LOGE(TAG, "led_strip_refresh failed: %s", esp_err_to_name(err));
        }
    }
}

static void visualiser_task(void *arg)
{
    (void)arg;
    static int16_t raw[FFT_N * 2];        /* stereo interleaved */
    float level = 0.0f;
    size_t filled = 0;

    while (1) {
        /*
         * Accumulate. A stream buffer returns as soon as its trigger level is
         * met, so a read is usually partial -- discarding those would mean the
         * FFT never sees a whole frame, and re-rendering on every partial read
         * turns this into a spin loop that hammers the strip driver flat out.
         */
        size_t got = xStreamBufferReceive(pcm_stream, (uint8_t *)raw + filled,
                                          sizeof(raw) - filled, pdMS_TO_TICKS(100));
        filled += got;

        if (filled < sizeof(raw)) {
            if (got == 0) {
                /* Genuinely starved, not merely mid-frame: decay to black rather
                 * than freezing on the last frame. Bounded to one render per
                 * timeout, so this cannot become a spin. */
                level *= 0.8f;
                render(level, 0.0f);
            }
            continue;
        }
        filled = 0;

        for (int i = 0; i < FFT_N; i++) {
            mono[i] = ((float)raw[2 * i] + (float)raw[2 * i + 1]) / 65536.0f;
        }

        float band[BEAT_BANDS];
        compute_bands(band);

        float strength = 0.0f;
        if (beat_det_update(&beat, band, esp_timer_get_time(), &strength)) {
            level = 0.3f + 0.7f * strength;
        } else {
            level *= 0.85f;               /* ~23 ms per frame, so a visible decay */
        }
        render(level, band[0]);
    }
}

void visualiser_start(void)
{
    /* Trigger level is one full analysis frame: waking the task for a handful of
     * bytes at a time is pure overhead now that partial reads accumulate. */
    pcm_stream = xStreamBufferCreate(STREAM_BYTES, FFT_N * 2 * sizeof(int16_t));
    assert(pcm_stream);

    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, FFT_N));
    dsps_wind_hann_f32(window, FFT_N);
    beat_det_init(&beat);

    /*
     * SPI + DMA, not bit-banging and not RMT.
     *
     * Bit-banging is out for the obvious reason: it disables interrupts for the
     * whole strip update, starving the I2S DMA and the Bluetooth stack.
     *
     * RMT was the first choice but does not survive this workload on the ESP32.
     * led_strip's RMT backend calls rmt_enable() / rmt_disable() around *every*
     * frame, and rmt_disable races the transmit-done ISR as it walks the channel
     * through RUN -> WAIT -> ENABLE. Landing in WAIT fails with
     * "channel can't be disabled in state 3", after which the channel is stuck
     * enabled and every later frame fails. A busy BT stack delaying the RMT ISR
     * widens that window; it took roughly 650 frames to hit in practice.
     *
     * The SPI backend transmits in one DMA-driven spi_device_transmit() with no
     * enable/disable cycle, so there is no FSM to race. The original ESP32's RMT
     * has no DMA at all, which makes SPI the better fit here regardless.
     *
     * Cost: the whole SPI bus is reserved, even though only MOSI is used.
     */
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = CONFIG_DANCEFLOOR_LED_GPIO,
        .max_leds = CONFIG_DANCEFLOOR_LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_spi_config_t spi_cfg = {
        .clk_src = SPI_CLK_SRC_DEFAULT,
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_cfg, &spi_cfg, &strip));
    ESP_ERROR_CHECK(led_strip_clear(strip));

    /* Core 1: core 0 belongs to the Bluetooth stack. */
    xTaskCreatePinnedToCore(visualiser_task, "vis", 4096, NULL, 4, NULL, 1);
    ESP_LOGI(TAG, "started: %d LEDs on GPIO %d",
             CONFIG_DANCEFLOOR_LED_COUNT, CONFIG_DANCEFLOOR_LED_GPIO);
}
