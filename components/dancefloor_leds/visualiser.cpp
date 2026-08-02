#include "visualiser.h"

#include <array>
#include <atomic>
#include <cinttypes>
#include <cmath>
#include <cstring>
#include <optional>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_dsp.h"

#include "beat_detect.h"
#include "led_strip_wrapper.hpp"

namespace {

constexpr const char *TAG = "vis";

/* 43 Hz bins at 44.1 kHz. 512 is too coarse to resolve a kick fundamental -- it
 * was forced on us briefly when Bluedroid shared this chip, and the two-chip
 * split bought the ~20 kB back. */
constexpr int FFT_N        = 1024;
constexpr int SAMPLE_RATE  = 44100;
constexpr int CHANNELS     = 2;
/* Four analysis frames of headroom, not two. A short send loses audio AND
 * breaks the block alignment, so the buffer is sized to make that rare. */
constexpr int STREAM_BYTES = FFT_N * CHANNELS * (int)sizeof(int16_t) * 4;

constexpr uint32_t LED_COUNT = CONFIG_DANCEFLOOR_LED_COUNT;

/* Applied to every pixel on the way out. See Kconfig for why it is not 100. */
constexpr float BRIGHTNESS = CONFIG_DANCEFLOOR_LED_BRIGHTNESS / 100.0f;

#if   CONFIG_DANCEFLOOR_LED_TYPE_SK6812
constexpr LedStrip::Type STRIP_TYPE = LedStrip::Type::SK6812_RGBW;
#elif CONFIG_DANCEFLOOR_LED_TYPE_WS2811
constexpr LedStrip::Type STRIP_TYPE = LedStrip::Type::WS2811;
#else
constexpr LedStrip::Type STRIP_TYPE = LedStrip::Type::WS2812;
#endif

/*
 * Band edges in FFT bins at 44.1 kHz / 1024 (43.07 Hz per bin):
 *   0: 43-129 Hz    kick
 *   1: 172-990 Hz   low-mid / snare body
 *   2: 1.0-5.0 kHz  presence
 *   3: 5.0-22 kHz   air / hats
 * Bin 0 is DC and deliberately excluded.
 */
constexpr std::array<int, BEAT_BANDS> BAND_LO = { 1,  4,  24, 117 };
constexpr std::array<int, BEAT_BANDS> BAND_HI = { 3, 23, 116, FFT_N / 2 - 1 };

/* Empirical, and tuned against synthetic kicks rather than real music. Expect to
 * revisit this with a recording -- see tools/desktop_satellite.py --record. */
constexpr float BAND_GAIN = 12.0f;

StreamBufferHandle_t pcm_stream;
std::optional<LedStrip> strip;
beat_det_t beat;

/* esp_timer_get_time() + this = master-clock time. Written by the audio task on
 * anchoring, read by the render task; 64-bit, so both sides go through a
 * critical section rather than risking a torn read of a value that changes by
 * minutes. */
portMUX_TYPE s_offset_lock = portMUX_INITIALIZER_UNLOCKED;
int64_t s_master_offset_raw;

int64_t master_offset()
{
    portENTER_CRITICAL(&s_offset_lock);
    const int64_t v = s_master_offset_raw;
    portEXIT_CRITICAL(&s_offset_lock);
    return v;
}

/* Block alignment, and the statistics that make a disagreement between two units
 * diagnosable instead of a matter of opinion. All touched only by the feed
 * (audio task) except the counters, which the report reads and clears. */
bool    s_align_pending = true;
bool    s_mark_align_point;
int32_t s_skip_frames;

/*
 * Where aligned audio starts, as a byte index into the stream that actually
 * passed through the buffer. Written by the feed, read by the analysis task.
 *
 * 32-bit and allowed to wrap: at 176 kB/s that is every 6.8 hours, and every
 * comparison is a signed difference between two values a fraction of a second
 * apart, which is wrap-safe.
 */
std::atomic<uint32_t> s_align_at_byte;
uint32_t s_sent_total;          /* feed side only */
/* Atomic, not volatile: these are incremented by the audio task and read and
 * cleared by the analysis task, and C++20 deprecates compound assignment on a
 * volatile. Relaxed ordering -- they are statistics, not a handshake. */
std::atomic<uint32_t> s_dropped;  /* PCM bytes lost because the analysis fell behind */
std::atomic<uint32_t> s_aligns;   /* re-alignments; should settle to 0 per window */
std::atomic<uint32_t> s_onsets;
std::atomic<uint32_t> s_frames;

uint32_t take(std::atomic<uint32_t> &c) { return c.exchange(0, std::memory_order_relaxed); }
void bump(std::atomic<uint32_t> &c, uint32_t n = 1) { c.fetch_add(n, std::memory_order_relaxed); }

/* FFT scratch. File scope rather than stack: 12 kB would blow a task stack. */
alignas(16) float fft_buf[FFT_N * 2];   /* complex interleaved */
float window[FFT_N];
float mono[FFT_N];

void compute_bands(float band[BEAT_BANDS])
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
            const float re = fft_buf[2 * k], im = fft_buf[2 * k + 1];
            sum += std::sqrt(re * re + im * im);
        }
        const float v = (sum / static_cast<float>(BAND_HI[b] - BAND_LO[b] + 1))
                        * BAND_GAIN / static_cast<float>(FFT_N) * 2.0f;
        band[b] = v > 1.0f ? 1.0f : v;
    }
}

struct Rgb { uint8_t r, g, b; };

/* led_strip v3 dropped the hsv2rgb helper that older versions shipped.
 * h 0..360, s and v 0..1. */
Rgb hsv2rgb(float h, float s, float v)
{
    const float c = v * s;
    const float x = c * (1.0f - std::fabs(std::fmod(h / 60.0f, 2.0f) - 1.0f));
    const float m = v - c;
    float rf, gf, bf;

    if      (h <  60.0f) { rf = c; gf = x; bf = 0; }
    else if (h < 120.0f) { rf = x; gf = c; bf = 0; }
    else if (h < 180.0f) { rf = 0; gf = c; bf = x; }
    else if (h < 240.0f) { rf = 0; gf = x; bf = c; }
    else if (h < 300.0f) { rf = x; gf = 0; bf = c; }
    else                 { rf = c; gf = 0; bf = x; }

    return { static_cast<uint8_t>((rf + m) * 255.0f),
             static_cast<uint8_t>((gf + m) * 255.0f),
             static_cast<uint8_t>((bf + m) * 255.0f) };
}

void render(float level, float bass, int64_t master_us)
{
    /* One pattern: brightness tracks the beat envelope, hue drifts slowly so a
     * static room does not look frozen between hits.
     *
     * Hue is a function of master-clock time, not of a counter incremented once
     * per render. Two units do not render at the same rate -- audio arrives in
     * different-sized lumps and a starved unit renders extra decay frames -- so
     * an accumulated hue drifts apart between units and beats in and out of
     * agreement over tens of seconds. Computed from the shared clock it is
     * identical on every unit by construction, and a unit that stalls for a
     * second rejoins in the right place instead of lagging forever. */
    constexpr float HUE_PERIOD_US = 28.0f * 1000000.0f;   /* one full cycle */
    float hue = std::fmod(static_cast<float>(master_us % static_cast<int64_t>(HUE_PERIOD_US))
                          / HUE_PERIOD_US * 360.0f, 360.0f);
    if (hue < 0.0f) hue += 360.0f;      /* master_us is positive, but % is not */

    const Rgb c = hsv2rgb(hue, 1.0f, level * BRIGHTNESS);

    for (uint32_t i = 0; i < LED_COUNT; i++) {
        /* Bass pushes colour outward from the centre of the strip. */
        const float pos = std::fabs(static_cast<float>(i) / LED_COUNT - 0.5f) * 2.0f;
        const float k = bass > pos ? 1.0f : 0.25f;
        strip->set(i, static_cast<uint8_t>(c.r * k),
                      static_cast<uint8_t>(c.g * k),
                      static_cast<uint8_t>(c.b * k));
    }

    /* Refresh failures are silent otherwise, and a wedged strip driver fails
     * every single frame -- log the first rather than flooding the console. */
    if (const esp_err_t err = strip->show(); err != ESP_OK) {
        static bool reported = false;
        if (!reported) {
            reported = true;
            ESP_LOGE(TAG, "strip refresh failed: %s", esp_err_to_name(err));
        }
    }
}

/* Envelope decay expressed as a time constant rather than a per-frame factor,
 * for the same reason the hue is: a unit that renders more often would otherwise
 * decay faster and its pulses would visibly differ in shape. 0.85 per 23 ms
 * frame, which is what this replaces, is a ~140 ms time constant. */
constexpr float DECAY_TAU_US = 140000.0f;

float decay(float level, int64_t dt_us)
{
    if (dt_us <= 0) return level;
    return level * std::exp(-static_cast<float>(dt_us) / DECAY_TAU_US);
}

void visualiser_task(void *arg)
{
    (void)arg;
    static int16_t raw[FFT_N * 2];        /* stereo interleaved */
    float level = 0.0f;
    size_t filled = 0;
    int64_t last_render_us = esp_timer_get_time();
    int64_t last_report_us = last_render_us;
    uint32_t recv_total = 0;      /* bytes taken out of the buffer, ever */
    uint32_t epoch = 0;           /* matches s_align_at_byte's initial value */

    while (true) {
        /*
         * Accumulate. A stream buffer returns as soon as its trigger level is
         * met, so a read is usually partial -- discarding those would mean the
         * FFT never sees a whole frame, and re-rendering on every partial read
         * turns this into a spin loop that hammers the strip driver flat out.
         */
        const size_t got = xStreamBufferReceive(pcm_stream,
                                                reinterpret_cast<uint8_t *>(raw) + filled,
                                                sizeof(raw) - filled, pdMS_TO_TICKS(100));
        filled += got;
        recv_total += got;

        /*
         * Has the feed re-aligned? Then everything before its published byte
         * index is stale and must not contribute to a block, including whatever
         * partial block is being held here. Dropping only the feed side -- which
         * is what the first version did -- leaves this partial block in place
         * and the boundaries offset by its length, so the re-alignment silently
         * did nothing.
         */
        const uint32_t align_at = s_align_at_byte.load(std::memory_order_relaxed);
        if (align_at != epoch) {
            const int32_t ahead = static_cast<int32_t>(recv_total - align_at);
            if (ahead < 0) {
                filled = 0;             /* still draining pre-alignment audio */
                continue;
            }
            /* Keep only the tail belonging to the new epoch, and start the next
             * block exactly at the aligned byte. */
            epoch = align_at;
            const size_t keep = static_cast<size_t>(ahead) < filled
                                ? static_cast<size_t>(ahead) : filled;
            std::memmove(raw, reinterpret_cast<uint8_t *>(raw) + (filled - keep), keep);
            filled = keep;
        }

        if (filled < sizeof(raw)) {
            if (got == 0) {
                /* Genuinely starved, not merely mid-frame: decay to black rather
                 * than freezing on the last frame. Bounded to one render per
                 * timeout, so this cannot become a spin. */
                const int64_t now = esp_timer_get_time();
                const int64_t offset = master_offset();
                level = decay(level, now - last_render_us);
                last_render_us = now;
                render(level, 0.0f, now + offset);
            }
            continue;
        }
        filled = 0;

        for (int i = 0; i < FFT_N; i++) {
            mono[i] = (static_cast<float>(raw[2 * i]) +
                       static_cast<float>(raw[2 * i + 1])) / 65536.0f;
        }

        float band[BEAT_BANDS];
        compute_bands(band);

        const int64_t now = esp_timer_get_time();
        const int64_t offset = master_offset();
        float strength = 0.0f;
        bump(s_frames);
        if (beat_det_update(&beat, band, now, &strength)) {
            level = 0.3f + 0.7f * strength;
            bump(s_onsets);
        } else {
            level = decay(level, now - last_render_us);
        }
        last_render_us = now;
        render(level, band[0], now + offset);

        /*
         * Report every 5 s. The point is comparability: run two units side by
         * side and their `onsets` should match closely, because they are
         * analysing identical windows of identical audio. A persistent
         * difference means they are not -- check `drop` first.
         */
        if (now - last_report_us >= 5000000) {
            ESP_LOGI(TAG, "frames %" PRIu32 " | onsets %" PRIu32
                          " | drop %" PRIu32 " B | aligns %" PRIu32,
                     take(s_frames), take(s_onsets), take(s_dropped), take(s_aligns));
            last_report_us = now;
        }
    }
}

}  // namespace

void visualiser_feed(const uint8_t *pcm, uint32_t len, int64_t due_master_us)
{
    if (!pcm_stream) {
        return;
    }

    /*
     * Align the analysis blocks to the shared timeline before anything else.
     *
     * The detector chops the stream into fixed 1024-frame blocks, and where
     * those boundaries fall used to depend on when each unit's task happened to
     * see its first byte -- an arbitrary offset of up to 1023 frames, 23 ms,
     * between two units. A drum hit landing near a boundary is split across two
     * blocks on one unit and sits in the middle of one block on the other, so
     * its spectral flux differs and a marginal onset crosses the threshold on
     * one unit and not the other. Worse, the threshold is adaptive over the last
     * 43 blocks, so once two units disagree they keep disagreeing.
     *
     * Discarding a few frames here puts every unit's boundaries on the same
     * absolute positions in the stream, so they analyse identical windows and
     * reach identical decisions.
     *
     * The boundary is derived from `due_master_us` -- the instant this audio is
     * SCHEDULED to be heard, interpolated from the hub's per-packet play_at
     * stamps -- and not from a clock read here. Every unit gets the same play_at
     * for the same audio, so every unit computes the same boundary for the same
     * sample.
     *
     * Reading a clock here instead, which an earlier version did, aligns to when
     * this board reached this line. Two boards reach it a few ms apart, and that
     * skew lands straight on the block boundaries: 3 ms is 132 samples of 1024,
     * so roughly one transient in eight is split differently between them. It is
     * far better than the uniform 0-1023 offset of no alignment at all, which is
     * why it looked improved but not fixed.
     */
    const uint32_t frame_bytes = CHANNELS * sizeof(int16_t);
    if (s_align_pending && due_master_us > 0) {
        const int64_t idx = (due_master_us * SAMPLE_RATE) / 1000000;
        const int32_t into_block = static_cast<int32_t>(idx % FFT_N);
        s_skip_frames = into_block ? (FFT_N - into_block) : 0;
        s_align_pending = false;
        s_mark_align_point = true;
        bump(s_aligns);
    }
    if (s_skip_frames > 0) {
        const uint32_t have = len / frame_bytes;
        const uint32_t drop = s_skip_frames < static_cast<int32_t>(have)
                              ? static_cast<uint32_t>(s_skip_frames) : have;
        s_skip_frames -= static_cast<int32_t>(drop);
        pcm += drop * frame_bytes;
        len -= drop * frame_bytes;
        if (len == 0) {
            return;
        }
    }

    /*
     * Publish the byte index at which aligned audio begins.
     *
     * Discarding here is not on its own enough, and the first version of this
     * was broken for exactly that reason: the analysis task holds a partially
     * accumulated block, and completing it with freshly aligned data leaves its
     * boundaries offset by however much it happened to be holding. The discard
     * looked like it worked and did nothing. Both units start aligned, and the
     * first dropped feed on either one un-aligns it permanently.
     *
     * So the two sides agree on a position instead. Both count bytes that
     * actually passed through the buffer -- the count only advances by what the
     * send accepted, so a drop cannot make them disagree -- and the reader
     * throws away everything before this index.
     */
    if (s_mark_align_point) {
        s_align_at_byte.store(s_sent_total, std::memory_order_relaxed);
        s_mark_align_point = false;
    }

    /* 0 ticks: never block the audio task that calls this. A short send means
     * the analysis task fell behind and audio was lost -- which both breaks the
     * block alignment and makes this unit analyse different audio from its
     * neighbours, so count it and re-align rather than losing it silently. */
    const size_t sent = xStreamBufferSend(pcm_stream, pcm, len, 0);
    s_sent_total += sent;
    if (sent < len) {
        bump(s_dropped, len - sent);
        s_align_pending = true;
    }
}

void visualiser_set_master_offset(int64_t offset_us)
{
    portENTER_CRITICAL(&s_offset_lock);
    s_master_offset_raw = offset_us;
    portEXIT_CRITICAL(&s_offset_lock);
}

void visualiser_start(void)
{
    /* Trigger level is one full analysis frame: waking the task for a handful of
     * bytes at a time is pure overhead now that partial reads accumulate. */
    pcm_stream = xStreamBufferCreate(STREAM_BYTES, FFT_N * 2 * sizeof(int16_t));
    assert(pcm_stream);

    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(nullptr, FFT_N));
    dsps_wind_hann_f32(window, FFT_N);
    beat_det_init(&beat);

    /*
     * SPI + DMA, not bit-banging and not RMT.
     *
     * Bit-banging is out for the obvious reason: it disables interrupts for the
     * whole strip update, starving the I2S DMA and the Bluetooth stack.
     *
     * RMT was the first choice and does not survive this workload on the ESP32 --
     * see components/led_strip_wrapper/README.md for the failure and why SPI has
     * no equivalent. The original ESP32's RMT has no DMA at all, which makes SPI
     * the better fit here regardless.
     */
    strip.emplace(static_cast<gpio_num_t>(CONFIG_DANCEFLOOR_LED_GPIO), LED_COUNT,
                  STRIP_TYPE, LedStrip::Backend::SPI);
    strip->clear();
    strip->show();

    /* Core 1 alongside playback, at a lower priority than it: dropped frames
     * here are invisible, dropped audio is not. */
    xTaskCreatePinnedToCore(visualiser_task, "vis", 4096, nullptr, 4, nullptr, 1);
    ESP_LOGI(TAG, "started: %lu LEDs on GPIO %d at %d%% brightness",
             LED_COUNT, CONFIG_DANCEFLOOR_LED_GPIO, CONFIG_DANCEFLOOR_LED_BRIGHTNESS);
}
