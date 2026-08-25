
#include "visualiser.h"

#include <atomic>
#include <cinttypes>
#include <cmath>
#include <cstring>
#include <optional>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"

#include "freertos/idf_additions.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER
#include "driver/gpio.h"
#endif

#include "analysis.hpp"
#include "patterns.hpp"
#include "ml_lane.hpp"
#include "result_latch.hpp"
#include "led_strip_wrapper.hpp"

namespace {

constexpr const char *TAG = "vis";

#if CONFIG_DANCEFLOOR_LED_SOURCE_REMOTE
#define DF_ANALYSES_AUDIO      0
#define DF_TAKES_REMOTE_FRAMES 1
#else
#define DF_ANALYSES_AUDIO      1
#define DF_TAKES_REMOTE_FRAMES 0
#endif

#if CONFIG_DANCEFLOOR_ML
#define DF_RUNS_ANALYSERS 1
#else
#define DF_RUNS_ANALYSERS 0
#endif

#if DF_TAKES_REMOTE_FRAMES && DF_RUNS_ANALYSERS
#error "DANCEFLOOR_ML needs LED_SOURCE_LOCAL: a frame off the wire carries no spectrum for an analyser to read (see vis_frame_t)."
#endif

using df::FFT_N;
using df::HOP_N;
using df::TAIL_N;
using df::RATE;
using df::CHANNELS;

constexpr int STREAM_BYTES = FFT_N * CHANNELS * (int)sizeof(int16_t) * 8;
constexpr uint32_t FRAME_BYTES = CHANNELS * sizeof(int16_t);

constexpr size_t HOP_BYTES  = (size_t)HOP_N * CHANNELS * sizeof(int16_t);
constexpr size_t TAIL_BYTES = (size_t)TAIL_N * CHANNELS * sizeof(int16_t);

constexpr uint32_t LED_COUNT = CONFIG_DANCEFLOOR_LED_COUNT;

#ifndef CONFIG_DANCEFLOOR_LOG_PERIOD_S
#define CONFIG_DANCEFLOOR_LOG_PERIOD_S 20
#endif
constexpr int64_t LED_LOG_PERIOD_US = CONFIG_DANCEFLOOR_LOG_PERIOD_S * 1000000LL;

#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER

constexpr int64_t LED_MARKER_PERIOD_US = 1000000;

constexpr int64_t LED_MARKER_HIGH_US = 40000;

constexpr int64_t MARKER_IDLE_US = 1500000;

#if CONFIG_DANCEFLOOR_LED_MARKER_ACTIVE_LOW
constexpr int LED_MARKER_ON  = 0;
constexpr int LED_MARKER_OFF = 1;
#else
constexpr int LED_MARKER_ON  = 1;
constexpr int LED_MARKER_OFF = 0;
#endif

std::atomic<bool> s_marker_link{false};

void marker_write(int level)
{
    static int shown = -1;
    if (level == shown) return;
    shown = level;
    gpio_set_level(static_cast<gpio_num_t>(CONFIG_DANCEFLOOR_LED_MARKER_GPIO), level);
}

constexpr int64_t MARKER_BUSY_TOGGLE_US = 80 * 1000;

static esp_timer_handle_t s_marker_busy_timer;
static int s_marker_busy_level = LED_MARKER_OFF;

static void marker_busy_cb(void *)
{
    s_marker_busy_level = (s_marker_busy_level == LED_MARKER_ON) ? LED_MARKER_OFF
                                                                 : LED_MARKER_ON;
    gpio_set_level(static_cast<gpio_num_t>(CONFIG_DANCEFLOOR_LED_MARKER_GPIO),
                   s_marker_busy_level);
}

int marker_idle_level()
{
    return s_marker_link.load(std::memory_order_relaxed) ? LED_MARKER_ON
                                                         : LED_MARKER_OFF;
}

void marker_service(int64_t now, int64_t *flash_at, int64_t *lower_at)
{
    if (*lower_at && now >= *lower_at) {
        *lower_at = 0;
        marker_write(LED_MARKER_OFF);
    }
    if (*flash_at && now >= *flash_at) {
        *flash_at = 0;
        *lower_at = now + LED_MARKER_HIGH_US;
        marker_write(LED_MARKER_ON);

        static int told;
        if (told < 3) {
            told++;
            ESP_LOGW(TAG, "LED marker fired on GPIO %d (%d of 3) -- if the "
                          "LED is dark, this board's LED is not on that pin",
                     CONFIG_DANCEFLOOR_LED_MARKER_GPIO, told);
        }
    }
}

int64_t marker_clamp_nap(int64_t nap_ms, int64_t flash_at, int64_t lower_at)
{
    int64_t edge = flash_at;
    if (!edge || (lower_at && lower_at < edge)) {
        edge = lower_at;
    }
    if (!edge) {
        return nap_ms;
    }
    const int64_t left = (edge - esp_timer_get_time()) / 1000;
    if (left >= nap_ms) {
        return nap_ms;
    }
    return left > 1 ? left : 1;
}
#endif

constexpr float BRIGHTNESS = CONFIG_DANCEFLOOR_LED_BRIGHTNESS / 100.0f;

#if   CONFIG_DANCEFLOOR_LED_TYPE_SK6812
constexpr LedStrip::Type STRIP_TYPE = LedStrip::Type::SK6812_RGBW;
#elif CONFIG_DANCEFLOOR_LED_TYPE_WS2811
constexpr LedStrip::Type STRIP_TYPE = LedStrip::Type::WS2811;
#else
constexpr LedStrip::Type STRIP_TYPE = LedStrip::Type::WS2812;
#endif

StreamBufferHandle_t pcm_stream;
std::optional<LedStrip> strip;

#if DF_ANALYSES_AUDIO
df::Analysis analysis;
#endif

std::atomic<uint32_t> s_rate{df::RATE};
df::Pattern *pattern = nullptr;
uint8_t pixels[LED_COUNT * 3];

std::atomic<bool> s_align_pending{true};
bool     s_mark_align_point;
int32_t  s_skip_frames;
int64_t  s_pending_block_index;

std::atomic<uint32_t> s_align_at_byte;
std::atomic<long long> s_align_block_index;

std::atomic<uint32_t> s_align_gen;
uint32_t s_sent_total;

int64_t  s_ref_due;
uint32_t s_ref_byte;
bool     s_ref_valid;

constexpr int64_t ALIGN_DRIFT_US = 2000;

std::atomic<uint32_t> s_dropped;
std::atomic<uint32_t> s_aligns;
std::atomic<uint32_t> s_onsets;
std::atomic<uint32_t> s_frames;

std::atomic<uint32_t> s_booms;
std::atomic<uint32_t> s_marginal;

std::atomic<uint32_t> s_drifts;
std::atomic<int32_t>  s_last_drift_us;

[[maybe_unused]] uint32_t take(std::atomic<uint32_t> &c) { return c.exchange(0, std::memory_order_relaxed); }
void bump(std::atomic<uint32_t> &c, uint32_t n = 1) { c.fetch_add(n, std::memory_order_relaxed); }

void note_max(std::atomic<uint32_t> &c, uint32_t v)
{
    uint32_t prev = c.load(std::memory_order_relaxed);
    while (v > prev && !c.compare_exchange_weak(prev, v, std::memory_order_relaxed)) {
    }
}

constexpr uint32_t FRAME_RING = 32 * (FFT_N / HOP_N);
static_assert((FRAME_RING & (FRAME_RING - 1)) == 0,
              "FRAME_RING must be a power of two -- the uint32 wrap depends on it");
df::Frame s_fq[FRAME_RING];
std::atomic<uint32_t> s_fq_head;
std::atomic<uint32_t> s_fq_tail;
std::atomic<uint32_t> s_fq_flush;
std::atomic<uint32_t> s_late;
std::atomic<uint32_t> s_overrun;

[[maybe_unused]] std::atomic<uint32_t> s_ml_dropped;

#if DF_RUNS_ANALYSERS

void run_fast_lane(const uint8_t (&spec)[df::SPEC_BINS], int64_t index,
                   int64_t due_us, df::Result out[df::ML_SLOTS]);
#endif

std::atomic<uint32_t> s_render_sum, s_render_max, s_render_n;

std::atomic<uint32_t> s_pattern_sum, s_pattern_max;
std::atomic<uint32_t> s_show_sum, s_show_max;
std::atomic<uint32_t> s_wake_sum, s_wake_max, s_wake_n;

std::atomic<uint32_t> s_idle_dark;

std::atomic<int64_t (*)(int64_t)> s_to_local{nullptr};

std::atomic<void (*)(const vis_frame_t *)> s_publish{nullptr};

static_assert(VIS_BANDS == BEAT_BANDS, "wire frame lost a band");

#if DF_ANALYSES_AUDIO
void to_wire(const df::Frame &f, vis_frame_t *w)
{
    w->due_us = f.due_us;
    w->index  = f.index;
    std::memcpy(w->band, f.band, sizeof(w->band));
}
#endif

#if DF_TAKES_REMOTE_FRAMES
df::RemoteDetect s_remote_detect;
#endif

#if DF_TAKES_REMOTE_FRAMES
void from_wire(const vis_frame_t *w, df::Frame &f)
{
    f.due_us = w->due_us;
    f.index  = w->index;
    std::memcpy(f.band, w->band, sizeof(f.band));
    f.mag  = nullptr;

    std::memset(f.spec, 0, sizeof(f.spec));
    f.unit = 0;
}
#endif

bool enqueue(const df::Frame &f)
{
    const uint32_t head = s_fq_head.load(std::memory_order_relaxed);
    const uint32_t tail = s_fq_tail.load(std::memory_order_acquire);
    if (head - tail >= FRAME_RING) {
        bump(s_overrun);
        return false;
    }
    df::Frame &dst = s_fq[head % FRAME_RING];
    dst = f;

#if DF_RUNS_ANALYSERS
    df::Result fast_ml[df::ML_SLOTS];
    run_fast_lane(f.spec, f.index, f.due_us, fast_ml);
    for (int i = 0; i < df::ML_SLOTS; i++) {
        dst.ml[i] = fast_ml[i];
    }
    if (!df::ml_lane_feed(f.spec, f.index, f.due_us)) {
        bump(s_ml_dropped);
    }
#else
    for (int i = 0; i < df::ML_SLOTS; i++) {
        dst.ml[i] = df::result_none();
    }
#endif

    s_fq_head.store(head + 1, std::memory_order_release);
    return true;
}

df::ResultLatch s_latch;

[[maybe_unused]] std::atomic<uint32_t> s_ml_results;

constexpr int64_t RENDER_SLACK_US = 1000;

constexpr int64_t RENDER_NAP_MS = 20;

constexpr int64_t RENDER_LATE_US = 20000;

constexpr int64_t RENDER_IDLE_US = 500000;

void show(const uint8_t *rgb)
{
    for (uint32_t i = 0; i < LED_COUNT; i++) {
        strip->set(i, static_cast<uint8_t>(rgb[3 * i + 0] * BRIGHTNESS),
                      static_cast<uint8_t>(rgb[3 * i + 1] * BRIGHTNESS),
                      static_cast<uint8_t>(rgb[3 * i + 2] * BRIGHTNESS));
    }

    if (const esp_err_t err = strip->show(); err != ESP_OK) {
        static bool reported = false;
        if (!reported) {
            reported = true;
            ESP_LOGE(TAG, "strip refresh failed: %s", esp_err_to_name(err));
        }
    }
}

#if DF_RUNS_ANALYSERS

void run_fast_lane(const uint8_t (&spec)[df::SPEC_BINS], int64_t index,
                   int64_t due_us, df::Result out[df::ML_SLOTS])
{
    bool skip[df::ML_SLOTS];
    for (int i = 0; i < df::ML_SLOTS; i++) {
        skip[i] = s_latch.latched(i);
    }

    df::run_fast_lane(spec, index, due_us, skip, out);

    for (int i = 0; i < df::ML_SLOTS; i++) {
        if (df::result_valid(out[i])) {
            bump(s_ml_results);
        }
    }
}
#endif

#if DF_ANALYSES_AUDIO

void visualiser_task(void *arg)
{
    (void)arg;
    static int16_t raw[FFT_N * CHANNELS];

    size_t   filled = 0;
    uint32_t recv_total = 0;
    uint32_t seen_gen = 0;
    int64_t  block_index = 0;
    int64_t  last_report_us = esp_timer_get_time();
    bool     starved_shown = false;
    uint32_t seen_rate = s_rate.load(std::memory_order_relaxed);

    int64_t  cost_analysis = 0, cost_analysis_max = 0;
    uint32_t cost_n = 0;

    int64_t  cost_fast = 0, cost_fast_max = 0;

    while (true) {

        const uint32_t rate = s_rate.load(std::memory_order_relaxed);
        if (rate != seen_rate) {
            seen_rate = rate;
            analysis.init(static_cast<int>(rate));

            for (int i = 0; i < df::ML_SLOTS; i++) {
                if (df::Analyser *a = df::analyser_at(i);
                    a && !s_latch.latched(i) && !a->init(static_cast<int>(rate))) {

                    s_latch.set_latched(i, true);
                    ESP_LOGE(TAG, "analyser \"%s\" cannot run at %" PRIu32 " Hz -- retired",
                             a->spec().name, rate);
                }
            }
            if (pattern) pattern->reset();
            filled = 0;
            ESP_LOGW(TAG, "analysing at %" PRIu32 " Hz", rate);
        }

        const size_t got = xStreamBufferReceive(pcm_stream,
                                                reinterpret_cast<uint8_t *>(raw) + filled,
                                                sizeof(raw) - filled, pdMS_TO_TICKS(100));
        filled += got;
        recv_total += got;

        const uint32_t gen = s_align_gen.load(std::memory_order_acquire);
        if (gen != seen_gen) {
            const uint32_t align_at = s_align_at_byte.load(std::memory_order_relaxed);
            const int32_t ahead = static_cast<int32_t>(recv_total - align_at);
            if (ahead < 0) {
                filled = 0;
                continue;
            }
            seen_gen = gen;

#if DF_RUNS_ANALYSERS
            df::ml_lane_restart();
#endif
            block_index = s_align_block_index.load(std::memory_order_relaxed);
            const size_t keep = static_cast<size_t>(ahead) < filled
                                ? static_cast<size_t>(ahead) : filled;
            std::memmove(raw, reinterpret_cast<uint8_t *>(raw) + (filled - keep), keep);
            filled = keep;
        }

        if (filled < sizeof(raw)) {
            if (got == 0 && !starved_shown) {

                starved_shown = true;
                visualiser_flush();

                s_align_pending.store(true, std::memory_order_relaxed);
            }
            continue;
        }
        starved_shown = false;

        const int64_t due_us = block_index * HOP_N * 1000000LL / rate;
        const int64_t t_in = esp_timer_get_time();
        const df::Frame &f = analysis.process(raw, block_index, due_us, 0);
        const int64_t t_analysed = esp_timer_get_time();

        const int64_t t_fast = esp_timer_get_time();

        if constexpr (TAIL_BYTES > 0) {
            std::memmove(raw, reinterpret_cast<uint8_t *>(raw) + HOP_BYTES, TAIL_BYTES);
        }
        filled = TAIL_BYTES;

        block_index++;

        bump(s_frames);
        if (f.onset) bump(s_onsets);
        if (f.boom) bump(s_booms);
        if (f.boom_threshold > 0.0f &&
            std::fabs(f.boom_flux - f.boom_threshold) < 0.1f * f.boom_threshold) {
            bump(s_marginal);
        }

        enqueue(f);

        if (const auto publish = s_publish.load(std::memory_order_relaxed)) {
            vis_frame_t w;
            to_wire(f, &w);
            publish(&w);
        }
        {
            const int64_t a = t_analysed - t_in;
            cost_analysis += a;
            if (a > cost_analysis_max) cost_analysis_max = a;
            const int64_t m = t_fast - t_analysed;
            cost_fast += m;
            if (m > cost_fast_max) cost_fast_max = m;
            cost_n++;
        }

        const int64_t now = esp_timer_get_time();
        const uint32_t dropped = s_dropped.load(std::memory_order_relaxed);
        const uint32_t aligns  = s_aligns.load(std::memory_order_relaxed);
        const uint32_t drifts  = s_drifts.load(std::memory_order_relaxed);

        if (dropped || drifts || aligns > 1 || now - last_report_us >= LED_LOG_PERIOD_US) {
            ESP_LOGI(TAG, "frames %" PRIu32 " | onsets %" PRIu32
                          " | booms %" PRIu32 " (marginal %" PRIu32 ")"
                          " | drop %" PRIu32 " B | aligns %" PRIu32
                          " | drift %" PRIu32 " (last %+ld us) | %s",
                     take(s_frames), take(s_onsets), take(s_booms), take(s_marginal),
                     take(s_dropped), take(s_aligns),
                     take(s_drifts), (long)s_last_drift_us.load(std::memory_order_relaxed),
                     pattern ? pattern->name() : "no pattern");

            const uint32_t rn = take(s_render_n), rsum = take(s_render_sum);
            const uint32_t wn = take(s_wake_n), wsum = take(s_wake_sum);
            const uint32_t psum = take(s_pattern_sum), ssum = take(s_show_sum);

#if DF_RUNS_ANALYSERS
            const df::MlLaneStats ml = df::ml_lane_take_stats();
            ESP_LOGI(TAG, "ml: fast %lld/%lld us (mean/max) | slow %" PRIu32 "/%" PRIu32
                          " us | results %" PRIu32 "+%" PRIu32
                          " | late %" PRIu32 " | overrun %" PRIu32
                          " | lane drop %" PRIu32 " | restarts %" PRIu32,
                     cost_n ? cost_fast / cost_n : 0, cost_fast_max,
                     ml.cost_mean_us, ml.cost_max_us,
                     take(s_ml_results), ml.results,
                     s_latch.take_late(), s_latch.take_overrun(),
                     ml.dropped, ml.restarts);
#else
            ESP_LOGI(TAG, "ml: fast %lld/%lld us (mean/max) | no local lane"
                          " | results %" PRIu32
                          " | late %" PRIu32 " | overrun %" PRIu32,
                     cost_n ? cost_fast / cost_n : 0, cost_fast_max,
                     take(s_ml_results),
                     s_latch.take_late(), s_latch.take_overrun());
#endif
            cost_fast = cost_fast_max = 0;
            ESP_LOGI(TAG, "cost: analysis %lld/%lld us (mean/max) | render %" PRIu32
                          "/%" PRIu32 " us | pat %" PRIu32 "/%" PRIu32
                          " | show %" PRIu32 "/%" PRIu32
                          " | wake +%" PRIu32 "/%" PRIu32 " us"
                          " | queued %" PRIu32 " | late %" PRIu32
                          " | overrun %" PRIu32 " | dark %" PRIu32
                          " | n %" PRIu32 "/%" PRIu32
                          " | stack free %" PRIu32,
                     cost_analysis / cost_n, cost_analysis_max,
                     rn ? rsum / rn : 0, take(s_render_max),
                     rn ? psum / rn : 0, take(s_pattern_max),
                     rn ? ssum / rn : 0, take(s_show_max),
                     wn ? wsum / wn : 0, take(s_wake_max),
                     s_fq_head.load(std::memory_order_relaxed) -
                         s_fq_tail.load(std::memory_order_relaxed),
                     take(s_late), take(s_overrun), take(s_idle_dark), cost_n, rn,
                     uxTaskGetStackHighWaterMark(nullptr));
            cost_analysis = cost_analysis_max = 0;
            cost_n = 0;
            last_report_us = now;
        }
    }
}

#endif

void render_task(void *arg)
{
    (void)arg;
    uint32_t seen_flush = s_fq_flush.load(std::memory_order_relaxed);
    int64_t  drawn_at = 0;
#if !DF_ANALYSES_AUDIO
    int64_t  last_report_us = esp_timer_get_time();
#endif
#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER

    int64_t flash_at = 0;
    int64_t lower_at = 0;

    int64_t last_flash_sec = -1;

    int64_t marker_at   = 0;
    bool    marker_idle = true;
#endif

    while (true) {
#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER

        if (!marker_idle) {
            marker_service(esp_timer_get_time(), &flash_at, &lower_at);
        }
#endif
        const uint32_t flush = s_fq_flush.load(std::memory_order_acquire);
        if (flush != seen_flush) {
            seen_flush = flush;

            s_fq_tail.store(s_fq_head.load(std::memory_order_acquire),
                            std::memory_order_release);

            if (pattern) pattern->reset();

            s_latch.flush();
            std::memset(pixels, 0, sizeof(pixels));
            show(pixels);
#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER

            last_flash_sec = -1;
            flash_at = 0;
            lower_at = 0;
            marker_write(marker_idle ? marker_idle_level() : LED_MARKER_OFF);
#endif
        }

#if !DF_ANALYSES_AUDIO

        {
            const int64_t now_us = esp_timer_get_time();
            if (now_us - last_report_us >= LED_LOG_PERIOD_US) {
                last_report_us = now_us;
                const uint32_t rn = take(s_render_n), rsum = take(s_render_sum);
                const uint32_t wn = take(s_wake_n), wsum = take(s_wake_sum);
                const uint32_t psum = take(s_pattern_sum), ssum = take(s_show_sum);
                ESP_LOGI(TAG, "frames %" PRIu32 " | onsets %" PRIu32
                              " | booms %" PRIu32 " | render %" PRIu32 "/%" PRIu32
                              " us | pat %" PRIu32 "/%" PRIu32
                              " | show %" PRIu32 "/%" PRIu32
                              " | wake +%" PRIu32 "/%" PRIu32 " us"
                              " | queued %" PRIu32 " | late %" PRIu32
                              " | overrun %" PRIu32 " | dark %" PRIu32
                              " | ml %" PRIu32 " (late %" PRIu32
                              ", overrun %" PRIu32 ") | %s",
                         take(s_frames), take(s_onsets), take(s_booms),
                         rn ? rsum / rn : 0, take(s_render_max),
                         rn ? psum / rn : 0, take(s_pattern_max),
                         rn ? ssum / rn : 0, take(s_show_max),
                         wn ? wsum / wn : 0, take(s_wake_max),
                         s_fq_head.load(std::memory_order_relaxed) -
                             s_fq_tail.load(std::memory_order_relaxed),
                         take(s_late), take(s_overrun), take(s_idle_dark),
                         take(s_ml_results), s_latch.take_late(),
                         s_latch.take_overrun(),
                         pattern ? pattern->name() : "no pattern");
            }
        }
#endif
        const uint32_t tail = s_fq_tail.load(std::memory_order_relaxed);
        const uint32_t head = s_fq_head.load(std::memory_order_acquire);
        if (head == tail) {
            if (drawn_at && esp_timer_get_time() - drawn_at > RENDER_IDLE_US) {
                drawn_at = 0;
                if (pattern) pattern->reset();
                std::memset(pixels, 0, sizeof(pixels));
                show(pixels);
                bump(s_idle_dark);
            }
#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER

            if (esp_timer_get_time() - marker_at > MARKER_IDLE_US) {
                marker_idle = true;
                flash_at = 0;
                lower_at = 0;
                marker_write(marker_idle_level());
            }
#endif
            {
                int64_t nap_ms = RENDER_NAP_MS;
#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER

                nap_ms = marker_clamp_nap(nap_ms, flash_at, lower_at);
#endif
                vTaskDelay(pdMS_TO_TICKS(nap_ms));
            }
            continue;
        }

        const int64_t due = s_fq[tail % FRAME_RING].due_us;
        int64_t wait = 0;
        if (due > 0) {
            const auto to_local = s_to_local.load(std::memory_order_relaxed);
            wait = (to_local ? to_local(due) : due) - esp_timer_get_time();
        }
        if (wait > RENDER_SLACK_US) {

            int64_t nap = wait / 1000 < RENDER_NAP_MS ? wait / 1000 : RENDER_NAP_MS;
#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER

            nap = marker_clamp_nap(nap, flash_at, lower_at);
#endif
            const int64_t asked = nap ? nap : 1;
            const int64_t before = esp_timer_get_time();
            vTaskDelay(pdMS_TO_TICKS(asked));

            const int64_t over = (esp_timer_get_time() - before) - asked * 1000;
            bump(s_wake_n);
            if (over > 0) {
                bump(s_wake_sum, static_cast<uint32_t>(over));
                note_max(s_wake_max, static_cast<uint32_t>(over));
            }
            continue;
        }
        if (wait < -RENDER_LATE_US) {
            bump(s_late);
        }

        const int64_t t0 = esp_timer_get_time();
        df::Frame f = s_fq[tail % FRAME_RING];

        f.mag = nullptr;
        s_fq_tail.store(tail + 1, std::memory_order_release);

        {

            const int64_t hop_us =
                (int64_t)HOP_N * 1000000LL /
                (int64_t)s_rate.load(std::memory_order_relaxed);
            s_latch.take(f.due_us, hop_us, f.ml);
        }

#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER

        marker_at = t0;
        if (marker_idle) {

            marker_idle = false;
            lower_at = 0;
            marker_write(LED_MARKER_OFF);
        }

        if (f.due_us > 0) {
            const int64_t next_sec = f.due_us / LED_MARKER_PERIOD_US + 1;

            if (next_sec > last_flash_sec || flash_at) {
                last_flash_sec = next_sec;
                const int64_t at = next_sec * LED_MARKER_PERIOD_US;
                const auto to_local = s_to_local.load(std::memory_order_relaxed);
                flash_at = to_local ? to_local(at) : at;
            }
        }
#endif

        if (pattern) {

            const int64_t t_pat = esp_timer_get_time();
            pattern->render(f, pixels, LED_COUNT);
            const int64_t t_show = esp_timer_get_time();
            show(pixels);
            const int64_t t_end = esp_timer_get_time();
            bump(s_pattern_sum, static_cast<uint32_t>(t_show - t_pat));
            note_max(s_pattern_max, static_cast<uint32_t>(t_show - t_pat));
            bump(s_show_sum, static_cast<uint32_t>(t_end - t_show));
            note_max(s_show_max, static_cast<uint32_t>(t_end - t_show));
        }

        drawn_at = esp_timer_get_time();
        const uint32_t took = static_cast<uint32_t>(drawn_at - t0);
        s_render_sum.fetch_add(took, std::memory_order_relaxed);
        bump(s_render_n);
        note_max(s_render_max, took);
    }
}

}

void visualiser_marker_busy(bool on)
{
#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER
    if (on) {
        if (s_marker_busy_timer) {
            return;
        }
        gpio_config_t m = {};
        m.pin_bit_mask = 1ULL << CONFIG_DANCEFLOOR_LED_MARKER_GPIO;
        m.mode = GPIO_MODE_OUTPUT;
        if (gpio_config(&m) != ESP_OK) {
            return;
        }
        esp_timer_create_args_t a = {};
        a.callback = marker_busy_cb;
        a.name = "marker-busy";
        if (esp_timer_create(&a, &s_marker_busy_timer) != ESP_OK) {
            s_marker_busy_timer = nullptr;
            return;
        }
        s_marker_busy_level = LED_MARKER_OFF;
        esp_timer_start_periodic(s_marker_busy_timer, MARKER_BUSY_TOGGLE_US);
    } else {
        if (!s_marker_busy_timer) {
            return;
        }
        esp_timer_stop(s_marker_busy_timer);
        esp_timer_delete(s_marker_busy_timer);
        s_marker_busy_timer = nullptr;

        s_marker_busy_level = LED_MARKER_OFF;
        gpio_set_level(static_cast<gpio_num_t>(CONFIG_DANCEFLOOR_LED_MARKER_GPIO),
                       LED_MARKER_OFF);
    }
#else
    (void)on;
#endif
}

void visualiser_marker_set_link(bool up)
{
#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER
    s_marker_link.store(up, std::memory_order_relaxed);
#else

    (void)up;
#endif
}

void visualiser_set_clock(int64_t (*master_to_local)(int64_t))
{
    s_to_local.store(master_to_local, std::memory_order_relaxed);
}

void visualiser_set_publish(void (*publish)(const vis_frame_t *))
{
    s_publish.store(publish, std::memory_order_relaxed);
}

void visualiser_submit_frame(const vis_frame_t *f)
{
#if DF_TAKES_REMOTE_FRAMES
    if (!f) {
        return;
    }

    {
        static int64_t prev_index = -1, prev_due = 0;
        const int64_t want = (int64_t)HOP_N * 1000000 /
                             s_rate.load(std::memory_order_relaxed);
        if (f->index == prev_index + 1 && want > 0) {
            const int64_t got = f->due_us - prev_due;
            const int64_t err = got > want ? got - want : want - got;
            if (err > want / 8) {
                static bool told = false;
                if (!told) {
                    told = true;
                    ESP_LOGE(TAG, "sender's hop is %lld us, this build expects %lld us "
                                  "(hop %d) -- frames will be queued for the wrong "
                                  "depth; set DANCEFLOOR_LED_HOP to match the hub",
                             (long long)got, (long long)want, HOP_N);
                }
            }
        }
        prev_index = f->index;
        prev_due   = f->due_us;
    }

    {
        static uint32_t seen_rate = 0;
        const uint32_t rate = s_rate.load(std::memory_order_relaxed);
        if (rate != seen_rate) {
            seen_rate = rate;
            s_remote_detect.init();
        }
    }

    df::Frame local;
    from_wire(f, local);

    s_remote_detect.process(local.band, local.due_us, &local);

    if (enqueue(local)) {
        bump(s_frames);
        if (local.onset) bump(s_onsets);
        if (local.boom) bump(s_booms);
    }
#else

    (void)f;
#endif
}

int visualiser_hop(void)
{

    return HOP_N;
}

const char *visualiser_source_name(void)
{
#if CONFIG_DANCEFLOOR_LED_SOURCE_REMOTE
    return "remote";
#else
    return "local";
#endif
}

void visualiser_flush(void)
{

    s_fq_flush.fetch_add(1, std::memory_order_release);
}

void visualiser_feed(const uint8_t *pcm, uint32_t len, int64_t due_master_us)
{

    if (!pcm_stream) {
        return;
    }

    const int64_t rate = s_rate.load(std::memory_order_relaxed);

    if (due_master_us > 0 && s_ref_valid && s_skip_frames <= 0 && !s_mark_align_point &&
        !s_align_pending.load(std::memory_order_relaxed)) {
        const int64_t frames = static_cast<int64_t>(s_sent_total - s_ref_byte) / FRAME_BYTES;
        const int64_t drift = due_master_us - (s_ref_due + frames * 1000000LL / rate);
        if (drift > ALIGN_DRIFT_US || drift < -ALIGN_DRIFT_US) {
            s_last_drift_us.store(static_cast<int32_t>(drift), std::memory_order_relaxed);
            bump(s_drifts);
            s_align_pending.store(true, std::memory_order_relaxed);
        }
    }

    if (due_master_us > 0 && s_align_pending.exchange(false, std::memory_order_relaxed)) {
        const int64_t idx = (due_master_us * rate) / 1000000;
        const int32_t into_hop = static_cast<int32_t>(idx % HOP_N);
        s_skip_frames = into_hop ? (HOP_N - into_hop) : 0;
        s_pending_block_index = (idx + s_skip_frames) / HOP_N;
        s_mark_align_point = true;
        bump(s_aligns);
    }
    if (s_skip_frames > 0) {
        const uint32_t have = len / FRAME_BYTES;
        const uint32_t drop = s_skip_frames < static_cast<int32_t>(have)
                              ? static_cast<uint32_t>(s_skip_frames) : have;
        s_skip_frames -= static_cast<int32_t>(drop);
        pcm += drop * FRAME_BYTES;
        len -= drop * FRAME_BYTES;
        if (len == 0) {
            return;
        }
    }

    if (s_mark_align_point) {
        s_align_block_index.store(s_pending_block_index, std::memory_order_relaxed);
        s_align_at_byte.store(s_sent_total, std::memory_order_relaxed);
        s_align_gen.fetch_add(1, std::memory_order_release);

        s_ref_due = s_pending_block_index * HOP_N * 1000000LL / rate;
        s_ref_byte = s_sent_total;
        s_ref_valid = true;
        s_mark_align_point = false;
    }

    const size_t sent = xStreamBufferSend(pcm_stream, pcm, len, 0);
    s_sent_total += sent;
    if (sent < len) {
        bump(s_dropped, len - sent);
        s_align_pending.store(true, std::memory_order_relaxed);
    }
}

void visualiser_realign(void)
{

    s_align_pending.store(true, std::memory_order_relaxed);
}

void visualiser_set_rate(uint32_t hz)
{

    if (hz < 8000 || hz > 192000) {
        ESP_LOGE(TAG, "ignoring a stream rate of %" PRIu32 " Hz -- not a sample rate", hz);
        return;
    }

    if (s_rate.exchange(hz, std::memory_order_relaxed) == hz) {
        return;
    }

    s_align_pending.store(true, std::memory_order_relaxed);
    ESP_LOGW(TAG, "stream rate is %" PRIu32 " Hz", hz);
}

void visualiser_set_pattern(const char *name)
{
    df::Pattern *p = df::pattern_by_name(name);
    if (!p) {
        ESP_LOGW(TAG, "no pattern named \"%s\"", name ? name : "(null)");
        return;
    }
    p->reset();
    pattern = p;
    ESP_LOGI(TAG, "pattern: %s", p->name());
}

void visualiser_start(void)
{
#if DF_ANALYSES_AUDIO

    const size_t largest_before =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    const char *where = "internal";
#if CONFIG_SPIRAM

    pcm_stream = xStreamBufferCreateWithCaps(STREAM_BYTES, HOP_BYTES,
                                             MALLOC_CAP_SPIRAM);
    if (pcm_stream) {
        where = "PSRAM";
    } else {

        ESP_LOGW(TAG, "analysis stream: PSRAM refused %d bytes, falling back to "
                      "internal -- ESP_WIFI_STATIC_TX_BUFFER_NUM may now be too "
                      "high for this board", STREAM_BYTES);
        pcm_stream = xStreamBufferCreate(STREAM_BYTES, HOP_BYTES);
    }
#else
    pcm_stream = xStreamBufferCreate(STREAM_BYTES, HOP_BYTES);
#endif

    if (pcm_stream) {
        ESP_LOGI(TAG, "analysis stream: %d bytes in %s, largest internal block "
                      "%u -> %u -- the internal figure is the headroom for "
                      "ESP_WIFI_STATIC_TX_BUFFER_NUM, ~1.6 kB a buffer",
                 STREAM_BYTES, where, (unsigned)largest_before,
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    } else {
        ESP_LOGE(TAG, "analysis stream: %d bytes REFUSED -- largest internal "
                      "block was only %u. This unit will not draw. Lower "
                      "ESP_WIFI_STATIC_TX_BUFFER_NUM, or move this allocation "
                      "ahead of esp_wifi_init().",
                 STREAM_BYTES, (unsigned)largest_before);
    }
#endif

#if CONFIG_DANCEFLOOR_ENABLE_LED_MARKER
    {
        gpio_config_t m = {};
        m.pin_bit_mask = 1ULL << CONFIG_DANCEFLOOR_LED_MARKER_GPIO;
        m.mode = GPIO_MODE_OUTPUT;
        ESP_ERROR_CHECK(gpio_config(&m));

        gpio_set_level(static_cast<gpio_num_t>(CONFIG_DANCEFLOOR_LED_MARKER_GPIO),
                       LED_MARKER_OFF);
        ESP_LOGW(TAG, "LED marker on GPIO %d, active %s -- dark until joined, "
                      "solid when joined and idle, one flash per master-clock "
                      "second while playing; nothing corrects on it",
                 CONFIG_DANCEFLOOR_LED_MARKER_GPIO,
                 LED_MARKER_ON == 0 ? "low" : "high");
    }
#endif

#if DF_ANALYSES_AUDIO

    analysis.init(static_cast<int>(s_rate.load(std::memory_order_relaxed)));
#endif

    {
        const int rate = static_cast<int>(s_rate.load(std::memory_order_relaxed));

        const int frames_per_s = rate / HOP_N;
        for (int i = 0; i < df::ML_SLOTS; i++) {
            s_latch.set_latched(i, true);
            df::Analyser *a = df::analyser_at(i);
            if (!a) {
                continue;
            }
            const df::AnalyserSpec &sp = a->spec();
#if DF_RUNS_ANALYSERS
            if (sp.lane == df::Lane::Fast) {
                if (a->init(frames_per_s)) {
                    s_latch.set_latched(i, false);
                } else {
                    ESP_LOGE(TAG, "analyser \"%s\" refused to start -- slot %d idle",
                             sp.name, i);
                    continue;
                }
            }
#endif
            ESP_LOGI(TAG, "analyser %d: \"%s\" model %u | %s lane | %d bins @ "
                          "%d frames/s | shown %lld us late | %s",
                     i, sp.name, (unsigned)sp.model_id,
                     sp.lane == df::Lane::Fast ? "fast" : "slow",
                     df::SPEC_BINS, frames_per_s,
                     (long long)sp.present_delay_us,
                     DF_RUNS_ANALYSERS
                         ? (sp.lane == df::Lane::Fast ? "computed here, in the frame"
                                                      : "computed here, through the latch")
                         : "given to this unit");
        }

#if DF_RUNS_ANALYSERS

        df::ml_lane_start(&s_latch, frames_per_s);
#endif
    }

    pattern = df::pattern_at(0);
    if (df::Pattern *p = df::pattern_by_name(CONFIG_DANCEFLOOR_LED_PATTERN)) {
        pattern = p;
    } else {
        ESP_LOGW(TAG, "no pattern named \"%s\" -- falling back to \"%s\"",
                 CONFIG_DANCEFLOOR_LED_PATTERN, pattern ? pattern->name() : "none");
    }

    strip.emplace(static_cast<gpio_num_t>(CONFIG_DANCEFLOOR_LED_GPIO), LED_COUNT,
                  STRIP_TYPE, LedStrip::Backend::SPI);
    strip->clear();
    strip->show();

#if DF_ANALYSES_AUDIO

    if (!pcm_stream) {
        ESP_LOGE(TAG, "TASK \"vis\" NOT STARTED -- no analysis stream to read");
    } else if (xTaskCreatePinnedToCore(visualiser_task, "vis", 4096, nullptr, 4, nullptr, 1) != pdPASS) {
        ESP_LOGE(TAG, "TASK \"vis\" FAILED TO START -- no analysis, the strip "
                      "will not react to audio");
    }
#endif
    if (xTaskCreatePinnedToCore(render_task, "vis-draw", 3072, nullptr, 5, nullptr, 1) != pdPASS) {
        ESP_LOGE(TAG, "TASK \"vis-draw\" FAILED TO START -- the strip will stay dark");
    }

    ESP_LOGW(TAG, "started: %lu LEDs on GPIO %d at %d%% brightness, pattern %s, "
                  "frames %s | window %d, hop %d (%.1f frames/s at %" PRIu32 " Hz)",
             LED_COUNT, CONFIG_DANCEFLOOR_LED_GPIO, CONFIG_DANCEFLOOR_LED_BRIGHTNESS,
             pattern ? pattern->name() : "none", visualiser_source_name(),
             FFT_N, HOP_N,
             (double)s_rate.load(std::memory_order_relaxed) / HOP_N,
             s_rate.load(std::memory_order_relaxed));
}
