/*
 * Firmware half of the visualiser: get aligned blocks of audio out of the
 * playback path, hand them to the shared pipeline, put the result on the strip.
 *
 * Everything that decides what the lights DO lives in analysis.cpp and
 * patterns.cpp, which have no platform dependencies and are driven identically
 * by tools/pattern_lab on a laptop. This file owns only the parts that cannot
 * be: the stream buffer, the block alignment, the task, and the strip.
 */
#include "visualiser.h"

#include <atomic>
#include <cinttypes>
#include <cstring>
#include <optional>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "analysis.hpp"
#include "patterns.hpp"
#include "led_strip_wrapper.hpp"

namespace {

constexpr const char *TAG = "vis";

using df::FFT_N;
using df::RATE;
using df::CHANNELS;

/* Four analysis frames of headroom. A short send loses audio AND breaks the
 * block alignment, so the buffer is sized to make that rare. */
constexpr int STREAM_BYTES = FFT_N * CHANNELS * (int)sizeof(int16_t) * 4;
constexpr uint32_t FRAME_BYTES = CHANNELS * sizeof(int16_t);

constexpr uint32_t LED_COUNT = CONFIG_DANCEFLOOR_LED_COUNT;

/* Matches LOG_PERIOD_S in sync_proto.h, which this component does not include
 * -- it is deliberately free of the audio protocol. Kept in step by hand; the
 * cost of drifting apart is a console that is noisier than intended, not a
 * fault. */
constexpr int64_t LED_LOG_PERIOD_US = 20 * 1000000LL;

/* Applied to the pattern's output on the way to the strip. A device concern,
 * not a pattern one -- patterns work in full range and this scales it. */
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
df::Analysis analysis;
df::Pattern *pattern = nullptr;
uint8_t pixels[LED_COUNT * 3];

/*
 * Block alignment.
 *
 * The detector chops the stream into fixed FFT_N blocks, and every unit must cut
 * them at the same positions or a transient near a boundary is split on one
 * board and centred on another -- the marginal onsets then fire on one strip and
 * not the other. Boundaries are derived from the instant audio is SCHEDULED to
 * be heard, which all units agree on, and never from a clock read here, which
 * they do not: they reach this code milliseconds apart.
 *
 * Both sides count bytes that actually passed through the buffer, so a drop
 * cannot make them disagree about position.
 */
/*
 * Atomic because three tasks raise it: the audio task on a short send, this
 * component's own task when it starves, and the drift servo after a retune.
 * Only the audio task lowers it, with an exchange, so a request raised while an
 * alignment is being computed is not lost.
 */
std::atomic<bool> s_align_pending{true};
bool     s_mark_align_point;
int32_t  s_skip_frames;
int64_t  s_pending_block_index;

std::atomic<uint32_t> s_align_at_byte;      /* byte index where aligned audio starts */
std::atomic<long long> s_align_block_index; /* block number of that first sample */
/*
 * Bumped on every publish, and what the analysis task actually watches.
 *
 * It used to watch s_align_at_byte, on the assumption that a new alignment
 * always carries a new byte index. It does not, and the case where it does not
 * is the common one: while this task is behind, the buffer is full and feeds are
 * rejected ENTIRELY, so s_sent_total does not move. Every rejected feed re-arms
 * the alignment, and each one then publishes the same byte index with a later
 * block index. The reader took the first and could not see any of the rest --
 * they compared equal to what it had already adopted -- so it went on labelling
 * from an origin two or three blocks stale.
 *
 * Boundaries survived that, which is why it went unnoticed for so long: the
 * discard arithmetic still lands on a multiple of FFT_N, so both units cut
 * identical windows and simply dated them differently. due_us is what carries
 * the date, and every time-driven pattern is built on it, so the strips ran the
 * same animation from different points of its cycle -- stepping further apart at
 * each burst of drops and never recovering. Modelled in test_align.c, where the
 * byte-index reader mislabels ~53% of blocks over a run and this one mislabels
 * none.
 *
 * The first publish was ignored too, for the duller reason that a byte count of
 * zero is indistinguishable from the initial value. A generation counter has
 * neither problem.
 */
std::atomic<uint32_t> s_align_gen;
uint32_t s_sent_total;                      /* feed side only */

/* Statistics, so a disagreement between two units is diagnosable rather than a
 * matter of opinion. Atomic: incremented by the audio task, read and cleared by
 * the analysis task. */
std::atomic<uint32_t> s_dropped;
std::atomic<uint32_t> s_aligns;
std::atomic<uint32_t> s_onsets;
std::atomic<uint32_t> s_frames;

uint32_t take(std::atomic<uint32_t> &c) { return c.exchange(0, std::memory_order_relaxed); }
void bump(std::atomic<uint32_t> &c, uint32_t n = 1) { c.fetch_add(n, std::memory_order_relaxed); }

void show(const uint8_t *rgb)
{
    for (uint32_t i = 0; i < LED_COUNT; i++) {
        strip->set(i, static_cast<uint8_t>(rgb[3 * i + 0] * BRIGHTNESS),
                      static_cast<uint8_t>(rgb[3 * i + 1] * BRIGHTNESS),
                      static_cast<uint8_t>(rgb[3 * i + 2] * BRIGHTNESS));
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

    while (true) {
        const size_t got = xStreamBufferReceive(pcm_stream,
                                                reinterpret_cast<uint8_t *>(raw) + filled,
                                                sizeof(raw) - filled, pdMS_TO_TICKS(100));
        filled += got;
        recv_total += got;

        /*
         * Has the feed re-aligned? Everything before its published byte index is
         * stale and must not contribute to a block, this partial block included.
         * Dropping only the feed side leaves the boundaries offset by whatever
         * is held here, and the re-alignment silently does nothing.
         */
        /* Acquire, against the release on the publishing side: the byte index
         * and the block index must both be the ones this generation stored. */
        const uint32_t gen = s_align_gen.load(std::memory_order_acquire);
        if (gen != seen_gen) {
            const uint32_t align_at = s_align_at_byte.load(std::memory_order_relaxed);
            const int32_t ahead = static_cast<int32_t>(recv_total - align_at);
            if (ahead < 0) {
                filled = 0;             /* still draining pre-alignment audio */
                continue;
            }
            seen_gen = gen;
            block_index = s_align_block_index.load(std::memory_order_relaxed);
            const size_t keep = static_cast<size_t>(ahead) < filled
                                ? static_cast<size_t>(ahead) : filled;
            std::memmove(raw, reinterpret_cast<uint8_t *>(raw) + (filled - keep), keep);
            filled = keep;
        }

        if (filled < sizeof(raw)) {
            if (got == 0 && !starved_shown) {
                /* Genuinely starved rather than mid-block. Go dark once instead
                 * of holding the last frame; bounded to one refresh per timeout,
                 * so it cannot become a spin. */
                starved_shown = true;
                if (pattern) pattern->reset();
                std::memset(pixels, 0, sizeof(pixels));
                show(pixels);
                /* the stream broke; re-align on return */
                s_align_pending.store(true, std::memory_order_relaxed);
            }
            continue;
        }
        filled = 0;
        starved_shown = false;

        /* due_us is derived from the block index, not read from a clock, so it
         * is the same value on every unit for this same audio. */
        const int64_t due_us = block_index * FFT_N * 1000000LL / RATE;
        const df::Frame &f = analysis.process(raw, block_index, due_us, 0);
        block_index++;

        bump(s_frames);
        if (f.onset) bump(s_onsets);

        if (pattern) {
            pattern->render(f, pixels, LED_COUNT);
            show(pixels);
        }

        /*
         * Quiet when nothing is wrong, immediate when something is.
         *
         * A healthy window reports the same ~215 frames and 0 dropped bytes
         * every time, and two units doing that every 5 s is most of a console.
         * A window that dropped audio, or re-aligned more than the one time a
         * splice or retune explains, is worth seeing at once -- those are the
         * two things that put the strips out of step with each other.
         */
        const int64_t now = esp_timer_get_time();
        const uint32_t dropped = s_dropped.load(std::memory_order_relaxed);
        const uint32_t aligns  = s_aligns.load(std::memory_order_relaxed);
        if (dropped || aligns > 1 || now - last_report_us >= LED_LOG_PERIOD_US) {
            ESP_LOGI(TAG, "frames %" PRIu32 " | onsets %" PRIu32
                          " | drop %" PRIu32 " B | aligns %" PRIu32 " | %s",
                     take(s_frames), take(s_onsets), take(s_dropped), take(s_aligns),
                     pattern ? pattern->name() : "no pattern");
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

    /* Exchange rather than test-then-clear: a request raised by another task
     * while this one is mid-alignment would otherwise be overwritten. */
    if (due_master_us > 0 && s_align_pending.exchange(false, std::memory_order_relaxed)) {
        const int64_t idx = (due_master_us * RATE) / 1000000;
        const int32_t into_block = static_cast<int32_t>(idx % FFT_N);
        s_skip_frames = into_block ? (FFT_N - into_block) : 0;
        s_pending_block_index = (idx + s_skip_frames) / FFT_N;
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

    /* Publish where aligned audio begins, and which block that is. The
     * generation goes last, and with release ordering: it is what the analysis
     * task watches, so the two values must already be visible when it changes. */
    if (s_mark_align_point) {
        s_align_block_index.store(s_pending_block_index, std::memory_order_relaxed);
        s_align_at_byte.store(s_sent_total, std::memory_order_relaxed);
        s_align_gen.fetch_add(1, std::memory_order_release);
        s_mark_align_point = false;
    }

    /* 0 ticks: never block the audio task. A short send means the analysis fell
     * behind and audio was lost, which both breaks the alignment and makes this
     * unit analyse different audio from its neighbours -- count it and re-align
     * rather than losing it silently. */
    const size_t sent = xStreamBufferSend(pcm_stream, pcm, len, 0);
    s_sent_total += sent;
    if (sent < len) {
        bump(s_dropped, len - sent);
        s_align_pending.store(true, std::memory_order_relaxed);
    }
}

void visualiser_realign(void)
{
    /* Callable from any task: the flag is atomic and the audio task clears it
     * with an exchange. The next feed carrying a scheduled instant re-derives
     * the origin from it. */
    s_align_pending.store(true, std::memory_order_relaxed);
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
    /* Trigger level is one full analysis frame: waking the task for a handful
     * of bytes at a time is pure overhead now that partial reads accumulate. */
    pcm_stream = xStreamBufferCreate(STREAM_BYTES, FFT_N * CHANNELS * sizeof(int16_t));
    assert(pcm_stream);

    analysis.init();

    /* Configured name if it resolves, first pattern otherwise. A typo should
     * cost a line in the log, not a dark floor. */
    pattern = df::pattern_at(0);
    if (df::Pattern *p = df::pattern_by_name(CONFIG_DANCEFLOOR_LED_PATTERN)) {
        pattern = p;
    } else {
        ESP_LOGW(TAG, "no pattern named \"%s\" -- falling back to \"%s\"",
                 CONFIG_DANCEFLOOR_LED_PATTERN, pattern ? pattern->name() : "none");
    }

    /*
     * SPI + DMA, not bit-banging and not RMT.
     *
     * Bit-banging disables interrupts for the whole strip update, starving the
     * I2S DMA. RMT does not survive continuous refresh on the ESP32 -- see
     * components/led_strip_wrapper/README.md for the failure and why SPI has no
     * equivalent.
     */
    strip.emplace(static_cast<gpio_num_t>(CONFIG_DANCEFLOOR_LED_GPIO), LED_COUNT,
                  STRIP_TYPE, LedStrip::Backend::SPI);
    strip->clear();
    strip->show();

    /* Core 1 alongside playback, at a lower priority than it: a dropped frame
     * here is invisible, dropped audio is not. */
    xTaskCreatePinnedToCore(visualiser_task, "vis", 4096, nullptr, 4, nullptr, 1);
    ESP_LOGI(TAG, "started: %lu LEDs on GPIO %d at %d%% brightness, pattern %s",
             LED_COUNT, CONFIG_DANCEFLOOR_LED_GPIO, CONFIG_DANCEFLOOR_LED_BRIGHTNESS,
             pattern ? pattern->name() : "none");
}
