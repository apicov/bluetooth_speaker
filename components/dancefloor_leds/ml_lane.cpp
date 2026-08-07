/*
 * The slow lane. See ml_lane.hpp for what it is and why it is a separate task.
 */
#include "ml_lane.hpp"

#include <atomic>
#include <cinttypes>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "analyser.hpp"
#include "ml_window.hpp"

namespace df {
namespace {

constexpr const char *TAG = "mlan";

Analyser   *s_slow = nullptr;
int         s_slot = -1;
ResultLatch *s_latch = nullptr;

StreamBufferHandle_t s_stream = nullptr;

/*
 * How much decimated audio the lane will hold.
 *
 * It only has to cover the gap between the analysis task producing a hop and
 * this task getting round to it, which is bounded by one inference. 400 ms at
 * 16 kHz mono is 12.8 kB -- generous against a model that must finish inside a
 * presentation lead of 200 ms to be useful at all, and small enough to fit the
 * satellite that has ~52 kB free.
 *
 * Sized in TIME rather than samples, so a different model rate does not quietly
 * change how much slack there is.
 */
constexpr int STREAM_MS = 400;

/*
 * The origin handshake, in the shape visualiser.cpp already uses for its block
 * grid: publish the new origin and the byte count it starts at, then bump a
 * generation with release ordering. The reader adopts it by generation, not by
 * byte index -- visualiser.cpp records why a byte-index reader mislabelled half
 * its blocks.
 */
std::atomic<uint32_t> s_gen{0};
std::atomic<int64_t>  s_pending_origin{0};
std::atomic<uint32_t> s_pending_at_byte{0};
uint32_t s_fed_total = 0;          /* feeder-side only */

std::atomic<uint32_t> s_results{0}, s_dropped{0}, s_restarts{0};
std::atomic<uint32_t> s_cost_sum{0}, s_cost_n{0}, s_cost_max{0};

void bump(std::atomic<uint32_t> &c, uint32_t n = 1)
{
    c.fetch_add(n, std::memory_order_relaxed);
}

/*
 * One window: run the analyser, time it, and publish anything it produced.
 *
 * show_at_us, analyser and model_id come from the spec rather than from the
 * analyser -- an analyser that could set them could date its own results, which
 * is the one thing the presentation-delay rule forbids.
 */
void feed(SlowWindow &grid, const AnalyserSpec &sp, const int16_t *in, int n)
{
    grid.push(in, n, [&sp](const int16_t *window, int64_t index, int64_t due_us) {
        Result r{};
        const int64_t t0 = esp_timer_get_time();
        const bool produced = s_slow->process(window, index, due_us, &r);
        const int64_t took = esp_timer_get_time() - t0;

        bump(s_cost_sum, (uint32_t)took);
        bump(s_cost_n);
        uint32_t prev = s_cost_max.load(std::memory_order_relaxed);
        while (took > (int64_t)prev &&
               !s_cost_max.compare_exchange_weak(prev, (uint32_t)took,
                                                 std::memory_order_relaxed)) {
        }

        if (produced) {
            r.analyser   = (uint8_t)s_slot;
            r.model_id   = sp.model_id;
            r.show_at_us = due_us + sp.present_delay_us;
            s_latch->publish(s_slot, r);
            bump(s_results);
        }
    });
}

void ml_task(void *arg)
{
    (void)arg;

    const AnalyserSpec &sp = s_slow->spec();

    /* The grid lives in ml_window.hpp, so pattern_lab cuts the same windows.
     * All that is here is getting bytes to it and results away from it. */
    static SlowWindow grid;
    grid.configure(sp.window_n, sp.hop_n, sp.rate_hz);

    static int16_t chunk[256];
    uint32_t recv_total = 0;        /* bytes taken from the stream */
    uint32_t seen_gen = 0;

    while (true) {
        const size_t got = xStreamBufferReceive(s_stream, chunk, sizeof(chunk),
                                                pdMS_TO_TICKS(200));

        /*
         * A new timeline, adopted by GENERATION rather than by byte index --
         * visualiser.cpp records why a byte-index reader mislabelled half its
         * blocks. Everything before the published byte is from the old timeline
         * and is dropped, this partial window included.
         */
        const uint32_t gen = s_gen.load(std::memory_order_acquire);
        if (gen != seen_gen) {
            const uint32_t at = s_pending_at_byte.load(std::memory_order_relaxed);
            const int32_t ahead = (int32_t)(recv_total + (uint32_t)got - at);
            if (ahead < 0) {
                recv_total += (uint32_t)got;
                continue;               /* still draining the old timeline */
            }
            seen_gen = gen;
            s_slow->reset();
            grid.restart(s_pending_origin.load(std::memory_order_relaxed));
            /* Keep only what arrived at or after the new origin. */
            const size_t keep = (size_t)ahead < got ? (size_t)ahead : got;
            std::memmove(chunk, (uint8_t *)chunk + (got - keep), keep);
            recv_total += (uint32_t)got;
            feed(grid, sp, chunk, (int)(keep / sizeof(int16_t)));
            continue;
        }

        recv_total += (uint32_t)got;
        feed(grid, sp, chunk, (int)(got / sizeof(int16_t)));
    }
}

}  // namespace

int ml_lane_rate()
{
    for (int i = 0; i < ML_SLOTS; i++) {
        Analyser *a = analyser_at(i);
        if (a && a->spec().lane == Lane::Slow) {
            return a->spec().rate_hz;
        }
    }
    return 0;
}

int ml_lane_slot()
{
    for (int i = 0; i < ML_SLOTS; i++) {
        Analyser *a = analyser_at(i);
        if (a && a->spec().lane == Lane::Slow) {
            return i;
        }
    }
    return -1;
}

void ml_lane_start(ResultLatch *latch, int stream_rate_hz)
{
    s_slot = ml_lane_slot();
    if (s_slot < 0) {
        return;                     /* no slow analyser in this build */
    }
    s_slow  = analyser_at(s_slot);
    s_latch = latch;

    const AnalyserSpec &sp = s_slow->spec();

    /* One only -- see the note in ml_lane.hpp. Said out loud, because a second
     * one silently never running is the kind of absence that costs an evening. */
    for (int i = s_slot + 1; i < ML_SLOTS; i++) {
        Analyser *a = analyser_at(i);
        if (a && a->spec().lane == Lane::Slow) {
            ESP_LOGE(TAG, "analyser \"%s\" is also slow -- this lane runs one, "
                          "so it will NOT run", a->spec().name);
        }
    }

    if (sp.rate_hz <= 0) {
        ESP_LOGE(TAG, "analyser \"%s\" is slow but asks for the stream rate -- "
                      "not started", sp.name);
        s_slot = -1;
        return;
    }
    if (!s_slow->init(stream_rate_hz)) {
        ESP_LOGE(TAG, "analyser \"%s\" refused to start -- lane idle", sp.name);
        s_slot = -1;
        return;
    }

    const size_t bytes = (size_t)sp.rate_hz * STREAM_MS / 1000 * sizeof(int16_t);
    s_stream = xStreamBufferCreate(bytes, (size_t)sp.hop_n * sizeof(int16_t));
    if (!s_stream) {
        ESP_LOGE(TAG, "no memory for a %u byte lane buffer -- \"%s\" will not run",
                 (unsigned)bytes, sp.name);
        s_slot = -1;
        return;
    }

    ESP_LOGI(TAG, "slot %d: \"%s\" model %u | %d Hz mono from %d | window %d, hop %d "
                  "| reports every %d ms | shown %lld us late | buffer %u B",
             s_slot, sp.name, (unsigned)sp.model_id, sp.rate_hz, stream_rate_hz,
             sp.window_n, sp.hop_n, sp.hop_n * 1000 / sp.rate_hz,
             (long long)sp.present_delay_us, (unsigned)bytes);

    /*
     * Core 0, priority 3.
     *
     * Core 1 carries playback (8), the render task (5) and the analysis task
     * (4), and the whole reason this lane exists is that an inference does not
     * fit between two of the analysis task's frames. Putting it on the same core
     * would trade a late frame for late audio, which is the one exchange this
     * project never makes.
     *
     * Core 0 carries lwIP at 18 on the hub, which is bursty and far above this.
     * 4 kB of stack because a model's working set is in its arena, not here.
     */
    xTaskCreatePinnedToCore(ml_task, "mlan", 4096, nullptr, 3, nullptr, 0);
}

bool ml_lane_feed(const int16_t *mono, int n)
{
    if (!s_stream || n <= 0) {
        return true;
    }
    const size_t want = (size_t)n * sizeof(int16_t);
    const size_t sent = xStreamBufferSend(s_stream, mono, want, 0);
    s_fed_total += (uint32_t)sent;
    if (sent < want) {
        bump(s_dropped, (uint32_t)((want - sent) / sizeof(int16_t)));
        return false;
    }
    return true;
}

void ml_lane_restart(int64_t origin_us)
{
    if (!s_stream) {
        return;
    }
    s_pending_origin.store(origin_us, std::memory_order_relaxed);
    s_pending_at_byte.store(s_fed_total, std::memory_order_relaxed);
    /* Release last, against the acquire in the task: both values above must be
     * visible before the generation that publishes them changes. */
    s_gen.fetch_add(1, std::memory_order_release);
    bump(s_restarts);
}

MlLaneStats ml_lane_take_stats()
{
    const uint32_t n = s_cost_n.exchange(0, std::memory_order_relaxed);
    const uint32_t sum = s_cost_sum.exchange(0, std::memory_order_relaxed);
    MlLaneStats st;
    st.results      = s_results.exchange(0, std::memory_order_relaxed);
    st.dropped      = s_dropped.exchange(0, std::memory_order_relaxed);
    st.restarts     = s_restarts.exchange(0, std::memory_order_relaxed);
    st.cost_mean_us = n ? sum / n : 0;
    st.cost_max_us  = s_cost_max.exchange(0, std::memory_order_relaxed);
    return st;
}

}  // namespace df
