/**
 * @file ml_lane.cpp
 * @brief The slow analyser's task. ml_lane.hpp has the contract and the reason
 *        this lane exists at all.
 *
 * One analyser, one queue, one task. The queue is what decouples the analysis
 * task from a model that may take longer than a frame period: the analysis
 * task offers frames and never waits, and a lane that cannot keep up DROPS
 * them and counts it rather than stalling the audio path.
 *
 * The task is pinned to the core the audio and the strip are NOT on, at a
 * priority below both, which is the other half of the same guarantee.
 */
#include "ml_lane.hpp"

#include <atomic>
#include <cinttypes>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "analyser.hpp"

namespace df {
namespace {

/** @brief Log tag. */
constexpr const char *TAG = "mlan";

/** @brief The one slow analyser, or null if there is none. */
Analyser    *s_slow = nullptr;
/** @brief Its slot, or -1 if the lane is idle -- which is also what a failed
 *         start leaves behind, so nothing feeds a lane that did not come up. */
int          s_slot = -1;
/** @brief Where results are published for the render stage. */
ResultLatch *s_latch = nullptr;

/** @brief One frame, by value, so the analysis task's buffer is not borrowed
 *         across a task boundary. */
struct LaneFrame {
    uint8_t  spec[SPEC_BINS];   /**< The quantised spectrum. */
    int64_t  index;             /**< Its place on the shared grid. */
    int64_t  due_us;            /**< When its audio is heard. */
    /** @brief Which generation of the timeline it belongs to; see s_gen. */
    uint32_t gen;
};

/** @brief Frames the queue holds. Deep enough to absorb a burst from a decoder
 *         lump without the analysis task ever waiting, and shallow enough that
 *         a lane falling behind is noticed as drops rather than as latency. */
constexpr int QUEUE_FRAMES = 32;

/** @brief Analysis task to lane task. */
QueueHandle_t s_q = nullptr;

/**
 * @brief Timeline generation, bumped by ml_lane_restart().
 *
 * A restart cannot simply drain the queue: the analysis task is still filling
 * it, so anything drained here would be replaced by frames already in flight
 * from before the restart. Stamping each frame with the generation it was
 * queued in lets the lane task discard the stale ones as it reaches them, and
 * reset the analyser exactly once, at the first frame of the new generation.
 */
std::atomic<uint32_t> s_gen{0};

/** @brief Results published since the last ml_lane_take_stats(). */
std::atomic<uint32_t> s_results{0};
/** @brief Frames the queue could not take. */
std::atomic<uint32_t> s_dropped{0};
/** @brief ml_lane_restart() calls. */
std::atomic<uint32_t> s_restarts{0};
/** @brief Inference time, summed, for the mean. */
std::atomic<uint32_t> s_cost_sum{0};
/** @brief ...and how many went into it. */
std::atomic<uint32_t> s_cost_n{0};
/** @brief The worst inference this window, which is the figure a declared
 *         present_delay_us has to cover. */
std::atomic<uint32_t> s_cost_max{0};

/** @brief Add to a counter, relaxed -- these are read once a window and never
 *         ordered against anything.
 *  @param c  The counter.
 *  @param n  How much. */
void bump(std::atomic<uint32_t> &c, uint32_t n = 1)
{
    c.fetch_add(n, std::memory_order_relaxed);
}

/**
 * @brief Run the analyser over one frame, time it, and publish anything it
 *        produced.
 * @param sp  The analyser's spec, which dates the result.
 * @param f   The frame.
 */
void run_one(const AnalyserSpec &sp, const LaneFrame &f)
{
    Result r{};
    const int64_t t0 = esp_timer_get_time();
    const bool produced = s_slow->process(f.spec, f.index, f.due_us, &r);
    const int64_t took = esp_timer_get_time() - t0;

    bump(s_cost_sum, (uint32_t)took);
    bump(s_cost_n);
    /* Compare-and-swap rather than a plain compare-and-store: the stats reader
     * clears this concurrently, and a lost update here would under-report the
     * one figure a presentation delay is set from. */
    uint32_t prev = s_cost_max.load(std::memory_order_relaxed);
    while (took > (int64_t)prev &&
           !s_cost_max.compare_exchange_weak(prev, (uint32_t)took,
                                             std::memory_order_relaxed)) {
    }

    if (!produced) {
        return;
    }

    /* Dated from the SPEC, exactly as run_fast_lane() does it -- an analyser
     * that could set these could date its own results. */
    r.analyser   = (uint8_t)s_slot;
    r.model_id   = sp.model_id;
    r.show_at_us = f.due_us + sp.present_delay_us;
    s_latch->publish(s_slot, r);
    bump(s_results);

    /* Logged on CHANGE only. A result per context at this cadence would be a
     * steady drip of identical lines; what is worth seeing is when the answer
     * moves. */
    static uint8_t last_label = 0xFF;
    static bool    have_last;
    if (!have_last || r.label[0] != last_label) {
        have_last = true;
        last_label = r.label[0];
        const char *nm = s_slow->label_name(r.label[0]);
        if (nm) {
            ESP_LOGI(TAG, "%s: %s (%u) at %lld us",
                     sp.name, nm, (unsigned)r.score[0], (long long)r.show_at_us);
        } else {
            ESP_LOGI(TAG, "%s: label %u (%u) at %lld us",
                     sp.name, (unsigned)r.label[0], (unsigned)r.score[0],
                     (long long)r.show_at_us);
        }
    }
}

/**
 * @brief The lane task: take frames, discard the stale ones, run the rest.
 * @param arg  Unused; the FreeRTOS task signature.
 */
void ml_task(void *arg)
{
    (void)arg;

    const AnalyserSpec &sp = s_slow->spec();
    uint32_t seen_gen = s_gen.load(std::memory_order_acquire);

    LaneFrame f;
    while (true) {
        /* A timeout rather than an infinite wait, so the task is
         * unconditionally alive and a stack high-water reading of it means
         * something even on a silent floor. */
        if (xQueueReceive(s_q, &f, pdMS_TO_TICKS(200)) != pdTRUE) {
            continue;
        }

        /*
         * Signed difference, so the comparison survives the counter wrapping.
         * A NEWER generation means the timeline restarted: adopt it and reset
         * the analyser once. An OLDER one is a frame queued before a restart
         * and is dropped -- it describes audio that no longer precedes what
         * follows it.
         */
        if (f.gen != seen_gen) {
            if ((int32_t)(f.gen - seen_gen) > 0) {
                seen_gen = f.gen;
                s_slow->reset();
            } else {
                continue;
            }
        }

        run_one(sp, f);
    }
}

}

/* Declared in ml_lane.hpp, like the four below it. */
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

void ml_lane_start(ResultLatch *latch, int frames_per_s)
{
    s_slot = ml_lane_slot();
    if (s_slot < 0) {
        return;
    }
    s_slow  = analyser_at(s_slot);
    s_latch = latch;

    const AnalyserSpec &sp = s_slow->spec();

    /* One slow analyser per lane, and the second one is named rather than
     * silently ignored: a model that never reports looks exactly like one that
     * is running and finding nothing. */
    for (int i = s_slot + 1; i < ML_SLOTS; i++) {
        Analyser *a = analyser_at(i);
        if (a && a->spec().lane == Lane::Slow) {
            ESP_LOGE(TAG, "analyser \"%s\" is also slow -- this lane runs one, "
                          "so it will NOT run", a->spec().name);
        }
    }

    if (!s_slow->init(frames_per_s)) {
        ESP_LOGE(TAG, "analyser \"%s\" refused to start -- lane idle", sp.name);
        s_slot = -1;
        return;
    }

    s_q = xQueueCreate(QUEUE_FRAMES, sizeof(LaneFrame));
    if (!s_q) {
        ESP_LOGE(TAG, "no memory for a %u frame lane queue -- \"%s\" will not run",
                 (unsigned)QUEUE_FRAMES, sp.name);
        s_slot = -1;
        return;
    }

    ESP_LOGI(TAG, "slot %d: \"%s\" model %u | %d frames/s of %d bins "
                  "| shown %lld us late | queue %u frames (%u B)",
             s_slot, sp.name, (unsigned)sp.model_id, frames_per_s, SPEC_BINS,
             (long long)sp.present_delay_us,
             (unsigned)QUEUE_FRAMES, (unsigned)(QUEUE_FRAMES * sizeof(LaneFrame)));

    /* Core 0, away from the audio and the strip, at a priority below both. */
    if (xTaskCreatePinnedToCore(ml_task, "mlan", 4096, nullptr, 3, nullptr, 0) != pdPASS) {
        /* Checked, because without the task nothing drains the queue: every
         * frame would be dropped at ml_lane_feed() and the slot would report
         * nothing for the rest of the session. */
        ESP_LOGE(TAG, "TASK \"mlan\" FAILED TO START -- the slow lane will "
                      "report nothing");
    }
}

bool ml_lane_feed(const uint8_t (&spec)[SPEC_BINS], int64_t index, int64_t due_us)
{
    /* No lane is not a failure: a build with no slow analyser feeds nothing
     * and must not count a drop for it. */
    if (!s_q) {
        return true;
    }
    LaneFrame f;
    std::memcpy(f.spec, spec, sizeof(f.spec));
    f.index  = index;
    f.due_us = due_us;
    f.gen    = s_gen.load(std::memory_order_relaxed);

    if (xQueueSend(s_q, &f, 0) != pdTRUE) {
        bump(s_dropped);
        return false;
    }
    return true;
}

void ml_lane_restart()
{
    if (!s_q) {
        return;
    }
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

}
