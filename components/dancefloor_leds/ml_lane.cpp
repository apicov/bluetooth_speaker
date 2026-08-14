/*
 * The slow lane. See ml_lane.hpp for what it is and why it is a separate task.
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

constexpr const char *TAG = "mlan";

Analyser    *s_slow = nullptr;
int          s_slot = -1;
ResultLatch *s_latch = nullptr;

/*
 * One frame, as the lane carries it.
 *
 * `gen` is what makes a restart safe without any byte arithmetic. The feeder
 * stamps the current generation; the task drops anything stamped with an older
 * one and resets the analyser when it sees the number move. The audio lane this
 * replaces had to publish an origin and a byte count and compare against a
 * running total, because its grid was derived by counting -- visualiser.cpp
 * records how a byte-index reader mislabelled half its blocks. A frame carries
 * its own index and due_us, so none of that is needed.
 */
struct LaneFrame {
    uint8_t  spec[SPEC_BINS];
    int64_t  index;
    int64_t  due_us;
    uint32_t gen;
};

/*
 * How many frames of slack between the task producing one and this task getting
 * round to it, bounded by one inference.
 *
 * 32 frames is 372 ms at hop 512 and 44.1 kHz -- generous against a model that
 * must finish inside a presentation lead of 200 ms to be useful at all. At 88
 * bytes each that is 2.8 kB, against the 12.8 kB the decimated-audio buffer
 * cost for the same 400 ms of slack. That reduction is most of why this lane
 * now fits a unit which is also holding an 80 kB audio ring.
 */
constexpr int QUEUE_FRAMES = 32;

QueueHandle_t s_q = nullptr;

std::atomic<uint32_t> s_gen{0};

std::atomic<uint32_t> s_results{0}, s_dropped{0}, s_restarts{0};
std::atomic<uint32_t> s_cost_sum{0}, s_cost_n{0}, s_cost_max{0};

void bump(std::atomic<uint32_t> &c, uint32_t n = 1)
{
    c.fetch_add(n, std::memory_order_relaxed);
}

/*
 * One frame: run the analyser, time it, and publish anything it produced.
 *
 * show_at_us, analyser and model_id come from the spec rather than from the
 * analyser -- an analyser that could set them could date its own results, which
 * is the one thing the presentation-delay rule forbids.
 */
void run_one(const AnalyserSpec &sp, const LaneFrame &f)
{
    Result r{};
    const int64_t t0 = esp_timer_get_time();
    const bool produced = s_slow->process(f.spec, f.index, f.due_us, &r);
    const int64_t took = esp_timer_get_time() - t0;

    bump(s_cost_sum, (uint32_t)took);
    bump(s_cost_n);
    uint32_t prev = s_cost_max.load(std::memory_order_relaxed);
    while (took > (int64_t)prev &&
           !s_cost_max.compare_exchange_weak(prev, (uint32_t)took,
                                             std::memory_order_relaxed)) {
    }

    if (!produced) {
        return;
    }

    r.analyser   = (uint8_t)s_slot;
    r.model_id   = sp.model_id;
    r.show_at_us = f.due_us + sp.present_delay_us;
    s_latch->publish(s_slot, r);
    bump(s_results);

    /*
     * Say what changed, and only what changed.
     *
     * This is what reaches the laptop collector, which forwards ESP_LOG, so it
     * is also the record you read afterwards against the audio. Logging every
     * result would be one line a second per analyser -- five times the rate of
     * the periodic lines already here, for a value that mostly repeats. Logging
     * the TRANSITIONS gives a track's shape in a dozen lines and stays silent
     * through a steady passage.
     *
     * The instant printed is show_at_us, not now: it is the moment the strips
     * act on this, which is the only one worth lining up against a recording.
     * The counts in the periodic `ml:` line cover the results this does not
     * print.
     */
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

void ml_task(void *arg)
{
    (void)arg;

    const AnalyserSpec &sp = s_slow->spec();
    uint32_t seen_gen = s_gen.load(std::memory_order_acquire);

    LaneFrame f;
    while (true) {
        if (xQueueReceive(s_q, &f, pdMS_TO_TICKS(200)) != pdTRUE) {
            continue;
        }

        /*
         * A frame from before a restart. Drop it, and reset the analyser once
         * on the transition: a context spanning a splice describes audio that
         * was never played in that order.
         */
        if (f.gen != seen_gen) {
            if ((int32_t)(f.gen - seen_gen) > 0) {
                seen_gen = f.gen;
                s_slow->reset();
            } else {
                continue;               /* older than the current timeline */
            }
        }

        run_one(sp, f);
    }
}

}  // namespace

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
    if (xTaskCreatePinnedToCore(ml_task, "mlan", 4096, nullptr, 3, nullptr, 0) != pdPASS) {
        /* Checked: without it the lane's queue fills once and every feed is
         * refused from then on, which reads as a starved model rather than a
         * missing one. */
        ESP_LOGE(TAG, "TASK \"mlan\" FAILED TO START -- the slow lane will "
                      "report nothing");
    }
}

bool ml_lane_feed(const uint8_t (&spec)[SPEC_BINS], int64_t index, int64_t due_us)
{
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

}  // namespace df
