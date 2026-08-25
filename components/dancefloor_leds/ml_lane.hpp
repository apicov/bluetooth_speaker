/**
 * @file ml_lane.hpp
 * @brief The slow analyser's own task, and the firmware side of getting its
 *        answers onto the strip.
 *
 * A fast-lane analyser runs inline on the analysis task and needs none of
 * this. A slow one cannot: it may take longer than a frame period, and the
 * analysis task has one frame period per frame and most of that already spent
 * on the FFT. So it runs here, on its own task, off the core the audio and the
 * strip share and at a priority below both -- and its results reach the render
 * stage through a df::ResultLatch rather than being written into a frame,
 * because by the time one exists the frame it belongs to has been produced
 * long since.
 *
 * Private to the firmware: this header is not in include/, because nothing
 * outside visualiser.cpp drives the lane and the host harness runs the fast
 * lane directly.
 */
#pragma once

#include <stdint.h>

#include "analyser.hpp"
#include "result_latch.hpp"

namespace df {

/** @brief Which slot the slow analyser occupies, or -1 if there is none.
 *  @return The slot index. */
int ml_lane_slot();

/**
 * @brief Start the lane, if a slow analyser is configured and its init()
 *        succeeds.
 *
 * @param latch         Where results are published for the render stage.
 * @param frames_per_s  Passed to df::Analyser::init(); the analyser sizes any
 *                      context in time through it.
 */
void ml_lane_start(ResultLatch *latch, int frames_per_s);

/**
 * @brief Offer one analysis frame to the slow analyser.
 *
 * @param spec    The frame's quantised spectrum.
 * @param index   Its place on the shared grid.
 * @param due_us  When its audio is heard.
 * @return false if the frame was dropped rather than queued -- the lane is
 *         still working on an earlier one. Counted, because a lane that drops
 *         steadily is one whose model does not fit its declared delay.
 */
bool ml_lane_feed(const uint8_t (&spec)[SPEC_BINS], int64_t index, int64_t due_us);

/** @brief Drop the analyser's accumulated state. Called wherever the audio
 *         stops continuing what came before -- a timeline restart, a rate
 *         change, a realignment. */
void ml_lane_restart();

/** @brief What the lane did since the last call, for a periodic log line. */
struct MlLaneStats {
    uint32_t results;       /**< Results published. */
    uint32_t dropped;       /**< Frames the lane could not take. */
    uint32_t restarts;      /**< ml_lane_restart() calls. */
    uint32_t cost_mean_us;  /**< Mean inference time. */
    uint32_t cost_max_us;   /**< ...and the worst, which is what a
                             *   present_delay_us has to cover. */
};
/** @brief Read the stats and clear them. @return What the lane did. */
MlLaneStats ml_lane_take_stats();

}  // namespace df
