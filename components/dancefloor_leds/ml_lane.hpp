
#pragma once

#include <stdint.h>

#include "analyser.hpp"
#include "result_latch.hpp"

namespace df {

int ml_lane_slot();

void ml_lane_start(ResultLatch *latch, int frames_per_s);

bool ml_lane_feed(const uint8_t (&spec)[SPEC_BINS], int64_t index, int64_t due_us);

void ml_lane_restart();

struct MlLaneStats {
    uint32_t results;
    uint32_t dropped;
    uint32_t restarts;
    uint32_t cost_mean_us;
    uint32_t cost_max_us;
};
MlLaneStats ml_lane_take_stats();

}
