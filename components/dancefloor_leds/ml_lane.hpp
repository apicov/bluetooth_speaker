/*
 * The slow lane: analysers that cannot run inside a frame period.
 *
 * The analysis task has 11.6 ms per frame at hop 512 and has been measured
 * taking up to 21 ms of it for the FFT and the two detectors alone. Anything
 * with a real context window has to run somewhere else, and "somewhere else"
 * has to be off the core the audio and the strip share, or a long inference
 * stalls playback rather than merely being late itself.
 *
 * So: its own task, priority 3, pinned to core 0. Below the analysis task (4),
 * the render task (5) and playback (8), and not on core 1 with any of them.
 *
 * Internal to the component -- the audio code never sees this. visualiser.cpp
 * feeds it and it publishes into the same ResultLatch the render stage reads.
 *
 * ---------------------------------------------------------------------------
 * One slow analyser
 * ---------------------------------------------------------------------------
 *
 * The lane runs at most one, and refuses the rest with a log line. Two would
 * need two rolling windows, two resamplers where their rates differ, and a
 * rule for sliding one buffer by two different hops -- real complexity for a
 * case nobody has yet. The INTERFACE already supports any number: slots, specs
 * and the latch are all plural, and lifting this limit is a change to this file
 * alone. Fast analysers are unaffected; any number of those run.
 */
#pragma once

#include <stdint.h>

#include "result_latch.hpp"

namespace df {

/*
 * The rate the slow analyser wants, or 0 if there is no slow analyser in this
 * build. The feeder resamples to it.
 */
int ml_lane_rate();

/* Which slot the slow analyser occupies, or -1. */
int ml_lane_slot();

/*
 * Start the task. Does nothing if there is no slow analyser, so a build without
 * one pays no task, no stack and no buffer.
 *
 * `stream_rate_hz` is the rate of the audio the feeder is about to resample
 * FROM -- the lane needs it only to report the ratio and its filter checksum.
 */
void ml_lane_start(ResultLatch *latch, int stream_rate_hz);

/*
 * Hand the lane audio, already at ml_lane_rate(), mono.
 *
 * Non-blocking, and called from the analysis task -- so it must never wait.
 *
 * Returns false if it could not take everything. The caller MUST treat that as
 * a break in the stream and restart the lane: the grid is carried forward by
 * counting what arrives, so audio lost here mislabels every window after it, by
 * the amount lost, for good. Counting the loss without re-anchoring would leave
 * this unit cutting windows a neighbour does not -- the exact failure
 * visualiser.cpp's ALIGN_DRIFT_US check exists to catch one level up.
 */
bool ml_lane_feed(const int16_t *mono, int n);

/*
 * Tell the lane that the audio it is about to be fed starts a new timeline, and
 * what instant its first sample is due.
 *
 * Everything already queued describes the old one. Same event and same
 * reasoning as visualiser_flush(): a result carrying a show_at_us derived from
 * an origin that no longer exists would be drawn at a moment that means nothing.
 */
void ml_lane_restart(int64_t origin_us);

/* Read-and-clear counters, for the analysis task's periodic line. */
struct MlLaneStats {
    uint32_t results;      /* published */
    uint32_t dropped;      /* samples the lane could not accept */
    uint32_t restarts;
    uint32_t cost_mean_us; /* inference, over the window */
    uint32_t cost_max_us;
};
MlLaneStats ml_lane_take_stats();

}  // namespace df
