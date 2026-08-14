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
 * THIS LANE USED TO CARRY AUDIO, and most of it was that. It held a 12.8 kB
 * stream buffer of decimated PCM, a fixed-point resampler and its ~6.3 kB of
 * tables, and a SlowWindow that cut the same windows on every unit by counting
 * samples from a shared origin. All of it is gone. Analysers are handed the
 * quantised spectrum of one analysis frame, which arrives already on a grid
 * every unit agrees about -- Frame::index IS the agreement, so there is nothing
 * left to align. What remains here is the task, the latch and the timing.
 *
 * That is also why a unit with no audio can run a model: the frames come off the
 * radio, and this lane cannot tell.
 *
 * Internal to the component -- the audio code never sees this. visualiser.cpp
 * feeds it and it publishes into the same ResultLatch the render stage reads.
 *
 * ---------------------------------------------------------------------------
 * One slow analyser
 * ---------------------------------------------------------------------------
 *
 * The lane runs at most one, and refuses the rest with a log line. Two would
 * need two rolling contexts and a rule for reporting them independently -- real
 * complexity for a case nobody has yet. The INTERFACE already supports any
 * number: slots, specs and the latch are all plural, and lifting this limit is a
 * change to this file alone. Fast analysers are unaffected; any number of those
 * run.
 */
#pragma once

#include <stdint.h>

#include "analyser.hpp"
#include "result_latch.hpp"

namespace df {

/* Which slot the slow analyser occupies, or -1 if this build has none. */
int ml_lane_slot();

/*
 * Start the task. Does nothing if there is no slow analyser, so a build without
 * one pays no task, no stack and no queue.
 *
 * `frames_per_s` is how many analysis frames a second the lane is about to be
 * fed -- the stream rate over the hop. It is passed to the analyser's init() so
 * it can size a context in time; see Analyser::init().
 */
void ml_lane_start(ResultLatch *latch, int frames_per_s);

/*
 * Hand the lane one frame's spectrum.
 *
 * Non-blocking, and called from whichever task produced or received the frame --
 * so it must never wait.
 *
 * Returns false if the queue was full and the frame was dropped. UNLIKE THE
 * AUDIO LANE THIS REPLACED, THAT IS NOT A TIMELINE BREAK: every frame carries
 * its own index and due_us, so a dropped one costs the analyser a frame of
 * context and nothing after it is mislabelled. The caller counts it and carries
 * on. (The old lane derived its grid by counting samples, which is why losing
 * any meant re-anchoring.)
 */
bool ml_lane_feed(const uint8_t (&spec)[SPEC_BINS], int64_t index, int64_t due_us);

/*
 * Tell the lane that what follows belongs to a new timeline.
 *
 * Everything already queued describes the old one and is discarded, and the
 * analyser's accumulated context is dropped -- a second of context spanning a
 * splice describes audio that was never played in that order. Same event and
 * same reasoning as visualiser_flush().
 */
void ml_lane_restart();

/* Read-and-clear counters, for the analysis task's periodic line. */
struct MlLaneStats {
    uint32_t results;      /* published */
    uint32_t dropped;      /* frames the lane could not accept */
    uint32_t restarts;
    uint32_t cost_mean_us; /* inference, over the window */
    uint32_t cost_max_us;
};
MlLaneStats ml_lane_take_stats();

}  // namespace df
