/*
 * The analysis pipeline and the pattern interface, shared by the firmware and
 * the host harness in tools/pattern_lab.
 *
 * Nothing here touches a clock, a task, or a strip. That is deliberate: the
 * firmware half of the visualiser owns all of it, and everything in this file
 * is a pure function of the audio and the shared timeline, so the same code
 * produces the same lights on a laptop and on a board.
 *
 * ---------------------------------------------------------------------------
 * The one rule for patterns
 * ---------------------------------------------------------------------------
 *
 * A Pattern must be a pure function of the Frames it has been given.
 *
 * Every field of Frame is identical on every speaker for the same audio, so a
 * pattern obeying that rule is automatically in sync with its neighbours and no
 * further mechanism is needed. Reaching outside it does not fail loudly -- it
 * fails as strips that agree at first and drift apart over minutes, which is
 * expensive to diagnose. Four ways it has actually been broken here:
 *
 *   - accumulating per render call ("hue += 0.3") -- units do not render the
 *     same number of times, so the accumulators diverge
 *   - reading a wall clock -- units reach the same line milliseconds apart
 *   - measuring elapsed local time -- nearly the same, not exactly
 *   - anything random or uninitialised
 *
 * Use f.due_us for anything time-based and f.index for anything counted. Both
 * are shared. test_pattern_sync.c enforces this mechanically by running a
 * pattern as two units with different skew and drop histories and requiring
 * byte-identical output.
 */
#pragma once

#include <stdint.h>

#include "beat_detect.h"

namespace df {

constexpr int FFT_N    = 1024;          /* 43 Hz bins at 44.1 kHz */
constexpr int RATE     = 44100;
constexpr int BINS     = FFT_N / 2;
constexpr int CHANNELS = 2;

/* One analysis frame: ~23 ms of audio, reduced. */
struct Frame {
    int64_t      index;      /* block number, from an origin all units share */
    int64_t      due_us;     /* master-clock instant this audio is heard */
    const float *mag;        /* BINS raw magnitudes, lowest bin first */
    const float *band;       /* BEAT_BANDS, normalised to 0..1 */
    float        flux;       /* weighted spectral flux this frame */
    float        threshold;  /* what flux had to beat to count as an onset */
    bool         onset;
    float        strength;   /* 0..1 on an onset, else 0 */
    uint8_t      unit;       /* which speaker; 0 is the hub */
};

/* Implement this to make a new pattern. See the rule at the top of the file. */
class Pattern {
public:
    virtual ~Pattern() = default;
    virtual const char *name() const = 0;

    /* Write `count` RGB triples. Called once per analysis frame, ~43 Hz. */
    virtual void render(const Frame &f, uint8_t *rgb, uint32_t count) = 0;

    /* Drop any accumulated state. Called when the stream restarts. */
    virtual void reset() {}
};

/* Audio in, Frame out. Owns the FFT, the band split and the onset detector. */
class Analysis {
public:
    void init();

    /* `stereo` is exactly FFT_N interleaved 16-bit frames. The returned
     * reference is valid until the next call. */
    const Frame &process(const int16_t *stereo, int64_t index,
                         int64_t due_us, uint8_t unit);

private:
    alignas(16) float buf_[FFT_N * 2];   /* complex interleaved, for the FFT */
    float      win_[FFT_N];
    float      mag_[BINS];
    float      band_[BEAT_BANDS];
    beat_det_t beat_;
    Frame      frame_;
};

}  // namespace df
