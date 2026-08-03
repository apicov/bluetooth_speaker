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
 * are shared. test_pattern_sync.cpp enforces this mechanically by running a
 * pattern as two units with different join times, render counts and drop
 * histories and requiring byte-identical output, and it carries a deliberately
 * wrong pattern that it requires to fail.
 */
#pragma once

#include <stdint.h>

#include "beat_detect.h"

namespace df {

constexpr int FFT_N    = 1024;          /* 43 Hz bins at 44.1 kHz */
constexpr int RATE     = 44100;         /* what the tuning was measured at */
constexpr int BINS     = FFT_N / 2;
constexpr int CHANNELS = 2;

/*
 * Band edges in Hz, and the bins they land on at whatever rate the stream is.
 *
 * These used to be bin numbers -- { 1, 4, 24, 117 } -- with a comment giving the
 * frequencies they meant at 44.1 kHz. That is the same assumption that made the
 * rest of this file wrong at any other rate: the source picks the sample rate,
 * the bridge advertises 16/32/44.1/48 kHz, and nothing checks which one arrives.
 *
 * The frequencies are the ones the bins were, not the round numbers the old
 * comment quoted: bin 24 begins at 1033.6 Hz and bin 117 at 5038.8 Hz. That
 * distinction is not pedantry. The boom detector's flux floor of 0.02 was swept
 * against these exact bins over ten recordings, so a band that moves by one bin
 * at 44.1 kHz silently invalidates a measurement -- which is why the
 * static_assert below exists rather than a comment asking someone to be careful.
 */
constexpr int BAND_EDGE_HZ[] = { 43, 172, 1034, 5039 };
constexpr int BEAT_BANDS_N = (int)(sizeof(BAND_EDGE_HZ) / sizeof(BAND_EDGE_HZ[0]));
static_assert(BEAT_BANDS_N == BEAT_BANDS, "one edge per band");

/* Nearest bin to `hz`. Integer throughout so it is usable in a static_assert. */
constexpr int band_bin(int hz, int rate)
{
    return (hz * FFT_N + rate / 2) / rate;
}

/* The bins this reproduces at 44.1 kHz are the ones every tuning figure in
 * analysis.cpp was measured against. Changing them is a retune, not a refactor. */
static_assert(band_bin(BAND_EDGE_HZ[0], RATE) == 1,   "band 0 moved");
static_assert(band_bin(BAND_EDGE_HZ[1], RATE) == 4,   "band 1 moved");
static_assert(band_bin(BAND_EDGE_HZ[2], RATE) == 24,  "band 2 moved");
static_assert(band_bin(BAND_EDGE_HZ[3], RATE) == 117, "band 3 moved");

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

    /*
     * The zabumba's boom, detected separately from everything else.
     *
     * `onset` above is a weighted sum across all four bands, which is right for
     * music where the transients are broadband. Forró is not that: the triangle
     * plays continuous eighths or sixteenths at 4-8 kHz and contributes flux on
     * every subdivision, so a wideband detector follows the triangle and the
     * lights flicker with it instead of moving with the drum.
     *
     * This looks at the lowest band alone -- 43-129 Hz, where the mallet stroke
     * on the zabumba's big head lives, and where in a pe-de-serra trio there is
     * no bass guitar to compete. The stick stroke on the underside head is a mid
     * transient and is deliberately not wanted here.
     */
    bool         boom;
    float        boom_strength;
    float        boom_flux;      /* for tuning: what the low band actually did */
    float        boom_threshold;
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
    /*
     * `sample_rate` is the rate of the audio that will be fed, and it must be
     * the same rate the caller uses to derive due_us -- the two are the forward
     * and reverse of one conversion, and if they disagree the count and the
     * timeline separate at their difference. Pass df::RATE only if that is
     * genuinely what the stream is.
     */
    void init(int sample_rate);

    /* Override the boom detector's tuning after init(). Exists for pattern_lab
     * to sweep these against a recording rather than rebuild per value; the
     * firmware uses whatever init() sets. */
    void set_boom_tuning(float k, float flux_floor, int64_t refractory_us);

    /* `stereo` is exactly FFT_N interleaved 16-bit frames. The returned
     * reference is valid until the next call. */
    const Frame &process(const int16_t *stereo, int64_t index,
                         int64_t due_us, uint8_t unit);

private:
    alignas(16) float buf_[FFT_N * 2];   /* complex interleaved, for the FFT */
    float      win_[FFT_N];
    float      mag_[BINS];
    float      band_[BEAT_BANDS];
    int        band_lo_[BEAT_BANDS];     /* derived from BAND_EDGE_HZ and the rate */
    int        band_hi_[BEAT_BANDS];
    beat_det_t beat_;
    beat_det_t boom_;        /* the low band alone -- see Frame::boom */
    Frame      frame_;
};

}  // namespace df
