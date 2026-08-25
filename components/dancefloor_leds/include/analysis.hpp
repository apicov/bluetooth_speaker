/**
 * @file analysis.hpp
 * @brief The analysis pipeline and the pattern interface, shared by the
 *        firmware and the host harness in tools/pattern_lab.
 *
 * Nothing here touches a clock, a task, or a strip. That is deliberate: the
 * firmware half of the visualiser owns all of it, and everything in this file
 * is a pure function of the audio and the shared timeline, so the same code
 * produces the same lights on a laptop and on a board.
 *
 * THE ONE RULE FOR PATTERNS: a Pattern must be a pure function of the Frames
 * it has been given.
 *
 * Every field of Frame is identical on every speaker for the same audio, so a
 * pattern obeying that rule is automatically in sync with its neighbours and
 * no further mechanism is needed. Reaching outside it does not fail loudly --
 * it fails as strips that agree at first and drift apart over minutes, which
 * is expensive to diagnose. Four ways it has actually been broken here:
 *
 *   - accumulating per render call ("hue += 0.3") -- units do not render the
 *     same number of times, so the accumulators diverge
 *   - reading a wall clock -- units reach the same line milliseconds apart
 *   - measuring elapsed LOCAL time -- nearly the same, not exactly
 *   - anything random or uninitialised
 *
 * Use Frame::due_us for anything time-based and Frame::index for anything
 * counted. Both are shared. test_pattern_sync.cpp enforces this mechanically
 * by running a pattern as two units with different join times, render counts
 * and drop histories and requiring byte-identical output, and it carries a
 * deliberately wrong pattern that it requires to FAIL.
 */
#pragma once

#include <stdint.h>

#include "analyser.hpp"
#include "analysis_config.h"
#include "beat_detect.h"

namespace df {

/**
 * @brief The FFT's window length -- WINDOW-ONLY.
 *
 * It sets BINS, band_bin() and the band assertions below, the Hann table and
 * the magnitude normalisation, and the host FFT. None of those care how often
 * a window is taken. See analysis_config.h for where it is set.
 */
constexpr int FFT_N    = DF_FFT_N;
/** @brief How far the analysis advances between windows -- GRID-ONLY. It sets
 *         how often a frame is produced, and therefore the block grid the
 *         units align to. Nothing in the transform reads it. */
constexpr int HOP_N    = DF_HOP_N;
/** @brief What one window carries over into the next; zero when they do not
 *         overlap. */
constexpr int TAIL_N   = DF_TAIL_N;
/** @brief The rate the tuning was measured at. NOT an assumption about the
 *         stream: Analysis::init() takes the real rate. */
constexpr int RATE     = 44100;
/** @brief Usable bins out of one transform. */
constexpr int BINS     = FFT_N / 2;
/** @brief Channels in the audio handed to process(). */
constexpr int CHANNELS = 2;

/**
 * @brief Band edges in Hz, and the bins they land on at whatever rate the
 *        stream is.
 *
 * These were once bin NUMBERS, with a comment giving the frequencies they
 * meant at one rate. That is the same assumption that made the rest of this
 * wrong at any other rate: the source picks the sample rate and nothing checks
 * which one arrives.
 *
 * The frequencies are the ones the bins actually were, not the round numbers
 * a comment might quote, and that distinction is not pedantry: the boom
 * detector's flux floor was swept against these exact bins, so a band that
 * moves by one bin at the reference rate silently invalidates a measurement.
 * Hence the assertions below rather than a comment asking someone to be
 * careful.
 */
constexpr int BAND_EDGE_HZ[] = { 43, 172, 1034, 5039 };
/** @brief One edge per band, checked against the detector's own count. */
constexpr int BEAT_BANDS_N = (int)(sizeof(BAND_EDGE_HZ) / sizeof(BAND_EDGE_HZ[0]));
static_assert(BEAT_BANDS_N == BEAT_BANDS, "one edge per band");

/**
 * @brief Nearest bin to a frequency. Integer throughout, so it is usable in a
 *        static_assert.
 * @param hz    The frequency.
 * @param rate  The stream rate.
 * @return The bin index.
 */
constexpr int band_bin(int hz, int rate)
{
    return (hz * FFT_N + rate / 2) / rate;
}

/* The bins this reproduces at the reference rate are the ones every tuning
 * figure in analysis.cpp was measured against. Changing them is a retune, not
 * a refactor. */
static_assert(band_bin(BAND_EDGE_HZ[0], RATE) == 1,   "band 0 moved");
static_assert(band_bin(BAND_EDGE_HZ[1], RATE) == 4,   "band 1 moved");
static_assert(band_bin(BAND_EDGE_HZ[2], RATE) == 24,  "band 2 moved");
static_assert(band_bin(BAND_EDGE_HZ[3], RATE) == 117, "band 3 moved");

/**
 * @brief Bottom of the portable spectrum.
 *
 * SPEC_BINS bins, geometrically spaced between this and SPEC_HI_HZ: something
 * near a seventh of an octave each, which is near enough to how the ear
 * divides the range and to how anyone would draw it. Below a few hundred hertz
 * a bin is narrower than the FFT's resolution and several of them read the
 * same underlying bins -- inherent at this window length, and preferable to
 * spacing them linearly and spending most of the strip on the top two octaves.
 *
 * The width itself is in analyser.hpp, because Analyser::process() needs it
 * and this header includes that one. These two edges stay here, with the code
 * that turns FFT bins into them.
 */
constexpr float SPEC_LO_HZ  = 40.0f;
/** @brief Top of the portable spectrum. See SPEC_LO_HZ. */
constexpr float SPEC_HI_HZ  = 16000.0f;

/**
 * @brief The boom detector's threshold multiplier.
 *
 * Named here because two places need it: Analysis::init(), which applies it,
 * and pattern_lab, which sweeps around it. Literals in both is how the copies
 * came to disagree -- the tool substituted its own defaults for whichever
 * flags were not given, and its stand-in for the floor was a value known to be
 * wrong, so sweeping one parameter silently reverted another and every such
 * run measured something other than what it reported. The tool that reproduces
 * a measurement must not have its own idea of what the firmware does.
 *
 * The reasoning behind the value stays with the code that explains it, in
 * Analysis::init(). Same caveat as the wideband detector's constants -- see
 * beat_detect.h on these being a cross-unit agreement rather than a local
 * preference.
 */
constexpr float   BOOM_THRESHOLD_K  = 1.4f;
/** @brief The boom detector's flux floor. See BOOM_THRESHOLD_K. */
constexpr float   BOOM_FLUX_FLOOR   = 0.02f;
/** @brief The boom detector's refractory window. See BOOM_THRESHOLD_K. */
constexpr int64_t BOOM_REFRACTORY_US = 200000;

/** @brief One analysis frame: one FFT_N window of audio, reduced. One is
 *         produced every HOP_N samples; the window each frame describes does
 *         not change with the hop. */
struct Frame {
    int64_t      index;      /**< Block number, from an origin all units share. */
    int64_t      due_us;     /**< Master-clock instant this audio is heard. */
    /**
     * @brief The raw spectrum, by pointer, and the ONLY field that does not
     *        survive being stored.
     *
     * It points into the Analysis that produced it and is overwritten by the
     * next process(). Rendering is deferred -- frames are queued and drawn
     * when due -- so a queued frame's magnitudes are long gone by the time a
     * pattern sees it, and the render path sets this null rather than leaving
     * it dangling.
     *
     * So: usable by anything that consumes a frame immediately, which means
     * the host harness. NOT usable by a Pattern. It is kilobytes a frame,
     * which is why it is not simply copied like everything else here.
     */
    const float *mag;
    /** @brief Normalised to 0..1. BY VALUE, not by pointer, so a frame can be
     *         queued -- the cost of copying four floats is nothing against
     *         needing a second lifetime rule for one field of a struct that is
     *         otherwise plain data. */
    float        band[BEAT_BANDS];
    /** @brief The portable spectrum: by value, like #band, because a frame
     *         outlives the Analysis that made it. Same scale as #band,
     *         quantised -- 0 is silence, 255 the top of the normalised range.
     *         See SPEC_LO_HZ. */
    uint8_t      spec[SPEC_BINS];
    float        flux;       /**< Weighted spectral flux this frame. */
    float        threshold;  /**< What flux had to beat to count as an onset. */
    bool         onset;      /**< Whether it did. */
    float        strength;   /**< 0..1 on an onset, else 0. */
    /** @brief Which speaker computed the frame; 0 is the hub. Does not travel.
     *         Kept because Analysis::process() reports it and a diagnostic may
     *         yet want it. */
    uint8_t      unit;

    /**
     * @brief The zabumba's boom, detected separately from everything else.
     *
     * #onset is a weighted sum across all four bands, which is right for music
     * where the transients are broadband. Forró is not that: the triangle
     * plays continuous subdivisions in the upper bands and contributes flux on
     * every one, so a wideband detector follows the TRIANGLE and the lights
     * flicker with it instead of moving with the drum.
     *
     * This looks at the LOWEST band alone, where the mallet stroke on the
     * zabumba's big head lives and where in a pé-de-serra trio there is no
     * bass guitar to compete. The stick stroke on the underside head is a mid
     * transient and is deliberately not wanted here.
     */
    bool         boom;
    float        boom_strength;  /**< 0..1 on a boom, else 0. */
    float        boom_flux;      /**< For tuning: what the low band actually did. */
    float        boom_threshold; /**< ...and what it had to beat. */

    /**
     * @brief What each pluggable analyser last said, as of this frame.
     *
     * One slot per registered analyser, at the same index, so f.ml[i] is
     * always analyser i whatever else is enabled -- see DF_ML_SLOTS for what a
     * slot costs.
     *
     * NOT computed here and not filled by Analysis::process(). A fast-lane
     * analyser's result is written when the frame is produced, because its
     * window IS this frame's window and there is nothing to wait for. A
     * slow-lane one is latched in by the RENDER stage, which takes the newest
     * Result whose show_at_us has arrived by this frame's due_us -- see
     * df::ResultLatch.
     *
     * It is in Frame rather than reaching patterns by some other route so that
     * a Pattern stays what it was: a pure function of the Frames it is given.
     * A pattern reading this still obeys the rule at the top of this file,
     * because show_at_us and due_us are both derived by counting from a shared
     * origin -- so every unit latches the same Result into the same frame
     * index regardless of when its own inference happened to finish.
     *
     * Check df::result_valid() before reading it. RESULT_NONE is what a unit
     * sees at startup, after a flush, and for as long as a slow analyser is
     * still filling its first context -- which for a one-second model is a
     * second of music, not an edge case.
     */
    Result       ml[ML_SLOTS];
};

/** @brief Implement this to make a new pattern. See the rule at the top of the
 *         file. */
class Pattern {
public:
    virtual ~Pattern() = default;
    /** @brief For selection and logs. @return The pattern's name. */
    virtual const char *name() const = 0;

    /**
     * @brief Draw one frame.
     *
     * Called once per analysis frame.
     *
     * @param f          The frame; the ONLY thing this may depend on.
     * @param[out] rgb   @p count RGB triples.
     * @param count      LEDs on the strip.
     */
    virtual void render(const Frame &f, uint8_t *rgb, uint32_t count) = 0;

    /** @brief Drop any accumulated state. Called when the stream restarts. */
    virtual void reset() {}
};

/** @brief Audio in, Frame out. Owns the FFT, the band split and the two onset
 *         detectors. */
class Analysis {
public:
    /**
     * @brief Prepare for a stream.
     *
     * @param sample_rate  The rate of the audio that will be fed. It must be
     *                     the same rate the caller uses to derive due_us --
     *                     the two are the forward and reverse of one
     *                     conversion, and if they disagree the count and the
     *                     timeline separate at their difference. Pass df::RATE
     *                     only if that is genuinely what the stream is.
     */
    void init(int sample_rate);

    /**
     * @brief Override the boom detector's tuning after init().
     *
     * Exists for the host harness to sweep these against a recording rather
     * than rebuild per value; the firmware uses whatever init() sets.
     *
     * @param k               Threshold multiplier.
     * @param flux_floor      Below this, flux is silence.
     * @param refractory_us   Minimum spacing between booms.
     */
    void set_boom_tuning(float k, float flux_floor, int64_t refractory_us);

    /**
     * @brief The same for the wideband detector, and deliberately narrower:
     *        only the floor.
     *
     * Its threshold multiplier is private to beat_detect.c, so there is no
     * value a caller could pass as the "leave it alone" fallback -- and it
     * needs none, being provably invariant under a hop change, since scaling
     * flux scales the mean and the standard deviation with it. BEAT_HIST is an
     * array length and is swept by rebuilding. The floor is the one of the
     * three that is per-instance state and genuinely needs measuring.
     *
     * @param flux_floor  Below this, flux is silence.
     */
    void set_beat_floor(float flux_floor);

    /**
     * @brief Transform one window and detect on it.
     *
     * HOP-AGNOSTIC, deliberately: this transforms whatever window it is handed
     * and never asks where the previous one started. All the hop knowledge
     * lives in the callers, which is why overlapping the windows is a change
     * to them and not to this.
     *
     * @param stereo  Exactly FFT_N interleaved 16-bit frames.
     * @param index   The caller's statement of where on the shared grid this
     *                window sits.
     * @param due_us  ...and when its audio is heard.
     * @param unit    Which speaker is computing it.
     * @return The frame. Valid until the next call.
     */
    const Frame &process(const int16_t *stereo, int64_t index,
                         int64_t due_us, uint8_t unit);

private:
    alignas(16) float buf_[FFT_N * 2];   /**< Complex interleaved, for the FFT. */
    float      win_[FFT_N];              /**< The Hann window. */
    float      mag_[BINS];               /**< This window's magnitudes. */
    float      band_[BEAT_BANDS];        /**< ...reduced to bands. */
    int        band_lo_[BEAT_BANDS];     /**< Derived from BAND_EDGE_HZ and the rate. */
    int        band_hi_[BEAT_BANDS];     /**< Likewise; contiguous by construction. */
    int        spec_lo_[SPEC_BINS];      /**< Likewise, for the portable spectrum. */
    int        spec_hi_[SPEC_BINS];      /**< Likewise. */
    beat_det_t beat_;                    /**< The wideband detector. */
    beat_det_t boom_;                    /**< The low band alone; see Frame::boom. */
    Frame      frame_;                   /**< What process() returns a reference to. */
};

/**
 * @brief The detector half of the analysis, run where the bands ARRIVE rather
 *        than where the FFT ran.
 *
 * This is the second half of Analysis::process(), line for line: the same two
 * detector calls, on the same numbers, with the same tuning. A unit that takes
 * frames from the hub receives the band floats at full precision -- they are
 * this class's entire input -- and derives onset, boom and the strengths
 * locally instead of receiving them decided.
 *
 * Identical input bytes and the same plain-C detector mean identical decisions
 * to the unit that computed the frame, which is what keeps a remote strip on
 * the same pulses as the hub's without anything new being synchronised. The
 * FFT -- the part that is only deterministic per-target -- is the part that
 * still runs in exactly one place.
 *
 * beat_detect.h's warnings apply in full: the constants are a cross-unit
 * agreement now, and a unit that misses a frame disagrees on the marginal
 * onsets until the history it lost has turned over.
 *
 * Two detectors' worth of state. Owned by one task, like Analysis.
 */
class RemoteDetect {
public:
    /** @brief Drop the flux history. Call it exactly where a local unit calls
     *         Analysis::init() -- on a RATE CHANGE, which re-cuts the bands
     *         the flux was measured against -- and NOT on stream gaps, which
     *         Analysis also spans without reset. Matching its reset points is
     *         part of matching its decisions. */
    void init();

    /**
     * @brief Derive this frame's detector fields.
     *
     * @param band     The frame's own band energies, already normalised --
     *                 exactly what Analysis hands its own detector.
     * @param due_us   When this frame's audio is heard.
     * @param[out] f   Fills onset, strength, flux, threshold, boom,
     *                 boom_strength, boom_flux and boom_threshold. Every other
     *                 field, including mag (null) and the spectrum, is the
     *                 caller's.
     */
    void process(const float band[BEAT_BANDS], int64_t due_us, Frame *f);

private:
    beat_det_t beat_;        /**< The wideband detector. */
    beat_det_t boom_;        /**< The low band alone; see Frame::boom. */
};

}  // namespace df
