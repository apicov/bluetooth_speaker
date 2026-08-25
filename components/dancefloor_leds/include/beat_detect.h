/**
 * @file beat_detect.h
 * @brief Onset ("beat") detection for driving the LEDs.
 *
 * Takes BAND ENERGIES rather than raw samples, so the FFT stays the caller's
 * problem -- esp-dsp on target -- and this file has no platform dependencies
 * and can be unit-tested on the host.
 *
 * Deliberately onset detection, not BPM tracking. Lights need to fire ON the
 * transient; they do not need to know the tempo. A tempo tracker is a much
 * larger thing to get right and nothing here currently needs one.
 *
 * THE CONSTANTS BELOW ARE A CROSS-UNIT AGREEMENT, not local quality knobs, on
 * any floor where more than one unit runs this detector. df::RemoteDetect is
 * exactly that case: the hub does the FFT and each satellite runs its own
 * detector over the band energies it is sent. Identical decisions then require
 * identical constants AND identical state -- and the state is BEAT_HIST frames
 * of flux history plus a refractory instant. Two consequences worth knowing
 * before touching anything here:
 *
 *   - a value changed on one unit and not another becomes a SYNC fault rather
 *     than a taste difference, and it presents as strips that mostly agree and
 *     disagree on the marginal onsets, which is expensive to diagnose
 *   - BEAT_HIST sets how long a unit that missed frames takes to converge back
 *     onto its neighbours' threshold, so it is a sync parameter as well as a
 *     tuning one
 *
 * df::Frame::spec is NOT a substitute for the bands as this detector's input,
 * and that matters if the input is ever reconsidered: it is quantised to 8
 * bits through beat_normalise(), and flux is a frame-to-frame DIFFERENCE, so
 * the quantisation would land directly on the signal the detector runs on.
 * BEAT_FLUX_FLOOR is a couple of percent on a 0..1 scale -- a handful of counts
 * out of 255 -- and near-threshold differences are smaller than that.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Compiled as C, and unit-tested as C on the host, but called from the C++
 * visualiser, which would otherwise mangle these names and fail to link. */
#ifdef __cplusplus
extern "C" {
#endif

/** @brief Bands the detector weighs, roughly: kick, low-mid, presence, air. */
#define BEAT_BANDS 4

/**
 * @brief How many frames of flux the adaptive threshold is measured over.
 *
 * A FRAME COUNT, so the wall-clock span it covers is set by the analysis hop
 * and moves when that does -- which is deliberate, and was measured rather
 * than assumed. Both axes were swept:
 *
 *   - On TUNING, this value sits at the knee. Halving it makes the threshold
 *     estimate noticeably noisier; doubling it buys a similar improvement
 *     again but takes twice the history to do it, by which point the onset
 *     rate has almost stopped responding.
 *   - On CONVERGENCE -- how long a unit that missed frames takes to agree with
 *     its neighbours again -- shorter is simply better. The history turns over
 *     in exactly BEAT_HIST frames, but only a fraction of that is visible,
 *     since a threshold difference changes a decision only when the flux lands
 *     between the two thresholds.
 *
 * So the two axes agree and there was no trade-off to adjudicate. What is
 * worth knowing is the road not taken: scaling this up to hold a fixed
 * wall-clock span across a hop change is the obvious mechanical move, and it
 * is wrong on BOTH axes at once.
 *
 * It sizes both of df::Analysis's detectors, not just the wideband one.
 *
 * A command-line -DBEAT_HIST wins, which is what lets the host harness sweep
 * it. Unlike the flux floors this cannot be a runtime knob: it is the length
 * of beat_det_t::hist, and that struct is on the firmware's per-frame path and
 * must be identical on every unit that runs a detector.
 */
#ifndef BEAT_HIST
#  define BEAT_HIST  43
#endif

/**
 * @brief Below this, flux is treated as silence rather than signal.
 *
 * Without it the adaptive threshold collapses toward zero during quiet
 * passages and fires on dither noise.
 *
 * Swept over a large corpus at both hops: across a factor of five in either
 * direction the onset rate moves by under a percent, and this value sits well
 * clear of the one rung a drumless control rejects. What that ladder did turn
 * up is about the LONGER hop rather than this one -- with non-overlapping
 * windows the detector fires repeatedly on material with no drum in it at this
 * very floor, and with overlapping windows it fires none. Overlapping improved
 * its false-positive behaviour, which is the opposite of what a shorter hop
 * was expected to cost.
 */
#define BEAT_FLUX_FLOOR 0.02f

/** @brief Onsets closer together than this are suppressed. Far above any real
 *         kick pattern, while killing double-triggers on one hit. */
#define BEAT_REFRACTORY_US 120000

/** @brief One detector's state. Two of them run per unit: one wideband, one on
 *         the low band alone. */
typedef struct {
    float last_flux;             /**< Exposed for tuning; see beat_det_last_flux(). */
    float last_threshold;        /**< Likewise; see beat_det_last_threshold(). */
    float prev[BEAT_BANDS];      /**< Last frame's bands, to difference against. */
    float hist[BEAT_HIST];       /**< The flux history the threshold is built from. */
    int hist_n;                  /**< How much of it is filled. */
    int hist_next;               /**< Where the next flux lands. */
    int64_t last_onset_us;       /**< For the refractory window. */
    bool primed;                 /**< The first frame has no predecessor. */

    /*
     * Per-instance, so one detector can look for something different from
     * another. beat_det_init() sets the defaults above and callers may
     * override them afterwards -- which is what the boom detector does, since
     * a drum you want on the pulse wants a longer refractory than one you want
     * every stroke of.
     */
    float   threshold_k;      /**< Standard deviations above the mean; see beat_det_update(). */
    int64_t refractory_us;    /**< Default BEAT_REFRACTORY_US. */
    float   flux_floor;       /**< Default BEAT_FLUX_FLOOR. */
} beat_det_t;

/**
 * @brief Map a raw band magnitude onto the 0..1 range beat_det_update()
 *        expects.
 *
 * `raw / (1 + raw)`, which is monotonic over the whole input range and never
 * reaches 1 -- so an increase always produces a positive rise, however loud
 * the input.
 *
 * A hard CLAMP here instead destroys exactly the signal the detector runs on,
 * and did. Flux counts energy INCREASES, so a band pinned at 1.0 has a rise of
 * exactly zero and contributes nothing -- and against this FFT and gain the
 * bass band clamps for most mastered music. The kick, the band weighted
 * highest and the one the lights are meant to follow, went silent on precisely
 * the loud tracks it matters for.
 *
 * It compresses at the top, so the same kick yields less flux when it arrives
 * on a loud passage -- but the threshold is an adaptive mean plus a multiple
 * of the standard deviation over recent frames, so it tracks that.
 *
 * @param raw  A band magnitude; negative or NaN reads as zero.
 * @return The normalised value, in [0, 1).
 */
float beat_normalise(float raw);

/**
 * @brief Zero a detector and apply the default tuning.
 * @param d  The detector.
 */
void beat_det_init(beat_det_t *d);

/**
 * @brief Feed one analysis frame.
 *
 * @param d               The detector.
 * @param band            Per-band energy, already normalised by the caller to
 *                        roughly 0..1 -- see beat_normalise().
 * @param now_us          The MASTER-clock instant this frame's audio is heard,
 *                        for the refractory window. Never a local clock
 *                        reading: two units reach this line milliseconds
 *                        apart.
 * @param[out] strength   Optional. How far above threshold the onset was,
 *                        clamped to 0..1, for scaling LED brightness.
 * @return true when this frame is an onset.
 */
bool beat_det_update(beat_det_t *d, const float band[BEAT_BANDS],
                     int64_t now_us, float *strength);

/**
 * @brief The flux from the most recent update.
 *
 * Only interesting for tuning: plotted against a real recording, this and the
 * threshold show directly whether the threshold multiplier sits in a sensible
 * place.
 *
 * @param d  The detector.
 * @return The flux.
 */
float beat_det_last_flux(const beat_det_t *d);

/**
 * @brief The adaptive threshold from the most recent update.
 * @param d  The detector.
 * @return The threshold that flux had to beat.
 */
float beat_det_last_threshold(const beat_det_t *d);

#ifdef __cplusplus
}
#endif
