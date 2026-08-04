/*
 * Onset ("beat") detection for driving the LEDs.
 *
 * Takes band energies rather than raw samples, so the FFT stays the caller's
 * problem (esp-dsp on target) and this file has no platform dependencies and
 * can be unit-tested on the host.
 *
 * Deliberately onset detection, not BPM tracking. Lights need to fire *on* the
 * transient; they do not need to know the tempo. A tempo tracker is a much
 * larger thing to get right and nothing here currently needs one.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Compiled as C (and unit-tested as C on the host), but called from the C++
 * visualiser, which would otherwise mangle these names and fail to link. */
#ifdef __cplusplus
extern "C" {
#endif

#define BEAT_BANDS 4      /* roughly: kick, low-mid, presence, air */

/*
 * ---------------------------------------------------------------------------
 * These constants are on their way to becoming a cross-unit agreement
 * ---------------------------------------------------------------------------
 *
 * Today every value below is a local quality knob: exactly one unit runs this
 * detector for any given frame. A unit doing its own analysis decides for
 * itself, and a unit given frames by the hub is handed decisions already made,
 * so two units cannot disagree about a threshold they do not both apply.
 *
 * A third source mode is intended -- the hub does the FFT, and each satellite
 * runs its own detector on the band energies it is sent, sitting between
 * "analyse everything locally" and "be told everything". See DF_ANALYSES_AUDIO
 * and DF_TAKES_REMOTE_FRAMES in visualiser.cpp, which name the two capabilities
 * separately for exactly that reason. Frame::band is already what such a unit
 * would consume: four floats, full precision, and it already travels.
 *
 * Under that mode these stop being local. Every unit runs beat_det_update() on
 * the same received bands, so identical decisions require identical constants
 * AND identical state -- and the state is BEAT_HIST frames of flux history plus
 * a refractory instant. Two consequences worth knowing before touching anything
 * here:
 *
 *   - a value changed on one unit and not another becomes a sync fault rather
 *     than a taste difference, and it will present as strips that mostly agree
 *     and disagree on the marginal onsets, which is expensive to diagnose
 *   - BEAT_HIST sets how long a unit that missed frames takes to converge back
 *     onto its neighbours' threshold, so it is a sync parameter as well as a
 *     tuning one
 *
 * Frame::spec is NOT a substitute for band here, and that matters if the input
 * is ever reconsidered: it is quantised to 8 bits through x/(1+x), and flux is
 * a frame-to-frame difference, so the quantisation lands directly on the signal
 * the detector runs on. BOOM_FLUX_FLOOR is 0.02 on a 0..1 scale, about five
 * counts of 255, and near-threshold differences are smaller than that.
 */

/*
 * How many frames of flux the adaptive threshold is measured over.
 *
 * A FRAME COUNT, so the span of wall-clock it covers is set by the analysis hop
 * and moves when that does: 1.0 s at hop 1024, 0.5 s at 512, 0.25 s at 256. The
 * comment here used to claim 0.5 s "at a 512-sample hop", which described a hop
 * this code has never actually run at.
 *
 * It stays a frame count. That was measured, not assumed, and the measurement
 * is in docs/tuning-corpus.md section 7; both axes were swept over 22, 43, 64
 * and 86 at hop 512.
 *
 * On the tuning axis 43 sits at the knee. Going 22 -> 43 buys a 22% quieter
 * threshold estimate; going 43 -> 86 buys a further 19% and takes twice the
 * history to do it, by which point the onset rate has almost stopped responding
 * (199 per minute against 191 across that doubling).
 *
 * On the second axis above -- how long a unit that missed frames takes to agree
 * with its neighbours again -- shorter is simply better, and it was measured
 * rather than reasoned about, by tools/tuning/converge.cpp. The history turns
 * over in exactly BEAT_HIST frames, but only a fraction of that is visible,
 * since a threshold difference changes a decision only when the flux lands
 * between the two thresholds: at 43 and hop 512 the median loss event never
 * produces a disagreement at all, and the 95th percentile is back in agreement
 * within 37 frames. That figure roughly doubles at 86.
 *
 * So the two axes agree and there was no trade-off to adjudicate. What is worth
 * knowing is the road not taken: scaling this to 86 to hold the original 998 ms
 * span across the hop change is the obvious mechanical move, and it is wrong on
 * BOTH axes at once.
 *
 * Note also that this sizes both of Analysis's detectors, not just the wideband
 * one -- boom_ is a beat_det_t too.
 *
 * A command-line -DBEAT_HIST wins, which is what lets the host harness sweep it
 * -- `make HIST=86` in tools/pattern_lab or components/dancefloor_leds/test.
 * Unlike the flux floors this cannot be a runtime knob: it is the length of
 * beat_det_t::hist below, and that struct is on the firmware's per-frame path
 * and must be identical on every unit that runs a detector.
 */
#ifndef BEAT_HIST
#  define BEAT_HIST  43
#endif

/*
 * Below this, flux is treated as silence rather than signal. Without it the
 * adaptive threshold collapses toward zero during quiet passages and fires on
 * dither noise.
 *
 * Set to match the boom detector's floor and carried unswept ever since. It has
 * its own ladder now, at both hops and over 206 recordings -- docs/tuning-corpus.md
 * section 6 -- and 0.02 survives it: across a factor of five the onset rate moves
 * by under 1% on the large set, and 0.02 is well clear of 0.012, the one rung the
 * drumless control rejects at hop 512.
 *
 * The figure that ladder did turn up is about the OLD hop, not this one. At hop
 * 1024 this detector fires 11 times in 30 seconds on material with no drum in it,
 * at this very floor; at 512 it fires none. Overlapping the windows improved its
 * false-positive behaviour, which is the opposite of what a shorter hop was
 * expected to cost.
 */
#define BEAT_FLUX_FLOOR 0.02f

/* Onsets closer together than this are suppressed. 120 ms allows 500 BPM,
 * far above any real kick pattern, while killing double-triggers on one hit. */
#define BEAT_REFRACTORY_US 120000

typedef struct {
    float last_flux;      /* exposed for tuning; see beat_det_last_flux() */
    float last_threshold;
    float prev[BEAT_BANDS];
    float hist[BEAT_HIST];
    int hist_n;
    int hist_next;
    int64_t last_onset_us;
    bool primed;          /* first frame has no predecessor to difference against */

    /*
     * Per-instance, so one detector can be looking for something different from
     * another. beat_det_init() sets the defaults below and callers may override
     * them afterwards -- which is what the zabumba detector does, since a drum
     * you want on the pulse wants a longer refractory than one you want every
     * stroke of.
     */
    float   threshold_k;      /* default BEAT_THRESHOLD_K */
    int64_t refractory_us;    /* default BEAT_REFRACTORY_US */
    float   flux_floor;       /* default BEAT_FLUX_FLOOR */
} beat_det_t;

/*
 * Map a raw band magnitude onto the 0..1 range beat_det_update() expects.
 *
 * This used to be a hard clamp in the caller, and the clamp destroyed exactly
 * the signal the detector runs on. Flux counts energy INCREASES, so a band
 * pinned at 1.0 has a rise of exactly zero and contributes nothing -- and
 * measured against this FFT and gain, the bass band clamps for any 60 Hz
 * content above about -11 dBFS, which is most mastered music. The kick, the
 * band weighted highest and the one the lights are meant to follow, went
 * silent on precisely the loud tracks it matters for.
 *
 * raw / (1 + raw) is monotonic over the whole input range and never reaches 1,
 * so an increase always produces a positive rise however loud the input. It
 * compresses at the top -- the same kick yields less flux when it arrives on a
 * loud passage -- but the detector's threshold is an adaptive mean plus 1.8
 * standard deviations over the last 43 frames, so it tracks that.
 */
float beat_normalise(float raw);

void beat_det_init(beat_det_t *d);

/*
 * Feed one analysis frame. `band` holds per-band energy, already normalised by
 * the caller to a roughly 0..1 range. Returns true when this frame is an onset.
 * `strength` (optional) receives how far above threshold it was, clamped to
 * 0..1, for scaling LED brightness.
 */
bool beat_det_update(beat_det_t *d, const float band[BEAT_BANDS],
                     int64_t now_us, float *strength);

/*
 * The flux and the adaptive threshold from the most recent update.
 *
 * Only interesting for tuning: plotted against a real recording they show
 * directly whether BEAT_THRESHOLD_K sits in a sensible place, which is a
 * question nobody has yet answered with anything but synthetic kicks.
 */
float beat_det_last_flux(const beat_det_t *d);
float beat_det_last_threshold(const beat_det_t *d);

#ifdef __cplusplus
}
#endif
