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
#define BEAT_HIST  43     /* ~0.5 s of history at a 512-sample hop / 44.1 kHz */

/* Below this, flux is treated as silence rather than signal. Without it the
 * adaptive threshold collapses toward zero during quiet passages and fires on
 * dither noise. */
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
