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
    float prev[BEAT_BANDS];
    float hist[BEAT_HIST];
    int hist_n;
    int hist_next;
    int64_t last_onset_us;
    bool primed;          /* first frame has no predecessor to difference against */
} beat_det_t;

void beat_det_init(beat_det_t *d);

/*
 * Feed one analysis frame. `band` holds per-band energy, already normalised by
 * the caller to a roughly 0..1 range. Returns true when this frame is an onset.
 * `strength` (optional) receives how far above threshold it was, clamped to
 * 0..1, for scaling LED brightness.
 */
bool beat_det_update(beat_det_t *d, const float band[BEAT_BANDS],
                     int64_t now_us, float *strength);
