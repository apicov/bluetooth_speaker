/**
 * @file beat_detect.c
 * @brief Spectral-flux onset detection over an adaptive threshold.
 *
 * beat_detect.h owns the contract and the constants. What is here is the
 * arithmetic, and every part of it that has to be identical on two units --
 * see that header on these constants being a cross-unit agreement.
 *
 * No platform dependencies, so the host tests and the pattern harness drive
 * exactly this code.
 */
#include "beat_detect.h"

#include <math.h>
#include <string.h>

/** @brief Bass-weighted: on a dance floor the kick is what the lights should
 *         follow. Higher bands still contribute so snares and hats register,
 *         just less. */
static const float BAND_WEIGHT[BEAT_BANDS] = { 1.0f, 0.6f, 0.3f, 0.15f };

/**
 * @brief How many standard deviations above recent mean flux counts as an
 *        onset.
 *
 * Lower fires on texture, higher misses soft kicks. Private to this file
 * rather than in the header, and deliberately: it is provably invariant under
 * a hop change -- scaling flux scales the mean and the standard deviation with
 * it -- so unlike the flux floor it does not need a per-instance setter. A
 * detector wanting a different value sets beat_det_t::threshold_k after
 * beat_det_init(), which is what the boom detector does.
 */
#define BEAT_THRESHOLD_K 1.8f

float beat_normalise(float raw)
{
    if (!(raw > 0.0f)) {
        return 0.0f;
    }
    return raw / (1.0f + raw);
}

void beat_det_init(beat_det_t *d)
{
    memset(d, 0, sizeof(*d));
    /* Halved, so the subtraction in beat_det_update() has room and cannot
     * overflow against any plausible instant. */
    d->last_onset_us = INT64_MIN / 2;
    d->threshold_k   = BEAT_THRESHOLD_K;
    d->refractory_us = BEAT_REFRACTORY_US;
    d->flux_floor    = BEAT_FLUX_FLOOR;
}

/**
 * @brief Add one flux value to the ring the threshold is built from.
 * @param d     The detector.
 * @param flux  This frame's flux.
 */
static void hist_push(beat_det_t *d, float flux)
{
    d->hist[d->hist_next] = flux;
    d->hist_next = (d->hist_next + 1) % BEAT_HIST;
    if (d->hist_n < BEAT_HIST) {
        d->hist_n++;
    }
}

/**
 * @brief Feed one analysis frame; see beat_detect.h for the contract.
 *
 * Repeated here only because the array parameter stops Doxygen matching this
 * definition to that declaration. The reasoning is all in the header.
 *
 * @param d              The detector.
 * @param band           Per-band energy, normalised to roughly 0..1.
 * @param now_us         The MASTER-clock instant this frame's audio is heard.
 * @param[out] strength  Optional; how far above threshold, clamped to 0..1.
 * @return true when this frame is an onset.
 */
bool beat_det_update(beat_det_t *d, const float band[BEAT_BANDS],
                     int64_t now_us, float *strength)
{
    if (strength) {
        *strength = 0.0f;
    }

    /* Spectral flux: only energy INCREASES matter. A band decaying is not an
     * onset, and including the decay would smear the transient. */
    float flux = 0.0f;
    if (d->primed) {
        for (int i = 0; i < BEAT_BANDS; i++) {
            float rise = band[i] - d->prev[i];
            if (rise > 0.0f) {
                flux += rise * BAND_WEIGHT[i];
            }
        }
    }
    memcpy(d->prev, band, sizeof(d->prev));

    if (!d->primed) {
        d->primed = true;
        return false;
    }

    /*
     * The threshold is computed from history EXCLUDING this frame, so a large
     * onset cannot raise the bar it is being judged against -- hence the
     * hist_push() below rather than above.
     *
     * Summed OLDEST FIRST, not in array order. Float addition is not
     * associative, so the same history summed from a different starting index
     * gives a slightly different total. Two units holding identical flux can
     * sit at different rotations of the ring -- they start analysing at
     * whatever grid point their own alignment landed on -- so array order would
     * make the threshold depend on something the units do not share.
     * Iterating from hist_next makes the sum a function of the history alone.
     *
     * The difference is in the last bits, and the sync test passes on the
     * margin between flux and threshold being wide compared to it rather than
     * on the arithmetic being right. Overlapping windows shrink that margin
     * from both sides -- flux gets smaller and there are more near-threshold
     * frames -- so this stops being free before it stops being invisible.
     */
    float mean = 0.0f, var = 0.0f;
    if (d->hist_n > 0) {
        const int base = (d->hist_n < BEAT_HIST) ? 0 : d->hist_next;
        for (int j = 0; j < d->hist_n; j++) {
            mean += d->hist[(base + j) % d->hist_n];
        }
        mean /= d->hist_n;
        for (int j = 0; j < d->hist_n; j++) {
            float dv = d->hist[(base + j) % d->hist_n] - mean;
            var += dv * dv;
        }
        var /= d->hist_n;
    }
    float threshold = mean + d->threshold_k * sqrtf(var);
    if (threshold < d->flux_floor) {
        threshold = d->flux_floor;
    }
    d->last_flux = flux;
    d->last_threshold = threshold;

    hist_push(d, flux);

    /* Needs a little history before the threshold means anything. */
    if (d->hist_n < BEAT_HIST / 4) {
        return false;
    }
    if (flux <= threshold) {
        return false;
    }

    /*
     * The refractory window, and the one case where it must not apply.
     *
     * now_us is MASTER time, which counts from the hub's boot. Reset the hub
     * and it restarts near zero, so every frame that follows names an instant
     * BEFORE the last onset this detector fired on. The subtraction then goes
     * large and negative, which is smaller than any refractory, and the gate
     * would swallow every onset for as long as it took master time to climb
     * back past the old value -- the strip stops following the music until the
     * satellite itself is rebooted, which is exactly the fault this handles.
     *
     * A negative interval is not a short one. It says the timeline restarted
     * underneath us, so the stored instant belongs to a clock that no longer
     * exists and is DROPPED rather than compared against. Not a full init():
     * the flux history is still about the same audio and still valid -- see
     * df::RemoteDetect on why the reset points matter.
     */
    const int64_t since_onset = now_us - d->last_onset_us;
    if (since_onset < 0) {
        d->last_onset_us = INT64_MIN / 2;   /* as beat_det_init() leaves it */
    } else if (since_onset < d->refractory_us) {
        return false;
    }

    d->last_onset_us = now_us;
    if (strength) {
        float s = (flux - threshold) / (threshold > 0.0f ? threshold : 1.0f);
        *strength = s > 1.0f ? 1.0f : s;
    }
    return true;
}

float beat_det_last_flux(const beat_det_t *d)      { return d->last_flux; }
float beat_det_last_threshold(const beat_det_t *d) { return d->last_threshold; }
