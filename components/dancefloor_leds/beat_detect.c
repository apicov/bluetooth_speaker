#include "beat_detect.h"

#include <math.h>
#include <string.h>

/* Bass-weighted: on a dance floor the kick is what the lights should follow.
 * Higher bands still contribute so that snares and hats register, just less. */
static const float BAND_WEIGHT[BEAT_BANDS] = { 1.0f, 0.6f, 0.3f, 0.15f };

/* How many standard deviations above recent mean flux counts as an onset.
 * Lower fires on texture, higher misses soft kicks. */
#define BEAT_THRESHOLD_K 1.8f

void beat_det_init(beat_det_t *d)
{
    memset(d, 0, sizeof(*d));
    d->last_onset_us = INT64_MIN / 2;   /* halved: leaves room to subtract without overflow */
}

static void hist_push(beat_det_t *d, float flux)
{
    d->hist[d->hist_next] = flux;
    d->hist_next = (d->hist_next + 1) % BEAT_HIST;
    if (d->hist_n < BEAT_HIST) {
        d->hist_n++;
    }
}

bool beat_det_update(beat_det_t *d, const float band[BEAT_BANDS],
                     int64_t now_us, float *strength)
{
    if (strength) {
        *strength = 0.0f;
    }

    /* Spectral flux: only energy *increases* matter. A band decaying is not an
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

    /* Threshold is computed from history *excluding* this frame, so a large
     * onset cannot raise the bar it is being judged against. */
    float mean = 0.0f, var = 0.0f;
    if (d->hist_n > 0) {
        for (int i = 0; i < d->hist_n; i++) {
            mean += d->hist[i];
        }
        mean /= d->hist_n;
        for (int i = 0; i < d->hist_n; i++) {
            float dv = d->hist[i] - mean;
            var += dv * dv;
        }
        var /= d->hist_n;
    }
    float threshold = mean + BEAT_THRESHOLD_K * sqrtf(var);
    if (threshold < BEAT_FLUX_FLOOR) {
        threshold = BEAT_FLUX_FLOOR;
    }

    hist_push(d, flux);

    /* Needs a little history before the threshold means anything. */
    if (d->hist_n < BEAT_HIST / 4) {
        return false;
    }
    if (flux <= threshold) {
        return false;
    }
    if (now_us - d->last_onset_us < BEAT_REFRACTORY_US) {
        return false;
    }

    d->last_onset_us = now_us;
    if (strength) {
        float s = (flux - threshold) / (threshold > 0.0f ? threshold : 1.0f);
        *strength = s > 1.0f ? 1.0f : s;
    }
    return true;
}
