#include "beat_detect.h"

#include <math.h>
#include <string.h>

static const float BAND_WEIGHT[BEAT_BANDS] = { 1.0f, 0.6f, 0.3f, 0.15f };

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
    d->last_onset_us = INT64_MIN / 2;
    d->threshold_k   = BEAT_THRESHOLD_K;
    d->refractory_us = BEAT_REFRACTORY_US;
    d->flux_floor    = BEAT_FLUX_FLOOR;
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

    if (d->hist_n < BEAT_HIST / 4) {
        return false;
    }
    if (flux <= threshold) {
        return false;
    }

    const int64_t since_onset = now_us - d->last_onset_us;
    if (since_onset < 0) {
        d->last_onset_us = INT64_MIN / 2;
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
