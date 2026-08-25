
#include "resample.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define Q32_ONE   (1ULL << 32)
#define Q15_ONE   32768

#define CENTER (RESAMPLE_TAPS / 2)

static double sinc(double x)
{
    if (x > -1e-9 && x < 1e-9) {
        return 1.0;
    }
    const double pix = M_PI * x;
    return sin(pix) / pix;
}

int resample_init(resampler_t *r, int in_rate, int out_rate)
{
    if (!r || in_rate < 4000 || in_rate > 192000 ||
        out_rate < 4000 || out_rate > 192000) {
        return -1;
    }

    memset(r, 0, sizeof(*r));
    r->in_rate  = in_rate;
    r->out_rate = out_rate;
    r->step     = ((uint64_t)in_rate * Q32_ONE) / (uint64_t)out_rate;

    double fc = 0.5;
    if (out_rate < in_rate) {
        fc = 0.5 * (double)out_rate / (double)in_rate;
    }
    fc *= 0.90;

    for (int p = 0; p < RESAMPLE_PHASES; p++) {
        const double frac = (double)p / (double)RESAMPLE_PHASES;

        double raw[RESAMPLE_TAPS];
        double sum = 0.0;
        for (int t = 0; t < RESAMPLE_TAPS; t++) {

            const double x = (double)t - (double)CENTER + frac;

            const double u = (x + (double)CENTER) / (double)(RESAMPLE_TAPS - 1);
            const double w = (u < 0.0 || u > 1.0)
                           ? 0.0
                           : 0.42 - 0.5  * cos(2.0 * M_PI * u)
                                  + 0.08 * cos(4.0 * M_PI * u);
            raw[t] = 2.0 * fc * sinc(2.0 * fc * x) * w;
            sum += raw[t];
        }

        int32_t acc = 0;
        int     big = 0;
        for (int t = 0; t < RESAMPLE_TAPS; t++) {
            const double scaled = raw[t] / sum * (double)Q15_ONE;
            const int32_t v = (int32_t)(scaled < 0 ? scaled - 0.5 : scaled + 0.5);
            r->taps[p][t] = (int16_t)v;
            acc += v;
            if (v > r->taps[p][big]) {
                big = t;
            }
        }
        r->taps[p][big] = (int16_t)(r->taps[p][big] + (Q15_ONE - acc));
    }

    r->ready = 1;
    return 0;
}

void resample_reset(resampler_t *r)
{
    if (!r) return;
    memset(r->hist, 0, sizeof(r->hist));
    r->phase = 0;
}

int resample_max_out(const resampler_t *r, int n)
{
    if (!r || !r->ready || n <= 0) return 0;

    return (int)(((uint64_t)n * (uint64_t)r->out_rate) / (uint64_t)r->in_rate) + 1;
}

int resample_push(resampler_t *r, const int16_t *in, int n,
                  int16_t *out, int max_out)
{
    if (!r || !r->ready || !in || !out || n <= 0 || max_out <= 0) {
        return 0;
    }

    int produced = 0;

    for (int i = 0; i < n; i++) {

        for (int t = RESAMPLE_TAPS - 1; t > 0; t--) {
            r->hist[t] = r->hist[t - 1];
        }
        r->hist[0] = in[i];

        while (r->phase < Q32_ONE) {
            if (produced >= max_out) {

                return produced;
            }
            const uint32_t p = (uint32_t)(r->phase >> (32 - RESAMPLE_PHASE_BITS));
            const int16_t *tap = r->taps[p & (RESAMPLE_PHASES - 1)];

            int64_t acc = 0;
            for (int t = 0; t < RESAMPLE_TAPS; t++) {
                acc += (int64_t)tap[t] * (int64_t)r->hist[t];
            }
            acc >>= 15;
            if (acc >  32767) acc =  32767;
            if (acc < -32768) acc = -32768;
            out[produced++] = (int16_t)acc;

            r->phase += r->step;
        }
        r->phase -= Q32_ONE;
    }

    return produced;
}

uint32_t resample_table_checksum(const resampler_t *r)
{
    if (!r || !r->ready) return 0;

    uint32_t h = 2166136261u;
    const uint8_t *p = (const uint8_t *)r->taps;
    for (size_t i = 0; i < sizeof(r->taps); i++) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}
