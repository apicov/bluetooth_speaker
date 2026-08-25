/**
 * @file resample.c
 * @brief The arbitrary-ratio resampler. resample.h has the contract and the
 *        argument for why it is fixed point rather than float.
 *
 * A windowed-sinc polyphase filter: the table is built once per ratio in
 * resample_init(), and resample_push() is integer throughout so both cores
 * produce the same samples.
 */
#include "resample.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
/** @brief -std=c11 is strict enough to hide it. */
#define M_PI 3.14159265358979323846
#endif

/** @brief Unity in the Q32 phase accumulator. */
#define Q32_ONE   (1ULL << 32)
/** @brief Unity in the Q15 filter taps. */
#define Q15_ONE   32768

/** @brief The filter is centred in the history, which costs a constant group
 *         delay of this many INPUT samples. Constant, and identical on every
 *         unit, so it shifts the decimated stream against the original by a
 *         fixed amount rather than introducing any disagreement. */
#define CENTER (RESAMPLE_TAPS / 2)

/**
 * @brief Normalised sinc, with the removable singularity handled.
 * @param x  The argument.
 * @return sin(pi x) / (pi x), or 1 at the origin.
 */
static double sinc(double x)
{
    if (x > -1e-9 && x < 1e-9) {
        return 1.0;
    }
    const double pix = M_PI * x;
    return sin(pix) / pix;
}

/* Declared in resample.h, like the four below it. */
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

    /*
     * Cutoff, normalised to the INPUT rate.
     *
     * Decimating, the new Nyquist is the output's and everything above it must
     * go or it folds back into the band the model sees -- which is the whole
     * reason this is a filter and not a pick-every-nth. Interpolating, there
     * is nothing above the input's Nyquist to remove, so the cutoff stays
     * there.
     *
     * Backed off from Nyquist rather than sitting on it: a finite filter
     * cannot turn over instantly, and leaving the transition band inside the
     * range is what stops the corner from aliasing. It costs the top of the
     * band, which for a feature front end is nothing.
     */
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

            /* Distance from history tap t to the output position: hist[t]
             * holds the input sample t back from the newest, and the output
             * sits CENTER back from the newest plus `frac`. */
            const double x = (double)t - (double)CENTER + frac;

            /*
             * Blackman, centred on the FRACTIONAL position rather than on the
             * integer tap.
             *
             * Windowing at integer t leaves the window fixed while the sinc
             * slides underneath it, so every phase but zero is windowed
             * slightly off-centre and the stopband degrades with the fraction.
             * Measured, it cost several dB of alias rejection -- the
             * difference between passing and failing test_resample.c's
             * fold-back check.
             */
            const double u = (x + (double)CENTER) / (double)(RESAMPLE_TAPS - 1);
            const double w = (u < 0.0 || u > 1.0)
                           ? 0.0
                           : 0.42 - 0.5  * cos(2.0 * M_PI * u)
                                  + 0.08 * cos(4.0 * M_PI * u);
            raw[t] = 2.0 * fc * sinc(2.0 * fc * x) * w;
            sum += raw[t];
        }

        /*
         * Normalised to unity DC gain per phase, and normalised as INTEGERS.
         *
         * Scaling the doubles and rounding each independently leaves the row
         * summing to unity only by luck, and a row that sums slightly high is
         * a small gain step that appears every time the phase happens to land
         * there -- an amplitude ripple at the resampling period, which a level
         * feature would read as real. So the residual is pushed onto the
         * largest tap, where it is proportionally smallest.
         */
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

        /* Newest first, so tap 0 is the most recent sample. A shift is cheaper
         * than the index arithmetic a ring would need, at this length. */
        for (int t = RESAMPLE_TAPS - 1; t > 0; t--) {
            r->hist[t] = r->hist[t - 1];
        }
        r->hist[0] = in[i];

        /*
         * Emit every output whose position falls in the interval this input
         * sample just closed. Decimating, the step is more than one and most
         * inputs emit nothing; interpolating, it is less and one input emits
         * several.
         */
        while (r->phase < Q32_ONE) {
            if (produced >= max_out) {
                /* Cannot happen with a buffer sized by resample_max_out(), and
                 * is a lost sample if it ever does -- so leave the phase alone
                 * and stop, rather than advancing past audio never emitted. */
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

    /* FNV-1a over the table bytes, in a fixed order. Not cryptographic -- it
     * only has to change when the table does. */
    uint32_t h = 2166136261u;
    const uint8_t *p = (const uint8_t *)r->taps;
    for (size_t i = 0; i < sizeof(r->taps); i++) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}
