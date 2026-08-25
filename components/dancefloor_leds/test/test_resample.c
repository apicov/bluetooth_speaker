/**
 * @file test_resample.c
 * @brief Host test for the fixed-point resampler.
 *
 * Two kinds of case. The SIGNAL ones check it is a resampler at all: unity DC
 * gain, an in-band tone surviving, a tone above the new Nyquist NOT folding
 * back into the band, and an output count that tracks the ratio rather than
 * being assumed.
 *
 * The other kind is about determinism, which is why this resampler is fixed
 * point in the first place -- see resample.h. A unit running a model must
 * produce the same answer as its neighbours, and the resampler is upstream of
 * everything, so the filter table is pinned by CHECKSUM. The table is built
 * with doubles, which are soft-float on both parts and so should come out
 * identical; that is a real argument and also the kind that quietly stops
 * being true, and this is what would notice.
 */
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "resample.h"

#ifndef M_PI
/** @brief -std=c11 is strict enough to hide it. */
#define M_PI 3.14159265358979323846
#endif

/** @brief Cases that did not hold; main() returns non-zero if any. */
static int failures = 0;

/** @brief Report one case, with the measured figures formatted in.
 *  @param name  What is pinned. @param cond Whether it held.
 *  @param fmt   printf format for the detail, then its arguments. */
static void check(const char *name, int cond, const char *fmt, ...)
{
    char detail[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(detail, sizeof(detail), fmt, ap);
    va_end(ap);
    printf("%-52s %s  %s\n", name, cond ? "PASS" : "FAIL", detail);
    if (!cond) failures++;
}

/** @brief Amplitude at one frequency, by direct correlation against a
 *         quadrature pair -- a one-bin DFT, which is all these cases need and
 *         needs no FFT to be trusted.
 *  @param x  Samples. @param n How many. @param hz The frequency.
 *  @param rate  Their sample rate. @return The amplitude, 0..1 of full scale. */
static double amplitude_at(const int16_t *x, int n, double hz, int rate)
{
    double re = 0, im = 0;
    for (int i = 0; i < n; i++) {
        const double a = 2.0 * M_PI * hz * i / rate;
        re += x[i] * cos(a);
        im -= x[i] * sin(a);
    }
    return 2.0 * sqrt(re * re + im * im) / n / 32768.0;
}

/** @brief Fill a buffer with a sine.
 *  @param[out] x  Samples. @param n How many. @param hz The frequency.
 *  @param rate    Their sample rate. @param amp Amplitude, 0..1. */
static void fill_sine(int16_t *x, int n, double hz, int rate, double amp)
{
    for (int i = 0; i < n; i++) {
        x[i] = (int16_t)(amp * 32000.0 * sin(2.0 * M_PI * hz * i / rate));
    }
}

/** @brief Push through the resampler in chunks, as a real caller does.
 *
 * The chunking is the point: the phase accumulator must carry across calls, so
 * a run split into pieces has to produce exactly what one call would.
 *
 *  @param r  The resampler. @param in Input. @param n How many.
 *  @param[out] out  Output. @param max_out Room in it.
 *  @param chunk     Input samples per call.
 *  @return Output samples produced. */
static int push_chunked(resampler_t *r, const int16_t *in, int n,
                        int16_t *out, int max_out, int chunk)
{
    int produced = 0;
    for (int i = 0; i < n; i += chunk) {
        const int take = (n - i) < chunk ? (n - i) : chunk;
        produced += resample_push(r, in + i, take,
                                  out + produced, max_out - produced);
    }
    return produced;
}

/** @brief A constant comes out at the same level: the filter rows are
 *         normalised to unity DC gain, per phase. */
static void test_dc_gain_is_unity(void)
{
    resampler_t r;
    resample_init(&r, 44100, 16000);

    static int16_t in[44100], out[20000];
    for (int i = 0; i < 44100; i++) in[i] = 10000;

    const int n = push_chunked(&r, in, 44100, out, 20000, 512);

    long sum = 0;
    int  from = RESAMPLE_TAPS * 2, cnt = 0;
    for (int i = from; i < n; i++) { sum += out[i]; cnt++; }
    const double mean = cnt ? (double)sum / cnt : 0;

    check("a constant comes out at the same level", fabs(mean - 10000.0) < 30.0,
          "mean %.1f, expected 10000", mean);
}

/** @brief A tone well inside the new Nyquist survives decimation at close to
 *         its original amplitude. */
static void test_a_tone_in_band_survives(void)
{
    resampler_t r;
    resample_init(&r, 44100, 16000);

    static int16_t in[44100], out[20000];
    fill_sine(in, 44100, 1000.0, 44100, 0.9);
    const int n = push_chunked(&r, in, 44100, out, 20000, 512);

    const double a = amplitude_at(out + 64, n - 64, 1000.0, 16000);
    check("a 1 kHz tone survives decimation", a > 0.80 && a < 1.00,
          "amplitude %.3f of full scale, sent 0.9", a);
}

/** @brief ...and one above it does NOT fold back into the band. This is the
 *         case the tap count was raised for: energy in front of a model at a
 *         frequency nobody played is worse than losing the top of the band. */
static void test_a_tone_above_nyquist_does_not_alias(void)
{
    resampler_t r;
    resample_init(&r, 44100, 16000);

    static int16_t in[44100], out[20000];

    fill_sine(in, 44100, 12000.0, 44100, 0.9);
    const int n = push_chunked(&r, in, 44100, out, 20000, 512);

    const double folded = amplitude_at(out + 64, n - 64, 4000.0, 16000);
    check("12 kHz does not fold back to 4 kHz", folded < 0.02,
          "alias amplitude %.4f, sent 0.9", folded);
}

/** @brief A second in is a second out, at every rate the bridge advertises --
 *         the output count varies by one from call to call, and must still be
 *         right in the aggregate. */
static void test_output_count_tracks_the_ratio(void)
{
    const struct { int in, out; } rates[] = {
        { 44100, 16000 }, { 48000, 16000 }, { 32000, 16000 }, { 16000, 16000 },
    };

    int ok = 1;
    char detail[160] = "";
    for (size_t k = 0; k < sizeof(rates) / sizeof(rates[0]); k++) {
        resampler_t r;
        resample_init(&r, rates[k].in, rates[k].out);

        static int16_t in[48000], out[60000];
        memset(in, 0, sizeof(in));
        const int n_in = rates[k].in;
        const int n = push_chunked(&r, in, n_in, out, 60000, 512);

        if (n < rates[k].out - 2 || n > rates[k].out + 2) {
            ok = 0;
            snprintf(detail, sizeof(detail), "%d -> %d gave %d, expected ~%d",
                     rates[k].in, rates[k].out, n, rates[k].out);
        }
    }
    check("a second in is a second out at every rate", ok,
          ok ? "44.1/48/32/16 kHz all within 2 samples" : detail);
}

/** @brief resample_max_out() really does bound the output, so a caller sizing
 *         its buffer from it cannot hit the silent drop. */
static void test_max_out_bounds_what_is_produced(void)
{
    resampler_t r;
    resample_init(&r, 44100, 16000);

    static int16_t in[512], out[4096];
    memset(in, 0, sizeof(in));

    int ok = 1;
    int worst = 0;
    for (int rep = 0; rep < 200; rep++) {
        const int bound = resample_max_out(&r, 512);
        const int n = resample_push(&r, in, 512, out, 4096);
        if (n > bound) ok = 0;
        if (n > worst) worst = n;
    }
    check("resample_max_out is never exceeded", ok,
          "worst run produced %d samples from 512", worst);
}

/** @brief A reset unit matches a fresh one exactly -- what resample_reset()
 *         has to mean for a splice not to smear across the join. */
static void test_reset_clears_the_history(void)
{
    resampler_t a, b;
    resample_init(&a, 44100, 16000);
    resample_init(&b, 44100, 16000);

    static int16_t junk[4096], in[4096], oa[4096], ob[4096];
    for (int i = 0; i < 4096; i++) junk[i] = (int16_t)(i * 977);
    fill_sine(in, 4096, 700.0, 44100, 0.8);

    resample_push(&a, junk, 4096, oa, 4096);
    resample_reset(&a);
    const int na = resample_push(&a, in, 4096, oa, 4096);
    const int nb = resample_push(&b, in, 4096, ob, 4096);

    const int same = (na == nb) && memcmp(oa, ob, (size_t)na * sizeof(int16_t)) == 0;
    check("a reset unit matches a fresh one exactly", same,
          "%d vs %d samples", na, nb);
}

/** @brief The filter table's checksum, pinned. The table is built with
 *         doubles, which should be identical on both cores; this is what would
 *         notice if that ever stopped being true. */
static void test_the_table_has_not_moved(void)
{

    const struct { int in, out; uint32_t sum; } want[] = {
        { 44100, 16000, 0xe841ff13 },
        { 48000, 16000, 0x5416fc70 },
    };

    int ok = 1;
    char detail[160] = "both ratios match";
    for (size_t k = 0; k < sizeof(want) / sizeof(want[0]); k++) {
        resampler_t r;
        resample_init(&r, want[k].in, want[k].out);
        const uint32_t got = resample_table_checksum(&r);
        if (got != want[k].sum) {
            ok = 0;
            snprintf(detail, sizeof(detail), "%d -> %d is 0x%08x, pinned 0x%08x",
                     want[k].in, want[k].out, got, want[k].sum);
        }
    }
    check("the filter table has not moved", ok, "%s", detail);

}

/**
 * @brief Run every case and report.
 * @return 0 if all held, 1 otherwise, so `make check` fails the build.
 */
int main(void)
{
    test_dc_gain_is_unity();
    test_a_tone_in_band_survives();
    test_a_tone_above_nyquist_does_not_alias();
    test_output_count_tracks_the_ratio();
    test_max_out_bounds_what_is_produced();
    test_reset_clears_the_history();
    test_the_table_has_not_moved();

    printf("\n%s\n", failures ? "FAILURES" : "all tests passed");
    return failures ? 1 : 0;
}
