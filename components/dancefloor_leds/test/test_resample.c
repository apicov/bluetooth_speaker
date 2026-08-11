/*
 * The resampler, checked for the three things that would break a model quietly.
 *
 *   gain          a level feature reads amplitude directly, so a ratio that is
 *                 not unity at DC scales every answer the model gives
 *   aliasing      the whole reason this is a filter and not a pick-every-nth.
 *                 Content above the new Nyquist that folds back appears to the
 *                 model as energy at a frequency that was never played
 *   sample count  due_us for the decimated stream is derived by COUNTING what
 *                 comes out, so a resampler that emits one extra sample per
 *                 buffer puts a unit permanently out of step with its
 *                 neighbours -- silently, and at a rate nothing else reports
 *
 * And one that would break it loudly across a mixed floor: the filter table
 * must be identical on every unit. It is built with doubles, which are
 * soft-float on both parts and therefore deterministic, but that is an argument
 * about a toolchain rather than about arithmetic -- so the table is pinned by
 * checksum here and any change to it has to be deliberate.
 */
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "resample.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int failures = 0;

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

/* Amplitude at `hz`, by correlation. Enough to tell 0.9 from 0.02. */
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

static void fill_sine(int16_t *x, int n, double hz, int rate, double amp)
{
    for (int i = 0; i < n; i++) {
        x[i] = (int16_t)(amp * 32000.0 * sin(2.0 * M_PI * hz * i / rate));
    }
}

/* Push in realistic chunks, the way the lane does, so the phase carry between
 * calls is exercised rather than one giant buffer hiding it. */
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

static void test_dc_gain_is_unity(void)
{
    resampler_t r;
    resample_init(&r, 44100, 16000);

    static int16_t in[44100], out[20000];
    for (int i = 0; i < 44100; i++) in[i] = 10000;

    const int n = push_chunked(&r, in, 44100, out, 20000, 512);

    /* Skip the first taps -- the history starts at zero and has to fill. */
    long sum = 0;
    int  from = RESAMPLE_TAPS * 2, cnt = 0;
    for (int i = from; i < n; i++) { sum += out[i]; cnt++; }
    const double mean = cnt ? (double)sum / cnt : 0;

    check("a constant comes out at the same level", fabs(mean - 10000.0) < 30.0,
          "mean %.1f, expected 10000", mean);
}

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

static void test_a_tone_above_nyquist_does_not_alias(void)
{
    resampler_t r;
    resample_init(&r, 44100, 16000);

    static int16_t in[44100], out[20000];
    /* 12 kHz against a new Nyquist of 8 kHz would fold to 4 kHz. That is the
     * failure this filter exists to prevent, and it is inaudible in a log --
     * the model would simply see a 4 kHz component nobody played. */
    fill_sine(in, 44100, 12000.0, 44100, 0.9);
    const int n = push_chunked(&r, in, 44100, out, 20000, 512);

    const double folded = amplitude_at(out + 64, n - 64, 4000.0, 16000);
    check("12 kHz does not fold back to 4 kHz", folded < 0.02,
          "alias amplitude %.4f, sent 0.9", folded);
}

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
        const int n_in = rates[k].in;                 /* one second */
        const int n = push_chunked(&r, in, n_in, out, 60000, 512);

        /* One second in, one second out, to within a sample of rounding. */
        if (n < rates[k].out - 2 || n > rates[k].out + 2) {
            ok = 0;
            snprintf(detail, sizeof(detail), "%d -> %d gave %d, expected ~%d",
                     rates[k].in, rates[k].out, n, rates[k].out);
        }
    }
    check("a second in is a second out at every rate", ok,
          ok ? "44.1/48/32/16 kHz all within 2 samples" : detail);
}

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

static void test_reset_clears_the_history(void)
{
    resampler_t a, b;
    resample_init(&a, 44100, 16000);
    resample_init(&b, 44100, 16000);

    static int16_t junk[4096], in[4096], oa[4096], ob[4096];
    for (int i = 0; i < 4096; i++) junk[i] = (int16_t)(i * 977);
    fill_sine(in, 4096, 700.0, 44100, 0.8);

    /* `a` sees noise, then is reset, then the signal. `b` sees only the signal.
     * After a reset they must agree exactly -- that is what makes a splice
     * survivable. */
    resample_push(&a, junk, 4096, oa, 4096);
    resample_reset(&a);
    const int na = resample_push(&a, in, 4096, oa, 4096);
    const int nb = resample_push(&b, in, 4096, ob, 4096);

    const int same = (na == nb) && memcmp(oa, ob, (size_t)na * sizeof(int16_t)) == 0;
    check("a reset unit matches a fresh one exactly", same,
          "%d vs %d samples", na, nb);
}

static void test_the_table_has_not_moved(void)
{
    /*
     * Pinned, not computed. These are the checksums this code produced when it
     * was written; they exist so that a change to the table -- a different tap
     * count, a different window, a toolchain that rounds differently -- is a
     * failing test rather than two units that disagree in the field.
     *
     * If a change here is deliberate, update the numbers in the same commit as
     * the change and say why. If it is not, something in the build has moved
     * and every locally-analysing unit on the floor is now suspect.
     */
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

    /*
     * This pins the table against changes to THIS code on a host. It says
     * nothing about whether an LX6 and an LX7 build the same table from the
     * same source -- only two boards can answer that, which is why the lane
     * prints its checksum at startup. Two consoles settle it.
     */
}

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
