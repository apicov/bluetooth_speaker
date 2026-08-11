/*
 * Stream rate to model rate, in fixed point.
 *
 * Nearly every published audio model wants 16 kHz mono. The stream is whatever
 * the phone chose -- the bridge advertises 16, 32, 44.1 and 48 kHz and takes
 * what it is given -- so the ratio is a runtime value and 44100/16000 is not an
 * integer. Hence an arbitrary-ratio resampler rather than a decimator.
 *
 * ---------------------------------------------------------------------------
 * Why fixed point, and not because it is faster
 * ---------------------------------------------------------------------------
 *
 * The hub is an LX7 and a satellite is an LX6. A unit running a model locally
 * must produce the same answer as its neighbours or the strips separate, and
 * the resampler is upstream of everything -- one sample different here is a
 * different window into the model. Integer arithmetic is identical on both
 * parts by construction. Float is identical only as long as nothing selects a
 * different kernel per target, which is not a promise either chip's toolchain
 * makes.
 *
 * The one place a double appears is building the filter table in
 * resample_init(), which runs once. Doubles are soft-float on BOTH parts --
 * neither has a double FPU -- so newlib runs the same code and the table comes
 * out the same. That is a real argument rather than a hope, but it is also the
 * kind of argument that quietly stops being true, so test_resample.c pins the
 * table's checksum for the rates that matter and will fail if it ever moves.
 *
 * Plain C and no dependencies, because three build systems compile it and only
 * one of them is ESP-IDF.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Taps and phases: the filter's quality against its cost and its RAM.
 *
 * 32 taps at 64 phases is 4 kB of int16 table and 32 multiply-accumulates per
 * output sample -- at 16 kHz out that is 512k MAC/s, around a fifth of a
 * percent of one core at 240 MHz.
 *
 * 16 taps was tried first and reached only -30 dB against the fold-back check
 * in test_resample.c, which is close enough to audible content to put energy in
 * front of a model at a frequency nobody played. Doubling the taps costs
 * nothing that matters here and buys about 20 dB.
 *
 * Still a feature front end's filter and not a listener's: nothing decimated
 * here is ever played.
 *
 * Phases must be a power of two: the phase index is the top bits of a Q32
 * accumulator, which is only exact when it is.
 */
#define RESAMPLE_TAPS       32
#define RESAMPLE_PHASE_BITS 6
#define RESAMPLE_PHASES     (1 << RESAMPLE_PHASE_BITS)

typedef struct {
    int      in_rate;
    int      out_rate;
    uint64_t step;                  /* Q32 input samples per output sample */
    uint64_t phase;                 /* Q32, distance past the newest input */
    int16_t  taps[RESAMPLE_PHASES][RESAMPLE_TAPS];   /* Q15, unity DC gain */
    int16_t  hist[RESAMPLE_TAPS];   /* [0] is newest */
    int      ready;
} resampler_t;

/*
 * Build the table for one ratio. Call again to change it.
 *
 * Returns 0 on success, -1 if either rate is out of range. `out_rate` may
 * exceed `in_rate` -- the filter then passes everything below in_rate/2 and the
 * result is interpolation rather than decimation, which is correct if unusual
 * here.
 */
int resample_init(resampler_t *r, int in_rate, int out_rate);

/*
 * Forget the history, keep the table.
 *
 * Call whenever the audio about to be pushed does not continue the audio pushed
 * before it -- a splice, a realignment, a timeline restart. The same event that
 * makes visualiser_realign() necessary, and for the same reason: the filter's
 * history would otherwise smear across the join and the samples either side of
 * it would differ between a unit that saw the join and one that did not.
 */
void resample_reset(resampler_t *r);

/*
 * Push `n` mono input samples, take up to `max_out` output samples.
 *
 * Returns how many were produced, which varies by one from call to call because
 * the ratio is not an integer -- that is the point of the phase accumulator, and
 * a caller counting output samples to derive time must count what it is GIVEN
 * rather than what it expected.
 *
 * Never produces more than `max_out`; anything beyond that is DROPPED, not held,
 * and the caller must size its buffer so it cannot happen -- see
 * resample_max_out(). Silently discarding samples here would put a unit's
 * decimated stream permanently out of step with its neighbours' by the amount
 * it lost.
 */
int resample_push(resampler_t *r, const int16_t *in, int n,
                  int16_t *out, int max_out);

/* The most samples `resample_push` can return for `n` inputs -- for sizing the
 * caller's buffer so the drop above cannot occur. */
int resample_max_out(const resampler_t *r, int n);

/* A checksum over the filter table, so a build whose table differs from the one
 * the tuning was measured against says so rather than being subtly wrong. */
uint32_t resample_table_checksum(const resampler_t *r);

#ifdef __cplusplus
}
#endif
