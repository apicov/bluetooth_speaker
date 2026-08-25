/**
 * @file resample.h
 * @brief Stream rate to model rate, in fixed point.
 *
 * Nearly every published audio model wants 16 kHz mono. The stream is whatever
 * the phone chose -- the bridge advertises several rates and takes what it is
 * given -- so the ratio is a RUNTIME value and is not an integer. Hence an
 * arbitrary-ratio resampler rather than a decimator.
 *
 * WHY FIXED POINT, AND NOT BECAUSE IT IS FASTER. The hub and a satellite are
 * different cores. A unit running a model locally must produce the same answer
 * as its neighbours or the strips separate, and the resampler is upstream of
 * everything -- one sample different here is a different window into the
 * model. Integer arithmetic is identical on both parts by construction; float
 * is identical only as long as nothing selects a different kernel per target,
 * which is not a promise either toolchain makes.
 *
 * The one place a double appears is building the filter table in
 * resample_init(), which runs once. Doubles are soft-float on BOTH parts --
 * neither has a double FPU -- so the library runs the same code and the table
 * comes out the same. That is a real argument rather than a hope, but it is
 * also the kind of argument that quietly stops being true, so
 * test_resample.c pins the table's checksum and fails if it ever moves.
 *
 * Plain C and no dependencies, because three build systems compile it and only
 * one of them is ESP-IDF.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Filter length: the quality against the cost and the RAM.
 *
 * With the phase count below this is a few kilobytes of int16 table and this
 * many multiply-accumulates per output sample -- a fraction of a percent of
 * one core at the rates a model wants.
 *
 * HALF THIS WAS TRIED FIRST and reached only about -30 dB against the
 * fold-back check in test_resample.c, which is close enough to audible content
 * to put energy in front of a model at a frequency nobody played. Doubling the
 * taps costs nothing that matters here and buys roughly 20 dB.
 *
 * Still a feature front end's filter and not a listener's: nothing decimated
 * here is ever played.
 */
#define RESAMPLE_TAPS       32
/** @brief Phases as a power of two -- required, because the phase index is the
 *         top bits of a Q32 accumulator, which is only exact when it is. */
#define RESAMPLE_PHASE_BITS 6
/** @brief Sub-sample positions the filter is precomputed at. */
#define RESAMPLE_PHASES     (1 << RESAMPLE_PHASE_BITS)

/** @brief One resampler. Owned by one task; there is no locking here. */
typedef struct {
    int      in_rate;               /**< As given to resample_init(). */
    int      out_rate;              /**< Likewise. */
    uint64_t step;                  /**< Q32 input samples per output sample. */
    uint64_t phase;                 /**< Q32, distance past the newest input. */
    int16_t  taps[RESAMPLE_PHASES][RESAMPLE_TAPS];   /**< Q15, unity DC gain. */
    int16_t  hist[RESAMPLE_TAPS];   /**< Input history; [0] is newest. */
    int      ready;                 /**< Whether the table has been built. */
} resampler_t;

/**
 * @brief Build the table for one ratio. Call again to change it.
 *
 * @param r         The resampler.
 * @param in_rate   Stream rate.
 * @param out_rate  Model rate. May EXCEED @p in_rate -- the filter then passes
 *                  everything below the input's Nyquist and the result is
 *                  interpolation rather than decimation, which is correct if
 *                  unusual here.
 * @return 0 on success, -1 if either rate is out of range.
 */
int resample_init(resampler_t *r, int in_rate, int out_rate);

/**
 * @brief Forget the history, keep the table.
 *
 * Call whenever the audio about to be pushed does not continue the audio
 * pushed before it -- a splice, a realignment, a timeline restart. The same
 * event that makes visualiser_realign() necessary, and for the same reason:
 * the filter's history would otherwise smear across the join, and the samples
 * either side of it would differ between a unit that saw the join and one that
 * did not.
 *
 * @param r  The resampler.
 */
void resample_reset(resampler_t *r);

/**
 * @brief Push mono input samples and take output samples.
 *
 * @param r        The resampler.
 * @param in       Mono input.
 * @param n        How many input samples.
 * @param[out] out Output samples.
 * @param max_out  Room in @p out.
 * @return How many output samples were produced. This VARIES BY ONE from call
 *         to call, because the ratio is not an integer -- that is the point of
 *         the phase accumulator, and a caller deriving time from an output
 *         count must count what it is GIVEN rather than what it expected.
 *
 * Never produces more than @p max_out; anything beyond that is DROPPED, not
 * held, and the caller must size its buffer so it cannot happen -- see
 * resample_max_out(). Silently discarding samples here would put a unit's
 * decimated stream permanently out of step with its neighbours' by the amount
 * it lost.
 */
int resample_push(resampler_t *r, const int16_t *in, int n,
                  int16_t *out, int max_out);

/**
 * @brief The most resample_push() can return for a given input count, for
 *        sizing the caller's buffer so the drop above cannot occur.
 * @param r  The resampler.
 * @param n  Input samples that will be pushed.
 * @return The ceiling on output samples.
 */
int resample_max_out(const resampler_t *r, int n);

/**
 * @brief A checksum over the filter table, so a build whose table differs from
 *        the one the tuning was measured against says so rather than being
 *        subtly wrong.
 * @param r  The resampler.
 * @return The checksum, or 0 if no table has been built.
 */
uint32_t resample_table_checksum(const resampler_t *r);

#ifdef __cplusplus
}
#endif
