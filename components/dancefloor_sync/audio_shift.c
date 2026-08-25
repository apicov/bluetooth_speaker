/**
 * @file audio_shift.c
 * @brief The crossfade that hides a catch-up step. One function; see
 *        audio_shift.h for why it lives in the component and what the caller
 *        must guarantee about its arguments.
 *
 * The arithmetic is laid out for shift > 0 (drop) and reached for shift < 0
 * (insert) by the same expressions, because they ARE the same expressions: in
 * both cases the tail of the output comes from src[i+shift] and the fade
 * window straddles the handover. Keeping one path is what makes the two
 * directions provably mirror images -- a drop skips material, an insert
 * replays it, and nothing else differs.
 */
#include <stdint.h>

#include "audio_shift.h"

void audio_shift_chunk(int16_t *dst, const int16_t *src, unsigned frames,
                       int shift, unsigned fade, unsigned channels)
{
    const unsigned mag = (unsigned)(shift < 0 ? -shift : shift);
    const unsigned q = frames - mag;   /* output frame where the fade ends */
    const unsigned f0 = q - fade;      /* ...and where it begins */

    /* Plain head: the strand the output is currently on. */
    for (unsigned i = 0; i < f0; i++) {
        for (unsigned c = 0; c < channels; c++)
            dst[i * channels + c] = src[i * channels + c];
    }

    /* The crossfade: j/fade is the weight of the shifted strand. Equal-ramp
     * (linear against linear), so the two weights sum to one and a signal
     * crossed with itself at a small offset comes out at roughly its own level
     * -- no dip to hear. Same weight for every channel of a frame; they are
     * one sample of the same moment and must move together. */
    for (unsigned j = 0; j < fade; j++) {
        const unsigned ia = f0 + j;               /* this strand's frame    */
        const unsigned ib = ia + (unsigned)shift; /* the other, |shift| away */
        const int32_t wb = (int32_t)j;            /* 0 .. fade-1             */
        const int32_t wa = (int32_t)(fade - 1) - wb;
        for (unsigned c = 0; c < channels; c++) {
            const int32_t a = src[ia * channels + c];
            const int32_t b = src[ib * channels + c];
            dst[ia * channels + c] =
                (int16_t)((a * wa + b * wb) / (int32_t)(fade - 1));
        }
    }

    /* Plain tail: the shifted strand, from the end of the fade to the end of
     * the chunk. The next chunk reads on from src[frames+shift], adjacent. */
    for (unsigned i = q; i < frames; i++) {
        for (unsigned c = 0; c < channels; c++)
            dst[i * channels + c] = src[(i + (unsigned)shift) * channels + c];
    }
}
