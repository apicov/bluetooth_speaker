
#include <stdint.h>

#include "audio_shift.h"

void audio_shift_chunk(int16_t *dst, const int16_t *src, unsigned frames,
                       int shift, unsigned fade, unsigned channels)
{
    const unsigned mag = (unsigned)(shift < 0 ? -shift : shift);
    const unsigned q = frames - mag;
    const unsigned f0 = q - fade;

    for (unsigned i = 0; i < f0; i++) {
        for (unsigned c = 0; c < channels; c++)
            dst[i * channels + c] = src[i * channels + c];
    }

    for (unsigned j = 0; j < fade; j++) {
        const unsigned ia = f0 + j;
        const unsigned ib = ia + (unsigned)shift;
        const int32_t wb = (int32_t)j;
        const int32_t wa = (int32_t)(fade - 1) - wb;
        for (unsigned c = 0; c < channels; c++) {
            const int32_t a = src[ia * channels + c];
            const int32_t b = src[ib * channels + c];
            dst[ia * channels + c] =
                (int16_t)((a * wa + b * wb) / (int32_t)(fade - 1));
        }
    }

    for (unsigned i = q; i < frames; i++) {
        for (unsigned c = 0; c < channels; c++)
            dst[i * channels + c] = src[(i + (unsigned)shift) * channels + c];
    }
}
