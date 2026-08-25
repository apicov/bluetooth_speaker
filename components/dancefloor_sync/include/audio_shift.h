
#pragma once

#include <stdint.h>

#define RATE_TRIM_MAX_HZ 100

#define CATCHUP_ARM_US 25000

#define CATCHUP_CLEAR_US 10000

#define CATCHUP_HOLD_US 60000000

#define CATCHUP_MAX_US 150000

#define CATCHUP_SHIFT_MAX_DROP 8
#define CATCHUP_SHIFT_MAX_DUP  4
#define CATCHUP_SHIFT_MAX      CATCHUP_SHIFT_MAX_DROP
#define CATCHUP_FADE_FRAMES    64
_Static_assert(CATCHUP_SHIFT_MAX_DUP <= CATCHUP_SHIFT_MAX,
               "the dup clamp must fit the shared shift-buffer sizing");

#define CATCHUP_HIST_RESET_US 20000

void audio_shift_chunk(int16_t *dst, const int16_t *src, unsigned frames,
                       int shift, unsigned fade, unsigned channels);
