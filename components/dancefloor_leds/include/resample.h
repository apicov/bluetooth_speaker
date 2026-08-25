
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RESAMPLE_TAPS       32
#define RESAMPLE_PHASE_BITS 6
#define RESAMPLE_PHASES     (1 << RESAMPLE_PHASE_BITS)

typedef struct {
    int      in_rate;
    int      out_rate;
    uint64_t step;
    uint64_t phase;
    int16_t  taps[RESAMPLE_PHASES][RESAMPLE_TAPS];
    int16_t  hist[RESAMPLE_TAPS];
    int      ready;
} resampler_t;

int resample_init(resampler_t *r, int in_rate, int out_rate);

void resample_reset(resampler_t *r);

int resample_push(resampler_t *r, const int16_t *in, int n,
                  int16_t *out, int max_out);

int resample_max_out(const resampler_t *r, int n);

uint32_t resample_table_checksum(const resampler_t *r);

#ifdef __cplusplus
}
#endif
