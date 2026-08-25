
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BEAT_BANDS 4

#ifndef BEAT_HIST
#  define BEAT_HIST  43
#endif

#define BEAT_FLUX_FLOOR 0.02f

#define BEAT_REFRACTORY_US 120000

typedef struct {
    float last_flux;
    float last_threshold;
    float prev[BEAT_BANDS];
    float hist[BEAT_HIST];
    int hist_n;
    int hist_next;
    int64_t last_onset_us;
    bool primed;

    float   threshold_k;
    int64_t refractory_us;
    float   flux_floor;
} beat_det_t;

float beat_normalise(float raw);

void beat_det_init(beat_det_t *d);

bool beat_det_update(beat_det_t *d, const float band[BEAT_BANDS],
                     int64_t now_us, float *strength);

float beat_det_last_flux(const beat_det_t *d);
float beat_det_last_threshold(const beat_det_t *d);

#ifdef __cplusplus
}
#endif
