#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sdkconfig.h"
#include "sync_proto.h"

#if CONFIG_DANCEFLOOR_OUT_LEFT
#define AUDIO_CHANNEL_MODE_NAME "left"
#elif CONFIG_DANCEFLOOR_OUT_RIGHT
#define AUDIO_CHANNEL_MODE_NAME "right"
#elif CONFIG_DANCEFLOOR_OUT_MONO
#define AUDIO_CHANNEL_MODE_NAME "mono"
#else
#define AUDIO_CHANNEL_MODE_NAME "stereo"
#endif

static inline void audio_apply_channel_mode(int16_t *frames, size_t n_frames)
{
#if CONFIG_DANCEFLOOR_OUT_LEFT || CONFIG_DANCEFLOOR_OUT_RIGHT || CONFIG_DANCEFLOOR_OUT_MONO
    for (size_t i = 0; i < n_frames; i++) {
        int16_t *f = &frames[i * AUDIO_CHANNELS];
#if CONFIG_DANCEFLOOR_OUT_LEFT
        f[1] = f[0];
#elif CONFIG_DANCEFLOOR_OUT_RIGHT
        f[0] = f[1];
#else

        const int16_t s = (int16_t)(((int32_t)f[0] + (int32_t)f[1]) / 2);
        f[0] = s;
        f[1] = s;
#endif
    }
#else
    (void)frames;
    (void)n_frames;
#endif
}

static inline int32_t audio_volume_q16(uint8_t vol)
{
    static const int32_t taper[AUDIO_VOL_MAX + 1] = {
         0,     66,     69,     73,     77,     82,     86,     91,
        96,    102,    107,    113,    120,    127,    134,    141,
       149,    158,    166,    176,    186,    196,    207,    219,
       231,    244,    258,    273,    288,    304,    321,    339,
       359,    379,    400,    423,    446,    472,    498,    526,
       556,    587,    620,    655,    692,    731,    773,    816,
       862,    911,    962,   1016,   1073,   1134,   1198,   1265,
      1337,   1412,   1491,   1576,   1664,   1758,   1857,   1962,
      2072,   2189,   2313,   2443,   2581,   2726,   2880,   3042,
      3213,   3394,   3586,   3788,   4001,   4227,   4465,   4717,
      4982,   5263,   5560,   5873,   6204,   6554,   6923,   7313,
      7725,   8161,   8620,   9106,   9619,  10161,  10734,  11339,
     11978,  12653,  13366,  14119,  14915,  15756,  16643,  17581,
     18572,  19619,  20724,  21892,  23126,  24429,  25806,  27260,
     28796,  30419,  32133,  33944,  35857,  37878,  40012,  42267,
     44649,  47165,  49823,  52631,  55597,  58730,  62040,  65536,
    };
    return taper[vol > AUDIO_VOL_MAX ? AUDIO_VOL_MAX : vol];
}

static inline int32_t audio_volume_q15(uint8_t vol)
{
    return audio_volume_q16(vol) >> 1;
}

static inline void audio_apply_volume(int16_t *frames, size_t n_frames, uint8_t vol)
{
    if (vol >= AUDIO_VOL_MAX) {
        return;
    }
    const int32_t g = audio_volume_q15(vol);
    const size_t n = n_frames * AUDIO_CHANNELS;
    for (size_t i = 0; i < n; i++) {
        frames[i] = (int16_t)(((int32_t)frames[i] * g) >> 15);
    }
}

typedef int32_t audio_out_sample_t;
#define AUDIO_OUT_FRAME_BYTES (AUDIO_CHANNELS * (int)sizeof(audio_out_sample_t))
#define AUDIO_OUT_CHUNK_BYTES (AUDIO_FRAMES * AUDIO_OUT_FRAME_BYTES)

typedef struct {
    int32_t cur;
} audio_ramp_t;

#define AUDIO_RAMP_STEP_Q16 8192

static inline void audio_volume_write_i32(audio_out_sample_t *out,
                                          const int16_t *in,
                                          size_t n_frames, uint8_t vol,
                                          audio_ramp_t *ramp)
{
    const int32_t target = audio_volume_q16(vol);
    const int32_t from = ramp->cur;
    int32_t step = target - from;
    if (step > AUDIO_RAMP_STEP_Q16) {
        step = AUDIO_RAMP_STEP_Q16;
    } else if (step < -AUDIO_RAMP_STEP_Q16) {
        step = -AUDIO_RAMP_STEP_Q16;
    }
    ramp->cur = from + step;

    for (size_t f = 0; f < n_frames; f++) {
        const int32_t g = from + (int32_t)(step * (int32_t)f / (int32_t)n_frames);
        for (size_t c = 0; c < AUDIO_CHANNELS; c++) {
            const size_t i = f * AUDIO_CHANNELS + c;
            out[i] = (int32_t)in[i] * g;
        }
    }
}

static inline uint8_t audio_vol_effective(uint8_t vol, bool known, bool fallback_due)
{
    if (known) {
        return vol;
    }
    return fallback_due ? AUDIO_VOL_MAX : 0;
}

#define AUDIO_VOL_UNKNOWN_HOLD_US 30000000
