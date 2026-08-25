
#pragma once

#include <stdint.h>

#include "analyser.hpp"
#include "analysis_config.h"
#include "beat_detect.h"

namespace df {

constexpr int FFT_N    = DF_FFT_N;
constexpr int HOP_N    = DF_HOP_N;
constexpr int TAIL_N   = DF_TAIL_N;
constexpr int RATE     = 44100;
constexpr int BINS     = FFT_N / 2;
constexpr int CHANNELS = 2;

constexpr int BAND_EDGE_HZ[] = { 43, 172, 1034, 5039 };
constexpr int BEAT_BANDS_N = (int)(sizeof(BAND_EDGE_HZ) / sizeof(BAND_EDGE_HZ[0]));
static_assert(BEAT_BANDS_N == BEAT_BANDS, "one edge per band");

constexpr int band_bin(int hz, int rate)
{
    return (hz * FFT_N + rate / 2) / rate;
}

static_assert(band_bin(BAND_EDGE_HZ[0], RATE) == 1,   "band 0 moved");
static_assert(band_bin(BAND_EDGE_HZ[1], RATE) == 4,   "band 1 moved");
static_assert(band_bin(BAND_EDGE_HZ[2], RATE) == 24,  "band 2 moved");
static_assert(band_bin(BAND_EDGE_HZ[3], RATE) == 117, "band 3 moved");

constexpr float SPEC_LO_HZ  = 40.0f;
constexpr float SPEC_HI_HZ  = 16000.0f;

constexpr float   BOOM_THRESHOLD_K  = 1.4f;
constexpr float   BOOM_FLUX_FLOOR   = 0.02f;
constexpr int64_t BOOM_REFRACTORY_US = 200000;

struct Frame {
    int64_t      index;
    int64_t      due_us;

    const float *mag;

    float        band[BEAT_BANDS];

    uint8_t      spec[SPEC_BINS];
    float        flux;
    float        threshold;
    bool         onset;
    float        strength;

    uint8_t      unit;

    bool         boom;
    float        boom_strength;
    float        boom_flux;
    float        boom_threshold;

    Result       ml[ML_SLOTS];
};

class Pattern {
public:
    virtual ~Pattern() = default;
    virtual const char *name() const = 0;

    virtual void render(const Frame &f, uint8_t *rgb, uint32_t count) = 0;

    virtual void reset() {}
};

class Analysis {
public:

    void init(int sample_rate);

    void set_boom_tuning(float k, float flux_floor, int64_t refractory_us);

    void set_beat_floor(float flux_floor);

    const Frame &process(const int16_t *stereo, int64_t index,
                         int64_t due_us, uint8_t unit);

private:
    alignas(16) float buf_[FFT_N * 2];
    float      win_[FFT_N];
    float      mag_[BINS];
    float      band_[BEAT_BANDS];
    int        band_lo_[BEAT_BANDS];
    int        band_hi_[BEAT_BANDS];
    int        spec_lo_[SPEC_BINS];
    int        spec_hi_[SPEC_BINS];
    beat_det_t beat_;
    beat_det_t boom_;
    Frame      frame_;
};

class RemoteDetect {
public:
    void init();

    void process(const float band[BEAT_BANDS], int64_t due_us, Frame *f);

private:
    beat_det_t beat_;
    beat_det_t boom_;
};

}
