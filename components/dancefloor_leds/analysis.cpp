
#include "analysis.hpp"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <cstring>

#ifdef ESP_PLATFORM
#include "esp_dsp.h"
#else
extern "C" void df_fft_radix2(float *interleaved, int n);
#endif

namespace df {
namespace {

constexpr float BAND_GAIN = 12.0f;

void fft_init()
{
#ifdef ESP_PLATFORM
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(nullptr, FFT_N));
#endif
}

void fft_run(float *buf)
{
#ifdef ESP_PLATFORM
    dsps_fft2r_fc32(buf, FFT_N);
    dsps_bit_rev_fc32(buf, FFT_N);
#else
    df_fft_radix2(buf, FFT_N);
#endif
}

}

void Analysis::init(int sample_rate)
{

    if (sample_rate <= 0) {
        sample_rate = RATE;
    }

    for (int i = 0; i < ML_SLOTS; i++) {
        frame_.ml[i] = result_none();
    }

    for (int b = 0; b < BEAT_BANDS; b++) {
        int lo = band_bin(BAND_EDGE_HZ[b], sample_rate);
        if (lo < 1) lo = 1;
        if (lo > BINS - 1) lo = BINS - 1;
        band_lo_[b] = lo;
    }
    for (int b = 0; b < BEAT_BANDS; b++) {
        band_hi_[b] = (b + 1 < BEAT_BANDS) ? band_lo_[b + 1] - 1 : BINS - 1;
        if (band_hi_[b] < band_lo_[b]) {
            band_hi_[b] = band_lo_[b];
        }
    }

    {
        const float ratio = std::pow(SPEC_HI_HZ / SPEC_LO_HZ, 1.0f / SPEC_BINS);
        float edge = SPEC_LO_HZ;
        for (int s = 0; s < SPEC_BINS; s++) {
            const float next = edge * ratio;
            int lo = band_bin(static_cast<int>(edge + 0.5f), sample_rate);
            int hi = band_bin(static_cast<int>(next + 0.5f), sample_rate) - 1;
            if (lo < 1) lo = 1;
            if (lo > BINS - 1) lo = BINS - 1;
            if (hi > BINS - 1) hi = BINS - 1;
            if (hi < lo) hi = lo;
            spec_lo_[s] = lo;
            spec_hi_[s] = hi;
            edge = next;
        }
    }

    const float m = 1.0f / static_cast<float>(FFT_N - 1);
    for (int i = 0; i < FFT_N; i++) {
        win_[i] = 0.5f * (1.0f - std::cos(static_cast<float>(i) * 2.0f * float(M_PI) * m));
    }
    fft_init();
    beat_det_init(&beat_);

    beat_det_init(&boom_);
    boom_.threshold_k   = BOOM_THRESHOLD_K;
    boom_.refractory_us = BOOM_REFRACTORY_US;

    boom_.flux_floor = BOOM_FLUX_FLOOR;

    std::memset(&frame_, 0, sizeof(frame_));
}

void Analysis::set_boom_tuning(float k, float flux_floor, int64_t refractory_us)
{
    boom_.threshold_k   = k;
    boom_.flux_floor    = flux_floor;
    boom_.refractory_us = refractory_us;
}

void Analysis::set_beat_floor(float flux_floor)
{
    beat_.flux_floor = flux_floor;
}

const Frame &Analysis::process(const int16_t *stereo, int64_t index,
                               int64_t due_us, uint8_t unit)
{
    for (int i = 0; i < FFT_N; i++) {
        const float mono = (static_cast<float>(stereo[2 * i]) +
                            static_cast<float>(stereo[2 * i + 1])) / 65536.0f;
        buf_[2 * i]     = mono * win_[i];
        buf_[2 * i + 1] = 0.0f;
    }

    fft_run(buf_);

    for (int k = 0; k < BINS; k++) {
        const float re = buf_[2 * k], im = buf_[2 * k + 1];
        mag_[k] = std::sqrt(re * re + im * im);
    }

    for (int b = 0; b < BEAT_BANDS; b++) {
        float sum = 0.0f;
        for (int k = band_lo_[b]; k <= band_hi_[b]; k++) {
            sum += mag_[k];
        }

        const float v = (sum / static_cast<float>(band_hi_[b] - band_lo_[b] + 1))
                        * BAND_GAIN / static_cast<float>(FFT_N) * 2.0f;

        band_[b] = beat_normalise(v);
    }

    float strength = 0.0f;
    const bool onset = beat_det_update(&beat_, band_, due_us, &strength);

    float boom_band[BEAT_BANDS] = {0};
    boom_band[0] = band_[0];
    float boom_strength = 0.0f;
    const bool boom = beat_det_update(&boom_, boom_band, due_us, &boom_strength);

    frame_.index     = index;
    frame_.due_us    = due_us;

    for (int s = 0; s < SPEC_BINS; s++) {
        float sum = 0.0f;
        for (int k = spec_lo_[s]; k <= spec_hi_[s]; k++) {
            sum += mag_[k];
        }
        const float v = (sum / static_cast<float>(spec_hi_[s] - spec_lo_[s] + 1))
                        * BAND_GAIN / static_cast<float>(FFT_N) * 2.0f;
        const float n = beat_normalise(v);
        frame_.spec[s] = static_cast<uint8_t>(n * 255.0f + 0.5f);
    }

    frame_.mag       = mag_;
    std::memcpy(frame_.band, band_, sizeof(frame_.band));
    frame_.flux      = beat_det_last_flux(&beat_);
    frame_.threshold = beat_det_last_threshold(&beat_);
    frame_.onset     = onset;
    frame_.strength  = strength;
    frame_.unit      = unit;
    frame_.boom           = boom;
    frame_.boom_strength  = boom_strength;
    frame_.boom_flux      = beat_det_last_flux(&boom_);
    frame_.boom_threshold = beat_det_last_threshold(&boom_);
    return frame_;
}

void RemoteDetect::init()
{
    beat_det_init(&beat_);

    beat_det_init(&boom_);
    boom_.threshold_k   = BOOM_THRESHOLD_K;
    boom_.refractory_us = BOOM_REFRACTORY_US;
    boom_.flux_floor    = BOOM_FLUX_FLOOR;
}

void RemoteDetect::process(const float band[BEAT_BANDS], int64_t due_us,
                           Frame *f)
{
    float strength = 0.0f;
    const bool onset = beat_det_update(&beat_, band, due_us, &strength);

    float boom_band[BEAT_BANDS] = {0};
    boom_band[0] = band[0];
    float boom_strength = 0.0f;
    const bool boom = beat_det_update(&boom_, boom_band, due_us, &boom_strength);

    f->onset          = onset;
    f->strength       = strength;
    f->flux           = beat_det_last_flux(&beat_);
    f->threshold      = beat_det_last_threshold(&beat_);
    f->boom           = boom;
    f->boom_strength  = boom_strength;
    f->boom_flux      = beat_det_last_flux(&boom_);
    f->boom_threshold = beat_det_last_threshold(&boom_);
}

}
