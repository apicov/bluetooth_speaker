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

/*
 * Band edges in FFT bins at 44.1 kHz / 1024 (43.07 Hz per bin):
 *   0: 43-129 Hz    kick
 *   1: 172-990 Hz   low-mid / snare body
 *   2: 1.0-5.0 kHz  presence
 *   3: 5.0-22 kHz   air / hats
 * Bin 0 is DC and deliberately excluded.
 */
constexpr int BAND_LO[BEAT_BANDS] = { 1,  4,  24, 117 };
constexpr int BAND_HI[BEAT_BANDS] = { 3, 23, 116, BINS - 1 };

/*
 * Scales raw magnitudes towards the 0..1 the detector wants. Empirical, and set
 * against synthetic kicks rather than real music -- the whole point of the host
 * harness is to replace it with a number derived from a recording.
 *
 * It is no longer critical: beat_normalise() is monotonic over the whole input
 * range, so a wrong value here costs sensitivity rather than deleting the
 * signal, which is what the old hard clamp did.
 */
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

}  // namespace

void Analysis::init()
{
    /* Exactly esp-dsp's dsps_wind_hann_f32, so the host and the board window
     * identically -- symmetric Hann, not the periodic variant. */
    const float m = 1.0f / static_cast<float>(FFT_N - 1);
    for (int i = 0; i < FFT_N; i++) {
        win_[i] = 0.5f * (1.0f - std::cos(static_cast<float>(i) * 2.0f * float(M_PI) * m));
    }
    fft_init();
    beat_det_init(&beat_);
    std::memset(&frame_, 0, sizeof(frame_));
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
        for (int k = BAND_LO[b]; k <= BAND_HI[b]; k++) {
            sum += mag_[k];
        }
        const float v = (sum / static_cast<float>(BAND_HI[b] - BAND_LO[b] + 1))
                        * BAND_GAIN / static_cast<float>(FFT_N) * 2.0f;
        /* Soft, never a hard clamp: flux counts only increases, so a band
         * pinned at 1.0 has a rise of exactly zero and stops contributing. */
        band_[b] = beat_normalise(v);
    }

    float strength = 0.0f;
    const bool onset = beat_det_update(&beat_, band_, due_us, &strength);

    frame_.index     = index;
    frame_.due_us    = due_us;
    frame_.mag       = mag_;
    frame_.band      = band_;
    frame_.flux      = beat_det_last_flux(&beat_);
    frame_.threshold = beat_det_last_threshold(&beat_);
    frame_.onset     = onset;
    frame_.strength  = strength;
    frame_.unit      = unit;
    return frame_;
}

}  // namespace df
