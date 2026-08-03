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

    /*
     * The boom detector, tuned differently from the wideband one.
     *
     * Its input is one band rather than four, so its flux is smaller and less
     * noisy -- there is no triangle or snare adding variance to the history the
     * threshold is built from. A lower k therefore does not mean a twitchier
     * detector; it means the same sensitivity applied to a quieter signal.
     *
     * The refractory is longer because the target is the pulse of the drum, not
     * every stroke of it. At 140 BPM a quaver is 214 ms, so 200 ms admits the
     * boom pattern while suppressing the semiquaver fills that would otherwise
     * make the lights busier than the dance.
     *
     * Both are first guesses and both are meant to be measured -- pattern_lab
     * prints boom_flux against boom_threshold per frame for exactly that.
     */
    beat_det_init(&boom_);
    boom_.threshold_k   = 1.4f;
    boom_.refractory_us = 200000;

    /*
     * A much higher flux floor than the wideband detector's 0.02, and this is
     * the constant that makes the difference between finding the drum and
     * following whatever else is happening.
     *
     * A sharp transient anywhere leaks into the low bins -- a windowed FFT
     * cannot prevent it -- so accordion stabs raise band 0 slightly even with no
     * drum present. The floor exists to stop the adaptive threshold collapsing
     * onto that when there is no real bass, and 0.02 was chosen for a weighted
     * sum of four bands, not for one band on its own.
     *
     * Measured on synthetic forró: with a zabumba, booms fire at a flux of
     * 0.169 to 0.488; with the drum removed entirely and only accordion and
     * triangle left, the leakage fires at 0.022 to 0.114. 0.15 sits in the gap.
     *
     * That gap is real but its POSITION depends on how loud the material is,
     * since flux scales with BAND_GAIN and the mix. It is a synthetic number
     * until it has been checked against a recording, and it is the first thing
     * to move if the lights miss quiet drums or fire during quiet passages.
     */
    boom_.flux_floor = 0.15f;

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

    /*
     * The same detector over the low band alone. Masking the upper three to
     * zero rather than reweighting: flux counts increases, and a band held at
     * zero has a rise of exactly zero, so the weighted sum reduces to the bass
     * band times its weight of 1.0. No new code path, and the adaptive
     * threshold then adapts to the bass band's own statistics instead of to a
     * mixture dominated by whatever the triangle is doing.
     */
    /*
     * The low band alone. Masking the upper three to zero rather than
     * reweighting: flux counts increases, and a band held at zero has a rise of
     * exactly zero, so the weighted sum reduces to the bass band times its
     * weight of 1.0.
     */
    float boom_band[BEAT_BANDS] = {0};
    boom_band[0] = band_[0];
    float boom_strength = 0.0f;
    const bool boom = beat_det_update(&boom_, boom_band, due_us, &boom_strength);

    frame_.index     = index;
    frame_.due_us    = due_us;
    frame_.mag       = mag_;
    frame_.band      = band_;
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

}  // namespace df
