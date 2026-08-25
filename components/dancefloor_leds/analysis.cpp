/**
 * @file analysis.cpp
 * @brief The analysis pipeline: audio in, one Frame out, and the remote half
 *        that derives the same Frame fields from bands it did not compute
 *        itself.
 *
 * TWO IMPLEMENTATIONS OF ONE THING, deliberately in one file.
 * Analysis::process() is the local path -- window, FFT, bands, onset -- and
 * RemoteDetect::process() is what a unit runs when the bands arrived over the
 * air instead of coming out of its own audio feed. Keeping them adjacent is
 * the point: the two must agree field for field, and a divergence between them
 * shows up as strips that disagree across a floor, which is a SYNC fault
 * rather than a quality difference. Anything added to one belongs in the other
 * in the same commit.
 *
 * Pure functions of the audio and the shared timeline -- no clock, no task, no
 * strip -- so the host harness compiles this unchanged and gets the same
 * lights. analysis.hpp states the rule that depends on it, and the window and
 * hop it is cut by are in analysis_config.h.
 */
#include "analysis.hpp"

#include <cmath>

#ifndef M_PI
/** @brief Not guaranteed by the C++ math header under a strict standard mode. */
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
 * The bands, in the order the detector weights them:
 *   0: kick            1: low-mid / snare body
 *   2: presence        3: air / hats
 *
 * The edges are in Hz and the bins come from the stream's rate -- see
 * BAND_EDGE_HZ. Bin 0 is DC and deliberately excluded, which is why band 0
 * starts above zero.
 */

/**
 * @brief Scales raw magnitudes towards the 0..1 the detector wants.
 *
 * Empirical, and set against synthetic kicks rather than real music -- the
 * whole point of the host harness is to replace it with a number derived from
 * a recording.
 *
 * It is no longer critical: beat_normalise() is monotonic over the whole input
 * range, so a wrong value here costs SENSITIVITY rather than deleting the
 * signal, which is what a hard clamp did.
 */
constexpr float BAND_GAIN = 12.0f;

/** @brief Prepare the transform. A no-op off-target, where the host FFT needs
 *         no tables. */
void fft_init()
{
#ifdef ESP_PLATFORM
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(nullptr, FFT_N));
#endif
}

/**
 * @brief Transform one window in place, leaving natural bin order.
 * @param buf  Complex interleaved, 2*FFT_N floats.
 */
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

    /*
     * Nothing here fills Frame::ml -- the analysers are a separate stage and
     * their answers are put in by whoever runs them. But a frame handed
     * straight to a pattern, which is what the host harness does, never passes
     * through the firmware's enqueue that would otherwise set every slot. So
     * an untouched slot would arrive holding whatever the object was born
     * with, and zero happens to read as "analyser 0 said something" rather
     * than as "nothing has been said" -- a plausible-looking result no
     * consumer could distinguish from a real one.
     */

    for (int i = 0; i < ML_SLOTS; i++) {
        frame_.ml[i] = result_none();
    }

    /*
     * Bands from the rate the audio is ACTUALLY at.
     *
     * Contiguous by construction: each band runs to the bin below the next
     * one's start, so no bin is counted twice or missed, and the top band runs
     * to Nyquist. A rate low enough to push an edge past Nyquist would fold
     * the bands into each other, so they are clamped -- at the lowest rate the
     * bridge advertises the top edge is still well inside the range and
     * nothing collapses.
     */
    for (int b = 0; b < BEAT_BANDS; b++) {
        int lo = band_bin(BAND_EDGE_HZ[b], sample_rate);
        if (lo < 1) lo = 1;                          /* never DC */
        if (lo > BINS - 1) lo = BINS - 1;
        band_lo_[b] = lo;
    }
    for (int b = 0; b < BEAT_BANDS; b++) {
        band_hi_[b] = (b + 1 < BEAT_BANDS) ? band_lo_[b + 1] - 1 : BINS - 1;
        if (band_hi_[b] < band_lo_[b]) {
            band_hi_[b] = band_lo_[b];               /* a band is never empty */
        }
    }

    /*
     * The portable spectrum's bins, geometrically spaced -- see SPEC_LO_HZ.
     *
     * Edges from a RATIO rather than a table, so the spacing survives a rate
     * change: the frequencies are fixed, the bins they land on are not.
     * Clamped to Nyquist for the same reason the bands are, and never empty,
     * so a bin narrower than the FFT's resolution reads one bin rather than
     * none. Several of the low bins reading the same underlying bin is
     * expected.
     */
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

    /* Exactly esp-dsp's own Hann, so the host and the board window identically
     * -- symmetric, not the periodic variant. */
    const float m = 1.0f / static_cast<float>(FFT_N - 1);
    for (int i = 0; i < FFT_N; i++) {
        win_[i] = 0.5f * (1.0f - std::cos(static_cast<float>(i) * 2.0f * float(M_PI) * m));
    }
    fft_init();
    beat_det_init(&beat_);

    /*
     * The boom detector, tuned differently from the wideband one.
     *
     * Its input is ONE band rather than four, so its flux is smaller and less
     * noisy -- there is no triangle or snare adding variance to the history
     * the threshold is built from. A lower multiplier therefore does not mean
     * a twitchier detector; it means the same sensitivity applied to a quieter
     * signal.
     *
     * The refractory is longer because the target is the PULSE of the drum,
     * not every stroke of it: it admits the boom pattern while suppressing the
     * faster fills that would otherwise make the lights busier than the dance.
     */
    beat_det_init(&boom_);
    boom_.threshold_k   = BOOM_THRESHOLD_K;
    boom_.refractory_us = BOOM_REFRACTORY_US;
    /*
     * The flux floor, swept over a corpus of real recordings at both hops.
     *
     * It was an order of magnitude higher on the strength of SYNTHETIC
     * material, and that was wrong in the direction that leaves the strip
     * dark. The synthetic signal had the drum going from silence to a stroke,
     * so the low band swung the whole way and the flux was huge. Real material
     * has continuous bass under everything -- bass instrument, accordion left
     * hand, the drum itself ringing -- so the low band sits well above zero
     * and a stroke RISES from there, and beat_normalise() compresses what is
     * already loud, so the rise is smaller still. A floor set from the
     * synthetic case is above nearly every stroke in real music.
     */
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
    /* Downmix, scale and window in one pass. The divisor is two full-scale
     * int16 samples, so `mono` is in -1..1 before the window. Must stay the
     * same downmix df::downmix() does -- two front ends disagreeing about what
     * mono means is a difference nothing downstream could see. */
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

    /* Mean magnitude per band, normalised for the transform length so the
     * numbers do not move with FFT_N, then compressed onto 0..1. */
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

    /* The boom detector sees the LOWEST band alone, with the rest zeroed --
     * see Frame::boom. Zeros contribute no flux, so this is the same detector
     * looking at one band rather than a different algorithm. */
    float boom_band[BEAT_BANDS] = {0};
    boom_band[0] = band_[0];
    float boom_strength = 0.0f;
    const bool boom = beat_det_update(&boom_, boom_band, due_us, &boom_strength);

    frame_.index     = index;
    frame_.due_us    = due_us;

    /* The portable spectrum, on the same scale as the bands and quantised to a
     * byte -- what every analyser is handed, and what a unit taking frames off
     * the radio unpacks. See df::Analyser::process(). */
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

    /* By pointer, and valid only until the next call -- see Frame::mag. */
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
    /* The same two detectors with the same tuning as Analysis::init() sets. If
     * one of these lines changes, the other file's copy changes in the same
     * commit -- that is the whole point of the two being adjacent. */
    beat_det_init(&beat_);

    beat_det_init(&boom_);
    boom_.threshold_k   = BOOM_THRESHOLD_K;
    boom_.refractory_us = BOOM_REFRACTORY_US;
    boom_.flux_floor    = BOOM_FLUX_FLOOR;
}

void RemoteDetect::process(const float band[BEAT_BANDS], int64_t due_us,
                           Frame *f)
{
    /* Line for line the second half of Analysis::process(), on bands this unit
     * received rather than computed. Identical input bytes through the same
     * plain-C detector give identical decisions. */
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
