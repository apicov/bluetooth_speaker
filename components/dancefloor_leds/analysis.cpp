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
 * The bands, in the order the detector weights them:
 *   0: kick            1: low-mid / snare body
 *   2: presence        3: air / hats
 *
 * The edges are in Hz now and the bins come from the stream's rate -- see
 * BAND_EDGE_HZ in analysis.hpp. At 44.1 kHz they are { 1, 4, 24, 117 }, which is
 * what they have always been. Bin 0 is DC and deliberately excluded, which is
 * why band 0 starts at 43 Hz rather than at zero.
 */

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

void Analysis::init(int sample_rate)
{
    /*
     * Bands from the rate the audio is actually at.
     *
     * Contiguous by construction: each band runs to the bin below the next one's
     * start, so no bin is counted twice or missed, and the top band runs to
     * Nyquist. A rate low enough to push an edge past Nyquist would fold the
     * bands into each other, so they are clamped -- at 16 kHz, the lowest the
     * bridge advertises, the top edge is bin 323 of 512 and nothing collapses.
     */
    if (sample_rate <= 0) {
        sample_rate = RATE;
    }

    /*
     * Nothing here fills Frame::ml -- the analysers are a separate stage and
     * their answers are put in by whoever runs them. But a frame handed
     * straight to a pattern, which is what tools/pattern_lab does, never passes
     * through the firmware's enqueue() that would otherwise set every slot. So
     * an untouched slot would arrive holding whatever the object was born with,
     * and zero happens to read as "analyser 0 said something" rather than as
     * "nothing has been said" -- a plausible-looking result no consumer could
     * distinguish from a real one.
     */
    for (int i = 0; i < ML_SLOTS; i++) {
        frame_.ml[i] = result_none();
    }

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
     * The portable spectrum's bins, geometrically spaced -- see SPEC_BINS.
     *
     * Edges from a ratio rather than a table so the spacing survives a rate
     * change: the frequencies are fixed, the bins they land on are not. Clamped
     * to Nyquist for the same reason the bands are, and never empty, so a bin
     * narrower than the FFT's resolution reads one bin rather than none. Several
     * of the low bins reading the same underlying bin is expected.
     */
    {
        const float ratio = std::pow(SPEC_HI_HZ / SPEC_LO_HZ, 1.0f / SPEC_BINS);
        float edge = SPEC_LO_HZ;
        for (int s = 0; s < SPEC_BINS; s++) {
            const float next = edge * ratio;
            int lo = band_bin(static_cast<int>(edge + 0.5f), sample_rate);
            int hi = band_bin(static_cast<int>(next + 0.5f), sample_rate) - 1;
            if (lo < 1) lo = 1;                      /* never DC */
            if (lo > BINS - 1) lo = BINS - 1;
            if (hi > BINS - 1) hi = BINS - 1;
            if (hi < lo) hi = lo;
            spec_lo_[s] = lo;
            spec_hi_[s] = hi;
            edge = next;
        }
    }

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
    boom_.threshold_k   = BOOM_THRESHOLD_K;
    boom_.refractory_us = BOOM_REFRACTORY_US;

    /*
     * The flux floor, measured against ten forró recordings AT HOP 1024, and
     * re-measured at hop 512 against those ten plus 196 more. It did not move.
     * Full ladders, corpus and method in docs/tuning-corpus.md; this comment is
     * no longer the only record, which is what made the last retune expensive.
     *
     * It was 0.15 on the strength of synthetic material, and that was wrong by
     * an order of magnitude and in the wrong direction -- on real tracks it
     * fired 0 to 30 times a minute, three of them exactly zero, and a floor
     * left the strip dark.
     *
     * The synthetic signal had the drum going from silence to a stroke, so band
     * 0 swung the whole way and the flux was huge. Real forró has continuous
     * bass under everything -- baixo, accordion left hand, the drum itself
     * ringing -- so band 0 sits around 0.03 and a stroke RISES from there.
     * beat_normalise() compresses what is already loud, so the rise is smaller
     * still. Measured across the recordings: median low-band flux 0.0000, p90
     * 0.016 to 0.019, p99 0.038 to 0.131. A floor of 0.15 is above nearly every
     * stroke in the material.
     *
     * Swept over the ten tracks at hop 1024, booms per minute against floor:
     *
     *   0.15   0-16      dark, the bug
     *   0.06   0-80      still clipping most tracks
     *   0.03   50-114
     *   0.02   68-134    plateau begins; the adaptive threshold takes over
     *   0.012  82-139    ~10% more, and 3/min on a drumless passage
     *
     * 0.02 is the conservative end of that plateau: every track lands in a rate
     * consistent with one stroke per beat at forró tempo, and a synthetic
     * passage of dither, quiet accordion and triangle with no drum at all
     * produces exactly zero. It is also, coincidentally, the wideband
     * detector's own floor -- so the thing that makes this detector selective
     * is not the floor at all. It is the single-band input and the adaptive
     * threshold above it.
     *
     * At hop 512 that last sentence is not a remark, it is the result. Over 196
     * tracks, sweeping this floor across a factor of five moves the boom rate by
     * about 1% (98-201 per minute at 0.02, 97-200 at 0.028) and the near-
     * threshold frame rate not at all. The floor is simply not the binding
     * constraint on real music; it binds on quiet material and on nothing else,
     * and no value of it recovers the near-threshold rate that overlapping
     * windows raised from 2.2% to 2.9% of frames. So it stays where the ten
     * tracks put it.
     *
     * Lower it toward 0.012 if the lights miss strokes; that costs occasional
     * firing where there is no drum. At hop 512 that cost arrives sooner than
     * this ladder suggests -- 0.012 is the one rung the drumless control still
     * rejects there.
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
        /* Averaged over the bins in the band, not summed, so a band that holds
         * a different number of bins at a different rate still lands in the
         * same range -- the gain below stays a property of the music. */
        const float v = (sum / static_cast<float>(band_hi_[b] - band_lo_[b] + 1))
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
    /*
     * The portable spectrum, on exactly the scale band[] uses -- same averaging
     * over the bins of a bin, same BAND_GAIN, same beat_normalise() compression
     * -- so a pattern can read one against the other without a conversion, and
     * so nothing here needs a second empirical constant.
     *
     * Quantised last, and only here. 8 bits over a 0..1 range is a step of
     * 0.004, two orders below anything an eye resolves on a strip and well below
     * the difference between two units' own measurements of the same audio.
     */
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

/*
 * The remote half, in the same file as the half it mirrors so the two cannot
 * drift apart unnoticed -- a divergence here is a sync fault, not a local
 * quality difference. See analysis.hpp for what runs where and why.
 */
void RemoteDetect::init()
{
    beat_det_init(&beat_);

    /* The same three overrides Analysis::init() applies, for the same stated
     * reasons -- the quieter single-band input, the drum's pulse rather than
     * every stroke, and the floor the corpus measured. The reasoning lives
     * there; this is the copy that must not disagree. */
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

    /* The low band alone, masked rather than reweighted -- the same lines as
     * Analysis::process(): a band held at zero has a rise of exactly zero, so
     * the weighted sum reduces to the bass band times its weight of 1.0, and
     * the threshold adapts to the bass band's own statistics. */
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

}  // namespace df
