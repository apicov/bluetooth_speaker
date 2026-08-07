/*
 * The analyser registry, and the one analyser that needs no model.
 *
 * Same shape as patterns.cpp: a static table, no dynamic registration. Adding
 * an analyser means adding a class here and an entry in s_analysers[].
 */
#include "analyser.hpp"

#include <cstring>

#include "analysis_config.h"

namespace df {
namespace {

/*
 * Integer square root, 64-bit in and 32-bit out.
 *
 * Integer rather than std::sqrt because of the rule at the top of analyser.hpp.
 * A double sqrt would in fact be identical on both parts -- IEEE-754 requires
 * it to be correctly rounded, and neither chip has a double FPU so both use the
 * same soft-float -- but writing that reasoning into every analyser is how it
 * eventually gets it wrong. Integer arithmetic needs no such argument.
 *
 * Newton's method from a bit-length estimate; converges in a handful of
 * iterations for anything in range and terminates exactly.
 */
uint32_t isqrt64(uint64_t v)
{
    if (v == 0) return 0;
    uint64_t x = v, y = (x + 1) / 2;
    while (y < x) {
        x = y;
        y = (x + v / x) / 2;
    }
    return static_cast<uint32_t>(x);
}

/*
 * Level and brightness, from the time domain alone.
 *
 * This is not a model and does not pretend to be one. It exists so that the
 * whole path -- registry, lane, latch, wire format, pattern -- can be built and
 * proved with an analyser that has no arena, no weights and no dependencies, so
 * that when a real model is dropped in the only new thing to debug is the
 * model. It is also a genuinely useful pair of features: RMS and zero-crossing
 * rate are the oldest two in audio classification and still the first two most
 * front ends compute.
 *
 * All-integer, so it is bit-identical on an LX6 and an LX7 by construction
 * rather than by argument.
 *
 * Fast lane: it is a single pass over the window that df::Analysis has just
 * transformed anyway, which is nothing beside the FFT sharing the task.
 */
class ZcrRms final : public Analyser {
public:
    const AnalyserSpec &spec() const override { return spec_; }

    bool init(int stream_rate_hz) override
    {
        rate_ = stream_rate_hz > 0 ? stream_rate_hz : DF_RATE_FALLBACK;
        return true;
    }

    bool process(const int16_t *in, int64_t index, int64_t due_us,
                 Result *out) override
    {
        (void)due_us;

        uint64_t sum_sq = 0;
        uint32_t crossings = 0;
        for (int i = 0; i < DF_FFT_N; i++) {
            const int32_t s = in[i];
            sum_sq += static_cast<uint64_t>(s * s);
            /* A crossing is a sign change between consecutive samples. Zero
             * counts as positive so that silence reports no crossings rather
             * than one per sample. */
            if (i > 0 && ((in[i - 1] < 0) != (s < 0))) {
                crossings++;
            }
        }

        const uint32_t rms = isqrt64(sum_sq / DF_FFT_N);

        /*
         * Crossings expressed as a frequency, so the thresholds below mean the
         * same thing at 16, 32, 44.1 and 48 kHz -- all four of which the bridge
         * advertises to the phone. A rate-dependent threshold here would have
         * been a repeat of the band-edge bug analysis.hpp records.
         */
        const uint32_t zcr_hz =
            static_cast<uint32_t>(static_cast<uint64_t>(crossings) * rate_ / DF_FFT_N);

        uint8_t label;
        if (rms < QUIET_RMS)         label = LABEL_QUIET;
        else if (zcr_hz < LOW_HZ)    label = LABEL_LOW;
        else if (zcr_hz < BRIGHT_HZ) label = LABEL_MID;
        else                         label = LABEL_BRIGHT;

        /* Level, as a 0..255 the strip can use directly. Linear in amplitude,
         * not in dB: a pattern wanting a curve can apply its own, and doing it
         * here would bake one choice into everything downstream. */
        const uint32_t level = rms > 32767 ? 255u : (rms * 255u) / 32767u;

        out->index = index;
        out->n = 1;
        out->label[0] = label;
        out->score[0] = static_cast<uint8_t>(level);
        return true;
    }

    const char *label_name(uint8_t label) const override
    {
        switch (label) {
        case LABEL_QUIET:  return "quiet";
        case LABEL_LOW:    return "low";
        case LABEL_MID:    return "mid";
        case LABEL_BRIGHT: return "bright";
        default:           return nullptr;
        }
    }

private:
    enum : uint8_t { LABEL_QUIET = 0, LABEL_LOW = 1, LABEL_MID = 2, LABEL_BRIGHT = 3 };

    /* Below this RMS the crossing rate is measuring dither and the room, not
     * the music, so the classification is not reported as one. */
    static constexpr uint32_t QUIET_RMS = 300;
    /* Roughly where a bass line stops dominating the waveform, and roughly
     * where cymbals and consonants start to. Both are one octave wide as
     * decisions go; nothing downstream should treat them as precise. */
    static constexpr uint32_t LOW_HZ    = 400;
    static constexpr uint32_t BRIGHT_HZ = 3000;

    /* Used only if the lane calls init() before any rate is known. */
    static constexpr int DF_RATE_FALLBACK = 44100;

    static constexpr AnalyserSpec spec_ = {
        /* name             */ "zcr-rms",
        /* model_id         */ 1,
        /* rate_hz          */ 0,           /* the stream's own rate */
        /* window_n         */ DF_FFT_N,
        /* hop_n            */ DF_HOP_N,
        /* present_delay_us */ 0,           /* the answer exists when the window does */
        /* lane             */ Lane::Fast,
    };

    int rate_ = DF_RATE_FALLBACK;
};

ZcrRms s_zcr_rms;

Analyser *const s_analysers[] = { &s_zcr_rms };

/*
 * A registered analyser must have a slot in every Frame, because the slot index
 * IS the registry index -- that is what lets a pattern read f.ml[i] and know
 * which analyser it got without asking anything at runtime. Registering more
 * analysers than there are slots would silently give the last ones nowhere to
 * put an answer, so it fails the build instead.
 */
static_assert(sizeof(s_analysers) / sizeof(s_analysers[0]) <= ML_SLOTS,
              "more analysers registered than DF_ML_SLOTS -- raise it in "
              "analysis_config.h, or the extra ones have nowhere to report");

}  // namespace

int analyser_count()
{
    return static_cast<int>(sizeof(s_analysers) / sizeof(s_analysers[0]));
}

Analyser *analyser_at(int i)
{
    return (i >= 0 && i < analyser_count()) ? s_analysers[i] : nullptr;
}

Analyser *analyser_by_name(const char *name)
{
    if (!name) return nullptr;
    for (int i = 0; i < analyser_count(); i++) {
        if (std::strcmp(s_analysers[i]->spec().name, name) == 0) return s_analysers[i];
    }
    return nullptr;
}

}  // namespace df
