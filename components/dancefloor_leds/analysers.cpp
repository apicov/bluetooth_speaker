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
 * Level and brightness, from one frame's spectrum.
 *
 * This is not a model and does not pretend to be one. It exists so that the
 * whole path -- registry, lane, latch, pattern -- can be built and proved with
 * an analyser that has no arena, no weights and no dependencies, so that when a
 * real model is dropped in the only new thing to debug is the model.
 *
 * WAS "zcr-rms", AND THE RENAME IS THE POINT. It used to compute RMS and a
 * zero-crossing rate from the PCM window. Analysers are handed the spectrum
 * now, and the two features have frequency-domain twins that are better rather
 * than merely equivalent: level is the mean bin, and zero-crossing rate is a
 * crude time-domain estimate of the spectral centroid, which can now simply be
 * computed. model_id moved 1 -> 3 rather than being bumped by one, so that a
 * board still running the old firmware cannot collide with the new answer.
 *
 * Level survives the change because the spectrum's compression is fixed --
 * beat_normalise is `raw / (1 + raw)`, not an AGC against a running maximum --
 * so a quiet passage stays quiet in these bins. See Analyser::process().
 *
 * All-integer, so it is bit-identical on an LX6 and an LX7 by construction
 * rather than by argument.
 *
 * Fast lane: it is two passes over 64 bytes, which is nothing beside the FFT
 * that produced them.
 */
class LevelTilt final : public Analyser {
public:
    const AnalyserSpec &spec() const override { return spec_; }

    bool init(int) override { return true; }

    bool process(const uint8_t (&spec)[SPEC_BINS], int64_t index, int64_t due_us,
                 Result *out) override
    {
        (void)due_us;

        uint32_t sum = 0, weighted = 0;
        for (int i = 0; i < SPEC_BINS; i++) {
            sum      += spec[i];
            weighted += static_cast<uint32_t>(spec[i]) * static_cast<uint32_t>(i);
        }

        /* Level is the mean bin, already on the 0..255 scale a strip wants.
         * Linear in the normalised spectrum, not in dB: a pattern wanting a
         * curve can apply its own, and doing it here would bake one choice into
         * everything downstream. */
        const uint32_t level = sum / SPEC_BINS;

        /*
         * The spectral centroid, in BIN INDEX rather than in Hz.
         *
         * The bins are log-spaced, so an index is already a log frequency and
         * the centroid of the indices is the geometric-ish centre of the
         * spectrum -- which is what "brightness" means to an ear, and what the
         * zero-crossing rate was reaching for. Working in Hz would mean an
         * exp() per frame to get back a number the thresholds below would only
         * convert again.
         *
         * Undefined for silence, where the sum is zero. Reported as the bottom
         * bin, which the QUIET test below catches first anyway.
         */
        const uint32_t centroid = sum ? weighted / sum : 0;

        uint8_t label;
        if (level < QUIET_LEVEL)        label = LABEL_QUIET;
        else if (centroid < LOW_BIN)    label = LABEL_LOW;
        else if (centroid < BRIGHT_BIN) label = LABEL_MID;
        else                            label = LABEL_BRIGHT;

        out->index = index;
        out->n = 1;
        out->label[0] = label;
        out->score[0] = static_cast<uint8_t>(level > 255 ? 255u : level);
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

    /*
     * Below this mean bin the centroid is describing dither and the room rather
     * than the music, so the classification is not reported as one.
     *
     * NOT A TRANSLATION OF THE OLD RMS THRESHOLD, and the honest thing is to say
     * so. The old value was 300 of 32767 in the time domain; the compression
     * between here and there is fixed but not linear, so there is no exact
     * equivalent. This is a bench starting point -- read the score on a quiet
     * passage and on silence, and set it between them.
     */
    static constexpr uint32_t QUIET_LEVEL = 4;

    /*
     * The old LOW_HZ of 400 and BRIGHT_HZ of 3000, converted once and written
     * down as bins so nothing converts at runtime:
     *
     *   bin(f) = (SPEC_BINS - 1) * ln(f / 40) / ln(16000 / 40)
     *   400 Hz  -> 24.2      3000 Hz -> 45.4
     *
     * Both are one octave wide as decisions go; nothing downstream should treat
     * them as precise. If SPEC_LO_HZ, SPEC_HI_HZ or SPEC_BINS ever move, these
     * two are what has to be recomputed -- which is why the formula is here and
     * not only in a commit message.
     */
    static constexpr uint32_t LOW_BIN    = 24;
    static constexpr uint32_t BRIGHT_BIN = 45;

    static constexpr AnalyserSpec spec_ = {
        /* name             */ "level-tilt",
        /* model_id         */ 3,
        /* present_delay_us */ 0,           /* the answer exists when the frame does */
        /* lane             */ Lane::Fast,
    };
};

/*
 * Texture over a second of music, on the slow lane.
 *
 * The point of this one is the SHAPE, which is the shape every real audio model
 * uses and which the interface was built around:
 *
 *   it summarises each frame into a few bytes, accumulates a second of those
 *   summaries internally, and returns false until the second is complete.
 *
 * That is what makes it cheap. Holding a second of AUDIO would be 16000 samples
 * at 16 kHz -- 32 kB, and the reason the lane used to carry a resampler and a
 * 12.8 kB stream buffer. Holding a second of four-byte summaries is under a
 * kilobyte, and it is why this analyser now runs on a unit that has no audio at
 * all: the frames arrive over the radio already reduced. A mel front end feeding
 * a real model works exactly this way -- short frames in, a rolling buffer of
 * FEATURES, the model over those.
 *
 * model_id moved 2 -> 4 with the change of input, for the reason level-tilt's
 * did: a board still running the old firmware must not appear to agree.
 *
 * present_delay_us is small despite the second of context because the result is
 * labelled with the LAST frame of that second, not the first -- so the context
 * is already behind us when the answer appears. An analyser labelling by the
 * START of its context would need the full delay AnalyserSpec describes.
 *
 * All-integer, so it is bit-identical on an LX6 and an LX7 by construction.
 */
class Mood final : public Analyser {
public:
    const AnalyserSpec &spec() const override { return spec_; }

    bool init(int frames_per_s) override
    {
        /*
         * A second of context, in frames, from the rate the lane actually sees.
         *
         * Not a constant, because the hop is compiled in but the stream rate is
         * not: 44.1 kHz at hop 512 is 86 frames a second, 48 kHz at hop 256 is
         * 187. Sizing this from a hard-coded 86 would give a "one second" that
         * was 2.2 seconds on one of the four rates the bridge advertises.
         */
        context_n_ = frames_per_s > 0 ? frames_per_s : 86;
        if (context_n_ > CONTEXT_MAX) context_n_ = CONTEXT_MAX;
        if (context_n_ < 1)           context_n_ = 1;
        reset();
        return true;
    }

    void reset() override
    {
        n_ = 0;
        next_ = 0;
        since_report_ = 0;
    }

    bool process(const uint8_t (&spec)[SPEC_BINS], int64_t index, int64_t due_us,
                 Result *out) override
    {
        (void)due_us;

        uint32_t sum = 0, weighted = 0;
        for (int i = 0; i < SPEC_BINS; i++) {
            sum      += spec[i];
            weighted += static_cast<uint32_t>(spec[i]) * static_cast<uint32_t>(i);
        }

        hist_[next_].level    = static_cast<uint16_t>(sum / SPEC_BINS);
        hist_[next_].centroid = static_cast<uint16_t>(sum ? weighted / sum : 0);
        next_ = (next_ + 1) % context_n_;
        if (n_ < context_n_) n_++;

        /* One report per full context, and none until the first one is full --
         * an answer from a partial second would describe less music than every
         * later answer, and nothing downstream could tell. */
        if (++since_report_ < context_n_ || n_ < context_n_) {
            return false;
        }
        since_report_ = 0;

        /*
         * Summed oldest-first from the write pointer rather than in array
         * order, so the sum does not depend on where the ring happens to have
         * wrapped. The same rotation-independence beat_detect.c needed, and for
         * the same reason: two units that joined at different moments must add
         * the same numbers in the same order.
         */
        uint64_t sum_l = 0, sum_sq_l = 0, sum_c = 0;
        for (int k = 0; k < context_n_; k++) {
            const uint32_t v = hist_[(next_ + k) % context_n_].level;
            sum_l    += v;
            sum_sq_l += static_cast<uint64_t>(v) * v;
            sum_c    += hist_[(next_ + k) % context_n_].centroid;
        }

        const uint32_t mean = static_cast<uint32_t>(sum_l / context_n_);
        /* Population variance, integer: E[x^2] - E[x]^2, floored at zero
         * against the rounding in the two means. */
        const uint64_t mean_sq = sum_sq_l / context_n_;
        const uint64_t sq_mean = static_cast<uint64_t>(mean) * mean;
        const uint32_t sd = isqrt64(mean_sq > sq_mean ? mean_sq - sq_mean : 0);

        /* Spread as a percentage of level, which is what makes it a statement
         * about dynamics rather than about volume. */
        const uint32_t dynamics = mean ? (sd * 100u) / mean : 0;
        const uint32_t centroid = static_cast<uint32_t>(sum_c / context_n_);

        uint8_t label;
        if (mean < QUIET_LEVEL)          label = LABEL_CALM;
        else if (dynamics > DYNAMIC_PCT) label = LABEL_PEAK;
        else if (centroid > BUSY_BIN)    label = LABEL_BUSY;
        else                             label = LABEL_GROOVE;

        out->index = index;
        out->n = 1;
        out->label[0] = label;
        out->score[0] = static_cast<uint8_t>(mean > 255 ? 255u : mean);
        return true;
    }

    const char *label_name(uint8_t label) const override
    {
        switch (label) {
        case LABEL_CALM:   return "calm";
        case LABEL_GROOVE: return "groove";
        case LABEL_BUSY:   return "busy";
        case LABEL_PEAK:   return "peak";
        default:           return nullptr;
        }
    }

private:
    enum : uint8_t { LABEL_CALM = 0, LABEL_GROOVE = 1, LABEL_BUSY = 2, LABEL_PEAK = 3 };

    /* Frames of context the ring can hold: 48 kHz at hop 256 is 187.5 a second,
     * which is the fastest any supported combination produces. 4 bytes each, so
     * the whole ring is 768 B against the 32 kB a second of audio would be. */
    static constexpr int CONTEXT_MAX = 192;

    /* Same scale and the same caveat as level-tilt's QUIET_LEVEL -- a bench
     * starting point, not a conversion of the old time-domain threshold. */
    static constexpr uint32_t QUIET_LEVEL = 4;
    static constexpr uint32_t DYNAMIC_PCT = 60;
    /* The old BUSY_HZ of 2500, converted by the formula in level-tilt: 43.5. */
    static constexpr uint32_t BUSY_BIN    = 43;

    /*
     * A hundred milliseconds of margin, not a hundred milliseconds of need.
     *
     * The bound in AnalyserSpec is satisfied by zero here: the context ends at
     * the frame this is labelled with, so its audio arrived a presentation lead
     * ago and the answer is ready well before the frame is drawn. The margin
     * exists because compute is not constant -- a board that stalls for 150 ms
     * would otherwise miss the frame it named and differ from its neighbours
     * for one -- and because 100 ms of lag on a texture readout is invisible.
     *
     * Confirm it against the lane's `late` counter rather than trusting it.
     */
    static constexpr int64_t PRESENT_DELAY_US = 100000;

    static constexpr AnalyserSpec spec_ = {
        /* name             */ "mood",
        /* model_id         */ 4,
        /* present_delay_us */ PRESENT_DELAY_US,
        /* lane             */ Lane::Slow,
    };

    struct Sub { uint16_t level, centroid; };
    Sub hist_[CONTEXT_MAX];
    int context_n_ = 86;
    int n_ = 0, next_ = 0, since_report_ = 0;
};

LevelTilt s_level_tilt;
Mood      s_mood;

Analyser *const s_analysers[] = { &s_level_tilt, &s_mood };

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

void downmix(const int16_t *stereo, int n, int16_t *mono)
{
    for (int i = 0; i < n; i++) {
        mono[i] = static_cast<int16_t>((static_cast<int32_t>(stereo[2 * i]) +
                                        static_cast<int32_t>(stereo[2 * i + 1])) / 2);
    }
}

void run_fast_lane(const uint8_t (&spec)[SPEC_BINS], int64_t index,
                   int64_t due_us, const bool skip[ML_SLOTS], Result out[ML_SLOTS])
{
    for (int i = 0; i < ML_SLOTS; i++) {
        out[i] = result_none();

        Analyser *a = analyser_at(i);
        if (!a || (skip && skip[i])) {
            continue;
        }

        Result r{};
        if (a->process(spec, index, due_us, &r)) {
            const AnalyserSpec &sp = a->spec();
            r.analyser   = static_cast<uint8_t>(i);
            r.model_id   = sp.model_id;
            r.show_at_us = due_us + sp.present_delay_us;
            out[i] = r;
        }
    }
}

}  // namespace df
