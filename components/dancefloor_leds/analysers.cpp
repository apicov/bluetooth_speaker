
/**
 * @file analysers.cpp
 * @brief The registered analysers, the registry itself, and the fast lane that
 *        drives them.
 *
 * analyser.hpp says what an Analyser is and states the rule every one here
 * obeys: a pure function of the audio it is handed and the constants it was
 * compiled with, producing the same Result bit for bit on either core. Both
 * analysers below are therefore INTEGER throughout -- no float appears in
 * either, which is what makes that guarantee rather than a hope.
 *
 * The two are deliberately a matched pair: one fast analyser answering from a
 * single frame, and one slow one accumulating a context of them. That is the
 * shape the lane split exists for, and between them they exercise both.
 */
#include "analyser.hpp"

#include <cstring>

#include "analysis_config.h"

namespace df {
namespace {

/**
 * @brief Integer square root, by Newton's method.
 *
 * Integer rather than std::sqrt because the result reaches a Result and must
 * be identical on both cores -- see the rule in analyser.hpp.
 *
 * @param v  The value.
 * @return The floor of its square root.
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

/**
 * @brief How loud, and how bright: the cheapest useful thing that can be said
 *        about one frame.
 *
 * A fast-lane analyser, and the worked example of one. Its window IS the
 * frame's window, so its answer exists the moment the frame does and its
 * presentation delay is zero -- which is the only case where zero is correct.
 *
 * Two numbers over the quantised spectrum: the mean bin value as a level, and
 * the level-weighted mean bin index as a spectral centroid. That pair
 * separates quiet from loud and bass-heavy from bright, which is as much as a
 * strip can show from a single frame.
 */
class LevelTilt final : public Analyser {
public:
    const AnalyserSpec &spec() const override { return spec_; }

    /** @brief Nothing to size and nothing to fail on: this analyser holds no
     *         state between frames. @return true, always. */
    bool init(int) override { return true; }

    bool process(const uint8_t (&spec)[SPEC_BINS], int64_t index, int64_t due_us,
                 Result *out) override
    {
        (void)due_us;   /* the lane dates the result; see run_fast_lane() */

        uint32_t sum = 0, weighted = 0;
        for (int i = 0; i < SPEC_BINS; i++) {
            sum      += spec[i];
            weighted += static_cast<uint32_t>(spec[i]) * static_cast<uint32_t>(i);
        }
        /* Mean bin value: how loud, on the spectrum's own quantised scale. */
        const uint32_t level = sum / SPEC_BINS;
        /* Level-weighted mean bin INDEX, not a frequency -- the bins are
         * log-spaced, so an index is already something like a perceptual
         * position and needs no conversion. Zero on silence, where the
         * weighting has nothing to divide by. */
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

    /** @brief Below this mean level the frame is called quiet, whatever its
     *         tilt: a centroid computed from near-silence is noise. */
    static constexpr uint32_t QUIET_LEVEL = 4;

    /** @brief Centroid below this bin is "low". */
    static constexpr uint32_t LOW_BIN    = 24;
    /** @brief ...and above this one, "bright". Between them is "mid". */
    static constexpr uint32_t BRIGHT_BIN = 45;

    /** @brief Fast lane, so no presentation delay. */
    static constexpr AnalyserSpec spec_ = {
        /* name             */ "level-tilt",
        /* model_id         */ 3,
        /* present_delay_us */ 0,           /* the answer exists when the frame does */
        /* lane             */ Lane::Fast,
    };
};

/**
 * @brief What the last second or so has been like: calm, groove, busy or
 *        peak.
 *
 * A slow-lane analyser, and the worked example of one. It accumulates a
 * CONTEXT of frames and answers over the whole of it, which is exactly the
 * shape a real model has -- short frames in, a rolling buffer of features, an
 * answer over those -- and it is why it cannot run in the fast lane and why it
 * needs a presentation delay at all.
 *
 * The features are LevelTilt's two, per frame, kept as a ring. What it adds is
 * the statistics ACROSS them: the mean level, the coefficient of variation of
 * the level as a measure of dynamics, and the mean centroid. A stretch with
 * high dynamics is a peak whatever its brightness; a bright steady stretch is
 * busy; a dark steady one is a groove.
 */
class Mood final : public Analyser {
public:
    const AnalyserSpec &spec() const override { return spec_; }

    bool init(int frames_per_s) override
    {

        /* A context of roughly one second, sized in FRAMES from the rate
         * rather than hard-coded: the bridge advertises several sample rates,
         * so a fixed count would mean a different span on each. */
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
        (void)due_us;   /* the lane dates the result; see ml_lane.cpp */

        /* The same two features LevelTilt computes, per frame, kept as a
         * ring. */
        uint32_t sum = 0, weighted = 0;
        for (int i = 0; i < SPEC_BINS; i++) {
            sum      += spec[i];
            weighted += static_cast<uint32_t>(spec[i]) * static_cast<uint32_t>(i);
        }

        hist_[next_].level    = static_cast<uint16_t>(sum / SPEC_BINS);
        hist_[next_].centroid = static_cast<uint16_t>(sum ? weighted / sum : 0);
        next_ = (next_ + 1) % context_n_;
        if (n_ < context_n_) n_++;

        /* Returning false is normal: nothing can be said until the context is
         * full, and then only once per context. That is how an analyser with a
         * one-second window reports at 1 Hz without the lane needing to know
         * anything about it. */
        if (++since_report_ < context_n_ || n_ < context_n_) {
            return false;
        }
        since_report_ = 0;

        /* Summed OLDEST FIRST, from the ring's tail rather than in array
         * order. Integer addition IS associative, so unlike the detector's
         * float history this is not a correctness requirement -- but the two
         * loops should read the same way, and the next feature added here may
         * not be integer. */
        uint64_t sum_l = 0, sum_sq_l = 0, sum_c = 0;
        for (int k = 0; k < context_n_; k++) {
            const uint32_t v = hist_[(next_ + k) % context_n_].level;
            sum_l    += v;
            sum_sq_l += static_cast<uint64_t>(v) * v;
            sum_c    += hist_[(next_ + k) % context_n_].centroid;
        }

        const uint32_t mean = static_cast<uint32_t>(sum_l / context_n_);

        /* Variance as E[x^2] - E[x]^2, in 64 bits so the sum of squares over a
         * whole context cannot overflow. */
        const uint64_t mean_sq = sum_sq_l / context_n_;
        const uint64_t sq_mean = static_cast<uint64_t>(mean) * mean;
        const uint32_t sd = isqrt64(mean_sq > sq_mean ? mean_sq - sq_mean : 0);

        /* Coefficient of variation, as a percentage: the spread RELATIVE to
         * the level, so a loud steady passage and a quiet steady one both read
         * as steady. */
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

    /** @brief Ceiling on the context, so the ring is a fixed array: enough for
     *         a second at the highest frame rate any supported hop and sample
     *         rate produce. */
    static constexpr int CONTEXT_MAX = 192;

    /** @brief Below this mean level the stretch is called calm. */
    static constexpr uint32_t QUIET_LEVEL = 4;
    /** @brief Coefficient of variation above which it is a peak, in percent. */
    static constexpr uint32_t DYNAMIC_PCT = 60;
    /** @brief Mean centroid above which a steady stretch is busy rather than a
     *         groove. */
    static constexpr uint32_t BUSY_BIN    = 43;

    /**
     * @brief How late this analyser's answers are shown.
     *
     * The rule is on df::AnalyserSpec::present_delay_us and this is the worked
     * case. Mood labels a result by its context's LAST frame, so the context
     * is already behind by the time the answer exists and the arrival term of
     * the bound is satisfied outright. What remains to cover is the compute
     * and the publish, and the playback lead is subtracted from both -- so the
     * bound is slack and this is a round number comfortably above it rather
     * than a measurement.
     *
     * It must be the same number on every unit running this analyser, which is
     * why it is a constant here and not read from anything.
     */
    static constexpr int64_t PRESENT_DELAY_US = 100000;

    /** @brief Slow lane; see ml_lane.cpp. */
    static constexpr AnalyserSpec spec_ = {
        /* name             */ "mood",
        /* model_id         */ 4,
        /* present_delay_us */ PRESENT_DELAY_US,
        /* lane             */ Lane::Slow,
    };

    /** @brief One frame's features. */
    struct Sub { uint16_t level, centroid; };
    Sub hist_[CONTEXT_MAX];   /**< The ring. */
    int context_n_ = 86;      /**< How much of it is in use; set by init(). */
    int n_ = 0;               /**< How much of that is filled. */
    int next_ = 0;            /**< Where the next frame lands, and the tail. */
    int since_report_ = 0;    /**< Frames since the last answer. */
};

/** @brief The one instance of each analyser. Static, so nothing allocates. */
LevelTilt s_level_tilt;
Mood      s_mood;   /**< See s_level_tilt. */

/** @brief The registry. A static table, no dynamic registration: the INDEX is
 *         the slot, so adding one here is what makes f.ml[i] mean it. */
Analyser *const s_analysers[] = { &s_level_tilt, &s_mood };

static_assert(sizeof(s_analysers) / sizeof(s_analysers[0]) <= ML_SLOTS,
              "more analysers registered than DF_ML_SLOTS -- raise it in "
              "analysis_config.h, or the extra ones have nowhere to report");

}

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

/* Declared in analyser.hpp, like the four above it. */
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
        /* Cleared first, unconditionally: the caller's buffer is reused, and
         * whatever a previous frame left in a slot describes audio long
         * past. */
        out[i] = result_none();

        Analyser *a = analyser_at(i);
        if (!a || (skip && skip[i])) {
            continue;
        }

        Result r{};
        if (a->process(spec, index, due_us, &r)) {
            const AnalyserSpec &sp = a->spec();
            /* Filled from the SPEC, not by the analyser -- one that could set
             * these could date its own results, which is the one thing the
             * presentation-delay rule forbids. */
            r.analyser   = static_cast<uint8_t>(i);
            r.model_id   = sp.model_id;
            r.show_at_us = due_us + sp.present_delay_us;
            out[i] = r;
        }
    }
}

}
