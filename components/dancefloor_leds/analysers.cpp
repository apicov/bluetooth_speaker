
#include "analyser.hpp"

#include <cstring>

#include "analysis_config.h"

namespace df {
namespace {

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

        const uint32_t level = sum / SPEC_BINS;

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

    static constexpr uint32_t QUIET_LEVEL = 4;

    static constexpr uint32_t LOW_BIN    = 24;
    static constexpr uint32_t BRIGHT_BIN = 45;

    static constexpr AnalyserSpec spec_ = {
         "level-tilt",
         3,
         0,
         Lane::Fast,
    };
};

class Mood final : public Analyser {
public:
    const AnalyserSpec &spec() const override { return spec_; }

    bool init(int frames_per_s) override
    {

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

        if (++since_report_ < context_n_ || n_ < context_n_) {
            return false;
        }
        since_report_ = 0;

        uint64_t sum_l = 0, sum_sq_l = 0, sum_c = 0;
        for (int k = 0; k < context_n_; k++) {
            const uint32_t v = hist_[(next_ + k) % context_n_].level;
            sum_l    += v;
            sum_sq_l += static_cast<uint64_t>(v) * v;
            sum_c    += hist_[(next_ + k) % context_n_].centroid;
        }

        const uint32_t mean = static_cast<uint32_t>(sum_l / context_n_);

        const uint64_t mean_sq = sum_sq_l / context_n_;
        const uint64_t sq_mean = static_cast<uint64_t>(mean) * mean;
        const uint32_t sd = isqrt64(mean_sq > sq_mean ? mean_sq - sq_mean : 0);

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

    static constexpr int CONTEXT_MAX = 192;

    static constexpr uint32_t QUIET_LEVEL = 4;
    static constexpr uint32_t DYNAMIC_PCT = 60;

    static constexpr uint32_t BUSY_BIN    = 43;

    static constexpr int64_t PRESENT_DELAY_US = 100000;

    static constexpr AnalyserSpec spec_ = {
         "mood",
         4,
         PRESENT_DELAY_US,
         Lane::Slow,
    };

    struct Sub { uint16_t level, centroid; };
    Sub hist_[CONTEXT_MAX];
    int context_n_ = 86;
    int n_ = 0, next_ = 0, since_report_ = 0;
};

LevelTilt s_level_tilt;
Mood      s_mood;

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

}
