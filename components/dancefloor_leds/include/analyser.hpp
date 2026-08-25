
#pragma once

#include <stdint.h>

#include "analysis_config.h"

namespace df {

constexpr int SPEC_BINS = DF_SPEC_BINS;

constexpr int ML_SLOTS = DF_ML_SLOTS;

constexpr int RESULT_SCORES = 8;

constexpr uint8_t RESULT_NONE = 0xFF;

struct Result {

    int64_t index;

    int64_t show_at_us;

    uint8_t analyser;
    uint8_t model_id;
    uint8_t n;
    uint8_t label[RESULT_SCORES];
    uint8_t score[RESULT_SCORES];
};

constexpr bool result_valid(const Result &r) { return r.analyser != RESULT_NONE; }

constexpr Result result_none()
{
    Result r{};
    r.analyser = RESULT_NONE;
    return r;
}

enum class Lane { Fast, Slow };

struct AnalyserSpec {

    const char *name;

    uint8_t model_id;

    int64_t present_delay_us;

    Lane lane;
};

class Analyser {
public:
    virtual ~Analyser() = default;

    virtual const AnalyserSpec &spec() const = 0;

    virtual bool init(int frames_per_s) = 0;

    virtual void reset() {}

    virtual const char *label_name(uint8_t label) const { (void)label; return nullptr; }

    virtual bool process(const uint8_t (&spec)[SPEC_BINS], int64_t index,
                         int64_t due_us, Result *out) = 0;
};

int       analyser_count();
Analyser *analyser_at(int i);
Analyser *analyser_by_name(const char *name);

void run_fast_lane(const uint8_t (&spec)[SPEC_BINS], int64_t index,
                   int64_t due_us, const bool skip[ML_SLOTS], Result out[ML_SLOTS]);

void downmix(const int16_t *stereo, int n, int16_t *mono);

}
