
#pragma once

#include "analysis.hpp"

namespace df {

class PulsePattern : public Pattern {
public:
    const char *name() const override { return "pulse"; }
    void render(const Frame &f, uint8_t *rgb, uint32_t count) override;
    void reset() override;

private:
    float   level_ = 0.0f;
    int64_t last_due_us_ = 0;
};

class BoomPattern : public Pattern {
public:
    const char *name() const override { return "boom"; }
    void render(const Frame &f, uint8_t *rgb, uint32_t count) override;
    void reset() override;

private:
    float   level_ = 0.0f;
    int64_t last_due_us_ = 0;
};

int         pattern_count();
Pattern    *pattern_at(int i);
Pattern    *pattern_by_name(const char *name);

}
