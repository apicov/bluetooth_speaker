/*
 * Built-in patterns. Add new ones here and to pattern_list().
 *
 * Read the rule at the top of analysis.hpp first: a pattern may use only what
 * is in the Frame it is handed. Everything below derives its animation from
 * f.due_us and its state from earlier Frames, which is what keeps the strips
 * agreeing without any communication between them.
 */
#pragma once

#include "analysis.hpp"

namespace df {

/*
 * Brightness tracks the beat envelope, hue drifts slowly so a static room does
 * not look frozen between hits, and bass pushes the colour outward from the
 * centre of the strip.
 *
 * The pattern the project shipped with, and the baseline to beat.
 */
class PulsePattern : public Pattern {
public:
    const char *name() const override { return "pulse"; }
    void render(const Frame &f, uint8_t *rgb, uint32_t count) override;
    void reset() override;

private:
    float   level_ = 0.0f;
    int64_t last_due_us_ = 0;
};

/* Number of built-in patterns, and lookup by name or index. Returns nullptr if
 * there is no such pattern. */
int         pattern_count();
Pattern    *pattern_at(int i);
Pattern    *pattern_by_name(const char *name);

}  // namespace df
