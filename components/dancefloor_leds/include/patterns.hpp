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

/*
 * Follows the zabumba's boom and nothing else -- see Frame::boom.
 *
 * Deliberately plain: a flash on the drum, decaying on shared time, with the
 * bass level setting how much of the strip lights. Its job is to make it
 * obvious whether the detector is finding the drum, so anything decorative
 * would get in the way of reading it. Run it against `pulse` on the same track
 * in pattern_lab -- if the two disagree, the wideband detector is following the
 * triangle.
 */
class BoomPattern : public Pattern {
public:
    const char *name() const override { return "boom"; }
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
