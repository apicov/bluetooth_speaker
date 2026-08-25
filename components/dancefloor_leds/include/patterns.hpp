/**
 * @file patterns.hpp
 * @brief The built-in patterns. Add new ones here and to the registry in
 *        patterns.cpp.
 *
 * Read the rule at the top of analysis.hpp first: a pattern may use only what
 * is in the df::Frame it is handed. Everything below derives its animation
 * from the frame's own instant and its state from earlier frames, which is
 * what keeps the strips agreeing without any communication between them.
 */
#pragma once

#include "analysis.hpp"

namespace df {

/**
 * @brief Brightness tracks the beat envelope, hue drifts slowly so a static
 *        room does not look frozen between hits, and bass pushes the colour
 *        outward from the centre of the strip.
 *
 * The pattern the project shipped with, and the baseline to beat.
 */
class PulsePattern : public Pattern {
public:
    /** @brief @return "pulse". */
    const char *name() const override { return "pulse"; }
    /** @copydoc Pattern::render */
    void render(const Frame &f, uint8_t *rgb, uint32_t count) override;
    /** @copydoc Pattern::reset */
    void reset() override;

private:
    float   level_ = 0.0f;        /**< The envelope, decayed on SHARED time. */
    int64_t last_due_us_ = 0;     /**< ...against the previous frame's instant. */
};

/**
 * @brief Follows the zabumba's boom and nothing else -- see df::Frame::boom.
 *
 * Deliberately plain: a flash on the drum, decaying on shared time, with the
 * bass level setting how much of the strip lights. Its job is to make it
 * obvious whether the detector is finding the drum, so anything decorative
 * would get in the way of reading it. Run it against "pulse" on the same track
 * in the host harness -- if the two disagree, the wideband detector is
 * following the triangle.
 */
class BoomPattern : public Pattern {
public:
    /** @brief @return "boom". */
    const char *name() const override { return "boom"; }
    /** @copydoc Pattern::render */
    void render(const Frame &f, uint8_t *rgb, uint32_t count) override;
    /** @copydoc Pattern::reset */
    void reset() override;

private:
    float   level_ = 0.0f;        /**< The envelope, decayed on SHARED time. */
    int64_t last_due_us_ = 0;     /**< ...against the previous frame's instant. */
};

/** @brief How many built-in patterns there are. @return The count. */
int         pattern_count();
/** @brief The pattern at an index. @param i Index. @return It, or null. */
Pattern    *pattern_at(int i);
/** @brief Look one up by name. @param name The name. @return It, or null. */
Pattern    *pattern_by_name(const char *name);

}  // namespace df
