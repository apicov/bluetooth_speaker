/**
 * @file patterns.cpp
 * @brief The built-in patterns, and the registry visualiser.cpp selects from.
 *
 * patterns.hpp says what each one is for. What is here is the rendering, and
 * every line of it obeys the rule at the top of analysis.hpp: the only inputs
 * are the frame's own fields, and every animation is derived from the frame's
 * SHARED instant rather than from anything local. That is what keeps two
 * strips agreeing without either knowing the other exists.
 */
#include "patterns.hpp"

#include <cmath>
#include <cstring>

namespace df {
namespace {

/** @brief One LED's colour. */
struct Rgb { uint8_t r, g, b; };

/**
 * @brief HSV to RGB.
 * @param h  Hue, 0..360.
 * @param s  Saturation, 0..1.
 * @param v  Value, 0..1.
 * @return The colour.
 */
Rgb hsv2rgb(float h, float s, float v)
{
    const float c = v * s;
    const float x = c * (1.0f - std::fabs(std::fmod(h / 60.0f, 2.0f) - 1.0f));
    const float m = v - c;
    float rf, gf, bf;

    if      (h <  60.0f) { rf = c; gf = x; bf = 0; }
    else if (h < 120.0f) { rf = x; gf = c; bf = 0; }
    else if (h < 180.0f) { rf = 0; gf = c; bf = x; }
    else if (h < 240.0f) { rf = 0; gf = x; bf = c; }
    else if (h < 300.0f) { rf = x; gf = 0; bf = c; }
    else                 { rf = c; gf = 0; bf = x; }

    return { static_cast<uint8_t>((rf + m) * 255.0f),
             static_cast<uint8_t>((gf + m) * 255.0f),
             static_cast<uint8_t>((bf + m) * 255.0f) };
}

/** @brief Clamp to 0..1. @param v The value. @return It, clamped. */
float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

/** @brief How long the hue takes to go round once. Slow enough that a static
 *         room does not look frozen between hits, and derived from the frame's
 *         shared instant rather than accumulated per render -- see the rule at
 *         the top of analysis.hpp. */
constexpr float HUE_PERIOD_US = 28.0f * 1000000.0f;

/** @brief Time constant of the pulse envelope's decay, applied over the gap
 *         between two frames' instants -- so a unit that rendered a different
 *         number of times still decays by the same amount. */
constexpr float DECAY_TAU_US = 140000.0f;

/** @brief How much of each end of the strip the bass pushes colour into. */
constexpr float EDGE_WIDTH = 0.15f;

/** @brief The boom envelope's decay, longer than the pulse's because the drum
 *         it follows is slower than the transients the wideband detector
 *         fires on. */
constexpr float BOOM_TAU_US = 260000.0f;

/** @brief The one instance of each pattern. Static, so nothing allocates. */
PulsePattern s_pulse;
BoomPattern  s_boom;   /**< See s_pulse. */
/** @brief The registry, in the shape the analyser one also uses: a static
 *         table, no dynamic registration. */
Pattern *const s_patterns[] = { &s_pulse, &s_boom };

}

void PulsePattern::reset()
{
    level_ = 0.0f;
    last_due_us_ = 0;
}

void PulsePattern::render(const Frame &f, uint8_t *rgb, uint32_t count)
{
    /* Decayed over the gap between two frames' SHARED instants, not per render
     * call and not against a local clock. A unit that dropped frames or
     * rendered at a different rate still arrives at the same level. */
    if (last_due_us_ != 0 && f.due_us > last_due_us_) {
        const float dt = static_cast<float>(f.due_us - last_due_us_);
        level_ *= std::exp(-dt / DECAY_TAU_US);
    }
    last_due_us_ = f.due_us;
    if (f.onset) {
        level_ = 0.3f + 0.7f * f.strength;
    }

    float hue = std::fmod(static_cast<float>(f.due_us % static_cast<int64_t>(HUE_PERIOD_US))
                          / HUE_PERIOD_US * 360.0f, 360.0f);
    if (hue < 0.0f) hue += 360.0f;

    const Rgb c = hsv2rgb(hue, 1.0f, level_);
    const float bass = f.band[0];

    const float centre = (count > 1) ? (count - 1) * 0.5f : 1.0f;

    for (uint32_t i = 0; i < count; i++) {
        const float pos = std::fabs(static_cast<float>(i) - centre) / centre;

        const float k = 0.25f + 0.75f * clamp01((bass - pos) / EDGE_WIDTH + 0.5f);
        rgb[3 * i + 0] = static_cast<uint8_t>(c.r * k);
        rgb[3 * i + 1] = static_cast<uint8_t>(c.g * k);
        rgb[3 * i + 2] = static_cast<uint8_t>(c.b * k);
    }
}

void BoomPattern::reset()
{
    level_ = 0.0f;
    last_due_us_ = 0;
}

void BoomPattern::render(const Frame &f, uint8_t *rgb, uint32_t count)
{

    if (last_due_us_ != 0 && f.due_us > last_due_us_) {
        const float dt = static_cast<float>(f.due_us - last_due_us_);
        level_ *= std::exp(-dt / BOOM_TAU_US);
    }
    last_due_us_ = f.due_us;
    if (f.boom) {
        level_ = 0.35f + 0.65f * f.boom_strength;
    }

    const float lit = clamp01(level_);
    const uint8_t r = static_cast<uint8_t>(255.0f * lit);
    const uint8_t g = static_cast<uint8_t>(90.0f  * lit);

    const float centre = (count > 1) ? (count - 1) * 0.5f : 1.0f;
    const float reach  = 0.25f + 0.75f * clamp01(f.band[0]);

    for (uint32_t i = 0; i < count; i++) {
        const float pos = std::fabs(static_cast<float>(i) - centre) / centre;
        const float k = pos <= reach ? 1.0f : 0.15f;
        rgb[3 * i + 0] = static_cast<uint8_t>(r * k);
        rgb[3 * i + 1] = static_cast<uint8_t>(g * k);
        rgb[3 * i + 2] = 0;
    }
}

int pattern_count() { return static_cast<int>(sizeof(s_patterns) / sizeof(s_patterns[0])); }

Pattern *pattern_at(int i)
{
    return (i >= 0 && i < pattern_count()) ? s_patterns[i] : nullptr;
}

Pattern *pattern_by_name(const char *name)
{
    if (!name) return nullptr;
    for (int i = 0; i < pattern_count(); i++) {
        if (std::strcmp(s_patterns[i]->name(), name) == 0) return s_patterns[i];
    }
    return nullptr;
}

}
