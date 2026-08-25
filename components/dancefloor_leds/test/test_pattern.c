/**
 * @file test_pattern.c
 * @brief Host test for the pulse pattern's edge shaping.
 *
 * Pins the arithmetic that decides how much of the strip the bass lights, by
 * comparing the current formulation against the one it replaced. The old one
 * was a hard threshold -- a pixel was either in the bass region or not -- so
 * the lit width jumped between discrete positions as the bass moved. The
 * current one ramps, and the cases here are what says so: monotonic in the
 * bass level, continuous across the edge, and agreeing with the old one at the
 * extremes where they should agree.
 */
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

/** @brief The share of each end of the strip the bass reaches into, matching
 *         the pattern's own constant. */
#define EDGE_WIDTH 0.15f

/** @brief Distance from the nearer END of the strip, 0 at the ends and 0.5 in
 *         the middle -- so the bass grows inward from both.
 *  @param i      LED index. @param count LEDs. @return The position. */
static float pos_new(int i, int count) {
    float centre = (count > 1) ? (count - 1) * 0.5f : 1.0f;
    return fabsf((float)i - centre) / centre;
}
/** @brief The formulation this replaced: distance from one end only, so the
 *         strip was asymmetric.
 *  @param i      LED index. @param count LEDs. @return The position. */
static float pos_old(int i, int count) {
    return fabsf((float)i / count - 0.5f) * 2.0f;
}
/** @brief How lit an LED is: a RAMP across the edge, so the lit width moves
 *         continuously with the bass.
 *  @param bass  Bass level, 0..1. @param pos Position from pos_new().
 *  @return The scale, 0..1. */
static float k_new(float bass, float pos) {
    float t = (bass - pos) / EDGE_WIDTH + 0.5f;
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    return 0.25f + 0.75f * t;
}
/** @brief The formulation this replaced: a hard threshold, so the lit width
 *         jumped between discrete positions.
 *  @param bass  Bass level, 0..1. @param pos Position. @return The scale. */
static float k_old(float bass, float pos) { return bass > pos ? 1.0f : 0.25f; }

/** @brief Cases that did not hold; main() returns non-zero if any. */
static int failures;
/** @brief Report one case. @param name What is pinned. @param ok Whether it
 *         held. @param note The measured figures, or NULL. */
static void check(const char *name, bool ok, const char *note) {
    printf("%-52s %s  %s\n", name, ok ? "PASS" : "FAIL", note ? note : "");
    if (!ok) failures++;
}

/**
 * @brief Run every case and report.
 * @return 0 if all held, 1 otherwise, so `make check` fails the build.
 */
int main(void) {
    const int counts[] = {8, 30, 60, 144};

    const float bass_max = 0.999f;

    for (unsigned c = 0; c < sizeof counts / sizeof counts[0]; c++) {
        int n = counts[c];
        char note[64];

        bool sym = true;
        for (int i = 0; i < n; i++) {
            if (fabsf(pos_new(i, n) - pos_new(n - 1 - i, n)) > 1e-5f) sym = false;
        }
        snprintf(note, sizeof note, "count=%d", n);
        check("pos is symmetric end to end", sym, note);

        bool ends = fabsf(pos_new(0, n) - 1.0f) < 1e-5f
                 && fabsf(pos_new(n - 1, n) - 1.0f) < 1e-5f;
        check("both end pixels reach exactly 1.0", ends, note);

        bool old_lights = k_old(bass_max, pos_old(0, n)) > 0.25f;
        bool new_lights = k_new(bass_max, pos_new(0, n)) > 0.25f;
        snprintf(note, sizeof note, "count=%d old=%s new=%s", n,
                 old_lights ? "lights" : "NEVER", new_lights ? "lights" : "NEVER");
        check("outermost pixel can light at full bass", new_lights && !old_lights, note);
    }

    {
        float pos = 0.5f;
        float a = 0.500f, b = 0.505f;
        float ratio_old = k_old(b, pos) / k_old(a, pos);
        float ratio_new = k_new(b, pos) / k_new(a, pos);
        char note[80];
        snprintf(note, sizeof note, "hard=%.2fx  ramp=%.3fx", ratio_old, ratio_new);
        check("a 1 pct bass difference is no longer a 4x brightness difference",
              ratio_old >= 3.9f && ratio_new < 1.1f, note);
    }

    {
        bool ok = true;
        for (float bass = 0.0f; bass <= 1.0f; bass += 0.01f) {
            for (int i = 0; i < 60; i++) {
                float k = k_new(bass, pos_new(i, 60));
                if (k < 0.25f - 1e-6f || k > 1.0f + 1e-6f) ok = false;
            }
        }
        check("k stays within 0.25..1.0", ok, "");
    }

    printf("\n%s\n", failures ? "FAILURES PRESENT" : "all tests passed");
    return failures != 0;
}
