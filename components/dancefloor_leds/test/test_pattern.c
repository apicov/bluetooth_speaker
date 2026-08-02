/*
 * The strip pattern's geometry: where the bass edge sits and how sharply it
 * turns on. Pure arithmetic, so it is testable on the host even though the
 * renderer itself needs a strip.
 *
 * Two defects prompted this.
 *
 * pos was |i/COUNT - 0.5| * 2, which measures distance from the midpoint of the
 * strip's LENGTH rather than from the midpoint between the end PIXELS. That put
 * pixel 0 at exactly 1.0 and the last pixel at 1 - 2/COUNT -- lopsided -- and
 * since band values cannot exceed 1.0 and the test was a strict >, pixel 0 could
 * never light at any bass level, at any strip length.
 *
 * The test was also a hard on/off, turning an arbitrarily small difference in
 * bass between two units into a 4x brightness difference on the edge pixel.
 */
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#define EDGE_WIDTH 0.15f

static float pos_new(int i, int count) {
    float centre = (count > 1) ? (count - 1) * 0.5f : 1.0f;
    return fabsf((float)i - centre) / centre;
}
static float pos_old(int i, int count) {
    return fabsf((float)i / count - 0.5f) * 2.0f;
}
static float k_new(float bass, float pos) {
    float t = (bass - pos) / EDGE_WIDTH + 0.5f;
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    return 0.25f + 0.75f * t;
}
static float k_old(float bass, float pos) { return bass > pos ? 1.0f : 0.25f; }

static int failures;
static void check(const char *name, bool ok, const char *note) {
    printf("%-52s %s  %s\n", name, ok ? "PASS" : "FAIL", note ? note : "");
    if (!ok) failures++;
}

int main(void) {
    const int counts[] = {8, 30, 60, 144};

    /* band values are capped at just under 1.0 by beat_normalise(), so 1.0 is
     * the least upper bound on bass. */
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

        /* The actual bug: can the outermost pixel ever be brighter than the
         * floor? Old formula, no. New formula, yes. */
        bool old_lights = k_old(bass_max, pos_old(0, n)) > 0.25f;
        bool new_lights = k_new(bass_max, pos_new(0, n)) > 0.25f;
        snprintf(note, sizeof note, "count=%d old=%s new=%s", n,
                 old_lights ? "lights" : "NEVER", new_lights ? "lights" : "NEVER");
        check("outermost pixel can light at full bass", new_lights && !old_lights, note);
    }

    /* The amplification property, which is why the ramp exists. */
    {
        float pos = 0.5f;
        float a = 0.500f, b = 0.505f;         /* two units, 1% apart in bass */
        float ratio_old = k_old(b, pos) / k_old(a, pos);
        float ratio_new = k_new(b, pos) / k_new(a, pos);
        char note[80];
        snprintf(note, sizeof note, "hard=%.2fx  ramp=%.3fx", ratio_old, ratio_new);
        check("a 1 pct bass difference is no longer a 4x brightness difference",
              ratio_old >= 3.9f && ratio_new < 1.1f, note);
    }

    /* Monotonic and bounded, so nothing can exceed the brightness budget. */
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
