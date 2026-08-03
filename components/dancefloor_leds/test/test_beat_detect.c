/*
 * Host-side tests for onset detection.
 *   make check
 *
 * Frames are 512 samples at 44.1 kHz -> 11.61 ms hop, so 43 frames is ~0.5 s,
 * which is one beat at 120 BPM.
 */
#include "beat_detect.h"

#include <stdio.h>
#include <string.h>

#define HOP_US 11610
#define FRAMES_PER_BEAT_120 43

static int failures = 0;

static void check(const char *name, bool cond, const char *detail)
{
    printf("%-46s %s%s%s\n", name, cond ? "PASS" : "FAIL",
           detail ? "  " : "", detail ? detail : "");
    if (!cond) failures++;
}

static uint32_t rng_state = 0xbeef1234u;
static float noise(float amp)
{
    rng_state = rng_state * 1664525u + 1013904223u;
    return ((float)((rng_state >> 8) & 0xffff) / 65535.0f - 0.5f) * 2.0f * amp;
}

/* A kick: low bands spike, then decay over the following frames. */
static void kick_frame(float band[BEAT_BANDS], int frames_since, float level)
{
    float env = (frames_since < 0) ? 0.0f : level / (1.0f + (float)frames_since * 1.5f);
    band[0] = 0.05f + env;
    band[1] = 0.04f + env * 0.5f;
    band[2] = 0.03f + env * 0.15f;
    band[3] = 0.02f + env * 0.05f;
}

int main(void)
{
    /* 1. Silence must never produce onsets, even though flux variance is zero
     *    and the adaptive threshold collapses. This is what BEAT_FLUX_FLOOR is for. */
    {
        beat_det_t d; beat_det_init(&d);
        int onsets = 0;
        for (int i = 0; i < 400; i++) {
            float band[BEAT_BANDS] = {0};
            if (beat_det_update(&d, band, (int64_t)i * HOP_US, NULL)) onsets++;
        }
        char s[48]; snprintf(s, sizeof s, "onsets=%d", onsets);
        check("silence produces no onsets", onsets == 0, s);
    }

    /* 2. Near-silence with dither noise must also stay quiet. */
    {
        beat_det_t d; beat_det_init(&d);
        int onsets = 0;
        for (int i = 0; i < 400; i++) {
            float band[BEAT_BANDS];
            for (int b = 0; b < BEAT_BANDS; b++) band[b] = 0.002f + noise(0.001f);
            if (beat_det_update(&d, band, (int64_t)i * HOP_US, NULL)) onsets++;
        }
        char s[48]; snprintf(s, sizeof s, "onsets=%d", onsets);
        check("dither noise produces no onsets", onsets == 0, s);
    }

    /* 3. A sustained tone is one onset at its start, not a continuous stream. */
    {
        beat_det_t d; beat_det_init(&d);
        int onsets = 0;
        for (int i = 0; i < 400; i++) {
            float band[BEAT_BANDS];
            float lvl = (i < 50) ? 0.0f : 0.5f;
            for (int b = 0; b < BEAT_BANDS; b++) band[b] = lvl;
            if (beat_det_update(&d, band, (int64_t)i * HOP_US, NULL)) onsets++;
        }
        char s[48]; snprintf(s, sizeof s, "onsets=%d", onsets);
        check("sustained tone gives a single onset", onsets == 1, s);
    }

    /* 4. The real case: a 120 BPM kick over ~20 s should find ~40 beats. */
    {
        beat_det_t d; beat_det_init(&d);
        int onsets = 0, total_beats = 0;
        int last_kick = -1000;
        for (int i = 0; i < FRAMES_PER_BEAT_120 * 40; i++) {
            if (i % FRAMES_PER_BEAT_120 == 0) { last_kick = i; total_beats++; }
            float band[BEAT_BANDS];
            kick_frame(band, i - last_kick, 0.6f);
            for (int b = 0; b < BEAT_BANDS; b++) band[b] += noise(0.004f);
            if (beat_det_update(&d, band, (int64_t)i * HOP_US, NULL)) onsets++;
        }
        char s[80]; snprintf(s, sizeof s, "found %d of %d beats", onsets, total_beats);
        /* The first few frames are spent priming the history, so allow a small miss. */
        check("120 BPM kick detected", onsets >= total_beats - 2 && onsets <= total_beats, s);
    }

    /* 5. Refractory: a doubled hit 2 frames apart must count once, not twice. */
    {
        beat_det_t d; beat_det_init(&d);
        int onsets = 0;
        int last_kick = -1000;
        for (int i = 0; i < 400; i++) {
            if (i == 200 || i == 202) last_kick = i;
            float band[BEAT_BANDS];
            kick_frame(band, i - last_kick, 0.6f);
            if (beat_det_update(&d, band, (int64_t)i * HOP_US, NULL)) onsets++;
        }
        char s[48]; snprintf(s, sizeof s, "onsets=%d", onsets);
        check("refractory suppresses a double hit", onsets == 1, s);
    }

    /* 6. A slow fade-in is not an onset. This is the adaptive threshold's job:
     *    a fixed threshold would fire somewhere on the way up. */
    {
        beat_det_t d; beat_det_init(&d);
        int onsets = 0;
        for (int i = 0; i < 400; i++) {
            float band[BEAT_BANDS];
            float lvl = 0.8f * (float)i / 400.0f;
            for (int b = 0; b < BEAT_BANDS; b++) band[b] = lvl;
            if (beat_det_update(&d, band, (int64_t)i * HOP_US, NULL)) onsets++;
        }
        char s[48]; snprintf(s, sizeof s, "onsets=%d", onsets);
        check("slow fade-in produces no onsets", onsets == 0, s);
    }

    /* 7. Strength must be populated and in range, since it scales LED brightness. */
    {
        beat_det_t d; beat_det_init(&d);
        float max_s = -1.0f;
        bool any = false;
        int last_kick = -1000;
        for (int i = 0; i < 400; i++) {
            if (i % FRAMES_PER_BEAT_120 == 0) last_kick = i;
            float band[BEAT_BANDS], s;
            kick_frame(band, i - last_kick, 0.6f);
            if (beat_det_update(&d, band, (int64_t)i * HOP_US, &s)) {
                any = true;
                if (s > max_s) max_s = s;
                if (s < 0.0f || s > 1.0f) { max_s = 99.0f; break; }
            }
        }
        char s[48]; snprintf(s, sizeof s, "max=%.3f", max_s);
        check("strength stays within 0..1", any && max_s >= 0.0f && max_s <= 1.0f, s);
    }

    /* ---- beat_normalise(): the band scaling the visualiser feeds in ---- */

    {
        /* The bug this replaced: a hard clamp at 1.0. Flux counts increases, so
         * once a band pinned, a louder kick produced a rise of exactly zero and
         * vanished from the detector. Measured against the real FFT and gain,
         * the bass band clamped above about -11 dBFS -- most mastered music. */
        bool ok = true;
        float worst = 1e9f;
        for (float v = 0.5f; v < 40.0f; v *= 1.3f) {
            float rise = beat_normalise(v * 1.2f) - beat_normalise(v);
            if (rise <= 0.0f) ok = false;
            if (rise < worst) worst = rise;
        }
        char m[64]; snprintf(m, sizeof m, "smallest rise=%.5f", worst);
        check("a 20 pct louder band always rises, however loud", ok, m);

        /* The same case stated as the old code would have failed it. */
        float clamped_lo = 3.0f > 1.0f ? 1.0f : 3.0f;
        float clamped_hi = 3.6f > 1.0f ? 1.0f : 3.6f;
        check("hard clamp would have given zero rise here",
              clamped_hi - clamped_lo == 0.0f
              && beat_normalise(3.6f) - beat_normalise(3.0f) > 0.0f, "");
    }
    {
        bool mono = true, bounded = true;
        float prev = -1.0f;
        for (float v = 0.0f; v < 100.0f; v += 0.05f) {
            float n = beat_normalise(v);
            if (n < prev) mono = false;
            if (n < 0.0f || n >= 1.0f) bounded = false;
            prev = n;
        }
        check("normalise is monotonic", mono, "");
        check("normalise stays in [0,1)", bounded, "");
        check("normalise handles zero and negatives",
              beat_normalise(0.0f) == 0.0f && beat_normalise(-5.0f) == 0.0f, "");
    }

    /*
     * The flux floor is what separates a bass-drum detector from a detector
     * that follows whatever leaked into the bass.
     *
     * A sharp transient anywhere leaks into the low bins -- a windowed FFT
     * cannot prevent it -- so accordion stabs and the like raise the low band a
     * little even with no drum in the music at all. Measured on synthetic
     * forró: a real zabumba produced a low-band flux of 0.17 to 0.49, and the
     * leakage with the drum removed entirely produced 0.02 to 0.11. Everything
     * hangs on the detector telling those apart, and the only thing that does
     * is the floor beneath the adaptive threshold.
     *
     * Both cases run here, so the test states the difference rather than
     * asserting the new behaviour: the default floor CANNOT reject the leakage,
     * whatever else is tuned.
     */
    {
        beat_det_t leaky, floored;
        beat_det_init(&leaky);                  /* default floor, 0.02 */
        beat_det_init(&floored);
        floored.flux_floor = 0.15f;             /* what the boom detector uses */

        int leaky_hits = 0, floored_hits = 0;
        float band[BEAT_BANDS] = {0};
        for (int i = 0; i < 600; i++) {
            /* Low band twitching at the level pure leakage produces: a rise of
             * ~0.09, well above the default floor and well below a drum. */
            band[0] = (i % 12 == 0) ? 0.09f + noise(0.004f) : noise(0.004f);
            int64_t t = (int64_t)i * 23220;     /* one analysis frame at 44.1 kHz */
            float s;
            if (beat_det_update(&leaky, band, t, &s))   leaky_hits++;
            if (beat_det_update(&floored, band, t, &s)) floored_hits++;
        }
        char d[80];
        snprintf(d, sizeof d, "default floor fired %d, raised floor fired %d",
                 leaky_hits, floored_hits);
        check("leakage-sized rises get through the default floor", leaky_hits > 5, d);
        check("a raised flux floor rejects them", floored_hits == 0, d);
    }
    {
        /* And it must not reject a real drum: the same detector, fed rises the
         * size a zabumba actually produced, still fires on every one. */
        beat_det_t floored;
        beat_det_init(&floored);
        floored.flux_floor = 0.15f;

        int hits = 0, strokes = 0;
        float band[BEAT_BANDS] = {0};
        for (int i = 0; i < 600; i++) {
            const bool hit = (i % 20 == 0);
            if (hit) strokes++;
            band[0] = hit ? 0.45f + noise(0.01f) : noise(0.01f);
            int64_t t = (int64_t)i * 23220;
            float s;
            if (beat_det_update(&floored, band, t, &s)) hits++;
        }
        char d[64];
        snprintf(d, sizeof d, "found %d of %d strokes", hits, strokes);
        check("the raised floor still finds a real drum", hits >= strokes - 2, d);
    }

    printf("\n%s\n", failures ? "FAILURES PRESENT" : "all tests passed");
    return failures != 0;
}
