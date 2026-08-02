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

    printf("\n%s\n", failures ? "FAILURES PRESENT" : "all tests passed");
    return failures != 0;
}
