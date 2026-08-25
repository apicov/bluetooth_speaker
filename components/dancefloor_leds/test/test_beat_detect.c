
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

        check("120 BPM kick detected", onsets >= total_beats - 2 && onsets <= total_beats, s);
    }

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

    {

        bool ok = true;
        float worst = 1e9f;
        for (float v = 0.5f; v < 40.0f; v *= 1.3f) {
            float rise = beat_normalise(v * 1.2f) - beat_normalise(v);
            if (rise <= 0.0f) ok = false;
            if (rise < worst) worst = rise;
        }
        char m[64]; snprintf(m, sizeof m, "smallest rise=%.5f", worst);
        check("a 20 pct louder band always rises, however loud", ok, m);

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

    {
        beat_det_t leaky, floored;
        beat_det_init(&leaky);
        beat_det_init(&floored);
        floored.flux_floor = 0.15f;

        int leaky_hits = 0, floored_hits = 0;
        float band[BEAT_BANDS] = {0};
        for (int i = 0; i < 600; i++) {

            band[0] = (i % 12 == 0) ? 0.09f + noise(0.004f) : noise(0.004f);
            int64_t t = (int64_t)i * 23220;
            float s;
            if (beat_det_update(&leaky, band, t, &s))   leaky_hits++;
            if (beat_det_update(&floored, band, t, &s)) floored_hits++;
        }
        char d[80];
        snprintf(d, sizeof d, "default floor fired %d, raised floor fired %d",
                 leaky_hits, floored_hits);
        check("small rises pass the default floor", leaky_hits > 5, d);
        check("raising the floor rejects them", floored_hits == 0, d);
    }
    {

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
        check("a raised floor still finds large rises", hits >= strokes - 2, d);
    }

    {
        beat_det_t d; beat_det_init(&d);
        float band[BEAT_BANDS] = {0};
        int before = 0, after = 0;

        const int64_t hub_up_us = 3600LL * 1000000LL;
        for (int i = 0; i < 400; i++) {
            const int since = i % FRAMES_PER_BEAT_120;
            kick_frame(band, since, 0.6f);
            float s;
            if (beat_det_update(&d, band, hub_up_us + (int64_t)i * HOP_US, &s)) {
                before++;
            }
        }

        for (int i = 0; i < 400; i++) {
            const int since = i % FRAMES_PER_BEAT_120;
            kick_frame(band, since, 0.6f);
            float s;
            if (beat_det_update(&d, band, (int64_t)i * HOP_US, &s)) {
                after++;
            }
        }

        char det[80];
        snprintf(det, sizeof det, "%d onsets before the reset, %d after",
                 before, after);
        check("a timeline restart does not deafen the detector",
              before > 5 && after >= before - 1, det);
    }
    {

        beat_det_t d; beat_det_init(&d);
        float band[BEAT_BANDS] = {0};
        for (int i = 0; i < 200; i++) {
            kick_frame(band, i % FRAMES_PER_BEAT_120, 0.6f);
            float s;
            beat_det_update(&d, band, 3600LL * 1000000LL + (int64_t)i * HOP_US, &s);
        }
        int hits = 0;
        int64_t prev = 0, closest = -1;
        for (int i = 0; i < 200; i++) {

            kick_frame(band, (i % 2) ? -1 : 0, 0.6f);
            const int64_t t = (int64_t)i * HOP_US;
            float s;
            if (!beat_det_update(&d, band, t, &s)) {
                continue;
            }

            if (hits++ && (closest < 0 || t - prev < closest)) {
                closest = t - prev;
            }
            prev = t;
        }
        char det[96];
        snprintf(det, sizeof det, "%d onsets, closest pair %lld us, window %d us",
                 hits, (long long)closest, BEAT_REFRACTORY_US);
        check("the refractory window survives the restart",
              hits > 1 && closest >= BEAT_REFRACTORY_US, det);
    }

    printf("\n%s\n", failures ? "FAILURES PRESENT" : "all tests passed");
    return failures != 0;
}
