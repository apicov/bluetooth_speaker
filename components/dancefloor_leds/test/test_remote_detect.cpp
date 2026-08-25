
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "analysis.hpp"
#include "visualiser.h"

namespace {

int failures = 0;

void check(const char *name, bool cond, const char *detail)
{
    std::printf("%-52s %s  %s\n", name, cond ? "PASS" : "FAIL", detail);
    if (!cond) {
        failures++;
    }
}

static_assert(sizeof(vis_frame_t) == 32, "vis_frame_t changed size");

constexpr int BLOCKS = 900 * (df::FFT_N / df::HOP_N);
constexpr int KICK_EVERY = df::RATE / 2;
constexpr int LOSE_EVERY = 7;

constexpr int CONVERGE = BEAT_HIST * 2;

uint32_t rng = 0x1234abcdu;
float noise()
{
    rng = rng * 1664525u + 1013904223u;
    return ((float)(rng >> 8) / 8388608.0f - 1.0f) * 0.01f;
}

std::vector<int16_t> make_audio()
{
    const int SAMPLES = (BLOCKS - 1) * df::HOP_N + df::FFT_N;
    std::vector<int16_t> pcm((size_t)SAMPLES * df::CHANNELS);
    int last_kick = -KICK_EVERY;

    for (int i = 0; i < SAMPLES; i++) {
        if (i % KICK_EVERY == 0) {
            last_kick = i;
        }
        const float t   = (float)(i - last_kick) / df::RATE;
        const float env = std::exp(-t / 0.06f);
        const float s   = 0.70f * env * std::sin(2.0f * (float)M_PI * 55.0f * t)
                        + 0.08f * std::sin(2.0f * (float)M_PI * 300.0f * (float)i / df::RATE)
                        + noise();
        const int16_t v = (int16_t)(s * 20000.0f);
        pcm[(size_t)i * 2 + 0] = v;
        pcm[(size_t)i * 2 + 1] = v;
    }
    return pcm;
}

void pack(const df::Frame &f, vis_frame_t *w)
{
    w->due_us = f.due_us;
    w->index  = f.index;
    std::memcpy(w->band, f.band, sizeof(w->band));
}

void unpack(const vis_frame_t *w, df::Frame &f)
{
    f.due_us = w->due_us;
    f.index  = w->index;
    std::memcpy(f.band, w->band, sizeof(f.band));

    std::memset(f.spec, 0, sizeof(f.spec));
    f.mag  = nullptr;
    f.unit = 0;
}

bool same_frame(const df::Frame &a, const df::Frame &b)
{
    return a.onset == b.onset
        && a.boom == b.boom
        && a.strength == b.strength
        && a.boom_strength == b.boom_strength
        && a.flux == b.flux
        && a.threshold == b.threshold
        && a.boom_flux == b.boom_flux
        && a.boom_threshold == b.boom_threshold
        && a.index == b.index
        && a.due_us == b.due_us
        && std::memcmp(a.band, b.band, sizeof(a.band)) == 0;
}

bool same_decision(const df::Frame &a, const df::Frame &b)
{
    return a.onset == b.onset && a.boom == b.boom;
}

int mirror(const std::vector<int16_t> &pcm, int *onsets)
{
    df::Analysis sender;
    sender.init(df::RATE);
    df::RemoteDetect receiver;
    receiver.init();

    int differ = 0;
    *onsets = 0;
    for (int i = 0; i < BLOCKS; i++) {
        const int64_t due = i * df::HOP_N * 1000000LL / df::RATE;
        const df::Frame &src = sender.process(
            &pcm[(size_t)i * df::HOP_N * df::CHANNELS], i, due, 0);

        vis_frame_t w;
        pack(src, &w);
        df::Frame remote;
        unpack(&w, remote);
        receiver.process(remote.band, remote.due_us, &remote);

        *onsets += src.onset ? 1 : 0;
        differ += same_frame(src, remote) ? 0 : 1;
    }
    return differ;
}

int lossy(const std::vector<int16_t> &pcm, int *worst)
{
    df::Analysis sender;
    sender.init(df::RATE);
    df::RemoteDetect full, losing;
    full.init();
    losing.init();

    int streak = 0;
    *worst = 0;
    for (int i = 0; i < BLOCKS; i++) {
        const int64_t due = i * df::HOP_N * 1000000LL / df::RATE;
        const df::Frame &src = sender.process(
            &pcm[(size_t)i * df::HOP_N * df::CHANNELS], i, due, 0);

        vis_frame_t w;
        pack(src, &w);
        df::Frame a, b;
        unpack(&w, a);
        unpack(&w, b);
        full.process(a.band, a.due_us, &a);
        if (i % LOSE_EVERY != 0) {
            losing.process(b.band, b.due_us, &b);
            streak = same_decision(a, b) ? 0 : streak + 1;
            if (streak > *worst) {
                *worst = streak;
            }
        }

    }
    return streak;
}

}

int main(void)
{
    const std::vector<int16_t> pcm = make_audio();

    int onsets = 0;
    const int differ = mirror(pcm, &onsets);

    char detail[96];
    std::snprintf(detail, sizeof(detail), "%d frames, %d onsets, %d differ",
                  BLOCKS, onsets, differ);
    check("every delivered frame agrees exactly", differ == 0, detail);

    const int kicks = ((BLOCKS - 1) * df::HOP_N + df::FFT_N) / KICK_EVERY;
    std::snprintf(detail, sizeof(detail), "%d onsets, %d kicks over %d frames",
                  onsets, kicks, BLOCKS);
    check("the signal actually produces onsets", onsets >= kicks / 2, detail);

    int worst = 0;
    const int tail = lossy(pcm, &worst);
    std::snprintf(detail, sizeof(detail),
                  "worst disagreement streak %d, converged at end: %s",
                  worst, tail == 0 ? "yes" : "no");
    check("a lost frame disagrees only transiently",
          tail == 0 && worst <= CONVERGE, detail);

    if (failures) {
        std::printf("\n%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("\nall green\n");
    return 0;
}
