/**
 * @file test_pattern_sync.cpp
 * @brief Host test for the one rule at the top of analysis.hpp: a Pattern must
 *        be a pure function of the Frames it has been given.
 *
 * Breaking that rule does not fail loudly on a floor -- it fails as strips
 * that agree at first and drift apart over minutes, which is expensive to
 * diagnose. So it is enforced MECHANICALLY here, by running a pattern as two
 * units that differ in every way two real units differ: when they joined, how
 * many times each rendered, and which frames each lost. Byte-identical output
 * is the requirement.
 *
 * DriftPattern is the control, and the reason this test can be trusted: it
 * accumulates per render call, which is one of the ways the rule has actually
 * been broken here, and the test REQUIRES it to fail. A conformance test with
 * no failing case only proves it is not looking.
 */
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "analysis.hpp"
#include "patterns.hpp"

namespace {

int failures = 0;

void check(const char *name, bool cond, const std::string &detail)
{
    std::printf("%-52s %s  %s\n", name, cond ? "PASS" : "FAIL", detail.c_str());
    if (!cond) {
        failures++;
    }
}

constexpr int LEDS   = 30;

constexpr int BLOCKS = 900 * (df::FFT_N / df::HOP_N);
constexpr int KICK_EVERY = df::RATE / 2;

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

struct Unit {
    df::Analysis    an;
    df::Pattern    *pat;
    std::vector<uint8_t> rgb;

    void init(df::Pattern *p)
    {
        an.init(df::RATE);
        pat = p;
        pat->reset();
        rgb.assign(LEDS * 3, 0);
    }

    void block(const int16_t *audio, int64_t index, int renders = 1)
    {
        const int64_t due = index * df::HOP_N * 1000000LL / df::RATE;
        const df::Frame &f = an.process(audio, index, due, 0);
        for (int i = 0; i < renders; i++) {
            pat->render(f, rgb.data(), LEDS);
        }
    }
};

class DriftPattern : public df::Pattern {
public:
    const char *name() const override { return "drift"; }
    void reset() override { hue_ = 0.0f; level_ = 0.0f; }

    void render(const df::Frame &f, uint8_t *rgb, uint32_t count) override
    {
        hue_ = std::fmod(hue_ + 0.31f, 360.0f);
        level_ *= 0.85f;
        if (f.onset) {
            level_ = 1.0f;
        }
        for (uint32_t i = 0; i < count; i++) {
            rgb[3 * i + 0] = (uint8_t)(level_ * 255.0f);
            rgb[3 * i + 1] = (uint8_t)(hue_ / 360.0f * 255.0f);
            rgb[3 * i + 2] = (uint8_t)(f.band[0] * 255.0f);
        }
    }

private:
    float hue_ = 0.0f, level_ = 0.0f;
};

bool same(const Unit &a, const Unit &b)
{
    return std::memcmp(a.rgb.data(), b.rgb.data(), a.rgb.size()) == 0;
}

int replay(const std::vector<int16_t> &pcm, df::Pattern *pa, df::Pattern *pb)
{
    Unit a, b;
    a.init(pa);
    b.init(pb);
    int differ = 0;

    for (int i = 0; i < BLOCKS; i++) {
        const int16_t *blk = &pcm[(size_t)i * df::HOP_N * df::CHANNELS];
        a.block(blk, i);
        b.block(blk, i);
        if (!same(a, b)) {
            differ++;
        }
    }
    return differ;
}

int late_join(const std::vector<int16_t> &pcm, df::Pattern *pa, df::Pattern *pb, int join)
{
    Unit a, b;
    a.init(pa);
    b.init(pb);
    int differ = 0;

    for (int i = 0; i < BLOCKS; i++) {
        const int16_t *blk = &pcm[(size_t)i * df::HOP_N * df::CHANNELS];
        a.block(blk, i);
        if (i < join) {
            continue;
        }
        b.block(blk, i);
        if (i >= join + CONVERGE && !same(a, b)) {
            differ++;
        }
    }
    return differ;
}

int double_render(const std::vector<int16_t> &pcm, df::Pattern *pa, df::Pattern *pb)
{
    Unit a, b;
    a.init(pa);
    b.init(pb);
    int differ = 0;

    for (int i = 0; i < BLOCKS; i++) {
        const int16_t *blk = &pcm[(size_t)i * df::HOP_N * df::CHANNELS];
        a.block(blk, i, 1);
        b.block(blk, i, 2);
        if (!same(a, b)) {
            differ++;
        }
    }
    return differ;
}

int gap(const std::vector<int16_t> &pcm, df::Pattern *pa, df::Pattern *pb,
        int from, int len)
{
    Unit a, b;
    a.init(pa);
    b.init(pb);
    int differ = 0;

    for (int i = 0; i < BLOCKS; i++) {
        const int16_t *blk = &pcm[(size_t)i * df::HOP_N * df::CHANNELS];
        a.block(blk, i);
        if (i >= from && i < from + len) {
            continue;
        }
        b.block(blk, i);
        if (i >= from + len + CONVERGE && !same(a, b)) {
            differ++;
        }
    }
    return differ;
}

}

/**
 * @brief Run every case and report.
 * @return 0 if all held, 1 otherwise, so `make check` fails the build.
 */
int main(void)
{
    const std::vector<int16_t> pcm = make_audio();

    {
        Unit u;
        df::PulsePattern p;
        u.init(&p);
        int onsets = 0;
        for (int i = 0; i < BLOCKS; i++) {
            const int16_t *blk = &pcm[(size_t)i * df::HOP_N * df::CHANNELS];
            const int64_t due = (int64_t)i * df::HOP_N * 1000000LL / df::RATE;
            if (u.an.process(blk, i, due, 0).onset) {
                onsets++;
            }
        }
        check("the test signal actually produces onsets", onsets >= 30,
              std::to_string(onsets) + " onsets in " + std::to_string(BLOCKS) + " blocks");
    }

    std::printf("\npulse -- must agree exactly:\n");
    {
        df::PulsePattern a, b;
        check("  same blocks, two instances", replay(pcm, &a, &b) == 0, "");
    }
    {
        df::PulsePattern a, b;
        const int d = late_join(pcm, &a, &b, 100);
        check("  one unit joins at block 100", d == 0,
              d ? std::to_string(d) + " blocks differ after convergence" : "converged");
    }
    {
        df::PulsePattern a, b;
        const int d = double_render(pcm, &a, &b);
        check("  one unit renders every block twice", d == 0,
              d ? std::to_string(d) + " blocks differ" : "identical");
    }
    {
        df::PulsePattern a, b;
        const int d = gap(pcm, &a, &b, 300, 40);
        check("  one unit loses 40 blocks mid-stream", d == 0,
              d ? std::to_string(d) + " blocks differ after convergence" : "converged");
    }

    std::printf("\nboom -- the pattern the firmware ships, must agree exactly:\n");
    {
        Unit u;
        df::BoomPattern p;
        u.init(&p);
        int booms = 0;
        for (int i = 0; i < BLOCKS; i++) {
            const int16_t *blk = &pcm[(size_t)i * df::HOP_N * df::CHANNELS];
            if (u.an.process(blk, i, (int64_t)i * df::HOP_N * 1000000LL / df::RATE, 0).boom) {
                booms++;
            }
        }
        check("  the test signal produces booms", booms >= 10,
              std::to_string(booms) + " booms in " + std::to_string(BLOCKS) + " blocks");
    }
    {
        df::BoomPattern a, b;
        check("  same blocks, two instances", replay(pcm, &a, &b) == 0, "");
    }
    {
        df::BoomPattern a, b;
        const int d = late_join(pcm, &a, &b, 100);
        check("  one unit joins at block 100", d == 0,
              d ? std::to_string(d) + " blocks differ after convergence" : "converged");
    }
    {
        df::BoomPattern a, b;
        const int d = double_render(pcm, &a, &b);
        check("  one unit renders every block twice", d == 0,
              d ? std::to_string(d) + " blocks differ" : "identical");
    }
    {
        df::BoomPattern a, b;
        const int d = gap(pcm, &a, &b, 300, 40);
        check("  one unit loses 40 blocks mid-stream", d == 0,
              d ? std::to_string(d) + " blocks differ after convergence" : "converged");
    }

    std::printf("\ndrift -- accumulates per render, must be caught:\n");
    {
        DriftPattern a, b;
        check("  replay is still deterministic (expected)", replay(pcm, &a, &b) == 0,
              "a broken pattern can still be reproducible");
    }
    {
        DriftPattern a, b;
        check("  late join is caught", late_join(pcm, &a, &b, 100) > 0,
              "never converges, as it must not");
    }
    {
        DriftPattern a, b;
        check("  double render is caught", double_render(pcm, &a, &b) > 0,
              "render count changes the output");
    }
    {
        DriftPattern a, b;
        check("  lost blocks are caught", gap(pcm, &a, &b, 300, 40) > 0,
              "the gap is never forgotten");
    }

    std::printf(failures ? "\nFAILED\n" : "\nall tests passed\n");
    return failures ? 1 : 0;
}
