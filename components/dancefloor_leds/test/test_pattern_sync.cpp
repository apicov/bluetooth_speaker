/*
 * The determinism rule, enforced mechanically.
 *
 * analysis.hpp states it: a pattern may use only what is in the Frame it is
 * handed. Two units cut the same blocks out of the same audio and derive the
 * same due_us for them -- test_align.c pins that half -- so if what they render
 * depends only on the Frame stream, they light identically without exchanging
 * anything at all. That is the whole design: nothing about the lights is
 * transmitted, because nothing needs to be.
 *
 * A violation does not look like a bug. Both strips work, agree at first, and
 * separate over minutes into something that reads as a hardware fault. That is
 * expensive to find on a dance floor and nearly free to catch here.
 *
 * Three ways it has actually been broken, and what catches each:
 *
 *   accumulating per render call      a unit that renders a block twice, or one
 *                                     that joined late, never converges
 *   state not derived from the Frames replaying identical blocks gives a
 *                                     different answer the second time
 *   history that outlives a gap       a unit that missed blocks stays different
 *                                     once the stream is identical again
 *
 * DriftPattern below is deliberately wrong in the first way -- the hue advances
 * per render and the envelope decays per render, both of which look perfectly
 * reasonable in isolation. The test REQUIRES it to fail. A run where it passes
 * means these checks have stopped being able to see the fault, which is worth
 * knowing before they are trusted to guard a real pattern.
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
/*
 * A fixed amount of AUDIO, not a fixed number of frames.
 *
 * 900 frames is ~21 s at hop 1024 but only ~10 s at hop 512, and the signal
 * below is kicks at a fixed rate -- so holding the frame count fixed would quietly
 * halve the number of kicks the detector is shown every time the hop halved, and
 * the "signal actually produces onsets" guard would fail for a reason that has
 * nothing to do with what it is guarding. Scaling by FFT_N / HOP_N keeps every
 * hop looking at the same ~21 s of the same music, which is also the only way a
 * result at one hop can be compared with a result at another.
 */
constexpr int BLOCKS = 900 * (df::FFT_N / df::HOP_N);   /* ~21 s at 44.1 kHz */
constexpr int KICK_EVERY = df::RATE / 2;        /* 120 BPM */

/*
 * Convergence allowance.
 *
 * A unit that joined late or missed audio carries different history for a
 * while: the detector holds BEAT_HIST frames of flux, and the pattern's
 * envelope is whatever the last onset left. Both are bounded, so the units are
 * required to agree EXACTLY after this many identical blocks -- not to agree
 * approximately, and not to agree eventually.
 *
 * One residual risk these checks cannot rule out. beat_det_update() sums its
 * flux history in ARRAY order, and hist_next depends on how many frames that
 * unit has pushed, so two units holding identical history at different
 * rotations sum the same numbers in a different order and can differ in the
 * last bits of the threshold. An onset flips only if flux lands within ~1e-7 of
 * it, which nothing here comes close to, so these pass -- but they pass because
 * the margin is wide, not because the arithmetic is rotation-independent.
 * Summing from the oldest entry rather than from index 0 would make it so.
 */
/* Derived, not 80: BEAT_HIST is swept by `make HIST=...` and a fixed allowance
 * would stop covering the history it exists to outlast. */
constexpr int CONVERGE = BEAT_HIST * 2;

uint32_t rng = 0x1234abcdu;
float noise()
{
    rng = rng * 1664525u + 1013904223u;
    return ((float)(rng >> 8) / 8388608.0f - 1.0f) * 0.01f;
}

/* Kicks over a quiet bed: the detector needs something to find, and the bed
 * keeps the bands from sitting at zero where flux is trivially zero too. */
std::vector<int16_t> make_audio()
{
    /* BLOCKS windows starting every HOP_N, so the last one runs FFT_N past its
     * own start -- not BLOCKS * FFT_N, which is only the same while the windows
     * do not overlap. */
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

/* One speaker: its own analysis state, its own pattern state, its own pixels. */
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

    /* Analyse one block and render it. `renders` > 1 is a unit that rendered
     * the same frame more than once, which must make no difference. */
    void block(const int16_t *audio, int64_t index, int renders = 1)
    {
        const int64_t due = index * df::HOP_N * 1000000LL / df::RATE;
        const df::Frame &f = an.process(audio, index, due, 0);
        for (int i = 0; i < renders; i++) {
            pat->render(f, rgb.data(), LEDS);
        }
    }
};

/*
 * Wrong on purpose: hue advances once per render and the envelope decays once
 * per render, so both are functions of how many times this unit has been
 * called rather than of the audio. Every unit calls a different number of
 * times.
 */
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

/* ---------------------------------------------------------------- scenarios */

/* Both units see every block. Any disagreement at all is state that did not
 * come from the audio. */
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

/* B joins at block `join`, having never seen anything before it. Compared only
 * once both have had CONVERGE identical blocks. */
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

/* B renders every block twice -- a unit whose render loop ran more often than
 * its neighbour's, which the firmware has done before and may do again. */
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

/* B loses a run of blocks in the middle -- a burst of dropped feeds -- and then
 * carries on with the same audio as A. */
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
            continue;                       /* B never sees these */
        }
        b.block(blk, i);
        if (i >= from + len + CONVERGE && !same(a, b)) {
            differ++;
        }
    }
    return differ;
}

}  // namespace

int main(void)
{
    const std::vector<int16_t> pcm = make_audio();

    /* Sanity: the audio has to contain onsets, or every check below passes by
     * rendering nothing interesting. */
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

    /*
     * The same scenarios against boom, which is what the firmware is actually
     * configured to run (CONFIG_DANCEFLOOR_LED_PATTERN) and which had no
     * coverage here at all -- every check above ran pulse.
     *
     * It is also the pattern where a disagreement is worst to look at. Pulse
     * ramps its pixels, so two units differing slightly differ slightly; boom
     * writes full brightness inside `reach` and 15% outside it, so the same
     * disagreement is one strip lit and the other dark.
     */
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

    /*
     * The same scenarios against a pattern that breaks the rule. Each must be
     * CAUGHT, or the checks above are agreeing with the code rather than
     * testing it.
     */
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
