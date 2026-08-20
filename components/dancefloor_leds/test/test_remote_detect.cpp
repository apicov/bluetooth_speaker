/*
 * The remote detector, checked against the half it mirrors.
 *
 * vis_frame_t no longer carries the detector's answers. A unit taking frames
 * from the hub runs df::RemoteDetect over the float bands the frame does
 * carry, and this file is the property that arrangement stands on: the same
 * numbers through the same code give the same answers, bit for bit, as the
 * unit that analysed the audio. If that ever stops holding, a remote strip
 * and the hub's diverge on the marginal onsets -- a sync fault that presents
 * as flaky hardware, which is worth catching on the host instead.
 *
 * The mirror is exact float equality, not a tolerance. Nothing in the compared
 * path rounds: band[] crosses the wire struct as raw bytes, and the two
 * detectors start from init() and see the same frames in the same order, so
 * their states are identical by construction. A difference is not noise; it
 * is a divergence someone introduced on one side or the other.
 *
 * The second property is the one beat_detect.h states for this mode: a unit
 * that MISSES frames disagrees only until the history it lost has turned
 * over, and agrees exactly again after that. The measured figures live in
 * tools/tuning/converge.cpp; what is pinned here is that the property holds
 * structurally, at every hop the Makefile sweeps.
 *
 * THIS TEST GOT MORE LOAD-BEARING, NOT LESS, when the satellites stopped
 * agreeing with each other by construction. The floor is mixed now: an S3
 * satellite analyses its own audio while a classic ESP32 takes frames off the
 * wire, so "every strip agrees" rests on the two paths reaching identical
 * decisions from identical bands -- which is exactly what mirror() asserts. It
 * is the only place that property is checked anywhere, on any machine.
 */
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

/*
 * The wire frame is 32 bytes: 8 + 8 + 4 floats, packed.
 * A change to that number is a protocol change -- the receiver's build guard
 * turns it into a strip that stays dark, which is the designed behaviour, but
 * it wants to be a decision rather than an accident, so the size is pinned
 * here where the frame is exercised, not only where it is declared.
 *
 * It was 96 while the frame also carried spec[], the 64-byte quantised
 * spectrum. Only the pluggable analysers ever read that, so every satellite
 * with DANCEFLOOR_ML off received two thirds of each frame and discarded it.
 */
static_assert(sizeof(vis_frame_t) == 32, "vis_frame_t changed size");

constexpr int BLOCKS = 900 * (df::FFT_N / df::HOP_N);   /* ~21 s at 44.1 kHz */
constexpr int KICK_EVERY = df::RATE / 2;        /* 120 BPM */
constexpr int LOSE_EVERY = 7;                   /* the lossy unit misses ~1 in 7 */

/*
 * How many identical frames a unit that missed one needs before its decisions
 * are the sender's again. Derived, not fixed, for the same reason as in
 * test_pattern_sync: BEAT_HIST is sweepable and a constant allowance would
 * stop covering the history it exists to outlast.
 */
constexpr int CONVERGE = BEAT_HIST * 2;

uint32_t rng = 0x1234abcdu;
float noise()
{
    rng = rng * 1664525u + 1013904223u;
    return ((float)(rng >> 8) / 8388608.0f - 1.0f) * 0.01f;
}

/* Kicks over a quiet bed, as in test_pattern_sync: the detector needs
 * something to find, and the bed keeps the bands off zero where flux would be
 * trivially zero too. */
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

/*
 * The four conversions to_wire() and from_wire() make, mirrored here because
 * visualiser.cpp is firmware and cannot be linked into a host test. The field
 * lists cannot drift silently: a field added to or removed from vis_frame_t
 * breaks this file's compilation, and the size assert above catches anything
 * that compiles anyway.
 */
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
    /* Mirrors from_wire(): the spectrum did not travel, so the receiver's copy
     * is silence rather than whatever the struct happened to hold. */
    std::memset(f.spec, 0, sizeof(f.spec));
    f.mag  = nullptr;
    f.unit = 0;
}

/* Everything a pattern or a diagnostic reads that the wire was supposed to
 * preserve. mag is excluded by design: it never travels and is null on the
 * remote side. spec[] left this list for the same reason -- it stopped
 * travelling, and a unit taking frames cannot run the analysers that read it
 * (DANCEFLOOR_ML depends on LED_SOURCE_LOCAL). No pattern reads either. */
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

/* ---------------------------------------------------------------- scenarios */

/*
 * Every frame delivered. The receiver must agree with the sender on every
 * field, exactly, for the whole run -- this is the property the lighter frame
 * trades on, so any difference at all is a failure.
 */
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

/*
 * Every LOSE_EVERY-th frame lost. The receiver's decisions must come back to
 * the sender's within CONVERGE frames of each loss and stay there until the
 * next one.
 *
 * Compared as decisions, not as full fields, for the reason test_pattern_sync
 * records: the two detectors have pushed different numbers of frames, so
 * their histories sit at different rotations and the threshold sums can
 * differ in its last bits without any decision moving.
 */
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
        /* The frame after a loss is allowed to differ -- that is the whole
         * event -- but the streak must not outlive the history. */
    }
    return streak;    /* must have reconverged by the end */
}

}  // namespace

int main(void)
{
    const std::vector<int16_t> pcm = make_audio();

    int onsets = 0;
    const int differ = mirror(pcm, &onsets);

    char detail[96];
    std::snprintf(detail, sizeof(detail), "%d frames, %d onsets, %d differ",
                  BLOCKS, onsets, differ);
    check("every delivered frame agrees exactly", differ == 0, detail);

    /* A guard against the vacuous pass: if the audio stops producing onsets,
     * agreement on nothing is not evidence of anything. Half the kicks is a
     * loose floor -- the point is that the detector found the signal, not how
     * tightly it tracked it. */
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
