/*
 * The presentation-delay rule, enforced mechanically.
 *
 * analyser.hpp states it: an analyser's result is shown at
 * `window due_us + a DECLARED constant`, never at whenever the inference
 * happened to finish. ResultLatch is what turns that promise into a moment on
 * the strip, so it is where the promise can be broken.
 *
 * A violation does not look like a bug. Both strips work and both show
 * plausible answers, and they differ only on the frames where one unit's model
 * was a little slower than the other's -- which is a function of what else each
 * board was doing, so it is intermittent, unreproducible, and reads as a radio
 * fault. Exactly the class of failure test_pattern_sync.cpp exists to catch one
 * level down, and caught the same way: run two units that differ in every way
 * except the audio, and require identical output.
 *
 * The scenarios below differ in WHEN each unit publishes -- which is the thing
 * that genuinely varies between an LX6 and an LX7 -- and require the same
 * result to land on the same frame regardless.
 */
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "result_latch.hpp"

namespace {

int failures = 0;

void check(const char *name, bool cond, const std::string &detail)
{
    std::printf("%-52s %s  %s\n", name, cond ? "PASS" : "FAIL", detail.c_str());
    if (!cond) {
        failures++;
    }
}

constexpr int     RATE   = 44100;
constexpr int64_t HOP_US = (int64_t)DF_HOP_N * 1000000 / RATE;

/* The instant frame `n` is heard, on the grid every unit shares. */
int64_t frame_due(int64_t n) { return n * DF_HOP_N * 1000000LL / RATE; }

df::Result make(int slot, int64_t window_due, int64_t delay_us, uint8_t label)
{
    df::Result r = df::result_none();
    r.index      = window_due / (HOP_US ? HOP_US : 1);
    r.analyser   = (uint8_t)slot;
    r.model_id   = 7;
    r.show_at_us = window_due + delay_us;
    r.n          = 1;
    r.label[0]   = label;
    r.score[0]   = 200;
    return r;
}

/*
 * One unit, drawing frames in order and latching as it goes.
 *
 * `publish_lead_frames` is the whole point: how many frames BEFORE a result's
 * show_at_us this unit manages to publish it. A fast board publishes early, a
 * slow one barely in time. Both must draw the same thing.
 */
struct Unit {
    df::ResultLatch latch;
    std::vector<int> drawn;      /* label shown on each frame, -1 for none */

    Unit() { latch.set_latched(0, true); }

    void run(int64_t frames, int64_t result_every, int64_t delay_us,
             int64_t publish_lead_frames, int64_t join_at = 0)
    {
        for (int64_t n = 0; n < frames; n++) {
            /* Publish every result whose show_at_us is `publish_lead_frames`
             * away -- i.e. this unit got round to it now. */
            for (int64_t w = 0; w < frames; w += result_every) {
                const int64_t show = frame_due(w) + delay_us;
                const int64_t publish_at = show - publish_lead_frames * HOP_US;
                if (frame_due(n) <= publish_at && publish_at < frame_due(n + 1)) {
                    latch.publish(0, make(0, frame_due(w), delay_us,
                                          (uint8_t)((w / result_every) % 250)));
                }
            }

            if (n < join_at) {
                continue;               /* not drawing yet */
            }
            df::Result out[df::ML_SLOTS];
            for (auto &o : out) o = df::result_none();
            latch.take(frame_due(n), HOP_US, out);
            drawn.push_back(df::result_valid(out[0]) ? (int)out[0].label[0] : -1);
        }
    }
};

/* Compare the tail both units drew, from the later join onward. */
bool same_tail(const Unit &a, const Unit &b, size_t &first_diff)
{
    const size_t n = a.drawn.size() < b.drawn.size() ? a.drawn.size() : b.drawn.size();
    for (size_t i = 0; i < n; i++) {
        const size_t ai = a.drawn.size() - n + i;
        const size_t bi = b.drawn.size() - n + i;
        if (a.drawn[ai] != b.drawn[bi]) {
            first_diff = i;
            return false;
        }
    }
    return true;
}

void test_shown_at_the_declared_instant()
{
    df::ResultLatch latch;
    latch.set_latched(0, true);

    /* A window at frame 10, declared to be shown 500 ms later. */
    const int64_t window_due = frame_due(10);
    const int64_t delay_us   = 500000;
    latch.publish(0, make(0, window_due, delay_us, 42));

    int first_shown = -1;
    for (int64_t n = 0; n < 4000; n++) {
        df::Result out[df::ML_SLOTS];
        for (auto &o : out) o = df::result_none();
        latch.take(frame_due(n), HOP_US, out);
        if (df::result_valid(out[0]) && first_shown < 0) {
            first_shown = (int)n;
        }
    }

    /* The first frame at or after window_due + delay, and not one before. */
    int64_t want = 0;
    while (frame_due(want) < window_due + delay_us) want++;

    char d[128];
    std::snprintf(d, sizeof(d), "shown on frame %d, declared instant is frame %lld",
                  first_shown, (long long)want);
    check("a result appears on the frame its delay names", first_shown == (int)want, d);
}

void test_nothing_before_its_time()
{
    df::ResultLatch latch;
    latch.set_latched(0, true);
    latch.publish(0, make(0, frame_due(0), 1000000, 5));   /* 1 s delay */

    bool early = false;
    for (int64_t n = 0; n < 40; n++) {                     /* ~460 ms at hop 512 */
        df::Result out[df::ML_SLOTS];
        for (auto &o : out) o = df::result_none();
        latch.take(frame_due(n), HOP_US, out);
        if (df::result_valid(out[0])) early = true;
    }
    check("nothing is shown before its show_at_us", !early,
          "a result held for 1 s must not leak out early");
}

void test_units_publishing_at_different_times_agree()
{
    /* Same audio, same declared delay, wildly different publish timing --
     * which is exactly the difference between two chips running one model. */
    Unit fast, slow;
    fast.run(/*frames*/ 900, /*result_every*/ 43, /*delay*/ 400000, /*lead*/ 15);
    slow.run(/*frames*/ 900, /*result_every*/ 43, /*delay*/ 400000, /*lead*/ 1);

    size_t diff = 0;
    const bool ok = same_tail(fast, slow, diff);
    char d[160];
    std::snprintf(d, sizeof(d), ok ? "%zu frames identical"
                                   : "first difference at frame %zu",
                  ok ? fast.drawn.size() : diff);
    check("two units publishing 14 frames apart draw the same", ok, d);
}

void test_late_join_converges()
{
    Unit early, late;
    early.run(900, 43, 400000, 8, /*join_at*/ 0);
    late.run(900, 43, 400000, 8, /*join_at*/ 300);

    size_t diff = 0;
    const bool ok = same_tail(early, late, diff);
    char d[160];
    std::snprintf(d, sizeof(d), ok ? "%zu frames identical from the join"
                                   : "first difference at frame %zu",
                  ok ? late.drawn.size() : diff);
    check("a unit that joins late converges exactly", ok, d);
}

void test_late_result_is_counted()
{
    df::ResultLatch latch;
    latch.set_latched(0, true);

    /* Drawn well past the instant it named -- the model missed its frame. */
    latch.publish(0, make(0, frame_due(0), 0, 1));
    df::Result out[df::ML_SLOTS];
    for (auto &o : out) o = df::result_none();
    latch.take(frame_due(50), HOP_US, out);

    const uint32_t late = latch.take_late();
    char d[128];
    std::snprintf(d, sizeof(d), "counted %u", (unsigned)late);
    check("a result that missed its frame is counted", late == 1, d);

    /* And one that is on time is not. */
    df::ResultLatch ok_latch;
    ok_latch.set_latched(0, true);
    ok_latch.publish(0, make(0, frame_due(10), 0, 1));
    for (auto &o : out) o = df::result_none();
    ok_latch.take(frame_due(10), HOP_US, out);
    check("a result drawn on its own frame is not", ok_latch.take_late() == 0,
          "on-time results must not inflate the counter");
}

void test_flush_drops_the_old_timeline()
{
    df::ResultLatch latch;
    latch.set_latched(0, true);
    latch.publish(0, make(0, frame_due(0), 0, 9));
    latch.flush();

    df::Result out[df::ML_SLOTS];
    for (auto &o : out) o = df::result_none();
    latch.take(frame_due(100), HOP_US, out);
    check("a flush drops what was waiting", !df::result_valid(out[0]),
          "a show_at_us from a dead timeline must not be drawn");
}

void test_unlatched_slots_are_left_alone()
{
    df::ResultLatch latch;
    latch.set_latched(0, false);        /* this unit computes slot 0 itself */

    df::Result out[df::ML_SLOTS];
    for (auto &o : out) o = df::result_none();
    out[0] = make(0, frame_due(3), 0, 77);   /* already in the frame */
    latch.take(frame_due(3), HOP_US, out);

    check("a slot computed in the fast lane is not overwritten",
          df::result_valid(out[0]) && out[0].label[0] == 77,
          "the render stage must leave fast-lane results where it found them");
}

void test_overrun_is_counted_not_hidden()
{
    df::ResultLatch latch;
    latch.set_latched(0, true);
    /* Publish more than a slot holds without ever draining it. */
    for (uint32_t i = 0; i < df::LATCH_PENDING + 5; i++) {
        latch.publish(0, make(0, frame_due(i), 0, (uint8_t)i));
    }
    const uint32_t over = latch.take_overrun();
    char d[128];
    std::snprintf(d, sizeof(d), "counted %u, expected 5", (unsigned)over);
    check("results dropped by a full slot are counted", over == 5, d);
}

}  // namespace

int main()
{
    std::printf("hop %d, one frame is %lld us\n\n", DF_HOP_N, (long long)HOP_US);

    test_shown_at_the_declared_instant();
    test_nothing_before_its_time();
    test_units_publishing_at_different_times_agree();
    test_late_join_converges();
    test_late_result_is_counted();
    test_flush_drops_the_old_timeline();
    test_unlatched_slots_are_left_alone();
    test_overrun_is_counted_not_hidden();

    std::printf("\n%s\n", failures ? "FAILURES" : "all tests passed");
    return failures ? 1 : 0;
}
