/**
 * @file test_result_latch.cpp
 * @brief Host test for df::ResultLatch: that a slow analyser's answer reaches
 *        the same frame on every unit.
 *
 * The latch is where a result waits for the moment it describes, and its
 * correctness claim is a cross-unit one: every unit latches the same result
 * into the same frame index REGARDLESS of when its own inference finished,
 * which is the one thing about it that genuinely differs per board. So the
 * cases publish at deliberately different times on two simulated units and
 * require identical output.
 *
 * The rest are the failure modes that must not be hidden: a result that
 * arrives too late is COUNTED rather than quietly landing in a later frame, an
 * overrun is counted rather than dropped, a flush drops answers dated against
 * a timeline that no longer exists, and a slot the unit computes itself is
 * left alone.
 *
 * The two analysers at the end are a matched pair, and the second is the
 * control: one keys its answer on the WINDOW, which is shared, and must
 * converge across units; the other keys on its own CALL COUNT, which is not,
 * and the test requires it to be caught.
 */
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "analyser.hpp"
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

struct Unit {
    df::ResultLatch latch;
    std::vector<int> drawn;

    Unit() { latch.set_latched(0, true); }

    void run(int64_t frames, int64_t result_every, int64_t delay_us,
             int64_t publish_lead_frames, int64_t join_at = 0)
    {
        for (int64_t n = 0; n < frames; n++) {

            for (int64_t w = 0; w < frames; w += result_every) {
                const int64_t show = frame_due(w) + delay_us;
                const int64_t publish_at = show - publish_lead_frames * HOP_US;
                if (frame_due(n) <= publish_at && publish_at < frame_due(n + 1)) {
                    latch.publish(0, make(0, frame_due(w), delay_us,
                                          (uint8_t)((w / result_every) % 250)));
                }
            }

            if (n < join_at) {
                continue;
            }
            df::Result out[df::ML_SLOTS];
            for (auto &o : out) o = df::result_none();
            latch.take(frame_due(n), HOP_US, out);
            drawn.push_back(df::result_valid(out[0]) ? (int)out[0].label[0] : -1);
        }
    }
};

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
    latch.publish(0, make(0, frame_due(0), 1000000, 5));

    bool early = false;
    for (int64_t n = 0; n < 40; n++) {
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

    Unit fast, slow;
    fast.run( 900,  43,  400000,  15);
    slow.run( 900,  43,  400000,  1);

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
    early.run(900, 43, 400000, 8,  0);
    late.run(900, 43, 400000, 8,  300);

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

    latch.publish(0, make(0, frame_due(0), 0, 1));
    df::Result out[df::ML_SLOTS];
    for (auto &o : out) o = df::result_none();
    latch.take(frame_due(50), HOP_US, out);

    const uint32_t late = latch.take_late();
    char d[128];
    std::snprintf(d, sizeof(d), "counted %u", (unsigned)late);
    check("a result that missed its frame is counted", late == 1, d);

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
    latch.set_latched(0, false);

    df::Result out[df::ML_SLOTS];
    for (auto &o : out) o = df::result_none();
    out[0] = make(0, frame_due(3), 0, 77);
    latch.take(frame_due(3), HOP_US, out);

    check("a slot computed in the fast lane is not overwritten",
          df::result_valid(out[0]) && out[0].label[0] == 77,
          "the render stage must leave fast-lane results where it found them");
}

void test_overrun_is_counted_not_hidden()
{
    df::ResultLatch latch;
    latch.set_latched(0, true);

    for (uint32_t i = 0; i < df::LATCH_PENDING + 5; i++) {
        latch.publish(0, make(0, frame_due(i), 0, (uint8_t)i));
    }
    const uint32_t over = latch.take_overrun();
    char d[128];
    std::snprintf(d, sizeof(d), "counted %u, expected 5", (unsigned)over);
    check("results dropped by a full slot are counted", over == 5, d);
}

class WindowAnalyser final : public df::Analyser {
public:
    const df::AnalyserSpec &spec() const override { return spec_; }
    bool init(int) override { return true; }
    void reset() override {}
    bool process(const uint8_t (&)[df::SPEC_BINS], int64_t index, int64_t,
                 df::Result *out) override
    {
        out->index = index;
        out->n = 1;
        out->label[0] = (uint8_t)(index % 250);
        out->score[0] = 100;
        return true;
    }
private:
    static constexpr df::AnalyserSpec spec_ = {
        "window", 1, 0, df::Lane::Fast };
};

class CountingAnalyser final : public df::Analyser {
public:
    const df::AnalyserSpec &spec() const override { return spec_; }
    bool init(int) override { calls_ = 0; return true; }
    void reset() override { calls_ = 0; }
    bool process(const uint8_t (&)[df::SPEC_BINS], int64_t index, int64_t,
                 df::Result *out) override
    {
        out->index = index;
        out->n = 1;
        out->label[0] = (uint8_t)(calls_++ % 250);
        out->score[0] = 100;
        return true;
    }
private:
    static constexpr df::AnalyserSpec spec_ = {
        "counting", 2, 0, df::Lane::Fast };
    int64_t calls_ = 0;
};

std::vector<int> run_analyser(df::Analyser &a, int64_t frames, int64_t join)
{
    a.init(RATE / DF_HOP_N);
    std::vector<int> labels;

    const uint8_t spec[df::SPEC_BINS] = {};
    for (int64_t n = join; n < frames; n++) {
        df::Result r = df::result_none();
        if (a.process(spec, n, frame_due(n), &r)) {
            labels.push_back(r.label[0]);
        }
    }
    return labels;
}

bool tails_agree(const std::vector<int> &a, const std::vector<int> &b)
{
    const size_t n = a.size() < b.size() ? a.size() : b.size();
    for (size_t i = 0; i < n; i++) {
        if (a[a.size() - n + i] != b[b.size() - n + i]) return false;
    }
    return n > 0;
}

void test_a_window_keyed_analyser_agrees()
{
    WindowAnalyser a, b;
    const auto early = run_analyser(a, 900, 0);
    const auto late  = run_analyser(b, 900, 300);
    check("an analyser keyed on the window survives a late join",
          tails_agree(early, late),
          "same window in, same answer out, whatever came before");
}

void test_a_call_counting_analyser_is_caught()
{
    CountingAnalyser a, b;
    const auto early = run_analyser(a, 900, 0);
    const auto late  = run_analyser(b, 900, 300);
    check("an analyser keyed on its call count is caught",
          !tails_agree(early, late),
          "never converges, as it must not");
}

}

/**
 * @brief Run every case and report.
 * @return 0 if all held, 1 otherwise, so `make check` fails the build.
 */
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
    std::printf("\n");
    test_a_window_keyed_analyser_agrees();
    test_a_call_counting_analyser_is_caught();

    std::printf("\n%s\n", failures ? "FAILURES" : "all tests passed");
    return failures ? 1 : 0;
}
