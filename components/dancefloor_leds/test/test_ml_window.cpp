/*
 * The slow lane's grid, checked for the property that keeps two units together.
 *
 * Two units must cut the SAME windows out of the same audio and label them with
 * the same instant. They do not exchange anything, so nothing corrects a
 * disagreement -- it simply persists, and shows up as two strips that respond to
 * different moments in the music.
 *
 * The scenarios below feed the same audio in different-sized pieces, which is
 * what actually varies between units: the analysis task hands the lane a hop at
 * a time, but how many the lane collects per wakeup depends on scheduling. That
 * must make no difference at all.
 */
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "ml_window.hpp"

namespace {

int failures = 0;

void check(const char *name, bool cond, const std::string &detail)
{
    std::printf("%-52s %s  %s\n", name, cond ? "PASS" : "FAIL", detail.c_str());
    if (!cond) failures++;
}

constexpr int RATE   = 16000;
constexpr int WINDOW = 400;
constexpr int HOP    = 160;

struct Seen {
    int64_t index;
    int64_t due_us;
    int16_t first;      /* the window's first sample, to prove WHAT it holds */
};

/*
 * Push `n` samples through in pieces of `chunk`, recording every window.
 *
 * The samples are their own absolute position, so a window's first sample IS
 * the position it starts at -- which is what makes "did both units cut the same
 * audio" checkable rather than merely plausible.
 */
std::vector<Seen> run(int n, int chunk, int64_t origin_us = 0)
{
    df::SlowWindow g;
    g.configure(WINDOW, HOP, RATE);
    g.restart(origin_us);

    std::vector<Seen> seen;
    std::vector<int16_t> buf(n);
    for (int i = 0; i < n; i++) buf[i] = (int16_t)i;

    for (int i = 0; i < n; i += chunk) {
        const int take = (n - i) < chunk ? (n - i) : chunk;
        g.push(buf.data() + i, take,
               [&](const int16_t *w, int64_t index, int64_t due_us) {
                   seen.push_back({ index, due_us, w[0] });
               });
    }
    return seen;
}

void test_windows_land_on_the_hop_grid()
{
    const auto s = run(16000, 160);
    bool ok = !s.empty();
    size_t bad = 0;
    for (size_t k = 0; k < s.size(); k++) {
        /* Window k starts at sample k*HOP, and the samples carry their own
         * position, so this checks the CONTENT and not just the label. */
        if (s[k].first != (int16_t)(k * HOP) || s[k].index != (int64_t)k) {
            ok = false;
            bad = k;
            break;
        }
    }
    char d[128];
    std::snprintf(d, sizeof(d), ok ? "%zu windows, every one on its hop"
                                   : "window %zu holds the wrong audio",
                  ok ? s.size() : bad);
    check("every window starts on the hop grid", ok, d);
}

void test_due_us_counts_from_the_origin()
{
    const int64_t origin = 1234567;
    const auto s = run(16000, 160, origin);

    bool ok = true;
    for (size_t k = 0; k < s.size(); k++) {
        const int64_t want = origin + (int64_t)k * HOP * 1000000LL / RATE;
        if (s[k].due_us != want) { ok = false; break; }
    }
    check("due_us is the origin plus counted samples", ok,
          "derived by counting, never read from a clock");
}

void test_chunking_makes_no_difference()
{
    /* The same audio, delivered in wildly different pieces -- which is exactly
     * what differs between two boards whose tasks were scheduled differently. */
    const auto a = run(16000, 160);
    const auto b = run(16000, 37);
    const auto c = run(16000, 4096);

    bool ok = a.size() == b.size() && a.size() == c.size();
    if (ok) {
        for (size_t k = 0; k < a.size(); k++) {
            if (a[k].index != b[k].index || a[k].due_us != b[k].due_us ||
                a[k].first != b[k].first ||
                a[k].index != c[k].index || a[k].due_us != c[k].due_us ||
                a[k].first != c[k].first) {
                ok = false;
                break;
            }
        }
    }
    char d[160];
    std::snprintf(d, sizeof(d), "%zu / %zu / %zu windows at chunk 160 / 37 / 4096",
                  a.size(), b.size(), c.size());
    check("the delivery chunk size changes nothing", ok, d);
}

void test_nothing_before_an_origin()
{
    df::SlowWindow g;
    g.configure(WINDOW, HOP, RATE);
    /* No restart() -- there is no timeline yet. */

    std::vector<int16_t> buf(16000, 1000);
    int emitted = 0;
    g.push(buf.data(), (int)buf.size(),
           [&](const int16_t *, int64_t, int64_t) { emitted++; });

    check("no window is emitted without an origin", emitted == 0,
          "a window with no timeline could only be labelled with a guess");
}

void test_restart_drops_the_old_timeline()
{
    df::SlowWindow g;
    g.configure(WINDOW, HOP, RATE);
    g.restart(0);

    std::vector<int16_t> old_audio(300, 7);      /* a partial window */
    g.push(old_audio.data(), 300, [](const int16_t *, int64_t, int64_t) {});

    /* New timeline. The 300 samples held must not appear in its first window. */
    g.restart(5000000);
    std::vector<int16_t> fresh(WINDOW, 42);
    int16_t first_sample = -1;
    int64_t first_due = -1;
    g.push(fresh.data(), WINDOW,
           [&](const int16_t *w, int64_t, int64_t due) {
               if (first_sample < 0) { first_sample = w[0]; first_due = due; }
           });

    check("a restart drops what was held", first_sample == 42 && first_due == 5000000,
          "audio from a dead timeline must not lead the first new window");
}

void test_a_window_holds_contiguous_audio()
{
    const auto s = run(16000, 160);
    /* Re-run capturing whole windows, to prove the tail slide keeps a suffix
     * rather than the right number of the wrong samples. */
    df::SlowWindow g;
    g.configure(WINDOW, HOP, RATE);
    g.restart(0);

    std::vector<int16_t> buf(16000);
    for (int i = 0; i < 16000; i++) buf[i] = (int16_t)i;

    bool ok = true;
    g.push(buf.data(), 16000, [&](const int16_t *w, int64_t, int64_t) {
        for (int j = 1; j < WINDOW; j++) {
            if ((int16_t)(w[j] - w[j - 1]) != 1) { ok = false; return; }
        }
    });
    char d[128];
    std::snprintf(d, sizeof(d), "%zu windows, each %d contiguous samples",
                  s.size(), WINDOW);
    check("a window holds contiguous audio", ok, d);
}

}  // namespace

int main()
{
    std::printf("window %d, hop %d, rate %d\n\n", WINDOW, HOP, RATE);

    test_windows_land_on_the_hop_grid();
    test_due_us_counts_from_the_origin();
    test_chunking_makes_no_difference();
    test_nothing_before_an_origin();
    test_restart_drops_the_old_timeline();
    test_a_window_holds_contiguous_audio();

    std::printf("\n%s\n", failures ? "FAILURES" : "all tests passed");
    return failures ? 1 : 0;
}
