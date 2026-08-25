/**
 * @file out_tty.cpp
 * @brief The escape codes, the bar glyphs and the sleep behind TtyRender.
 *        Declared in out_tty.hpp.
 */
#include "out_tty.hpp"

#include <chrono>
#include <cstdio>
#include <thread>
#include <unistd.h>

namespace {

/**
 * @brief Monotonic microseconds, for pacing.
 *
 * steady_clock rather than system_clock: the render only ever measures
 * intervals, and a clock that can be stepped backwards would stall it.
 *
 * @return Microseconds since an arbitrary origin.
 */
int64_t now_us()
{
    using namespace std::chrono;
    return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

/** @brief Nine glyphs, so a band reads as a height rather than a number. */
const char *BAR[9] = { " ", "▁", "▂", "▃", "▄",
                       "▅", "▆", "▇", "█" };

/**
 * @brief Pick the glyph for a band value.
 * @param v  Nominally 0..1; anything outside is clamped rather than refused.
 * @return One of BAR, never null.
 */
const char *bar(float v)
{
    int i = (int)(v * 8.0f + 0.5f);
    if (i < 0) i = 0;
    if (i > 8) i = 8;
    return BAR[i];
}

}

void TtyRender::begin(int leds, const char *pattern_name)
{
    /* Not a terminal: the carriage returns below would make a mess of a file,
     * and the caller has --png and --csv for that. */
    enabled_ = isatty(STDOUT_FILENO);
    if (!enabled_) {
        std::fprintf(stderr, "stdout is not a terminal -- skipping the live render\n");
        return;
    }
    std::printf("pattern \"%s\", %d LEDs -- bands are kick / low-mid / presence / air\n",
                pattern_name, leds);
    std::fflush(stdout);
    start_us_ = now_us();
}

void TtyRender::frame(const uint8_t *rgb, int leds, const df::Frame &f, double speed)
{
    if (!enabled_) {
        return;
    }
    frames_++;
    if (f.onset) {
        onsets_++;
        onset_hold_ = 3;        /* one frame is under 12 ms; three is visible */
    }

    /* Paced against the audio rather than against a frame counter: due_us is
     * the instant this block would be heard, so sleeping until it keeps the
     * render honest even on a machine slower than the strip. */
    if (speed > 0.0) {
        const int64_t due  = start_us_ + (int64_t)((double)f.due_us / speed);
        const int64_t wait = due - now_us();
        if (wait > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(wait));
        }
    }

    std::fputc('\r', stdout);
    for (int i = 0; i < leds; i++) {
        std::printf("\x1b[48;2;%u;%u;%um ",
                    (unsigned)rgb[3 * i + 0], (unsigned)rgb[3 * i + 1],
                    (unsigned)rgb[3 * i + 2]);
    }
    std::printf("\x1b[0m %6.1fs %s%s%s%s  flux %.3f/%.3f %s\x1b[K",
                (double)f.due_us / 1e6,
                bar(f.band[0]), bar(f.band[1]), bar(f.band[2]), bar(f.band[3]),
                (double)f.flux, (double)f.threshold,
                onset_hold_ > 0 ? "●" : " ");
    std::fflush(stdout);

    if (onset_hold_ > 0) {
        onset_hold_--;
    }
}

void TtyRender::end()
{
    if (!enabled_) {
        return;
    }
    std::printf("\x1b[0m\n%lld frames, %lld onsets\n",
                (long long)frames_, (long long)onsets_);
    std::fflush(stdout);
}
