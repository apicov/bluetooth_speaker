/*
 * The slow lane's grid: turning a stream of decimated samples into windows that
 * every unit cuts at the same positions and labels with the same instant.
 *
 * This is the analyser-side twin of the block alignment in visualiser.cpp, and
 * it exists for the same reason. Two units must cut the SAME windows out of the
 * same audio, or a transient near a boundary lands inside one unit's window and
 * outside its neighbour's, and the two models answer differently about music
 * that was identical.
 *
 * The whole of it is counting. An origin is established once -- the instant the
 * first sample after a restart is heard -- and every window after that is
 * located by adding up samples, never by reading a clock. That is what makes
 * the grid a property of the AUDIO rather than of when each board got round to
 * it.
 *
 * Header-only, no clock, no task, no FreeRTOS, so tools/pattern_lab and the
 * unit tests drive exactly the code the boards run -- the same reason
 * result_latch.hpp is where it is.
 */
#pragma once

#include <stdint.h>

#include <cstring>

namespace df {

/*
 * The longest window the slow lane will hold, in samples at the analyser's rate.
 *
 * 512 is 32 ms at 16 kHz, which is longer than any sensible feature frame. It
 * is deliberately NOT big enough for a model's whole context: an analyser with
 * a second of context declares a short window and accumulates internally, which
 * is what keeps this buffer at 1 kB instead of 32 kB. See Mood in analysers.cpp
 * for the shape, and the note there for why it matters on a satellite.
 */
constexpr int ML_WINDOW_MAX = 512;

class SlowWindow {
public:
    /*
     * Both lengths, and the rate they are counted at. Clamped rather than
     * rejected: a spec asking for more than ML_WINDOW_MAX is a build-time
     * mistake, and running on a shorter window is a great deal easier to
     * diagnose than a buffer overrun.
     */
    void configure(int window_n, int hop_n, int rate_hz)
    {
        window_n_ = window_n > ML_WINDOW_MAX ? ML_WINDOW_MAX : window_n;
        if (window_n_ < 1) window_n_ = 1;
        hop_n_ = hop_n < window_n_ ? hop_n : window_n_;
        if (hop_n_ < 1) hop_n_ = 1;
        rate_hz_ = rate_hz > 0 ? rate_hz : 16000;
        reset();
    }

    /*
     * A new timeline starts here, and its first sample is heard at `origin_us`.
     *
     * Everything held is from before it and is dropped -- keeping it would put
     * audio from the old timeline at the front of the first new window, which
     * is the one window nothing downstream could tell was wrong.
     */
    void restart(int64_t origin_us)
    {
        origin_us_ = origin_us;
        have_origin_ = true;
        reset();
    }

    int  window_n() const { return window_n_; }
    int  hop_n() const { return hop_n_; }
    bool ready() const { return have_origin_; }

    /*
     * Push `n` samples and call `fn(window, index, due_us)` for each complete
     * window they finish.
     *
     * `due_us` names the instant the window's FIRST sample is heard, counted
     * from the origin -- the same convention df::Frame::due_us uses, so a
     * result and a frame naming the same instant mean the same thing.
     *
     * Nothing is emitted before restart() has given an origin: a window with no
     * timeline behind it could only be labelled with a guess.
     */
    template <typename F>
    void push(const int16_t *in, int n, F &&fn)
    {
        if (!have_origin_ || !in) {
            return;
        }
        while (n > 0) {
            const int take = (window_n_ - filled_) < n ? (window_n_ - filled_) : n;
            std::memcpy(buf_ + filled_, in, (size_t)take * sizeof(int16_t));
            filled_ += take;
            in += take;
            n -= take;

            if (filled_ < window_n_) {
                return;
            }

            const int64_t due_us =
                origin_us_ + consumed_ * 1000000LL / (int64_t)rate_hz_;
            fn(static_cast<const int16_t *>(buf_), index_, due_us);

            /* Advance one hop, keeping the newest tail. The buffer must remain a
             * contiguous SUFFIX of what has been pushed -- every check on this
             * grid looks at where a window starts rather than what is in it, so
             * keeping the oldest instead would leave the right number of samples
             * holding the wrong audio and nothing would report it. */
            const int tail = window_n_ - hop_n_;
            if (tail > 0) {
                std::memmove(buf_, buf_ + hop_n_, (size_t)tail * sizeof(int16_t));
            }
            filled_ = tail;
            consumed_ += hop_n_;
            index_++;
        }
    }

private:
    void reset()
    {
        filled_ = 0;
        consumed_ = 0;
        index_ = 0;
    }

    int16_t buf_[ML_WINDOW_MAX];
    int     window_n_ = ML_WINDOW_MAX;
    int     hop_n_    = ML_WINDOW_MAX;
    int     rate_hz_  = 16000;
    int     filled_   = 0;
    int64_t consumed_ = 0;      /* samples advanced past, since the origin */
    int64_t index_    = 0;      /* windows emitted since the origin */
    int64_t origin_us_ = 0;
    bool    have_origin_ = false;
};

}  // namespace df
