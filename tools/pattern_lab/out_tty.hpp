/**
 * @file out_tty.hpp
 * @brief Live render in the terminal: the strip in 24-bit background colours,
 *        the detector's numbers beside it, paced to the audio.
 *
 * Redraws one line in place, so it needs a terminal. If stdout is redirected
 * the whole render turns itself off after saying so once, rather than filling
 * a pipe with escape codes.
 */
#pragma once

#include <cstdint>

#include "analysis.hpp"

/** @brief One run's terminal render. Call begin(), then frame() per analysis
 *         frame, then end(). */
class TtyRender {
public:
    /**
     * @brief Decide whether to render at all, and print the header if so.
     *
     * @param leds          Strip length, for the header line.
     * @param pattern_name  Which pattern is being run, for the same.
     */
    void begin(int leds, const char *pattern_name);

    /**
     * @brief Draw one frame, having first slept until it is due.
     *
     * @param rgb    @p leds pixels, three bytes each, already brightness-scaled.
     * @param leds   How many.
     * @param f      The frame behind them; its bands, flux and onset are shown.
     * @param speed  Multiplier on real time. 0 renders as fast as the machine
     *               manages, which is what you want when skimming a track.
     */
    void frame(const uint8_t *rgb, int leds, const df::Frame &f, double speed);

    /** @brief Close the line and print the frame and onset totals. */
    void end();

private:
    bool    enabled_    = false;   /**< False when stdout is not a terminal; everything above is then a no-op. */
    int64_t start_us_   = 0;       /**< now_us() at begin(), the origin frame() paces against. */
    int64_t frames_     = 0;       /**< Frames drawn so far. */
    int64_t onsets_     = 0;       /**< Onsets seen so far. */
    int     onset_hold_ = 0;       /**< Frames the onset marker stays lit for. */
};
