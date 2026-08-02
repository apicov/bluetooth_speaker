/* Live render in the terminal: the strip drawn with 24-bit background colours,
 * the detector's numbers beside it, paced to the audio so it runs at the speed
 * the room would. Redraws one line in place, so it needs a terminal -- if
 * stdout is redirected it says so once and stays quiet. */
#pragma once

#include <cstdint>

#include "analysis.hpp"

class TtyRender {
public:
    void begin(int leds, const char *pattern_name);

    /* One analysis frame. `speed` is a multiplier on real time; 0 means run as
     * fast as the machine can, which is what you want when skimming a track. */
    void frame(const uint8_t *rgb, int leds, const df::Frame &f, double speed);

    void end();

private:
    bool    enabled_    = false;
    int64_t start_us_   = 0;
    int64_t frames_     = 0;
    int64_t onsets_     = 0;
    int     onset_hold_ = 0;
};
