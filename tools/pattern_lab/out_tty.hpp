#pragma once

#include <cstdint>

#include "analysis.hpp"

class TtyRender {
public:
    void begin(int leds, const char *pattern_name);

    void frame(const uint8_t *rgb, int leds, const df::Frame &f, double speed);

    void end();

private:
    bool    enabled_    = false;
    int64_t start_us_   = 0;
    int64_t frames_     = 0;
    int64_t onsets_     = 0;
    int     onset_hold_ = 0;
};
