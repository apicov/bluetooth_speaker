#pragma once

#include <cstdint>
#include <string>
#include <vector>

struct Wav {
    int rate = 0;
    int channels = 0;
    std::vector<int16_t> samples;
};

bool wav_read(const std::string &path, Wav &out, std::string &err);
