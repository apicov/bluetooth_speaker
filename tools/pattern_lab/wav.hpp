/* Minimal WAV reader: 16-bit PCM, mono or stereo. Enough for the files
 * tools/desktop_satellite.py --record writes, and nothing more. */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

struct Wav {
    int rate = 0;
    int channels = 0;
    std::vector<int16_t> samples;   /* always interleaved stereo after reading */
};

bool wav_read(const std::string &path, Wav &out, std::string &err);
