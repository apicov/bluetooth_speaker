/**
 * @file wav.hpp
 * @brief Minimal WAV reader: 16-bit PCM, mono or stereo, and nothing else.
 *
 * Only enough to get a track into df::Analysis. A mono file is widened on the
 * way in, so a caller never has to ask which it got.
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

/** @brief One decoded file: the format it was in, and all of its audio. */
struct Wav {
    int rate = 0;                   /**< Sample rate, as the fmt chunk states it. */
    int channels = 0;               /**< Always 2 once wav_read() has succeeded. */
    std::vector<int16_t> samples;   /**< Interleaved stereo; a mono file appears in both channels. */
};

/**
 * @brief Read a whole WAV file into memory.
 *
 * Walks the RIFF chunks rather than assuming fmt then data, because some
 * writers -- Python's wave module among them -- put LIST or fact chunks
 * between the two. A data chunk shorter than its header claims is accepted at
 * the length that was actually there.
 *
 * @param path  The file to read.
 * @param out   Filled in on success, left alone on failure.
 * @param err   Set to a short reason when this returns false.
 * @return True if a fmt and a data chunk were both found and understood.
 */
bool wav_read(const std::string &path, Wav &out, std::string &err);
