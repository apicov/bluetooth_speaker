/* Whole track as an image: one row per analysis frame, one column per LED.
 * Time runs downward, so a missed beat is a gap in the stripes and a dead pixel
 * is a blank column. */
#pragma once

#include <cstdint>
#include <string>

bool png_write(const std::string &path, const uint8_t *rgb, int w, int h, std::string &err);
