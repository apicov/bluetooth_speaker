/**
 * @file out_png.hpp
 * @brief The whole track as an image: one row per analysis frame, one column
 *        per LED.
 *
 * Time runs downward, which is what makes the picture worth having -- a missed
 * beat is a gap in the stripes and a dead pixel is a blank column, both
 * visible at a glance over a track that would take minutes to watch.
 */
#pragma once

#include <cstdint>
#include <string>

/**
 * @brief Write an RGB buffer out as a truecolour PNG.
 *
 * Deflated through zlib at level 6, with every scanline carrying filter type 0
 * -- flat colour compresses well enough that a filter worth choosing would buy
 * nothing here.
 *
 * @param path  Where to write; an existing file is overwritten.
 * @param rgb   @p w * @p h pixels, three bytes each, row-major.
 * @param w     Columns, one per LED. Must be positive.
 * @param h     Rows, one per analysis frame. Must be positive.
 * @param err   Set to a short reason when this returns false.
 * @return True if the file was written and closed without error.
 */
bool png_write(const std::string &path, const uint8_t *rgb, int w, int h, std::string &err);
