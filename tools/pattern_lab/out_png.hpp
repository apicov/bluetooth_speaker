#pragma once

#include <cstdint>
#include <string>

bool png_write(const std::string &path, const uint8_t *rgb, int w, int h, std::string &err);
