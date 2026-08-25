/**
 * @file out_png.cpp
 * @brief A PNG writer in eighty lines, since zlib is already linked for the
 *        deflate and the rest of the format is three chunks. Declared in
 *        out_png.hpp.
 */
#include "out_png.hpp"

#include <cstdio>
#include <cstring>
#include <vector>
#include <zlib.h>

namespace {

/**
 * @brief Append a big-endian 32-bit value.
 *
 * PNG is big-endian throughout, which is the one thing about the format that
 * has to be got right in more than one place.
 *
 * @param v  Grown by four bytes.
 * @param x  The value to append.
 */
void be32(std::vector<uint8_t> &v, uint32_t x)
{
    v.push_back(uint8_t(x >> 24)); v.push_back(uint8_t(x >> 16));
    v.push_back(uint8_t(x >> 8));  v.push_back(uint8_t(x));
}

/**
 * @brief Write one PNG chunk: length, type, payload, CRC.
 *
 * The CRC covers the type and the payload but not the length, which is the
 * detail the format gets wrong in every hand-rolled writer.
 *
 * @param f     Open for binary writing.
 * @param type  Exactly four characters, not NUL-terminated on the wire.
 * @param data  The payload; may be null when @p len is zero.
 * @param len   Payload length.
 */
void chunk(std::FILE *f, const char *type, const uint8_t *data, size_t len)
{
    std::vector<uint8_t> hdr;
    be32(hdr, uint32_t(len));
    std::fwrite(hdr.data(), 1, hdr.size(), f);
    std::fwrite(type, 1, 4, f);
    if (len) std::fwrite(data, 1, len, f);

    uLong c = crc32(0L, reinterpret_cast<const Bytef *>(type), 4);
    if (len) c = crc32(c, data, uInt(len));
    std::vector<uint8_t> crc;
    be32(crc, uint32_t(c));
    std::fwrite(crc.data(), 1, crc.size(), f);
}

}

bool png_write(const std::string &path, const uint8_t *rgb, int w, int h, std::string &err)
{
    if (w <= 0 || h <= 0) { err = "empty image"; return false; }

    /* Every scanline is prefixed with a filter byte, and 0 means "none" --
     * flat colour deflates well enough that choosing per line would buy
     * nothing. */
    std::vector<uint8_t> raw;
    raw.reserve(size_t(h) * (size_t(w) * 3 + 1));
    for (int y = 0; y < h; y++) {
        raw.push_back(0);
        raw.insert(raw.end(), rgb + size_t(y) * w * 3, rgb + size_t(y + 1) * w * 3);
    }

    uLongf zlen = compressBound(uLong(raw.size()));
    std::vector<uint8_t> z(zlen);
    if (compress2(z.data(), &zlen, raw.data(), uLong(raw.size()), 6) != Z_OK) {
        err = "deflate failed"; return false;
    }
    z.resize(zlen);

    std::FILE *f = std::fopen(path.c_str(), "wb");
    if (!f) { err = "cannot open for writing"; return false; }

    static const uint8_t sig[8] = { 137, 'P', 'N', 'G', '\r', '\n', 26, '\n' };
    std::fwrite(sig, 1, 8, f);

    std::vector<uint8_t> ihdr;
    be32(ihdr, uint32_t(w));
    be32(ihdr, uint32_t(h));
    ihdr.push_back(8);
    ihdr.push_back(2);
    ihdr.push_back(0);
    ihdr.push_back(0);
    ihdr.push_back(0);
    chunk(f, "IHDR", ihdr.data(), ihdr.size());
    chunk(f, "IDAT", z.data(), z.size());
    chunk(f, "IEND", nullptr, 0);

    const bool ok = std::ferror(f) == 0;
    std::fclose(f);
    if (!ok) err = "write failed";
    return ok;
}
