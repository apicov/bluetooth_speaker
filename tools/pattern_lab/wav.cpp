#include "wav.hpp"

#include <cstdio>
#include <cstring>

namespace {

bool rd(std::FILE *f, void *p, size_t n) { return std::fread(p, 1, n, f) == n; }

uint32_t le32(const uint8_t *p) { return uint32_t(p[0]) | (p[1] << 8) | (p[2] << 16) | (uint32_t(p[3]) << 24); }
uint16_t le16(const uint8_t *p) { return uint16_t(uint16_t(p[0]) | (p[1] << 8)); }

}  // namespace

bool wav_read(const std::string &path, Wav &out, std::string &err)
{
    std::FILE *f = std::fopen(path.c_str(), "rb");
    if (!f) { err = "cannot open"; return false; }

    uint8_t hdr[12];
    if (!rd(f, hdr, 12) || std::memcmp(hdr, "RIFF", 4) || std::memcmp(hdr + 8, "WAVE", 4)) {
        err = "not a RIFF/WAVE file"; std::fclose(f); return false;
    }

    int bits = 0;
    bool have_fmt = false;

    /* Walk the chunks rather than assuming fmt then data: some writers put
     * LIST or fact chunks in between, and Python's wave module can too. */
    for (;;) {
        uint8_t ch[8];
        if (!rd(f, ch, 8)) { err = have_fmt ? "no data chunk" : "no fmt chunk"; std::fclose(f); return false; }
        const uint32_t size = le32(ch + 4);

        if (!std::memcmp(ch, "fmt ", 4)) {
            std::vector<uint8_t> fmt(size < 16 ? 16 : size, 0);
            if (!rd(f, fmt.data(), size)) { err = "truncated fmt"; std::fclose(f); return false; }
            const uint16_t tag = le16(fmt.data());
            out.channels = le16(fmt.data() + 2);
            out.rate     = int(le32(fmt.data() + 4));
            bits         = le16(fmt.data() + 14);
            if (tag != 1 && tag != 0xFFFE) { err = "not PCM"; std::fclose(f); return false; }
            if (bits != 16) { err = "only 16-bit PCM is supported"; std::fclose(f); return false; }
            if (out.channels < 1 || out.channels > 2) { err = "only mono or stereo"; std::fclose(f); return false; }
            have_fmt = true;
        } else if (!std::memcmp(ch, "data", 4)) {
            if (!have_fmt) { err = "data before fmt"; std::fclose(f); return false; }
            std::vector<int16_t> raw(size / 2);
            const size_t got = std::fread(raw.data(), 2, raw.size(), f);
            raw.resize(got);
            std::fclose(f);
            if (out.channels == 2) {
                out.samples = std::move(raw);
                out.samples.resize(out.samples.size() / 2 * 2);
            } else {
                out.samples.resize(raw.size() * 2);
                for (size_t i = 0; i < raw.size(); i++) {
                    out.samples[2 * i] = out.samples[2 * i + 1] = raw[i];
                }
                out.channels = 2;
            }
            return true;
        } else {
            if (std::fseek(f, long(size + (size & 1)), SEEK_CUR) != 0) {
                err = "truncated file"; std::fclose(f); return false;
            }
        }
    }
}
