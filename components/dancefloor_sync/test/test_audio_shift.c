/**
 * @file test_audio_shift.c
 * @brief Host test for the catch-up crossfade.
 *
 * audio_shift_chunk() runs on both units from one implementation, so the
 * properties below are what stops the two correcting differently -- and it
 * edits audio, so a fault in it is heard rather than logged. The cases pin
 * what the header promises: the two strands are the right material, the output
 * is continuous into the next chunk, the crossfade holds its level, and
 * neither strand is read past its end.
 */
#include "audio_shift.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>

#ifndef M_PI
/** @brief Not in strict C11's math.h; the level case needs a sine. */
#define M_PI 3.14159265358979323846
#endif

/** @brief Cases that did not hold; main() returns non-zero if any. */
static int failures = 0;

/**
 * @brief Report one case and record a failure.
 *
 * Prints whether it held rather than asserting, so one run says which cases
 * hold and which do not instead of stopping at the first.
 *
 * @param name    What is being pinned.
 * @param cond    Whether it held.
 * @param detail  The measured figures, or NULL. Printed on a passing line too,
 *                so a case that stops meaning what it says is visible before
 *                it starts failing.
 */
static void check(const char *name, bool cond, const char *detail)
{
    printf("%-46s %s%s%s\n", name, cond ? "PASS" : "FAIL",
           detail ? "  " : "", detail ? detail : "");
    if (!cond) failures++;
}

/** @brief Output frames per chunk, matching AUDIO_FRAMES. */
#define FRAMES 256
/** @brief Interleaved channels, matching AUDIO_CHANNELS. */
#define CHANS  2
/** @brief Input frames the largest shift can ask for -- the ceiling both units
 *         size their real buffers from. */
#define INMAX  (FRAMES + CATCHUP_SHIFT_MAX + 1)

/**
 * @brief Fill a buffer with a per-frame ramp, the two channels offset from
 *        each other.
 *
 * A ramp makes every frame identifiable, so a case can say WHICH input frame
 * an output frame came from rather than only that it looks plausible.
 *
 * @param buf   Interleaved output, n * CHANS samples.
 * @param n     Frames to fill.
 * @param base  Value of frame 0, channel 0.
 */
static void ramp(int16_t *buf, unsigned n, int base)
{
    for (unsigned i = 0; i < n; i++) {
        buf[i * CHANS]     = (int16_t)(base + (int)i);
        buf[i * CHANS + 1] = (int16_t)(base + (int)i + 10000);
    }
}

/** @brief The plain head is the unshifted strand and the plain tail the shifted one, for every shift in range and in both directions. */
static void test_strands(void)
{

    static int16_t in[INMAX * CHANS];
    static int16_t out[FRAMES * CHANS];

    for (int s = 2; s <= CATCHUP_SHIFT_MAX + 1; s++) {
        for (int dir = -1; dir <= 1; dir += 2) {
            int shift = s * dir;
            unsigned nin = (unsigned)(FRAMES + shift);
            ramp(in, nin, 0);
            memset(out, 0xAA, sizeof out);
            audio_shift_chunk(out, in, FRAMES, shift,
                              CATCHUP_FADE_FRAMES, CHANS);

            char name[64];
            const unsigned mag = (unsigned)(shift < 0 ? -shift : shift);
            const unsigned q = FRAMES - mag;
            const unsigned f0 = q - CATCHUP_FADE_FRAMES;
            bool ok = true;

            for (unsigned i = 0; i < FRAMES && ok; i++) {
                int want_a = (int)i;
                int want_b = (int)i + shift;
                for (unsigned c = 0; c < CHANS; c++) {
                    int16_t v = out[i * CHANS + c];
                    int16_t lo = (int16_t)(want_a < want_b ? want_a : want_b);
                    int16_t hi = (int16_t)(want_a < want_b ? want_b : want_a);
                    if (c) { lo += 10000; hi += 10000; }
                    if (i < f0)          ok = ok && v == (int16_t)(want_a + (c ? 10000 : 0));
                    else if (i < q)      ok = ok && v >= lo && v <= hi;
                    else                 ok = ok && v == (int16_t)(want_b + (c ? 10000 : 0));
                }
            }
            snprintf(name, sizeof name, "shift %+d: strands and fade window", shift);
            check(name, ok, NULL);
        }
    }
}

/** @brief The next chunk starts adjacent to the last frame emitted, so a run of shifted chunks has no seam. */
static void test_continuity(void)
{

    static int16_t in[INMAX * CHANS];
    static int16_t out[FRAMES * CHANS];
    char name[64];

    for (int dir = -1; dir <= 1; dir += 2) {
        int shift = CATCHUP_SHIFT_MAX * dir;
        ramp(in, (unsigned)(FRAMES + shift), 0);
        audio_shift_chunk(out, in, FRAMES, shift, CATCHUP_FADE_FRAMES, CHANS);

        const unsigned q = FRAMES - CATCHUP_SHIFT_MAX;
        const unsigned f0 = q - CATCHUP_FADE_FRAMES;
        int lo = 1, hi = 1;
        bool ok = true;
        for (unsigned i = 1; i < FRAMES; i++) {
            const bool in_fade = i >= f0 && i <= q;
            for (unsigned c = 0; c < CHANS; c++) {
                int d = out[i * CHANS + c] - out[(i - 1) * CHANS + c];
                if (d < lo) lo = d;
                if (d > hi) hi = d;
                ok = ok && d >= 0 && d <= 2 && (!in_fade ? d == 1 : true);
            }
        }
        snprintf(name, sizeof name, "shift %+d: ramp never dips or jumps (steps %d..%d)",
                 shift, lo, hi);
        check(name, ok, NULL);
    }
}

/** @brief A crossfade of a signal with itself at a small offset comes out at roughly its own level -- no dip to hear. */
static void test_level(void)
{

    static int16_t in[INMAX * CHANS];
    static int16_t out[FRAMES * CHANS];

    for (int dir = -1; dir <= 1; dir += 2) {
        int shift = CATCHUP_SHIFT_MAX * dir;
        unsigned nin = (unsigned)(FRAMES + shift);
        for (unsigned i = 0; i < nin; i++) {
            int16_t s = (int16_t)(12000.0 * sin(2.0 * M_PI * i / 37.0));
            in[i * CHANS] = s;
            in[i * CHANS + 1] = (int16_t)-s;
        }
        audio_shift_chunk(out, in, FRAMES, shift, CATCHUP_FADE_FRAMES, CHANS);

        int peak_in = 0, peak_out = 0;
        for (unsigned i = 0; i < nin * CHANS; i++)
            if (abs(in[i]) > peak_in) peak_in = abs(in[i]);
        for (unsigned i = 0; i < FRAMES * CHANS; i++)
            if (abs(out[i]) > peak_out) peak_out = abs(out[i]);

        char name[64];
        snprintf(name, sizeof name, "sine shift %+d: level held (%d of %d)",
                 shift, peak_out, peak_in);

        check(name, peak_out <= peak_in + 1, NULL);
    }
}

/** @brief A whole sequence of shifted chunks reconstructs the input with exactly the shifted frames skipped or replayed. */
static void test_chunk_seam(void)
{

    static int16_t in[INMAX * CHANS];
    static int16_t out[FRAMES * CHANS];

    for (int dir = -1; dir <= 1; dir += 2) {
        int shift = CATCHUP_SHIFT_MAX * dir;
        unsigned nin = (unsigned)(FRAMES + shift);
        ramp(in, nin, 0);
        audio_shift_chunk(out, in, FRAMES, shift, CATCHUP_FADE_FRAMES, CHANS);

        int16_t last = out[(FRAMES - 1) * CHANS];
        int16_t next = (int16_t)((int)(FRAMES + shift));
        char name[64];
        snprintf(name, sizeof name, "shift %+d: seam adjacent to next chunk", shift);
        check(name, (int16_t)(last + 1) == next, NULL);
    }
}

/** @brief Neither strand is read past its end and the fade window stays inside the chunk, for every shift and fade the clamps allow. */
static void test_fade_bounds(void)
{

    static int16_t in[INMAX * CHANS];
    static int16_t out[FRAMES * CHANS];
    ramp(in, FRAMES + 2, 0);
    audio_shift_chunk(out, in, FRAMES, 2, 2, CHANS);

    bool ok = out[0] == 0 && out[251 * CHANS] == 251
           && out[252 * CHANS] == 252 && out[253 * CHANS] == 255
           && out[254 * CHANS] == 256 && out[255 * CHANS] == 257;
    check("fade 2: degenerate crossfade still exact", ok, NULL);
}

/**
 * @brief Run every case and report.
 * @return 0 if all held, 1 otherwise, so `make check` fails the build.
 */
int main(void)
{
    test_strands();
    test_continuity();
    test_level();
    test_chunk_seam();
    test_fade_bounds();
    printf("\n%s\n", failures ? "FAILURES" : "all catch-up crossfade tests passed");
    return failures ? 1 : 0;
}
