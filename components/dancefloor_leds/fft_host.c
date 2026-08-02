/*
 * Radix-2 complex FFT for the host harness.
 *
 * The firmware uses esp-dsp, which is hardware-assisted and stays as it is --
 * this exists so tools/pattern_lab can run the identical analysis pipeline on a
 * laptop without pulling in the ESP-IDF.
 *
 * Two correct FFTs of the same input agree to float rounding, not bit for bit,
 * so a threshold decision sitting on a knife edge could in principle differ
 * between the harness and the board. That is fine for designing patterns and
 * would not be fine for anything claiming exact equivalence -- if that is ever
 * needed, compile this on the target too and drop esp-dsp.
 *
 * Output is in natural bin order, matching dsps_fft2r_fc32 followed by
 * dsps_bit_rev_fc32, and matching numpy.fft.fft. test_fft.c checks that against
 * a directly evaluated DFT.
 */
#ifndef ESP_PLATFORM

#include <math.h>

#ifndef M_PI                      /* -std=c11 is strict enough to hide it */
#define M_PI 3.14159265358979323846
#endif

void df_fft_radix2(float *d, int n)
{
    /* Decimation in time: bit-reverse the input, then combine in place. */
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            float tr = d[2 * i],     ti = d[2 * i + 1];
            d[2 * i]     = d[2 * j];
            d[2 * i + 1] = d[2 * j + 1];
            d[2 * j]     = tr;
            d[2 * j + 1] = ti;
        }
    }

    for (int len = 2; len <= n; len <<= 1) {
        const double ang = -2.0 * M_PI / (double)len;
        const double wr = cos(ang), wi = sin(ang);
        for (int i = 0; i < n; i += len) {
            double cr = 1.0, ci = 0.0;
            for (int k = 0; k < len / 2; k++) {
                const int a = i + k, b = i + k + len / 2;
                const double ur = d[2 * a],     ui = d[2 * a + 1];
                const double vr = d[2 * b] * cr - d[2 * b + 1] * ci;
                const double vi = d[2 * b] * ci + d[2 * b + 1] * cr;
                d[2 * a]     = (float)(ur + vr);
                d[2 * a + 1] = (float)(ui + vi);
                d[2 * b]     = (float)(ur - vr);
                d[2 * b + 1] = (float)(ui - vi);
                const double nr = cr * wr - ci * wi;
                ci = cr * wi + ci * wr;
                cr = nr;
            }
        }
    }
}

#endif /* !ESP_PLATFORM */
