
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <string.h>

void df_fft_radix2(float *d, int n);

static int failures;
static void check(const char *name, int ok, const char *note) {
    printf("%-46s %s  %s\n", name, ok ? "PASS" : "FAIL", note ? note : "");
    if (!ok) failures++;
}

int main(void) {
    enum { N = 256 };
    static float in[N], buf[2 * N];
    double worst = 0.0;

    srand(7);
    for (int i = 0; i < N; i++) {
        in[i] = (float)((rand() / (double)RAND_MAX) * 2.0 - 1.0);
    }
    for (int i = 0; i < N; i++) { buf[2 * i] = in[i]; buf[2 * i + 1] = 0.0f; }
    df_fft_radix2(buf, N);

    for (int k = 0; k < N; k++) {
        double re = 0, im = 0;
        for (int t = 0; t < N; t++) {
            double a = -2.0 * M_PI * k * t / N;
            re += in[t] * cos(a);
            im += in[t] * sin(a);
        }
        double dr = buf[2 * k] - re, di = buf[2 * k + 1] - im;
        double err = sqrt(dr * dr + di * di) / (sqrt(re * re + im * im) + 1e-9);
        if (err > worst) worst = err;
    }
    char m[64]; snprintf(m, sizeof m, "worst relative error %.2e", worst);
    check("matches a direct DFT on random input", worst < 1e-4, m);

    {
        const int K = 17;
        for (int i = 0; i < N; i++) {
            double a = 2.0 * M_PI * K * i / N;
            buf[2 * i] = (float)cos(a); buf[2 * i + 1] = 0.0f;
        }
        df_fft_radix2(buf, N);
        double at_k = hypot(buf[2 * K], buf[2 * K + 1]);
        double leak = 0;
        for (int k = 0; k < N / 2; k++) {
            if (k == K) continue;
            double m2 = hypot(buf[2 * k], buf[2 * k + 1]);
            if (m2 > leak) leak = m2;
        }
        char s[80]; snprintf(s, sizeof s, "bin=%.1f  worst other bin=%.4f", at_k, leak);
        check("a pure tone lands in exactly one bin", at_k > N / 2.5 && leak < 0.01, s);
    }

    printf("\n%s\n", failures ? "FAILURES PRESENT" : "all tests passed");
    return failures != 0;
}
