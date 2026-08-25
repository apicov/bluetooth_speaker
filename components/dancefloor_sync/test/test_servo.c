
/**
 * @file test_servo.c
 * @brief Host test for the shared rate servo.
 *
 * df_servo.c is deliberately free of FreeRTOS, i2s and timers precisely so it
 * can be driven from here under plain gcc, which is how a change to the loop
 * is checked before any board is flashed. Both units run this same loop, so a
 * case that fails here is a cross-unit sync error waiting to happen.
 *
 * What each case pins is the boundary a soak found the hard way -- the depth
 * net being a floor rather than a replacement, the catch-up hysteresis, the
 * cooldown, the fine/coarse split at RATE_TRIM_MAX_HZ, and the deadband being
 * measured against the trim already applied rather than against zero.
 */
#include "df_servo.h"

#include "audio_shift.h"
#include "sync_proto.h"

#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>

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
    printf("%-52s %s%s%s\n", name, cond ? "PASS" : "FAIL",
           detail ? "  " : "", detail ? detail : "");
    if (!cond) failures++;
}

/** @brief The stream rate every case runs at. Nothing here depends on its
 *         exact value; it only has to be the same on both sides of a
 *         comparison. */
#define RATE 44100

/**
 * @brief A neutral input: phase valid, nothing armed, clock on rate.
 * @return The struct, for a case to perturb one field of.
 */
static df_servo_in_t base(void)
{
    df_servo_in_t in = {
        .phase_valid    = true,
        .med_us         = 0,
        .have_med       = false,
        .catchup_held   = false,
        .depth_ms       = 0,
        .depth_net_held = false,
        .rate           = RATE,
        .tx_rate        = RATE,
        .trim_hz_now    = 0,
        .catchup_now    = 0,
    };
    return in;
}

/**
 * @brief One window: fold in a reading, then decide on it.
 *
 * The two calls in the order the firmware makes them, since df_servo_step()
 * acts on what df_servo_ema() left behind.
 *
 * @param s    The loop's carried state.
 * @param in   This window's measurements.
 * @param err  The phase reading to fold in.
 * @return What the servo decided.
 */
static df_servo_out_t step(df_servo_t *s, df_servo_in_t *in, int32_t err)
{
    df_servo_out_t out;
    df_servo_ema(s, err, false);
    df_servo_step(s, in, &out);
    return out;
}

/** @brief The average adopts its first sample, weights a new one by a quarter, adopts again after a reset, and converges rather than overshooting. */
static void test_ema(void)
{
    df_servo_t s = {0};

    check("ema: first sample adopted", df_servo_ema(&s, 8000, false) == 8000, NULL);

    check("ema: quarter weight on the new sample",
          df_servo_ema(&s, 0, false) == 6000, NULL);

    check("ema: reset adopts rather than averages",
          df_servo_ema(&s, -20000, true) == -20000, NULL);

    df_servo_t t = {0};
    for (int i = 0; i < 40; i++) df_servo_ema(&t, 10000, false);
    check("ema: settles on a standing error", t.err_ema == 10000, NULL);
}

/** @brief The depth net is a FLOOR: it only ever strengthens a correction that already agrees with it, and never caps one. Held off, it does nothing. */
static void test_depth_net(void)
{

    df_servo_t s = {0};
    df_servo_in_t in = base();
    in.depth_ms = 249;
    df_servo_out_t out = step(&s, &in, 83432);
    check("depth net: floor does not cut an agreeing phase",
          out.adj == 36 && out.adj_phase == 36, NULL);
    check("depth net: silent when it did not fire", !out.depth_net_fired, NULL);

    df_servo_t s2 = {0};
    df_servo_in_t in2 = base();
    in2.depth_ms = 249;
    df_servo_out_t out2 = step(&s2, &in2, 0);
    check("depth net: lifts a phase that disagrees",
          out2.adj == 20 && out2.adj_phase == 0 && out2.depth_net_fired, NULL);

    df_servo_t s3 = {0};
    df_servo_in_t in3 = base();
    in3.depth_ms = -249;
    df_servo_out_t out3 = step(&s3, &in3, 0);
    check("depth net: floors a nearly-empty ring at -20 Hz",
          out3.adj == -20 && out3.depth_net_fired, NULL);

    df_servo_t s4 = {0};
    df_servo_in_t in4 = base();
    in4.depth_ms = 119;
    df_servo_out_t out4 = step(&s4, &in4, 0);
    check("depth net: inside +-120 ms it does nothing",
          out4.adj == 0 && !out4.depth_net_fired, NULL);

    df_servo_t s5 = {0};
    df_servo_in_t in5 = base();
    in5.depth_ms = -249;
    in5.depth_net_held = true;
    df_servo_out_t out5 = step(&s5, &in5, 0);
    check("depth net: held off after an anchor",
          out5.adj == 0 && !out5.depth_net_fired, NULL);
}

/** @brief A wild phase reading cannot produce a wild rate: the correction is clamped to RATE_TRIM_MAX_HZ in both directions. */
static void test_clamp(void)
{

    df_servo_t s = {0};
    df_servo_in_t in = base();
    df_servo_out_t out = step(&s, &in, 2000000);
    check("clamp: a wild reading cannot exceed the ceiling",
          out.adj == RATE_TRIM_MAX_HZ, NULL);

    df_servo_t s2 = {0};
    df_servo_in_t in2 = base();
    df_servo_out_t out2 = step(&s2, &in2, -2000000);
    check("clamp: and cannot exceed it downward",
          out2.adj == -RATE_TRIM_MAX_HZ, NULL);
}

/** @brief The catch-up debt: capped at CATCHUP_MAX_US, deepened but not weakened on the same side, replaced on a sign flip, stood down below CATCHUP_CLEAR_US, held off during the start hold, and silent without a phase reading. */
static void test_catchup(void)
{

    df_servo_t s = {0};
    df_servo_in_t in = base();
    in.med_us = 40000;
    in.have_med = true;
    df_servo_out_t out = step(&s, &in, 40000);
    check("catchup: arms from a median past the threshold",
          out.catchup_write && out.catchup_frames_new == 1764, NULL);

    df_servo_t s2 = {0};
    df_servo_in_t in2 = base();
    in2.med_us = 40000;
    in2.have_med = false;
    df_servo_out_t out2 = step(&s2, &in2, 40000);
    check("catchup: no median, no arm", !out2.catchup_write, NULL);

    df_servo_t s3 = {0};
    df_servo_in_t in3 = base();
    in3.med_us = 40000;
    in3.have_med = true;
    in3.catchup_held = true;
    df_servo_out_t out3 = step(&s3, &in3, 40000);
    check("catchup: held off at a stream start", !out3.catchup_write, NULL);

    df_servo_t s4 = {0};
    df_servo_in_t in4 = base();
    in4.med_us = 900000;
    in4.have_med = true;
    df_servo_out_t out4 = step(&s4, &in4, 900000);
    const int32_t cap = (int32_t)((int64_t)CATCHUP_MAX_US * RATE / 1000000);
    check("catchup: capped at CATCHUP_MAX_US",
          out4.catchup_write && out4.catchup_frames_new == cap, NULL);

    df_servo_t s5 = {0};
    df_servo_in_t in5 = base();
    in5.med_us = 30000;
    in5.have_med = true;
    in5.catchup_now = 1764;
    df_servo_out_t out5 = step(&s5, &in5, 30000);
    check("catchup: a shallower ask on the same side is ignored",
          !out5.catchup_write, NULL);

    df_servo_t s6 = {0};
    df_servo_in_t in6 = base();
    in6.med_us = 60000;
    in6.have_med = true;
    in6.catchup_now = 1000;
    df_servo_out_t out6 = step(&s6, &in6, 60000);
    check("catchup: a deeper ask on the same side replaces",
          out6.catchup_write && out6.catchup_frames_new == 2646, NULL);

    df_servo_t s7 = {0};
    df_servo_in_t in7 = base();
    in7.med_us = -40000;
    in7.have_med = true;
    in7.catchup_now = 1764;
    df_servo_out_t out7 = step(&s7, &in7, -40000);
    check("catchup: a sign flip replaces the debt",
          out7.catchup_write && out7.catchup_frames_new == -1764, NULL);

    df_servo_t s8 = {0};
    df_servo_in_t in8 = base();
    in8.med_us = 5000;
    in8.have_med = false;
    in8.catchup_now = 1764;
    df_servo_out_t out8 = step(&s8, &in8, 5000);
    check("catchup: stands down under CLEAR, median or not",
          out8.catchup_write && out8.catchup_frames_new == 0, NULL);

    df_servo_t s9 = {0};
    df_servo_in_t in9 = base();
    in9.med_us = 15000;
    in9.have_med = true;
    in9.catchup_now = 1764;
    df_servo_out_t out9 = step(&s9, &in9, 15000);
    check("catchup: the ARM/CLEAR gap is hysteresis", !out9.catchup_write, NULL);

    df_servo_t s10 = {0};
    df_servo_in_t in10 = base();
    in10.phase_valid = false;
    in10.med_us = 40000;
    in10.have_med = true;
    df_servo_out_t out10 = step(&s10, &in10, 40000);
    check("catchup: silent without a phase reading", !out10.catchup_write, NULL);
}

/** @brief When the servo acts at all: the deadband, the cooldown that follows a correction, and the boundary at RATE_TRIM_MAX_HZ where a fine trim becomes a coarse retune. */
static void test_act(void)
{

    df_servo_t s = {0};
    df_servo_in_t in = base();
    df_servo_out_t out = step(&s, &in, 5000);
    check("act: inside the deadband nothing moves", !out.act, NULL);

    df_servo_t s2 = {0};
    df_servo_in_t in2 = base();
    df_servo_out_t out2 = step(&s2, &in2, 20000);
    check("act: past the deadband the fine trim moves",
          out2.act && !out2.coarse && out2.trim_hz == 8, NULL);

    df_servo_t s3 = {0};
    df_servo_in_t in3 = base();
    df_servo_out_t first = step(&s3, &in3, 20000);
    check("act: the first window acts", first.act, NULL);
    int acted = 0;
    for (int i = 0; i < 4; i++) {
        df_servo_in_t nx = base();
        nx.trim_hz_now = 8;
        if (step(&s3, &nx, 200000).act) acted++;
    }
    check("act: four windows of cooldown follow", acted == 0, NULL);
    df_servo_in_t after = base();
    after.trim_hz_now = 8;
    check("act: and the fifth is free to move",
          step(&s3, &after, 200000).act, NULL);

    df_servo_t s4 = {0};
    df_servo_in_t in4 = base();
    in4.tx_rate = 44100;
    df_servo_out_t out4 = step(&s4, &in4, 2000000);
    check("act: at the ceiling the trim is still FINE",
          out4.act && !out4.coarse && out4.trim_hz == RATE_TRIM_MAX_HZ, NULL);

    df_servo_t s5 = {0};
    df_servo_in_t in5 = base();
    in5.tx_rate = 44100 - RATE_TRIM_MAX_HZ - 1;
    df_servo_out_t out5 = step(&s5, &in5, 2000000);
    check("act: one Hz past the ceiling goes COARSE",
          out5.act && out5.coarse, NULL);

    df_servo_t s6 = {0};
    df_servo_in_t in6 = base();
    in6.phase_valid = false;
    in6.depth_ms = 249;
    df_servo_out_t out6 = step(&s6, &in6, 200000);
    check("act: never without a phase reading", !out6.act, NULL);

    df_servo_t s7 = {0};
    df_servo_in_t in7 = base();
    in7.phase_valid = false;
    in7.depth_ms = 249;
    df_servo_out_t out7 = step(&s7, &in7, 0);
    check("act: the depth net still reports without phase",
          out7.depth_net_fired && out7.adj == 20, NULL);
    check("act: still does not act on it", !out7.act, NULL);
}

/** @brief The deadband is measured against the trim ALREADY APPLIED, so restating a standing trim is not a correction. */
static void test_deadband_is_against_applied_trim(void)
{
    df_servo_t s = {0};
    df_servo_in_t in = base();
    in.trim_hz_now = 8;
    df_servo_out_t out = step(&s, &in, 20000);
    check("deadband: a restatement of the standing trim does not act",
          !out.act, NULL);
}

/**
 * @brief Run every case and report.
 * @return 0 if all held, 1 otherwise, so `make check` fails the build.
 */
int main(void)
{
    test_ema();
    test_depth_net();
    test_clamp();
    test_catchup();
    test_act();
    test_deadband_is_against_applied_trim();

    printf("\n%s\n", failures ? "FAILURES" : "all passed");
    return failures ? 1 : 0;
}
