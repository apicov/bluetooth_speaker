/*
 * Host-side tests for the shared output-rate servo.
 *   cc -std=c11 -Wall -Wextra -I../include test_servo.c ../df_servo.c -o t && ./t
 *
 * WHY THESE EXIST. Until 2026-08-19 this loop lived twice, once in each unit,
 * and the only way to check a change to it was to flash two boards and read a
 * log window. That is why three of the branches below carry a date and a
 * measurement: each one was a bug that a soak found after the fact. With the
 * loop in a component that needs no ESP-IDF, they can be pinned here instead.
 *
 * The claims under test are the ones df_servo.h and audio_shift.h make. In
 * particular the 2026-08-15 case is checked by its own numbers: the depth net
 * is a FLOOR and must no longer cut a phase correction that already agrees
 * with it.
 */
#include "df_servo.h"

#include "audio_shift.h"
#include "sync_proto.h"

#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>

static int failures = 0;

static void check(const char *name, bool cond, const char *detail)
{
    printf("%-52s %s%s%s\n", name, cond ? "PASS" : "FAIL",
           detail ? "  " : "", detail ? detail : "");
    if (!cond) failures++;
}

#define RATE 44100

/* A window with nothing going on: valid phase, ring on target, no debt, no
 * median. Tests below change only the field they are about. */
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

/* Run one window: fold the reading in, then decide. The order the callers use. */
static df_servo_out_t step(df_servo_t *s, df_servo_in_t *in, int32_t err)
{
    df_servo_out_t out;
    df_servo_ema(s, err, false);
    df_servo_step(s, in, &out);
    return out;
}

/* ------------------------------------------------------------------ the EMA */

static void test_ema(void)
{
    df_servo_t s = {0};

    /* First sample is adopted outright rather than averaged toward from zero,
     * which would otherwise take four windows to reach a standing error. */
    check("ema: first sample adopted", df_servo_ema(&s, 8000, false) == 8000, NULL);

    /* Then a quarter of the way each window: (3*8000 + 0)/4 = 6000. */
    check("ema: quarter weight on the new sample",
          df_servo_ema(&s, 0, false) == 6000, NULL);

    /* A splice or a step drops the history: the average restarts at the new
     * sample instead of averaging across the discontinuity. */
    check("ema: reset adopts rather than averages",
          df_servo_ema(&s, -20000, true) == -20000, NULL);

    /* Settling: a standing error is reached, not overshot. */
    df_servo_t t = {0};
    for (int i = 0; i < 40; i++) df_servo_ema(&t, 10000, false);
    check("ema: settles on a standing error", t.err_ema == 10000, NULL);
}

/* ------------------------------------------------------------- the depth net */

static void test_depth_net(void)
{
    /*
     * THE 2026-08-15 CASE, in the satellite's own numbers.
     *
     *   buffer 449 ms | phase +268516 us
     *   servo: smoothed +83432 us -> trim +20 Hz (20 frames/s)
     *
     * 83432 * 44100 / 1e8 = 36 Hz. The ring was 249 ms past target, so the old
     * code overwrote 36 with the net's 20 at the exact moment the ring was
     * deepest -- and a ring that deep IS playing that late, so both were asking
     * for the same thing. As a floor the larger correction must now stand.
     */
    df_servo_t s = {0};
    df_servo_in_t in = base();
    in.depth_ms = 249;
    df_servo_out_t out = step(&s, &in, 83432);
    check("depth net: floor does not cut an agreeing phase",
          out.adj == 36 && out.adj_phase == 36, NULL);
    check("depth net: silent when it did not fire", !out.depth_net_fired, NULL);

    /* It still WINS when the two disagree, which is the case it was written
     * for: a ring heading for full while phase reads fine. */
    df_servo_t s2 = {0};
    df_servo_in_t in2 = base();
    in2.depth_ms = 249;
    df_servo_out_t out2 = step(&s2, &in2, 0);
    check("depth net: lifts a phase that disagrees",
          out2.adj == 20 && out2.adj_phase == 0 && out2.depth_net_fired, NULL);

    /* ...and in the other direction. */
    df_servo_t s3 = {0};
    df_servo_in_t in3 = base();
    in3.depth_ms = -249;
    df_servo_out_t out3 = step(&s3, &in3, 0);
    check("depth net: floors a nearly-empty ring at -20 Hz",
          out3.adj == -20 && out3.depth_net_fired, NULL);

    /* No steady-state effect: +-120 ms is the band, and inside it the net is
     * never reached. Every `buffer 165-250 ms` window ever logged sits here. */
    df_servo_t s4 = {0};
    df_servo_in_t in4 = base();
    in4.depth_ms = 119;
    df_servo_out_t out4 = step(&s4, &in4, 0);
    check("depth net: inside +-120 ms it does nothing",
          out4.adj == 0 && !out4.depth_net_fired, NULL);

    /* The satellite holds it off after an anchor, where a fresh stream is below
     * target by construction and the net once asked for -20 Hz to rescue a ring
     * that was not in trouble. */
    df_servo_t s5 = {0};
    df_servo_in_t in5 = base();
    in5.depth_ms = -249;
    in5.depth_net_held = true;
    df_servo_out_t out5 = step(&s5, &in5, 0);
    check("depth net: held off after an anchor",
          out5.adj == 0 && !out5.depth_net_fired, NULL);
}

/* ---------------------------------------------------------------- the clamp */

static void test_clamp(void)
{
    /* The arithmetic has produced -138000 once already. Whatever the reading,
     * the ask never leaves +-RATE_TRIM_MAX_HZ. */
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

/* ------------------------------------------------------------- the catch-up */

static void test_catchup(void)
{
    /* Armed beyond CATCHUP_ARM_US, from a median, in frames at the stream rate:
     * 40000 us * 44100 / 1e6 = 1764 frames. */
    df_servo_t s = {0};
    df_servo_in_t in = base();
    in.med_us = 40000;
    in.have_med = true;
    df_servo_out_t out = step(&s, &in, 40000);
    check("catchup: arms from a median past the threshold",
          out.catchup_write && out.catchup_frames_new == 1764, NULL);

    /* Never armed without one. A splice zeroes the debt and resets the history
     * precisely because the readings before it described the error just paid,
     * so arming on the raw survivor is arming on the paid error itself. */
    df_servo_t s2 = {0};
    df_servo_in_t in2 = base();
    in2.med_us = 40000;
    in2.have_med = false;
    df_servo_out_t out2 = step(&s2, &in2, 40000);
    check("catchup: no median, no arm", !out2.catchup_write, NULL);

    /* Nor in the first CATCHUP_HOLD_US of a stream, where a fresh timeline's
     * transient is past the arm threshold with nothing wrong -- this armed 1448
     * frames of replay in the first window of the 2026-08-18 soak. */
    df_servo_t s3 = {0};
    df_servo_in_t in3 = base();
    in3.med_us = 40000;
    in3.have_med = true;
    in3.catchup_held = true;
    df_servo_out_t out3 = step(&s3, &in3, 40000);
    check("catchup: held off at a stream start", !out3.catchup_write, NULL);

    /* Capped at CATCHUP_MAX_US, so a huge displacement is corrected in faded
     * steps rather than one long shred. */
    df_servo_t s4 = {0};
    df_servo_in_t in4 = base();
    in4.med_us = 900000;
    in4.have_med = true;
    df_servo_out_t out4 = step(&s4, &in4, 900000);
    const int32_t cap = (int32_t)((int64_t)CATCHUP_MAX_US * RATE / 1000000);
    check("catchup: capped at CATCHUP_MAX_US",
          out4.catchup_write && out4.catchup_frames_new == cap, NULL);

    /* The debt only GROWS here -- a shallower ask on the same side is ignored,
     * so this loop never bids against the drain playback is running. */
    df_servo_t s5 = {0};
    df_servo_in_t in5 = base();
    in5.med_us = 30000;
    in5.have_med = true;
    in5.catchup_now = 1764;
    df_servo_out_t out5 = step(&s5, &in5, 30000);
    check("catchup: a shallower ask on the same side is ignored",
          !out5.catchup_write, NULL);

    /* ...but a deeper one on the same side replaces it. */
    df_servo_t s6 = {0};
    df_servo_in_t in6 = base();
    in6.med_us = 60000;
    in6.have_med = true;
    in6.catchup_now = 1000;
    df_servo_out_t out6 = step(&s6, &in6, 60000);
    check("catchup: a deeper ask on the same side replaces",
          out6.catchup_write && out6.catchup_frames_new == 2646, NULL);

    /* A sign flip replaces outright: the drain overshot and the old debt is
     * pointing the wrong way. */
    df_servo_t s7 = {0};
    df_servo_in_t in7 = base();
    in7.med_us = -40000;
    in7.have_med = true;
    in7.catchup_now = 1764;
    df_servo_out_t out7 = step(&s7, &in7, -40000);
    check("catchup: a sign flip replaces the debt",
          out7.catchup_write && out7.catchup_frames_new == -1764, NULL);

    /* Under CATCHUP_CLEAR_US it stands down, leaving the remainder to the fine
     * trim -- and this branch keeps working on the raw fallback, because
     * standing a stale debt down early is safe in a way arming one is not. */
    df_servo_t s8 = {0};
    df_servo_in_t in8 = base();
    in8.med_us = 5000;
    in8.have_med = false;
    in8.catchup_now = 1764;
    df_servo_out_t out8 = step(&s8, &in8, 5000);
    check("catchup: stands down under CLEAR, median or not",
          out8.catchup_write && out8.catchup_frames_new == 0, NULL);

    /* The gap between ARM and CLEAR is hysteresis: in it, neither happens. */
    df_servo_t s9 = {0};
    df_servo_in_t in9 = base();
    in9.med_us = 15000;
    in9.have_med = true;
    in9.catchup_now = 1764;
    df_servo_out_t out9 = step(&s9, &in9, 15000);
    check("catchup: the ARM/CLEAR gap is hysteresis", !out9.catchup_write, NULL);

    /* Nothing at all without a phase measurement. */
    df_servo_t s10 = {0};
    df_servo_in_t in10 = base();
    in10.phase_valid = false;
    in10.med_us = 40000;
    in10.have_med = true;
    df_servo_out_t out10 = step(&s10, &in10, 40000);
    check("catchup: silent without a phase reading", !out10.catchup_write, NULL);
}

/* ------------------------------------------ the deadband, cooldown and split */

static void test_act(void)
{
    /*
     * The deadband is PHASE_DEADBAND_US of phase, not of rate: at 44.1 kHz,
     * 7000 * 44100 / 1e8 = 3 Hz. A correction smaller than that is not made.
     */
    df_servo_t s = {0};
    df_servo_in_t in = base();
    df_servo_out_t out = step(&s, &in, 5000);   /* 5000*44100/1e8 = 2 Hz */
    check("act: inside the deadband nothing moves", !out.act, NULL);

    /* Past it, the fine trim takes it: playback drops or duplicates one frame
     * at a time, no channel-down. */
    df_servo_t s2 = {0};
    df_servo_in_t in2 = base();
    df_servo_out_t out2 = step(&s2, &in2, 20000);  /* 8 Hz */
    check("act: past the deadband the fine trim moves",
          out2.act && !out2.coarse && out2.trim_hz == 8, NULL);

    /*
     * Then a cooldown of four windows -- ~20 s against a ~100 s correction --
     * so the loop cannot chase its own last correction while the buffer is
     * still responding to it.
     */
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

    /*
     * The COARSE/FINE boundary is a size, not a kind. Beyond RATE_TRIM_MAX_HZ
     * software cannot absorb it without shredding the audio, so the clock has
     * to move -- the satellite's once-per-stream case, where i2s_start() ran at
     * a hardcoded 44100 and the source is measured at ~42600.
     */
    df_servo_t s4 = {0};
    df_servo_in_t in4 = base();
    in4.tx_rate = 44100;
    df_servo_out_t out4 = step(&s4, &in4, 2000000);   /* clamps to +100 Hz */
    check("act: at the ceiling the trim is still FINE",
          out4.act && !out4.coarse && out4.trim_hz == RATE_TRIM_MAX_HZ, NULL);

    /* One Hz past it, the clock has to carry the correction instead. Reached
     * here through tx_rate rather than through the ask, which is the real
     * case: the clock is parked somewhere the trim cannot reach from. */
    df_servo_t s5 = {0};
    df_servo_in_t in5 = base();
    in5.tx_rate = 44100 - RATE_TRIM_MAX_HZ - 1;
    df_servo_out_t out5 = step(&s5, &in5, 2000000);
    check("act: one Hz past the ceiling goes COARSE",
          out5.act && out5.coarse, NULL);

    /* Nothing acts without a phase reading, whatever the depth says. */
    df_servo_t s6 = {0};
    df_servo_in_t in6 = base();
    in6.phase_valid = false;
    in6.depth_ms = 249;
    df_servo_out_t out6 = step(&s6, &in6, 200000);
    check("act: never without a phase reading", !out6.act, NULL);

    /*
     * ...but the depth net still RUNS without one, which is the satellite's
     * ordering: the ring depth is measured rather than inferred, so it is
     * evidence on its own. (The hub returns before reaching here, so this
     * costs it nothing.)
     */
    /*
     * A small ask, so the floor is what decides: out6 above asks for 88 Hz,
     * which is already past the net's 20 and correctly left alone.
     */
    df_servo_t s7 = {0};
    df_servo_in_t in7 = base();
    in7.phase_valid = false;
    in7.depth_ms = 249;
    df_servo_out_t out7 = step(&s7, &in7, 0);
    check("act: the depth net still reports without phase",
          out7.depth_net_fired && out7.adj == 20, NULL);
    check("act: still does not act on it", !out7.act, NULL);
}

/* --------------------------------------------------------------- the deadband
 * is measured against what is ALREADY APPLIED, not against the clock. */

static void test_deadband_is_against_applied_trim(void)
{
    df_servo_t s = {0};
    df_servo_in_t in = base();
    in.trim_hz_now = 8;                 /* the servo already asked for 8 Hz */
    df_servo_out_t out = step(&s, &in, 20000);   /* and asks for 8 Hz again */
    check("deadband: a restatement of the standing trim does not act",
          !out.act, NULL);
}

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
