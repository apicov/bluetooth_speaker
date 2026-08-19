/*
 * The output-rate servo, as one loop both units run.
 *
 * WHY THIS IS SHARED. The hub and the satellite each hold their position in the
 * timeline by trimming the rate they consume audio at, and until 2026-08-19 they
 * did it from two copies of the same ~120 lines. Both copies said, in their own
 * comments, that they must never disagree:
 *
 *   "the two servos are the same loop, and a correction rate that differs
 *    between them is a cross-unit sync error by construction"
 *
 * and audio_shift.h says the same thing about the mechanism this arms:
 *
 *   "unequal correction between the units is a cross-unit sync error by
 *    construction (see rate_trim_hz), and shared code cannot disagree with
 *    itself"
 *
 * That invariant was enforced only by whoever edited one file remembering to
 * edit the other. It held -- the 2026-08-15 "floor, not a replacement" fix was
 * landed in both copies in a single commit precisely because it had to be -- but
 * remembering is not a mechanism. This is.
 *
 * WHAT IS HERE AND WHAT IS NOT. This file is arithmetic and nothing else: no
 * FreeRTOS, no i2s, no gpio, no esp_timer, no logging. It decides; the caller
 * measures, actuates and reports. That is what lets it be driven from the host
 * test (components/dancefloor_sync/test/test_servo.c) with plain gcc, which is
 * how a change to the loop is checked before any board is flashed -- and the
 * reason the split is worth the indirection it costs.
 *
 * Everything genuinely per-unit stays with the unit: the gate that decides
 * whether a stream is running at all, how the ring depth is measured, which
 * actuator a coarse correction reaches (retune_output on the satellite,
 * retune_dac on the hub), and every log line.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * THE TWO CALLERS DO NOT FEED THIS THE SAME INPUT, and that is deliberate.
 *
 * The satellite EMAs the RAW phase reading; the hub EMAs the MEDIAN the play
 * task publishes, falling back to raw while the median is invalid. See the
 * hub's "THE EMA WAS THE RIGHT FILTER ON THE WRONG INPUT" note for how it got
 * there -- 15.7 ms of swing between consecutive raw samples, and 10 retunes in
 * 365 s against a real drift of ~14 ppm.
 *
 * Unifying the CODE does not unify that choice and must not: which of the two
 * is right for the satellite is a question for the soak logs and its own
 * hardware run, not a side effect of moving lines into a component. So `err_in`
 * arrives already chosen, and this file has no opinion about it.
 *
 * The same applies to `depth_net_held`: the satellite holds the depth net for
 * DEPTH_NET_HOLD_US after an anchor and the hub has no such hold, because the
 * hub's ring is fed over a stream buffer rather than the radio and never showed
 * the fault the hold exists for.
 */
typedef struct {
    /*
     * Whether there is a phase measurement at all. Gates the catch-up arm and
     * the correction itself; the depth net still runs without it, because the
     * ring depth is measured rather than inferred and is evidence on its own.
     */
    bool     phase_valid;

    /*
     * The catch-up arm's input: a median if one exists, otherwise the caller's
     * raw fallback. `have_med` gates the ARM only -- the stand-down below
     * CATCHUP_CLEAR_US keeps working on the fallback, because standing a stale
     * debt down early is safe in a way arming one is not.
     */
    int32_t  med_us;
    bool     have_med;

    /* Within CATCHUP_HOLD_US of a stream (re)start: a fresh timeline's first
     * minute is past CATCHUP_ARM_US with nothing wrong. Arm only. */
    bool     catchup_held;

    /* Ring depth against target, and whether the depth net is being held off
     * (the satellite's DEPTH_NET_HOLD_US after an anchor; the hub passes
     * false). */
    int32_t  depth_ms;
    bool     depth_net_held;

    /* The rate the servo works in -- stream_rate on the satellite, rate_ema on
     * the hub -- and the rate the CLOCK is actually running at. */
    uint32_t rate;
    int32_t  tx_rate;

    /* The fine trim and the catch-up debt as they stand, so this can decide
     * against what is already applied rather than against nothing. */
    int32_t  trim_hz_now;
    int32_t  catchup_now;
} df_servo_in_t;

typedef struct {
    /* The smoothed error the decision was made on, for the caller's log line. */
    int32_t  err_ema;

    /* What phase alone asked for, and what survived the depth net. Equal unless
     * the net fired, which is what `depth_net_fired` says and what the caller
     * reports -- rare by construction, and the branch that changed on
     * 2026-08-15. */
    int32_t  adj_phase;
    int32_t  adj;
    bool     depth_net_fired;

    /* Whether the caller should assign catchup_frames, and to what. Split this
     * way because the debt is spent by playback: this decides, the caller
     * writes, and nothing here reads back what playback has drained. */
    bool     catchup_write;
    int32_t  catchup_frames_new;

    /* Whether to act at all -- deadband and cooldown both cleared -- and if so
     * whether the clock has to move (COARSE) or the fine trim can absorb it. */
    bool     act;
    bool     coarse;
    int32_t  trim_hz;
    uint32_t desired_rate;
} df_servo_out_t;

/*
 * The state the loop carries between its 5 s windows. One per unit, owned by
 * the caller and zero-initialised -- which is the correct starting state: no
 * average yet, and no cooldown outstanding.
 */
typedef struct {
    int32_t err_ema;
    bool    err_ema_valid;
    int     cooldown;
} df_servo_t;

/*
 * Fold one reading into the smoothed error and return it.
 *
 * Separate from df_servo_step() because the two callers return early at
 * different points: the hub gives up as soon as the phase is invalid, while the
 * satellite carries on far enough to run the depth net. Both update the average
 * first and unconditionally, so that stays here where they can share it, and
 * each unit's early-out stays in the unit where it is readable.
 *
 * A 4-sample EMA: 5 s per window, so ~20 s of memory.
 */
int32_t df_servo_ema(df_servo_t *s, int32_t err_in, bool reset_history);

/*
 * Decide what the servo should do this window. Pure: touches nothing but `s`'s
 * cooldown and `out`.
 *
 * Call after df_servo_ema(); `s->err_ema` is what this acts on. The caller then
 * actuates whatever `out` asks for and logs it.
 */
void df_servo_step(df_servo_t *s, const df_servo_in_t *in, df_servo_out_t *out);
