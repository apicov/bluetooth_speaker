/**
 * @file df_servo.h
 * @brief The output-rate servo, as one loop both units run.
 *
 * The hub and the satellite each hold their position in the timeline by
 * trimming the rate they consume audio at. They must not do it differently: a
 * correction rate that differs between the units is a cross-unit sync error by
 * construction, and the same is true of the catch-up this arms (see
 * audio_shift.h). Two copies of the arithmetic enforce that only by whoever
 * edits one remembering to edit the other; one copy enforces it structurally.
 *
 * WHAT IS HERE AND WHAT IS NOT. This file is arithmetic and nothing else: no
 * FreeRTOS, no i2s, no gpio, no timer, no logging. It decides; the caller
 * measures, actuates and reports. That is what lets it be driven from
 * test/test_servo.c under plain gcc, which is how a change to the loop is
 * checked before any board is flashed -- and the reason the split is worth the
 * indirection it costs.
 *
 * Everything genuinely per-unit stays with the unit: the gate that decides
 * whether a stream is running at all, how the ring depth is measured, which
 * actuator a coarse correction reaches (retune_output on the satellite,
 * retune_dac on the hub), and every log line.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief One window's measurements, as the caller has already chosen them.
 *
 * THE TWO CALLERS DO NOT FEED THIS THE SAME INPUT, and that is deliberate. The
 * satellite smooths the RAW phase reading; the hub smooths the MEDIAN its play
 * task publishes, falling back to raw while no median is valid, because the
 * hub's raw reading carries far more scatter at its load. Unifying the CODE
 * does not unify that choice and must not -- which one is right for the
 * satellite is a question for its own hardware runs, not a side effect of
 * moving lines into a component. So the reading arrives already chosen and
 * this file has no opinion about it. The same applies to #depth_net_held.
 */
typedef struct {
    /** @brief Whether there is a phase measurement at all. Gates the catch-up
     *         arm and the correction itself; the depth net still runs without
     *         it, because ring depth is measured rather than inferred and is
     *         evidence on its own. */
    bool     phase_valid;

    /** @brief The catch-up arm's input: a median if one exists, otherwise the
     *         caller's raw fallback. */
    int32_t  med_us;
    /** @brief Whether #med_us is a real median. Gates the ARM only -- the
     *         stand-down below CATCHUP_CLEAR_US keeps working on the fallback,
     *         because standing a stale debt down early is safe in a way arming
     *         one is not. */
    bool     have_med;

    /** @brief Within CATCHUP_HOLD_US of a stream (re)start, so the arm is held
     *         off. Arm only; see CATCHUP_HOLD_US. */
    bool     catchup_held;

    /** @brief Ring depth against target, in ms. */
    int32_t  depth_ms;
    /** @brief Whether the depth net is being held off -- the satellite's
     *         post-anchor hold, during which a fresh stream is below target by
     *         construction. The hub's ring is fed over a stream buffer rather
     *         than the radio and never showed that fault, so it passes false. */
    bool     depth_net_held;

    /** @brief The rate the servo works in: stream_rate on the satellite,
     *         rate_ema on the hub. */
    uint32_t rate;
    /** @brief The rate the CLOCK is actually running at, which the trim is
     *         expressed as an offset from. */
    int32_t  tx_rate;

    /** @brief The fine trim as it stands, so the deadband is measured against
     *         what is already applied rather than against nothing. */
    int32_t  trim_hz_now;
    /** @brief The catch-up debt as it stands, for the same reason. */
    int32_t  catchup_now;
} df_servo_in_t;

/** @brief What the caller should do about them. */
typedef struct {
    /** @brief The smoothed error the decision was made on, for the caller's
     *         log line. */
    int32_t  err_ema;

    /** @brief What phase alone asked for. */
    int32_t  adj_phase;
    /** @brief ...and what survived the depth net. Equal unless the net fired. */
    int32_t  adj;
    /** @brief Whether the net changed the answer -- rare by construction, and
     *         worth reporting because a log window has no other way to see it. */
    bool     depth_net_fired;

    /** @brief Whether the caller should assign catchup_frames. Split from the
     *         value because the debt is SPENT by playback: this decides, the
     *         caller writes, and nothing here reads back what has drained. */
    bool     catchup_write;
    /** @brief ...and what to assign, when it should. */
    int32_t  catchup_frames_new;

    /** @brief Whether to act at all -- deadband and cooldown both cleared. */
    bool     act;
    /** @brief If acting, whether the CLOCK has to move rather than the fine
     *         trim absorbing it. See RATE_TRIM_MAX_HZ for the boundary. */
    bool     coarse;
    /** @brief The fine correction to apply, in Hz against tx_rate. */
    int32_t  trim_hz;
    /** @brief The absolute rate a coarse correction should retune to. */
    uint32_t desired_rate;
} df_servo_out_t;

/**
 * @brief The state the loop carries between its windows.
 *
 * One per unit, owned by the caller and zero-initialised -- which is the
 * correct starting state: no average yet, and no cooldown outstanding.
 */
typedef struct {
    int32_t err_ema;        /**< The smoothed error. */
    bool    err_ema_valid;  /**< False until the first reading, and after a reset. */
    int     cooldown;       /**< Windows still to wait before acting again. */
} df_servo_t;

/**
 * @brief Fold one reading into the smoothed error and return it.
 *
 * Separate from df_servo_step() because the two callers return early at
 * different points: the hub gives up as soon as the phase is invalid, while
 * the satellite carries on far enough to run the depth net. Both update the
 * average first and unconditionally, so that stays here where they can share
 * it, and each unit's early-out stays where it is readable.
 *
 * @param s              The loop's carried state.
 * @param err_in         This window's phase error, us; + = playing late.
 * @param reset_history  Drop what was accumulated: the readings before a
 *                       splice or a step describe a situation that no longer
 *                       exists.
 * @return The smoothed error, which is what df_servo_step() then acts on.
 */
int32_t df_servo_ema(df_servo_t *s, int32_t err_in, bool reset_history);

/**
 * @brief Decide what the servo should do this window.
 *
 * Pure: touches nothing but @p s's cooldown and @p out. Call after
 * df_servo_ema(), whose result is what this acts on. The caller then actuates
 * whatever @p out asks for and logs it.
 *
 * @param s        The loop's carried state; only the cooldown is written.
 * @param in       This window's measurements.
 * @param[out] out What to do about them.
 */
void df_servo_step(df_servo_t *s, const df_servo_in_t *in, df_servo_out_t *out);
