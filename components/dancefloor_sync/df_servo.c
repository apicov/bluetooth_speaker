/*
 * The output-rate servo, as one loop both units run. See df_servo.h for why
 * this is shared and what deliberately stayed with each unit.
 *
 * The comments below are the originals from satellite/main/servo.c and
 * hub_s3/main/servo.c, moved here with the code they describe rather than
 * summarised. Almost every constant and every branch in this file records the
 * run that produced it, and that reasoning is the most valuable thing in it.
 */
#include "df_servo.h"

#include "audio_shift.h"
#include "sync_proto.h"

/*
 * Smooth before acting, and separate two things that look identical in a
 * single reading:
 *
 *   jitter -- delivery is bursty, so the level swings +-40 ms with no
 *             trend. Correcting for it would retune constantly, and each
 *             retune is an audible click.
 *   drift  -- the crystals genuinely differ (~14 ppm measured), so the
 *             level walks steadily in one direction.
 *
 * Averaging kills the first and leaves the second. The old code coped by
 * using a wide deadband instead, which meant real drift could reach ~60 ms
 * before anything happened -- clearly audible echo, and about 75 minutes away
 * at 14 ppm. Fine in a short test, wrong over an evening.
 *
 * The history is dropped rather than averaged across a splice or a step,
 * because the readings before one describe a situation that no longer exists.
 */
int32_t df_servo_ema(df_servo_t *s, int32_t err_in, bool reset_history)
{
    if (reset_history) {
        s->err_ema_valid = false;      /* history describes a different world */
    }
    s->err_ema = s->err_ema_valid ? (s->err_ema * 3 + err_in) / 4 : err_in;
    s->err_ema_valid = true;
    return s->err_ema;
}

void df_servo_step(df_servo_t *s, const df_servo_in_t *in, df_servo_out_t *out)
{
    out->err_ema            = s->err_ema;
    out->catchup_write      = false;
    out->catchup_frames_new = in->catchup_now;
    out->act                = false;
    out->coarse             = false;
    out->trim_hz            = in->trim_hz_now;

    /*
     * Late (positive error) means we are behind the timeline, so play faster.
     *
     * Spread over ~100 s, not 40. The buffer takes tens of seconds to respond,
     * so a 40 s loop was still correcting after the error had gone and sailed
     * past it -- both units converged to near zero then overshot to +10 ms and
     * oscillated. Real drift is only ~0.8 ms per minute, so the loop can afford
     * to be much gentler than the disturbance it corrects.
     */
    int32_t adj = (int32_t)((int64_t)s->err_ema * in->rate / 100000000LL);
    /*
     * Belt and braces against the arithmetic above, which has produced -138000
     * once already. The actuators refuse anything wilder, but the number should
     * not get that far in the first place. The drift correction is small by
     * nature -- real drift is ~14 ppm -- so anything larger is a bad phase
     * reading, not a rate error.
     */
    if (adj >  RATE_TRIM_MAX_HZ) adj =  RATE_TRIM_MAX_HZ;
    if (adj < -RATE_TRIM_MAX_HZ) adj = -RATE_TRIM_MAX_HZ;

    /*
     * Safety net: if the buffer is heading for empty or full, that matters more
     * than phase.
     *
     * A FLOOR, NOT A REPLACEMENT, since 2026-08-15.
     *
     * This used to assign adj outright, which meant it could only ever WEAKEN a
     * phase correction that already agreed with it. Measured on the satellite's
     * soak in tools/soak/logs-soak-20260815-224002, in that unit's own words:
     *
     *   buffer 449 ms | phase +268516 us
     *   servo: smoothed +83432 us -> trim +20 Hz (20 frames/s)
     *
     * The phase term asked for 83432 * 44100 / 1e8 = 36 Hz and got 20, because
     * the ring was 249 ms past target and this branch overwrote it. So at the
     * exact moment the ring was deepest, the guard that exists to protect the
     * ring cut the recovery to 0.45 ms/s -- and a ring that deep IS playing that
     * late, so the two were asking for the same thing and the guard won anyway.
     * Recovery then took minutes and a track boundary ended it first.
     *
     * Depth still WINS when the two disagree, which is the case this was written
     * for: a ring heading for empty or full while phase reads fine. When they
     * agree, the larger correction stands.
     *
     * The hub's own ring is fed over a stream buffer rather than the radio, so
     * it never showed the fault; the fix landed on both units in one commit all
     * the same, because a correction rate that differs between them is a
     * cross-unit sync error by construction.
     *
     * No steady-state effect. Depth only leaves +-120 ms during exactly the
     * events this is about; every log window that ever read `buffer 165-250 ms`
     * never reached this branch and still does not.
     *
     * `depth_net_held` is the satellite's DEPTH_NET_HOLD_US after an anchor,
     * during which the depth is not evidence of anything this exists to catch --
     * a fresh stream starts below target by construction, and the net once fired
     * on that, asking for -20 Hz to rescue a ring that was not in trouble. The
     * hub has no such hold and passes false.
     */
    const int32_t adj_phase = adj;       /* what phase alone asked for */
    if (in->depth_net_held) {
        /* say nothing; the caller's status line already prints the depth */
    } else if (in->depth_ms < -120) {
        if (adj > -20) adj = -20;        /* nearly empty: slow down, at least */
    } else if (in->depth_ms > 120) {
        if (adj < 20) adj = 20;          /* nearly full: speed up, at least */
    }
    out->adj_phase       = adj_phase;
    out->adj             = adj;
    /*
     * Say so when the net is in play, because this is the branch that changed
     * and a log window has no other way to show it. Under the old code the two
     * numbers were the whole story: `phase 36 -> net 20` was the cap, and is
     * what the 2026-08-15 commit exists to stop. Rare by construction -- depth
     * only leaves +-120 ms during a delivery burst.
     */
    out->depth_net_fired = (adj != adj_phase);

    /*
     * THE LARGE-ERROR CATCH-UP, armed here, spent by playback.
     *
     * The trim above is capped at RATE_TRIM_MAX_HZ = 2.27 ms/s. The soak of
     * 2026-08-17 recorded both satellites knocked +40 to +150 ms in one stroke
     * by a hub tx-fail burst, which is a minute or more at that rate -- a minute
     * or more of audible echo, usually ended by the audible boundary splice
     * rather than by the servo. So beyond CATCHUP_ARM_US the servo asks for the
     * splice's own move -- skip or replay material -- and playback spends it a
     * few frames per chunk under a crossfade (audio_shift.c): 31 ms/s,
     * inaudible by construction, both units from the same code.
     *
     * The situation is the hub's to cause: a tx-fail burst on its radio starves
     * the satellites' audio AND its own ring at once, so both sides carry the
     * same knock.
     *
     * Armed from a MEDIAN, not from the EMA. The EMA's settling (~4 windows =
     * 20 s) outlives the whole drain, so arming from it would re-arm error the
     * drain had already paid and overshoot to the other side; the median is
     * current to a few readings and ignores the scatter a single reading
     * carries.
     *
     * Since 2026-08-18 the arm also requires that median to EXIST. The raw
     * fallback used to arm whenever the history was too short to offer a median
     * -- the ~100 ms after a splice or a start -- but a splice zeroes the debt
     * and resets the history precisely because the readings before it described
     * the error just paid, so arming on the raw survivor is arming on the paid
     * error itself. The stand-down below keeps the raw fallback and its timing,
     * because standing a stale debt down early is safe in a way arming one is
     * not.
     *
     * The debt only grows here -- never shrinks -- and only playback shrinks it,
     * as it spends. That split stops this loop from bidding against the drain,
     * and makes the value mean one thing at all times: frames still to skip or
     * replay. The one exception is a sign flip: if the drain has overshot, the
     * old debt is pointing the wrong way and is replaced.
     *
     * The caller's writes race the play task's decrements at 5 s against ~6 ms
     * cadence. Benign in both directions, by the same reasoning as rate_trim_hz:
     * the next window re-arms from a fresh median either way.
     */
    if (in->phase_valid) {
        const int32_t med = in->med_us;
        if (!in->catchup_held && in->have_med &&
            (med >= CATCHUP_ARM_US || med <= -CATCHUP_ARM_US)) {
            const int32_t cap = (int32_t)((int64_t)CATCHUP_MAX_US * in->rate
                                          / 1000000);
            int32_t want = (int32_t)((int64_t)med * in->rate / 1000000);
            if (want >  cap) want =  cap;
            if (want < -cap) want = -cap;
            const int32_t now = in->catchup_now;
            if ((want > 0) != (now > 0)) {
                out->catchup_write      = true;
                out->catchup_frames_new = want;     /* overshot: reversed */
            } else if (want > 0 ? want > now : want < now) {
                out->catchup_write      = true;
                out->catchup_frames_new = want;     /* same side, deeper */
            }
        } else if (med < CATCHUP_CLEAR_US && med > -CATCHUP_CLEAR_US) {
            out->catchup_write      = true;
            out->catchup_frames_new = 0;  /* remainder is the fine trim's to finish */
        }
    }

    const uint32_t desired = (uint32_t)((int32_t)in->rate + adj);
    out->desired_rate = desired;

    if (!in->phase_valid) {
        return;                        /* nothing measured yet */
    }

    /*
     * Deadband, stated in phase error rather than in rate.
     *
     * It used to be tx_rate/5000, described in a comment as "~8 ms of
     * accumulated drift before a correction". That read 0.02% of the sample
     * rate as if it were milliseconds: 8 Hz of threshold is really 8e8/44100 =
     * ~20 ms of phase, per unit and in either direction.
     *
     * Affordable at 7 ms because a retune is cheap now -- measured at 1 to 7 ms,
     * essentially just the channel outage. It was not while the satellite's
     * drift task spun through the ring during that outage; see
     * PHASE_DEADBAND_US.
     */
    int32_t deadband = (int32_t)((int64_t)PHASE_DEADBAND_US * in->rate
                                 / 100000000LL);
    if (deadband < 1) {
        deadband = 1;
    }

    /*
     * The correction, as an offset from the rate the CLOCK is actually running
     * at. This is the number that used to be handed to the actuator as an
     * absolute rate; splitting it out is what lets the two actuators be chosen
     * between, because their boundary is a size and not a kind.
     */
    const int32_t trim_hz = (int32_t)desired - in->tx_rate;
    out->trim_hz = trim_hz;

    /*
     * Windows to wait after a correction before considering another. The buffer
     * takes tens of seconds to respond, and acting again before it has is how a
     * servo ends up chasing its own corrections. The hub had no cooldown at all
     * to begin with, so it retuned every window and chased its own previous
     * correction.
     */
    if (s->cooldown > 0) {
        s->cooldown--;
        return;
    }

    /*
     * Deadband against the trim ALREADY APPLIED, which is where tx_rate used to
     * be read: the servo tolerates PHASE_DEADBAND_US of its own error before
     * moving, and that meaning is unchanged. Only what moves has changed.
     */
    const int32_t step = trim_hz - in->trim_hz_now;
    if (step > deadband || step < -deadband) {
        /*
         * COARSE: too big for software to absorb without shredding the audio,
         * so the clock has to move. Beyond the trim ceiling only the clock can
         * help -- a source measured at ~42600 against a 44100 output is 40000
         * ppm and drains a 250 ms buffer in five seconds. In steady state it
         * never happens; on the satellite it is also how a stream is first
         * matched at all, because i2s_start() runs at a hardcoded 44100 before
         * any stream exists.
         *
         * FINE: playback drops or duplicates one frame at a time. No
         * channel-down, and continuous rather than stepped.
         */
        out->act    = true;
        out->coarse = (trim_hz > RATE_TRIM_MAX_HZ || trim_hz < -RATE_TRIM_MAX_HZ);
        s->cooldown = 4;               /* ~20 s against a ~100 s correction */
    }
}
