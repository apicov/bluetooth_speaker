/**
 * @file df_servo.c
 * @brief The output-rate servo's arithmetic. See df_servo.h for why this is
 *        shared between the two units and what deliberately stayed with each.
 *
 * Nothing here touches hardware or the clock. Every branch is reachable from
 * test/test_servo.c, which is where the boundaries below are pinned.
 */
#include "df_servo.h"

#include "audio_shift.h"
#include "sync_proto.h"

/*
 * Smooth before acting, and separate two things that look identical in a
 * single reading:
 *
 *   jitter -- delivery is bursty, so the level swings tens of milliseconds
 *             with no trend. Correcting for it would retune constantly.
 *   drift  -- the crystals genuinely differ, so the level walks steadily in
 *             one direction.
 *
 * Averaging kills the first and leaves the second. Coping with a wide deadband
 * instead means real drift can reach an audible echo before anything happens:
 * fine in a short test, wrong over an evening.
 *
 * A 4-sample average, so at the callers' window length it carries something
 * under half a minute of memory. Declared in df_servo.h.
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
     * Spread over roughly a hundred seconds. The buffer takes tens of seconds
     * to respond, so a loop tuned much tighter is still correcting after the
     * error has gone and sails past it -- both units converge to near zero,
     * overshoot, and oscillate. Real drift is a fraction of a millisecond per
     * minute, so the loop can afford to be far gentler than the disturbance it
     * corrects.
     */
    int32_t adj = (int32_t)((int64_t)s->err_ema * in->rate / 100000000LL);
    /*
     * Belt and braces against the arithmetic above, which has produced a wild
     * value at least once. The actuators refuse anything that large, but the
     * number should not get that far in the first place: the drift correction
     * is small by nature, so anything larger is a bad phase reading rather
     * than a rate error.
     */
    if (adj >  RATE_TRIM_MAX_HZ) adj =  RATE_TRIM_MAX_HZ;
    if (adj < -RATE_TRIM_MAX_HZ) adj = -RATE_TRIM_MAX_HZ;

    /*
     * Safety net: if the buffer is heading for empty or full, that matters
     * more than phase.
     *
     * A FLOOR, NOT A REPLACEMENT. Assigning `adj` outright here can only ever
     * WEAKEN a phase correction that already agrees with it -- and a ring far
     * past target IS a unit playing that late, so the two are usually asking
     * for the same thing. Overwriting then caps the recovery at the exact
     * moment the ring is deepest, which is the opposite of what a guard on the
     * ring should do.
     *
     * Depth still WINS when the two disagree, which is the case this was
     * written for: a ring heading for empty or full while phase reads fine.
     * When they agree, the larger correction stands.
     *
     * No steady-state effect. Depth only leaves this band during exactly the
     * delivery events it is about, and an ordinary window never reaches this
     * branch.
     *
     * depth_net_held is the satellite's post-anchor hold, during which the
     * depth is not evidence of anything this exists to catch -- a fresh stream
     * starts below target by construction, and the net once fired on that,
     * asking to rescue a ring that was not in trouble. The hub has no such
     * hold and passes false.
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

    /* Say so when the net is in play, because a log window has no other way to
     * show which of the two numbers the correction came from. Rare by
     * construction -- depth only leaves the band during a delivery burst. */
    out->depth_net_fired = (adj != adj_phase);

    /*
     * THE LARGE-ERROR CATCH-UP, armed here and spent by playback.
     *
     * The trim above is capped at RATE_TRIM_MAX_HZ, which for a knock of a
     * hundred milliseconds or more is a minute of audible echo, usually ended
     * by the audible boundary splice rather than by the servo. So beyond
     * CATCHUP_ARM_US the servo asks for the splice's own move -- skip or
     * replay material -- and playback spends it a few frames per chunk under a
     * crossfade (audio_shift.c), inaudibly, both units from the same code.
     *
     * The situation is the hub's to cause: a transmit-failure burst on its
     * radio starves the satellites' audio AND its own ring at once, so both
     * sides carry the same knock.
     *
     * Armed from a MEDIAN, not from the average. The average's settling time
     * outlives the whole drain, so arming from it would re-arm error the drain
     * had already paid and overshoot to the other side; the median is current
     * to a few readings and ignores the scatter a single reading carries.
     *
     * The arm also requires that median to EXIST. Falling back to the raw
     * reading arms whenever the history is too short to offer a median --
     * which is precisely the window just after a splice or a start, and a
     * splice zeroes the debt and resets the history exactly because the
     * readings before it described the error just paid. Arming on the raw
     * survivor is arming on the paid error itself. The stand-down below keeps
     * the raw fallback, because standing a stale debt down early is safe in a
     * way arming one is not.
     *
     * The debt only GROWS here and only playback shrinks it, as it spends.
     * That split stops this loop bidding against the drain and makes the value
     * mean one thing at all times: frames still to skip or replay. The one
     * exception is a sign flip -- if the drain has overshot, the old debt
     * points the wrong way and is replaced.
     *
     * The caller's writes race the play task's decrements at window cadence
     * against chunk cadence. Benign in both directions: the next window
     * re-arms from a fresh median either way.
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
     * Deadband, stated in phase error rather than in rate. Expressing it as a
     * fraction of the sample rate instead reads as milliseconds while meaning
     * something else entirely -- a few hertz of threshold is tens of
     * milliseconds of phase, per unit and in either direction.
     */
    int32_t deadband = (int32_t)((int64_t)PHASE_DEADBAND_US * in->rate
                                 / 100000000LL);
    if (deadband < 1) {
        deadband = 1;
    }

    /* The correction, as an offset from the rate the CLOCK is actually running
     * at. Splitting it out from an absolute rate is what lets the two
     * actuators be chosen between, because their boundary is a size and not a
     * kind. */
    const int32_t trim_hz = (int32_t)desired - in->tx_rate;
    out->trim_hz = trim_hz;

    /* Windows to wait after a correction before considering another. The
     * buffer takes tens of seconds to respond, and acting again before it has
     * is how a servo ends up chasing its own corrections. */
    if (s->cooldown > 0) {
        s->cooldown--;
        return;
    }

    /* Deadband against the trim ALREADY APPLIED: the servo tolerates
     * PHASE_DEADBAND_US of its own error before moving. */
    const int32_t step = trim_hz - in->trim_hz_now;
    if (step > deadband || step < -deadband) {
        /*
         * COARSE means too big for software to absorb without shredding the
         * audio, so the clock has to move; see RATE_TRIM_MAX_HZ. In steady
         * state it never happens, and on the satellite it is also how a stream
         * is first matched at all, because i2s_start() runs at a fixed rate
         * before any stream exists. FINE means playback drops or duplicates
         * one frame at a time: no channel-down, and continuous rather than
         * stepped.
         */
        out->act    = true;
        out->coarse = (trim_hz > RATE_TRIM_MAX_HZ || trim_hz < -RATE_TRIM_MAX_HZ);
        s->cooldown = 4;               /* windows, against a much longer correction */
    }
}
