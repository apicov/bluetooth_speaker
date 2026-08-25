
#include "df_servo.h"

#include "audio_shift.h"
#include "sync_proto.h"

int32_t df_servo_ema(df_servo_t *s, int32_t err_in, bool reset_history)
{
    if (reset_history) {
        s->err_ema_valid = false;
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

    int32_t adj = (int32_t)((int64_t)s->err_ema * in->rate / 100000000LL);

    if (adj >  RATE_TRIM_MAX_HZ) adj =  RATE_TRIM_MAX_HZ;
    if (adj < -RATE_TRIM_MAX_HZ) adj = -RATE_TRIM_MAX_HZ;

    const int32_t adj_phase = adj;
    if (in->depth_net_held) {

    } else if (in->depth_ms < -120) {
        if (adj > -20) adj = -20;
    } else if (in->depth_ms > 120) {
        if (adj < 20) adj = 20;
    }
    out->adj_phase       = adj_phase;
    out->adj             = adj;

    out->depth_net_fired = (adj != adj_phase);

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
                out->catchup_frames_new = want;
            } else if (want > 0 ? want > now : want < now) {
                out->catchup_write      = true;
                out->catchup_frames_new = want;
            }
        } else if (med < CATCHUP_CLEAR_US && med > -CATCHUP_CLEAR_US) {
            out->catchup_write      = true;
            out->catchup_frames_new = 0;
        }
    }

    const uint32_t desired = (uint32_t)((int32_t)in->rate + adj);
    out->desired_rate = desired;

    if (!in->phase_valid) {
        return;
    }

    int32_t deadband = (int32_t)((int64_t)PHASE_DEADBAND_US * in->rate
                                 / 100000000LL);
    if (deadband < 1) {
        deadband = 1;
    }

    const int32_t trim_hz = (int32_t)desired - in->tx_rate;
    out->trim_hz = trim_hz;

    if (s->cooldown > 0) {
        s->cooldown--;
        return;
    }

    const int32_t step = trim_hz - in->trim_hz_now;
    if (step > deadband || step < -deadband) {

        out->act    = true;
        out->coarse = (trim_hz > RATE_TRIM_MAX_HZ || trim_hz < -RATE_TRIM_MAX_HZ);
        s->cooldown = 4;
    }
}
