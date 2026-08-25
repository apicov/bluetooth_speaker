
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {

    bool     phase_valid;

    int32_t  med_us;
    bool     have_med;

    bool     catchup_held;

    int32_t  depth_ms;
    bool     depth_net_held;

    uint32_t rate;
    int32_t  tx_rate;

    int32_t  trim_hz_now;
    int32_t  catchup_now;
} df_servo_in_t;

typedef struct {

    int32_t  err_ema;

    int32_t  adj_phase;
    int32_t  adj;
    bool     depth_net_fired;

    bool     catchup_write;
    int32_t  catchup_frames_new;

    bool     act;
    bool     coarse;
    int32_t  trim_hz;
    uint32_t desired_rate;
} df_servo_out_t;

typedef struct {
    int32_t err_ema;
    bool    err_ema_valid;
    int     cooldown;
} df_servo_t;

int32_t df_servo_ema(df_servo_t *s, int32_t err_in, bool reset_history);

void df_servo_step(df_servo_t *s, const df_servo_in_t *in, df_servo_out_t *out);
