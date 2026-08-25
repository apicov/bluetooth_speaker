
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void streamer_start(void);

void streamer_fec_start(void);

#define TASK_ANY_CORE (-1)

void task_start(TaskFunction_t fn, const char *name, uint32_t stack,
                UBaseType_t prio, int core);

void streamer_feed(const uint8_t *pcm, uint32_t len);

void streamer_send_sbc(const uint8_t *sbc, uint16_t len, uint32_t frames, bool marker);

void streamer_mark_here(void);

void streamer_request_restart(void);

void streamer_send_meta(const uint8_t *meta, uint16_t len);

void streamer_set_volume(uint8_t volume);

void streamer_send_vol(uint8_t volume);

void vol_repeat_start(void);

void streamer_begin_packet(void);

void streamer_set_sample_rate(uint32_t hz);

uint32_t streamer_take_dropped(void);
