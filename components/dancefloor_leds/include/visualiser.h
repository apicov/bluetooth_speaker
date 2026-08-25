
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VIS_BANDS      4

typedef struct __attribute__((packed)) {
    int64_t due_us;
    int64_t index;
    float   band[VIS_BANDS];
} vis_frame_t;

void visualiser_set_publish(void (*publish)(const vis_frame_t *f));

void visualiser_submit_frame(const vis_frame_t *f);

const char *visualiser_source_name(void);

int visualiser_hop(void);

void visualiser_marker_set_link(bool up);

void visualiser_marker_busy(bool on);

void visualiser_start(void);

void visualiser_set_clock(int64_t (*master_to_local)(int64_t master_us));

void visualiser_flush(void);

void visualiser_set_pattern(const char *name);

void visualiser_set_rate(uint32_t hz);

void visualiser_realign(void);

void visualiser_feed(const uint8_t *pcm, uint32_t len, int64_t due_master_us);

#ifdef __cplusplus
}
#endif
