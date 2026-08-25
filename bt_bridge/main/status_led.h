#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void status_led_start(void);

void status_led_set_connected(bool connected);

void status_led_note_audio(void);

#ifdef __cplusplus
}
#endif
