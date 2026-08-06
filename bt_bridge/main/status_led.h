#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Configure the pins and start the task that drives them. Safe to call with
 * either pin (or both) set to -1 in menuconfig; the LED simply does not exist. */
void status_led_start(void);

/* A2DP link state: the "connected" LED is solid on while this is true. */
void status_led_set_connected(bool connected);

/* One A2DP audio packet arrived. Cheap enough for the audio callback -- a
 * timestamp store, no locking -- and it is what makes the second LED blink. */
void status_led_note_audio(void);

#ifdef __cplusplus
}
#endif
