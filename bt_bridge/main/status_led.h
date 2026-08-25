/**
 * @file status_led.h
 * @brief The two front-panel LEDs, and the audio-gap diagnostic that shares
 *        their clock.
 *
 * One LED is solid while a phone is connected, the other blinks while audio is
 * actually moving. Both are read-only indicators: nothing here feeds back into
 * the link or the Bluetooth stack, and the calls below only leave a value
 * behind for the driving task to pick up.
 *
 * Either LED can be removed in menuconfig without removing the diagnostic; see
 * status_led.c.
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configure the pins and start the task that drives them.
 *
 * Safe to call with either pin, or both, disabled in menuconfig: the LED simply
 * does not exist, and the task starts regardless.
 */
void status_led_start(void);

/**
 * @brief Record the A2DP link state.
 *
 * @param connected  true while a phone is connected. Anything in between --
 *                   connecting, disconnecting -- is false: the LED means "a
 *                   phone is on the other end", not "something is happening".
 */
void status_led_set_connected(bool connected);

/**
 * @brief Record that one A2DP audio packet arrived.
 *
 * A timestamp store and nothing else, with no locking, which is what makes it
 * cheap enough to call from the audio callback. It is also the only thing on
 * this chip that knows whether the source is still feeding.
 */
void status_led_note_audio(void);

#ifdef __cplusplus
}
#endif
