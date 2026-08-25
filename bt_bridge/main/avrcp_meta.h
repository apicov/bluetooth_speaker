/**
 * @file avrcp_meta.h
 * @brief AVRCP: track metadata and track-change notifications from the phone,
 *        and absolute volume back from it.
 *
 * A2DP alone carries audio and nothing about it. Bringing AVRCP up adds two
 * things this system wants. Title, artist and album are the obvious one. The
 * more useful one is the track-change notification: an unambiguous instant at
 * which splicing the audio timeline is inaudible, which silence detection
 * cannot offer because it false-triggers on a quiet passage.
 *
 * The target half is the other direction. Without a target the handset has
 * nowhere to send volume, so it scales the PCM before the SBC encoder and every
 * step down is resolution destroyed before the audio leaves the phone. With
 * one, the phone transmits at full scale and states a level instead. This unit
 * has no output of its own, so it forwards that level rather than acting on it;
 * the speakers apply it at their own output.
 *
 * Everything here is published onto the SBC link, never consumed locally.
 */
#pragma once

#include "esp_avrc_api.h"

/**
 * @brief Initialise the AVRCP controller and target, and start the volume
 *        heartbeat.
 *
 * Call before the A2DP sink starts, so that AVRCP is advertised by the time the
 * phone connects and asks for it.
 */
void avrcp_meta_start(void);

/**
 * @brief Controller events: connection state, track change, metadata responses.
 *
 * Registered directly rather than through a wrapper, so the signature matches
 * esp_avrc_ct_cb_t exactly -- casting a differently-typed function pointer is
 * undefined behaviour, and the compiler rejects it anyway.
 *
 * @param event  Which controller event arrived.
 * @param param  Its payload, owned by the stack for the duration of the call.
 */
void avrcp_meta_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);

/**
 * @brief Target events: connection state, absolute volume, and the phone
 *        registering for volume-change notifications.
 *
 * Same signature rule as avrcp_meta_ct_cb().
 *
 * @param event  Which target event arrived.
 * @param param  Its payload, owned by the stack for the duration of the call.
 */
void avrcp_meta_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param);

/**
 * @brief What the phone last said the level should be, 0..AUDIO_VOL_MAX.
 *
 * For the paths that re-state the volume without a command to act on. Full
 * scale until the phone states otherwise, which on a unit with no output of its
 * own is a value to forward rather than a loudness.
 *
 * @return The current absolute volume.
 */
uint8_t avrcp_meta_volume(void);
