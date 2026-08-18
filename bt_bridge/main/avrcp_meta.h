/*
 * AVRCP controller: track metadata and track-change notifications.
 *
 * The A2DP example never initialises AVRCP -- hence "AVRC not Init, not using
 * it" in its log, and the phone's request for PSM 23 being refused. Enabling it
 * gives title/artist/album, and more usefully a track-change event: an
 * unambiguous instant at which splicing audio is inaudible.
 */
#pragma once

#include "esp_avrc_api.h"

void avrcp_meta_start(void);

/* Signature matches esp_avrc_ct_cb_t exactly -- casting a differently-typed
 * function pointer is undefined behaviour, and the compiler rejects it. */
void avrcp_meta_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);

/* The target half: absolute volume, forwarded to the speakers rather than
 * acted on here. Same signature rule as the controller callback above. */
void avrcp_meta_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param);

/* What the phone last said the level should be, 0-127. For the paths that
 * re-state it without a command to act on; see the heartbeat in avrcp_meta.c
 * for why anything re-states it at all. */
uint8_t avrcp_meta_volume(void);
