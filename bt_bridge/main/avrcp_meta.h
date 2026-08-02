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
