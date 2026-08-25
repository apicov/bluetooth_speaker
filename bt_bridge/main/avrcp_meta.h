
#pragma once

#include "esp_avrc_api.h"

void avrcp_meta_start(void);

void avrcp_meta_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);

void avrcp_meta_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param);

uint8_t avrcp_meta_volume(void);
