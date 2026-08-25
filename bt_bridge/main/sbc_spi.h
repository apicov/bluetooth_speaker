
#pragma once

#include <stdint.h>

#include "sbc_link.h"

void sbc_link_start(void);

void sbc_link_send(const uint8_t *sbc, uint16_t len);

void sbc_link_send_meta(const link_meta_t *meta);

void sbc_link_send_vol(uint8_t volume);
