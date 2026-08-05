/*
 * Forwards undecoded SBC frames from the A2DP sink to the hub over SPI, this
 * chip being the master.
 *
 * See components/dancefloor_sync/include/sbc_link.h for the wire format, why
 * the link is not I2S, and why going back to a clocked link is not the same
 * mistake.
 */
#pragma once

#include <stdint.h>

#include "sbc_link.h"

void sbc_link_start(void);

/* Called from the Bluetooth callback. Copies and returns immediately -- the
 * A2DP callback must never wait on the link. */
void sbc_link_send(const uint8_t *sbc, uint16_t len);

/* Occasional, small, and not on the audio path -- safe to send from the AVRCP
 * callback. */
void sbc_link_send_meta(const link_meta_t *meta);
