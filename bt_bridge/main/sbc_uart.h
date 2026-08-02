/*
 * Forwards undecoded SBC frames from the A2DP sink to the hub over UART.
 * See components/dancefloor_sync/include/sbc_link.h for why UART and not I2S.
 */
#pragma once

#include <stdint.h>

#include "sbc_link.h"

void sbc_uart_start(void);

/* Called from the Bluetooth callback. Copies and returns immediately -- the
 * A2DP callback must never wait on a UART. */
void sbc_uart_send(const uint8_t *sbc, uint16_t len);

/* Occasional, small, and not on the audio path -- safe to send from the AVRCP
 * callback. */
void sbc_uart_send_meta(const link_meta_t *meta);
