/*
 * Audio input for the hub: undecoded SBC over SPI from the bridge, decoded here
 * for the local DAC and the LEDs. This chip is the SPI SLAVE; the bridge clocks.
 *
 * Replaces audio_in.c (I2S slave). The I2S receiver silently dropped ~1.15% of
 * frames because it was sampling a clock from the bridge's crystal -- proven by
 * inverting clock ownership, which took the loss to exactly zero. A UART link
 * came between the two and was retired with the classic hub on 2026-08-12.
 *
 * Going back to a clocked link is the obvious objection to SPI, and the answer
 * is that CS makes it transactional where I2S was continuous -- the bit counter
 * resets every frame, so nothing accumulates. That argument, and the `crc`
 * counter that tests it, are in sbc_link.h. Read it before changing this file.
 */
#pragma once

void sbc_in_start(void);
