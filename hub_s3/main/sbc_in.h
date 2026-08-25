/**
 * @file sbc_in.h
 * @brief Audio input for the hub: undecoded SBC over SPI from the bridge,
 *        decoded here for the local DAC and the LEDs.
 *
 * This chip is the SPI SLAVE; the bridge clocks. CS makes the link
 * transactional where an I2S receiver would be continuous -- the bit counter
 * resets every frame, so no framing error accumulates -- and the `crc`
 * counter in sbc_link.h tests exactly that. Read sbc_link.h before changing
 * this file.
 */
#pragma once

/** @brief Bring up the SPI slave link, the decoder and the rx task. */
void sbc_in_start(void);
