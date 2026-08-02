/*
 * Audio input for the hub: undecoded SBC over UART from the bridge, decoded
 * here for the local DAC and the LEDs.
 *
 * Replaces audio_in.c (I2S slave). The I2S receiver silently dropped ~1.15% of
 * frames because it was sampling a clock from the bridge's crystal -- proven by
 * inverting clock ownership, which took the loss to exactly zero. UART is
 * self-clocking per byte, so no clock is shared and the failure mode cannot
 * occur. It also carries a quarter of the bytes.
 */
#pragma once

void sbc_in_start(void);
