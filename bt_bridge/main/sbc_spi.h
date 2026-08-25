/**
 * @file sbc_spi.h
 * @brief The bridge's end of the SBC link: undecoded A2DP payloads out to the
 *        hub over SPI, this chip being the master.
 *
 * The wire format, and the argument for a clocked link, belong to both ends and
 * live in sbc_link.h. This header is only the sending side of it.
 *
 * Every send below is a queue-and-return: nothing on the Bluetooth side ever
 * blocks on the wire, and a packet the queue cannot take is counted and lost
 * rather than waited for.
 */
#pragma once

#include <stdint.h>

#include "sbc_link.h"

/**
 * @brief Bring the link up: handshake pin, SPI bus, queue and transmit task.
 *
 * The sends below are no-ops until this has run, and stay no-ops if it fails,
 * so a link that never started costs audio rather than a crash.
 */
void sbc_link_start(void);

/**
 * @brief Queue one undecoded SBC payload for the hub.
 *
 * Called from the A2DP audio callback, which must never wait on the link: this
 * copies and returns.
 *
 * @param sbc  The payload bytes; the caller keeps them.
 * @param len  How many. Zero is ignored, and more than SBC_LINK_MAX_PAYLOAD is
 *             counted and dropped rather than truncated.
 */
void sbc_link_send(const uint8_t *sbc, uint16_t len);

/**
 * @brief Queue the current track metadata for the hub.
 *
 * Occasional, small and off the audio path, so it is safe to send straight from
 * the AVRCP callback.
 *
 * @param meta  Copied into the packet; the caller keeps ownership.
 */
void sbc_link_send_meta(const link_meta_t *meta);

/**
 * @brief Queue an absolute volume for the hub.
 *
 * Same properties as sbc_link_send_meta(): rare, tiny, and called from a
 * callback rather than from the audio path.
 *
 * @param volume  0..AUDIO_VOL_MAX, as AVRCP states it. The bridge has no output
 *                of its own, so this is forwarded and not applied.
 */
void sbc_link_send_vol(uint8_t volume);
