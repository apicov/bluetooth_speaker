/*
 * M5 master side: re-broadcast the A2DP audio to satellites over WiFi.
 *
 * Runs a SoftAP (there is no router in a field), answers satellite time probes,
 * and multicasts PCM chunks tagged with the master-clock instant each should be
 * played. Multicast rather than per-satellite unicast so radio airtime does not
 * scale with the number of speakers.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

void streamer_start(void);

/* Decoded PCM for THIS unit's own speaker. Non-blocking. */
void streamer_feed(const uint8_t *pcm, uint32_t len);

/*
 * Multicast one SBC packet to the satellites, undecoded.
 *
 * `frames` is how many PCM frames it decodes to -- the caller knows, having just
 * decoded it, and the presentation timeline advances by exactly that much.
 */
void streamer_send_sbc(const uint8_t *sbc, uint16_t len, uint32_t frames, bool marker);

/* Tag the audio about to be fed as a sync marker point. */
void streamer_mark_here(void);

/* From ESP_A2D_AUDIO_CFG_EVT. The presentation timeline advances at this rate,
 * so getting it wrong makes every satellite drift against the master. */
void streamer_set_sample_rate(uint32_t hz);

/* Bytes dropped since the last call because the input buffer was full. */
uint32_t streamer_take_dropped(void);
