/*
 * M5 master side: re-send the A2DP audio to satellites over WiFi.
 *
 * Runs a SoftAP (there is no router in a field), answers satellite time probes,
 * and sends undecoded SBC tagged with the master-clock instant each packet
 * should be played at.
 *
 * Unicast to each registered listener, not multicast. Group-addressed frames are
 * never acknowledged and so never retried, which cost ~20% of packets at every
 * PHY rate tried; see streamer.c. Airtime scales with speaker count as a result,
 * which is affordable because the payload is SBC rather than PCM.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void streamer_start(void);

/*
 * xTaskCreate with the return value actually read. Defined in streamer.c.
 *
 * Here rather than in hub.h because sbc_in.c is the other caller and has its own
 * `TAG`, which hub.h's extern would collide with. Every task in this firmware
 * goes through this: it counts failures into n_task_fail, names them, and says
 * so, which is what turns "a unit came up missing a task" from silence into a
 * CRIPPLED line. sbc_in.c open-coded its own check before this was reachable,
 * and so was the one task whose failure nothing counted.
 */
#define TASK_ANY_CORE (-1)

void task_start(TaskFunction_t fn, const char *name, uint32_t stack,
                UBaseType_t prio, int core);

/* Decoded PCM for THIS unit's own speaker. Non-blocking. */
void streamer_feed(const uint8_t *pcm, uint32_t len);

/*
 * Send one SBC packet to every registered satellite, undecoded.
 *
 * `frames` is how many PCM frames it decodes to -- the caller knows, having just
 * decoded it, and the presentation timeline advances by exactly that much.
 */
void streamer_send_sbc(const uint8_t *sbc, uint16_t len, uint32_t frames, bool marker);

/* Tag the audio about to be fed as a sync marker point. */
void streamer_mark_here(void);

/* Flag the next audio packet as a track boundary, so every unit snaps its
 * phase error to zero when playback reaches it. */
void streamer_request_restart(void);

/* Forward track metadata to every registered satellite. Rare and small, so it
 * simply goes out alongside the audio unicast. */
void streamer_send_meta(const uint8_t *meta, uint16_t len);

/* Call before feeding a packet's audio: records where it starts, so it can be
 * paired with the play_at that streamer_send_sbc() assigns to it. */
void streamer_begin_packet(void);

/* From ESP_A2D_AUDIO_CFG_EVT. The presentation timeline advances at this rate,
 * so getting it wrong makes every satellite drift against the master. */
void streamer_set_sample_rate(uint32_t hz);

/* Bytes dropped since the last call because the input buffer was full. */
uint32_t streamer_take_dropped(void);
