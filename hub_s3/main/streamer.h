/*
 * M5 master side: re-send the A2DP audio to satellites over WiFi.
 *
 * Runs a SoftAP (there is no router in a field), answers satellite time probes,
 * and sends undecoded SBC tagged with the master-clock instant each packet
 * should be played at.
 *
 * MULTICAST to one group by default (DANCEFLOOR_AUDIO_MCAST), with the
 * per-listener unicast path kept behind the same switch.
 *
 * This paragraph used to say the opposite, in capitals: "unicast to each
 * registered listener, not multicast", because group-addressed frames are never
 * acknowledged and so never retried, measured at ~20% loss at every PHY rate
 * tried. That measurement was real and its conclusion was wrong -- it was taken
 * at the 1 Mbps basic rate 802.11b forces, and with 11b dropped the group goes
 * at 6 Mbps OFDM and loss is 0.2-0.3%, which a speaker cannot hear.
 *
 * What makes it the default is not loss, it is SCALING: one transmission feeds
 * every listener, so this hub's packet rate is FLAT in speaker count, where
 * unicasting the same two lanes costs 50 + 96xN. The claim that "airtime scales
 * with speaker count as a result" is exactly what no longer holds; README.md
 * and sync_proto.h carry the packet-rate figures.
 *
 * sync_proto.h has the measurements in the order they were taken, and the
 * transmit-buffer count that has to come with the choice. The code is in net.c
 * (sockets, the group) and timeline.c (fan_out); this file is startup order and
 * the API, and has not held that code since 2026-08-12.
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
 * simply goes out over the client list, which is where the audio used to go
 * too -- see the note above on what moved to the group and what did not. */
void streamer_send_meta(const uint8_t *meta, uint16_t len);

/* A new volume from the phone, via the bridge: clamped, kept, and sent on to
 * every listener. This is what the SBC input task calls. */
void streamer_set_volume(uint8_t volume);

/* Playback volume to every listener, without changing it. The standing repeat
 * and the join push both go through this; see the definition for why it is
 * addressed the way it is. */
void streamer_send_vol(uint8_t volume);

/* Start the once-a-second volume repeat. Called once, at startup, after the
 * socket exists -- see the definition for why it is a timer and not a counter
 * in the audio path. */
void vol_repeat_start(void);

/* Call before feeding a packet's audio: records where it starts, so it can be
 * paired with the play_at that streamer_send_sbc() assigns to it. */
void streamer_begin_packet(void);

/* From ESP_A2D_AUDIO_CFG_EVT. The presentation timeline advances at this rate,
 * so getting it wrong makes every satellite drift against the master. */
void streamer_set_sample_rate(uint32_t hz);

/* Bytes dropped since the last call because the input buffer was full. */
uint32_t streamer_take_dropped(void);
