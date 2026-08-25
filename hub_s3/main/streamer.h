/**
 * @file streamer.h
 * @brief The hub's downlink: startup order and the send API.
 *
 * The hub runs a SoftAP (there is no router in a field), answers satellite
 * time probes, and sends undecoded SBC tagged with the master-clock instant
 * each packet should be played at. UNICAST to each registered listener, not
 * multicast: group-addressed frames get no link ACK and no retry, and
 * unicast is the transport the floor measured stable on. The price is that
 * airtime scales with speaker count -- the hub's packet rate is 50 + 96xN;
 * sync_proto.h carries the figures.
 *
 * The sockets live in net.c, the sends in timeline.c and clients.c; this
 * file is only startup order and the API.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/** @brief Bring up the AP, the socket and every task (streamer.c). */
void streamer_start(void);

/** @brief Claim the XOR parity buffer from PSRAM. Called by
 *         streamer_start(); separate because the buffer belongs to
 *         timeline.c, where the send path that fills it lives. */
void streamer_fec_start(void);

/** @brief Sentinel for task_start()'s core argument: no pinning. */
#define TASK_ANY_CORE (-1)

/**
 * @brief xTaskCreate with the return value actually read. Every task in this
 *        firmware goes through this: it counts failures into n_task_fail,
 *        names the task, and says so -- which turns a unit that came up
 *        missing a task from silence into a CRIPPLED line.
 *
 * Declared here rather than hub.h because sbc_in.c is the other caller and
 * carries its own TAG, which hub.h's extern would collide with.
 *
 * @param fn     The task body.
 * @param name   Task name, for the failure line.
 * @param stack  Stack depth in bytes.
 * @param prio   FreeRTOS priority.
 * @param core   Core to pin to, or TASK_ANY_CORE.
 */
void task_start(TaskFunction_t fn, const char *name, uint32_t stack,
                UBaseType_t prio, int core);

/**
 * @brief Decoded PCM for THIS unit's own speaker. Non-blocking; what does
 *        not fit counts in s_feed_dropped.
 * @param pcm  Interleaved stereo frames.
 * @param len  Bytes of pcm.
 */
void streamer_feed(const uint8_t *pcm, uint32_t len);

/**
 * @brief Send one SBC packet to every registered satellite, undecoded.
 * @param sbc     The SBC payload.
 * @param len     Its length in bytes.
 * @param frames  How many PCM frames it decodes to -- the caller knows,
 *                having just decoded it, and the timeline advances by
 *                exactly that much.
 * @param marker  True if this packet carries the sync marker tag.
 */
void streamer_send_sbc(const uint8_t *sbc, uint16_t len, uint32_t frames, bool marker);

/** @brief Tag the audio about to be fed as a sync marker point. */
void streamer_mark_here(void);

/** @brief Flag the next audio packet as a track boundary, so every unit
 *         snaps its phase error to zero when playback reaches it. */
void streamer_request_restart(void);

/**
 * @brief Forward track metadata to every registered satellite.
 * @param meta  The metadata blob.
 * @param len   Its length in bytes.
 */
void streamer_send_meta(const uint8_t *meta, uint16_t len);

/**
 * @brief A new volume from the phone, via the bridge: clamped, kept, and
 *        sent on to every listener. What the SBC input task calls.
 * @param volume  The requested level, 0-127.
 */
void streamer_set_volume(uint8_t volume);

/**
 * @brief Playback volume to every listener, without changing it. The
 *        standing repeat and the join push both go through this.
 * @param volume  The level to send.
 */
void streamer_send_vol(uint8_t volume);

/** @brief Start the once-a-second volume repeat. Called once, at startup,
 *         after the socket exists. */
void vol_repeat_start(void);

/** @brief Call before feeding a packet's audio: records where it starts, so
 *         it can be paired with the play_at that streamer_send_sbc()
 *         assigns to it. */
void streamer_begin_packet(void);

/**
 * @brief Record the input's sample rate, from ESP_A2D_AUDIO_CFG_EVT. The
 *        presentation timeline advances at this rate, so getting it wrong
 *        makes every satellite drift against the master.
 * @param hz  The configured rate.
 */
void streamer_set_sample_rate(uint32_t hz);

/** @brief Bytes dropped since the last call because the input buffer was
 *         full.
 *  @return The dropped byte count; resets the counter. */
uint32_t streamer_take_dropped(void);
