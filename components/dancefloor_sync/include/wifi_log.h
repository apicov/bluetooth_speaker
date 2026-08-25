/**
 * @file wifi_log.h
 * @brief Off-board log shipping over the dancefloor sync socket.
 *
 * With CONFIG_DANCEFLOOR_WIFI_LOGS set, every log line is mirrored to a laptop
 * collector -- the hub relays the satellites' lines too -- and a low-priority
 * task does the sending. With it unset the whole module compiles to the no-ops
 * below, so a production flash carries none of it.
 *
 * See sync_proto.h for the messages (MSG_LOG, MSG_HEALTH, MSG_LOG_SUB) and for
 * why the collector needs a role and a stamped source address to tell units
 * apart.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* Not transitively guaranteed: an includer that has not pulled in an IDF
 * header first would otherwise take the no-op branch below. */
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#if CONFIG_DANCEFLOOR_WIFI_LOGS

#include "sync_proto.h"   /* LOG_ROLE_HUB / LOG_ROLE_SAT */

/**
 * @brief Install the log hook and start the shipper task. Call once, after the
 *        WiFi and socket bring-up.
 *
 * @param role     LOG_ROLE_HUB or LOG_ROLE_SAT, stamped on every message so
 *                 the collector can split its sources.
 * @param dest_ip  The initial destination, or NULL. A satellite passes the hub
 *                 address; the hub passes NULL and waits for
 *                 wifi_log_note_collector() to be called from a received
 *                 MSG_LOG_SUB.
 */
void wifi_log_init(uint8_t role, const char *dest_ip);

/**
 * @brief Point the shipper at an address, stickily -- no expiry.
 *
 * The satellite points at the hub once, at init. Contrast
 * wifi_log_note_collector(), which is the hub's expiring registration.
 *
 * @param dest_ip  Dotted-quad destination, or NULL to send nowhere.
 */
void wifi_log_set_dest(const char *dest_ip);

/**
 * @brief Refresh the collector destination from a received MSG_LOG_SUB.
 *
 * The registration expires, so a collector that stops sending is dropped
 * rather than being relayed to for ever. It resends every few seconds, so the
 * TTL only bounds cleanup.
 *
 * @param s_addr  The source address in network byte order, straight from
 *                recvfrom()'s sockaddr.
 */
void wifi_log_note_collector(uint32_t s_addr);

/**
 * @brief Send an already-built message to the current destination.
 *
 * Used for a MSG_HEALTH built locally, and by the hub to relay a satellite's
 * MSG_LOG or MSG_HEALTH verbatim.
 *
 * @param buf  The message.
 * @param len  Its length.
 * @return false if there is no current destination, or the send failed.
 */
bool wifi_log_send_to_dest(const void *buf, size_t len);

/**
 * @brief Log lines this unit lost, for health_msg_t::log_dropped.
 *
 * Either the hook could not queue the line or the non-blocking send found no
 * transmit buffer. Nonzero means the collector's stream has holes, which is
 * worth knowing before concluding anything from a gap in it.
 *
 * @return The running count.
 */
unsigned wifi_log_dropped(void);

/**
 * @brief Log lines there was no destination for, for
 *        health_msg_t::log_no_dest.
 * @return The running count.
 */
unsigned wifi_log_no_dest(void);

#else  /* !CONFIG_DANCEFLOOR_WIFI_LOGS -- the whole feature compiles out */

static inline void wifi_log_init(uint8_t role, const char *dest_ip) { (void)role; (void)dest_ip; }
static inline void wifi_log_set_dest(const char *dest_ip) { (void)dest_ip; }
static inline void wifi_log_note_collector(uint32_t s_addr) { (void)s_addr; }
static inline bool wifi_log_send_to_dest(const void *buf, size_t len) { (void)buf; (void)len; return false; }
static inline unsigned wifi_log_dropped(void) { return 0; }
static inline unsigned wifi_log_no_dest(void) { return 0; }

#endif

#ifdef __cplusplus
}
#endif
