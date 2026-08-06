/*
 * Off-board log shipping over the dancefloor sync socket.
 *
 * When CONFIG_DANCEFLOOR_WIFI_LOGS is set, every ESP_LOG line is mirrored to a
 * laptop collector (the hub relays satellite logs too), and a low-priority
 * task does the sending. When it is unset the whole module compiles to no-ops,
 * so a production flash carries none of it. See sync_proto.h for the messages
 * (MSG_LOG / MSG_HEALTH / MSG_LOG_SUB) and the reasoning on the receive path.
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

/*
 * Install the ESP_LOG hook and start the shipper task. Call once, after the
 * WiFi/socket bring-up. `role` is LOG_ROLE_HUB or LOG_ROLE_SAT (stamped on
 * every message so the collector can split sources). `dest_ip` is the initial
 * destination -- the satellite passes the hub address; the hub passes NULL and
 * waits for wifi_log_note_collector() from a received MSG_LOG_SUB.
 */
void wifi_log_init(uint8_t role, const char *dest_ip);

/* Sticky destination, no expiry. The satellite points at the hub once at init. */
void wifi_log_set_dest(const char *dest_ip);

/*
 * Refresh the collector destination from a received MSG_LOG_SUB. `s_addr` is the
 * network-byte-order source address straight from recvfrom()'s sockaddr; the hub
 * forwards to it for ~30 s after the last refresh, then drops until it returns.
 */
void wifi_log_note_collector(uint32_t s_addr);

/*
 * Send an already-built message (a MSG_HEALTH built locally, or a MSG_LOG /
 * MSG_HEALTH the hub is relaying verbatim) to the current destination. Returns
 * false if there is no current destination or the send failed.
 */
bool wifi_log_send_to_dest(const void *buf, size_t len);

/* Diagnostics, carried in health_msg_t: how many log lines were lost (hook queue
 * full, or the non-blocking send found no TX buffer), and how many there was no
 * destination for. Nonzero dropped means the collector's stream has holes. */
unsigned wifi_log_dropped(void);
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
