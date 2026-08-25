
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#if CONFIG_DANCEFLOOR_WIFI_LOGS

#include "sync_proto.h"

void wifi_log_init(uint8_t role, const char *dest_ip);

void wifi_log_set_dest(const char *dest_ip);

void wifi_log_note_collector(uint32_t s_addr);

bool wifi_log_send_to_dest(const void *buf, size_t len);

unsigned wifi_log_dropped(void);
unsigned wifi_log_no_dest(void);

#else

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
