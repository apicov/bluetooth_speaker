/**
 * @file wifi_log.c
 * @brief The log capture and shipper, i.e. the whole of
 *        CONFIG_DANCEFLOOR_WIFI_LOGS. wifi_log.h owns the contract.
 *
 * The hook runs in whichever task logged. It writes to the UART FIRST, by
 * calling the previous vprintf the log system handed back, so console output
 * is unchanged in content and timing; then it enqueues a copy of the line. It
 * never blocks and never allocates -- a non-blocking send into a statically
 * allocated queue, drop-on-full. A blocking send in the hook would close a
 * loss -> log -> overflow -> loss loop, so the shipper task is the only thing
 * that calls sendto() for captured lines.
 *
 * The socket is non-blocking for the same reason. wifi_log_send_to_dest() is
 * also called from the hub's probe task -- the RECEIVE path -- to relay a
 * satellite's line, and a blocking sendto() would wait on the TCP/IP thread.
 * Diagnostics must never make the unit wait: a line that cannot be handed to
 * the driver now is counted and abandoned.
 *
 * Pre-init lines cannot be captured: those go out before the hook is
 * installed, and it is not reinstalled afterwards. That is fine -- this is for
 * the steady-state bench view, not the boot.
 */
/* Ahead of the guard: IDF does not force-include sdkconfig.h, so without this
 * the #if below is evaluated with the symbol undefined and the file compiles
 * to an empty object while the header still declares the prototypes. */
#include "sdkconfig.h"

#if CONFIG_DANCEFLOOR_WIFI_LOGS

#include "wifi_log.h"

/*
 * Log v2 hands the hook one call per fragment (header, tag, body) instead of
 * one composite line, so the level and tag parse below would silently produce
 * garbage. Both apps select v1 today; this catches a later flip at build time.
 */
#if CONFIG_LOG_VERSION != 1
#error "wifi_log parses the log-v1 composite line; select CONFIG_LOG_VERSION_1"
#endif

#include <assert.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

/** @brief How long the hub keeps forwarding to an address after its last
 *         MSG_LOG_SUB. The collector resends every few seconds, so this only
 *         bounds cleanup. */
#define COLLECTOR_TTL_US  (30 * 1000000)

/** @brief Queue depth. Logs are bursty but low-rate at INFO level, so this
 *         absorbs a short run and the dropped counter says if it is too
 *         shallow. Kept shallow on purpose -- the hub's heap is the tightest
 *         in the system. */
#define QUEUE_DEPTH 16

/** @brief One captured line, before the shipper adds the wire header. Copied
 *         by value into the queue, so the hook never allocates. */
typedef struct {
    uint8_t  level;              /**< The level char: 'E', 'W', 'I', 'D'. */
    uint8_t  tag_len;            /**< Real bytes of #tag. */
    uint16_t msg_len;            /**< Real bytes of #msg. */
    char     tag[LOG_TAG_MAX];   /**< Not necessarily terminated. */
    char     msg[LOG_MSG_MAX];   /**< Likewise. */
} log_item_t;

/** @brief The vprintf the log system was using before the hook; the hook calls
 *         it so the console is unchanged. */
static vprintf_like_t s_prev;
/** @brief Hook to shipper. */
static QueueHandle_t   s_queue;
/** @brief Send-only, unbound, non-blocking. */
static int             s_sock = -1;
/** @brief LOG_ROLE_HUB or LOG_ROLE_SAT, stamped on every message. */
static uint8_t         s_role = LOG_ROLE_HUB;

/** @brief Destination, network byte order; 0 = none. */
static volatile uint32_t s_dest_addr;
/** @brief 0 means the destination is STICKY (a satellite's hub address);
 *         otherwise the deadline after which it is forgotten until the next
 *         subscribe. */
static volatile int64_t  s_dest_expire_us;

/** @brief Lines lost: the hook queue was full, or the send failed. Reported in
 *         health_msg_t::log_dropped. */
static volatile uint32_t s_dropped;
/** @brief Lines the shipper had nowhere to send. */
static volatile uint32_t s_no_dest;

/** @brief Statically allocated, so the heap-constrained hub need not find a
 *         few kilobytes for the queue. */
static log_item_t   s_queue_storage[QUEUE_DEPTH];
/** @brief ...and its control block. */
static StaticQueue_t s_queue_ctrl;

/**
 * @brief The log hook: mirror the line to the UART, then queue a copy.
 *
 * The v1 path calls this with the COMPOSITE format "L (ts) tag: msg\n" and
 * args (timestamp, tag, user...). With colours on, the line is wrapped in
 * escapes, so a leading one is skipped rather than forbidden -- the level char
 * is the first byte after it. The tag is the text between ") " and the first
 * ": ". If the shape is ever not what is expected, the whole line ships as the
 * message with an empty tag rather than being dropped.
 *
 * @param fmt   The composite format.
 * @param args  Its arguments.
 * @return Whatever the previous vprintf returned.
 */
static int wifi_log_hook(const char *fmt, va_list args)
{
    /* UART first, unchanged. */
    int rc = 0;
    if (s_prev) {
        va_list uart;
        va_copy(uart, args);
        rc = s_prev(fmt, uart);
        va_end(uart);
    }

    /* Format the composite line into our own buffer. The headroom leaves room
     * for the "L (timestamp) <tag>: " prefix in front of a full-length
     * message. */
    char line[LOG_MSG_MAX + 48];
    va_list ap;
    va_copy(ap, args);
    int n = vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);
    if (n <= 0) {
        return rc;
    }

    /* Skip a leading colour escape, if colours are on. */
    char *body = line;
    if (body[0] == '\033' && body[1] == '[') {
        char *m = strchr(body, 'm');
        if (!m) {
            return rc;
        }
        body = m + 1;
    }

    char     tag[LOG_TAG_MAX];
    size_t   tag_len = 0;
    const char *msg_start = body;
    char *paren = strchr(body, ')');
    if (paren && paren[1] == ' ') {
        const char *tagb = paren + 2;
        const char *colon = strstr(tagb, ": ");
        if (colon && colon > tagb) {
            tag_len = (size_t)(colon - tagb);
            if (tag_len > LOG_TAG_MAX - 1) {
                tag_len = LOG_TAG_MAX - 1;
            }
            memcpy(tag, tagb, tag_len);
            tag[tag_len] = '\0';
            msg_start = colon + 2;
        }
    }

    /* Strip the trailing newline the composite format adds... */
    size_t msg_len = strlen(msg_start);
    while (msg_len > 0 && (msg_start[msg_len - 1] == '\n' ||
                           msg_start[msg_len - 1] == '\r')) {
        msg_len--;
    }

    /* ...and the matching colour reset, if colours are on. */
    if (msg_len >= 4 && msg_start[msg_len - 1] == 'm' &&
        msg_start[msg_len - 4] == '\033') {
        msg_len -= 4;
    }
    if (msg_len > LOG_MSG_MAX) {
        msg_len = LOG_MSG_MAX;
    }

    log_item_t item;
    memset(&item, 0, sizeof item);
    item.level   = (uint8_t)body[0];
    item.tag_len = (uint8_t)tag_len;
    item.msg_len = (uint16_t)msg_len;
    memcpy(item.tag, tag, tag_len);
    memcpy(item.msg, msg_start, msg_len);

    if (xQueueSend(s_queue, &item, 0) != pdTRUE) {
        s_dropped++;
    }
    return rc;
}

/**
 * @brief Where to send right now, honouring the expiry.
 * @param[out] out  The destination, network byte order.
 * @return false if there is none at the moment.
 */
static bool dest_now(uint32_t *out)
{
    uint32_t addr = s_dest_addr;
    if (addr == 0) {
        return false;
    }
    int64_t exp = s_dest_expire_us;
    if (exp != 0 && esp_timer_get_time() > exp) {
        return false;
    }
    *out = addr;
    return true;
}

/**
 * @brief Drain the queue and send. The only thing that calls sendto() for a
 *        captured line; see the file header for why the hook must not.
 * @param arg  Unused; the FreeRTOS task signature.
 */
static void shipper_task(void *arg)
{
    (void)arg;
    log_item_t item;
    uint32_t   seq = 0;

    while (1) {
        xQueueReceive(s_queue, &item, portMAX_DELAY);

        uint32_t dest;
        if (!dest_now(&dest)) {
            s_no_dest++;
            continue;
        }

        log_msg_t m;
        memset(&m, 0, sizeof m);
        m.type    = MSG_LOG;
        m.level   = item.level;
        m.role    = s_role;
        m.tag_len = item.tag_len;
        m.msg_len = item.msg_len;
        m.seq     = seq++;
        m.src_ip  = 0;   /* the hub stamps the relay; 0 = self on the origin */
        memcpy(m.tag, item.tag, item.tag_len);
        memcpy(m.msg, item.msg, item.msg_len);

        struct sockaddr_in to = {
            .sin_family = AF_INET,
            .sin_port   = htons(SYNC_PORT),
            .sin_addr.s_addr = dest,
        };
        if (sendto(s_sock, &m, LOG_MSG_BYTES(item.msg_len), 0,
                   (struct sockaddr *)&to, sizeof to) < 0) {
            s_dropped++;   /* no TX buffer right now; the line is gone */
        }
    }
}

/* Declared in wifi_log.h, like the five entry points below it. */
void wifi_log_init(uint8_t role, const char *dest_ip)
{
    s_role = role;
    s_queue = xQueueCreateStatic(QUEUE_DEPTH, sizeof(log_item_t),
                                 (uint8_t *)s_queue_storage, &s_queue_ctrl);
    assert(s_queue);

    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);   /* send-only, unbound */
    assert(s_sock >= 0);
    /* Never wait on the TCP/IP thread: see the note at the top of the file. */
    fcntl(s_sock, F_SETFL, fcntl(s_sock, F_GETFL, 0) | O_NONBLOCK);

    if (dest_ip) {
        wifi_log_set_dest(dest_ip);
    }

    /* Install last: nothing above logs, and the queue and socket must exist
     * before any line can reach the hook. */
    s_prev = esp_log_set_vprintf(wifi_log_hook);

    /*
     * Checked, and this one is the worst of the set to lose silently: without
     * the shipper nothing drains the queue, so every captured line is dropped
     * at the hook and the unit goes quiet over the air while its console
     * carries on normally. A remote unit would look dead. The console still
     * works, so this line reaches somebody.
     */
    if (xTaskCreate(shipper_task, "wifi_log", 3072, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE("wifi_log", "TASK \"wifi_log\" FAILED TO START -- this unit's "
                             "logs will not leave it over WiFi");
    }
}

void wifi_log_set_dest(const char *dest_ip)
{
    if (!dest_ip) {
        s_dest_addr = 0;
        return;
    }
    s_dest_addr      = inet_addr(dest_ip);   /* network byte order */
    s_dest_expire_us = 0;                    /* sticky */
}

void wifi_log_note_collector(uint32_t s_addr)
{
    s_dest_addr      = s_addr;               /* already network byte order */
    s_dest_expire_us = esp_timer_get_time() + COLLECTOR_TTL_US;
}

bool wifi_log_send_to_dest(const void *buf, size_t len)
{
    uint32_t dest;
    if (!dest_now(&dest)) {
        s_no_dest++;
        return false;
    }
    struct sockaddr_in to = {
        .sin_family = AF_INET,
        .sin_port   = htons(SYNC_PORT),
        .sin_addr.s_addr = dest,
    };
    if (sendto(s_sock, buf, len, 0, (struct sockaddr *)&to, sizeof to) != (ssize_t)len) {
        s_dropped++;   /* would-block on a full TX pool, mostly */
        return false;
    }
    return true;
}

unsigned wifi_log_dropped(void) { return s_dropped; }
unsigned wifi_log_no_dest(void) { return s_no_dest; }

#endif
