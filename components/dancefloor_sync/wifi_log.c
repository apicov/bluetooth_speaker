
#include "sdkconfig.h"

#if CONFIG_DANCEFLOOR_WIFI_LOGS

#include "wifi_log.h"

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

#define COLLECTOR_TTL_US  (30 * 1000000)

#define QUEUE_DEPTH 16

typedef struct {
    uint8_t  level;
    uint8_t  tag_len;
    uint16_t msg_len;
    char     tag[LOG_TAG_MAX];
    char     msg[LOG_MSG_MAX];
} log_item_t;

static vprintf_like_t s_prev;
static QueueHandle_t   s_queue;
static int             s_sock = -1;
static uint8_t         s_role = LOG_ROLE_HUB;

static volatile uint32_t s_dest_addr;
static volatile int64_t  s_dest_expire_us;

static volatile uint32_t s_dropped;
static volatile uint32_t s_no_dest;

static log_item_t   s_queue_storage[QUEUE_DEPTH];
static StaticQueue_t s_queue_ctrl;

static int wifi_log_hook(const char *fmt, va_list args)
{

    int rc = 0;
    if (s_prev) {
        va_list uart;
        va_copy(uart, args);
        rc = s_prev(fmt, uart);
        va_end(uart);
    }

    char line[LOG_MSG_MAX + 48];
    va_list ap;
    va_copy(ap, args);
    int n = vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);
    if (n <= 0) {
        return rc;
    }

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

    size_t msg_len = strlen(msg_start);
    while (msg_len > 0 && (msg_start[msg_len - 1] == '\n' ||
                           msg_start[msg_len - 1] == '\r')) {
        msg_len--;
    }

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
        m.src_ip  = 0;
        memcpy(m.tag, item.tag, item.tag_len);
        memcpy(m.msg, item.msg, item.msg_len);

        struct sockaddr_in to = {
            .sin_family = AF_INET,
            .sin_port   = htons(SYNC_PORT),
            .sin_addr.s_addr = dest,
        };
        if (sendto(s_sock, &m, LOG_MSG_BYTES(item.msg_len), 0,
                   (struct sockaddr *)&to, sizeof to) < 0) {
            s_dropped++;
        }
    }
}

void wifi_log_init(uint8_t role, const char *dest_ip)
{
    s_role = role;
    s_queue = xQueueCreateStatic(QUEUE_DEPTH, sizeof(log_item_t),
                                 (uint8_t *)s_queue_storage, &s_queue_ctrl);
    assert(s_queue);

    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    assert(s_sock >= 0);

    fcntl(s_sock, F_SETFL, fcntl(s_sock, F_GETFL, 0) | O_NONBLOCK);

    if (dest_ip) {
        wifi_log_set_dest(dest_ip);
    }

    s_prev = esp_log_set_vprintf(wifi_log_hook);

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
    s_dest_addr      = inet_addr(dest_ip);
    s_dest_expire_us = 0;
}

void wifi_log_note_collector(uint32_t s_addr)
{
    s_dest_addr      = s_addr;
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
        s_dropped++;
        return false;
    }
    return true;
}

unsigned wifi_log_dropped(void) { return s_dropped; }
unsigned wifi_log_no_dest(void) { return s_no_dest; }

#endif
