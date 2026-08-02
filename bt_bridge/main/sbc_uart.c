#include "sbc_uart.h"

#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "sbc_link.h"

#define UART_PORT   UART_NUM_1
#define UART_TX_PIN 25          /* reuses the old I2S DATA wire */
#define UART_RX_PIN 34          /* input-only pin, unused: this link is one-way */

/* ~10 A2DP packets. Dropping here is better than stalling Bluetooth, and the
 * seq field means the hub can see it happened. */
#define QUEUE_BYTES (SBC_LINK_MAX_PAYLOAD * 10)

static const char *TAG = "sbc_uart";

static RingbufHandle_t s_ring;
static uint32_t s_seq;
static uint32_t s_dropped;

void sbc_uart_send(const uint8_t *sbc, uint16_t len)
{
    if (!s_ring || len == 0 || len > SBC_LINK_MAX_PAYLOAD) {
        return;
    }

    /*
     * The header, sequence number included, is built HERE rather than in the
     * transmit task. Numbering at transmit time meant a packet dropped by a full
     * queue never got a number, so the hub saw a continuous sequence and
     * reported no loss -- the drops were completely invisible. Numbering at
     * enqueue makes every drop show up as a gap at the far end.
     */
    struct {
        sbc_link_hdr_t hdr;
        uint8_t payload[SBC_LINK_MAX_PAYLOAD];
    } pkt;

    pkt.hdr.sync0 = SBC_LINK_SYNC0;
    pkt.hdr.sync1 = SBC_LINK_SYNC1;
    pkt.hdr.kind = LINK_KIND_SBC;
    pkt.hdr.pad = 0;
    pkt.hdr.len = len;
    pkt.hdr.seq = s_seq++;
    pkt.hdr.checksum = sbc_link_checksum(sbc, len);
    memcpy(pkt.payload, sbc, len);

    if (xRingbufferSend(s_ring, &pkt, sizeof(sbc_link_hdr_t) + len, 0) != pdTRUE) {
        s_dropped++;
    }
}

void sbc_uart_send_meta(const link_meta_t *meta)
{
    if (!s_ring) {
        return;
    }
    struct {
        sbc_link_hdr_t hdr;
        link_meta_t    meta;
    } pkt;

    pkt.hdr.sync0 = SBC_LINK_SYNC0;
    pkt.hdr.sync1 = SBC_LINK_SYNC1;
    pkt.hdr.kind = LINK_KIND_META;
    pkt.hdr.pad = 0;
    pkt.hdr.len = sizeof(link_meta_t);
    pkt.hdr.seq = s_seq++;
    pkt.hdr.checksum = sbc_link_checksum((const uint8_t *)meta, sizeof(link_meta_t));
    pkt.meta = *meta;

    if (xRingbufferSend(s_ring, &pkt, sizeof(pkt), 0) != pdTRUE) {
        s_dropped++;
    }
}

static void tx_task(void *arg)
{
    while (1) {
        size_t len = 0;
        uint8_t *item = (uint8_t *)xRingbufferReceive(s_ring, &len, portMAX_DELAY);
        if (!item) {
            continue;
        }

        /* Already framed by sbc_uart_send(); just push the bytes. */
        uart_write_bytes(UART_PORT, item, len);
        vRingbufferReturnItem(s_ring, item);

        /* Report only when the count moves. Reprinting a static total every
         * five seconds makes a finished startup burst look like an ongoing
         * fault, which is worse than saying nothing. */
        static uint32_t last_reported;
        static int64_t last_report_at;
        int64_t now = esp_timer_get_time();
        if (s_dropped != last_reported && now - last_report_at > 2000000) {
            ESP_LOGW(TAG, "queue full: %" PRIu32 " dropped (+%" PRIu32 ")",
                     s_dropped, s_dropped - last_reported);
            last_reported = s_dropped;
            last_report_at = now;
        }
    }
}

void sbc_uart_start(void)
{
    uart_config_t cfg = {
        .baud_rate = SBC_LINK_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 1024, 8192, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    s_ring = xRingbufferCreate(QUEUE_BYTES, RINGBUF_TYPE_NOSPLIT);
    assert(s_ring);
    xTaskCreate(tx_task, "sbc_tx", 3072, NULL, 10, NULL);

    ESP_LOGI(TAG, "SBC link up: GPIO %d at %d baud", UART_TX_PIN, SBC_LINK_BAUD);
}
