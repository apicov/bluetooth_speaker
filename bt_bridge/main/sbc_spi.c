#include "sbc_spi.h"

#include <inttypes.h>
#include <stdatomic.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "sbc_link.h"

#define SPI_LINK_HOST SPI2_HOST
#define PIN_SCK       14
#define PIN_MOSI      13
#define PIN_CS        15

#define PIN_HANDSHAKE 25

#define HANDSHAKE_TIMEOUT_MS 100

#define QUEUE_BYTES (SBC_LINK_MAX_PAYLOAD * 20)

static const char *TAG = "sbc_spi";

static RingbufHandle_t s_ring;
static spi_device_handle_t s_spi;
static TaskHandle_t s_tx_task;
static uint8_t *s_frame;

static struct {
    spi_link_hdr_t hdr;
    uint8_t payload[SBC_LINK_MAX_PAYLOAD];
} s_pkt;

static _Atomic uint32_t s_seq;
static uint32_t s_dropped;
static uint32_t s_oversize;
static uint32_t s_stalled;
static uint32_t s_txerr;

void sbc_link_send(const uint8_t *sbc, uint16_t len)
{
    if (!s_ring || len == 0) {
        return;
    }

    if (len > SBC_LINK_MAX_PAYLOAD) {
        s_oversize++;
        return;
    }

    s_pkt.hdr.kind = LINK_KIND_SBC;
    s_pkt.hdr.rsv = 0;
    s_pkt.hdr.rsv2 = 0;
    s_pkt.hdr.len = len;
    s_pkt.hdr.seq = s_seq++;
    s_pkt.hdr.crc = sbc_link_crc16(&s_pkt.hdr, sbc, len);
    memcpy(s_pkt.payload, sbc, len);

    if (xRingbufferSend(s_ring, &s_pkt, sizeof(spi_link_hdr_t) + len, 0) != pdTRUE) {
        s_dropped++;
    }
}

void sbc_link_send_meta(const link_meta_t *meta)
{
    if (!s_ring) {
        return;
    }
    struct {
        spi_link_hdr_t hdr;
        link_meta_t    meta;
    } pkt;

    pkt.hdr.kind = LINK_KIND_META;
    pkt.hdr.rsv = 0;
    pkt.hdr.rsv2 = 0;
    pkt.hdr.len = sizeof(link_meta_t);
    pkt.hdr.seq = s_seq++;
    pkt.hdr.crc = sbc_link_crc16(&pkt.hdr, meta, sizeof(link_meta_t));
    pkt.meta = *meta;

    if (xRingbufferSend(s_ring, &pkt, sizeof(pkt), 0) != pdTRUE) {
        s_dropped++;
    }
}

void sbc_link_send_vol(uint8_t volume)
{
    if (!s_ring) {
        return;
    }
    struct {
        spi_link_hdr_t hdr;
        link_vol_t     vol;
    } pkt;

    pkt.vol.volume = volume;
    pkt.hdr.kind = LINK_KIND_VOL;
    pkt.hdr.rsv = 0;
    pkt.hdr.rsv2 = 0;
    pkt.hdr.len = sizeof(link_vol_t);
    pkt.hdr.seq = s_seq++;
    pkt.hdr.crc = sbc_link_crc16(&pkt.hdr, &pkt.vol, sizeof(link_vol_t));

    if (xRingbufferSend(s_ring, &pkt, sizeof(pkt), 0) != pdTRUE) {
        s_dropped++;
    }
}

static void IRAM_ATTR handshake_isr(void *arg)
{
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_tx_task, &woken);
    if (woken) {
        portYIELD_FROM_ISR();
    }
}

static bool wait_for_handshake(void)
{
    if (gpio_get_level(PIN_HANDSHAKE)) {
        return true;
    }
    ulTaskNotifyTake(pdTRUE, 0);
    if (gpio_get_level(PIN_HANDSHAKE)) {
        return true;
    }
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(HANDSHAKE_TIMEOUT_MS)) == 0) {
        s_stalled++;
        return false;
    }
    return true;
}

static void report_when_moved(const char *what, uint32_t count,
                              uint32_t *last, int64_t *last_at)
{
    const int64_t now = esp_timer_get_time();
    if (count != *last && now - *last_at > 2000000) {
        ESP_LOGW(TAG, "%s: %" PRIu32 " dropped (+%" PRIu32 ")",
                 what, count, count - *last);
        *last = count;
        *last_at = now;
    }
}

#define TX_IDLE_TICK_MS 500

static void tx_task(void *arg)
{
    static uint32_t last_dropped, last_oversize, last_stalled, last_txerr;
    static int64_t dropped_at, oversize_at, stalled_at, txerr_at;

    while (1) {
        size_t len = 0;
        uint8_t *item = (uint8_t *)xRingbufferReceive(s_ring, &len,
                                                      pdMS_TO_TICKS(TX_IDLE_TICK_MS));
        if (item) {

            memcpy(s_frame, item, len);
            memset(s_frame + len, 0, SBC_LINK_FRAME_BYTES - len);
            vRingbufferReturnItem(s_ring, item);

            if (wait_for_handshake()) {
                spi_transaction_t t = {
                    .length = SBC_LINK_FRAME_BYTES * 8,
                    .tx_buffer = s_frame,
                };

                const esp_err_t err = spi_device_transmit(s_spi, &t);
                if (err != ESP_OK) {
                    s_txerr++;
                }
            }
        }

        report_when_moved("queue full", s_dropped, &last_dropped, &dropped_at);
        report_when_moved("payload past the link ceiling", s_oversize,
                          &last_oversize, &oversize_at);
        report_when_moved("hub never raised the handshake", s_stalled,
                          &last_stalled, &stalled_at);
        report_when_moved("SPI transmit failed", s_txerr, &last_txerr, &txerr_at);
    }
}

void sbc_link_start(void)
{
    const gpio_config_t hs = {
        .pin_bit_mask = 1ULL << PIN_HANDSHAKE,
        .mode = GPIO_MODE_INPUT,

        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&hs));

    const spi_bus_config_t bus = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SBC_LINK_FRAME_BYTES,
    };
    const spi_device_interface_config_t dev = {
        .clock_speed_hz = SBC_LINK_SPI_HZ,
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_LINK_HOST, &bus, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_LINK_HOST, &dev, &s_spi));

    s_frame = heap_caps_malloc(SBC_LINK_FRAME_BYTES, MALLOC_CAP_DMA);
    assert(s_frame);

    s_ring = xRingbufferCreate(QUEUE_BYTES, RINGBUF_TYPE_NOSPLIT);
    assert(s_ring);

    if (xTaskCreate(tx_task, "sbc_tx", 3072, NULL, 10, &s_tx_task) != pdPASS) {
        s_tx_task = NULL;
        ESP_LOGE(TAG, "TASK \"sbc_tx\" FAILED TO START -- the SBC link is DOWN, "
                      "no audio will reach the hub. Heap %u free, largest %u",
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        return;
    }

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_HANDSHAKE, handshake_isr, NULL));

    ESP_LOGI(TAG, "SBC link up: SPI master at %d Hz, sck %d mosi %d cs %d, "
                  "handshake in on %d", SBC_LINK_SPI_HZ, PIN_SCK, PIN_MOSI,
             PIN_CS, PIN_HANDSHAKE);
}
