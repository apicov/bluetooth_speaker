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

/*
 * HSPI. Nothing else on this chip uses a SPI bus -- it runs Bluetooth and this
 * link and nothing more -- so the choice is free here, unlike on the hub where
 * the LED strip already holds SPI2.
 *
 * The three signals sit on HSPI's IOMUX pins rather than anywhere convenient.
 * Routing SPI through the GPIO matrix adds delay that only begins to matter as
 * the clock rises, and raising the clock is the entire point of this change, so
 * paying nothing for it now beats rediscovering it at 10 MHz.
 *
 * No MISO: the link is one-way. That is worth a pin as well -- HSPI's MISO is
 * GPIO 12, the MTDI strapping pin that must be low at boot, and not claiming it
 * means never having to think about that.
 */
#define SPI_LINK_HOST SPI2_HOST
#define PIN_SCK       14
#define PIN_MOSI      13
#define PIN_CS        15

/*
 * The handshake, and the pin it inherited.
 *
 * ESP-IDF's spi_slave loses any transfer the master clocks with no transaction
 * queued -- there is no buffering in the peripheral to catch it. So the hub
 * raises this line when it has a buffer armed and drops it for the duration of
 * the transfer, and this chip does not start one until it is high. It is not an
 * optimisation: without it the link loses frames at random, and nothing on
 * either side could tell that from corruption.
 *
 * GPIO 25 was the UART TX pin. Reusing it means the wire between the two boards
 * is already run and the old lead does not become litter.
 */
#define PIN_HANDSHAKE 25

/*
 * How long to wait for the hub to arm a buffer before giving up on a packet.
 *
 * Long enough that a busy hub is never mistaken for an absent one -- it re-arms
 * between decodes, which is microseconds -- and short enough that a hub which
 * is off, reflashing or crashed does not back the queue up behind it. At 100 ms
 * the queue holds a couple of packets' grace before it starts dropping, and
 * both of those counters are reported.
 */
#define HANDSHAKE_TIMEOUT_MS 100

/*
 * Dropping here is better than stalling Bluetooth, and the seq field means the
 * hub can see it happened.
 *
 * Sized in units of the CEILING (2048) rather than of a real packet, so the
 * name understates it: a phone sends ~833 bytes at 50 packets/s, and with the
 * ring item overhead that is ~24 packets per 10 units, about 480 ms of audio.
 * Raised from 10 to 20, so ~1 s.
 *
 * WHAT IT DOES AND DOES NOT BUY, because the arithmetic is not obvious. The
 * ring does not simply fill when the hub goes away: wait_for_handshake() gives
 * up after HANDSHAKE_TIMEOUT_MS and DISCARDS the head packet, so during a stall
 * the ring drains at ~10/s while A2DP fills at ~50/s. It therefore only
 * overflows after ~600 ms of continuous hub unavailability -- 1.2 s now -- and
 * by then s_stalled has already counted several drops. Deeper helps a burst;
 * it does not help a hub that is not listening.
 *
 * RAISED BEFORE THE MEASUREMENT THAT WOULD JUSTIFY IT, deliberately and at the
 * cost of one confounded run. The hub reports "gaps N (lost M)" and this board
 * now reports which of its counters moved, but no soak has yet been read with
 * both. So if the next quiet run is taken as proof this was the fault, it is
 * not: s_dropped moving is the only thing that would say so.
 */
#define QUEUE_BYTES (SBC_LINK_MAX_PAYLOAD * 20)

static const char *TAG = "sbc_spi";

static RingbufHandle_t s_ring;
static spi_device_handle_t s_spi;
static TaskHandle_t s_tx_task;
static uint8_t *s_frame;        /* DMA-capable, SBC_LINK_FRAME_BYTES */

/*
 * Staging for the ring item, file-scope rather than on the stack. The A2DP
 * audio callback is the sole caller of sbc_link_send (the BT stack invokes it
 * serially), so a static is safe; and keeping SBC_LINK_MAX_PAYLOAD off the
 * callback's stack means raising the ceiling cannot grow the BT task frame.
 */
static struct {
    spi_link_hdr_t hdr;
    uint8_t payload[SBC_LINK_MAX_PAYLOAD];
} s_pkt;

/*
 * Atomic because it is incremented from TWO task contexts, which is easy to
 * miss: the Bluetooth stack's, through sbc_link_send() and
 * sbc_link_send_meta(), and the esp_timer task's, through sbc_link_send_vol()
 * from vol_heartbeat_cb() in avrcp_meta.c. On a dual-core part those genuinely
 * interleave, and a lost increment hands two frames the same number.
 *
 * LATENT, NOT OBSERVED, and worth saying so plainly: the collision window is a
 * few instructions against a 5 s heartbeat, order 1e-4 per hour. It is NOT the
 * cause of the ~3-per-window sequence gaps this link has always shown, and
 * anyone reading this while chasing those should keep looking. The hub survives
 * it either way -- it resyncs expect_seq, and its `lost` counter takes only
 * forward jumps -- so this is correctness, not a repair.
 */
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

    /*
     * Too big for the protocol to carry, and this used to return silently: the
     * one place in this file that loses audio without counting it.
     *
     * The ceiling is SBC_LINK_MAX_PAYLOAD (2048), sized for the codec's own
     * SBC_MAX_BITPOOL at any phone MTU, so a handset should never produce a
     * packet this refuses -- but "should" is exactly why it needs a counter
     * rather than a comment. If it ever moves, the bitpool and the ceiling are
     * out of step; silent, that would present as a hub-side gap: a link fault
     * investigated on the link, when the link was innocent and the ceiling was
     * here.
     */
    if (len > SBC_LINK_MAX_PAYLOAD) {
        s_oversize++;
        return;
    }

    /*
     * The header, sequence number included, is built HERE rather than in the
     * transmit task. Numbering at transmit time meant a packet dropped by a full
     * queue never got a number, so the hub saw a continuous sequence and
     * reported no loss -- the drops were completely invisible. Numbering at
     * enqueue makes every drop show up as a gap at the far end.
     */
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

/*
 * Block until the hub has a buffer armed, or give up on this packet.
 *
 * Almost always returns on the first line: the hub keeps two transactions
 * queued and re-arms one while it decodes the other, so the line is high except
 * during a transfer. The slow path is an interrupt and a notification rather
 * than a poll because this chip runs Bluetooth Classic, and a task spinning on
 * a GPIO is the one thing that budget cannot absorb.
 *
 * The double read around clearing the notification is not defensive padding. A
 * notification left by an earlier edge would make the wait return at once with
 * the line still low, and an edge arriving between the clear and the wait would
 * be discarded -- so read the pin again after clearing, and only then block.
 */
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

/* Report only when the count moves. Reprinting a static total every five
 * seconds makes a finished startup burst look like an ongoing fault, which is
 * worse than saying nothing. Shared by all three counters so none of them can
 * quietly acquire different reporting rules from the others. */
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

/*
 * How long to park waiting for a packet before going round anyway.
 *
 * NOT idleness, and not a poll: it is what keeps the reports at the bottom of
 * the loop running when the source has stopped. They used to sit behind a
 * portMAX_DELAY receive, so drops went unprinted for as long as the phone
 * stayed quiet -- which is exactly the window an A2DP dropout is investigated
 * in. sbc_in.c on the hub solved the same problem the same way and says so.
 */
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
            /*
             * Already framed by sbc_link_send(); pad it out to the fixed size
             * the hub has armed a buffer for. The pad carries nothing --
             * hdr.len says where the real bytes stop and the CRC covers only
             * those -- it exists so that neither end has to learn the other's
             * intentions before the transfer starts.
             */
            memcpy(s_frame, item, len);
            memset(s_frame + len, 0, SBC_LINK_FRAME_BYTES - len);
            vRingbufferReturnItem(s_ring, item);

            if (wait_for_handshake()) {
                spi_transaction_t t = {
                    .length = SBC_LINK_FRAME_BYTES * 8,
                    .tx_buffer = s_frame,
                };
                /*
                 * COUNTED, not ESP_ERROR_CHECK. This used to abort, which took
                 * the whole bridge down and silenced every speaker in the room
                 * for a transient on one transfer -- the harshest response on a
                 * link where every other failure is counted and survived
                 * (oversize, queue full, a hub that never raises the
                 * handshake). One lost frame is a hole the hub already sees as
                 * a sequence gap; a reboot is a hole plus a reconnect.
                 */
                const esp_err_t err = spi_device_transmit(s_spi, &t);
                if (err != ESP_OK) {
                    s_txerr++;
                }
            }
        }

        /*
         * Outside the `if`, so they still print when nothing is arriving. The
         * receive above times out for this reason.
         */
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
        /*
         * Pulled down, so an absent hub reads "not ready" rather than floating
         * into whatever the last transfer left on the wire. A hub that is off,
         * reflashing or crashed then shows up as a stall count instead of a
         * stream clocked into nothing.
         */
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&hs));

    const spi_bus_config_t bus = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = -1,          /* one-way link */
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
    /*
     * Checked, and the early return is load-bearing rather than tidy: the
     * handshake ISR below notifies s_tx_task by handle, so installing it after a
     * failed create would hand an interrupt a NULL task to wake. Better to leave
     * the link unarmed and say so -- a bridge that never clocks a frame is a
     * stall count on the hub, which is at least a symptom somebody can read.
     */
    if (xTaskCreate(tx_task, "sbc_tx", 3072, NULL, 10, &s_tx_task) != pdPASS) {
        s_tx_task = NULL;
        ESP_LOGE(TAG, "TASK \"sbc_tx\" FAILED TO START -- the SBC link is DOWN, "
                      "no audio will reach the hub. Heap %u free, largest %u",
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        return;
    }

    /* After the task exists, because the ISR notifies it by handle. */
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_HANDSHAKE, handshake_isr, NULL));

    ESP_LOGI(TAG, "SBC link up: SPI master at %d Hz, sck %d mosi %d cs %d, "
                  "handshake in on %d", SBC_LINK_SPI_HZ, PIN_SCK, PIN_MOSI,
             PIN_CS, PIN_HANDSHAKE);
}
