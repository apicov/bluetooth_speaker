/**
 * @file sbc_spi.c
 * @brief SPI master half of the SBC link: queue here, frame and clock it out
 *        there. Declared in sbc_spi.h.
 *
 * Everything the Bluetooth stack hands over is copied into a ring buffer and
 * left; one task drains that ring, pads each item to the fixed frame size and
 * clocks it to the hub once the handshake says a buffer is armed. Nothing on
 * the audio path waits, and each way a packet can be lost has a counter that is
 * printed when it moves.
 */
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

/**
 * @name The bus and the three signals on it
 *
 * Nothing else on this chip uses a SPI bus -- it runs Bluetooth and this link
 * and nothing else -- so the controller is free to choose here, unlike on the
 * hub where the LED strip already holds one.
 *
 * The signals sit on this controller's IOMUX pins rather than anywhere
 * convenient. Routing SPI through the GPIO matrix adds delay that only begins
 * to matter as the clock rises, and the clock is the knob this link is tuned
 * with, so paying nothing for it costs nothing now and cannot bite later.
 *
 * There is no MISO, because the link is one-way. That is worth a pin in itself:
 * this controller's MISO is the MTDI strapping pin, which must be low at boot,
 * and not claiming it means never having to think about that.
 * @{
 */
#define SPI_LINK_HOST SPI2_HOST   /**< The controller carrying the link. */
#define PIN_SCK       14          /**< Clock, driven by this chip. */
#define PIN_MOSI      13          /**< Data out to the hub. */
#define PIN_CS        15          /**< Chip select, and the frame boundary. */
/** @} */

/**
 * @brief The hub's "a buffer is armed, you may clock a frame" line, an input
 *        here.
 *
 * ESP-IDF's spi_slave loses any transfer the master clocks with no transaction
 * queued -- the peripheral has nowhere to put it -- so the hub raises this line
 * while a buffer is armed and drops it for the duration of a transfer, and this
 * chip does not start one until it is high. It is not an optimisation: without
 * it the link loses frames at random, and neither end could tell that from
 * corruption.
 */
#define PIN_HANDSHAKE 25

/**
 * @brief How long to wait for the hub to arm a buffer before giving up on a
 *        packet.
 *
 * Long enough that a busy hub is never mistaken for an absent one -- it re-arms
 * between decodes, which is far shorter than this -- and short
 * enough that a hub which is off, reflashing or crashed cannot back the queue
 * up behind it. Both outcomes are counted: giving up increments #s_stalled,
 * and a queue that fills behind it increments #s_dropped.
 */
#define HANDSHAKE_TIMEOUT_MS 100

/**
 * @brief Ring capacity, in bytes.
 *
 * Dropping here is better than stalling Bluetooth, and the sequence number
 * means the hub can see that it happened.
 *
 * Sized in units of the payload CEILING rather than of a real packet, so the
 * count understates the depth: an ordinary A2DP payload is a fraction of
 * SBC_LINK_MAX_PAYLOAD, so the ring holds several times that many real
 * packets.
 *
 * What that does and does not buy is worth being explicit about, because the
 * arithmetic is not obvious. The ring does not simply fill when the hub goes
 * away: wait_for_handshake() gives up after #HANDSHAKE_TIMEOUT_MS and discards
 * the head packet, so during a stall the ring drains at a fixed slow rate while
 * A2DP keeps filling it far faster. It therefore only overflows after a
 * sustained period of hub unavailability, and by then #s_stalled has already
 * counted the drops. Depth helps a burst; it does not help a hub that is not
 * listening.
 */
#define QUEUE_BYTES (SBC_LINK_MAX_PAYLOAD * 20)

/** @brief ESP_LOG tag for the link. */
static const char *TAG = "sbc_spi";

/** @brief The queue between the Bluetooth callbacks and tx_task(). */
static RingbufHandle_t s_ring;
/** @brief The SPI device this link transmits on. */
static spi_device_handle_t s_spi;
/** @brief tx_task(), by handle, because handshake_isr() notifies it. */
static TaskHandle_t s_tx_task;
/** @brief DMA-capable transmit buffer, SBC_LINK_FRAME_BYTES long. */
static uint8_t *s_frame;

/**
 * @brief Staging for one audio ring item, at file scope rather than on the
 *        caller's stack.
 *
 * The A2DP audio callback is the only caller of sbc_link_send(), and the
 * Bluetooth stack invokes it serially, so one shared buffer is safe. Keeping
 * SBC_LINK_MAX_PAYLOAD off that callback's stack also means raising the ceiling
 * cannot grow the Bluetooth task's frame.
 */
static struct {
    spi_link_hdr_t hdr;                       /**< Filled in by sbc_link_send(). */
    uint8_t payload[SBC_LINK_MAX_PAYLOAD];    /**< Only hdr.len bytes are real. */
} s_pkt;

/**
 * @brief Frame sequence number, shared by all three senders.
 *
 * Atomic because it is incremented from two task contexts, which is easy to
 * miss: the Bluetooth stack's, through sbc_link_send() and
 * sbc_link_send_meta(), and the esp_timer task's, through sbc_link_send_vol()
 * on the volume heartbeat in avrcp_meta.c. On a dual-core part those genuinely
 * interleave, and a lost increment would hand two frames the same number.
 *
 * The collision window is a few instructions against a heartbeat measured in
 * seconds, so this is correctness rather than the repair of anything the link
 * is known to suffer from. The hub survives a repeat either way -- it resyncs
 * its expected sequence, and its loss counter only takes forward jumps.
 */
static _Atomic uint32_t s_seq;
/** @brief Packets the ring had no room for. */
static uint32_t s_dropped;
/** @brief Payloads larger than the link can carry; see sbc_link_send(). */
static uint32_t s_oversize;
/** @brief Packets abandoned because the handshake never came. */
static uint32_t s_stalled;
/** @brief Transfers the SPI driver refused. */
static uint32_t s_txerr;

/* Declared in sbc_spi.h, like the two below it. */
void sbc_link_send(const uint8_t *sbc, uint16_t len)
{
    if (!s_ring || len == 0) {
        return;
    }

    /*
     * Too big for the protocol to carry, and counted rather than returned
     * silently. The ceiling is sized for the codec's own maximum bitpool at any
     * phone MTU, so a handset should never produce a packet this refuses -- but
     * "should" is exactly why it wants a counter and not a comment. If it ever
     * moves, the bitpool and the ceiling are out of step; silently, that would
     * present as a gap at the hub, which is a link fault investigated on the
     * link when the link was innocent.
     */
    if (len > SBC_LINK_MAX_PAYLOAD) {
        s_oversize++;
        return;
    }

    /*
     * The header, sequence number included, is built HERE rather than in the
     * transmit task. Numbering at transmit time means a packet dropped by a
     * full queue never gets a number, so the hub sees a continuous sequence and
     * reports no loss -- the drops become invisible. Numbering at enqueue makes
     * every drop show up as a gap at the far end.
     */
    s_pkt.hdr.kind = LINK_KIND_SBC;
    s_pkt.hdr.rsv = 0;
    s_pkt.hdr.rsv2 = 0;
    s_pkt.hdr.len = len;
    s_pkt.hdr.seq = s_seq++;
    s_pkt.hdr.crc = sbc_link_crc16(&s_pkt.hdr, sbc, len);
    memcpy(s_pkt.payload, sbc, len);

    /* Zero timeout: the audio callback returns either way, and a full ring is a
     * counted drop rather than back-pressure onto Bluetooth. */
    if (xRingbufferSend(s_ring, &s_pkt, sizeof(spi_link_hdr_t) + len, 0) != pdTRUE) {
        s_dropped++;
    }
}

void sbc_link_send_meta(const link_meta_t *meta)
{
    if (!s_ring) {
        return;
    }
    /* On the stack, unlike the audio staging buffer: this is small, and the
     * callers are rare enough that sharing one would only add a rule. */
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

    /* Payload before the CRC, which covers it. */
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

/**
 * @brief Handshake rising edge: wake the transmit task.
 *
 * Everything else the edge means is decided in wait_for_handshake(); this only
 * ends the wait.
 *
 * @param arg  Unused.
 */
static void IRAM_ATTR handshake_isr(void *arg)
{
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_tx_task, &woken);
    if (woken) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief Block until the hub has a buffer armed, or give up on this packet.
 *
 * Almost always returns on the first line: the hub keeps a spare transaction
 * queued and re-arms one while it decodes the other, so the line is high except
 * during a transfer. The slow path is an interrupt and a notification rather
 * than a poll because this chip runs Bluetooth Classic, and a task spinning on
 * a GPIO is the one thing that budget cannot absorb.
 *
 * The second read, after clearing the notification, is not defensive padding. A
 * notification left by an earlier edge would make the wait return at once with
 * the line still low, and an edge arriving between the clear and the wait would
 * be discarded -- so read the pin again after clearing, and only then block.
 *
 * @return true if the hub is ready and the transfer may start; false if it did
 *         not answer within #HANDSHAKE_TIMEOUT_MS, in which case #s_stalled has
 *         been incremented and the packet is the caller's to abandon.
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

/**
 * @brief Log a counter, but only when it has moved and not too often.
 *
 * Reprinting a static total on every pass makes a finished startup burst look
 * like an ongoing fault, which is worse than saying nothing. Shared by all four
 * counters so that none of them can quietly acquire different reporting rules
 * from the others.
 *
 * @param what     What the counter counts, for the log line.
 * @param count    Its value now.
 * @param last     The value at the previous report; updated when one is made.
 * @param last_at  When that report was made, in esp_timer microseconds; updated
 *                 with it, and what rate-limits the printing.
 */
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

/**
 * @brief How long tx_task() parks waiting for a packet before going round
 *        anyway.
 *
 * Not idleness and not a poll: it is what keeps the reports at the bottom of
 * the loop running when the source has stopped. Behind an indefinite receive,
 * drops would go unprinted for as long as the phone stayed quiet -- which is
 * exactly the window an A2DP dropout is investigated in.
 */
#define TX_IDLE_TICK_MS 500

/**
 * @brief Drain the ring: pad each item to a full frame and clock it to the hub.
 * @param arg  Unused.
 */
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
             * Already framed by the sender; this only pads it out to the fixed
             * size the hub has armed a buffer for. The padding carries nothing
             * -- hdr.len says where the real bytes stop and the CRC covers only
             * those -- it exists so that neither end has to learn the other's
             * intentions before the transfer starts.
             *
             * Copied out and returned to the ring before the wait below, so a
             * hub that is slow to answer does not also hold the queue's head.
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
                 * Counted, not ESP_ERROR_CHECK. Aborting here would take the
                 * whole bridge down and silence every speaker in the room over
                 * a transient on one transfer -- the harshest possible response
                 * on a link where every other failure is counted and survived.
                 * One lost frame is a hole the hub already sees as a sequence
                 * gap; a reboot is that hole plus a reconnect.
                 */
                const esp_err_t err = spi_device_transmit(s_spi, &t);
                if (err != ESP_OK) {
                    s_txerr++;
                }
            }
        }

        /* Outside the `if`, so they still print when nothing is arriving. The
         * receive above times out for this reason. */
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
        /* One at a time: the task builds the next frame only after the
         * current transfer has returned. */
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
     * handshake ISR below notifies s_tx_task by handle, so installing it after
     * a failed create would hand an interrupt a NULL task to wake. Better to
     * leave the link unarmed and say so -- a bridge that never clocks a frame
     * is a stall count on the hub, which is at least a symptom somebody can
     * read.
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
