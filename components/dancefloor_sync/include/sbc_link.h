/*
 * Bridge -> hub link, carrying undecoded SBC.
 *
 * Replaces the I2S link. I2S moves samples on a shared clock, which forced the
 * hub to sample a foreign crystal -- measured at ~1.15% of frames silently
 * dropped by the I2S slave receiver. UART is self-clocking per byte and
 * tolerates percent-level clock differences by design, which is exactly the
 * property that was missing.
 *
 * It also carries a quarter of the data: ~330 kbps of SBC instead of 1.4 Mbps
 * of PCM. One signal wire instead of three.
 */
#pragma once

#include <stdint.h>

/*
 * 500 kbaud 8N1 = 50 kB/s against ~42 kB/s of measured payload (50 packets/s of
 * ~830 bytes) -- 84% utilisation, which is tighter than anyone would choose.
 *
 * It is chosen because these leads will not carry more. Measured per 5 s:
 *
 *     1000 k : ~50% of packets corrupt
 *      750 k : ~20-30 bad sync, 15-20 CRC errors
 *      500 k : 2-3 bad sync, 0-1 CRC errors
 *
 * That ceiling is a property of the wiring, not the protocol. Proper leads (or
 * SPI, which has ~30x the headroom) would lift it.
 *
 * Change here only -- both ends share this header and must agree.
 */
#define SBC_LINK_BAUD      500000

/* Two sync bytes chosen to be unlikely in SBC payload and asymmetric, so a
 * resync cannot lock onto a reversed pair. */
#define SBC_LINK_SYNC0     0xA5
#define SBC_LINK_SYNC1     0x5A

/* One A2DP packet can hold several SBC frames. */
#define SBC_LINK_MAX_PAYLOAD 1024

typedef struct __attribute__((packed)) {
    uint8_t  sync0;
    uint8_t  sync1;
    uint16_t len;         /* payload bytes following the header */
    uint32_t seq;         /* detects loss without needing a timer */
    uint8_t  checksum;    /* XOR of payload; cheap, and enough to spot corruption */
} sbc_link_hdr_t;

static inline uint8_t sbc_link_checksum(const uint8_t *p, uint16_t n)
{
    uint8_t c = 0;
    for (uint16_t i = 0; i < n; i++) {
        c ^= p[i];
    }
    return c;
}
