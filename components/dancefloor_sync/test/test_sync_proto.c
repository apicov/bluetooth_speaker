/*
 * Host-side tests for the clock offset estimator.
 *   cc -std=c11 -Wall -Wextra -I../main test_sync_proto.c ../main/sync_proto.c -o test && ./test
 *
 * Simulation model, where master_clock = local_clock + TRUE_OFFSET:
 *   t1 = satellite send        (local clock)
 *   t2 = t1 + up   + offset    (master clock)
 *   t3 = t2 + service          (master clock)
 *   t4 = t3 - offset + down    (local clock)
 * which makes the estimator's error exactly (up - down) / 2.
 */
#include "sync_proto.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

/* An independent CRC-16/CCITT-FALSE, so the link's CRC is checked against
 * another implementation rather than against itself. Pinned on the published
 * "123456789" -> 0x29B1 vector before it certifies anything. */
static uint16_t ref_crc16(const uint8_t *p, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)p[i] << 8;
        for (int k = 0; k < 8; k++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static int failures = 0;

static void check(const char *name, bool cond, const char *detail)
{
    printf("%-46s %s%s%s\n", name, cond ? "PASS" : "FAIL",
           detail ? "  " : "", detail ? detail : "");
    if (!cond) failures++;
}

/* Deterministic PRNG - a flaky test here would be worse than no test. */
static uint32_t rng_state = 0x1234567u;
static int32_t rnd(int32_t lo, int32_t hi)
{
    rng_state = rng_state * 1664525u + 1013904223u;
    return lo + (int32_t)((rng_state >> 8) % (uint32_t)(hi - lo + 1));
}

static void probe(sync_est_t *e, int64_t t1, int64_t offset, int64_t up,
                  int64_t service, int64_t down)
{
    int64_t t2 = t1 + up + offset;
    int64_t t3 = t2 + service;
    int64_t t4 = t3 - offset + down;
    sync_est_add(e, t1, t2, t3, t4);
}

int main(void)
{
    const int64_t TRUE_OFFSET = 1234567;  /* us */
    int64_t est;

    /* 1. Not enough samples yet. */
    {
        sync_est_t e; sync_est_init(&e);
        probe(&e, 1000, TRUE_OFFSET, 500, 100, 500);
        probe(&e, 2000, TRUE_OFFSET, 500, 100, 500);
        check("rejects estimate below SYNC_MIN_SAMPLES", !sync_est_offset(&e, &est), NULL);
    }

    /* 2. Symmetric, noiseless: must be exact. */
    {
        sync_est_t e; sync_est_init(&e);
        for (int i = 0; i < 5; i++)
            probe(&e, 1000 * (i + 1), TRUE_OFFSET, 500, 100, 500);
        bool ok = sync_est_offset(&e, &est);
        char d[64]; snprintf(d, sizeof d, "est=%" PRId64, est);
        check("symmetric noiseless path is exact", ok && est == TRUE_OFFSET, d);
    }

    /* 3. Symmetric with jitter: well inside the 1 ms budget. */
    {
        sync_est_t e; sync_est_init(&e);
        for (int i = 0; i < SYNC_WINDOW; i++)
            probe(&e, 1000 * (i + 1), TRUE_OFFSET, rnd(400, 900), rnd(50, 150), rnd(400, 900));
        sync_est_offset(&e, &est);
        int64_t err = llabs(est - TRUE_OFFSET);
        char d[64]; snprintf(d, sizeof d, "err=%" PRId64 " us", err);
        check("jittered symmetric path within 1 ms", err < 1000, d);
    }

    /* 4. One WiFi retry inflating a single probe by 80 ms. A mean would be
     *    dragged ~8 ms off; minimum-delay selection never even considers it. */
    {
        sync_est_t e; sync_est_init(&e);
        for (int i = 0; i < 9; i++)
            probe(&e, 1000 * (i + 1), TRUE_OFFSET, 500, 100, 500);
        probe(&e, 10000, TRUE_OFFSET, 80000, 100, 500);   /* the outlier */
        sync_est_offset(&e, &est);
        int64_t err = llabs(est - TRUE_OFFSET);
        char d[64]; snprintf(d, sizeof d, "err=%" PRId64 " us", err);
        check("min-RTT ignores an 80 ms retry outlier", err < 1000, d);
    }

    /* 5. Sustained asymmetry is the error floor, not something the median fixes.
     *    2 ms up against 200 us down must show up as ~900 us of error. */
    {
        sync_est_t e; sync_est_init(&e);
        for (int i = 0; i < SYNC_WINDOW; i++)
            probe(&e, 1000 * (i + 1), TRUE_OFFSET, 2000, 100, 200);
        sync_est_offset(&e, &est);
        int64_t err = est - TRUE_OFFSET;
        char d[64]; snprintf(d, sizeof d, "err=%" PRId64 " us (expected ~900)", err);
        check("asymmetric path errs by half the asymmetry", err == 900, d);
    }

    /* 6. Ring buffer must retain only the most recent SYNC_WINDOW probes, so a
     *    clock that steps is eventually forgotten rather than averaged forever. */
    {
        sync_est_t e; sync_est_init(&e);
        for (int i = 0; i < SYNC_WINDOW; i++)
            probe(&e, 1000 * (i + 1), TRUE_OFFSET, 500, 100, 500);
        const int64_t NEW_OFFSET = TRUE_OFFSET + 50000;
        for (int i = 0; i < SYNC_WINDOW; i++)
            probe(&e, 100000 + 1000 * (i + 1), NEW_OFFSET, 500, 100, 500);
        sync_est_offset(&e, &est);
        char d[64]; snprintf(d, sizeof d, "est=%" PRId64, est);
        check("window forgets superseded offsets", est == NEW_OFFSET, d);
    }

    /* 7. The reason for selecting on delay rather than taking a median.
     *    Nine slow, badly asymmetric probes (5 ms up, 0.5 ms down -> +2250 us of
     *    error each) and one fast symmetric probe. A median would return roughly
     *    2250 us of error; picking the lowest-RTT sample returns the clean one.
     *    This mirrors the measured 5-14 ms RTT spread on a real SoftAP link. */
    {
        sync_est_t e; sync_est_init(&e);
        for (int i = 0; i < 9; i++)
            probe(&e, 1000 * (i + 1), TRUE_OFFSET, 5000, 100, 500);
        probe(&e, 10000, TRUE_OFFSET, 600, 100, 600);     /* the clean one */
        sync_est_offset(&e, &est);
        int64_t err = llabs(est - TRUE_OFFSET);
        char d[72]; snprintf(d, sizeof d, "err=%" PRId64 " us (median would be ~2250)", err);
        check("min-RTT picks the symmetric probe", err < 100, d);
    }

    /*
     * 8. The master rebooting is not drift.
     *
     * Its clock then counts from a new origin, and every sample already in the
     * window is wrong by its entire previous uptime. The window spans 2.5 s and
     * minimum-RTT selection may pick any sample in it, so without this a
     * satellite anchors on an offset that is an hour stale, schedules playback
     * for a time that never arrives, and hands the phase servo a number no
     * correction can fix. Observed on hardware as a satellite reporting -699
     * seconds of phase and then aborting on the sample rate that produced.
     */
    {
        sync_est_t e; sync_est_init(&e);
        for (int i = 0; i < SYNC_WINDOW; i++)
            probe(&e, 1000 * (i + 1), TRUE_OFFSET, 1000, 100, 1000);
        check("a full window is settled", sync_est_settled(&e), NULL);

        /* Master reboots: its clock restarts, so the offset drops by its uptime. */
        const int64_t after = TRUE_OFFSET - 3595000000LL;
        probe(&e, 20000, after, 1000, 100, 1000);

        check("a clock-origin step is not trusted as an estimate",
              !sync_est_settled(&e), "window discarded, playback holds");

        /* One surviving sample is not an estimate, and the caller is told so
         * rather than handed it. */
        check("no estimate is offered from the remains",
              !sync_est_offset(&e, &est), "count fell below SYNC_MIN_SAMPLES");

        /* Once enough probes on the new clock have landed, it is the new offset
         * that comes out. The stale one is gone entirely, not merely outvoted. */
        for (int i = 0; i < SYNC_MIN_SAMPLES; i++)
            probe(&e, 21000 + 1000 * i, after, 1000, 100, 1000);
        char d[80];
        snprintf(d, sizeof d, "est=%" PRId64 ", stale would be %" PRId64,
                 sync_est_offset(&e, &est) ? est : 0, TRUE_OFFSET);
        check("the stale window cannot be selected from", llabs(est - after) < 100, d);

        /* And it settles again on the new clock rather than wedging. */
        for (int i = 0; i < SYNC_WINDOW; i++)
            probe(&e, 30000 + 1000 * i, after, 1000, 100, 1000);
        sync_est_offset(&e, &est);
        check("it re-settles on the new origin",
              sync_est_settled(&e) && llabs(est - after) < 100, NULL);
    }

    /*
     * 9. A single corrupt probe must cost two samples, not the whole session:
     *    the garbage becomes the baseline, the next good sample steps away from
     *    it and resets again, seeded correctly that time.
     */
    {
        sync_est_t e; sync_est_init(&e);
        for (int i = 0; i < 5; i++)
            probe(&e, 1000 * (i + 1), TRUE_OFFSET, 1000, 100, 1000);
        probe(&e, 6000, TRUE_OFFSET + 9000000000LL, 1000, 100, 1000);   /* garbage */
        for (int i = 0; i < SYNC_WINDOW; i++)
            probe(&e, 10000 + 1000 * i, TRUE_OFFSET, 1000, 100, 1000);
        sync_est_offset(&e, &est);
        char d[64]; snprintf(d, sizeof d, "est=%" PRId64, est);
        check("one corrupt probe does not wedge the estimator",
              sync_est_settled(&e) && llabs(est - TRUE_OFFSET) < 100, d);
    }

    /* 10. Round-trip conversion is what actually schedules playback. */
    {
        int64_t master_now = 9000000;
        int64_t local = sync_to_local(master_now, TRUE_OFFSET);
        check("sync_to_local inverts the offset", local + TRUE_OFFSET == master_now, NULL);
    }

    /* ------------------------------------------------ the splice phase filter */

    /* 11. Too few readings is not an estimate. The caller splices on nothing
     *     rather than on one sample taken just after a re-anchor. */
    {
        sync_phase_hist_t h; sync_phase_reset(&h);
        int32_t med;
        bool early = false;
        for (int i = 0; i < SYNC_PHASE_MIN; i++) {
            early = early || sync_phase_median(&h, &med);
            sync_phase_push(&h, 1000);
        }
        check("no median below SYNC_PHASE_MIN", !early, NULL);
        check("a median appears at SYNC_PHASE_MIN", sync_phase_median(&h, &med), NULL);
    }

    /* 12. A steady reading must survive the filter unchanged. If the phase
     *     really is +8 ms the splice must still correct +8 ms. */
    {
        sync_phase_hist_t h; sync_phase_reset(&h);
        for (int i = 0; i < SYNC_PHASE_HIST; i++) sync_phase_push(&h, 8231);
        int32_t med = 0; bool ok = sync_phase_median(&h, &med);
        char d[64]; snprintf(d, sizeof d, "med=%" PRId32, med);
        check("constant input passes through unchanged", ok && med == 8231, d);
    }

    /*
     * 13. The whole reason this exists.
     *
     * Eight readings agreeing on +2 ms and one at +50 ms -- the shape the hub
     * actually produces, where two reads a millisecond apart differed by 15.7 ms.
     * The mean moves by 5.3 ms and the splice cuts that much real audio for an
     * error that is not there; the median does not move at all.
     *
     * The comparison is computed here rather than asserted in a comment, so the
     * test is what documents the choice.
     */
    {
        static const int32_t reading[SYNC_PHASE_HIST] =
            { 2000, 2100, 1900, 2000, 50000, 2050, 1950, 2000, 2100 };
        sync_phase_hist_t h; sync_phase_reset(&h);
        int64_t sum = 0;
        for (int i = 0; i < SYNC_PHASE_HIST; i++) {
            sync_phase_push(&h, reading[i]);
            sum += reading[i];
        }
        int32_t med = 0;
        sync_phase_median(&h, &med);
        const int32_t mean = (int32_t)(sum / SYNC_PHASE_HIST);
        char d[96];
        snprintf(d, sizeof d, "med=%" PRId32 ", mean=%" PRId32 " (%+" PRId32 " us of phantom phase)",
                 med, mean, mean - med);
        check("one outlier in nine cannot move the median",
              med == 2000 && mean - med > 5000, d);
    }

    /*
     * 14. It must forget. The history spans ~180 ms of playback and a splice or
     *     a re-anchor resets it, but nothing else bounds how long a unit runs --
     *     a reading from an earlier position must not still be voting.
     */
    {
        sync_phase_hist_t h; sync_phase_reset(&h);
        for (int i = 0; i < SYNC_PHASE_HIST; i++) sync_phase_push(&h, -30000);
        for (int i = 0; i < SYNC_PHASE_HIST; i++) sync_phase_push(&h, 4000);
        int32_t med = 0; sync_phase_median(&h, &med);
        char d[64]; snprintf(d, sizeof d, "med=%" PRId32, med);
        check("the ring keeps only the newest window", med == 4000, d);
    }

    /*
     * 15. A partly-filled history must not count the slots it has not written.
     *
     * Five readings of +9 ms with four zeroed slots left over: a median taken
     * over all nine slots returns 0 and the boundary passes with no correction
     * at all. Run this against a version that sorts SYNC_PHASE_HIST instead of
     * count -- it fails, which is the point of writing it.
     */
    {
        sync_phase_hist_t h; sync_phase_reset(&h);
        for (int i = 0; i < 5; i++) sync_phase_push(&h, 9000);
        int32_t med = 0; bool ok = sync_phase_median(&h, &med);
        char d[72]; snprintf(d, sizeof d, "med=%" PRId32 " (unwritten slots would give 0)", med);
        check("unwritten slots do not vote", ok && med == 9000, d);
    }

    /* 16. Reset must actually clear, not just rewind -- a splice calls it and
     *     the next boundary must not see anything from before the correction. */
    {
        sync_phase_hist_t h; sync_phase_reset(&h);
        for (int i = 0; i < SYNC_PHASE_HIST; i++) sync_phase_push(&h, 12000);
        sync_phase_reset(&h);
        int32_t med;
        check("reset drops the whole history", !sync_phase_median(&h, &med), NULL);
    }

    /*
     * 17. The SPI link's frame must satisfy the SPI slave's DMA rule in the
     *     TYPE, not at the allocation. The driver needs a 4-byte-aligned buffer
     *     whose length is a multiple of 4, and sbc_link.h claims the 12-byte
     *     header is sized to deliver that. Nothing else checks the claim, and a
     *     field added to the header later would break it silently on hardware.
     */
    {
        char d[80];
        snprintf(d, sizeof d, "hdr=%zu frame=%zu", sizeof(spi_link_hdr_t),
                 (size_t)SBC_LINK_FRAME_BYTES);
        check("the SPI frame is a multiple of 4 bytes",
              sizeof(spi_link_hdr_t) == 12 && SBC_LINK_FRAME_BYTES % 4 == 0, d);
    }

    /*
     * The WiFi hop must accept at least what the SPI hop delivers. The two
     * ceilings are independent literals that have to move together; if the WiFi
     * one fell below the SPI one, the hub forwarder would refuse what the link
     * just carried -- a silent drop on a different layer than the one above.
     */
    {
        char d[80];
        snprintf(d, sizeof d, "spi=%zu wifi=%zu",
                 (size_t)SBC_LINK_MAX_PAYLOAD, (size_t)AUDIO_MAX_PAYLOAD);
        check("the WiFi ceiling covers the SPI ceiling",
              AUDIO_MAX_PAYLOAD >= SBC_LINK_MAX_PAYLOAD, d);
    }


    /*
     * THE APPLIED THRESHOLD MUST NOT BIND AT REAL PAYLOAD SIZES.
     *
     * This is the test that was missing. Everything above checks the FEC
     * arithmetic and all of it passed while the hub was splitting every payload
     * in two, doubling the audio packet rate, discarding 15% of its own audio at
     * the socket and slewing the timeline at twice the rate the satellites could
     * follow. Not one assertion here noticed, because none of them asked the
     * question that mattered: how many datagrams does one A2DP payload become?
     *
     * A phone at bitpool 53, 44.1 kHz joint stereo sends ~119-byte SBC frames,
     * ~7 to a payload, ~825 bytes -- and up to 1024 has been seen. If the
     * threshold sbc_in.c cuts on is below that, the rate doubles and both
     * TIMELINE_SLEW_US and the TX buffer pool are silently resized with it.
     */
    {
        char d[96];
        snprintf(d, sizeof d, "mtu-cap=%zu", (size_t)AUDIO_TX_PAYLOAD_MTU_MAX);
        check("the applied cap is header-plus-payload against the MTU",
              AUDIO_TX_PAYLOAD_MTU_MAX == 1446 &&
              AUDIO_MSG_BYTES(AUDIO_TX_PAYLOAD_MTU_MAX) == AUDIO_UDP_MTU, d);

        /* One datagram per payload at every size this source produces. */
        static const size_t seen[] = { 512, 700, 825, 1024, 1200, 1446 };
        bool one_each = true;
        for (size_t i = 0; i < sizeof seen / sizeof seen[0]; i++) {
            if (seen[i] > AUDIO_TX_PAYLOAD_MTU_MAX) {
                one_each = false;
            }
        }
        snprintf(d, sizeof d, "up to %zu B stays one datagram",
                 (size_t)AUDIO_TX_PAYLOAD_MTU_MAX);
        check("a real A2DP payload is not split", one_each, d);
    }

    /*
     * 18. The CRC is CRC-16/CCITT-FALSE, checked against an independent
     *     implementation rather than against a number this file copied from
     *     the one under test.
     *
     * ref_crc16() is pinned first on the published check value for "123456789",
     * 0x29B1, so a wrong reference cannot certify a wrong production function.
     */
    {
        check("the reference CRC matches the published vector",
              ref_crc16((const uint8_t *)"123456789", 9) == 0x29B1, NULL);

        spi_link_hdr_t h = { .kind = LINK_KIND_SBC, .len = 9, .seq = 12345,
                             .crc = 0 };
        const uint8_t payload[9] = "123456789";
        uint8_t flat[sizeof h + 9];
        memcpy(flat, &h, sizeof h);
        memcpy(flat + sizeof h, payload, 9);

        char d[64];
        const uint16_t got = sbc_link_crc16(&h, payload, 9);
        snprintf(d, sizeof d, "got=0x%04X ref=0x%04X", got,
                 ref_crc16(flat, sizeof flat));
        check("the frame CRC covers header then payload",
              got == ref_crc16(flat, sizeof flat), d);
    }

    /*
     * 19. Whatever is already in the crc field must not change the answer.
     *
     * That convention is what lets the sender compute over a header it has
     * filled in and the receiver over a header carrying the sender's value,
     * neither one copying the struct to blank a field. If it ever stops
     * holding, every frame fails its check on hardware and nothing here says
     * why.
     */
    {
        const uint8_t payload[32] = { 1, 2, 3 };
        spi_link_hdr_t a = { .kind = LINK_KIND_SBC, .len = 32, .seq = 7, .crc = 0 };
        spi_link_hdr_t b = a;
        b.crc = 0xBEEF;
        check("the crc field does not feed itself",
              sbc_link_crc16(&a, payload, 32) == sbc_link_crc16(&b, payload, 32), NULL);
    }

    /*
     * 20. The error the XOR byte could not see.
     *
     * Flip the same bit in two payload bytes and the XOR is unchanged -- it is
     * a parity per bit position, so any even number of flips in one column is
     * invisible to it. The first half passes, which is the point of writing it:
     * the UART link would have accepted this frame, and on a wire running 20x
     * faster there is no resync scan behind it to notice.
     *
     * The XOR is reproduced here rather than called. sbc_link_checksum() went
     * with the UART declarations when the classic hub was retired, and this test
     * is the reason the byte is still worth describing -- it says what the CRC
     * was chosen over, so it has to keep computing the thing it argues against.
     */
    {
        uint8_t good[64], bad[64];
        for (int i = 0; i < 64; i++) good[i] = (uint8_t)(i * 7 + 3);
        memcpy(bad, good, sizeof bad);
        bad[0] ^= 0x01;
        bad[1] ^= 0x01;

        uint8_t xor_good = 0, xor_bad = 0;
        for (int i = 0; i < 64; i++) {
            xor_good ^= good[i];
            xor_bad  ^= bad[i];
        }

        spi_link_hdr_t h = { .kind = LINK_KIND_SBC, .len = 64, .seq = 99, .crc = 0 };
        check("the XOR the link used to carry misses this",
              xor_good == xor_bad, NULL);
        check("the CRC catches it",
              sbc_link_crc16(&h, good, 64) != sbc_link_crc16(&h, bad, 64), NULL);
    }

    /*
     * 21. The log/health messages are the collector's wire format, unpacked in
     *     Python against this packed C layout. A field added, removed or
     *     reordered here shifts every offset after it silently and the
     *     collector reads garbage, so the sizes are pinned the way the SPI
     *     frame is above. log_msg_t is variable-length (only the first msg_len
     *     bytes of its 192-byte array go on the wire); health_msg_t and
     *     log_sub_msg_t are fixed.
     */
    {
        char d[96];
        snprintf(d, sizeof d, "log=%zu health=%zu sub=%zu",
                 sizeof(log_msg_t), sizeof(health_msg_t), sizeof(log_sub_msg_t));
        check("the log/health message sizes are pinned",
              sizeof(log_msg_t) == 222 && sizeof(health_msg_t) == 108 &&
              sizeof(log_sub_msg_t) == 5, d);
    }

    /*
     * 22. LOG_MSG_BYTES is bytes-on-the-wire for a given message length -- the
     *     fixed header plus the payload, never the whole 222-byte ceiling -- and
     *     the ceiling itself must stay clear of the MTU so a longest line never
     *     fragments. The receiver sizes its buffer off this relationship.
     */
    {
        char d[96];
        const size_t hdr = sizeof(log_msg_t) - LOG_MSG_MAX;
        snprintf(d, sizeof d, "hdr=%zu full=%zu mtu=1500",
                 hdr, LOG_MSG_BYTES(LOG_MSG_MAX));
        check("LOG_MSG_BYTES is header + payload and clears the MTU",
              LOG_MSG_BYTES(0) == hdr &&
              LOG_MSG_BYTES(LOG_MSG_MAX) == sizeof(log_msg_t) &&
              LOG_MSG_BYTES(LOG_MSG_MAX) <= 1500, d);
    }

    printf("\n%s\n", failures ? "FAILURES PRESENT" : "all tests passed");
    return failures != 0;
}
