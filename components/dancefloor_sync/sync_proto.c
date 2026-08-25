/**
 * @file sync_proto.c
 * @brief The arithmetic behind sync_proto.h: the clock estimator, the phase
 *        median, the link CRC and the XOR parity.
 *
 * Everything the two firmwares must compute IDENTICALLY, in one place, so they
 * cannot compute it differently. sync_proto.h owns every contract; what is
 * here is the reasoning that belongs to the implementation. Free of ESP-IDF,
 * so test_sync_proto.c drives all of it under plain gcc.
 */
#include "sync_proto.h"

#include <stddef.h>
#include <string.h>

void sync_est_init(sync_est_t *e)
{
    memset(e, 0, sizeof(*e));
}

void sync_est_add(sync_est_t *e, int64_t t1, int64_t t2, int64_t t3, int64_t t4)
{
    const int64_t offset = ((t2 - t1) + (t3 - t4)) / 2;

    /*
     * Has the master's clock changed origin?
     *
     * Two consecutive probes are a fraction of a second apart. Drift moves the
     * offset by a few microseconds in that time and path asymmetry by a few
     * milliseconds, so a step of a whole second is neither: the master
     * rebooted or was reflashed, and every sample already in the window
     * describes a clock that no longer exists.
     *
     * Keeping them is not a small error. Minimum-RTT selection is free to pick
     * any sample in the window, and a pre-reboot sample is wrong by the
     * master's entire previous uptime. A satellite that anchors on that plays
     * at a time that never arrives, and the phase servo downstream is handed a
     * number no correction can fix.
     *
     * Discarding the window drops count below SYNC_MIN_SAMPLES, so
     * sync_est_settled() goes false and playback holds until a clean estimate
     * exists -- which is what that mechanism is for. It costs one window.
     *
     * The new sample becomes the baseline, so a single corrupt probe costs two
     * windows rather than wedging: the next good sample steps away from the
     * corrupt one and resets again, seeded correctly that time.
     */
    if (e->count > 0) {
        const int last = (e->next + SYNC_WINDOW - 1) % SYNC_WINDOW;
        const int64_t step = offset - e->offset[last];
        if (step > SYNC_STEP_US || step < -SYNC_STEP_US) {
            memset(e, 0, sizeof(*e));
        }
    }

    e->offset[e->next] = offset;
    e->delay[e->next]  = (t4 - t1) - (t3 - t2);
    e->next = (e->next + 1) % SYNC_WINDOW;
    if (e->count < SYNC_WINDOW) {
        e->count++;
    }
}

bool sync_est_offset(const sync_est_t *e, int64_t *offset_out)
{
    if (e->count < SYNC_MIN_SAMPLES) {
        return false;
    }

    /* Walk oldest to newest so that '<=' lets a newer sample win an exact tie.
     * When the buffer is full the oldest entry is the one about to be
     * overwritten. */
    int start = (e->count == SYNC_WINDOW) ? e->next : 0;
    int best = -1;

    for (int n = 0; n < e->count; n++) {
        int i = (start + n) % SYNC_WINDOW;
        if (best < 0 || e->delay[i] <= e->delay[best]) {
            best = i;
        }
    }

    *offset_out = e->offset[best];
    return true;
}

void sync_phase_reset(sync_phase_hist_t *h)
{
    memset(h, 0, sizeof(*h));
}

void sync_phase_push(sync_phase_hist_t *h, int32_t us)
{
    h->v[h->next] = us;
    h->next = (uint8_t)((h->next + 1) % SYNC_PHASE_HIST);
    if (h->count < SYNC_PHASE_HIST) {
        h->count++;
    }
}

bool sync_phase_median(const sync_phase_hist_t *h, int32_t *out)
{
    if (h->count < SYNC_PHASE_MIN) {
        return false;
    }

    /*
     * Insertion sort of at most SYNC_PHASE_HIST elements, on a copy. Called
     * once per track boundary from the playback task, so the cost is
     * irrelevant and the simplicity is not -- this runs on both units and must
     * be obviously the same computation on each.
     *
     * Only the first `count` slots are read. The rest are zero from the reset
     * and would otherwise vote as perfect readings while the history fills.
     */
    int32_t s[SYNC_PHASE_HIST];
    for (int i = 0; i < h->count; i++) {
        int32_t v = h->v[i];
        int j = i - 1;
        while (j >= 0 && s[j] > v) {
            s[j + 1] = s[j];
            j--;
        }
        s[j + 1] = v;
    }

    /* With an even count -- only possible before the history has filled --
     * this takes the upper of the two middle elements rather than averaging
     * them. Averaging would reintroduce exactly the sensitivity to a single
     * reading that this function exists to remove. */
    *out = s[h->count / 2];
    return true;
}

/**
 * @brief CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final
 *        xor.
 *
 * Bitwise rather than table-driven. At this stream's packet rate and covered
 * span it is a low single-digit percentage of one core on each chip --
 * affordable on both ends, and if it ever stops being, a 256-entry table is a
 * change to this function and nothing else.
 *
 * @param crc  Running value; start at 0xFFFF.
 * @param p    Bytes to fold in.
 * @param n    How many.
 * @return The updated running value.
 */
static uint16_t crc16_bytes(uint16_t crc, const uint8_t *p, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) {
        crc ^= (uint16_t)p[i] << 8;
        for (int k = 0; k < 8; k++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

uint16_t sbc_link_crc16(const void *hdr, const void *payload, uint16_t len)
{
    const uint16_t off = offsetof(spi_link_hdr_t, crc);
    const uint8_t *h = (const uint8_t *)hdr;
    const uint8_t zero[2] = { 0, 0 };
    uint16_t crc = 0xFFFF;

    /* The crc field reads as zero wherever it lands, so both ends compute over
     * the header they already hold rather than copying it to blank a field. */
    crc = crc16_bytes(crc, h, off);
    crc = crc16_bytes(crc, zero, sizeof(zero));
    crc = crc16_bytes(crc, h + off + 2, (uint16_t)(sizeof(spi_link_hdr_t) - off - 2));
    crc = crc16_bytes(crc, (const uint8_t *)payload, len);
    return crc;
}

/**
 * @brief XOR src into dst, bytewise and deliberately so.
 *
 * A word-at-a-time loop would need both operands at the same alignment, and
 * the codeword straddles the audio header -- so the payload half starts on an
 * odd boundary in the accumulator and there is nothing to align to without
 * changing the wire format. The cost of not bothering is one codeword per
 * packet at the audio packet rate, which is noise beside the SBC decode on the
 * same core; the CRC above spends many times more.
 *
 * @param dst  Accumulator, updated in place.
 * @param src  Bytes to fold in.
 * @param n    How many.
 */
static void xor_bytes(uint8_t *dst, const uint8_t *src, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) {
        dst[i] ^= src[i];
    }
}

bool audio_fec_xor_in(uint8_t *acc, uint16_t *span, const audio_msg_t *m)
{
    if (m->payload_len > AUDIO_FEC_PAYLOAD_MAX) {
        return false;
    }
    const uint16_t len = (uint16_t)AUDIO_MSG_BYTES(m->payload_len);

    /* Only the bytes this member actually has. Everything past them is the
     * implicit zero padding, and XOR-ing zeros is a no-op -- which is exactly
     * why unequal SBC lengths need no length table on the wire. */
    xor_bytes(acc, (const uint8_t *)m, len);
    if (len > *span) {
        *span = len;
    }
    return true;
}

bool audio_fec_extract(const uint8_t *acc, uint16_t span, uint32_t want_seq,
                       audio_msg_t *out)
{

    /* A codeword shorter than a bare header cannot be a packet, and reading a
     * payload_len out of it would be reading past the parity the sender
     * sent. */
    if (span < AUDIO_MSG_BYTES(0) || span > AUDIO_FEC_CODEWORD_MAX) {
        return false;
    }

    /* Copied out before anything is trusted, so the checks below read from a
     * properly typed object rather than reaching into acc[] at hand-computed
     * offsets. */
    memcpy(out, acc, span);

    /* The three checks are the two-loss defence; audio_fec_extract()'s
     * contract in sync_proto.h has the argument. They also catch a hub and a
     * satellite built with different K, where the receiver folds a genuinely
     * different set of packets than the sender. */
    if (out->type != MSG_AUDIO || out->format != AUDIO_FMT_SBC ||
        out->seq != want_seq) {
        return false;
    }

    /* And the length has to be consistent with the parity that carried it. A
     * recovered payload_len longer than the span means the decoder would read
     * bytes the sender never covered -- past the end of the recovered packet
     * and into whatever the caller's buffer held before. */
    if (out->payload_len > AUDIO_FEC_PAYLOAD_MAX ||
        AUDIO_MSG_BYTES(out->payload_len) > span) {
        return false;
    }
    return true;
}
