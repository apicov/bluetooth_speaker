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

    *out = s[h->count / 2];
    return true;
}

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

    crc = crc16_bytes(crc, h, off);
    crc = crc16_bytes(crc, zero, sizeof(zero));
    crc = crc16_bytes(crc, h + off + 2, (uint16_t)(sizeof(spi_link_hdr_t) - off - 2));
    crc = crc16_bytes(crc, (const uint8_t *)payload, len);
    return crc;
}

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

    xor_bytes(acc, (const uint8_t *)m, len);
    if (len > *span) {
        *span = len;
    }
    return true;
}

bool audio_fec_extract(const uint8_t *acc, uint16_t span, uint32_t want_seq,
                       audio_msg_t *out)
{

    if (span < AUDIO_MSG_BYTES(0) || span > AUDIO_FEC_CODEWORD_MAX) {
        return false;
    }

    memcpy(out, acc, span);

    if (out->type != MSG_AUDIO || out->format != AUDIO_FMT_SBC ||
        out->seq != want_seq) {
        return false;
    }

    if (out->payload_len > AUDIO_FEC_PAYLOAD_MAX ||
        AUDIO_MSG_BYTES(out->payload_len) > span) {
        return false;
    }
    return true;
}
