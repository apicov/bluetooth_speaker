
#include "sync_proto.h"
#include "audio_out.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>

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
    const int64_t TRUE_OFFSET = 1234567;
    int64_t est;

    {
        sync_est_t e; sync_est_init(&e);
        probe(&e, 1000, TRUE_OFFSET, 500, 100, 500);
        probe(&e, 2000, TRUE_OFFSET, 500, 100, 500);
        check("rejects estimate below SYNC_MIN_SAMPLES", !sync_est_offset(&e, &est), NULL);
    }

    {
        sync_est_t e; sync_est_init(&e);
        for (int i = 0; i < 5; i++)
            probe(&e, 1000 * (i + 1), TRUE_OFFSET, 500, 100, 500);
        bool ok = sync_est_offset(&e, &est);
        char d[64]; snprintf(d, sizeof d, "est=%" PRId64, est);
        check("symmetric noiseless path is exact", ok && est == TRUE_OFFSET, d);
    }

    {
        sync_est_t e; sync_est_init(&e);
        for (int i = 0; i < SYNC_WINDOW; i++)
            probe(&e, 1000 * (i + 1), TRUE_OFFSET, rnd(400, 900), rnd(50, 150), rnd(400, 900));
        sync_est_offset(&e, &est);
        int64_t err = llabs(est - TRUE_OFFSET);
        char d[64]; snprintf(d, sizeof d, "err=%" PRId64 " us", err);
        check("jittered symmetric path within 1 ms", err < 1000, d);
    }

    {
        sync_est_t e; sync_est_init(&e);
        for (int i = 0; i < 9; i++)
            probe(&e, 1000 * (i + 1), TRUE_OFFSET, 500, 100, 500);
        probe(&e, 10000, TRUE_OFFSET, 80000, 100, 500);
        sync_est_offset(&e, &est);
        int64_t err = llabs(est - TRUE_OFFSET);
        char d[64]; snprintf(d, sizeof d, "err=%" PRId64 " us", err);
        check("min-RTT ignores an 80 ms retry outlier", err < 1000, d);
    }

    {
        sync_est_t e; sync_est_init(&e);
        for (int i = 0; i < SYNC_WINDOW; i++)
            probe(&e, 1000 * (i + 1), TRUE_OFFSET, 2000, 100, 200);
        sync_est_offset(&e, &est);
        int64_t err = est - TRUE_OFFSET;
        char d[64]; snprintf(d, sizeof d, "err=%" PRId64 " us (expected ~900)", err);
        check("asymmetric path errs by half the asymmetry", err == 900, d);
    }

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

    {
        sync_est_t e; sync_est_init(&e);
        for (int i = 0; i < 9; i++)
            probe(&e, 1000 * (i + 1), TRUE_OFFSET, 5000, 100, 500);
        probe(&e, 10000, TRUE_OFFSET, 600, 100, 600);
        sync_est_offset(&e, &est);
        int64_t err = llabs(est - TRUE_OFFSET);
        char d[72]; snprintf(d, sizeof d, "err=%" PRId64 " us (median would be ~2250)", err);
        check("min-RTT picks the symmetric probe", err < 100, d);
    }

    {
        sync_est_t e; sync_est_init(&e);
        for (int i = 0; i < SYNC_WINDOW; i++)
            probe(&e, 1000 * (i + 1), TRUE_OFFSET, 1000, 100, 1000);
        check("a full window is settled", sync_est_settled(&e), NULL);

        const int64_t after = TRUE_OFFSET - 3595000000LL;
        probe(&e, 20000, after, 1000, 100, 1000);

        check("a clock-origin step is not trusted as an estimate",
              !sync_est_settled(&e), "window discarded, playback holds");

        check("no estimate is offered from the remains",
              !sync_est_offset(&e, &est), "count fell below SYNC_MIN_SAMPLES");

        for (int i = 0; i < SYNC_MIN_SAMPLES; i++)
            probe(&e, 21000 + 1000 * i, after, 1000, 100, 1000);
        char d[80];
        snprintf(d, sizeof d, "est=%" PRId64 ", stale would be %" PRId64,
                 sync_est_offset(&e, &est) ? est : 0, TRUE_OFFSET);
        check("the stale window cannot be selected from", llabs(est - after) < 100, d);

        for (int i = 0; i < SYNC_WINDOW; i++)
            probe(&e, 30000 + 1000 * i, after, 1000, 100, 1000);
        sync_est_offset(&e, &est);
        check("it re-settles on the new origin",
              sync_est_settled(&e) && llabs(est - after) < 100, NULL);
    }

    {
        sync_est_t e; sync_est_init(&e);
        for (int i = 0; i < 5; i++)
            probe(&e, 1000 * (i + 1), TRUE_OFFSET, 1000, 100, 1000);
        probe(&e, 6000, TRUE_OFFSET + 9000000000LL, 1000, 100, 1000);
        for (int i = 0; i < SYNC_WINDOW; i++)
            probe(&e, 10000 + 1000 * i, TRUE_OFFSET, 1000, 100, 1000);
        sync_est_offset(&e, &est);
        char d[64]; snprintf(d, sizeof d, "est=%" PRId64, est);
        check("one corrupt probe does not wedge the estimator",
              sync_est_settled(&e) && llabs(est - TRUE_OFFSET) < 100, d);
    }

    {
        int64_t master_now = 9000000;
        int64_t local = sync_to_local(master_now, TRUE_OFFSET);
        check("sync_to_local inverts the offset", local + TRUE_OFFSET == master_now, NULL);
    }

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

    {
        sync_phase_hist_t h; sync_phase_reset(&h);
        for (int i = 0; i < SYNC_PHASE_HIST; i++) sync_phase_push(&h, 8231);
        int32_t med = 0; bool ok = sync_phase_median(&h, &med);
        char d[64]; snprintf(d, sizeof d, "med=%" PRId32, med);
        check("constant input passes through unchanged", ok && med == 8231, d);
    }

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

    {
        sync_phase_hist_t h; sync_phase_reset(&h);
        for (int i = 0; i < SYNC_PHASE_HIST; i++) sync_phase_push(&h, -30000);
        for (int i = 0; i < SYNC_PHASE_HIST; i++) sync_phase_push(&h, 4000);
        int32_t med = 0; sync_phase_median(&h, &med);
        char d[64]; snprintf(d, sizeof d, "med=%" PRId32, med);
        check("the ring keeps only the newest window", med == 4000, d);
    }

    {
        sync_phase_hist_t h; sync_phase_reset(&h);
        for (int i = 0; i < 5; i++) sync_phase_push(&h, 9000);
        int32_t med = 0; bool ok = sync_phase_median(&h, &med);
        char d[72]; snprintf(d, sizeof d, "med=%" PRId32 " (unwritten slots would give 0)", med);
        check("unwritten slots do not vote", ok && med == 9000, d);
    }

    {
        sync_phase_hist_t h; sync_phase_reset(&h);
        for (int i = 0; i < SYNC_PHASE_HIST; i++) sync_phase_push(&h, 12000);
        sync_phase_reset(&h);
        int32_t med;
        check("reset drops the whole history", !sync_phase_median(&h, &med), NULL);
    }

    {
        char d[80];
        snprintf(d, sizeof d, "hdr=%zu frame=%zu", sizeof(spi_link_hdr_t),
                 (size_t)SBC_LINK_FRAME_BYTES);
        check("the SPI frame is a multiple of 4 bytes",
              sizeof(spi_link_hdr_t) == 12 && SBC_LINK_FRAME_BYTES % 4 == 0, d);
    }

    {
        char d[80];
        snprintf(d, sizeof d, "spi=%zu wifi=%zu",
                 (size_t)SBC_LINK_MAX_PAYLOAD, (size_t)AUDIO_MAX_PAYLOAD);
        check("the WiFi ceiling covers the SPI ceiling",
              AUDIO_MAX_PAYLOAD >= SBC_LINK_MAX_PAYLOAD, d);
    }

    {
        char d[96];
        snprintf(d, sizeof d, "mtu-cap=%zu", (size_t)AUDIO_TX_PAYLOAD_MTU_MAX);
        check("the applied cap is header-plus-payload against the MTU",
              AUDIO_TX_PAYLOAD_MTU_MAX == 1446 &&
              AUDIO_MSG_BYTES(AUDIO_TX_PAYLOAD_MTU_MAX) == AUDIO_UDP_MTU, d);

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

    {
        const uint8_t payload[32] = { 1, 2, 3 };
        spi_link_hdr_t a = { .kind = LINK_KIND_SBC, .len = 32, .seq = 7, .crc = 0 };
        spi_link_hdr_t b = a;
        b.crc = 0xBEEF;
        check("the crc field does not feed itself",
              sbc_link_crc16(&a, payload, 32) == sbc_link_crc16(&b, payload, 32), NULL);
    }

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

    {
        char d[96];
        snprintf(d, sizeof d, "log=%zu health=%zu sub=%zu",
                 sizeof(log_msg_t), sizeof(health_msg_t), sizeof(log_sub_msg_t));
        check("the log/health message sizes are pinned",
              sizeof(log_msg_t) == 222 && sizeof(health_msg_t) == 108 &&
              sizeof(log_sub_msg_t) == 5, d);
    }

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

    {
        check("full volume is exactly unity",
              audio_volume_q15(AUDIO_VOL_MAX) == 32768, "no rounding loss at the top");
        check("zero volume is silence", audio_volume_q15(0) == 0, NULL);

        bool monotonic = true;
        for (int v = 1; v <= AUDIO_VOL_MAX; v++) {
            if (audio_volume_q15(v) < audio_volume_q15(v - 1)) {
                monotonic = false;
            }
        }
        check("the taper never goes backwards", monotonic,
              "a slider that drops in level as it is raised");

        int16_t a[8], b[8];
        for (int i = 0; i < 8; i++) {
            a[i] = b[i] = (int16_t)(i * 4000 - 16000);
        }
        audio_apply_volume(a, 4, AUDIO_VOL_MAX);
        check("full volume does not touch the samples",
              memcmp(a, b, sizeof a) == 0, NULL);

        int16_t q[8] = { 0 };
        audio_apply_volume(q, 4, 40);
        bool quiet_stays = true;
        for (int i = 0; i < 8; i++) {
            if (q[i] != 0) {
                quiet_stays = false;
            }
        }
        check("silence maps to silence at any volume", quiet_stays, NULL);

        int16_t s[8];
        for (int i = 0; i < 8; i++) {
            s[i] = (int16_t)(i * 4000 - 16000);
        }
        audio_apply_volume(s, 4, AUDIO_VOL_MAX / 2);
        bool shrunk = true;
        for (int i = 0; i < 8; i++) {
            const int16_t o = (int16_t)(i * 4000 - 16000);
            if (abs(s[i]) > abs(o) || (o > 0 && s[i] < 0) || (o < 0 && s[i] > 0)) {
                shrunk = false;
            }
        }
        char d[96];
        snprintf(d, sizeof d, "half slider = q15 %d of 32768",
                 (int)audio_volume_q15(AUDIO_VOL_MAX / 2));
        check("half volume attenuates without inverting", shrunk, d);

        check("vol_msg_t is two bytes on the wire", sizeof(vol_msg_t) == 2,
              "type + level, and nothing a padding byte could hide in");
    }

    {
        check("q16 endpoints are exact",
              audio_volume_q16(0) == 0 && audio_volume_q16(AUDIO_VOL_MAX) == 65536,
              "unity is a shift of 16, silence is silence");

        bool strict = true;
        for (int v = 2; v <= AUDIO_VOL_MAX; v++) {
            if (audio_volume_q16(v) <= audio_volume_q16(v - 1)) {
                strict = false;
            }
        }
        check("the taper strictly increases", strict, "no two positions alike");

        bool strict15 = true;
        for (int v = 2; v <= AUDIO_VOL_MAX; v++) {
            if (audio_volume_q15(v) <= audio_volume_q15(v - 1)) {
                strict15 = false;
            }
        }
        check("halving it keeps that", strict15,
              "q15 is q16 >> 1, and a shift must not flatten a step");

        const double nominal = 60.0 / (AUDIO_VOL_MAX - 1);
        double worst = 0.0, worst_up = 0.0;
        int worst_at = 0, worst_up_at = 0;
        for (int v = 2; v <= AUDIO_VOL_MAX; v++) {
            const double db = 20.0 * log10((double)audio_volume_q16(v)
                                           / (double)audio_volume_q16(v - 1));
            const double err = fabs(db - nominal);
            if (err > worst) {
                worst = err;
                worst_at = v;
            }
            if (v >= 16 && err > worst_up) {
                worst_up = err;
                worst_up_at = v;
            }
        }
        char d[96];
        snprintf(d, sizeof d, "worst %.3f dB at %d, nominal %.4f",
                 worst, worst_at, nominal);
        check("every step is the same size in dB", worst < 0.1, d);
        snprintf(d, sizeof d, "worst %.3f dB at %d, above the rounding floor",
                 worst_up, worst_up_at);
        check("and tighter still once the entries are big enough to be exact",
              worst_up < 0.05, d);

        const double floor_db = 20.0 * log10((double)audio_volume_q16(1) / 65536.0);
        snprintf(d, sizeof d, "%.2f dB at level 1", floor_db);
        check("the bottom of the curve is where the generator put it",
              fabs(floor_db + 60.0) < 0.1, d);
    }

    {
        bool ok = true, unity_exact = true;
        long long worst_lo = 0, worst_hi = 0;
        for (int v = 0; v <= AUDIO_VOL_MAX; v++) {
            const long long g = audio_volume_q16(v);
            for (long long x = -32768; x <= 32767; x++) {
                const long long y = x * g;
                if (y < INT32_MIN || y > INT32_MAX) {
                    ok = false;
                }
                if (y < worst_lo) worst_lo = y;
                if (y > worst_hi) worst_hi = y;

                if ((x > 0 && y < 0) || (x < 0 && y > 0) ||
                    llabs(y) > llabs(x) * 65536) {
                    ok = false;
                }
                if (v == AUDIO_VOL_MAX && y != x * 65536) {
                    unity_exact = false;
                }
            }
        }
        char d[96];
        snprintf(d, sizeof d, "range %lld .. %lld against %d .. %d",
                 worst_lo, worst_hi, INT32_MIN, INT32_MAX);
        check("the 32-bit multiply fits, at every level and every sample", ok, d);
        check("full volume widens to exactly in << 16", unity_exact,
              "so a full-scale build is bit-identical through the conversion");
    }

    {
        char d[96];
        snprintf(d, sizeof d, "ring %d B, out %d B, frame %d B",
                 (int)AUDIO_CHUNK_BYTES, (int)AUDIO_OUT_CHUNK_BYTES,
                 (int)AUDIO_OUT_FRAME_BYTES);
        check("the output chunk is exactly twice the ring chunk",
              AUDIO_OUT_CHUNK_BYTES == 2 * AUDIO_CHUNK_BYTES &&
              AUDIO_OUT_FRAME_BYTES == AUDIO_CHANNELS * (int)sizeof(int32_t), d);
    }

    {
        audio_ramp_t r = { 0 };
        int16_t in[AUDIO_FRAMES * AUDIO_CHANNELS];
        int32_t out[AUDIO_FRAMES * AUDIO_CHANNELS];
        for (int i = 0; i < AUDIO_FRAMES * AUDIO_CHANNELS; i++) {
            in[i] = (int16_t)((i % 2) ? 12000 : -9000);
        }

        int chunks = 0;
        while (r.cur != audio_volume_q16(AUDIO_VOL_MAX) && chunks < 64) {
            audio_volume_write_i32(out, in, AUDIO_FRAMES, AUDIO_VOL_MAX, &r);
            chunks++;
        }
        char d[96];
        snprintf(d, sizeof d, "%d chunks from silence to unity", chunks);
        check("the ramp reaches its target", chunks <= 8, d);
        check("and stops there", r.cur == audio_volume_q16(AUDIO_VOL_MAX),
              "no overshoot past the level it was asked for");

        audio_volume_write_i32(out, in, AUDIO_FRAMES, AUDIO_VOL_MAX, &r);
        bool exact = true;
        for (int i = 0; i < AUDIO_FRAMES * AUDIO_CHANNELS; i++) {
            if (out[i] != (int32_t)in[i] * 65536) {
                exact = false;
            }
        }
        check("a settled ramp at full volume is the plain widening", exact, NULL);

        audio_ramp_t mid = { audio_volume_q16(40) };
        int16_t quiet[AUDIO_FRAMES * AUDIO_CHANNELS] = { 0 };
        audio_volume_write_i32(out, quiet, AUDIO_FRAMES, AUDIO_VOL_MAX, &mid);
        bool silent = true;
        for (int i = 0; i < AUDIO_FRAMES * AUDIO_CHANNELS; i++) {
            if (out[i] != 0) {
                silent = false;
            }
        }
        check("silence stays silence at any point in a ramp", silent, NULL);
    }

    {
        check("an untold unit plays silence",
              audio_vol_effective(99, false, false) == 0,
              "the level beside the flag is meaningless until the flag is set");
        check("an untold unit falls back to full scale, eventually",
              audio_vol_effective(99, false, true) == AUDIO_VOL_MAX,
              "so a hub that never speaks cannot silence a floor forever");
        check("a told unit plays what it was told",
              audio_vol_effective(40, true, false) == 40 &&
              audio_vol_effective(40, true, true) == 40,
              "the deadline stops mattering once anything has been said");
        check("a deliberate mute is not ignorance",
              audio_vol_effective(0, true, true) == 0,
              "somebody who muted the room from the phone has not said nothing");
    }

    {
        enum { K = 4 };
        static audio_msg_t grp[K];
        static uint8_t acc[AUDIO_FEC_CODEWORD_MAX];
        static uint8_t rec[AUDIO_FEC_CODEWORD_MAX];
        const uint16_t lens[K] = { 851, 823, 877, 12 };
        uint16_t span = 0;
        bool built = true;

        for (int i = 0; i < K; i++) {
            memset(&grp[i], 0, sizeof(grp[i]));
            grp[i].type = MSG_AUDIO;
            grp[i].format = AUDIO_FMT_SBC;
            grp[i].marker = (i == 2);
            grp[i].restart = (i == 3);
            grp[i].payload_len = lens[i];
            grp[i].seq = 1000 + (uint32_t)i;
            grp[i].sample_rate = 44100;
            grp[i].frames = 882;
            grp[i].play_at = 5000000LL + i * 20000LL;
            for (uint16_t b = 0; b < lens[i]; b++) {
                grp[i].payload[b] = (uint8_t)(b * 7 + i * 31 + 1);
            }
            built = built && audio_fec_xor_in(acc, &span, &grp[i]);
        }
        check("parity spans the longest member's codeword",
              built && span == AUDIO_MSG_BYTES(877), NULL);

        static uint8_t parity[AUDIO_FEC_CODEWORD_MAX];
        memcpy(parity, acc, span);

        int recovered = 0, exact = 0;
        for (int drop = 0; drop < K; drop++) {
            uint16_t s2 = 0;
            memset(acc, 0, sizeof(acc));
            for (int i = 0; i < K; i++) {
                if (i != drop) {
                    audio_fec_xor_in(acc, &s2, &grp[i]);
                }
            }
            for (uint16_t b = 0; b < span; b++) {
                acc[b] ^= parity[b];
            }
            if (audio_fec_extract(acc, span, grp[drop].seq, (audio_msg_t *)rec)) {
                recovered++;
                if (memcmp(rec, &grp[drop],
                           AUDIO_MSG_BYTES(grp[drop].payload_len)) == 0) {
                    exact++;
                }
            }
        }
        char d[64];
        snprintf(d, sizeof d, "recovered=%d exact=%d of %d", recovered, exact, K);
        check("any single member is rebuilt byte for byte", recovered == K &&
              exact == K, d);
    }

    {
        enum { K = 4 };
        static audio_msg_t grp[K];
        static uint8_t acc[AUDIO_FEC_CODEWORD_MAX];
        static uint8_t parity[AUDIO_FEC_CODEWORD_MAX];
        static uint8_t rec[AUDIO_FEC_CODEWORD_MAX];
        uint16_t span = 0;

        for (int i = 0; i < K; i++) {
            memset(&grp[i], 0, sizeof(grp[i]));
            grp[i].type = MSG_AUDIO;
            grp[i].format = AUDIO_FMT_SBC;
            grp[i].payload_len = (uint16_t)(800 + i);
            grp[i].seq = 2000 + (uint32_t)i;
            grp[i].frames = 882;
            grp[i].play_at = 90000LL + i;
            for (uint16_t b = 0; b < grp[i].payload_len; b++) {
                grp[i].payload[b] = (uint8_t)(b + i);
            }
            audio_fec_xor_in(acc, &span, &grp[i]);
        }
        memcpy(parity, acc, span);

        uint16_t s2 = 0;
        memset(acc, 0, sizeof(acc));
        audio_fec_xor_in(acc, &s2, &grp[2]);
        audio_fec_xor_in(acc, &s2, &grp[3]);
        for (uint16_t b = 0; b < span; b++) {
            acc[b] ^= parity[b];
        }
        check("two losses in one group are refused",
              !audio_fec_extract(acc, span, grp[0].seq, (audio_msg_t *)rec) &&
              !audio_fec_extract(acc, span, grp[1].seq, (audio_msg_t *)rec),
              "the rebuilt header is both packets at once, and is neither");

        s2 = 0;
        memset(acc, 0, sizeof(acc));
        for (int i = 1; i < K; i++) {
            audio_fec_xor_in(acc, &s2, &grp[i]);
        }
        for (uint16_t b = 0; b < span; b++) {
            acc[b] ^= parity[b];
        }
        check("a recovery for the wrong seq is refused",
              !audio_fec_extract(acc, span, grp[0].seq + 1, (audio_msg_t *)rec) &&
              audio_fec_extract(acc, span, grp[0].seq, (audio_msg_t *)rec),
              "same codeword, and only the seq the group actually lost passes");

        check("an impossible span is refused",
              !audio_fec_extract(acc, (uint16_t)(AUDIO_MSG_BYTES(0) - 1),
                                 grp[0].seq, (audio_msg_t *)rec) &&
              !audio_fec_extract(acc, (uint16_t)(AUDIO_FEC_CODEWORD_MAX + 1),
                                 grp[0].seq, (audio_msg_t *)rec),
              "a truncated parity must not be read as a short packet");
    }

    {
        char d[96];
        snprintf(d, sizeof d, "payload-max=%zu codeword=%zu datagram=%zu",
                 (size_t)AUDIO_FEC_PAYLOAD_MAX, (size_t)AUDIO_FEC_CODEWORD_MAX,
                 (size_t)AUDIO_FEC_MSG_BYTES(AUDIO_FEC_CODEWORD_MAX));
        check("a full parity datagram is exactly one MTU",
              AUDIO_FEC_MSG_BYTES(AUDIO_FEC_CODEWORD_MAX) == AUDIO_UDP_MTU &&
              AUDIO_FEC_CODEWORD_MAX == AUDIO_MSG_BYTES(AUDIO_FEC_PAYLOAD_MAX), d);

        snprintf(d, sizeof d, "851 B payload -> %zu B parity vs %zu B audio",
                 (size_t)AUDIO_FEC_MSG_BYTES(AUDIO_MSG_BYTES(851)),
                 (size_t)AUDIO_MSG_BYTES(851));
        check("parity for a typical group is one typical packet",
              AUDIO_FEC_MSG_BYTES(AUDIO_MSG_BYTES(851)) < AUDIO_UDP_MTU &&
              AUDIO_FEC_MSG_BYTES(AUDIO_MSG_BYTES(851)) <
                  AUDIO_MSG_BYTES(851) + AUDIO_FEC_HDR_BYTES + 1, d);

        static audio_msg_t big;
        static uint8_t acc[AUDIO_FEC_CODEWORD_MAX];
        uint16_t span = 0;
        memset(&big, 0, sizeof(big));
        big.payload_len = AUDIO_FEC_PAYLOAD_MAX + 1;
        check("a payload parity cannot cover is refused, not truncated",
              !audio_fec_xor_in(acc, &span, &big) && span == 0,
              "truncation is what made every recovery of the old scheme partial");
    }

    printf("\n%s\n", failures ? "FAILURES PRESENT" : "all tests passed");
    return failures != 0;
}
