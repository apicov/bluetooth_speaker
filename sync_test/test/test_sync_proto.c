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
#include <inttypes.h>

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

    /* 8. Round-trip conversion is what actually schedules playback. */
    {
        int64_t master_now = 9000000;
        int64_t local = sync_to_local(master_now, TRUE_OFFSET);
        check("sync_to_local inverts the offset", local + TRUE_OFFSET == master_now, NULL);
    }

    printf("\n%s\n", failures ? "FAILURES PRESENT" : "all tests passed");
    return failures != 0;
}
