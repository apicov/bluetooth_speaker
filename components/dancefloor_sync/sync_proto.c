#include "sync_proto.h"

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
     * Two consecutive probes are a quarter of a second apart. Drift moves the
     * offset by a few microseconds in that time and path asymmetry by a few
     * milliseconds, so a step of a whole second is neither: the master rebooted
     * or was reflashed, and every sample already in the window describes a clock
     * that no longer exists.
     *
     * Keeping them is not a small error. The window spans 2.5 s, minimum-RTT
     * selection is free to pick any sample in it, and a pre-reboot sample is
     * wrong by the master's entire previous uptime -- an hour of it, measured
     * here. A satellite that anchors on that plays at a time that never arrives,
     * and the phase servo downstream is handed a number no correction can fix.
     *
     * Discarding the window drops `count` below SYNC_MIN_SAMPLES, so
     * sync_est_settled() goes false and playback holds until a clean estimate
     * exists -- which is what that mechanism was built for. It costs 2.5 s.
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

    /*
     * Walk oldest to newest so that '<=' lets a newer sample win an exact tie.
     * When the buffer is full the oldest entry is the one about to be overwritten.
     */
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
