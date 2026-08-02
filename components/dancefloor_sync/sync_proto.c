#include "sync_proto.h"

#include <string.h>

void sync_est_init(sync_est_t *e)
{
    memset(e, 0, sizeof(*e));
}

void sync_est_add(sync_est_t *e, int64_t t1, int64_t t2, int64_t t3, int64_t t4)
{
    e->offset[e->next] = ((t2 - t1) + (t3 - t4)) / 2;
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
