/*
 * The block aligner, on the host.
 *
 * Exists because the first version of it was broken in a way that looked like
 * it worked: the feed side discarded frames to land on a block boundary, but
 * the analysis task held a partially accumulated block and completing it with
 * freshly aligned data left the boundaries offset by whatever it happened to be
 * holding. Both units started aligned and the first dropped feed on either one
 * un-aligned it permanently -- visible as one strip firing on a beat while the
 * other stayed dark.
 *
 * The property under test is the one that matters on hardware: every block a
 * unit analyses must start at a stream position that is a multiple of FFT_N in
 * master-clock samples, because that is what makes two units analyse identical
 * windows and therefore reach identical onset decisions.
 *
 * Both the old and new logic are here, so the test demonstrates the difference
 * rather than merely asserting the new one is fine.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define FFT_N        1024
#define CHANNELS     2
#define FRAME_BYTES  (CHANNELS * (int)sizeof(int16_t))
#define BLOCK_BYTES  (FFT_N * FRAME_BYTES)
#define STREAM_BYTES (BLOCK_BYTES * 4)
#define CHUNK_FRAMES 256                    /* AUDIO_FRAMES, one feed call */

/* A byte-accurate FIFO whose contents are the ABSOLUTE frame index each frame
 * came from, so the reader can be checked against ground truth. */
typedef struct {
    int64_t frame[STREAM_BYTES / FRAME_BYTES];
    int n;
} fifo_t;

static int fifo_space(const fifo_t *f) {
    return (int)(sizeof(f->frame) / sizeof(f->frame[0])) - f->n;
}
static int fifo_push(fifo_t *f, const int64_t *src, int nframes) {
    int room = fifo_space(f);
    int take = nframes < room ? nframes : room;
    memcpy(f->frame + f->n, src, (size_t)take * sizeof(int64_t));
    f->n += take;
    return take;                            /* short push == dropped audio */
}
static int fifo_pop(fifo_t *f, int64_t *dst, int nframes) {
    int take = nframes < f->n ? nframes : f->n;
    memcpy(dst, f->frame, (size_t)take * sizeof(int64_t));
    memmove(f->frame, f->frame + take, (size_t)(f->n - take) * sizeof(int64_t));
    f->n -= take;
    return take;
}

typedef struct {
    fifo_t   fifo;
    /* feed side */
    int      align_pending, mark_align_point, skip_frames;
    uint32_t sent_total, align_at;
    /* reader side */
    int64_t  raw[FFT_N];
    int      filled;
    uint32_t recv_total, epoch;
    /* results */
    int      blocks, misaligned, drops;
} unit_t;

static void unit_init(unit_t *u) {
    memset(u, 0, sizeof(*u));
    u->align_pending = 1;
}

/* One feed call: `nframes` frames starting at absolute index `first`. */
static void feed(unit_t *u, int64_t first, int nframes) {
    int64_t buf[CHUNK_FRAMES];
    for (int i = 0; i < nframes; i++) buf[i] = first + i;
    int64_t *p = buf;

    if (u->align_pending) {
        /* The real code derives this from the master clock; the frame index of
         * the audio in hand is the same quantity, exactly. */
        int into = (int)(first % FFT_N);
        u->skip_frames = into ? FFT_N - into : 0;
        u->align_pending = 0;
        u->mark_align_point = 1;
    }
    if (u->skip_frames > 0) {
        int drop = u->skip_frames < nframes ? u->skip_frames : nframes;
        u->skip_frames -= drop;
        p += drop; nframes -= drop;
        if (nframes == 0) return;
    }
    if (u->mark_align_point) {
        u->align_at = u->sent_total;
        u->mark_align_point = 0;
    }
    int took = fifo_push(&u->fifo, p, nframes);
    u->sent_total += (uint32_t)(took * FRAME_BYTES);
    if (took < nframes) { u->align_pending = 1; u->drops++; }   /* forces re-align */
}

/* One reader pass. `use_fix` selects the new epoch handling. */
static void reader(unit_t *u, int use_fix) {
    int64_t tmp[FFT_N];
    int want = FFT_N - u->filled;
    int got = fifo_pop(&u->fifo, tmp, want);
    if (got == 0) return;
    memcpy(u->raw + u->filled, tmp, (size_t)got * sizeof(int64_t));
    u->filled += got;
    u->recv_total += (uint32_t)(got * FRAME_BYTES);

    if (use_fix) {
        uint32_t align_at = u->align_at;
        if (align_at != u->epoch) {
            int32_t ahead = (int32_t)(u->recv_total - align_at);
            if (ahead < 0) { u->filled = 0; return; }
            u->epoch = align_at;
            int keep_frames = ahead / FRAME_BYTES;
            if (keep_frames > u->filled) keep_frames = u->filled;
            memmove(u->raw, u->raw + (u->filled - keep_frames),
                    (size_t)keep_frames * sizeof(int64_t));
            u->filled = keep_frames;
        }
    }

    if (u->filled < FFT_N) return;
    u->blocks++;
    if (u->raw[0] % FFT_N != 0) u->misaligned++;   /* THE property */
    u->filled = 0;
}

static void run(int use_fix, const char *name) {
    unit_t u; unit_init(&u);
    srand(12345);
    int64_t pos = 7777;                    /* arbitrary start, not on a boundary */
    int stall_left = 0;

    for (int step = 0; step < 200000; step++) {
        feed(&u, pos, CHUNK_FRAMES);
        pos += CHUNK_FRAMES;
        /* The reader normally keeps up. Occasionally it stalls for long enough
         * that the FIFO fills and the feed drops -- which is the whole case the
         * fix exists for. A single missed pass is not enough: the FIFO holds 16
         * feeds, so the stall has to be a burst. */
        if (stall_left > 0) { stall_left--; }
        else {
            reader(&u, use_fix); reader(&u, use_fix);
            if ((rand() % 400) == 0) stall_left = 20 + rand() % 20;
        }
    }
    printf("  %-28s blocks %6d  drops %5d  misaligned %6d  %s\n",
           name, u.blocks, u.drops, u.misaligned,
           u.misaligned ? "<-- MISALIGNED" : "aligned");
}

int main(void) {
    printf("block alignment across dropped feeds:\n");
    run(0, "feed-side discard only");
    run(1, "with reader epoch handling");

    /* Re-run the fixed version and fail the build if it ever misaligns. */
    unit_t u; unit_init(&u); srand(999);
    int64_t pos = 31; int stall_left = 0;
    for (int step = 0; step < 200000; step++) {
        feed(&u, pos, CHUNK_FRAMES); pos += CHUNK_FRAMES;
        if (stall_left > 0) { stall_left--; }
        else {
            reader(&u, 1); reader(&u, 1);
            if ((rand() % 300) == 0) stall_left = 20 + rand() % 30;
        }
    }
    printf("\nsecond seed, fixed:          blocks %6d  drops %5d  misaligned %6d\n",
           u.blocks, u.drops, u.misaligned);
    if (u.drops < 50)    { printf("FAIL: too few drops to exercise the fix\n"); return 1; }
    if (u.misaligned)    { printf("FAIL\n"); return 1; }
    printf("\nall tests passed\n");
    return 0;
}
