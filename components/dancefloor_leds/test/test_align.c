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
    int64_t  pending_block_index, align_block_index;
    uint32_t align_gen;
    /* reader side */
    int64_t  raw[FFT_N];
    int      filled;
    uint32_t recv_total, epoch, seen_gen;
    int64_t  block_index;
    /* results */
    int      blocks, misaligned, mislabelled, drops;
    int64_t  last_block_start, last_block_index;
} unit_t;

static void unit_init(unit_t *u) {
    memset(u, 0, sizeof(*u));
    u->align_pending = 1;
}

/*
 * Label mode. LABEL_CLOCK reads this board's clock at the moment of the feed,
 * which is what the second version of the aligner did; LABEL_DUE uses the
 * instant the audio is scheduled to be heard, which every unit derives from the
 * same per-packet play_at and therefore agrees on.
 */
#define LABEL_CLOCK 0
#define LABEL_DUE   1

/* One feed call: `nframes` frames starting at absolute index `first`.
 * `skew_frames` is how far this board's clock reading sits from the content it
 * is actually holding -- audio phase error plus task wake jitter. */
static void feed_mode(unit_t *u, int64_t first, int nframes, int mode, int skew_frames) {
    int64_t buf[CHUNK_FRAMES];
    for (int i = 0; i < nframes; i++) buf[i] = first + i;
    int64_t *p = buf;

    if (u->align_pending) {
        int64_t label = (mode == LABEL_DUE) ? first : first + skew_frames;
        int into = (int)(((label % FFT_N) + FFT_N) % FFT_N);
        u->skip_frames = into ? FFT_N - into : 0;
        u->pending_block_index = (label + u->skip_frames) / FFT_N;
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
        u->align_block_index = u->pending_block_index;
        u->align_gen++;
        u->mark_align_point = 0;
    }
    int took = fifo_push(&u->fifo, p, nframes);
    u->sent_total += (uint32_t)(took * FRAME_BYTES);
    if (took < nframes) { u->align_pending = 1; u->drops++; }   /* forces re-align */
}

static void feed(unit_t *u, int64_t first, int nframes) {
    feed_mode(u, first, nframes, LABEL_DUE, 0);
}

/*
 * One reader pass. `variant` selects how the reader notices a re-alignment:
 *
 *   READER_NONE  feed-side discard only, as the first version shipped
 *   READER_BYTE  compare the published byte index against the last one seen
 *   READER_GEN   compare a generation counter
 *
 * READER_BYTE fixed the boundaries but not the labelling, because a byte count
 * does not identify a publish. While the reader is behind, the buffer is full
 * and feeds are rejected entirely, so sent_total does not move: each rejected
 * feed re-arms the alignment and republishes the SAME byte index with a LATER
 * block index. The reader adopts the first and cannot see the rest, so it
 * labels from an origin several blocks stale -- for good, and again at every
 * burst of drops.
 *
 * Boundaries survive it, which is exactly why it hid: the discard arithmetic
 * still lands on a multiple of FFT_N, so two units cut identical windows and
 * merely date them differently. due_us carries the date.
 *
 * The first publish is missed as well, for the duller reason that a byte count
 * of zero equals the value it is compared against.
 */
#define READER_NONE 0
#define READER_BYTE 1
#define READER_GEN  2

static void reader(unit_t *u, int variant) {
    int64_t tmp[FFT_N];
    int want = FFT_N - u->filled;
    int got = fifo_pop(&u->fifo, tmp, want);
    if (got == 0) return;
    memcpy(u->raw + u->filled, tmp, (size_t)got * sizeof(int64_t));
    u->filled += got;
    u->recv_total += (uint32_t)(got * FRAME_BYTES);

    if (variant != READER_NONE) {
        int changed = (variant == READER_GEN) ? (u->align_gen != u->seen_gen)
                                              : (u->align_at  != u->epoch);
        if (changed) {
            int32_t ahead = (int32_t)(u->recv_total - u->align_at);
            if (ahead < 0) { u->filled = 0; return; }
            u->epoch = u->align_at;
            u->seen_gen = u->align_gen;
            u->block_index = u->align_block_index;
            int keep_frames = ahead / FRAME_BYTES;
            if (keep_frames > u->filled) keep_frames = u->filled;
            memmove(u->raw, u->raw + (u->filled - keep_frames),
                    (size_t)keep_frames * sizeof(int64_t));
            u->filled = keep_frames;
        }
    }

    if (u->filled < FFT_N) return;
    u->blocks++;
    if (u->raw[0] % FFT_N != 0) u->misaligned++;   /* single-unit property */
    /*
     * The label, which is the half the boundary check cannot see. due_us is
     * block_index * FFT_N / RATE, so the index must be the block number of the
     * content in the block -- otherwise two units cutting identical windows
     * still date them differently, and every animation keyed to due_us runs at
     * a different point of its cycle on each strip.
     */
    if (u->raw[0] != u->block_index * FFT_N) u->mislabelled++;
    u->last_block_start = u->raw[0];
    u->last_block_index = u->block_index;
    u->block_index++;
    u->filled = 0;
}

static void run(int variant, const char *name) {
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
            reader(&u, variant); reader(&u, variant);
            if ((rand() % 400) == 0) stall_left = 20 + rand() % 20;
        }
    }
    printf("  %-28s blocks %6d  drops %5d  misaligned %6d  mislabelled %6d  %s\n",
           name, u.blocks, u.drops, u.misaligned, u.mislabelled,
           (u.misaligned || u.mislabelled) ? "<-- BROKEN" : "aligned and labelled");
}

/*
 * A splice -- the correction every unit applies at a track boundary to null its
 * phase error -- takes samples out of the stream the reader is counting, or puts
 * samples in that the timeline does not account for. Either way the origin
 * established at the last alignment stops describing the audio, and since each
 * unit splices by its own error, the strips step apart at every track change and
 * stay apart.
 *
 * 6045 frames is 137 ms, a plausible correction and deliberately not a whole
 * number of blocks, so it breaks the boundaries as well as the labels.
 */
static void splice_case(int realign, const char *name) {
    unit_t u; unit_init(&u);
    int64_t pos = 4096;
    int splices = 0;

    for (int step = 0; step < 20000; step++) {
        if (step == 5000 || step == 12000) {
            pos += 6045;                    /* content skipped at the boundary */
            splices++;
            if (realign) u.align_pending = 1;
        }
        feed(&u, pos, CHUNK_FRAMES);
        pos += CHUNK_FRAMES;
        reader(&u, READER_GEN); reader(&u, READER_GEN);
    }
    printf("  %-28s splices %d  blocks %6d  misaligned %6d  mislabelled %6d  %s\n",
           name, splices, u.blocks, u.misaligned, u.mislabelled,
           (u.misaligned || u.mislabelled) ? "<-- BROKEN" : "aligned and labelled");
}

/*
 * The property that actually matters: TWO units, whose feed calls land a few ms
 * apart, must cut their analysis blocks at the same CONTENT positions. If they
 * do not, a transient near a boundary is split on one and centred on the other,
 * and the marginal onsets are detected by one unit and missed by the other.
 */
static void two_units(int mode, const char *name, int skew_ms) {
    unit_t a, b; unit_init(&a); unit_init(&b);
    int skew = skew_ms * 44100 / 1000;      /* B's clock reads this far off A's */
    int64_t pos = 4242;
    int mismatch = 0, compared = 0;

    for (int step = 0; step < 20000; step++) {
        feed_mode(&a, pos, CHUNK_FRAMES, mode, 0);
        feed_mode(&b, pos, CHUNK_FRAMES, mode, skew);
        pos += CHUNK_FRAMES;
        reader(&a, READER_GEN); reader(&a, READER_GEN);
        reader(&b, READER_GEN); reader(&b, READER_GEN);
        if (a.blocks > 2 && a.blocks == b.blocks) {
            compared++;
            if (a.last_block_start != b.last_block_start) mismatch++;
        }
    }
    printf("  %-30s skew %2d ms   compared %5d   differing block starts %5d  %s\n",
           name, skew_ms, compared, mismatch, mismatch ? "<-- DISAGREE" : "identical");
}

int main(void) {
    printf("block alignment across dropped feeds:\n");
    run(READER_NONE, "feed-side discard only");
    run(READER_BYTE, "reader watches byte index");
    run(READER_GEN,  "reader watches generation");

    printf("\nsplice at a track boundary:\n");
    splice_case(0, "spliced, no re-align");
    splice_case(1, "spliced, re-aligned");

    printf("\ntwo units, same audio, feed calls a few ms apart:\n");
    two_units(LABEL_CLOCK, "labelled by local clock", 3);
    two_units(LABEL_CLOCK, "labelled by local clock", 5);
    two_units(LABEL_DUE,   "labelled by scheduled time", 3);
    two_units(LABEL_DUE,   "labelled by scheduled time", 5);

    /* Re-run the fixed version and fail the build if it ever misaligns. */
    unit_t u; unit_init(&u); srand(999);
    int64_t pos = 31; int stall_left = 0;
    for (int step = 0; step < 200000; step++) {
        feed(&u, pos, CHUNK_FRAMES); pos += CHUNK_FRAMES;
        if (stall_left > 0) { stall_left--; }
        else {
            reader(&u, READER_GEN); reader(&u, READER_GEN);
            if ((rand() % 300) == 0) stall_left = 20 + rand() % 30;
        }
    }
    printf("\nsecond seed, fixed:          blocks %6d  drops %5d  misaligned %6d  "
           "mislabelled %6d\n", u.blocks, u.drops, u.misaligned, u.mislabelled);
    if (u.drops < 50)    { printf("FAIL: too few drops to exercise the fix\n"); return 1; }
    if (u.misaligned)    { printf("FAIL: blocks cut at the wrong positions\n"); return 1; }
    if (u.mislabelled)   { printf("FAIL: blocks carry the wrong index\n"); return 1; }

    /* A splice must not leave the reader labelling audio against an origin the
     * splice invalidated. */
    {
        unit_t s; unit_init(&s);
        int64_t p = 4096;
        for (int step = 0; step < 20000; step++) {
            if (step == 5000 || step == 12000) { p += 6045; s.align_pending = 1; }
            feed(&s, p, CHUNK_FRAMES);
            p += CHUNK_FRAMES;
            reader(&s, READER_GEN); reader(&s, READER_GEN);
        }
        if (s.blocks < 1000) { printf("FAIL: too few blocks after splices\n"); return 1; }
        if (s.misaligned)    { printf("FAIL: splice left the boundaries offset\n"); return 1; }
        if (s.mislabelled)   { printf("FAIL: splice left the labels offset\n"); return 1; }
    }

    /* And the cross-unit property, which is the one the LEDs actually show. */
    {
        unit_t a, b; unit_init(&a); unit_init(&b);
        int64_t pos = 4242; int mismatch = 0, compared = 0;
        for (int step = 0; step < 20000; step++) {
            feed_mode(&a, pos, CHUNK_FRAMES, LABEL_DUE, 0);
            feed_mode(&b, pos, CHUNK_FRAMES, LABEL_DUE, 5 * 44100 / 1000);
            pos += CHUNK_FRAMES;
            reader(&a, 1); reader(&a, 1); reader(&b, 1); reader(&b, 1);
            if (a.blocks > 2 && a.blocks == b.blocks) {
                compared++;
                if (a.last_block_start != b.last_block_start) mismatch++;
            }
        }
        if (compared < 1000) { printf("FAIL: too few comparisons\n"); return 1; }
        if (mismatch)        { printf("FAIL: units disagree on block starts\n"); return 1; }
    }
    printf("\nall tests passed\n");
    return 0;
}
