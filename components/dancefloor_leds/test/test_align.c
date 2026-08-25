/**
 * @file test_align.c
 * @brief Host test for the block alignment: that two units cut their analysis
 *        windows at the same sample positions.
 *
 * The single most sync-critical property in this component, and the one
 * nothing on a board can check -- a unit doing its own analysis exchanges
 * nothing with its neighbours, which is exactly what makes them stay in step
 * and exactly what makes a disagreement invisible. So it is checked here, by
 * simulating two units side by side.
 *
 * The model mirrors visualiser.cpp's feed and analysis stages: a stream fifo,
 * a byte counter, an origin published to a reader that counts from it. What
 * the cases vary is everything a real floor varies -- when each unit joined,
 * how the audio was chunked into it, whether one of them dropped bytes, and
 * whether a splice went unreported. The requirement is always the same: the
 * two units must cut identical blocks, and must recover to identical blocks
 * after any of it.
 *
 * LABEL_CLOCK against LABEL_DUE is the case that justifies the design. Feeding
 * from a CLOCK reading rather than from the scheduled instant is what an
 * earlier version did, and the test carries it so the failure it produces is
 * visible rather than argued about.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "analysis_config.h"

/* The firmware's own window and hop, from the shared header rather than
 * restated here -- a test cutting a different grid from the code would prove
 * nothing about it. */
/** @brief The analysis window. */
#define FFT_N        DF_FFT_N
/** @brief ...and the hop between windows. */
#define HOP_N        DF_HOP_N
/** @brief What one window carries into the next. */
#define TAIL_N       DF_TAIL_N
/** @brief Interleaved channels, as the firmware has them. */
#define CHANNELS     2
/** @brief Bytes per audio frame. */
#define FRAME_BYTES  (CHANNELS * (int)sizeof(int16_t))
/** @brief Bytes per analysis window. */
#define BLOCK_BYTES  (FFT_N * FRAME_BYTES)
/** @brief The simulated stream buffer's depth. */
#define STREAM_BYTES (BLOCK_BYTES * 4)
/** @brief Frames per feed call, so the audio arrives in chunks rather than all
 *         at once -- a real caller feeds a packet at a time. */
#define CHUNK_FRAMES 256

/**
 * @brief The simulated stream buffer.
 *
 * Holds one int64 per audio FRAME rather than samples: each carries the label
 * that frame should end up with, so a window can be checked against what it
 * was actually made of. That is the whole trick this test rests on -- the
 * audio is its own expected answer.
 */
typedef struct {
    int64_t frame[STREAM_BYTES / FRAME_BYTES];   /**< The labels. */
    int n;                                       /**< How many are held. */
} fifo_t;

/** @brief Room left. @param f The fifo. @return Frames it can still take. */
static int fifo_space(const fifo_t *f) {
    return (int)(sizeof(f->frame) / sizeof(f->frame[0])) - f->n;
}
/** @brief Push frames, taking only what fits -- the firmware's stream buffer
 *         drops the rest, and so must this.
 *  @param f  The fifo. @param src Labels. @param nframes How many.
 *  @return How many were taken. */
static int fifo_push(fifo_t *f, const int64_t *src, int nframes) {
    int room = fifo_space(f);
    int take = nframes < room ? nframes : room;
    memcpy(f->frame + f->n, src, (size_t)take * sizeof(int64_t));
    f->n += take;
    return take;
}
/** @brief Pop up to @p nframes frames.
 *  @param f  The fifo. @param[out] dst Labels. @param nframes How many wanted.
 *  @return How many were given. */
static int fifo_pop(fifo_t *f, int64_t *dst, int nframes) {
    int take = nframes < f->n ? nframes : f->n;
    memcpy(dst, f->frame, (size_t)take * sizeof(int64_t));
    memmove(f->frame, f->frame + take, (size_t)(f->n - take) * sizeof(int64_t));
    f->n -= take;
    return take;
}

/** @brief The drift tolerance in FRAMES, matching ALIGN_DRIFT_US in
 *         visualiser.cpp at the reference rate. */
#define DRIFT_FRAMES (2 * 44100 / 1000)

/** @brief Whether the simulated feed runs the drift check. Turned off to show
 *         what an unreported splice costs WITHOUT it, which is what the check
 *         was added for. */
static int g_drift_check = 1;

/** @brief One simulated unit: the feed side, the analysis side, and what its
 *         windows turned out to be. Mirrors visualiser.cpp's state field for
 *         field, because a model that simplified it would stop testing it. */
typedef struct {
    fifo_t   fifo;                  /**< The stream buffer between the two sides. */

    int      align_pending;         /**< An alignment is wanted. */
    int      mark_align_point;      /**< The next bytes fed carry the origin. */
    int      skip_frames;           /**< Part-hop still to drop before it. */
    uint32_t sent_total;            /**< Frames fed since boot. */
    uint32_t align_at;              /**< Where the origin falls in that count. */
    int64_t  pending_block_index;   /**< The block index it lands on... */
    int64_t  align_block_index;     /**< ...once published. */
    uint32_t align_gen;             /**< Bumped when an origin is published. */

    int64_t  ref_frame;             /**< The reference pair the drift check... */
    uint32_t ref_byte;              /**< ...extrapolates from. */
    int      ref_valid;             /**< Whether it means anything yet. */
    int      drifts;                /**< Times the check re-derived the origin. */

    int64_t  raw[FFT_N];            /**< The window being filled. */
    int      filled;                /**< How much of it. */
    uint32_t recv_total;            /**< Frames taken from the fifo. */
    uint32_t epoch;                 /**< Unused by the model; kept for symmetry. */
    uint32_t seen_gen;              /**< The last origin this side adopted. */
    int64_t  block_index;           /**< Counted from that origin. */

    int      blocks;                /**< Windows cut. */
    int      misaligned;            /**< ...that did not start on a hop boundary. */
    int      mislabelled;           /**< ...whose index did not match their audio. */
    int      drops;                 /**< Frames the fifo refused. */
    int      discontiguous;         /**< Windows that did not follow the last. */
    int64_t  last_block_start;      /**< For the contiguity check. */
    int64_t  last_block_index;      /**< Likewise. */

    int64_t  worst_err;             /**< Worst label error seen, in frames. */
} unit_t;

/** @brief Reset a unit to its boot state. @param u The unit. */
static void unit_init(unit_t *u) {
    memset(u, 0, sizeof(*u));
    u->align_pending = 1;
}

/**
 * @brief Label the audio from a CLOCK reading, which is the way that does not
 *        work.
 *
 * Two units reach the feed a few milliseconds apart, so each labels the same
 * audio differently and their block grids come apart by that skew. Carried
 * here so the failure is visible rather than argued about; the two-unit case
 * runs both modes and requires this one to disagree.
 */
#define LABEL_CLOCK 0
/** @brief Label it by the SCHEDULED instant, which is what the firmware does:
 *         every unit is handed the same instant for the same audio, so every
 *         unit derives the same label. */
#define LABEL_DUE   1

/**
 * @brief Feed one run of audio into a unit, the way visualiser_feed() does.
 *
 * @param u            The unit.
 * @param first        Label of the first frame.
 * @param nframes      How many frames.
 * @param mode         LABEL_CLOCK or LABEL_DUE.
 * @param skew_frames  How far this unit's own clock is from its neighbour's;
 *                     only LABEL_CLOCK is affected by it, which is the point.
 */
static void feed_mode(unit_t *u, int64_t first, int nframes, int mode, int skew_frames) {
    int64_t buf[CHUNK_FRAMES];
    for (int i = 0; i < nframes; i++) buf[i] = first + i;
    int64_t *p = buf;

    const int64_t label = (mode == LABEL_DUE) ? first : first + skew_frames;

    if (g_drift_check && u->ref_valid && u->skip_frames <= 0 && !u->mark_align_point &&
        !u->align_pending) {
        int64_t since = (int64_t)(uint32_t)(u->sent_total - u->ref_byte) / FRAME_BYTES;
        int64_t drift = label - (u->ref_frame + since);
        if (drift > DRIFT_FRAMES || drift < -DRIFT_FRAMES) {
            u->align_pending = 1;
            u->drifts++;
        }
    }

    if (u->align_pending) {
        int into = (int)(((label % HOP_N) + HOP_N) % HOP_N);
        u->skip_frames = into ? HOP_N - into : 0;
        u->pending_block_index = (label + u->skip_frames) / HOP_N;
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
        u->ref_frame = u->pending_block_index * HOP_N;
        u->ref_byte = u->sent_total;
        u->ref_valid = 1;
        u->mark_align_point = 0;
    }
    int took = fifo_push(&u->fifo, p, nframes);
    u->sent_total += (uint32_t)(took * FRAME_BYTES);
    if (took < nframes) { u->align_pending = 1; u->drops++; }
}

/** @brief feed_mode() in the mode the firmware uses.
 *  @param u  The unit. @param first First label. @param nframes How many. */
static void feed(unit_t *u, int64_t first, int nframes) {
    feed_mode(u, first, nframes, LABEL_DUE, 0);
}

/** @brief The analysis side ignores alignment entirely -- the behaviour before
 *         any of this existed, kept so the cases show what it costs. */
#define READER_NONE 0
/** @brief It honours the byte position but not the generation, so it cannot
 *         tell a new origin from the old one. */
#define READER_BYTE 1
/** @brief It honours both, which is what visualiser_task() does. */
#define READER_GEN  2

/**
 * @brief Drain what has been fed and cut windows from it, the way
 *        visualiser_task() does, checking each one as it goes.
 *
 * @param u        The unit.
 * @param variant  READER_NONE, READER_BYTE or READER_GEN.
 */
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
    if (u->raw[0] % HOP_N != 0) u->misaligned++;

    if (u->raw[0] != u->block_index * HOP_N) u->mislabelled++;
    {
        int64_t e = u->raw[0] - u->block_index * HOP_N;
        if (e < 0) e = -e;
        if (e > u->worst_err) u->worst_err = e;
    }

    for (int j = 0; j < FFT_N; j++) {
        if (u->raw[j] != u->raw[0] + j) { u->discontiguous++; break; }
    }
    u->last_block_start = u->raw[0];
    u->last_block_index = u->block_index;
    u->block_index++;

    if (TAIL_N > 0) {
        memmove(u->raw, u->raw + HOP_N, (size_t)TAIL_N * sizeof(u->raw[0]));
    }
    u->filled = TAIL_N;
}

/** @brief One unit, fed evenly, against one reader variant.
 *  @param variant  READER_NONE, READER_BYTE or READER_GEN.
 *  @param name     What the line should say. */
static void run(int variant, const char *name) {
    unit_t u; unit_init(&u);
    srand(12345);
    int64_t pos = 7777;
    int stall_left = 0;

    for (int step = 0; step < 200000; step++) {
        feed(&u, pos, CHUNK_FRAMES);
        pos += CHUNK_FRAMES;

        if (stall_left > 0) { stall_left--; }
        else {
            reader(&u, variant); reader(&u, variant);
            if ((rand() % 400) == 0) stall_left = 20 + rand() % 20;
        }
    }
    printf("  %-28s blocks %6d  drops %5d  misaligned %6d  mislabelled %6d  "
           "torn %6d  %s\n",
           name, u.blocks, u.drops, u.misaligned, u.mislabelled, u.discontiguous,
           (u.misaligned || u.mislabelled || u.discontiguous)
               ? "<-- BROKEN" : "aligned and labelled");
}

/**
 * @brief A splice mid-stream: audio the timeline accounts for never arrives.
 *
 * @param realign  Whether the caller reports it, as visualiser_realign() does.
 * @param check    Whether the drift check is on.
 * @param name     What the line should say.
 *
 * The pair is the point. Reported, the origin is re-derived at the instant of
 * the event. Unreported but checked, it is re-derived once the error grows
 * past the tolerance. Neither, and every window from then on is mislabelled,
 * for good.
 */
static void splice_case(int realign, int check, const char *name) {
    unit_t u; unit_init(&u);
    int64_t pos = 4096;
    int splices = 0;

    g_drift_check = check;
    for (int step = 0; step < 20000; step++) {
        if (step == 5000 || step == 12000) {
            pos += 6045;
            splices++;
            if (realign) u.align_pending = 1;
        }
        feed(&u, pos, CHUNK_FRAMES);
        pos += CHUNK_FRAMES;
        reader(&u, READER_GEN); reader(&u, READER_GEN);
    }
    g_drift_check = 1;
    printf("  %-28s splices %d  blocks %6d  misaligned %6d  mislabelled %6d  "
           "torn %6d  %s\n",
           name, splices, u.blocks, u.misaligned, u.mislabelled, u.discontiguous,
           (u.misaligned || u.mislabelled || u.discontiguous)
               ? "<-- BROKEN" : "aligned and labelled");
}

/**
 * @brief A hole in delivery: audio the timeline does NOT account for arrives,
 *        or a run of it goes missing.
 *
 * @param check      Whether the drift check is on.
 * @param size       How big the hole is, in frames.
 * @param alternate  Whether to alternate the sign, so the errors could cancel
 *                   -- which is the case a check comparing only against the
 *                   last reading would miss.
 * @param name       What the line should say.
 */
static void hole_case(int check, int size, int alternate, const char *name) {
    unit_t u; unit_init(&u);
    int64_t pos = 4096;
    int holes = 0;

    g_drift_check = check;
    for (int step = 0; step < 20000; step++) {
        if (step > 0 && step % 2500 == 0) {
            pos += (alternate && (holes % 2)) ? -size : size;
            holes++;
        }
        feed(&u, pos, CHUNK_FRAMES);
        pos += CHUNK_FRAMES;
        reader(&u, READER_GEN); reader(&u, READER_GEN);
    }
    g_drift_check = 1;

    printf("  %-28s holes %d  found %2d  mislabelled %6d  worst %5lld frames "
           "(%4.1f ms)  %s\n",
           name, holes, u.drifts, u.mislabelled, (long long)u.worst_err,
           u.worst_err * 1000.0 / 44100.0,
           !check ? "nothing bounds it"
                  : (u.worst_err > DRIFT_FRAMES + size ? "<-- UNBOUNDED" : "bounded"));
}

/**
 * @brief TWO units, joining at different times with different clock skew, and
 *        the requirement that they cut identical blocks.
 *
 * The case the whole file exists for. Under LABEL_DUE they must agree exactly,
 * whatever the skew; under LABEL_CLOCK they must not, which is what says the
 * test can tell the difference.
 *
 * @param mode     LABEL_CLOCK or LABEL_DUE.
 * @param name     What the line should say.
 * @param skew_ms  How far apart the two units' own clocks are.
 */
static void two_units(int mode, const char *name, int skew_ms) {
    unit_t a, b; unit_init(&a); unit_init(&b);
    int skew = skew_ms * 44100 / 1000;
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

/**
 * @brief Run every case and report.
 * @return 0 if all held, 1 otherwise, so `make check` fails the build.
 */
int main(void) {
    printf("block alignment across dropped feeds:\n");
    run(READER_NONE, "feed-side discard only");
    run(READER_BYTE, "reader watches byte index");
    run(READER_GEN,  "reader watches generation");

    printf("\nsplice at a track boundary:\n");
    splice_case(0, 0, "spliced, nothing notices");
    splice_case(0, 1, "spliced, drift check only");
    splice_case(1, 1, "spliced, re-aligned");

    printf("\naudio gained or lost with nobody reporting it:\n");
    hole_case(0, 1500, 1, "1500 frames, no check");
    hole_case(1, 1500, 1, "1500 frames, drift check");

    hole_case(0, 60, 0, "60 frames, no check");
    hole_case(1, 60, 0, "60 frames, drift check");

    printf("\ntwo units, same audio, feed calls a few ms apart:\n");
    two_units(LABEL_CLOCK, "labelled by local clock", 3);
    two_units(LABEL_CLOCK, "labelled by local clock", 5);
    two_units(LABEL_DUE,   "labelled by scheduled time", 3);
    two_units(LABEL_DUE,   "labelled by scheduled time", 5);

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
           "mislabelled %6d  torn %6d\n",
           u.blocks, u.drops, u.misaligned, u.mislabelled, u.discontiguous);
    if (u.drops < 50)    { printf("FAIL: too few drops to exercise the fix\n"); return 1; }
    if (u.misaligned)    { printf("FAIL: blocks cut at the wrong positions\n"); return 1; }
    if (u.mislabelled)   { printf("FAIL: blocks carry the wrong index\n"); return 1; }
    if (u.discontiguous) { printf("FAIL: window content is not contiguous audio\n"); return 1; }

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
        if (s.discontiguous) { printf("FAIL: splice tore the window content\n"); return 1; }
    }

    {
        unit_t h; unit_init(&h);
        int64_t p = 4096;
        int holes = 0;
        for (int step = 0; step < 20000; step++) {
            if (step > 0 && step % 2500 == 0) {
                p += (holes % 2) ? -1500 : 1500;
                holes++;
            }
            feed(&h, p, CHUNK_FRAMES);
            p += CHUNK_FRAMES;
            reader(&h, READER_GEN); reader(&h, READER_GEN);
        }
        if (holes < 5)         { printf("FAIL: too few holes to exercise the check\n"); return 1; }
        if (h.drifts < holes)  { printf("FAIL: %d holes, only %d found\n", holes, h.drifts); return 1; }
        if (h.misaligned)      { printf("FAIL: unreported hole left the boundaries offset\n"); return 1; }
        if (h.mislabelled)     { printf("FAIL: unreported hole left the labels offset\n"); return 1; }
        if (h.discontiguous)   { printf("FAIL: unreported hole tore the window content\n"); return 1; }
    }

    {
        unit_t h; unit_init(&h);
        int64_t p = 4096;
        const int size = 60;
        int holes = 0;
        for (int step = 0; step < 40000; step++) {
            if (step > 0 && step % 500 == 0) { p += size; holes++; }
            feed(&h, p, CHUNK_FRAMES);
            p += CHUNK_FRAMES;
            reader(&h, READER_GEN); reader(&h, READER_GEN);
        }
        if (holes < 50) { printf("FAIL: too few holes to accumulate\n"); return 1; }
        if (h.drifts < 5) { printf("FAIL: %d small holes never tripped the check\n", holes); return 1; }
        if (h.worst_err > DRIFT_FRAMES + size) {
            printf("FAIL: label error reached %lld frames, bound is %d\n",
                   (long long)h.worst_err, DRIFT_FRAMES + size);
            return 1;
        }
    }

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
