/*
 * Where a model's working memory comes from.
 *
 * A TFLM tensor arena is the one buffer in this project that is large, touched
 * only by a low-priority task on the other core, and useless to shrink -- the
 * interpreter needs what it needs. That combination is what makes it the only
 * thing here worth putting in PSRAM, and it is why hub_s3 now enables PSRAM
 * with CONFIG_SPIRAM_USE_CAPS_ALLOC: ordinary malloc still never returns it, so
 * the ring, the DMA buffers and every stack stay exactly where they were
 * measured.
 *
 * The two boards are not the same and the code must not pretend they are:
 *
 *   hub_s3, and a future S3 satellite    8 MB of octal PSRAM
 *   classic ESP32 satellite              none, ever, and ~52 kB free
 *
 * So this asks for PSRAM, takes internal SRAM if there is none, and SAYS WHICH.
 * Falling back silently is the failure worth avoiding -- a model sized for the
 * hub that quietly takes 200 kB of a satellite's internal heap does not fail at
 * the allocation, it fails an hour later as an audio dropout nobody connects to
 * it.
 *
 * Plain C so the resampler's neighbours can all be built the same way.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ML_ARENA_NONE = 0,
    ML_ARENA_PSRAM,        /* external, cache-backed */
    ML_ARENA_INTERNAL,     /* the same SRAM everything else competes for */
} ml_arena_where_t;

typedef struct {
    uint8_t         *base;
    size_t           bytes;
    ml_arena_where_t where;
} ml_arena_t;

/*
 * Take `bytes` for a model, preferring PSRAM.
 *
 * Returns false and leaves *a zeroed if neither heap can supply it. A model
 * that cannot have its arena must not run at all -- an Analyser whose init()
 * returns false is skipped and reported, which is the whole reason init() can
 * fail.
 *
 * `name` is only used in the log line, so a floor with two models says which
 * one took what.
 *
 * 16-byte aligned: TFLM wants its arena aligned and will silently waste the
 * head of an unaligned one.
 */
bool ml_arena_take(ml_arena_t *a, size_t bytes, const char *name);

/* Give it back. Safe on a zeroed or already-freed arena. */
void ml_arena_give(ml_arena_t *a);

/* "psram" / "internal" / "none", for logs and the HEALTH line. */
const char *ml_arena_where(const ml_arena_t *a);

#ifdef __cplusplus
}
#endif
