/**
 * @file ml_arena.h
 * @brief Where a model's working memory comes from.
 *
 * A TFLM tensor arena is the one buffer in this project that is large, touched
 * only by a low-priority task on the other core, and useless to shrink -- the
 * interpreter needs what it needs. That combination is what makes it the only
 * thing here worth putting in PSRAM, and why the hub enables PSRAM with
 * capability-only allocation: ordinary malloc still never returns it, so the
 * ring, the DMA buffers and every stack stay exactly where they were measured.
 *
 * The two boards are not the same and the code must not pretend they are. The
 * S3 hub has megabytes of PSRAM; a classic satellite has none, ever, and tens
 * of kilobytes of internal heap free while analysing. So this asks for PSRAM,
 * takes internal SRAM if there is none, and SAYS WHICH. Falling back silently
 * is the failure worth avoiding: a model sized for the hub that quietly takes
 * a large share of a satellite's internal heap does not fail at the
 * allocation, it fails an hour later as an audio dropout nobody connects to
 * it.
 *
 * Plain C, so the resampler's neighbours can all be built the same way.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Which heap an arena came from. */
typedef enum {
    ML_ARENA_NONE = 0,     /**< No arena; the analyser must not run. */
    ML_ARENA_PSRAM,        /**< External, cache-backed. */
    ML_ARENA_INTERNAL,     /**< The same SRAM everything else competes for. */
} ml_arena_where_t;

/** @brief One model's arena. */
typedef struct {
    uint8_t         *base;    /**< 16-byte aligned; NULL if none. */
    size_t           bytes;   /**< As requested. */
    ml_arena_where_t where;   /**< Which heap it came from. */
} ml_arena_t;

/**
 * @brief Take an arena for a model, preferring PSRAM.
 *
 * 16-byte aligned: TFLM wants its arena aligned and will silently waste the
 * head of an unaligned one.
 *
 * @param[out] a  The arena; zeroed on failure.
 * @param bytes   How much the interpreter needs.
 * @param name    Used only in the log line, so a floor with two models says
 *                which one took what. NULL is accepted.
 * @return false if neither heap can supply it. A model that cannot have its
 *         arena must not run at all -- an df::Analyser whose init() returns
 *         false is skipped and reported, which is the whole reason init() can
 *         fail.
 */
bool ml_arena_take(ml_arena_t *a, size_t bytes, const char *name);

/**
 * @brief Give an arena back. Safe on a zeroed or already-freed one.
 * @param a  The arena.
 */
void ml_arena_give(ml_arena_t *a);

/**
 * @brief Which heap it came from, as a word for logs and the health line.
 * @param a  The arena, or NULL.
 * @return "psram", "internal" or "none".
 */
const char *ml_arena_where(const ml_arena_t *a);

#ifdef __cplusplus
}
#endif
