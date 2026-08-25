/**
 * @file ml_arena.c
 * @brief The tensor arena allocator. ml_arena.h has the contract and the
 *        argument for why this asks for PSRAM and says which heap it got.
 */
#include "ml_arena.h"

#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

/** @brief Log tag. */
static const char *TAG = "mlarena";

/** @brief TFLM aligns its own tensors within the arena but assumes the arena
 *         itself is aligned; an unaligned base silently costs the head of
 *         it. */
#define ARENA_ALIGN 16

/* Declared in ml_arena.h, like the two below it. */
bool ml_arena_take(ml_arena_t *a, size_t bytes, const char *name)
{
    if (!a || bytes == 0) {
        return false;
    }
    memset(a, 0, sizeof(*a));
    if (!name) {
        name = "model";
    }

    /*
     * PSRAM first, and only by explicit capability.
     *
     * With capability-only PSRAM allocation this is the only way to reach it,
     * which is exactly the property that lets PSRAM be enabled at all without
     * moving anything else. On a board with no PSRAM the capability simply
     * cannot be satisfied and this returns NULL, which is the intended path
     * and not an error -- hence no log here.
     */
#ifdef MALLOC_CAP_SPIRAM
    a->base = heap_caps_aligned_alloc(ARENA_ALIGN, bytes,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (a->base) {
        a->bytes = bytes;
        a->where = ML_ARENA_PSRAM;
        ESP_LOGI(TAG, "%s: %u B arena in PSRAM (%u B free there)",
                 name, (unsigned)bytes,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        return true;
    }
#endif

    /*
     * Internal SRAM, and said loudly.
     *
     * This is a satellite's only option and it is the one that hurts: the same
     * heap the ring, the WiFi buffers and every stack come from, on a board
     * with tens of kilobytes free while analysing. A model sized for the hub
     * landing here does not fail at the allocation -- it fails later, as an
     * audio dropout nobody connects to the model.
     *
     * So the free figures are printed beside the request. If the margin looks
     * thin here it IS thin, and the answer is a smaller model.
     */
    a->base = heap_caps_aligned_alloc(ARENA_ALIGN, bytes,
                                      MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (a->base) {
        a->bytes = bytes;
        a->where = ML_ARENA_INTERNAL;
        ESP_LOGW(TAG, "%s: %u B arena in INTERNAL SRAM -- no PSRAM on this board. "
                      "%u B free, largest block %u B",
                 name, (unsigned)bytes,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        return true;
    }

    ESP_LOGE(TAG, "%s: no %u B arena anywhere -- largest internal block is %u B. "
                  "This analyser will not run.",
             name, (unsigned)bytes,
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    return false;
}

void ml_arena_give(ml_arena_t *a)
{
    if (!a || !a->base) {
        return;
    }
    heap_caps_free(a->base);
    memset(a, 0, sizeof(*a));
}

const char *ml_arena_where(const ml_arena_t *a)
{
    if (!a) return "none";
    switch (a->where) {
    case ML_ARENA_PSRAM:    return "psram";
    case ML_ARENA_INTERNAL: return "internal";
    default:                return "none";
    }
}
