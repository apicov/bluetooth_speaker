
#include "ml_arena.h"

#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "mlarena";

#define ARENA_ALIGN 16

bool ml_arena_take(ml_arena_t *a, size_t bytes, const char *name)
{
    if (!a || bytes == 0) {
        return false;
    }
    memset(a, 0, sizeof(*a));
    if (!name) {
        name = "model";
    }

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
