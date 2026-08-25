
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ML_ARENA_NONE = 0,
    ML_ARENA_PSRAM,
    ML_ARENA_INTERNAL,
} ml_arena_where_t;

typedef struct {
    uint8_t         *base;
    size_t           bytes;
    ml_arena_where_t where;
} ml_arena_t;

bool ml_arena_take(ml_arena_t *a, size_t bytes, const char *name);

void ml_arena_give(ml_arena_t *a);

const char *ml_arena_where(const ml_arena_t *a);

#ifdef __cplusplus
}
#endif
