
#pragma once

#include <stdint.h>

#define SBC_LINK_MAX_PAYLOAD 2048

typedef enum {
    LINK_KIND_SBC  = 0,
    LINK_KIND_META = 1,
    LINK_KIND_VOL  = 2,
} link_kind_t;

#define SBC_LINK_SPI_HZ CONFIG_DANCEFLOOR_SBC_LINK_SPI_HZ

typedef struct __attribute__((packed)) {
    uint8_t  kind;
    uint8_t  rsv;
    uint16_t len;
    uint32_t seq;
    uint16_t crc;
    uint16_t rsv2;
} spi_link_hdr_t;

#define SBC_LINK_FRAME_BYTES (sizeof(spi_link_hdr_t) + SBC_LINK_MAX_PAYLOAD)

uint16_t sbc_link_crc16(const void *hdr, const void *payload, uint16_t len);

#define META_TEXT_LEN 64

typedef struct __attribute__((packed)) {
    uint32_t track_id;
    char     title[META_TEXT_LEN];
    char     artist[META_TEXT_LEN];
    char     album[META_TEXT_LEN];
} link_meta_t;

typedef struct __attribute__((packed)) {
    uint8_t volume;
} link_vol_t;

#define AUDIO_VOL_MAX 127
