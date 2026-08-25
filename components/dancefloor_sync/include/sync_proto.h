
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sbc_link.h"

#define SYNC_PORT        5001

#define SYNC_WINDOW      10
#define SYNC_MIN_SAMPLES 3

#define SYNC_STEP_US     1000000

typedef enum {
    MSG_TIME_REQ = 1,
    MSG_TIME_RSP = 2,

    MSG_AUDIO    = 4,
    MSG_META     = 5,
    MSG_SPLICE   = 6,
    MSG_TSF      = 7,
    MSG_FRAME    = 8,
    MSG_LOG      = 9,
    MSG_HEALTH   = 10,
    MSG_LOG_SUB  = 11,

    MSG_VOL      = 13,

    MSG_AUDIO_FEC = 14,
} msg_type_t;

#define AUDIO_FRAMES    256
#define AUDIO_CHANNELS  2
#define AUDIO_CHUNK_BYTES (AUDIO_FRAMES * AUDIO_CHANNELS * (int)sizeof(int16_t))

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint32_t seq;
    int64_t t1;
    int64_t t2;
    int64_t t3;
} time_msg_t;

typedef struct __attribute__((packed)) {
    uint8_t type;
    int64_t tsf;
    int64_t local;
} tsf_msg_t;

typedef struct __attribute__((packed)) {
    uint8_t type;
    int32_t applied_us;
    int32_t phase_us;

    int32_t applied_alt_us;
} splice_msg_t;

#define MARKER_EVERY_PKTS 100
#define MARKER_PULSE_US   200

#define PHASE_DEADBAND_US 7000

typedef enum {

    AUDIO_FMT_SBC = 1,
} audio_fmt_t;

#define AUDIO_MAX_PAYLOAD 2048

typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint8_t  format;
    uint8_t  marker;

    uint8_t  restart;
    uint16_t payload_len;
    uint32_t seq;
    uint32_t sample_rate;
    uint32_t frames;
    int64_t  play_at;
    uint8_t  payload[AUDIO_MAX_PAYLOAD];
} audio_msg_t;

#define FRAME_PAYLOAD_MAX 384

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint8_t len;
    uint8_t count;
    uint8_t payload[FRAME_PAYLOAD_MAX];
} frame_msg_t;

#define FRAME_MSG_BYTES(n) (sizeof(frame_msg_t) - FRAME_PAYLOAD_MAX + (n))

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint8_t payload[196];
} meta_msg_t;

_Static_assert(sizeof(link_meta_t) <= 196, "link_meta_t outgrew meta_msg_t.payload");

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint8_t volume;
} vol_msg_t;

#define LOG_ROLE_HUB 0
#define LOG_ROLE_SAT 1

#define LOG_TAG_MAX   16
#define LOG_MSG_MAX   192

typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint8_t  level;
    uint8_t  role;
    uint8_t  tag_len;
    uint16_t msg_len;
    uint32_t seq;
    uint32_t src_ip;
    char     tag[LOG_TAG_MAX];
    char     msg[LOG_MSG_MAX];
} log_msg_t;

#define LOG_MSG_BYTES(m) (sizeof(log_msg_t) - LOG_MSG_MAX + (size_t)(m))

_Static_assert(sizeof(log_msg_t) <= 1500, "log_msg_t ceiling exceeds the MTU");

typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint8_t  role;
    uint8_t  clock_src;
    uint8_t  _rsv;
    uint32_t seq;
    uint32_t src_ip;
    uint64_t uptime_s;
    uint32_t heap_cur;
    uint32_t heap_min;
    uint32_t heap_win;
    uint32_t heap_largest;
    uint32_t hw_play;
    uint32_t hw_mon;

    uint32_t underruns;
    uint32_t reanchors_or_restarts;
    uint32_t splices;
    uint32_t retunes;
    uint32_t retunes_refused;
    uint32_t gaps_or_sta_left;
    uint32_t wifi_drops_or_oversize;
    uint32_t alloc_fail;
    uint32_t phase_drop;
    uint32_t short_reads;
    uint32_t short_frames;
    uint32_t ring_full_or_sta_dropped;
    uint32_t upgrades_or_sta_nolease;
    uint32_t anchors_refused_or_timeout;

    uint32_t log_dropped;
    uint32_t log_no_dest;
} health_msg_t;

#define LOG_SUB_MAGIC 0x4C4F4731u
typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint32_t magic;
} log_sub_msg_t;

#define AUDIO_MSG_BYTES(n) (sizeof(audio_msg_t) - AUDIO_MAX_PAYLOAD + (n))

#define AUDIO_UDP_MTU 1472

#define AUDIO_TX_PAYLOAD_MTU_MAX (AUDIO_UDP_MTU - AUDIO_MSG_BYTES(0))

#define AUDIO_FEC_HDR_BYTES 8

#define AUDIO_FEC_PAYLOAD_MAX (AUDIO_UDP_MTU - AUDIO_FEC_HDR_BYTES - AUDIO_MSG_BYTES(0))
#define AUDIO_FEC_CODEWORD_MAX AUDIO_MSG_BYTES(AUDIO_FEC_PAYLOAD_MAX)

#define AUDIO_FEC_K_MAX 8

typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint8_t  count;
    uint16_t span;
    uint32_t base_seq;
    uint8_t  parity[AUDIO_FEC_CODEWORD_MAX];
} audio_fec_msg_t;

#define AUDIO_FEC_MSG_BYTES(n) (sizeof(audio_fec_msg_t) - AUDIO_FEC_CODEWORD_MAX + (n))

_Static_assert(AUDIO_FEC_MSG_BYTES(0) == AUDIO_FEC_HDR_BYTES,
               "AUDIO_FEC_HDR_BYTES and audio_fec_msg_t's header disagree -- "
               "AUDIO_FEC_PAYLOAD_MAX is computed from the former");
_Static_assert(AUDIO_FEC_MSG_BYTES(AUDIO_FEC_CODEWORD_MAX) == AUDIO_UDP_MTU,
               "a full parity datagram would fragment");
_Static_assert(AUDIO_FEC_K_MAX <= 8, "the receiver's seen-mask is one byte");

bool audio_fec_xor_in(uint8_t *acc, uint16_t *span, const audio_msg_t *m);

bool audio_fec_extract(const uint8_t *acc, uint16_t span, uint32_t want_seq,
                       audio_msg_t *out);

_Static_assert(FRAME_MSG_BYTES(FRAME_PAYLOAD_MAX) <= AUDIO_UDP_MTU,
               "a full frame batch would fragment");

typedef struct {
    int64_t offset[SYNC_WINDOW];
    int64_t delay[SYNC_WINDOW];
    int count;
    int next;
} sync_est_t;

void sync_est_init(sync_est_t *e);

void sync_est_add(sync_est_t *e, int64_t t1, int64_t t2, int64_t t3, int64_t t4);

bool sync_est_offset(const sync_est_t *e, int64_t *offset_out);

static inline bool sync_est_settled(const sync_est_t *e)
{
    return e->count >= SYNC_WINDOW;
}

static inline int64_t sync_to_local(int64_t master_us, int64_t offset)
{
    return master_us - offset;
}

#define SYNC_PHASE_HIST 9
#define SYNC_PHASE_MIN  5

typedef struct {
    int32_t v[SYNC_PHASE_HIST];
    uint8_t next;
    uint8_t count;
} sync_phase_hist_t;

void sync_phase_reset(sync_phase_hist_t *h);

void sync_phase_push(sync_phase_hist_t *h, int32_t us);

bool sync_phase_median(const sync_phase_hist_t *h, int32_t *out);
