/*
 * Clock synchronisation between dancefloor units.
 *
 * Deliberately free of ESP-IDF dependencies so the estimator can be unit-tested
 * on the host -- this is the part of the system most likely to be subtly wrong,
 * and hardware bring-up is a bad place to discover that.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sbc_link.h"

#define SYNC_PORT        5001

/* Used only by the M4 harness in sync_test/, which still multicasts its blink
 * announcements. The audio path unicasts to registered listeners and touches
 * neither this nor MSG_BLINK -- see hub/main/streamer.c. */
#define SYNC_MCAST_ADDR  "239.12.34.56"
#define SYNC_WINDOW      10      /* probes retained for the median */
#define SYNC_MIN_SAMPLES 3       /* below this the estimate is not trusted */

typedef enum {
    MSG_TIME_REQ = 1,   /* satellite -> master */
    MSG_TIME_RSP = 2,   /* master -> satellite, unicast */
    MSG_BLINK    = 3,   /* sync_test harness only, multicast */
    MSG_AUDIO    = 4,   /* master -> listeners */
    MSG_META     = 5,   /* master -> listeners, track metadata */
} msg_type_t;

/*
 * Audio chunking.
 *
 * 256 frames is 5.8 ms at 44.1 kHz and puts the packet at 1041 bytes, safely
 * under a 1500-byte MTU so nothing fragments. Larger chunks would fragment;
 * smaller ones spend more of the link on headers.
 */
#define AUDIO_FRAMES    256
#define AUDIO_CHANNELS  2
#define AUDIO_CHUNK_BYTES (AUDIO_FRAMES * AUDIO_CHANNELS * (int)sizeof(int16_t))

/* Wire format. Fixed-width and packed: both ends are xtensa here, but the host
 * test builds this too, and an accidental layout change would be invisible. */
typedef struct __attribute__((packed)) {
    uint8_t type;
    uint32_t seq;
    int64_t t1;   /* satellite transmit, satellite clock */
    int64_t t2;   /* master receive,    master clock    */
    int64_t t3;   /* master transmit,   master clock    */
} time_msg_t;

typedef struct __attribute__((packed)) {
    uint8_t type;
    int64_t play_at;  /* master clock, microseconds */
} blink_msg_t;

/*
 * Audio alignment measurement.
 *
 * Every unit pulses a GPIO at the moment it plays the sample whose *master-clock*
 * time crosses a multiple of MARKER_PERIOD_US. If two units are in sync their
 * pulses coincide; the gap between them is the sync error, measured on the audio
 * itself rather than inferred from clock estimates.
 *
 * Master-clock time is used rather than a sample count from each unit's own
 * start, so a satellite that joins late still marks the same instants.
 *
 * The pulse fires when a chunk is written to the output, not when it reaches the
 * DAC, so it sits ahead of the sound by the DMA depth. That offset is identical
 * on every unit and cancels in the comparison.
 */
/*
 * Marked by CONTENT, not by time: every unit pulses when the audio from a
 * tagged packet reaches its output, so both are marking the same sample.
 *
 * The first attempt derived the instant from samples_played / sample_rate using
 * the nominal 44100. Each unit actually plays at whatever its drift servo last
 * set, so a 0.4% difference accumulated ~8 ms between markers and the two units'
 * pulses slid apart independently -- the measurement reported servo divergence
 * rather than audio misalignment.
 *
 * Tying the marker to a packet removes rate from the question entirely: if two
 * units emit the same sample at the same instant they are in sync, whatever
 * their clocks are doing.
 *
 * Packets arrive ~50/s, so every 100th is about 2 s.
 */
#define MARKER_EVERY_PKTS 100
#define MARKER_PULSE_US   200

typedef enum {
    AUDIO_FMT_PCM = 0,   /* interleaved 16-bit stereo */
    AUDIO_FMT_SBC = 1,   /* one or more back-to-back SBC frames */
} audio_fmt_t;

#define AUDIO_MAX_PAYLOAD 1024

/*
 * One chunk of audio. `play_at` is the master-clock instant the *first* sample
 * should hit the DAC; the satellite converts it with sync_to_local().
 *
 * Carries SBC rather than PCM. Decoded audio is 1.4 Mbps in 172 packets/s;
 * the SBC it was decoded from is ~330 kbps in 50 packets/s. Measured on this
 * hardware, PCM cost ~7% of packets rejected by a full WiFi TX queue plus ~13%
 * lost on the air at 24% duty cycle. Quartering both fixes both, and costs
 * nothing in quality -- it is the same SBC, decoded at the far end instead.
 *
 * `seq` matters as much as the timestamp: UDP loses packets, and a gap must be
 * filled with the right amount of silence or every later sample plays early and
 * the whole stream slides.
 *
 * Only the first `payload_len` bytes are transmitted, so packets are as small as
 * the audio allows.
 */
typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint8_t  format;        /* audio_fmt_t */
    uint8_t  marker;        /* 1 = pulse the sync GPIO when this audio plays */
    /*
     * 1 = a track boundary starts here. When playback reaches this audio, snap
     * the accumulated phase error to zero by skipping or inserting samples.
     *
     * The splice must happen HERE, not when the track-change notification
     * arrives: at that moment the buffer still holds ~200 ms of the previous
     * track, and correcting immediately would cut its ending. Waiting until
     * playback arrives puts the splice exactly at the boundary, where a track
     * change makes it inaudible.
     */
    uint8_t  restart;
    uint16_t payload_len;
    uint32_t seq;
    uint32_t sample_rate;
    uint32_t frames;        /* PCM frames this payload decodes to */
    int64_t  play_at;
    uint8_t  payload[AUDIO_MAX_PAYLOAD];
} audio_msg_t;

/* Track metadata forwarded from the bridge. Carries link_meta_t verbatim, so
 * there is exactly one definition of the fields (see sbc_link.h). */
typedef struct __attribute__((packed)) {
    uint8_t type;           /* MSG_META */
    uint8_t payload[196];   /* sizeof(link_meta_t) */
} meta_msg_t;

/* The two definitions must not drift apart; sbc_link.h owns the fields. */
_Static_assert(sizeof(link_meta_t) <= 196, "link_meta_t outgrew meta_msg_t.payload");

/* Bytes to send for a payload of `n`. */
#define AUDIO_MSG_BYTES(n) (sizeof(audio_msg_t) - AUDIO_MAX_PAYLOAD + (n))

typedef struct {
    int64_t offset[SYNC_WINDOW];
    int64_t delay[SYNC_WINDOW];   /* round-trip time, for minimum-delay selection */
    int count;
    int next;
} sync_est_t;

void sync_est_init(sync_est_t *e);

/*
 * Fold one completed probe exchange into the estimate.
 * t4 is the satellite receive time on the satellite clock.
 *
 * offset = ((t2 - t1) + (t3 - t4)) / 2, the standard NTP estimator. It assumes
 * the two path delays are equal; they are not, and that asymmetry is the error
 * floor. The median across probes is what keeps WiFi retries from wrecking it.
 */
void sync_est_add(sync_est_t *e, int64_t t1, int64_t t2, int64_t t3, int64_t t4);

/*
 * Best offset estimate, in microseconds, such that master_time = local_time + offset.
 * False until SYNC_MIN_SAMPLES probes have landed.
 *
 * Selects the offset from the probe with the *lowest* round-trip time rather
 * than taking a median. A fast round trip had little queuing in either
 * direction, so its paths were closer to symmetric -- and asymmetry is the only
 * error the estimator cannot see. Measured RTT on a SoftAP link swings between
 * about 5 ms and 14 ms, so the choice of sample matters more than averaging
 * across all of them. This is what PTP does.
 *
 * Ties favour the newer sample, since an old one has had time to drift.
 */
bool sync_est_offset(const sync_est_t *e, int64_t *offset_out);

/*
 * True once a full window of probes has landed, i.e. the estimate has had a
 * chance to find a genuinely low-RTT sample rather than the best of three.
 *
 * Matters for anchoring playback: `play_at` is consulted once, at stream start,
 * so an offset error at that instant is permanent. Waiting the extra couple of
 * seconds costs nothing and removes a whole class of "one speaker is slightly
 * out" that would be untraceable afterwards.
 */
static inline bool sync_est_settled(const sync_est_t *e)
{
    return e->count >= SYNC_WINDOW;
}

/* Master clock -> local clock. */
static inline int64_t sync_to_local(int64_t master_us, int64_t offset)
{
    return master_us - offset;
}
