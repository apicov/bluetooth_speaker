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

/*
 * Offset step between consecutive probes that means the master's clock changed
 * origin rather than drifted -- see sync_est_add(). Drift moves it by a few
 * microseconds per probe and asymmetry by a few milliseconds; a second is
 * neither.
 */
#define SYNC_STEP_US     1000000

typedef enum {
    MSG_TIME_REQ = 1,   /* satellite -> master */
    MSG_TIME_RSP = 2,   /* master -> satellite, unicast */
    MSG_BLINK    = 3,   /* sync_test harness only, multicast */
    MSG_AUDIO    = 4,   /* master -> listeners */
    MSG_META     = 5,   /* master -> listeners, track metadata */
    MSG_SPLICE   = 6,   /* satellite -> master, what it corrected at a boundary */
    MSG_TSF      = 7,   /* master -> satellite, measurement only, see tsf_msg_t */
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
 * The master's 802.11 TSF against its own clock. MEASUREMENT ONLY -- nothing
 * reads it but a log line, and the probe estimator continues to drive every
 * anchor, offset and splice.
 *
 * TSF is the WiFi MAC's own microsecond counter. The AP maintains it, every
 * beacon carries it, and each associated station's MAC hardware timestamps the
 * beacon on arrival and slaves its local copy to it. That hardware timestamp is
 * what real PTP relies on and what a software stamp either side of a sendto()
 * cannot provide -- everything between "read the clock" and "the frame left"
 * lands in the error budget, and that path asymmetry is the estimator's floor
 * (see docs/clock-sync.md §2).
 *
 * With TSF there is no round trip to be asymmetric. Each unit relates its OWN
 * TSF to its OWN esp_timer, and since both TSFs track the same AP counter:
 *
 *     offset = (master_local - master_tsf) - (sat_local - sat_tsf)
 *
 * The two reads on each side are not atomic, so a few microseconds of skew is
 * inherent. That is far below what this is trying to distinguish.
 *
 * Zero means the interface is not associated or has not yet seen a beacon; the
 * sender skips it and the receiver ignores it.
 */
typedef struct __attribute__((packed)) {
    uint8_t type;      /* MSG_TSF */
    int64_t tsf;       /* esp_wifi_get_tsf_time(WIFI_IF_AP) */
    int64_t local;     /* esp_timer_get_time(), read adjacently */
} tsf_msg_t;

/*
 * What a satellite corrected at a track boundary, reported so the master can
 * print how far apart the units had drifted.
 *
 * Every unit splices by its OWN phase error against the same published
 * timeline, so the difference between two units' corrections is how far apart
 * they had come to be. Both are measured at the boundary, which is the one
 * instant that recurs identically in every track -- unlike a reading taken
 * wherever a log window happened to fall, which depends on how long since the
 * last boundary reset it.
 *
 * The marker GPIO answers the same question physically, and better, because it
 * sees things no software reading can. It is a bench instrument: two boards, a
 * wire and a common ground. This works over the WiFi that is there anyway, for
 * every satellite rather than the one that happens to be wired.
 *
 * Sent from the probe task, not from playback: a sendto() in the audio path is
 * exactly the kind of thing that costs a buffer. Up to PROBE_PERIOD_MS late,
 * which against track-length intervals is nothing.
 */
typedef struct __attribute__((packed)) {
    uint8_t type;         /* MSG_SPLICE */
    int32_t applied_us;   /* + = skipped content (was late), - = inserted silence */
    int32_t phase_us;     /* the phase error it was correcting */
} splice_msg_t;

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

/*
 * Smoothed phase error each unit tolerates before retuning its output clock.
 *
 * Shared, because it is not a per-unit preference: every unit deadbands around
 * its own reading of the same timeline, so the worst case between any two of
 * them is twice this, whatever the servos report individually.
 *
 * Bounded below by what a RETUNE costs, which is now measured rather than
 * guessed. On the bench, forcing same-rate retunes and reading the phase either
 * side: the channel is down 1.8 to 5.8 ms and the phase step is 1.1 to 6.9 ms,
 * mean 3.3 ms against a mean outage of 3.1 ms. The step IS the outage, to
 * within the several ms the phase wanders on its own.
 *
 * This was briefly raised to 20000 on the theory that frequent retunes were
 * what made the strips drift. They were not. A retune used to cost up to 50 ms
 * because the satellite's playback task ran flat out while the channel was down
 * -- see the `retuning` guard in satellite/main/main.c -- and once that was
 * fixed the cost fell to the outage. 7000 is what the build that measured well
 * actually had.
 *
 * The floor is the clock: retuning happens in whole Hz, 22.7 ppm at 44.1 kHz,
 * so ~2.3 ms of phase is the smallest correction expressible at 44.1 kHz.
 */
#define PHASE_DEADBAND_US 7000


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
