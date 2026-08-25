/**
 * @file sync_proto.h
 * @brief The wire format every dancefloor unit speaks, and the clock estimator
 *        behind it.
 *
 * Deliberately free of ESP-IDF dependencies so the estimator can be unit-tested
 * on the host: this is the part of the system most likely to be subtly wrong,
 * and hardware bring-up is a bad place to discover that. test_sync_proto.c is
 * that test.
 *
 * Every wire struct is fixed-width and packed. Both firmware ends are the same
 * architecture, but the host test builds these too, and an accidental layout
 * change would otherwise be invisible.
 *
 * Message numbers are burned rather than reused when a message is retired. A
 * board still flashed with an older firmware speaks a protocol this one no
 * longer does, and a type that is silently reinterpreted as something else is
 * worse than one that is simply unknown -- an unknown type is already ignored
 * by every dispatch.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sbc_link.h"

/** @brief The one UDP port everything here is carried on. */
#define SYNC_PORT        5001

/** @brief Probes retained for the offset selection. */
#define SYNC_WINDOW      10
/** @brief Below this many the estimate is not trusted at all. */
#define SYNC_MIN_SAMPLES 3

/**
 * @brief Offset step between consecutive probes that means the master's clock
 *        changed ORIGIN rather than drifted.
 *
 * Drift moves the offset by a few microseconds between probes and path
 * asymmetry by a few milliseconds; a whole second is neither. See
 * sync_est_add() for what is done about it.
 */
#define SYNC_STEP_US     1000000

/** @brief What a datagram's first byte says it is. */
typedef enum {
    MSG_TIME_REQ = 1,   /**< Satellite -> master: a probe. */
    MSG_TIME_RSP = 2,   /**< Master -> satellite, unicast: the reply. */
    /* 3 was MSG_BLINK, for an early bring-up harness. Burned; see @file. */
    MSG_AUDIO    = 4,   /**< Master -> listeners: one chunk of audio. */
    MSG_META     = 5,   /**< Master -> listeners: track metadata. */
    MSG_SPLICE   = 6,   /**< Satellite -> master: what it corrected at a boundary. */
    MSG_TSF      = 7,   /**< Master -> satellite, measurement only; see tsf_msg_t. */
    MSG_FRAME    = 8,   /**< Master -> listeners: a batch of analysis frames. */
    MSG_LOG      = 9,   /**< Any WiFi unit -> collector: one formatted log line. */
    MSG_HEALTH   = 10,  /**< Any WiFi unit -> collector: the structured snapshot. */
    MSG_LOG_SUB  = 11,  /**< Collector -> hub: "send logs here"; see log_sub_msg_t. */
    /* 12 was MSG_ML, one analyser's result sent to a unit that was given them
     * rather than computing them. Removed when the analysers became
     * spectrum-fed: a unit receiving MSG_FRAME already has the spectrum, so it
     * runs the model itself and there is nothing left to distribute. Burned. */
    MSG_VOL      = 13,  /**< Master -> listeners: playback volume; see vol_msg_t. */
    /**
     * @brief Master -> listeners: one XOR parity packet per group of audio
     *        packets. See audio_fec_msg_t.
     *
     * Its own type rather than a field on audio_msg_t, because a satellite
     * that predates it must go on reading the audio unchanged and an unknown
     * type is already ignored.
     */
    MSG_AUDIO_FEC = 14,
} msg_type_t;

/**
 * @brief Frames per audio chunk.
 *
 * Sized so the packet stays safely under a 1500-byte MTU and nothing
 * fragments. Larger chunks would fragment; smaller ones spend more of the link
 * on headers.
 */
#define AUDIO_FRAMES    256
/** @brief Interleaved channels in the stream. Fixed: the wire format is stereo
 *         whatever a unit chooses to play (see audio_out.h). */
#define AUDIO_CHANNELS  2
/** @brief One chunk in the RING domain. audio_out.h has the DAC-domain
 *         counterpart and the reason the two are kept apart. */
#define AUDIO_CHUNK_BYTES (AUDIO_FRAMES * AUDIO_CHANNELS * (int)sizeof(int16_t))

/** @brief One round of the clock probe. The satellite fills t1 and stamps t4
 *         on arrival; the master fills t2 and t3. */
typedef struct __attribute__((packed)) {
    uint8_t type;    /**< MSG_TIME_REQ or MSG_TIME_RSP. */
    uint32_t seq;    /**< Echoed back, so a reply can be matched to its probe. */
    int64_t t1;      /**< Satellite transmit, satellite clock. */
    int64_t t2;      /**< Master receive, master clock. */
    int64_t t3;      /**< Master transmit, master clock. */
} time_msg_t;

/**
 * @brief The master's 802.11 TSF against its own clock. MEASUREMENT ONLY --
 *        nothing reads it but a log line, and the probe estimator continues to
 *        drive every anchor, offset and splice.
 *
 * TSF is the WiFi MAC's own microsecond counter. The AP maintains it, every
 * beacon carries it, and each associated station's MAC hardware timestamps the
 * beacon on arrival and slaves its local copy to it. That hardware timestamp
 * is what real PTP relies on and what a software stamp either side of a
 * sendto() cannot provide: everything between "read the clock" and "the frame
 * left" lands in the error budget, and that path asymmetry is the probe
 * estimator's floor.
 *
 * With TSF there is no round trip to be asymmetric. Each unit relates its OWN
 * TSF to its OWN local clock, and since both TSFs track the same AP counter:
 *
 *     offset = (master_local - master_tsf) - (sat_local - sat_tsf)
 *
 * The two reads on each side are not atomic, so a few microseconds of skew is
 * inherent -- far below what this is trying to distinguish.
 */
typedef struct __attribute__((packed)) {
    uint8_t type;      /**< MSG_TSF. */
    /** @brief The AP interface's TSF. Zero means the interface is not
     *         associated or has not yet seen a beacon; the sender skips it and
     *         the receiver ignores it. */
    int64_t tsf;
    int64_t local;     /**< The local clock, read adjacently. */
} tsf_msg_t;

/**
 * @brief What a satellite corrected at a track boundary, reported so the
 *        master can print how far apart the units had drifted.
 *
 * Every unit splices by its OWN phase error against the same published
 * timeline, so the difference between two units' corrections is how far apart
 * they had come to be. Both are measured at the boundary, which is the one
 * instant that recurs identically in every track -- unlike a reading taken
 * wherever a log window happened to fall, which depends on how long it has
 * been since the last boundary reset it.
 *
 * The marker GPIO answers the same question physically, and better, because it
 * sees what no software reading can. It is a bench instrument: two boards, a
 * wire and a common ground. This works over the WiFi that is there anyway, for
 * every satellite rather than the one that happens to be wired.
 *
 * Sent from the probe task, not from playback: a sendto() in the audio path is
 * exactly the kind of thing that costs a transmit buffer. It arrives up to one
 * probe period late, which against track-length intervals is nothing.
 */
typedef struct __attribute__((packed)) {
    uint8_t type;         /**< MSG_SPLICE. */
    int32_t applied_us;   /**< + = skipped content (was late), - = inserted silence. */
    int32_t phase_us;     /**< The phase error it was correcting. */
    /**
     * @brief The correction this unit WOULD have applied on the OTHER
     *        estimator -- currently the raw newest reading, against the median
     *        the splice actually uses.
     *
     * Same units and same clamp as #applied_us, so the two subtract
     * meaningfully; that is the whole reason it is a correction rather than a
     * phase value. Kept so the comparison that chose the median survives the
     * change, and so a revert has something to check itself against.
     *
     * WIRE COMPATIBILITY: only the MEANING of the number has ever flipped --
     * same offset, same width, same clamp -- so a unit on either build parses
     * the other's message. Both firmwares must be reflashed together for the
     * printed comparison to mean anything.
     */
    int32_t applied_alt_us;
} splice_msg_t;

/**
 * @brief Packets between marker pulses, for the bench alignment instrument.
 *
 * Marked by CONTENT, not by time: every unit pulses when the audio from a
 * tagged packet reaches its output, so both are marking the same sample. If
 * two units emit the same sample at the same instant they are in sync,
 * whatever their clocks are doing.
 *
 * Deriving the instant from a sample count and a nominal rate instead does not
 * work, and was tried: each unit plays at whatever its own servo last set, so
 * the two units' pulses slide apart independently and the measurement reports
 * servo divergence rather than audio misalignment.
 */
#define MARKER_EVERY_PKTS 100
/** @brief Pulse width. Long enough to catch on a scope, short enough that the
 *         busy-wait it costs the playback path is negligible. */
#define MARKER_PULSE_US   200

/**
 * @brief Smoothed phase error each unit tolerates before correcting its output
 *        rate.
 *
 * Shared, because it is not a per-unit preference: every unit deadbands around
 * its own reading of the same timeline, so the worst case between any two of
 * them is twice this, whatever the servos report individually.
 *
 * It was once bounded below by what a clock RETUNE costs, since a correction
 * smaller than the disturbance it causes is not worth making. That bound no
 * longer binds: corrections this size are made in software now, by dropping or
 * duplicating one frame at a time, so they cost no channel outage at all and
 * the whole-Hz floor of a clock retune is a property of an actuator this no
 * longer uses. Only a coarse rate MATCH still retunes the clock.
 *
 * The value is kept unchanged all the same, and deliberately: the point of
 * that change was to remove an interruption without moving the sync behaviour,
 * and every cross-unit figure this project has recorded was measured here.
 * Tightening it is now affordable in a way it was not, and it is the obvious
 * next experiment -- but a separate one, with its own flash and its own
 * cross-unit reading.
 */
#define PHASE_DEADBAND_US 7000

/** @brief What audio_msg_t::payload holds. */
typedef enum {
    /* 0 was AUDIO_FMT_PCM, from when the master decoded for everyone and sent
     * samples. Satellites decode their own SBC now, at a quarter of the
     * airtime, so nothing produces it and a satellite rejects any packet not
     * marked SBC. Burned; see @file. */
    AUDIO_FMT_SBC = 1,   /**< One or more back-to-back SBC frames. */
} audio_fmt_t;

/**
 * @brief Ceiling on one audio packet's payload.
 *
 * The same maximum SBC payload as SBC_LINK_MAX_PAYLOAD, on the WiFi hop to the
 * satellites instead of the SPI hop from the bridge. It must stay at least as
 * large or the forwarder would refuse what the link delivered;
 * test_sync_proto.c asserts it does not drift below. Only the first
 * payload_len bytes go on the wire (AUDIO_MSG_BYTES), so the array is a
 * ceiling and not a per-packet cost.
 */
#define AUDIO_MAX_PAYLOAD 2048

/**
 * @brief One chunk of audio, and the timeline it belongs to.
 *
 * Carries SBC rather than PCM. Decoded audio is several times the bitrate in
 * several times the packet count, and on this hardware PCM cost both a share
 * of packets rejected by a full transmit queue and a share lost on the air.
 * Quartering both fixes both and costs nothing in quality -- it is the same
 * SBC, decoded at the far end instead.
 */
typedef struct __attribute__((packed)) {
    uint8_t  type;          /**< MSG_AUDIO. */
    uint8_t  format;        /**< audio_fmt_t. */
    uint8_t  marker;        /**< 1 = pulse the sync GPIO when this audio plays. */
    /**
     * @brief 1 = a track boundary starts here: when playback reaches this
     *        audio, snap the accumulated phase error to zero by skipping or
     *        inserting samples.
     *
     * The splice must happen HERE, not when the track-change notification
     * arrives: at that moment the buffer still holds a lead's worth of the
     * previous track, and correcting immediately would cut its ending. Waiting
     * until playback arrives puts the splice exactly at the boundary, where a
     * track change makes it inaudible.
     */
    uint8_t  restart;
    uint16_t payload_len;   /**< Real bytes of #payload; the rest is not sent. */
    /** @brief Matters as much as the timestamp: UDP loses packets, and a gap
     *         must be filled with the right amount of silence or every later
     *         sample plays early and the whole stream slides. */
    uint32_t seq;
    uint32_t sample_rate;   /**< The stream's rate, as the decoder reports it. */
    uint32_t frames;        /**< PCM frames this payload decodes to. */
    /** @brief The master-clock instant the FIRST sample should reach the DAC.
     *         A satellite converts it with sync_to_local(). */
    int64_t  play_at;
    uint8_t  payload[AUDIO_MAX_PAYLOAD];  /**< See #payload_len. */
} audio_msg_t;

/**
 * @brief Room for a whole beacon's worth of analysis frames.
 *
 * The per-frame size is deliberately not visible here: the LED component does
 * not depend on the protocol and must stay buildable on its own, so this
 * number is written out and checked where the two do meet (clients.c
 * static-asserts the batch against this cap).
 *
 * A frame carries the timeline labels and the detector bands, which is what a
 * receiver actually consumes. It once also carried the quantised spectrum,
 * which only the pluggable analysers read -- so every satellite with them off
 * took those bytes many times a second and threw them away. Dropping it bought
 * AIRTIME, not buffers: a static transmit buffer holds one datagram whatever
 * its size, so the datagram rate and the window each one occupies are exactly
 * as before.
 *
 * The headroom above what one beacon strictly holds is deliberate: the
 * analysis task does not run at a metronomic rate, and a decoder lump hands it
 * several frames at once. The spare absorbs that without the batch overflowing
 * and cutting itself short.
 */
#define FRAME_PAYLOAD_MAX 384

/**
 * @brief Analysis frames, for units that draw what another unit decided.
 *
 * A BATCH, NOT A FRAME, and the reason is the pace rather than the analysis.
 * The hub computes frames far faster than the frame lane may send them --
 * TX_FRAME_PACE_US gates it to one send per beacon, and everything offered in
 * between was dropped. That is a detector running at a fraction of the rate
 * its thresholds were tuned at, since its adaptive window is a frame COUNT,
 * and it is visible from across the field as a hub whose strip follows the
 * music and satellites whose strips lurch.
 *
 * The pace is not the fault and stays: sending faster only occupies transmit
 * buffers the audio needs. What changes is how much rides in the one datagram
 * the beacon releases -- every frame since the last one, so the burst the pace
 * was introduced to prevent still does not form.
 *
 * WHAT IT COSTS is loss granularity: a lost packet is now a whole beacon's
 * worth of consecutive frames rather than one. The receiving detector
 * converges back within its history length, because that history is the only
 * state it has. If it ever reads as a stutter, the fix is to split the batch
 * into a FIXED two or three sub-packets per beacon -- still deterministic, and
 * still nothing like the unpaced lane. TX_FRAME_BATCH in hub.h is the one
 * knob.
 */
typedef struct __attribute__((packed)) {
    uint8_t type;                        /**< MSG_FRAME. */
    /** @brief Bytes per frame; every frame in the batch is this size. Carried
     *         so the two ends can disagree about it and SAY SO instead of
     *         reading past it -- a hub and a satellite on different builds is
     *         the failure this protocol is most likely to meet, and a silently
     *         reinterpreted frame is worse than a refused one. */
    uint8_t len;
    uint8_t count;                       /**< Frames that follow, back to back; >= 1. */
    uint8_t payload[FRAME_PAYLOAD_MAX];  /**< len * count bytes are real. */
} frame_msg_t;

/** @brief On-wire bytes for n TOTAL payload bytes -- len * count, not one
 *         frame. */
#define FRAME_MSG_BYTES(n) (sizeof(frame_msg_t) - FRAME_PAYLOAD_MAX + (n))

/** @brief Track metadata forwarded from the bridge. Carries link_meta_t
 *         verbatim, so there is exactly one definition of the fields; see
 *         sbc_link.h. */
typedef struct __attribute__((packed)) {
    uint8_t type;           /**< MSG_META. */
    uint8_t payload[196];   /**< A link_meta_t, copied in as bytes. */
} meta_msg_t;

/* The two definitions must not drift apart; sbc_link.h owns the fields. */
_Static_assert(sizeof(link_meta_t) <= 196, "link_meta_t outgrew meta_msg_t.payload");

/**
 * @brief Playback volume, hub -> listeners.
 *
 * Applied at each unit's output by audio_volume_write_i32(); see audio_out.h
 * for the taper and why it lives there.
 *
 * ADDRESSED LIKE THE AUDIO, to the same client list the audio walks. That is
 * the requirement rather than an implementation detail: a unit must hear the
 * level exactly when it hears the stream.
 *
 * Sent several times when the phone changes it, pushed once when a satellite
 * is given an address, and repeated on a slow timer regardless. The repeats
 * earn their place because a level is STATE rather than a stream: a unit that
 * missed the one packet carrying it stays wrong until something says so again,
 * and the repeat bounds how long a unit that was never told stays silent (see
 * audio_vol_effective()).
 */
typedef struct __attribute__((packed)) {
    uint8_t type;           /**< MSG_VOL. */
    uint8_t volume;         /**< 0..AUDIO_VOL_MAX, AUDIO_VOL_MAX = unity. */
} vol_msg_t;

/** @brief log_msg_t::role and health_msg_t::role: the hub. */
#define LOG_ROLE_HUB 0
/** @brief ...and a satellite. */
#define LOG_ROLE_SAT 1

/** @brief Ceiling on a captured log tag. */
#define LOG_TAG_MAX   16
/** @brief Ceiling on a captured log line. Only the first msg_len bytes are
 *         sent, so this is not a per-packet cost. */
#define LOG_MSG_MAX   192

/**
 * @brief One formatted log line, shipped off-board for centralised analysis.
 *
 * The bench runs several boards at once, and the interesting events -- a
 * retune on one unit against a clean window on another, a heap dip that
 * precedes an allocation failure, two units' phase lines side by side -- only
 * read across consoles. This carries each log line to a laptop collector so
 * they land in one merged stream. See wifi_log.h for the shipper, and note
 * that the RECEIVE path never sends one: a blocking send there would close a
 * loss -> log -> overflow -> loss loop.
 *
 * The collector is the hub's funnel: every unit sends to the hub, and the hub
 * forwards to whichever laptop registered with MSG_LOG_SUB. Every packet the
 * collector receives therefore comes from the hub, so the UDP source address
 * is useless for telling units apart. #role and #src_ip are what split them.
 */
typedef struct __attribute__((packed)) {
    uint8_t  type;        /**< MSG_LOG. */
    uint8_t  level;       /**< The level char as printed: 'E', 'W', 'I', 'D'. */
    uint8_t  role;        /**< LOG_ROLE_HUB or LOG_ROLE_SAT. */
    uint8_t  tag_len;     /**< Real bytes of #tag. */
    uint16_t msg_len;     /**< Real bytes of #msg; the rest is not sent. */
    uint32_t seq;         /**< Per-origin, so the collector can see its own holes. */
    /** @brief The originating unit's own address, network order. Stamped by
     *         the hub on relay, because a satellite does not know its DHCP
     *         lease. 0 means the hub itself. */
    uint32_t src_ip;
    char     tag[LOG_TAG_MAX];   /**< Not necessarily terminated; see #tag_len. */
    char     msg[LOG_MSG_MAX];   /**< Likewise; see #msg_len. */
} log_msg_t;

/** @brief Bytes to send for a message of m payload bytes. */
#define LOG_MSG_BYTES(m) (sizeof(log_msg_t) - LOG_MSG_MAX + (size_t)(m))

_Static_assert(sizeof(log_msg_t) <= 1500, "log_msg_t ceiling exceeds the MTU");

/**
 * @brief The structured counterpart to the HEALTH log line.
 *
 * Shipped on the same slow cadence as the narration that already holds every
 * field in scope. A fixed layout, not a formatted string, so the collector
 * writes one CSV row per snapshot and the numbers plot directly.
 *
 * The two units log different counters, so the fields past the common heap and
 * stack set are a UNION: the name is the satellite's and the hub's alias is in
 * the comment. Both units fill every field -- a counter that does not exist on
 * one role is left 0 -- so the collector unpacks one layout regardless of
 * role.
 *
 * The size is pinned by an assertion in test_sync_proto.c and by the struct
 * format in the collector. Growing it means editing both in lockstep, and the
 * hub relays a satellite's snapshot with sizeof rather than the received
 * length, so a version skew between units would be silent.
 */
typedef struct __attribute__((packed)) {
    uint8_t  type;          /**< MSG_HEALTH. */
    uint8_t  role;          /**< LOG_ROLE_HUB or LOG_ROLE_SAT. */
    uint8_t  clock_src;     /**< Satellite: 0 = probe estimator, 1 = TSF. Hub: 0. */
    uint8_t  _rsv;          /**< Padding, to keep the fields below aligned. */
    uint32_t seq;           /**< Per-origin. */
    uint32_t src_ip;        /**< As log_msg_t::src_ip. */
    uint64_t uptime_s;      /**< Seconds since boot. */
    uint32_t heap_cur;      /**< Free now. */
    uint32_t heap_min;      /**< Lowest since boot. */
    uint32_t heap_win;      /**< Lowest this window, taken and cleared. */
    uint32_t heap_largest;  /**< Largest free block. */
    uint32_t hw_play;       /**< Stack headroom, play task. */
    uint32_t hw_mon;        /**< Hub: ring_monitor. Satellite: drift task. */
    uint32_t underruns;                    /**< Output ran dry. */
    uint32_t reanchors_or_restarts;        /**< Sat: reanchors. Hub: restarts. */
    uint32_t splices;                      /**< Boundary corrections applied. */
    uint32_t retunes;                      /**< Coarse clock moves. */
    uint32_t retunes_refused;              /**< ...and those the actuator refused. */
    uint32_t gaps_or_sta_left;             /**< Sat: arrival gaps. Hub: sta-left. */
    uint32_t wifi_drops_or_oversize;       /**< Sat: wifi-drops. Hub: wifi-over. */
    uint32_t alloc_fail;                   /**< Allocation failures. */
    uint32_t phase_drop;                   /**< Phase points the queue could not hold. */
    uint32_t short_reads;                  /**< Reads that came up short. */
    uint32_t short_frames;                 /**< ...and the frames they were short by. */
    uint32_t ring_full_or_sta_dropped;     /**< Sat: ring-full. Hub: sta-dropped. */
    uint32_t upgrades_or_sta_nolease;      /**< Sat: anchor upgrades. Hub: sta-nolease. */
    uint32_t anchors_refused_or_timeout;   /**< Sat: anchors-refused. Hub: sta-timeout. */
    /** @brief Lines the capture itself lost -- the hook could not queue them,
     *         or the non-blocking send could not hand them to the driver.
     *         Nonzero means the merged stream has holes, which is worth
     *         reading before concluding anything from a gap between two
     *         lines. */
    uint32_t log_dropped;
    /** @brief ...and those with no collector registered. */
    uint32_t log_no_dest;
} health_msg_t;

/** @brief "LOG1" in ASCII. Stops a stray two-byte packet -- an unknown type,
 *         or a truncated probe -- from being read as a subscribe and pulling
 *         audio-bound traffic onto a laptop. */
#define LOG_SUB_MAGIC 0x4C4F4731u
/** @brief The collector's registration. Sent to the hub every few seconds; the
 *         hub forwards logs to the most recent sender until its TTL lapses.
 *         See wifi_log_note_collector(). */
typedef struct __attribute__((packed)) {
    uint8_t  type;        /**< MSG_LOG_SUB. */
    uint32_t magic;       /**< LOG_SUB_MAGIC. */
} log_sub_msg_t;

/** @brief Bytes to send for an audio payload of n. */
#define AUDIO_MSG_BYTES(n) (sizeof(audio_msg_t) - AUDIO_MAX_PAYLOAD + (n))

/** @brief What one UDP datagram may carry: a 1500-byte MTU less 20 bytes of IP
 *         and 8 of UDP. On-link under the SoftAP, so nothing fragments this
 *         further and nothing routes it. */
#define AUDIO_UDP_MTU 1472

/**
 * @brief The largest payload a sender may put in one audio datagram.
 *
 * At the payload sizes a phone actually produces it never binds, so the packet
 * rate is unchanged from every log ever captured. It exists for the case that
 * always could have fragmented and never had a guard: a maximum-size A2DP
 * payload, since the only ceiling checked was AUDIO_MAX_PAYLOAD.
 *
 * A cap that DID bind -- one sized so a whole redundant copy fits beside the
 * payload -- was tried and reverted: it splits every packet in two, and the
 * doubled packet rate both exhausts the transmit buffers and doubles the
 * timeline slew, which is applied per AUDIO packet. Anything that reintroduces
 * such a cap has to make TIMELINE_SLEW_US per unit of audio first -- which is
 * exactly why the parity below is a datagram of its own.
 */
#define AUDIO_TX_PAYLOAD_MTU_MAX (AUDIO_UDP_MTU - AUDIO_MSG_BYTES(0))

/**
 * @brief The parity datagram's own header: type + count + span + base_seq.
 *
 * XOR PARITY: one extra datagram per group of K audio packets, which recovers
 * any ONE of them whole.
 *
 * WHAT IT REPLACES. Redundancy used to be piggybacked, each audio packet
 * carrying copies of previous payloads in its own tail. That pinned every
 * packet at the MTU instead of its natural size -- most of an extra packet's
 * airtime, permanently, on a link whose group rate has no aggregation -- and a
 * whole copy still did not fit beside the payload, so every recovery came back
 * incomplete with a decode error on the end of it. Not sometimes: every one.
 *
 * Parity inverts the economics. The redundancy is its own datagram, so the
 * audio packets go back to their natural size and nothing is truncated; the
 * cost is 1/K of the audio bytes and 1/K of the datagram rate instead of one
 * times both.
 *
 * IT COSTS NO TIMELINE SLEW, which is the objection that killed the previous
 * attempt. TIMELINE_SLEW_US is applied once per audio packet, not per
 * datagram, so shrinking payloads to make copies fit doubled the slew along
 * with the packet rate. A parity datagram never enters that path: it carries
 * no play_at, does not advance the timeline, and is not counted in the packet
 * rate the servo steers on.
 *
 * THE CODEWORD IS THE WHOLE MESSAGE, HEADER INCLUDED. Each member contributes
 *
 *     [ AUDIO_MSG_BYTES(0) bytes of audio_msg_t header ][ payload, zero-padded ]
 *
 * and the parity is the XOR of all K. Padding is implicit -- a shorter payload
 * simply stops contributing, which is identical to XOR-ing zeros -- so the
 * unequal SBC lengths this stream produces need no length table on the wire.
 *
 * Recovering the header along with the payload is what makes the repair WHOLE
 * rather than approximate: seq, frames, play_at, marker and restart all come
 * back, so a recovered packet goes through the ordinary receive path and is
 * indistinguishable from one that arrived. No length guessing, no silence pad,
 * no separate fill routine that has to reproduce the real path's accounting.
 *
 * It is also the integrity check; audio_fec_extract() enforces it.
 *
 * GROUPS ARE ALIGNED ON seq, so both ends derive membership from arithmetic
 * and nothing has to be negotiated: member index is seq % K, base is seq -
 * (seq % K). base_seq and count travel on the parity anyway, so a hub and a
 * satellite built with different K disagree loudly instead of combining the
 * wrong packets.
 */
#define AUDIO_FEC_HDR_BYTES 8

/**
 * @brief The longest audio payload parity can cover: what is left of one
 *        datagram after the parity header and the audio header the codeword
 *        carries.
 *
 * It sits only a few bytes below AUDIO_TX_PAYLOAD_MTU_MAX, so the only
 * payloads it excludes are ones already within those few bytes of fragmenting,
 * and it is far above what a phone produces. A group containing one is sent
 * without parity and counted, rather than sending a parity that would fragment
 * or one that silently covers less than it claims.
 */
#define AUDIO_FEC_PAYLOAD_MAX (AUDIO_UDP_MTU - AUDIO_FEC_HDR_BYTES - AUDIO_MSG_BYTES(0))
/** @brief The longest codeword, i.e. a full header plus that payload. */
#define AUDIO_FEC_CODEWORD_MAX AUDIO_MSG_BYTES(AUDIO_FEC_PAYLOAD_MAX)

/**
 * @brief The ceiling on K, not the value of it -- DANCEFLOOR_AUDIO_FEC_K picks
 *        that.
 *
 * Eight because the receiver tracks which members it has seen in one byte, and
 * because the LATENCY argument runs out well before the bitmask does: a loss
 * can only be repaired once the parity arrives, so the receiver must HOLD the
 * packets behind the hole until then, and that hold is (K-2) packet times in
 * the worst case. At the default K that is a small fraction of the playback
 * lead; at this ceiling it is a large one.
 */
#define AUDIO_FEC_K_MAX 8

/** @brief One group's parity. */
typedef struct __attribute__((packed)) {
    uint8_t  type;        /**< MSG_AUDIO_FEC. */
    /** @brief Members XORed into #parity, 2..AUDIO_FEC_K_MAX. With #base_seq,
     *         this is what lets a receiver check the group it reconstructed
     *         against the group the sender actually built -- the whole of the
     *         two-loss and mismatched-K defence. */
    uint8_t  count;
    /** @brief Bytes of #parity that follow: the longest codeword in the group,
     *         so a group of ordinary payloads sends far less than the ceiling.
     *         Carried explicitly rather than derived from the datagram length,
     *         so a truncated receive is refused instead of silently XOR-ing
     *         short. */
    uint16_t span;
    uint32_t base_seq;    /**< seq of member 0 of the group. */
    uint8_t  parity[AUDIO_FEC_CODEWORD_MAX];  /**< See #span. */
} audio_fec_msg_t;

/** @brief Bytes to send for a parity of `n` span. */
#define AUDIO_FEC_MSG_BYTES(n) (sizeof(audio_fec_msg_t) - AUDIO_FEC_CODEWORD_MAX + (n))

_Static_assert(AUDIO_FEC_MSG_BYTES(0) == AUDIO_FEC_HDR_BYTES,
               "AUDIO_FEC_HDR_BYTES and audio_fec_msg_t's header disagree -- "
               "AUDIO_FEC_PAYLOAD_MAX is computed from the former");
_Static_assert(AUDIO_FEC_MSG_BYTES(AUDIO_FEC_CODEWORD_MAX) == AUDIO_UDP_MTU,
               "a full parity datagram would fragment");
_Static_assert(AUDIO_FEC_K_MAX <= 8, "the receiver's seen-mask is one byte");

/**
 * @brief XOR one member's codeword into acc[], growing *span to cover it.
 *
 * Adding is the same operation at both ends: the hub folds in every packet it
 * sends, the satellite folds in every packet that arrives and then folds in
 * the parity itself -- leaving exactly the codeword of whatever did not
 * arrive.
 *
 * @param[in,out] acc   AUDIO_FEC_CODEWORD_MAX bytes, zeroed at the start of a
 *                      group.
 * @param[in,out] span  Bytes of @p acc in use; zeroed with it.
 * @param m             The member to fold in.
 * @return false, leaving @p acc untouched, if the payload is longer than
 *         parity can cover -- which means the group gets no parity at all.
 */
bool audio_fec_xor_in(uint8_t *acc, uint16_t *span, const audio_msg_t *m);

/**
 * @brief The reverse: validate what the XOR left and copy it out as a whole
 *        message.
 *
 * @param acc       Parity XOR every member that DID arrive, so it is the
 *                  missing member's codeword.
 * @param span      Bytes of @p acc that are meaningful.
 * @param want_seq  The seq the caller believes is missing. A recovered header
 *                  saying anything else means the group was not what the
 *                  receiver thought it was.
 * @param[out] out  The recovered packet; meaningful only when this returns
 *                  true, since a refused recovery may leave it holding
 *                  whatever the XOR produced.
 * @return true if the recovery is trustworthy. It is refused for a type or
 *         format that is not audio, a seq that is not @p want_seq, or a
 *         payload_len that does not fit inside @p span. Those three together
 *         are the two-loss defence: with one member missing the XOR reproduces
 *         it exactly and all three pass by construction, while with two
 *         missing it produces the XOR of both, which is not a packet. Any one
 *         of them matching by accident is possible; all three at once is not.
 */
bool audio_fec_extract(const uint8_t *acc, uint16_t span, uint32_t want_seq,
                       audio_msg_t *out);

/* The frame lane against the same MTU. Here rather than beside frame_msg_t
 * because this is where the MTU is defined. */
_Static_assert(FRAME_MSG_BYTES(FRAME_PAYLOAD_MAX) <= AUDIO_UDP_MTU,
               "a full frame batch would fragment");

/** @brief A satellite's rolling window of completed probe exchanges. */
typedef struct {
    int64_t offset[SYNC_WINDOW];  /**< Offset each probe implied. */
    int64_t delay[SYNC_WINDOW];   /**< Round-trip time, for minimum-delay selection. */
    int count;                    /**< Probes held, up to SYNC_WINDOW. */
    int next;                     /**< Where the next one lands. */
} sync_est_t;

/**
 * @brief Empty the window.
 * @param e  The estimator.
 */
void sync_est_init(sync_est_t *e);

/**
 * @brief Fold one completed probe exchange into the estimate.
 *
 * offset = ((t2 - t1) + (t3 - t4)) / 2, the standard NTP estimator. It assumes
 * the two path delays are equal; they are not, and that asymmetry is the error
 * floor. Selecting across probes is what keeps WiFi retries from wrecking it.
 *
 * @param e   The estimator.
 * @param t1  Satellite transmit, satellite clock.
 * @param t2  Master receive, master clock.
 * @param t3  Master transmit, master clock.
 * @param t4  Satellite receive, satellite clock.
 */
void sync_est_add(sync_est_t *e, int64_t t1, int64_t t2, int64_t t3, int64_t t4);

/**
 * @brief Best offset estimate, such that master_time = local_time + offset.
 *
 * Selects the offset from the probe with the LOWEST round-trip time rather
 * than taking a median. A fast round trip had little queuing in either
 * direction, so its paths were closer to symmetric -- and asymmetry is the only
 * error the estimator cannot see. Measured round-trip time on a SoftAP link
 * swings by a factor of two or three, so the choice of sample matters more than
 * averaging across all of them. This is what PTP does, and on hardware it beat
 * a median by an order of magnitude.
 *
 * Ties favour the newer sample, since an old one has had time to drift.
 *
 * @param e               The estimator.
 * @param[out] offset_out The offset, us.
 * @return false until SYNC_MIN_SAMPLES probes have landed.
 */
bool sync_est_offset(const sync_est_t *e, int64_t *offset_out);

/**
 * @brief True once a FULL window of probes has landed, i.e. the estimate has
 *        had a chance to find a genuinely low-RTT sample rather than the best
 *        of three.
 *
 * Matters for anchoring playback: play_at is consulted once, at stream start,
 * so an offset error at that instant is permanent. Waiting the extra couple of
 * seconds costs nothing and removes a whole class of "one speaker is slightly
 * out" that would be untraceable afterwards.
 *
 * @param e  The estimator.
 * @return Whether it has settled.
 */
static inline bool sync_est_settled(const sync_est_t *e)
{
    return e->count >= SYNC_WINDOW;
}

/**
 * @brief Master clock -> local clock.
 * @param master_us  An instant on the master's clock.
 * @param offset     From sync_est_offset().
 * @return The same instant on this unit's clock.
 */
static inline int64_t sync_to_local(int64_t master_us, int64_t offset)
{
    return master_us - offset;
}

/**
 * @brief Readings held for the splice's median.
 *
 * Odd, so the median is an element and not an average of two.
 *
 * The servo smooths its input; the splice needs to as well, and cannot reuse
 * the servo's average -- that is updated once per servo window, so at a
 * boundary it is many seconds stale. This is a separate, short filter over the
 * raw readings themselves.
 *
 * MEDIAN, not mean. What is left after the overshoot and write-instant
 * corrections both units apply is preemption latency on a board also running a
 * SoftAP, an SBC decode and the bridge link: bounded below, long-tailed to the
 * right. There is a minimum latency and no mechanism that makes a reading
 * early. The mean is dragged by that tail; the median sits on the mode. If the
 * noise is symmetric after all, the median merely costs about a quarter more
 * in standard error at this window length, which is nothing against the
 * scatter being removed -- so it is the right answer under both models and the
 * mean under only one. Same shape as sync_est_offset()'s minimum-RTT
 * selection, for the same reason.
 */
#define SYNC_PHASE_HIST 9
/** @brief Below this many readings no median is offered -- the guard against
 *         splicing on one or two samples taken just after a re-anchor. */
#define SYNC_PHASE_MIN  5

/** @brief The history itself. Owned by whichever task takes the readings. */
typedef struct {
    int32_t v[SYNC_PHASE_HIST];  /**< The readings, in arrival order. */
    uint8_t next;                /**< Where the next one lands. */
    uint8_t count;               /**< Readings held, up to SYNC_PHASE_HIST. */
} sync_phase_hist_t;

/**
 * @brief Drop the history: every reading in it describes a position the unit
 *        has left.
 * @param h  The history.
 */
void sync_phase_reset(sync_phase_hist_t *h);

/**
 * @brief Add one accepted phase reading.
 * @param h   The history.
 * @param us  The reading, microseconds; + = playing late.
 */
void sync_phase_push(sync_phase_hist_t *h, int32_t us);

/**
 * @brief The median of what is held.
 *
 * A full history spans a fraction of a second at packet cadence. Over that
 * span drift, the hub's timeline slew and the satellite's offset slew are all
 * far below the scatter being removed, and all common-mode across units, so
 * nothing here separates them.
 *
 * @param h        The history.
 * @param[out] out The median, us.
 * @return false below SYNC_PHASE_MIN readings.
 */
bool sync_phase_median(const sync_phase_hist_t *h, int32_t *out);
