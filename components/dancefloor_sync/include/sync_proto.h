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
    /* 3 was MSG_BLINK, for the M4 blink harness. The number is deliberately
     * not reused: an old board still flashed with that firmware would be
     * talking a protocol this one no longer speaks, and a silently
     * reinterpreted type is worse than an unknown one. */
    MSG_AUDIO    = 4,   /* master -> listeners */
    MSG_META     = 5,   /* master -> listeners, track metadata */
    MSG_SPLICE   = 6,   /* satellite -> master, what it corrected at a boundary */
    MSG_TSF      = 7,   /* master -> satellite, measurement only, see tsf_msg_t */
    MSG_FRAME    = 8,   /* master -> listeners, one analysis frame */
    MSG_LOG      = 9,   /* any wifi unit -> collector, one formatted log line */
    MSG_HEALTH   = 10,  /* any wifi unit -> collector, the structured HEALTH snapshot */
    MSG_LOG_SUB  = 11,  /* collector -> hub: "send logs here" (see log_sub_msg_t) */
    MSG_ML       = 12,  /* master -> listeners, one analyser result */
    MSG_VOL      = 13,  /* master -> listeners, playback volume (see vol_msg_t) */
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
    /*
     * The correction this unit WOULD have applied on the OTHER estimator. Same
     * units and same clamp as applied_us, so the two subtract meaningfully --
     * that is the whole reason it is the correction rather than a phase value.
     *
     * The roles have swapped, and the field was renamed when they did. It used to
     * be `applied_med_us`: the splice ran on the newest raw reading and this
     * carried what the median would have asked for. The median won -- the raw
     * reading carries ~15.7 ms of scatter, and two units splicing on independent
     * noisy samples land in different places, which is why a track change
     * sometimes improved cross-unit sync and sometimes degraded it. Both units
     * now splice on sync_phase_median() and this carries the RAW counterfactual.
     *
     * Kept rather than deleted so the comparison that justified the change
     * survives it, and so a revert has something to check itself against.
     *
     * WIRE COMPATIBILITY: the layout is unchanged -- same offset, same width,
     * same clamp -- so a unit on either build parses the other's message. Only
     * the meaning of the number flips, and both firmwares must be reflashed
     * together for the printed comparison to mean anything.
     */
    int32_t applied_alt_us;
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
 * Smoothed phase error each unit tolerates before correcting its output rate.
 *
 * Shared, because it is not a per-unit preference: every unit deadbands around
 * its own reading of the same timeline, so the worst case between any two of
 * them is twice this, whatever the servos report individually.
 *
 * It used to be bounded below by what a RETUNE costs, measured rather than
 * guessed. On the bench, forcing same-rate retunes and reading the phase either
 * side: the channel is down 1.8 to 5.8 ms and the phase step is 1.1 to 6.9 ms,
 * mean 3.3 ms against a mean outage of 3.1 ms. The step IS the outage, to
 * within the several ms the phase wanders on its own.
 *
 * This was briefly raised to 20000 on the theory that frequent retunes were
 * what made the strips drift. They were not. A retune used to cost up to 50 ms
 * because the satellite's playback task ran flat out while the channel was down
 * -- see the `retuning` guard in satellite/main/out.c -- and once that was
 * fixed the cost fell to the outage. 7000 is what the build that measured well
 * actually had.
 *
 * NEITHER BOUND STILL BINDS, as of 2026-08-14. Corrections this size are made
 * in software now, by dropping or duplicating one frame at a time (see
 * rate_trim_hz on either unit), so a correction costs no outage at all and the
 * whole-Hz floor -- 22.7 ppm at 44.1 kHz, ~2.3 ms of phase, the smallest step a
 * clock retune could express -- is a property of an actuator that is no longer
 * used here. Only a coarse rate MATCH still retunes the clock.
 *
 * 7000 is kept unchanged all the same, and deliberately: the point of that
 * change was to remove an interruption without moving the sync behaviour, and
 * every cross-unit figure this project has recorded was measured at this value.
 * Tightening it is now affordable in a way it was not, and it is the obvious
 * next experiment -- but it is a separate one, with its own flash and its own
 * TRACK DIVERGENCE reading. See docs/clock-sync.md.
 */
#define PHASE_DEADBAND_US 7000


typedef enum {
    /*
     * 0 was AUDIO_FMT_PCM, interleaved 16-bit stereo, from when the master
     * decoded for everyone and sent samples. Satellites decode their own SBC
     * now -- it costs a quarter of the airtime -- so nothing produces this
     * format and satellite rejects any packet not marked SBC.
     *
     * Kept as a burned number for the same reason as MSG_BLINK above: a board
     * still running the old firmware would put a 0 here and mean PCM by it, and
     * a format byte silently reinterpreted as something else is worse than one
     * that is simply unknown.
     */
    AUDIO_FMT_SBC = 1,   /* one or more back-to-back SBC frames */
} audio_fmt_t;

/*
 * Sized to match SBC_LINK_MAX_PAYLOAD (sbc_link.h): the same max SBC payload, on
 * the WiFi hop to satellites instead of the SPI hop from the bridge. It must
 * stay >= SBC_LINK_MAX_PAYLOAD or the forwarder refuses what the link delivered
 * -- test_sync_proto.c asserts it does not drift below. Only the first
 * payload_len bytes go on the wire (AUDIO_MSG_BYTES), so the array is a ceiling,
 * not a per-packet cost.
 */
#define AUDIO_MAX_PAYLOAD 2048

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
/*
 * One analysis frame, for units that draw what another unit decided.
 *
 * The payload is a vis_frame_t from components/dancefloor_leds -- copied in as
 * bytes rather than declared here, because that component does not depend on
 * this one and must stay buildable alone. `len` is carried so the two ends can
 * disagree about the frame's size and say so, instead of reading past it: a hub
 * and a satellite on different builds is the failure this protocol is most
 * likely to meet, and a silently reinterpreted frame would be worse than a
 * refused one.
 *
 * Unicast to each listener like the audio, NOT multicast. Group-addressed
 * frames are never acknowledged and so never retried -- measured at ~20% loss
 * at every PHY rate tried -- and 20% of frames missing is a visibly broken
 * strip. ~5 kB/s per listener at 43 Hz, against the 30-40 the audio already
 * costs, so paying for retries here is cheap.
 */
#define FRAME_PAYLOAD_MAX 160

typedef struct __attribute__((packed)) {
    uint8_t type;           /* MSG_FRAME */
    uint8_t len;            /* bytes of payload that follow */
    uint8_t payload[FRAME_PAYLOAD_MAX];
} frame_msg_t;

#define FRAME_MSG_BYTES(n) (sizeof(frame_msg_t) - FRAME_PAYLOAD_MAX + (n))

/*
 * One pluggable analyser's result, for a unit that is given them rather than
 * computing them.
 *
 * A separate message from MSG_FRAME rather than more bytes inside it, and the
 * reason is cadence. Frames go out at the analysis rate -- 86 a second at hop
 * 512 -- while an analyser with a second of context reports once a second.
 * Carrying a result in every frame would multiply its cost by eighty and change
 * the size of a struct both ends already agree about.
 *
 * The payload is an ml_result_t from components/dancefloor_leds, copied in as
 * bytes for the same reason vis_frame_t is: that component does not depend on
 * this one and must stay buildable alone. `len` is carried so the two ends can
 * disagree about the size and say so rather than reading past it.
 *
 * Cost is whatever the analysers report. A slow one at 1 Hz is 36 bytes a
 * second and beneath notice; a FAST one publishes at the frame rate and costs
 * about what a vis_frame_t stream does, which is worth knowing before turning
 * one on for a floor rather than a bench.
 */
#define ML_PAYLOAD_MAX 64

typedef struct __attribute__((packed)) {
    uint8_t type;           /* MSG_ML */
    uint8_t len;            /* bytes of payload that follow */
    uint8_t payload[ML_PAYLOAD_MAX];
} ml_msg_t;

#define ML_MSG_BYTES(n) (sizeof(ml_msg_t) - ML_PAYLOAD_MAX + (n))

typedef struct __attribute__((packed)) {
    uint8_t type;           /* MSG_META */
    uint8_t payload[196];   /* sizeof(link_meta_t) */
} meta_msg_t;

/* The two definitions must not drift apart; sbc_link.h owns the fields. */
_Static_assert(sizeof(link_meta_t) <= 196, "link_meta_t outgrew meta_msg_t.payload");

/*
 * Playback volume, hub -> listeners. Applied at each unit's output by
 * audio_apply_volume(); see that for the taper and why it lives there.
 *
 * Sent when the phone changes it AND repeated every telemetry window, which is
 * what makes a satellite that joined late, or missed the change, converge on
 * the right level rather than sitting at a stale one forever. Two bytes at
 * 0.2/s is not worth a smarter scheme, and this way there is no join handshake
 * to get wrong.
 *
 * Unicast per client, like meta and unlike audio: it is rare, it must not be
 * held for a DTIM burst behind the audio it would delay, and a satellite that
 * missed one gets the next within the window.
 */
typedef struct __attribute__((packed)) {
    uint8_t type;           /* MSG_VOL */
    uint8_t volume;         /* 0-127, 127 = unity */
} vol_msg_t;

/*
 * One formatted log line, shipped off-board for centralised analysis.
 *
 * The bench runs several boards at once and the interesting events -- a
 * retune on one unit against a clean window on another, a heap dip that
 * precedes an alloc-fail, two units' phase lines side by side -- only read
 * across consoles. This carries each ESP_LOG line to a laptop collector so
 * they land in one merged stream. The receive path never sends it: a blocking
 * send there would close the same loss -> log -> mailbox-overflow -> loss loop
 * that took the per-event ESP_LOGW calls out of handle_audio()
 * (satellite/main/main.c:286). The hook that fills this runs in whichever
 * task logged, but only enqueues (non-blocking, drop-on-full); a separate
 * low-priority task drains the queue and does the sendto().
 *
 * Variable-length, like the audio and frame messages: only the first msg_len
 * bytes of `msg` go on the wire (LOG_MSG_BYTES), so the 192-byte array is a
 * ceiling, not a per-packet cost. The whole ceiling stays under a 1500-byte
 * MTU so nothing fragments.
 *
 * The collector is the hub's funnel: every unit sends to the hub, and the hub
 * forwards to whichever laptop registered with MSG_LOG_SUB. Every packet the
 * collector receives therefore comes from the hub's 192.168.4.1, so the UDP
 * source address is useless for telling units apart. `role` and `src_ip` are
 * what split them: role distinguishes hub from satellite, and src_ip (in
 * network byte order) is the satellite's own address, stamped by the hub on
 * relay because the satellite does not know its DHCP lease. 0 means "the hub
 * itself" on a hub-originated packet.
 */
#define LOG_ROLE_HUB 0
#define LOG_ROLE_SAT 1

#define LOG_TAG_MAX   16
#define LOG_MSG_MAX   192

typedef struct __attribute__((packed)) {
    uint8_t  type;        /* MSG_LOG */
    uint8_t  level;       /* the level char as printed: 'E','W','I','D' */
    uint8_t  role;        /* LOG_ROLE_HUB / LOG_ROLE_SAT */
    uint8_t  tag_len;
    uint16_t msg_len;
    uint32_t seq;
    uint32_t src_ip;      /* network order; hub stamps on relay, 0 = self */
    char     tag[LOG_TAG_MAX];
    char     msg[LOG_MSG_MAX];
} log_msg_t;

/* Bytes to send for a message of m payload bytes. */
#define LOG_MSG_BYTES(m) (sizeof(log_msg_t) - LOG_MSG_MAX + (size_t)(m))

_Static_assert(sizeof(log_msg_t) <= 1500, "log_msg_t ceiling exceeds the MTU");

/*
 * The structured counterpart to the HEALTH line, shipped every ~60 s beside
 * the ring_monitor_task (hub) / drift_task (satellite) narration that already
 * holds every field in scope. A fixed layout, not a formatted string, so the
 * collector writes one CSV row per snapshot and the numbers plot directly.
 *
 * The two units log different counters, so the fields past the common heap and
 * stack set are a union: the name is the satellite's and the hub alias is in
 * the comment. Both units fill every field -- a counter that does not exist on
 * one role is left 0 -- so the collector unpacks one layout regardless of role.
 * role/src_ip are as for log_msg_t: the hub stamps src_ip on relay.
 */
typedef struct __attribute__((packed)) {
    uint8_t  type;          /* MSG_HEALTH */
    uint8_t  role;          /* LOG_ROLE_HUB / LOG_ROLE_SAT */
    uint8_t  clock_src;     /* satellite: 0 = probe estimator, 1 = TSF; hub: 0 */
    uint8_t  _rsv;
    uint32_t seq;
    uint32_t src_ip;        /* network order; hub stamps on relay, 0 = self */
    uint64_t uptime_s;
    uint32_t heap_cur;
    uint32_t heap_min;      /* since boot */
    uint32_t heap_win;      /* lowest this minute, taken and cleared */
    uint32_t heap_largest;  /* largest free block */
    uint32_t hw_play;       /* stack headroom, play task */
    uint32_t hw_mon;        /* hub: ring_monitor; satellite: drift task */
    /* counters common to both roles, then the role-specific tail. */
    uint32_t underruns;
    uint32_t reanchors_or_restarts;        /* sat: reanchors, hub: restarts */
    uint32_t splices;
    uint32_t retunes;
    uint32_t retunes_refused;              /* both: n_retunes_bad */
    uint32_t gaps_or_sta_left;             /* sat: gaps, hub: sta-left */
    uint32_t wifi_drops_or_oversize;       /* sat: wifi-drops, hub: wifi-over */
    uint32_t alloc_fail;
    uint32_t phase_drop;
    uint32_t short_reads;
    uint32_t short_frames;
    uint32_t ring_full_or_sta_dropped;     /* sat: ring-full, hub: sta-dropped */
    uint32_t upgrades_or_sta_nolease;      /* sat: anchor upgrades, hub: sta-nolease */
    uint32_t anchors_refused_or_timeout;   /* sat: anchors-refused, hub: sta-timeout */
    /*
     * How faithful the capture itself is, from wifi_log. dropped counts lines
     * the hook could not queue or the non-blocking send could not hand to the
     * driver; no_dest counts those with no collector registered. Nonzero
     * dropped means the merged stream has holes -- read it before concluding
     * anything from a gap between two lines.
     */
    uint32_t log_dropped;
    uint32_t log_no_dest;
} health_msg_t;

/*
 * The collector's registration. Sent to the hub every few seconds; the hub
 * forwards logs to the most recent sender for ~30 s after the last one. The
 * magic stops a stray two-byte packet (an unknown type, or a truncated probe)
 * from being read as a subscribe and pulling audio-bound traffic onto the
 * laptop. "LOG1" in ASCII.
 */
#define LOG_SUB_MAGIC 0x4C4F4731u
typedef struct __attribute__((packed)) {
    uint8_t  type;        /* MSG_LOG_SUB */
    uint32_t magic;       /* LOG_SUB_MAGIC */
} log_sub_msg_t;

/* Bytes to send for a payload of `n`. */
#define AUDIO_MSG_BYTES(n) (sizeof(audio_msg_t) - AUDIO_MAX_PAYLOAD + (n))

/*
 * Optional trailing forward-error-correction redundancy.
 *
 * audio_msg_t itself is UNCHANGED: when no redundancy is attached this is the
 * same wire format every log was captured against, and an older receiver reads
 * payload_len bytes and ignores anything after. Redundancy is appended AFTER the
 * first payload_len bytes as one or more self-describing blocks, found by length
 * rather than by a version byte -- so a receiver from this branch recovers from
 * any sender that attached it, and a sender that did not is read unchanged.
 *
 * Each block is a packed header (red_len, red_seq_ofs) followed by red_len bytes
 * of a previous packet's SBC. red_seq_ofs says which previous packet it recovers
 * (1 = the packet immediately before this one). A block lives in the unused tail
 * of payload[] on the wire; the attach path is MTU-guarded so a packet carries as
 * many whole blocks as fit and never fragments.
 *
 * Decoded audio is ~1.4 Mbps in 172 packets/s; the SBC sent over the air is
 * ~330 kbps in ~50 packets/s of ~825-byte payloads. A copy is attached only as
 * far as the MTU allows, so at those payload sizes it is TRUNCATED to about
 * three quarters and the satellite pads the rest with silence -- roughly 6 ms
 * per "recovered" packet, which the log still reports as a clean recovery.
 *
 * That is why the depth defaults to 0. A whole copy needs 2 x 825 + 29 bytes
 * against a 1472-byte MTU and cannot fit, and making it fit by shrinking the
 * payload doubles the packet rate, which this hub cannot afford. The full
 * measurement is in DANCEFLOOR_AUDIO_FEC_DEPTH's Kconfig help; it is kept there
 * rather than here because it is a decision about a setting, not about the wire
 * format, and the wire format is what this header is for.
 */
#define AUDIO_RED_HDR_BYTES   3                     /* u16 red_len + u8 red_seq_ofs */
#define AUDIO_RED_BYTES(r)    (AUDIO_RED_HDR_BYTES + (r))

/* On-wire bytes for an audio message of n payload bytes followed by r bytes of
 * trailing redundancy. AUDIO_MSG_BYTES(n) is the r == 0 case. */
#define AUDIO_MSG_RED_BYTES(n, r) (AUDIO_MSG_BYTES(n) + AUDIO_RED_BYTES(r))

/*
 * What one UDP datagram may carry: 1500-byte Ethernet-equivalent MTU less 20
 * bytes of IP and 8 of UDP. On-link under the SoftAP, so nothing fragments this
 * further and nothing routes it.
 */
#define AUDIO_UDP_MTU 1472

/*
 * The largest payload a sender may put in one datagram: header plus payload
 * against the MTU, 1446 bytes.
 *
 * At the ~825-byte payloads a phone produces it never binds, so the packet rate
 * is unchanged from every log ever captured. It exists for the case that always
 * could have fragmented and never had a guard: a 1500-byte A2DP payload becoming
 * a 1526-byte datagram, since the only ceiling checked was AUDIO_MAX_PAYLOAD at
 * 2048. The FEC attach path clamps its own copies to whatever is left, so it
 * cannot push a packet over on its own.
 *
 * A cap that DID bind -- one sized so a whole redundant copy fits beside the
 * payload -- was tried and reverted: it splits every packet in two, and the
 * doubled packet rate both exhausts the transmit buffers and doubles the
 * timeline slew, which is per packet. See DANCEFLOOR_AUDIO_FEC_DEPTH's Kconfig
 * help for the measurement. Anything that reintroduces such a cap has to make
 * TIMELINE_SLEW_US per unit of audio first.
 */
#define AUDIO_TX_PAYLOAD_MTU_MAX (AUDIO_UDP_MTU - AUDIO_MSG_BYTES(0))

/*
 * Which downlink this build speaks, as a short tag for the periodic status
 * lines. It is a compile-time choice (see DANCEFLOOR_AUDIO_MCAST), and until it
 * appeared here the two builds were indistinguishable in any log window: the
 * only tell was a one-time boot line that scrolls out of a capture the moment a
 * run starts. FEC depth rides along because it is the other half of the same
 * experiment and changes between bench phases (Phase 1 = fec 0, Phase 2 = fec 1).
 */
#if CONFIG_DANCEFLOOR_AUDIO_MCAST
#define AUDIO_TRANSPORT_TAG "mcast"
#else
#define AUDIO_TRANSPORT_TAG "unicast"
#endif

/*
 * The same for the analysis frames, which are a separate switch
 * (DANCEFLOOR_MCAST_FRAMES) because they are a separate loss budget. Printed by
 * the hub only -- a satellite receives frames and has no transport to report --
 * and printed for the same reason as the audio tag: at 86 frames a second per
 * satellite this is the difference between a hub that transmits ~146 packets a
 * second and one that transmits over a thousand, which is far too large a
 * difference to have to infer from a boot line that has scrolled away.
 */
#if CONFIG_DANCEFLOOR_MCAST_FRAMES
#define FRAMES_TRANSPORT_TAG "mcast"
#else
#define FRAMES_TRANSPORT_TAG "unicast"
#endif

typedef struct __attribute__((packed)) {
    uint16_t red_len;        /* bytes of red_payload that follow; 0 == none */
    uint8_t  red_seq_ofs;    /* this recovers the packet (seq - red_seq_ofs) */
} audio_red_hdr_t;

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

/*
 * A short history of raw phase readings, for the track-boundary splice.
 *
 * The servo has smoothed its input since it was measured triggering on noise;
 * the splice never did. It snaps this unit's position using the single most
 * recent reading, and on the hub that reading is not trustworthy on its own:
 * two reads of it a millisecond apart differed by 15.7 ms, and
 * docs/architecture.md §16 lists "the hub's absolute phase reading wanders" as
 * a wart with the cause unfound. So at every boundary the hub jumps to a
 * position several milliseconds wrong in a direction nothing predicts, while
 * the satellite -- quieter by a factor of three, its load being a fraction of
 * the hub's -- lands closer. The two splice to different places, which is why a
 * track change sometimes improves cross-unit sync and sometimes degrades it.
 *
 * The EMA the servo uses cannot serve here: it is updated once per 5 s window,
 * so at a boundary it is up to 20 s stale. This is a separate, short filter
 * over the raw readings themselves.
 *
 * MEDIAN, not mean. What is left after the overshoot and wrote_at corrections
 * (see either unit's crossing loop) is preemption latency on a board also
 * running a SoftAP, SBC decode and the bridge SPI link: bounded below, long-tailed
 * to the right. There is a minimum latency and no mechanism that makes a
 * reading early. The mean is dragged by that tail; the median sits on the mode.
 * If the noise is symmetric after all, the median merely costs ~1.25x in
 * standard error at this window length, which is nothing against a 15.7 ms
 * swing -- so it is the right answer under both models and the mean under only
 * one. It is the same shape as the offset estimator's minimum-RTT selection,
 * which beat a median 117 us to 1080 us on hardware for the same reason.
 */
#define SYNC_PHASE_HIST 9   /* odd, so the median is an element and not an average */
#define SYNC_PHASE_MIN  5   /* below this no median is offered */

typedef struct {
    int32_t v[SYNC_PHASE_HIST];
    uint8_t next;
    uint8_t count;
} sync_phase_hist_t;

void sync_phase_reset(sync_phase_hist_t *h);

/* One accepted phase reading, in microseconds, + = playing late. */
void sync_phase_push(sync_phase_hist_t *h, int32_t us);

/*
 * The median of what is held. False below SYNC_PHASE_MIN readings, which is the
 * guard against splicing on one or two samples taken just after a re-anchor.
 *
 * Nine readings arrive in about 180 ms at ~50 packets/s. Over that span drift
 * contributes 5 us, the hub's timeline slew 0.4 ms and the satellite's offset
 * slew 78 us -- all far below the millisecond scatter being removed, and all
 * common-mode across units, so nothing here separates them.
 */
bool sync_phase_median(const sync_phase_hist_t *h, int32_t *out);
