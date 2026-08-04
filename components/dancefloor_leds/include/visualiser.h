/*
 * FFT -> onset detection -> LED strip, shared by the hub and every satellite.
 *
 * Each unit analyses its OWN copy of the audio rather than being told what to
 * display. Sending analysis results over the network would add a second thing to
 * keep synchronised; the audio is already synchronised, so anything derived from
 * it locally is synchronised too, for free.
 *
 * C API with a C++ implementation: the callers are plain C audio code and should
 * stay that way, while the pattern rendering benefits from the LedStrip wrapper.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Mirrors df::BEAT_BANDS and df::SPEC_BINS, which are C++ and cannot be seen
 * from here. visualiser.cpp static_asserts that they still agree. */
#define VIS_BANDS      4
#define VIS_SPEC_BINS 64

/*
 * One analysis frame, in the form that can leave the unit that computed it.
 *
 * This is df::Frame minus the one field that cannot travel -- `mag`, 512 floats
 * pointing into an Analysis -- and with the spectrum in its reduced form. 123
 * bytes, so a stream of these at 43 Hz is ~5 kB/s against the 30-40 the audio
 * already uses on the same radio.
 *
 * Deliberately declared here and not in sync_proto.h. This component does not
 * depend on the audio protocol and must stay buildable on its own; the unit
 * that transports a frame is the one that knows how, and copies these bytes
 * into whatever message it sends. Packed for the same reason -- the two ends
 * may not be the same build.
 *
 * A unit fed these instead of computing them cannot tell the difference from a
 * pattern's point of view, which is the whole purpose: the algorithm runs in
 * one place, so no algorithm has to be proved deterministic across units to
 * stay in sync.
 */
typedef struct __attribute__((packed)) {
    int64_t due_us;         /* master-clock instant this frame describes */
    int64_t index;          /* block number, from an origin all units share */
    float   band[VIS_BANDS];
    float   flux;
    float   threshold;
    float   strength;
    float   boom_strength;
    float   boom_flux;
    float   boom_threshold;
    uint8_t onset;
    uint8_t boom;
    uint8_t unit;           /* which speaker computed it; 0 is the hub */
    uint8_t spec[VIS_SPEC_BINS];
} vis_frame_t;

/*
 * Called with every frame this unit computes, if set.
 *
 * For a unit that sends its frames to others. Runs on the analysis task, so it
 * must not block -- hand the bytes to a queue or a socket and return.
 * Registering nothing, which is the default, simply publishes nothing.
 */
void visualiser_set_publish(void (*publish)(const vis_frame_t *f));

/*
 * Hand this unit a frame computed somewhere else.
 *
 * Goes into the same queue local analysis fills and is drawn at the instant it
 * names, so the two sources are interchangeable by construction. Safe from any
 * task. Ignored unless this unit is built to take frames from elsewhere -- see
 * CONFIG_DANCEFLOOR_LED_SOURCE -- because a unit doing its own analysis and
 * also accepting somebody else's would draw two timelines at once.
 */
void visualiser_submit_frame(const vis_frame_t *f);

/* Whether this unit computes its own frames or is given them. Reportable, so a
 * mixed floor is a fact in the log rather than a puzzle. */
const char *visualiser_source_name(void);

/* Brings up the strip and starts the analysis and render tasks. Call once. */
void visualiser_start(void);

/*
 * How to convert a master-clock instant into this board's local clock.
 *
 * Analysis and display are separate stages now: a frame is computed whenever
 * the audio for it arrives and drawn when the instant it describes comes round.
 * Waiting for that instant is the one thing in here that needs a clock, so it
 * is the one thing that has to be told about the offset between them.
 *
 * This used to be true of nothing here, deliberately -- every unit derived
 * due_us from the play_at stamps the audio carried, so the whole component
 * worked in master time and never needed an offset. Deriving is still how the
 * label is produced; what is new is that something now has to WAIT for it.
 *
 * Passed as a function rather than a number because the satellite's offset is
 * not constant: it is slewed toward the live estimate at 200 ppm, so a value
 * copied once would go stale at exactly the crystal difference -- the same bug
 * docs/clock-sync.md section 9 records in the audio path, where the servo was
 * fed its own drift as a reference.
 *
 * Leave it unset on the hub, where local time IS master time. A unit that never
 * calls this draws every frame at the instant its label names, which is correct
 * there and is also the safest thing to do anywhere else.
 */
void visualiser_set_clock(int64_t (*master_to_local)(int64_t master_us));

/*
 * Drop every frame computed but not yet drawn.
 *
 * Call when the timeline restarts -- a re-anchor or an underrun recovery -- and
 * not for a splice, which visualiser_realign() covers. The difference is what
 * happened to due_us: a splice moves audio around WITHIN a timeline, so queued
 * labels stay true, while a re-anchor establishes a new origin and every label
 * still queued describes an instant on a timeline that no longer exists. Drawn
 * anyway, those become a burst of animation from the old origin at the moment
 * the new one starts.
 *
 * The same shape of bug as the stale phase point in docs/clock-sync.md section
 * 9, which was queued before a timeline restart and left the hub's ring servo
 * dead for the rest of the session.
 */
void visualiser_flush(void);

/*
 * Switch pattern by name. See pattern_count()/pattern_at() in patterns.hpp for
 * what exists; tools/pattern_lab lists them and runs them against a WAV.
 */
void visualiser_set_pattern(const char *name);

/*
 * Tell the visualiser what rate the audio is at.
 *
 * It must be the SAME rate this unit uses to derive the due_us it passes to
 * visualiser_feed() -- `sample_rate` on the hub, `stream_rate` on the satellite.
 * The two are the forward and reverse of one conversion between an instant and a
 * sample position, so if they disagree the count and the timeline separate at
 * exactly their difference: 8.8% for a 48 kHz source against the 44.1 kHz this
 * used to assume, which is 88 ms of divergence per second of audio.
 *
 * Not a preference and not a tuning knob. The source chooses the rate -- the
 * bridge advertises 16, 32, 44.1 and 48 kHz to the phone and takes what it is
 * given -- so this is the firmware finding out, not deciding.
 *
 * Safe from any task, and cheap when the rate has not changed. A change re-cuts
 * the analysis bands, drops the detector history built at the old rate, and
 * re-derives the block origin.
 */
void visualiser_set_rate(uint32_t hz);

/*
 * Tell the visualiser the audio it is about to be fed no longer continues the
 * audio it was fed before -- samples were skipped or inserted between them.
 *
 * Call it after any splice, and after anything else that means audio counted
 * here was not actually heard -- retuning the output clock does exactly that,
 * because disabling the I2S channel discards the DMA buffer. Callable from any
 * task.
 *
 * Block boundaries and due_us are both carried forward by COUNTING what arrives
 * here, from an origin established once against the scheduled timeline. That is
 * what lets two units cut identical blocks without exchanging anything. A splice
 * breaks the count: audio the timeline still accounts for never arrives (a skip)
 * or audio it does not account for does (an insert), and everything after it is
 * mislabelled by the length of the splice -- for good, since nothing re-derives
 * the origin on its own.
 *
 * Each unit splices by its own phase error, so the two strips step apart at
 * every track boundary and never recover. Re-deriving the origin from the next
 * scheduled instant costs one dropped analysis block and puts them back
 * together.
 *
 * Calling this is no longer the only defence. visualiser_feed() compares the
 * count against the `due_master_us` it is handed and re-derives on its own if
 * they have come apart -- see ALIGN_DRIFT_US in visualiser.cpp, and the two
 * silent breakages that motivated it. Still call this: it corrects at the
 * instant of the event rather than once the error has grown to 10 ms, and a
 * caller that knows exactly what it did should say so.
 */
void visualiser_realign(void);

/*
 * Feed interleaved 16-bit stereo PCM. Non-blocking: never delays audio.
 *
 * Feed this from the ARRIVAL path -- where audio enters the playback ring -- and
 * not from where it is handed to the DAC. That is the opposite of what this said
 * before rendering became scheduled, and the reason is worth keeping.
 *
 * Feeding from the DAC meant a frame was computed at the moment its audio was
 * played, which left an algorithm one frame period to run in and no way to look
 * ahead at all. Feeding from arrival puts the ~200 ms the audio is buffered for
 * at the analysis's disposal instead. The objection to it -- that lights driven
 * from arrival run 200 ms ahead of their own speaker -- was correct, and stopped
 * applying when frames began to be drawn at the instant they name rather than
 * the instant they were computed.
 *
 * So: this is safe only while the render stage is scheduled. Move it back to the
 * DAC and the lead disappears; move the scheduling away and the lead returns.
 *
 * `due_master_us` is the master-clock instant this chunk's FIRST sample is
 * SCHEDULED to be heard -- interpolated from the play_at stamps the hub puts on
 * every packet, not read from a clock. That distinction is the whole point.
 *
 * It labels CONTENT. Every unit receives the same play_at for the same audio, so
 * every unit derives the same label for the same sample, and the analysis blocks
 * can be cut at positions all units agree on. Reading a clock at this moment
 * instead -- which an earlier version did -- labels the audio with whenever this
 * particular board happened to get here, and two boards get here a few ms apart
 * through audio phase error and task jitter. That skew lands directly on the
 * block boundaries: 3 ms of it puts the two units 132 samples out of 1024, so
 * one in eight transients is split differently and the marginal ones are
 * detected by one unit and missed by the other.
 *
 * Pass 0 if no timeline is established yet; alignment simply waits.
 */
void visualiser_feed(const uint8_t *pcm, uint32_t len, int64_t due_master_us);

#ifdef __cplusplus
}
#endif
