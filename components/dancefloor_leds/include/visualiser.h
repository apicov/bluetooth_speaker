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
 * Feed this from the PLAYBACK path -- where samples are handed to the DAC -- and
 * not from wherever they arrive. Audio sits in a ~200 ms buffer between the two,
 * and lights driven from the arrival side run that far ahead of their own
 * speaker, which is clearly visible.
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
