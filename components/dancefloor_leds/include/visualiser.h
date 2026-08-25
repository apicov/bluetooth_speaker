/**
 * @file visualiser.h
 * @brief FFT -> onset detection -> LED strip, shared by the hub and every
 *        satellite.
 *
 * Each unit analyses its OWN copy of the audio rather than being told what to
 * display. Sending analysis results over the network would add a second thing
 * to keep synchronised; the audio is already synchronised, so anything derived
 * from it locally is synchronised too, for free. A unit that IS given frames
 * is the exception, and it takes them whole -- see visualiser_submit_frame().
 *
 * C API with a C++ implementation: the callers are plain C audio code and
 * should stay that way, while the pattern rendering benefits from the LedStrip
 * wrapper.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Mirrors df::BEAT_BANDS, which is C++ and cannot be seen from here.
 *         visualiser.cpp static_asserts that the two still agree. */
#define VIS_BANDS      4

/**
 * @brief One analysis frame, in the form that can leave the unit that computed
 *        it.
 *
 * This is df::Frame reduced to what a receiver cannot rebuild for itself: the
 * timeline labels and the detector's input. Small enough that a stream of them
 * is a fraction of what the audio already uses on the same radio.
 *
 * THE DETECTOR OUTPUTS ARE ABSENT ON PURPOSE. #band travels at FULL float
 * precision -- it is beat_det_update()'s exact input, and the receiver runs the
 * same detector over the same numbers (df::RemoteDetect) -- so the onset and
 * boom a pattern sees are DERIVED at the far end rather than sent. What still
 * runs in exactly one place is the FFT, which is the part that could not be
 * proved deterministic across two different cores; the detector is plain C
 * over identical input bytes, which needs no proof.
 *
 * THE SPECTRUM DOES NOT TRAVEL either, and that is what keeps this small. Only
 * the pluggable analysers ever read it -- no pattern touches it -- so a
 * satellite with the analysers off would have received most of every frame and
 * discarded it. The consequence is a BUILD rule rather than a runtime one: a
 * unit that takes frames from the wire cannot run the analysers, because the
 * bytes they read are not there, and the Kconfig dependency makes that
 * combination unselectable. A unit that wants models computes its own
 * spectrum, which costs it the FFT.
 *
 * It could not have served the detector's purpose anyway: it is quantised, and
 * flux is a frame-to-frame difference, so the quantisation would land directly
 * on the signal the detector runs on. beat_detect.h says this in full.
 *
 * Declared HERE and not in sync_proto.h, deliberately. This component does not
 * depend on the audio protocol and must stay buildable on its own; the unit
 * that transports a frame is the one that knows how, and copies these bytes
 * into whatever message it sends. Packed for the same reason -- the two ends
 * may not be the same build.
 */
typedef struct __attribute__((packed)) {
    int64_t due_us;         /**< Master-clock instant this frame describes. */
    int64_t index;          /**< Block number, from an origin all units share. */
    float   band[VIS_BANDS];/**< Normalised band energies; the detector's input. */
} vis_frame_t;

/**
 * @brief Register a callback to receive every frame this unit computes.
 *
 * For a unit that sends its frames to others. Registering nothing, which is
 * the default, simply publishes nothing.
 *
 * @param publish  Called on the analysis task, so it must NOT block -- hand
 *                 the bytes to a queue or a socket and return.
 */
void visualiser_set_publish(void (*publish)(const vis_frame_t *f));

/**
 * @brief Hand this unit a frame computed somewhere else.
 *
 * Goes into the same queue local analysis fills and is drawn at the instant it
 * names, so the two sources are interchangeable by construction. Safe from any
 * task.
 *
 * Ignored unless this unit is built to take frames from elsewhere: a unit
 * doing its own analysis AND accepting somebody else's would draw two
 * timelines at once.
 *
 * @param f  The frame.
 */
void visualiser_submit_frame(const vis_frame_t *f);

/**
 * @brief Whether this unit computes its own frames or is given them.
 * @return A word for the log, so a mixed floor is a fact in the capture rather
 *         than a puzzle.
 */
const char *visualiser_source_name(void);

/**
 * @brief The analysis hop this unit was built for, in samples.
 *
 * The other half of the same question. Units on different hops cut different
 * windows and so reach different decisions, and a unit doing its own analysis
 * CANNOT detect that -- nothing crosses between locally analysing units, which
 * is exactly the property that makes them stay in step. So it is reported, and
 * two consoles settle it.
 *
 * @return The hop, in samples.
 */
int visualiser_hop(void);

/**
 * @brief Tell the marker LED whether this unit is joined to the floor.
 *
 * The marker has three states and this supplies one bit of them:
 *
 *     dark        not joined -- booting, or dropped off the AP
 *     solid lit   joined, nothing playing
 *     flashing    audio is being drawn, one flash per master-clock second
 *
 * Which makes the LED answer the two questions asked of a satellite from
 * across a dark field, in the order they are usually asked: is it on the floor
 * at all, and is the floor in step. Without this it answers only the second,
 * and a unit that never joined looks exactly like a joined unit with nothing
 * playing.
 *
 * The FLASH is still the only thing the eye should compare between units. What
 * this adds is the level BETWEEN flashes.
 *
 * Safe from any task; it stores a flag the render task reads. A unit that
 * never calls this -- the hub, which is the AP and has nothing to join --
 * keeps the original behaviour exactly. A no-op unless the marker is
 * configured in, so callers need no conditional of their own.
 *
 * @param up  Whether this unit is on the floor.
 */
void visualiser_marker_set_link(bool up);

/**
 * @brief Blink the marker fast while the boot channel survey is sampling the
 *        band, and stop when it is done.
 *
 * The survey is seconds of scanning and dwelling before the AP exists, and
 * from outside the board that is indistinguishable from a hang. This makes the
 * wait legible without a console: fast blink means "sampling", and the blink
 * STOPPING is the ready signal.
 *
 * IT DOES NOT JOIN THE THREE-STATE SCHEME, deliberately. dark, solid and flash
 * all mean something about a running floor, and this happens before there is
 * one. It runs off its own timer, touches the pin directly, and on stop leaves
 * it dark -- which is where visualiser_start() drives it anyway, so nothing
 * downstream can tell this ran.
 *
 * Safe to call before visualiser_start(); it configures the pin itself.
 * Calling it twice the same way is a no-op, and it is a no-op entirely unless
 * the marker is configured in.
 *
 * @param on  Whether the survey is running.
 */
void visualiser_marker_busy(bool on);

/** @brief Bring up the strip and start the analysis and render tasks. Call
 *         once. */
void visualiser_start(void);

/**
 * @brief Tell the visualiser how to convert a master-clock instant into this
 *        board's local clock.
 *
 * Analysis and display are separate stages: a frame is computed whenever the
 * audio for it arrives and DRAWN when the instant it describes comes round.
 * Waiting for that instant is the one thing in here that needs a clock, so it
 * is the one thing that has to be told about the offset.
 *
 * Passed as a FUNCTION rather than a number because a satellite's offset is
 * not constant -- it is slewed toward the live estimate -- so a value copied
 * once would go stale at exactly the crystal difference.
 *
 * @param master_to_local  The conversion. Leave it unset on the hub, where
 *                         local time IS master time; a unit that never calls
 *                         this draws every frame at the instant its label
 *                         names, which is correct there and the safest thing
 *                         to do anywhere else.
 */
void visualiser_set_clock(int64_t (*master_to_local)(int64_t master_us));

/**
 * @brief Drop every frame computed but not yet drawn.
 *
 * Call when the timeline RESTARTS -- a re-anchor or an underrun recovery --
 * and not for a splice, which visualiser_realign() covers. The difference is
 * what happened to the labels: a splice moves audio around WITHIN a timeline,
 * so queued labels stay true, while a re-anchor establishes a new origin and
 * every label still queued describes an instant on a timeline that no longer
 * exists. Drawn anyway, those become a burst of animation from the old origin
 * at the moment the new one starts.
 */
void visualiser_flush(void);

/**
 * @brief Switch pattern by name.
 * @param name  See df::pattern_count() and df::pattern_at() for what exists;
 *              tools/pattern_lab lists them and runs them against a WAV. An
 *              unknown name leaves the current pattern in place.
 */
void visualiser_set_pattern(const char *name);

/**
 * @brief Tell the visualiser what rate the audio is at.
 *
 * It must be the SAME rate the caller uses to derive the instants it passes to
 * visualiser_feed(). The two are the forward and reverse of one conversion
 * between an instant and a sample position, so if they disagree the count and
 * the timeline separate at exactly their difference -- which for a source rate
 * one step away from the assumed one is tens of milliseconds per second of
 * audio.
 *
 * Not a preference and not a tuning knob. The SOURCE chooses the rate -- the
 * bridge advertises several and takes what the phone gives -- so this is the
 * firmware finding out, not deciding.
 *
 * Safe from any task, and cheap when the rate has not changed. A change
 * re-cuts the analysis bands, drops the detector history built at the old
 * rate, and re-derives the block origin.
 *
 * @param hz  The stream rate.
 */
void visualiser_set_rate(uint32_t hz);

/**
 * @brief Tell the visualiser that the audio it is about to be fed no longer
 *        continues the audio it was fed before -- samples were skipped or
 *        inserted between them.
 *
 * Call it after any splice, and after anything else that means audio counted
 * here was not actually heard: retuning the output clock does exactly that,
 * because disabling the I2S channel discards the DMA buffer. Callable from any
 * task.
 *
 * Block boundaries and frame labels are both carried forward by COUNTING what
 * arrives, from an origin established once against the scheduled timeline.
 * That is what lets two units cut identical blocks without exchanging
 * anything. A splice breaks the count -- audio the timeline still accounts for
 * never arrives, or audio it does not account for does -- and everything after
 * it is mislabelled by the length of the splice, for good, since nothing
 * re-derives the origin on its own. Each unit splices by its own phase error,
 * so the two strips would step apart at every track boundary and never
 * recover.
 *
 * Calling this is no longer the only defence: visualiser_feed() compares the
 * count against the instant it is handed and re-derives on its own if they
 * have come apart. Still call it -- it corrects at the instant of the event
 * rather than once the error has grown past that tolerance, and a caller that
 * knows exactly what it did should say so.
 */
void visualiser_realign(void);

/**
 * @brief Feed interleaved 16-bit stereo PCM. Non-blocking: never delays audio.
 *
 * FEED THIS FROM THE ARRIVAL PATH -- where audio enters the playback ring --
 * and not from where it is handed to the DAC. Feeding from the DAC means a
 * frame is computed at the moment its audio is played, which leaves an
 * algorithm one frame period to run in and no way to look ahead at all.
 * Feeding from arrival puts the whole playback lead at the analysis's
 * disposal. The objection to it -- that lights driven from arrival run ahead
 * of their own speaker -- was correct, and stopped applying when frames began
 * to be DRAWN at the instant they name rather than the instant they were
 * computed. So this is safe only while the render stage is scheduled: move it
 * back to the DAC and the lead disappears; move the scheduling away and the
 * lead returns.
 *
 * @param pcm            Interleaved 16-bit stereo.
 * @param len            Bytes of it.
 * @param due_master_us  The master-clock instant this chunk's FIRST sample is
 *                       SCHEDULED to be heard -- interpolated from the stamps
 *                       the hub puts on every packet, NOT read from a clock.
 *                       That distinction is the whole point: it labels
 *                       CONTENT, so every unit derives the same label for the
 *                       same sample and the analysis blocks can be cut at
 *                       positions all units agree on. Reading a clock here
 *                       labels the audio with whenever this particular board
 *                       happened to get here, and two boards get here
 *                       milliseconds apart -- skew that lands directly on the
 *                       block boundaries, so transients are split differently
 *                       and the marginal ones are detected by one unit and
 *                       missed by the other. Pass 0 if no timeline is
 *                       established yet; alignment simply waits.
 */
void visualiser_feed(const uint8_t *pcm, uint32_t len, int64_t due_master_us);

#ifdef __cplusplus
}
#endif
