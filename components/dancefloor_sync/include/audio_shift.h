/**
 * @file audio_shift.h
 * @brief The faded catch-up: how a unit closes a large phase error over a few
 *        seconds rather than a minute, without waiting for a track boundary.
 *
 * A lost-packet burst leaves a unit genuinely late -- the audio is gone, the
 * gap was filled with silence, and the silence took output time the timeline
 * does not give back. The fine rate trim can only walk that back at
 * RATE_TRIM_MAX_HZ, which for a knock of a hundred milliseconds or more is a
 * minute of audible echo, usually ended by the audible boundary splice rather
 * than by the servo.
 *
 * The mechanism here is the splice's own move -- drop or replay frames --
 * applied continuously and inaudibly: a few frames per chunk, each hidden
 * under a short crossfade. That corrects tens of milliseconds per second
 * instead of a couple, so the error is gone in seconds and the boundary splice
 * is left with nothing to do.
 *
 * Both units run it from this one file, which is the point of putting it in
 * the component rather than copying it into two play.c files: unequal
 * correction between the units is a cross-unit sync error by construction, and
 * shared code cannot disagree with itself.
 */
#pragma once

#include <stdint.h>

/**
 * @brief Widest trim the servo may ever ask for, in Hz, and the boundary
 *        between its two actuators.
 *
 * Within it the correction is made in SOFTWARE, by dropping or duplicating one
 * frame at a time. Beyond it only the clock can help: a source running far
 * enough from the output drains a buffer in seconds, which no drop rate short
 * of shredding the audio would absorb.
 *
 * Real drift between two crystals is a few parts per million, so an ordinary
 * fine correction is well under a hertz and this is orders of magnitude above
 * it; the depth net below asks for 20 Hz. Anything reaching this bound is a
 * broken measurement rather than a correction, which is why the clamp exists
 * at all.
 *
 * ONE NUMBER FOR BOTH UNITS. It is the ceiling on how fast either may correct,
 * so the two disagreeing about it is a cross-unit sync error by exactly the
 * argument above. Each unit's own header keeps the half of the reasoning that
 * is its own: the satellite's coarse case is not hypothetical, because
 * i2s_start() runs at a fixed rate before any stream exists, and the hub's is
 * why the bound is deliberately NOT applied inside retune_dac().
 */
#define RATE_TRIM_MAX_HZ 100

/**
 * @brief Phase error beyond which the servo arms a catch-up debt rather than
 *        leaving it to the fine trim.
 *
 * Well clear of PHASE_DEADBAND_US and of the delivery jitter the servo smooths
 * through, and low enough that the fine trim nulls anything under it in a few
 * seconds. The catch-up is for knocks, not for drift.
 */
#define CATCHUP_ARM_US 25000

/** @brief ...and below this an armed catch-up stands down, leaving the
 *         remainder to the fine trim. The gap between ARM and CLEAR is the
 *         hysteresis that stops it arming and disarming around one flapping
 *         reading. */
#define CATCHUP_CLEAR_US 10000

/**
 * @brief No arming for this long after a stream (re)start.
 *
 * A fresh timeline's first phase readings are past CATCHUP_ARM_US with nothing
 * wrong: the output DMA is refilling, so the readings inside that window are
 * not paced by the DAC and do not describe a position error. Armed on, they
 * buy a few seconds of pitch-bent playback after every start for an error the
 * fine trim is already walking off.
 *
 * Only the ARM waits. The trim and the stand-down keep working throughout, so
 * a real error present at a start is still corrected -- just not by replaying
 * material into a ring that has only begun to fill.
 */
#define CATCHUP_HOLD_US 60000000

/**
 * @brief The largest debt armed at once, in us of phase.
 *
 * The same figure as MAX_SPLICE_MS on both units, for the same reason: a
 * larger error means something is wrong that skipping audio will not fix. A
 * bigger error arms again once this much has drained, so a genuinely huge
 * displacement is corrected in steps, each of them faded -- never in one long
 * shred.
 */
#define CATCHUP_MAX_US 150000

/**
 * @brief Frames taken per chunk when playing LATE, i.e. skipping forward.
 *
 * The two directions are deliberately not the same size. A replay stretches
 * the material and bends the pitch DOWN; a drop bends it up by the same
 * amount. The down bend is the one a listener names, so the replay side runs
 * at half this rate -- see CATCHUP_SHIFT_MAX_DUP. The drop side is not free
 * either, but a late unit is chasing real lost time, and halving its rate
 * doubles the echo it carries meanwhile.
 */
#define CATCHUP_SHIFT_MAX_DROP 8
/** @brief Frames taken per chunk when playing EARLY, i.e. holding back. Half
 *         the drop rate; see CATCHUP_SHIFT_MAX_DROP for why. */
#define CATCHUP_SHIFT_MAX_DUP  4
/** @brief The larger of the two clamps, which is what both units size their
 *         shared shift buffers from. Hence the assertion below. */
#define CATCHUP_SHIFT_MAX      CATCHUP_SHIFT_MAX_DROP
/**
 * @brief The crossfade that hides each take, in frames.
 *
 * Long enough that a crossfade between a signal and itself a few frames away
 * is inaudible on music, short enough to sit inside one chunk with room for
 * both strands. Both clamps must leave the plain-copy regions non-empty: fade
 * plus CATCHUP_SHIFT_MAX plus one for the fine trim folded in, against
 * AUDIO_FRAMES.
 */
#define CATCHUP_FADE_FRAMES    64
_Static_assert(CATCHUP_SHIFT_MAX_DUP <= CATCHUP_SHIFT_MAX,
               "the dup clamp must fit the shared shift-buffer sizing");

/**
 * @brief How much silent movement the phase history may accumulate before it
 *        is dropped and rebuilt.
 *
 * Readings older than that much drain describe a position the unit has left,
 * exactly as readings taken before a splice do -- and the reset is the same
 * call both units already make there. Set at the same scale as each unit's
 * step-logging threshold, so a full debt ages the history a small number of
 * times on its way down rather than once or continuously.
 */
#define CATCHUP_HIST_RESET_US 20000

/**
 * @brief Build one output chunk by crossing the input onto a strand `shift`
 *        frames away in the same material.
 *
 * Both directions run through the same expressions:
 *
 *     shift > 0   playing LATE: skip forward. Consumes frames+shift input
 *                 frames and produces `frames` output frames.
 *     shift < 0   playing EARLY: hold back. Consumes frames+shift (fewer) and
 *                 produces `frames`, by playing part of the input twice.
 *
 * The two strands are crossfaded over `fade` frames ending at output frame
 * frames-|shift|, so the fade never runs off the end of either: the plain head
 * needs src up to that point, the tail reads src[i+shift] up to exactly the
 * last input frame, and the next chunk begins at src[frames+shift] adjacent to
 * the last frame emitted. Continuous in both directions with no amplitude dip,
 * because a crossfade of a signal with itself at a small offset sums to
 * roughly the signal.
 *
 * @param dst       Receives exactly @p frames output frames; must not overlap
 *                  @p src.
 * @param src       Input, holding frames+shift frames.
 * @param frames    Output frames to produce.
 * @param shift     Frames to skip (positive) or replay (negative). Call only
 *                  with 1 < |shift|; the fine trim handles +-1 without this.
 * @param fade      Crossfade length in frames; >= 2, and |shift| + fade <
 *                  @p frames.
 * @param channels  Interleaved channels per frame.
 *
 * The caller knows the input count (frames+shift) and counts it in
 * samples_played, exactly as the boundary splice counts what it discards.
 */
void audio_shift_chunk(int16_t *dst, const int16_t *src, unsigned frames,
                       int shift, unsigned fade, unsigned channels);
