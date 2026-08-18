/*
 * The faded catch-up: how a unit closes a large phase error over a few seconds
 * rather than a minute, without waiting for a track boundary to splice at.
 *
 * WHY THIS EXISTS. A lost-packet burst leaves a satellite genuinely late -- the
 * audio is gone, the gap was filled with silence, and the silence took DAC time
 * the timeline does not give back. Measured on the 2026-08-17 soak: steps of
 * +40 to +150 ms on both satellites at once, after hub tx-fail bursts, walking
 * back at the 2.27 ms/s ceiling of RATE_TRIM_MAX_HZ -- a minute or more of
 * audible echo, usually ended by the audible "skipped 150 ms to null phase"
 * boundary splice rather than by the servo.
 *
 * The mechanism is the splice's own move -- drop or insert frames -- applied
 * continuously and inaudibly: a few frames per chunk, each hidden under a
 * short crossfade. At 8 frames per 256-frame chunk the drop side corrects
 * 31 ms of phase per second: a 150 ms error is gone in ~5 s instead of ~70,
 * and the boundary splice is left with nothing to do. The replay side runs
 * at half that (see CATCHUP_SHIFT_MAX_DUP), because a replay bends the pitch
 * down where a drop bends it up, and the down bend is the one a listener
 * names: "the playing sampling sometimes gets slower", 2026-08-18.
 *
 * Both units run it, from this one file, which is the point of putting it in
 * the component rather than copying it into two play.c files: unequal
 * correction between the units is a cross-unit sync error by construction (see
 * rate_trim_hz), and shared code cannot disagree with itself.
 */
#pragma once

#include <stdint.h>

/*
 * Beyond this the servo arms a catch-up debt rather than leaving the error to
 * the fine trim. 25 ms is well clear of the +-7 ms deadband and the few ms of
 * delivery jitter the servo smooths through, and the fine trim's 2.27 ms/s
 * nulls anything under it in ~11 s -- the catch-up is for knocks, not drift.
 */
#define CATCHUP_ARM_US 25000

/* ...and below this an armed catch-up stands down, leaving the remainder (at
 * most this much) to the fine trim. The gap between ARM and CLEAR is the
 * hysteresis that stops it arming and disarming around one flapping reading. */
#define CATCHUP_CLEAR_US 10000

/*
 * No arming for this long after a stream (re)starts, since 2026-08-18.
 *
 * The hub's documented first-minute phase -- the DMA refill transient,
 * measured at -30285 us median on the 2026-08-18 soak -- exceeds
 * CATCHUP_ARM_US, so within the first minutes every unit armed ~1200-1450
 * frames of replay for an error the fine trim was already walking off at
 * ~0.3 ms/s, which meant a few seconds of pitch-bent playback right after
 * every boot for nothing. By 60 s the trim has the residual under the arm
 * threshold. Only the ARM waits; the trim and the stand-down keep working.
 * If a soak still shows first-minute replay, raise this -- to 90 s -- before
 * touching the arm.
 */
#define CATCHUP_HOLD_US 60000000

/*
 * The largest debt armed at once, in us of phase. The same 150 ms as
 * MAX_SPLICE_MS on both units, for the same reason: a larger error means
 * something is wrong that skipping audio will not fix. A bigger error arms
 * again after this much has drained, so a genuinely huge displacement is
 * corrected in steps, each of them faded -- never in one long shred.
 */
#define CATCHUP_MAX_US 150000

/*
 * Frames taken per 256-frame chunk, split by direction since 2026-08-18, and
 * the crossfade that hides each take.
 *
 * Drops (playing late, skip forward) run 8/chunk: 1376 frames/s at the 172
 * chunks/s of a 44.1 kHz stream, 31 ms/s. Replays (playing early, hold back)
 * run 4/chunk, half that: a replay stretches the material -- every replayed
 * frame bends the pitch down, 3.1% of the stream at 8/chunk against 1.6% at
 * 4 -- and the down bend is the audible one on sustained material. The drop
 * bend is the same size and is not free either, but a late unit is chasing
 * real lost time and halving its rate doubles the echo it carries meanwhile.
 * 688 frames/s still clears a full 150 ms debt in ~10 s.
 *
 * The fade is 64 frames, 1.5 ms -- long enough that a crossfade between a
 * signal and itself 8 frames away is inaudible on music, short enough to sit
 * inside one chunk with room for both strands.
 *
 * Both must leave the plain-copy regions non-empty: fade + CATCHUP_SHIFT_MAX
 * + 1 (the fine trim folded in) is 73 against AUDIO_FRAMES 256.
 *
 * CATCHUP_SHIFT_MAX stays as the ceiling both units size their shared shift
 * buffers from -- it is the larger of the two clamps, so the assert.
 */
#define CATCHUP_SHIFT_MAX_DROP 8
#define CATCHUP_SHIFT_MAX_DUP  4
#define CATCHUP_SHIFT_MAX      CATCHUP_SHIFT_MAX_DROP
#define CATCHUP_FADE_FRAMES    64
_Static_assert(CATCHUP_SHIFT_MAX_DUP <= CATCHUP_SHIFT_MAX,
               "the dup clamp must fit the shared shift-buffer sizing");

/*
 * How much silent movement the phase history may accumulate before it is
 * dropped and rebuilt: 20 ms, the same scale as each unit's step-logging
 * threshold. Readings older than that much drain describe a position the unit
 * has left, exactly as readings before a splice do -- and the reset is the
 * same call both units already make there. In frames at the play task's
 * stream rate, it is ~882 at 44.1 kHz: a 150 ms debt ages the history a
 * little under twice on its way down.
 */
#define CATCHUP_HIST_RESET_US 20000

/*
 * Build one output chunk by crossing the input onto a strand `shift` frames
 * away in the same material.
 *
 *   shift > 0   playing LATE: skip forward. Consumes frames+shift input
 *               frames, produces frames output frames.
 *   shift < 0   playing EARLY: hold back. Consumes frames+shift (i.e. fewer)
 *               and produces frames, by playing part of the input twice.
 *
 * The two strands are crossfaded over `fade` frames ending at output frame
 * frames-|shift|, so the fade never runs off the end of either strand: the
 * plain head needs src up to that point, the tail reads src[i+shift] up to
 * exactly the last input frame, and the next chunk begins at src[frames+shift]
 * adjacent to the last frame emitted -- continuous in both directions with no
 * amplitude dip, because a crossfade of a signal with itself at a small offset
 * sums to roughly the signal.
 *
 * Call only with 1 < |shift| (the fine trim handles +-1 without this), fade
 * >= 2, |shift| + fade < frames, and non-overlapping dst and src. dst
 * receives exactly `frames` output frames; the caller knows the input count
 * (frames+shift) and counts it in samples_played, exactly as the boundary
 * splice counts what it discards.
 */
void audio_shift_chunk(int16_t *dst, const int16_t *src, unsigned frames,
                       int shift, unsigned fade, unsigned channels);
