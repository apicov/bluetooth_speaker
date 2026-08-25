/**
 * @file audio_out.h
 * @brief The last two things done to audio before it reaches the DAC: which
 *        channel this unit plays, and how loud.
 *
 * Both are applied on the LAST buffer before the output and nowhere else. What
 * sits in the ring must stay the stream as received, because everything that
 * reads it -- the visualiser, a splice, a short-read pad -- wants the stream
 * rather than this unit's rendering of it.
 *
 * The frame COUNT is untouched by all of it, which is the property that makes
 * doing it here safe. samples_played, the phase queue, the splice arithmetic
 * and the rate servo all count frames, and a frame is still a frame; only what
 * is in its slots, and how wide those slots are on the wire to the DAC,
 * changes -- at the last possible moment, after every counter has been
 * updated.
 *
 * Integer throughout, and the tapers are integer tables. The hub and the
 * satellite are different cores; a float evaluated on both could round
 * differently and put the same stream out at two levels, which is the same
 * class of fault as the two disagreeing about rate. The floats live in
 * tools/gen_vol_table.py and never in an image.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sdkconfig.h"
#include "sync_proto.h"

/**
 * @brief This unit's channel mode, as a word for the boot log.
 *
 * Every unit carries the same stereo stream -- the wire format is fixed at
 * AUDIO_CHANNELS interleaved int16 and nothing here changes that -- so placing
 * one speaker as the left of a pair is a decision about what to put in the DMA
 * buffer, taken on the unit that plays it.
 */
#if CONFIG_DANCEFLOOR_OUT_LEFT
#define AUDIO_CHANNEL_MODE_NAME "left"
#elif CONFIG_DANCEFLOOR_OUT_RIGHT
#define AUDIO_CHANNEL_MODE_NAME "right"
#elif CONFIG_DANCEFLOOR_OUT_MONO
#define AUDIO_CHANNEL_MODE_NAME "mono"
#else
#define AUDIO_CHANNEL_MODE_NAME "stereo"
#endif

/**
 * @brief Rewrite interleaved stereo frames in place for this unit's mode.
 *
 * A selected channel is copied into BOTH slots rather than the other slot
 * being muted. Which slot an amplifier latches is a hardware strap and is not
 * knowable from here, so muting a slot is a coin flip between the right answer
 * and silence. Duplicating is right either way, and costs no loudness.
 *
 * Under the default stereo build the body is empty and every call disappears,
 * so an untouched image is bit-identical to one built before this existed.
 *
 * @param frames   The buffer, interleaved AUDIO_CHANNELS int16 per frame.
 * @param n_frames How many frames it holds.
 */
static inline void audio_apply_channel_mode(int16_t *frames, size_t n_frames)
{
#if CONFIG_DANCEFLOOR_OUT_LEFT || CONFIG_DANCEFLOOR_OUT_RIGHT || CONFIG_DANCEFLOOR_OUT_MONO
    for (size_t i = 0; i < n_frames; i++) {
        int16_t *f = &frames[i * AUDIO_CHANNELS];
#if CONFIG_DANCEFLOOR_OUT_LEFT
        f[1] = f[0];
#elif CONFIG_DANCEFLOOR_OUT_RIGHT
        f[0] = f[1];
#else

        const int16_t s = (int16_t)(((int32_t)f[0] + (int32_t)f[1]) / 2);
        f[0] = s;
        f[1] = s;
#endif
    }
#else
    (void)frames;
    (void)n_frames;
#endif
}

/**
 * @brief The playback gain for a level, in Q16 with unity at 65536.
 *
 * Playback volume is applied here rather than by the phone. Without an AVRCP
 * target the handset has nowhere to send a volume command, so it scales the
 * PCM itself before the SBC encoder -- and every step down the slider is
 * resolution thrown away before the audio ever reaches the air. The bridge
 * advertises a target, so the phone transmits at full scale and says how loud
 * it wants it; this is where that is honoured.
 *
 * LINEAR IN dB, from a deep floor at 1 up to unity at AUDIO_VOL_MAX, with 0 a
 * hard mute that is not on the curve. Every step of the slider is the same
 * fraction of a dB, so one click of the phone -- which steps in multiples of
 * five -- is the same change in loudness wherever the thumb is. A linear or
 * square-law gain instead spends the whole usable range in the bottom of the
 * travel, which is where it has the least resolution to spend.
 *
 * Q16 IS THE REAL TABLE and q15 is derived from it: unity at 65536 is what
 * makes the widening multiply in audio_volume_write_i32() exact -- unity
 * becomes a shift of 16 -- and halving it lands on exactly 32768 for the
 * 16-bit path, with no rounding loss at the top and no loss of monotonicity
 * anywhere.
 *
 * @param vol  0..AUDIO_VOL_MAX; larger values are clamped.
 * @return The gain, Q16.
 */
static inline int32_t audio_volume_q16(uint8_t vol)
{
    static const int32_t taper[AUDIO_VOL_MAX + 1] = {
         0,     66,     69,     73,     77,     82,     86,     91,
        96,    102,    107,    113,    120,    127,    134,    141,
       149,    158,    166,    176,    186,    196,    207,    219,
       231,    244,    258,    273,    288,    304,    321,    339,
       359,    379,    400,    423,    446,    472,    498,    526,
       556,    587,    620,    655,    692,    731,    773,    816,
       862,    911,    962,   1016,   1073,   1134,   1198,   1265,
      1337,   1412,   1491,   1576,   1664,   1758,   1857,   1962,
      2072,   2189,   2313,   2443,   2581,   2726,   2880,   3042,
      3213,   3394,   3586,   3788,   4001,   4227,   4465,   4717,
      4982,   5263,   5560,   5873,   6204,   6554,   6923,   7313,
      7725,   8161,   8620,   9106,   9619,  10161,  10734,  11339,
     11978,  12653,  13366,  14119,  14915,  15756,  16643,  17581,
     18572,  19619,  20724,  21892,  23126,  24429,  25806,  27260,
     28796,  30419,  32133,  33944,  35857,  37878,  40012,  42267,
     44649,  47165,  49823,  52631,  55597,  58730,  62040,  65536,
    };
    return taper[vol > AUDIO_VOL_MAX ? AUDIO_VOL_MAX : vol];
}

/**
 * @brief The same taper for the 16-bit path.
 *
 * A shift, not a second table: 65536 >> 1 is exactly 32768 and 0 >> 1 is
 * exactly 0, so the endpoints stay exact, and halving a strictly increasing
 * table leaves it strictly increasing -- checked in test_sync_proto.c rather
 * than assumed.
 *
 * @param vol  0..AUDIO_VOL_MAX.
 * @return The gain, Q15.
 */
static inline int32_t audio_volume_q15(uint8_t vol)
{
    return audio_volume_q16(vol) >> 1;
}

/**
 * @brief Scale interleaved frames in place, 16-bit in and 16-bit out.
 *
 * Full volume returns immediately, so the common case costs one comparison.
 * Prefer audio_volume_write_i32() wherever the output is 32-bit: attenuating
 * within 16 bits spends the attenuation out of the signal.
 *
 * @param frames   The buffer; must be the last one before the output.
 * @param n_frames How many frames it holds.
 * @param vol      0..AUDIO_VOL_MAX.
 */
static inline void audio_apply_volume(int16_t *frames, size_t n_frames, uint8_t vol)
{
    if (vol >= AUDIO_VOL_MAX) {
        return;
    }
    const int32_t g = audio_volume_q15(vol);
    const size_t n = n_frames * AUDIO_CHANNELS;
    for (size_t i = 0; i < n; i++) {
        frames[i] = (int16_t)(((int32_t)frames[i] * g) >> 15);
    }
}

/**
 * @brief One sample as the DAC takes it.
 *
 * THE OUTPUT IS 32-BIT AND THE RING IS NOT: two byte-currencies, kept apart.
 * AUDIO_CHUNK_BYTES (sync_proto.h) is the RING domain -- int16 frames, what
 * the stream is made of, what samples_played counts, what audio_shift_chunk()
 * moves around. AUDIO_OUT_* is the DAC domain, and the only place a byte count
 * crosses between them is the conversion in this header.
 *
 * WHY WIDEN AT ALL. Attenuating a 16-bit sample into 16 bits spends the
 * attenuation out of the signal: well down the taper only a fraction of the
 * sixteen bits survive, and the smallest samples truncate to zero. That is not
 * a quiet version of the music but a distorted one, and what it discards it
 * discards as signal-correlated error rather than as noise -- which is why a
 * quiet passage at a low setting stops sounding like anything at all.
 * Widening first makes the multiply EXACT: all sixteen bits of programme
 * material reach the DAC at every level, and the only thing attenuation
 * changes is how far above the DAC's own fixed noise floor they sit.
 */
typedef int32_t audio_out_sample_t;
/** @brief Bytes per frame in the DAC domain. */
#define AUDIO_OUT_FRAME_BYTES (AUDIO_CHANNELS * (int)sizeof(audio_out_sample_t))
/** @brief Bytes per chunk in the DAC domain, the counterpart of
 *         AUDIO_CHUNK_BYTES. */
#define AUDIO_OUT_CHUNK_BYTES (AUDIO_FRAMES * AUDIO_OUT_FRAME_BYTES)

/**
 * @brief The gain a unit is currently at, which is not always the gain it was
 *        told.
 *
 * A level change is a step, and the largest one in this system is the 0 ->
 * level that happens when a unit is finally told how loud, mid-programme, out
 * of silence. Stepping a gain between two samples is a click; moving it across
 * a chunk is not, and one chunk is far too short to hear as a fade and far too
 * long to hear as an edge.
 *
 * Owned by the playback task and read by nothing else, so it needs no volatile
 * and no lock. Zeroed at playback start, which also removes the cold-start
 * click for free.
 *
 * NOT SYNCHRONISED BETWEEN UNITS, deliberately. Two speakers starting a
 * few-tens-of-milliseconds fade a few milliseconds apart is inaudible; making
 * them agree would mean putting the level on the timeline, which is a wire
 * format change for a property nobody can hear.
 */
typedef struct {
    int32_t cur;   /**< Q16, as audio_volume_q16() returns. */
} audio_ramp_t;

/** @brief Gain movement allowed per chunk: full travel in eight of them. One
 *         slider click resolves inside a single chunk; only the mute-to-audible
 *         jump uses the whole ramp, which is the one that needs it. */
#define AUDIO_RAMP_STEP_Q16 8192

/**
 * @brief Widen and scale into the 32-bit output buffer, moving the gain toward
 *        its target across the chunk.
 *
 * OUT OF PLACE, which is what lets the splice's `const` silence buffer go
 * through exactly the same path as real audio -- silence needs widening even
 * though it needs no attenuating.
 *
 * THE MULTIPLY CANNOT OVERFLOW, and it is worth showing rather than asserting.
 * The gain is Q16 with unity 65536, so unity is exactly a shift of 16:
 *
 *     in = -32768, g = 65536:  -2147483648 = INT32_MIN, exactly representable
 *     in = +32767, g = 65536:   2147418112 < INT32_MAX
 *
 * Every other (in, g) is smaller in magnitude than one of those two, so there
 * is no saturation branch because there is nothing to saturate. There is no
 * rounding term either, and no dither: nothing is discarded, so there is no
 * quantisation error to round or to shape. That absence is deliberate, and
 * this paragraph is why it is not an oversight.
 *
 * The gain is interpolated PER FRAME rather than held for the chunk and
 * stepped at its boundary -- a staircase of chunk-sized steps is just a
 * quieter click. The interpolation product stays well inside int32, and it is
 * integer throughout, so both chips compute the same samples.
 *
 * @param[out] out  DAC-domain buffer, n_frames * AUDIO_CHANNELS samples.
 * @param in        Ring-domain input, the same frame count.
 * @param n_frames  Frames to convert.
 * @param vol       0..AUDIO_VOL_MAX, the level being moved toward.
 * @param ramp      Where the gain has got to; updated by one step.
 */
static inline void audio_volume_write_i32(audio_out_sample_t *out,
                                          const int16_t *in,
                                          size_t n_frames, uint8_t vol,
                                          audio_ramp_t *ramp)
{
    const int32_t target = audio_volume_q16(vol);
    const int32_t from = ramp->cur;
    int32_t step = target - from;
    if (step > AUDIO_RAMP_STEP_Q16) {
        step = AUDIO_RAMP_STEP_Q16;
    } else if (step < -AUDIO_RAMP_STEP_Q16) {
        step = -AUDIO_RAMP_STEP_Q16;
    }
    ramp->cur = from + step;

    for (size_t f = 0; f < n_frames; f++) {
        const int32_t g = from + (int32_t)(step * (int32_t)f / (int32_t)n_frames);
        for (size_t c = 0; c < AUDIO_CHANNELS; c++) {
            const size_t i = f * AUDIO_CHANNELS + c;
            out[i] = (int32_t)in[i] * g;
        }
    }
}

/**
 * @brief What to play when nobody has said how loud: NOTHING, for a while.
 *
 * "Never been told a level" and "was told full scale" were once the same
 * state, because the default WAS full scale, on the argument that a missed
 * message should play loud rather than silent. The two failures are not
 * symmetrical and that argument treated them as if they were. A speaker that
 * is briefly silent is a speaker somebody walks over to. A speaker that is
 * briefly tens of dB too loud, into a floor of people, is not a fault you get
 * to investigate calmly -- and at the bottom of the taper, where this floor is
 * actually used, that error is at its largest.
 *
 * So: silence until told, with the old rule kept only as a bounded fallback
 * for the case it was really protecting against -- a hub that is never going
 * to say anything, because it is running a build from before any of this
 * existed. That is a bench condition with somebody standing next to it.
 *
 * In the healthy case there is nothing to wait for: a satellite cannot play
 * audio it has not been sent, and the unit that sends the audio sends the
 * level, on the join push, before the first packet it could apply to.
 *
 * THREE RULES, each of which is a way this goes wrong if forgotten:
 *
 * 1. The fallback is a LOCAL PLAYBACK DECISION and is never relayed. If the
 *    hub folded it back into audio_volume and sent that, a hub whose bridge
 *    had died would blast a floor whose satellites were sitting correctly at
 *    the bottom of the taper. It lives here, in a pure function, and the
 *    senders are gated on @p known.
 * 2. @p known is STICKY. A hub going away does not make the last level wrong;
 *    it takes the audio with it. Nothing ever clears it.
 * 3. A muted unit STILL WRITES. Skipping the write to save the work would
 *    stall the DAC, break the phase reference the play task dates its readings
 *    from, and bring the unit back out of position. Zeros cost the same as
 *    samples.
 *
 * Deliberately told-and-zero is honoured as zero: somebody who muted the room
 * from the phone is not somebody who has said nothing.
 *
 * @param vol           The level last received.
 * @param known         Whether anything has ever stated one.
 * @param fallback_due  Whether AUDIO_VOL_UNKNOWN_HOLD_US has elapsed.
 * @return The level to actually play at.
 */
static inline uint8_t audio_vol_effective(uint8_t vol, bool known, bool fallback_due)
{
    if (known) {
        return vol;
    }
    return fallback_due ? AUDIO_VOL_MAX : 0;
}

/**
 * @brief How long a unit stays silent before deciding nobody will ever tell it.
 *
 * Measured FROM BOOT, not from playback start. The question is "has anything
 * ever told us", and a live hub answers it within milliseconds of the DHCP
 * lease, long before audio can anchor. From playback start it would instead
 * impose a fresh silence at every re-anchor, which is a healthy event.
 *
 * Thirty seconds against a one-second repeat from the hub and a five-second
 * heartbeat from the bridge: the healthy case wins that race by a wide margin,
 * and what is left is long enough that it cannot be reached by a burst of
 * loss.
 */
#define AUDIO_VOL_UNKNOWN_HOLD_US 30000000
