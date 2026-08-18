#pragma once

/*
 * Which of the two channels this unit's speaker plays.
 *
 * Every unit carries the same stereo stream -- the wire format is fixed at
 * AUDIO_CHANNELS interleaved int16 and nothing here changes that -- so placing
 * one speaker as the left of a pair is a decision about what to put in the DMA
 * buffer, taken on the unit that plays it.
 *
 * A selected channel is copied into BOTH slots rather than the other slot being
 * muted. Which slot an amplifier latches is a hardware strap (the MAX98357A's SD
 * pin, a PCM5102A's mono wiring) and is not knowable from here, so muting a slot
 * is a coin flip between the right answer and silence. Duplicating is right
 * either way, and costs no loudness.
 *
 * Frame count and byte count are untouched by construction: this only decides
 * what lands in each slot of a frame that already exists. Nothing downstream --
 * samples_played, the phase queue, the splice arithmetic, the rate servo --
 * can see it, which is the property that makes it safe to do here rather than
 * upstream of the timeline.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sdkconfig.h"
#include "sync_proto.h"

#if CONFIG_DANCEFLOOR_OUT_LEFT
#define AUDIO_CHANNEL_MODE_NAME "left"
#elif CONFIG_DANCEFLOOR_OUT_RIGHT
#define AUDIO_CHANNEL_MODE_NAME "right"
#elif CONFIG_DANCEFLOOR_OUT_MONO
#define AUDIO_CHANNEL_MODE_NAME "mono"
#else
#define AUDIO_CHANNEL_MODE_NAME "stereo"
#endif

/*
 * Rewrite n_frames interleaved stereo frames in place for this unit's mode.
 *
 * Call it on the last buffer before the output and nowhere else: the copy in
 * the ring is the stream as received, and anything reading that -- the
 * visualiser, a splice, a short-read pad -- wants both channels.
 *
 * Under the default stereo build the body is empty and every call disappears,
 * so an untouched image is bit-identical to one built before this existed.
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
        /* int32 intermediate: two hard-panned peaks sum past INT16_MAX, and
         * the wrap is a full-scale sign flip -- an audible click rather than
         * the clip it would be mistaken for. */
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

/*
 * Playback volume, applied here rather than by the phone.
 *
 * Without an AVRCP TARGET the handset has nowhere to send a volume command, so
 * it scales the PCM itself before the SBC encoder -- and every step down the
 * slider is resolution thrown away before the audio ever reaches the air. The
 * bridge advertises a target now, so the phone transmits at full scale and says
 * how loud it wants it; this is where that is honoured.
 *
 * AVRCP absolute volume is 0-127 by specification. 127 is unity and is the
 * default everywhere, so a unit that has never been told a volume plays at full
 * level rather than silently.
 *
 * SQUARE LAW, not linear. A linear gain spends most of the slider's travel in
 * the top few dB and drops off a cliff at the bottom; squaring approximates an
 * audio taper closely enough to feel right, in two multiplies and no table.
 *
 * INTEGER, and therefore identical on both units. The hub is an LX7 and the
 * satellite an LX6; anything involving a float here could round differently on
 * the two and put the same stream out at two different levels. Same reasoning as
 * the fixed-point resampler in dancefloor_leds.
 *
 * Frame count and byte count are untouched, exactly as for the channel mode
 * above, which is what makes it safe to do at the output rather than upstream of
 * the timeline. samples_played, the phase queue, the splice arithmetic and the
 * rate servo cannot see it.
 *
 * AUDIO_VOL_MAX is in sbc_link.h, with the wire type that carries it.
 */
static inline int32_t audio_volume_q15(uint8_t vol)
{
    if (vol >= AUDIO_VOL_MAX) {
        return 32768;                       /* exactly unity, no rounding loss */
    }
    return ((int32_t)vol * vol * 32768) / (AUDIO_VOL_MAX * AUDIO_VOL_MAX);
}

/*
 * Scale n_frames interleaved frames in place. Call it on the last buffer before
 * the output, beside audio_apply_channel_mode(), and nowhere else -- what is in
 * the ring must stay the stream as received, or a splice would play attenuated
 * audio at a stale gain.
 *
 * Full volume returns immediately, so the common case costs one comparison and
 * the image is bit-identical to one built before this existed.
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

/*
 * THE OUTPUT IS 32-BIT, AND THE RING IS NOT. Two byte-currencies, kept apart.
 *
 * AUDIO_CHUNK_BYTES (sync_proto.h) stays the RING domain: int16 frames, what the
 * stream is made of, what samples_played counts, what audio_shift_chunk() moves
 * around. AUDIO_OUT_* below is the DAC domain, and the only place a byte count
 * crosses between them is the conversion in this header.
 *
 * WHY WIDEN AT ALL. Attenuating a 16-bit sample into 16 bits spends the
 * attenuation out of the signal: at -37 dB, which is 15/127 on the taper, under
 * ten of the sixteen bits survive and everything below 71 truncates to 0 or -1.
 * That is not a quiet version of the music, it is a distorted one, and what it
 * discards it discards as signal-correlated error rather than as noise -- which
 * is why a quiet passage at a low setting stops sounding like anything at all.
 * Widening first makes the multiply EXACT: all sixteen bits of programme
 * material reach the DAC at every level, and the only thing attenuation changes
 * is how far above the DAC's own fixed noise floor they sit.
 *
 * The frame COUNT is untouched by all of this, which is the property that keeps
 * it safe here. samples_played, the phase queue, the splice arithmetic and the
 * rate servo count frames, and a frame is still a frame; only its width on the
 * wire to the DAC changed, at the last possible moment, after every counter has
 * been updated.
 */
typedef int32_t audio_out_sample_t;
#define AUDIO_OUT_FRAME_BYTES (AUDIO_CHANNELS * (int)sizeof(audio_out_sample_t))
#define AUDIO_OUT_CHUNK_BYTES (AUDIO_FRAMES * AUDIO_OUT_FRAME_BYTES)

/*
 * Widen n_frames of interleaved int16 into the 32-bit output buffer, applying
 * the level on the way.
 *
 * OUT OF PLACE, which is what lets the splice's `const quiet` buffer go through
 * exactly the same path as real audio. The gain used to have to sit outside the
 * write for that reason; it does not any more, and silence needs widening even
 * though it needs no attenuating.
 *
 * THE MULTIPLY CANNOT OVERFLOW, and it is worth showing rather than asserting.
 * The gain is q15 with unity 32768, so the doubling below puts unity at exactly
 * 1 << 16:
 *
 *     in = -32768, g = 32768:  -1073741824 * 2 = -2147483648 = INT32_MIN, exact
 *     in = +32767, g = 32768:   1073709056 * 2 =  2147418112 < INT32_MAX
 *
 * Every other (in, g) is smaller in magnitude than one of those two, so there is
 * no saturation branch because there is nothing to saturate. There is no
 * rounding term either, and no dither: nothing is discarded, so there is no
 * quantisation error to round or to shape. That absence is deliberate and this
 * paragraph is why it is not an oversight.
 */
static inline void audio_volume_write_i32(audio_out_sample_t *out,
                                          const int16_t *in,
                                          size_t n_frames, uint8_t vol)
{
    const int32_t g = audio_volume_q15(vol);
    const size_t n = n_frames * AUDIO_CHANNELS;
    for (size_t i = 0; i < n; i++) {
        out[i] = (int32_t)in[i] * g * 2;
    }
}

/*
 * What to play when nobody has said how loud: NOTHING, for a while.
 *
 * "Never been told a level" and "was told full scale" used to be the same state,
 * because the default WAS full scale. sbc_link.h argued for that: a missed
 * message should play loud rather than silent. The soak capture of 2026-08-18
 * is what disproves it -- `sat1, uptime 5997, VOLUME 20/127`, a satellite that
 * played about six seconds before it learnt the level. The room was set to
 * 20/127. The unit played at 127. That is 32 dB too loud, into a floor of
 * people, for six seconds.
 *
 * The two failures are not symmetrical and were being treated as if they were. A
 * speaker that is briefly silent is a speaker somebody walks over to. A speaker
 * that is briefly 32 dB too loud is not a fault you get to investigate calmly,
 * and at the bottom of the taper -- where this floor is actually used -- the
 * error is at its largest.
 *
 * So: silence until told, and the old rule kept only as a bounded fallback, for
 * the case it was really protecting against -- a hub that is never going to say
 * anything, because it is running a build from before any of this existed. That
 * is a bench condition with somebody standing next to it, not a party.
 *
 * A satellite cannot play audio it has not been sent, and the unit that sends
 * the audio is the unit that sends the level, so in the healthy case there is
 * nothing to wait for: the level arrives on the join push, before the first
 * packet it could apply to.
 *
 * THREE RULES, each of which is a way this goes wrong if forgotten:
 *
 * 1. The fallback is a LOCAL PLAYBACK DECISION and is never relayed. If the hub
 *    folded it back into audio_volume and broadcast that, a hub whose bridge had
 *    died would blast a floor whose satellites were sitting correctly at -50 dB.
 *    It lives here, in a pure function, and the senders are gated on `known`.
 * 2. `known` is STICKY. A hub going away does not make the last level wrong; it
 *    takes the audio with it. Nothing ever clears it.
 * 3. A muted unit STILL WRITES. Skipping the write to save the work would stall
 *    the DAC, break the wrote_at phase reference and bring the unit back out of
 *    position. Zeros cost the same as samples.
 *
 * Deliberately told-and-zero is honoured as zero. Somebody who muted the room
 * from the phone is not somebody who has said nothing.
 */
static inline uint8_t audio_vol_effective(uint8_t vol, bool known, bool fallback_due)
{
    if (known) {
        return vol;
    }
    return fallback_due ? AUDIO_VOL_MAX : 0;
}

/*
 * How long a unit stays silent before deciding nobody is ever going to tell it.
 *
 * Measured FROM BOOT, not from playback start. The question is "has anything
 * ever told us", and a live hub answers it within milliseconds of the DHCP lease
 * -- long before audio can anchor. From playback start would instead impose a
 * fresh silence at every re-anchor, which is a healthy event.
 *
 * Thirty seconds against a 1 s repeat from the hub and a 5 s heartbeat from the
 * bridge: the healthy case wins that race by a factor of six, and what is left
 * is long enough that it cannot be reached by a burst of loss.
 */
#define AUDIO_VOL_UNKNOWN_HOLD_US 30000000
