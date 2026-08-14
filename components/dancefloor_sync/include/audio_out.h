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
