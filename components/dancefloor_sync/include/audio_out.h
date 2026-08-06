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
