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

/* Brings up the strip and starts the analysis task. Call once. */
void visualiser_start(void);

/*
 * Add this to esp_timer_get_time() to get master-clock time. The hub passes 0,
 * since its local clock IS the master clock; a satellite passes the offset it
 * anchored playback with, and again whenever it re-anchors.
 *
 * Anything in a pattern that advances on its own -- a hue drift, a chase, a
 * decay -- has to be a function of this rather than of how many frames this
 * board happens to have rendered. Render counts differ between units: audio
 * arrives in different-sized lumps, tasks are scheduled differently, and a
 * starved unit renders extra decay frames. Keyed to frame count, two units
 * drift apart and beat against each other. Keyed to the shared clock they
 * agree, and cannot accumulate error, because nothing is being integrated.
 */
void visualiser_set_master_offset(int64_t offset_us);

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
