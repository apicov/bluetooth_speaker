/*
 * M2 + M3: music-reactive NeoPixels.
 *
 * The A2DP callback forks PCM here. Analysis and LED rendering happen on a
 * separate task pinned to core 1, well away from the Bluetooth stack.
 */
#pragma once

#include <stdint.h>

void visualiser_start(void);

/*
 * Feed interleaved 16-bit stereo PCM. Called from the Bluetooth task, so this
 * must never block: analysis is best-effort and drops samples under pressure.
 * Losing an analysis frame costs one dropped LED update; stalling the A2DP
 * callback costs audible audio.
 */
void visualiser_feed(const uint8_t *pcm, uint32_t len);
