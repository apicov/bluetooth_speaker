/*
 * Host-test stub.
 *
 * audio_out.h includes sdkconfig.h for the channel-mode symbols, which only
 * exist inside an ESP-IDF build. Empty here, so the host build takes the same
 * default the boards do -- stereo, every channel-mode branch compiled out --
 * and the volume taper below it is what gets tested.
 */
#pragma once
