/*
 * The two lengths the analysis is cut by, and the one place they are decided.
 *
 * They used to be one number. FFT_N was both the length of the window handed to
 * the FFT and the distance the analysis advanced between windows, so windows
 * never overlapped -- and nothing in the code said so, because a hop that is
 * always equal to the window is invisible. Every onset-detection algorithm worth
 * trying assumes overlap, so the conflation blocked the experiment outright.
 *
 * Plain C and no dependencies, because three build systems include it and only
 * one of them is ESP-IDF: tools/pattern_lab/Makefile and
 * components/dancefloor_leds/test/Makefile compile the shared sources with bare
 * -I../include. A Kconfig-only hop would give the host harness one value and the
 * firmware another, silently, and the harness is where the tuning is measured.
 */
#pragma once

/*
 * -DESP_PLATFORM is set by the IDF build and by nothing else, so this is the one
 * portable way to ask "am I being compiled for a board".
 *
 * The include is not optional and not a courtesy. There is no -include
 * sdkconfig.h in this project's compile lines; CONFIG macros reach visualiser.cpp
 * only because freertos/FreeRTOS.h happens to pull sdkconfig.h in before they are
 * used. A header whose value depends on a CONFIG macro cannot rely on an
 * accident of include order in its includers -- it would compile everywhere and
 * be wrong wherever the order differed.
 */
#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif

/*
 * The window: how many samples the FFT transforms. 43 Hz bins at 44.1 kHz.
 *
 * This is the only role FFT_N keeps. It sets the bin count, the band edges, the
 * Hann table and the magnitude normalisation, and every tuning figure in
 * analysis.cpp was measured against those -- so changing it is a retune of the
 * detector, not a change to the frame rate.
 */
#define DF_FFT_N 1024

/*
 * The hop: how many samples the analysis advances between windows.
 *
 * DF_HOP_N == DF_FFT_N reproduces the behaviour that shipped, exactly. Anything
 * smaller overlaps the windows and raises the frame rate by the same factor.
 *
 * A command-line -DDF_HOP_N wins over the Kconfig choice, which is what lets
 * `make check-hops` sweep the host tests over every supported value without
 * touching a config anywhere.
 */
#ifndef DF_HOP_N
#  if defined(CONFIG_DANCEFLOOR_LED_HOP_256)
#    define DF_HOP_N 256
#  elif defined(CONFIG_DANCEFLOOR_LED_HOP_512)
#    define DF_HOP_N 512
#  else
#    define DF_HOP_N DF_FFT_N
#  endif
#endif

/* What is carried over from one window to the next. Zero when they do not
 * overlap, which is why the slide that keeps it can compile to nothing. */
#define DF_TAIL_N (DF_FFT_N - DF_HOP_N)

/*
 * Checked with #if rather than static_assert deliberately: these are macros, the
 * preprocessor evaluates all four, and it behaves identically in C and C++ with
 * no question about which standard the includer was compiled to. A bad hop then
 * fails at the top of the file with the reason, rather than somewhere downstream
 * with a length that does not divide.
 *
 * The divisibility is a choice, not a law. It makes {k * FFT_N} a subset of
 * {k * HOP_N}, so the finer grid refines the old one instead of cutting across
 * it -- which is what lets two units on different hops still agree about where a
 * window may begin, and what makes a hop change safe for the block alignment.
 */
#if DF_HOP_N <= 0
#error "DF_HOP_N must be positive"
#endif
#if DF_HOP_N > DF_FFT_N
#error "DF_HOP_N cannot exceed DF_FFT_N -- a hop past the window would skip audio"
#endif
#if (DF_FFT_N % DF_HOP_N) != 0
#error "DF_FFT_N must be a whole number of hops -- see the note above"
#endif
#if (DF_HOP_N & (DF_HOP_N - 1)) != 0
#error "DF_HOP_N must be a power of two"
#endif
