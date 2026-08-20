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
 * 512 is 50% overlap, which is what ordinary onset-detection algorithms assume
 * and what this pipeline now runs. DF_HOP_N == DF_FFT_N reproduces the
 * behaviour that shipped before overlap existed.
 *
 * This fallback must equal the Kconfig default below it, and that is not a
 * tidiness point. It is what the host harness gets -- pattern_lab and the unit
 * tests compile with no sdkconfig.h at all -- so if the two disagreed, every
 * figure measured on the host would describe a pipeline the boards do not run.
 * That is the whole reason the hop is not a Kconfig symbol alone.
 *
 * A command-line -DDF_HOP_N beats both, which is what lets `make check-hops`
 * sweep the host tests over every supported value without touching a config.
 */
#ifndef DF_HOP_N
#  if defined(CONFIG_DANCEFLOOR_LED_HOP_1024)
#    define DF_HOP_N DF_FFT_N
#  elif defined(CONFIG_DANCEFLOOR_LED_HOP_256)
#    define DF_HOP_N 256
#  else
#    define DF_HOP_N 512
#  endif
#endif

/* What is carried over from one window to the next. Zero when they do not
 * overlap, which is why the slide that keeps it can compile to nothing. */
#define DF_TAIL_N (DF_FFT_N - DF_HOP_N)

/*
 * Bins in the portable spectrum -- Frame::spec, and what every analyser is
 * handed. 64 log-spaced bins from 40 Hz to 16 kHz; analysis.hpp has the
 * reasoning for the spacing and owns the two edge frequencies.
 *
 * HERE rather than in analysis.hpp because analyser.hpp needs it for
 * Analyser::process() and analysis.hpp includes analyser.hpp, not the other way
 * round.
 *
 * IT IS NO LONGER A WIRE WIDTH. It used to be the width of the array in
 * vis_frame_t as well, static_asserted against VIS_SPEC_BINS -- three headers
 * agreeing on one number. The spectrum stopped travelling (see vis_frame_t),
 * so a frame is 32 bytes of timeline labels and bands, and this number is a
 * purely local one: two headers agreeing, and nothing on the air.
 */
#define DF_SPEC_BINS 64

/*
 * How many pluggable analysers may run at once, and therefore how many Results
 * every frame carries.
 *
 * One slot per registered analyser, indexed the same way, so a pattern reading
 * f.ml[i] always gets analyser i whatever else is or is not enabled. That
 * costs RAM in the frame queue and it is not free: a df::Result is 40 bytes,
 * the queue holds 32 * (FFT_N / HOP_N) frames, so each slot is 2.5 kB at hop
 * 512 and 5 kB at hop 256. Two slots is the default because the case this was
 * built for is one fast analyser beside one slow one -- a per-hop detector and
 * a model with a second of context, which is precisely the pair that could not
 * share a lane.
 *
 * Set it to 1 on a unit that runs a single analyser and wants the 2.5 kB back.
 *
 * Here rather than in Kconfig alone for the same reason as DF_HOP_N above: the
 * host harness and the unit tests compile with no sdkconfig.h at all, and a
 * struct whose size differed between the laptop and the board would make every
 * figure measured on the laptop describe a different pipeline.
 */
#ifndef DF_ML_SLOTS
#  if defined(CONFIG_DANCEFLOOR_ML_SLOTS)
#    define DF_ML_SLOTS CONFIG_DANCEFLOOR_ML_SLOTS
#  else
#    define DF_ML_SLOTS 2
#  endif
#endif

#if DF_ML_SLOTS < 1
#error "DF_ML_SLOTS must be at least 1 -- Frame::ml is a fixed array"
#endif

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
