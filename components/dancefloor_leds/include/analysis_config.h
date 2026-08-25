/**
 * @file analysis_config.h
 * @brief The two lengths the analysis is cut by, and the one place they are
 *        decided.
 *
 * They were once a single number: the FFT's window length was also the
 * distance the analysis advanced between windows, so windows never overlapped
 * -- and nothing in the code said so, because a hop always equal to the window
 * is invisible. Every onset-detection algorithm worth trying assumes overlap,
 * so the conflation blocked the experiment outright. They are two numbers now,
 * and each has exactly one role; see df::FFT_N and df::HOP_N.
 *
 * Plain C and no dependencies, because THREE build systems include this and
 * only one of them is ESP-IDF: tools/pattern_lab and this component's own test
 * directory compile the shared sources with a bare include path. A value that
 * came from Kconfig alone would give the host harness one number and the
 * firmware another, silently -- and the harness is where the tuning is
 * measured.
 */
#pragma once

/*
 * -DESP_PLATFORM is set by the IDF build and by nothing else, so this is the
 * one portable way to ask "am I being compiled for a board".
 *
 * The include is not optional and not a courtesy. There is no -include
 * sdkconfig.h in this project's compile lines; CONFIG macros reach
 * visualiser.cpp only because a FreeRTOS header happens to pull sdkconfig.h in
 * before they are used. A header whose value depends on a CONFIG macro cannot
 * rest on an accident of include order in its includers -- it would compile
 * everywhere and be wrong wherever that order differed.
 */
#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif

/**
 * @brief The WINDOW: how many samples the FFT transforms.
 *
 * This is the only role it keeps. It sets the bin count, the band edges, the
 * Hann table and the magnitude normalisation, and every tuning figure in
 * analysis.cpp was measured against those -- so changing it is a RETUNE of the
 * detector, not a change to the frame rate.
 */
#define DF_FFT_N 1024

/**
 * @brief The HOP: how many samples the analysis advances between windows.
 *
 * Half the window is 50% overlap, which is what ordinary onset-detection
 * algorithms assume. Setting it equal to DF_FFT_N reproduces the behaviour
 * that shipped before overlap existed.
 *
 * The fallback here must equal the Kconfig default, and that is not a
 * tidiness point: the fallback is what the host harness gets, since it
 * compiles with no sdkconfig.h at all. If the two disagreed, every figure
 * measured on the host would describe a pipeline the boards do not run. That
 * is the whole reason the hop is not a Kconfig symbol alone.
 *
 * A command-line -DDF_HOP_N beats both, which is what lets the host tests
 * sweep every supported value without touching a config.
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

/** @brief What one window carries over into the next. Zero when they do not
 *         overlap, which is why the slide that keeps it can compile to
 *         nothing. */
#define DF_TAIL_N (DF_FFT_N - DF_HOP_N)

/**
 * @brief Bins in the portable spectrum -- df::Frame::spec, and what every
 *        analyser is handed.
 *
 * Log-spaced from the low edge to the high one; analysis.hpp has the reasoning
 * for the spacing and owns the two frequencies.
 *
 * HERE rather than in analysis.hpp because analyser.hpp needs it for
 * df::Analyser::process(), and analysis.hpp includes analyser.hpp rather than
 * the other way round.
 *
 * It is no longer a WIRE width. It was once the width of an array in
 * vis_frame_t as well -- three headers agreeing on one number -- but the
 * spectrum stopped travelling, so this is now a purely local figure: two
 * headers agreeing, and nothing on the air.
 */
#define DF_SPEC_BINS 64

/**
 * @brief How many pluggable analysers may run at once, and therefore how many
 *        df::Result every frame carries.
 *
 * One slot per registered analyser, indexed the same way, so a pattern reading
 * f.ml[i] always gets analyser i whatever else is or is not enabled.
 *
 * That costs RAM in the frame queue and it is not free: the queue holds a
 * window's worth of frames, so each slot is kilobytes, and a shorter hop
 * multiplies it. Two is the default because the case this was built for is one
 * fast analyser beside one slow one -- a per-hop detector and a model with a
 * second of context, which is precisely the pair that cannot share a lane. Set
 * it to 1 on a unit running a single analyser to get the rest back.
 *
 * Here rather than in Kconfig alone for the same reason as DF_HOP_N: the host
 * harness compiles with no sdkconfig.h, and a struct whose size differed
 * between the laptop and the board would make every figure measured on the
 * laptop describe a different pipeline.
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
 * Checked with #if rather than static_assert deliberately: these are macros,
 * the preprocessor evaluates all four, and it behaves identically in C and C++
 * with no question about which standard the includer was compiled to. A bad
 * hop then fails at the top of this file with the reason, rather than
 * somewhere downstream with a length that does not divide.
 *
 * The divisibility is a CHOICE, not a law. It makes the coarse grid a subset
 * of the fine one, so a shorter hop refines the old grid instead of cutting
 * across it -- which is what lets two units on different hops still agree
 * about where a window may begin, and what makes a hop change safe for the
 * block alignment.
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
