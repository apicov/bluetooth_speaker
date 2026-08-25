
#pragma once

#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif

#define DF_FFT_N 1024

#ifndef DF_HOP_N
#  if defined(CONFIG_DANCEFLOOR_LED_HOP_1024)
#    define DF_HOP_N DF_FFT_N
#  elif defined(CONFIG_DANCEFLOOR_LED_HOP_256)
#    define DF_HOP_N 256
#  else
#    define DF_HOP_N 512
#  endif
#endif

#define DF_TAIL_N (DF_FFT_N - DF_HOP_N)

#define DF_SPEC_BINS 64

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
