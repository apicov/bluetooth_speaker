# pattern_lab -- the firmware's LED pipeline, built for the host.
#
# analysis.cpp, patterns.cpp and beat_detect.c are compiled straight out of
# components/dancefloor_leds rather than copied, so this cannot drift from what
# the strips actually do. fft_host.c stands in for esp-dsp.
#
# PATH is pinned for the same reason as the test makefile: sourcing ESP-IDF's
# export.sh puts the xtensa cross-assembler ahead of the host one, which breaks
# native builds with "as: unrecognized option '--64'".

LEDS := ../../components/dancefloor_leds

CC       := /usr/bin/gcc
CXX      := /usr/bin/g++
CFLAGS   := -std=c11     -Wall -Wextra -O2 -I$(LEDS)/include
CXXFLAGS := -std=gnu++20 -Wall -Wextra -O2 -I$(LEDS)/include
LDLIBS   := -lz -lm

SRCS := main.cpp wav.cpp out_png.cpp out_tty.cpp $(LEDS)/analysis.cpp $(LEDS)/patterns.cpp
HDRS := wav.hpp out_png.hpp out_tty.hpp $(LEDS)/include/analysis.hpp $(LEDS)/include/patterns.hpp

pattern_lab: $(SRCS) $(HDRS) fft_host.o beat_detect.o
	env PATH=/usr/bin:/bin $(CXX) $(CXXFLAGS) $(SRCS) fft_host.o beat_detect.o -o $@ $(LDLIBS)

# Compiled AS C: fft_host.c has no extern "C" guarding it, so building it as C++
# mangles df_fft_radix2 and analysis.cpp cannot find it.
fft_host.o: $(LEDS)/fft_host.c
	env PATH=/usr/bin:/bin $(CC) $(CFLAGS) -c $< -o $@

beat_detect.o: $(LEDS)/beat_detect.c $(LEDS)/include/beat_detect.h
	env PATH=/usr/bin:/bin $(CC) $(CFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm -f pattern_lab fft_host.o beat_detect.o
