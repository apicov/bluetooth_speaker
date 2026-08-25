LEDS := ../../components/dancefloor_leds

CC       := /usr/bin/gcc
CXX      := /usr/bin/g++
CFLAGS   := -std=c11     -Wall -Wextra -O2 -I$(LEDS)/include
CXXFLAGS := -std=gnu++20 -Wall -Wextra -O2 -I$(LEDS)/include

HOP ?=
ifneq ($(HOP),)
CFLAGS   += -DDF_HOP_N=$(HOP)
CXXFLAGS += -DDF_HOP_N=$(HOP)
endif

HIST ?=
ifneq ($(HIST),)
CFLAGS   += -DBEAT_HIST=$(HIST)
CXXFLAGS += -DBEAT_HIST=$(HIST)
endif
LDLIBS   := -lz -lm

SRCS := main.cpp wav.cpp out_png.cpp out_tty.cpp $(LEDS)/analysis.cpp $(LEDS)/patterns.cpp \
        $(LEDS)/analysers.cpp
HDRS := wav.hpp out_png.hpp out_tty.hpp $(LEDS)/include/analysis.hpp \
        $(LEDS)/include/patterns.hpp $(LEDS)/include/analyser.hpp \
        $(LEDS)/include/result_latch.hpp \

pattern_lab: $(SRCS) $(HDRS) fft_host.o beat_detect.o
	env PATH=/usr/bin:/bin $(CXX) $(CXXFLAGS) $(SRCS) fft_host.o beat_detect.o -o $@ $(LDLIBS)

fft_host.o: $(LEDS)/fft_host.c
	env PATH=/usr/bin:/bin $(CC) $(CFLAGS) -c $< -o $@

beat_detect.o: $(LEDS)/beat_detect.c $(LEDS)/include/beat_detect.h
	env PATH=/usr/bin:/bin $(CC) $(CFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm -f pattern_lab fft_host.o beat_detect.o
