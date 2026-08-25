LEDS := ../../components/dancefloor_leds

CC       := /usr/bin/gcc
CXX      := /usr/bin/g++
CFLAGS   := -std=c11     -Wall -Wextra -O2 -I$(LEDS)/include
CXXFLAGS := -std=gnu++20 -Wall -Wextra -O2 -I$(LEDS)/include

HIST ?=
ifneq ($(HIST),)
CFLAGS   += -DBEAT_HIST=$(HIST)
CXXFLAGS += -DBEAT_HIST=$(HIST)
endif

HOP ?=
ifneq ($(HOP),)
CFLAGS   += -DDF_HOP_N=$(HOP)
CXXFLAGS += -DDF_HOP_N=$(HOP)
endif

converge: converge.cpp beat_detect.o $(LEDS)/include/analysis.hpp $(LEDS)/include/beat_detect.h
	env PATH=/usr/bin:/bin $(CXX) $(CXXFLAGS) converge.cpp beat_detect.o -o $@ -lm

beat_detect.o: $(LEDS)/beat_detect.c $(LEDS)/include/beat_detect.h
	env PATH=/usr/bin:/bin $(CC) $(CFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm -f converge beat_detect.o
