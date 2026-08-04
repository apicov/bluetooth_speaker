# converge -- the BEAT_HIST convergence probe. See converge.cpp.
#
# beat_detect.c is compiled straight out of the component, like pattern_lab
# does, so this measures the detector the boards run rather than a model of it.
#
# PATH is pinned for the same reason as the other two host makefiles: sourcing
# ESP-IDF's export.sh puts the xtensa cross-assembler ahead of the host one,
# which breaks native builds with "as: unrecognized option '--64'".

LEDS := ../../components/dancefloor_leds

CC       := /usr/bin/gcc
CXX      := /usr/bin/g++
CFLAGS   := -std=c11     -Wall -Wextra -O2 -I$(LEDS)/include
CXXFLAGS := -std=gnu++20 -Wall -Wextra -O2 -I$(LEDS)/include

# The history under test: `make clean && make HIST=86`. The whole point of the
# tool, so unlike the other makefiles this is the variable that moves. Both flag
# sets, or the C and C++ halves link disagreeing definitions of beat_det_t.
HIST ?=
ifneq ($(HIST),)
CFLAGS   += -DBEAT_HIST=$(HIST)
CXXFLAGS += -DBEAT_HIST=$(HIST)
endif

# The hop only reaches the trace's header, which converge reads rather than
# assumes -- but analysis.hpp static_asserts against it, so it must still agree.
HOP ?=
ifneq ($(HOP),)
CFLAGS   += -DDF_HOP_N=$(HOP)
CXXFLAGS += -DDF_HOP_N=$(HOP)
endif

converge: converge.cpp beat_detect.o $(LEDS)/include/analysis.hpp $(LEDS)/include/beat_detect.h
	env PATH=/usr/bin:/bin $(CXX) $(CXXFLAGS) converge.cpp beat_detect.o -o $@ -lm

# Compiled AS C: it is a C file with extern "C" guards in its header, and
# building it as C++ would change nothing here but would diverge from how the
# firmware and the other two harnesses build it.
beat_detect.o: $(LEDS)/beat_detect.c $(LEDS)/include/beat_detect.h
	env PATH=/usr/bin:/bin $(CC) $(CFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm -f converge beat_detect.o
