# PATH is pinned: sourcing ESP-IDF's export.sh puts the xtensa cross-assembler
# ahead of the host one, breaking native builds with "as: unrecognized option '--64'".
CC       := /usr/bin/gcc
CXX      := /usr/bin/g++
CFLAGS   := -std=c11 -Wall -Wextra -O2 -I../include
CXXFLAGS := -std=gnu++20 -Wall -Wextra -O2 -I../include

# The analysis hop, for `make HOP=512 check` and the sweep below. Empty means
# whatever analysis_config.h defaults to, which is what the firmware gets when
# no Kconfig choice is made. It must reach BOTH sets of flags: beat_detect.c and
# fft_host.c are compiled as C and linked into the C++ tests, and a hop that
# differed between the two halves would be a link that silently disagrees.
HOP ?=
ifneq ($(HOP),)
CFLAGS   += -DDF_HOP_N=$(HOP)
CXXFLAGS += -DDF_HOP_N=$(HOP)
endif

# The detector's flux history, for `make HIST=86 check`. Same both-flag rule as
# the hop, and for a sharper reason: BEAT_HIST is the length of an array inside
# beat_det_t, so a value that differed between the C and C++ halves would link
# two disagreeing definitions of the same struct.
HIST ?=
ifneq ($(HIST),)
CFLAGS   += -DBEAT_HIST=$(HIST)
CXXFLAGS += -DBEAT_HIST=$(HIST)
endif

BINS := test_align test_pattern test_beat_detect test_fft test_pattern_sync \
        test_result_latch

test_align: test_align.c
	env PATH=/usr/bin:/bin $(CC) $(CFLAGS) $< -o $@

test_pattern: test_pattern.c
	env PATH=/usr/bin:/bin $(CC) $(CFLAGS) $< -o $@ -lm

test_beat_detect: test_beat_detect.c ../beat_detect.c ../include/beat_detect.h
	env PATH=/usr/bin:/bin $(CC) $(CFLAGS) test_beat_detect.c ../beat_detect.c -o $@ -lm

test_fft: test_fft.c ../fft_host.c
	env PATH=/usr/bin:/bin $(CC) $(CFLAGS) test_fft.c ../fft_host.c -o $@ -lm

# The C halves are compiled AS C and linked in as objects. fft_host.c has no
# extern "C" declaration guarding it, so building it as C++ mangles
# df_fft_radix2 and analysis.cpp cannot find it.
fft_host.o: ../fft_host.c
	env PATH=/usr/bin:/bin $(CC) $(CFLAGS) -c $< -o $@

beat_detect.o: ../beat_detect.c ../include/beat_detect.h
	env PATH=/usr/bin:/bin $(CC) $(CFLAGS) -c ../beat_detect.c -o $@

# Header-only and dependency-free: ResultLatch touches no clock, no task and no
# strip, which is what makes the sync property testable here at all.
test_result_latch: test_result_latch.cpp ../include/result_latch.hpp ../include/analyser.hpp
	env PATH=/usr/bin:/bin $(CXX) $(CXXFLAGS) test_result_latch.cpp -o $@

test_pattern_sync: test_pattern_sync.cpp ../analysis.cpp ../patterns.cpp \
                   ../include/analysis.hpp ../include/patterns.hpp fft_host.o beat_detect.o
	env PATH=/usr/bin:/bin $(CXX) $(CXXFLAGS) test_pattern_sync.cpp \
		../analysis.cpp ../patterns.cpp fft_host.o beat_detect.o -o $@ -lm

.PHONY: check check-hops clean

# Every supported hop, because the alignment arithmetic and the window slide are
# what a hop change actually touches and they are only exercised by running them.
# The window-contiguity check in test_align is the one that has teeth here: at
# HOP == FFT_N the tail slide compiles to nothing, so 1024 alone proves little
# about it.
check-hops:
	@for h in 1024 512 256; do \
	    echo "=== hop $$h ==="; \
	    $(MAKE) --no-print-directory clean >/dev/null; \
	    $(MAKE) --no-print-directory HOP=$$h check || exit 1; \
	    echo; \
	done
	@$(MAKE) --no-print-directory clean >/dev/null
	@echo "all hops green"

check: $(BINS)
	@./test_align
	@echo
	@./test_pattern
	@echo
	@./test_beat_detect
	@echo
	@./test_fft
	@echo
	@./test_pattern_sync
	@echo
	@./test_result_latch

clean:
	rm -f $(BINS) fft_host.o beat_detect.o
