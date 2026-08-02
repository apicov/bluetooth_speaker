# PATH is pinned: sourcing ESP-IDF's export.sh puts the xtensa cross-assembler
# ahead of the host one, breaking native builds with "as: unrecognized option '--64'".
CC       := /usr/bin/gcc
CXX      := /usr/bin/g++
CFLAGS   := -std=c11 -Wall -Wextra -O2 -I../include
CXXFLAGS := -std=gnu++20 -Wall -Wextra -O2 -I../include

BINS := test_align test_pattern test_beat_detect test_fft test_pattern_sync

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

test_pattern_sync: test_pattern_sync.cpp ../analysis.cpp ../patterns.cpp \
                   ../include/analysis.hpp ../include/patterns.hpp fft_host.o beat_detect.o
	env PATH=/usr/bin:/bin $(CXX) $(CXXFLAGS) test_pattern_sync.cpp \
		../analysis.cpp ../patterns.cpp fft_host.o beat_detect.o -o $@ -lm

.PHONY: check clean
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

clean:
	rm -f $(BINS) fft_host.o beat_detect.o
