# PATH is pinned: sourcing ESP-IDF's export.sh puts the xtensa cross-assembler
# ahead of the host one, breaking native builds with "as: unrecognized option '--64'".
CC     := /usr/bin/gcc
CFLAGS := -std=c11 -Wall -Wextra -O2 -I../include

BINS := test_align test_pattern test_beat_detect

test_align: test_align.c
	env PATH=/usr/bin:/bin $(CC) $(CFLAGS) $< -o $@

test_pattern: test_pattern.c
	env PATH=/usr/bin:/bin $(CC) $(CFLAGS) $< -o $@ -lm

test_beat_detect: test_beat_detect.c ../beat_detect.c ../include/beat_detect.h
	env PATH=/usr/bin:/bin $(CC) $(CFLAGS) test_beat_detect.c ../beat_detect.c -o $@ -lm

.PHONY: check clean
check: $(BINS)
	@./test_align
	@echo
	@./test_pattern
	@echo
	@./test_beat_detect

clean:
	rm -f $(BINS)
