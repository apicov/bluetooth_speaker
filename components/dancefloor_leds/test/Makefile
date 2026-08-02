# PATH is pinned: sourcing ESP-IDF's export.sh puts the xtensa cross-assembler
# ahead of the host one, breaking native builds with "as: unrecognized option '--64'".
CC     := /usr/bin/gcc
CFLAGS := -std=c11 -Wall -Wextra -O2 -I../include

BINS := test_align test_beat

test_align: test_align.c
	env PATH=/usr/bin:/bin $(CC) $(CFLAGS) $< -o $@

test_beat: ../../../hub/test/test_beat_detect.c ../beat_detect.c
	env PATH=/usr/bin:/bin $(CC) $(CFLAGS) $^ -o $@ -lm

.PHONY: check clean
check: $(BINS)
	@./test_align
	@echo
	@./test_beat

clean:
	rm -f $(BINS)
