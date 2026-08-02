# PATH is pinned: sourcing ESP-IDF's export.sh puts the xtensa cross-assembler
# ahead of the host one, breaking native builds with "as: unrecognized option '--64'".
CC     := /usr/bin/gcc
CFLAGS := -std=c11 -Wall -Wextra -I../main

test_beat_detect: test_beat_detect.c ../main/beat_detect.c ../main/beat_detect.h
	env PATH=/usr/bin:/bin $(CC) $(CFLAGS) test_beat_detect.c ../main/beat_detect.c -o $@ -lm

.PHONY: check clean
check: test_beat_detect
	./test_beat_detect

clean:
	rm -f test_beat_detect
