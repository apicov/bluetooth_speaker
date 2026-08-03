# PATH is pinned because sourcing ESP-IDF's export.sh puts the xtensa
# cross-assembler ahead of the host one, which breaks a native build with
# "as: unrecognized option '--64'".
CC      := /usr/bin/gcc
CFLAGS  := -std=c11 -Wall -Wextra -I../include

test_sync_proto: test_sync_proto.c ../sync_proto.c ../include/sync_proto.h
	env PATH=/usr/bin:/bin $(CC) $(CFLAGS) test_sync_proto.c ../sync_proto.c -o $@

.PHONY: check clean
check: test_sync_proto
	./test_sync_proto

clean:
	rm -f test_sync_proto
