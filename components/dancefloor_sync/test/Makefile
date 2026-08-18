# PATH is pinned because sourcing ESP-IDF's export.sh puts the xtensa
# cross-assembler ahead of the host one, which breaks a native build with
# "as: unrecognized option '--64'".
CC      := /usr/bin/gcc
CFLAGS  := -std=c11 -Wall -Wextra -I../include -I.

# audio_out.h and sbc_link.h are prerequisites because the volume taper and
# AUDIO_VOL_MAX live in them, and a stale binary silently tests the old
# curve -- which is exactly what happened when the taper was replaced.
test_sync_proto: test_sync_proto.c ../sync_proto.c ../include/sync_proto.h \
                 ../include/audio_out.h ../include/sbc_link.h
	env PATH=/usr/bin:/bin $(CC) $(CFLAGS) test_sync_proto.c ../sync_proto.c -lm -o $@

test_audio_shift: test_audio_shift.c ../audio_shift.c ../include/audio_shift.h
	env PATH=/usr/bin:/bin $(CC) $(CFLAGS) test_audio_shift.c ../audio_shift.c -lm -o $@

.PHONY: check clean
check: test_sync_proto test_audio_shift
	./test_sync_proto
	./test_audio_shift

clean:
	rm -f test_sync_proto test_audio_shift
