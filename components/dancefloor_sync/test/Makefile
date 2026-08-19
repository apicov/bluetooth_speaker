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

# audio_shift.h and sync_proto.h are prerequisites because RATE_TRIM_MAX_HZ,
# the CATCHUP_* thresholds and PHASE_DEADBAND_US all live in them -- a stale
# binary would silently test the servo against the old boundaries.
test_servo: test_servo.c ../df_servo.c ../include/df_servo.h \
            ../include/audio_shift.h ../include/sync_proto.h
	env PATH=/usr/bin:/bin $(CC) $(CFLAGS) test_servo.c ../df_servo.c -lm -o $@

.PHONY: check clean
check: test_sync_proto test_audio_shift test_servo
	./test_sync_proto
	./test_audio_shift
	./test_servo

clean:
	rm -f test_sync_proto test_audio_shift test_servo
