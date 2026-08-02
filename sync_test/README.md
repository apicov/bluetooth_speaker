# M4 — two-unit clock synchronisation

The make-or-break milestone. No audio, no DAC, no amp, no enclosure — two bare
ESP32 devkits and a two-channel scope. If two boards cannot agree on a clock to
within a millisecond, the streaming architecture does not work and it is worth
knowing that before spending money on drivers and batteries.

This stage uses only WiFi, so it would run on any variant — but the project
standardises on the **classic ESP32** for every unit. One BOM, interchangeable
spares, and any board can be promoted to master since they all have Bluetooth
Classic.

## Host tests

The offset estimator is deliberately free of ESP-IDF dependencies so it can be
tested natively:

```sh
cd test
make        # or: gcc -std=c11 -Wall -Wextra -I../main test_sync_proto.c ../main/sync_proto.c -o test_sync_proto
./test_sync_proto
```

Note: if your shell profile sources ESP-IDF's `export.sh`, the xtensa
cross-assembler shadows the host one and the build fails with
`as: unrecognized option '--64'`. The Makefile pins `PATH` to avoid this.

## Flashing two boards

```sh
. ~/.espressif/v6.0.1/esp-idf/export.sh
idf.py set-target esp32

# Board A — master
idf.py menuconfig      # Dancefloor -> role master = y   (default)
idf.py -p /dev/ttyUSB0 flash monitor

# Board B — satellite
idf.py menuconfig      # Dancefloor -> role master = n
idf.py -p /dev/ttyUSB1 flash monitor
```

The satellite logs its offset estimate once a second:

```
I (12345) sync: offset -418723 us (rtt 2140 us)
```

The absolute value is meaningless — it is the difference between two arbitrary
boot times. What matters is that it settles and stops wandering.

## Measurement

### Without a scope (recommended)

Wire the **satellite's blink output** to the **master's monitor input**, plus a
common ground:

| Satellite | Master |
|---|---|
| blink GPIO (default 4) | monitor GPIO (default 21) |
| GND | GND |

The master times the incoming edge against its own announced deadline and logs
the answer directly — no instruments needed:

```
I (34521) sync: SYNC ERROR: +83 us   (satellite late)
```

**Pass: consistently within ±1000 µs.**

> One output into one input. Do **not** connect the two boards' *blink* pins
> together — both are outputs, and tying them makes one drive against the other.

### With a scope

Probe the blink GPIO on both boards, common ground, trigger on the master. Every
two seconds both pulse high for 10 ms. Pass is rising edges within 1 ms. Worth
doing if you have one: it measures the pins directly and shares no code with the
thing under test.

## Interpreting failure

- **Delta of tens of ms** — the satellite is falling back on a stale or missing
  offset. Check for `blink deadline missed` warnings in the log.
- **Delta stable but large, e.g. a consistent 900 µs** — path asymmetry, not
  jitter. The median cannot remove this; it is the estimator's error floor
  (`test_sync_proto.c` case 5 demonstrates it deliberately). Fix by selecting
  the minimum-RTT probe rather than the median, which is what PTP does.
- **Delta wanders slowly over minutes** — that's crystal drift, and it is
  expected here. M6 is what corrects it. At this stage only check that
  re-probing pulls it back.

## Design notes

- `WIFI_PS_NONE` is essential. Default power save parks the radio between
  beacons and adds tens of milliseconds to exactly the packets being timed.
- `CONFIG_FREERTOS_HZ=1000`: at the default 100 Hz, `vTaskDelay` granularity is
  10 ms and would dominate a sub-millisecond measurement.
- `blink_task` wakes 2 ms early and busy-waits the remainder. Sleeping straight
  to the deadline would fold scheduler jitter into the measurement.
- Time probes are unicast; blink announcements are multicast, which is the same
  path audio will take in M5 so it gets exercised early.
