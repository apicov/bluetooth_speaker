# Dancefloor satellite

Joins the master's SoftAP, keeps its clock aligned with the hub's, receives
unicast SBC, decodes it, and plays each chunk at the instant it was stamped for.
Drives an LED strip from the audio it is about to play — or, if built for
`DANCEFLOOR_LED_SOURCE_REMOTE`, from analysis frames the hub sends, drawn at the
instant each names. See [`../docs/architecture.md`](../docs/architecture.md) §12.

**No Bluetooth here.** The master owns the phone connection; this board only
listens on WiFi. Any ESP32 variant would work for this role, but the project
standardises on the classic ESP32 so every unit is interchangeable with the
master, which does need Bluetooth Classic.

> [`../docs/satellite-audit.md`](../docs/satellite-audit.md) reads this firmware
> for clarity, modularity and what a second target costs. Its findings were
> acted on over 2026-08-12: the firmware is nine files rather than one, the
> internal-DAC path is gone, and the S3 is a target rather than a fork. §2 of
> the audit records what was done and what was deliberately left.

## How the source is laid out

One 2437-line `main.c` until 2026-08-12. The split changed no behaviour — every
function body moved verbatim.

| file | what it owns |
|---|---|
| `main.c` | `app_main`, task creation, and the 5 s loop the two slow ticks share |
| `sat.h` | shared state, **and which task is allowed to write each field** |
| `sat_state.c` | the definitions for it, nothing else |
| `net.c` | joining the SoftAP, the reconnect handler, the UDP socket |
| `clock.c` | the probe task, TSF-or-estimator, the offset slew |
| `rx.c` | datagram demux, then one function per policy: anchor, gap, decode |
| `play.c` | the playback timeline: the scheduled start, phase, splice, marker |
| `out.c` | the I2S channel, the write path, retuning the output clock |
| `servo.c` | one 5 s window of rate control |
| `telemetry.c` | the heap windows, the allocation report, HEALTH and MEM |

`sat.h` is the one to read first. It is where the ownership rules live, and
where the analysis of which shared values can be torn by a concurrent read now
sits — including the three it says are **not** fixed, and why.

Run as many of these as you like. Registration is implicit: the hub sends audio
to whatever has sent it a time probe in the last **2 seconds**, so a unit that is
keeping its clock synchronised is by definition alive and listening. In practice
the timeout rarely does the work — the hub registers a satellite the moment DHCP
hands it an address, and forgets it the moment it disassociates cleanly. What the
timeout covers is the unit that vanishes without saying so, and it is 2 s rather
than the old 10 because during that window the hub keeps unicasting at a station
that is not there, taking a DMA buffer per send that the driver will not free
until it gives up.

## Build and flash

```sh
. ~/.espressif/v6.0.1/esp-idf/export.sh
idf.py set-target esp32          # or: idf.py set-target esp32s3
idf.py -p /dev/ttyUSB2 flash monitor
```

**One source, two targets.** `set-target` is the whole difference between a
classic-ESP32 satellite and a Seeed XIAO ESP32-S3 one. Pins, PSRAM, flash size
and the console route come from `sdkconfig.defaults.esp32s3`, which IDF reads
after `sdkconfig.defaults` when the target is the S3; the pin *defaults* are
also keyed on `IDF_TARGET_ESP32S3` in `main/Kconfig.projbuild`, so a bare
`set-target` build is already correct. There is deliberately no `satellite_s3/`
directory — see [`../docs/satellite-audit.md`](../docs/satellite-audit.md) §F1
for what the equivalent fork cost on the hub.

Changing target rewrites `sdkconfig` from the defaults, so re-check anything you
had set by hand in `menuconfig`.

Between edits, without a full build:

```sh
../tools/syntax_check.py satellite
```

It reuses the compile flags from the last real build and runs the compiler
`-fsyntax-only`, so it catches unbalanced `#if` nesting, missing symbols and bad
format strings in a second. It only sees the branches the last build's
`sdkconfig` selected; `--with` / `--without` force the others.

Pins, LED count, brightness, pattern and the bench instruments are under
`idf.py menuconfig` -> **Dancefloor satellite** and **Dancefloor LEDs**.

## Wiring

Same as the hub's own DAC, on the same pins.

| ESP32 | PCM5102A |
|---|---|
| GPIO 26 | BCK |
| GPIO 27 | LRCK |
| GPIO 25 | DIN |
| GND | GND, SCK |
| 3V3 | VIN |

XSMT must be high or the DAC stays muted.

### LEDs

GPIO 18 -> 74AHCT125 level shifter -> WS2812 DIN. 330 Ω series on data, 1000 µF
across the strip supply, separate 5 V supply, common ground.

### Bench instruments, optional, off by default

| | |
|---|---|
| GPIO 4 | Audio marker. Wire to the hub's GPIO 21, plus a common ground, and the hub reports how far this unit's audio is from its own. |
| GPIO 2 | LED marker. One flash per second, in step on every unit. Onboard LED on many boards, but not all. |

Nothing corrects on either. Every servo and every splice closes through
`play_at` and the phase queue, so enabling or disabling them changes no
behaviour.

## How playback timing works

Two mechanisms, and the second is the one that matters over an evening.

**The start is scheduled.** The first chunk's `play_at` is converted to local
time using the clock offset, and the satellite waits for that instant. After
that `i2s_channel_write` blocks once the DMA buffers are full, so the DAC paces
everything.

**Position is then held, not just rate.** Every packet says when its first
sample is due; recording that against the ring position it lands at gives a
direct reading of where playback is versus where the timeline says it should be
when it gets there. The servo trims the output clock to null that. Aligning only
the start is not enough: the crystals differ by about 10.6 ppm, which is 38 ms
of divergence in an hour.

**A lost packet becomes silence of exactly the right length.** Skipping it would
pull every later frame earlier and slide the whole stream against the master
permanently, which is far worse than a brief gap.

**At a track boundary the accumulated phase error is spliced away.** A track
change is the one moment inserting or dropping audio is inaudible, so that is
when it happens, and it is applied when playback *reaches* the flagged audio
rather than when the notification arrives.

The clock offset comes from 802.11 TSF when a fresh reading is available, and
from the probe estimator otherwise. See `../docs/clock-sync.md`.

## Reading the log

```
I sat: stream start: play_at 9415360 -> local 2256381 (in 183 ms) [TSF]
I sat: playback started: scheduled 9415360, actual 9415361 (+1 us) [master]
```

The tag says which clock source anchored playback. `actual` within a few
microseconds of `scheduled` means the unit kept its appointment.

```
I sat: buffer 200 ms | phase -1470 us (smoothed -1540 us)
```

**Smoothed phase is the number to watch**, not the buffer. Buffer depth moves
with delivery jitter and says nothing about position; phase is the direct
reading of how far this unit is from the published timeline. It should sit
within a few milliseconds and be spliced back toward zero at each track change.

```
W sat: HEALTH: up 3721 s | heap 83380 (min 73924, window 81002, largest 61240)
       | stack play 3048 drift 1796 | underruns 0 anchors 1 splices 2
       | retunes 7 (0 refused) | gaps 0 wifi-drops 0 | alloc-fail 0
       | clock TSF (tsf 1/probe 0) | leds local hop 512 (rx 0, bad 0)
```

Every 60 s, and the line a long run is judged on. **An uptime that resets to 0
means it crashed and rebooted.**

`min` is the all-time heap watermark and cannot be dated — one bad moment an hour
ago pins it for the whole run — which is what `window` is for: the lowest seen
this minute, cleared by each line, so a dip lands in a named minute. `largest` is
the largest free block and fails before the total does, since a fragmented heap
refuses a contiguous request while the free total still looks fine.
`alloc-fail` counts allocations that actually failed, which used to be silent and
surfaced as an underrun.

`clock TSF (tsf N/probe M)` says which source is live and how many anchors used
each; a rising `probe` means TSF is dropping out.

`leds <source> hop <N> (rx, bad)` is the other half of the same question. Both
values are also printed at startup. They are reported because a unit doing its
own analysis **cannot detect** that its neighbour is on a different hop or a
different source — nothing crosses between locally analysing units, which is
exactly the property that makes them stay in step. Two consoles settle it. `rx`
counts frames received from the hub and `bad` counts frames refused for being the
wrong size, which means the hub and this unit are not the same build.

## Verification

1. Pair a phone to the master (**Dancefloor**) and play something.
2. Satellite logs `stream start` then `playback started` within a second.
3. Both speakers produce audio, and standing between them there is no echo.
4. Smoothed phase stays within a few milliseconds rather than walking away.
5. The hub prints `TRACK DIVERGENCE (wifi)` once per track. That is the number
   to compare across builds; it is taken at a track boundary, which is the one
   instant that recurs identically in every track.
6. Over a long run, `HEALTH` counters stay near zero and the heap figures stay
   flat.

Steady separation over several minutes used to be expected and is not any more.
If smoothed phase walks away without recovering, something is wrong.
