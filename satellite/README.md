# Dancefloor satellite

Joins the master's SoftAP, keeps its clock aligned with the hub's, receives
unicast SBC, decodes it, and plays each chunk at the instant it was stamped for.
Drives an LED strip from the audio it is about to play — or, if built for
`DANCEFLOOR_LED_SOURCE_REMOTE`, from analysis frames the hub sends, drawn at the
instant each names.

**No Bluetooth here.** The master owns the phone connection; this board only
listens on WiFi. Any ESP32 variant would work for this role, but the project
standardises on the classic ESP32 so every unit is interchangeable with the
master, which does need Bluetooth Classic.

**The two targets are no longer the same unit with different pins.** A classic
satellite receives, decodes, plays and draws the hub's frames. A XIAO S3
satellite does all of that *and* runs the pluggable analysers, including
TensorFlow Lite Micro models — on the spectrum those same frames carry, so it
still analyses no audio of its own. `DANCEFLOOR_ML` is what separates them and
it is set in `sdkconfig.defaults.esp32s3` alone; see "Where a difference between
the two goes" below for how that generalises.

> This firmware was read once for clarity, modularity and what a second target
> costs, and the findings were acted on over 2026-08-12: it is nine files rather
> than one, the internal-DAC path is gone, and the S3 is a target rather than a
> fork.

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
classic-ESP32 satellite and a Seeed XIAO ESP32-S3 one. Pins, PSRAM, flash size,
the WiFi buffer placement and the console route come from
`sdkconfig.defaults.esp32s3`, which IDF reads after `sdkconfig.defaults` when the
target is the S3; the pin *defaults* are also keyed on `IDF_TARGET_ESP32S3` in
`main/Kconfig.projbuild`, so a bare `set-target` build is already correct. There
is deliberately no `satellite_s3/` directory: the hub carried the equivalent
fork for a while, and it was retired rather than reconciled.

**Which XIAO.** The S3 defaults are written for the **XIAO ESP32-S3 Plus**, which
is what `hub_s3/` runs on, so a hub and a satellite are the same board wired the
same way — I2S on pads `D8`/`D9`/`D10`, the strip on `D0`, the LED marker on `D1`.
The pad numbering is identical on the plain XIAO and the Sense, so those work too;
the only line that is Plus-specific is `CONFIG_ESPTOOLPY_FLASHSIZE_16MB`, which
should go back to `8MB` on the other two. The file says so at that line.

The S3 console is on USB-C for bring-up, which costs radio performance —
`sdkconfig.defaults.esp32s3` carries a commented-out UART block to switch to once
the unit is playing, and the reasoning behind it.

### Two images, side by side

The two builds coexist permanently in one checkout — no `set-target`, no
switching, nothing to rewrite between them:

| | classic | S3 |
|---|---|---|
| tracked config | `sdkconfig.defaults` **+** `sdkconfig.defaults.esp32` | `sdkconfig.defaults` **+** `sdkconfig.defaults.esp32s3` |
| generated config | `sdkconfig` | `sdkconfig.s3` |
| build directory | `build/` | `build.s3/` |

[`tools/sat.py`](../tools/sat.py) is the short way, and prints the full command
it runs so nothing is hidden:

```sh
tools/sat.py build   s3
tools/sat.py flash   classic
tools/sat.py monitor s3
```

The long way, which is what it assembles:

```sh
idf.py build                                                    # classic
idf.py -B build.s3 -DIDF_TARGET=esp32s3 -DSDKCONFIG=sdkconfig.s3 build
```

**`-DSDKCONFIG` is needed on every S3 invocation, not just the first.** Omit it
with `-B build.s3` and that build directory is silently reconfigured from the
classic `sdkconfig` — wrong pins, and since the ML work, the wrong feature set
too, reported by nothing louder than behaviour. That is the whole reason
`tools/sat.py` exists.

Use a full `flash`, never `app-flash`: the flash is stamped DIO and upgraded to
QIO by the bootloader, so a stale bootloader leaves the app running DIO.
`/dev/ttyACM0` rather than `ttyUSB*` because the XIAO has no USB-UART bridge —
`sat.py` guesses accordingly and takes `--port` to override.

The boot line says which image is running: `BUILD <date> <time> <target>
elf:<hash>`.

### Where a difference between the two goes

Three tracked files, one per audience. IDF appends `sdkconfig.defaults.<target>`
after `sdkconfig.defaults` for **any** target, so this works in both directions:

| file | applies to |
|---|---|
| `sdkconfig.defaults` | both |
| `sdkconfig.defaults.esp32` | classic only — `LED_SOURCE_REMOTE`, and the memory argument for it |
| `sdkconfig.defaults.esp32s3` | S3 only |

Code that belongs to one of them is a whole file wrapped in `#if
CONFIG_DANCEFLOOR_<THING>`, with the symbol set from that target's file —
`analyser_tflm.cpp` is the worked example. **The code never tests the target**;
it tests the capability, and the config file decides. That keeps the rule
`visualiser.cpp` states — "adding a mode, not re-deriving which of ten
conditionals meant which thing" — true both ways, and makes a third board a
fourth config file rather than a rewrite of every `#if`.

When a setting in `sdkconfig.defaults` stops being common to both, move it down
into the two target files rather than leaving it in the base with an override on
top. A base value that is always overridden reads as a shared default and is not
one.

`DANCEFLOOR_LED_SOURCE` is the worked example, and the reason
`sdkconfig.defaults.esp32` exists at all. It sat in the base as `REMOTE` while
both boards agreed. They stopped agreeing when `spec[]` came off the wire: the S3
runs the ML analysers, analysers read a spectrum, and a unit taking frames no
longer receives one — so the S3 went `LOCAL` and computes its own. Rather than
leave `REMOTE` in the base and override it on one target, each target now states
its own, with the reason beside it.

The hop and the pattern stayed in the base, deliberately. Those must not differ
across the floor, and a locally-analysing S3 and a frame-taking classic exchange
nothing that would reveal a mismatch — the only instrument is reading `hop` off
two consoles. Keeping them where both targets read them makes a mismatch require
an edit rather than an omission.

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

Same as the hub's own DAC, on the same pins — and on the S3 that is literally the
same pins as `hub_s3/`, so the two boards are wired identically.

| PCM5102A | classic ESP32 | XIAO ESP32-S3 |
|---|---|---|
| BCK | GPIO 26 | GPIO 7, pad `D8` |
| LRCK | GPIO 27 | GPIO 8, pad `D9` |
| DIN | GPIO 25 | GPIO 9, pad `D10` |
| GND | GND, SCK | GND, SCK |
| VIN | 3V3 | 3V3 |

XSMT must be high or the DAC stays muted.

### LEDs

Data pin -> 74AHCT125 level shifter -> WS2812 DIN. 330 Ω series on data, 1000 µF
across the strip supply, separate 5 V supply, common ground. The data pin is
**GPIO 18** on a classic ESP32 and **GPIO 1** (pad `D0`) on the XIAO, which does
not bring 18 out.

### Bench instruments, optional, off by default

| classic | XIAO S3 | |
|---|---|---|
| GPIO 4 | GPIO 4, pad `D3` | Audio marker. Wire to the hub's monitor pin, plus a common ground, and the hub reports how far this unit's audio is from its own. |
| GPIO 2 | GPIO 2, pad `D1` | LED marker. One flash per second, in step on every unit. Onboard LED on many classic boards; on the XIAO wire an external one to `D1` — the documented onboard LED at GPIO 21 does not light. |

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
from the probe estimator otherwise.

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
