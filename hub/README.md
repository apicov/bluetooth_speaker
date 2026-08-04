# Dancefloor hub — chip B of the two-chip master

Takes SBC in over UART from the `bt_bridge` chip, owns the timeline the whole
system plays to, unicasts that SBC to the satellites, and decodes it for its own
DAC and LED strip. It also publishes each analysis frame it computes, for
satellites built to draw what this unit decided rather than analyse for
themselves — see [`../docs/architecture.md`](../docs/architecture.md) §12.

**No Bluetooth on this chip.** See [`../docs/two-chip-master.md`](../docs/two-chip-master.md)
for why the master is split in two, and [`../docs/sbc-link.md`](../docs/sbc-link.md)
for the wire between them.

## Build and flash

```sh
. ~/.espressif/v6.0.1/esp-idf/export.sh
idf.py set-target esp32
idf.py -p /dev/ttyUSB1 flash monitor
```

Pins, LED count, brightness, pattern and the bench instruments are under
`idf.py menuconfig` -> **Dancefloor hub** and **Dancefloor LEDs**.

## Wiring

### From the bridge chip — one wire, plus ground

| bt_bridge | hub | |
|---|---|---|
| GPIO 25 (TX) | GPIO 23 (RX) | SBC at 500 kbaud, one way |
| GND | GND | **required** |

The hub's end is `DANCEFLOOR_SBC_UART_RX_PIN`, and 23 is only its default —
it reuses the wire the old three-wire I2S link used for DATA. There is no
transmit half at all: this chip never answers the bridge, so nothing is
configured for TX and no second pad is spent on it.

That pin was a literal in `sbc_in.c` until the S3 port needed it moved, which
is what makes §14's "all configurable under `idf.py menuconfig`" true rather
than nearly true. It is worth being configurable on this board too: 23 is not a
pin every ESP32 variant *has*, and `uart_set_pin()` rejects an absent one inside
`ESP_ERROR_CHECK` — so a wrong number aborts at boot rather than failing to
build.

This used to be three-wire I2S with the hub as slave. It is UART now: I2S is a
clocked bus with no framing, so a single lost bit shifted every sample after it
and nothing in the protocol could notice or recover. The UART link carries
framed packets with a sync word, length and checksum, so a corrupt packet is
dropped and the next one is fine. `sbc-link.md` has the full account.

### To its own DAC (this chip is the I2S **master**)

| ESP32 | PCM5102A |
|---|---|
| GPIO 26 | BCK |
| GPIO 27 | LRCK |
| GPIO 25 | DIN |
| GND | GND, SCK |
| 3V3 | VIN |

XSMT must be high or the DAC stays muted. GPIO 25 is also the bridge's TX pin —
different chips, so not a conflict.

### LEDs

GPIO 18 -> 74AHCT125 level shifter -> WS2812 DIN. 330 Ω series on data, 1000 µF
across the strip supply, separate 5 V supply, common ground. Budget 60 mA per
pixel at full white; the brightness cap defaults to 10%.

### Bench instruments — optional, off by default

| | |
|---|---|
| GPIO 21 | Monitor input. Wire a satellite's marker pin here to measure audio sync. |
| GPIO 4 | Audio marker output. |
| GPIO 2 | LED marker — one flash per second, in step on every unit. Onboard LED on many boards, but not all. |

None of these correct anything. Every servo and every splice closes through
`play_at` and the phase queue, so enabling or disabling them changes no
behaviour — a correction loop closed through an instrument that is absent in
deployment would be tuned to a configuration nobody ships.

## What to look for in the log

```
I stream:  SoftAP "dancefloor" pass "dancefloor" ch 11, radio at defaults
I stream:  streaming on port 5001, unicast to registered listeners
I sbc_in:  SBC link listening on GPIO 23 at 500000 baud
I stream:  timeline start
I stream:  local playback started: scheduled ..., actual ... (+3 us)
```

Nothing arrives until a phone connects and plays, so before that the hub sits
waiting with `pkts 0`. That is expected, not a fault.

```
I sbc_in:  pkts 252 | 44100 Hz x2 | eff 44050 Hz | sync 0 crc 0 gaps 0 dec 0
           | fed-drop 0 B | max gap 100189 us
```

`eff` is the honest health metric: PCM samples per second actually reaching
playback. `sync` and `crc` in single digits are fine, tens are not. `fed-drop`
**must be 0**.

`max gap` is the longest silence between audio packets, and **79–112 ms is
normal** — the source delivers in bursts and always has. Every other counter
here describes a packet that arrived and was *wrong*; a source that simply stops
sending trips none of them, which is why this one had to be added.

```
W stream:  HEALTH: up 3721 s | heap 59464 (min 52268, window 58120, largest 55296)
           | stack play 3036 mon 1864 | underruns 0 restarts 1 splices 1
           | retunes 9 (0 refused) | sta-left 0 (dropped 0, no-lease 0)
           | sta-timeout 0 | alloc-fail 0
```

Every 60 s, and the line a long run is judged on. The counters never reset, so
they answer "has this been happening slowly for an hour", which no per-window
rate can. **An uptime that resets to 0 means it crashed and rebooted**, which is
easy to miss in a long log.

The four heap terms answer four different questions:

| | |
|---|---|
| `heap` | free now |
| `min` | all-time watermark. **Cannot be dated** — one bad moment an hour ago pins it for the rest of the run |
| `window` | lowest seen this minute, sampled every 5 s and cleared by each line. This is the one that places a dip in time |
| `largest` | largest free block, and it fails *before* `heap` does — the WiFi driver wants 1152 B contiguous, so a fragmented heap refuses that while the free total still looks comfortable |

`alloc-fail` counts allocations that actually failed, which used to be silent and
surfaced as an underrun — a symptom, not a cause. It records rather than logs and
lives in IRAM, because a hook in flash would fault in the one condition it exists
to observe; `ring_monitor_task` notices within 5 s and prints the detail from a
context where logging is safe.

`sta-left N (dropped M, no-lease K) | sta-timeout T` is **how** a satellite left.
`dropped` is a clean disassociation resolved to an IP through the DHCP lease
table, ~11 ms after the event. `no-lease` is the event arriving when the lease
had already gone. `sta-timeout` is the ungraceful case — power cut, out of range
— where no event fires at all and the 2 s registry timeout does the work. Without
that last one a run where a satellite vanished mid-track and a run where nothing
happened printed the same line.

**3–5 `alloc-fail` per mid-track disconnect is the floor, not a regression.**
Those are frames already handed to the WiFi driver before the satellite was
dropped, and nothing above the driver can reach a buffer it has already taken.
Audio is unaffected — underruns stay 0 across it. A disconnect while nothing is
playing costs zero, because the residual *is* the in-flight sends and there are
none; reading that zero as a better floor would hide the case that matters.

`timeline off by ... slewing back` is not alarming on its own, and this is a
reversal of what this file used to say. The timeline oscillates ±130 ms with the
delivery burst pattern, past the 120 ms threshold, and recovers within a second
or two — so the line only prints if an episode lasts more than 5 s. The old
advice ("should not appear during steady playback; the bridge's steady I2S
output is what fixed it") was wrong on both halves: the bridge moved the
burstiness rather than removing it, and the mechanism used to *jump* rather than
slew, writing a transient trough permanently into the timeline and stepping
every unit's phase by −127 ms.

## Warts worth knowing before you debug

The hub's raw phase reading swings by up to 15 ms between consecutive reads,
cause unfound. Watch the smoothed figure, and treat any single-sample number
derived from hub phase — including the per-retune costs it prints — as
untrustworthy. The satellite's readings are quiet by comparison.

## One config difference from the satellite, on purpose

`CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0=y` is set here and **not** on the
satellite. The lwIP tcpip thread runs at priority 18, above every task this
firmware creates, and core locking is off in this build — so every `sendto()`
posts to that thread and blocks, and the work happens there rather than in the
caller. This unit drives ~43 audio sends/s plus 86 frame sends/s per satellite
against a satellite's 4 probes/s, so pinning it off the core carrying `play`,
`vis-draw` and `vis` was the whole fix for the render starvation seen here: wake
overshoot mean 2057 → 634 µs, pattern arithmetic max 23226 → 5254 µs, late frames
10 → 0. The satellite is deliberately left unpinned as the control that
measurement is against. Do not "fix" the asymmetry without reading
[`../docs/architecture.md`](../docs/architecture.md) §13 first.
