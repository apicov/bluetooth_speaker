# Dancefloor hub — chip B of the two-chip master

Takes SBC in over UART from the `bt_bridge` chip, owns the timeline the whole
system plays to, unicasts that SBC to the satellites, and decodes it for its own
DAC and LED strip.

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
W stream:  HEALTH: up 3721 s | heap 59464 (min 52268, largest 48120)
           | stack play 3036 mon 1864 | underruns 0 restarts 1 splices 1
           | retunes 9 (0 refused) | sta-left 0
```

Every 60 s, and the line a long run is judged on. Minimum-ever heap matters more
than current — a leak shows as the minimum walking down — and `largest` catches
fragmentation that total free heap hides. The counters never reset, so they
answer "has this been happening slowly for an hour", which no per-window rate
can. **An uptime that resets to 0 means it crashed and rebooted**, which is easy
to miss in a long log.

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
