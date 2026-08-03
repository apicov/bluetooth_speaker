# Dancefloor satellite

Joins the master's SoftAP, keeps its clock aligned with the hub's, receives
unicast SBC, decodes it, and plays each chunk at the instant it was stamped for.
Drives an LED strip from the audio it is about to play.

**No Bluetooth here.** The master owns the phone connection; this board only
listens on WiFi. Any ESP32 variant would work for this role, but the project
standardises on the classic ESP32 so every unit is interchangeable with the
master, which does need Bluetooth Classic.

Run as many of these as you like. Registration is implicit: the hub sends audio
to whatever has sent it a time probe in the last 10 seconds, so a unit that is
keeping its clock synchronised is by definition alive and listening.

## Build and flash

```sh
. ~/.espressif/v6.0.1/esp-idf/export.sh
idf.py set-target esp32
idf.py -p /dev/ttyUSB2 flash monitor
```

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
W sat: HEALTH: up 3721 s | heap 83380 (min 73924, largest 61240)
       | stack play 3048 drift 1796 | underruns 0 anchors 1 splices 2
       | retunes 7 (0 refused) | gaps 0 wifi-drops 0 | clock TSF (tsf 1/probe 0)
```

Every 60 s, and the line a long run is judged on. Minimum-ever heap matters more
than current, since a leak shows as the minimum walking down, and `largest`
catches fragmentation that total free heap hides. `clock TSF (tsf N/probe M)`
says which source is live and how many anchors used each; a rising `probe` means
TSF is dropping out. **An uptime that resets to 0 means it crashed and
rebooted.**

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
