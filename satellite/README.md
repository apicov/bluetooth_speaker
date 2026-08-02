# Dancefloor satellite

Joins the master's SoftAP, keeps its clock aligned, receives multicast PCM and
plays each chunk at the instant it was stamped for.

**No Bluetooth here.** The master owns the phone connection; this board only
listens on WiFi and drives a DAC. Any ESP32 variant would work, but the project
standardises on the classic ESP32 so every unit is interchangeable.

## Build and flash

```sh
. ~/.espressif/v6.0.1/esp-idf/export.sh
idf.py set-target esp32
idf.py -p /dev/ttyUSB1 flash monitor
```

## Wiring

Identical to the master — PCM5102A on the same pins, configurable under
`menuconfig` → *Dancefloor satellite*.

| ESP32 | PCM5102A |
|---|---|
| GPIO 26 | BCK |
| GPIO 27 | LRCK |
| GPIO 25 | DIN |
| GND | GND, SCK |
| 3V3 | VIN |

Remember XSMT must be high or the DAC stays muted.

## How playback timing works

Only the **first** chunk's timing is decided explicitly. The satellite converts
its `play_at` into local time with the measured clock offset, waits, and starts
writing. After that `i2s_channel_write` blocks when the DMA buffers are full, so
I2S itself paces everything.

That means the *start* is aligned to well under a millisecond, and thereafter
each board free-runs on its own crystal. See `docs/clock-sync.md`.

**Lost packets become silence of exactly the right length.** Skipping a missing
chunk would pull every later frame earlier and slide the whole stream against the
master permanently — a far worse artefact than a 5.8 ms gap.

## Reading the log

```
I (4210) sat: stream start: play_at 1234567 -> local 1100821 (in 248 ms)
I (4460) sat: playback started
I (9460) sat: buffer 45312 bytes (256 ms)
```

The **buffer** line every 5 s is the drift signal. Both boards nominally run at
44.1 kHz but differ by ~14 ppm, so a satellite clocking slightly slow accumulates
audio it cannot consume and the number climbs; clocking fast, it drains toward
underrun.

**Expect that number to move.** M5 aligns the start; it does not stop the clocks
diverging. A slow, steady change of a few ms per minute is the drift behaving
exactly as measured in M4, and it is precisely the signal M6 will act on.

## Verification

1. Pair a phone to the master (**Dancefloor**) and play something.
2. Satellite logs `stream start` then `playback started`.
3. Both speakers produce audio.
4. Standing between them, no audible echo or comb filtering at the start.
5. `buffer` drifts slowly rather than jumping — jumps mean packet loss.

Expect them to gradually separate over several minutes. That is M6's job, not a
failure of M5.
