# Dancefloor hub — chip B of the two-chip master

Takes audio in over I2S from the `bt_bridge` chip, owns the clock the whole
system synchronises to, multicasts audio to the satellites, and drives its own
DAC and LED strip.

**No Bluetooth on this chip.** See `docs/two-chip-master.md` for why the master
is split in two.

## Build and flash

```sh
. ~/.espressif/v6.0.1/esp-idf/export.sh
idf.py set-target esp32
idf.py -p /dev/ttyUSB1 flash monitor
```

## Wiring

### From the bridge chip (this chip is the I2S **slave**)

| bt_bridge | hub | Signal |
|---|---|---|
| GPIO 26 | GPIO 21 | BCK |
| GPIO 27 | GPIO 22 | WS / LRCK |
| GPIO 25 | GPIO 23 | DATA |
| GND | GND | **required** |

Avoid 16/17 (PSRAM on WROVER) and 5 (strapping pin) if you change these.

### To its own DAC (this chip is the I2S **master**)

| ESP32 | PCM5102A |
|---|---|
| GPIO 26 | BCK |
| GPIO 27 | LRCK |
| GPIO 25 | DIN |
| GND | GND, SCK |
| 3V3 | VIN |

XSMT must be high or the DAC stays muted. Different chip from the bridge, so
reusing 26/27/25 here is not a conflict.

### LEDs

GPIO 18 → 74AHCT125 level shifter → WS2812 DIN. 330 Ω series on data, 1000 µF
across the strip supply, separate 5 V supply, common ground. Budget 60 mA per
pixel at full white.

## What to look for in the log

```
I stream:   SoftAP "dancefloor" up, streaming to 239.12.34.56:5001
I stream:   free heap after WiFi init: ... bytes
I vis:      started: 60 LEDs on GPIO 18
I audio_in: I2S slave receiver up on BCK 21 / WS 22 / DIN 23
I audio_in: measured input rate 44098 Hz
I stream:   timeline start
I stream:   local playback started
```

`measured input rate` is the useful one. As an I2S slave this chip follows
whatever the bridge clocks, so the rate is measured rather than assumed — a
phone negotiating 48 kHz would otherwise throw every timeline calculation out by
9%. A number near nominal means the link is healthy.

The bridge does not clock I2S until a phone connects and plays, so before that
this chip sits waiting. That is expected, not a fault.

`timeline off by ... resyncing` should **not** appear during steady playback. It
did constantly on the old single-chip master, where audio came straight from
A2DP in ~100 ms bursts. The bridge's steady I2S output is what fixed it, so if
it comes back, something real is wrong.
