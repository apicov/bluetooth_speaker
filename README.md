# Dancefloor

A phone plays music over Bluetooth. Several battery-powered ESP32 speakers
scattered around a field play it **at the same instant**, each pulsing an LED
strip on the beat.

Measured on hardware: **0.1–0.5 ms** between two speakers just after a track
boundary, a few milliseconds by the end of a long track, against a ~5 ms
threshold where a listener starts to notice. Nothing about the lights is
transmitted between units — each derives them from audio that is already
synchronised, so they agree for free.

## The shape of it

```
 phone ──A2DP/SBC──▶ bt_bridge ──SBC over UART──▶ hub ──SBC over WiFi──▶ satellites
                     (chip A)      500 kbaud      (chip B)   unicast      (any number)
                                                     │                        │
                                                  speaker                  speaker
                                                  + strip                  + strip
```

| Directory | What it is |
|---|---|
| `bt_bridge/` | Chip A of the master. Bluetooth A2DP sink; forwards raw SBC. Nothing else. |
| `hub/` | Chip B. SoftAP, the presentation timeline every unit obeys, its own speaker and strip. |
| `satellite/` | Joins the hub's SoftAP, plays the stream, drives a strip. Any number of these. |
| `components/dancefloor_sync/` | Wire format and the clock estimator. No ESP-IDF dependencies, host-testable. |
| `components/dancefloor_leds/` | FFT, onset detection, patterns, strip driver. Shared by hub and satellites. |
| `components/sbc_decoder/` | Vendored SBC decoder. |
| `tools/pattern_lab/` | The LED pipeline on a laptop, compiled from the firmware sources. |

The master is two chips because Bluetooth and WiFi on one ESP32 fight over the
radio, the memory and the CPU. See [`docs/two-chip-master.md`](docs/two-chip-master.md).

**Classic ESP32 everywhere**, master and satellite alike. Only the original
ESP32 has Bluetooth Classic, which A2DP requires — the S3, C3 and C6 cannot ever
receive it. One part number, interchangeable spares.

## Two ideas the whole thing rests on

**Schedule the future, never the present.** The hub does not say "play now"; it
says "play at master-time T", roughly 200 ms ahead. Each unit converts T to its
own clock and waits. Network jitter stops mattering as long as the packet
arrives before T.

**Derive, do not transmit.** Every unit runs the same FFT and beat detector over
the audio it is about to play. Sending analysis results would add a second thing
to keep synchronised; the audio already is, so anything computed from it locally
is too.

## Build and flash

Needs ESP-IDF v6.

```sh
. ~/.espressif/v6.0.1/esp-idf/export.sh

cd bt_bridge && idf.py -p /dev/ttyUSB0 flash monitor   # chip A
cd hub       && idf.py -p /dev/ttyUSB1 flash monitor   # chip B
cd satellite && idf.py -p /dev/ttyUSB2 flash monitor   # each satellite
```

Pins, LED count, brightness and pattern are under `idf.py menuconfig` →
**Dancefloor \***.

## Tests

Host-side, no hardware, no ESP-IDF:

```sh
cd components/dancefloor_sync/test && make check   # the clock estimator
cd components/dancefloor_leds/test && make check   # FFT, onsets, patterns,
                                                   # block alignment, cross-unit
                                                   # determinism
```

Two of those exist because of specific field failures and are worth knowing
about. `test_align` pins that two units cut and label their analysis blocks
identically; `test_pattern_sync` pins that a pattern handed those blocks renders
identical pixels whatever its unit's join time, render count or drop history.
Both carry a deliberately broken case that the suite **requires** to fail — a
test that only passes against correct code has not been shown to detect
anything.

## Working on the lights

`tools/pattern_lab` runs the identical analysis and pattern code over a WAV on a
laptop, compiled from `components/dancefloor_leds` rather than copied, so it
cannot drift from what the strips do.

```sh
cd tools/pattern_lab && make
./pattern_lab track.wav                  # live in the terminal
./pattern_lab track.wav --png out.png    # the whole track as an image
./pattern_lab track.wav --csv trace.csv  # per-frame numbers for tuning
```

WAVs come from the desktop client, which lives in its own repository —
[`dancefloor-tools`](../dancefloor-tools) — and joins the hub's WiFi like any
other satellite.

## Reading a running system

Both firmwares print a `HEALTH` line every 60 s with uptime, heap (current,
minimum-ever and largest free block), per-task stack headroom, and cumulative
counts of underruns, re-anchors, splices, retunes, lost-packet gaps and WiFi
drops. Anything eventful is logged as it happens; the periodic lines are on
`CONFIG_DANCEFLOOR_LOG_PERIOD_S`.

`TRACK DIVERGENCE` prints once per track, at a track boundary — the one instant
that recurs identically in every track, so it is comparable across tracks,
sessions and builds. A reading taken anywhere else depends on where in that cycle
you looked.

## Documents

| | |
|---|---|
| [`docs/architecture.md`](docs/architecture.md) | The way in. Concepts first, then the system, then the decisions. |
| [`docs/clock-sync.md`](docs/clock-sync.md) | The sync maths, the phase servo, TSF, measured results. |
| [`docs/sbc-link.md`](docs/sbc-link.md) | The wire between the two master chips. |
| [`docs/two-chip-master.md`](docs/two-chip-master.md) | Why the master is split, with memory numbers. |

Both long documents end with the pattern that recurred at every level of this
project: **every real fault was invisible until something counted it.** Several
were actively disguised as something else. If you extend this, add the counter
before you form the theory.

## State

Working on hardware: Bluetooth in, synchronised audio out of multiple speakers,
beat-reactive strips that agree with each other. What is left is power,
enclosures and a field test — and a long soak, since the longest evidenced
session is ten minutes against a four-hour target. `docs/architecture.md` §16
and §17 list the known warts honestly, including the ones with no explanation
yet.
