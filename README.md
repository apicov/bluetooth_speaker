# Dancefloor

A phone plays music over Bluetooth. Several battery-powered ESP32 speakers
scattered around a field play it **at the same instant**, each pulsing an LED
strip on the beat.

Measured on hardware: **0.1–0.5 ms** between two speakers just after a track
boundary, a few milliseconds by the end of a long track, against a ~5 ms
threshold where a listener starts to notice. By default nothing about the lights
is transmitted between units — each derives them from audio that is already
synchronised, so they agree for free. A unit can now be built to draw analysis
frames the hub sends it instead, which is a second way to reach the same
agreement rather than a replacement; see [`docs/architecture.md`](docs/architecture.md) §12.

## The shape of it

```
phone --A2DP/SBC--> bt_bridge --SBC over UART--> hub --SBC over WiFi--> satellites
                    (chip A)       500 kbaud     (chip B)    unicast     (any number)
                                                    |                        |
                                                 speaker                  speaker
                                                 + strip                  + strip
```

| Directory | What it is |
|---|---|
| `bt_bridge/` | Chip A of the master. Bluetooth A2DP sink; forwards raw SBC. Nothing else. |
| `hub/` | Chip B. SoftAP, the presentation timeline every unit obeys, its own speaker and strip. |
| `hub_s3/` | The same hub firmware built for an ESP32-S3. Experimental, never run on hardware. |
| `satellite/` | Joins the hub's SoftAP, plays the stream, drives a strip. Any number of these. |
| `components/dancefloor_sync/` | Wire format and the clock estimator. No ESP-IDF dependencies, host-testable. |
| `components/dancefloor_leds/` | FFT, onset detection, patterns, strip driver. Shared by hub and satellites. |
| `components/sbc_decoder/` | Vendored SBC decoder. |
| `tools/pattern_lab/` | The LED pipeline on a laptop, compiled from the firmware sources. |
| `tools/tuning/` | The detector's tuning harness — sweeps, the negative control, the corpus manifest. |

The master is two chips because Bluetooth and WiFi on one ESP32 fight over the
radio, the memory and the CPU. See [`docs/two-chip-master.md`](docs/two-chip-master.md).

**Classic ESP32 everywhere**, master and satellite alike. Only the original
ESP32 has Bluetooth Classic, which A2DP requires — the S3, C3 and C6 cannot ever
receive it. One part number, interchangeable spares.

That constraint binds `bt_bridge` and nothing else. The hub runs no Bluetooth,
so it is the one image here an S3 can carry, and `hub_s3/` does exactly that on
a Seeed XIAO ESP32-S3. It is a departure from the one-part-number decision
rather than an exception granted by it: an S3 hub cannot be promoted to bridge,
and a spare classic ESP32 needs a rebuild to replace it.

## Wiring

Everything below applies to a classic-ESP32 hub and an S3 hub alike; only the
pin numbers move. A satellite is wired like a hub minus the bridge link.

### Bridge to hub

One signal wire and a common ground, whichever hub you are using. The bridge is
always a classic ESP32 and its end never changes.

| | bridge → | → classic hub | → S3 hub |
|---|---|---|---|
| Board | ESP32 | ESP32 | XIAO ESP32-S3 |
| Signal | **GPIO 25** (TX) | **GPIO 23** (RX) | **GPIO 44** (RX), pad `D7` |
| Ground | GND | GND | GND |

500 kbaud, SBC, one direction only. Wire the bridge's TX to the hub's RX and the
grounds together; that is the whole link.

**There is no return wire.** The bridge talks and the hub listens, so neither
chip configures a TX pin at its receiving end. The bridge does name GPIO 34 as
its RX, an input-only pin it never reads.

**Use every ground you can.** `docs/architecture.md` §14 says GND ×4 and means
it — this link will not run above 500 kbaud on thin breadboard leads, and the
failure is progressive rather than obvious. At 750 kbaud it is 20–30 bad sync
words and 15–20 CRC errors per 5 s; at 1 Mbaud, half the packets are corrupt.
[`docs/sbc-link.md`](docs/sbc-link.md) has the ladder.

The hub's end is a `menuconfig` value — `DANCEFLOOR_SBC_UART_RX_PIN` under
**Dancefloor hub**, defaulting to 23 on the classic build and 44 on the S3 one.
The bridge's is not: GPIO 25 is a literal in `bt_bridge/main/sbc_uart.c`, since
that chip is always the same part on the same board.

Be careful moving it on an S3: **GPIO 22–25 do not exist on that part** (its
pins are 0–21 and 26–48), 26–32 are wired to the SPI flash and 33–37 to the
PSRAM, and the XIAO breaks out only eleven pads in total. A pin the target does
not have is rejected by `uart_set_pin()` inside `ESP_ERROR_CHECK`, so the board
aborts at boot rather than failing to build. [`hub_s3/README.md`](hub_s3/README.md)
has the full pad map.

Verifying the link is up, from the hub's log either way:

```
I sbc_in:  SBC link listening on GPIO 23 at 500000 baud
I sbc_in:  pkts 252 | 44100 Hz x2 | eff 44050 Hz | sync 0 crc 0 gaps 0 dec 0
```

`sync` and `crc` in single digits are fine; tens mean the wiring, not the code.
Nothing arrives at all until a phone connects and plays, so `pkts 0` before that
is expected rather than a fault.

### The NeoPixel strip

Every unit drives its own strip — the hub is a full speaker, not a base station
— so this is identical on a hub, an S3 hub and a satellite except for the data
pin.

| | classic hub | S3 hub | satellite |
|---|---|---|---|
| WS2812 data | **GPIO 18** | **GPIO 1**, pad `D0` | GPIO 18 |

```
   ESP32 / XIAO            74AHCT125            WS2812 strip
                          (VCC = 5 V)

  data GPIO (3.3 V) ────▶ 1A     1Y ──[330 Ω]──▶ DIN
              GND ──┬───▶ GND
                    └───▶ 1OE   tie low to enable the channel

  5 V supply  + ────────▶ VCC
              + ──────────────────────────────▶ strip 5 V
              − ──────────────────────────────▶ strip GND
              − ──────────────────────────────▶ board GND

                    1000 µF across strip 5 V / GND, at the first pixel
```

Four things, and all four earn their place:

- **A 74AHCT125 level shifter is not optional.** WS2812 wants 5 V logic and both
  boards drive 3.3 V. It works on a bench often enough to be misleading and
  fails outdoors in the cold. Power the shifter from **5 V**, not 3.3 — that is
  the whole point of it — and tie `1OE` low so the channel is enabled.
- **330 Ω in series on the data line**, at the strip end, against ringing.
- **1000 µF across the strip's 5 V and ground**, close to the first pixel.
- **Separate 5 V supply for the strip, with its ground tied to the board's.**
  Budget ~60 mA per pixel at full white; ~150 pixels is ~45 W, which is far past
  anything a devkit's regulator will pass. The 8-pixel default is the bring-up
  figure precisely because it will run off the board.

The strip is driven as **SPI2 MOSI through the GPIO matrix**, which is why the
data pin does not have to be an SPI-labelled pad on either board. It is
deliberately not RMT: `led_strip` enables and disables an RMT channel per frame
and `rmt_disable` races the transmit-done interrupt, which wedges the strip
after some minutes. `docs/architecture.md` §12 has the failure.

Under **Dancefloor LEDs** in `menuconfig`: `LED_COUNT` is the *total* pixels on
the one data line, so four chained 8-LED sticks is 32 and not 8; `LED_TYPE` gets
the wire order right, and setting it wrong swaps colours rather than failing, so
it looks like a bug in the patterns; and `LED_BRIGHTNESS` caps every pixel,
defaulting to 10%, which dark-adapted eyes outdoors cannot tell from full and
which reclaims most of the LED power budget.

### Bench markers

Three optional instruments, all off by default, none of which any control loop
closes through — enabling or disabling them changes no behaviour, only whether
the measurement exists.

**There are two different markers and they are easy to confuse.** One measures
audio, one measures light, and GPIO 21 swaps roles between the two hub builds:

| | | classic hub | S3 hub |
|---|---|---|---|
| Audio marker | out | GPIO 4 | GPIO 4, pad `D3` |
| Audio monitor | in | GPIO 21 | GPIO 5, pad `D4` |
| LED marker | out | GPIO 2 | GPIO 21 |

**The audio marker pair** (`DANCEFLOOR_ENABLE_MARKER`, under *Dancefloor hub*)
measures how far apart two units' *audio* really is, at the speaker rather than
inferred from clock estimates. Wire a **satellite's** marker pin to the **hub's**
monitor pin, plus a common ground, and the hub reports the gap directly. It
needs a wire between two boards, so a deployed floor cannot have it.

**The LED marker** (`DANCEFLOOR_ENABLE_LED_MARKER`, under *Dancefloor LEDs*)
flashes once per second of master-clock time, on every unit at the same instant.
Stand two boards side by side: if the LEDs blink together the chain agrees, and
if one lags it is obvious with no console and no wiring at all.

It is worth knowing why it exists, because it is the only instrument that covers
what it covers. Every other measurement here — the audio marker, `AUDIO SYNC`,
`TRACK DIVERGENCE` — watches the *audio* path. Between playback and the pixels
sit the analysis stream buffer, the FFT, the detector and the render, and
nothing measures any of it. The strips could be visibly out of step with the
speakers perfectly aligned and no counter anywhere would say so.

It normally needs no wiring, because the default is an onboard LED — but which
pin that is varies by board and cannot be detected:

| Board | Pin |
|---|---|
| DOIT ESP32 DEVKIT V1, NodeMCU-32S, most WROOM-32 clones | GPIO 2 |
| WEMOS/LOLIN D32, some TTGO | GPIO 5 |
| XIAO ESP32-S3 | GPIO 21, **active low** |
| Official Espressif ESP32-DevKitC | none — power LED only |

If the log says the marker fired and nothing lit, this is the setting to change.
With no usable onboard LED, wire one to any free pin through a resistor, or
point it at something you can watch on a scope.

Two cautions. On the XIAO the onboard LED is **active low** and the driver does
not know that, so it reads inverted against every other unit — one gap per
second rather than one flash, which is still perfectly usable for "are they
together" but will look wrong beside a classic board. And the setting's range
stops at GPIO 33, so on the XIAO the free pads `D1`/`D2`/`D5` (GPIO 2/3/6) can
take it but `D6` (GPIO 43) cannot.

The eye resolves maybe 10–20 ms and a 240 fps phone camera about 4 ms, so this
answers "are they together", not "by how much". The numbers live in `AUDIO SYNC`
and `TRACK DIVERGENCE`.

## Two ideas the whole thing rests on

**Schedule the future, never the present.** The hub does not say "play now"; it
says "play at master-time T", roughly 200 ms ahead. Each unit converts T to its
own clock and waits. Network jitter stops mattering as long as the packet
arrives before T.

**Derive, do not transmit.** Every unit runs the same FFT and beat detector over
the audio it is about to play. Sending analysis results would add a second thing
to keep synchronised; the audio already is, so anything computed from it locally
is too.

That second thing now exists as an option, and it is worth being precise about
why it does not break the idea. A unit built for `LED_SOURCE_REMOTE` runs no
analysis and draws frames the hub computed, each at the master-clock instant it
carries — so it still keeps its own appointment against a shared timeline, and
still corrects nothing against anything. What it buys is that the algorithm need
not be proved identical across units, because only one copy of the decision
exists. What guarantees agreement is narrower than before: units taking frames
from the *same* source render identically.

## Build and flash

Needs ESP-IDF v6.

```sh
. ~/.espressif/v6.0.1/esp-idf/export.sh

cd bt_bridge && idf.py -p /dev/ttyUSB0 flash monitor   # chip A
cd hub       && idf.py -p /dev/ttyUSB1 flash monitor   # chip B
cd satellite && idf.py -p /dev/ttyUSB2 flash monitor   # each satellite
```

For an S3 hub, use `hub_s3/` in place of `hub/` — same firmware, different
target and pin map:

```sh
cd hub_s3 && idf.py set-target esp32s3 && idf.py -p /dev/ttyACM0 flash monitor
```

Note `ttyACM0`, not `ttyUSB`. The XIAO has no USB-UART bridge — the S3's USB
peripheral is the connector — so the port disappears on every reboot.

Pins, LED count, brightness and pattern are under `idf.py menuconfig` ->
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
minimum-ever, lowest-this-minute and largest free block), per-task stack
headroom, failed allocations, and cumulative counts of underruns, re-anchors,
splices, retunes, lost-packet gaps and WiFi drops. The hub adds how satellites
left — cleanly, unresolved, or by timing out — and the satellite adds its clock
source and its LED frame source. Anything eventful is logged as it happens; the
periodic lines are on `CONFIG_DANCEFLOOR_LOG_PERIOD_S`.

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
| [`docs/tuning-corpus.md`](docs/tuning-corpus.md) | What the beat detector was tuned against, and the commands to do it again. |

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
