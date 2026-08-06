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
phone --A2DP/SBC--> bt_bridge --SBC over SPI--> hub_s3 --SBC over WiFi--> satellites
                    (chip A)     5 MHz, 4 wires   (chip B)   unicast      (any number)
                                                    |                        |
                                                 speaker                  speaker
                                                 + strip                  + strip
```

| Directory | What it is |
|---|---|
| `bt_bridge/` | Chip A of the master. Bluetooth A2DP sink; forwards raw SBC. Nothing else. |
| `hub/` | Chip B. SoftAP, the presentation timeline every unit obeys, its own speaker and strip. |
| `hub_s3/` | The same hub firmware built for an ESP32-S3 — the hub on the bench now. The classic `hub/` it superseded still speaks the UART the bridge no longer sends; see [`docs/hub-s3-gap-list.md`](docs/hub-s3-gap-list.md) §7. |
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

Most of what follows applies to a classic-ESP32 hub and an S3 hub alike, only
the pin numbers moving — the bridge link is the exception, now SPI and S3-only.
A satellite is wired like a hub minus the bridge link.

### Bridge to hub

Four signals, a handshake line and a common ground. The bridge is always a
classic ESP32 and drives the link as SPI master; the hub is the slave clocked by
it, one direction only — no MISO.

| | bridge (ESP32) | S3 hub (XIAO ESP32-S3) | |
|---|---|---|---|
| SCK | **GPIO 14** | **GPIO 44**, pad `D7` | HSPI IOMUX on the bridge |
| MOSI | **GPIO 13** | **GPIO 6**, pad `D5` | the data |
| CS | **GPIO 15** | **GPIO 5**, pad `D4` | the framing — one assertion is one packet |
| HANDSHAKE | **GPIO 25** (in) | **GPIO 3**, pad `D2` (out) | hub says "buffer armed" |
| Ground | GND ×4 | GND ×4 | keep them all |

The bridge's pins are literals in `bt_bridge/main/sbc_spi.c` — HSPI on its IOMUX
pins, chosen once and not moved. The hub's are `menuconfig` values
(`DANCEFLOOR_SBC_SPI_*_PIN` under *Dancefloor hub*), defaulting to the pads
above. The one clock knob is `DANCEFLOOR_SBC_LINK_SPI_HZ` under *Dancefloor
diagnostics*: the SCK the bridge drives, default **5 MHz**, changed by
reflashing the bridge only.

**Add a 10 kΩ pull-up from CS to 3V3.** CS is active-low and carries the
framing, and the bridge's pin is high-Z until the firmware drives it — during
boot, reset or a reflash the line floats, a floating CS reads as asserted, and
the slave then clocks whatever sits on SCK and MOSI into a buffer as if it were
a packet. The pull-up holds it deasserted until the master takes over, the
mirror of the handshake line's pull-down for the opposite fault.

**The handshake is not optional.** ESP-IDF's `spi_slave` loses any transfer the
master clocks with nothing queued, so the hub holds the line high while a buffer
is armed and drops it for the transfer, and the bridge waits on a GPIO interrupt
rather than a poll — that chip runs Bluetooth Classic, and a task spinning on a
pin is what its budget cannot absorb. The line is pulled down on the bridge, so a
hub that is off, reflashing or crashed reads as a stall rather than a stream
clocked into nothing. [`docs/sbc-link.md`](docs/sbc-link.md) has the reasoning.

**Use every ground you can** — GND ×4, the same lesson as the UART before it on
the same breadboard jumpers. The 2060-byte frame (sized for the codec's bitpool
ceiling) is clean at 5 MHz and fails at 10 MHz on thin leads; the failure is the
hub's `short` counter moving while the bridge stays clean, so drop the clock, do
not chase the code.

> The classic ESP32 `hub/` still listens on the UART this replaced, and the
> bridge no longer sends it — a classic hub paired with the current bridge plays
> silence, and no counter says why (`max gap` grows, everything else reads zero,
> exactly like a phone that stopped). The S3 hub is the one on the bench;
> [`docs/hub-s3-gap-list.md`](docs/hub-s3-gap-list.md) §7 has the port back.

Verifying the link is up, from the S3 hub's log:

```
I sbc_in: pkts 252 | 44100 Hz x2 | eff 44050 Hz | hdr 0 crc 0 short 0 gaps 0
```

`crc` and `short` at zero mean the wiring is keeping up. `short` moving means CS
is splitting transfers (framing); `crc` moving means SCK/MOSI bit errors (signal
integrity) — in the second case drop the clock and reflash the bridge. Nothing
arrives until a phone connects and plays, so `pkts 0` before that is expected
rather than a fault.

### Bridge status LEDs

Two optional indicators on the bridge, so a box with no console attached still
says what it is doing.

| | pin | means |
|---|---|---|
| Connected | **GPIO 32** | solid on while a phone is connected over A2DP |
| Streaming | **GPIO 33** | blinks at 0.5 Hz — a second on, a second off — while audio packets are arriving |

```
  GPIO 32 ──[330 Ω]──▶ LED ──▶ GND
  GPIO 33 ──[330 Ω]──▶ LED ──▶ GND
```

Both are `menuconfig` values under *Bridge status LEDs*
(`BRIDGE_LED_CONNECTED_GPIO`, `BRIDGE_LED_STREAMING_GPIO`); **-1 disables**
either one, and nothing else changes if you leave them unwired.

That is every pin the bridge uses, so the whole chip fits in one table:

| GPIO | | |
|---|---|---|
| 13 | MOSI, out | the SPI link |
| 14 | SCK, out | |
| 15 | CS, out | 10 kΩ pull-up to 3V3 |
| 25 | HANDSHAKE, in | |
| 32 | Connected LED, out | |
| 33 | Streaming LED, out | |

Everything else is free. If you move an LED, keep off 34–39 — input-only on a
classic ESP32, they cannot drive anything, and the Kconfig range stops at 33 for
that reason — and off 6–11, which are the flash. GPIO 2 works but is worth
avoiding here: it is a strapping pin, and it is already the default for
`DANCEFLOOR_ENABLE_LED_MARKER` on the other builds, so pointing this at it too
would give one light two meanings across the floor.

**The blink follows the packets, not the phone's reported state.** It is driven
from the A2DP audio callback and goes dark after 300 ms with nothing arriving,
so a stream that stalls with the phone still calling itself "playing" shows as a
dark LED next to a lit one. That pair — connected but not streaming — is the
useful reading: it separates a pairing problem from a playback one before you
reach for a console. What it does *not* say is whether the hub is receiving any
of it; the LED is lit by packets leaving this chip, and `sbc_in` on the hub is
still the only thing that reports what arrived.

### The PCM5102A DAC

Every unit drives its own DAC — the hub is a full speaker, not a base station —
and in every case **the ESP32 is the I2S master**, generating the clocks the DAC
follows.

| PCM5102A | classic hub | classic satellite | S3 hub (`hub_s3/`) |
|---|---|---|---|
| BCK | GPIO 26 | GPIO 26 | **GPIO 7**, pad `D8` |
| LRCK | GPIO 27 | GPIO 27 | **GPIO 8**, pad `D9` |
| DIN | GPIO 25 | GPIO 25 | **GPIO 9**, pad `D10` |
| VIN | 3V3 | 3V3 | 3V3 |
| GND | GND | GND | GND |
| SCK | GND | GND | GND |

There is no S3 satellite build. If you make one, the pins above are what it
would take — `hub_s3/` and `satellite/` read the same three Kconfig symbols.

Four things that are not obvious from the table:

- **`XSMT` must be high or the DAC stays silently muted.** No error, no log line,
  no sound. This is the first thing to check.
- **`SCK` goes to ground.** The PCM5102A derives its own internal clock, and the
  firmware sets `mclk = I2S_GPIO_UNUSED` to match. It is not a pin you wire.
- **GPIO 25 is also the bridge's UART TX**, and that is not a conflict — those
  are different chips. It only looks alarming when both pin maps are on one page.
- **The S3's pads are not fixed.** S3 I2S routes through the GPIO matrix, so any
  of the eleven work; all three are `menuconfig` values under *Dancefloor hub*.
  They sit on the SPI-labelled pads so the DAC is one ribbon off the end of the
  header.

**Which channels a box plays** is a `menuconfig` choice on the hub and on each
satellite, `DANCEFLOOR_OUTPUT_CHANNELS`: stereo (the default), left only, right
only, or a `(L+R)/2` mono downmix. Every unit still receives the same stereo
stream — this only decides what that box's speaker does with it — so a stereo
pair is two differently-configured images, and the `OUTPUT:` line at boot says
which one a board is running. The selected channel goes into *both* I2S slots
rather than the other being muted, because which slot an amp latches is a
hardware strap and muting the wrong one gives silence.

A satellite can run with **no DAC at all** for bring-up:
`DANCEFLOOR_USE_INTERNAL_DAC` plays through the ESP32's built-in converters on
GPIO 25 (left) and 26 (right). They are 8-bit — 48 dB of dynamic range against
96 — so quiet passages sit in obvious hiss, and it is a way to test a board with
nothing wired rather than a way to listen. It is also **classic-ESP32 only**:
the S3 has no internal DAC hardware, so an S3 unit needs the PCM5102A.

> Nothing in this project has been heard through a real PCM5102A yet. M1–M3 were
> marked complete on log output and on the desktop client's audio; the boards are
> still to be wired. See `docs/architecture.md` §16.

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
| LED marker | out | GPIO 2 | GPIO 2, pad `D1` |

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
| Official Espressif ESP32-DevKitC | none — power LED only |
| XIAO ESP32-S3 | none that works — see below |

If the log says the marker fired and nothing lit, this is the setting to change.
With no usable onboard LED, wire one to any free pin through a resistor, or
point it at something you can watch on a scope.

**On the XIAO, wire an external one.** Its onboard LED is documented as GPIO 21
and does not light when pointed at, cause unresolved — the pin may differ by
board revision, the Sense's SD slot reportedly uses 21 for CS, and it is active
low regardless, so even working it would read inverted against every other unit.
Pad `D1` is free and is GPIO 2, which is already this setting's default, so
nothing needs configuring:

```
  GPIO 2 (D1) ──[330 Ω]──▶ LED ──▶ GND
```

The other free pads are `D2` (GPIO 3, a strapping pin — JTAG source select),
`D5` (GPIO 6) and `D6` (GPIO 43, the ROM UART0 TX, which can glitch at boot).
`D6` cannot take this setting anyway: the range stops at GPIO 33, since 34–39 on
a classic ESP32 are input-only and cannot drive anything.

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
