# Dancefloor

A phone plays music over Bluetooth. Several battery-powered ESP32 speakers
scattered around a field play it **at the same instant**, each pulsing an LED
strip on the beat.

Nothing corrects itself against a neighbour. The hub stamps every chunk of audio
with a master-clock instant 350 ms in the future, and every unit — the hub
included — converts that instant to its own clock and waits for it. Units agree
because they all keep the same appointment, not because they listen to each
other. What that costs in practice is printed as `TRACK DIVERGENCE`, once per
track, on the hub.

## The shape of it

```
phone --A2DP/SBC--> bt_bridge --SBC over SPI--> hub_s3 --SBC over WiFi--> satellites
                    (chip A)     5 MHz, 4 wires   (chip B)   unicast      (any number)
                                                    |                        |
                                                 speaker                  speaker
                                                 + strip                  + strip
```

Everything on the WiFi hop is unicast to each registered satellite, so the hub's
airtime grows with speaker count: audio, clock replies and analysis frames are
all sent once per satellite. Multicast would make that flat in N and was tried;
it works, but unicast is what proved stable in use and it is what ships.

| Directory | What it is |
|---|---|
| `bt_bridge/` | Chip A of the master. Bluetooth A2DP sink; forwards raw SBC. Nothing else. |
| `hub_s3/` | Chip B, on an ESP32-S3. SoftAP, the presentation timeline every unit obeys, its own speaker and strip. The only hub. |
| `satellite/` | Joins the hub's SoftAP, plays the stream, drives a strip. Any number of these. One source tree, two targets. |
| `components/dancefloor_sync/` | Wire format, the clock estimator, the rate servo, the shared audio-output and radio settings. No ESP-IDF dependencies in the parts that matter, host-testable. |
| `components/dancefloor_leds/` | FFT, onset detection, patterns, the visualiser, and the pluggable analysers. Shared by hub and satellites. |
| `components/led_strip_wrapper/` | The WS2812 driver, as SPI2 MOSI rather than RMT. |
| `components/sbc_decoder/` | Vendored SBC decoder. |
| `tools/sat.py` | Builds, flashes and monitors either satellite; one source tree, two images. |
| `tools/pattern_lab/` | The LED pipeline on a laptop, compiled from the firmware sources. |
| `tools/tuning/` | The beat detector's tuning harness — sweeps over a corpus, driven through `pattern_lab`. |
| `tools/soak/` | Long-run capture and analysis. Reads every unit's console at once into `raw.log`, `metrics.csv` and `events.csv`; `analyse.py` says what happened. |
| `tools/satsim/` | N fake satellites on a laptop, to load the hub's unicast fan-out. Also checks the hub's XOR parity against what actually arrived. |
| `tools/log_collector/` | Collects logs over WiFi from every unit, plus the bridge's console over USB, into one merged stream. |
| `tools/gen_vol_table.py` | Regenerates the volume taper table in `audio_out.h`. |
| `tools/syntax_check.py` | Syntax-checks firmware sources from `compile_commands.json`, with no working IDF environment. |

The master is two chips because Bluetooth and WiFi on one ESP32 fight over the
radio, the memory and the CPU.

**Classic ESP32 for the bridge.** Only the original ESP32 has Bluetooth Classic,
which A2DP requires — the S3, C3 and C6 cannot ever receive it. That constraint
binds `bt_bridge` and nothing else. The hub runs no Bluetooth, so it is the one
image here an S3 can carry, and `hub_s3/` does exactly that on a Seeed XIAO
ESP32-S3. An S3 hub cannot be promoted to bridge, and a classic ESP32 needs a
rebuild to replace it.

## Wiring

Per-board pin maps live with the boards: [`hub_s3/README.md`](hub_s3/README.md)
has the S3 hub's ten used GPIOs, and [`satellite/README.md`](satellite/README.md)
has the satellite's for both targets. The bridge has no README of its own, so its
wiring is here.

### Bridge to hub

Four signals, a handshake line and a common ground. The bridge is always a
classic ESP32 and drives the link as SPI master; the hub is the slave clocked by
it, one direction only — no MISO.

| | bridge (ESP32) | S3 hub (XIAO ESP32-S3) | |
|---|---|---|---|
| SCK | **GPIO 14** | **GPIO 44**, pad `D7` | IOMUX pins on the bridge |
| MOSI | **GPIO 13** | **GPIO 6**, pad `D5` | the data |
| CS | **GPIO 15** | **GPIO 5**, pad `D4` | the framing — one assertion is one packet |
| HANDSHAKE | **GPIO 25** (in) | **GPIO 3**, pad `D2` (out) | hub says "buffer armed" |
| Ground | GND ×4 | GND ×4 | keep them all |

The bridge's pins are literals in `bt_bridge/main/sbc_spi.c`, chosen once and not
moved. The hub's are `menuconfig` values (`DANCEFLOOR_SBC_SPI_*_PIN` under
*Dancefloor hub*), defaulting to the pads above. The one clock knob is
`DANCEFLOOR_SBC_LINK_SPI_HZ` under *Dancefloor diagnostics*: the SCK the bridge
drives, default **5 MHz**, changed by reflashing the bridge only.

**Add a 10 kΩ pull-up from CS to 3V3.** CS is active-low and carries the framing,
and the bridge's pin is high-Z until the firmware drives it — during boot, reset
or a reflash the line floats, a floating CS reads as asserted, and the slave then
clocks whatever sits on SCK and MOSI into a buffer as if it were a packet. The
pull-up holds it deasserted until the master takes over, the mirror of the
handshake line's pull-down for the opposite fault.

**The handshake is not optional.** ESP-IDF's `spi_slave` loses any transfer the
master clocks with nothing queued, so the hub holds the line high while a buffer
is armed and drops it for the transfer, and the bridge waits on a GPIO interrupt
rather than a poll — that chip runs Bluetooth Classic, and a task spinning on a
pin is what its budget cannot absorb. The line is pulled down on the bridge, so a
hub that is off, reflashing or crashed reads as a stall rather than a stream
clocked into nothing.

**Use every ground you can** — GND ×4. The 2060-byte frame (a 12-byte header plus
`SBC_LINK_MAX_PAYLOAD`, sized for the codec's bitpool ceiling) is clean at 5 MHz
and failed at 10 MHz on thin breadboard leads; the failure is the hub's `short`
counter moving while the bridge stays clean, so drop the clock, do not chase the
code.

Verifying the link is up, from the S3 hub's log:

```
I sbc_in: pkts 252 | 44100 Hz x2 | eff 44050 Hz | hdr 0 crc 0 short 0 gaps 0 ...
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
| Streaming | **GPIO 33** | blinks at 0.5 Hz while audio packets are arriving |

```
  3V3 ──[330 Ω]──▶ LED ──▶ GPIO 32
  3V3 ──[330 Ω]──▶ LED ──▶ GPIO 33
```

**Both LEDs are wired to 3V3, not to ground** — the pin sinks the current, so a
*low* level lights them. `BRIDGE_LED_ACTIVE_LOW` says so and defaults to `y`. If
you wire them the other way round (pin → resistor → LED → GND) set it to `n`,
because getting it wrong does not give you a dark LED, it gives you an inverted
one: the connected LED solid whenever *no* phone is connected.

All three are `menuconfig` values under *Bridge status LEDs*
(`BRIDGE_LED_CONNECTED_GPIO`, `BRIDGE_LED_STREAMING_GPIO`,
`BRIDGE_LED_ACTIVE_LOW`); **-1 disables** either LED, and nothing else changes if
you leave them unwired.

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
that reason — and off 6–11, which are the flash.

**The blink follows the packets, not the phone's reported state.** It is driven
from the A2DP audio callback and goes dark after 300 ms with nothing arriving, so
a stream that stalls with the phone still calling itself "playing" shows as a
dark LED next to a lit one. That pair — connected but not streaming — is the
useful reading: it separates a pairing problem from a playback one before you
reach for a console. What it does *not* say is whether the hub is receiving any
of it; the LED is lit by packets leaving this chip, and `sbc_in` on the hub is
still the only thing that reports what arrived.

### The rest, in one paragraph each

**The DAC.** Every unit drives its own PCM5102A — the hub is a full speaker, not
a base station — and in every case the ESP32 is the I2S master. Pins are in the
two per-board READMEs. Two things that catch everyone: **`XSMT` must be high or
the DAC stays silently muted** — no error, no log line, no sound, and it is the
first thing to check — and **`SCK` goes to ground**, because the PCM5102A derives
its own internal clock and the firmware sets `mclk = I2S_GPIO_UNUSED` to match.

**Which channels a box plays** is `DANCEFLOOR_OUTPUT_CHANNELS` under *Dancefloor
audio output*: stereo (the default), left only, right only, or an `(L+R)/2`
downmix. Every unit still receives the same stereo stream — this only decides
what that box's speaker does with it — so a stereo pair is two
differently-configured images, and the `OUTPUT:` line at boot says which one a
board is running. The selected channel goes into *both* I2S slots rather than the
other being muted, because which slot an amp latches is a hardware strap and
muting the wrong one gives silence.

**The strip.** A **74AHCT125 level shifter is not optional**: WS2812 wants 5 V
logic, both boards drive 3.3 V, and it works on a bench often enough to be
misleading before failing outdoors in the cold. Power the shifter from 5 V and
tie `1OE` low. 330 Ω in series on the data line at the strip end, 1000 µF across
the strip's 5 V and ground at the first pixel, and a **separate 5 V supply for
the strip with its ground tied to the board's** — a full-white pixel draws enough
that a devkit regulator is not in the running. The 8-pixel default is the
bring-up figure precisely because it will run off the board.

It is driven as **SPI2 MOSI through the GPIO matrix**, which is why the data pin
need not be an SPI-labelled pad on either board. It is deliberately not RMT:
`led_strip` enables and disables an RMT channel per frame and `rmt_disable` races
the transmit-done interrupt, which wedges the strip after some minutes.

Under **Dancefloor LEDs** in `menuconfig`: `LED_COUNT` is the *total* pixels on
the one data line, so four chained 8-LED sticks is 32 and not 8; `LED_TYPE` gets
the wire order right, and setting it wrong swaps colours rather than failing, so
it looks like a bug in the patterns; and `LED_BRIGHTNESS` caps every pixel,
defaulting to 10%, which dark-adapted eyes outdoors cannot tell from full and
which reclaims most of the LED power budget.

**Two bench markers, off by default, and easy to confuse.** One measures audio
and one measures light. Nothing corrects on either, so enabling or disabling them
changes no behaviour — only whether the measurement exists.

- The **audio marker pair** (`DANCEFLOOR_ENABLE_MARKER`) measures how far apart
  two units' *audio* really is, at the speaker rather than inferred from clock
  estimates. It needs a wire between two boards, so a deployed floor cannot have
  it — and on the S3 hub it is finished anyway, because its monitor pin became
  the SBC link's CS.
- The **LED marker** (`DANCEFLOOR_ENABLE_LED_MARKER`) flashes once per second of
  master-clock time, on every unit at the same instant. Stand two boards side by
  side: if the LEDs blink together the chain agrees. It exists because it is the
  only instrument that covers what it covers — every other measurement here
  watches the *audio* path, and between playback and the pixels sit the analysis
  buffer, the FFT, the detector and the render, none of which anything counts.

  The LED goes to 3V3, not to ground, which is
  `DANCEFLOOR_LED_MARKER_ACTIVE_LOW` and defaults to `y`. The two must not be
  mixed across a floor: a unit with the setting wrong is lit *between* flashes
  and dark for the flash, which looks like a sync fault rather than a wiring
  mistake. The eye resolves maybe 10–20 ms, so this answers "are they together",
  not "by how much".

## Two ideas the whole thing rests on

**Schedule the future, never the present.** The hub does not say "play now"; it
says "play at master-time T", where T is `LEAD_US` ahead — 350 ms, in
`hub_s3/main/hub.h`. Each unit converts T to its own clock and waits. Network
jitter stops mattering as long as the packet arrives before T.

**One decision, drawn at the instant it names.** Analysis frames carry the
master-clock instant they belong to, so a unit renders them on the same timeline
it plays audio on — which is what makes a frame safe to send over the network at
all. A unit can get its frames either way, and `DANCEFLOOR_LED_SOURCE` is the
choice:

- `LED_SOURCE_LOCAL` runs the FFT and the detector over the audio this unit is
  about to play. Nothing about the lights is transmitted; agreement is inherited
  from the audio, which is already synchronised. This is the Kconfig default, and
  what the hub and the S3 satellite ship as.
- `LED_SOURCE_REMOTE` runs no FFT. Frames arrive carrying the four bands, and
  `df::RemoteDetect` derives this unit's own onset and boom from them — what
  travels is the detector's *input*, not the hub's answer. The classic satellite
  ships as this (`satellite/sdkconfig.defaults.esp32`), and that is a memory
  decision before it is anything else: local analysis wants a 32 kB contiguous
  stream and another task stack on top of the ring, which a classic ESP32 does
  not have, and the way it does not fit is silent — tasks fail to start and the
  unit still associates, still takes a lease and still looks healthy from the
  hub.

What guarantees agreement differs between the two: local units agree because the
detector is deterministic over identical input, which `test_align` and
`test_pattern_sync` pin; remote units agree because they are reading the same
source. `test_remote_detect` is the standing proof that the two paths reach the
same decision, which is what a mixed floor rests on.

The hub's side of it is `DANCEFLOOR_PUBLISH_FRAMES`, on by default. Turn it off
when every satellite is `LED_SOURCE_LOCAL` and recover that airtime.

## Build and flash

Needs ESP-IDF v6.

```sh
get_idf          # i.e. . ~/.espressif/tools/activate_idf_v6.0.1.sh

cd bt_bridge && idf.py -p /dev/ttyUSB0 flash monitor   # chip A
cd hub_s3    && idf.py -p /dev/ttyACM0 flash           # chip B, see below
tools/sat.py flash classic|s3                          # each satellite
```

**The hub takes two ports, not one.** Flashing goes over `ttyACM0` — the S3's own
USB peripheral wired straight to the connector, so it disappears on every reboot
— and the console does not: it is UART0 at 921600 on a separate wire. So
`flash monitor` on one port does not work here, and
[`hub_s3/README.md`](hub_s3/README.md) has the recipe and the reason.

**The satellite is one tree built two ways**, classic and S3, and the S3 form
needs `-DSDKCONFIG` on *every* invocation or the build directory is silently
reconfigured from the classic config. `tools/sat.py` assembles the line; see
[`satellite/README.md`](satellite/README.md).

Pins, LED count, brightness and pattern are under `idf.py menuconfig` →
**Dancefloor \***.

## Tests

Host-side, no hardware, no ESP-IDF:

```sh
cd components/dancefloor_sync/test && make check   # the clock estimator, the
                                                   # servo, the wire format
cd components/dancefloor_leds/test && make check   # FFT, onsets, patterns,
                                                   # block alignment, cross-unit
                                                   # determinism
```

`make check-hops` in the LED tests reruns the whole suite at each supported
analysis hop, because the alignment arithmetic is only exercised by running it.

Three of these exist because of specific field failures and are worth knowing
about. `test_align` pins that two units cut and label their analysis blocks
identically; `test_pattern_sync` pins that a pattern handed those blocks renders
identical pixels whatever its unit's join time, render count or drop history;
`test_remote_detect` pins that a unit given frames reaches the same decision as
one that analysed the audio itself. They carry deliberately broken cases that the
suite **requires** to fail — a test that only passes against correct code has not
been shown to detect anything.

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
shorter periodic lines are on `CONFIG_DANCEFLOOR_LOG_PERIOD_S`.

`TRACK DIVERGENCE` prints once per track, at a track boundary — the one instant
that recurs identically in every track, so it is comparable across tracks,
sessions and builds. A reading taken anywhere else depends on where in that cycle
you looked.

For anything longer than a bench check, `tools/soak/capture.py` reads every
unit's console at once and `analyse.py` reduces a session to the questions a long
run exists to answer, rebooting boards included. Past sessions are kept beside
them in `tools/soak/`.

## Documents

| | |
|---|---|
| [`docs/wifi.md`](docs/wifi.md) | WiFi from the band up: 802.11 fundamentals, the ESP32 driver and every configuration option, then this link read as a worked example. |
| [`docs/open-questions.md`](docs/open-questions.md) | Things the firmware does not yet know, each with the test that would settle it and the decision that follows from each outcome. |
| [`hub_s3/README.md`](hub_s3/README.md) | The S3 hub: pin map, the two-port flash recipe, what the board is configured to use, measured size. |
| [`satellite/README.md`](satellite/README.md) | The satellite: source layout, the two-target build, wiring, playback timing, reading its log. |
| [`components/led_strip_wrapper/README.md`](components/led_strip_wrapper/README.md) | The strip driver. |

Every component and app also carries a `Doxyfile` configured to *check* the
comments rather than render them, so a doc block that names something the code
does not have is a warning.

One pattern recurred at every level of this project: **every real fault was
invisible until something counted it**, and several were actively disguised as
something else. If you extend this, add the counter before you form the theory.

## State

Working on hardware: Bluetooth in, synchronised audio out of multiple speakers,
beat-reactive strips that agree with each other, and multi-hour soaks captured in
`tools/soak/`. What is left is power, enclosures and a field test.
`docs/open-questions.md` holds the questions that are open on purpose.
