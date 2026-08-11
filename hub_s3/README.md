# Dancefloor hub, ESP32-S3 port — experimental

The same hub firmware as [`../hub`](../hub), built for a **Seeed Studio XIAO
ESP32-S3 Plus**. Same role: SBC in over UART from the `bt_bridge` chip, owns
the timeline the whole system plays to, unicasts that SBC to the satellites,
decodes it for its own DAC and LED strip, publishes analysis frames.

**Nothing here has been run on hardware.** It compiles and links; that is the
whole of the evidence so far.

## Why the hub is the one role that can move

`../docs/architecture.md` §2 fixes the chip choice for `bt_bridge` and only for
`bt_bridge`: A2DP is Bluetooth Classic, and of the ESP32 family only the
original part has it. The hub runs no Bluetooth — that is the entire point of
the split — so it is the one image in the project an S3 can carry.

What §2 *also* says is that the project chose classic ESP32 everywhere anyway,
for one part number and interchangeable spares. This directory is a departure
from that decision, not an oversight in it.

## What is actually different from `../hub`

Three files, and `diff -r ../hub .` is meant to stay short enough to read.
**Every `.c`, every `.h` and both `main/CMakeLists.txt` are byte-identical**,
and should stay that way. What differs is configuration and prose.

| File | Change |
|---|---|
| `CMakeLists.txt` | `project(hub_s3)` |
| `main/Kconfig.projbuild` | pin defaults: UART RX 23 → 44, monitor 21 → 5 |
| `sdkconfig.defaults` | target, flash size, USB console, and the whole pin map |

The one *source* change the port needed went into `../hub` instead of here, so
there is no divergence to maintain: the UART RX pin was a literal `23` in
`sbc_in.c` and is `DANCEFLOOR_SBC_UART_RX_PIN` in both builds now, defaulting to
23 there and 44 here. The transmit half is gone entirely — the bridge talks and
this chip listens, so `UART_PIN_NO_CHANGE` replaces a GPIO 22 that was being
driven as an idle-high TX line with nothing connected to it.

That was not cosmetic. **GPIO 22–25 do not exist on the ESP32-S3** — its pins
are 0–21 and 26–48 — so both hardcoded numbers were rejected by
`uart_set_pin()`, inside an `ESP_ERROR_CHECK`, at boot rather than at compile
time. 26–32 exist but are wired to the SPI flash and 33–37 to the octal PSRAM,
so of the classic hub's seven pins only the marker survives the move.

## Pin map

The board breaks out eleven GPIOs and nothing else, and the hub wants seven of
them, so this is close to the only arrangement rather than one among many.

| Function | XIAO pad | GPIO | classic hub |
|---|---|---|---|
| UART RX ← bridge | D7 (RX) | 44 | 23 |
| I2S BCK → DAC | D8 (SCK) | 7 | 26 |
| I2S LRCK → DAC | D9 (MISO) | 8 | 27 |
| I2S DATA → DAC | D10 (MOSI) | 9 | 25 |
| WS2812 data | D0 | 1 | 18 |
| Sync marker out | D3 | 4 | 4 |
| Sync monitor in | D4 | 5 | 21 |

Free afterwards: D1, D2, D5. (D6 = GPIO 43 is the console UART TX.)

The I2S trio sits on the three SPI-labelled pads so the DAC is one ribbon off
the end of the header. The LED strip does not need those pads — the SPI backend
reaches its pin through the GPIO matrix, which is fine at the ~2.4 MHz the
WS2812 encoding runs at.

The **74AHCT125 is still required**. The S3 drives 3.3 V exactly as the classic
ESP32 does, so nothing about the level-shifting argument in
`../docs/architecture.md` §12 changes.

**The LED sync marker is active low here**, as it now is on every unit on this
floor: the LED goes from 3V3 through a resistor to the pin, and the pin sinks it
to light it. `CONFIG_DANCEFLOOR_LED_MARKER_ACTIVE_LOW` carries that and defaults
to `y`.

This paragraph used to say the opposite — that GPIO 21's active-low onboard LED
would read inverted "which the driver does not know", left unfixed "because a
polarity flag in shared code wants a better reason than one board". Three boards
rewired to VCC is that better reason, and the flag exists. What has *not* changed
is the advice about GPIO 21 itself: it did not light when pointed at, on a Sense
or on the Plus, and the reason was never found. Use an external LED on `D1`
(GPIO 2), the default.

## Which XIAO ESP32-S3, and why it barely matters

This build ran on a **Sense** first and is on a **Plus** now. The move cost one
line — the flash size — and not a single pin, because the plain XIAO, the Sense
and the Plus are the same ESP32-S3R8 behind the same eleven-pad header with the
same GPIO numbers on it. Anything in this directory that names a variant is
describing the board on the bench, not a dependency.

| | plain | Sense | Plus |
|---|---|---|---|
| Flash | 8 MB | 8 MB | **16 MB** |
| PSRAM | 8 MB octal | 8 MB octal | 8 MB octal |
| D0–D10 header | same | same | same |
| Extra GPIOs | — | — | ~9 on rear/SMD pads, D11–D15 = GPIO 38–42 confirmed |
| Camera / PDM mic / SD | — | yes | — |

**The Sense's sensors were never worth anything here.** A hub analyses the
synchronised stream and deliberately never listens to a room (§12), so the
microphone was dead weight; the camera more so. The one thing they did cost was
a caveat — the SD slot is reportedly wired to the same `D8`/`D9`/`D10` SPI pads
the I2S output uses, with GPIO 21 for CS — and the Plus has no card reader, so
that caveat is now gone rather than merely unverified.

**What the Plus adds is not used either**, but it is worth knowing about: the
rear pads mean a pin taken by the SBC link is no longer gone for good. The
marker/monitor instrument lost GPIO 5 to it and could be given GPIO 38–42
instead, by anyone willing to solder to a pad rather than a header pin.

The one piece of the board that *is* worth something on any of the three: the
u.FL connector and external antenna, on the unit that is the SoftAP in a field.

## Build and flash

```sh
. ~/.espressif/v6.0.1/esp-idf/export.sh
idf.py set-target esp32s3
idf.py -p /dev/ttyACM0 flash          # hold BOOT, tap RESET first
idf.py -p /dev/ttyUSB0 -b 921600 monitor
```

**Two ports, and they are different things.** Flashing goes over `ttyACM0`, the
S3's own USB peripheral wired straight to the connector — there is no USB-UART
bridge on this board. The console does not: it comes out of **UART0 TX on
GPIO 43 (pad D6)** into a separate USB-UART adapter, whatever that enumerates as,
at 921600 baud. Wire adapter RX ← GPIO 43, adapter GND ← board GND.

The console was moved off USB deliberately and it is not cosmetic. This part sets
`SOC_WIFI_PHY_NEEDS_USB_WORKAROUND`; IDF's mitigation is to power the USB PHY
down when WiFi starts, and a USB console forces `ESP_PHY_ENABLE_USB=y` to keep it
up, which IDF's own help says lowers WiFi performance. `sdkconfig.defaults` has
the full reasoning next to the settings.

**Hold BOOT and tap RESET before every flash.** Since the USB PHY now goes down a
few hundred ms into boot, the port vanishes once the app is running and esptool
cannot reset the board into download mode on its own.

## Size, measured

Both from the link maps, so these are static figures and not the runtime heap
the `HEALTH` line reports.

| | classic hub | this |
|---|---|---|
| Free RAM at link | 104,457 B DRAM | **184,113 B DIRAM** |
| `.bss` + `.data` | 76,279 B | 77,352 B |
| Binary | 903,720 B | 900,977 B |

The S3 merges IRAM and DRAM into one 341,760-byte DIRAM pool, so "free at link"
is the honest comparison and it is **+80 kB, a 76% increase**. Static data is
within a kilobyte of the classic build, as it should be — same code.

The binary is *not* smaller in any meaningful way, and at ~901 kB against a
1 MB single-app partition it is 14% from the ceiling. That is true of the
classic hub too (903 kB) and is not something this port introduced, but this
board has 16 MB of flash and could carry a much larger app partition if it ever
matters.

## The measurement this port exists to make

`../docs/architecture.md` §13 records that pinning lwIP to CPU0 fixed the
pattern task — max 23226 → 5254 µs — and **failed** to improve analysis
(17400 → 17370 µs), concluding that one is "mostly the FFT doing its own work."
esp-dsp has an LX7 SIMD path the classic part cannot use, so that is the number
to read first.

`CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0=y` is carried over here, and it is carried
over **without evidence**. The reasoning still holds — priority 18, core locking
off, ~30× a satellite's send load — but the numbers behind it were taken on an
LX6 with the generic FFT. Run it both ways before trusting it.

## What this port invalidates

Not bugs, but things that stop being true, and every one of them is load-bearing
somewhere in the docs.

- **The satellite is no longer the control.** §13 leaves the satellite
  deliberately unpinned as the reference the lwIP measurement is against. That
  comparison only means something while both units are the same silicon.
- **The 15.7 ms phase wander wart has no cause.** §16 says the hub's raw phase
  reading swings between consecutive reads and the satellite's does not, and
  that asymmetry is the best evidence anyone has about it. Change the chip and
  you can no longer tell a new behaviour from the old bug.
- **"Does the other unit have this bug too" gets harder**, which is the exact
  question §18's third postscript says produced most of the wasted effort in the
  drift work.
- **A spare can no longer be promoted** to this role without a rebuild.

## If this works out

The endgame is not two copies of a 92 kB `streamer.c`. Once the S3 build is
worth keeping, fold it back into `../hub` — the UART pin becomes Kconfig there
too, and everything in this directory's `sdkconfig.defaults` becomes
`hub/sdkconfig.defaults.esp32s3`, which ESP-IDF applies automatically on top of
the base defaults when the target is set. That leaves one source tree and two
configs.

Until then, keep the diff to those four files. `../docs/architecture.md`'s
postscript on the drift work is specifically about a fix landing in one unit and
not the other; two hub source trees is the machinery for making that happen
weekly.
