# Dancefloor hub — ESP32-S3

The hub. There is one, and this is it.

Undecoded SBC in **over SPI** from the `bt_bridge` chip; owns the timeline the
whole system plays to; unicasts that SBC on to the satellites over WiFi; decodes
it for its own DAC and LED strip; publishes analysis frames. It runs a SoftAP,
answers satellite time probes, and is a full speaker in its own right.

Built for a **Seeed Studio XIAO ESP32-S3 Plus**.

> This directory used to be `hub_s3/`, an experimental port sitting beside a
> classic-ESP32 `hub/`. **`hub/` was retired on 2026-08-12 and deleted**, along
> with the UART link it was the last thing still speaking. Nothing here is a port
> any more, and there is no other hub to stay diffable against.

**Run on hardware**, two sessions on 2026-08-12 — see `../docs/satellite-audit.md`
§7.5 and §7.6 for what they showed, including four clean minutes with sync
converging inside ~1 ms and `tx-fail` at zero after association.

## Why the hub is the one role that can be an S3

`../docs/architecture.md` §2 fixes the chip choice for `bt_bridge` and only for
`bt_bridge`: A2DP is Bluetooth Classic, and of the ESP32 family only the original
part has it. The hub runs no Bluetooth — that is the entire point of the two-chip
split — so it is the one image in the project an S3 can carry.

What §2 *also* says is that the project chose classic ESP32 everywhere anyway,
for one part number and interchangeable spares. This board is a departure from
that decision, not an oversight in it. The satellite is still a classic ESP32.

## Pin map

The board breaks out eleven GPIOs and nothing else. Ten are spoken for.

| Function | XIAO pad | GPIO |
|---|---|---|
| SBC link SCK ← bridge | D7 (RX) | 44 |
| SBC link MOSI ← bridge | D5 | 6 |
| SBC link CS ← bridge | D4 | 5 |
| SBC link handshake → bridge | D2 | 3 |
| I2S BCK → DAC | D8 (SCK) | 7 |
| I2S LRCK → DAC | D9 (MISO) | 8 |
| I2S DATA → DAC | D10 (MOSI) | 9 |
| WS2812 data | D0 | 1 |
| LED sync marker | D1 | 2 |
| Console UART TX | D6 | 43 |

**Free: D3 (GPIO 4) only.**

The SBC link is **SPI3**. SPI2 belongs to the LED strip, which reserves a whole
bus to drive one WS2812 pin. The S3 has no IOMUX pins for SPI3, so everything
goes through the GPIO matrix — irrelevant at the clock this link runs at, and the
first thing to suspect if `crc` starts moving as the clock is raised.

The I2S trio sits on the three SPI-labelled pads so the DAC is one ribbon off the
end of the header.

**GPIO 22–25 do not exist on this part** — its pins are 0–21 and 26–48. 26–32 are
wired to the SPI flash and 33–37 to the octal PSRAM. That is why none of the
classic hub's pin numbers survived the move.

**The marker/monitor instrument is finished on this board.** It needs GPIO 4 *and*
GPIO 5, and GPIO 5 became the SBC link's CS. That was the cheapest of the four
pads that could have gone: the instrument is off by default, needs a wire between
two boards a deployed floor cannot have, and nothing corrects on what it
measures. `TRACK DIVERGENCE` over WiFi still covers every satellite, not just a
wired one.

The **74AHCT125 is still required**. The S3 drives 3.3 V exactly as the classic
ESP32 does, so nothing about the level-shifting argument in
`../docs/architecture.md` §12 changes.

**The LED sync marker is active low**, as on every unit on this floor: the LED
goes from 3V3 through a resistor to the pin, and the pin sinks it to light it.
`CONFIG_DANCEFLOOR_LED_MARKER_ACTIVE_LOW` carries that and defaults to `y`.

Do not expect the onboard LED. GPIO 21 is documented as the XIAO's and did not
light when pointed at, on a Sense or on a Plus; the reason was never found. Use
an external LED on D1 (GPIO 2), which is the default.

## Which XIAO ESP32-S3, and why it barely matters

This build ran on a **Sense** first and is on a **Plus** now. The move cost one
line — the flash size — and not a single pin, because the plain XIAO, the Sense
and the Plus are the same ESP32-S3R8 behind the same eleven-pad header with the
same GPIO numbers on it. Anything here that names a variant is describing the
board on the bench, not a dependency.

| | plain | Sense | Plus |
|---|---|---|---|
| Flash | 8 MB | 8 MB | **16 MB** |
| PSRAM | 8 MB octal | 8 MB octal | 8 MB octal |
| D0–D10 header | same | same | same |
| Extra GPIOs | — | — | ~9 on rear/SMD pads, D11–D15 = GPIO 38–42 confirmed |
| Camera / PDM mic / SD | — | yes | — |

**The Sense's sensors were never worth anything here.** A hub analyses the
synchronised stream and deliberately never listens to a room (§12), so the
microphone was dead weight and the camera more so. The Plus has no card reader,
which also retires a caveat: the Sense's SD slot is reportedly wired to the same
D8/D9/D10 pads the I2S output uses.

**What the Plus adds is not used**, but it is worth knowing: the rear pads mean a
pin taken by the SBC link is no longer gone for good. The marker/monitor
instrument lost GPIO 5 and could be given GPIO 38–42 instead, by anyone willing
to solder to a pad rather than a header pin.

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
bridge on this board. The console does not: it comes out of **UART0 TX on GPIO 43
(pad D6)** into a separate USB-UART adapter, whatever that enumerates as, at
921600 baud. Wire adapter RX ← GPIO 43, adapter GND ← board GND.

The console was moved off USB deliberately and it is not cosmetic. This part sets
`SOC_WIFI_PHY_NEEDS_USB_WORKAROUND`; IDF's mitigation is to power the USB PHY
down when WiFi starts, and a USB console forces `ESP_PHY_ENABLE_USB=y` to keep it
up, which IDF's own help says lowers WiFi performance. `sdkconfig.defaults` has
the full reasoning next to the settings.

**Hold BOOT and tap RESET before every flash.** The USB PHY goes down a few
hundred ms into boot, so the port vanishes once the app is running and esptool
cannot reset the board into download mode on its own.

**It must be a full `idf.py flash`, never `app-flash`.** The second-stage
bootloader is what raises the flash to quad mode during init, so a stale one
leaves the app in DIO. Relatedly: `idf.py flash` prints `--flash-mode dio` even
with QIO selected. That is correct — IDF stamps the image header DIO so the ROM
can read it. Do not "fix" it. If a board will not boot, hold BOOT and tap RESET
for ROM download mode, set DIO back, reflash; the ROM downloader is in silicon
and cannot be lost.

## What this board is configured to use

Four settings that are not IDF defaults and are each load-bearing. All are in
`sdkconfig.defaults` with the reasoning beside them.

| | value | why |
|---|---|---|
| CPU | **240 MHz** | with the cache below, took `analysis` 3900 → 1940 µs mean |
| Instruction cache | **32 kB** | the S3 offers 16 or 32 and **defaults to 16** |
| Flash | **QIO, 80 MHz** | ~40 MB/s against DIO/40's ~10; sets what every cache miss costs |
| PSRAM | **on, `CAPS_ALLOC` only** | see `../docs/hub-audit.md` §5 |

The instruction cache is the trap worth remembering on any future retarget:
`CONFIG_ESP32S3_INSTRUCTION_CACHE_32KB` has no counterpart on the classic ESP32,
so retargeting could not inherit a value and silently took the default. **A
symbol that exists only on the new target cannot be inherited from the old one,
and no diff of the tracked files will ever show it.**

Together those took `analysis` max from nearly double the 11.6 ms frame period to
comfortably under it, and carried downstream: `render` 880 → 470 µs, `wake`
overshoot +490 → +93 µs, `late` and `overrun` 0.

## Size, measured

From the link map, so these are static figures and not the runtime heap the
`HEALTH` line reports.

| | |
|---|---|
| Free RAM at link | 184,113 B DIRAM |
| `.bss` + `.data` | 77,352 B |
| Binary | 900,977 B |

The S3 merges IRAM and DRAM into one 341,760-byte DIRAM pool, so "free at link"
is the honest figure. At ~901 kB against a 1 MB single-app partition the binary
is 14% from the ceiling; this board has 16 MB of flash and could carry a much
larger app partition if that ever matters.

**Do not read the `HEALTH` line's heap figures as internal memory.**
`esp_get_free_heap_size()` reports the 8 MB PSRAM pool, which nothing on the
audio path can use — every `HEALTH` line has read ~8.4 MB free while internal SRAM
went to 1.5 kB. The `MEM:` line exists to replace it. `../docs/hub-audit.md` §5
has the baselines to compare against.

## Still open on this board

- **`CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0=y` is carried without evidence.**
  `../docs/architecture.md` §13 recorded pinning lwIP to CPU0 fixing the pattern
  task (max 23226 → 5254 µs) and *failing* to improve analysis (17400 → 17370 µs),
  concluding the latter is "mostly the FFT doing its own work". Those numbers were
  taken on an LX6 with the generic FFT; esp-dsp has an LX7 SIMD path this part
  uses automatically. Run it both ways before trusting it.
- **Nothing past one satellite has been measured.** The airtime model predicts
  ~30 with rate adaptation and A-MPDU on; nobody has put more than one on the
  floor with this config.
- **The satellite is no longer the control.** §13 leaves the satellite
  deliberately unpinned as the reference the lwIP measurement is against, and
  that comparison only means something while both units are the same silicon.
- **The 15.7 ms phase-wander wart lost its best evidence.** §16 says the hub's raw
  phase reading swings between consecutive reads and the satellite's does not, and
  that asymmetry was the strongest clue about it. With different silicon on each
  side, a new behaviour can no longer be told from the old bug.
- **A spare can no longer be promoted** to this role without a rebuild.
