# What the S3 hub has that the classic hub does not

Written 2026-08-05, after the session that took the S3 hub from "worse than the
classic" to measurably better. It exists because that session's conclusion was
counter-intuitive and easy to lose:

> The S3 was never worse. It was running at 160 MHz with half the instruction
> cache and 2-bit flash, because those are the defaults for a target that had no
> counterparts to inherit from the classic hub's config. Configured to use what
> it actually has, it does the job comfortably.

The list below is the porting backlog. It is deliberately split by whether the
classic hub *can* have the thing, because two of the biggest wins are
chip-specific and three are just settings nobody has copied across yet.

---

## 1. Portable — the classic hub could have these today

These are the ones worth acting on. Nothing here depends on the S3.

### 1.1 CPU at 240 MHz

| | classic hub | S3 hub |
|---|---|---|
| `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ` | **160** | **240** |

Both parts support 240 MHz. 160 is simply the IDF default and neither build ever
changed it. On the S3 this was half of a change that cut analysis cost in half —
see §3 for the numbers.

Costs power and heat, and shifts timing across every task. Worth a run with the
`vis: cost` line watched.

### 1.2 Flash in QIO mode, and at 80 MHz

| | classic hub | S3 hub |
|---|---|---|
| mode | `ESPTOOLPY_FLASHMODE_DIO` | **`ESPTOOLPY_FLASHMODE_QIO`** |
| frequency | 40 MHz | **80 MHz** |
| resulting bandwidth | ~10 MB/s | **~40 MB/s** |

DIO drives two data lines (IO0, IO1); QIO drives four, adding IO2 and IO3, which
on a plain part are the WP and HOLD pins and get repurposed as data. The
application is mapped from flash and pulled through the instruction cache a line
at a time, so this sets what every cache miss costs.

The classic hub is at **4× less flash bandwidth than the S3 hub** on both axes
combined. Both are portable in principle; whether the classic board tolerates
QIO and 80 MHz depends on its flash part and wiring.

**Two traps, both already documented in `hub_s3/sdkconfig.defaults`:**

- `idf.py flash` will print `--flash-mode dio` *even with QIO selected*. That is
  correct — IDF stamps the image header DIO so the ROM can read it, and the
  second-stage bootloader raises the flash to quad during init. Do not "fix" it.
- It must be a full `idf.py flash`, never `app-flash`. The bootloader performs
  the upgrade, so a stale one leaves the app in DIO.

Recovery if a board will not boot: hold BOOT, tap RESET for ROM download mode,
set DIO back, reflash. The ROM downloader is in silicon and cannot be lost.

### 1.3 SoftAP forced to HT20

Only in `hub_s3/main/streamer.c`, after `esp_wifi_set_config()`:

```c
const esp_err_t bw = esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW20);
```

Left alone the driver negotiates HT40 (`wifi:new:<11,2>`, stations joining
`40D`), which on channel 11 puts the secondary at channel 7 and occupies most of
the 2.4 GHz band. Nothing here can use the width — the traffic is ~135 small
datagrams a second per satellite, limited by transmit opportunities rather than
bits per symbol, and with the PHY rate pinned to 6 Mbps it could not use it even
in principle.

**This was the single biggest radio win of the session**, taking `tx-fail` from
1016 to roughly 400. The classic hub is presumably also negotiating HT40 and
paying the same interference cost for nothing.

Note `WIFI_BW20`, not `WIFI_BW_HT20` — IDF 6 removed the older spelling.

### 1.4 `RESYNC_US` at 150 ms

| | classic hub | S3 hub |
|---|---|---|
| `RESYNC_US` | 120000 | **150000** |

The measured swing of the hub timeline against real time reaches ±132 ms on
normal bursty SBC delivery, so a 120 ms threshold trips about seven times a
minute on nothing being wrong.

**Depends on the satellite, not the hub.** `LEAD + RESYNC` bounds how much a
satellite must buffer, and this is only affordable because the satellite ring
went 64 → 80 kB (371 → 464 ms) in the same session. The satellite firmware
already carries that, so this is safe to port — 200 + 150 = 350 against 464.

---

## 2. Not portable — S3 silicon or board

Listed so nobody spends time trying.

### 2.1 32 KB instruction cache

`CONFIG_ESP32S3_INSTRUCTION_CACHE_32KB` **does not exist on the classic ESP32**;
`components/esp_system/port/soc/esp32/` has no `Kconfig.cache` at all, because
the classic part's cache is fixed at 32 KB per core.

This is the trap that caused the whole investigation. The S3 can run 16 or 32 KB
and IDF defaults to **16**. Because the symbol has no classic-hub counterpart,
retargeting could not inherit a value — it silently took the default, and the S3
hub ran half the instruction cache of the board it was copied from.

> "The configuration started the same as the classic ESP32" is true, and is
> exactly the reason. A symbol that exists only on the new target cannot be
> inherited from the old one, and no diff of the tracked files will ever show it.

Same shape as the LED-pattern note in `hub_s3/sdkconfig.defaults`, which cost a
false sync-fault reading. **Worth checking for other target-only symbols on any
future retarget.**

### 2.2 LX7 with SIMD

The S3's esp-dsp uses the SIMD FFT path; the classic hub's LX6 uses the generic
one. Automatic, no configuration. This was the one S3 advantage the build was
already collecting before this session.

### 2.3 512 kB internal SRAM (vs 320 kB)

Not currently exploited by anything — the extra goes to the heap. Relevant if the
hub's local ring or buffers ever need to grow; the S3 has room the classic does
not.

### 2.4 Octal PSRAM, 8 MB

**Deliberately off, and should stay off.** The ring is touched from the playback
path and PSRAM cache-miss jitter is the one thing that loop cannot absorb. Also
the wrong board: the ring that bounds `LEAD + RESYNC` belongs to the *satellite*,
which is a classic ESP32 with no PSRAM. A 500 ms lead would need ~107 kB of
satellite ring against a 106 kB largest free block — it would fail to allocate on
the board that has to hold it. **Memory on the hub buys the lead nothing.**

### 2.5 8 MB flash (vs 4 MB)

Board difference. No use for it yet.

### 2.6 UART console instead of USB Serial/JTAG — S3-only *problem*, now solved

| | classic hub | S3 hub |
|---|---|---|
| console | `ESP_CONSOLE_UART_DEFAULT`, via a bridge chip | `ESP_CONSOLE_UART_CUSTOM`, GPIO 43 |
| `ESP_PHY_ENABLE_USB` | n/a | **n** |

Listed here because the classic hub needs nothing: this was an S3-only problem
created by the S3-only console, and the classic hub's bridge chip meant its
radio never saw USB at all.

`SOC_WIFI_PHY_NEEDS_USB_WORKAROUND` is set on the S3. IDF's fix is to disable
the USB PHY when WiFi initialises, for best WiFi performance; `ESP_PHY_ENABLE_USB`
overrides that fix, and it defaults to `y` whenever the console is USB
Serial/JTAG. So the original build ran this unit's radio with IDF's own
mitigation permanently off — on the one board whose measured problem was the
transmit path.

Now: console on **UART0 TX = GPIO 43** (pad D6, silkscreened TX, free), RX = -1
since GPIO 44 is the SBC link and a log needs no input, 921600 baud, and
`ESP_PHY_ENABLE_USB=n`.

Two settings are load-bearing and easy to lose: `ESP_CONSOLE_SECONDARY_NONE`,
because the secondary-console choice defaults back to USB Serial/JTAG as soon as
the primary is not USB, and `ESP_PHY_ENABLE_USB` itself, whose default is
`y if IDF_TARGET_ESP32S3` regardless of the console.

**Bench cost:** the USB PHY is down from the moment WiFi starts, so the port
vanishes a few hundred ms into boot and esptool cannot reset the board itself.
Hold BOOT and tap RESET before every flash. Wiring is adapter RX ← GPIO 43,
adapter GND ← board GND.

**Unmeasured.** Nothing yet says how much WiFi performance this returns. It is
one run to find out, and now possible to run at all.

---|---|---|
| console | `ESP_CONSOLE_UART_DEFAULT`, via a bridge chip | `ESP_CONSOLE_USB_SERIAL_JTAG` |

A **liability**, not a feature. `CONFIG_SOC_WIFI_PHY_NEEDS_USB_WORKAROUND=y` is
set on this part — the USB peripheral and the WiFi PHY interact, and on this
board the console *is* that peripheral. IDF applies a workaround, so it should be
fine, but every measurement in the session was taken with a USB monitor attached.

**Untested and cheap:** run the hub with the monitor detached for a few minutes,
then reattach and compare `tx-fail` and satellite gaps. If they differ, the
confound was on for every reading taken so far.

---

## 3. What the portable items bought, measured

All on the S3 hub, same firmware, same satellite, same room.

| config | `analysis` mean | `analysis` max |
|---|---|---|
| 16 kB cache, 160 MHz, DIO | 3900 µs | 21000 µs |
| 32 kB cache, 160 MHz, DIO | 3040 µs | 14000 µs |
| 32 kB cache, **240 MHz, QIO** | **1940 µs** | **9100 µs** |

Against an 11.6 ms frame period, the max went from nearly double it to
comfortably under. It carried downstream: `render` 880 → 470 µs, `wake`
overshoot +490 → +93 µs, `pat` max 4000 → 2000 µs, `late` and `overrun` 0.

`tx-fail` went to **zero in every window**, with a satellite joining a playing
stream — the exact case that measured 1016 at the start of the session and 420
midway. Hub heap minimum rose from 34616 to 52880 with the *same* 32 TX buffers,
because the driver was draining them instead of backing up.

---

## 4. Settled elsewhere — do not re-open on either hub

Recorded here so the porting work does not re-litigate them.

| question | answer | evidence |
|---|---|---|
| PHY rate | **6 Mbps pinned** | 6: 1.2 gaps/min · 12: ~115 · 24: 23% loss · adaptive: 3.5–14 |
| TX AMPDU | **off** | forced — `set_fix_rate` refuses to run with it enabled |
| TX buffers | **32** | 64 gave 329 vs 32's 420 — noise, and the heap wants the 32 |
| PSRAM | **off** | §2.4 |

---

## 5. Socket receive queue — now also portable

`CONFIG_LWIP_UDP_RECVMBOX_SIZE`:

| | value |
|---|---|
| satellite | 32 |
| **S3 hub** | **32** |
| classic hub | 6 (IDF default) |

Raised on the satellite first, where the case was obvious: one socket taking
~136 datagrams a second through a six-deep queue is 44 ms of headroom, and lwIP
discards past that silently.

The hub's case is quieter. Its socket carries only time probes and splice
reports — four probes a second per satellite — so at one satellite, six deep is
1.5 s of slack. That is why this was nearly left alone.

It does not stay that way. The rate scales with the floor: at `MAX_CLIENTS` = 8
it is 32 datagrams a second and under 200 ms of headroom, from eight units
free-running on independent 250 ms timers whose probes will sometimes bunch.
`probe_task` serves this socket at priority 6, below playback and the sync
monitor.

**The failure mode is what makes it worth doing.** A probe lost in this queue is
indistinguishable from a satellite that stopped probing, and `CLIENT_TIMEOUT_US`
is 2 s — eight probes. Lose enough and the hub forgets a satellite that is
sitting right there, drops it from the send list, and stops sending it audio
until the next probe gets through. That is a silent speaker, reported as
`sta-timeout`, and no counter anywhere distinguishes it from a real departure.

Port it to the classic hub with the §1 items.
