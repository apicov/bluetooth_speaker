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

> **The classic `hub/` was retired on 2026-08-12 and deleted from the tree.**
> `hub_s3/` is the only hub. That resolves this document's backlog by removing
> the thing it was a backlog *for*, so read it accordingly:
>
> - **§6 and §7 are closed, not done.** They described work the classic hub owed
>   — the sync features it never received, and the SPI link it never learned to
>   speak. Nothing owes them now. §7.2's instruction to delete `sbc_link_hdr_t`
>   "the moment the port lands" was carried out on retirement instead; the UART
>   declarations are gone from `sbc_link.h`.
> - **§1 is still live, and its audience changed.** Those four are portable wins
>   that were never taken up on a classic ESP32 — and the *satellite* is still a
>   classic ESP32. 240 MHz, QIO flash at 80 MHz, HT20 and the `RESYNC_US` value
>   are unclaimed there. See [`satellite-audit.md`](satellite-audit.md) §4.
> - **§2–§5 are unaffected**: they describe the S3 hub itself.

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

**On, for two things by name: the tensor arena and the WiFi/lwIP buffers.** This
section said "deliberately off, and should stay off" for most of the port's
life, then "for exactly one allocation" once the arena arrived. The second is no
longer true either — see the exemption below — but the reasoning that produced
both is intact, and what it refuses is still refused.

The buffer is the TFLM tensor arena: large, touched only by a low-priority task
on the other core, and useless to shrink. `CONFIG_SPIRAM_USE_CAPS_ALLOC` is the
line that makes turning PSRAM on safe rather than merely possible — ordinary
`malloc()` never returns PSRAM, so the ring, the DMA buffers, the frame queue and
every stack stay in internal SRAM with the timing they were
measured with. PSRAM is reachable only by asking for it by name: in this tree
that is `heap_caps_aligned_alloc(..., MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)` in
`ml_arena.c`, plus `wifi_malloc()` once the symbol below is set. The
alternative, `SPIRAM_USE_MALLOC`, would hand PSRAM to anything above a threshold
— that is the "cache-miss jitter in every task" the old note warned about, and
CAPS_ALLOC is how the warning is respected rather than overruled.

**One exemption has since been added: WiFi and lwIP.** That sentence used to
include "the WiFi buffers" in the list above, and it stopped being true when
`CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP` went on. The reason is a measurement that
could not be taken before the `MEM:` line existed: **the registered internal heap
on this part is ~268 kB, not 512 kB** — ~102 kB is DRAM shadowed by IRAM code,
~96 kB is static `.data`/`.bss`, ~33 kB is ROM-reserved — and ~246 kB of that is
live, leaving ~22 kB for WiFi's 32 dynamic TX and 32 dynamic RX buffers to spike
into. Their ceiling is ~105 kB. The internal watermark reached **1520 bytes**,
and a 1700-byte dynamic TX buffer failed to allocate. `wifi_malloc()` now prefers
PSRAM with internal fallback retained, so nothing that succeeded before can fail.

The refusals below are unchanged by that, and the distinction is the point: WiFi
is the pool that spikes *and* the one furthest from the playback loop. The ring
is neither.

Still refused, and the refusals are the durable part of this section:

- **Not for the ring.** It is touched from the playback path, and PSRAM
  cache-miss jitter is the one thing that loop cannot absorb.
- **Not for the lead.** The ring that bounds `LEAD + RESYNC` belongs to the
  *satellite*, a classic ESP32 with no PSRAM at all. A 500 ms lead would need
  ~107 kB of satellite ring against a 106 kB largest free block — it would fail
  to allocate on the board that has to hold it. **Memory on the hub buys the lead
  nothing**, which is the whole answer to "the S3 has plenty of PSRAM, can we not
  have a 500 ms lead". The larger ring that argument wanted was taken later in
  the satellite's own internal SRAM, 64 → 80 kB.

Unmeasured, and worth measuring: enabling the PSRAM controller changes cache
behaviour even when nothing allocates from it. Read the vis `cost:` line and the
HEALTH heap figures against the recorded baseline — analysis 3900/21000 µs
mean/max, hub min heap ~68 kB, 0 audio gaps/min over 18 min.

**`hub min heap ~68 kB` in that baseline cannot be compared against this board,**
and the trap is worth naming because it hid a real fault for as long as PSRAM has
been on. That figure is a *classic-hub* number. The classic ESP32 has one pool, so
`esp_get_free_heap_size()` was the internal figure; on this part the same call
reports the 8 MB PSRAM pool, which nothing on the audio path can use. The field
did not change meaning loudly — it changed pools silently, and every `HEALTH` line
since has read ~8.4 MB free while internal SRAM went to 1.5 kB. The `MEM:` line
exists to replace it, and the first real internal baseline on this part is:

```
MEM: internal 22056 free (min 5808, window 20676, largest 11776) | total 8407688
MEM: internal 21912 free (min 1520, window 20512, largest 11776) | total 8407544
```

Compare `MEM:` against that, not `HEALTH`'s heap terms.

### 2.5 16 MB flash (vs 4 MB)

Board difference. No use for it yet. This was 8 MB while the bench board was a
XIAO ESP32-S3 Sense and is 16 on the Plus that replaced it; the app is ~901 kB
against a 1 MB single-app partition either way, so the headroom has never been
the constraint.

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

## 4. Settled elsewhere — do not re-open without a new reason

Recorded here so the porting work does not re-litigate them. Two of these were
re-opened, on a reason the original measurements did not have; the table gives
what is in `hub_s3/sdkconfig.defaults` now, and the row below it says what it
used to be and why it moved.

| question | answer | evidence |
|---|---|---|
| PHY rate | **unpinned (0, adaptive)** | paired with AMPDU below — see *the flip* |
| TX AMPDU | **on**, `TX_BA_WIN 6` | ditto; the two are forced to move together |
| TX buffers | **32** | 64 gave 329 vs 32's 420 — noise, and the heap wants the 32 |
| PSRAM | **on, CAPS_ALLOC only** | §2.4 — one named allocation, the TFLM arena; `malloc()` never returns it |

**The flip, and why the old answer was not wrong.** The rate was pinned at
6 Mbps and aggregation off for most of this project, on measurements that still
stand: 6 Mbps gave 1.2 gaps/min, 12 gave ~115, 24 measured 23% loss, and
adaptive-plus-AMPDU gave 3.5–14. 6 Mbps is the most robust OFDM rate there is,
and every step away from it cost more in steady-state margin than it returned.
The two settings are welded together — `esp_wifi_internal_set_fix_rate` refuses
to run with aggregation enabled — so they could only ever move as a pair.

What changed is the *goal*, not the physics. Unicast fan-out makes downlink
airtime scale with satellite count, and at 6 Mbps the duty-cycle wall arrives
around 6–8 satellites. A-MPDU amortises the ~350 µs of per-frame
DIFS/backoff/preamble across a burst and lets rate adaptation reach MCS7
(~65 Mbps), which is roughly 10× the airtime headroom — that is what buys
satellite count, and pinning 6 Mbps cannot.

The 3.5–14 gaps/min that condemned adaptive+AMPDU was measured with a 16 kB
I-cache and the USB-PHY mitigation off. **Both have since flipped** (§2.1, §2.6),
so that number describes hardware that no longer exists. Re-measured on the
current configuration: **0 audio gaps/min over 18 min at one satellite, tx-fail
0, hub min heap ~68 kB** — clears the ~2.5 gaps/min bar the old number failed.
The satellite needs no rebuild; AMPDU RX is already on with `RX_BA_WIN 6`.

**Still unmeasured:** anything past one satellite. The airtime model predicts
~30, and nobody has put more than one on the floor with this config. The
satellite also logged 17 brief STA re-associations that the play buffer absorbed
without an audible gap, which is unexplained rather than benign.

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

---

## 6. Sync work landed on `hub_s3/` only — the current porting debt

Everything above is configuration. This section is **code**, and it is a
different kind of debt: the two `streamer.c` files were near-identical and are
now diverging on purpose, because the bench has an S3 hub on it and the classic
hub is not being run. Nothing here is S3-specific — the classic hub has the same
bugs — so all of it is portable, and none of it has been tested on that board.

The work is staged, each stage evaluated on `TRACK DIVERGENCE (wifi)` over a
session before the next lands. Stage 0 is behaviour-neutral instrumentation and
is what is in the tree now; the rest is written up in the plan that produced it.

### 6.1 What has landed (Stage 0 — measurement only)

Nothing here changes what either unit does. Every item is a counter or a log
line, and the `apart` figure must be unchanged in distribution from the build
before it — if it moved, the stage was not neutral.

| change | where | notes for the classic hub |
|---|---|---|
| `sync_phase_hist_t`, `sync_phase_reset/push/median` | `components/dancefloor_sync/` | **shared already** — the classic hub picks it up for free, and the host tests come with it |
| `splice_msg_t.applied_med_us` | `sync_proto.h` | **a wire change.** A classic hub against a satellite on this build reads a `splice_msg_t` four bytes shorter than it expects, and its `n >= sizeof(splice_msg_t)` guard makes it print *no* divergence line at all. Silent, and it looks like a satellite that stopped reporting. Port before pairing them |
| push readings into the hist; reset at play-task start and after a splice | `hub_s3/main/streamer.c`, `satellite/main/main.c` | mechanical |
| median shadow computed at the boundary; `TRACK DIVERGENCE` gains a `median:` clause | both | mechanical |
| `s_retune_done_at`, `s_retune_tail_left`; `RETUNE COST` gains the crossing age; three `RETUNE TAIL` lines | both | mechanical |
| `n_phase_drop` | both | the hub counts it only when a timeline start was *not* the reason for skipping |
| `n_short_reads` / `n_short_frames` | both | |
| `REFILL` armed at every playback **start**, not only after a retune; tagged `start` / `retune` | both | **the classic hub has no `REFILL` instrument at all** — it is a satellite one that was never ported, and the hub is the unit with the larger startup offset. See §6.2 |
| rejoin latency, first-anchor clock source and probe age, `n_tsf_wide` | satellite only | nothing to port |
| stale comments corrected | both | `LOCAL_RING_BYTES` said "32 kB is 181 ms" against a 64 kB define; the restart-flag comment claimed the timeline step is capped at `RESYNC_US` and inside `MAX_SPLICE_MS`, which stopped being true when `RESYNC_US` became 150000; the satellite's TSF block still called TSF "measurement only" long after `clock_offset()` began anchoring on it |

### 6.2 What is queued, and why the classic hub wants it

These are the actual fixes. Each names a defect the classic hub shares unless
said otherwise.

**The DMA prefill at playback start is unguarded, on both units.** Found from a
field observation that reconnecting a satellite gives perfect sync while a cold
start sometimes does not — the two cases differ in that a reconnect restarts one
unit against a converged reference, and a cold start restarts both at once.
`i2s_channel_write()` does not block while descriptors are free, so on an empty
channel the first writes return at memory speed: `samples_played` advances by
the whole DMA depth (6 × `AUDIO_FRAMES` = 34.8 ms at 44.1 kHz) against a
`wrote_at` that has barely moved, and every phase reading dated inside that
window is measured against a reference the DAC is not pacing. This is the same
mechanism the `retuning` park exists to prevent, where `clock-sync.md` records it
costing +42, +43 and +50 ms; nothing guards it at startup. It is very likely the
"-42 ms (hub), -26 ms (satellite)" startup phase in §8 of that document — a 16 ms
cross-unit difference taking ~45 s to walk off — which it notes "nothing accounts
for at anchor time" and calls compensating "the obvious next improvement". The
fix is to withhold phase readings until the first blocking write, exactly as the
retune path does. **Applies to the classic hub identically, and the classic hub
additionally needs the `REFILL` instrument itself, which it has never had.**

**The splice runs on one raw phase reading.** Both units snap their playback
position at a track boundary using the newest reading, while the servo has used
a 4-sample average since it was measured triggering on noise. The hub's own
comment records two reads of `s_phase_err_us` a millisecond apart differing by
15.7 ms, and §16 of `architecture.md` lists the wander as a wart with the cause
unfound. So at every boundary the hub jumps to a position several ms wrong in a
direction nothing predicts while the satellite — a third the scatter, a fraction
the load — lands closer, and the two splice to *different* places. That is why a
track change sometimes improves cross-unit sync and sometimes degrades it. The
fix is `sync_phase_median()`; the shadow in §6.1 is what decides whether it
ships. **Applies to the classic hub identically, and must land on both units at
once** — one-sided would guarantee they splice to different places, which is the
bug itself.

**Only one phase reading is withheld after a retune.** Crossings arrive ~20 ms
apart and the transient is measured landing 1–22 ms after the retune, inside the
refill every time, so a one-shot flag hands most of the disturbance to the servo
as position error and each retune injects what the next one corrects. Replace
with a `RETUNE_SETTLE_US` window sized from the `RETUNE TAIL` ages. **Same
one-shot flag on the classic hub.**

**A hard timeline jump is unannounced.** At `|err| > RESYNC_HARD_US` the hub
jumps `next_play_at` and sets no `restart` flag, so satellites take a step of up
to the threshold with no splice hint, below `PHASE_INSANE_US` and therefore with
no re-anchor either — the servo walks it off at 2.27 ms/s. Flag a boundary a few
packets after the jump, once the median window is clear of the discontinuity.
**The classic hub has this too, and worse in one respect and better in another:
its `RESYNC_HARD_US` is still 1 s, so it fires more rarely and costs more when it
does.**

**The underrun-recovery restart flag can be lost.** `s_underrun_recover` is
consumed into a local before the source-steadiness gate, which can then return
before the flag is ever written to a packet — so no satellite learns the timeline
restarted, reintroducing the exact bug the comment above it was written to fix.
**S3-only, because the gate is S3-only.** The classic hub should still take the
restructuring so the two files stay diffable and the next person is not misled.

**Client aging only runs inside the audio send loop**, so nothing ages out while
audio is stopped. Move it to `ring_monitor_task`'s 5 s tick. **Applies to the
classic hub.**

**A short ring read biases `samples_played`.** The pad is played but was never in
the ring, while `samples_played` advances by a whole chunk regardless, so every
later phase point is permanently displaced — the same shape as the "silence
inserted for a lost packet was not counted in `samples_in`" bug that put a unit
20 ms out per loss. Whether it fires at all is what `n_short_reads` is for.
**Applies to the classic hub.**

### 6.3 The rule this section exists to serve

Same one as everywhere else here: when a fix lands in one unit, the question is
not whether it works but whether the other unit has the same bug. The answer for
every item above is yes. What is missing is not the analysis but a classic hub on
a bench, and the cost of waiting is that the two files drift — so this list is
the substitute for the diff that would otherwise have shown it.

---

## 7. The SPI link — `hub/` can no longer receive anything

Everything in §6 is a defect the classic hub shares and could go on running
without. This one is different in kind: it is not a bug it has, it is a wire it
no longer speaks.

The bridge forwards SBC over **SPI** now, and there is only one bridge. The
classic hub still listens on a UART at 500 kbaud for a byte stream nobody is
sending. It builds, it boots, its `sbc_in` reports `pkts 0`, and it plays
nothing. Pairing it with the current bridge produces silence, not a degraded
link, and no counter anywhere says why — `max gap` grows, everything else stays
at zero, which reads exactly like a phone that stopped.

### 7.1 Why the link moved

The UART was 50 kB/s hard against a measured ~42 kB/s of payload — 84% of the
wire — and 500000 baud was where breadboard jumpers began producing bad sync
bytes, not where the protocol stopped. `max_bitpool` was capped at 53 for that
reason and could not rise on the UART. SPI cleared the wire's capacity, the
payload ceilings are up to 2048, and `max_bitpool` advertises 250 again — but
the phone negotiates 53 back regardless, so on the current handset the raise is
headroom, not a realised gain. The 2060-byte frame the 2048 ceiling implies is
clean at 5 MHz (it fails at 10 MHz on these jumpers; the smaller 1036-byte frame
was the one clean at 10 MHz). See [`sbc-link.md`](sbc-link.md).

### 7.2 What the port needs

`hub/main/sbc_in.c` was **byte-identical** to `hub_s3/main/sbc_in.c` until this
change. That file is now the whole diff — the port is to take it, and change
only the pins.

| item | detail |
|---|---|
| bus | **VSPI / `SPI3_HOST`**. `SPI2_HOST` is the LED strip on this hub too |
| pins | four, and the classic part has them to spare, unlike the XIAO. SCK can keep GPIO 23, the pin the UART arrived on. Avoid 16/17 (PSRAM die on WROVER), 5 and 12 (strapping), and 34–39 (input only, so no handshake output there) |
| handshake | an **output** on the hub, input on the bridge. Not optional — `spi_slave` loses any transfer clocked with nothing queued |
| header | `spi_link_hdr_t`, 12 bytes, no sync words. `sbc_link.h` kept the old `sbc_link_hdr_t` purely so the classic firmware still compiled; **it was deleted on retirement, 2026-08-12**, along with `SBC_LINK_BAUD`, the sync bytes and `sbc_link_checksum()` |
| checksum | CRC-16 via `sbc_link_crc16()`, not the XOR byte |
| framing | fixed 2060-byte transactions (12-byte header + 2048 payload, sized for the codec's bitpool 250), two DMA buffers queued alternately. **5 MHz**, not 10: the 2060-byte frame fails on breadboard jumpers at 10 MHz |
| wifi hop | `AUDIO_MAX_PAYLOAD` rose to 2048 with the SPI ceiling (shared header, flows in automatically). The classic hub dropped `len > AUDIO_MAX_PAYLOAD` **silently**; `hub_s3` has the `wifi-over` counter, so the surviving hub has the tripwire |
| log line | `sync` becomes `hdr`, and the SPI line gains two columns the UART one never had: `short` (a transfer that did not arrive whole -- CS split it) and `dcrc` (an SBC frame whose own CRC failed despite the link CRC passing). Both come along when the file is copied; only `dcrc` is meaningful on a UART, and the classic hub does not report it yet |
| decode split | `dcrc` vs `dec` is told apart by `sbc_decoder_last_result()` in the shared `components/sbc_decoder/`. The classic hub already links it; only its `sbc_in.c` call site would need it, and only when it adopts the `dcrc` column. `short` is SPI-only -- a UART has no `trans_len` |
| Kconfig | `DANCEFLOOR_SBC_UART_RX_PIN` → the four `DANCEFLOOR_SBC_SPI_*_PIN` symbols, plus `DANCEFLOOR_SBC_LINK_SPI_HZ` (shared through `dancefloor_sync`; only the bridge's value is on the wire) |
| CMakeLists | `esp_driver_uart` → `esp_driver_spi` |

### 7.3 What the classic hub does *not* have to give up

The S3 paid a pin for this and the classic hub does not have to. On the XIAO the
four signals took all but one free pad, and `DANCEFLOOR_MONITOR_GPIO` (GPIO 5)
was one of them — so the marker/monitor instrument, which needs GPIO 4 **and**
GPIO 5, is finished on that board. The classic hub breaks out far more pins and
can keep both the marker pair and the LED marker while taking four for SPI.

Worth knowing because it inverts the usual direction of this document: on this
item the classic hub can end up with **more** instrument than the S3, and if the
marker measurement is ever wanted again, that is the board to want it on.
