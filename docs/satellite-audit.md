# Satellite code audit — clarity, modularity, expandability

Written 2026-08-12, before the satellite acquires a second target (a Seeed
Studio ESP32-S3). It asks three questions of `satellite/` as it stands: can a
human reviewer follow it, is it modular, and what will extending it cost.

> **Status: acted on, same day.** The audit was written first and the findings
> below are unedited. Everything in §6's suggested order was then carried out on
> branch `docs/satellite-audit-drop-classic-hub`, plus one thing the audit did
> not propose: the internal-DAC path was deleted outright rather than guarded.
> Each finding now carries a **DONE** / **PARTLY DONE** / **DELIBERATELY NOT
> DONE** note. §7 is the record of what changed.
>
> **Both targets build and link**, and the classic build has now been **run on
> hardware** — hub plus one satellite, 16:39 on 2026-08-12, about two and a half
> minutes. §7.5 records what that run showed, including the one thing it
> contradicted. No S3 board has been flashed.

> **The verdict up front.** The substance of this firmware is unusually strong
> and its structure is not. Almost every non-obvious constant carries the run
> that produced it — `ANCHOR_MIN_LEAD_US` records the 144 refusals and the one
> that got through, `GAP_RESYNC_MS` records the 443 ms of silence that cost five
> minutes of servo, `CAP_USABLE_INTERNAL` records the evening a dead satellite
> looked healthy. That reasoning is the most valuable thing in the tree and it
> is *why* the file is 2437 lines. Every remedy below is about moving it, never
> about deleting it. A restructure that thins these comments has destroyed more
> than it fixed.

---

## 1. Measurements

Taken on `satellite/main/main.c` at the time of writing. The commands are in §5
so the table can be regenerated after any of it moves.

| | |
|---|---|
| `satellite/main/main.c` | **2437 lines** |
| — comment and blank | 1291 (**53%**) |
| — code | 1147 |
| file-scope statics | **74**, of which **60** are `volatile` |
| function-local statics | 68 |
| preprocessor directives | 43 (`ENABLE_VISUALISER` ×7, `USE_INTERNAL_DAC` ×7) |
| "see the hub's copy" cross-references | **26** |
| longest functions | `handle_audio` **371**, `play_task` **352**, `drift_task` **339**, `rx_task` 210 |

For scale, the whole satellite is one file roughly the size of the hub's
`streamer.c` (2661 lines), and larger than every shared component's `.c`/`.cpp`
put together except the visualiser.

---

## 2. Findings

Ranked by what they will cost when the S3 arrives. Each says what it is, where
it is, what it costs now, and what it costs then.

### F1 — Porting by forking has been tried here, and has just been abandoned

> **DONE.** `satellite/` builds for both targets from one source: `idf.py
> set-target esp32s3` picks up `sdkconfig.defaults.esp32s3` and the
> `IDF_TARGET_ESP32S3` pin defaults in `main/Kconfig.projbuild`. No
> `satellite_s3/` exists and the S3 defaults file says why.


**Evidence.** `hub_s3/` was created as a copy of `hub/`. At the point `hub/` was
deleted the two shared **~1983 of ~2024 lines** in `streamer.c` and had drifted
in both directions. `docs/hub-s3-gap-list.md` — 481 lines — existed for no other
purpose than to reconcile them by hand, and says so:

> "What is missing is not the analysis but a classic hub on a bench, and the
> cost of waiting is that the two files drift — so this list is the substitute
> for the diff that would otherwise have shown it."

**What it cost.** Its §1 lists four portable wins that were measured on the S3
and never carried back. Its §6 lists sync work that landed on one copy only. Its
§7 records that the classic hub had silently stopped being able to receive
audio at all. The fork did not just duplicate code; it duplicated it and then
let the copies disagree about whether they worked.

**What it costs on the S3.** A `satellite_s3/` made the same way doubles a
2437-line file whose correctness rests on constants measured on specific
hardware, and every one of the 26 "see the hub's copy" cross-references becomes
a three-way relationship.

**Remedy.** One `satellite/` project, `idf.py set-target`, and a
`sdkconfig.defaults.esp32s3` beside the existing `sdkconfig.defaults`. Chip
differences go behind `SOC_*` guards (F5) and per-target Kconfig defaults. Large
— but the alternative has a 481-line price tag already written down.

*This is the finding the rest of the list serves: F2–F7 are each a reason the
one-source arrangement is currently harder than it needs to be.*

### F2 — One translation unit, six concerns

> **DONE.** Nine files; largest is `rx.c` at 682 lines. Every body moved
> verbatim — the only semantic edits were three `continue;` → `return;` where
> the servo's loop body became a function, and three loop locals becoming
> zero-initialised file statics. Verified by normalising both versions and
> diffing statement multisets: nothing else differs.


**Evidence.** `main.c` holds, with nothing but banner comments between them:

| concern | roughly |
|---|---|
| WiFi STA lifecycle + reconnect | `wifi_event`, `wifi_start_sta` (`:490–589`) |
| UDP socket and message demux | `socket_start`, `rx_task` (`:591–604`, `:1197–1407`) |
| clock source selection (TSF vs probe) | `clock_offset`, the `MSG_TSF` arm (`:812–822`, `:1285–1402`) |
| SBC decode and ring feed | `handle_audio` (`:824–1195`) |
| playback timeline: anchor, phase, splice, marker | `play_task`, `track_offset` (`:1941–2317`) |
| output clock servo | `drift_task`, `retune_output` (`:1481–1911`) |
| telemetry: HEALTH, MEM, RX window, alloc hook | scattered across `drift_task` and `:450–458` |

**What it costs a reviewer.** `handle_audio()` is 371 lines in which anchor
policy, gap policy, phase-queue bookkeeping and the decode loop interleave, so
anchor policy cannot be reviewed without reading the decoder. `drift_task()` is
339 lines of which roughly 200 are telemetry that has nothing to do with the
servo it is named for — the HEALTH line, the MEM line, the RX window narration
and the allocation-failure report all live inside the rate-control loop, for the
stated and good reason that it is a task that can afford to block on a UART.
Good reason, wrong room.

**Remedy.** Split verbatim, no behaviour change, into `satellite/main/`:
`net.c` (WiFi + socket), `clock.c` (TSF/probe selection + slew), `rx.c` (demux,
decode, ring feed), `play.c` (timeline, splice, marker), `servo.c` (rate
control), `telemetry.c` (all periodic reporting). Medium and mechanical. Doing
this *before* the S3 port is what makes F1's one-source arrangement reviewable;
doing it after means doing it twice.

**Follow-up, done separately.** Moving `handle_audio()` into `rx.c` made that
file coherent but left the function itself at 371 lines, still interleaving four
policies. It was then broken into one function per decision — `anchor_stream`
(152), `absorb_sequence_gap` (98), `decode_into_ring` (49),
`record_packet_positions` (22), `upgrade_provisional_anchor` (11) — with a
29-line `handle_audio` that is just the order those are taken in. Unlike the
file split this changed control flow rather than moving it: eight bare `return`
statements became typed results, six `return false` and three `return true`,
which the statement-multiset diff accounts for exactly. Anchor policy can now be
read without reading the decoder.

### F3 — 60 `volatile` globals are the cross-task interface, and the file's own tearing rule is applied unevenly

> **PARTLY DONE, and the part not done is the interesting one.** The TSF pair
> is now behind a sequence lock — it had a second bug the audit missed:
> `clock_offset()` read `at`, decided it was fresh, then read `us`, so a publish
> landing between the two lines paired a fresh timestamp with a stale offset,
> on the path that anchors a stream. `stream_start_local` was **not** converted:
> it has two writers (rx writes the anchor, play writes 0), and a seqlock with
> two writers is silently broken. `sat.h` now carries the per-value analysis,
> including what tearing actually costs for each and which three are accepted.


**Evidence.** `main.c:108–115` states the rule explicitly and correctly:

> "32-bit, not 64: these are read by the playback task while the receive task
> writes them, and a 64-bit load is two instructions on this CPU — a torn read
> yields a garbage position and a wild marker."

Nine 64-bit values then cross tasks anyway: `stream_start_local` (`:91`),
`anchor_at` (`:97`), `tsf_offset_us` / `tsf_offset_at` (`:263–264`),
`wifi_down_at` / `rejoined_at` / `est_newest_at` (`:377–379`), and the retune
pair (`:1440`, `:1457`).

**How much this matters.** Less than it looks, and not nothing.
`stream_start_local` is the sharpest case — written by `rx_task`, read by
`play_task` to compute the scheduled wait — but it is written only at an anchor,
and the `== 0` park test is immune to tearing because both halves are zero. The
exposure is a torn *non-zero* read producing a wild wait instant. Rare. Also
exactly the class of fault this file elsewhere refuses to leave uncounted.

**What it costs on the S3.** The S3 is dual-core and `play_task` is already
pinned to core 1 (`:2432`) while `rx_task` floats, so the two genuinely run
concurrently rather than interleaving on one core. A rare race becomes a less
rare one.

**Remedy.** Decide the rule once and apply it uniformly — `atomic_int_least64_t`
or a documented seqlock for the few that must be 64-bit — and state in one place
which task owns each field. Today that ownership is real but distributed across
twelve separate comments. *Flagged as a decision worth taking deliberately, not
as a proven defect.*

### F4 — A shared component reads Kconfig symbols that each project must redeclare

> **DONE.** The `DANCEFLOOR_OUT_*` choice moved to
> `components/dancefloor_sync/Kconfig`, beside `audio_out.h` which reads it.
> Both project copies deleted; one declaration site remains.


**Evidence.** `components/dancefloor_sync/include/audio_out.h` consumes
`CONFIG_DANCEFLOOR_OUT_LEFT` / `_RIGHT` / `_MONO`. Those are declared in
`satellite/main/Kconfig.projbuild:35–44` **and independently again** in
`hub_s3/main/Kconfig.projbuild:88–97`. The retired `hub/` never declared them,
so its build silently took the `#else` branch and played stereo — not by
decision, by omission.

**Why it is the worst-shaped bug here.** It fails silently, in a shared header,
in a way that looks like a working default. Any third project — a
`satellite_s3/`, or a second satellite variant — inherits the trap on day one.

**Remedy.** Move the `choice` into `components/dancefloor_sync/Kconfig`, which
already hosts precisely this kind of shared symbol and already explains the
principle in its `DANCEFLOOR_LOG_PERIOD_S` help text:

> "Lives here because both firmwares require this component, and because Kconfig
> symbols are global in ESP-IDF."

Small. Highest value per line in this document.

### F5 — The internal-DAC path cannot build on an S3, and nothing declares that

> **SUPERSEDED — the path was deleted, not guarded.** On the reasoning that it
> was a bring-up aid nobody had used since real DACs arrived, and that a
> `depends on` would still leave seven preprocessor branches threading the
> playback and servo paths for every future target to carry. 2437 → 2376 lines
> and 43 → 29 directives before the split even started.


**Evidence.** `main.c:26–28` includes `driver/dac_continuous.h` under
`CONFIG_DANCEFLOOR_USE_INTERNAL_DAC`. The ESP32-S3 has no DACs. The Kconfig
symbol (`satellite/main/Kconfig.projbuild:84`) carries no `depends on`, so
menuconfig will happily offer the option on an S3 target and the failure arrives
as a missing header rather than as an unavailable option.

**Remedy.** One line: `depends on SOC_DAC_SUPPORTED`. Small, and it is the first
concrete thing F1's one-source arrangement needs.

### F6 — Five live experiments are indistinguishable from load-bearing code

> **PARTLY DONE, from the bench logs rather than by opinion.** One retired: the
> refill-after-retune probe, 25 of 26 samples reading `0 frames`, so
> `i2s_channel_disable()` drains rather than discards — which `retune_output()`
> already assumed. The other three stay, each with what the logs now say written
> into its comment: the retune tail reaches **further** than the one withheld
> reading covers (78 crossings, 17–74 ms after, median 43), the splice median
> shadow agreed with the raw value 3 times out of 3, and `wide-span` had read 0
> on every HEALTH line — which the 16:39 run then contradicted outright, see
> §7.5.


**Evidence.** Each is individually well justified and collectively unowned:

| what | where | its own label |
|---|---|---|
| DMA refill probe | `:464–481`, `:2028–2032` | "**TEMPORARY** … Delete once the question is settled" |
| retune tail narration | `:1445`, `:2164–2169` | "**MEASUREMENT ONLY**" |
| playback-start refill | `:2025` | "**MEASUREMENT ONLY**: the REFILL line reports, nothing withholds" |
| median splice shadow | `:248`, `:2207` | "**SHADOW** … Acted on by nothing here" |
| TSF wide-span counter | `:356`, `:1350` | "**COUNTED, NOT ENFORCED**" |
| forced same-rate retune | `Kconfig:120` | bench only, servo bypass |

**What it costs.** A reviewer cannot tell an open question from a settled
mechanism without reading every comment in full, and a port carries all six
across without knowing they were meant to be temporary. `s_refill_why`,
`retune_tail_left` and `splice_report_med` all exist to answer questions that
may already have answers in the bench logs under `tools/log_collector/`.

**Remedy.** No code change needed today. A table — this one — kept current, with
each experiment's question and its retirement condition. Retire what the logs
have already answered before porting, not after.

### F7 — Deployment constants are compiled in, and one of them reasons from the wrong chip

> **PARTLY DONE.** `RING_BYTES` is now `DANCEFLOOR_RING_KB`, default 80 on both
> targets, with the Kconfig help explaining that the classic part's 117 kB is
> the reason for 80 and that an S3's 512 kB is an unmeasured opportunity rather
> than a free win. `AP_SSID` / `AP_PASS` / `MASTER_IP` are still `#define`s in
> `sat.h`: moving credentials to Kconfig touches the hub too, and pairing them
> across both firmwares is its own change.


**Evidence.** `AP_SSID`, `AP_PASS`, `MASTER_IP` are `#define`s at `main.c:47–49`,
duplicated against the hub's copies with nothing checking they agree.

`RING_BYTES` (`main.c:76`) is more interesting. Its 23-line comment reasons
explicitly from the classic ESP32's free heap:

> "It is affordable here rather than on the hub because the hub is not the unit
> that has to hold it. This one is, and it is the classic ESP32 — 117 kB free
> with a largest block of 106 kB, so 80 kB fits and 107 kB … would not allocate
> at all. That asymmetry is worth knowing before anyone proposes a longer lead
> on the strength of the S3 hub's PSRAM: the buffer a lead has to fit in is on
> the other board."

That paragraph is correct today and inverts the moment a satellite is an S3 with
PSRAM — the comment anticipates the argument and answers it for the wrong
future. It is the single clearest example of why F1 wants per-target values
rather than a fork that quietly keeps a classic-ESP32 number.

**Remedy.** Credentials to Kconfig in the shared component (with F4).
`RING_BYTES` becomes a per-target default, and that comment gets its second half.

---

## 3. What is already right

Named explicitly, because a restructure that loses these has gone backwards.

- **The component split is real.** `dancefloor_sync`, `dancefloor_leds`,
  `sbc_decoder` and `led_strip_wrapper` are genuine boundaries, and the first
  has no ESP-IDF dependency and a host test suite that runs in a second. This is
  the pattern F1 and F4 want *more* of, not a new idea being proposed.
- **`sync_proto.h` already owns the right things**: the wire format,
  `sync_to_local()`, `sync_phase_hist_t` and the median helper, `AUDIO_FRAMES`,
  `PHASE_DEADBAND_US`, `MARKER_PULSE_US`. The extraction habit exists; it simply
  stopped short of the unit logic.
- **Two whole classes of silent failure are closed.** `task_start()` (`:2351`)
  reads the return `xTaskCreate` used to discard, and `on_alloc_failed`
  (`:450`) is correctly `IRAM_ATTR` with the reason written down. Both were
  written after the failure they prevent, and both say so.
- **The HEALTH line is genuinely diagnostic** — windowed heap minimum beside the
  all-time watermark, largest-free-block beside the total, and the LED source
  and hop, which the satellite README correctly explains cannot be detected any
  other way between locally-analysing units.
- **`audio_apply_channel_mode()`** is a model of placing a per-unit decision
  where it provably cannot perturb the shared timeline, and documents that
  property rather than leaving it to be rediscovered.

---

## 4. ESP32-S3 readiness checklist

Every classic-ESP32 assumption found, and what each needs.

| item | where | status |
|---|---|---|
| internal DAC path | `main.c:26`, `Kconfig:84` | **needs a guard** — `depends on SOC_DAC_SUPPORTED` (F5) |
| `RING_BYTES` = 80 kB | `main.c:76` | **per-target value** — reasoning is classic-specific and inverts with PSRAM (F7) |
| `CAP_USABLE_INTERNAL` | `main.c:400–419` | **portable, comment already correct** — it notes the IRAM-only region does not exist on the S3, so the two masks nearly agree there |
| I2S pins 26/27/25 | `Kconfig:3–13` | **per-target defaults** — XIAO S3 numbering differs |
| LED GPIO 18, marker GPIO 2 | `dancefloor_leds/Kconfig` | **per-target defaults** — that Kconfig's help already notes GPIO 43 costs the XIAO's D6 |
| `play_task` pinned to core 1 | `main.c:2432` | **portable**, but see F3 — real concurrency changes the tearing exposure |
| CPU at 240 MHz | not set | **unclaimed** — gap-list §1.1; both parts support it, satellite runs the 160 MHz default |
| QIO flash at 80 MHz | not set | **unclaimed** — gap-list §1.2, ~4× instruction-fetch bandwidth, board-dependent |
| HT20 | station side | **check** — gap-list §1.3 was the session's single biggest radio win on the AP side |
| `CONFIG_LWIP_UDP_RECVMBOX_SIZE=32` | `sdkconfig.defaults` | **already claimed**, with the arithmetic written down |
| `FREERTOS_HZ=1000` | `sdkconfig.defaults` | **already claimed** |

The three unclaimed rows are the ones `docs/hub-s3-gap-list.md` §1 measured on
the hub and nobody carried to a classic ESP32. The satellite is a classic ESP32.
They are free wins available **today**, before any S3 exists.

---

## 5. Reproducing the measurements

```sh
cd satellite/main
wc -l main.c
# comment ratio, statics, volatiles
python3 - <<'EOF'
import re
src = open('main.c').read()
nocom = re.sub(r'//.*', '', re.sub(r'/\*.*?\*/', '', src, flags=re.S))
code = [l for l in nocom.split('\n') if l.strip()]
print("lines", src.count('\n')+1, "code", len(code))
print("file-scope statics", len(re.findall(r'^static ', nocom, flags=re.M)))
print("volatile", len(re.findall(r'^static volatile', nocom, flags=re.M)))
EOF
# function lengths
awk '/^(static|void)[^;]*\)$/{n=$0} /^\{/{s=NR;f=n} /^\}/{if(f)printf "%5d  %s\n",NR-s+1,f; f=""}' main.c | sort -rn | head
# cross-references to the hub
grep -c "the hub's" main.c
```

---

## 6. Suggested order, if any of this is acted on

Sequenced so each step makes the next cheaper, and so nothing structural happens
before the free wins are banked.

1. **F5** and **F4** — two small, contained changes that remove the two traps a
   second target would hit first.
2. **§4's three unclaimed rows** — 240 MHz, QIO/80 MHz, HT20 on the station.
   Config only, measurable on the bench today, no S3 required.
3. **F6** — retire the experiments the bench logs have already answered.
   Shrinks what step 4 has to move.
4. **F2** — the verbatim split into six files. No behaviour change; verify by
   bench run, not by reading.
5. **F3** — decide the concurrency rule with the split's clearer ownership in
   hand.
6. **F1** — add the S3 target to the now-modular `satellite/`.

Steps 1–3 are worth doing whether or not the S3 ever arrives.

---

## 7. What was done, 2026-08-12

All on branch `docs/satellite-audit-drop-classic-hub`, in §6's order.

### 7.1 The shape of it

| | before | after |
|---|---|---|
| files in `satellite/main/` | 1 `.c` | 9 `.c` + `sat.h` |
| largest file | 2437 lines | `rx.c`, 682 |
| largest function | `handle_audio`, 371 lines | `rx_task`, 209 |
| preprocessor directives | 43 | 25 |
| declaration sites for `DANCEFLOOR_OUT_*` | 2 (+1 project missing it) | 1 |
| targets buildable from this source | 1 | 2, both verified |

Both images link and fit:

| target | binary | app partition free |
|---|---|---|
| `esp32` | 0xd5350 (874 kB) | 17% |
| `esp32s3` | 0xd8950 (887 kB) | 15% |

```sh
idf.py set-target esp32   && idf.py build
# or side by side, without either config clobbering the other:
idf.py -B build.s3 -DIDF_TARGET=esp32s3 -DSDKCONFIG=sdkconfig.s3 build
```

### 7.2 Beyond the findings

Two things happened that §2 did not propose.

**The internal-DAC path was deleted** rather than guarded — see F5. Requested
directly, and it made the split materially smaller.

**`tools/syntax_check.py` was written**, because a 2400-line restructure with no
way to compile was not a reasonable thing to attempt. It reuses the flags from
the last real `idf.py build` out of `compile_commands.json` and runs the
compiler `-fsyntax-only`, needing no IDF virtualenv. It caught two real errors
during the split: an orphan `#endif` left by the DAC removal, and four slice
boundaries that cut through the middle of comments. `--with` / `--without` shadow
`sdkconfig.h` to reach branches the last build did not select, which is how all
nine files were checked under seven configurations.

It has real limits, stated in its docstring: it does not link, it does not model
Kconfig `depends on`, and a new Kconfig symbol needs a real reconfigure before
it exists. **It is not a build.**

### 7.3 What to watch on the first hardware run

The split is verified faithful *statically*. These are where a mistake would
show, in the order it would show:

1. **It boots and joins.** Both targets link, so missing and duplicate
   definitions are already ruled out; what a link cannot tell you is whether
   every task actually started. Watch for `CRIPPLED:` on the join line.
2. **`OUTPUT:` at boot** still says `I2S external DAC` with the expected buffer
   depth, and `channels=stereo`. The channel-mode symbols moved Kconfig menus;
   if they came through wrong, this line says so.
3. **`stream start` then `playback started` within a second**, with `actual`
   within a few µs of `scheduled`. This exercises the reordered clock path,
   which is the change with real behaviour behind it — the TSF seqlock.
4. **`HEALTH` after 60 s**, with `clock TSF (tsf N/probe 0)`. If `probe` climbs
   where it used to read TSF, `tsf_fresh()` is refusing readings it should
   accept and the seqlock is the suspect.
5. **Smoothed phase over ten minutes.** The servo body is unchanged, but it now
   runs as a called function with its state in file scope rather than as a loop
   with locals. If those three values are not surviving between windows the
   servo will behave as though every window is its first.
6. **240 MHz and QIO flash.** Both are new on this unit and both are the kind of
   change that works until it does not. A board that will not boot after
   flashing is the QIO trap in `sdkconfig.defaults`; the recovery is there.

### 7.4 Still open

- `AP_SSID` / `AP_PASS` / `MASTER_IP` are still compiled in (F7).
- `stream_start_local` and `anchor_at` still have the tearing exposure `sat.h`
  documents; the honest fix for the first is single ownership (F3).
- The retune settle window is unsized, and the logs say the disturbance reaches
  further than the current one-shot withholding covers (F6).
- No S3 satellite has been **run**. The target builds and the image fits; the
  board has not been on a bench, and the S3 pin choices are inherited from
  `hub_s3` rather than confirmed against a wired satellite.

---

## 7.5 The first hardware run, 2026-08-12 16:39

Hub plus one satellite (192.168.4.3), classic ESP32, about two and a half
minutes. Both concerns §7.3 raised are answered, and one claim written into a
comment the day before turned out to be wrong.

### Confirmed working

**The TSF sequence lock is not demoting.** This was the change with real
behaviour behind it and the failure would have been silent. `clock TSF (tsf
1/probe 0)` at 5 s and again at 65 s — one anchor, taken on TSF, no fallback to
the estimator at any point. Both stream starts read `[TSF]`, and both reported
`playback started: scheduled X, actual X (+0 us)`.

**The servo's state survives between windows.** The other silent failure: with
`err_ema`, `err_ema_valid` and `cooldown` moved from `drift_task` locals to file
scope in `servo.c`, a mistake would have made every window behave like the
first. It does not — the smoothed value evolves continuously across windows
(−32254 → −29651 → −24645 → −19992 → −15324 → −12100 µs) and retunes are spaced
~40 s apart, which is the cooldown holding. A servo that had lost its state
would have retuned every window and never smoothed.

Also clean: `alloc-fail 0`, `phase-drop 0`, `short-reads 0`, `leds remote hop
512 (rx 5506, bad 0)`, `ml remote (rx 5508, bad 0)` — so the split did not
disturb the frame path either.

### Contradicted: `wide-span` is not zero

The comment on `n_tsf_wide` said every HEALTH line so far reported `wide-span
0`, and concluded that enforcing `TSF_SPAN_MAX_US` at 100 µs would be free. This
run read **1 at five seconds and 21 by 65 s**, against roughly 240 samples a
minute — about 9% — with `span max` reaching 210 and 214 µs in ordinary windows.

Acting on the old reading would have demoted TSF to the probe estimator for one
sample in eleven, silently. That is precisely the failure the counter exists to
prevent, so the instrument worked; the conclusion drawn from it was premature.
What changed between runs is not established. Worth ruling out before choosing
any threshold, because it decides whether 100 µs is too tight or the wide spans
are themselves the symptom.

### Sharpened: the withheld retune reading is sometimes the wrong one

Two of the three satellite retunes logged `crossed -3414 us` and `crossed -392
us` after the retune — **negative**, meaning the crossing being withheld from
the servo happened *before* the retune completed. In those cases the one reading
held back is a pre-retune reading and the actual post-retune transient goes
straight to the servo unfiltered.

This strengthens what F6 already said from the log corpus. Not only does the
tail reach further than one reading covers; sometimes that one reading is not
even measuring the retune.

### Not ours

Two things in the run are worth chasing but belong to the hub, not this work:

- **`tx-fail` climbing on the hub** — 4, then 17, then 67. The satellite's
  underrun at 16:41:48 follows it directly: `gaps 1 (182 ms silence)`, `too big
  to fill 1`, so `GAP_RESYNC_MS` refused to paper over a 182 ms hole and asked
  for a clean re-anchor, which it got, at `+0 us`. That is the gap policy
  behaving exactly as designed on a link that dropped a burst.
- **The hub's internal memory** — `MEM: internal 7824 free (min 5944, largest
  3584)`. The satellite has 85 kB free; the hub has under 8 kB with a largest
  block of 3.5 kB, and a task stack cannot be cut from that.

### Watch next

`stack free` on `drift_task` fell from 2632 to 1128 bytes between the 5 s and
65 s lines. The task is unchanged in what it does, but it now reaches the same
work through two calls instead of inline, which costs a frame. 1128 bytes of
3072 is not a problem; it is worth a glance on a longer run before assuming it
has levelled off.

---

## 7.6 The second run, 2026-08-12 17:09 — and a negative result

Hub reflashed with the merged WiFi-in-PSRAM config; satellite left on its 16:33
build. Four minutes. It is a markedly better run than §7.5, and **the merge is
not why** — which is the more useful finding of the two.

### The negative result: reflashing the hub changed nothing functional

| | 16:39 | 17:09 |
|---|---|---|
| hub internal free | 7824 | 7812 |
| min watermark | 5944 | 5928 |
| largest block | 3584 | 3584 |

Identical. Had WiFi's buffers just left internal SRAM, those figures would have
moved. Three things explain why they did not, and together they are conclusive:

1. Deleting `hub_s3/sdkconfig` and regenerating it from the merged defaults
   produced a **byte-identical file** — so the bench board was already running
   `SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y` before either run.
2. The branch's only change to `hub_s3/main/streamer.c` is the `ALLOCATION
   FAILED` line format and its comment. No behaviour.
3. `alloc-fail 0` in both runs.

The fix is real and is working — the wip commit's pre-fix watermark was **1520
bytes** with a 1700-byte WiFi TX buffer failing to allocate, against 5928 now —
but it has been working since before this audit began. What the merge bought was
putting a load-bearing config under version control instead of leaving it in a
gitignored file. That was worth doing and is invisible in a log.

**The trap this closes is worth naming.** A config that only exists in
`sdkconfig` looks exactly like a config that is committed, right up until
someone deletes the file or clones the repo. This project has hit that before —
the pattern and hop comments in `satellite/sdkconfig.defaults` were written after
`hub_s3` came up cycling a 28-second hue against its neighbours' fixed amber.
Here the same trap was holding the hub's memory fix.

### What did improve, and why it cannot be credited

**Sync converged, which §7.5's run never did.** Satellite smoothed phase walked
−28346 → −22683 → −16573 → −12569 → −8526 → −6206 → −3689 → −1173 → +150 → +582
→ **+997 µs**, crossing zero and holding inside ~1 ms. The 16:39 run reached
−12100 and then took an underrun back to −30594.

**A clean four minutes**: `underruns 0`, `gaps 0`, `splices 0`, `alloc-fail 0`,
`leds remote hop 512 (rx 15769, bad 0)`. Previously 1 underrun and 4 gaps.

**`tx-fail`**: one spike of 142 at association, then zero across the remaining
twelve samples (a single 1 at the end). Previously recurring — 4, 17, 67.

None of that is attributable. The satellite is at a different address (`.2`
against `.3`, with `sta-left 1 (dropped 1)` on the hub), the radio environment is
uncontrolled, and two runs is not a comparison. Recorded as observation, not as
a result.

### Provisionally answered: PSRAM contention

The question behind the arena/WiFi discussion — does sharing PSRAM cost the
analysis path anything?

| | 16:39 | 17:09 |
|---|---|---|
| `vis: cost: analysis` mean | 1264–1328 µs | 1213–1262 µs |
| `vis: ml: fast` mean | 748–779 µs | 752–818 µs |

No measurable cost, and if anything slightly better. **Caveat that matters:** the
current analysers are cheap and the slow lane reads `0/0 us`. This says nothing
about a real TFLM model streaming a large arena, which is the case the arena
Kconfig help warns about. It does establish that WiFi in PSRAM is not, by
itself, a problem.

### Resolved

`drift` stack free settled at **956 bytes** and stayed there across the 65 s,
125 s, 185 s and 245 s lines. §7.3 flagged it as possibly still falling after the
telemetry/servo split added a call frame; it levelled off. 956 of 3072 is thin
but stable.

### Getting worse

`wide-span` reached **64 by 185 s** — about 18/min, the same rate as §7.5, but
the reported `span max` hit **649, 673, 499 and 371 µs** against 121–247 in the
earlier run. `TSF_SPAN_MAX_US` is 100. This is the second run in a row to move
against the "enforcing it would be free" conclusion that §7.5 already retracted,
and the spans are now an order of magnitude past the threshold rather than
twice it. Worth understanding before TSF is filtered on span — and worth noting
that the reported figure is a running maximum cleared per log line, so these are
individual samples, not a drift in the typical case, which stays at 73–99 µs.
