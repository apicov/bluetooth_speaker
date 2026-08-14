# Hub code audit — clarity, modularity, expandability

Written 2026-08-12, after `satellite/` had the same pass and became nine files
the same day. It asks three questions of `hub_s3/` as it stands: can a human
reviewer follow it, is it modular, and what will extending it cost.

> **Status: acted on, same day.** The audit was written first and §2's findings
> below are **unedited**. Each now carries a **DONE** / **PARTLY DONE** /
> **DELIBERATELY NOT DONE** note, and §9 records what changed. All four images in
> the tree build — hub, satellite on both targets, and the bridge — and the split
> is verified faithful by a statement-multiset diff (§9.2).
>
> **Run on hardware the same evening**, hub plus one satellite, 18:09. §10 records
> it: the servo kept its state, memory is unchanged, nothing came up crippled, and
> the run answered two of §2's open questions and contradicted one of §3's
> assumptions. One watch item — the track-boundary splice — was not exercised.
>
> **§3's seven defects were then fixed — six of them.** §11 records it. The
> seventh was **withdrawn on §10's measurements**, which showed the retune
> transient is a step rather than a decaying tail, so withholding more readings
> would starve the servo rather than protect it. Not yet run on hardware.

> **The verdict up front.** The same one the satellite got, and for the same
> reason: **the substance of this firmware is unusually strong and its structure
> is not.** Almost every non-obvious constant carries the run that produced it —
> `CLIENT_TIMEOUT_US` records the 124 failed allocations over exactly ten
> seconds, `RESYNC_HARD_US` records the 380 ms displacement that took five
> minutes of slew, `CAP_USABLE_INTERNAL` records the line that read 8407580 bytes
> free beside a failed 1700-byte request. That reasoning is the most valuable
> thing in the tree and it is *why* `streamer.c` is 2679 lines. Every remedy
> below is about moving it, never about deleting it. **A restructure that thins
> these comments has destroyed more than it fixed.**

This document also absorbs the four sections of `docs/hub-s3-gap-list.md` that
described *this* hub rather than the retired classic one. That file was deleted
with this audit: its purpose was to reconcile two hubs by hand, and there is one
hub. See §8.

---

## 1. Measurements

Taken on `hub_s3/main/` at the time of writing, with the satellite audit's own
commands (§6) so the table can be regenerated after any of it moves.

| | satellite, before its split | hub, now |
|---|---|---|
| largest file | `main.c`, 2437 lines | **`streamer.c`, 2679 lines** |
| — comment and blank | 1291 (53%) | **1482 (55%)** |
| — code | 1147 | **1197** |
| file-scope statics | 74, of which 60 `volatile` | **89, of which 47 `volatile`** |
| preprocessor directives | 43 | **87** |
| longest functions | 371 / 352 / 339 / 210 | **360 / 318 / 315 / 161 / 144** |

The rest of the project, for scale:

| file | lines | verdict |
|---|---|---|
| `hub_s3/main/streamer.c` | 2679 | the subject of this document |
| `hub_s3/main/sbc_in.c` | 404 | single concern, well scoped, no finding against it |
| `hub_s3/main/main.c` | 54 | fine |

**87 preprocessor directives is the figure that should worry a reviewer most**,
because it is double the satellite's pre-split count and it means the default
config compiles only a fraction of this file. `ENABLE_MARKER`,
`ENABLE_VISUALISER`, `PUBLISH_FRAMES`, `PUBLISH_ML`, `WIFI_LOGS`, `AP_OPEN`,
`DISABLE_PMF`, `WIFI_PHY_RATE_MBPS` and `RETUNE_BENCH_S` all gate real code here.
§7's `s_sync_*` note records what that already cost once: four variables sat
inside the marker guard, the marker defaults to `n`, and **the hub did not
compile from its own tracked config** — every build ever run carried a local
`sdkconfig` with the marker on, so nothing showed it.

---

## 2. Findings

Ranked by what they cost a reviewer today and what they will cost the next person
to extend this. Each says what it is, where it is, and what it costs.

### H1 — One translation unit, nine concerns

> **DONE.** Twelve files plus `hub.h`; largest `.c` is `timeline.c` at 390 lines,
> against `streamer.c`'s 2686. Every body moved verbatim -- the only semantic
> edits were three `continue;` -> `return;` where the servo's loop body became a
> function, and four open-coded client snapshots becoming one helper (H5).
> Verified by normalising both versions and diffing statement multisets: §9.2
> accounts for every difference.
>
> **Followed up separately** (§12), as the satellite did for `handle_audio`: the
> two functions the split left long are now one function per decision --
> `local_play_task` 363 -> 44 lines, `streamer_send_sbc` 313 -> 83.


**Evidence.** `streamer.c` holds, with nothing but banner comments between them:

| concern | roughly |
|---|---|
| SoftAP lifecycle, PMF, HT20, PHY rate | `wifi_start_ap`, `wifi_event` (`:1276–1457`) |
| send list + ARP seeding | `client_seen/joined/gone`, `arp_add/drop` (`:502–755`) |
| client fan-out | `streamer_send_meta`, `publish_frame`, `publish_ml` (`:835–937`) |
| timeline publication + slew | `streamer_send_sbc` (`:944–1259`) |
| local playback, splice, marker crossing | `local_play_task` (`:1762–2122`) |
| I2S bring-up and retune | `i2s_start`, `retune_dac` (`:1600–1755`) |
| output clock servo | inside `ring_monitor_task` (`:2494–2622`) |
| telemetry: HEALTH, MEM, alloc hook | also inside `ring_monitor_task` (`:2305–2476`) |
| time-probe server, TSF, log relay | `probe_task` (`:2126–2270`) |

**What it costs a reviewer.** `ring_monitor_task()` is 318 lines of which roughly
170 are telemetry that has nothing to do with the servo it is named for — the
HEALTH line, the MEM line, the allocation-failure report and the structured
`health_msg_t` all live inside the rate-control loop, for the stated and good
reason that it is a task that can afford to block on a UART. **This is the
identical fault the satellite's 339-line `drift_task` had, with the identical
justification.** Good reason, wrong room.

`local_play_task()` is 360 lines in which phase-queue bookkeeping, splice policy,
the marker pulse, the refill instrument and the DAC write interleave, so splice
policy cannot be reviewed without reading the phase crossing loop.

`streamer_send_sbc()` is 315 lines carrying timeline start policy, source
steadiness, slew control, restart flagging, the phase queue, the visualiser
anchor and the unicast fan-out — seven decisions in one function.

**Remedy.** Split verbatim, no behaviour change. See §6.

### H2 — 47 `volatile` globals are the cross-task interface, and the file's own tearing rule is applied unevenly

> **DONE, in two passes.** `hub.h` first carried the ownership statement and a
> per-value analysis of what tearing costs; `local_start` was left alone because
> it had two writers and a seqlock with two writers is silently broken. **Single
> ownership was then taken** (§12): the play task parks on a 32-bit `local_epoch`
> the timeline publishes after the instant, and writes neither, so the timeline
> is the only writer of both and the pair reads safely without a lock. The servo
> asks the new `s_playing` rather than `local_start == 0`, which stopped being
> the question once the value survived an underrun. The other eight remain
> documented as accepted, with why.


**Evidence.** `streamer.c:200–205` states the rule explicitly and correctly:

> "32-bit, not 64: read by the playback task while the receive task writes them,
> and a 64-bit load is two instructions here — a torn read gives a garbage
> position and a wild marker. int32 holds 13 hours of frames at 44.1 kHz."

Nine 64-bit values then cross tasks anyway: `local_start` (`:265`), `s_marker_at`
(`:192`), `s_vis_anchor_due` (`:773`), `s_sync_err_us` and `s_sync_at`
(`:1484–1485`), `s_hub_splice_at` (`:1490`), `s_retune_outage_us` (`:1669`),
`s_retune_done_at` (`:1692`).

**How much this matters.** `local_start` is the sharpest, and it is the hub's
exact analogue of the satellite's `stream_start_local` — including the reason
that value was **not** fixed there. It has **two writers**: `streamer_send_sbc`
sets it at `:1257`, `local_play_task` zeroes it at `:1826`. A seqlock with two
writers is silently broken, so the honest fix is single ownership, not a lock.
The `== 0` park test is immune to tearing because both halves are zero; the
exposure is a torn *non-zero* read producing a wild wait instant.

**What it costs here specifically.** The satellite audit noted this exposure gets
worse on a dual-core part. **The hub already is one**, and already pins `play` to
core 1 (`:2656`) while `probe` and `ringmon` float — so these tasks genuinely run
concurrently rather than interleaving on one core. The satellite's F3 was written
about a hypothetical future; on the hub it is the present.

**Remedy.** State ownership once, in a `hub.h` modelled on `sat.h` — which is the
best file in this tree for exactly this and already carries the per-value
analysis of what tearing costs. Then decide the few that must be 64-bit
deliberately. *Flagged as a decision worth taking, not as a proven defect.*

### H3 — Five Kconfig symbols are declared twice, and two of them must agree across units

> **DONE for the two that matter.** `DANCEFLOOR_AP_OPEN` and
> `DANCEFLOOR_DISABLE_PMF` moved to `components/dancefloor_sync/Kconfig` under a
> new "Dancefloor radio pairing" menu; both project copies deleted. The other
> three stay per-project on purpose -- `MARKER_GPIO` and `ENABLE_MARKER` are
> per-board and `RETUNE_BENCH_S` is per-bench, so two declaration sites is the
> correct number for them.


**Evidence.** `DANCEFLOOR_OUT_*` moved into `components/dancefloor_sync/Kconfig`
when the satellite was audited, and that file's help text states the principle:

> "Lives here because both firmwares require this component, and because Kconfig
> symbols are global in ESP-IDF."

Five symbols still have two declaration sites — `hub_s3/main/Kconfig.projbuild`
and `satellite/main/Kconfig.projbuild`:

| symbol | hub default | satellite default | verdict |
|---|---|---|---|
| `DANCEFLOOR_ENABLE_MARKER` | n | n | per-board, fine |
| `DANCEFLOOR_MARKER_GPIO` | 4 | 4 | per-board, fine |
| `DANCEFLOOR_RETUNE_BENCH_S` | 0 | 0 | per-bench, fine |
| **`DANCEFLOOR_AP_OPEN`** | n | n | **must match** |
| **`DANCEFLOOR_DISABLE_PMF`** | y | y | **must match** |

**Why the last two are different in kind.** One firmware is the AP and the other
is the STA of the same link. The satellite's own prompt string says so outright —
"Hub SoftAP has no password (**must match the hub**)" — and nothing checks it.
They agree today by coincidence of two independently-maintained defaults.

**Why it is the worst-shaped bug here.** It fails silently and it fails as
something else: an `AP_OPEN` mismatch surfaces as a station that will not
associate, which `wifi_start_ap`'s own comment (`:1315–1321`) records presenting
as "incorrect password rather than anything that points at the real cause".

### H4 — `AP_SSID` / `AP_PASS` are compiled into both firmwares independently

> **DONE.** Both are `CONFIG_DANCEFLOOR_AP_SSID` / `_AP_PASS` in the same new
> menu, consumed by `streamer.c` and `sat.h` alike. `MASTER_IP` stayed a
> `#define` in `sat.h`: it is the esp_netif SoftAP default rather than a shared
> secret, and the hub never declared it.


**Evidence.** `streamer.c:32–33` and `satellite/main/sat.h:86–88` each `#define`
them, with nothing checking they agree.

This is the half of the satellite audit's F7 that was explicitly left open:

> "`AP_SSID` / `AP_PASS` / `MASTER_IP` are still `#define`s in `sat.h`: moving
> credentials to Kconfig touches the hub too, and pairing them across both
> firmwares is its own change."

**The hub is that other half**, and this audit is the moment the change stops
touching a project that is not being worked on.

### H5 — The same client snapshot is open-coded four times

> **DONE.** One `clients_snapshot()` in `clients.c`, four call sites. The
> message building stays four separate functions, because publish_ml's comment
> defending that is about cadence and rate limits and is untouched by this.


**Evidence.** `streamer_send_meta` (`:843–853`), `publish_frame` (`:881–894`),
`publish_ml` (`:922–935`) and `streamer_send_sbc` (`:1211–1243`) each do
`portENTER_CRITICAL` → `memcpy` a `client_t snapshot[MAX_CLIENTS]` →
`portEXIT_CRITICAL` → iterate → `sendto`.

**What is genuinely deliberate, and what is not.** `publish_ml`'s comment
defends being a copy of `publish_frame`:

> "Deliberately a copy of publish_frame() rather than a shared helper taking a
> type and a length. The two differ in the message they build and in nothing else
> today, but they are on different cadences and different budgets […] and the
> first thing either is likely to grow is its own rate limit."

**That argument is sound and this finding does not touch it.** It defends the
message building. It does not cover the snapshot, which is byte-identical in all
four and is precisely where `MAX_CLIENTS` scaling lands — the gap list's own
airtime model predicts ~30 satellites against a `MAX_CLIENTS` of 8, and a
stack-allocated snapshot copied under a spinlock four times per packet is the
thing that has to change when that number moves.

### H6 — Two instruments have nothing behind them

> **DONE, both halves.** `task_start()` is declared in `streamer.h` and
> `sbc_in.c` now uses it, so the one task whose failure costs all the audio is no
> longer the one task nothing counted. `telemetry_tick()` gained a `CRIPPLED:`
> line that reads `n_task_fail` **and** `s_task_fail_names` -- its own line
> rather than a HEALTH field, for the same reason `MEM:` is its own: HEALTH
> already truncates at the collector.


**`s_task_fail_names` is written and never read.** `task_start()` accumulates
failed task names into a 64-byte buffer (`:412`, `:441–446`). Nothing in
`hub_s3/` reads it, and `n_task_fail` appears on neither the `HEALTH` line nor
`health_msg_t`. The satellite reads both at `net.c:93` and emits
`CRIPPLED: N task(s) failed to start: <names>`.

The hub's comment (`:423–427`) explains why it needs no *repeat* of that line:

> "Its console is a dedicated UART wire at 921600 rather than a shared USB bridge
> […] so a boot-time ESP_LOGE here actually gets out. The satellite's does not,
> which is why its copy defers the news to GOT_IP."

**That reasoning is correct and does not cover the buffer.** No path on any
console reads it. By this file's own standard — `sbc_in.c:320` refuses to keep "a
column that could only ever print 0" on the grounds that it would be "an
instrument with nothing behind it" — this is the same thing.

**`task_start()` is static to `streamer.c`.** `sbc_in.c:394` therefore open-codes
its own checked `xTaskCreatePinnedToCore` with a duplicated error message and no
counter increment, so a failed `sbc_in` task leaves `n_task_fail` at 0. The
satellite audit listed `task_start()` under "what is already right" — closing a
whole class of silent failure — but it was only ever closed inside one
translation unit.

### H7 — Six live experiments are indistinguishable from load-bearing code

> **PARTLY DONE, and on the logs rather than on opinion.** Nothing was retired
> during the restructure, which is the discipline this finding asks for. The
> 18:09 run then answered two of the six and **both were retired** (§12): the
> SERVO DIVERGES raw/EMA shadow, and the refill probe's retune arm. Four remain
> live and the table above is still their record.


**Evidence.** Each is individually well justified and collectively unowned:

| what | where | its own label |
|---|---|---|
| median splice shadow | `:1492–1503`, `:1974–1991` | "**SHADOW**, acted on by nothing here" |
| REFILL instrument | `:474–500`, `:2105–2116` | "**MEASUREMENT ONLY**" |
| RETUNE TAIL narration | `:1671–1693`, `:1934–1947` | "**MEASUREMENT ONLY**" |
| servo raw/EMA shadow | `:2587–2592`, `:2602–2613` | "The raw input is the shadow now" |
| TSF on the AP interface | `:2238–2268` | "Measurement only" |
| forced same-rate retune | `Kconfig:224`, `:2482–2492` | bench only, servo bypass |

**What it costs.** A reviewer cannot tell an open question from a settled
mechanism without reading every comment in full, and each carries a real cost:
`SERVO DIVERGES` and `RETUNE TAIL` both log at `ESP_LOGW`, and the TSF probe
reply sends a second datagram per probe per satellite.

**Remedy.** No code change needed today — a table, this one, kept current with
each experiment's question and its retirement condition. The satellite audit
retired one of its six **from the bench logs rather than by opinion**, and the
same corpus under `tools/log_collector/` may already answer the REFILL and
RETUNE TAIL questions here. Retire what the logs have answered before the split,
not after: it shrinks what has to move.

### H8 — Both of the hub's documents were written against a directory that no longer exists

> **DONE.** `docs/hub-s3-gap-list.md` is deleted, its four surviving sections
> carried into §3, §4 and §5 here, and §8 records what died with it.
> `hub_s3/README.md` is rewritten as the hub's own README. Four stale
> cross-references elsewhere in the tree were repointed or dropped;
> `satellite-audit.md`'s two mentions were left, being a dated record that reads
> correctly in the past tense.


`hub/` was deleted on 2026-08-12.

- **`hub_s3/README.md`** opens "The same hub firmware as [`../hub`](../hub)",
  tells the reader `diff -r ../hub .` "is meant to stay short enough to read",
  carries a "classic hub" column in two tables, and closes with "fold it back
  into `../hub`". Every one of those instructions now points at nothing.
- **`docs/hub-s3-gap-list.md`**, 481 lines, existed for no purpose but to
  reconcile two hubs by hand. Its §6.2 was nevertheless **the hub's only live
  defect list** — seven real, still-present bugs — buried in a document named for
  a comparison that no longer exists. A reviewer asking "what is wrong with the
  hub" would not open a file called *What the S3 hub has that the classic hub
  does not*.

**Remedy.** Delete the gap list, carry its four surviving sections here (§3, §4,
§8), and rewrite the README as the hub's own rather than as a port note.

### H9 — Seven comments describe the current SBC link as a UART

> **DONE.** All seven corrected, including the Bluetooth claim in
> `streamer.c:1884` that the two-chip split had made false.


The link has been **SPI** since the bridge moved to `sbc_spi.c`. The code is
clean — no `esp_driver_uart` in either `CMakeLists.txt`, no
`DANCEFLOOR_SBC_UART_RX_PIN`, and `sbc_link.h`'s UART declarations went with the
classic hub on 2026-08-12. **The prose did not follow**, and these are not
history notes, they are present-tense descriptions of the wrong wire:

| where | said |
|---|---|
| `sbc_in.h:2` | "undecoded SBC over **UART** from the bridge" — the header's one-line summary of what the file does |
| `main.c:4` | the architecture diagram: `[bt_bridge] --UART--> [this chip]` |
| `streamer.c:1599` | "That link is now **UART**, but there is no reason to move this" |
| `streamer.c:88, :94` | attributes the burst pattern to the UART, and says "the I2S link used to hide this; **UART** does not" |
| `streamer.c:1884` | "preemption a board also running **Bluetooth**, a SoftAP, SBC decode and the bridge **UART** hands out" |
| `sync_proto.h:522` | the same list, in the shared header |
| `dancefloor_leds/Kconfig:118` | pins taken "on the hub the SBC **UART RX (23)**" — a pin that does not exist on this part |

`streamer.c:1884` carries a **second** error that is older and worse: this chip
runs no Bluetooth. That is the entire point of the two-chip split, and `main.c`
says so ten lines from the top. The comment predates the split and was carried
across it unread.

**Why this is worth a finding rather than a typo fix.** `sbc_in.h` is the file a
reviewer opens to learn what the input path *is*, and it told them the wrong
thing in its first sentence. The distinction that matters — I2S was continuous
and accumulated alignment error, SPI is transactional and CS resets the bit
counter every frame — is argued properly in `sbc_link.h`, and `sbc_in.h` was
still carrying the superseded UART version of that argument.

---

## 3. Live defects — behaviour, not structure

Confirmed against `streamer.c` as it stands rather than taken from the gap list.
These are listed here so the split does not silently carry them, and they land
**after** it, one at a time, each with its own `TRACK DIVERGENCE` window.

1. **The underrun-recovery restart flag can be lost.** `s_underrun_recover` is
   consumed into `recovered` at `:982–986`; the source-steadiness gate at `:999`
   can then `return` at `:1004` before `msg.restart` is ever written at `:1143`.
   The flag is gone, `recovered` is false on the next packet, and **no satellite
   ever learns the timeline restarted** — reintroducing the exact bug the comment
   above it was written to fix.
2. **A hard timeline jump is unannounced.** `:1076–1083` jumps `next_play_at` and
   sets no `restart` flag, so satellites take a step of up to `RESYNC_HARD_US`
   with no splice hint, below `PHASE_INSANE_US` and so with no re-anchor either.
   The servo walks it off at 2.27 ms/s.
3. **Client aging only runs inside the audio send loop** (`:1229`), so nothing
   ages out while audio is stopped. Belongs on `ring_monitor_task`'s 5 s tick.
4. **The DMA prefill at playback start is unguarded.** `s_refill_active` reports
   and withholds nothing (`:494`, `:2105–2116`). On an empty channel the first
   writes return at memory speed, so `samples_played` advances by the whole DMA
   depth (6 × `AUDIO_FRAMES` = 34.8 ms) against a `wrote_at` that has barely
   moved, and every phase reading dated inside that window is measured against a
   reference the DAC is not pacing. Very likely the "−42 ms (hub), −26 ms
   (satellite)" startup phase in `clock-sync.md` §8.
5. **The splice runs on one raw phase reading** (`:1968`), while the servo has
   used a 4-sample average since it was measured triggering on noise. The file's
   own comment records two reads of `s_phase_err_us` a millisecond apart
   differing by **15.7 ms**. The median is computed beside it (`:1983–1991`) and
   read by nothing.
6. **Only one phase reading is withheld after a retune** (`s_retune_watch`,
   `:1926`), against a transient the satellite's logs put at 17–74 ms after the
   retune, median 43 — and crossings arrive ~20 ms apart.
7. **A short ring read biases `samples_played`** (`:1830–1835`): the pad is
   played but was never in the ring, while `samples_played` advances by a whole
   chunk regardless, permanently displacing every later phase point. Whether it
   fires at all is what `n_short_reads` is for.

Items 5 and 6 must land on **both** units at once — one-sided would guarantee the
two splice to different places, which is the bug itself.

---

## 4. Settled elsewhere — do not re-open without a new reason

Carried from the deleted gap list §4, because these were expensively measured and
re-litigating them is the failure this table exists to prevent.

| question | answer | evidence |
|---|---|---|
| PHY rate | **unpinned (0, adaptive)** | paired with AMPDU — see *the flip* |
| TX AMPDU | **on**, `TX_BA_WIN 6` | ditto; the two are forced to move together |
| TX buffers | **32** | 64 gave 329 vs 32's 420 — noise, and the heap wants the 32 |
| PSRAM | **on, `CAPS_ALLOC` only** | §5 — `malloc()` never returns it |
| SoftAP bandwidth | **HT20** | the single biggest radio win: `tx-fail` 1016 → ~400 |
| `CONFIG_LWIP_UDP_RECVMBOX_SIZE` | **32** | §5 |

**The flip, and why the old answer was not wrong.** The rate was pinned at 6 Mbps
and aggregation off for most of this project, on measurements that still stand:
6 Mbps gave 1.2 gaps/min, 12 gave ~115, 24 measured 23% loss, and
adaptive-plus-AMPDU gave 3.5–14. The two settings are welded together —
`esp_wifi_internal_set_fix_rate` refuses to run with aggregation enabled — so
they could only ever move as a pair.

What changed is the *goal*, not the physics. Unicast fan-out makes downlink
airtime scale with satellite count, and at 6 Mbps the duty-cycle wall arrives
around 6–8 satellites. A-MPDU amortises the ~350 µs of per-frame
DIFS/backoff/preamble across a burst and lets rate adaptation reach MCS7
(~65 Mbps), roughly 10× the airtime headroom — that is what buys satellite count,
and pinning 6 Mbps cannot.

The 3.5–14 gaps/min that condemned adaptive+AMPDU was measured with a 16 kB
I-cache and the USB-PHY mitigation off. Both have since flipped, so that number
describes hardware that no longer exists. Re-measured on the current
configuration: **0 audio gaps/min over 18 min at one satellite, `tx-fail` 0**.

**Still unmeasured:** anything past one satellite. The airtime model predicts
~30, and nobody has put more than one on the floor with this config.

**The socket receive queue.** `CONFIG_LWIP_UDP_RECVMBOX_SIZE` is 32 on both
units. The hub's case is the quiet one — its socket carries only time probes and
splice reports, four probes a second per satellite, so at one satellite six deep
would be 1.5 s of slack. It does not stay that way: at `MAX_CLIENTS` = 8 it is 32
datagrams a second and under 200 ms of headroom, from eight units free-running on
independent 250 ms timers whose probes will sometimes bunch. **The failure mode
is what makes it worth doing** — a probe lost in that queue is indistinguishable
from a satellite that stopped probing, `CLIENT_TIMEOUT_US` is 2 s, and the hub
drops a satellite that is sitting right there. A silent speaker, reported as
`sta-timeout`, with no counter distinguishing it from a real departure.

---

## 5. PSRAM, and the memory figures that mean something on this board

Carried from the deleted gap list §2.4, because it is the most easily
misread thing about this unit.

**PSRAM is on, for two things by name: the TFLM tensor arena and the WiFi/lwIP
buffers.** `CONFIG_SPIRAM_USE_CAPS_ALLOC` is the line that makes turning it on
safe rather than merely possible — ordinary `malloc()` never returns PSRAM, so
the ring, the DMA buffers, the frame queue and every stack stay in internal SRAM
with the timing they were measured with. PSRAM is reachable only by asking for it
by name: `heap_caps_aligned_alloc(..., MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)` in
`ml_arena.c`, plus `wifi_malloc()`. The alternative, `SPIRAM_USE_MALLOC`, would
hand PSRAM to anything above a threshold — cache-miss jitter in every task.

**The WiFi exemption rests on a measurement.** The registered internal heap on
this part is **~268 kB, not 512 kB** — ~102 kB is DRAM shadowed by IRAM code,
~96 kB static `.data`/`.bss`, ~33 kB ROM-reserved — and ~246 kB of that is live,
leaving ~22 kB for WiFi's 32 dynamic TX and 32 dynamic RX buffers to spike into
against a ceiling of ~105 kB. The internal watermark reached **1520 bytes** and a
1700-byte dynamic TX buffer failed to allocate. `wifi_malloc()` now prefers PSRAM
with internal fallback retained, so nothing that succeeded before can fail.

**Still refused, and the refusals are the durable part:**

- **Not for the local ring.** It is touched from the playback path, and PSRAM
  cache-miss jitter is the one thing that loop cannot absorb.
- **Not for the lead.** The ring that bounds `LEAD + RESYNC` belongs to the
  *satellite*, a classic ESP32 with no PSRAM. A 500 ms lead would need ~107 kB of
  satellite ring against a 106 kB largest free block — it would fail to allocate
  on the board that has to hold it. **Memory on the hub buys the lead nothing.**

**Read `MEM:`, not `HEALTH`'s heap terms.** This is the trap that hid a real
fault for as long as PSRAM has been on. `esp_get_free_heap_size()` reports the
8 MB PSRAM pool, which nothing on the audio path can use, so every `HEALTH` line
read ~8.4 MB free while internal SRAM went to 1.5 kB. The baseline to compare
against:

```
MEM: internal 22056 free (min 5808, window 20676, largest 11776) | total 8407688
MEM: internal 21912 free (min 1520, window 20512, largest 11776) | total 8407544
```

and, from the 2026-08-12 bench runs with the fix in place, `internal 7812 free
(min 5928, largest 3584)` — the hub is the tight unit, and a task stack cannot be
cut from a 3.5 kB largest block.

**`8BIT` in `CAP_USABLE_INTERNAL` is not decoration.** `MALLOC_CAP_INTERNAL`
alone also matches regions that are internal but 32-bit-access-only, which
nothing needing byte access can use. It costs nothing on the S3, but it cost an
evening on the satellite, where that figure read 31 kB free while the pool a
stack comes from had 396 bytes.

---

## 6. Suggested order, if any of this is acted on

Sequenced so each step makes the next cheaper, and so nothing structural happens
before the contained wins are banked.

1. **H8** — delete the gap list, rewrite the README. Costs nothing and stops the
   next reviewer being sent to a directory that is not there.
2. **H3, H4, H6** — three small contained changes removing the traps a third
   project would hit first.
3. **H7** — retire the experiments the bench logs have already answered. Shrinks
   what step 4 has to move.
4. **H1** — the verbatim split. No behaviour change; verify by bench run, not by
   reading.
5. **H2** — decide the concurrency rule with the split's clearer ownership in
   hand.
6. **§3** — the seven live defects, one commit each, after a clean bench run.

Steps 1–3 are worth doing whether or not the split ever happens.

**Reproducing the measurements in §1:**

```sh
cd hub_s3/main
wc -l streamer.c
python3 - <<'EOF'
import re
src = open('streamer.c').read()
nocom = re.sub(r'//.*', '', re.sub(r'/\*.*?\*/', '', src, flags=re.S))
code = [l for l in nocom.split('\n') if l.strip()]
print("lines", src.count('\n')+1, "code", len(code))
print("file-scope statics", len(re.findall(r'^static ', nocom, flags=re.M)))
print("volatile", len(re.findall(r'^static volatile', nocom, flags=re.M)))
print("directives", len(re.findall(r'^\s*#', nocom, flags=re.M)))
EOF
awk '/^(static|void|int|bool|esp_err_t|uint[0-9]*_t)[^;]*\)$/{n=$0} /^\{/{s=NR;f=n} /^\}/{if(f)printf "%5d  %s\n",NR-s+1,f; f=""}' streamer.c | sort -rn | head
```

---

## 7. What is already right

Named explicitly, because a restructure that loses these has gone backwards.

- **The component split is real.** `dancefloor_sync`, `dancefloor_leds`,
  `sbc_decoder` and `led_strip_wrapper` are genuine boundaries, and the first has
  no ESP-IDF dependency and a host test suite. This is the pattern H1 wants
  *more* of, not a new idea.
- **`sbc_in.c` is the counter-example to `streamer.c`.** 404 lines, one concern,
  and its log line is disciplined — quiet when the link is clean, immediate when
  it is not, and it refuses to keep a column that could only ever print 0.
- **The counters are the project's real achievement.** `n_sta_timeout`,
  `n_phase_drop`, `n_wifi_oversize`, `n_short_reads` and the allocation hook each
  exist because a fault was once invisible, and each records which one.
- **`on_alloc_failed()` is correctly `IRAM_ATTR` with the reason written down**
  (`:2281–2294`), including why it must not log and why the live heap figures
  cannot be snapshotted inside it.
- **The `MEM:` line exists at all**, and its comment explains why `HEALTH`'s heap
  figure could not answer the question on a PSRAM board.
- **`audio_apply_channel_mode()` is called last** (`:2093–2097`), deliberately
  after every count, so a per-unit speaker placement provably cannot perturb the
  shared timeline — and the comment says so rather than leaving it to be
  rediscovered.
- **The `s_sync_*` guard note** (`:1505–1516`) is the model for what this audit
  wants more of: it records that four variables sat inside `#if
  CONFIG_DANCEFLOOR_ENABLE_MARKER`, that the marker defaults to `n`, and that the
  hub therefore **did not compile from its own tracked config** — caught only
  because every bench build carried a local `sdkconfig` with the marker on.

---

## 8. What the deleted gap list contained

`docs/hub-s3-gap-list.md`, removed with this audit. Recorded here so the deletion
is legible rather than mysterious.

| section | subject | disposition |
|---|---|---|
| §1 | four portable config wins the classic hub never took | **died with `hub/`**; the satellite claimed 240 MHz and QIO in its own audit |
| §2.1–2.3, §2.5, §2.6 | S3-only silicon and board notes | **obsolete** — there is no other hub to be "not portable" to |
| §2.4 | PSRAM, `CAPS_ALLOC`, the `MEM:` baseline | **carried to §5** |
| §3 | what the portable items bought, measured | **carried in part**: 32 kB cache + 240 MHz + QIO took `analysis` 3900→1940 µs mean, 21000→9100 µs max |
| §4 | settled questions, the PHY-rate/AMPDU flip | **carried to §4** |
| §5 | socket receive queue arithmetic | **carried to §4** |
| §6.1 | Stage-0 instrumentation that landed | **done, and now just history** |
| §6.2 | seven live defects | **carried to §3** |
| §6.3, §7 | the porting rule; the SPI link the classic hub never learned | **died with `hub/`** |

---

## 9. What was done, 2026-08-12

All on branch `docs/hub-audit-split-streamer`, in §6's order.

### 9.1 The shape of it

| | before | after |
|---|---|---|
| files in `hub_s3/main/` | 3 `.c` + 2 `.h` | 13 `.c` + 3 `.h` |
| largest file | `streamer.c`, 2686 | `sbc_in.c`, 403 (unchanged); largest new file `timeline.c`, 390 |
| `streamer.c` | 2686 lines, nine concerns | **106 lines**: `task_start()` and `streamer_start()` |

(§1 measured 2679; H9's seven comment corrections landed before the split, so
2686 is what was actually cut up.)
| largest function | `local_play_task`, 363 | `local_play_task`, 363 (**unchanged, see §9.4**) |
| declaration sites for `AP_OPEN` / `DISABLE_PMF` | 2 each | 1 each |
| declaration sites for the SSID and password | 2 `#define`s each | 1 Kconfig symbol each |
| open-coded client snapshots | 4 | 1 |
| instruments written but never read | 2 | 0 |

The twelve modules, and what each owns:

| file | lines | |
|---|---|---|
| `hub.h` | 725 | shared state, **and the ownership statement** |
| `hub_state.c` | 78 | the definitions, nothing else |
| `streamer.c` | 106 | `streamer_start()`, `task_start()` |
| `net.c` | 221 | SoftAP bring-up, WiFi events, the socket |
| `clients.c` | 303 | send list, ARP seeding, every fan-out over it |
| `timeline.c` | 390 | `streamer_send_sbc()` and the feed path |
| `play.c` | 380 | this unit's own speaker, splice, phase crossing |
| `out.c` | 174 | the I2S channel and every rate change |
| `servo.c` | 195 | rate control, and the 5 s tick |
| `telemetry.c` | 246 | HEALTH, MEM, CRIPPLED, the allocation hook |
| `probe.c` | 158 | time probes, TSF, splice reports, log relay |
| `marker.c` | 102 | the bench instrument |

All four images build:

| target | binary | app partition free |
|---|---|---|
| `hub_s3` (esp32s3) | 0xe20f0 (926 kB) | 12% |
| `satellite` (esp32) | 0xd5380 (873 kB) | 17% |
| `satellite` (esp32s3) | 0xd8990 (887 kB) | 15% |
| `bt_bridge` (esp32) | 0xe4b50 (937 kB) | 68% |

### 9.2 How the split was verified faithful

Not by reading. Three checks, in order.

**A statement-multiset diff.** Both versions normalised (comments stripped,
whitespace collapsed), then the multiset of statements diffed. **Every one of the
differences is accounted for**, and the list contains no executable logic:

- 98 statements only in the old file: every `static <decl>` that became an
  `extern` in `hub.h` plus a definition in `hub_state.c`; the 13 `static void
  <fn>` that lost `static` to cross a file boundary; the two literal `#define
  AP_SSID/AP_PASS`; **3 × `continue;`**; and 4 × the snapshot boilerplate.
- 219 only in the new files: the matching `extern`/definition pairs and function
  declarations, 11 × `#include "hub.h"`, **3 × `return;`** (the `continue`s, in
  `servo_tick`), 4 × `clients_snapshot(snapshot)` plus the one helper, the
  `telemetry_tick` / `servo_tick` / `ring_monitor_task` scaffolding, and the
  three statements of the new `CRIPPLED:` line.

**Twelve configurations syntax-checked** with `tools/syntax_check.py`, because 87
preprocessor directives means the default build compiles only part of this code:
default; marker on; visualiser off; `PUBLISH_FRAMES` off; `PUBLISH_ML` off;
`WIFI_LOGS` off; `AP_OPEN` on; PMF not disabled; `RETUNE_BENCH_S=30`; PHY rate
pinned at 6 and at 24; marker on with the visualiser off. All twelve compile —
including the marker branch, which §1 records **had not been building from the
tracked config**.

**A real build** of all four images, with zero warnings.

### 9.3 One behaviour change, deliberately

The `CRIPPLED:` line is new output, and it is the only intended behaviour
difference in this branch. It reports a condition that could already occur and
was already counted; nothing else about it changes. Everything else is a move.

### 9.4 Still open

- **`local_play_task` is still 363 lines and `streamer_send_sbc` still 313.**
  Breaking them into one function per decision is the analogue of the satellite's
  `handle_audio` follow-up, and that was done as its **own** change after the file
  split, because unlike a move it alters control flow. Deliberately not attempted
  here: it would make this branch's first bench run ambiguous, which is the one
  thing the verbatim discipline exists to prevent.
- **The seven live defects in §3 are untouched**, by design. They land one at a
  time, each with its own `TRACK DIVERGENCE` window, after a clean run.
- **`local_start`'s two writers** (H2). The fix is single ownership.
- **H7's six experiments** are catalogued and none retired; that wants the bench
  logs.
- **No hardware run.** Everything above is static verification.

### 9.5 What to watch on the first hardware run

In the order a mistake would show:

1. **It boots and the tasks start.** A `CRIPPLED:` line now names anything that
   did not — including `sbc_in`, which could not report before.
2. **`OUTPUT: I2S external DAC`** with the expected buffer depth and
   `channels=stereo`. The Kconfig moves would show here.
3. **`SoftAP "dancefloor"`** and a satellite associating. If it does not, suspect
   the `AP_OPEN` / `DISABLE_PMF` menu move before anything else — those two now
   come from a different menu, and a satellite rebuilt from a stale `sdkconfig`
   could disagree with the hub. **Rebuild both units from this branch.**
4. **`timeline start`** then `local playback started` with `actual` within a few
   µs of `scheduled`.
5. **`HEALTH` and `MEM:` after 60 s.** Compare `internal ... free` against the
   recorded 7812 / min 5928 / largest 3584. `ringmon` now reaches the same work
   through two call frames, which costs stack — `stack ... mon` is the figure.
6. **Smoothed phase over ten minutes.** `servo_tick()`'s state is still declared
   where it always was, as function-local statics, so this should be unchanged;
   but it is the failure the satellite's equivalent split nearly had. A retune
   every window with the smoothed value never settling means they lost their
   state.
7. **`TRACK DIVERGENCE` across a track change**, `apart` unchanged in
   distribution from the pre-split build. The split is behaviour-neutral or it is
   not done.

---

## 10. The first hardware run, 2026-08-12 18:09

Hub plus one satellite (192.168.4.2), about two and a half minutes, collected over
WiFi. The first run of the split firmware. **§9.5's list is answered except its
last item**, and the run also settled two of §2's open questions.

### Confirmed working

**Every task started.** No `CRIPPLED:` line — which is now a statement with
something behind it, because `sbc_in` reports through `task_start()` for the first
time. The link is clean: `pkts 250` per window, `hdr 0 crc 0 short 0 gaps 0 dec 0
dcrc 0`, `fed-drop 0 B`.

**The Kconfig move did not break association.** This was the change most likely to
fail invisibly — `AP_OPEN` and `DISABLE_PMF` come from a different menu now, and a
hub/satellite mismatch presents as an incorrect password rather than as itself.
The satellite joined and held, `sta-left 0 (dropped 0)`.

**The servo's state survives between windows.** The one silent failure the
satellite's equivalent split nearly had. Smoothed phase evolves continuously
across every window:

```
-26931 -> -25725 -> -24426 -> -23054 -> -21627 -> -15275 -> -13772
       -> -12508 -> -10299 ->  -6810 ->  -4957 ->  -4303 -> -1913 us
```

with retunes 30 s and 45 s apart. A servo whose statics had not survived would
retune every 5 s window and never smooth. Both units converged toward zero over
the run, and the cross-unit gap closed from ~5.2 ms to ~3.8 ms.

**The split cost no internal memory.**

| | recorded baseline | this run, 65 s | 125 s |
|---|---|---|---|
| `internal free` | 7812 | 7932 | 7808 |
| `min` | 5928 | **5928** | **5928** |
| `largest` | 3584 | **3584** | **3584** |

Also clean: `underruns 0`, `alloc-fail 0`, `phase-drop 0`, `short-reads 0`,
`restarts 1` (the initial start).

### Not exercised — the one item still open

The track change at 18:11:05 (`TRACK #7`) produced **no `track boundary flagged`
line, no `TRACK BOUNDARY` and `splices 0`**. The likely explanation is benign:
`have_track` in `sbc_in.c` starts false at boot, so the first metadata of a
session records the id without requesting a splice. But it means the splice path,
`s_restart_pos`, and the `TRACK DIVERGENCE` comparison — §9.5's item 7, and the
one place `play.c`'s moved code does the most work — **are unverified by this
run.** It needs a session with two track changes in it.

### Answered: the REFILL-after-retune probe (H7)

All three hub retunes logged `REFILL after retune: 0 frames (0 ms) before a write
blocked`. That is the same reading that retired the satellite's copy of this
experiment — 25 of 26 samples at zero — and it means `i2s_channel_disable()`
drains rather than discards, which `retune_dac()` already assumed. **The hub now
confirms it independently, so this experiment can be retired on evidence rather
than opinion.**

The *playback-start* case is untouched by this: boot preceded the capture, so no
`REFILL after start` line was seen. That is the half that matters for live defect
4, and it is still unmeasured.

### Sharpened: the retune tail is a step, not a decaying transient (§3 item 6)

Live defect 6 assumes the post-retune disturbance outlasts the single withheld
reading. This run says otherwise on both units. The tail readings settle
immediately and stay flat across ~70 ms:

| unit | crossing ages after retune | net from before |
|---|---|---|
| hub, retune 1 | 11365 / 31689 / 51996 / 69420 µs | +1738 / +1745 / +1735 / +1745 |
| hub, retune 2 | 11364 / 31702 / 51999 / 72320 µs | +178 / +199 / +179 / +183 |
| satellite | 7688 / 28005 / 48325 / 68649 µs | −587 / −590 / −591 / −587 |

A decaying transient would show the net shrinking across the four. It does not
move. **This is evidence that the one withheld reading is sufficient and that
`RETUNE_SETTLE_US` may not need to exist**, which is the opposite of what §3 item
6 expected. Not conclusive from three retunes, but it is the first direct
measurement of the question, and it should be checked before that fix is written
rather than after.

### Also observed

- **`SERVO DIVERGES` fires often**, always in the same direction: the raw reading
  would retune, the smoothed one holds. That is the EMA doing exactly what it was
  added for, and it is data for retiring the raw/EMA shadow (H7) too.
- **`stack ... mon 1024`** of 3072, identical at 65 s and 125 s, so it has
  levelled off. `ring_monitor_task` now reaches the same work through two call
  frames, which costs a frame — the same shape as the satellite's `drift` settling
  at 956. No pre-split figure for this was ever recorded, so it is "thin and
  stable", not "worse".
- **The satellite's `wide-span` keeps climbing** — 21 at 65 s, 45 at 125 s, with
  `span max` reaching 504 µs against a `TSF_SPAN_MAX_US` of 100. Third run in a
  row moving against "enforcing it would be free". Not this branch's doing and not
  the hub's, but still unexplained.
- The `bridge serial /dev/ttyUSB0` retries are the collector's USB link to the
  bridge, not firmware.

---

## 11. The seven defects, fixed — 2026-08-12

§3's list, acted on after the split's clean bench run rather than mixed into it.
**Six were fixed; one was withdrawn on the evidence §10 produced.**

Hub, satellite (both targets) and the bridge all build; the 23-case host suite in
`components/dancefloor_sync/test/` passes; eight Kconfig permutations across both
projects syntax-check. **Not yet run on hardware.**

### 11.1 Fixed

**1 — the underrun restart flag can be lost.** `s_underrun_recover` is no longer
consumed on sight. It stays set until a packet actually reaches the `msg.restart`
assignment, and is cleared in the timeline-start branch past the steadiness gate.
The gate returns for up to `SOURCE_STEADY_US` after a hole — which is exactly the
state an underrun leaves — so the old code threw the flag away with the dropped
packet and no satellite ever learned the timeline had restarted. Re-entry while
still waiting is harmless: `next_play_at` is already 0 and `recovered` is
recomputed from the flag each call.

**2 — a hard timeline jump was unannounced.** A jump at `RESYNC_HARD_US` now arms
`s_jump_arm = SYNC_PHASE_HIST` and flags an ordinary boundary that many packets
later. **Not on the jump itself**, because at that instant every unit's phase
reading — the hub's included — still describes the timeline that just ended;
waiting the length of the splice's median window is waiting for that window to be
clear of the discontinuity. Without this, satellites took a step of up to 300 ms
with no splice hint and, being below `PHASE_INSANE_US`, no re-anchor either, then
walked it off at 2.27 ms/s — over two minutes of every speaker audibly elsewhere.

**3 — client aging ran only in the audio send loop.** Extracted to
`clients_age(now)` in `clients.c` and called from **both** the send path (before
the snapshot, so the fast path is unchanged) and `telemetry_tick()`'s 5 s pass.
Aging previously could not happen while audio was stopped, so a satellite that
vanished during a pause stayed on the send list until audio resumed. `n_sta_timeout`
still counts once per departure, because the slot is cleared and a cleared slot
never re-enters the branch.

**4 — the DMA prefill at playback start was unguarded.** Phase readings taken
while `s_refill_active` are now withheld from the servo and counted as
`n_refill_withheld` (new, on `HEALTH`). On an empty channel the first writes
return at memory speed, so `samples_played` advances by the whole DMA depth
(34.8 ms) against a `wrote_at` that has barely moved — those readings are not
phase errors at all. This is the same mechanism the `retuning` park prevents
mid-stream, where `clock-sync.md` records it costing +42, +43 and +50 ms.

> A non-zero `refill-withheld` at every playback start is the guard working. One
> that keeps climbing mid-stream means the channel is repeatedly running empty,
> which is an underrun by another name.

**5 — the splice ran on one raw phase reading.** Both units now splice on
`sync_phase_median()`, falling back to the raw value when the history holds fewer
than `SYNC_PHASE_MIN` readings. **Landed on both units in the same change**, which
the audit required: one-sided is strictly worse than neither side, because it
guarantees the two splice by different estimators.

The shadow inverted with it, and the wire field was renamed to match rather than
left lying: `splice_msg_t.applied_med_us` → **`applied_alt_us`**, carrying the raw
counterfactual now. Same offset, same width, same clamp — the layout is untouched
and either build parses the other's message — but the *meaning* flips, so **both
firmwares must be reflashed together** for the comparison to mean anything. The
`TRACK DIVERGENCE` lines say `raw:` where they used to say `median:`.

> **The revert condition, stated so it can be checked:** if `raw:` is consistently
> the *smaller* `apart` figure across a session, this change was wrong.

**7 — a short ring read biased `samples_played`.** It now advances by the frames
actually read from the ring, not the padded chunk size. `samples_played` is a
position in the ring stream and is compared against `s_phase_q[].pos`,
`s_marker_sample` and `s_restart_pos`, all of which count only frames that were
really in the ring; the pad never was. The displacement was permanent and
cumulative. The pad does take DAC time, so the timing reference shifts by up to
one chunk for one pass — a one-off against something that never went away.

### 11.2 Withdrawn on the evidence — defect 6

**Not fixed, and this is the finding.** §3 item 6 proposed replacing the one-shot
`s_retune_watch` with a sized `RETUNE_SETTLE_US` window, on the theory that the
post-retune transient outlasts the single withheld reading.

§10 measured it on both units and the transient **does not decay** — it is a step
that settles at the first crossing and holds flat across ~70 ms:

| unit | net from before the retune, four successive crossings |
|---|---|
| hub | +1738 / +1745 / +1735 / +1745 |
| hub | +178 / +199 / +179 / +183 |
| satellite | −587 / −590 / −591 / −587 |

A decaying tail would shrink across the four. Withholding more readings would
therefore withhold *valid* ones and starve the servo, which is the opposite of the
intent. **One withheld reading is the right number, and `RETUNE_SETTLE_US` should
not exist** unless a future run shows a genuinely decaying tail.

Three retunes per unit is thin evidence, so the `RETUNE TAIL` narration stays
armed rather than being retired with it — it is now the instrument that would
catch the case this decision assumes away.

### 11.3 What to watch on the next run

1. **A track change with two boundaries in one session** — §10 never exercised the
   splice at all, and fixes 2, 5 and 7 all land on that path. `TRACK DIVERGENCE`
   with `apart` at or below the pre-change figure, and `raw:` no smaller.
2. **`refill-withheld`** non-zero once at start, then static.
3. **`sta-timeout`** now able to move while audio is stopped — pull a satellite
   during a pause and it should age out within ~7 s rather than never.
4. **An underrun**, if one can be provoked: the `timeline restart flagged at seq`
   line should now actually appear, and satellites should re-splice.
5. **Both units reflashed together.** Fix 5 changes what `applied_alt_us` means.

---

## 12. The follow-ups §9.4 owed — 2026-08-12

Three items §9.4 listed as still open, taken after the split's hardware run
rather than mixed into it. All build; the 23-case host suite passes; five Kconfig
permutations syntax-check. **Not yet run on hardware.**

### 12.1 One function per decision

The split left two functions long because breaking them alters control flow
rather than moving it, and doing that inside a behaviour-neutral change would
have made the bench run ambiguous. Same reason the satellite did `handle_audio`
as its own commit.

| | before | after | largest extracted |
|---|---|---|---|
| `local_play_task` | 363 | **44** | `absorb_phase_crossings`, 154 |
| `streamer_send_sbc` | 313 | **83** | `steer_timeline`, 85 |

Both are now the order their decisions are taken in, and nothing else.
`local_play_task` reads: park for a retune, read a chunk, absorb phase crossings,
apply a track boundary, pulse the marker, write to the DAC. `streamer_send_sbc`
reads: note the arrival, start or steer the timeline, flag boundaries, record the
phase point, fan out, advance. **Splice policy can now be read without reading
the crossing loop**, which is what H1 said it cost.

The extracted functions are mostly comment — `absorb_phase_crossings` is 154
lines of which the argument for why one phase reading is trustworthy is the bulk.
That is the point: the reasoning stayed with the decision it justifies.

**Verified by statement-multiset diff, both files.** `timeline.c`'s "only in
before" set is **empty** — nothing was lost at all, because the function's own
statics moved to file scope keeping their names, so no body text was rewritten.
`play.c` shows exactly the intended changes and nothing else: three loop locals
became file statics because the loop body became functions, the underrun `break`
became a typed result, and `got_frames` became an out-parameter.

### 12.2 `local_start` has one owner

H2's named fix, finally takeable because the split made the ownership visible.

It had two writers: `streamer_send_sbc()` assigned the start instant and
`local_play_task()` zeroed it to make its outer loop park. **Two writers is what
made a seqlock silently broken**, so the 64-bit tearing exposure was documented
and accepted instead.

The play task now parks on **`local_epoch`**, a 32-bit generation the timeline
increments *after* writing `local_start`, and reads the epoch *before* the
instant. That leaves one writer for both and makes the pair safe without a lock:
a reader seeing a new epoch sees the value belonging to it, since both are
volatile — the compiler may not reorder either pair — and this core orders stores
in program order. A 32-bit epoch cannot tear, which is the rule
`s_marker_sample` and `s_samples_in` were already written down for. Wrapping
after 2^32 starts is harmless: the test is inequality, not ordering.

One consequence worth knowing: **`local_start == 0` stopped being able to mean
"is anything playing"**, because the value now survives an underrun. The servo
asks the new `s_playing`, which the play task owns and which is what it actually
wanted.

> **`stream_start_local` on the satellite is the same shape and still has two
> writers.** This is the worked example for it, and the reason to expect the fix
> to be cheap there too.

### 12.3 Two experiments retired, on the logs

H7's discipline is that these are retired from the bench corpus, not by opinion
formed during a restructure. Two of the six now have logs behind them.

**The `SERVO DIVERGES` shadow.** It computed what the raw phase reading would
have asked for and logged every disagreement with the average. Its question was
whether smoothing the servo's input declines retunes the raw value would make.
The 18:09 run fired it repeatedly and **every firing was the same way round** —
raw would retune, smoothed held — while phase converged from −26.9 ms to −1.9 ms
with retunes 30 and 45 s apart. That is the averaging doing exactly what it was
added for; a shadow that only ever says so has nothing left to find. The raw
value still prints beside the smoothed one on the servo line, so the two remain
comparable at every retune.

**The refill probe's retune arm.** It re-armed on the assumption that
`i2s_channel_disable()` discards the DMA descriptors. It drains them. The
satellite retired its copy on 25 of 26 samples reading `0 frames`; the hub read
`0 frames` on all three of its retunes independently. There is no window there to
measure and none to withhold readings inside — `s_retune_watch` already withholds
the single reading the step needs, which §11.2 established is the right number.
`s_refill_why` goes with it, having only one thing left to say.

**The start arm stays and is load-bearing** — the channel really is empty there,
because the task was parked and it drained. That is live defect 4's guard.

**Four experiments remain live**: the median splice shadow (now inverted, and
carrying the revert condition for §11.1's fix 5), the REFILL instrument at start,
the `RETUNE TAIL` narration, and the TSF probe. The §2 H7 table is still their
record.

### 12.4 What is left

- **Nothing from §9.4 is now open except hardware.** The splice path still has
  not executed in any run, so §11's fixes 2, 5 and 7 remain unproven, and 12.1
  and 12.2 have not been run at all.
- **Watch on the next run**, beyond §11.3's list: `local playback started`
  appearing once per timeline start and not repeating (the epoch handshake), and
  the servo continuing to hold off while parked (`s_playing`).
- The satellite's `stream_start_local` (12.2).

---

## 13. The transmit path, 2026-08-13

A session that began as "audible gaps on satellites and on the hub" and ended
somewhere else entirely. Recorded in the order the evidence arrived, because two
of the three changes made first were wrong and the reasons are the useful part.

### 13.1 What the gaps actually were

Not the radio. **The hub was discarding its own audio at the socket.** `tx-fail`
had a per-errno tally already and every one was `ENOMEM`; what it lacked was a
way to say how many of them were *audio*, which is the only kind the room hears.
`s_tx_fail_audio` and a `pkts/s` figure were added for that, and with them the
satellite's gap rate tracks the hub's audio rejections almost one for one.

The measurements, all at two satellites, `fec 0` unless stated:

| configuration | hub tx-fail (audio / 20 s) | satellite gaps |
|---|---|---|
| frames unicast, 26 buffers | 0 | 0.17/s |
| frames unicast, 26 buffers, **FEC 1** | 10–58 | ~0.8/s |
| frames **mcast**, 26 buffers | 14–27 | 1.1/s |
| frames **mcast**, 32 buffers, ML lane off | **0** | **0 in 125 s** |

### 13.2 Two changes that made it worse, and why

**Sizing the audio payload so a whole FEC copy fits.** A depth-1 copy needs
`2 × 825 + 29` bytes against a 1472-byte MTU and cannot fit; capping the payload
to make it fit binds on *every* packet a phone sends, so each became two
datagrams. The audio packet rate went 50 → ~100/s, which exhausted the 26 static
TX buffers — `tx-fail 370 (312 audio)` in one window, ~15% of the hub's audio
gone before it reached the air — and, by a second route nobody predicted from the
packet size, doubled the timeline slew, because `TIMELINE_SLEW_US` is 20 µs **per
packet** and was sized against ~50/s to stay inside the servo's 2.27 ms/s
ceiling. Satellites could not follow: `phase +271874 us`, `anchors refused 40
late`, re-anchoring every few seconds. `hub.h` had predicted exactly this
("anything near that leaves the units unable to keep up and puts the error
back") and it was not read.

**Moving the analysis frames to multicast at 26 buffers.** Argued on airtime —
`96×N` unicast sends becoming ~96 — which is correct arithmetic and the wrong
question. What decides it is the **DTIM burst**: a SoftAP holds group-addressed
frames and releases them after each beacon, while unicast leaves immediately with
rate adaptation. At `dtim_period 1` on a 100 ms beacon, audio alone is ~5 frames
a burst and 86 analysis frames a second makes it ~15, each holding a buffer until
its window opens. The pool empties on the burst, not the average.

Both were bundled into one flash with three other changes, which is why
attribution took four more flash cycles. **One variable per run**; the
`pkts/s` counter that would have caught the first fault in a single log window
was added *after* it.

### 13.3 What worked

- **FEC off.** Depth 1 spent ~5% of the stream to recover ~70% of a 0.3% loss,
  and could not recover it whole at any payload size a phone produces.
- **Concealment instead of silence.** A lost packet now fades out from the last
  audio that played and the resuming audio fades in. The missing ~20 ms cannot be
  invented; the two discontinuities around it — which are most of what a dropout
  *sounds* like — are gone for nothing.
- **The ML lane off the hub** (`DANCEFLOOR_ML_SOURCE_REMOTE`), which returned
  ~24.9 kB of internal SRAM: `MEM: internal` min 6740 → 22012. The Kconfig's
  "about 25 kB" was exact. Nothing was lost — the slow lane had logged
  `slow 0/0 us` and `results N+0` in every run ever captured, producing nothing,
  while `ml-throt ~1450` per window says ~77 of the fast one's results a second
  were computed and thrown away.
- **32 TX buffers**, which that SRAM bought, and which absorbs the burst 26 could
  not.
- **The log shipper off.** Bursty unicast from the same pool, burstiest exactly
  when something is wrong.

### 13.4 Counters added

`s_tx_fail_audio`, `s_audio_pkts` (as `pkts/s`), `n_fec_truncated`,
`n_fec_short_frames`, `n_fec_decode_err`, `n_gap_short_resyncs`, `n_seq_dropped`,
`n_decode_err`, `n_recv_err`, and a DMA-starvation counter on both units from the
I2S driver's own `on_send_q_ovf`.

Two of them corrected faults nobody could see. A gap fill that did not fit the
ring used to under-count `samples_in` and slide the timeline permanently — the
note in `absorb_sequence_gap()` said so and nothing acted on it; measured at
`phase +250217 us` held for over two minutes. It now forces a re-anchor. And the
FEC path called `sbc_decoder_init()` on a copy the MTU had truncated, which is
essentially every recovery, resetting the *shared* decoder's synthesis history
and putting a transient into the live stream on top of the silence.

### 13.5 What is left

- **Nothing past two satellites has ever been run.** The analysis frames provably
  stop scaling; the ~8/s per satellite of clock traffic that still does has never
  been measured at scale. This is the one claim resting on arithmetic alone.
- `TIMELINE_SLEW_US` is still **per packet**. It is correct at the current rate
  and is a trap for anything that changes packetisation again.
- The tracked-versus-flashed trap bit twice in one session: the hub's ML source
  was never in `sdkconfig.defaults`, so a clone built a hub that *ran* the lane,
  and a buffer count was later raised in the defaults without reaching a board.
  The check that catches it is cheap — delete `sdkconfig`, rebuild, read the
  generated header — and is worth running whenever a default changes.

---

## 14. The rate servo's actuator (2026-08-14)

The retune was the last self-inflicted interruption in this unit's audio path.
Everything else it interrupts, it interrupts on purpose: `tx-fail 0`, no
underruns, `dma-starve 0`. A clock retune took the I2S channel down for 1.7–6.2
ms, mean ~3.6 ms, once every 20–45 s — to apply ±4 Hz against ~14 ppm of real
drift.

The fine correction is now made in software, by dropping or duplicating one PCM
frame at a time (`rate_trim_hz`, one frame in ~71,000 at real drift). The clock
keeps the coarse job, which software cannot do. See clock-sync.md §8 for the
split and the reasoning.

### 14.1 What was deliberately left alone

The servo's input, gain, deadband and cooldown are untouched, and
`PHASE_DEADBAND_US` stays at 7000. Widening it is the obvious way to cut retunes
and the wrong one — two units at opposite edges of a wider band are that much
further apart, and cross-unit audio currently measures 0.5–2.5 ms, the best this
project has recorded. The only edit to the loop is that the deadband now compares
the requested trim against the trim already applied, where it used to compare
`desired` against `tx_rate`. Same meaning, different actuator.

`TIMELINE_SLEW_US` is untouched and still correct: the 2.27 ms/s ceiling it is
sized against is `RATE_TRIM_MAX_HZ / 44100`, which did not move, and neither did
the packet rate. This was checked explicitly because §13.5 names it as a trap and
§13.2 is the record of it being sprung.

### 14.2 Process, since §13.2 is the record of getting this wrong

Two flashes, one variable each. The satellite's `samples_played += AUDIO_FRAMES`
had to become consumed-frames before the trim could work, and that is also a
standalone fix to a bias `sat.h` had counted-but-not-fixed for weeks — so it went
in on its own, with the expectation that it changes nothing visible.

The `TRIM:` counters went in **before** either flash, not after. §13.2 records a
doubled packet rate passing through a build, two test suites and a review because
the one counter that would have shown it was added afterwards.

### 14.3 First run on hardware (2026-08-14)

Both units, one track, ~4 minutes. `retunes 0 (0 refused)` on both — the coarse
path was never reached, which is what it should do at a matched 44100.

| | hub | satellite |
|---|---|---|
| phase at playback start | -32555 us | -32435 us |
| phase after ~165 s | -784 us | -1728 us |
| retunes | 0 | 0 |
| underruns / dma-starve / short-reads | 0 | 0 |
| splices | 0 | 0 |

Both walked -32 ms to under -2 ms with no clock retune and no channel-down. The
two tracked each other to within roughly 0.5–1 ms throughout the convergence,
compared at equal times since their own playback start (they started 35 s apart).

The counters matched the arithmetic exactly. The drop/dup rate is `|trim_hz|`
frames per second, and the hub logged 720 frames across the 60 s window where
the trim was -14 then -10 Hz, and 300 where it was -10 then -6.

Every correction was a DUPLICATE; neither unit dropped a frame. Startup phase is
negative on both — the ~30 ms DMA refill — so the servo spent the whole run
asking them to slow down. The drop path is therefore **still untested on
hardware**, which the first run to start late will exercise.

Two things this corrected:

- **The servo line printed the wrong unit.** `1 frame per %ld ms` was fed
  `tx_rate / |trim_hz|`, which is frames between corrections, not milliseconds:
  it read "3150 ms" where the truth was 71. Now `%ld frames/s`, which is
  `|trim_hz|` and is directly comparable with the `TRIM:` deltas.
- **"One frame every 1.6 s" describes steady state only.** Converging from the
  startup phase asks for ~320 ppm, so it runs at ~14 frames/s for the first
  minute or two — twenty times the crystals' own difference, and within striking
  distance of the depth net's 20/s. Nothing was audible at that rate on music,
  which is the closest thing to evidence the net has.
