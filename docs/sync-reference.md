# Synchronisation reference

**What this is:** a lookup table for the synchronisation system — the functions,
the messages, and every parameter with its value, its units and the file that
decides it.

**What this is not:** the reasoning. Almost every constant here was paid for with
a measurement, and the argument for its value lives in the comment above the
`#define` and in [`clock-sync.md`](clock-sync.md). This page tells you *what* and
*where*; those tell you *why*. If a number here disagrees with the code, the code
is right — and the fix belongs here.

---

## 1. The shape of it

The problem is that several battery-powered boards, each with its own crystal,
must put the same sample out of their DACs at the same instant. It is solved in
layers, each of which assumes the one above it is imperfect.

**Layer 1 — agree on what time it is.** Each satellite probes the hub four times
a second (`MSG_TIME_REQ` / `MSG_TIME_RSP`) and keeps a window of ten exchanges.
The offset between the two clocks is the standard NTP estimator, but the reported
answer is the sample with the **lowest round-trip time** rather than an average:
a fast round trip queued least in either direction, so its two path delays were
closest to symmetric, and that asymmetry is the only error the estimator cannot
see. → `clock-sync.md` §2, §3.

**Layer 2 — schedule the future, never the present.** The hub never says "play
now". Every audio packet carries `play_at`, a master-clock instant `LEAD_US`
(250 ms) in the future. Each unit converts it with `sync_to_local()` and waits.
Network jitter stops mattering entirely, as long as the packet arrives before
its instant. → `clock-sync.md` §4.

**Layer 3 — hold position, continuously.** The lead only aligns the *start*.
Crystals differ by ~14 ppm, so every unit measures its own phase error against
the published timeline and trims the rate it *consumes* audio at. Two actuators,
split by size: within `RATE_TRIM_MAX_HZ` playback drops or duplicates one frame
at a time (no outage, continuous); beyond it only a clock retune can help, which
happens about once per stream. → `clock-sync.md` §8.

**Layer 4 — re-anchor at track boundaries.** A proportional loop with a deadband
leaves standing error inside the deadband, and nothing removes it. A track change
is the one instant a splice is inaudible, so at a boundary each unit snaps its
phase to zero by skipping or inserting audio. Both units splice on
`sync_phase_median()`, not a raw reading — two units splicing on independent
noisy samples land in different places. → `clock-sync.md` §8.

**Layer 5 — the faded catch-up.** Newer than the four-layer framing. A burst of
lost packets leaves a unit genuinely late: the audio is gone and the silence that
filled the gap took DAC time. That is 40–150 ms in one stroke, which the fine
trim walks off at 2.27 ms/s — a minute of audible echo. Beyond `CATCHUP_ARM_US`
the servo arms a debt and playback spends it a few frames per chunk, each hidden
under a short crossfade (`audio_shift_chunk()`). → `audio_shift.h`.

> **The lights need no layer of their own.** Every unit runs the same detector
> over audio that is already synchronised, so the strips agree for free. A unit
> can instead draw analysis frames the hub sends (`MSG_FRAME`) — a second route
> to the same agreement, not a replacement. → `architecture.md` §12.

---

## 2. Messages on the wire

All UDP on port `SYNC_PORT` (5001), defined in `sync_proto.h`. Every lane is
unicast. "Client list" means the hub's registered listeners — one `sendto` per
satellite — where the rest are addressed to a single unit.

| # | Type | Struct | Direction | Addressing | Cadence |
|---|---|---|---|---|---|
| 1 | `MSG_TIME_REQ` | `time_msg_t` | satellite → hub | unicast | 4/s (`PROBE_PERIOD_MS`) |
| 2 | `MSG_TIME_RSP` | `time_msg_t` | hub → satellite | unicast — the answer belongs to the unit that asked | one per request |
| 4 | `MSG_AUDIO` | `audio_msg_t` | hub → listeners | client list | ~50/s |
| 5 | `MSG_META` | `meta_msg_t` | hub → listeners | client list | on track change |
| 6 | `MSG_SPLICE` | `splice_msg_t` | satellite → hub | unicast | per track boundary |
| 7 | `MSG_TSF` | `tsf_msg_t` | hub → satellite | unicast | one per probe, 4/s — **measurement only** |
| 8 | `MSG_FRAME` | `frame_msg_t` | hub → listeners | client list | batched: ~9.8 datagrams/s carrying the ~86/s analysis rate |
| 9 | `MSG_LOG` | `log_msg_t` | any unit → collector | unicast via the hub | per log line |
| 10 | `MSG_HEALTH` | `health_msg_t` | any unit → collector | unicast via the hub | ~60 s |
| 11 | `MSG_LOG_SUB` | `log_sub_msg_t` | collector → hub | unicast | every few seconds |
| 13 | `MSG_VOL` | `vol_msg_t` | hub → listeners | **same set as the audio** | 1/s, ×3 on change, once on join |

**Two burned numbers.** `3` was `MSG_BLINK` and `12` was `MSG_ML`. Neither is
reused, deliberately: a board still running old firmware would be speaking a
protocol this one does not, and a silently reinterpreted type is worse than an
unknown one. The same rule burned `AUDIO_FMT_PCM = 0` in `audio_fmt_t`.

**Key fields of `audio_msg_t`** — the packet everything else hangs off:

| Field | Meaning |
|---|---|
| `play_at` | Master-clock instant the **first** sample is due. Converted with `sync_to_local()` |
| `seq` | Matters as much as the timestamp: a lost packet must be filled with exactly the right silence, or every later sample plays early and the whole stream slides |
| `frames` | PCM frames the payload decodes to. The timeline advances by exactly this |
| `restart` | 1 = a track boundary starts here → splice when playback *reaches* it, not when the notification arrives |
| `marker` | 1 = pulse the sync GPIO when this audio plays. Marked by **content**, so unequal playback rates cannot slide the two units' pulses apart |
| `payload_len` | Only these bytes go on the wire; the 2048-byte array is a ceiling, not a per-packet cost |

---

## 3. Functions

### The estimator — `components/dancefloor_sync/` (host-testable, no ESP-IDF)

| Function | Contract |
|---|---|
| `sync_est_init(e)` | Zero the window |
| `sync_est_add(e, t1, t2, t3, t4)` | Fold in one completed exchange. `offset = ((t2−t1) + (t3−t4)) / 2` |
| `sync_est_offset(e, *out)` | Best offset such that `master = local + offset`. Selects the **lowest-RTT** sample, ties to the newer. False below `SYNC_MIN_SAMPLES` |
| `sync_est_settled(e)` | True once a full `SYNC_WINDOW` has landed. Gate anchoring on this — `play_at` is consulted once, so an offset error at that instant is permanent |
| `sync_to_local(master_us, offset)` | `master_us − offset`. The one conversion |
| `sync_phase_reset(h)` | Drop the phase history. Call after anything that moves the unit — a splice, an anchor, a large catch-up drain |
| `sync_phase_push(h, us)` | One accepted reading, µs, **+ = playing late** |
| `sync_phase_median(h, *out)` | Median of what is held; false below `SYNC_PHASE_MIN`. Median, not mean: the residual noise is preemption latency — bounded below, long-tailed right |
| `audio_shift_chunk(dst, src, frames, shift, fade, channels)` | Build one output chunk crossfaded onto a strand `shift` frames away. **`shift > 0` = playing late, skip forward** (consumes `frames+shift`); **`shift < 0` = playing early, hold back** (consumes fewer, replays some). Requires `1 < \|shift\|`, `fade ≥ 2`, `\|shift\| + fade < frames`, non-overlapping buffers |

### The hub — `hub_s3/main/`

| Function | File | Contract |
|---|---|---|
| `streamer_set_sample_rate(hz)` | `out.c` | The rate the timeline advances at. Wrong here means every satellite drifts against the master |
| `streamer_begin_packet()` | `timeline.c` | Call before feeding a packet's audio, so its start can be paired with the `play_at` assigned to it |
| `streamer_send_sbc(sbc, len, frames, marker)` | `timeline.c` | Send one undecoded SBC packet; advance the timeline by `frames` |
| `streamer_feed(pcm, len)` | `timeline.c` | Decoded PCM for the hub's **own** speaker. Non-blocking |
| `streamer_mark_here()` | `timeline.c` | Tag the audio about to be fed as a marker point |
| `streamer_request_restart()` | `timeline.c` | Flag the next packet as a track boundary |
| `streamer_send_vol(v)` / `streamer_set_volume(v)` | `clients.c` | Send the level / change and send it. Addressed to the same set as the audio, by construction |
| `servo_tick()` | `servo.c` | One 5 s window of rate control for the hub's own speaker. Measures, actuates and logs; the loop itself is `df_servo_step()` |
| `local_play_task()` | `play.c` | The hub's speaker: phase crossings, splice, marker, write. The hub delays its own audio by `LEAD_US` like everyone else |

### The satellite — `satellite/main/`

| Function | File | Contract |
|---|---|---|
| `probe_task()` | `clock.c` | Sends `MSG_TIME_REQ` every `PROBE_PERIOD_MS`; also where `MSG_SPLICE` is sent from (never from the audio path — a `sendto()` there costs a buffer) |
| `clock_offset(*out, *used_tsf)` | `clock.c` | Fresh TSF if available, else the probe estimator. `used_tsf` reports which |
| `track_offset()` | `clock.c` | Keep the local→master conversion current, **slewed at `OFFSET_SLEW_PPM`, never stepped**. Holding it fixed feeds the drift back in as the servo's own reference |
| `servo_tick()` | `servo.c` | One 5 s window: smooth the phase, pick an actuator, arm or stand down the catch-up. The decision itself is `df_servo_step()`, shared with the hub |
| `play_task()` | `play.c` | The scheduled start, phase measurement, splice, marker, catch-up spend |

---

## 4. Parameters

### Estimator and clock

| Name | Value | Units | File |
|---|---|---|---|
| `SYNC_PORT` | 5001 | — | `sync_proto.h` |
| `SYNC_WINDOW` | 10 | probes retained | `sync_proto.h` |
| `SYNC_MIN_SAMPLES` | 3 | probes | `sync_proto.h` |
| `SYNC_STEP_US` | 1 000 000 | µs — above this the master's clock changed *origin*, it did not drift | `sync_proto.h` |
| `SYNC_PHASE_HIST` | 9 | readings — odd, so the median is an element | `sync_proto.h` |
| `SYNC_PHASE_MIN` | 5 | readings below which no median is offered | `sync_proto.h` |
| `PROBE_PERIOD_MS` | 250 | ms — min-RTT *holds* its best sample, so a slower probe means a staler one | `sat.h` |
| `OFFSET_SLEW_PPM` | 200 | ppm — ~15× the drift it must follow | `clock.c` |
| `TSF_MAX_AGE_US` | 1 000 000 | µs | `sat.h` |
| `TSF_SPAN_MAX_US` | 100 | µs | `sat.h` |

### Timeline (hub)

| Name | Value | Units | File |
|---|---|---|---|
| `LEAD_US` | 250 000 | µs of presentation lead. Every ms is latency; too little and jitter eats it | `hub.h` |
| `RESYNC_US` | 150 000 | µs the timeline may wander before the slew walks it back | `hub.h` |
| `RESYNC_HARD_US` | 300 000 | µs — past this it is re-anchored rather than slewed | `hub.h` |
| `TIMELINE_SLEW_US` | 20 | µs **per audio packet**. Anything that changes the packet rate changes the slew rate | `hub.h` |
| `SOURCE_STALL_US` | 300 000 | µs of silence that counts as the source having stalled | `hub.h` |
| `SOURCE_STEADY_US` | 500 000 | µs it must run clean before a start — exactly twice the lead | `hub.h` |
| `SOURCE_GIVE_UP_US` | 5 000 000 | µs | `hub.h` |
| `TIMELINE_HOLD_STARVE_MS` | 150 | ms | `hub.h` |
| `TIMELINE_HOLD_GIVE_UP_US` | 30 000 000 | µs | `hub.h` |
| `AUDIO_FRAMES` | 256 | frames/chunk = 5.8 ms at 44.1 kHz, 1041 bytes — under the MTU, so nothing fragments | `sync_proto.h` |
| `MARKER_EVERY_PKTS` | 100 | packets ≈ 2 s | `sync_proto.h` |
| `MARKER_PULSE_US` | 200 | µs | `sync_proto.h` |
| `MAX_CLIENTS` | 15 | the radio's own ceiling. Two other limits must match it | `hub.h` |
| `CLIENT_TIMEOUT_US` | 2 000 000 | µs — covers the ungraceful departure only | `hub.h` |
| `LOCAL_RING_BYTES` | 80 × 1024 | 464 ms. Falsified by the `fed-drop` counter | `hub.h` |

### Servo (both units)

| Name | Value | Units | File |
|---|---|---|---|
| `PHASE_DEADBAND_US` | 7 000 | µs. **Shared, not a per-unit preference** — the worst case between any two units is twice this | `sync_proto.h` |
| `RATE_TRIM_MAX_HZ` | 100 | Hz. Widest trim, **and** the fine/coarse actuator boundary. 2.27 ms/s at 44.1 kHz. **Shared** since 2026-08-19 — it was the same number defined in both headers | `audio_shift.h` |
| servo window | 5000 | ms — `vTaskDelay` in `drift_task` / `ring_monitor_task` | `main.c`, `servo.c` |
| `cooldown` | 4 | windows ≈ 20 s, against a ~40 s correction time | `df_servo.c` |
| EMA weight | 3:1 | `(err_ema * 3 + err_in) / 4`, ~100 s loop | `df_servo.c` |
| `DEPTH_NET_HOLD_US` | 20 000 000 | µs after an anchor during which depth is not evidence | `sat.h` |
| `RING_TARGET_MS` | 250 | ms. **Held equal to the hub's `LEAD_US` by hand** | `sat.h` |
| `RING_BYTES` | `DANCEFLOOR_RING_KB` × 1024, default 80 kB | 464 ms; must cover `LEAD_US + RESYNC_US` | `sat.h` |
| `REFILL_FAST_US` | 1000 | µs — below this the write did not block, so the phase reading is dated against an unpaced reference | `hub.h`, `sat.h` |

The depth term is a **floor, not a replacement**: it only ever strengthens a
phase correction that agrees with it, and wins outright only when the two
disagree (a ring heading for empty or full while phase reads fine).

### Splice and anchor

| Name | Value | Units | File |
|---|---|---|---|
| `MAX_SPLICE_MS` | 150 | ms. A larger error is a bug, not drift | `hub.h`, `sat.h` |
| `SPLICE_INSERT_HEADROOM_MS` | 50 | ms the ring must keep for an insert | `hub.h`, `sat.h` |
| `PHASE_INSANE_US` | 1 000 000 | µs — not the timeline we anchored to | `sat.h` |
| `PHASE_Q_LEN` | 32 | queued crossings | `hub.h`, `sat.h` |
| `ANCHOR_MIN_LEAD_US` | 125 000 | µs = **half the hub's lead**. Whatever lead survives to here *is* the prefill | `sat.h` |
| `ANCHOR_MIN_INTERVAL_US` | 1 000 000 | µs | `sat.h` |
| `ANCHOR_GIVE_UP_US` | 5 000 000 | µs | `sat.h` |
| `GAP_RESYNC_MS` | 150 | ms of gap past which it re-anchors instead of filling | `sat.h` |

### Catch-up (`audio_shift.h`, both units, one implementation)

| Name | Value | Units |
|---|---|---|
| `CATCHUP_ARM_US` | 25 000 | µs — clear of the ±7 ms deadband; the fine trim nulls anything under it in ~11 s |
| `CATCHUP_CLEAR_US` | 10 000 | µs — stand-down. The gap to `ARM` is the hysteresis |
| `CATCHUP_HOLD_US` | 60 000 000 | µs after a (re)start with no arming — the DMA refill transient exceeds `ARM` with nothing wrong |
| `CATCHUP_MAX_US` | 150 000 | µs of debt at once, same as `MAX_SPLICE_MS`. A bigger error re-arms after this drains, so it corrects in faded steps |
| `CATCHUP_SHIFT_MAX_DROP` | 8 | frames/chunk — 1376 frames/s, **31 ms/s** |
| `CATCHUP_SHIFT_MAX_DUP` | 4 | frames/chunk — half that, **deliberately asymmetric** |
| `CATCHUP_FADE_FRAMES` | 64 | frames = 1.5 ms |
| `CATCHUP_HIST_RESET_US` | 20 000 | µs of silent drain before the phase history is rebuilt |

> **Why the one asymmetry here.** A drop bends the pitch up and a replay bends it
> down, by the same amount — but the down bend is the one a listener names ("the
> playing sampling sometimes gets slower", 2026-08-18). A late unit is also
> chasing real lost time, and halving *its* rate doubles the echo it carries
> meanwhile. 688 frames/s still clears a full 150 ms debt in ~10 s.

### Three conventions held equal by hand

Each spans two images, so the compiler cannot check it, and **none of them fails
loudly**:

| Convention | Failure mode if it drifts |
|---|---|
| `RING_TARGET_MS` (sat) = `LEAD_US` (hub) | The servo's depth net holds the unit at the wrong depth — a standing phase error nothing explains |
| `ANCHOR_MIN_LEAD_US` = `LEAD_US` / 2 | Anchors refused, or accepted on too little prefill |
| `PHASE_DEADBAND_US` shared by both roles | Two units deadband around different tolerances; the cross-unit error is no longer twice either one |

---

## 5. Reading the logs

| Line | Says |
|---|---|
| `buffer N ms \| phase +N us (smoothed +N us)` | The servo's 5 s window: ring depth, raw phase, and the EMA it acts on. **+ = playing late** |
| `servo: smoothed +N us -> trim +N Hz (N frames/s)` | A fine correction — software, no outage |
| `servo: smoothed +N us -> COARSE, output N Hz` | A clock retune. Expect one per stream; more means something is wrong |
| `depth net: buffer +N ms, phase asked +N Hz -> +N Hz` | The ring guard raised the correction. Rare by construction |
| `track boundary: skipped/inserted N ms to null phase` | A splice happened |
| `TRACK BOUNDARY: hub spliced +N ms \| hub phase +N us (median ...)` | The hub's own splice, with the median it used and the raw counterfactual |
| `TRACK DIVERGENCE: satellite +N us` | **The number this project measures itself on.** Two units' corrections at the same boundary, subtracted |
| `AUDIO SYNC: satellite +N us (late/early)` | From the marker GPIO — measured on the audio, not inferred from clock estimates |
| `RETUNE COST: phase +N -> +N us (net +N)` | What a clock retune actually cost in phase |
| `REFILL after start: N frames (N ms)` | The unpaced-write window; readings dated inside it are suspect |

Cross-unit figures come in two flavours and they answer different questions:
`AUDIO SYNC` is physical (the marker pin, or a scope on two boards), and
`TRACK DIVERGENCE` is derived (each unit's own correction, over the WiFi that is
there anyway, for every satellite rather than the one that happens to be wired).

---

## 6. Where to read next

- [`clock-sync.md`](clock-sync.md) — the maths, the measurements, what each
  change bought, and what is still unsolved. Start at §1.
- [`architecture.md`](architecture.md) §10 — the same layers in prose, in the
  context of the whole signal path; §13 for every non-default setting.
- `components/dancefloor_sync/include/sync_proto.h` — the wire format, and the
  argument for each constant above.
- `components/dancefloor_sync/include/audio_shift.h` — the catch-up in full.
- `hub_s3/main/hub.h` and `satellite/main/sat.h` — the shared state, and the
  rules about which task may write what. Read these before either `play.c`.
- `make -C components/dancefloor_sync/test check` — the estimator, the phase
  median, the crossfade and the volume taper, on the host.
