# Clock synchronisation between units

How two ESP32s that have never shared a wire agree on when "now" is, to within
a fraction of a millisecond — and why that is the foundation the whole
multi-speaker system rests on.

**Status: measured and passing.** End-to-end error on hardware settled at roughly
**−130 to −430 µs** against a 1 ms budget — a ~260 µs constant bias from path
asymmetry plus ~300 µs of scatter, with no accumulating drift. See §7 for how it
got there and §8 for what remains.

**There are two clock sources, and the one described in §2–§3 is now the
fallback.** A satellite prefers 802.11 TSF when it has a reading less than a
second old, and drops back to the probe estimator otherwise — see §10. Almost
everything below still applies either way: the schedule-the-future design (§4),
the phase servo (§8) and the splices are all indifferent to where the offset
came from. What changes with TSF is that the round trip disappears, and with it
the path asymmetry that §2 identifies as the error floor.

Implemented in `components/dancefloor_sync/` (the estimator and the wire format,
no ESP-IDF dependencies, host-testable). The hub and satellite use the component
directly. The M4 blink harness that first exercised it has been deleted now that
the audio path is the thing under test; its host tests moved into the component
alongside the code they cover.

---

## 1. Three separate problems

Each board runs `esp_timer_get_time()`, which counts microseconds since *that
board* booted. Three things stop you using it directly.

**Different origins.** Measured offset on our hardware was ~124,270,929 µs —
124 seconds. That is not error. It means the master was powered on about two
minutes before the satellite. The clocks are counting from different moments.

**Variable delay.** "Play now" is useless: the message arrives late, and
differently late every time. Measured round-trip on a direct SoftAP link swung
between **4.7 ms and 14 ms**.

**Different rates.** Crystals are not identical. We measured **10.6 ppm** of
relative error between our two boards, within the ±10–40 ppm you should expect.

Problems 1 and 2 are what this document solves. Problem 3 is drift, and it needs
a different fix — see §8.

---

## 2. Measuring the offset

You cannot ask another machine what time it is, because the answer is stale by an
unknown amount before it arrives. The standard trick — this is what NTP and PTP
do — is to record four timestamps around a single round trip.

```
satellite                                   master
    |
t1  |---------- MSG_TIME_REQ --------------->|  t2   arrival, master clock
    |                                        |
    |                                        |      (service time S)
    |                                        |
t4  |<--------- MSG_TIME_RSP ----------------|  t3   reply, master clock
    |
```

| | Whose clock | Meaning |
|---|---|---|
| `t1` | satellite | when the probe was sent |
| `t2` | master | when it arrived |
| `t3` | master | when the reply was sent |
| `t4` | satellite | when the reply came back |

`t2` and `t3` ride back inside the reply, so the satellite ends up holding all
four. The master keeps no state at all — it just stamps and echoes.

### The algebra

Let `OFF` be what we want: `master_clock − local_clock`. Let `U` be the one-way
delay out, `D` the one-way delay back, `S` the master's service time.

```
t2 = (t1 + U) + OFF          arrival in local time, expressed on the master clock
t3 = t2 + S
t4 = (t3 − OFF) + D          reply in master time, expressed on the local clock
```

Subtract:

```
t2 − t1 =  U + OFF           the trip out, plus the offset
t3 − t4 =      OFF − D       the offset, minus the trip back
```

Add them and halve:

```
offset = ((t2 − t1) + (t3 − t4)) / 2  =  OFF + (U − D) / 2
```

**If the two directions took equally long, `U − D` is zero and the result is
exact.** That is the entire basis of the method.

Round-trip time falls out of the same four numbers:

```
rtt = (t4 − t1) − (t3 − t2)  =  U + D
```

Total elapsed on the satellite, minus however long the master sat on it.

### The weakness

The error term is `(U − D) / 2` — **half the path asymmetry**. If the outbound
trip took 5 ms and the return took 1 ms, you are 2 ms out.

Nothing in the measurement can detect this. The satellite observes `U + D` and
can never separate the halves. Asymmetry is invisible, and it is the reason the
target here is 1 ms rather than 10 µs.

### The other weakness: the master's clock can change identity

The window spans 2.5 s and minimum-RTT selection is free to pick any sample in
it. If the master reboots or is reflashed, its clock restarts from a new origin,
and for those 2.5 s the window holds samples from **two different clocks**.
`count` never decreases, so `sync_est_settled()` stays true throughout and a
satellite will anchor on whichever sample the selection liked — possibly one
from before the restart, wrong by the master's entire previous uptime.

Measured on hardware: 3595 seconds of it. The satellite scheduled playback for
an instant that had already passed, every phase reading was an hour out, the
`int32_t` it was cast into wrapped to −699 s, the smoothing overflowed on top,
and the servo asked for a 4.29 GHz sample rate. `ESP_ERROR_CHECK` turned that
into an abort, so the room heard a speaker rebooting in a loop.

`sync_est_add()` now discards the window when consecutive probes disagree by
more than `SYNC_STEP_US` (1 s). Drift moves the offset by microseconds in a
quarter second and asymmetry by milliseconds, so nothing legitimate reaches it.
`count` falls below `SYNC_MIN_SAMPLES`, `sync_est_settled()` goes false, and
playback holds for 2.5 s until a clean estimate exists — which is what that
mechanism was already for. The new sample seeds the fresh window, so a single
corrupt probe costs two samples rather than wedging the estimator. Both are
pinned in `test_sync_proto.c`.

---

## 3. Why we select on minimum RTT

The first implementation took the **median** of ten probes. That handles WiFi
retries well — a probe delayed 80 ms doesn't move a median at all — but it
averages good and bad samples together.

The measured 4.7–14 ms RTT spread argues for something better. A probe that came
back *fast* had little queuing in either direction, so its `U` and `D` were close
to equal, so its `(U − D) / 2` error was small. A slow probe was delayed
somewhere, and you don't know which side.

So: **keep the sample with the lowest RTT and discard the rest.** This is what
PTP does.

The effect on real hardware was large:

| Selection | Offset spread |
|---|---|
| Median of 10 | ~1080 µs |
| Minimum RTT | **~117 µs** |

`test/test_sync_proto.c` case 7 isolates why: nine badly asymmetric probes plus
one clean fast one gives 0 µs of error with min-RTT, against ~2250 µs from a
median.

### The cost, and why the probe period is 250 ms

Min-RTT is a *hold-the-best* estimator, so the value steps rather than tracking
smoothly, and a held sample ages. With ten probes at 1 s intervals the window
spans 10 s, and at ~14 ppm a 10-second-old sample carries ~140 µs of staleness —
growing the whole time it is held.

This showed up clearly in the end-to-end measurement. With a 1 s probe period the
sync error slid in a dead straight line:

```
-151  -173  -195  -217  -239 µs      +22 µs every 2 s  =  14 µs/s
```

That gradient is the crystal drift, not estimator noise. `PROBE_PERIOD_MS` is
therefore 250 ms: the same ten samples to choose from, but a 2.5 s window instead
of 10 s, cutting staleness fourfold for four tiny packets a second.

Note this only shortens the *staleness*. It does not stop the clocks diverging —
that is M6's job (§8).

---

## 4. The actual trick: schedule the future

The maths above is the fiddly part. This is the part that makes it work, and it
is much simpler.

**The master never says "play now."** Every packet says *"play this at
master-time T"*, where T is `LEAD_US` — 250 ms — ahead. Each satellite converts
with `local = T − offset` and waits. (The M4 harness said "blink at T" instead;
the sentence is the design, and the audio path inherited it unchanged.)

```c
static inline int64_t sync_to_local(int64_t master_us, int64_t offset)
{
    return master_us - offset;
}
```

Network jitter stops mattering. The announcement can take 4 ms or 14 ms; as long
as it arrives *before* T, every unit fires at the same instant.

You are not synchronising the event. You are synchronising the **schedule**, and
letting each board independently keep its own appointment.

A useful consequence: nothing passes between the boards at the moment of firing.
Each one keeps its own appointment independently. The measurement wire described
in §12 observes the result; it plays no part in producing it.

The cost is latency. Nothing can happen sooner than the lead time, so the
satellite holds a ring around a 250 ms target to match (`RING_TARGET_MS`, held
equal to the lead by hand), and total end-to-end delay is ~150–200 ms of
Bluetooth plus that — roughly half a second. Fine for
music, useless for video.

---

## 5. Getting the timestamps right

The estimator is only as good as its stamps, so:

- `t2` is taken **immediately** after `recvfrom` returns, before any parsing.
- `t3` is taken **immediately** before `sendto`, so service time is excluded.
- The task waiting for the scheduled instant wakes 2 ms early and then busy-waits
  to the deadline. Sleeping straight there would fold FreeRTOS scheduler jitter
  into the measurement. This started in the M4 blink task and is now how the
  satellite holds its first sample; after that, I2S paces everything.
- `CONFIG_FREERTOS_HZ=1000`. At the default 100 Hz, `vTaskDelay` granularity is
  10 ms — ten times the entire error budget.
- `esp_wifi_set_ps(WIFI_PS_NONE)`. Power save parks the radio between beacons and
  adds tens of milliseconds to exactly the packets being timed.

Any one of these omissions is enough to make the result meaningless.

---

## 6. Reading the logs

```
I (21746) sync: offset 124270929 us (rtt 7700 us)
```

- **offset** — how far ahead the master's clock reads. The absolute value is just
  the boot-time difference and carries no information. What matters is that it is
  *stable*.
- **rtt** — this probe's round trip. Lower means a more trustworthy sample.

Healthy behaviour is a value that settles and then creeps slowly in one
direction. Wandering in both directions means the estimator is still noisy;
jumping by milliseconds means probes are being lost or badly delayed.

---

## 7. What changed, and what each change bought

The first working version was not the one that shipped. Three changes took it
from "roughly right" to comfortably inside budget, each driven by a measurement
rather than a guess.

| # | Change | Problem it addressed | Measured effect |
|---|---|---|---|
| 1 | Median → **minimum-RTT** selection | RTT swung 4.7–14 ms; a median blends trustworthy and untrustworthy probes together | Offset spread **1080 µs → 117 µs** |
| 2 | Probe period **1 s → 250 ms** | Min-RTT holds its best sample until beaten, so a 10-probe window meant up to 10 s of staleness | Error stopped accumulating: a monotonic −22 µs-per-announcement ramp became bounded scatter |
| 3 | **GPIO loopback monitor** | Verification needed an oscilloscope nobody had | M4 became measurable with one jumper wire |

### 1. Selecting instead of averaging

The original estimator took the median of ten probes. That rejects outliers well
— an 80 ms WiFi retry doesn't move a median at all — but it still mixes a clean
2 ms round trip with a congested 14 ms one, and the congested one carries far
more asymmetry error.

Minimum-RTT keeps the single fastest probe and discards the rest, on the
reasoning in §3: a fast round trip had little queuing *in either direction*, so
its two halves were close to equal. Test case 7 isolates the difference — nine
badly asymmetric probes plus one clean fast one gives 0 µs of error with min-RTT
against ~2250 µs from a median.

### 2. Shortening the window

Min-RTT introduced a new failure mode: it is a *hold-the-best* estimator, so a
lucky early sample stays the winner until it ages out. On hardware this appeared
as a perfectly straight ramp of −22 µs per announcement — the crystals diverging
while the estimate sat frozen.

The fix was not a better algorithm but a shorter window: same ten samples,
gathered over 2.5 s instead of 10 s. The ramp disappeared.

This one is worth remembering as a pattern. The first change traded *jitter
rejection* for *freshness*, and the second paid the freshness back. Neither is
visible without measuring on real hardware.

### 3. Making it measurable at all

Wiring the satellite's blink output into an input pin on the master turned a
milestone that was blocked on test equipment into one verifiable with a jumper
wire, and produced the data that drove change 2. Worth doing early: a milestone
you cannot measure is a milestone you cannot finish. The harness is gone and the
same wire now carries the audio marker instead (§12), which is the same idea
pointed at the thing that actually ships.

### Bugs fixed along the way

- `recvfrom`'s address-length argument was hoisted out of the master's receive
  loop. It is written back on every call, so every packet after the first saw a
  shrinking value.
- `CONFIG_DANCEFLOOR_ROLE_MASTER` was used directly as a C expression. A Kconfig
  `bool` set to `n` is left **undefined**, not defined as `0`, so the satellite
  build failed to compile while the master built fine.
- Marker and monitor pins became per-board `menuconfig` options. Board
  silkscreens disagree — `G4` is GPIO 4 on one board, `D4` is emphatically not on
  another.

### What was deliberately not done

The residual error is a **~260 µs constant bias** (the satellite fires early)
plus ~300 µs of scatter. The bias is almost certainly path asymmetry (§2), and
now that the loopback wire measures true error it could simply be calibrated out
with a constant.

That was rejected. 260 µs is **9 cm of sound travel** — less than a listener
shifting their weight, and orders of magnitude below what speaker placement
contributes. The constant would also be specific to this pair of boards and this
access point, so it would look precise while meaning nothing. The system is at
~300 µs against a 1 ms budget and a ~5 ms audibility threshold; further tuning
optimises a term nobody can hear.

---

## 8. Holding position, not just rate

Aligning the start instant and matching rates is not enough. A servo on buffer
depth keeps the playback *rate* equal to the arrival rate, but says nothing
about *position* -- and depth moves with network jitter, so each unit nudges its
rate in response to noise. Two units seeing different jitter end up with rates
differing by ~0.03% at any moment, which is several ms of relative movement.

Measured: 10-25 ms of wander between hub and satellite while each unit's own
buffer sat perfectly stable at ~200 ms — its target then, 250 now — with the
satellite's rate visibly
hunting (44100 -> 44113 -> 44100 -> 44086).

Every packet already carries the instant its first sample is due. Recording that
against the ring position the audio lands at gives a direct phase reading when
playback arrives there: where we are, versus where the timeline says we should
be. Both units servo on that instead, the hub included -- it publishes the
timeline, so it must hold itself to it or the reference itself moves.

Buffer depth is kept only as a guard against running empty or overflowing, which
phase control alone would not see coming.

### Measured

| | Result |
|---|---|
| Clock offset between boards | ~300 us (M4, two independent methods) |
| Playback start vs schedule | +6 us (hub), +0 us (satellite) |
| Phase at startup | -42 ms (hub), -26 ms (satellite) |
| Phase after ~45 s | under 1 ms (hub), ~2 ms (satellite) |
| Hub-to-satellite audio | 10-25 ms before, **2-4 ms** after |

The startup offset is output pipeline latency -- DMA depth plus where playback
began within the buffer -- which nothing accounts for at anchor time. The servo
absorbs it over ~45 s. Compensating it at anchor would start near zero instead,
and is the obvious next improvement.

### Track-boundary re-anchoring

The servo alone walks error off over a couple of minutes, which leaves the start
of a session -- and any recovery -- audibly out. Correcting faster means skipping
or inserting audio, and that is a splice.

A track change is the one moment a splice is inaudible, and AVRCP reports it
rather than making us infer it. Silence detection was considered and rejected: a
quiet passage would trigger it and splice something audible, whereas a track
change is unambiguous.

The correction is applied when playback *reaches* the flagged packet, not when
the notification arrives -- at that moment the buffer still holds ~200 ms of the
previous track, and correcting immediately would cut its ending.

Measured over a session, corrections shrink as the servo converges:

```
track boundary: skipped 8 ms to null phase
track boundary: skipped 2 ms to null phase
```

### Settled behaviour

After convergence, with track-boundary corrections active:

| | Value |
|---|---|
| Smoothed phase | +4 to +12 ms standing offset, stable within ~3 ms |
| Instantaneous phase | +-5 ms (satellite), +-15 ms (hub — see the wart) |
| Buffer | 165-250 ms around the 200 ms target of the day (250 since) |
| Hub-to-satellite audio, just after a splice | **0.1 to 0.5 ms** |
| Hub-to-satellite audio, late in a track | 2 to 9 ms |
| Playback start vs schedule | +5 us (hub), +1 us (satellite), same instant |

The cross-unit figure is quoted twice on purpose. A track boundary nulls phase
on both units, so the error resets to near zero at every track change and grows
between them — `+99 µs` two seconds after a splice, several ms by the end of a
long track. Quoting a single number for it invites exactly the mistake of
comparing two log windows taken at different points of that cycle, which is how
a regression was diagnosed here that did not exist.

The growth between boundaries is deadband-bound: each unit tolerates
`PHASE_DEADBAND_US` (7 ms) of its own error before correcting, so the worst case
between two of them is twice that, and the observed 2-9 ms sits inside it.

The smoothed phase no longer converges to zero, and that is inherent: this is a
proportional loop with a deadband, so once the rate is right the position error
accumulated on the way there is inside the deadband and nothing removes it. The
splice does.

Servo gain matters more than it looks. At a 40 s correction time both units
converged and then overshot to +10 ms, oscillating rather than settling -- the
buffer takes tens of seconds to respond, so the loop was still correcting after
the error had gone. At ~100 s it settles. Real drift is ~0.8 ms per minute, so
the loop can afford to be far gentler than the disturbance it corrects.

### Known wart

The hub's absolute phase reading does not settle. It is now quantified: two
reads of the same variable, one millisecond apart, in adjacent log lines.

```
local ring ... | phase +26786 us (smoothed +8996 us)
servo: phase +11108 us (smoothed +8996), ...
```

**15.7 ms of swing between consecutive samples**, with the average sitting still
at +9 ms through it. Another pair moved 8.7 ms. The satellite's readings are far
quieter, +-5 ms, so this is the hub's own measurement and not something shared.
The cause is still not found.

Two consequences worth knowing:

- The servo used to act on that raw number directly, so it was substantially
  triggering on measurement noise. Both units then servoed on a 4-sample EMA
  (§8), which cut the hub's retunes about fourfold and changed the cross-unit
  figure not at all — the excursions between splices are the same either way, so
  they are not a servo-input artefact.

  **The hub now feeds that EMA a median rather than the raw reading.** The EMA
  was the right filter on the wrong input: it averages across servo ticks 5 s
  apart, so rejecting scatter that lives inside 180 ms costs it tens of seconds
  of memory, and an average only mixes an outlier in where a median discards it.
  The play task publishes the median of the same nine packet-cadence readings the
  splice has used since it stopped splicing on one sample, and the servo's EMA
  runs on that. The deadband was deliberately **not** widened — that would have
  been the obvious way to cut retunes and the wrong one, since two units at
  opposite edges of a wider band are that much further apart.

  Honest reading of the result: on the runs since, `phase` and `median` track
  each other to within a few µs (`phase -1443 us (median -1439, smoothed ...)`),
  so the 15.7 ms scatter is not present in them and the filter is not currently
  earning its place. It is cheap, it cannot hurt, and the status line prints raw,
  median and average together precisely so this stays checkable.
- Any single-sample figure derived from the hub's phase is untrustworthy,
  including the per-retune costs it prints. The satellite's are usable; the
  hub's are only meaningful averaged over many.

Treat the cross-unit measurement as the meaningful one, and the hub's absolute
phase as indicative only.

### What a retune costs

Retuning the output clock is not free, and until it was measured the cost was
guessed at twice and wrongly both times.

`CONFIG_DANCEFLOOR_RETUNE_BENCH_S` forces a retune to the rate already set, on a
timer. Nothing about the audio legitimately changes, so everything it costs is
the cost of retuning itself, with no rate change and no drift in the way. On the
satellite, whose phase reading is quiet enough to trust:

| channel down | net phase step |
|---|---|
| 2330 us | +2421 us |
| 5818 us | +6871 us |
| 2878 us | +1156 us |
| 1821 us | +3468 us |
| 5150 us | +1113 us |

Mean net 3.3 ms against a mean outage of 3.1 ms. **The step is the outage**, to
within the several ms the phase wanders on its own.

It was not always. The same bench before the fix below produced +42, +43 and +50
ms steps, with 5432 bytes overflowing the visualiser's buffer and nine
re-alignments in a single window. `i2s_channel_write()` returns *immediately*
once the channel is disabled — it does not block and does not write — and the
satellite's `dac_write()` ignored the return value. So for the whole outage the
playback task pulled chunks from the ring at memory speed, counted every one in
`samples_played`, fed every one to the visualiser, and threw them at a channel
that was not running. Milliseconds of outage cost tens of milliseconds of buffer.

The hub had been guarded against this since "a measured 54 ms correction cost
177 ms of buffer". The satellite never was, and every retune it had ever
performed paid for it. It holds `retuning` and parks its play task now, exactly
as the hub does.

The discarded DMA buffer — audio counted as played that never reached the
speaker — remains unmeasured, and by construction always will be from inside the
firmware: every reading derived from `samples_played` agrees those frames were
played. Only the marker GPIO can see it.

### Paying it once per stream instead of twice a minute

That cost is now mostly not paid at all. Across many runs the outage measured
**1.7 to 6.2 ms, mean ~3.6 ms, once every 20–45 s per unit** — to apply ±4 Hz
against ~14 ppm of real crystal drift. On the hub it was the last remaining
self-inflicted interruption in the audio path: `tx-fail 0`, no underruns,
`dma-starve 0`, everything else it interrupts it interrupts on purpose.

Since 2026-08-14 the servo has two actuators and picks between them by size:

- **Fine, `|trim| ≤ RATE_TRIM_MAX_HZ` (100 Hz).** `rate_trim_hz` names a rate the
  DAC is *not* running at, and playback consumes the ring at that rate instead —
  dropping one frame when it needs to get through the stream faster, duplicating
  one when it needs to get through it slower. At 14 ppm that is one frame in
  ~71,000, about one every 1.6 s. No channel-down, and continuous where the
  clock was stepped.
- **Coarse, beyond it.** The clock still moves, because software cannot absorb a
  source measured at ~42,600 against a 44,100 output — 40,000 ppm drains a 250 ms
  buffer in five seconds. On the hub that is `streamer_set_sample_rate()`'s 1%
  test; on the satellite it is the servo itself, because `i2s_start()` runs with
  a hardcoded 44100 before any stream exists, so the first match arrives as a
  ~1500 Hz step. Coarse retunes happen about once per stream.

**The control loop did not change.** Same input, same gain, same
`PHASE_DEADBAND_US`, same cooldown; the deadband simply moved from comparing
`desired` against `tx_rate` to comparing the requested trim against the trim
already applied. That was deliberate — the point was to remove an interruption
without moving the sync behaviour, so that a change in `TRACK DIVERGENCE` could
only be attributed to the actuator.

Two consequences worth recording:

- **The deadband's floor is gone.** It was "retuning happens in whole Hz, 22.7 ppm
  at 44.1 kHz, so ~2.3 ms is the smallest expressible correction". That was a
  property of the clock. The software path has no such floor, which is what makes
  tightening `PHASE_DEADBAND_US` — the lever named in §9 as affordable-but-untested
  — a genuinely one-line experiment now. It is deliberately *not* taken here.
- **`TIMELINE_SLEW_US` is unaffected.** The 2.27 ms/s ceiling it is sized against
  is `RATE_TRIM_MAX_HZ / 44100`, and that is unchanged, as is the packet rate.

`samples_played` advances by frames **consumed from the ring** — 257 on a drop,
255 on a duplicate. It is a position in the ring stream, compared against
`samples_in`, so it must count what was consumed rather than what was handed to
the DAC; getting that backwards would make the servo chase its own correction.
The hub always counted it that way. The satellite did not, and was fixed first
and separately, because it also had the short-read bias `n_short_frames` counts.

`TRIM:` on each unit's console is the instrument: the trim the servo asked for,
and the frames playback actually dropped and duplicated. They are the only pair
that can disagree.

---

## 9. What is still unsolved

**Drift.** Our two boards differ by 10.6 ppm, so the offset does not hold still:

| Elapsed | Divergence if uncorrected |
|---|---|
| 1 minute | 0.6 ms |
| 10 minutes | 6.4 ms |
| 1 hour | 38 ms |
| 4-hour party | **153 ms** |

Re-probing tracks this fine for blinking an LED, because you can simply jump to
the new offset. **Audio cannot jump** — that is an audible click, and at 44.1 kHz
one board is playing ~10.6 extra samples per million.

So M6 does not correct the clock. It corrects the *playback rate*: dropping or
inserting a sample occasionally, at a zero crossing, so the slower unit catches
up inaudibly.

**This is solved now, and the way it stayed broken is the interesting part.**
The satellite converted local time to master time with the offset it measured
once, when playback anchored, and then held it. That offset goes stale at
exactly the crystal difference — so the phase measurement was biased by exactly
the drift, and the servo faithfully parked the speaker at the growing error
instead of removing it. The drift the servo existed to correct was being fed
back in as its own reference.

Nothing in the logs showed it, which is why it survived: the marker pulse is
derived from the same conversion, so the cross-unit measurement read correct
while the sound slid. `stream_offset` is now slewed toward the live estimate at
200 ppm — roughly 15x the drift it has to follow, slow enough that minimum-RTT
noise is averaged away rather than mistaken for a real position error.

Two other things were quietly feeding the same servo bad numbers:

- Silence inserted for a lost packet went into the ring without being counted in
  `samples_in`, the position every marker and phase point is recorded against.
  One lost packet put all of them ~20 ms early, permanently, and again on the
  next loss. The servo then held the speaker at that error. Invisible for the
  same reason as above — the marker fires off the same skewed count.
- The hub queued a phase point captured before a timeline restart reset the
  ring, so on an underrun recovery it referred to an origin that no longer
  existed. Playback never reached it, the queue filled, and `s_phase_valid`
  stayed false for the rest of the session: ring servo stopped, and the
  visualiser was fed a due of zero forever.

**Asymmetry.** Sustained path asymmetry is invisible to the estimator (§2) and no
selection strategy removes it. It is the error floor. Test case 5 in
`test_sync_proto.c` pins the behaviour deliberately: 2 ms up against 200 µs down
produces exactly 900 µs of error.

**Mid-track divergence.** Cross-unit error resets at every track boundary and
grows to a few ms by the end of a long track. That growth is deadband-bound, so
the lever is `PHASE_DEADBAND_US`, and tightening it is now affordable in a way
it was not — twice over. A retune costs a few ms rather than fifty and no longer
drains the ring; and since the software rate trim (§8) a *fine* correction costs
no outage at all, which also removes the whole-Hz floor of ~2.3 ms that used to
sit under the deadband. Both obstacles are gone and it is still **untested**.
Before touching it, put a per-track divergence number in the logs — the hub and
satellite each print their splice size at a boundary, and the difference between
those two is how far apart they had drifted, condensed to one figure per track.
That is the metric to compare builds on; a glance at a log window is not, and
twice here it produced a confident diagnosis of a regression that did not exist.

**The hub's phase noise.** 15.7 ms between consecutive reads, cause unknown. It
is filtered rather than understood. Most of it turned out to be quantisation --
phase was dated where the playback loop NOTICED a crossing rather than where the
crossing happened, and samples_played moves 256 frames at a time, so the reading
was up to 5.8 ms late by an amount uncorrelated between samples. Correcting that
exactly (the overshoot is known, and writes are paced by the DAC) roughly halved
the scatter. It did not eliminate it, and the remainder is unexplained.

---

## 10. The other clock: 802.11 TSF

Everything in §2 and §3 measures the offset with a round trip, and §2 says why
that has an error floor: the estimator observes `U + D` and can never separate
the halves, so sustained path asymmetry is invisible and unremovable.

TSF has no round trip in it. The AP maintains a 64-bit microsecond counter,
every beacon carries it, and each associated station's **MAC hardware**
timestamps the beacon on arrival and slaves its local copy. That hardware
timestamp is what real PTP's precision actually rests on, and what a stamp
either side of a `sendto()` cannot be — everything between "read the clock" and
"the frame left" lands in the error budget.

Each unit relates its own TSF to its own `esp_timer`, and because both track the
same AP counter:

```
offset = (master_local − master_tsf) − (sat_local − sat_tsf)
```

The master sends its `(tsf, local)` pair on the back of each probe reply
(`MSG_TSF`, 17 bytes, four times a second per satellite). No network delay
appears in the result, so no asymmetry can.

### Measured against the estimator

Both were computed side by side for two long runs before either was trusted.

| | |
|---|---|
| Offset agreement | ~450 µs, **stable bias**, no trend over 10 min |
| **Rate agreement** | **0.12 ppm** over 9.7 minutes |
| Sample-to-sample step, TSF | 1–80 µs |
| Sample-to-sample step, estimator | 50–200 µs |
| Playback anchors at | 4.06 s, against 5.8–11 s before |

The rate figure is the one that matters. Two methods built on entirely different
physics, drifting at −3.55 and −3.43 ppm respectively — that is the hub-satellite
crystal difference, and they agree on it to a tenth of a ppm. The ~450 µs offset
bias sits about where §2 puts the estimator's own asymmetry error, so it is as
likely to be the estimator's as TSF's.

The faster start is not a bonus, it is structural: the 2.5 s settling wait exists
because minimum-RTT selection needs a full window to find an uncongested round
trip. TSF is not built from round trips, so there is no congested sample to
average away and one beacon is as good as ten.

### What it did not buy

**Cross-unit audio was unchanged**, 0.1–10 ms either way. That is the figure a
listener experiences, and TSF did not move it. What it removed is a class of
failure — the asymmetry floor, the settling wait, and a master reboot poisoning
the window — rather than anything audible.

### The fallback, and why it stays

`clock_offset()` in `satellite/main/main.c` prefers TSF while a reading is under
`TSF_MAX_AGE_US` old and falls back to `sync_est_offset()` otherwise. It has to:
`esp_wifi_get_tsf_time()` returns 0 before association and before the first
beacon, and a master that does not send `MSG_TSF` at all would leave a satellite
with nothing to anchor on.

Everything in §2's "other weakness" therefore stays — `SYNC_STEP_US`, the window
discard, the settling wait. They guard the fallback path, and TSF resets on
reassociation, which is exactly the discontinuity the phase-insane re-anchor
handles.

### Caveats worth carrying

The measurement is not free: `esp_wifi_get_tsf_time()` takes ~65–125 µs, so the
read pair is bracketed by two TSF reads and the `esp_timer` read is centred
between them. The span is reported beside every sample.

TSF also steps ±300–600 µs occasionally, always as a jump and an immediate
return. It is not read skew — the span is 130–240 µs on essentially all of them.
The likely cause is the beacon-driven correction re-slaving the local counter
every 102.4 ms. They self-cancel and the servo sees the average.

`git tag probe-estimator-clock` marks the last commit before the switch.

---

## 11. Code map

| File | Responsibility |
|---|---|
| `components/dancefloor_sync/include/sync_proto.h` | Wire format, estimator state, `sync_to_local()` |
| `components/dancefloor_sync/sync_proto.c` | Offset maths and min-RTT selection. No ESP-IDF deps |
| `components/dancefloor_sync/include/audio_shift.h` | The faded catch-up: the clamps, and `audio_shift_chunk()` |
| `components/dancefloor_sync/audio_shift.c` | The crossfade itself. Shared, so the two units cannot disagree |
| `components/dancefloor_sync/test/` | Host tests — `make check` |
| `satellite/main/clock.c` | `clock_offset()` picks TSF or the estimator; `track_offset()` slews it. Was `main.c` until the satellite was split on 2026-08-12 |
| `components/dancefloor_sync/df_servo.c` | The rate-control loop itself. Shared, so the two units cannot disagree about how fast either corrects |
| `satellite/main/servo.c`, `hub_s3/main/servo.c` | What each role measures, actuates and logs around it |
| `satellite/main/play.c`, `hub_s3/main/play.c` | Phase crossings, the splice, the marker, the catch-up spend |
| `hub_s3/main/timeline.c` | The published timeline: `play_at`, the slew, boundary flags |

[`sync-reference.md`](sync-reference.md) is the lookup table for all of it — every
function and parameter with its value and its file, without the reasoning.

The estimator moved into a component when the hub and satellite came to need it
too, and its tests moved in alongside it when the M4 blink harness that used to
host them was deleted — the same shape as `components/dancefloor_leds/test/`,
where every suite sits next to the code it exercises.

The estimator is deliberately free of platform dependencies. It is the part most
likely to be subtly wrong, and hardware bring-up is a bad place to discover that.

**Everything the clock itself uses is unicast** — probes, time responses and the
TSF message all go to the station that asked. That is the right shape for them
whatever else the socket carries: a probe is a question from one unit, its
answer belongs to that unit, and there are only 4 a second of each per satellite.

The socket is shared with traffic that is *not* unicast any more, and the history
is worth keeping straight because it reversed. The M4 harness announced over
multicast on the reasoning that one transmission feeding every satellite keeps
airtime independent of unit count; the measurement went the other way, since
group-addressed frames are never acknowledged and so never retried, which put a
~20% loss floor under the audio. Multicast was removed from the whole system and
`SYNC_MCAST_ADDR` went with the harness.

**Audio and analysis frames are group-addressed again**, to `239.0.0.1`. The 20%
was measured at the 1 Mbps basic rate that 802.11b forces; with 11b dropped the
group goes at 6 Mbps OFDM and loss is 0.2–0.3%. See `architecture.md` §4 for what
else had to be true first. Nothing about the clock changed — the estimator still
sees only unicast round trips, and the reason a satellite must keep probing is
still that probing is what puts it on the metadata list.

---

## 12. Verification

**Estimator** — `cd components/dancefloor_sync/test && make check`. Cases covering exactness on
symmetric paths, outlier rejection, the asymmetry floor, window ageing, min-RTT
selection, and the master changing clock origin — that last group asserts the
window is discarded, that no estimate is offered from what survives, that it
re-settles on the new origin, and that one corrupt probe does not wedge it.

**Retune cost** — set `CONFIG_DANCEFLOOR_RETUNE_BENCH_S` to 30 on **one** unit,
leaving the other as a reference, and read the `RETUNE COST` lines. It forces a
retune to the rate already set, so what it reports is the cost of retuning with
the rate change and the drift removed from the experiment. Set it back to 0
afterwards; the costs accumulate faster than the servo absorbs them and the
audio degrades over a long run.

A stable offset in the log means the satellite *believes* it is synced. It does
not prove the audio lands where it should — that needs an independent
measurement, and there are two ways to get one.

This used to be done with the M4 blink harness, which announced "flash at time T"
and timed the result. That harness is deleted; what replaced it measures the
audio path itself, which is the thing under test now.

### Self-measurement, no instruments needed

The hub can time the satellite itself, using the **audio marker**:

```
   SATELLITE                              HUB
   GPIO 4  marker (output)  ───────────>  GPIO 21  monitor (input)
   GND                      ───────────   GND
```

Roughly every 2 s the hub flags one audio packet with `marker`. Every unit that
plays that packet — the hub included — pulses its marker pin at the instant the
audio reaches the DAC. The hub already knows when its own pulse happened, so the
gap between that and the observed edge **is** the audio sync error, in a single
time base with no clock conversion involved. It prints:

```
W (34521) stream: AUDIO SYNC: satellite +83 us (late)
```

Wire propagation is nanoseconds and GPIO interrupt latency a few microseconds,
both negligible at this scale. Both pins are under `menuconfig` → *Dancefloor
hub* / *Dancefloor satellite*; enable `DANCEFLOOR_ENABLE_MARKER` on each.

> This is one **output** into one **input**. Do not connect the two boards'
> *marker* pins together — those are both outputs, and tying them makes one drive
> against the other.

Readings beyond ±500 ms are discarded rather than reported: markers are 2 s
apart, so a gap that large is a missed pulse and calling it a sync error would
mislead. The reading is taken on every marker but printed on
`CONFIG_DANCEFLOOR_LOG_PERIOD_S`, since one every 2 s is more than anyone reads —
the value is kept for the track-boundary summary either way.

Nothing corrects on this. Every servo and every splice closes through `play_at`
and the phase queue, so enabling or disabling the instrument changes no
behaviour — a correction loop closed through an instrument absent in deployment
would be tuned to a configuration nobody ships.

Without any wiring at all, `TRACK DIVERGENCE (wifi)` reports the same quantity
once per track from the splices each unit reports back over WiFi. Take that one
in preference to a figure scavenged from log windows: it is sampled at a track
boundary, the one instant that recurs identically in every track, so it is
comparable across tracks, sessions and builds. Cross-unit error resets there and
grows until the next one — §8.

### With a scope

Probe the marker GPIO on both units with a **common ground**, trigger on the hub.
Pass is rising edges within 1 ms.

Worth doing if you have one, since it measures the pins directly and shares no
code with the thing being tested.

There is a second marker for the *light* path, `DANCEFLOOR_ENABLE_LED_MARKER` on
GPIO 2, which flashes once per second of master-clock time on every unit. It
exists because every measurement above covers the audio only: between playback
and the pixels sit the analysis stream buffer, the FFT, the detector and the
render, and nothing else reports on any of it. Two boards side by side settle it
with no console and no wiring.
