# Clock synchronisation between units

How two ESP32s that have never shared a wire agree on when "now" is, to within
a fraction of a millisecond — and why that is the foundation the whole
multi-speaker system rests on.

**Status: measured and passing.** End-to-end error on hardware settled at roughly
**−130 to −430 µs** against a 1 ms budget — a ~260 µs constant bias from path
asymmetry plus ~300 µs of scatter, with no accumulating drift. See §7 for how it
got there and §8 for what remains.

Implemented in `sync_test/main/sync_proto.{c,h}` (the estimator, no ESP-IDF
dependencies, host-testable) and `sync_test/main/main.c` (WiFi, sockets, tasks).

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

**The master never says "blink now."** It says *"blink at master-time T"*, where
T is 500 ms ahead. Each satellite converts with `local = T − offset` and waits.

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
in §10 observes the result; it plays no part in producing it.

The cost is latency. Nothing can happen sooner than the lead time, which is why
M5 budgets 300 ms of audio buffer and accepts ~0.5 s of total delay.

---

## 5. Getting the timestamps right

The estimator is only as good as its stamps, so:

- `t2` is taken **immediately** after `recvfrom` returns, before any parsing.
- `t3` is taken **immediately** before `sendto`, so service time is excluded.
- The blink task wakes 2 ms early and then busy-waits to the deadline. Sleeping
  straight there would fold FreeRTOS scheduler jitter into the measurement.
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

Wiring the satellite's blink output into an input pin on the master (§10) turned a
milestone that was blocked on test equipment into one verifiable with a jumper
wire, and produced the data that drove change 2. Worth doing early: a milestone
you cannot measure is a milestone you cannot finish.

### Bugs fixed along the way

- `recvfrom`'s address-length argument was hoisted out of the master's receive
  loop. It is written back on every call, so every packet after the first saw a
  shrinking value.
- `CONFIG_DANCEFLOOR_ROLE_MASTER` was used directly as a C expression. A Kconfig
  `bool` set to `n` is left **undefined**, not defined as `0`, so the satellite
  build failed to compile while the master built fine.
- Blink and monitor pins became per-board `menuconfig` options. Board silkscreens
  disagree — `G4` is GPIO 4 on one board, `D4` is emphatically not on another.

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
buffer sat perfectly stable at ~200 ms, with the satellite's rate visibly
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

### Known wart

The hub's absolute phase reading does not settle: it wanders +6 ms to +24 ms with
steps too large to be rate effects. Both units share whatever causes it, so the
*difference* between them stays small -- which is why cross-unit alignment
measures 2-4 ms while the absolute figures swing. The cause has not been found.

Treat the cross-unit measurement as the meaningful one, and the absolute phase
figure as indicative only.

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

**Asymmetry.** Sustained path asymmetry is invisible to the estimator (§2) and no
selection strategy removes it. It is the error floor. Test case 5 in
`test_sync_proto.c` pins the behaviour deliberately: 2 ms up against 200 µs down
produces exactly 900 µs of error.

---

## 10. Code map

| File | Responsibility |
|---|---|
| `sync_test/main/sync_proto.h` | Wire format, estimator state, `sync_to_local()` |
| `sync_test/main/sync_proto.c` | Offset maths and min-RTT selection. No ESP-IDF deps |
| `sync_test/main/main.c` | SoftAP, UDP sockets, probe/announce/blink tasks |
| `sync_test/test/` | Host tests — `make check` |

The estimator is deliberately free of platform dependencies. It is the part most
likely to be subtly wrong, and hardware bring-up is a bad place to discover that.

Time probes are **unicast**; blink announcements are **multicast**, because that
is the path audio will take in M5 — one transmission feeding every satellite, so
radio airtime does not scale with unit count.

---

## 11. Verification

**Estimator** — `cd sync_test/test && make check`. Eight cases covering exactness
on symmetric paths, outlier rejection, the asymmetry floor, window ageing, and
min-RTT selection.

A stable offset in the log means the satellite *believes* it is synced. It does
not prove the pulse lands where it should — that needs an independent
measurement, and there are two ways to get one.

### Self-measurement, no instruments needed

The master can time the satellite itself:

```
   SATELLITE                              MASTER
   blink GPIO (output)  ───────────────>  monitor GPIO (input)
   GND                  ───────────────   GND
```

The master already knows the master-clock instant it announced, so the gap
between that and the observed edge **is** the sync error, in a single time base
with no clock conversion involved. It prints:

```
I (34521) sync: SYNC ERROR: +83 us   (satellite late)
```

Wire propagation is nanoseconds and GPIO interrupt latency a few microseconds,
both negligible at this scale. Set the pin in `menuconfig` → *Dancefloor* →
*GPIO the master watches*; it must differ from the blink GPIO.

> This is one **output** into one **input**. Do not connect the two boards'
> *blink* pins together — those are both outputs, and tying them makes one drive
> against the other.

Errors beyond ±100 ms are reported as "not a sync measurement": a pulse is only
10 ms wide, so a gap that large means an announcement went missing rather than
the clocks disagreeing.

### With a scope

Probe the blink GPIO on both units with a **common ground**, trigger on the
master. Both pulse high for 10 ms every 2 s. Pass is rising edges within 1 ms.

Worth doing if you have one, since it measures the pins directly and shares no
code with the thing being tested.
