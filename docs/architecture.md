# The firmware, from a packet to a pixel

A study guide to the code. [`README.md`](../README.md) says what the boxes are
and how to wire them; this says what the software does and where each decision
lives.

The system's whole difficulty is one sentence: **several independent boards, on
independent crystals, must put the same sample in the air at the same instant,
for hours, with no feedback between them.** Almost every mechanism below exists
because of that, and the ones that do not exist — a leader election, a
neighbour-following loop, a drift-correcting mesh — were not needed once the
appointment model was chosen. That model is in §12, and it is the shortest
section here on purpose.

## Contents

**[How to read this](#how-to-read-this)**

**Part I — The shape**
[1 Three chips and one clock](#1-three-chips-and-one-clock) ·
[2 The task graph](#2-the-task-graph) ·
[3 Shared state and who writes it](#3-shared-state-and-who-writes-it)

**Part II — Following the audio**
[4 Phone to bridge](#4-phone-to-bridge) ·
[5 Bridge to hub](#5-bridge-to-hub) ·
[6 The presentation timeline](#6-the-presentation-timeline) ·
[7 Hub to satellites](#7-hub-to-satellites) ·
[8 Anchoring and playback](#8-anchoring-and-playback)

**Part III — Staying together**
[9 Estimating the master clock](#9-estimating-the-master-clock) ·
[10 Measuring position](#10-measuring-position) ·
[11 The four actuators](#11-the-four-actuators) ·
[12 Why nothing corrects to a neighbour](#12-why-nothing-corrects-to-a-neighbour)

**Part IV — Following the light**
[13 The analysis grid](#13-the-analysis-grid) ·
[14 Onsets](#14-onsets) ·
[15 Local frames or remote ones](#15-local-frames-or-remote-ones) ·
[16 The one rule for patterns](#16-the-one-rule-for-patterns)

**Part V — When it breaks**
[17 What each counter was born from](#17-what-each-counter-was-born-from) ·
[18 Reading a HEALTH line](#18-reading-a-health-line) ·
[19 TRACK DIVERGENCE](#19-track-divergence) ·
[20 Symptom to counter to file](#20-symptom-to-counter-to-file) ·
[21 Where the invariants are pinned](#21-where-the-invariants-are-pinned)

## How to read this

Three documents already exist and this one does not repeat them.
[`README.md`](../README.md) is the hardware and the operating story: wiring,
pin maps, Kconfig, build and flash. [`docs/wifi.md`](wifi.md) is the radio from
the band up. [`hub_s3/README.md`](../hub_s3/README.md) and
[`satellite/README.md`](../satellite/README.md) are per-board: pin maps, the
two-port flash recipe, the two-target build, and — in the satellite's case —
[How playback timing works](../satellite/README.md#how-playback-timing-works)
and [Reading the log](../satellite/README.md#reading-the-log), which are that
board's own account of what §8 and §18 describe from the outside.

What is *not* written anywhere else is the connecting layer, and that is this
document. Every mechanism here is documented in full at the place it lives —
the headers in this project carry the reasoning, not the code — so the value of
this guide is knowing **which header to open**, and in what order.

**Read the code in this order.** Each file's header block is the real
documentation; the note beside it says what you get from it.

| | file | what it teaches |
|---|---|---|
| 1 | [`components/dancefloor_sync/include/sync_proto.h`](../components/dancefloor_sync/include/sync_proto.h) | every message on the air, the clock estimator, and why numbers are burned rather than reused |
| 2 | [`hub_s3/main/hub.h`](../hub_s3/main/hub.h) | the hub's ~90 shared values, each with its owning task, and every timing constant |
| 3 | [`satellite/main/sat.h`](../satellite/main/sat.h) | the same for the satellite, plus the anchor rules |
| 4 | [`hub_s3/main/timeline.c`](../hub_s3/main/timeline.c) | the timeline itself — this is the hub's reason for existing |
| 5 | [`satellite/main/rx.c`](../satellite/main/rx.c) | anchoring, loss concealment, parity repair |
| 6 | [`components/dancefloor_sync/include/df_servo.h`](../components/dancefloor_sync/include/df_servo.h) and [`audio_shift.h`](../components/dancefloor_sync/include/audio_shift.h) | the two correction mechanisms both units share |
| 7 | [`components/dancefloor_leds/include/analysis.hpp`](../components/dancefloor_leds/include/analysis.hpp) | the pattern contract, and the four ways it has been broken |

Everything else can be reached from those seven.

**One habit worth taking from the codebase before reading any of it.** The
README puts it at the end; it is more useful at the start:

> every real fault was invisible until something counted it, and several were
> actively disguised as something else.

That is why Part V is a fifth of this document, why `hub.h` is 824 lines of
declarations with a paragraph on each, and why several counters exist that
nothing acts on. A counter here is not logging. It is the instrument that made
a fault visible, kept so the fault cannot come back unseen.

---

# Part I — The shape

## 1 Three chips and one clock

```
   phone
     |  A2DP / SBC, Bluetooth Classic
     v
 +----------------+        +------------------+       +---------------+
 |   bt_bridge    |  SPI   |     hub_s3       | WiFi  |   satellite   |
 |  ESP32 classic |=======>|   ESP32-S3       |======>|  ESP32 / S3   |
 |                | 5 MHz  |                  |unicast|   x N         |
 | A2DP sink      | 4 wire | SoftAP           | UDP   | joins the AP  |
 | AVRCP meta     |        | THE TIMELINE     | :5001 | anchors       |
 | forwards SBC   |        | its own speaker  |       | its speaker   |
 +----------------+        +---------+--------+       +-------+-------+
                                     |                        |
                                  I2S DAC                  I2S DAC
                                  SPI LEDs                 SPI LEDs
```

**Three roles, and only the first is forced.** Bluetooth Classic is what A2DP
requires and only the original ESP32 has it — the S3, C3 and C6 cannot ever
receive it. That binds `bt_bridge/` to a classic ESP32 and nothing else.

**The master is two chips because one was measured and did not fit.** From
[`hub_s3/main/main.c`](../hub_s3/main/main.c): on a single chip, Bluedroid and
the WiFi stack together left 164 bytes of DRAM free, which forced the FFT down
to 512 points and every buffer to a quarter size. Split, each chip has ~200 kB
spare and neither radio contends with the other. The hub runs no Bluetooth,
which is the only reason it can be an S3.

**The hub is a full speaker, not a base station.** It drives its own DAC and
its own strip, and — this is the part that matters — it delays its own audio by
the same `LEAD_US` it gives everyone else. A hub that played immediately would
be 350 ms ahead of every satellite on the floor.

### The two ideas everything else is downstream of

**Schedule the future, never the present.** The hub never says *play now*. It
says *play at master-time T*, where T is `LEAD_US` (350 ms,
[`hub.h`](../hub_s3/main/hub.h)) ahead of when the packet was stamped. Each unit
converts T to its own clock and waits. Network jitter stops mattering entirely,
as long as the packet arrives before T.

**One decision, drawn at the instant it names.** An analysis frame carries the
master-clock instant its audio is heard, so a unit draws it on the same timeline
it plays audio on. That is what makes a frame safe to send over a network at
all — and what lets a unit compute frames locally instead, with no agreement
mechanism, because both units are reading the same clock.

### What is deliberately absent

| not present | why |
|---|---|
| leader election | there is one hub, compiled as one |
| neighbour-following / drift mesh | §12 — units keep an appointment, they do not watch each other |
| multicast | tried, works, and lost a large share of packets at every PHY rate; unicast gets link-layer ACK and retransmission |
| PCM on the air | four times the bitrate in four times the packets; satellites decode their own SBC |
| BPM tracking | lights fire *on* the transient; tempo is a much larger thing to get right and nothing needs it |
| a resend queue for audio | a deferred resend arrives after a newer packet and is binned as a seq-drop; there is one immediate retry and no queue |

## 2 The task graph

Both firmwares start their tasks through a wrapper that reads the return value
— `task_start()` in [`hub_s3/main/streamer.c`](../hub_s3/main/streamer.c) and
[`satellite/main/main.c`](../satellite/main/main.c). A task that fails to start
is counted in `n_task_fail`, named in `s_task_fail_names`, and printed as
`CRIPPLED`. This exists because a unit that loses a task runs on without it and
looks exactly like one that has gone quiet.

**Hub** — [`hub_s3/`](../hub_s3/), ESP32-S3, dual core.

| task | prio | core | owns |
|---|---|---|---|
| `sbc_in` | 9 | 1 | the SPI link; the timeline's **input** side ([`sbc_in.c`](../hub_s3/main/sbc_in.c), [`timeline.c`](../hub_s3/main/timeline.c)) |
| `syncmon` | 9 | any | marker builds only ([`marker.c`](../hub_s3/main/marker.c)) |
| `play` | 8 | 1 | `local_play_task` — what playback has reached ([`play.c`](../hub_s3/main/play.c)) |
| `probe` | 6 | any | probe replies; the client list ([`probe.c`](../hub_s3/main/probe.c)) |
| `vis-draw` | 5 | 1 | render frames when due ([`visualiser.cpp`](../components/dancefloor_leds/visualiser.cpp)) |
| `vis` | 4 | 1 | FFT and detection |
| `ringmon` | 3 | any | `telemetry_tick()` then `servo_tick()` ([`servo.c`](../hub_s3/main/servo.c), [`telemetry.c`](../hub_s3/main/telemetry.c)) |
| WiFi event | — | — | station join/leave; the client list |

**Satellite** — [`satellite/`](../satellite/), one source tree, two targets.

| task | prio | core | owns |
|---|---|---|---|
| `play` | 8 | 1 | what playback has reached ([`play.c`](../satellite/main/play.c)) |
| `rx` | 7 | any | every datagram in; decode; the ring ([`rx.c`](../satellite/main/rx.c)) |
| `probe` | 6 | any | clock probes, splice reports, self-mute ([`clock.c`](../satellite/main/clock.c)) |
| `vis-draw` / `vis` | 5 / 4 | 1 | as on the hub, when analysis is local |
| `drift` | 3 | any | `telemetry_tick()` then `servo_tick()` ([`telemetry.c`](../satellite/main/telemetry.c), [`servo.c`](../satellite/main/servo.c)) |

**Bridge** — [`bt_bridge/`](../bt_bridge/). No task graph of its own worth
drawing: Bluedroid's stack task calls
`bt_app_a2d_audio_data_cb()` ([`main.c`](../bt_bridge/main/main.c)), which hands
SBC to the SPI master in [`sbc_spi.c`](../bt_bridge/main/sbc_spi.c). That chip
is running Bluetooth Classic and its budget cannot absorb anything else — which
is why the link's handshake is a GPIO interrupt rather than a poll.

**Two things to notice.** `play` is pinned to core 1 on both units and outranks
everything except link input, because it is the only task with a hard deadline:
the DAC does not wait. And the servo sits at priority 3 on both — it decides
once every five seconds and correcting late costs nothing, while preempting
playback to do it would cost an underrun.

## 3 Shared state and who writes it

[`hub.h`](../hub_s3/main/hub.h) and [`sat.h`](../satellite/main/sat.h) each open
with a table naming, for every shared value, **the task whose execution performs
the write** — not where the line of code sits. Roughly ninety values on the hub.
Those two tables are the reference; this section is only how to read them.

**`volatile` here means "another task writes this". It does not mean atomic.**
That distinction is the whole of the discipline:

> A 64-bit load is two instructions on this CPU, so a reader can catch half of a
> write. A torn read differs from both the old and the new value only when the
> HIGH word changes — for a monotonic microsecond clock, once per 71.6 minutes,
> and for anything set to or crossing **zero, every time**.

So each 64-bit shared value is handled one of three ways, and every one of them
says which it is at its declaration:

| approach | example | why |
|---|---|---|
| **made 32-bit** | `s_samples_in`, `s_marker_sample` | an `int32` still holds 13 hours of frames at 44.1 kHz; a torn 64-bit read would give a wild ring position |
| **published under a handshake** | `local_start` / `local_epoch` | `local_epoch` is incremented *after* `local_start` is written and read *before* it, and is 32-bit so it cannot tear |
| **accepted, with the cost stated** | `s_sync_at`, `s_retune_outage_us` | each is printed or compared against a threshold, so a torn read costs one wrong log line — never a wrong decision |

The satellite's `tsf` pair takes a fourth route, a sequence lock, because an
offset hovering near zero changes sign and its two halves only mean anything
together.

**Where a value has two writers, it says so.** `stream_start_local` on the
satellite is written by `rx_task` at an anchor and by `play_task` when it parks;
`sat.h` states the exposure (one 5 s window in which the servo believes a stream
is running) and states that giving it one owner would be a design change, not an
annotation. That is the house style for a known imperfection: name it, bound its
cost, and do not pretend it away.

---

# Part II — Following the audio

One packet, end to end. Each step names the function that performs it.

```
phone
  |  bt_app_a2d_audio_data_cb()                  bt_bridge/main/main.c
  v
  |  sbc_link_send()   12-byte header + SBC, CRC16, CS = framing
  v                                             bt_bridge/main/sbc_spi.c
 SPI 5 MHz, HANDSHAKE gates each transfer
  |
  v  rx_task -> sbc_decoder -> streamer_feed()   hub_s3/main/sbc_in.c
  |                                \-> visualiser_feed()
  v  streamer_send_sbc()                         hub_s3/main/timeline.c
  |     source_steady_enough() -> start_timeline() | steer_timeline()
  |     flag_boundaries() ; record_phase_point()
  |     msg.play_at = next_play_at
  |     fan_out()  -> unicast to each client
  |     fec_note_sent() -> send_fec_to_clients()
  v
 WiFi UDP :5001
  |
  v  rx_task -> handle_audio()                   satellite/main/rx.c
  |     clock_offset() ; anchor_stream()
  |     fec_hold_offer() / fec_repair()
  |     absorb_sequence_gap() -> fill_gap()
  |     record_packet_positions() ; decode_into_ring()
  v  play_task  waits for stream_start_local     satellite/main/play.c
  v  I2S -> PCM5102A -> speaker
```

## 4 Phone to bridge

The bridge is an A2DP sink and **it never decodes anything**. SBC arrives from
the phone and SBC leaves for the hub. The decode happens once per *listening*
unit, at the far end, which is what makes the WiFi hop affordable.

Alongside audio it forwards two other things, both as their own link kinds: track
metadata from AVRCP ([`avrcp_meta.c`](../bt_bridge/main/avrcp_meta.c),
`LINK_KIND_META`) and absolute volume (`LINK_KIND_VOL`). Volume matters more
than it looks: the hub relays it to every satellite, and each unit attenuates
its *own* output, so the air always carries full scale and the dynamic range is
not spent on a level decision.

The two status LEDs ([`status_led.c`](../bt_bridge/main/status_led.c)) are
driven from the audio callback, not from the phone's reported state — see the
README. Connected-but-not-streaming is the useful reading.

## 5 Bridge to hub

[`components/dancefloor_sync/include/sbc_link.h`](../components/dancefloor_sync/include/sbc_link.h)
is the shared definition; both ends include it and neither has a copy.

**Twelve-byte header, `SBC_LINK_MAX_PAYLOAD` of 2048 bytes, CRC16 over header
and payload, and CS is the framing** — one assertion is exactly one frame. The
header is twelve bytes so that header-plus-payload is a multiple of four, which
the SPI slave's DMA requires; satisfying that in the type beats remembering it
at every allocation.

**Why SPI and not I2S, when an earlier design used I2S and lost about one frame
in a hundred.** The header states it as a property of the peripherals rather
than the wire:

- **I2S is continuous.** The slave tracks a free-running foreign clock forever
  with nothing to realign against, so alignment error accumulates and no
  boundary ever discards it.
- **SPI is transactional.** CS resets the bit counter at every assertion.
  Nothing accumulates across frames, the worst a clock difference can cost is
  one frame, and the CRC catches that one.

Measured across many hours: zero bad headers, zero CRC failures, zero CS-split
transfers. What that does not cover is packets lost *before* the wire —
`sbc_in`'s `lost` counter says how many, and the bridge's own counters say
whose.

The link's status line is the wiring check:

```
I sbc_in: pkts 252 | 44100 Hz x2 | eff 44050 Hz | hdr 0 crc 0 short 0 gaps 0 ...
```

`short` moving means CS is splitting transfers — framing. `crc` moving means
SCK/MOSI bit errors — signal integrity, so drop `DANCEFLOOR_SBC_LINK_SPI_HZ`
and reflash the bridge. Do not chase the code.

## 6 The presentation timeline

[`hub_s3/main/timeline.c`](../hub_s3/main/timeline.c). This is the hub's reason
for existing, and everything on the floor — including the hub's own speaker —
is downstream of one number, `next_play_at`.

`streamer_send_sbc()` takes its decisions in a fixed order, one function each:

**1. Is the source running?** `note_arrival()` tracks the gap between packets;
`source_steady_enough()` gates a *start* on it. A2DP arrives in ~43 ms lumps,
so healthy 5 s windows carry a 79–112 ms gap — `SOURCE_STALL_US` (300 ms)
separates a real hole from that pattern. `SOURCE_STEADY_US` is 500 ms, twice
`LEAD_US`, so a source that passes it can fill the ring before playback arrives.
`SOURCE_GIVE_UP_US` (5 s) bounds the wait.

**2. Start a timeline, or steer the one that exists.**

`start_timeline()` publishes a new origin: resets the local ring, zeroes
`s_samples_in`, clears the phase queue, flushes the visualiser. Every unit on
the floor re-anchors to what it sets. Note what it does *not* do — `local_start`
is assigned at the *end* of `streamer_send_sbc()`, because position zero of the
freshly reset ring belongs to the **next** packet, not this one.

`steer_timeline()` handles the ordinary case, and its design is the most
instructive thing in the file:

| `err = next_play_at - target` | action | why |
|---|---|---|
| within `RESYNC_US` (±70 ms) | nothing | bursty delivery crosses this band several times a minute; that is the design |
| beyond it | **slew** at `TIMELINE_SLEW_US` (20 µs/packet ≈ 1 ms/s) | a jump steps every unit's phase at once, and each then servos it off with a loop built for parts-per-million — minutes of every speaker being audibly elsewhere |
| beyond `RESYNC_HARD_US` (300 ms) | **jump**, and arm a boundary | past this no satellite can anchor and one already playing cannot trim fast enough |
| ...but ring under `TIMELINE_HOLD_STARVE_MS` (150 ms) | **hold** the jump | a starved ring manufactures this error by itself: `next_play_at` stops advancing while `target` keeps moving |

The slew rate is bounded by what the servo can follow: a unit may trim up to
`RATE_TRIM_MAX_HZ`, which is 2.27 ms/s at 44.1 kHz, so a slew near that would
outrun the units it is supposed to be leading.

The hold is the subtlest guard here. Jumping re-stamps the whole fleet to fix a
*delivery* problem, and each jump arms a boundary every unit splices at — so a
burst of source wander becomes a burst of audible splices with the rings
ballooning behind the inserts. Held, the same excursion slews back on its own.

**3. Boundary flags.** `flag_boundaries()` sets `msg.restart` when a track
changes, when an underrun restart needs the satellites to come along, or when an
armed post-jump boundary counts down. The count-down matters: the flag cannot
fire on the jump itself, because every unit's phase still describes the timeline
that just ended, so it waits `SYNC_PHASE_HIST` packets for the splice's window
to clear.

**4. Where the audio sits.** `record_phase_point()` pairs the ring position
captured by `streamer_begin_packet()` with `next_play_at`, and pushes it onto
the phase queue. This is the servo's only input — §10. It is skipped at a
timeline start, and that skip is load-bearing: the captured position describes
an origin that has been reset away, and queuing it would wedge the queue and
stop the servo for the rest of the session.

**5. Send.** §7.

**6. Advance.** The timeline advances by the audio actually sent, not by wall
clock — stamping `now + lead` each time would fold task jitter into playback.
**The sub-microsecond remainder is carried** in `s_play_at_rem`. Dropping it
loses a fraction of a microsecond per packet, which at ~50 packets/s is tens of
parts per million, and nothing corrects it because it lives entirely inside both
the timeline deadband and the servo's: the lead in front of every stamp shrinks
steadily until an ordinary delivery gap empties a satellite's ring. That is
~93 ms an hour, from a discarded modulo.

## 7 Hub to satellites

Everything rides one UDP port, `SYNC_PORT` 5001, and everything is **unicast to
each registered satellite** — so the hub's airtime grows with speaker count.
Multicast was tried and works, but is never acknowledged and never retried, and
lost a large share of packets at every PHY rate. Unicast gets link-layer ACK and
retransmission and measured essentially clean. That is affordable at SBC
bitrates and would not have been at PCM.

### The message set

From [`sync_proto.h`](../components/dancefloor_sync/include/sync_proto.h).
**Retired numbers are burned, never reused** — a board on older firmware speaks
a protocol this one does not, and a type silently reinterpreted as something
else is worse than one that is simply unknown, because every dispatch already
ignores an unknown type.

| # | message | direction |
|---|---|---|
| 1 | `MSG_TIME_REQ` | satellite → hub: a clock probe |
| 2 | `MSG_TIME_RSP` | hub → satellite, unicast |
| ~~3~~ | *burned* — an early bring-up blink harness | |
| 4 | `MSG_AUDIO` | hub → listeners: one chunk, with its `play_at` |
| 5 | `MSG_META` | hub → listeners: track metadata |
| 6 | `MSG_SPLICE` | satellite → hub: what it corrected at a boundary |
| 7 | `MSG_TSF` | hub → satellite, **measurement only** |
| 8 | `MSG_FRAME` | hub → listeners: a *batch* of analysis frames |
| 9 / 10 | `MSG_LOG` / `MSG_HEALTH` | any unit → the collector, via the hub |
| 11 | `MSG_LOG_SUB` | collector → hub: "send logs here" |
| ~~12~~ | *burned* — one analyser's result, before analysers became spectrum-fed | |
| 13 | `MSG_VOL` | hub → listeners: playback volume |
| 14 | `MSG_AUDIO_FEC` | hub → listeners: one XOR parity per group |

`MSG_AUDIO_FEC` is its own type rather than a field on `audio_msg_t` precisely
so that a satellite predating it goes on reading the audio unchanged.

### Lanes, and why audio is privileged

`sendto()` fails with `ENOMEM` when the WiFi driver's static TX buffer pool is
exhausted. Every failure goes through `tx_fail_note(lane, errno)`, and the lanes
are ordered audible-first — a refused audio packet is a hole on every satellite
at once, a refused frame is one repaint, a refused level is covered by the 1 Hz
repeat.

| lane | pacing | on `ENOMEM` |
|---|---|---|
| `TX_LANE_AUDIO` | none — `fan_out()` is never gated | one immediate retry, then counted |
| `TX_LANE_FRAME` | `TX_FRAME_PACE_US` (102.4 ms) | stands down for `TX_BACKOFF_US` (40 ms) |
| `TX_LANE_VOL`, `TX_LANE_META`, `TX_LANE_PROBE`, `TX_LANE_FEC` | own cadences | likewise |

`tx_fail_note()` raises `s_tx_congested_until`; every non-audio lane skips past
it and counts the skip in `n_tx_cong_skip`. `fan_out()` ignores it entirely.
**Redundancy stands down under congestion too** — `n_fec_cong_skip` is parity
withheld so it does not displace the audio it protects.

### Frames are batched, not paced away

The hub computes analysis frames at ~86/s and the frame lane may send at ~10/s.
Before batching, everything offered in between was simply dropped — a detector
running at a fraction of the rate its thresholds were tuned at, since its
adaptive window is a frame *count*. It was visible from across the field: the
hub's strip followed the music and the satellites' strips lurched.

The pace stays, because sending faster only occupies transmit buffers the audio
needs. What changed is how much rides in the one datagram the pace releases:
`TX_FRAME_BATCH` (12) frames, every frame since the last flush. The cost is
loss *granularity* — one lost datagram is now twelve consecutive frames — and
the receiving detector converges back within its history length. `TX_FRAME_BATCH`
is the one knob; if it ever reads as a stutter, halve it, do not shorten the
pace.

### XOR parity

One extra datagram per group of K audio packets recovers any **one** of them
whole. `DANCEFLOOR_AUDIO_FEC_K` picks K, up to `AUDIO_FEC_K_MAX` (8).

**The codeword is the whole message, header included** — `AUDIO_MSG_BYTES(0)`
bytes of header, then the payload, zero-padded. Padding is implicit, since a
shorter payload simply stops contributing, so the unequal SBC lengths this
stream produces need no length table on the wire. Recovering the header along
with the payload is what makes the repair *whole*: `seq`, `frames`, `play_at`,
`marker` and `restart` all come back, so a recovered packet goes through the
ordinary receive path and is indistinguishable from one that arrived.

Group membership is arithmetic on `seq` — member index is `seq % K`, base is
`seq - (seq % K)` — so nothing is negotiated, and `base_seq` and `count` travel
on the parity anyway so a hub and satellite built with different K disagree
loudly instead of combining the wrong packets.

**What it replaced, and why the replacement is not the same trade.** Redundancy
used to be piggybacked: each audio packet carried copies of previous payloads in
its own tail. That pinned every packet at the MTU instead of its natural size —
most of an extra packet's airtime, permanently — and a whole copy still did not
fit beside the payload, so *every* recovery came back incomplete with a decode
error on the end. Parity inverts the economics: the redundancy is its own
datagram, the audio goes back to its natural size, and the cost is 1/K of the
bytes and 1/K of the datagram rate.

**And it costs no timeline slew**, which is the objection that killed the
previous attempt. `TIMELINE_SLEW_US` is applied once per *audio* packet, so
shrinking payloads to make copies fit doubled the slew along with the packet
rate. A parity datagram carries no `play_at`, never advances the timeline, and
is not counted in the packet rate the servo steers on.

The parity buffer lives in **PSRAM**, deliberately. The WiFi driver's static TX
buffers must be DMA-capable internal SRAM; spending a datagram of that on
redundancy would take it from the very pool whose exhaustion drops the audio in
the first place. Parity is simply off on a board with no PSRAM to give.

## 8 Anchoring and playback

[`satellite/main/rx.c`](../satellite/main/rx.c). `handle_audio()` runs a fixed
sequence of gates, and the *order* carries meaning at two points.

**1. Get an offset.** `clock_offset()` — §9. Without one, nothing else can
happen and the packet is dropped.

**2. Gauge the lead.** `msg->play_at - (arrived_at + offset)`, taken *before*
every refusal below, so a packet that is rejected still reports how it arrived.
Paired against the hub's `n_lead_min_us`, the two subtract to a transit time.

**3. Parity hold** — decided *before* the anchor, and that order is
load-bearing: anchoring resets the ring, and a hold whose packets outlived that
reset would be replayed onto a stream they do not belong to.

**4. `anchor_stream()`.** Converts `play_at` to local time and sets
`stream_start_local`, the instant playback begins. It refuses in three ways:

| refusal | constant | why |
|---|---|---|
| estimator not settled | `SYNC_WINDOW` probes | `play_at` is consulted once, at stream start, so an offset error at that instant is **permanent** |
| lead too thin | `ANCHOR_MIN_LEAD_US` (125 ms) | the scheduled wait is the *only* thing that prefills the ring, so whatever lead survives the trip **is** the prefill |
| anchored too recently | `ANCHOR_MIN_INTERVAL_US` (1 s) | a re-anchor that does not stick is worse than none |

Past `ANCHOR_GIVE_UP_US` (5 s) it anchors on a bad packet anyway rather than
leave the speaker silent, and marks it `anchor_provisional` so
`upgrade_provisional_anchor()` can replace it when a properly-led packet turns
up.

**5. `absorb_sequence_gap()`.** UDP loses packets, and this is where the design
becomes visible: **a gap must be filled with the right amount of silence, or
every later sample plays early and the whole stream slides.** That is why
`audio_msg_t` carries `seq` and `frames` as well as `play_at`. Below
`GAP_RESYNC_MS` (150 ms) `fill_gap()` conceals it; beyond, it is an outage and
the unit re-anchors instead.

**6. `audio_deliver()`** — `record_packet_positions()`, which is the
satellite's twin of `record_phase_point()`, and then `decode_into_ring()`.

Playback ([`play.c`](../satellite/main/play.c)) then waits for
`stream_start_local`, and from there does nothing but read chunks, apply the
trim and the catch-up shift, and write to I2S. Every correction it makes is
§11.

---

# Part III — Staying together

## 9 Estimating the master clock

Each satellite needs one number: `offset`, such that `master = local + offset`.
`sync_to_local()` is the whole of the conversion.

### The probe estimator

`sync_est_add()` in
[`sync_proto.c`](../components/dancefloor_sync/sync_proto.c) — the standard NTP
form, four timestamps:

```
    satellite                          hub
       t1  ---- MSG_TIME_REQ ---->      t2
                                        |
       t4  <--- MSG_TIME_RSP -----      t3

    offset = ((t2 - t1) + (t3 - t4)) / 2
    delay  =  (t4 - t1) - (t3 - t2)
```

That estimator assumes the two path delays are equal. They are not, and **that
asymmetry is the error floor** — it is the one error the estimator cannot see.

**`sync_est_offset()` selects the offset from the probe with the lowest
round-trip time. It does not average.** A fast round trip had little queuing in
either direction, so its paths were closer to symmetric. Measured RTT on a
SoftAP link swings by a factor of two or three, so *which sample you pick*
matters more than averaging across all of them. This is what PTP does, and on
this hardware it beat a median by an order of magnitude. Ties favour the newer
sample, since an old one has had time to drift.

`SYNC_WINDOW` is 10 probes at `PROBE_PERIOD_MS` 250, so the window is 2.5 s.
`sync_est_settled()` requires a *full* window before playback may anchor —
`SYNC_MIN_SAMPLES` (3) is only enough to offer an estimate at all. Those extra
couple of seconds remove a whole class of "one speaker is slightly out" that
would be untraceable afterwards.

An offset step of `SYNC_STEP_US` (a whole second) between consecutive probes
means the master changed *origin* rather than drifted — drift moves the offset
by microseconds and path asymmetry by milliseconds.

### TSF, and why it is measurement only

`MSG_TSF` carries the hub's 802.11 TSF against its own clock. TSF is the WiFi
MAC's microsecond counter: the AP maintains it, every beacon carries it, and
each station's MAC **hardware-timestamps** the beacon on arrival. That hardware
stamp is what real PTP relies on and what a software stamp either side of a
`sendto()` cannot provide — everything between "read the clock" and "the frame
left" lands in the error budget.

With TSF there is no round trip to be asymmetric at all:

```
offset = (master_local - master_tsf) - (sat_local - sat_tsf)
```

`clock_offset()` in [`clock.c`](../satellite/main/clock.c) prefers a fresh TSF
reading (`TSF_MAX_AGE_US`, 1 s) and falls back to the estimator. `sat.h` marks
the wire message measurement-only; which source an anchor actually used is
printed on the `stream start` line and carried in `health_msg_t::clock_src`, so
a floor can be read either way.

### The offset is slewed, never stepped

`track_offset()` moves `stream_offset` toward the live estimate at
`OFFSET_SLEW_PPM` (200 ppm) and no faster. Min-RTT selection moves the *raw*
estimate by milliseconds as probes rotate through the window, and stepping the
timeline by that would be audible. 200 ppm is comfortably above the crystal
difference it has to follow and slow enough that nothing jumps.

**The probes are also the registration.** The hub keeps a satellite on its send
list for as long as probes keep arriving (`CLIENT_TIMEOUT_US`, 2 s ≈ 8 probes),
so stopping them is how a satellite leaves — which is exactly what the
self-mute in `mute_tick()` does when a unit has gone deaf.

## 10 Measuring position

**Servoing on buffer depth is not enough, and this is the single most important
thing to understand about the sync design.** Depth matches the playback *rate*
to the arrival rate but says nothing about *position*, and depth moves with
network jitter — so two units seeing different jitter settle at slightly
different rates and drift apart while each one's own buffer looks perfectly
stable.

So position is measured directly. Every packet says exactly when its first
sample should play, so recording that against the ring position it lands at
gives a position reading when playback reaches it:

```
   rx task                                  play task
   -------                                  ---------
   phase_q[head] = { pos, play_at }   -->    when samples_played >= pos:
   head++                                      err = crossed_at - play_at
                                               (+ = playing late)
```

`phase_pt_t` is 32 slots, single producer, single consumer, 32-bit indices, no
lock. A full queue drops a point into `n_phase_drop` — and a wedged phase queue
is not a degradation but a **stop**, because the servo runs on nothing else.

### Two corrections that are pure arithmetic

`absorb_phase_crossings()` in [`play.c`](../hub_s3/main/play.c) does not read a
clock at the crossing, and both reasons are worth internalising:

**Overshoot.** `s_samples_played` advances a whole chunk per iteration, so by
the time the loop notices it passed `pos`, it passed it up to a chunk ago — by
an amount that depends on where `pos` fell on the chunk grid, and is therefore
uncorrelated sample to sample. That is pure quantisation noise on the servo's
only input. Writes are paced by the DAC, so the instant `samples_played` *was*
`pos` is exactly `overshoot / rate` ago. Arithmetic, not a filter.

**Write instant.** The reading is dated from `s_wrote_at` — when the DAC last
took a chunk — not from a clock read in the loop. That write is the only
DAC-paced event in the pass; everything between it and the measurement is paced
by nothing, and on a board also running a SoftAP, an SBC decode and the bridge
SPI link, "nothing" means whatever preemption it is handed.

### Median, not mean

What survives both corrections is preemption latency: bounded below,
long-tailed to the right. There is a minimum latency and no mechanism that makes
a reading *early*. The mean is dragged by that tail; the median sits on the
mode.

`sync_phase_hist_t` holds `SYNC_PHASE_HIST` (9, odd so the median is an element)
raw readings, and offers none below `SYNC_PHASE_MIN` (5) — the guard against
splicing on one or two samples taken just after a re-anchor. If the noise turns
out to be symmetric after all, the median costs about a quarter more in standard
error at this window length, which is nothing against the scatter removed. It is
the right answer under both models and the mean under only one.

The two units do **not** feed the servo the same input, deliberately: the
satellite smooths the raw reading, the hub smooths the published median, because
the hub's raw reading carries far more scatter at its load. `df_servo.h` states
that unifying the code must not unify that choice.

## 11 The four actuators

This is the piece that lives in no single file. A unit has four ways to move
itself on the timeline, and they differ in range, in cost, and in what they
touch.

| | mechanism | range | cost | actuator |
|---|---|---|---|---|
| 1 | **fine rate trim** | ≤ `RATE_TRIM_MAX_HZ` (100 Hz) | one frame per period | `trim_due()` in both `play.c` |
| 2 | **catch-up debt** | tens of ms/s | a few crossfaded frames per chunk | `chunk_shift()` + [`audio_shift.c`](../components/dancefloor_sync/audio_shift.c) |
| 3 | **coarse retune** | beyond the trim | a channel outage | `retune_dac()` / `retune_output()` |
| 4 | **boundary splice** | ≤ `MAX_SPLICE_MS` (150 ms) | inaudible *only* at a track change | `apply_track_boundary()` |

**1. The fine trim** is the everyday one. The DAC clock stays put; playback
consumes the ring at an effective `tx_rate + rate_trim_hz` by dropping one frame
to get through the stream faster and duplicating one to get through it slower.
Positive means playing late, so consume faster. The servo writes it once per
5 s window; playback spends it, banking the sub-frame remainder exactly the way
the timeline banks `s_play_at_rem`. `n_trim_drops` and `n_trim_dups` are its
instrument — flat while the trim is non-zero means it is not running, both
climbing together means the servo is hunting.

**2. The catch-up debt** exists because the fine trim is too slow for a *knock*.
A lost-packet burst leaves a unit genuinely late: the audio is gone, the gap was
filled with silence, and the silence took output time the timeline does not give
back. At `RATE_TRIM_MAX_HZ` a hundred-millisecond knock is a minute of audible
echo, usually ended by the boundary splice rather than by the servo. The debt
applies the splice's own move — drop or replay frames — continuously and
inaudibly, a few frames per chunk under a short crossfade, correcting tens of
milliseconds per second. Armed above `CATCHUP_ARM_US` (25 ms), stood down below
`CATCHUP_CLEAR_US` (10 ms); the gap between them is the hysteresis that stops it
flapping. **The servo arms it and only playback shrinks it.**

**3. The coarse retune** moves the clock, and is reached only when the
correction exceeds `RATE_TRIM_MAX_HZ` — which real drift never does, since two
crystals differ by a few parts per million and an ordinary fine correction is
well under a hertz. Anything reaching that bound is a broken measurement rather
than a correction, which is why the clamp exists at all. It costs a channel
outage, and its first phase reading afterwards is a transient that is logged and
thrown away rather than handed to the servo: left in, every retune injects the
disturbance the next one would correct.

**4. The boundary splice** is the only one that moves the unit at a stroke, and
it is the only one that is allowed to be audible — because it happens where a
track changes and nobody can hear it. It nulls the accumulated phase error
outright.

Three things about it are worth knowing:

- **It happens where the audio is, not where the notification arrived.** The
  `restart` flag rides on the packet; when *playback* reaches that audio, the
  splice fires. At the moment the notification arrives the buffer still holds a
  lead's worth of the previous track, and correcting then would cut its ending.
- **It acts on the median.** A splice taken on a single reading lands several
  milliseconds wrong in a direction nothing predicts — and the hub and the
  satellite, with very different loads and therefore very different scatter,
  would splice to *different places*. That is why the change had to land on both
  units at once, and did.
- **The insert side is clamped.** Its zeros take DAC time and consume nothing
  from the ring, so receive keeps pushing while they play — the insert is the
  one splice that can overflow the ring it is fixing. `SPLICE_INSERT_HEADROOM_MS`
  bounds it, and whatever the clamp eats is left standing for the catch-up
  drain. The skip side needs no clamp: its discard loop reads with a zero
  timeout and stops on an empty ring by construction.

### The servo that drives 1–3

[`df_servo.c`](../components/dancefloor_sync/df_servo.c) is **arithmetic and
nothing else** — no FreeRTOS, no I2S, no GPIO, no logging. It decides; the
caller measures, actuates and reports. That is what lets `test_servo.c` drive it
under plain gcc, and it is why the file exists at all: a correction rate that
differs between the units is a cross-unit sync error *by construction*, and two
copies of the arithmetic enforce agreement only by whoever edits one remembering
to edit the other.

`df_servo_ema()` folds each window into a 4-sample average — under half a minute
of memory at a 5 s window. That separates two things which look identical in a
single reading: **jitter**, which swings tens of milliseconds with no trend, and
**drift**, which walks steadily one way. Averaging kills the first and leaves
the second. Coping with a wide deadband instead means real drift can reach an
audible echo before anything happens — fine in a short test, wrong over an
evening.

`df_servo_step()` then spreads the correction over roughly a hundred seconds.
The buffer takes tens of seconds to respond, so a tighter loop is still
correcting after the error has gone, sails past it, and both units oscillate.
Real drift is a fraction of a millisecond per minute; the loop can afford to be
far gentler than the disturbance it corrects.

`PHASE_DEADBAND_US` (7 ms) is **shared, and is not a per-unit preference**:
every unit deadbands around its own reading of the same timeline, so the worst
case between any two of them is twice it. It was once bounded below by what a
clock retune costs — a correction smaller than the disturbance it causes is not
worth making — and that bound no longer binds, since corrections this size are
made in software now. The value was kept unchanged anyway, deliberately: the
point of that change was to remove an interruption without moving the sync
behaviour, and every cross-unit figure this project has recorded was measured at
this value. Tightening it is the obvious next experiment, and a separate one.

## 12 Why nothing corrects to a neighbour

The shortest section here, and the reason the previous three are as short as
they are.

**No unit ever measures, hears, or corrects toward another unit.** There is no
neighbour discovery, no drift mesh, no leader-follower loop, and nothing on the
wire that would carry one. Units agree because they all keep the same
appointment:

1. the hub stamps every chunk with a master-clock instant `LEAD_US` in the
   future;
2. every unit — the hub included — converts that instant to its own clock and
   waits for it;
3. every unit measures its own position against that same published timeline;
4. every unit corrects **its own** error, with the same arithmetic, from the
   same shared constants.

Two units are in sync because they are both in sync with the timeline, not
because either has ever heard of the other. Nothing has to converge, so nothing
can fail to converge, and adding an Nth speaker changes nobody's behaviour.

What it costs is that a shared constant is now a **cross-unit contract**. A
value changed on one unit and not another is a sync fault, not a taste
difference. Three files say so explicitly, and all three put the constant where
neither unit owns it: `PHASE_DEADBAND_US` in `sync_proto.h`,
`RATE_TRIM_MAX_HZ` in `audio_shift.h`, the whole servo in `df_servo.c`. The
same rule reaches into the lights — `beat_detect.h` says its constants are a
cross-unit agreement rather than local quality knobs.

**What it actually costs in practice is measured, not assumed**, and printed
once per track as `TRACK DIVERGENCE` — §19.

---

# Part IV — Following the light

The lights ride on the audio's synchronisation and add no mechanism of their
own. That sentence is the whole design, and the rest of this part is the four
places it could quietly stop being true.

```
 PCM (on arrival, not at the DAC)
   |  visualiser_feed(pcm, len, due_master_us)     visualiser.cpp
   |     drift check -> re-derive origin -> round UP to the hop grid
   v
 pcm_stream  (a stream buffer, FFT_N*8 frames)
   |
   v  visualiser_task           FFT_N=1024, HOP_N=512
   |     Hann -> FFT -> 4 bands -> beat_det_update() -> df::Frame
   |     frame.index  = block number from a shared origin
   |     frame.due_us = index * HOP_N / rate
   v
 frame ring (32 * FFT_N/HOP_N slots)   ---> publish_frame() ---> MSG_FRAME
   |                                        (hub only)
   v  render_task    waits until due_us, then draws
   |     s_latch.take(due_us)   <- slow analysers' results land here
   |     pattern->render(frame, pixels)
   v  LedStrip::show()  -> SPI2 MOSI -> WS2812
```

## 13 The analysis grid

`FFT_N` is 1024 and `HOP_N` is 512 by default — 50% overlap, which is what
ordinary onset-detection algorithms assume. The two are kept strictly apart in
[`analysis.hpp`](../components/dancefloor_leds/include/analysis.hpp): `FFT_N` is
**window-only** (it sets the bins, the band edges, the Hann table, the
normalisation) and `HOP_N` is **grid-only** (it sets how often a frame is
produced, and therefore the block grid the units align to). Nothing in the
transform reads the hop.

The four bands are split at `BAND_EDGE_HZ` = 43, 172, 1034, 5039 Hz, and each
edge's bin index is pinned by a `static_assert` — so a change to `FFT_N` that
moves a band fails the build rather than quietly retuning the detector.

### Where two units are made to agree

**`visualiser_feed()` is the mechanism, and it is four lines of arithmetic.**
The audio is fed on *arrival* rather than at the DAC, and each chunk is dated
with the master-clock instant it will be heard. To start a block:

```c
const int64_t idx      = (due_master_us * rate) / 1000000;   /* sample position */
const int32_t into_hop = idx % HOP_N;
s_skip_frames          = into_hop ? (HOP_N - into_hop) : 0;  /* drop the part-hop */
s_pending_block_index  = (idx + s_skip_frames) / HOP_N;
```

**Rounding the shared instant up to the hop grid is what makes two units
agree.** The position is derived from an instant every unit shares, so every
unit computes the same one, drops the same part-hop, and starts its windows on
the same sample. From then on both units cut and label identically because they
are counting the same samples from the same origin.

Two things can break that count, and both re-derive the origin rather than
letting it slide:

- **the drift check** — extrapolate the sample count from the reference pair and
  compare against the instant this chunk was handed. Beyond `ALIGN_DRIFT_US`
  (2 ms) audio was skipped or inserted without anyone saying so. Counted in
  `drifts`.
- **a refused byte** — the stream buffer was full, so the count is broken and
  every frame after it would be mislabelled by the amount dropped. Counted in
  `dropped`, and an alignment is requested.

The published origin uses release/acquire ordering on a generation counter, so
the analysis task never sees a half-published origin.

`test_align` is the standing proof that two units cut and label their blocks
identically. It exists because that arithmetic is only exercised by running it,
and `make check-hops` reruns the whole suite at every supported hop for the same
reason.

## 14 Onsets

[`beat_detect.h`](../components/dancefloor_leds/include/beat_detect.h) takes
**band energies, not samples**, so the FFT stays the caller's problem —
esp-dsp on target, a host FFT in `pattern_lab` — and the detector itself has no
platform dependencies and is unit-tested under plain gcc.

It is **onset detection, not BPM tracking**, deliberately: lights need to fire
*on* the transient and do not need to know the tempo, and a tempo tracker is a
much larger thing to get right than anything here needs.

Two detectors run over each frame:

- **`onset`** — weighted spectral flux across all four bands, which is right for
  music whose transients are broadband.
- **`boom`** — the lowest band alone. Forró is not broadband: the triangle plays
  continuous subdivisions in the upper bands and contributes flux on every one,
  so a wideband detector follows the *triangle* and the lights flicker with it
  instead of moving with the drum. The zabumba's mallet stroke lives in the low
  band, where in a pé-de-serra trio there is no bass guitar to compete.

**Its constants are a cross-unit agreement, not local quality knobs.** On any
floor where more than one unit runs this detector — and `RemoteDetect` is
exactly that case — identical decisions require identical constants *and*
identical state, and the state is `BEAT_HIST` (43) frames of flux history plus a
refractory instant. Two consequences:

- a value changed on one unit and not another is a **sync fault**, not a taste
  difference, and it presents as strips that mostly agree and disagree on the
  marginal onsets — expensive to diagnose;
- `BEAT_HIST` sets how long a unit that missed frames takes to converge back
  onto its neighbours' threshold, so it is a sync parameter as well as a tuning
  one.

## 15 Local frames or remote ones

`DANCEFLOOR_LED_SOURCE` picks how a unit gets its frames, and what guarantees
agreement differs between the two.

| | `LED_SOURCE_LOCAL` | `LED_SOURCE_REMOTE` |
|---|---|---|
| FFT | this unit's, over the audio it is about to play | none |
| on the air | nothing | `MSG_FRAME` batches |
| agrees because | the detector is deterministic over identical input | it is reading the same source |
| ships on | hub, S3 satellite (the Kconfig default) | classic satellite |
| pinned by | `test_align`, `test_pattern_sync` | `test_remote_detect` |

**What travels is the detector's *input*, not the hub's answer.** `vis_frame_t`
carries the timeline labels and the four band energies at **full float
precision**, and the receiver runs the same detector over the same numbers
(`df::RemoteDetect` in
[`analysis.cpp`](../components/dancefloor_leds/analysis.cpp)). The onset and
boom a pattern sees are derived at the far end. What runs in exactly one place
is the FFT — the part that could not be proved deterministic across two
different cores; the detector is plain C over identical bytes, which needs no
proof.

**The spectrum does not travel**, and that is what keeps the frame small. Only
the pluggable analysers ever read it — no pattern touches it — so a satellite
with the analysers off received most of every frame and discarded it. Dropping
it bought **airtime, not buffers**: a static transmit buffer holds one datagram
whatever its size, so the datagram rate and the window each one occupies are
exactly as before. The consequence is a build rule rather than a runtime one — a
unit taking frames from the wire cannot run the analysers, and Kconfig makes
that combination unselectable.

It could not have served as the detector's input anyway: `spec` is quantised to
8 bits, and flux is a frame-to-frame *difference*, so the quantisation would
land directly on the signal the detector runs on. `BEAT_FLUX_FLOOR` is 0.02 on a
0–1 scale — a handful of counts out of 255 — and near-threshold differences are
smaller than that.

**The remote choice is a memory decision before it is anything else.** Local
analysis wants a 32 kB contiguous stream and another task stack on top of the
ring, which a classic ESP32 does not have — and the way it does not fit is
silent: tasks fail to start and the unit still associates, still takes a lease
and still looks healthy from the hub. That silence is what `n_task_fail` and the
`CRIPPLED` line exist to break.

`test_remote_detect` is the standing proof that the two paths reach the same
decision, which is what a mixed floor rests on.

## 16 The one rule for patterns

> **A Pattern must be a pure function of the Frames it has been given.**

Every field of `Frame` is identical on every speaker for the same audio, so a
pattern obeying that rule is automatically in sync with its neighbours and no
further mechanism is needed.

Breaking it does not fail loudly. It fails as strips that agree at first and
drift apart over minutes. `analysis.hpp` lists the four ways it has actually
been broken here:

| broken by | why it diverges |
|---|---|
| accumulating per render call (`hue += 0.3`) | units do not render the same number of times |
| reading a wall clock | units reach the same line milliseconds apart |
| measuring elapsed **local** time | nearly the same, not exactly |
| anything random or uninitialised | — |

Use `Frame::due_us` for anything time-based and `Frame::index` for anything
counted. Both are shared. `test_pattern_sync.cpp` enforces this mechanically by
running a pattern as two units with different join times, render counts and drop
histories and requiring byte-identical output — and it carries a deliberately
wrong pattern that it **requires to fail**.

One field does not survive being stored: `Frame::mag`, the raw spectrum, is a
pointer into the `Analysis` that produced it and is overwritten by the next
`process()`. Rendering is deferred, so a queued frame's magnitudes are long gone
by the time a pattern sees it, and the render path sets the pointer null rather
than leaving it dangling. It is usable by anything consuming a frame
immediately — which means the host harness — and never by a Pattern.

### Where a slow analyser's answer waits

[`result_latch.hpp`](../components/dancefloor_leds/include/result_latch.hpp)
solves a problem that only exists because frames are drawn at the instant they
name. A model with a long context cannot answer until that much audio has
arrived, which is well after the instant its window began — by then the frame
that window belongs to is already on the strip. Rather than delaying the *strip*
(which would delay the FFT's frames too, for an algorithm that does not need
it), the result carries its own later instant, `show_at_us`, and waits.

**It is latched at render, not at frame production**, and that is the whole
trick. The analysis stage runs as far ahead as it has audio for — up to the
entire playback lead — so a frame due at T is produced well before T. Latching
there would require every result ready that far ahead of its own display time,
spending the whole lead before the model had started. The render stage reaches
that frame *at* T, so latching there hands the slow lane the entire lead as
working time. That is the only reason a model taking tens of milliseconds fits
at all.

**It stays in step across units** because `show_at_us` is a window's own instant
plus a compile-time constant, and `due_us` counts from a shared origin — so
every unit latches the same result into the same frame index, regardless of when
its own inference actually finished, which is the one thing here that genuinely
differs per board. A result arriving after its frame was drawn lands in a later
frame here and in the same later frame nowhere else, so it is **counted as
late** rather than hidden: the fix is a larger `present_delay_us` and nothing
else will say so.

---

# Part V — When it breaks

## 17 What each counter was born from

The README's closing line is the design principle of this whole part:

> every real fault was invisible until something counted it, and several were
> actively disguised as something else.

So the counters are not logging. Each one is the instrument that made a
particular fault visible, kept so it cannot come back unseen — and several are
explicitly **measurement only**, acted on by nothing.

A few worth knowing before you need them:

| counter | the fault it was born from |
|---|---|
| `n_phase_drop` | a full phase queue is not a degradation but a **stop** — the servo runs on nothing else |
| `n_refill_withheld` | `i2s_channel_write()` does not block while descriptors are free, so on an empty channel the first writes return at memory speed and phase readings dated inside that window are measured against a reference the DAC is not pacing |
| `n_short_reads` / `n_short_frames` | padded silence is played but was never in the ring, while `samples_played` advances a whole chunk — any firing permanently displaces every later phase point |
| `n_trim_drops` / `n_trim_dups` | a correction you cannot see is a correction you cannot attribute |
| `n_refuse_near_frame` | the first reading that can say whether the frame lane is competing for TX buffers, or the pool is being drained by something that is not us |
| `n_audio_retry` / `n_audio_retry_ok` | judge on the **pair**: tracking means a transiently empty pool, near-zero means the retry should come out again |
| `n_fanout_gap_max_us` | `fan_out()` is *called* on schedule whether or not the `sendto` inside it succeeds, so a gauge stamping on the call reads perfectly through a stretch where the hub emitted nothing at all |
| `n_lead_min_us` (hub) vs `lead-min` (satellite) | the two subtract to a **transit time**; hub flat while satellite collapses means packets left on time and were held after `sendto()` |
| `n_sta_timeout` vs `n_sta_dropped` | a unit that loses power or leaves range raises no event and is dropped silently; the split separates a handled departure from an unseen one |
| `n_alloc_fail` | `CONFIG_HEAP_ABORT_WHEN_ALLOCATION_FAILS` is too violent for a dance floor, and silent failure surfaces as an underrun — which names the symptom, not the cause |
| `n_task_fail` / `CRIPPLED` | a unit that loses a task runs on without it and looks exactly like one that has gone quiet |
| `log_dropped` | non-zero means the merged capture **has holes**, which is worth reading before concluding anything from a gap between two lines |

Two habits show up throughout and are worth copying:

**Split a counter when "how many" and "whose" pick different fixes.**
`s_tx_lane_fail[]` breaks `sendto()` failures out per lane; `tx_fail_summary()`
tallies them by `errno`; `tx_burst_summary()` reports the *shape* of a storm —
beacon-spaced clusters against one unbroken stall.

**Keep a shadow of the decision you replaced.** `s_hub_splice_alt_us` is what
the boundary correction *would* have been on the raw reading instead of the
median. Nothing acts on it. It is reported beside the real figure at the same
boundary — the only place the two are comparable — so the comparison that chose
the median survives the change and a revert has something to check itself
against.

## 18 Reading a HEALTH line

**The log lines are a wire format, not decoration.** `tools/soak/capture.py`
classifies a line by its literal prefix and turns it into metrics columns with
one regex over `key then number` pairs — so a key must be a single hyphenated
word immediately followed by its value. Renaming or reordering one silently
drops a column from the captured metrics. Tidying these strings breaks a tool.
[`satellite/main/telemetry.c`](../satellite/main/telemetry.c) says so at the top
of the file.

| line | cadence | what it is for |
|---|---|---|
| `ARRIVAL 5s:` | every window, **unconditionally** | says the unit is alive and how audio is reaching it — a fault that stops audio would also stop a conditional line |
| `RX 5s:` | only when a fault counter moved | differenced against what it last said |
| `HEALTH:` `TRIM:` `MEM:` | once a minute (`CONFIG_DANCEFLOOR_LOG_PERIOD_S`) | the slow picture |
| `PHASE STEP:` `TRACK DIVERGENCE:` | as they happen | §19 |

`HEALTH` carries uptime, heap (current, minimum-ever, **lowest this window**,
and largest free block), per-task stack headroom, allocation failures, and
cumulative counts of underruns, restarts, splices, retunes, phase drops, short
reads and WiFi events. The hub adds how satellites left — cleanly, unresolved,
or by timing out; the satellite adds its clock source and its LED frame source.

Three readings that are easy to get wrong:

**The windowed heap figure is the point, not the watermark.** A window far below
the current free figure dates a dip *to that line*, which the since-boot
watermark cannot do. Same discipline everywhere: taken and cleared by the line
that prints it.

**Watch internal SRAM, not the whole heap.** With `SPIRAM_USE_CAPS_ALLOC`,
ordinary `malloc` never returns PSRAM, so the ring, DMA buffers, WiFi buffers
and every stack live in internal SRAM — and the whole-heap figure cannot see
that pool run out. `CAP_USABLE_INTERNAL` is the mask that can, and `MEM:` is the
line that prints it.

**Counters that must be read as a pair.** `TRIM:` prints what the servo asked
for beside what playback did about it — the only pair that can disagree. Flat
means the trim is off; both climbing means it is hunting across zero. Likewise
`ARRIVAL`'s `gap-max` beside the hub's `n_fanout_gap_max_us`, and the two
`lead-min` figures.

## 19 TRACK DIVERGENCE

**The measurement the whole project exists to keep small**, printed once per
track by [`play.c`](../hub_s3/main/play.c):

```
TRACK DIVERGENCE: satellite +N us (marker, M ms before this boundary) |
hub spliced +X ms | hub phase +Y us (median +Z us; raw would have spliced +W ms)
```

**Why a track boundary and not a log window.** A boundary is the one instant
that recurs identically in every track: every unit nulls its phase there. So the
reading taken just *before* one is how far the speakers had drifted over a whole
track — comparable across tracks, sessions and builds. A reading taken anywhere
else depends on where in that cycle you happened to look.

**Why it works without any unit measuring another.** Every unit splices by its
*own* phase error against the *same* published timeline, so the difference
between two units' corrections is how far apart they had come to be. Satellites
report theirs over the WiFi that is there anyway (`MSG_SPLICE`, sent from the
probe task — a `sendto()` in the audio path is exactly the kind of thing that
costs a transmit buffer), covering the whole floor rather than the one board
that happens to be wired.

The `marker` figure is the physical answer to the same question, from
[`marker.c`](../hub_s3/main/marker.c): a wire between two boards, timestamping
each unit's pulse when the audio from a *tagged packet* reaches its output.
Marked by **content, not by time** — deriving the instant from a sample count
and a nominal rate was tried and reports servo divergence rather than audio
misalignment, because each unit plays at whatever its own servo last set.

Both are bench instruments in the sense that nothing corrects on either. That is
the point: they measure the open loop.

## 20 Symptom to counter to file

| symptom | look at | then open |
|---|---|---|
| hub gets nothing from the bridge | `sbc_in`'s `pkts`, `hdr`, `crc`, `short` | [`sbc_in.c`](../hub_s3/main/sbc_in.c); §5 — `short` is framing, `crc` is signal integrity |
| a satellite joined and never plays | `sta-nolease`; the satellite's `stream start` line | `MAX_CLIENTS`, `max_connection` and `CONFIG_LWIP_DHCPS_MAX_STATION_NUM` must agree ([`hub.h`](../hub_s3/main/hub.h)) |
| one speaker silent, others fine | `anchor-late` / `anchor-soon`, `anchor_provisional` | `anchor_stream()` in [`rx.c`](../satellite/main/rx.c); §8 |
| audible holes on **every** satellite at once | `tx-fail` lanes, `n_audio_retry`, `tx_burst_summary()` | §7 — a refused audio packet is a hole everywhere |
| holes on **one** satellite | that unit's `gaps`, `fec-parity`, `wifi-drops`, `hub-rssi` | its own link, not the hub's |
| the hub's own audio breaks up | `underruns`, `fed-drop`, `refill-withheld`, `short-reads` | `fed-drop` non-zero means the source stalled longer than the ring holds, and memory is not the answer |
| speakers drift apart over an evening | `TRACK DIVERGENCE`; `TRIM:` pairs | §11, §19 |
| audible jumps at track changes | `splices`, and the `median` vs `raw` figures on the divergence line | `apply_track_boundary()`; §11 |
| repeated `timeline start` | `restarts`, `underruns` | `steer_timeline()`'s hold vs jump; §6 |
| strips lurch on satellites, hub's is smooth | the LED line's `late` and `queued` | §7 (batching) and §13 |
| strips agree, then diverge over minutes | nothing will say so directly | §16 — a pattern reached outside its Frame |
| strips disagree on marginal onsets only | build stamps on both units | §14 — a `beat_detect.h` constant differs |
| unit associates but looks half-dead | `CRIPPLED`, `n_task_fail`, `alloc-fail` | §15 — local analysis does not fit on a classic ESP32 |
| gaps in the captured log itself | `log_dropped`, `log_no_dest` | the capture, not the firmware |

For anything longer than a bench check, `tools/soak/capture.py` reads every
unit's console at once and `analyse.py` reduces a session to the questions a
long run exists to answer. Past sessions are kept beside them in `tools/soak/`.

## 21 Where the invariants are pinned

The host tests are the executable half of this document. They need no hardware
and no ESP-IDF:

```sh
cd components/dancefloor_sync/test && make check   # clock estimator, servo, wire format
cd components/dancefloor_leds/test && make check   # FFT, onsets, patterns, alignment
make check-hops                                    # ...and again at every supported hop
```

| test | what it refuses to let change |
|---|---|
| `test_sync_proto` | wire layouts, the estimator, `AUDIO_MAX_PAYLOAD` not drifting below `SBC_LINK_MAX_PAYLOAD`, `health_msg_t`'s size |
| `test_servo` | every branch of the shared loop — §11 |
| `test_audio_shift` | the faded catch-up's arithmetic |
| `test_align` | two units cut and label their analysis blocks identically — §13 |
| `test_pattern_sync` | a pattern renders identical pixels whatever its unit's join time, render count or drop history — §16 |
| `test_remote_detect` | a unit given frames reaches the same decision as one that analysed the audio itself — §15 |

**Three of them carry deliberately broken cases that the suite requires to
fail.** That is not thoroughness for its own sake: a test that only passes
against correct code has not been shown to detect anything, and every one of
those three guards a fault whose signature is *strips that look fine for the
first minute*.

`tools/pattern_lab` runs the identical analysis and pattern code over a WAV on a
laptop, **compiled from `components/dancefloor_leds` rather than copied**, so it
cannot drift from what the strips do. `tools/satsim` puts N fake satellites on a
laptop to load the hub's unicast fan-out, and checks the hub's XOR parity
against what actually arrived.

Every component and app also carries a `Doxyfile` configured to **check** the
comments rather than render them, so a doc block naming something the code does
not have is a warning. This document holds itself to the same bar.
