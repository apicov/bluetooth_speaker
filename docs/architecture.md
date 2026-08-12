# Dancefloor — architecture and concepts

A phone plays Spotify over Bluetooth. Several battery-powered speakers scattered
around a field play that music **at the same instant**, each pulsing a NeoPixel
strip on the beat.

This document is the way in. It explains the concepts the system is built on
before it explains the system, so it should be readable without prior knowledge
of Bluetooth audio, WiFi, or embedded audio pipelines. Four companion documents
go deeper on the parts that needed it:

| Document | Covers |
|---|---|
| [`clock-sync.md`](clock-sync.md) | The time-sync maths, the servo, measured results |
| [`sbc-link.md`](sbc-link.md) | The wire between the two master chips |
| [`two-chip-master.md`](two-chip-master.md) | Why the master is two chips, memory numbers |
| [`tuning-corpus.md`](tuning-corpus.md) | The recordings the beat detector was tuned against, and the harness that reproduces it |

---

## Part I — The concepts

### 1. Why any of this is hard

The whole problem reduces to one sentence: **two computers that have never
shared a wire must agree on when to move a speaker cone.**

Three physical facts make that difficult.

**Sound is slow.** 343 m/s, so 1 ms of timing error is 34 cm of apparent speaker
movement. Below about 5 ms nobody can hear a difference; above ~20 ms you hear
two speakers instead of one, and around 30–50 ms it becomes a distinct echo.
That sets the budget: **under 5 ms between any two units**, and there is no
point chasing microseconds because walking two paces past a speaker changes the
arrival time more than the whole error budget.

**Crystals are not identical.** Each board's timekeeping comes from a quartz
crystal accurate to roughly ±10–40 parts per million. Measured between our two
boards: **10.6 ppm**. That sounds tiny until you integrate it.

| Elapsed | Divergence if never corrected |
|---|---|
| 1 minute | 0.6 ms |
| 10 minutes | 6.4 ms |
| 1 hour | 38 ms |
| A 4-hour party | **153 ms** |

So starting in sync is not enough. Sync has to be maintained continuously,
forever, for as long as the music plays.

**Radio delivery is unpredictable.** A WiFi packet takes between 4.7 ms and
14 ms to make a round trip on a quiet direct link, and much longer if it has to
be retransmitted. Any scheme where the master says "play now" fails, because
"now" is already wrong by an unknown amount when the message lands.

### 2. Bluetooth, and the choice it forces

Bluetooth is really two incompatible technologies sharing a brand.

**Bluetooth Classic (BR/EDR)** is the older one. It carries streaming audio and
is what every Bluetooth speaker uses. **Bluetooth Low Energy (BLE)** is a
different radio protocol designed for sensors and watches — small, occasional
messages, very low power. BLE cannot carry a music stream at all.

The profile we need is **A2DP** — *Advanced Audio Distribution Profile*, the
standard for streaming stereo audio to a speaker. It is Bluetooth Classic only.

This forces the chip choice for the whole project. Of the ESP32 family, **only
the original ESP32** (WROOM/WROVER) has Bluetooth Classic. The newer, faster,
cheaper S3, C3 and C6 are BLE-only and can never receive A2DP. Everything else
in this project could run on a newer chip; the Bluetooth receiver cannot.

**Decision: classic ESP32 everywhere**, master and satellite alike. One part
number, interchangeable spares, and any board can be promoted to master if one
dies in a field at 2 a.m.

There is a second profile in play. **AVRCP** — *Audio/Video Remote Control
Profile* — is the side channel that carries track title, artist and album, and
notifies when the track changes. That notification turns out to matter for
synchronisation, not just for display: a track change is the one moment a
splice is inaudible, so it is when every unit nulls its accumulated phase error.
See §10.

### 3. Codecs, and what "lossy" costs here

Raw CD-quality audio is 44,100 samples per second × 2 channels × 16 bits =
**1.411 Mbps**. Bluetooth cannot carry that, so A2DP compresses.

**SBC** — *Subband Coding* — is the mandatory A2DP codec that every device
supports. It runs at roughly **328 kbps**: about a quarter the size, and lossy,
meaning information is discarded and cannot be recovered.

This is the quality ceiling of the entire project, and it is set before any of
our code runs. The phone compresses Spotify's already-compressed audio into SBC,
and that is the best the system will ever sound. Nothing downstream can improve
it. What downstream *can* do is avoid making it worse.

That framing led to the single most useful decision in the audio path: **the
system moves SBC around, not decoded audio.**

```
phone ──SBC 328 kbps──▶ bridge ──SBC──▶ hub ──SBC──▶ satellites
                                          └── decodes for its own speaker
```

Decoding once at each final destination rather than early and re-transmitting
the result costs **nothing in quality** — it is the same SBC bitstream, decoded
at the far end instead of the near end — while quartering every link's
bandwidth. Measured, that difference was the line between a working system and
a broken one (§9).

### 4. WiFi, and how packets actually get lost

There is no router in a field, so the hub runs a **SoftAP** — it *is* the access
point, and satellites associate with it directly. One radio hop, no
infrastructure, nothing to configure on site.

Audio moves over **UDP**. TCP is wrong for this: it guarantees delivery by
retransmitting until success, which for live audio means a late packet arriving
long after the moment it was needed, while everything behind it queues up. For
real-time audio, **a lost packet is better than a late one**. UDP simply drops
it and moves on, and the receiver fills the gap.

The choice that mattered far more, though, was unicast versus multicast.

**Multicast** sends one copy addressed to a group; every subscriber receives
that single transmission. It is elegant, and airtime does not grow as speakers
are added. This is what the project used originally.

**Unicast** sends a separate copy to each recipient. More airtime, more packets.

Multicast lost about **20% of audio packets** at every PHY rate tried — 1 Mbps,
6 Mbps, 24 Mbps alike. The cause is not bandwidth and not interference. It is in
the 802.11 standard: **multicast frames are never acknowledged and therefore
never retransmitted.** A unicast frame that collides is retried at the link
layer within microseconds and usually gets through. A multicast frame that
collides is simply gone, and nothing anywhere notices.

That is a 20% packet loss floor that no amount of tuning can reach.

**Decision: unicast to every registered listener.** With ≤8 speakers the extra
airtime is affordable and the loss goes to approximately zero. Multicast has
been removed from the audio path entirely.

Registration is implicit and has a pleasant property: the hub sends audio to
whatever has sent it a **time-sync probe** recently. A unit that is keeping its
clock synchronised is by definition alive and listening, so one mechanism does
both jobs and there is nothing to configure. Stop probing and you stop receiving.

"Recently" is **2 seconds**, and it used to be 10. The window matters more than a
registry timeout usually does, because during it the hub keeps unicasting audio
and analysis frames at a station that is not there, and each of those sends takes
a 1152-byte DMA buffer the driver cannot deliver and will not free until it gives
up. Pulling a satellite mid-track at the old timeout produced **124 failed
allocations over exactly ten seconds**, free heap down to 4580 bytes, and
recovery the instant the timeout expired. Not shorter than 2 s either: a
satellite forgotten in error gets nothing until its next probe, and at a 250 ms
probe period against ~150 ms of ring that is an underrun rather than a glitch.

Two events bracket the timeout, so it only ever handles the case neither can see:

- **Joining.** A satellite goes on the send list when the DHCP server hands it an
  address, rather than a quarter-second later at its first probe. This needs a
  static ARP entry seeded alongside it, and that detail is the whole of it —
  registering at DHCP time *without* one is worse than not registering at all,
  because `dhcpserver.c` removes its own static entry immediately before raising
  the event, so unicasts pile into a pending-ARP queue that drops its overflow.
  Measured: 161 tx-failures on a clean join. With the entry seeded, zero.
- **Leaving.** A clean disassociation drops the satellite at once, resolving its
  MAC to an IP through the DHCP lease table. That covers a unit being reflashed
  or restarted, which is what actually happens.

What the timeout is left with is the ungraceful departure — power cut, out of
range — where no event fires at all and the AP notices inactivity far later. It
counts those separately (`sta-timeout`), because without that a run where a
satellite vanished mid-track and a run where nothing happened printed the same
line.

### 5. Getting sound out of a chip: I2S, DMA, and why not the internal DAC

**I2S** (*Inter-IC Sound*) is the standard serial bus for digital audio between
chips. Three signals:

| Signal | Meaning |
|---|---|
| **BCK** | Bit clock — one pulse per bit |
| **WS** / LRCK | Word select — which channel this sample belongs to; its frequency *is* the sample rate |
| **DATA** | The audio bits |

One side generates the clocks (**master**) and the other follows them
(**slave**). Remember that distinction — it caused the project's longest
debugging session (§8).

**DMA** — *Direct Memory Access* — is what makes audio playback survivable. The
I2S peripheral fetches samples straight from memory without waking the CPU. The
CPU refills a buffer every few milliseconds; the hardware does the rest. Without
it, every interrupt anywhere in the system becomes an audible click.

The descriptors are a **circular list**, and that has a consequence worth knowing
before you meet it: a channel left enabled with nobody writing does not fall
silent — it replays the buffers it already holds, forever. At 6 descriptors of
`AUDIO_FRAMES` that is the last 34.8 ms looping at 28.7 Hz, which is far louder
and more unpleasant than the music it came from. Both playback tasks park without
disabling the channel when their ring starves, so every stall used to end in that
buzz: pull the Bluetooth link mid-track and the floor howled until the phone came
back. The fix is one flag, `auto_clear` on the channel config, which has the
driver zero each buffer at its own EOF — after it has played, before the writer
is handed it back, so it cannot race the writer. A stall now drains to digital
zero within one traversal and stays there. Both units set it; the boot line says
`silence on starve` when they do.

The classic ESP32 has built-in DACs, but they are **8-bit** — 48 dB of dynamic
range against 96 dB for 16-bit. Audibly bad: quiet passages sit in obvious hiss.
The satellite carried a build option to use them for bring-up with no hardware
wired; it was removed on 2026-08-12, having outlived its purpose and being
classic-only in a project acquiring an S3 satellite. The output is an external
**PCM5102A** I2S DAC.

### 6. Serial links, and the difference between two kinds of clock

Three serial links appear in this project and the distinction between them is
what §8 turns on, so it is worth having first.

**UART** is asynchronous. One wire, no clock at all. The receiver recovers
timing from the **start bit** at the beginning of every byte, resynchronising on
each one, so the two ends only need to agree on the baud rate to within a
percent or so. Nothing is shared and nothing can drift out of step.

**I2S** and **SPI** both send a clock alongside the data, and are usually filed
together as "synchronous" for that reason. For our purposes they are not alike:

- **I2S is continuous.** The clock free-runs and the slave tracks it for ever,
  with no boundary at which alignment is re-established.
- **SPI is transactional.** A chip-select line frames every transfer and the
  slave's bit counter resets at each assertion. Nothing carries over between
  frames.

That difference is the whole of §8. It is why an I2S slave failed here, why a
UART was the fix, and why the link is now SPI without being the same mistake
twice.

---

## Part II — The system

### 7. The signal path, end to end

```
  ┌─────────────────── MASTER ENCLOSURE ────────────────────┐
  │                                                          │
  │   ESP32 #1: bt_bridge          ESP32-S3 #2: hub          │
  │   ┌──────────────────┐         ┌──────────────────────┐  │
 phone │ A2DP sink        │   SPI   │ SoftAP + clock master│  │
 ──────▶ AVRCP metadata   ├────────▶│ SBC decoder          │  │
  BT  │ forwards raw SBC │  1 MHz  │ presentation timeline│  │
  │   │ no decode at all │ 4 wires │ I2S ──▶ DAC ──▶ amp  │  │
  │   └──────────────────┘ +hshake │ SPI ──▶ NeoPixels    │  │
  │                                └──────────┬───────────┘  │
  └───────────────────────────────────────────┼──────────────┘
                                              │ WiFi, UDP unicast
                          ┌───────────────────┼───────────────────┐
                          ▼                   ▼                   ▼
                   ┌─────────────┐     ┌─────────────┐     ┌──────────────┐
                   │ satellite 1 │     │ satellite 2 │     │ desktop      │
                   │ decode+play │     │ decode+play │     │ (Python)     │
                   │ + LEDs      │     │ + LEDs      │     │ listen/record│
                   └─────────────┘     └─────────────┘     └──────────────┘
```

**The master is also a speaker.** The hub chip drives its own DAC and its own LED
strip, exactly like a satellite. It is not a base station.

Three firmware images live in this repository, plus two host tools and a Python
client that lives elsewhere:

| Project | Role |
|---|---|
| `bt_bridge/` | Chip A. Bluetooth only. Receives A2DP, forwards raw SBC over SPI |
| `hub_s3/` | Chip B, an ESP32-S3. WiFi SoftAP, clock master, decoder, DAC, LEDs, streamer. The classic-ESP32 `hub/` was retired 2026-08-12 |
| `satellite/` | Every additional speaker. Receives, decodes, plays, lights |
| `tools/pattern_lab/` | The LED pipeline on a laptop, compiled from the firmware sources |
| `tools/tuning/` | The detector's sweep harness, behind [`tuning-corpus.md`](tuning-corpus.md) |

### 8. Why the master is two chips

Not a preference — a measurement.

On a single chip, Bluedroid (the Bluetooth stack) and the WiFi stack together
left **164 bytes** of DRAM free. `esp_wifi_init()` failed outright at first, and
once it started, Bluetooth could not allocate its buffers when a phone
connected. Making it run at all meant halving the FFT, cutting the lead time,
and shrinking every buffer to a quarter of its proper size.

Splitting the master across two ESP32s returns all of it:

| Firmware | DRAM used | Free |
|---|---|---|
| Single-chip master | 92.4 kB / 124.6 kB | **32 kB** |
| `bt_bridge` | 61.0 kB / 124.6 kB | 63.6 kB |
| `hub` | 59.8 kB / 180.7 kB | **121 kB** |
| `satellite` | 37.7 kB / 180.7 kB | 143 kB |

It solved three problems, only one of which was expected:

**Memory.** ~200 kB spare per chip instead of two stacks fighting over 320 kB.

**Radio contention.** Bluetooth and WiFi share the 2.4 GHz band. On one chip
they also share one radio and one antenna, and must take turns. Two chips means
two radios; the contention simply does not arise.

**Burstiness.** This one was a surprise, and the original claim here was wrong.
Bluetooth delivers audio in ~100 ms lumps, so on the single-chip master the
presentation timeline raced ahead during a burst and fell behind while the buffer
starved, tripping the resync guard on every cycle. This section used to say the
bridge chip absorbed that entirely and the hub received evenly paced audio.

It does not. `sbc_in` now measures the longest silence between audio packets in
every window, and **every** window contains one of 79–112 ms, median 100, against
a ~43 ms nominal spacing. The burstiness is still there end to end; the bridge
moved it rather than removing it.

What the split did buy is real — memory, no radio contention, and a decode that
is not competing with Bluedroid — but the timeline still oscillates by ±130 ms
with the burst pattern, and the hub's streamer had to be taught to slew rather
than jump because of it. See clock-sync.md §9.

Cost: about $5 and one wire, on the master only.

#### The wire between them, and the bug that chose it

The first design sent **decoded PCM over I2S**, bridge as master, hub as slave.
It silently lost **~1.15% of frames**, and finding out why took longer than
anything else in the project. Every intuition was wrong:

| Suspect | Ruled out by |
|---|---|
| Bridge clock wrong | Logic analyser: WS exactly 44,100 Hz |
| I2S driver broken | Self-loopback on one board: zero loss over 45 s |
| WiFi / DMA bandwidth | Loopback with no WiFi loses identically |
| CPU load from the FFT | 577/s before, 577/s with the visualiser disabled |
| Read scheduling | Max read gap only 0.7% above average |
| Signal integrity | Hardware edge counter: WS 44,099 exact, BCLK ratio 32.000 exact |
| **Asynchronous slave clocking** | **Inverting clock ownership: loss to zero** |

**The ESP32's I2S slave receiver drops frames when sampling a clock generated by
a different chip's crystal.** Everything on the wire was perfect; the peripheral
did not deliver what arrived at its pins.

UART sidesteps the whole category, because **no clock is shared** — each byte
resynchronises on its own start bit. It also carries a quarter of the bytes (SBC
rather than PCM) and needs one signal wire instead of three. That was the link
for most of this project's life.

It ran at **500 kbaud**, which is 84% utilisation and tighter than anyone would
choose. The ceiling was a property of breadboard jumper leads, not the protocol:

| Baud | Result per 5 s |
|---|---|
| 1,000,000 | ~50% of packets corrupt |
| 750,000 | 20–30 bad sync, 15–20 CRC errors |
| **500,000** | **0–2 bad sync, 0–1 CRC errors** |

#### And why it is SPI now

84% of a 50 kB/s wire leaves nothing, and it was being paid for in audio. The
SBC endpoint advertised `max_bitpool = 250` — about five times what the wire
could carry — with nothing enforcing the difference, so a phone choosing a
higher bitpool simply had the surplus dropped. `max_bitpool` came down to **53**
to make the advertisement honest, which is a quality ceiling set by jumper
leads.

SPI removes it: 1 MHz is 20× the UART and 10 MHz is 25×.

The objection is obvious — that is a shared clock again, and a shared clock is
what §8 is about. The answer is the distinction in §6. I2S is continuous, so a
slave tracking a foreign crystal accumulates error with no boundary to discard
it at; SPI is transactional, CS resets the bit counter every frame, and the
worst a clock difference can cost is one frame that the CRC catches. **That is
an argument rather than a measurement**, and `crc` in the `sbc_in` line is the
instrument that tests it.

CS also pays for itself in deleted code: one assertion is one packet, so the
sync bytes, the resync scan and the whole boundary-recovery half of the receiver
are gone.

The cost is pins — four instead of one, which on an eleven-pad XIAO ended the
marker/monitor instrument — and a handshake line, because ESP-IDF's `spi_slave`
loses any transfer clocked with nothing queued.

Full detail, and the bring-up order, in [`sbc-link.md`](sbc-link.md).

### 9. Over the air

Each SBC packet from the bridge is forwarded to every registered listener as one
UDP datagram carrying the compressed audio plus a header:

```c
struct {
    uint8_t  type;          /* MSG_AUDIO */
    uint8_t  format;        /* SBC */
    uint8_t  marker;        /* 1 = pulse the sync GPIO when this audio plays */
    uint8_t  restart;       /* 1 = a track boundary starts here */
    uint16_t payload_len;
    uint32_t seq;           /* so a loss is detectable */
    uint32_t sample_rate;
    uint32_t frames;        /* PCM frames this decodes to */
    int64_t  play_at;       /* master-clock microseconds */
    uint8_t  payload[];     /* the SBC itself */
};
```

Two fields deserve attention.

**`play_at`** is the whole synchronisation scheme in one field (§10).

**`seq`** exists because UDP loses packets, and a gap must be filled with
*exactly* the right amount of silence. Get it wrong and every subsequent sample
plays early, so the entire stream slides out of alignment and never recovers.

Packets are ~1041 bytes at 50/s, comfortably under a 1500-byte MTU so nothing
fragments. Had this been PCM it would be 172 packets/s at 1.4 Mbps — measured at
~7% of packets rejected by a full transmit queue plus ~13% lost on air.
Forwarding SBC quartered both.

Three other message types share the socket. `MSG_TIME_REQ`/`MSG_TIME_RSP` are the
clock probes (§10), `MSG_META` carries the track title and artist the bridge got
over AVRCP, and `MSG_FRAME` carries one analysis frame — the hub's own, for units
built to draw what it decided rather than analyse for themselves (§12). Frames go
unicast for the same reason the audio does, and cost ~5 kB/s per listener against
the 30–40 the audio already uses. The hub publishes them whenever its visualiser
is enabled; a satellite doing its own analysis simply ignores them.

`MSG_FRAME` carries a `len` alongside the payload, which is the one piece of
defensive framing in the protocol. A hub and a satellite on different builds is
the mismatch this system is most likely to meet, and a frame of the wrong shape
reinterpreted silently would put garbage on a strip rather than nothing. The
receiver checks the length, refuses the frame, and says so exactly once —
43 complaints a second would bury everything else in the log.

Message type **3 is a deliberate gap**. It was `MSG_BLINK`, for the M4 bring-up
harness, and it is not reused: a board still running old firmware speaks a
protocol this one no longer knows, and an unknown message type is a much better
failure than a silently reinterpreted one.

### 10. Synchronisation

This is the heart of the project and has [its own document](clock-sync.md). What
follows is the shape of it in four layers, each solving what the one before
could not.

#### Layer 1 — agree on what time it is

Each board's `esp_timer_get_time()` counts microseconds since *that board*
booted, so two boards disagree by however long apart they were switched on
(measured once: 124 seconds).

There are now **two** ways to close that gap, and the round trip described here
is the fallback. A satellite prefers **802.11 TSF** — the WiFi MAC's own
microsecond counter, carried in every beacon and timestamped in hardware by the
receiving MAC — and drops back to the round trip when TSF is unavailable. TSF
has no network delay in the measurement, so it carries none of the asymmetry
described below. clock-sync.md §10 has the numbers; the two agreed on rate to
0.12 ppm over ten minutes before either was trusted.

The round trip, still in use as the fallback: the satellite stamps a probe going
out, the hub stamps arrival and reply, and the satellite stamps the return. Four timestamps
around one round trip give both the offset and the round-trip time:

```
offset = ((t2 − t1) + (t3 − t4)) / 2
rtt    =  (t4 − t1) − (t3 − t2)
```

The offset is exact **if the two directions took equally long**. They do not,
and that asymmetry is the error floor — invisible to the measurement, because
the satellite only ever observes the sum.

Which is why the estimator **keeps the sample with the lowest RTT** and discards
the other nine rather than averaging. A fast round trip had little queuing in
either direction, so its two halves were closest to equal. This is what PTP
does, and the effect was large:

| Selection | Offset spread |
|---|---|
| Median of 10 | ~1080 µs |
| **Minimum RTT** | **~117 µs** |

#### Layer 2 — schedule the future instead of announcing the present

This is the simple idea that makes it work.

**The hub never says "play now."** Every packet says *"play this at master-time
T"*, where T is 200 ms ahead. Each unit converts to its own clock —
`local = T − offset` — and waits.

Network jitter stops mattering. The packet can take 4 ms or 14 ms; as long as it
arrives before T, every unit starts the same sample at the same instant. Nothing
passes between units at the moment of playback: each keeps its own appointment.

You are not synchronising the event. You are synchronising the **schedule**.

The cost is latency — nothing can happen sooner than the lead time. Total
end-to-end delay is ~150–200 ms of Bluetooth plus 200 ms of buffer, so roughly
half a second. Fine for music, useless for video. That is an accepted trade, not
a bug.

#### Layer 3 — hold position, continuously

Starting together is not staying together, because of the 10.6 ppm crystal
difference. Audio cannot jump to a corrected offset — that is an audible click —
so instead each unit trims its **playback rate** slightly, playing 44,098 or
44,102 Hz instead of 44,100 until the error is walked off.

The subtlety is *what* to servo on. The obvious choice is buffer depth: keep the
ring at 200 ms and you must be playing at the rate audio arrives. True, but it
controls **rate without controlling position**, and buffer depth moves with
network jitter, so each unit chases noise. Two units chasing different noise ran
~0.03% apart at any moment.

Measured: **10–25 ms of wander** between hub and satellite while each unit's own
buffer sat perfectly stable.

The fix is to servo on **phase**: every packet already carries the instant its
first sample is due, so recording that against the ring position it lands at
gives a direct reading of *where we are versus where the timeline says we should
be*. Both units servo on that — the hub included, since it publishes the
timeline and must hold itself to it or the reference itself moves. Buffer depth
is kept only as a guard against running empty.

Gain matters more than it looks. At a 40-second correction time both units
converged and then **overshot to +10 ms and oscillated** — the buffer takes tens
of seconds to respond, so the loop was still correcting after the error was
gone. At ~100 seconds it settles. Real drift is ~0.8 ms per minute, so the loop
can afford to be far gentler than the disturbance it corrects.

#### Layer 4 — re-anchor at track boundaries

The servo walks error off over a couple of minutes, which leaves the start of a
session audibly out. Correcting faster means skipping or inserting audio, and
that is a splice — normally audible.

**A track change is the one moment a splice is inaudible**, and AVRCP reports it
rather than making us guess. Silence detection was considered and rejected: a
quiet passage in a song would trigger it and splice something audible, whereas a
track change is unambiguous.

The correction is applied when playback *reaches* the flagged packet, not when
the notification arrives — at that moment the buffer still holds ~200 ms of the
previous track, and correcting immediately would cut its ending. Hence the
`restart` flag travelling with the audio rather than as its own message.

Corrections shrink over a session as the servo converges:

```
track boundary: skipped 8 ms to null phase
track boundary: skipped 2 ms to null phase
```

#### Where it landed

| Measurement | Value |
|---|---|
| Clock offset error between boards | ~300 µs |
| Playback start vs schedule | +5 µs (hub), +1 µs (satellite) — same instant |
| Smoothed phase | ±1–2 ms |
| Instantaneous phase | ±5 ms (delivery jitter) |
| **Hub-to-satellite audio alignment** | **0.2–3 ms** |
| Buffer depth | 165–250 ms around a 200 ms target |
| SBC link integrity | `crc 0 gaps 0` |

Against a 5 ms audibility threshold, comfortably inside.

### 11. Playback

Each unit runs the same shape of pipeline:

```
UDP ──▶ seq check ──▶ SBC decode ──▶ ring buffer (64 kB) ──▶ I2S DMA ──▶ DAC
              │                            ▲
        gap? insert silence          phase servo trims the rate
```

The SBC decoder is vendored from Bluedroid into `components/sbc_decoder/` rather
than pulled in with the whole Bluetooth stack — a satellite has no Bluetooth and
should not carry ~950 kB of stack to borrow one decoder.

A gap in `seq` is filled with the right number of silent samples, computed from
the `frames` field, so the timeline stays honest. The alternative — just
skipping — makes everything afterwards play early forever.

### 12. LEDs and beat detection

By default each unit analyses **its own copy** of the audio. Sending analysis
results over the network would add a second thing to synchronise; the audio is
already synchronised, so anything derived from it locally is synchronised too,
for free. (Frames *can* now be sent instead — see "Two sources" below — but that
is a way of removing the need to prove an algorithm identical across units, not a
retreat from this idea.)

```
PCM ──▶ 1024-point FFT (esp-dsp) ──▶ 4 bands ──▶ spectral flux ──▶ onset ──▶ Frame
Frame ──▶ (queue, wait for due_us) ──▶ pattern ──▶ strip
```

**Spectral flux** measures how much energy *increased* since the last frame, per
band. A drum hit is a sudden broadband rise; a sustained note is not. Comparing
against a rolling ~0.5 s history makes it adaptive to loud and quiet passages,
and a 120 ms refractory period stops one kick registering three times.

Four bands, roughly kick / low-mid / presence / air.

#### The window and the hop are two different numbers

They used to be one, and nothing said so. **The window is 1024 samples** and that
is what fixes the 43 Hz bin width, the band edges and every tuning figure ever
measured. **The hop — how far the analysis advances between windows — defaults to
512**, a 50% overlap, giving 86 frames a second. Changing the window would be a
retune; changing the hop is not.

The detector was re-swept at hop 512 over 206 recordings and **no constant
moved**: both flux floors and `BEAT_HIST` are the same numbers at 512 as at 1024.
[`tuning-corpus.md`](tuning-corpus.md) is the corpus, the commands and the
ladders. Two of those constants needed no measuring at all and the document says
why — the refractory periods are already in microseconds, and the two
`threshold_k` values are provably invariant, since scaling flux scales the mean
and the standard deviation identically.

What the hop *does* change cannot be tuned away, and it is worth knowing before
comparing two captures: **which strokes fire**. Flux is a frame-to-frame
difference, so at hop 512 it is differenced across 11.6 ms of audio and at 1024
across 23.2 ms, and those are different measurements of the same music. Only 64%
of the booms at 1024 have a boom at 512 within one window of them, against 91%
for a floor change at a fixed hop. Roughly a third of the strokes move. That is a
property of the hop, not a tuning error, and no floor value reduces it.

The hop must divide the window and be a power of two, so it is a menuconfig
*choice* rather than a number — a bad value cannot be selected at all instead of
failing the build.

#### Analysis and drawing are separate stages

A frame is computed when the audio for it arrives, and drawn when the instant it
names comes round. Those are two tasks with a queue between them, and the split
is what lets a frame be computed on one unit and drawn on another at all.

It also means the render task is the one thing in the component that needs a
clock: `due_us` is a master-clock instant, so a satellite has to convert it. The
offset is passed as a *function* rather than a number, because the satellite's
offset is slewed toward the live estimate at 200 ppm — a value copied once would
go stale at exactly the crystal difference, which is the same bug clock-sync.md
§9 records in the audio path. The hub leaves it unset, since local time is master
time there.

When the timeline restarts — a re-anchor, an underrun recovery — the queue is
flushed and the strip goes **dark** rather than holding its last frame. Every
label still queued describes an instant on a timeline that no longer exists, and
drawing them anyway produces a burst of animation from the old origin. A lit
strip looks like it is working; a dark one says plainly that there is nothing to
show. A splice is different and does *not* flush: it moves audio around within a
timeline, so queued labels stay true.

**LEDs must run off the presentation timeline, not the arrival timeline.**
Otherwise the lights lead the sound by the entire buffer depth — 200 ms, which
looks obviously wrong.

Both units therefore feed the analysis from the point where samples are handed
to the DAC: `write_audio()` on the satellite, and immediately before
`i2s_channel_write()` in the hub's `local_play_task`. The hub originally fed
from the packet-arrival path in `sbc_in.c` and its lights genuinely did run
~200 ms early.

#### Agreeing without communicating

The pulse agrees across units for free, because each analyses audio that is
already synchronised. Everything else has to be *derived from the audio* rather
than from anything local, and that turns out to be the whole difficulty.

Two things are needed, and both come from the same source.

**Blocks must be cut at the same content positions.** The detector takes a
1024-sample window every `HOP_N` samples, and if two units cut them at different
offsets a transient near a boundary is split on one board and centred on the
other — the marginal onsets then fire on one strip and not its neighbour.
Boundaries are derived from `play_at`, the instant the audio is *scheduled* to be
heard, which every unit receives identically. Never from a clock read locally:
units reach that line milliseconds apart, and 3 ms of skew puts them 132 samples
out of 1024.

The hop being a power of two that divides the window is what keeps this safe when
the hop changes: it makes the finer grid a *refinement* of the coarser one rather
than a cut across it, so two units on different hops still agree about where a
window may begin.

**Animation must be a function of shared time, not of local counts.** `due_us`
comes from the block index, so the same audio carries the same label on every
unit. A hue that accumulated per render would drift apart, because units do not
render the same number of times — audio arrives in different-sized lumps and a
starved unit renders extra frames. Nothing is integrated, so nothing accumulates
error, and a unit that stalls rejoins in the right place instead of lagging for
ever.

Both properties depend on the visualiser's count of what has passed through it
staying true. Three things break it loudly: a short send when the analysis task
falls behind, a **splice** at a track boundary, and a **retune**, which discards
the DMA buffer — audio the playback task has already counted and already fed
here, that nobody heard. Each calls `visualiser_realign()`, which re-derives the
origin from the next scheduled instant at the cost of one dropped block. Without
it the two strips step apart at every one of those events and never recover;
with it they resynchronise within a block.

That list was presented here as complete. It never was, and the ones outside it
break the count in exactly the same way while calling nothing at all:

- audio **dropped before it reaches the playback path**, so content the timeline
  still accounts for never arrives at the visualiser — the hub's local ring and
  the satellite's receive ring both drop when full, and both only count it
- a **short read** from the playback ring, zero-filled to a whole chunk: audio
  the timeline does *not* account for, invented and fed here as if it were real

Neither knows it is doing anything to the LEDs, so neither reports it, and the
result is a permanent offset on one unit. Measured on the host over forró-shaped
material: two units whose audio is offset by 2.9 ms render **15% of frames
visibly differently**, and at one packet — 42 ms, what a single unreported drop
used to cost for good — **3.4% of frames have one strip lit and the other dark**.
That is what "they look synchronised but sometimes one is lit and the other
isn't" was.

So the count no longer depends on callers being exhaustive. `visualiser_feed()`
is handed `due_master_us` on every call, which is where the timeline says this
audio is, and it knows where its own count says it is; if the two part company by
more than `ALIGN_DRIFT_US` it re-derives the origin itself, whatever caused it.
Still call `visualiser_realign()` — it corrects at the instant of the event
rather than once the error has grown — but forgetting to is no longer permanent.

`test_align.c` and `test_pattern_sync.cpp` pin both halves mechanically — see
§15.

#### Two sources for a frame

Everything above is about making two units *independently* reach the same
decision. There is now a second way to get agreement, which is to have only one
decision: the hub publishes each frame it computes, and a unit built for
`CONFIG_DANCEFLOOR_LED_SOURCE_REMOTE` draws those instead of analysing anything.
No FFT and no detector run there; the analysis task is not even started.

| Source | What the unit does |
|---|---|
| `LOCAL` (default) | Runs its own FFT and both detectors. Costs the analysis, gives the pattern the full spectrum |
| `REMOTE` | Runs no analysis. Draws the hub's frames at the instant each names |

The received frame goes into the same queue local analysis fills and is drawn the
same way, so the two sources are interchangeable by construction rather than by
agreement. Nothing is corrected against anything in either case — a remote unit
still converts `due_us` with its own offset and keeps its own appointment.

**Mixing the two across a floor is intended, not a hazard.** A unit on `LOCAL`
can run a different pattern or a different detector entirely. What that costs is
the strength of the guarantee: it is no longer "every strip agrees" but "units
taking frames from the same source render identically". An *unintended* mismatch
is still the expensive bug, which is why both the source and the hop are printed
at startup and carried in the `HEALTH` line — a unit doing its own analysis
cannot detect that its neighbour is on a different hop, because nothing crosses
between locally analysing units, and that is exactly the property that makes them
stay in step. Two consoles settle it.

`Frame::mag` — 512 floats — does not travel and is null on any received frame.
Patterns using the quantised `spec[]` work either way. This is also the constraint
that shapes the detector's tuning: anything a remote unit might one day be asked
to compute has to be derivable from the four `band` floats that do travel, which
[`tuning-corpus.md`](tuning-corpus.md) §9 makes executable rather than a promise.

#### Where the code lives

`components/dancefloor_leds/` is shared by the hub and every satellite: the same
FFT, the same detector, the same patterns, one copy. It is split so that
everything deciding what the lights *do* has no platform dependencies:

| | |
|---|---|
| `analysis.cpp` | FFT, band split, onset detection — audio in, `Frame` out |
| `patterns.cpp` | `Frame` in, pixels out |
| `visualiser.cpp` | the parts that cannot be shared: stream buffer, block alignment, the analysis and render tasks and the queue between them, strip |
| `fft_host.c` | a radix-2 FFT for the host, where esp-dsp does not exist |

That split exists so `tools/pattern_lab` can run the identical pipeline over a
WAV on a laptop — same analysis, same patterns, compiled from these files rather
than copied. Designing a pattern is a taste problem, and taste needs turnaround
that reflashing two boards cannot give.

Its `Kconfig` owns the LED options for both firmwares — GPIO, count, strip model,
the analysis hop, the frame source, and a **brightness cap defaulting to 10%**,
which is the figure the power budget assumes and which dark-adapted eyes cannot
distinguish from full outdoors.

The strip itself is driven through `components/led_strip_wrapper/`, an RAII C++
wrapper vendored from the `esp32c3_neopixel` project:

```cpp
LedStrip strip{GPIO_NUM_18, 60, LedStrip::Type::WS2812, LedStrip::Backend::SPI};
strip.set(i, r, g, b);
strip.show();
```

The `Backend` parameter is the one thing added to it, and it exists because of
the next section. It defaults to `RMT`, so the upstream project — a separate
repository, left untouched — still compiles against this copy unchanged.

#### Never bit-bang WS2812

WS2812 pixels are timed to ~150 ns. The common Arduino-style approach disables
interrupts for the whole strip update, which starves the I2S DMA and produces
clicks. Two hardware options can do it without blocking:

**RMT** — a peripheral for generating arbitrary pulse trains. The obvious
choice, and the one this project used first. It **wedged** after some minutes:

```
rmt_tx_disable: channel can't be disabled in state 3
```

`led_strip` 3.0.3 enables and disables the RMT channel on every frame, and
`rmt_disable` races the transmit-done interrupt through its state machine.

**SPI + DMA** — encode each pixel bit as a byte pattern and clock it out of the
MOSI pin. Less elegant, reserves a whole SPI bus to use one pin, and has no
state to race. **This is what the project uses.**

A **74AHCT125 level shifter** is required: WS2812 wants 5 V logic and the ESP32
drives 3.3 V. It works on a bench often enough to be misleading and fails
outdoors in the cold.

---

## Part III — Decisions and operation

### 13. Configuration, and why each setting is what it is

Every non-default setting in the project, with its reason. Most of these were
paid for.

#### Both radios

| Setting | Value | Why |
|---|---|---|
| `CONFIG_FREERTOS_HZ` | `1000` | At the default 100 Hz, `vTaskDelay` granularity is **10 ms** — ten times the entire sync error budget |
| `esp_wifi_set_ps(WIFI_PS_NONE)` | no power save | Power save parks the radio between beacons and adds tens of ms to exactly the packets being timed |
| SoftAP channel | pinned to 11 | Costs nothing, and helps Bluetooth's adaptive frequency hopping route around us |

#### `bt_bridge`

| Setting | Value | Why |
|---|---|---|
| `BTDM_CTRL_MODE_BR_EDR_ONLY` | `y` | No BLE — A2DP is Classic, and BLE support costs memory for nothing |
| `BT_A2DP_USE_EXTERNAL_CODEC` | `y` | Selects the **undecoded** A2DP callback. This chip forwards SBC and never decodes. Not a choice: unset, Bluedroid decodes internally and the callback goes unfed, so the hub silently gets nothing |
| `ESPTOOLPY_FLASHSIZE_4MB` + custom partitions | | Bluedroid alone is ~950 kB; the stock 1 MB factory partition leaves 10% free |
| No WiFi at all | | The entire point of the split |
| `BT_ACL_CONNECTIONS` | **left at 4** | See below |

> **Do not set `CONFIG_BT_ACL_CONNECTIONS=1`**, even though only one phone ever
> connects. It sizes Bluedroid's buffer pools, and shrinking them throttles the
> stream to a few percent of full rate. The signature is thoroughly misleading:
> the link stays healthy and *active*, the phone shows connected and playing,
> but the sink ringbuffer underflows continuously with growing gaps — 1.5 s,
> 2.9 s, 4.6 s, 8.9 s — because it takes seconds to accumulate a buffer that
> should fill in 116 ms. It was added during the single-chip memory crisis and
> the split makes it pointless.

#### `hub`

| Setting | Value | Why |
|---|---|---|
| `LEAD_US` | 200 ms | Presentation lead. Enough to absorb WiFi jitter; every ms is latency |
| `LOCAL_RING_BYTES` | 64 kB | ~370 ms of stereo PCM |
| `MAX_CLIENTS` | 8 | Registry size; unicast airtime is what really limits this |
| `CLIENT_TIMEOUT_US` | **2 s** | Was 10 s. Only covers the ungraceful departure now — see §4 for the 124 failed allocations that shortened it |
| `MARKER_EVERY_PKTS` | 100 (~2 s) | Sync measurement pulses |
| `ESP_WIFI_AMPDU_TX_ENABLED` | `n` | Required, or `esp_wifi_internal_set_fix_rate()` returns `ESP_ERR_NOT_SUPPORTED` |
| PHY rate | pinned, 6 Mbps | See the wart in §16 |
| `CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0` | `y`, **hub only** | See below |
| `CONFIG_DANCEFLOOR_LED_HOP_512` | `y` | The component default too, but pinned in the tracked config because a hop mismatch across a floor is the expensive bug |

> **The lwIP tcpip thread is pinned to CPU0 on the hub.** It runs at priority 18
> — above every task this firmware creates — and IDF leaves it unpinned. Core
> locking is off in this build, so every `sendto()` posts to that thread and
> blocks: the work happens *there*, not in the caller. The hub drives ~43 audio
> sends/s plus 86 frame sends/s per satellite against a satellite's 4 probes/s,
> so ~30× the priority-18 work, free to land on the core also carrying play,
> vis-draw and vis. Pinning it was the whole fix for the hub's render starvation:
> wake overshoot mean 2057 → 634 µs, pattern arithmetic max 23226 → 5254 µs, late
> frames 10 → 0. It is set on the hub **only** — the satellite is deliberately
> left unpinned as the control the measurement is against.
>
> Recorded because the first diagnosis was wrong: the starvation was read as
> `sbc_in` at priority 9 pinned to core 1, which only blocks vis-draw while it is
> decoding and yields at every send. One prediction from the fix also failed —
> analysis was expected to improve alongside the pattern task and did not
> (17400 → 17370 µs max). The pattern's max was 240× its mean and could only be
> preemption; analysis's was 8×, which is mostly the FFT doing its own work.

#### `satellite`

| Setting | Value | Why |
|---|---|---|
| `PROBE_PERIOD_MS` | 250 | Min-RTT *holds* its best sample, so a 10-probe window at 1 s meant up to 10 s of staleness — visible as a dead-straight −22 µs-per-announcement ramp. Same ten samples over 2.5 s instead |
| `RING_TARGET_MS` | 200 | Matches the hub's lead |
| `MAX_SPLICE_MS` | 150 | Ceiling on a track-boundary correction; anything larger is a bug, not drift |
| ~~`DANCEFLOOR_USE_INTERNAL_DAC`~~ | — | Was a build option for testing with no DAC wired: 8-bit, audibly poor. Removed 2026-08-12; see §5 |
| `DANCEFLOOR_LED_SOURCE` | `LOCAL` | Analyse the audio this unit holds, rather than draw frames the hub sends. See §12 |

### 14. Pins and wiring

**Between the two master chips** — four signals and a shared ground. The bridge
is the SPI master, the hub the slave, and the link is one-way so there is no
MISO:

| bt_bridge (ESP32) | hub_s3 (XIAO ESP32-S3) | |
|---|---|---|
| GPIO 14 (SCK) | GPIO 44 (D7) | HSPI IOMUX on the bridge |
| GPIO 13 (MOSI) | GPIO 6 (D5) | |
| GPIO 15 (CS) | GPIO 5 (D4) | the framing — one assertion is one packet |
| GPIO 25 (HANDSHAKE in) | GPIO 3 (D2, out) | hub says "buffer armed"; not optional |
| GND ×4 | GND ×4 | keep them all — jumper leads set the UART's ceiling and will set this one's |

GPIO 25 and GPIO 44 were the UART's TX and RX, so two leads were already run.
GPIO 5 was the sync monitor input, and taking it ends the marker/monitor
instrument on this board — it needs GPIO 4 and GPIO 5 both.

`hub/`, the classic ESP32 hub, expected the old UART on GPIO 23 and could no
longer receive anything from the bridge. It was retired on 2026-08-12 rather
than ported.

**Two status LEDs on the bridge**, both optional and neither feeding back into
anything — GPIO 32 solid while A2DP is connected, GPIO 33 blinking while audio
packets arrive. Configurable under *Bridge status LEDs*, `-1` for absent. The
blink is driven from the audio callback rather than from `ESP_A2D_AUDIO_STATE`,
so it reports bytes actually leaving the chip; connected-lit-but-dark is a
stalled stream, and the pair separates a pairing fault from a playback one
without a console.

**On the hub and each satellite:**

| Function | GPIO | Note |
|---|---|---|
| I2S BCK → DAC | 26 | |
| I2S WS/LRCK → DAC | 27 | |
| I2S DATA → DAC | 25 | |
| WS2812 data | 18 | Via 74AHCT125 level shifter, 330 Ω series resistor |
| Sync marker (output) | 4 | Pulses when marked audio plays |
| Sync monitor (input, hub) | 21 | Wire a satellite's marker pin here + common ground |

All configurable under `idf.py menuconfig` → *Dancefloor*, because board
silkscreens disagree — `G4` is GPIO 4 on one board, `D4` is emphatically not on
another.

> **Avoid GPIO 16/17.** On WROVER modules they are wired to the PSRAM die even
> when PSRAM is disabled in software. **Avoid GPIO 5** — it is a strapping pin,
> and another chip driving it at boot can change the boot mode.

### 15. Building, running, and reading the output

```sh
. ~/.espressif/v6.0.1/esp-idf/export.sh

cd bt_bridge && idf.py -p /dev/ttyUSB0 flash monitor   # chip A
cd hub       && idf.py -p /dev/ttyUSB1 flash monitor   # chip B
cd satellite && idf.py -p /dev/ttyUSB2 flash monitor   # each satellite
```

Host-side unit tests need no hardware:

```sh
cd components/dancefloor_sync/test && make check # the clock estimator
cd components/dancefloor_leds/test && make check # FFT, beat detection, patterns,
                                                 # block alignment, cross-unit
                                                 # determinism
```

The LED suite moved in alongside the code it exercises when the visualiser was
split into `analysis.cpp` and `patterns.cpp`; `hub/test` is gone. Two of its
cases are worth knowing about: `test_align` pins that two units cut and label
their analysis blocks identically, and `test_pattern_sync` pins that a pattern
handed those blocks renders identical pixels whatever its unit's join time,
render count or drop history. Those two properties are why nothing about the
lights is transmitted between units.

The same pipeline runs on a laptop, which is where pattern work belongs:

```sh
cd tools/pattern_lab && make
./pattern_lab track.wav                  # live in the terminal
./pattern_lab track.wav --png out.png    # whole track as an image
./pattern_lab track.wav --csv trace.csv  # per-frame numbers for tuning
```

It compiles `analysis.cpp`, `patterns.cpp` and `beat_detect.c` straight out of
the component rather than copying them, so it cannot drift from what the strips
do. Feed it the WAVs the desktop client writes (see below).

> The user's shell profile puts the xtensa toolchain on `PATH`, which breaks
> host `gcc` builds. Both test Makefiles pin `PATH=/usr/bin:/bin` for this
> reason.

Listen from a laptop without building anything. The desktop client lives in its
own repository — [`dancefloor-tools`](../../dancefloor-tools) — because it is
built against nothing here:

```sh
nmcli device wifi connect dancefloor password dancefloor
cd ../dancefloor-tools
python3 desktop_satellite.py --record ~/dancefloor-tracks \
  | aplay -f S16_LE -r 44100 -c 2 --buffer-time=1000000 -
```

It decodes via `ffmpeg`, plays through the pipe, and saves one WAV per track
(skipping Spotify ad breaks, which it recognises by their artist field). Those
recordings exist to tune the beat detector against real music offline, which is
what `tools/pattern_lab` consumes.

One thing that split does not remove: that client carries a hand-maintained copy
of the wire format from `sync_proto.h`, and nothing checks the two still agree.
A mismatch shows up as garbled audio or a rising malformed-packet count, never as
an error. It has caught people out twice.

#### The log lines worth knowing

The periodic lines print every `CONFIG_DANCEFLOOR_LOG_PERIOD_S` (20 s by
default), not every window. Two of them print *immediately* on a bad window
regardless, because waiting to hear about a fault is how faults get missed.

```
I sbc_in: pkts 252 | 44100 Hz x2 | eff 44050 Hz | hdr 0 crc 0 gaps 0 dec 0
          | fed-drop 0 B | max gap 100189 us
```

| Field | Meaning |
|---|---|
| `eff` | **PCM samples/s actually reaching playback** — the honest health metric |
| `hdr` / `crc` | Link integrity. Single digits fine, tens are not. `hdr` is a frame refused on an impossible `kind` or `len`; it was called `sync` on the UART and counted bytes skipped hunting for a sync word, which SPI's CS framing makes meaningless |
| `gaps` | Packets the bridge dropped. Visible only because `seq` is assigned at *enqueue*, not at transmit |
| `fed-drop` | PCM discarded because a buffer was full. **Must be 0** |
| `max gap` | Longest silence between audio packets. **79–112 ms is normal** — the source is bursty. Past 150 ms the window prints at once |

Every counter but `max gap` describes a packet that *arrived and was wrong*. A
source that simply stops sending trips none of them, which is why `max gap` had
to be added: a 118 ms hole was invisible to all of them.

```
I stream: local ring 33280 bytes (188 ms) | phase +6234 us (smoothed +6431 us)
```

Smoothed phase is the number to watch; the raw one is noisy on the hub. Depth
should sit near 200 ms.

```
W stream: HEALTH: up 3721 s | heap 59464 (min 52268, window 58120, largest 55296)
          | stack play 3036 mon 1864 | underruns 0 restarts 1 splices 1
          | retunes 9 (0 refused) | sta-left 0 (dropped 0, no-lease 0)
          | sta-timeout 0 | alloc-fail 0
W sat:    HEALTH: ... | gaps 0 wifi-drops 0 | alloc-fail 0
          | clock TSF (tsf 1/probe 0) | leds remote hop 512 (rx 31558, bad 0)
```

Every 60 s, and the line for a long run. Cumulative counters never reset, so they
answer "has this been happening slowly for an hour", which no per-window rate can.

The four heap terms are four different questions. `min` is the all-time watermark
and **cannot be dated** — one bad moment an hour ago pins it for the rest of the
run, which is why `window` exists: the lowest seen this minute, sampled every 5 s
and cleared by each line, so a dip can be placed in time. `largest` is the largest
free block, and it fails before `heap` does — the WiFi driver wants 1152 B
contiguous, so a fragmented heap refuses that while the free total still looks
comfortable. `alloc-fail` counts allocations that actually failed; it records
rather than logs, and lives in IRAM, because a hook in flash would fault in the
one condition it exists to observe.

**On `hub_s3` those four terms describe the wrong pool**, and the `MEM:` line
beside them exists for that reason. PSRAM is on there with
`CONFIG_SPIRAM_USE_CAPS_ALLOC`, so ordinary `malloc()` never returns PSRAM and
the ring, the DMA buffers, the WiFi buffers and every stack stay in internal
SRAM — while `esp_get_free_heap_size()` reports the 8 MB PSRAM pool, which
nothing on the audio path can use. The gap is not academic: this unit has
printed `heap 8407580 free` in the same second that a 1700-byte internal request
failed. `MEM:` prints the internal pool with the same four questions asked of it,
plus the whole-heap pair for comparison.

**The mask is `MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT`, and the `8BIT` half is
load-bearing.** `MALLOC_CAP_INTERNAL` on its own also matches memory that is
internal but 32-bit-access-only — on the classic ESP32 the IRAM heap is
registered `INTERNAL|EXEC|32BIT` (`heap/port/esp32/memory_layout.c`), and nothing
needing byte access can touch it: not `malloc()`, not a task stack. The first
version of this line used the bare mask and printed

```
MEM: internal 31760 free (min 31424, window 31760, largest 30720) | total 396 (largest 208)
```

on a satellite whose real usable heap was **396 bytes**, and which had already
failed 177 allocations of 4096 bytes — three of its four tasks never started. The
31 kB was real and entirely unusable. Read `total` against `internal` on that
line: if `internal` is the larger of the two, the mask is wrong again.

`ALLOCATION FAILED` reports both pools for the same reason, and spells out the
capability bits: `caps 0x1800 INTERNAL` rather than the bare hex it used to
print, which had to be decoded by hand while a floor was down. Read its `min`
first — the live figures beside it are sampled by the monitor task up to 5 s
later, so on a transient they describe a condition that has already passed. The
first capture printed `internal 21912 free, largest 11776` next to a failed
1700-byte request, which disproves the failure it accompanies; `min 1520` is what
explained it. The hook cannot sample the live figures itself: the heap-statistics
functions live in flash, and the hook is `IRAM_ATTR` because an allocation can
fail from an ISR with the flash cache disabled.

**The internal budget is smaller than the datasheet headline.** The S3 has 512 kB
of SRAM but only ~268 kB is ever a registered heap — ~102 kB is DRAM shadowed by
IRAM code (`.dram0.dummy`), ~96 kB is static `.data`/`.bss`, ~33 kB is
ROM-reserved. Of that, ~246 kB is live in a running hub: ~53 kB of task stacks
and ~122 kB of this app's own buffers, the largest being `local_ring` at 64 kB
and the visualiser's analysis stream at 33 kB. What remains has to absorb WiFi's
transient buffer demand, which is why `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP` is
on — see `hub_s3/sdkconfig.defaults` for the measurement that turned it on.

`sta-left N (dropped M, no-lease K) | sta-timeout T` is how a satellite left.
`dropped` is a clean disassociation, resolved to an IP through the DHCP lease
table, ~11 ms after the event. `no-lease` is the event arriving when the lease had
already gone. `sta-timeout` is the ungraceful case — power cut, out of range —
where no event fires at all and the 2 s timeout does the work, since the AP
notices inactivity far later than that. Without it, a run where a satellite
vanished mid-track and a run where nothing happened print the same line.

`clock TSF (tsf N/probe M)` says which clock source is live and how many anchors
used each; a rising `probe` means TSF is dropping out.

```
W stream: TRACK DIVERGENCE (wifi): 192.168.4.2 spliced +8 ms (phase +8231 us),
          hub spliced +14 ms -> -6 ms apart
```

One line per track, taken at a track boundary — the one instant that recurs
identically in every track, so it is comparable across tracks, sessions and
builds. Cross-unit error resets there and grows until the next one, so a reading
taken anywhere else depends on where in that cycle you looked. Judging two builds
by glancing at log windows produced three confident wrong diagnoses in this
project.

### 16. Known warts

**The hub's absolute phase reading wanders**, and it is now quantified: two reads
of the same variable one millisecond apart differed by **15.7 ms**, with the
running average sitting still at +9 ms through it. Cause unfound. The satellite's
readings are quiet by comparison (±5 ms), so this is the hub's own measurement
rather than something shared. It is filtered rather than understood — both units
servo on a 4-sample average now — and the practical rule is unchanged: treat the
cross-unit number as the meaningful one and any single-sample figure derived from
the hub's phase, including the per-retune costs it prints, as untrustworthy.

**Cross-unit error grows between track boundaries.** A splice nulls phase on both
units, so the figure resets to ~0.1 ms at every track change and grows to a few
ms by the end of a long track. That growth is bounded by the deadband — each unit
tolerates 7 ms of its own error — so the worst case between two of them is twice
that. Quoting one number for cross-unit sync is misleading; see clock-sync.md §8.

**The retune's discarded DMA buffer is unmeasured.** Disabling the I2S channel
throws away audio the playback task counted as played and fed to the LEDs. The
LED half is handled (`visualiser_realign()`); the audio half cannot be seen from
inside the firmware at all, because every reading derived from `samples_played`
agrees those frames were played. Only the marker GPIO can measure it, and nobody
has. Since `auto_clear` (§5), whichever of those buffers had already played comes
back as silence rather than a repeat — the same frames are still lost, so the
figure is unchanged, but the artifact is quieter.

**The LED drift check bounds the error, it does not remove it.** `ALIGN_DRIFT_US`
is 2 ms, so holes smaller than that accumulate until they add up to something
that trips it, and two units can sit up to that far apart in the meantime — worth
roughly 15% of frames rendering visibly differently, per the figures in §12. The
bound is the guarantee; zero is not. Lower the constant if strips are seen
disagreeing between track boundaries, and read `drift` in the `vis` log line to
see whether it is firing at all.

**`beat_det_update()` sums its flux history in array order.** The ring's rotation
depends on how many frames that unit has pushed, so two units holding identical
history at different rotations sum the same values in a different order and can
differ in the last bits of the threshold. An onset flips only within ~1e-7 of it,
so `test_pattern_sync.cpp` passes — on margin, not on guarantee. Summing from the
oldest entry would settle it.

**The source delivers in ~100 ms bursts, and always has.** Every 5 s window
contains a silence of 79–112 ms between audio packets against a ~43 ms nominal
spacing. The consequence is that the hub's presentation timeline does not drift,
it *oscillates* by ±130 ms with the burst pattern, which is past the 120 ms
resync threshold — so the mechanism trips on entirely normal delivery about
seven times a minute. That is survivable because the timeline slews rather than
jumps, but the threshold cannot simply be raised: `LEAD + RESYNC` bounds what a
satellite must buffer, 200 + 120 = 320 ms against a 372 ms ring, and the swing
is 132. Fixing it properly means a smaller lead or a larger ring.

**A ~118 ms delivery pause has been seen with no cause found.** Distinct from the
routine burstiness above, it landed once mid-track with no track change nearby,
no CRC error, no sequence gap and no decode error. `max gap` in the `sbc_in`
line is the instrument for it now.

**The PHY rate is still pinned interface-wide.**
`esp_wifi_internal_set_fix_rate()` applies to all AP-interface transmission, not
just multicast, so with multicast gone it fixes *unicast* at 6 Mbps and disables
rate adaptation — the very mechanism that fixed the earlier 23% loss. At ~7%
airtime it is not hurting anything measurable, and
`CONFIG_DANCEFLOOR_WIFI_PHY_RATE_MBPS=0` is very likely the right value. It is
left at 6 because the system was measured working there and the change deserves
a listening test rather than a reasoned argument.

**Two of the wideband detector's constants have still never been tuned against a
recording.** The flux floors have been, both of them, over 206 recordings at two
hops — see [`tuning-corpus.md`](tuning-corpus.md), which also swept `BEAT_HIST`
and argues that the two `threshold_k` values need no sweep because a hop change
cannot reshape `flux > mean + k·sd`. What that leaves is **`BEAT_THRESHOLD_K`
(1.8) and `BAND_GAIN` (12.0)**, which the `pulse` pattern runs on and which were
set against synthetic kicks in the host test. Invariance under a hop change is
not the same as being right in absolute terms: they fire on real music and the
units agree, but nobody has measured how many beats are missed or invented. That
is the measurement the recordings still exist to make.

It matters less than it did. The band scaling used to end in a hard clamp at
1.0, and since spectral flux counts only energy *increases*, a pinned band had a
rise of exactly zero -- measured against this FFT and gain, the bass band clamped
for any 60 Hz content above about -11 dBFS, so on most mastered music the kick
contributed nothing at all and detection ran on the higher bands alone.
`beat_normalise()` is now monotonic over the whole input range, so a wrong gain
costs sensitivity rather than deleting the signal.

**Whether 512 is the right hop has not been established, only that the detector
is tuned for it.** The retune answered "do the constants still hold at this hop"
— they do — and in doing so measured something it could not fix: a hop change
moves roughly a third of the strokes, and no value of any constant changes that
(§12). Which strokes the strip *should* follow is a question about music, wants a
listening session rather than a sweep, and has not had one. So 1024 is not a
safer fallback; it is a different choice, and it is the one now carrying
constants confirmed against a corpus at 512.

**Nothing has been heard through a real DAC.** M1, M2 and M3 were marked
complete on log output and on the desktop client's audio. The PCM5102A boards
are still to be wired.

### 17. What is left

| Milestone | State |
|---|---|
| M1–M2 Bluetooth speaker + LEDs | Done, on logs and desktop audio |
| M3 Beat detection driving LEDs | **Working on hardware** — two units pulsing together on real music. A `boom` detector follows the zabumba's low band alone. Both flux floors and `BEAT_HIST` are now swept against a checked-in corpus of 206 recordings at two hops (`tuning-corpus.md`); `BEAT_THRESHOLD_K` and `BAND_GAIN` are still untuned |
| M4–M6 Clock sync, streaming, drift | Done and measured. Drift needed a second pass: the satellite was converting with an offset frozen at anchoring, which fed the drift back in as the servo's own reference. The clock source is now 802.11 TSF with the probe estimator as fallback |
| M7 Coexistence | Done — resolved by splitting the chips |
| M8 Power, enclosure, field test | **Untouched** |
| Long-run behaviour | **Unknown.** The longest evidenced session is ten minutes against a four-hour target. `HEALTH` exists to answer it and has never been given the chance |

The soak is the cheapest of these and blocks the least: it needs no parts, only
time. Both units print a `HEALTH` line every 60 s with uptime, four heap figures,
per-task stack headroom, failed allocations, and cumulative counts of underruns,
re-anchors, splices, retunes, lost-packet gaps and WiFi drops, plus a `MEM:` line
carrying the internal-SRAM pool — the one that constrains the hub, and the one
the `HEALTH` figures stopped describing when PSRAM went on (§15). Over ten minutes
heap was flat and every counter stayed near zero, which is encouraging and is not
evidence.

M8 in outline: 12 V LiFePO4, an XL4016-class 5 V/10 A buck (an LM2596 cannot do
this current), IP65 sleeved strips, cable glands, and a four-hour overnight
field test. Budget ~30–40 W per unit — LEDs at 30–40% brightness, which at night
looks just as vivid to dark-adapted eyes and reclaims most of the LED power
budget.

### 18. Code map

| Path | Responsibility |
|---|---|
| `components/dancefloor_sync/` | Wire formats and the clock estimator. **No ESP-IDF dependencies** — it is the part most likely to be subtly wrong, and hardware bring-up is a bad place to find out |
| `components/sbc_decoder/` | Vendored OI SBC decoder from Bluedroid |
| `bt_bridge/main/sbc_spi.c` | Frames SBC onto the SPI link as master, `seq` assigned at enqueue |
| `bt_bridge/main/avrcp_meta.c` | Track metadata and change notifications |
| `bt_bridge/main/status_led.c` | The two front-panel LEDs — connected solid, streaming blinking |
| `hub_s3/main/streamer.c` | SoftAP, sockets, client registry, timeline, DAC, phase servo, frame publisher |
| `hub_s3/main/sbc_in.c` | SPI slave receive, decode, feed |
| `components/dancefloor_leds/` | Shared by hub and satellites: FFT → bands → onset → patterns, plus the LED Kconfig both use |
| `components/led_strip_wrapper/` | RAII C++ strip driver, RMT or SPI backend |
| `satellite/main/main.c` | The whole satellite — receive, decode, servo, play, light |
| `tools/pattern_lab/` | The LED pipeline over a WAV on a laptop, compiled from the component |
| `tools/tuning/` | `sweep.py` and `converge.cpp` — the harness behind `tuning-corpus.md` |

---

## Postscript: the pattern in the failures

Worth stating plainly, because it recurred at every level of this system.

**Every real fault was invisible until something counted it.** The 1.15% I2S
frame loss, the 20% multicast loss, the silently-dropping input buffer, the
bridge-side drops hidden by numbering packets at transmit instead of enqueue,
the desktop client parsing a stale header and reporting healthy statistics while
decoding nothing — none of these announced themselves. Several actively looked
like something else, and the logs read fine throughout.

And in almost every case the reasoning was wrong first, usually more than once,
until an instrument settled it. The I2S investigation eliminated six plausible
suspects before the real one. The multicast fix was made *worse* by a change
that was theoretically sound.

The practical form of that lesson, for anyone extending this: **before
investigating a symptom, add a counter for the thing you believe is being
lost.** Everything in this system that works, works because something counts it.

### And a second pattern, from getting the LEDs to agree

That one took three rounds, and each round shipped a test that passed:

| Round | Test written | Why it proved nothing |
|---|---|---|
| Block alignment | Producer/consumer with drops | The reader always drained the queue, so no drop was ever exercised |
| Re-alignment after a drop | Same, with real drops | Correct — it caught the bug, 99.8% of blocks misaligned |
| Content vs clock labelling | Single unit, blocks correctly spaced | The property is about **two** units. One unit's blocks were correctly spaced the whole time; the two just landed on different content |

The failure mode is not laziness — each test measured a real property, carefully.
It measured the property that was easy to state rather than the one the hardware
was showing. The discipline that actually works: **run the new test against the
broken version first.** If it passes, the test is wrong, and writing it has told
you nothing about the code.

### And a third, from chasing a drift that had already been fixed once

Two habits produced most of the wasted effort in the drift work.

**A fix applied where it was found rather than where it applies.** The hub had
parked its playback across a retune since "a measured 54 ms correction cost
177 ms of buffer". The satellite runs the same loop, calls the same blocking
write, and never got the guard — so for its entire life every retune it
performed drained its ring at memory speed, costing up to 50 ms each. The
mirror image was also true: the satellite smoothed its phase and the hub did
not. Both were one-line differences between two files that are supposed to
behave identically. **When a fix lands in one unit, the question is not whether
it works but whether the other unit has the same bug.**

**Diagnoses read off incomparable samples.** Three wrong conclusions in a row,
each confidently argued:

| Concluded | From | Actually |
|---|---|---|
| Frequent retunes were desynchronising the strips | Four phase steps differenced across 5 s log ticks carrying ±5 ms of wander each | The satellite was spinning through its ring during the outage |
| A restart flag was causing splices | Plausible mechanism, never checked against the log | It had never fired once — there were no underruns |
| Smoothing the hub's phase made sync worse | Comparing a log window late in a track against one just after a splice | The splice resets cross-unit error, so the two windows were different points of the same cycle |

Each was acted on. Two shipped. The pattern is the same every time: a difference
was computed between two numbers that were not measuring the same thing.

What broke the cycle was **forcing the event on demand instead of waiting for
it**. `CONFIG_DANCEFLOOR_RETUNE_BENCH_S` retunes to the rate already set, so the
rate change and the drift are removed from the experiment and only the cost
remains; the phase is captured immediately either side of the event rather than
differenced across log ticks. Twenty samples in ten minutes, against four
scavenged from a whole session — and the answer contradicted both standing
theories within one run.

The generalisation: **if a measurement only happens when the system decides to
do something, add a way to make it happen on demand.** Waiting for the event
means comparing samples taken under conditions you did not control, and that is
where confident, wrong answers come from.
