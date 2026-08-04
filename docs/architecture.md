# Dancefloor — architecture and concepts

A phone plays Spotify over Bluetooth. Several battery-powered speakers scattered
around a field play that music **at the same instant**, each pulsing a NeoPixel
strip on the beat.

This document is the way in. It explains the concepts the system is built on
before it explains the system, so it should be readable without prior knowledge
of Bluetooth audio, WiFi, or embedded audio pipelines. Three companion documents
go deeper on the parts that needed it:

| Document | Covers |
|---|---|
| [`clock-sync.md`](clock-sync.md) | The time-sync maths, the servo, measured results |
| [`sbc-link.md`](sbc-link.md) | The wire between the two master chips |
| [`two-chip-master.md`](two-chip-master.md) | Why the master is two chips, memory numbers |

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
whatever has sent it a **time-sync probe** in the last 10 seconds. A unit that
is keeping its clock synchronised is by definition alive and listening, so one
mechanism does both jobs and there is nothing to configure. Stop probing and you
stop receiving.

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

The ESP32 has built-in DACs, but they are **8-bit** — 48 dB of dynamic range
against 96 dB for 16-bit. Audibly bad: quiet passages sit in obvious hiss. They
are useful for bring-up when you have no hardware, and the satellite has a build
option for exactly that, but the real output is an external **PCM5102A** I2S
DAC.

### 6. UART, and why the boring option won

**UART** is asynchronous serial — the oldest and simplest digital link there is.
One wire, no clock. The receiver recovers timing from the **start bit** at the
beginning of every byte, resynchronising on each one, which is why the two ends
only need to agree on the baud rate to within a percent or so.

That property — **no shared clock** — is the entire reason it is used here, and
it is worth understanding why before reading §8.

---

## Part II — The system

### 7. The signal path, end to end

```
  ┌─────────────────── MASTER ENCLOSURE ────────────────────┐
  │                                                          │
  │   ESP32 #1: bt_bridge          ESP32 #2: hub             │
  │   ┌──────────────────┐         ┌──────────────────────┐  │
 phone │ A2DP sink        │  UART   │ SoftAP + clock master│  │
 ──────▶ AVRCP metadata   ├────────▶│ SBC decoder          │  │
  BT  │ forwards raw SBC │ 500 kbd │ presentation timeline│  │
  │   │ no decode at all │  1 wire │ I2S ──▶ DAC ──▶ amp  │  │
  │   └──────────────────┘         │ SPI ──▶ NeoPixels    │  │
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

Four firmware images live in this repository, plus a Python client:

| Project | Role |
|---|---|
| `bt_bridge/` | Chip A. Bluetooth only. Receives A2DP, forwards raw SBC over UART |
| `hub/` | Chip B. WiFi SoftAP, clock master, decoder, DAC, LEDs, streamer |
| `satellite/` | Every additional speaker. Receives, decodes, plays, lights |
| `tools/pattern_lab/` | The LED pipeline on a laptop, compiled from the firmware sources |

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
with the burst pattern, and `hub/main/streamer.c` had to be taught to slew rather
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
rather than PCM) and needs one signal wire instead of three.

Running at **500 kbaud**, which is 84% utilisation and tighter than anyone would
choose. That ceiling is a property of breadboard jumper leads, not the protocol:

| Baud | Result per 5 s |
|---|---|
| 1,000,000 | ~50% of packets corrupt |
| 750,000 | 20–30 bad sync, 15–20 CRC errors |
| **500,000** | **0–2 bad sync, 0–1 CRC errors** |

Full detail in [`sbc-link.md`](sbc-link.md).

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

Each unit analyses **its own copy** of the audio. Sending analysis results over
the network would add a second thing to synchronise; the audio is already
synchronised, so anything derived from it locally is synchronised too, for free.

```
PCM ──▶ 1024-point FFT (esp-dsp) ──▶ 4 bands ──▶ spectral flux ──▶ onset ──▶ pattern ──▶ strip
```

**Spectral flux** measures how much energy *increased* since the last frame, per
band. A drum hit is a sudden broadband rise; a sustained note is not. Comparing
against a rolling ~0.5 s history makes it adaptive to loud and quiet passages,
and a 120 ms refractory period stops one kick registering three times.

Four bands, roughly kick / low-mid / presence / air.

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

**Blocks must be cut at the same content positions.** The detector chops the
stream into fixed 1024-sample blocks, and if two units cut them at different
offsets a transient near a boundary is split on one board and centred on the
other — the marginal onsets then fire on one strip and not its neighbour.
Boundaries are derived from `play_at`, the instant the audio is *scheduled* to be
heard, which every unit receives identically. Never from a clock read locally:
units reach that line milliseconds apart, and 3 ms of skew puts them 132 samples
out of 1024.

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

#### Where the code lives

`components/dancefloor_leds/` is shared by the hub and every satellite: the same
FFT, the same detector, the same patterns, one copy. It is split so that
everything deciding what the lights *do* has no platform dependencies:

| | |
|---|---|
| `analysis.cpp` | FFT, band split, onset detection — audio in, `Frame` out |
| `patterns.cpp` | `Frame` in, pixels out |
| `visualiser.cpp` | the parts that cannot be shared: stream buffer, block alignment, task, strip |
| `fft_host.c` | a radix-2 FFT for the host, where esp-dsp does not exist |

That split exists so `tools/pattern_lab` can run the identical pipeline over a
WAV on a laptop — same analysis, same patterns, compiled from these files rather
than copied. Designing a pattern is a taste problem, and taste needs turnaround
that reflashing two boards cannot give.

Its `Kconfig` owns the LED options for both firmwares — GPIO, count, strip model,
and a **brightness cap defaulting to 10%**, which is the figure the power budget
assumes and which dark-adapted eyes cannot distinguish from full outdoors.

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
| `EXAMPLE_A2DP_SINK_USE_EXTERNAL_CODEC` | `y` | Selects the **undecoded** A2DP callback. This chip forwards SBC and never decodes |
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
| `CLIENT_TIMEOUT_US` | 10 s | A satellite that stops probing is forgotten |
| `MARKER_EVERY_PKTS` | 100 (~2 s) | Sync measurement pulses |
| `ESP_WIFI_AMPDU_TX_ENABLED` | `n` | Required, or `esp_wifi_internal_set_fix_rate()` returns `ESP_ERR_NOT_SUPPORTED` |
| PHY rate | pinned, 6 Mbps | See the wart in §16 |

#### `satellite`

| Setting | Value | Why |
|---|---|---|
| `PROBE_PERIOD_MS` | 250 | Min-RTT *holds* its best sample, so a 10-probe window at 1 s meant up to 10 s of staleness — visible as a dead-straight −22 µs-per-announcement ramp. Same ten samples over 2.5 s instead |
| `RING_TARGET_MS` | 200 | Matches the hub's lead |
| `MAX_SPLICE_MS` | 150 | Ceiling on a track-boundary correction; anything larger is a bug, not drift |
| `DANCEFLOOR_USE_INTERNAL_DAC` | `n` | Build option for testing with no DAC wired. 8-bit, audibly poor |

### 14. Pins and wiring

**Between the two master chips** — one signal wire and a shared ground:

| bt_bridge | hub | |
|---|---|---|
| GPIO 25 (UART TX) | GPIO 23 (UART RX) | the only signal |
| GND ×4 | GND ×4 | keep them all — this link will not run above 500 kbaud on thin leads |

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
I sbc_in: pkts 252 | 44100 Hz x2 | eff 44050 Hz | sync 0 crc 0 gaps 0 dec 0
          | fed-drop 0 B | max gap 100189 us
```

| Field | Meaning |
|---|---|
| `eff` | **PCM samples/s actually reaching playback** — the honest health metric |
| `sync` / `crc` | UART link integrity. Single digits fine, tens are not |
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
has.

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

**The wideband detector's thresholds have never been tuned against a recording.**
The `boom` detector has: `flux_floor` was set from ten forró tracks (see below).
`BEAT_THRESHOLD_K` and `BAND_GAIN`, which the `pulse` pattern runs on, have not.
`BEAT_THRESHOLD_K` (1.8) and `BAND_GAIN` (12.0) were set against synthetic kicks
in the host test. They fire on real music and the units agree, but nobody has
measured how many beats are missed or invented. This is what the WAV recording
exists to fix.

It matters less than it did. The band scaling used to end in a hard clamp at
1.0, and since spectral flux counts only energy *increases*, a pinned band had a
rise of exactly zero -- measured against this FFT and gain, the bass band clamped
for any 60 Hz content above about -11 dBFS, so on most mastered music the kick
contributed nothing at all and detection ran on the higher bands alone.
`beat_normalise()` is now monotonic over the whole input range, so a wrong gain
costs sensitivity rather than deleting the signal.

**Nothing has been heard through a real DAC.** M1, M2 and M3 were marked
complete on log output and on the desktop client's audio. The PCM5102A boards
are still to be wired.

### 17. What is left

| Milestone | State |
|---|---|
| M1–M2 Bluetooth speaker + LEDs | Done, on logs and desktop audio |
| M3 Beat detection driving LEDs | **Working on hardware** — two units pulsing together on real music. A `boom` detector follows the zabumba's low band alone, tuned against ten forró recordings; the wideband `pulse` thresholds are still untuned |
| M4–M6 Clock sync, streaming, drift | Done and measured. Drift needed a second pass: the satellite was converting with an offset frozen at anchoring, which fed the drift back in as the servo's own reference. The clock source is now 802.11 TSF with the probe estimator as fallback |
| M7 Coexistence | Done — resolved by splitting the chips |
| M8 Power, enclosure, field test | **Untouched** |
| Long-run behaviour | **Unknown.** The longest evidenced session is ten minutes against a four-hour target. `HEALTH` exists to answer it and has never been given the chance |

The soak is the cheapest of these and blocks the least: it needs no parts, only
time. Both units print a `HEALTH` line every 60 s with uptime, current and
minimum-ever heap, per-task stack headroom, and cumulative counts of underruns,
re-anchors, splices, retunes, lost-packet gaps and WiFi drops. Over ten minutes
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
| `bt_bridge/main/sbc_uart.c` | Frames SBC onto the UART, `seq` assigned at enqueue |
| `bt_bridge/main/avrcp_meta.c` | Track metadata and change notifications |
| `hub/main/streamer.c` | SoftAP, sockets, client registry, timeline, DAC, phase servo |
| `hub/main/sbc_in.c` | UART receive, decode, feed |
| `components/dancefloor_leds/` | Shared by hub and satellites: FFT → bands → onset → patterns, plus the LED Kconfig both use |
| `components/led_strip_wrapper/` | RAII C++ strip driver, RMT or SPI backend |
| `satellite/main/main.c` | The whole satellite — receive, decode, servo, play, light |

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
