# WiFi, from the air to the audio

Everything this project does over a radio crosses one hop: an ESP32-S3 hub runs
a SoftAP, classic-ESP32 satellites join it as stations, and unicast UDP carries
SBC audio and a shared timeline to every speaker on the floor. When that hop is
healthy nobody thinks about it. When it is not, the failure arrives as silence
from one speaker in a field, and the cause is somewhere between a neighbour's
download and a Kconfig default nobody chose.

This document is the missing middle. It explains WiFi from the band upwards —
what a channel is, what a frame is, what happens when two radios talk at once —
then the ESP32's implementation of it, then every configuration option the
ESP-IDF exposes, and finally this system read as a worked example. Concepts
first, always; this project appears as evidence, never as a prerequisite.

**Everything here is quoted from what is actually on this machine.** ESP-IDF
**v6.0.1** at `/home/pico/.espressif/v6.0.1/esp-idf` (the one `get_idf`
activates), and the current state of this repository. Line references are to
those trees. Where a source comment in this repo has been overtaken by events,
this document says so rather than repeating it.

```
                    ┌──────────────────────────────────────┐
   Part I           │  the air: band, frames, joining      │   no ESP here
                    ├──────────────────────────────────────┤
   Part II          │  the ESP32: silicon, driver, config  │   IDF v6.0.1
                    ├──────────────────────────────────────┤
   Part III         │  this system, read as an example     │   dancefloor
                    └──────────────────────────────────────┘
```

---

## Contents

**Part I — The air**

- [§1 The band](#1-the-band)
- [§2 Getting bits into the air](#2-getting-bits-into-the-air)
- [§3 Sharing the air](#3-sharing-the-air)
- [§4 Frames](#4-frames)
- [§5 Joining a network](#5-joining-a-network)
- [§6 Staying joined](#6-staying-joined)

**Part II — The ESP32's WiFi**

- [§7 The silicon](#7-the-silicon)
- [§8 The stack, and who calls whom](#8-the-stack-and-who-calls-whom)
- [§9 `wifi_init_config_t`, the one struct that matters](#9-wifi_init_config_t-the-one-struct-that-matters)
- [§10 Configuring an interface](#10-configuring-an-interface)
- [§11 Events](#11-events)
- [§12 Buffers and memory](#12-buffers-and-memory)
- [§13 Every configuration option](#13-every-configuration-option)
- [§14 Sniffer mode](#14-sniffer-mode)
- [§15 ESP-NOW](#15-esp-now)
- [§16 Coexistence with Bluetooth](#16-coexistence-with-bluetooth)
- [§17 What changed in IDF 6.0](#17-what-changed-in-idf-60)

**Part III — Reading this system**

- [§18 Power-on to first audio packet](#18-power-on-to-first-audio-packet)
- [§19 The channel survey, read as a worked example](#19-the-channel-survey-read-as-a-worked-example)
- [§20 Eight things that broke](#20-eight-things-that-broke)
- [§21 How to measure your own link](#21-how-to-measure-your-own-link)
- [§22 Appendices](#22-appendices)

---
---

# Part I — The air

Nothing in this part is ESP-specific. It is 802.11, and it would be equally true
of a laptop or a router. Skip it only if you already know what a DTIM beacon is
and why channel 6 is not "between" 1 and 11.

---

## 1 The band

WiFi in this project lives in the **2.4 GHz ISM band** — 2400 to 2483.5 MHz, an
unlicensed slice of spectrum shared with Bluetooth, microwave ovens, wireless
keyboards, video senders and every other 2.4 GHz WiFi network within earshot.
"Unlicensed" is the whole story: nobody is coordinating, nobody has priority,
and your only right is to transmit politely and take what you get.

### Channels are centre frequencies, not slots

The band is divided into channels numbered 1 to 13 (14 in Japan, 1–11 in the
US). They are **5 MHz apart**:

| Channel | Centre |
|---|---|
| 1 | 2412 MHz |
| 2 | 2417 MHz |
| 6 | 2437 MHz |
| 11 | 2462 MHz |
| 13 | 2472 MHz |

But a 20 MHz WiFi transmission is roughly **22 MHz wide**. Five apart, twenty-two
wide: channels overlap heavily. Channel 1 occupies about 2401–2423 MHz, and
channel 3's spectrum sits almost entirely inside it.

```
   ch1        ch2   ch3   ch4   ch5        ch6                    ch11
    |          |     |     |     |          |                      |
 ┌───────────────────────┐                                     
 │      22 MHz wide      │  ch1: 2401 - 2423 MHz
 └───────────────────────┘
             ┌───────────────────────┐
             │                       │  ch5: 2421 - 2443 MHz  ← overlaps 1 AND 6
             └───────────────────────┘
                       ┌───────────────────────┐
                       │                       │  ch6: 2426 - 2448 MHz
                       └───────────────────────┘

 2400        2410        2420       2430       2440       2450       2460  MHz
```

This is why **1, 6 and 11** are the only three non-overlapping channels in the
2.4 GHz band, and why every deployment guide tells you to use them. Anything
within 4 channels of a candidate lands on top of it — which is exactly the
constant this project encodes:

```c
/* 2.4 GHz channels sit 5 MHz apart and are 22 MHz wide, so anything within
 * four of a candidate lands on top of it. */
#define CHANNEL_OVERLAP 4
```
— `hub_s3/main/net.c:83`

### Why overlap is worse than sharing

Two networks on the *same* channel are polite to each other. They can hear each
other's transmissions, so the carrier-sense mechanism in §3 works: each waits
for the other to finish. Throughput halves, but nothing is destroyed.

Two networks on *overlapping* channels — 1 and 3, say — cannot decode each other,
but their energy still lands in each other's receivers. Neither defers, both
transmit, and both frames are corrupted. **Partial overlap is worse than full
overlap.** A network on channel 3 damages channels 1 and 6 simultaneously and
defers to neither.

This is the practical reason the hub's boot-time survey scans all thirteen
channels rather than just its three candidates. The comment records the survey
that forced the change:

> In one workshop channels 1 and 6 each looked nearly bare on their own centre;
> four networks sitting on channel 5 were the largest contributor to both, and
> summed occupancy came out 533 and 608 against channel 11's 87. A scan
> restricted to 1/6/11 would have read 1 and 6 as clear and picked one of them.

— `hub_s3/main/net.c:60-67`

### RSSI, dBm, and the one arithmetic mistake everyone makes

Signal strength is reported in **dBm** — decibels relative to one milliwatt. It
is a logarithmic scale, so:

| dBm | Power | Meaning |
|---|---|---|
| 0 dBm | 1 mW | |
| −30 dBm | 0.001 mW | very close to the transmitter |
| −60 dBm | 1 nW | comfortable |
| −80 dBm | 10 pW | marginal; rate adaptation drops to slow modes |
| −90 dBm | 1 pW | at or below the noise floor |

Every 3 dB is a factor of two in power; every 10 dB is a factor of ten.

**You cannot add dBm figures.** Two networks at −60 dBm are not −120 dBm, nor
−60 dBm; they are −57 dBm, because the *powers* add and 2× is +3 dB. To combine
signal strengths you must convert to linear power, sum, and convert back. That
is precisely what the survey does when it totals the beacon power on a channel:

```c
power[k] += powf(10.0f, rec.rssi / 10.0f);
```
— `hub_s3/main/net.c:294`, converted back for printing by `occupancy_dbm()` at
`net.c:92-95`

### 5 GHz, and why it is not here

The 5 GHz band has far more non-overlapping channels and far less interference,
and every argument in this section improves there. It is not an option for this
project: the classic ESP32 and the ESP32-S3 are **2.4 GHz only**. The ESP32-C5
and C6 add 5 GHz and Wi-Fi 6, which is worth knowing when the next hardware
decision comes round, but nothing in this build can use it.

---

## 2 Getting bits into the air

A frame does not travel as bits. It travels as a modulated radio wave, and the
scheme used to modulate it decides how many bits per second and how robust they
are against noise. WiFi has accumulated several, and a 2.4 GHz ESP32 supports
three generations of them.

### The three generations on 2.4 GHz

| Standard | Name | Modulation | Rates |
|---|---|---|---|
| **802.11b** (1999) | — | DSSS / CCK | 1, 2, 5.5, 11 Mbps |
| **802.11g** (2003) | — | OFDM | 6, 9, 12, 18, 24, 36, 48, 54 Mbps |
| **802.11n** (2009) | HT (High Throughput) | OFDM + MIMO | MCS 0–7 on one spatial stream: 6.5 – 72.2 Mbps |

The ESP32 speaks all three. It has **one antenna and one spatial stream**, so
802.11n gives it MCS 0 through 7 only — MCS 8–15 need two streams.

The IDF exposes these as a bitmap you set per interface:

```c
#define WIFI_PROTOCOL_11B         0x1     /**< 802.11b protocol */
#define WIFI_PROTOCOL_11G         0x2     /**< 802.11g protocol */
#define WIFI_PROTOCOL_11N         0x4     /**< 802.11n protocol */
#define WIFI_PROTOCOL_LR          0x8     /**< Low Rate protocol */
#define WIFI_PROTOCOL_11A         0x10    /**< 802.11a protocol */
#define WIFI_PROTOCOL_11AC        0x20    /**< 802.11ac protocol */
#define WIFI_PROTOCOL_11AX        0x40    /**< 802.11ax protocol */
```
— `components/esp_wifi/include/esp_wifi_types_generic.h:463-469`

`WIFI_PROTOCOL_LR` is Espressif's own long-range mode — proprietary, ESP-to-ESP
only, roughly 1/4 the rate for several dB of extra link budget. It is not 802.11
and no phone or router will speak it.

### Rate is airtime, not throughput

This is the single most useful reframing in the whole subject.

A 1500-byte frame is 12000 bits. At 6 Mbps that is 2000 µs of air. At 54 Mbps it
is 222 µs. **The frame is the same frame; what changes is how long the channel is
occupied delivering it.** Since the channel is shared, airtime is the resource
that is actually scarce — not bits per second.

That is why a single slow device degrades a whole network: a station stuck at
1 Mbps because it is far away takes 12 ms to send what a nearby station sends in
0.2 ms, and for those 12 ms nobody else can transmit at all. It is also why this
project measures channel busyness in **airtime**, not in frame counts or beacon
strength:

```c
/* bits * 10 / tenths-of-Mbit = microseconds. */
s_occ_busy_us += (uint32_t)c->sig_len * 8u * 10u / tenths + OCCUPANCY_OVERHEAD_US;
```
— `hub_s3/main/net.c:177`

The rate tables that feed it are worth reading, because they are the 802.11 rate
set written out:

```c
static const uint16_t legacy_rate_tenths[16] = {
     10,  20,  55, 110,      /* 1, 2, 5.5, 11 long preamble          */
     10,  20,  55, 110,      /* 0x04 unused; 2, 5.5, 11 short        */
    480, 240, 120,  60,      /* 48, 24, 12, 6                        */
    540, 360, 180,  90,      /* 54, 36, 18, 9                        */
};

/* MCS0..7 at 20 MHz, long GI, tenths of a Mbit/s. */
static const uint16_t ht_mcs_tenths[8] = { 65, 130, 195, 260, 390, 520, 585, 650 };
```
— `hub_s3/main/net.c:138-146`

The first four rows are 802.11b and g. The second table is 802.11n MCS0–7 at
20 MHz with a long guard interval: 6.5, 13, 19.5, 26, 39, 52, 58.5, 65 Mbps.

### The three multipliers on an HT rate

An 802.11n rate is the MCS number modified by three things, and the occupancy
callback applies all three:

```c
if (c->sig_mode == 1) {                       /* HT */
    tenths = ht_mcs_tenths[c->mcs & 7];
    if (c->cwb) {
        tenths *= 2;                          /* 40 MHz */
    }
    if (c->sgi) {
        tenths = tenths * 10 / 9;             /* short guard interval */
    }
} else {
    tenths = legacy_rate_tenths[c->rate & 15];
}
```
— `hub_s3/main/net.c:160-172`

- **Channel width** (`cwb`). HT40 bonds two adjacent 20 MHz channels into one
  40 MHz channel. The code above doubles the rate for it, which is a slight
  under-estimate — HT40 carries 108 data subcarriers against HT20's 52, so the
  true ratio is 2.08 and MCS7 HT40 is 135 Mbps rather than 130. For an airtime
  estimate that errs on the side of "busier", which is the safe direction. See
  below.
- **Guard interval** (`sgi`). The gap between OFDM symbols that absorbs
  multipath echo. Long GI is 800 ns, short is 400 ns; shortening it gains
  about 11% (hence `× 10/9`) at the cost of tolerance for reflective rooms.
- **Spatial streams.** Not applicable — the ESP32 has one.

### HT40 is usually a mistake on 2.4 GHz

HT40 doubles the rate by occupying 40 MHz instead of 20. On 5 GHz, where there
is spectrum to spare, that is free throughput. On 2.4 GHz, where there are only
three non-overlapping channels, **a single HT40 network occupies most of the
band** and collides with everything in it.

The IDF driver will negotiate HT40 by default if it can. This project turns it
off, and the reasoning generalises:

> Left alone, this AP negotiated HT40: the log reads `wifi:new:<11,2>` and
> stations join as `bgn, 40D`. On channel 11 that puts the secondary at
> channel 7, so the AP occupies roughly the whole 2.4 GHz band and collides
> with every other network in it. Nothing here can use the width — the traffic
> is ~135 small datagrams a second per satellite, which is limited by transmit
> opportunities rather than by bits per symbol […]
>
> So it was paying the full interference cost of HT40 for none of its
> throughput.

— `hub_s3/main/net.c:531-544`

The general rule: **HT40 helps only if your bottleneck is bits per symbol.** If
your bottleneck is transmit opportunities — many small packets, as almost all
embedded traffic is — HT40 costs you interference and buys you nothing.

The call itself is one line, and must come *after* `esp_wifi_set_config()`,
which resets bandwidth to the default:

```c
const esp_err_t bw = esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW20);
```
— `hub_s3/main/net.c:559`

### Rate adaptation

Nobody picks a rate by hand in normal operation. The driver watches whether
frames are being acknowledged and moves up and down the rate table
automatically — fast rates while the link is good, falling back when
acknowledgements start failing. This is **rate adaptation**, and it is the
single most valuable automatic behaviour in the MAC.

You *can* pin the rate, via a private API, and this project has a Kconfig knob
for it. The comment beside it is the argument for not doing so:

> Unicast has no such fallback. It uses rate adaptation, which is what fixed
> the 23% loss that a fixed 24 Mbps caused. Note
> `esp_wifi_internal_set_fix_rate()` applies to ALL transmission on the
> interface, so any non-zero value here now pins unicast and disables that
> adaptation.

— `hub_s3/main/net.c:618-628`; the knob defaults to `0` (off) at
`hub_s3/sdkconfig.defaults:309`

Note the "private API" part: `esp_wifi_internal_set_fix_rate()` lives in
`components/esp_wifi/include/esp_private/wifi.h:278`, not in the public header.
Using it is supported in the sense that it works, and unsupported in the sense
that nothing promises it will exist next release. §9 comes back to this.

---

## 3 Sharing the air

One channel, many radios, no coordinator. 802.11's answer is **CSMA/CA** —
Carrier Sense Multiple Access with Collision Avoidance — and understanding it is
what makes latency behaviour on a busy channel comprehensible instead of
mysterious.

### Listen, wait, count down, send

Before transmitting, a station:

1. **Listens.** If the channel is busy, wait until it is not. (This is *carrier
   sense*, and it has two forms: physical — actual energy on the air, called
   Clear Channel Assessment or CCA — and virtual, from the duration field in
   frames it has overheard.)
2. **Waits a fixed interval** once the channel goes quiet, called an
   inter-frame space. DIFS for ordinary data (34 µs on OFDM), SIFS for the
   privileged replies like acknowledgements (16 µs). SIFS being shorter is what
   guarantees an ACK gets out before anyone else can start.
3. **Counts down a random backoff.** Pick a random number of slot times (9 µs
   each) from the *contention window*, and count down — pausing the countdown
   whenever the channel goes busy again. Transmit at zero.
4. **Waits for an ACK.** Every unicast frame must be acknowledged, SIFS after it
   ends. No ACK means the frame is presumed lost.
5. **Retries, with the contention window doubled.** The window starts at 0–15
   slots and doubles on each failure to 0–31, 0–63, up to 0–1023. This is
   *binary exponential backoff*, and it is what stops a busy channel collapsing
   into pure collision.

```
   channel busy          quiet
  ──────────────┐
                └──────────────────────────────────────────────────────
                 |<-DIFS->|<-- random backoff -->|<-- frame -->|<S>|ACK|
                                  9 µs slots                    16 µs
```

### What this means for latency

Everything in that sequence is *waiting*. On an idle channel the wait is tens of
microseconds and invisible. On a busy channel:

- carrier sense defers you indefinitely while someone else transmits;
- collisions cause retries, and each retry doubles the expected backoff;
- retries are invisible to your application — `sendto()` returned success long
  ago.

So **a congested channel damages latency and jitter long before it damages
throughput.** A link can deliver every packet and still deliver them in bursts
separated by hundreds of milliseconds. For a bulk file transfer that is
invisible. For synchronised audio it is the whole problem.

This project measures exactly that gap, and had to reach for an unusual
instrument to do it — see §20, and `tx_done_cb()` at `hub_s3/main/net.c:848`.

### Per-frame overhead is roughly constant

Add up the fixed costs around one frame: preamble and PHY header (20 µs for
OFDM), DIFS, the average backoff, SIFS, and the ACK itself. It comes to a few
tens of microseconds regardless of how big the frame was or how fast the rate.

That constant is why small frames are so expensive in airtime terms. A 100-byte
frame at 54 Mbps takes 15 µs to transmit and perhaps 50 µs of overhead to
deliver: **77% of the airtime is overhead.** It is also why this project's
airtime estimate uses a flat per-frame constant:

```c
#define OCCUPANCY_OVERHEAD_US 50   /* preamble + IFS + ACK, flat per frame */
```
— `hub_s3/main/net.c:127`

And it is the underlying reason for frame aggregation (§6).

### The hidden node problem

Carrier sense only works if everyone can hear everyone. Consider three stations
in a line: A can hear the AP, C can hear the AP, but A and C cannot hear each
other. Both sense the channel as idle, both transmit, and both frames collide at
the AP. Neither ever knows why.

```
      A  ←──────→  AP  ←──────→  C
      └── cannot hear each other ──┘
```

802.11's optional cure is **RTS/CTS**: A sends a short Request To Send, the AP
replies with a Clear To Send that *everyone* including C can hear, and C defers
for the announced duration. It costs two extra frames per transmission, so it is
normally enabled only above a size threshold, if at all.

On a dancefloor this is a live concern — satellites are scattered around a field
and cannot necessarily hear each other, only the hub. In practice the hub does
almost all the transmitting, which makes the classic hidden-node collision less
likely than it looks, but it is the right lens for reading unexplained loss
between two satellites at opposite corners.

---

## 4 Frames

Everything on the air is a frame. There are exactly three classes, and knowing
which class a problem lives in narrows it enormously.

| Class | What it does | Examples |
|---|---|---|
| **Management** | Building and tearing down associations | Beacon, Probe Request/Response, Authentication, Association Request/Response, Deauthentication, Disassociation, Action |
| **Control** | Coordinating access to the medium | ACK, RTS, CTS, Block Ack, Block Ack Request, PS-Poll |
| **Data** | Your actual payload | Data, QoS Data, Null Data |

The IDF's sniffer callback tells you which class each captured frame belongs to:

```c
typedef enum {
    WIFI_PKT_MGMT,  /**< Management frame, indicates 'buf' argument is wifi_promiscuous_pkt_t */
    WIFI_PKT_CTRL,  /**< Control frame, indicates 'buf' argument is wifi_promiscuous_pkt_t */
    WIFI_PKT_DATA,  /**< Data frame, indicates 'buf' argument is wifi_promiscuous_pkt_t */
    WIFI_PKT_MISC,  /**< Other type, such as MIMO etc. 'buf' argument is wifi_promiscuous_pkt_t but the payload is zero length. */
} wifi_promiscuous_pkt_type_t;
```
— `components/esp_wifi/include/esp_wifi_types_generic.h:697-702`

### The MAC header, and why there are four addresses

```
 ┌──────┬──────┬─────────┬─────────┬─────────┬──────┬─────────┬─────────┬─────┐
 │ Frame│ Dur- │ Address │ Address │ Address │ Seq  │ Address │ Payload │ FCS │
 │ Ctrl │ ation│    1    │    2    │    3    │ Ctrl │    4    │         │     │
 │  2 B │  2 B │   6 B   │   6 B   │   6 B   │ 2 B  │  6 B *  │ 0-2304  │ 4 B │
 └──────┴──────┴─────────┴─────────┴─────────┴──────┴─────────┴─────────┴─────┘
                                                     * only in mesh/WDS frames
```

Ethernet has two addresses — source and destination. 802.11 has three or four,
because a frame is usually *relayed*: a station sends to the AP, which forwards
to the destination. The three are receiver, transmitter, and the "other end"
(source or destination depending on direction), plus the BSSID identifying which
network the frame belongs to. The fourth appears only in wireless-bridge and
mesh frames.

Two other fields earn their keep:

- **Duration.** How long this exchange will occupy the channel. Every station
  that overhears it sets a countdown — the Network Allocation Vector — and stays
  quiet for that long without needing to sense anything. This is *virtual*
  carrier sense, and it is how RTS/CTS silences the hidden node.
- **Sequence Control.** A 12-bit sequence number and a 4-bit fragment number.
  Duplicate detection and block-ack reordering both hang off this.

The **FCS** is a CRC-32 over the whole frame. A frame that fails it is discarded
by the hardware and never reaches software — normally. §14 covers the case where
you deliberately want to see them anyway.

### Beacons

The AP transmits a **beacon** on a fixed period, by default every 100 TU. A
*TU* — Time Unit — is **1024 µs**, so 100 TU is 102.4 ms, not 100 ms. Every
timing argument that treats beacons as arriving every 100 ms is off by 2.4%, and
this project has a comment about getting that exactly wrong (§20).

A beacon carries the SSID, the supported rates, the security parameters, the
channel, the current TSF value, and the traffic indication map. It is how a
passive scan finds a network, how a station tracks the AP's clock, and how
buffered broadcast traffic is released. It is the metronome of a BSS.

The ESP-IDF exposes the interval as a field on the SoftAP config, with the
constraint stated plainly:

```c
uint16_t beacon_interval;                 /**< Beacon interval which should be multiples of 100. Unit: TU(time unit, 1 TU = 1024 us). Range: 100 ~ 60000. Default value: 100 */
```
— `components/esp_wifi/include/esp_wifi_types_generic.h:533`

**100 TU is the floor.** You cannot beacon faster than every 102.4 ms on an
ESP SoftAP, and §20 records what happened here when someone tried.

### What the radio tells you about a received frame

Every frame the ESP hands to a promiscuous callback is prefixed by a metadata
header. It is the closest thing to a spectrum analyser this hardware offers, and
it is worth reading in full:

```c
typedef struct {
    signed rssi: 8;               /**< Received Signal Strength Indicator(RSSI) of packet. unit: dBm */
    unsigned rate: 5;             /**< PHY rate encoding of the packet. Only valid for non HT(11bg) packet */
    unsigned : 1;                 /**< reserved */
    unsigned sig_mode: 2;         /**< Protocol of the received packet, 0: non HT(11bg) packet; 1: HT(11n) packet; 3: VHT(11ac) packet */
    unsigned : 16;                /**< reserved */
    unsigned mcs: 7;              /**< Modulation Coding Scheme. If is HT(11n) packet, shows the modulation, range from 0 to 76(MSC0 ~ MCS76) */
    unsigned cwb: 1;              /**< Channel Bandwidth of the packet. 0: 20MHz; 1: 40MHz */
    ...
    unsigned aggregation: 1;      /**< Aggregation. 0: MPDU packet; 1: AMPDU packet */
    unsigned stbc: 2;             /**< Space Time Block Code(STBC). 0: non STBC packet; 1: STBC packet */
    unsigned fec_coding: 1;       /**< Forward Error Correction(FEC). Flag is set for 11n packets which are LDPC */
    unsigned sgi: 1;              /**< Short Guide Interval(SGI). 0: Long GI; 1: Short GI */
    signed noise_floor: 8;        /**< noise floor of Radio Frequency Module(RF). unit: dBm*/
    unsigned ampdu_cnt: 8;        /**< the number of subframes aggregated in AMPDU */
    unsigned channel: 4;          /**< primary channel on which this packet is received */
    unsigned secondary_channel: 4; /**< secondary channel on which this packet is received. 0: none; 1: above; 2: below */
    unsigned : 8;                 /**< reserved */
    unsigned timestamp: 32;       /**< timestamp. The local time when this packet is received. It is precise only if modem sleep or light sleep is not enabled. unit: microsecond */
    ...
    unsigned sig_len: 12;         /**< length of packet including Frame Check Sequence(FCS) */
    unsigned : 12;                /**< reserved */
    unsigned rx_state: 8;         /**< state of the packet. 0: no error; others: error numbers which are not public */
} wifi_pkt_rx_ctrl_t;
```
— `components/esp_wifi/include/local/esp_wifi_types_native.h:37-86`, abridged;
the exact field layout is `#if`-guarded per target, so read it for *your* chip

```c
typedef struct {
    wifi_pkt_rx_ctrl_t rx_ctrl; /**< metadata header */
    uint8_t payload[0];       /**< Data or management payload. Length of payload is described by rx_ctrl.sig_len. Type of content determined by packet type argument of callback. */
} wifi_promiscuous_pkt_t;
```
— `esp_wifi_types_native.h:110-113`

`rssi`, `sig_len`, `rate`/`mcs`, `sig_mode`, `cwb`, `sgi` and `channel` are all
reliable and are what the airtime calculation in §2 is built from. Not every
field is: see §14 for `noise_floor`, which reads as a measurement and is not one.

---

## 5 Joining a network

The sequence from "radio on" to "I have an IP address" is five distinct steps.
They fail differently, so it pays to know which one you are in.

```
   scan ──▶ authenticate ──▶ associate ──▶ 4-way handshake ──▶ DHCP
    │            │               │               │               │
  find a      legacy          "I am a         derive the      get an
  network    formality      client of you"    session keys    address
                                                                 │
                                              ┌──────────────────┘
                                              │  ONLY NOW can you send IP traffic
```

The last arrow is the one embedded projects get wrong. **Associated is not
connected.** A station can be perfectly associated, with the radio reporting
everything healthy, and have no address and no route. §20 covers what that
costs.

### Scanning

Two kinds:

- **Passive.** Sit on each channel and listen for beacons. Costs nothing on the
  air, but you must dwell at least one beacon interval per channel to have a
  chance of hearing anything — and one interval gives you roughly one chance.
- **Active.** Broadcast a Probe Request and collect Probe Responses. Much
  faster, but you are transmitting, and you are announcing yourself.

The IDF's defaults tell you what the driver thinks reasonable:

```c
#define WIFI_ACTIVE_SCAN_MIN_DEFAULT_TIME 0             /**< Default minimum active scan time per channel */
#define WIFI_ACTIVE_SCAN_MAX_DEFAULT_TIME 120           /**< Default maximum active scan time per channel */
#define WIFI_PASSIVE_SCAN_DEFAULT_TIME 360              /**< Default passive scan time per channel */
#define WIFI_SCAN_HOME_CHANNEL_DWELL_DEFAULT_TIME 30    /**< Default time spent at home channel between scanning consecutive channels */
```
— `esp_wifi_types_generic.h:185-188`

360 ms passive against 120 ms active — because passive needs roughly three
beacon intervals to be confident, and active needs only a round trip. Thirteen
channels at 360 ms is about 4.7 seconds, which is why a full passive scan is a
boot-time-only affair.

This project scans passively and says why:

> **PASSIVE.** This unit has no business transmitting probe requests into a
> room it is about to be the AP of.
>
> 360 ms, WHICH IS IDF'S OWN DEFAULT (`WIFI_PASSIVE_SCAN_DEFAULT_TIME`), up
> from the 120 that used to be here. A beacon interval is ~102 ms, so 120
> gives roughly ONE chance per AP — the old comment read that as "catches at
> least one from everything that is there", and it is nearer a coin toss per
> network.

— `hub_s3/main/net.c:265-275`

### Authentication and association

**Authentication** in modern WiFi is a formality. Open System authentication is
a two-frame exchange that always succeeds; the real security happens later. (WEP
had Shared Key authentication here, and it was worse than useless — it leaked
keystream. It is gone.)

**Association** is the real handshake: the station requests, the AP grants, and
the station receives an Association ID (AID). From this point the AP will relay
its frames and buffer traffic for it.

WPA3 changes the picture: **SAE** (Simultaneous Authentication of Equals,
"Dragonfly") replaces the formality with a genuine password-authenticated key
exchange that happens *during* authentication and is resistant to offline
dictionary attack. That is the substantive security difference between WPA2 and
WPA3-Personal.

### The 4-way handshake

Both sides already know the **PMK** — the Pairwise Master Key — derived from the
passphrase and the SSID. The handshake proves each side has it without ever
transmitting it, and derives fresh per-session keys.

```
   AP                                                        Station
    │  1. ANonce (AP's random number)                             │
    │ ──────────────────────────────────────────────────────────▶ │
    │                                          station now has both
    │                                          nonces and can derive PTK
    │  2. SNonce + MIC (proves station has the PMK)               │
    │ ◀────────────────────────────────────────────────────────── │
    │  AP derives PTK, verifies the MIC                           │
    │  3. GTK (group key), encrypted + MIC                        │
    │ ──────────────────────────────────────────────────────────▶ │
    │  4. ACK                                                     │
    │ ◀────────────────────────────────────────────────────────── │
```

- **PTK** — Pairwise Transient Key. Unique per station per session. Encrypts
  unicast.
- **GTK** — Group Temporal Key. Shared by everyone on the BSS. Encrypts
  broadcast and multicast, which is why every station can decrypt them and why
  multicast is not private within a network.
- **MIC** — Message Integrity Code. The proof of possession.

**This is where a wrong password fails**, and the failure is confusing: the
station reports something like a handshake timeout rather than "wrong password",
because from the protocol's point of view nothing was rejected — a MIC simply
did not verify. `WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT = 15` and
`WIFI_REASON_HANDSHAKE_TIMEOUT = 204` are both in the reason table.

The confusion runs the other way too. This project has a comment recording a
misconfigured PMF field surfacing as an apparent password failure:

> `pmf_cfg` left unset makes some clients unhappy during the WPA2 handshake —
> which surfaces as "incorrect password" rather than anything that points at
> the real cause.

— `hub_s3/main/net.c:428-433`

### PMF, and reason 209

**Protected Management Frames** (802.11w) closes an old hole: management frames
are unencrypted, so anyone can forge a deauthentication and knock a station off
a network. PMF cryptographically protects deauth and disassoc frames. It is
mandatory for WPA3 and optional for WPA2.

It brings with it the **SA Query**: if an AP receives an association request
from a station it believes is already associated, it does not act on it. It
sends an SA Query to the existing association and waits. If the real station
answers, the new request was a forgery and is ignored. If nothing answers, the
existing association is torn down.

That last branch is a real failure mode when a station's PMF implementation does
not answer promptly, and it produces a distinctive reason code:

```c
WIFI_REASON_SA_QUERY_TIMEOUT                   = 209,    /**< SA query timeout */
```
— `esp_wifi_types_generic.h:170`

This project hit exactly that, and the write-up is a good model of how to reason
about a security trade-off:

> Observed: the AP starts an SA Query, the satellite does not answer six
> attempts, and the AP disassociates it with reason 209 — 1.7 s off the
> network, twice in the first 65 seconds of a run. The satellite never
> noticed: it counted zero disconnects while this unit counted two […]
>
> What is given up is protection of management frames — spoofed
> deauth/disassoc. Data stays encrypted under WPA2-PSK and the password is
> unchanged. For a closed floor with two boards that is a poor trade against
> losing a speaker every half minute.

— `hub_s3/main/net.c:563-587`

The mechanics matter for anyone else who needs to do this. In IDF 6,
`pmf_cfg.capable` is deprecated and forced true internally, so the only opt-out
is a dedicated call, and it has strict ordering:

```c
const esp_err_t pmf = esp_wifi_disable_pmf_config(WIFI_IF_AP);
```
— `hub_s3/main/net.c:588`, and its station twin at `satellite/main/net.c:344`.
It must be called **after `esp_wifi_set_config()` and before `esp_wifi_start()`**,
and it fails on a WPA3 or WPA2/WPA3-mixed SoftAP.

### DHCP, the step that is not WiFi at all

Association gives you a link. It does not give you an address. On an ESP SoftAP
the DHCP server runs inside `esp_netif`, hands out addresses on `192.168.4.x`
and puts the AP itself at `192.168.4.1`.

The station side raises two separate events — `WIFI_EVENT_STA_CONNECTED` when
the association completes and `IP_EVENT_STA_GOT_IP` when the lease lands — and
treating them as one is a genuine trap. §20 covers what it cost here.

### The full reason-code table

When a disconnect arrives, `wifi_event_sta_disconnected_t.reason` is the single
most informative byte available. Codes 1–68 are the 802.11 standard's own;
200 and up are Espressif's additions describing failures that never reached the
air. The full table is in [§22](#22-appendices).

---

## 6 Staying joined

Association is not a state that maintains itself. Several mechanisms keep it
alive, and each is a knob somebody can turn.

### TSF: the network's shared clock

Every BSS has a **Timing Synchronization Function** — a 64-bit microsecond
counter maintained by the AP. Every beacon carries its current value. Each
associated station's MAC **hardware** timestamps the beacon on arrival and
slaves its local TSF copy to it.

That "hardware" is the important word. A software timestamp taken around a
`sendto()` includes everything between reading the clock and the frame actually
leaving — scheduling, queueing, contention — and that variability is
irreducible. A MAC-layer timestamp is taken by the radio at the moment the
beacon arrives.

The IDF exposes it directly:

```c
int64_t esp_wifi_get_tsf_time(wifi_interface_t interface);
```
— `components/esp_wifi/include/esp_wifi.h:1313`

This is a genuinely under-used facility, and it is the foundation of this
project's clock synchronisation. Since both ends' TSFs track the *same* AP
counter, relating each unit's own TSF to its own local clock gives an offset
with no round trip in it:

```
    offset = (master_local - master_tsf) - (sat_local - sat_tsf)
```
— `components/dancefloor_sync/include/sync_proto.h:89`

Zero means the interface is not associated or has not yet seen a beacon.

Reading it correctly takes a little care, because the two clock reads are not
atomic. The satellite brackets one against the other so a preempted sample can
be identified and discarded:

```c
const int64_t tsf_a = esp_wifi_get_tsf_time(WIFI_IF_STA);
const int64_t my_local = esp_timer_get_time();
const int64_t tsf_b = esp_wifi_get_tsf_time(WIFI_IF_STA);
const int64_t span = tsf_b - tsf_a;
const int64_t my_tsf = (tsf_a + tsf_b) / 2;   /* centred on the timer read */
```
— `satellite/main/rx.c:971-975`

> **Note on a stale comment.** The block above `tsf_msg_t` in
> `sync_proto.h:73-74` still says TSF is "MEASUREMENT ONLY — nothing reads it
> but a log line". That has not been true since `clock_offset()` was written:
> `satellite/main/clock.c:100-107` prefers TSF whenever a reading is less than
> a second old and falls back to the round-trip estimator otherwise. The
> comment also cites `docs/clock-sync.md`, which no longer exists. Read the
> code, not that paragraph.

### DTIM and the buffering of group traffic

A station in power save is asleep between beacons. The AP therefore cannot send
it anything at will — it must buffer.

Each beacon carries a **TIM**, a Traffic Indication Map, a bitmap saying which
AIDs have unicast traffic waiting. A sleeping station wakes for the beacon,
checks its bit, and stays awake to collect if set.

Broadcast and multicast are different: they are for everybody, so they cannot be
delivered on demand. The AP buffers them and releases them **after a DTIM
beacon** — a Delivery Traffic Indication Message, which is every *n*th beacon
where *n* is `dtim_period`.

```
  beacon   beacon   beacon   DTIM     beacon   beacon   beacon   DTIM
    │        │        │      beacon     │        │        │      beacon
  ──┴────────┴────────┴────────┬───────┴────────┴────────┴────────┬────
                               │                                  │
                       group frames released              and again here
                                             dtim_period = 4 in this diagram
```

The consequence, which is not intuitive: **group-addressed frames are held for
`beacon_interval × dtim_period` regardless of whether any station is actually
sleeping.** With `dtim_period = 1` and the 100 TU floor, that is a fixed 102.4 ms
of latency on every multicast frame — and each held frame occupies a driver TX
buffer while it waits.

This project measured that directly, and it is one of the reasons the audio
stream is unicast:

> Power save is not a consideration here — every satellite sets `WIFI_PS_NONE`
> — and that does NOT mean group frames went out immediately: the AP buffers
> them for DTIM regardless of whether any station is sleeping. The 2026-08-20
> soak measured it directly, with ENOMEM refusals arriving at the beacon rate
> (median 40 bursts per 5 s window against 48.8 beacons).

— `hub_s3/main/net.c:479-485`

### Power save

A station may tell the AP it is going to sleep, and the AP will buffer for it.
Three modes:

```c
typedef enum {
    WIFI_PS_NONE,        /**< No power save */
    WIFI_PS_MIN_MODEM,   /**< Minimum modem power saving. In this mode, station wakes up to receive beacon every DTIM period */
    WIFI_PS_MAX_MODEM,   /**< Maximum modem power saving. In this mode, interval to receive beacons is determined by the listen_interval parameter in wifi_sta_config_t */
} wifi_ps_type_t;
```
— `esp_wifi_types_generic.h:374-378`

**The default is `WIFI_PS_MIN_MODEM`, not `WIFI_PS_NONE`** — a fact worth
knowing, because it means a fresh ESP-IDF project has power save on and pays
tens of milliseconds of latency for it without anyone choosing that. This
project turns it off explicitly at both ends:

```c
/* Power save would park the radio between beacons and add tens of ms to
 * the packets whose timing we depend on. */
ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
```
— `hub_s3/main/net.c:596`, and `satellite/main/net.c:349`

If you are on a battery and latency does not matter, the reverse trade is
excellent: `WIFI_PS_MAX_MODEM` with a large `listen_interval` can cut average
current by an order of magnitude.

### Aggregation and block ack

§3 established that per-frame overhead is roughly constant. If you must send
many frames, the way to avoid paying that constant repeatedly is to send them
as one transmission.

- **A-MSDU** aggregates several payloads into one MAC frame. One header, one
  FCS, one ACK. Cheapest, but a single bit error destroys everything in it.
- **A-MPDU** aggregates several complete MAC frames into one PHY transmission,
  each with its own FCS. Slightly more overhead, but individually recoverable —
  and that recovery is the point.

A-MPDU is acknowledged by a **Block Ack**: a single control frame carrying a
bitmap of which subframes arrived. The receiver asks for retransmission of just
the missing ones.

**This is the part that is routinely misunderstood, and it cost this project a
soak to learn.** Block ack is not only a throughput feature. It is also the
mechanism by which individual lost frames are selectively retransmitted. Turning
aggregation off does not just cost you throughput — it removes the loss recovery
too:

> The mechanism did what it was meant to — the worst single refusal stretch
> fell to 486 ms and satellite underruns roughly halved — but that was never
> the fault. […]
>
> **THE COST THE ESTIMATE MISSED.** Block-ack is also efficient selective
> retransmission; without it a lost frame is simply lost. `gaps` more than
> tripled.

— `hub_s3/sdkconfig.defaults:196-205`

The **BA window** is how many subframes may be outstanding in one block-ack
exchange. It is negotiated on the air — the two ends agree on the minimum of
what each offers — and it is a buffer commitment on both sides. §12 and §20
return to it.

### Keepalives and inactivity

An AP will eventually disassociate a station it has not heard from. The IDF
exposes the timeout:

```c
esp_err_t esp_wifi_set_inactive_time(wifi_interface_t ifx, uint16_t sec);
```
— `esp_wifi.h:1333`

A station that only ever *receives* must still transmit something occasionally —
even a null data frame — or it will be thrown off as idle. This is
`WIFI_REASON_DISASSOC_DUE_TO_INACTIVITY = 4`. In this project the satellites'
clock probes double as keepalives, which is why the point never had to be
argued.

The reverse direction is **beacon timeout**: a station that stops hearing
beacons concludes the AP is gone. That produces
`WIFI_REASON_BEACON_TIMEOUT = 200` and a `WIFI_EVENT_STA_BEACON_TIMEOUT` event.

---
---
# Part II — The ESP32's WiFi

Everything from here is ESP-IDF **v6.0.1** as installed on this machine at
`/home/pico/.espressif/v6.0.1/esp-idf`. Paths in this part are relative to that
root unless they clearly belong to this repository.

---

## 7 The silicon

### What is in the chip

The ESP32 family implements 802.11 in hardware up to and including the MAC. A
dedicated radio does the modulation and demodulation; a MAC block does carrier
sense, backoff, acknowledgement, retry, encryption and aggregation. Firmware
sitting on one of the CPU cores drives it.

That division is why some things are cheap and some are impossible. Rate
adaptation, retries and ACKs happen without the CPU knowing. But you cannot,
for example, get a per-frame timestamp at nanosecond resolution, or transmit an
arbitrary PHY waveform, because those live below the interface the hardware
exposes.

### One antenna, one spatial stream

Every 2.4 GHz ESP32 has a single antenna and a single spatial stream. That
caps 802.11n at MCS 0–7: 65 Mbps at 20 MHz with a long guard interval, 72.2
with a short one, and 150 Mbps at 40 MHz with a short one. Real throughput is
well below all of those.

Some modules have two antenna connectors and an RF switch. That is
**diversity**, not MIMO: the chip picks whichever antenna is hearing better, one
at a time. In IDF 6 the antenna API moved out of `esp_wifi` into `esp_phy` —
`esp_phy_set_ant()`, `esp_phy_set_ant_gpio()` — see §17.

### What differs between targets

Capabilities live in `components/soc/<target>/include/soc/soc_caps.h`, and the
headers `#if`-guard on them. Two that matter here:

| | classic ESP32 | ESP32-S3 |
|---|---|---|
| Bluetooth | Classic **and** BLE (`SOC_BT_CLASSIC_SUPPORTED`, `soc_caps.h:394`) | BLE only (`SOC_BLE_SUPPORTED`, `soc_caps.h:522`) |
| FTM (fine timing measurement) | not present | `SOC_WIFI_FTM_SUPPORT`, `soc_caps.h:512` |
| GCMP ciphers | not present | `SOC_WIFI_GCMP_SUPPORT`, `soc_caps.h:513` |
| USB/WiFi PHY interference | — | `SOC_WIFI_PHY_NEEDS_USB_WORKAROUND`, `soc_caps.h:519` |
| Max SoftAP clients | 15 | 15 |

The Bluetooth row is why this project's master is two chips at all: A2DP needs
Bluetooth Classic, which only the original ESP32 has. §16.

The last row is a genuine hardware quirk worth knowing about if you ever run
WiFi and native USB on an S3. The USB PHY and the WiFi PHY interfere, and the
IDF's mitigation is blunt — it powers the USB PHY down when WiFi starts. This
project takes the mitigation and pays the price:

```
CONFIG_ESP_PHY_ENABLE_USB=n
```
— `hub_s3/sdkconfig.defaults:104`, with about 40 lines of explanation above it,
including what it costs at the bench: the USB PHY is down from the moment WiFi
starts, so the USB console goes away.

The maximum SoftAP client count is a per-target constant:

```c
#define ESP_WIFI_MAX_CONN_NUM  (4)        /**< max number of stations which can connect to ESP32C2 soft-AP */
#define ESP_WIFI_MAX_CONN_NUM  (10)       /**< max number of stations which can connect to ESP32C3/ESP32C6/ESP32C5/ESP32C61 soft-AP */
#define ESP_WIFI_MAX_CONN_NUM  (15)       /**< max number of stations which can connect to ESP32/ESP32S3/ESP32S2 soft-AP */
```
— `components/esp_wifi/include/local/esp_wifi_types_native.h:20-24`, the three
branches of one `#if`

### Closed source, and what that means in practice

The 802.11 MAC implementation ships as prebuilt static libraries:

```
components/esp_wifi/lib/esp32/
    libcore.a  libespnow.a  libmesh.a  libnet80211.a  libpp.a
    libsmartconfig.a  libwapi.a
```

`libpp.a` is the "packet processor" — the layer just above the hardware.
`libnet80211.a` is the MAC state machine. These are binary blobs and you cannot
read them.

What *is* open, and worth reading, is everything around them:
`components/esp_wifi/src/wifi_init.c` (the init path, the power-management
callback registration), `components/esp_wifi/esp32*/esp_adapter.c` (the OS
shim — this is where `wifi_malloc()` is defined and therefore where the PSRAM
allocation question in §12 is decided), and the whole of `esp_netif`.

The practical consequence: when behaviour inside the driver surprises you, you
cannot read the source to find out why. You must **measure**. That constraint
shapes all of Part III.

### RF calibration

The radio needs calibrating against temperature and supply. The `esp_phy`
component does this at init and stores the result in NVS, which is why
`nvs_flash_init()` must run before `esp_wifi_init()` in almost every project.

The calibration mode is configurable:

| Symbol | Behaviour |
|---|---|
| `ESP_PHY_RF_CAL_PARTIAL` | default — quick calibration using stored data |
| `ESP_PHY_RF_CAL_NONE` | reuse stored data entirely; fastest boot, worst RF if conditions changed |
| `ESP_PHY_RF_CAL_FULL` | full calibration every boot; ~100 ms slower, best RF |

— `components/esp_phy/Kconfig:136-154`

If you are chasing a WiFi problem that only appears in a cold room or only after
a battery change, `ESP_PHY_RF_CAL_FULL` is a cheap thing to eliminate.

---

## 8 The stack, and who calls whom

```
   ┌───────────────────────────────────────────────────────────────┐
   │  your app       socket(), sendto(), recvfrom()                │
   ├───────────────────────────────────────────────────────────────┤
   │  lwIP           TCP/UDP/IP, ARP, the DHCP client and server   │
   │                 runs on its own task ("tiT")                  │
   ├───────────────────────────────────────────────────────────────┤
   │  esp_netif      glue: binds a WiFi interface to an lwIP netif │
   │                 owns the DHCP server, posts IP_EVENT          │
   ├───────────────────────────────────────────────────────────────┤
   │  esp_wifi       the driver. Public API + a WiFi task.         │
   │                 posts WIFI_EVENT. Owns the RX/TX buffer pools │
   ├───────────────────────────────────────────────────────────────┤
   │  libnet80211 /  the 802.11 MAC state machine and packet       │
   │  libpp          processor. Closed source.                     │
   ├───────────────────────────────────────────────────────────────┤
   │  PHY + MAC hw   modulation, CSMA/CA, ACK, retry, crypto       │
   └───────────────────────────────────────────────────────────────┘

   esp_event ── the default event loop task, carrying WIFI_EVENT and
                IP_EVENT to whatever registered for them
```

Four separate tasks are involved in getting one packet out: yours, lwIP's
`tiT`, the WiFi driver's, plus interrupt context underneath. Each is a queue,
and each queue is a place a packet can wait or be dropped.

### The bring-up order, and why it is not negotiable

```
   nvs_flash_init()                    ← PHY calibration data lives here
        │
   esp_netif_init()                    ← starts the lwIP task
        │
   esp_event_loop_create_default()     ← creates the event loop task
        │
   esp_netif_create_default_wifi_ap()  ← or _sta(). Makes the netif AND
        │                                registers the default handlers that
        │                                wire WiFi events to lwIP
   esp_wifi_init(&cfg)                 ← allocates all the buffers; starts the
        │                                WiFi task
   esp_wifi_set_mode(...)
        │
   esp_wifi_set_config(IF, &cfg)
        │
   [ esp_wifi_disable_pmf_config() ]   ← must be here: after set_config,
        │                                before start
   esp_wifi_start()
        │
   [ esp_wifi_set_ps(...) ]            ← must be after start
   [ esp_wifi_set_tx_done_cb(...) ]    ← must be after start
```

Several of those ordering constraints are not obvious and are learned by having
a call return an error. This project annotates each of them where it makes the
call; they are collected here because they are general.

`esp_netif_create_default_wifi_ap()` / `_sta()` do more than allocate a struct.
They register the default event handlers that connect the two layers — the
handler that starts DHCP when the station associates, the handler that starts
the DHCP server when the AP starts. Skip them and you have a working radio with
no IP stack at all.

Which is sometimes exactly what you want. The hub's channel survey does a scan
with no netif deliberately:

```c
/* Scanning needs STA mode and a started radio. No STA netif is created for
 * it: a scan wants no IP stack, and the AP netif made above is untouched. */
ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
ESP_ERROR_CHECK(esp_wifi_start());
```
— `hub_s3/main/net.c:258-261`

### The real sequences in this repo

Hub, `hub_s3/main/net.c:400-416`:

```c
ESP_ERROR_CHECK(esp_netif_init());
ESP_ERROR_CHECK(esp_event_loop_create_default());
s_ap_netif = esp_netif_create_default_wifi_ap();
ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                   WIFI_EVENT_AP_STADISCONNECTED,
                                                   wifi_event, NULL, NULL));
ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                   IP_EVENT_ASSIGNED_IP_TO_CLIENT,
                                                   wifi_event, NULL, NULL));

wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
ESP_ERROR_CHECK(esp_wifi_init(&cfg));
```

Satellite, `satellite/main/net.c:309-332` — same shape, `_sta` instead of `_ap`,
and it registers for `ESP_EVENT_ANY_ID` on `WIFI_EVENT` rather than picking
individual events.

The whole-system order is in `hub_s3/main/streamer.c:50-105`
(`nvs_flash_init()` → allocation hook → ring → `wifi_start_ap()` →
`socket_start()` → `wifi_log_init()` → I2S → tasks) and
`satellite/main/main.c:148-203`.

### esp_netif, briefly

`esp_netif` is a thin abstraction between a driver and a TCP/IP stack. In
practice that stack is lwIP. Its jobs:

- own the netif handle, IP configuration and DHCP client/server;
- translate WiFi driver events into `IP_EVENT`;
- give you thread-safe access to lwIP state.

That last point has a sharp edge. lwIP is not thread-safe, and reaching into it
from your own task will corrupt it eventually. `esp_netif` provides a way to run
a callback *on the lwIP task*:

```c
esp_err_t esp_netif_tcpip_exec(esp_netif_callback_fn fn, void *ctx);
```
— `components/esp_netif/include/esp_netif.h:1084`

This project uses it to seed a static ARP entry the moment a client gets its
DHCP lease, so the first audio packet does not have to wait for ARP resolution
(`hub_s3/main/clients.c`). It also uses the DHCP server's own lease table to turn
a departing station's MAC back into the IP its send list is keyed by:

```c
esp_err_t esp_netif_dhcps_get_clients_by_mac(esp_netif_t *esp_netif, int num, esp_netif_pair_mac_ip_t *mac_ip_pair);
```
— `esp_netif.h:720`

Both are worth knowing about. Neither is in any getting-started tutorial.

---

## 9 `wifi_init_config_t`, the one struct that matters

`esp_wifi_init()` takes a single struct, and that struct is where every memory
and feature decision in §12 and §13 actually lands. It is worth reading once in
full:

```c
typedef struct {
    wifi_osi_funcs_t*      osi_funcs;              /**< WiFi OS functions */
    wpa_crypto_funcs_t     wpa_crypto_funcs;       /**< WiFi station crypto functions when connect */
    int                    static_rx_buf_num;      /**< WiFi static RX buffer number */
    int                    dynamic_rx_buf_num;     /**< WiFi dynamic RX buffer number */
    int                    tx_buf_type;            /**< WiFi TX buffer type */
    int                    static_tx_buf_num;      /**< WiFi static TX buffer number */
    int                    dynamic_tx_buf_num;     /**< WiFi dynamic TX buffer number */
    int                    rx_mgmt_buf_type;       /**< WiFi RX MGMT buffer type */
    int                    rx_mgmt_buf_num;        /**< WiFi RX MGMT buffer number */
    int                    cache_tx_buf_num;       /**< WiFi TX cache buffer number */
    int                    csi_enable;             /**< WiFi channel state information enable flag */
    int                    ampdu_rx_enable;        /**< WiFi AMPDU RX feature enable flag */
    int                    ampdu_tx_enable;        /**< WiFi AMPDU TX feature enable flag */
    int                    amsdu_tx_enable;        /**< WiFi AMSDU TX feature enable flag */
    int                    nvs_enable;             /**< WiFi NVS flash enable flag */
    int                    nano_enable;            /**< Nano option for printf/scan family enable flag */
    int                    rx_ba_win;              /**< WiFi Block Ack RX window size */
    int                    wifi_task_core_id;      /**< WiFi Task Core ID */
    int                    beacon_max_len;         /**< WiFi softAP maximum length of the beacon */
    int                    mgmt_sbuf_num;          /**< WiFi management short buffer number, the minimum value is 6, the maximum value is 32 */
    uint64_t               feature_caps;           /**< Enables additional WiFi features and capabilities */
    bool                   sta_disconnected_pm;    /**< WiFi Power Management for station at disconnected status */
    int                    espnow_max_encrypt_num; /**< Maximum encrypt number of peers supported by espnow */
    int                    tx_hetb_queue_num;      /**< WiFi TX HE TB QUEUE number for STA HE TB PPDU transmission */
    bool                   dump_hesigb_enable;     /**< enable dump sigb field */
    int                    magic;                  /**< WiFi init magic number, it should be the last field */
} wifi_init_config_t;
```
— `components/esp_wifi/include/esp_wifi.h:97-124`

You are not meant to fill this in by hand. There is a macro, and every field of
it is a `CONFIG_*` symbol:

```c
#define WIFI_INIT_CONFIG_DEFAULT() { \
    .osi_funcs = &g_wifi_osi_funcs, \
    .wpa_crypto_funcs = g_wifi_default_wpa_crypto_funcs, \
    .static_rx_buf_num = CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM,\
    .dynamic_rx_buf_num = CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM,\
    .tx_buf_type = CONFIG_ESP_WIFI_TX_BUFFER_TYPE,\
    .static_tx_buf_num = WIFI_STATIC_TX_BUFFER_NUM,\
    .dynamic_tx_buf_num = WIFI_DYNAMIC_TX_BUFFER_NUM,\
    ...
    .magic = WIFI_INIT_CONFIG_MAGIC\
}
```
— `esp_wifi.h:316-343`

**This is the mechanism by which menuconfig actually reaches the driver.** When
§13 says "this option controls X", this macro is how. It is also why the header
warns you, twice, to always start from the macro:

> Always use `WIFI_INIT_CONFIG_DEFAULT` macro to initialize the configuration to
> default values, this can guarantee all the fields get correct value when more
> fields are added into `wifi_init_config_t` in future release. If you want to
> set your own initial values, overwrite the default values which are set by
> `WIFI_INIT_CONFIG_DEFAULT`. Please be notified that the field 'magic' of
> `wifi_init_config_t` should always be `WIFI_INIT_CONFIG_MAGIC`!

— `esp_wifi.h:351-355`

Both firmwares here do exactly that and override nothing:

```c
wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
ESP_ERROR_CHECK(esp_wifi_init(&cfg));
```
— `hub_s3/main/net.c:415-416`, `satellite/main/net.c:315-316`

Everything is set through `sdkconfig.defaults` instead, which is the right
choice: it keeps the tuning in one reviewable file rather than scattered through
C, and it survives someone reading the C and not knowing what a default is.

### Public and private headers

The public API is `esp_wifi.h` (89 functions), `esp_wifi_types_generic.h` (the
types), `esp_now.h` (21 functions), `esp_netif.h`.

There is also `components/esp_wifi/include/esp_private/wifi.h`. It is installed,
it compiles, and it contains genuinely useful things — but the directory name is
the contract. This project uses two of them and flags both:

| Private API | What it gives you | Where used |
|---|---|---|
| `esp_wifi_internal_set_fix_rate()` (`esp_private/wifi.h:278`) | pin the PHY rate; disables rate adaptation | `hub_s3/main/net.c:629`, behind a Kconfig knob defaulting to off |
| `esp_wifi_set_tx_done_cb()` (`esp_private/wifi.h:611`) | a callback when a frame actually leaves the radio | `hub_s3/main/net.c:601` |

The second one is the more interesting: there is no public way to learn what the
radio did, as opposed to what `sendto()` accepted. §20 is largely about why that
distinction matters.

---

## 10 Configuring an interface

### Modes

```c
esp_err_t esp_wifi_set_mode(wifi_mode_t mode);
```
— `esp_wifi.h:392`

| Mode | Meaning |
|---|---|
| `WIFI_MODE_NULL` | radio initialised, no interface. Sniffing works here. |
| `WIFI_MODE_STA` | station: join someone else's network |
| `WIFI_MODE_AP` | SoftAP: be the network |
| `WIFI_MODE_APSTA` | both at once — **on the same channel**, always |
| `WIFI_MODE_NAN` | Wi-Fi Aware |

`WIFI_MODE_APSTA` deserves its warning. There is one radio. When it is a station
it must follow its AP's channel, and its own SoftAP is therefore stuck on that
same channel. Join a network on channel 1 and your SoftAP moves to channel 1,
disconnecting anything that had joined it on channel 6. Every "my AP keeps
dropping clients when the station reconnects" bug is this.

### The SoftAP config

```c
typedef struct {
    uint8_t ssid[32];                         /**< SSID of soft-AP. If ssid_len field is 0, this must be a Null terminated string. Otherwise, length is set according to ssid_len. */
    uint8_t password[64];                     /**< Password of soft-AP. */
    uint8_t ssid_len;                         /**< Optional length of SSID field. */
    uint8_t channel;                          /**< Channel of soft-AP. Set to 0 for auto selection (min channel: typically 1 for 2.4G, 36 for 5G). Other invalid values return ESP_ERR_INVALID_ARG. */
    wifi_auth_mode_t authmode;                /**< Auth mode of soft-AP. Do not support AUTH_WEP, AUTH_WAPI_PSK and AUTH_OWE in soft-AP mode. ... */
    uint8_t ssid_hidden;                      /**< Broadcast SSID or not, default 0, broadcast the SSID */
    uint8_t max_connection;                   /**< Max number of stations allowed to connect in */
    uint16_t beacon_interval;                 /**< Beacon interval which should be multiples of 100. Unit: TU(time unit, 1 TU = 1024 us). Range: 100 ~ 60000. Default value: 100 */
    uint8_t csa_count;                        /**< Channel Switch Announcement Count. Notify the station that the channel will switch after the csa_count beacon intervals. Default value: 3 */
    uint8_t dtim_period;                      /**< Dtim period of soft-AP. Range: 1 ~ 10. Default value: 1 */
    wifi_cipher_type_t pairwise_cipher;       /**< Pairwise cipher of SoftAP, group cipher will be derived using this. ... */
    bool ftm_responder;                       /**< Enable FTM Responder mode */
    wifi_pmf_config_t pmf_cfg;                /**< Configuration for Protected Management Frame */
    wifi_sae_pwe_method_t sae_pwe_h2e;        /**< Configuration for SAE PWE derivation method. Default value :2 (WPA3_SAE_PWE_BOTH) */
    uint8_t transition_disable: 1;            /**< Whether to enable transition disable feature */
    uint8_t sae_ext: 1;                       /**< Enable SAE EXT feature. SOC_GCMP_SUPPORT is required for this feature. */
    uint8_t wpa3_compatible_mode: 1;          /**< Enable WPA3 compatible authmode feature. ... */
    uint8_t reserved: 5;                      /**< Reserved for future feature set */
    wifi_bss_max_idle_config_t bss_max_idle_cfg;  /**< Configuration for bss max idle, effective if CONFIG_WIFI_BSS_MAX_IDLE_SUPPORT is enabled */
    uint16_t gtk_rekey_interval;              /**< GTK rekeying interval in seconds. If set to 0, GTK rekeying is disabled. Range: 60 ~ 65535 including 0. */
} wifi_ap_config_t;
```
— `esp_wifi_types_generic.h:525-546`, lightly abridged

Fields whose defaults are worth a second look:

- **`channel = 0`** means auto-select, and "auto" on an ESP SoftAP means the
  lowest channel, not a good one. If the channel matters to you, choose it. §19
  is a whole worked example of choosing it well.
- **`dtim_period`** — range 1 to 10, and **zero is invalid**. A zero-initialised
  struct gives you an invalid value here, which is why this project sets it
  explicitly (`hub_s3/main/net.c:445` and the comment at `:426-433`).
- **`beacon_interval`** — 100 TU floor, as §4 established.
- **`max_connection`** — capped by `ESP_WIFI_MAX_CONN_NUM` for the target. Note
  this is one of three limits that must agree in a project like this one: the
  AP's `max_connection`, the application's own client table, and
  `CONFIG_LWIP_DHCPS_MAX_STATION_NUM`. The third is the quiet one — run out of
  DHCP leases and a station associates fine and simply never gets an address.
  Here all three are 15 (`hub_s3/main/net.c:442`, `hub.h:1291`,
  `hub_s3/sdkconfig.defaults:981`).
- **`csa_count`** — Channel Switch Announcement. A SoftAP that changes channel
  can warn its stations this many beacons ahead instead of just vanishing.

### The station config

```c
typedef struct {
    uint8_t ssid[32];                         /**< SSID of target AP. */
    uint8_t password[64];                     /**< Password of target AP. */
    wifi_scan_method_t scan_method;           /**< Do all channel scan or fast scan */
    bool bssid_set;                           /**< Whether set MAC address of target AP or not. ... */
    uint8_t bssid[6];                         /**< MAC address of target AP*/
    uint8_t channel;                          /**< Channel hint for target AP. ... Set to 0 for no preference */
    uint16_t listen_interval;                 /**< Listen interval for ESP32 station to receive beacon when WIFI_PS_MAX_MODEM is set. Units: AP beacon intervals. Defaults to 3 if set to 0. */
    wifi_sort_method_t sort_method;           /**< Sort the connect AP in the list by rssi or security mode */
    wifi_scan_threshold_t  threshold;         /**< When scan_threshold is set, only APs which have an auth mode that is more secure than the selected auth mode and a signal stronger than the minimum RSSI will be used. */
    wifi_pmf_config_t pmf_cfg;                /**< Configuration for Protected Management Frame. Will be advertised in RSN Capabilities in RSN IE. */
    uint32_t rm_enabled: 1;                   /**< Whether Radio Measurements are enabled for the connection */
    uint32_t btm_enabled: 1;                  /**< Whether BSS Transition Management is enabled for the connection. ... */
    uint32_t mbo_enabled: 1;                  /**< Whether MBO is enabled for the connection. ... */
    uint32_t ft_enabled: 1;                   /**< Whether FT is enabled for the connection */
    uint32_t owe_enabled: 1;                  /**< Whether OWE is enabled for the connection */
    ...
    uint8_t failure_retry_cnt;                /**< Number of connection retries station will do before moving to next AP. ... */
    ...
} wifi_sta_config_t;
```
— `esp_wifi_types_generic.h:553-589`, abridged

Three fields are worth knowing about:

- **`channel`** is a *hint*: scan starting there. If your AP's channel is fixed
  and known, setting this shortens the join noticeably.
- **`listen_interval`** only matters under `WIFI_PS_MAX_MODEM`. It is how many
  beacon intervals the station may sleep through. Bigger = less current, more
  latency.
- **`threshold`** filters candidate APs by minimum RSSI and minimum auth mode.
  The auth-mode filter is a real security control: without it, a station
  configured for WPA2 will happily join an open network with the same SSID.

This project sets only what it needs (`satellite/main/net.c:318-326`): SSID,
password, and — in the diagnostic open-AP build — `threshold.authmode`.

### Channel and bandwidth

```c
esp_err_t esp_wifi_set_channel(uint8_t primary, wifi_second_chan_t second);
```
— `esp_wifi.h:799`

```c
typedef enum {
    WIFI_SECOND_CHAN_NONE = 0,  /**< The channel width is HT20 */
    WIFI_SECOND_CHAN_ABOVE,     /**< The channel width is HT40 and the secondary channel is above the primary channel */
    WIFI_SECOND_CHAN_BELOW,     /**< The channel width is HT40 and the secondary channel is below the primary channel */
} wifi_second_chan_t;
```
— `esp_wifi_types_generic.h:179-183`

`esp_wifi_set_channel()` is for sniffing and for `WIFI_MODE_NULL`. To set a
SoftAP's channel you use `wifi_ap_config_t.channel`; a station's channel is
whatever its AP says.

Bandwidth is separate and **must be set after `esp_wifi_set_config()`**, which
resets it:

```c
esp_err_t esp_wifi_set_bandwidth(wifi_interface_t ifx, wifi_bandwidth_t bw);
```
— `esp_wifi.h:757`

In IDF 6 the values are `WIFI_BW20` and `WIFI_BW40`; the old `WIFI_BW_HT20` and
`WIFI_BW_HT40` spellings were removed (§17).

Note the asymmetry, which surprises people: **bandwidth is an AP-side setting.**
A station follows the BSS it joins. This project's satellite `sdkconfig.defaults`
carries an explicit *negative* entry about that at `:42-49` — HT20 is not set
there, because the call would do nothing.

### Protocol, TX power, power save

```c
esp_err_t esp_wifi_set_protocol(wifi_interface_t ifx, uint8_t protocol_bitmap);
esp_err_t esp_wifi_set_max_tx_power(int8_t power);
esp_err_t esp_wifi_set_ps(wifi_ps_type_t type);
```
— `esp_wifi.h:715`, `:1145`, `:680`

`esp_wifi_set_max_tx_power()` takes power in **0.25 dBm units** — 78 is 19.5 dBm.
There is a Kconfig ceiling too (`ESP_PHY_MAX_WIFI_TX_POWER`).

Turning TX power *down* is a common instinct and often wrong. This project tried
it and reverted, and the reason generalises:

> It was once capped at 13 dBm to stop this radio swamping the Bluetooth
> receiver when both shared one chip; the two-chip split removed that need, and
> the cap did real harm by denying rate adaptation the SNR margin it needs.

— `hub_s3/main/net.c:608-612`

Rate adaptation trades SNR for speed. Take away the SNR and it has nothing to
trade — you get a slower link that occupies *more* airtime, not less.

---

## 11 Events

The WiFi driver does not call you back directly for state changes. It posts to
the default event loop, and you register handlers.

```c
esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, handler, NULL, NULL);
esp_event_handler_instance_register(IP_EVENT,  IP_EVENT_STA_GOT_IP, handler, NULL, NULL);
```

### The events that matter

| Event | When | Data struct |
|---|---|---|
| `WIFI_EVENT_STA_START` | `esp_wifi_start()` completed in STA mode | — |
| `WIFI_EVENT_STA_CONNECTED` | association complete. **Not** an IP address | `wifi_event_sta_connected_t` |
| `WIFI_EVENT_STA_DISCONNECTED` | association lost or attempt failed | `wifi_event_sta_disconnected_t` — carries `reason` |
| `WIFI_EVENT_STA_BEACON_TIMEOUT` | stopped hearing the AP | — |
| `WIFI_EVENT_SCAN_DONE` | an async scan finished | `wifi_event_sta_scan_done_t` |
| `WIFI_EVENT_AP_START` / `_STOP` | SoftAP up / down | — |
| `WIFI_EVENT_AP_STACONNECTED` | a station associated | `wifi_event_ap_staconnected_t` — MAC + AID |
| `WIFI_EVENT_AP_STADISCONNECTED` | a station left | `wifi_event_ap_stadisconnected_t` — MAC + reason |
| `WIFI_EVENT_AP_WRONG_PASSWORD` | a station failed the handshake | — |
| `WIFI_EVENT_HOME_CHANNEL_CHANGE` | the operating channel moved | `wifi_event_home_channel_change_t` |
| `IP_EVENT_STA_GOT_IP` | **DHCP lease acquired — you can now send** | `ip_event_got_ip_t` |
| `IP_EVENT_STA_LOST_IP` | lease lost | — |
| `IP_EVENT_AP_STAIPASSIGNED` / `IP_EVENT_ASSIGNED_IP_TO_CLIENT` | the SoftAP's DHCP server issued a lease | `ip_event_assigned_ip_to_client_t` — MAC **and** IP |

The full enum is at `esp_wifi_types_generic.h:1086-1150` and runs to about fifty
entries, most of them for features (WPS, NAN, TWT, DPP, FTM) you will never
enable.

### Associated is not connected, and the events say so

`WIFI_EVENT_STA_CONNECTED` and `IP_EVENT_STA_GOT_IP` are separate events with a
DHCP exchange between them, and a station can sit between them indefinitely. The
radio is content, so no disconnect event is coming. Nothing in the driver will
ever rescue you.

This project builds a three-state machine rather than a two-state one for
exactly that reason:

```c
/*
 *   !s_assoc               not on the AP          -> ask to connect
 *   s_assoc && !s_link_up  on the AP, no address  -> wait, then tear it down
 *   s_link_up              on the floor           -> nothing to do
 */
static volatile bool    s_assoc;
static volatile bool    s_link_up;
```
— `satellite/main/net.c:62-84`

and the middle state has a watchdog:

> An association that succeeds raises `STA_CONNECTED` and nothing else. If
> DHCP then never completes there is no lease, and — this is the part that
> strands the unit — no `STA_DISCONNECTED` either, because as far as the radio
> is concerned everything is fine. […] Only a reboot ended it.

— `satellite/main/net.c:119-128`

The cure is to stop being associated — `esp_wifi_disconnect()` raises
`STA_DISCONNECTED`, which drops into the reconnect path that already exists
(`satellite/main/net.c:162`).

The AP side has a matching subtlety. This project registers for
`IP_EVENT_ASSIGNED_IP_TO_CLIENT`, not `WIFI_EVENT_AP_STACONNECTED`:

```c
/* The arrival half. Not WIFI_EVENT_AP_STACONNECTED: a station is associated
 * before it has an address, and the send list is keyed by one. */
```
— `hub_s3/main/net.c:409-410`

### The trap: handlers run on the event loop task

**Every handler for every event base shares one task.** Block in one and you
block them all.

This is not a hypothetical:

> **THIS USED TO BE A vTaskDelay INSIDE THE EVENT HANDLER**, which runs on the
> default event loop's task — so a disconnect stopped `esp_event` dead for a
> whole second, and a streak of them stopped it for a second each. Everything
> the loop carries was stuck behind it, `GOT_IP` included: **the lease that ends
> the outage was queued behind the sleep taken because the outage had started.**

— `satellite/main/net.c:91-97` (commit `0597d5b`)

The fix is the general one: handlers set state and return; a task elsewhere acts
on the state. Here the retry deadline is a variable the probe task polls
(`wifi_retry_tick()`, `satellite/main/net.c:112-179`).

The same rule applies to the promiscuous callback (§14) and the TX-done callback
— both run on the WiFi driver's own task, where the budget is even tighter.

---

## 12 Buffers and memory

On a chip with a few hundred kilobytes of internal RAM, WiFi buffer
configuration is not a tuning detail. It is the difference between a link that
works and one that intermittently refuses to transmit.

### The pools

The driver keeps several separate pools, and they behave differently.

| Pool | What it holds | Allocated |
|---|---|---|
| **static RX** | every 802.11 frame the hardware receives, before parsing | at `esp_wifi_init()`, freed at deinit. ~1.6 kB each |
| **dynamic RX** | a copy of each *data* frame, handed up to lwIP | on demand, freed when the upper layer is done |
| **static TX** | a copy of each frame to be transmitted | at init. ~1.6 kB each |
| **dynamic TX** | same, sized to the frame | on demand |
| **cache TX** | frames queued in PSRAM when no static TX buffer is free | on demand, PSRAM only |
| **RX mgmt** | management frames | per `ESP_WIFI_RX_MGMT_BUF_NUM_DEF` |
| **mgmt short** | management packets under 64 bytes | dynamic, up to `ESP_WIFI_MGMT_SBUF_NUM` |

The static/dynamic choice is a real trade:

> The default type of buffer in Wi-Fi drivers is "dynamic". Most of the time the
> dynamic buffer can significantly save memory. However, it makes the
> application programming a little more difficult, because in this case the
> application needs to consider memory usage in Wi-Fi.

— `docs/en/api-guides/wifi-driver/wifi-performance-and-power-save.rst:30`

Static buffers are reserved up front and cannot fail. Dynamic buffers share the
heap with your application, so a WiFi burst and an application allocation can
starve each other — and the failure surfaces as `ENOMEM` from `sendto()`, or as
dropped receives, rather than as anything that names WiFi.

### The peak-memory formula

The IDF gives you the arithmetic directly:

> - `b_rx` the number of dynamic RX buffers that are configured
> - `b_tx` the number of dynamic TX buffers that are configured
> - `m_rx` the maximum packet size that the Wi-Fi driver can receive
> - `m_tx` the maximum packet size that the Wi-Fi driver can send
>
> ```
> p = (b_rx * m_rx) + (b_tx * m_tx)
> ```

— `wifi-performance-and-power-save.rst:36-48`

With the defaults — 32 dynamic RX and 32 dynamic TX at up to ~1.6 kB each —
that ceiling is over 100 kB. On an S3 with a large application, it does not fit,
and this project measured exactly that collision:

> What is left has to absorb WiFi's 32 dynamic TX and 32 dynamic RX buffers,
> whose transient ceiling is ~105 kB. It cannot […]
>
> The failure was caught in the act:
>
>     E ALLOCATION FAILED 2 time(s): largest request 1700 B (caps 0x1800 INTERNAL)

— `hub_s3/sdkconfig.defaults:858-866`

### Where WiFi memory comes from

By default, **WiFi buffers must be in internal RAM.** The chain is worth
knowing, because when you see caps `0x1800` in an allocation failure this is
what it means:

```
wifi_malloc()                      ← esp_wifi/esp32s3/esp_adapter.c
   └─ heap_caps_malloc_default()
        └─ ANDs in MALLOC_CAP_INTERNAL, because malloc_alwaysinternal_limit
           is MALLOC_DISABLE_EXTERNAL_ALLOCS
             └─ DEFAULT | INTERNAL = 0x1800
```

Setting `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y` changes `wifi_malloc()` to
`heap_caps_malloc_prefer(size, 2, DEFAULT|SPIRAM, DEFAULT|INTERNAL)` — PSRAM
first, internal retained as fallback. Only WiFi and lwIP move; nothing else in
your application is affected.

The reasoning for doing it here is a good template for the decision:

> WiFi is both the pool that spikes and the one furthest from that loop, which
> is what makes it the right thing to move and the ring the wrong thing.
>
> RISK, and how to falsify it: WiFi buffers now sit behind the PSRAM cache.

— `hub_s3/sdkconfig.defaults:876-885`

**A trap worth knowing about.** Enabling that symbol silently *removes* the
dynamic-TX option from the config system entirely, because of this dependency in
`components/esp_wifi/Kconfig`:

```
depends on !(SPIRAM_TRY_ALLOCATE_WIFI_LWIP && !SPIRAM_IGNORE_NOTFOUND)
```

The build drops to static TX buffers with a default of 16, and any
`CONFIG_ESP_WIFI_DYNAMIC_TX_BUFFER_NUM` line in your `sdkconfig.defaults`
becomes dead text that the build does not warn you about. This repo records that
at `hub_s3/sdkconfig.defaults:386-399` and `:477-479`.

Two other symbols move their defaults when PSRAM allocation is on:
`ESP_WIFI_STATIC_RX_BUFFER_NUM` (10 → 16) and `ESP_WIFI_RX_BA_WIN` (6 → 16).
This project pins both, and the reason is not only about RAM:

> The BA window matters on the air, not just in RAM: the satellite is built
> with 6, and moving only this end changes what the two negotiate.

— `hub_s3/sdkconfig.defaults:895-897`

That is the general point: **`ESP_WIFI_RX_BA_WIN` and `ESP_WIFI_TX_BA_WIN` are
not local settings.** They are offers made on the air, and the two ends settle
on a common value. Change one end and you have changed the link.

### Cache TX buffers, and the honest way to use one

`ESP_WIFI_CACHE_TX_BUFFER_NUM` only exists when PSRAM allocation is on. When a
frame arrives from lwIP and no static TX buffer is free, the driver queues it in
PSRAM instead of rejecting it.

That sounds unambiguously good and is not. A queue does not create airtime; it
converts a *refused* packet into a *late* one. For bulk transfer that is a win.
For anything with a deadline, both outcomes are failures, and the queue has the
additional cost of hiding the evidence:

> WHY A DEEPER QUEUE IS NOT EXPECTED TO HELP, stated so the run can falsify it.
> The pool is 38 buffers against ~130 datagrams/s, i.e. ~292 ms deep, so a
> refusal means the medium was gone for 292 ms PLUS the refusal stretch. Those
> stretches ran p50 63 ms, p90 288 ms, p99 926 ms — so the shortest stall that
> refuses anything is already ~355 ms, against a `LEAD_US` of 350 ms. A packet
> this queue holds through one of them arrives with no lead left and is
> unplayable when it emerges. **The queue converts a refused packet into a late
> one, and both are silence.**

— `hub_s3/sdkconfig.defaults:919-928`

The measurement discipline that goes with it — read `air-gap-max` first,
`tx-fail` last, because absorbing the evidence is exactly what the queue does —
is §20 and §21.

### lwIP's own buffers

lwIP allocates separately, and its queues are just as capable of dropping your
packets. The two that matter most for a UDP application:

| Symbol | Effect |
|---|---|
| `CONFIG_LWIP_UDP_RECVMBOX_SIZE` | how many datagrams may be queued for one socket before the stack drops them. Default 6; **this project uses 32** at both ends (`hub_s3/sdkconfig.defaults:586`, `satellite/sdkconfig.defaults:67`) |
| `CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0/1` | which core the lwIP task runs on. Pinning it away from your latency-critical task is usually right (`hub_s3/sdkconfig.defaults:774`) |
| `CONFIG_LWIP_DHCPS_MAX_STATION_NUM` | how many DHCP leases the SoftAP will issue. **The quiet limit** — exceed it and a station associates and never gets an address |

A burst of packets arriving while your receive task is busy will overflow
`UDP_RECVMBOX_SIZE` and be dropped inside lwIP, with no error anywhere. If you
are losing packets and the radio counters are clean, look here.

---

## 13 Every configuration option

`components/esp_wifi/Kconfig` is 893 lines and about ninety options. Most
projects touch three or four. This section walks all of them by group, saying
what each does and when you would touch it.

Defaults quoted are what `make menuconfig` starts from on an ESP32/S3 target;
some vary by chip via `SOC_*` symbols.

### Buffers

| Symbol | Default | What it does | When to touch it |
|---|---|---|---|
| `ESP_WIFI_STATIC_RX_BUFFER_NUM` | 10 (16 with PSRAM alloc) | hardware RX buffers, ~1.6 kB each, allocated at init | raise for high RX throughput; must be ≥ `RX_BA_WIN` if A-MPDU RX is on |
| `ESP_WIFI_DYNAMIC_RX_BUFFER_NUM` | 32 | copies handed to lwIP; **0 means unlimited** | lower it if RX bursts are exhausting the heap; 0 is dangerous |
| `ESP_WIFI_TX_BUFFER` | dynamic | choice: static or dynamic TX buffers | static for predictability, dynamic to save RAM |
| `ESP_WIFI_STATIC_TX_BUFFER_NUM` | 16 | count, if static | this is the pool depth that decides when `sendto()` returns `ENOMEM` |
| `ESP_WIFI_DYNAMIC_TX_BUFFER_NUM` | 32 | count, if dynamic | **disappears entirely** when `SPIRAM_TRY_ALLOCATE_WIFI_LWIP` is on |
| `ESP_WIFI_CACHE_TX_BUFFER_NUM` | 32 | PSRAM queue for frames that find no static TX buffer | only exists with PSRAM alloc; see §12 for why deeper is not better |
| `ESP_WIFI_MGMT_RX_BUFFER` | static | choice for management RX buffers | static first, to avoid fragmentation |
| `ESP_WIFI_RX_MGMT_BUF_NUM_DEF` | 5 | management RX buffer count, range 1–10 | raise on a busy AP with many joining stations |
| `ESP_WIFI_MGMT_SBUF_NUM` | 32 | dynamic buffers for management packets under 64 bytes, range 6–32 | rarely |

This project sets `STATIC_RX_BUFFER_NUM=10`, `STATIC_TX_BUFFER_NUM=38` and
`CACHE_TX_BUFFER_NUM=16` on the hub. 38 static TX buffers against ~130
datagrams/s is a queue about 292 ms deep — the arithmetic that made the cache
buffer's behaviour predictable.

### Aggregation

| Symbol | Default | What it does |
|---|---|---|
| `ESP_WIFI_AMPDU_TX_ENABLED` | y | aggregate outgoing frames into A-MPDU |
| `ESP_WIFI_TX_BA_WIN` | 6 | outstanding subframes in a TX block-ack exchange, range 2–32 |
| `ESP_WIFI_AMPDU_RX_ENABLED` | y | accept incoming A-MPDU |
| `ESP_WIFI_RX_BA_WIN` | 6 (16 with PSRAM alloc) | RX block-ack window, range 2–32 |
| `ESP_WIFI_AMSDU_TX_ENABLED` | n | aggregate at the MSDU level as well |

The IDF's own advice on the windows is blunt and correct:

> Generally a bigger value means higher throughput but more memory. Most of time
> we should NOT change the default value unless special reason, e.g. test the
> maximum UDP TX throughput with iperf etc.

— `components/esp_wifi/Kconfig:186-190`

**Turning aggregation off is almost never the right move**, because block ack is
also selective retransmission — §6, and §20 for the run that proved it here. If
you need to reduce how many frames one retry chain can hold, reduce the *window*
and keep the mechanism.

### CPU, IRAM and throughput

| Symbol | Default | What it does |
|---|---|---|
| `ESP_WIFI_TASK_CORE_ID` | core 0 | which core the WiFi task is pinned to |
| `ESP_WIFI_IRAM_OPT` | y | place hot WiFi functions in IRAM — "more than 10Kbytes of IRAM memory will be saved" if off, at the cost of throughput |
| `ESP_WIFI_EXTRA_IRAM_OPT` | n (y on Wi-Fi 6 parts) | a further ~5 kB of IRAM |
| `ESP_WIFI_RX_IRAM_OPT` | y | RX path in IRAM — "more than 17Kbytes" if off |
| `ESP_WIFI_SLP_IRAM_OPT` | n (y on Wi-Fi 6 parts) | sleep path in IRAM |

— help text from `Kconfig:273-298`

The three IRAM options are the first thing to turn *off* when you are out of
IRAM, and the throughput cost is real but usually acceptable for a
low-bandwidth application. Note the defaults already flip to `n` on a classic
ESP32 with Bluetooth and PSRAM both enabled — the config system knows that
combination is tight.

`ESP_WIFI_TASK_CORE_ID` matters on dual-core parts. WiFi on core 0 and your
latency-critical work on core 1 (or vice versa) is a cheap and effective
separation.

### Security

| Symbol | Default | What it does |
|---|---|---|
| `ESP_WIFI_ENABLE_WPA3_SAE` | y | WPA3-Personal as a station |
| `ESP_WIFI_ENABLE_SAE_H2E` | y | Hash-to-Element, the faster/safer SAE derivation |
| `ESP_WIFI_ENABLE_SAE_PK` | y | SAE Public Key |
| `ESP_WIFI_SOFTAP_SAE_SUPPORT` | y | WPA3-Personal as a SoftAP |
| `ESP_WIFI_ENABLE_WPA3_OWE_STA` | y | Opportunistic Wireless Encryption — encrypted "open" networks |
| `ESP_WIFI_WPA3_COMPATIBLE_SUPPORT` | y | RSN-override compatibility mode |
| `ESP_WIFI_GCMP_SUPPORT` | n | GCMP-128/256 ciphers (needs `SOC_WIFI_GCMP_SUPPORT`) |
| `ESP_WIFI_GMAC_SUPPORT` | y | GMAC-128/256 integrity |
| `ESP_WIFI_WAPI_PSK` | n | the Chinese WAPI standard |
| `ESP_WIFI_SUITE_B_192` | n | NSA Suite B, 192-bit |
| `ESP_WIFI_ENTERPRISE_SUPPORT` | y | 802.1X / EAP — WPA2-Enterprise |
| `ESP_WIFI_MBEDTLS_CRYPTO` | y | use mbedTLS rather than the supplicant's own crypto |
| `ESP_WIFI_DPP_SUPPORT` | n | Wi-Fi Easy Connect (QR-code provisioning) |
| `ESP_WIFI_WPS_SOFTAP_REGISTRAR` | n | act as a WPS registrar |

If you are only ever going to run a WPA2-PSK SoftAP with known clients, turning
off WPA3, OWE, enterprise and DPP recovers a useful amount of flash and RAM.

There is **no PMF option in Kconfig** — PMF is per-interface, configured through
`wifi_ap_config_t.pmf_cfg` and `esp_wifi_disable_pmf_config()`. §5.

### Power save and sleep

| Symbol | Default | What it does |
|---|---|---|
| `ESP_WIFI_STA_DISCONNECTED_PM_ENABLE` | y | power management while *not* associated |
| `ESP_WIFI_ENHANCED_LIGHT_SLEEP` | n | modem receives beacons autonomously during light sleep |
| `ESP_WIFI_SLP_DEFAULT_MIN_ACTIVE_TIME` | 50 | minimum awake window, ms |
| `ESP_WIFI_SLP_DEFAULT_MAX_ACTIVE_TIME` | 10 | keep-alive ceiling, s |
| `ESP_WIFI_SLP_DEFAULT_WAIT_BROADCAST_DATA_TIME` | 15 | how long to wait for post-DTIM broadcast |
| `ESP_WIFI_SLP_BEACON_LOST_OPT` | — | sleep sooner when a beacon is missed |
| `ESP_WIFI_SLP_BEACON_LOST_TIMEOUT` | 10 | beacon-loss timeout |
| `ESP_WIFI_SLP_BEACON_LOST_THRESHOLD` | 3 | consecutive losses tolerated |
| `ESP_WIFI_SLP_PHY_ON_DELTA_EARLY_TIME` | 2 | wake the PHY this early for a beacon |
| `ESP_WIFI_SLP_PHY_OFF_DELTA_TIMEOUT_TIME` | 2 | keep it on this long after |
| `ESP_WIFI_SLP_SAMPLE_BEACON_FEATURE` | n | learn the AP's actual beacon offset |
| `ESP_WIFI_BSS_MAX_IDLE_SUPPORT` | conditional | BSS max idle period (keep-alive negotiation) |

This whole group is inert unless you use power save. Runtime `esp_wifi_set_ps()`
is the switch; these tune the behaviour once it is on. For a mains-powered,
latency-sensitive device, `WIFI_PS_NONE` makes the entire block irrelevant.

### Features you can usually turn off

| Symbol | Default | What it is |
|---|---|---|
| `ESP_WIFI_SOFTAP_SUPPORT` | y | SoftAP at all. **Turn off on a station-only device** — it saves real RAM and flash |
| `ESP_WIFI_NVS_ENABLED` | y | persist WiFi config in NVS |
| `ESP_WIFI_CSI_ENABLED` | n | Channel State Information — per-subcarrier channel estimates. Costs about `STATIC_RX_BUFFER_NUM` kB |
| `ESP_WIFI_FTM_ENABLE` | n | Fine Timing Measurement — 802.11mc ranging |
| `ESP_WIFI_NAN_SYNC_ENABLE` / `_USD_ENABLE` | n | Wi-Fi Aware |
| `ESP_WIFI_SCAN_CACHE` | n | keep scan results between scans |
| `ESP_WIFI_SOFTAP_BEACON_MAX_LEN` | 752 | beacon length ceiling; only relevant to ESP-MESH root-conflict resolution |
| `ESP_WIFI_PASSIVE_HIDDEN_AP_SUPPORT` | n | find hidden APs by passive scan under auto country policy |

CSI is worth a mention beyond "off by default": it gives you per-subcarrier
amplitude and phase for every received frame, which is the basis of every
WiFi-sensing demo you have seen — presence detection, gesture recognition,
breathing rate. It is a genuinely interesting capability and it is one Kconfig
symbol away.

### Roaming and 802.11k/v/r

| Symbol | Default | What it is |
|---|---|---|
| `ESP_WIFI_11KV_SUPPORT` | n | the umbrella for k and v |
| `ESP_WIFI_RRM_SUPPORT` | y (under the umbrella) | 802.11k Radio Resource Measurement — neighbour reports |
| `ESP_WIFI_WNM_SUPPORT` | y (under the umbrella) | 802.11v Wireless Network Management — BSS transition |
| `ESP_WIFI_11R_SUPPORT` | n | 802.11r Fast BSS Transition — fast roam without a full handshake |
| `ESP_WIFI_MBO_SUPPORT` | n | Multi Band Operation certification |
| `ESP_WIFI_ENABLE_ROAMING_APP` | n | experimental roaming helper (`wifi_apps/roaming_app/`) |

Irrelevant to a single-AP deployment; essential if your device moves across a
multi-AP site.

### Statistics and debug

| Symbol | Default | What it is |
|---|---|---|
| `ESP_WIFI_ENABLE_WIFI_TX_STATS` | n | per-rate TX statistics, readable via `esp_wifi_statis_dump()` |
| `ESP_WIFI_ENABLE_WIFI_RX_STATS` | n | the RX equivalent |
| `ESP_WIFI_ENABLE_WIFI_RX_MU_STATS` | n | MU-MIMO / OFDMA reception stats (Wi-Fi 6 parts) |
| `ESP_WIFI_DEBUG_PRINT` | n | WPA supplicant debug output — verbose, and the right tool for a handshake that fails |
| `ESP_WIFI_MODEM_RF_FLAG_UPDATE_DEBUG` | n | assertions on modem RF flag updates |

The `ENABLE_DUMP_*` family (`HESIGB`, `MU_CFO`, `CTRL_NDPA`, `CTRL_BFRP`) is
Wi-Fi 6 diagnostics and does nothing on a 2.4 GHz-only part.

### ESP-NOW

| Symbol | Default | What it is |
|---|---|---|
| `ESP_WIFI_ESPNOW_MAX_ENCRYPT_NUM` | 7 (2 on C2) | how many encrypted ESP-NOW peers are supported |

That is the only ESP-NOW build option, and it has a consequence that is easy
to miss:

> The number of hardware keys for encryption is fixed. And the espnow and SoftAP
> share the same hardware keys. So this configuration will affect the maximum
> connection number of SoftAP. Maximum espnow encrypted peers number + maximum
> number of connections of SoftAP = Max hardware keys number.

— `components/esp_wifi/Kconfig:572-577`

So on a device that is both a SoftAP and an ESP-NOW peer, every encrypted
ESP-NOW peer costs you one SoftAP client slot. Everything else about ESP-NOW is
runtime (§15).

### The PHY menu

`components/esp_phy/Kconfig`:

| Symbol | Default | What it is |
|---|---|---|
| `ESP_PHY_CALIBRATION_AND_DATA_STORAGE` | y | store calibration in NVS |
| `ESP_PHY_CALIBRATION_MODE` | partial | partial / none / full — see §7 |
| `ESP_PHY_MAX_WIFI_TX_POWER` | 20 | ceiling in dBm for `esp_wifi_set_max_tx_power()` |
| `ESP_PHY_REDUCE_TX_POWER` | — | back off TX power on high-current operations |
| `ESP_PHY_MAC_BB_PD` | — | power down MAC and baseband in light sleep |
| `ESP_PHY_ENABLE_USB` | y when the console is USB | keep the USB PHY on. **On an S3 this costs WiFi performance** — `SOC_WIFI_PHY_NEEDS_USB_WORKAROUND` |
| `ESP_PHY_IMPROVE_RX_11B` | n | better 802.11b reception in poor conditions |
| `ESP_PHY_PLL_TRACK_PERIOD_MS` | — | how often the PLL is retracked against temperature |
| `ESP_PHY_IRAM_OPT` | y | PHY functions in IRAM |
| `ESP_PHY_MULTIPLE_INIT_DATA_BIN` | n | ship several PHY init blobs and select at runtime (country/antenna variants) |

`ESP_PHY_ENABLE_USB` is the one to know about on an S3. It defaults to `y`
whenever the console is USB, which quietly overrides the interference
mitigation. Setting it to `n` reclaims the WiFi performance and costs you the
USB console.

### The coexistence menu

`components/esp_coex/Kconfig`:

| Symbol | What it is |
|---|---|
| `ESP_COEX_SW_COEXIST_ENABLE` | software coexistence arbitration between WiFi and Bluetooth on one radio |
| `ESP_COEX_EXTERNAL_COEXIST_ENABLE` | GPIO-based arbitration with an external radio |
| `ESP_COEX_POWER_MANAGEMENT` | coexistence-aware power management |
| `ESP_COEX_GPIO_DEBUG` | drive coexistence state onto GPIOs for a logic analyser |

§16.

---

## 14 Sniffer mode

Promiscuous mode hands your application every frame the radio hears on the
current channel, whether or not it is addressed to you and whether or not you
are associated. It is the most direct instrument the ESP offers.

```c
esp_err_t esp_wifi_set_promiscuous(bool en);
esp_err_t esp_wifi_set_promiscuous_rx_cb(wifi_promiscuous_cb_t cb);
esp_err_t esp_wifi_set_promiscuous_filter(const wifi_promiscuous_filter_t *filter);
esp_err_t esp_wifi_set_promiscuous_ctrl_filter(const wifi_promiscuous_filter_t *filter);
```

### What you can and cannot see

> If the sniffer mode is enabled, the following packets **can** be dumped to the
> application:
>
>  - 802.11 Management frame.
>  - 802.11 Data frame, including MPDU, AMPDU, and AMSDU.
>  - 802.11 MIMO frame, for MIMO frame, the sniffer only dumps the length of the frame.
>  - 802.11 Control frame.
>  - 802.11 CRC error frame.
>
> The following packets will **NOT** be dumped to the application:
>
>  - Other 802.11 error frames.

— `docs/en/api-guides/wifi-driver/wifi-modes.rst:59-70`

Note what is *not* in that list: **payloads of encrypted frames.** You see the
MAC header, the length, and the radio metadata of a neighbour's traffic. You do
not see what is inside it. For measuring airtime that is more than enough.

Sniffing works in `WIFI_MODE_NULL`, `STA`, `AP` and `APSTA` — so you can sniff
while associated. The IDF warns that it has a "great impact" on throughput while
you do, which is why the survey here does it before the AP starts.

### The filters

```c
#define WIFI_PROMIS_FILTER_MASK_ALL         (0xFFFFFFFF)  /**< Filter all packets */
#define WIFI_PROMIS_FILTER_MASK_MGMT        (1)           /**< Filter the packets with type of WIFI_PKT_MGMT */
#define WIFI_PROMIS_FILTER_MASK_CTRL        (1<<1)        /**< Filter the packets with type of WIFI_PKT_CTRL */
#define WIFI_PROMIS_FILTER_MASK_DATA        (1<<2)        /**< Filter the packets with type of WIFI_PKT_DATA */
#define WIFI_PROMIS_FILTER_MASK_MISC        (1<<3)        /**< Filter the packets with type of WIFI_PKT_MISC */
#define WIFI_PROMIS_FILTER_MASK_DATA_MPDU   (1<<4)        /**< Filter the MPDU which is a kind of WIFI_PKT_DATA */
#define WIFI_PROMIS_FILTER_MASK_DATA_AMPDU  (1<<5)        /**< Filter the AMPDU which is a kind of WIFI_PKT_DATA */
#define WIFI_PROMIS_FILTER_MASK_FCSFAIL     (1<<6)        /**< Filter the FCS failed packets, do not open it in general */
```
— `esp_wifi_types_generic.h:705-712`

"Filter" here means *pass through to the callback*, not *discard* — the naming
is unfortunate. There is a second, finer filter for control-frame subtypes
(`WIFI_PROMIS_CTRL_FILTER_MASK_*`, `:714-723`) if you want to see ACKs and
block-acks specifically.

`WIFI_PROMIS_FILTER_MASK_FCSFAIL` carries the header's own warning — "do not
open it in general" — because a busy channel produces a flood of them. There is
a good reason to open it anyway, and this project states it:

> **FCSFAIL IS ON DELIBERATELY**, against IDF's "do not open it in general". A
> frame that failed FCS still occupied the air, and collisions and marginal
> interferers are precisely what a beacon scan cannot see. The callback is a
> few adds, so the flood is affordable for half a second.

— `hub_s3/main/net.c:118-122`

### The callback contract

> the callback will be called directly in the Wi-Fi driver task, so if the
> application has a lot of work to do for each filtered packet, the
> recommendation is to post an event to the application task in the callback and
> defer the real work to the application task.

— `wifi-modes.rst:76`

On a busy channel this fires thousands of times a second. Do arithmetic on
stack variables and return. This project's callback is about fifteen lines and
allocates nothing (`hub_s3/main/net.c:155-179`).

### Which metadata fields to trust

The `rx_ctrl` struct in §4 is mostly reliable. One field is not, and the
discovery is instructive:

> **NO NOISE FLOOR HERE, and it was tried.** `rx_ctrl.noise_floor` read exactly
> -97 on every frame, every channel, both boots of the 2026-08-24 13:33 capture
> — one unique value in the whole run. It is not populated meaningfully on this
> path, and **a constant that looks like a measurement is worse than no
> measurement.**

— `hub_s3/main/net.c:148-153`

That last sentence is the transferable lesson. A field that always reads the
same is not a floor you can subtract; it is a placeholder that will make your
SNR calculation confidently wrong.

Also note the header's own caveat on `timestamp`: "It is precise only if modem
sleep or light sleep is not enabled."

### Transmitting raw frames

The inverse of sniffing exists too:

```c
esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len, bool en_sys_seq);
```
— `esp_wifi.h:1214`

You supply a complete 802.11 frame. The driver adds the FCS and (optionally) the
sequence number. It refuses to send frames that would impersonate another
device, so it is not a tool for deauth attacks, but it is how you emit custom
management frames or beacons with vendor content.

The gentler version of the same idea is **vendor-specific information
elements** — `esp_wifi_set_vendor_ie()` lets you attach your own data to the
beacons and probe responses your SoftAP already sends, and
`esp_wifi_set_vendor_ie_cb()` lets you receive other devices'. That is a
zero-association broadcast channel, and it is the mechanism ESP-NOW is built on.

---

## 15 ESP-NOW

ESP-NOW is Espressif's connectionless protocol: send a payload straight to
another ESP's MAC address, with no association, no DHCP, no IP stack. It is not
802.11 in the standards sense — it rides inside **vendor-specific action
frames**, which is a management frame type that needs no association to send or
receive.

**This project does not use it.** It is covered here properly anyway, because it
is the obvious alternative to what this project does, and the reasons it was not
chosen are the useful part.

### The API

```c
esp_err_t esp_now_init(void);
esp_err_t esp_now_register_recv_cb(esp_now_recv_cb_t cb);
esp_err_t esp_now_register_send_cb(esp_now_send_cb_t cb);
esp_err_t esp_now_add_peer(const esp_now_peer_info_t *peer);
esp_err_t esp_now_send(const uint8_t *peer_addr, const uint8_t *data, size_t len);
esp_err_t esp_now_set_pmk(const uint8_t *pmk);
esp_err_t esp_now_set_peer_rate_config(const uint8_t *peer_addr, esp_now_rate_config_t *config);
esp_err_t esp_now_set_wake_window(uint16_t window);
```
— `components/esp_wifi/include/esp_now.h:136-373`, the useful subset of 21
functions

A minimal sender is `esp_wifi_init()` + `esp_wifi_start()` + `esp_now_init()` +
`esp_now_add_peer()` + `esp_now_send()`. No SSID, no password, no address.

### The limits, which are the design

```c
#define ESP_NOW_MAX_TOTAL_PEER_NUM   20        /*!< Maximum number of ESPNOW total peers */
#define ESP_NOW_MAX_ENCRYPT_PEER_NUM 6         /*!< Maximum number of ESPNOW encrypted peers */

#define ESP_NOW_MAX_IE_DATA_LEN      250       /**< Maximum data length in a vendor-specific element */
#define ESP_NOW_MAX_DATA_LEN  ESP_NOW_MAX_IE_DATA_LEN   /**< Maximum length of data sent in each ESPNOW transmission for v1.0 */
#define ESP_NOW_MAX_DATA_LEN_V2      1470      /**< Maximum length of data sent in each ESPNOW transmission for v2.0 */
```
— `esp_now.h:32-37`

**250 bytes** is the classic ESP-NOW payload limit, and it comes straight from
the maximum length of a vendor-specific information element. ESP-NOW v2 raises
it to 1470 by fragmenting across several elements. Version skew is a real
concern:

> However, v1.0 devices can receive v2.0 packets if the packet length is less
> than or equal to `ESP_NOW_MAX_IE_DATA_LEN`. For packets exceeding this length,
> the v1.0 devices will either truncate the data to the first
> `ESP_NOW_MAX_IE_DATA_LEN` bytes or discard the packet entirely.

— `esp_now.h:150-151`; check with `esp_now_get_version()`

Twenty peers, of which a handful may be encrypted. The two headline numbers
disagree, and it is worth knowing which one binds: the header defines
`ESP_NOW_MAX_ENCRYPT_PEER_NUM` as 6, but the limit the driver actually enforces
comes from `wifi_init_config_t.espnow_max_encrypt_num`, which
`WIFI_INIT_CONFIG_DEFAULT()` fills from `CONFIG_ESP_WIFI_ESPNOW_MAX_ENCRYPT_NUM`
— default 7, range 0–17 outside the C2. Encryption uses a two-level scheme:
a **PMK** shared device-wide (`esp_now_set_pmk()`) and an **LMK** per peer, set
in `esp_now_peer_info_t`. It is CCMP, not the WPA handshake — there is no key
negotiation, you provision both sides.

### The channel constraint

Peers must be on the same channel, and if the device is also a station, its
channel is dictated by its AP. `esp_now_peer_info_t.channel` set to 0 means "use
the current channel". A pure ESP-NOW device with no association can sit wherever
you tell it via `esp_wifi_set_channel()`, but two ESP-NOW devices that are also
stations on different APs cannot talk to each other at all.

This is the most common ESP-NOW deployment problem: it works on the bench with
two unassociated boards and stops working the moment one of them joins a
network.

### ESP-NOW versus a SoftAP, honestly

| | ESP-NOW | SoftAP + station + UDP |
|---|---|---|
| Join time | none | ~1–3 s (scan, auth, assoc, handshake, DHCP) |
| Payload | 250 B (v1) / 1470 B (v2) | up to the MTU, ~1460 B for UDP |
| Peers | 20, of which `CONFIG_ESP_WIFI_ESPNOW_MAX_ENCRYPT_NUM` may be encrypted | 15 stations on this hardware |
| Addressing | MAC | IP, with DHCP |
| Liveness | you must build it | association state and DHCP lease are free |
| Delivery status | per-send callback with success/fail | nothing at the socket level |
| Latency | lower — no association, no DHCP, less stack | higher, but not by much once joined |
| Power save | `esp_now_set_wake_window()` | the standard PS machinery |
| Talking to a phone or laptop | impossible | trivial |

**Why this project uses the SoftAP path.** The traffic is ~50 + 96×N unicast
datagrams a second of SBC audio plus a timeline, up to 15 satellites. It needs:

- **more than 250 bytes per packet.** An SBC audio chunk is around a kilobyte.
  ESP-NOW v2 would now cover it; v1 would not, and it did not when the decision
  was made.
- **a liveness signal it does not have to invent.** Association and DHCP lease
  give the hub a client list and a departure event for free — the whole of
  `hub_s3/main/clients.c` is built on `IP_EVENT_ASSIGNED_IP_TO_CLIENT` and
  `WIFI_EVENT_AP_STADISCONNECTED`.
- **a real socket**, because the same UDP port also carries clock probes,
  telemetry, analysis frames and the log-forwarding channel in
  `components/dancefloor_sync/wifi_log.c`.
- **TSF**, which requires an association. `esp_wifi_get_tsf_time()` returns 0
  on an interface that has not seen a beacon, and TSF is the primary clock
  source here (§6, §20).

The last one is decisive and is not obvious: **ESP-NOW gives you no shared
clock.** Everything this project does rests on one.

When ESP-NOW *is* the right answer: small telemetry payloads, many-to-one sensor
networks, battery devices that must wake, send and sleep in milliseconds, and
anything where a 1–3 second join is unacceptable.

The IDF example is at `examples/wifi/espnow/main/espnow_example_main.c`, and it
is a good one — it covers broadcast discovery, unicast, encryption and the send
callback in a single readable file.

---

## 16 Coexistence with Bluetooth

WiFi and Bluetooth both live in 2.4 GHz, and on a single ESP32 they share one
radio, one antenna, one memory pool and CPU time. They cannot both transmit.

**Software coexistence** (`ESP_COEX_SW_COEXIST_ENABLE`) is an arbiter that time-
slices between them, prioritising by traffic type. It works, and it costs both
sides. Bluetooth also has its own mitigation — **AFH**, Adaptive Frequency
Hopping, which learns which parts of the band are busy and stops hopping there.

AFH is why this project pins its WiFi channel for the whole session:

> The channel has to be fixed for the session — Bluetooth's AFH routes around a
> known interferer far better than a moving one, and `bt_bridge` carries the
> A2DP source over the same air — but nothing in that argument says it has to
> be fixed at COMPILE time, and this floor changes venue.

— `hub_s3/main/net.c:55-61`

That is a nice distinction: **AFH needs the channel known and stationary, not
known early.** Choosing it at boot and never moving it satisfies both.

### The two-chip decision

This project's master is two chips: a classic ESP32 running Bluetooth A2DP and
nothing else, and an ESP32-S3 running WiFi and the audio pipeline, joined by
SPI. `bt_bridge/sdkconfig.defaults:22` puts it plainly: "No WiFi on this chip at
all — that is the entire point of the split."

The reasons, in order of weight:

1. **Memory.** A2DP sink plus a WiFi SoftAP plus audio buffers does not fit
   comfortably in a classic ESP32's internal RAM.
2. **Radio.** Coexistence arbitration costs both links, and A2DP is a
   continuous isochronous stream with no slack.
3. **CPU.** SBC decode, FFT, onset detection and a WiFi fan-out to 15 clients
   is a lot for two 240 MHz cores that are also servicing a radio.

The constraint that forces the *classic* ESP32 specifically is Bluetooth
Classic: only the original ESP32 has it, and A2DP requires it. The S3, C3 and C6
are BLE-only and cannot ever receive A2DP.

If you are building something that needs both radios on one chip, the honest
expectation is: it works, throughput on both drops, and latency on both becomes
much less predictable. Whether that is acceptable is entirely
application-specific — and it is worth measuring rather than assuming, in both
directions.

---

## 17 What changed in IDF 6.0

Most WiFi tutorials, forum answers and blog posts you will find were written
against IDF 4.x or 5.x. Several will not compile. The full list is at
`docs/en/migration-guides/release-6.x/6.0/wifi.rst`; these are the ones you are
most likely to trip over.

| Was | Now |
|---|---|
| `WIFI_BW_HT20` / `WIFI_BW_HT40` | `WIFI_BW20` / `WIFI_BW40` |
| `ESP_IF_WIFI_STA` / `ESP_IF_WIFI_AP` | `WIFI_IF_STA` / `WIFI_IF_AP` (and `esp_interface.h` is gone) |
| `esp_wifi_set_ant()`, `esp_wifi_set_ant_gpio()` | `esp_phy_set_ant()`, `esp_phy_set_ant_gpio()` |
| `esp_wifi_config_espnow_rate()` | `esp_now_set_peer_rate_config()` |
| `esp_wifi_wps_start(timeout_ms)` | `esp_wifi_wps_start(void)` |
| `esp_supp_dpp_init(callback)` | `esp_supp_dpp_init(void)`; use `WIFI_EVENT_DPP_*` |
| `esp_rrm_send_neighbor_rep_request()` | `esp_rrm_send_neighbor_report_request()` |
| `WIFI_REASON_ASSOC_EXPIRE` | `WIFI_REASON_AUTH_EXPIRE` |
| `WIFI_REASON_NOT_AUTHED` | `WIFI_REASON_CLASS2_FRAME_FROM_NONAUTH_STA` |
| `WIFI_REASON_NOT_ASSOCED` | `WIFI_REASON_CLASS3_FRAME_FROM_NONASSOC_STA` |
| `WIFI_AUTH_WPA3_EXT_PSK` | `WIFI_AUTH_WPA3_PSK` |
| `esp_wifi_init()` twice returned `ESP_OK` | now returns `ESP_ERR_INVALID_STATE` |

Two behavioural changes rather than renames:

- **`pmf_cfg.capable` is deprecated and forced true internally.** You can no
  longer opt out of PMF capability by clearing that field; the only way is
  `esp_wifi_disable_pmf_config()`, and it must sit between
  `esp_wifi_set_config()` and `esp_wifi_start()`. §5.
- **`wifi.rst` was split.** The old monolithic
  `docs/en/api-guides/wifi.rst` is now a directory,
  `docs/en/api-guides/wifi-driver/`, with `overview.rst`,
  `station-scenarios.rst`, `wifi-modes.rst`, `wifi-mac-protocols.rst`,
  `wifi-performance-and-power-save.rst`, `security-and-roaming.rst` and
  `wifi-vendor-features.rst`. Any link you have to the old path is dead.

Also new in 6.0: `examples/wifi/wifi_nvs_config`, and NAN split into
synchronised (`esp_wifi_nan_sync_start()`) and unsynchronised-discovery roles.

---
---
# Part III — Reading this system

Everything in Parts I and II is general. This part is one system, read closely,
because a concept you have seen fail is a concept you keep.

---

## 18 Power-on to first audio packet

### The hub

```
  streamer.c:50    nvs_flash_init()                    PHY calibration store
                        │
                   allocation-failure hook, audio ring
                        │
  net.c:400        wifi_start_ap()
                     ├─ esp_netif_init()
                     ├─ esp_event_loop_create_default()
                     ├─ esp_netif_create_default_wifi_ap()      → s_ap_netif
                     ├─ register WIFI_EVENT_AP_STADISCONNECTED
                     ├─ register IP_EVENT_ASSIGNED_IP_TO_CLIENT
                     ├─ esp_wifi_init(WIFI_INIT_CONFIG_DEFAULT())
                     │
                     ├─ survey_channel()               ~7.4 s, marker blinking
                     │     ├─ esp_wifi_set_mode(STA); esp_wifi_start()
                     │     ├─ passive scan, 13 channels × 360 ms
                     │     ├─ occupancy_survey(): 3 rounds × 3 channels × 300 ms
                     │     └─ esp_wifi_stop()
                     │
                     ├─ wifi_config_t: SSID, WPA2-PSK, ch, dtim 1, beacon 100 TU
                     ├─ esp_wifi_set_mode(AP); esp_wifi_set_config(WIFI_IF_AP)
                     ├─ esp_wifi_get_config()          read-back: what was KEPT
                     ├─ esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW20)
                     ├─ esp_wifi_disable_pmf_config(WIFI_IF_AP)
                     ├─ esp_wifi_start()
                     ├─ esp_wifi_set_ps(WIFI_PS_NONE)
                     └─ esp_wifi_set_tx_done_cb(tx_done_cb)
                        │
  net.c              socket_start()                    one UDP socket, SYNC_PORT
```

Three details in that sequence are worth pulling out.

**The survey runs after `esp_wifi_init()` and before the AP config**, because it
needs a driver to scan with and its answer is what `wc.ap.channel` gets. It
switches the radio to STA mode, scans, and switches back — without ever creating
a STA netif, because a scan wants no IP stack.

**The config is read back.** `esp_wifi_set_config()` returning `ESP_OK` means
IDF did not reject your struct on the way in. It does not mean the driver kept
your values:

> It was added while that one still said 50, and on its first boot it printed
> `beacon_interval 100 TU (asked 50)` — the driver had been quietly discarding
> the value for as long as it had been written, because `ESP_ERROR_CHECK`
> passing says only that IDF does not validate the field on the way in, not
> that the driver kept it.

— `hub_s3/main/net.c:504-509`

That is a habit worth stealing wholesale. **Read back anything you set that
matters.**

**The ordering constraints are all real.** `set_bandwidth` after `set_config`
because `set_config` resets it. `disable_pmf_config` between `set_config` and
`start`. `set_ps` and `set_tx_done_cb` after `start` — the driver rejects the
latter before that.

### The satellite

```
  main.c:148       nvs_flash_init()
                        │
  net.c:308        wifi_start_sta()
                     ├─ esp_netif_init(), event loop, create_default_wifi_sta()
                     ├─ esp_wifi_init(WIFI_INIT_CONFIG_DEFAULT())
                     ├─ wifi_config_t: SSID, password
                     ├─ register WIFI_EVENT / ESP_EVENT_ANY_ID
                     ├─ register IP_EVENT_STA_GOT_IP
                     ├─ esp_wifi_set_mode(STA); esp_wifi_set_config(WIFI_IF_STA)
                     ├─ esp_wifi_disable_pmf_config(WIFI_IF_STA)
                     ├─ esp_wifi_start()
                     └─ esp_wifi_set_ps(WIFI_PS_NONE)
                        │
                   ── WIFI_EVENT_STA_START ──▶  esp_wifi_connect(), arm retry
                        │
                   ── WIFI_EVENT_STA_CONNECTED ──▶  s_assoc = true
                        │                            arm the 10 s lease watchdog
                        │
                   ── IP_EVENT_STA_GOT_IP ──▶  s_link_up = true
                        │                       marker LED solid
                        │
                   socket_start()               bind SYNC_PORT
                        │
                   probes to 192.168.4.1 ──▶ hub adds it to the send list
                        │
                   MSG_TSF arrives ──▶ TSF becomes the clock source
```

The satellite's marker LED is a three-level readout of exactly this sequence —
dark for "not on the AP", solid for "have a lease, no audio yet", flashing for
"playing". From across a dark field that is the entire diagnostic.

Note where audio actually starts: **the satellite is not sent anything until it
probes.** Its clock probes are what put it on the hub's send list, which means
the liveness signal and the sync mechanism are the same traffic. That is a good
pattern — one thing to go wrong instead of two.

---

## 19 The channel survey, read as a worked example

`hub_s3/main/net.c:53-393`, active when `CONFIG_DANCEFLOOR_WIFI_CHANNEL == 0`.
It is about 340 lines including comments, and it exercises passive scanning,
promiscuous mode, `rx_ctrl` decoding, linear-power arithmetic and a genuinely
interesting statistical decision. If you read only one thing in this repository
to learn the ESP WiFi API, read this.

### The problem

A SoftAP has to sit on a channel. `wifi_ap_config_t.channel = 0` means
"auto", and auto means the lowest available, not the best. A fixed compile-time
constant is worse than it looks: the constant here was 11, and it was the
Kconfig default, never a measurement.

### Measurement one: who is out there

A passive scan across **all thirteen channels**, not just the three candidates:

```c
const wifi_scan_config_t sc = {
    .show_hidden = true,
    .scan_type = WIFI_SCAN_TYPE_PASSIVE,
    .scan_time = { .passive = WIFI_PASSIVE_SCAN_DEFAULT_TIME },
};
esp_err_t err = esp_wifi_scan_start(&sc, true);
```
— `hub_s3/main/net.c:276-280`

All thirteen because of §1: a network on channel 5 damages 1 and 6 and appears
on neither. Passive because this unit is about to be the AP in this room and has
no business probing. 360 ms per channel because a beacon interval is ~102 ms and
one interval is a coin toss.

The final `true` makes the call blocking — about 4.7 seconds, which is why the
LED blinks (`visualiser_marker_busy(true)` at `net.c:256`; from outside the
board, 7.4 seconds of no AP and no audio looks exactly like a hang).

Results are drained one at a time, and the reason is a memory constraint worth
noting:

```c
wifi_ap_record_t rec;
while (esp_wifi_scan_get_ap_record(&rec) == ESP_OK) {
    seen++;
    for (int k = 0; k < 3; k++) {
        if (abs((int)rec.primary - cand[k]) <= CHANNEL_OVERLAP) {
            power[k] += powf(10.0f, rec.rssi / 10.0f);
            nets[k]++;
        }
    }
}
```
— `hub_s3/main/net.c:289-300`

> One record at a time, which frees each as it goes — the bulk call would want
> an array sized for a band this unit has not seen yet, and internal RAM here
> runs at ~12 kB free. Draining to `ESP_FAIL` is also what releases the list, so
> there is nothing left to clear.

— `net.c:285-288`

Two things counted per candidate: **how many** networks overlap it, and their
**summed linear power**. The linear conversion is §1's arithmetic.

### Measurement two: how busy it actually is

A beacon says a network exists. It says nothing about whether it is busy, and
ten idle neighbours can cost less than one saturated one. The second measurement
uses promiscuous mode to count **airtime**:

```c
static uint32_t occupancy_dwell(int channel, uint32_t *frames)
{
    s_occ_busy_us = 0;
    s_occ_frames = 0;
    esp_wifi_set_channel((uint8_t)channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(true);
    vTaskDelay(pdMS_TO_TICKS(OCCUPANCY_DWELL_MS));
    esp_wifi_set_promiscuous(false);
    *frames += s_occ_frames;
    return s_occ_busy_us / OCCUPANCY_DWELL_MS;     /* us / ms = permille */
}
```
— `hub_s3/main/net.c:182-192`

Microseconds divided by milliseconds is parts per thousand — a neat unit trick
that avoids any floating point in the hot path.

The callback is §2's airtime formula (`net.c:155-179`), and it is deliberately
tiny because it runs on the WiFi driver task on every frame.

One property is easy to miss and is the reason this measurement is better than
the scan: **the radio's receive bandwidth is about 20 MHz, so a dwell on channel
1 natively hears channel 5's traffic.** The overlap problem that the 13-channel
scan has to model from channel-number distance, this measures directly.

### Maximum, not mean, and why

```c
for (int round = 0; round < OCCUPANCY_ROUNDS; round++) {
    for (int k = 0; k < 3; k++) {
        const uint32_t b = occupancy_dwell(cand[k], &frames[k]);
        if (b > busy_max[k]) {
            busy_max[k] = b;
        }
    }
}
```
— `hub_s3/main/net.c:230-237`

Two decisions in six lines.

**Interleaved**, so a passing burst is not charged entirely to whichever channel
happened to be under the receiver.

**Maximum, not mean.** The argument is the best sentence in the file:

> A mean answers "how busy is this channel typically"; what ruins audio is how
> bad it gets, and a channel quiet 90% of the time and saturated for the other
> 10% is exactly the one to avoid. The maximum is the statistic that says so.

— `net.c:196-200`

And the data that forced it — two deliberate reboots 57 seconds apart in the
same room, with a single 500 ms dwell per channel:

```
             boot 1    boot 2
  ch1-busy      54        62     stable
  ch6-busy      57       196     3.4x, and 16 frames against 144
  ch11-busy     75        70     stable
  chose          6        11     flipped
```
— `net.c:207-213`

The finding is not that the reading was noisy. It is that **the variance itself
is the signal**: ch1 and ch11 held still while ch6 moved 3.4×, which identifies
ch6 as the bursty one.

### The ranking, and the reversal

The obvious design ranks by occupancy, since occupancy is the better question.
This code does the opposite, and the comment explains why:

> **WHY THE REVERSAL.** Occupancy is the better question and the worse
> measurement. […] the harmful traffic is intermittent — a download on a nearby
> machine, demonstrated by ear — and no sample taken at boot can see traffic
> that starts an hour later.
>
> Network count is the signal that has held still. ch11 reads 1-2 networks
> across every soak on file, on two different antennas and on `capture.py`'s
> independent sweep as well, while ch1 and ch6 read 6-7. A channel with one
> neighbour is the better bet for the next four hours than one with seven,
> whatever half a second of dwell happened to catch.

— `net.c:322-344`

So **count decides, occupancy vetoes**:

```c
#define OCCUPANCY_VETO_PERMILLE 150

int best = 0;
for (int k = 1; k < 3; k++) {
    if (nets[k] < nets[best]) {
        best = k;
    }
}
if (busy[best] >= OCCUPANCY_VETO_PERMILLE) {
    for (int k = 0; k < 3; k++) {
        if (busy[k] < busy[best]) {
            ...
            best = k;
        }
    }
}
```
— `hub_s3/main/net.c:346-364`

150‰ is well above the 54–75‰ the quiet channels idle at, so the veto fires on
congestion rather than on sampling noise.

The transferable idea: **a prediction that has held still beats a measurement
that has not, and the measurement's job is to overrule the prediction only when
it is unambiguous.**

### The output is a wire format

Two log lines, in `key value` pairs, so `tools/soak/capture.py` lands every
figure in `metrics.csv` without a parser change (`net.c:366-389`). The occupancy
figures went on a *second* line rather than extending the first, because the
first is a format three soaks of logs already parse.

That is a small discipline with a large payoff: **the log is an interface.**
§21.

---

## 20 Eight things that broke

Each of these is a general concept with a specific bill attached.

### 1. HT40 — paying for width you cannot use

**Concept:** §2. HT40 doubles the rate and doubles the occupied spectrum.

**What happened:** the driver negotiated HT40 by default. On channel 11 the
secondary landed on channel 7, so the AP occupied most of the 2.4 GHz band. The
traffic is ~135 small datagrams a second per satellite — limited by transmit
opportunities, not by bits per symbol — so none of the width was usable.

**Fix:** `esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW20)` after `set_config`
(`hub_s3/main/net.c:559`).

**Lesson:** HT40 on 2.4 GHz is only worth it if your bottleneck is bits per
symbol. For small-packet traffic it is pure interference cost.

### 2. PMF and reason 209 — a security feature disconnecting your own devices

**Concept:** §5. PMF's SA Query tears down an association whose station does not
answer.

**What happened:** the AP started an SA Query, the satellite failed to answer
six attempts, and the AP disassociated it with reason 209 — 1.7 s off the
network, twice in the first 65 seconds. **The satellite counted zero
disconnects while the hub counted two.**

**Fix:** `esp_wifi_disable_pmf_config()` on both ends, behind a Kconfig switch,
with the trade written down (`net.c:563-588`, `satellite/main/net.c:334-341`).

**Lesson:** two of them. A trade-off recorded is a trade-off you can revisit.
And when two ends disagree about how many disconnects happened, **the one that
noticed is right** — which is why the hub's `sta-left` counter exists at all.

### 3. A-MPDU off — removing the cure with the symptom

**Concept:** §6. Block ack is throughput *and* selective retransmission.

**What happened:** aggregation was disabled to reduce buffer holding. It worked
on its own terms — worst refusal stretch 1313 → 486 ms, satellite underruns
roughly halved — and made everything else worse: `gaps` more than tripled
(38/38 → 167/112), and the project logged its first hub-side underrun.

**Fix:** back on, with the measurements recorded and a gate on reopening it
(`hub_s3/sdkconfig.defaults:185-215`).

**Lesson:** "the mechanism was right and the instrument was too blunt". When a
mitigation removes two behaviours and you only wanted one gone, look for the
knob that separates them.

### 4. The BA window — the knob that separates them

**Concept:** §6, §12. The window bounds how many subframes one retry chain
holds, without removing block ack.

**What happened:** with aggregation back on, `air-gap-max` and `txdone` (§20.6)
showed frames being *held and released in bursts* rather than lost — 505 ms
median air-gap in refusing windows against 141 ms in clean ones, with 0
`txdone-fail` in 3,003,948 frames. That is a drain, and the window bounds it.
The window went to 2.

**Then it went back to 6**, and this is the part worth learning from:

> BACK AT 6, AND PARKED — not withdrawn. The antenna was changed on 2026-08-24
> before this had been soaked, and **two variables in one run answer neither.**
> It returns to 6 so the antenna is the only thing that moved against the 00:04
> soak; the case for 2 is below and stands until a run refutes it.

— `hub_s3/sdkconfig.defaults:217-221`

**Lesson:** the discipline of one variable per run, applied to a change you
already believe in. Note also the last line of that block: the satellite's
`RX_BA_WIN` stays at 6 deliberately, because the two ends negotiate the minimum
and this sets the window from the transmitting side.

### 5. ENOMEM and the cache TX buffer — a fix that hides its own evidence

**Concept:** §12. A PSRAM queue in front of the static TX pool.

**What happened:** 1,180 audio refusals in 7.19 h, every one `ENOMEM`. Nothing
on the hub predicted them — pivoting all 7,696 status windows, ring depth,
lead, phase, RSSI and churn read the same in refusing windows as in clean ones.

The arithmetic said a deeper queue would not help: 38 buffers against ~130
datagrams/s is ~292 ms of depth, so a refusal already means the medium was gone
for 292 ms *plus* the refusal stretch, and those ran to p99 926 ms — against a
350 ms audio lead.

**What was done:** the queue was enabled at 16 (~123 ms of added delay, inside
what the lead can absorb) explicitly **as a measurement, not as a fix**, with
the reading order stated in advance: `air-gap-max` first, the satellites' own
figures second, `tx-fail` last "and only to confirm it fell. On its own it now
means nothing: absorbing the evidence is exactly what the queue does."

— `hub_s3/sdkconfig.defaults:900-946`

**Lesson:** before you add a queue, know which of your instruments it blinds.

### 6. `tx_done_cb` — measuring what the radio did, not what `sendto()` accepted

**Concept:** §9's private APIs, and the general problem that every signal you
have is stamped at the wrong end of the pipeline.

**What happened:** every hub-side stall signal was stamped at `sendto()` return
— `tx-fail` counts a refusal, `fanout-gap-max` measures the gap between accepted
sends, `lead-min` is computed as the packet is built. Turn the cache queue on
and all three go quiet whether or not the air improved.

**Fix:** register a callback that fires when a frame actually leaves the radio.
The gap between callbacks is the stall itself, measured past every queue in
front of it:

```c
static void tx_done_cb(uint8_t ifidx, uint8_t *data, uint16_t *data_len, bool txStatus)
{
    (void)ifidx; (void)data; (void)data_len;
    const int64_t now = esp_timer_get_time();
    n_txdone++;
    if (!txStatus) {
        n_txdone_fail++;
    }
    if (s_txdone_prev_at) {
        const int64_t gap = now - s_txdone_prev_at;
        if (gap > n_air_gap_max_us) {
            n_air_gap_max_us = (int32_t)(gap > INT32_MAX ? INT32_MAX : gap);
        }
    }
    s_txdone_prev_at = now;
}
```
— `hub_s3/main/net.c:848-863`, registered at `net.c:601`

`txStatus` is false when the frame was transmitted and never acknowledged —
retries exhausted, i.e. the air. So the two counters together separate two
causes:

```
   air-gap large, txdone-fail rising ... the medium. Frames went and died.
   air-gap large, txdone-fail flat .... frames are NOT being lost.
```

And the comment is honest about what it still cannot separate — a driver that
never dequeues looks the same as a busy medium deferring you in CCA
(`net.c:830-838`).

**Lesson:** when your instruments are all on one side of a queue, get one on the
other side, even if it means a private API.

### 7. The DHCP lease — associated is not connected

**Concept:** §5, §11.

**What happened:** a satellite associated, DHCP never completed, and no
`STA_DISCONNECTED` followed because as far as the radio was concerned everything
was fine. The unit sat on the AP with no address, its probes went nowhere, and
**nothing in the firmware was unhappy enough to try anything. Only a reboot
ended it.**

**Fix:** a third state and a 10-second watchdog whose action is
`esp_wifi_disconnect()` — deliberately dropping into the reconnect path that
already exists rather than inventing a second one
(`satellite/main/net.c:112-179`).

The 10 seconds is chosen against lwIP's behaviour rather than against patience:
the DHCP client retransmits on a doubling backoff, so it is roughly three
attempts.

**Lesson:** any state your system can enter and not leave needs a timeout, and
the timeout's action should be to re-enter a state you already handle.

### 8. TSF — the clock that was there all along

**Concept:** §6.

**What happened:** clock synchronisation was built on round-trip probes, whose
error floor is path asymmetry — everything between "read the clock" and "the
frame left" lands in the budget. TSF has no round trip: both ends slave to the
same AP counter in MAC hardware.

Measured, the two agree within ~450 µs with a stable bias, and TSF's
sample-to-sample step is 1–80 µs against the estimator's 50–200 µs
(`satellite/main/clock.c:90-95`).

**Fix:** prefer TSF when a reading is less than a second old, keep the estimator
as the fallback for when TSF reads 0 — not associated, no beacon yet, or a hub
that does not send `MSG_TSF` (`clock.c:100-107`).

**Lesson:** for anything that needs a shared clock across associated devices,
`esp_wifi_get_tsf_time()` is free, hardware-timestamped and almost nobody uses
it. Read it twice around your local clock read so the sample carries its own
error bar (`satellite/main/rx.c:971-975`).

---

## 21 How to measure your own link

The recurring finding across this entire project, stated once:

> **Every real fault was invisible until something counted it.** Several were
> actively disguised as something else. If you extend this, add the counter
> before you form the theory.

### What the driver will tell you

| Call | Gives you |
|---|---|
| `esp_wifi_ap_get_sta_list(&list)` | every associated station: MAC, RSSI, and `phy_11b/g/n` flags |
| `esp_wifi_sta_get_ap_info(&rec)` | a `wifi_ap_record_t` for the AP you are joined to — including **its** RSSI as you hear it |
| `esp_wifi_statis_dump(modules)` | the driver's internal statistics (`esp_wifi.h:1358`); needs `ESP_WIFI_ENABLE_WIFI_TX_STATS` / `_RX_STATS` |
| `esp_wifi_get_tsf_time(ifx)` | the BSS clock, hardware-timestamped |
| `esp_wifi_set_tx_done_cb(cb)` | per-frame completion and ack status (private API) |
| promiscuous mode | everything else |

**RSSI is directional, and both directions matter.** The hub's
`esp_wifi_ap_get_sta_list()` gives the signal it *hears from* each satellite —
the uplink. It says nothing about the downlink. This project added the other
half specifically because a run could not be decided without it:

> The hub prints `rssi-min`, and that is `esp_wifi_ap_get_sta_list()` — the
> signal it HEARS FROM us. It says nothing about the signal we hear from IT,
> and the two are only equal if both ends are healthy. Every soak on file has
> therefore measured one direction and assumed the other.
>
> Read as a PAIR against the hub's `rssi-min` for the same minute. Antenna gain
> is reciprocal, so a healthy link reads roughly symmetric; a persistent gap
> between the two is a fault in one end's transmit chain rather than in the air
> between them.

— `satellite/main/telemetry.c:281-299`

The `phy_11n` flag in the station list is worth watching too: a station that
associated without 11n gets no aggregation, which means a transmit queue that
drains one frame at a time (`hub_s3/main/servo.c:193-210`).

### Reading the driver's own log lines

The WiFi driver logs at `I` level with a `wifi:` prefix, and the shorthand is
undocumented. The two you will see most:

| Line | Means |
|---|---|
| `wifi:new:<11,2>` | now operating on primary channel 11, secondary code 2 — i.e. **HT40 with the secondary below**. `<11,0>` is HT20 |
| `wifi:station: xx:xx:… join, AID=1, bgn, 40D` | a station joined; it supports 802.11b, g and n; `40D` means HT40 with the secondary below (`20` would be HT20) |
| `wifi:<ba-add>idx:0, ifx:1, tid:0, TAHI:…` | a block-ack session was established |

`<11,2>` and `40D` are how the HT40 problem in §20.1 was spotted.

### Instrument design, as practised here

Four habits from this repo that are worth stealing:

**1. Count the shape, not just the size.** A minimum or a mean throws away the
distribution. When ENOMEM refusals needed characterising, a single number could
not distinguish "continuous refusal" from "refusal at the beacon rate", so the
instrument became a four-bucket histogram of gaps between refusal stretches,
placed around the 102.4 ms DTIM hold:

```
   <25 ms     back-to-back: the pool is refusing continuously
   25-75      sub-beacon
   75-150     THE BEACON. A pile here is the DTIM hold, stated directly.
   >150       multi-beacon: a release came round and did not clear the backlog
```
— `hub_s3/main/net.c:763-767`

> Read the buckets against each other, not the absolute counts: `gaps 2/1/44/3`
> is a beacon-locked queue, `gaps 61/0/0/0` is a stall that never lets go.

**2. Distinguish "not measured" from "zero".** A failed read that prints `0`
becomes a data point that never existed. Both firmwares print text where a
number could be misread:

```c
if (esp_wifi_ap_get_sta_list(&sta) != ESP_OK) {
    sta.num = -1;   /* says "not measured", which is not "none joined" */
}
```
— `hub_s3/main/servo.c:202-203`; the satellite does the same with `"none"` for a
failed RSSI read (`telemetry.c:306-310`).

**3. Say *why* a failure happened, not just how often.** A bare count of failed
sends cannot distinguish two faults with completely different fixes:

> `ENOMEM` — the WiFi driver is out of TX buffers. A load or memory problem […]
> `EHOSTUNREACH` — lwIP has no ARP entry and the pending-ARP queue is dropping
> its overflow. A JOIN problem […]
>
> Guessing between them from a bare count is how an evening gets spent on the
> wrong one.

— `hub_s3/main/net.c:666-672`

**4. Make the log a wire format.** Hyphenated `key value` pairs, so a script can
parse them into a CSV without a bespoke parser, and so adding a figure does not
break three soaks' worth of existing logs. `tools/soak/capture.py` reads them;
`tools/soak/analyse.py` pivots them.

### Cross-check against something that is not your device

`tools/soak/capture.py:355-430` runs an independent 2.4 GHz sweep from the
laptop via `nmcli`, scored the same way as the hub's own survey. When the two
disagreed — the hub picking a channel its own scan counted ten networks on,
while the laptop read six there and one on channel 11 — that disagreement was
the finding that led to the occupancy measurement in §19.

**A measurement your device makes about itself cannot detect its own systematic
error.** A second, independent instrument can.

---

## 22 Appendices

### A. Disconnect reason codes

From `components/esp_wifi/include/esp_wifi_types_generic.h:113-173`. Codes 1–68
are 802.11's; 200 and above are Espressif's, describing failures that never
reached the air.

| Code | Name | Usually means |
|---|---|---|
| 1 | `UNSPECIFIED` | |
| 2 | `AUTH_EXPIRE` | the AP aged out your authentication |
| 3 | `AUTH_LEAVE` | deauthenticated because the peer is leaving |
| 4 | `DISASSOC_DUE_TO_INACTIVITY` | you stopped transmitting. Send keepalives |
| 5 | `ASSOC_TOOMANY` | the AP is at `max_connection` |
| 6 | `CLASS2_FRAME_FROM_NONAUTH_STA` | you sent a frame the AP thinks you are not authenticated for |
| 7 | `CLASS3_FRAME_FROM_NONASSOC_STA` | ditto, association |
| 8 | `ASSOC_LEAVE` | disassociated because the peer is leaving |
| 13 | `IE_INVALID` | malformed information element |
| 14 | `MIC_FAILURE` | integrity check failed — key mismatch or attack |
| 15 | `4WAY_HANDSHAKE_TIMEOUT` | **usually the wrong password** |
| 16 | `GROUP_KEY_UPDATE_TIMEOUT` | GTK rekey failed |
| 17 | `IE_IN_4WAY_DIFFERS` | security parameters changed mid-handshake |
| 18–24 | cipher/AKM invalid, RSN IE problems | security configuration mismatch |
| 34 | `MISSING_ACKS` | |
| 39 | `TIMEOUT` | |
| 46/47 | `PEER_INITIATED` / `AP_INITIATED` | the other end chose to |
| 200 | `BEACON_TIMEOUT` | you stopped hearing the AP |
| 201 | `NO_AP_FOUND` | the SSID was not seen at all |
| 202 | `AUTH_FAIL` | |
| 203 | `ASSOC_FAIL` | |
| 204 | `HANDSHAKE_TIMEOUT` | **also usually the wrong password** |
| 205 | `CONNECTION_FAIL` | |
| 206 | `AP_TSF_RESET` | the AP's clock restarted — it rebooted |
| 207 | `ROAMING` | |
| 208 | `ASSOC_COMEBACK_TIME_TOO_LONG` | |
| **209** | **`SA_QUERY_TIMEOUT`** | **PMF. §5 and §20.2** |
| 210 | `NO_AP_FOUND_W_COMPATIBLE_SECURITY` | the AP exists but your auth mode does not match |
| 211 | `NO_AP_FOUND_IN_AUTHMODE_THRESHOLD` | your `threshold.authmode` excluded it |
| 212 | `NO_AP_FOUND_IN_RSSI_THRESHOLD` | your `threshold.rssi` excluded it |

### B. Rate table and airtime

Airtime is for the payload only; add roughly 50 µs of per-frame overhead (§3).

| Mode | Rate | 100 B | 1500 B |
|---|---|---|---|
| 11b | 1 Mbps | 800 µs | 12000 µs |
| 11b | 11 Mbps | 73 µs | 1091 µs |
| 11g | 6 Mbps | 133 µs | 2000 µs |
| 11g | 24 Mbps | 33 µs | 500 µs |
| 11g | 54 Mbps | 15 µs | 222 µs |
| 11n MCS0 | 6.5 Mbps | 123 µs | 1846 µs |
| 11n MCS3 | 26 Mbps | 31 µs | 462 µs |
| 11n MCS7 | 65 Mbps | 12 µs | 185 µs |
| 11n MCS7 HT40 SGI | 150 Mbps | 5 µs | 80 µs |

The two tables the hub uses to do this at runtime are at
`hub_s3/main/net.c:138-146`. Note the legacy table's index order: it is the
`wifi_phy_rate_t` enum's order, not ascending — `WIFI_PHY_RATE_1M_L` is `0x00`
and `_9M` is `0x0F`.

### C. Public API index

**Lifecycle** — `esp_wifi_init`, `esp_wifi_deinit`, `esp_wifi_start`,
`esp_wifi_stop`, `esp_wifi_restore`

**Mode and config** — `esp_wifi_set_mode`, `esp_wifi_get_mode`,
`esp_wifi_set_config`, `esp_wifi_get_config`, `esp_wifi_set_storage`

**Station** — `esp_wifi_connect`, `esp_wifi_disconnect`,
`esp_wifi_sta_get_ap_info`, `esp_wifi_sta_get_rssi`,
`esp_wifi_set_rssi_threshold`

**SoftAP** — `esp_wifi_ap_get_sta_list`, `esp_wifi_ap_get_sta_aid`,
`esp_wifi_deauth_sta`

**Scanning** — `esp_wifi_scan_start`, `esp_wifi_scan_stop`,
`esp_wifi_scan_get_ap_num`, `esp_wifi_scan_get_ap_records`,
`esp_wifi_scan_get_ap_record`, `esp_wifi_clear_ap_list`,
`esp_wifi_set_scan_parameters`

**Radio** — `esp_wifi_set_channel`, `esp_wifi_get_channel`,
`esp_wifi_set_bandwidth`, `esp_wifi_set_bandwidths`, `esp_wifi_set_protocol`,
`esp_wifi_set_protocols`, `esp_wifi_set_max_tx_power`,
`esp_wifi_get_max_tx_power`, `esp_wifi_set_country`, `esp_wifi_set_country_code`

**Power** — `esp_wifi_set_ps`, `esp_wifi_get_ps`,
`esp_wifi_set_inactive_time`, `esp_wifi_get_inactive_time`

**MAC and time** — `esp_wifi_set_mac`, `esp_wifi_get_mac`,
`esp_wifi_get_tsf_time`

**Sniffing and raw frames** — `esp_wifi_set_promiscuous`,
`esp_wifi_set_promiscuous_rx_cb`, `esp_wifi_set_promiscuous_filter`,
`esp_wifi_set_promiscuous_ctrl_filter`, `esp_wifi_80211_tx`,
`esp_wifi_register_80211_tx_cb`, `esp_wifi_set_vendor_ie`,
`esp_wifi_set_vendor_ie_cb`

**CSI** — `esp_wifi_set_csi`, `esp_wifi_set_csi_config`,
`esp_wifi_set_csi_rx_cb`

**Security** — `esp_wifi_disable_pmf_config`. WPA2-Enterprise moved out of
`esp_wifi` entirely: it is `esp_eap_client_*` in
`components/wpa_supplicant/esp_supplicant/include/esp_eap_client.h`, alongside
`esp_wps.h`, `esp_dpp.h`, `esp_rrm.h` and `esp_wnm.h`

**Diagnostics** — `esp_wifi_statis_dump`

**ESP-NOW** — `esp_now_init`, `esp_now_deinit`, `esp_now_get_version`,
`esp_now_register_recv_cb`, `esp_now_register_send_cb`,
`esp_now_unregister_recv_cb`, `esp_now_unregister_send_cb`, `esp_now_send`,
`esp_now_add_peer`, `esp_now_del_peer`, `esp_now_mod_peer`, `esp_now_get_peer`,
`esp_now_fetch_peer`, `esp_now_is_peer_exist`, `esp_now_get_peer_num`,
`esp_now_set_pmk`, `esp_now_set_peer_rate_config`, `esp_now_set_wake_window`,
`esp_now_set_user_oui`, `esp_now_get_user_oui`

Full signatures and doc comments: `components/esp_wifi/include/esp_wifi.h` (89
functions) and `esp_now.h` (21).

### D. Glossary

| Term | |
|---|---|
| **AID** | Association ID. The AP's handle for one associated station |
| **A-MPDU** | Aggregated MAC Protocol Data Unit. Several complete frames in one transmission, individually acknowledged |
| **A-MSDU** | Aggregated MAC Service Data Unit. Several payloads in one frame |
| **BSS / BSSID** | Basic Service Set — one AP and its stations. The BSSID is normally the AP's MAC |
| **CCA** | Clear Channel Assessment. The physical half of carrier sense |
| **CCMP** | The AES-based cipher WPA2 uses |
| **CSI** | Channel State Information. Per-subcarrier channel estimate |
| **DIFS / SIFS** | Inter-frame spaces. SIFS is shorter, which is what gives ACKs priority |
| **DTIM** | Delivery Traffic Indication Message. The beacon after which buffered group traffic is released |
| **FCS** | Frame Check Sequence. CRC-32 over the frame |
| **GTK** | Group Temporal Key. Shared; encrypts broadcast and multicast |
| **HT / VHT / HE** | 802.11n / ac / ax |
| **MCS** | Modulation and Coding Scheme. An index into the rate table |
| **MIC** | Message Integrity Code |
| **MPDU / MSDU** | The frame as the MAC sends it / the payload handed to the MAC |
| **NAV** | Network Allocation Vector. The virtual carrier-sense countdown |
| **PMF** | Protected Management Frames, 802.11w |
| **PMK / PTK** | Pairwise Master Key (from the passphrase) / Pairwise Transient Key (per session) |
| **PPDU** | The complete PHY transmission: preamble, PHY header, and the MPDU or A-MPDU |
| **RSSI** | Received Signal Strength Indicator, in dBm |
| **SAE** | Simultaneous Authentication of Equals. WPA3's password exchange |
| **SA Query** | The PMF exchange that verifies an existing association is still real |
| **TIM** | Traffic Indication Map. Which sleeping stations have traffic waiting |
| **TSF** | Timing Synchronization Function. The BSS's shared microsecond clock |
| **TU** | Time Unit — **1024 µs**, not 1000 |
| **TWT** | Target Wake Time. Wi-Fi 6 scheduled sleep |

### E. Where to read more

In the IDF tree on this machine
(`/home/pico/.espressif/v6.0.1/esp-idf`):

| Path | |
|---|---|
| `docs/en/api-guides/wifi-driver/overview.rst` | the driver's own reference — feature list, programming model, event descriptions, the full menuconfig walkthrough |
| `docs/en/api-guides/wifi-driver/station-scenarios.rst` | scan and connect scenarios in exhaustive detail |
| `docs/en/api-guides/wifi-driver/wifi-performance-and-power-save.rst` | buffer usage, throughput numbers, how to improve performance, power save |
| `docs/en/api-guides/wifi-driver/wifi-mac-protocols.rst` | HT20/40, QoS, A-MPDU, A-MSDU, fragmentation |
| `docs/en/api-guides/wifi-driver/wifi-modes.rst` | AP scenarios, sniffer mode, NAN |
| `docs/en/api-guides/wifi-driver/wifi-vendor-features.rst` | raw 802.11 TX, vendor IEs, multiple antennas |
| `docs/en/api-guides/wifi-security.rst` | the security modes in full |
| `docs/en/api-reference/network/esp_now.rst` | ESP-NOW reference |
| `docs/en/migration-guides/release-6.x/6.0/wifi.rst` | §17's source |
| `examples/wifi/` | `getting_started`, `scan`, `fast_scan`, `softap_sta`, `espnow`, `power_save`, `iperf`, `ftm`, `itwt`, `roaming`, `wifi_aware`, `smart_config`, `wps`, `wifi_enterprise`, `wifi_easy_connect`, `wifi_nvs_config` |

In this repository, the code this document draws on:

| Path | |
|---|---|
| `hub_s3/main/net.c` | SoftAP bring-up, the channel survey, TX instrumentation |
| `satellite/main/net.c` | station bring-up, the join state machine, the lease watchdog |
| `hub_s3/main/clients.c` | the send list, ARP seeding, DHCP lease lookups |
| `hub_s3/sdkconfig.defaults` | every radio setting with the measurement that produced it |
| `components/dancefloor_sync/wifi_log.c` | log forwarding over the same socket |
| `tools/soak/` | the capture and analysis harness the log format exists for |
