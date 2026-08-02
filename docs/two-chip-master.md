# The two-chip master

The master unit is **two ESP32s**. Satellites remain single-chip.

```
          A2DP                I2S                    WiFi multicast
 phone ─────────▶ bt_bridge ────────▶ hub ──────────────────────────▶ satellites
                  (ESP32)             (ESP32)
                  Bluedroid           SoftAP, clock master, streamer
                  I2S master out      I2S slave in, own DAC, own LEDs
```

Both chips sit in the master enclosure. **The master is still a full speaker**:
the hub chip drives its own DAC and its own LED strip, exactly like a satellite.

---

## Why

Not a preference — a measurement. On one chip, Bluedroid and the WiFi stack
together left **164 bytes** of DRAM free. `esp_wifi_init()` failed outright at
first, and once it did start, Bluedroid could not allocate L2CAP buffers when the
phone connected.

Getting it to run at all cost:

- FFT cut from 1024 to 512 points, losing the bass resolution the beat detector
  was designed around
- Lead time cut from 250 ms to 100 ms
- Ring buffers cut from 64 kB to 24 kB
- WiFi RX/TX buffers reduced, AMPDU aggregation disabled

Splitting the chips returns all of it:

| Firmware | DRAM used | Free |
|---|---|---|
| Single-chip master (before) | 92.4 kB / 124.6 kB | **32 kB** |
| `bt_bridge` | 61.0 kB / 124.6 kB | 63.6 kB |
| `hub` | 59.8 kB / 180.7 kB | **121 kB** |
| `satellite` | 37.7 kB / 180.7 kB | 143 kB |

The *total* grows as well as the free space, because Bluedroid statically
reserves DRAM before allocating anything.

### Three problems solved, not one

**Memory.** ~200 kB spare per chip instead of two stacks fighting over 320 kB.

**Radio contention.** Two chips, two radios, two antennas. The Bluetooth/WiFi
coexistence risk that M7 exists to measure simply does not arise on either chip.

**A2DP burstiness.** This one was unexpected. Bluetooth delivers audio in ~100 ms
lumps, so on the single-chip master the presentation timeline raced ~50 ms ahead
while a burst was sent and fell ~50 ms behind while the buffer starved —
tripping the resync guard on every cycle:

```
W stream: timeline off by  51250 us, resyncing
W stream: timeline off by -50187 us, resyncing
W stream: timeline off by  53270 us, resyncing
```

The bridge chip absorbs that. Its I2S output is a steady 44.1 kHz stream, so the
hub receives evenly paced audio and the timeline stays accurate. This mattered
beyond the log noise: a satellite joining late, or re-anchoring after an
underrun, takes its timing from the first chunk it sees, so a timeline jerked
around by resyncs would hand it a start time up to 50 ms wrong.

---

## Wiring between the chips

Four wires. The bridge is I2S **master** (it generates the clocks); the hub is
I2S **slave**.

| bt_bridge (out) | hub (in) | Signal |
|---|---|---|
| GPIO 26 | GPIO 21 | BCK — bit clock |
| GPIO 27 | GPIO 22 | WS / LRCK — word select |
| GPIO 25 | GPIO 23 | DATA |
| GND | GND | **required** — common reference |

Pins are configurable under `menuconfig`; the hub's are in *Dancefloor hub*.

> **Avoid GPIO 16/17 for these.** On WROVER modules they are wired to the PSRAM
> die even when PSRAM is disabled in software. **Avoid GPIO 5** as well: it is a
> strapping pin, and another chip driving it at boot can change the boot mode.

The hub separately drives its own DAC on GPIO 26/27/25 and its LED strip on
GPIO 18. No conflict — those are different chips' pin 26.

### Why two I2S ports on the hub

`I2S_NUM_0` is the slave receiver from the bridge; `I2S_NUM_1` is the master
transmitter to the DAC. They cannot share a port: one follows the bridge's clock,
the other generates its own.

---

## Sample rate

The hub is an I2S slave, so it plays at whatever rate the bridge clocks. Rather
than assume 44.1 kHz, `audio_in.c` counts frames over 5-second windows and
reports the measured rate to the streamer. A phone negotiating 48 kHz would
otherwise put every timeline calculation out by 9%.

Watch for this line:

```
I audio_in: measured input rate 44098 Hz
```

A number near the nominal rate means the link is healthy. Something far off means
either a wiring fault or an unexpected negotiated rate.

---

## Building and flashing

Three separate firmwares:

```sh
. ~/.espressif/v6.0.1/esp-idf/export.sh

cd bt_bridge && idf.py -p /dev/ttyUSB0 flash monitor   # chip A, master enclosure
cd hub       && idf.py -p /dev/ttyUSB1 flash monitor   # chip B, master enclosure
cd satellite && idf.py -p /dev/ttyUSB2 flash monitor   # each satellite
```

`bt_bridge` needs the 4 MB partition table (Bluedroid alone is ~950 kB). The hub
and satellite are far smaller.

---

## Gotcha: BT_ACL_CONNECTIONS

Do **not** set `CONFIG_BT_ACL_CONNECTIONS=1` on the bridge, even though only one
phone ever connects. It sizes Bluedroid's ACL buffer pools, and shrinking them
throttles the stream to a few percent of full rate. The signature is misleading:
the link stays healthy and *active* (not sniff), the phone shows as connected and
playing, but the sink ringbuffer underflows continuously with the gaps between
refills growing — 1.5 s, 2.9 s, 4.6 s, 8.9 s — because it takes seconds to
accumulate a buffer that should fill in 116 ms.

At the correct default of 4, the stream runs at ~50 packets/s (about 20 ms of
audio each) with the ringbuffer quiet.

It was added while fighting for DRAM on the single-chip master. The split makes
it pointless: the bridge has ~63 kB spare.

---

## Cost

About $5 and four wires, on the master only. Satellites are unaffected.

The firmware got *simpler*, not more complex: `bt_bridge` is the stock ESP-IDF
A2DP sink example with a device name, and the hub is a satellite that happens to
source the stream and own the clock.
