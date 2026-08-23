#!/usr/bin/env python3
"""
Soak capture: read every unit's console over serial, print it, and write CSV.

The hub and the satellite each need their own USB-serial adapter, so this opens
one port per unit and reads them concurrently. Everything that arrives is echoed
to this terminal, prefixed with the unit it came from, so a soak can be watched
as well as recorded.

Written for the long run architecture.md 17 asks for and has never been given:
"the longest evidenced session is ten minutes against a four-hour target".

Outputs, in one session directory per run:

  raw.log      every line, timestamped, exactly as it arrived. AUTHORITATIVE --
               the parsing below is best-effort and this is what lets a soak be
               re-parsed later when it turns out a field was missed.
  metrics.csv  tidy/long: one row per NUMBER found, columns
               wall_s, wall_iso, unit, esp_ms, level, tag, kind, metric, value.
               Long rather than wide because the units print different lines with
               different fields, and a wide table of that is mostly holes. Pivot
               to taste:
                   df.pivot_table(index=["unit", "wall_s"],
                                  columns="metric", values="value")
  events.csv   the things that happen rather than measure -- underruns, timeline
               restarts, track changes, volume moves -- with their text intact.
               What you want to draw as vertical lines over the metrics.

HOW THE PARSING WORKS, and its limits. ESP_LOG lines are `I (12345) tag: body`,
and this project writes bodies as `key value unit | key value unit`. So the
numbers are extracted by a generic "word followed by a number" scan rather than
one regex per line, which means a counter added to a HEALTH line shows up here
with no change to this script. The cost is that number-before-word phrasings
("44100 Hz", "(0 refused)") are missed unless listed in EXTRA below. When a
figure you want is absent from metrics.csv, it is in raw.log and the fix is a
line in EXTRA.

Requires pyserial (pip install pyserial). pandas is NOT required to capture --
only to analyse afterwards.

BAUD IS PER UNIT, AND THE HUB IS NOT THE DEFAULT. hub_s3/sdkconfig.defaults sets
CONFIG_ESP_CONSOLE_UART_BAUDRATE=921600 on a custom UART (GPIO 43, because the
console was deliberately moved off USB); the satellite and the bridge are on the
IDF default of 115200. Reading the hub at 115200 produces a screen of accented
punctuation rather than an error, which is how this was found. Append `:BAUD` to
any unit that differs.

Usage:
    tools/soak/capture.py --unit hub=/dev/ttyUSB0:921600 --unit sat=/dev/ttyUSB1
    tools/soak/capture.py --unit hub=/dev/ttyUSB0:921600 \
                          --unit sat=/dev/ttyUSB1 \
                          --unit bridge=/dev/ttyUSB2 --out /tmp/soak

Stop with Ctrl-C; the files are flushed as it goes, so an interrupted or
power-cut soak keeps everything up to the last second.
"""

import argparse
import csv
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is missing: pip install pyserial")

# --- log line shape ---------------------------------------------------------

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")

# I (12345) tag: body
ESP_LINE = re.compile(r"^([IWEDV]) \((\d+)\) ([^:]{1,24}): (.*)$")

# "word 123", "word -45", "word +67". The key may carry hyphens and digits,
# which is what catches dma-starve, sta-left, short-reads and RX 5s.
KEYNUM = re.compile(r"([A-Za-z][A-Za-z0-9_-]*)\s+([+-]?\d+)")

# Longest prefix wins, so "local playback started" beats "local ring".
KINDS = [
    ("HEALTH:", "health"),
    ("TRIM:", "trim"),
    ("MEM:", "mem"),
    ("RX 5s:", "rx5s"),
    ("ARRIVAL 5s:", "arrival"),
    ("TSF:", "tsf"),
    ("servo:", "servo"),
    ("TRACK DIVERGENCE", "divergence"),
    ("TRACK BOUNDARY", "boundary"),
    ("track boundary:", "splice"),
    ("local ring ", "status"),
    ("buffer ", "status"),
    ("REFILL after start", "refill"),
    ("RETUNE COST", "retune_cost"),
    ("RETUNE TAIL", "retune_tail"),
    ("local playback started", "playback_start"),
    ("playback started", "playback_start"),
    ("local underrun", "underrun"),
    ("underrun,", "underrun"),
    ("timeline start", "timeline_start"),
    ("timeline restart", "timeline_restart"),
    ("stream start:", "stream_start"),
    ("VOLUME ", "volume"),
    ("TRACK #", "track"),
    ("pkts ", "sbc_in"),
    ("DAC clock retuned", "retune"),
    ("output clock retuned", "retune"),
    ("CRIPPLED:", "crippled"),
    ("ALLOCATION FAILED", "alloc_fail"),
    # bt_bridge. The A2DP source is the one link a soak could never see: when
    # the phone stops feeding, the hub starves and the bridge said nothing at
    # all until status_led.c was taught to log the gap it already measures.
    ("audio stopped", "audio_gap"),
    ("audio resumed", "audio_gap"),
    ("queue full:", "bridge_drop"),
    ("payload past the link ceiling:", "bridge_drop"),
    ("hub never raised the handshake:", "bridge_drop"),
    ("track change (#", "bridge_track"),
]

# Numbers the generic scan cannot see, because the figure comes before the word.
# Matched against the WHOLE body, so these may anchor on the label.
# kind -> [(regex, metric name)]
EXTRA = {
    "trim":       [(re.compile(r"TRIM:\s*([+-]?\d+)\s*Hz"), "trim_hz")],
    "status":     [(re.compile(r"\((\d+) ms\)"), "ring_ms"),
                   # The audio lane's own share of tx-fail. The generic scan sees
                   # the total and stops at the parenthesis, but the split is the
                   # whole point of the lane breakdown: a refused frame costs one
                   # late repaint, a refused audio packet is never retried and
                   # lands on the floor as a hole. Measured at ~11-13 ms of
                   # satellite starvation per refused audio packet across the
                   # three 2026-08-22 soaks.
                   (re.compile(r"tx-fail \d+ \((\d+) audio"), "tx_fail_audio"),
                   # The ENOMEM burst-gap histogram, four metrics rather than one
                   # because they are read against each other: back-to-back vs
                   # beacon-locked vs long stretches far apart are three different
                   # faults. The buckets and how to read them are documented at
                   # the histogram itself in hub_s3/main/net.c.
                   (re.compile(r"\| gaps (\d+)/\d+/\d+/\d+"), "burst-gap-lt25"),
                   (re.compile(r"\| gaps \d+/(\d+)/\d+/\d+"), "burst-gap-25-75"),
                   (re.compile(r"\| gaps \d+/\d+/(\d+)/\d+"), "burst-gap-75-150"),
                   (re.compile(r"\| gaps \d+/\d+/\d+/(\d+)"), "burst-gap-gt150")],
    "health":     [(re.compile(r"\((\d+) refused\)"), "retunes_refused")],
    "divergence": [(re.compile(r"->\s*([+-]?\d+) ms apart"), "apart_ms")],
    "servo":      [(re.compile(r"\((\d+) frames/s\)"), "frames_per_s")],

    # bt_bridge's lines all put the number where the generic scan cannot reach
    # it: behind a colon, or before the noun it counts. That is the case this
    # table exists for.
    #
    # `audio-gap-ms` is the one that matters. It is the length of a silence the
    # bridge saw on its own A2DP input, so a hub-side under-delivery with a gap
    # here is the phone, and one without is this side of the radio.
    "audio_gap":    [(re.compile(r"resumed after (\d+) ms"), "audio-gap-ms"),
                     (re.compile(r"last packet (\d+) ms ago"), "audio-stopped-ms")],
    # "queue full: 12 dropped (+3)" -- the total, not the delta, so it stays a
    # counter across a run rather than a series of increments.
    "bridge_drop":  [(re.compile(r":\s*(\d+) dropped"), "bridge-dropped")],
    "bridge_track": [(re.compile(r"#(\d+)"), "bridge-track-id")],
}

# Lines that mark a moment rather than measure one. Kept whole in events.csv.
EVENT_KINDS = {
    "underrun", "timeline_start", "timeline_restart", "playback_start",
    "stream_start", "track", "volume", "retune", "splice", "boundary",
    "divergence", "crippled", "alloc_fail", "refill",
}


def classify(body):
    """(kind, how many leading characters are label rather than data).

    A label ending in ':' is skipped before the number scan, because otherwise
    "RX 5s:" reads as the metric rx=5. A prefix that does NOT end in ':' is real
    data and must be kept -- "pkts " is the sbc_in line's first counter, and
    "local ring " carries the ring depth.
    """
    for prefix, kind in KINDS:
        if body.startswith(prefix):
            return kind, (len(prefix) if prefix.rstrip().endswith(":") else 0)
    return "other", 0


def numbers(kind, body, skip=0):
    """Every (metric, value) this line carries. Best effort; raw.log is truth."""
    out = []
    seen = set()
    for key, val in KEYNUM.findall(body[skip:]):
        k = key.lower()
        if k in seen:          # first wins; repeats are usually a different noun
            continue
        seen.add(k)
        out.append((k, int(val)))
    for pattern, name in EXTRA.get(kind, []):
        m = pattern.search(body)
        if m and name not in seen:
            seen.add(name)
            out.append((name, int(m.group(1))))
    return out


# --- capture ----------------------------------------------------------------

class Session:
    """Files, locks and counters shared by every reader thread."""

    def __init__(self, outdir):
        os.makedirs(outdir, exist_ok=True)
        self.dir = outdir
        self.lock = threading.Lock()
        self.raw = open(os.path.join(outdir, "raw.log"), "a", buffering=1)

        self.mfile = open(os.path.join(outdir, "metrics.csv"), "a", newline="")
        self.metrics = csv.writer(self.mfile)
        self.efile = open(os.path.join(outdir, "events.csv"), "a", newline="")
        self.events = csv.writer(self.efile)
        if self.mfile.tell() == 0:
            self.metrics.writerow(["wall_s", "wall_iso", "unit", "esp_ms",
                                   "level", "tag", "kind", "metric", "value"])
        if self.efile.tell() == 0:
            self.events.writerow(["wall_s", "wall_iso", "unit", "esp_ms",
                                  "level", "tag", "kind", "text"])

        self.n_lines = 0
        self.n_metrics = 0
        self.n_events = 0
        self.last_flush = time.time()
        self.stop = threading.Event()

    def record(self, unit, line):
        now = time.time()
        iso = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(now)) + \
            f".{int((now % 1) * 1000):03d}"
        clean = ANSI.sub("", line).rstrip("\r\n")
        if not clean.strip():
            return

        print(f"[{unit}] {clean}", flush=True)

        with self.lock:
            self.raw.write(f"{now:.3f}\t{iso}\t{unit}\t{clean}\n")
            self.n_lines += 1
            self.emit(now, iso, unit, clean)

            if now - self.last_flush > 2.0:
                self.mfile.flush()
                self.efile.flush()
                self.last_flush = now

    def emit(self, now, iso, unit, clean):
        """Derive metrics and events from one already-cleaned line.

        Apart from record() so that replay() can re-derive metrics.csv from a
        raw.log written earlier. raw.log is truth -- numbers() says so -- and a
        parser that cannot be re-run over it means every improvement to the
        extraction silently fails to reach the sessions already captured, which
        is exactly the sessions a change is being measured against.

        The caller holds the lock and owns the timestamp: replay takes both from
        the file rather than from the clock.
        """
        m = ESP_LINE.match(clean)
        if not m:
            return
        level, esp_ms, tag, body = m.group(1), int(m.group(2)), \
            m.group(3).strip(), m.group(4)
        kind, skip = classify(body)
        for metric, value in numbers(kind, body, skip):
            self.metrics.writerow([f"{now:.3f}", iso, unit, esp_ms,
                                   level, tag, kind, metric, value])
            self.n_metrics += 1
        if kind in EVENT_KINDS or level == "E":
            self.events.writerow([f"{now:.3f}", iso, unit, esp_ms,
                                  level, tag, kind, body])
            self.n_events += 1

    def close(self):
        with self.lock:
            for f in (self.raw, self.mfile, self.efile):
                try:
                    f.flush()
                    f.close()
                except Exception:
                    pass


def garbled(line):
    """True when a line is mostly replacement chars -- wrong baud, most likely.

    Reading a board below its console baud turns the stream into accent soup,
    and the soup rarely contains a newline, so it surfaces as one huge line
    rather than an error. This is what that looks like, so it can be said out
    loud instead of quietly capturing nothing for an hour.
    """
    if not line.strip():
        return False
    bad = sum(1 for c in line if c == "�" or
              (ord(c) < 32 and c not in "\t"))
    return bad > len(line) * 0.1


def reader(session, unit, port, baud):
    """One serial port, reopened for as long as the soak runs.

    A board reset drops the adapter on some cables, and a soak that ends because
    somebody nudged a USB plug at hour three is not a soak. Reconnects quietly
    and says so on the console, which also lands in raw.log as a marker.
    """
    while not session.stop.is_set():
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                session.record(unit, f"--- capture: opened {port} @ {baud} ---")
                buf = b""
                garbled_seen = False
                while not session.stop.is_set():
                    chunk = ser.read(4096) or b""
                    if chunk:
                        buf += chunk
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            text = line.decode("utf-8", errors="replace")
                            if not garbled_seen and garbled(text):
                                garbled_seen = True
                                session.record(
                                    unit,
                                    f"--- capture: GARBLED TEXT from {port} -- "
                                    f"wrong baud? reading @ {baud}; a board's "
                                    f"console needs its own rate ---")
                            session.record(unit, text)
                        # Garbled streams may never emit a newline; cap the
                        # buffer rather than grow it for four hours.
                        if len(buf) > 65536:
                            session.record(unit, buf.decode("utf-8",
                                                             errors="replace"))
                            buf = b""
        except serial.SerialException as e:
            if session.stop.is_set():
                return
            session.record(unit, f"--- capture: {port} lost ({e}), retrying ---")
            time.sleep(2.0)
        except Exception as e:                                  # noqa: BLE001
            if session.stop.is_set():
                return
            session.record(unit, f"--- capture: {port} error ({e}), retrying ---")
            time.sleep(2.0)


AIR_UNIT = "air"

# The three non-overlapping 2.4 GHz channels, and the overlap rule. Channels sit
# 5 MHz apart and are 22 MHz wide, so anything within four of a candidate lands
# on top of it. Identical to survey_channel() in hub_s3/main/net.c, deliberately:
# the hub prints one of these lines at boot from its own antenna, and the two are
# only comparable if they are scored the same way.
AIR_CANDIDATES = (1, 6, 11)
AIR_OVERLAP = 4

# Our own SoftAP, which this scan must not count as interference. It is the
# loudest thing in the room from where the laptop sits -- nmcli reports it at
# quality 100 -- so leaving it in pins whichever channel the hub chose at
# maximum occupancy and makes the whole metric read backwards. The hub's own
# survey has no such problem: it scans in STA mode before the AP is started and
# physically cannot see itself. Matches AP_SSID in hub_s3/main/hub.h.
AIR_OWN_SSID = "dancefloor"


def air_scan():
    """One 2.4 GHz sweep, as (nets_seen, {channel: (dbm, count)}), or None.

    WHY THIS EXISTS. Three soaks have been read for a hub TX ENOMEM fault that
    arrives in episodes, and nothing on the hub moves when one starts: internal
    free flat, stations 2, churn 0, RSSI pinned, source clean. The episodes share
    no time of day and no common uptime. What has never been recorded is what
    else was on the air, and this is the cheapest instrument that records it.

    WHAT IT DOES NOT MEASURE: airtime. A beacon says a network is PRESENT, not
    that it is busy, so a quiet neighbour starting a large transfer looks like
    nothing at all here. It will catch a network appearing, changing channel, or
    changing level, which are the likelier triggers. `iw survey dump` would give
    real channel busy time and is the upgrade if this proves too blunt -- it
    needs root and the interface parked on the hub's channel.

    IT MEASURES AN INTERFERER AS LOSS OF DECODE, NOT AS A RISE IN LEVEL. This
    was built expecting a busy channel to read louder during a hub TX episode.
    A passive scan only counts beacons it successfully decodes, so somebody
    shouting on a channel makes that channel's neighbours vanish from this list
    instead. It shows up as the count falling -- at the limit `ch{c}-nets 0`
    with `ch{c}-dbm -100`, which is the floor substituted below and not a
    measured level. analyse.py's AIR section reads it that way round and calls
    the zeros out; see the long comment on its section 7b for how far that got,
    which is not as far as it first looked.

    IT IS ALSO NOISY, measured here at ~10 dB sweep to sweep on a busy channel
    (ch1 read -60, -51, -61 in three consecutive sweeps) because a single scan
    does not always catch every AP. A quiet channel is far steadier (-69, -70,
    -70 on ch11 over the same three). So read a trend across several sweeps, not
    one reading: a few dB between adjacent sweeps is this, not a neighbour.

    THE 30 s DEFAULT IS NOT ABOUT COST. NetworkManager rate-limits `nmcli dev
    wifi rescan` to roughly every 10-15 s, so sweeping faster returns the CACHED
    list: one real blackout would repeat across several sweeps and read as a
    long one, manufacturing evidence for the very thing the sweep exists to
    test. Both blackouts seen so far were a single isolated sweep with normal
    readings either side, which is only meaningful while a sweep is an
    independent look at the band.
    """
    try:
        subprocess.run(["nmcli", "dev", "wifi", "rescan"],
                       capture_output=True, timeout=20)
    except Exception:                                           # noqa: BLE001
        pass        # NetworkManager rate-limits rescans; the cached list is fine
    out = subprocess.run(["nmcli", "-t", "-f", "SSID,CHAN,SIGNAL",
                          "dev", "wifi", "list"],
                         capture_output=True, text=True, timeout=20)
    if out.returncode != 0:
        raise RuntimeError((out.stderr or "nmcli failed").strip()[:120])

    power = {c: 0.0 for c in AIR_CANDIDATES}
    nets = {c: 0 for c in AIR_CANDIDATES}
    seen = 0
    for line in out.stdout.splitlines():
        parts = line.rsplit(":", 2)         # SSIDs may contain ':'
        if len(parts) != 3:
            continue
        if parts[0] == AIR_OWN_SSID:
            continue
        try:
            chan, sig = int(parts[1]), int(parts[2])
        except ValueError:
            continue
        if chan > 14:                       # 5 GHz shares no air with the hub
            continue
        seen += 1
        # nmcli reports SIGNAL as 0-100 QUALITY, not dBm. This is wpa_supplicant's
        # usual inverse and it is APPROXIMATE -- the hub's own survey line is the
        # calibrated one. Good enough for tracking change over a run, which is
        # the only thing being asked of it.
        dbm = sig / 2.0 - 100.0
        for c in AIR_CANDIDATES:
            if abs(chan - c) <= AIR_OVERLAP:
                power[c] += 10.0 ** (dbm / 10.0)
                nets[c] += 1
    # Linear sum back to dBm for printing, with a floor rather than -inf.
    return seen, {c: (int(round(10.0 * math.log10(power[c]))) if power[c] > 0
                      else -100, nets[c])
                  for c in AIR_CANDIDATES}


def air_watcher(session, interval):
    """Sweep the band every `interval` seconds for as long as the soak runs.

    Emits an ESP-shaped line so Session.emit() parses it with no special case,
    and lands it under a pseudo-unit rather than a second file so that a scan and
    a hub status line can be read against each other by wall_s. analyse.py drops
    AIR_UNIT from its per-unit loops; nothing else needs to know.
    """
    while not session.stop.is_set():
        try:
            seen, per = air_scan()
            body = " | ".join(
                [f"nets {seen}"] +
                [f"ch{c}-dbm {per[c][0]} ch{c}-nets {per[c][1]}"
                 for c in AIR_CANDIDATES])
            session.record(AIR_UNIT, f"I (0) air: {body}")
        except FileNotFoundError:
            session.record(AIR_UNIT, "--- capture: no nmcli, air scan off "
                                     "for this session ---")
            return
        except Exception as e:                                  # noqa: BLE001
            # One notice, then stop. A soak must not die, or fill its log, because
            # the laptop's WiFi went away.
            session.record(AIR_UNIT, f"--- capture: air scan failed ({e}), "
                                     f"off for this session ---")
            return
        session.stop.wait(interval)


def replay(outdir):
    """Rebuild metrics.csv and events.csv from an existing session's raw.log.

    For when the extraction has learned something the session predates. The
    three 2026-08-22 soaks were captured before tx-fail's lane split was pulled
    out, so without this the baseline runs could never produce the one number a
    fix for that fault has to be measured against.

    raw.log is not touched, and the two derived files are rewritten from
    nothing rather than appended to, so replaying twice is not additive.
    """
    raw_path = os.path.join(outdir, "raw.log")
    if not os.path.exists(raw_path):
        sys.exit(f"{raw_path} does not exist -- not a session directory")
    for name in ("metrics.csv", "events.csv"):
        path = os.path.join(outdir, name)
        if os.path.exists(path):
            os.remove(path)

    session = Session(outdir)
    kept = skipped = 0
    with open(raw_path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            parts = line.rstrip("\n").split("\t", 3)
            # capture.py's own banner lines ("--- capture: opened ... ---") have
            # the same four fields but no ESP prefix, so emit() drops them; a
            # line with fewer fields is a truncated write and is not guessed at.
            if len(parts) < 4:
                skipped += 1
                continue
            wall_s, iso, unit, clean = parts
            try:
                now = float(wall_s)
            except ValueError:
                skipped += 1
                continue
            session.emit(now, iso, unit, clean)
            kept += 1
    session.close()
    print(f"replayed {kept:,} lines from {raw_path}"
          + (f" ({skipped:,} unparseable)" if skipped else ""), file=sys.stderr)
    print(f"  -> {session.n_metrics:,} metrics, {session.n_events:,} events",
          file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(
        description="Capture every unit's console to CSV for a soak run.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="example, all four boards:\n"
               "  %(prog)s \\\n"
               "    --unit hub=/dev/serial/by-id/usb-FTDI_...-if00-port0:921600 \\\n"
               "    --unit bt_bridge=/dev/serial/by-id/usb-Silicon_Labs_... \\\n"
               "    --unit sat_classic=/dev/ttyUSB0 --unit sat_s3=/dev/ttyACM0\n\n"
               "CAPTURE bt_bridge. It is the A2DP source's only witness, and two\n"
               "soaks were read without it: the hub starved, the timeline jumped,\n"
               "and nothing could say whether the phone had stopped sending or the\n"
               "bridge had failed to forward. status_led.c logs the audio gap and\n"
               "sbc_spi.c the drops, but only onto that board's own console.\n\n"
               "by-id paths, not /dev/ttyUSB0: the hub and one satellite are both\n"
               "USB-serial and swap numbers between plugs, which relabels a whole\n"
               "run silently.\n\n"
               "The hub's console is 921600 in this repo's sdkconfig.defaults, on a\n"
               "custom UART; the satellites and bridge are the IDF default 115200.\n"
               "Reading a board at the wrong baud prints garbage, not an error.")
    ap.add_argument("--unit", action="append", metavar="NAME=PORT[:BAUD]",
                    help="repeatable, one per board: hub=/dev/ttyUSB0:921600")
    ap.add_argument("--replay", metavar="DIR",
                    help="re-derive DIR/metrics.csv and DIR/events.csv from its "
                         "raw.log with the current extraction, and exit; no "
                         "boards are opened and raw.log is not modified")
    ap.add_argument("--baud", type=int, default=115200,
                    help="baud for units that do not name their own "
                         "(default 115200, the IDF default)")
    ap.add_argument("--air-interval", type=int, default=30, metavar="SECONDS",
                    help="sweep 2.4 GHz every SECONDS and record what else is "
                         "on the air, as the pseudo-unit 'air' (default 30, "
                         "0 disables). Needs nmcli; a soak runs without it")
    ap.add_argument("--out", default=None,
                    help="session directory (default logs-soak-<timestamp>/, "
                         "which .gitignore already covers)")
    args = ap.parse_args()

    if args.replay:
        replay(args.replay)
        return
    if not args.unit:
        ap.error("--unit is required (or --replay DIR to rebuild an old session)")

    units = []
    for spec in args.unit:
        if "=" not in spec:
            ap.error(f"--unit wants NAME=PORT[:BAUD], got {spec!r}")
        name, port = spec.split("=", 1)
        port = port.strip()
        baud = args.baud
        # Only a trailing all-digit field is a baud; a device path could in
        # principle carry a colon and must not be truncated into one.
        if ":" in port:
            head, _, tail = port.rpartition(":")
            if head and tail.isdigit():
                port, baud = head, int(tail)
        units.append((name.strip(), port, baud))
    # Two boards under one name cannot be told apart later: the CSVs carry the
    # name, not the port, so their lines interleave into per-unit nonsense.
    seen = [u[0] for u in units]
    if len(set(seen)) != len(seen):
        dup = sorted({n for n in seen if seen.count(n) > 1})
        ap.error(f"duplicate unit name(s) {', '.join(dup)}: give each board its"
                 f" own --unit NAME; a shared name cannot be separated afterwards")

    outdir = args.out or time.strftime("logs-soak-%Y%m%d-%H%M%S")
    session = Session(outdir)

    print(f"capturing {len(units)} unit(s) into {outdir}/", file=sys.stderr)
    if args.air_interval > 0:
        print(f"  {'air':8s} 2.4 GHz sweep every {args.air_interval}s",
              file=sys.stderr)
    for name, port, baud in units:
        print(f"  {name:8s} {port} @ {baud}", file=sys.stderr)
    print("Ctrl-C to stop\n", file=sys.stderr)

    serial_threads = [threading.Thread(target=reader,
                                       args=(session, name, port, baud),
                                       daemon=True)
                      for name, port, baud in units]
    threads = list(serial_threads)
    if args.air_interval > 0:
        threads.append(threading.Thread(target=air_watcher,
                                        args=(session, args.air_interval),
                                        daemon=True))
    for t in threads:
        t.start()

    def bye(signum, frame):
        session.stop.set()
    signal.signal(signal.SIGINT, bye)
    signal.signal(signal.SIGTERM, bye)

    started = time.time()
    try:
        while not session.stop.is_set():
            time.sleep(0.5)
            # The SERIAL readers decide when a soak is over. air_watcher
            # retires by itself when nmcli is missing, and that must not end it.
            if not any(t.is_alive() for t in serial_threads):
                break
    finally:
        session.stop.set()
        for t in threads:
            t.join(timeout=3.0)
        session.close()
        mins = (time.time() - started) / 60.0
        print(f"\n{mins:.1f} min | {session.n_lines} lines | "
              f"{session.n_metrics} metrics | {session.n_events} events "
              f"-> {outdir}/", file=sys.stderr)


if __name__ == "__main__":
    main()
