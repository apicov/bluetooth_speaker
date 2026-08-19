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
import os
import re
import signal
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
]

# Numbers the generic scan cannot see, because the figure comes before the word.
# Matched against the WHOLE body, so these may anchor on the label.
# kind -> [(regex, metric name)]
EXTRA = {
    "trim":       [(re.compile(r"TRIM:\s*([+-]?\d+)\s*Hz"), "trim_hz")],
    "status":     [(re.compile(r"\((\d+) ms\)"), "ring_ms")],
    "health":     [(re.compile(r"\((\d+) refused\)"), "retunes_refused")],
    "divergence": [(re.compile(r"->\s*([+-]?\d+) ms apart"), "apart_ms")],
    "servo":      [(re.compile(r"\((\d+) frames/s\)"), "frames_per_s")],
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

            m = ESP_LINE.match(clean)
            if m:
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

            if now - self.last_flush > 2.0:
                self.mfile.flush()
                self.efile.flush()
                self.last_flush = now

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


def main():
    ap = argparse.ArgumentParser(
        description="Capture every unit's console to CSV for a soak run.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="example:\n"
               "  %(prog)s --unit hub=/dev/ttyUSB0:921600 --unit sat=/dev/ttyUSB1\n\n"
               "The hub's console is 921600 in this repo's sdkconfig.defaults, on a\n"
               "custom UART; the satellite and bridge are the IDF default 115200.\n"
               "Reading a board at the wrong baud prints garbage, not an error.")
    ap.add_argument("--unit", action="append", required=True,
                    metavar="NAME=PORT[:BAUD]",
                    help="repeatable, one per board: hub=/dev/ttyUSB0:921600")
    ap.add_argument("--baud", type=int, default=115200,
                    help="baud for units that do not name their own "
                         "(default 115200, the IDF default)")
    ap.add_argument("--out", default=None,
                    help="session directory (default logs-soak-<timestamp>/, "
                         "which .gitignore already covers)")
    args = ap.parse_args()

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
    for name, port, baud in units:
        print(f"  {name:8s} {port} @ {baud}", file=sys.stderr)
    print("Ctrl-C to stop\n", file=sys.stderr)

    threads = [threading.Thread(target=reader,
                                args=(session, name, port, baud),
                                daemon=True)
               for name, port, baud in units]
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
            if not any(t.is_alive() for t in threads):
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
