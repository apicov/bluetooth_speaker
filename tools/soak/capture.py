#!/usr/bin/env python3
"""
Soak capture: read every unit's console over serial, print it, and write CSV.

Each board needs its own USB-serial adapter, so this opens one port per unit
and reads them concurrently. Everything that arrives is echoed here, prefixed
with the unit it came from, so a soak can be watched as well as recorded.

Three files per run, in one session directory:

  raw.log      every line, timestamped, exactly as it arrived. AUTHORITATIVE --
               the parsing below is best-effort, and this is what lets a
               session be re-parsed later when it turns out a field was
               missed. That is what --replay does.
  metrics.csv  tidy/long: one row per NUMBER found, with columns
               wall_s, wall_iso, unit, esp_ms, level, tag, kind, metric, value.
               Long rather than wide because the units print different lines
               with different fields, and a wide table of that is mostly holes.
               Pivot to taste:
                   df.pivot_table(index=["unit", "wall_s"],
                                  columns="metric", values="value")
  events.csv   the things that happen rather than measure -- underruns,
               timeline restarts, track changes, volume moves -- with their
               text intact. What to draw as vertical lines over the metrics.

HOW THE PARSING WORKS, and its limits. ESP_LOG lines are `I (12345) tag: body`,
and this project writes bodies as `key value unit | key value unit`. So numbers
are extracted by a generic "word followed by a number" scan rather than by one
regex per line, which means a counter added to a HEALTH line appears here with
no change to this script. The cost is that number-before-word phrasings
("44100 Hz", "(0 refused)") are missed unless they are listed in EXTRA. When a
figure is absent from metrics.csv it is still in raw.log, and the fix is a line
in EXTRA followed by --replay.

Requires pyserial. pandas is NOT required to capture -- only to analyse
afterwards.

BAUD IS PER UNIT, AND THE HUB IS NOT THE DEFAULT. hub_s3/sdkconfig.defaults
sets CONFIG_ESP_CONSOLE_UART_BAUDRATE=921600 on a console deliberately moved
off USB onto GPIO 43; the satellites and the bridge are on the IDF default of
115200. Reading the hub at 115200 produces a screenful of accented punctuation
rather than an error, which garbled() exists to say out loud. Append `:BAUD` to
any unit that differs.

Usage:
    tools/soak/capture.py --unit hub=/dev/ttyUSB0:921600 --unit sat=/dev/ttyUSB1
    tools/soak/capture.py --unit hub=/dev/ttyUSB0:921600 \
                          --unit sat=/dev/ttyUSB1 \
                          --unit bridge=/dev/ttyUSB2 --out /tmp/soak

Stop with Ctrl-C. The files are flushed as it goes, so an interrupted or
power-cut soak keeps everything up to the last couple of seconds.
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


## @brief Escape sequences, stripped before anything else looks at a line.
ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")

## @brief An ESP_LOG line: `I (12345) tag: body`.
ESP_LINE = re.compile(r"^([IWEDV]) \((\d+)\) ([^:]{1,24}): (.*)$")

## @brief The generic number scan: "word 123", "word -45", "word +67".
#
# The key may carry hyphens and digits, which is what catches dma-starve,
# sta-left, short-reads and RX 5s.
KEYNUM = re.compile(r"([A-Za-z][A-Za-z0-9_-]*)\s+([+-]?\d+)")

## @brief Body prefix to line kind, longest prefix first.
#
# Order matters: "local playback started" has to be tested before "local ring".
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
    # bt_bridge. The A2DP source is the one link a soak could otherwise never
    # see: when the phone stops feeding, the hub starves, and only the bridge
    # knows whether anything was sent.
    ("audio stopped", "audio_gap"),
    ("audio resumed", "audio_gap"),
    ("queue full:", "bridge_drop"),
    ("payload past the link ceiling:", "bridge_drop"),
    ("hub never raised the handshake:", "bridge_drop"),
    ("track change (#", "bridge_track"),
]

## @brief Numbers the generic scan cannot see, because the figure comes before
#         the word. Kind -> [(regex, metric name)].
#
# Matched against the WHOLE body, so a pattern here may anchor on the label.
EXTRA = {
    "trim":       [(re.compile(r"TRIM:\s*([+-]?\d+)\s*Hz"), "trim_hz")],
    "status":     [(re.compile(r"\((\d+) ms\)"), "ring_ms"),
                   # The audio lane's own share of tx-fail. The generic scan
                   # sees the total and stops at the parenthesis, but the split
                   # is the point of the lane breakdown: a refused frame costs
                   # one late repaint, while a refused audio packet is never
                   # retried and lands on the floor as a hole.
                   (re.compile(r"tx-fail \d+ \((\d+) audio"), "tx_fail_audio"),
                   # The ENOMEM burst-gap histogram, four metrics rather than
                   # one because they are read against each other: back-to-back
                   # against beacon-locked against long stretches far apart are
                   # three different faults. The buckets are documented where
                   # they are printed, in hub_s3/main/net.c.
                   (re.compile(r"\| gaps (\d+)/\d+/\d+/\d+"), "burst-gap-lt25"),
                   (re.compile(r"\| gaps \d+/(\d+)/\d+/\d+"), "burst-gap-25-75"),
                   (re.compile(r"\| gaps \d+/\d+/(\d+)/\d+"), "burst-gap-75-150"),
                   (re.compile(r"\| gaps \d+/\d+/\d+/(\d+)"), "burst-gap-gt150")],
    "health":     [(re.compile(r"\((\d+) refused\)"), "retunes_refused")],
    "divergence": [(re.compile(r"->\s*([+-]?\d+) ms apart"), "apart_ms")],
    "servo":      [(re.compile(r"\((\d+) frames/s\)"), "frames_per_s")],

    # bt_bridge's lines all put the number where the generic scan cannot reach
    # it -- behind a colon, or before the noun it counts -- which is the case
    # this table exists for. `audio-gap-ms` is the one that matters: it is the
    # length of a silence the bridge saw on its own A2DP input, so a hub-side
    # under-delivery WITH a gap here is the phone, and one without it is this
    # side of the radio.
    "audio_gap":    [(re.compile(r"resumed after (\d+) ms"), "audio-gap-ms"),
                     (re.compile(r"last packet (\d+) ms ago"), "audio-stopped-ms")],
    # "queue full: 12 dropped (+3)" -- the total, not the delta, so it stays a
    # counter across a run rather than a series of increments.
    "bridge_drop":  [(re.compile(r":\s*(\d+) dropped"), "bridge-dropped")],
    "bridge_track": [(re.compile(r"#(\d+)"), "bridge-track-id")],
}

## @brief Kinds that mark a moment rather than measure one. Kept whole, with
#         their text, in events.csv.
EVENT_KINDS = {
    "underrun", "timeline_start", "timeline_restart", "playback_start",
    "stream_start", "track", "volume", "retune", "splice", "boundary",
    "divergence", "crippled", "alloc_fail", "refill",
}


def classify(body):
    """
    @brief Which kind of line this body is, and how much of it is label.

    A label ending in ':' is skipped before the number scan, because otherwise
    "RX 5s:" reads as the metric rx=5. A prefix that does NOT end in ':' is
    real data and is kept -- "pkts " is the sbc_in line's first counter, and
    "local ring " carries the ring depth.

    @param body  The part of the log line after the tag.
    @return (kind, characters to skip); ("other", 0) for anything unrecognised.
    """
    for prefix, kind in KINDS:
        if body.startswith(prefix):
            return kind, (len(prefix) if prefix.rstrip().endswith(":") else 0)
    return "other", 0


def numbers(kind, body, skip=0):
    """
    @brief Every (metric, value) a line carries.

    Best effort, and deliberately so: raw.log is the record, and --replay is
    how a session captured before an improvement here gets the benefit of it.

    @param kind  As classify() returned it; selects the EXTRA patterns.
    @param body  The part of the log line after the tag.
    @param skip  Leading characters to ignore, as classify() returned.
    @return A list of (metric, int). First occurrence of a name wins, since a
            repeat is usually a different noun.
    """
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


class Session:
    """
    @brief The three output files, the lock over them, and the run's counters.

    One instance per run, shared by every reader thread.
    """

    def __init__(self, outdir):
        """
        @brief Open or append to the session's three files and write headers.
        @param outdir  The session directory; created if it does not exist.
        """
        os.makedirs(outdir, exist_ok=True)
        ## @brief The session directory.
        self.dir = outdir
        ## @brief Held across every write, so two units cannot interleave
        #         mid-row.
        self.lock = threading.Lock()
        ## @brief The authoritative log, line-buffered so a power cut costs at
        #         most the line in flight.
        self.raw = open(os.path.join(outdir, "raw.log"), "a", buffering=1)

        ## @brief metrics.csv, flushed every couple of seconds rather than
        #         per row.
        self.mfile = open(os.path.join(outdir, "metrics.csv"), "a", newline="")
        ## @brief Writer over mfile.
        self.metrics = csv.writer(self.mfile)
        ## @brief events.csv. @see mfile
        self.efile = open(os.path.join(outdir, "events.csv"), "a", newline="")
        ## @brief Writer over efile.
        self.events = csv.writer(self.efile)
        if self.mfile.tell() == 0:
            self.metrics.writerow(["wall_s", "wall_iso", "unit", "esp_ms",
                                   "level", "tag", "kind", "metric", "value"])
        if self.efile.tell() == 0:
            self.events.writerow(["wall_s", "wall_iso", "unit", "esp_ms",
                                  "level", "tag", "kind", "text"])

        ## @brief Lines seen, for the summary at the end.
        self.n_lines = 0
        ## @brief Metric rows written.
        self.n_metrics = 0
        ## @brief Event rows written.
        self.n_events = 0
        ## @brief When the two CSVs were last flushed.
        self.last_flush = time.time()
        ## @brief Set on Ctrl-C; every loop in this file watches it.
        self.stop = threading.Event()

    def record(self, unit, line):
        """
        @brief Timestamp, clean, echo and file one arriving line.
        @param unit  Which board it came from.
        @param line  The raw line, escape codes and newline included.
        """
        now = time.time()
        iso = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(now)) + \
            f".{int((now % 1) * 1000):03d}"
        clean = ANSI.sub("", line).rstrip("\r\n")
        if not clean.strip():
            return

        print(f"[{unit}] {clean}", flush=True)

        ## @cond
        # Doxygen's Python scanner reads `with self.<attr>:` as a member
        # declaration and then attaches the next identifier it sees to this
        # class. Hiding the block costs nothing -- statements inside a method
        # are not documentation -- and keeps doc-gen/doxygen.warn a clean
        # coverage test rather than a file with two known lies in it.
        with self.lock:
            self.raw.write(f"{now:.3f}\t{iso}\t{unit}\t{clean}\n")
            self.n_lines += 1
            self.emit(now, iso, unit, clean)

            if now - self.last_flush > 2.0:
                self.mfile.flush()
                self.efile.flush()
                self.last_flush = now
        ## @endcond

    def emit(self, now, iso, unit, clean):
        """
        @brief Derive metrics and events from one already-cleaned line.

        Separate from record() so that replay() can rebuild metrics.csv from a
        raw.log written earlier. raw.log is the record, and a parser that
        cannot be re-run over it means every improvement to the extraction
        silently fails to reach the sessions already captured -- which are
        exactly the sessions a change is being measured against.

        @param now    Wall clock, seconds. The caller owns it: replay takes it
                      from the file rather than from the clock.
        @param iso    The same instant, formatted.
        @param unit   Which board.
        @param clean  The line, escape codes and newline already removed.

        The caller must hold the lock.
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
        """
        @brief Flush and close all three files. Safe to call once, at the end.
        """
        ## @cond
        with self.lock:
            for f in (self.raw, self.mfile, self.efile):
                try:
                    f.flush()
                    f.close()
                except Exception:
                    pass
        ## @endcond


def garbled(line):
    """
    @brief True when a line is mostly replacement characters.

    Reading a board below its console baud turns the stream into accent soup,
    and the soup rarely contains a newline, so it surfaces as one huge line
    rather than as an error. This is what that looks like, so it can be said
    out loud instead of quietly capturing nothing for an hour.

    @param line  One decoded line.
    @return True if more than a tenth of it is unreadable.
    """
    if not line.strip():
        return False
    bad = sum(1 for c in line if c == "�" or
              (ord(c) < 32 and c not in "\t"))
    return bad > len(line) * 0.1


def reader(session, unit, port, baud):
    """
    @brief One serial port, reopened for as long as the soak runs.

    A board reset drops the adapter on some cables, and a soak that ends
    because somebody nudged a USB plug at hour three is not a soak. This
    reconnects and says so on the console, which also lands in raw.log as a
    marker.

    @param session  Where the lines go.
    @param unit     The name to file them under.
    @param port     The device path.
    @param baud     Its console rate; see the module docstring, it is per unit.
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
                        # A garbled stream may never emit a newline, so cap
                        # the buffer rather than grow it for four hours.
                        # A garbled stream may never emit a newline, so cap
                        # the buffer rather than grow it for four hours.
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


## @brief Pseudo-unit the 2.4 GHz sweeps are filed under. Not a board, and
#         analyse.py drops it from its per-unit loops.
AIR_UNIT = "air"

## @brief The three non-overlapping 2.4 GHz channels.
AIR_CANDIDATES = (1, 6, 11)
## @brief The overlap rule: channels sit 5 MHz apart and are 22 MHz wide, so
#         anything within four of a candidate lands on top of it.
#
# Identical to survey_channel() in hub_s3/main/net.c, deliberately. The hub
# prints one of these lines at boot from its own antenna, and the two are only
# comparable if they are scored the same way.
AIR_OVERLAP = 4

## @brief Our own SoftAP, which this scan must not count as interference.
#
# It is the loudest thing in the room from where the laptop sits, so leaving it
# in pins whichever channel the hub chose at maximum occupancy and makes the
# whole metric read backwards. The hub's own survey has no such problem: it
# scans in STA mode before the AP is started and physically cannot see itself.
# Matches AP_SSID in hub_s3/main/hub.h.
AIR_OWN_SSID = "dancefloor"


def air_scan():
    """
    @brief One 2.4 GHz sweep, through nmcli.

    The instrument for the one variable a soak otherwise never records: what
    else was on the air. Cheap enough to run for hours beside a capture.

    Four things it is important not to over-read.

    IT DOES NOT MEASURE AIRTIME. A beacon says a network is PRESENT, not that
    it is busy, so a quiet neighbour starting a large transfer looks like
    nothing at all here. What it catches is a network appearing, changing
    channel, or changing level. `iw survey dump` gives real channel busy time
    and is the upgrade if this proves too blunt; it needs root and the
    interface parked on the hub's channel.

    IT MEASURES AN INTERFERER AS LOSS OF DECODE, NOT AS A RISE IN LEVEL. A
    passive scan only counts the beacons it successfully decodes, so somebody
    shouting on a channel makes that channel's NEIGHBOURS vanish from the list
    instead of making it read louder. It shows up as the count falling -- at
    the limit ch{c}-nets 0 with ch{c}-dbm -100, which is the floor substituted
    below and not a measured level.

    IT IS NOISY, several dB from sweep to sweep on a busy channel, because a
    single scan does not always catch every AP. A quiet channel is far
    steadier. So read a trend across several sweeps rather than one reading.

    ITS INTERVAL IS NOT ABOUT COST. NetworkManager rate-limits `nmcli dev wifi
    rescan`, so sweeping faster returns the CACHED list, and one real blackout
    would then repeat across several sweeps and read as a long one --
    manufacturing evidence for the very thing the sweep exists to test. A
    sweep is only worth anything while it is an independent look at the band.

    @return (nets_seen, {channel: (dbm, count)}).
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
        # nmcli reports SIGNAL as 0-100 QUALITY, not dBm. This is
        # wpa_supplicant's usual inverse, and it is APPROXIMATE -- the hub's
        # own survey line is the calibrated one. Good enough for tracking
        # change over a run, which is all this is asked for.
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
    """
    @brief Sweep the band every @p interval seconds for as long as the soak runs.

    Emits an ESP-shaped line so Session.emit() parses it with no special case,
    and files it under AIR_UNIT rather than in a second file, so that a sweep
    and a hub status line can be read against each other by wall_s.

    Retires quietly on the first failure. A soak must not die, or fill its
    log, because the laptop's WiFi went away.

    @param session   Where the lines go.
    @param interval  Seconds between sweeps.
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
            # One notice, then stop.
            session.record(AIR_UNIT, f"--- capture: air scan failed ({e}), "
                                     f"off for this session ---")
            return
        session.stop.wait(interval)


def replay(outdir):
    """
    @brief Rebuild metrics.csv and events.csv from an existing session's raw.log.

    For when the extraction has learned something a session predates: without
    this, an improvement to KINDS or EXTRA can never reach the runs a change
    is being compared against.

    raw.log is not touched, and the two derived files are rewritten from
    nothing rather than appended to, so replaying twice is not additive.

    @param outdir  An existing session directory.
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
            # This script's own banner lines ("--- capture: opened ... ---")
            # have the same four fields but no ESP prefix, so emit() drops
            # them. A line with FEWER fields is a truncated write, and is
            # counted rather than guessed at.
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
    """
    @brief Parse the arguments, start one thread per unit, and run until Ctrl-C.
    """
    ap = argparse.ArgumentParser(
        description="Capture every unit's console to CSV for a soak run.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="example, all four boards:\n"
               "  %(prog)s \\\n"
               "    --unit hub=/dev/serial/by-id/usb-FTDI_...-if00-port0:921600 \\\n"
               "    --unit bt_bridge=/dev/serial/by-id/usb-Silicon_Labs_... \\\n"
               "    --unit sat_classic=/dev/ttyUSB0 --unit sat_s3=/dev/ttyACM0\n\n"
               "CAPTURE bt_bridge. It is the A2DP source's only witness. Without\n"
               "it, a hub that starves and a timeline that jumps cannot be told\n"
               "apart from the phone having stopped sending -- status_led.c logs\n"
               "the audio gap and sbc_spi.c the drops, but only onto that board's\n"
               "own console.\n\n"
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


## @cond
# The entry point, not API.
if __name__ == "__main__":
    main()
## @endcond
