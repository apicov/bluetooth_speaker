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


ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")

ESP_LINE = re.compile(r"^([IWEDV]) \((\d+)\) ([^:]{1,24}): (.*)$")

KEYNUM = re.compile(r"([A-Za-z][A-Za-z0-9_-]*)\s+([+-]?\d+)")

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
    ("audio stopped", "audio_gap"),
    ("audio resumed", "audio_gap"),
    ("queue full:", "bridge_drop"),
    ("payload past the link ceiling:", "bridge_drop"),
    ("hub never raised the handshake:", "bridge_drop"),
    ("track change (#", "bridge_track"),
]

EXTRA = {
    "trim":       [(re.compile(r"TRIM:\s*([+-]?\d+)\s*Hz"), "trim_hz")],
    "status":     [(re.compile(r"\((\d+) ms\)"), "ring_ms"),
                   (re.compile(r"tx-fail \d+ \((\d+) audio"), "tx_fail_audio"),
                   (re.compile(r"\| gaps (\d+)/\d+/\d+/\d+"), "burst-gap-lt25"),
                   (re.compile(r"\| gaps \d+/(\d+)/\d+/\d+"), "burst-gap-25-75"),
                   (re.compile(r"\| gaps \d+/\d+/(\d+)/\d+"), "burst-gap-75-150"),
                   (re.compile(r"\| gaps \d+/\d+/\d+/(\d+)"), "burst-gap-gt150")],
    "health":     [(re.compile(r"\((\d+) refused\)"), "retunes_refused")],
    "divergence": [(re.compile(r"->\s*([+-]?\d+) ms apart"), "apart_ms")],
    "servo":      [(re.compile(r"\((\d+) frames/s\)"), "frames_per_s")],

    "audio_gap":    [(re.compile(r"resumed after (\d+) ms"), "audio-gap-ms"),
                     (re.compile(r"last packet (\d+) ms ago"), "audio-stopped-ms")],
    "bridge_drop":  [(re.compile(r":\s*(\d+) dropped"), "bridge-dropped")],
    "bridge_track": [(re.compile(r"#(\d+)"), "bridge-track-id")],
}

EVENT_KINDS = {
    "underrun", "timeline_start", "timeline_restart", "playback_start",
    "stream_start", "track", "volume", "retune", "splice", "boundary",
    "divergence", "crippled", "alloc_fail", "refill",
}


def classify(body):
    for prefix, kind in KINDS:
        if body.startswith(prefix):
            return kind, (len(prefix) if prefix.rstrip().endswith(":") else 0)
    return "other", 0


def numbers(kind, body, skip=0):
    out = []
    seen = set()
    for key, val in KEYNUM.findall(body[skip:]):
        k = key.lower()
        if k in seen:
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
    if not line.strip():
        return False
    bad = sum(1 for c in line if c == "�" or
              (ord(c) < 32 and c not in "\t"))
    return bad > len(line) * 0.1


def reader(session, unit, port, baud):
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
                        if len(buf) > 65536:
                            session.record(unit, buf.decode("utf-8",
                                                             errors="replace"))
                            buf = b""
        except serial.SerialException as e:
            if session.stop.is_set():
                return
            session.record(unit, f"--- capture: {port} lost ({e}), retrying ---")
            time.sleep(2.0)
        except Exception as e:
            if session.stop.is_set():
                return
            session.record(unit, f"--- capture: {port} error ({e}), retrying ---")
            time.sleep(2.0)


AIR_UNIT = "air"

AIR_CANDIDATES = (1, 6, 11)
AIR_OVERLAP = 4

AIR_OWN_SSID = "dancefloor"


def air_scan():
    try:
        subprocess.run(["nmcli", "dev", "wifi", "rescan"],
                       capture_output=True, timeout=20)
    except Exception:
        pass
    out = subprocess.run(["nmcli", "-t", "-f", "SSID,CHAN,SIGNAL",
                          "dev", "wifi", "list"],
                         capture_output=True, text=True, timeout=20)
    if out.returncode != 0:
        raise RuntimeError((out.stderr or "nmcli failed").strip()[:120])

    power = {c: 0.0 for c in AIR_CANDIDATES}
    nets = {c: 0 for c in AIR_CANDIDATES}
    seen = 0
    for line in out.stdout.splitlines():
        parts = line.rsplit(":", 2)
        if len(parts) != 3:
            continue
        if parts[0] == AIR_OWN_SSID:
            continue
        try:
            chan, sig = int(parts[1]), int(parts[2])
        except ValueError:
            continue
        if chan > 14:
            continue
        seen += 1
        dbm = sig / 2.0 - 100.0
        for c in AIR_CANDIDATES:
            if abs(chan - c) <= AIR_OVERLAP:
                power[c] += 10.0 ** (dbm / 10.0)
                nets[c] += 1
    return seen, {c: (int(round(10.0 * math.log10(power[c]))) if power[c] > 0
                      else -100, nets[c])
                  for c in AIR_CANDIDATES}


def air_watcher(session, interval):
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
        except Exception as e:
            session.record(AIR_UNIT, f"--- capture: air scan failed ({e}), "
                                     f"off for this session ---")
            return
        session.stop.wait(interval)


def replay(outdir):
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
        if ":" in port:
            head, _, tail = port.rpartition(":")
            if head and tail.isdigit():
                port, baud = head, int(tail)
        units.append((name.strip(), port, baud))
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
