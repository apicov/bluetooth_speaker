#!/usr/bin/env python3
"""
Centralised log/telemetry collector for the dancefloor bench.

Joins the hub's "dancefloor" WiFi (do that at the OS level first) and gathers
logs from every WiFi unit into one merged, timestamped stream, while also
reading the bt_bridge's console UART over USB -- the bridge has no WiFi, so its
logs only reach the laptop by wire.

Topology: each unit ships to the HUB; the hub relays its own and every
satellite's logs/telemetry to this laptop. So every UDP packet arrives from
192.168.4.1, and the in-band `role` / `src_ip` fields are what tell units apart
(not the UDP source address). The bridge is the one source that comes in over
serial, parsed from its ESP_LOG console lines.

Outputs, per session directory:
  all.jsonl   one record per log line: {ts, src, level, tag, msg}
  health.csv  one row per MSG_HEALTH per source, numeric columns for plotting

Requires pyserial only if --bridge is used (pip install pyserial); the UDP path
is plain stdlib.

The firmware side compiles out unless CONFIG_DANCEFLOOR_WIFI_LOGS is set, so a
production hub/satellite sends nothing this script can receive.
"""

import argparse
import csv
import json
import os
import socket
import struct
import sys
import threading
import time

# --- wire format (must match components/dancefloor_sync/include/sync_proto.h) --

PORT = 5001
MSG_LOG, MSG_HEALTH, MSG_LOG_SUB = 9, 10, 11
LOG_ROLE_HUB, LOG_ROLE_SAT = 0, 1
LOG_SUB_MAGIC = 0x4C4F4731  # "LOG1"

# log_msg_t fixed header: type level role tag_len msg_len seq src_ip tag[16]
LOG_HDR = struct.Struct("<BBBBHII16s")          # 30 bytes
# health_msg_t: 4B, seq, src_ip, uptime(Q), then 22 x uint32 (heap/stack/counters)
HEALTH = struct.Struct("<BBBBIIQ" + "I" * 22)   # 108 bytes
HEALTH_COLS = [
    "heap_cur", "heap_min", "heap_win", "heap_largest", "hw_play", "hw_mon",
    "underruns", "reanchors_or_restarts", "splices", "retunes", "retunes_refused",
    "gaps_or_sta_left", "wifi_drops_or_oversize", "alloc_fail", "phase_drop",
    "short_reads", "short_frames", "ring_full_or_sta_dropped",
    "upgrades_or_sta_nolease", "anchors_refused_or_timeout",
    "log_dropped", "log_no_dest",
]

# The C side pins the same two numbers in test_sync_proto.c ("the log/health
# message sizes are pinned"). Asserted here so an edit to a format string above
# fails on the spot rather than silently unpacking garbage off the wire.
assert LOG_HDR.size == 30 and HEALTH.size == 108, "wire format drifted from sync_proto.h"

SUB_PKT = struct.pack("<BI", MSG_LOG_SUB, LOG_SUB_MAGIC)  # log_sub_msg_t

# ANSI colours keyed by the level char ESP_LOG prints.
COLOR = {"E": "\033[31m", "W": "\033[33m", "I": "\033[36m", "D": "\033[2m"}
SRC_COLOR = "\033[35m"
RESET = "\033[0m"


def ip_str(src_ip):
    """src_ip is the wire bytes (network order) read out little-endian."""
    return socket.inet_ntoa(struct.pack("<I", src_ip))


def parse_console_line(line):
    """Split an ESP_LOG console composite 'L (ts) tag: msg' into (level, tag, msg).

    The same shape the firmware emits on UART and the collector reads off the
    bridge's serial port. A leading ANSI colour escape is skipped, since the
    bridge is a separate build and may have CONFIG_LOG_COLORS on. Returns
    ('?', '', line) if the shape is not met."""
    line = line.rstrip("\r\n")
    if line.startswith("\033["):
        m = line.find("m")
        if m < 0:
            return "?", "", line
        line = line[m + 1:]
        if line.endswith("\033[0m"):
            line = line[:-4]
    if len(line) < 4 or line[1:3] != " (" or ")" not in line:
        return "?", "", line
    level = line[0]
    after = line.split(") ", 1)
    if len(after) != 2:
        return "?", "", line
    rest = after[1]
    sep = rest.find(": ")
    if sep < 0:
        return level, "", rest
    return level, rest[:sep], rest[sep + 2:]


class Session:
    """Thread-safe merged sink: coloured console + jsonl + csv."""

    def __init__(self, out_dir):
        os.makedirs(out_dir, exist_ok=True)
        self.jsonl = open(os.path.join(out_dir, "all.jsonl"), "a", buffering=1)
        self.csvf = open(os.path.join(out_dir, "health.csv"), "a", newline="", buffering=1)
        self.csv = csv.writer(self.csvf)
        # Append mode: a re-run against the same --out dir continues the file, so
        # the header goes in only when it is empty.
        if self.csvf.tell() == 0:
            self.csv.writerow(["ts", "source", "role", "clock_src", "uptime_s"] + HEALTH_COLS)
        self.lock = threading.Lock()
        self.t0 = time.monotonic()

    def _ts(self):
        return time.strftime("%H:%M:%S")

    def log(self, source, level, tag, msg):
        ts = self._ts()
        rec = {"ts": ts, "src": source, "level": level, "tag": tag, "msg": msg}
        with self.lock:
            self.jsonl.write(json.dumps(rec, ensure_ascii=False) + "\n")
            col = COLOR.get(level, "")
            tagstr = f"{tag}: " if tag else ""
            print(f"{col}[{ts}]{RESET} {SRC_COLOR}{source:<14}{RESET} "
                  f"{col}{level}{RESET} {tagstr}{msg}")

    def health(self, source, role, clock_src, uptime_s, values):
        with self.lock:
            self.csv.writerow([self._ts(), source, role, clock_src, uptime_s] + list(values))
            # A short line so a live console also shows health arriving.
            print(f"\033[32m[{self._ts()}]{RESET} {SRC_COLOR}{source:<14}{RESET} "
                  f"HEALTH up {uptime_s}s heap {values[0]} (win {values[2]}) "
                  f"underruns {values[6]}")


def udp_loop(sock, session):
    while True:
        try:
            pkt, _ = sock.recvfrom(2048)
        except (socket.timeout, TimeoutError):
            # The socket's 1 s timeout, not an error: quiet units are the normal
            # case. TimeoutError is an OSError subclass, so it must be caught
            # first or the clause below would end the capture on the first
            # idle second.
            continue
        except OSError:
            return
        if not pkt:
            continue
        t = pkt[0]
        if t == MSG_LOG and len(pkt) >= LOG_HDR.size:
            (_ty, level, role, tag_len, msg_len, _seq, src_ip, tag) = \
                LOG_HDR.unpack(pkt[:LOG_HDR.size])
            msg = pkt[LOG_HDR.size:LOG_HDR.size + msg_len].decode("utf-8", "replace")
            tag = tag[:tag_len].decode("utf-8", "replace")
            source = "hub" if role == LOG_ROLE_HUB else f"sat-{ip_str(src_ip)}"
            session.log(source, chr(level), tag, msg)
        elif t == MSG_HEALTH and len(pkt) >= HEALTH.size:
            (_ty, role, clock_src, _rsv, _seq, src_ip, uptime_s, *vals) = \
                HEALTH.unpack(pkt[:HEALTH.size])
            source = "hub" if role == LOG_ROLE_HUB else f"sat-{ip_str(src_ip)}"
            session.health(source, role, clock_src, uptime_s, vals)


def serial_loop(port, baud, session):
    try:
        import serial  # pyserial
    except ImportError:
        print("pyserial not installed -- bridge serial tap disabled "
              "(pip install pyserial)", file=sys.stderr)
        return
    while True:
        try:
            with serial.Serial(port, baudrate=baud, timeout=1) as ser:
                while True:
                    raw = ser.readline()
                    if not raw:
                        continue
                    level, tag, msg = parse_console_line(raw.decode("utf-8", "replace"))
                    session.log("bridge", level, tag, msg)
        except OSError as e:
            print(f"bridge serial {port}: {e} -- retrying in 3s", file=sys.stderr)
            time.sleep(3)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hub", default="192.168.4.1", help="hub IP to send MSG_LOG_SUB to")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--bridge", default="/dev/ttyUSB0", help="bt_bridge serial port")
    ap.add_argument("--baud", type=int, default=115200, help="bt_bridge console baud")
    ap.add_argument("--no-bridge", action="store_true", help="do not tap the bridge serial")
    ap.add_argument("--out", default=None, help="session output dir (default: logs-<timestamp>)")
    ap.add_argument("--sub-interval", type=float, default=5.0, help="MSG_LOG_SUB cadence (s)")
    args = ap.parse_args()

    out_dir = args.out or "logs-" + time.strftime("%Y%m%d-%H%M%S")
    session = Session(out_dir)
    print(f"writing to {out_dir}/  (all.jsonl, health.csv)")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.port))
    sock.settimeout(1.0)

    threading.Thread(target=udp_loop, args=(sock, session), daemon=True).start()
    if not args.no_bridge:
        threading.Thread(target=serial_loop, args=(args.bridge, args.baud, session),
                         daemon=True).start()

    print(f"collecting on UDP/{args.port}, registering with hub {args.hub} "
          f"every {args.sub_interval:g}s  (Ctrl-C to stop)")
    try:
        while True:
            try:
                sock.sendto(SUB_PKT, (args.hub, args.port))
            except OSError:
                pass  # not on the AP yet; try again next interval
            time.sleep(args.sub_interval)
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
