#!/usr/bin/env python3
"""
Centralised log and telemetry collector for the dancefloor bench.

Gathers logs from every WiFi unit into one merged, timestamped stream, while
also reading the bt_bridge's console UART over USB -- the bridge has no WiFi,
so its logs only reach the laptop by wire. Join the hub's SoftAP at the OS
level first.

Topology: each unit ships to the HUB, and the hub relays its own and every
satellite's logs and telemetry here. So every UDP packet arrives from the hub's
address, and the in-band `role` and `src_ip` fields are what tell units apart
-- not the UDP source address.

Two files per session directory:

  all.jsonl   one record per log line: {ts, src, level, tag, msg}
  health.csv  one row per MSG_HEALTH per source, numeric columns for plotting

Registration is by subscription: this sends a MSG_LOG_SUB to the hub on a
timer, and the hub relays only while that is current.

Requires pyserial only for --bridge; the UDP path is plain stdlib. The firmware
side compiles out unless CONFIG_DANCEFLOOR_WIFI_LOGS is set, so a production
hub or satellite sends nothing this can receive.
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


# The wire format below must match
# components/dancefloor_sync/include/sync_proto.h.

## @brief The sync protocol's UDP port.
PORT = 5001
## @brief One relayed log line.
MSG_LOG = 9
## @brief One health/telemetry record.
MSG_HEALTH = 10
## @brief This collector's subscription; the hub relays only while it is current.
MSG_LOG_SUB = 11
## @brief The `role` value for the hub itself.
LOG_ROLE_HUB = 0
## @brief The `role` value for a satellite. Which one is in `src_ip`.
LOG_ROLE_SAT = 1
## @brief "LOG1", so a stray datagram on this port cannot subscribe.
LOG_SUB_MAGIC = 0x4C4F4731

## @brief log_msg_t's fixed header: type, level, role, tag_len, msg_len, seq,
#         src_ip, tag[16]. The tag and message follow it.
LOG_HDR = struct.Struct("<BBBBHII16s")
## @brief health_msg_t: four bytes, seq, src_ip, uptime, then the counters
#         named in HEALTH_COLS.
HEALTH = struct.Struct("<BBBBIIQ" + "I" * 22)
## @brief Column names for the counters in a health_msg_t, in wire order.
#
# Several are shared between the hub and a satellite under different meanings,
# which is why those names carry both halves.
HEALTH_COLS = [
    "heap_cur", "heap_min", "heap_win", "heap_largest", "hw_play", "hw_mon",
    "underruns", "reanchors_or_restarts", "splices", "retunes", "retunes_refused",
    "gaps_or_sta_left", "wifi_drops_or_oversize", "alloc_fail", "phase_drop",
    "short_reads", "short_frames", "ring_full_or_sta_dropped",
    "upgrades_or_sta_nolease", "anchors_refused_or_timeout",
    "log_dropped", "log_no_dest",
]

# The C side pins the same two sizes in test_sync_proto.c. Asserted here so an
# edit to a format string above fails on the spot rather than silently
# unpacking garbage off the wire.
assert LOG_HDR.size == 30 and HEALTH.size == 108, "wire format drifted from sync_proto.h"

## @brief The subscription datagram, which never varies. A log_sub_msg_t.
SUB_PKT = struct.pack("<BI", MSG_LOG_SUB, LOG_SUB_MAGIC)

## @brief ANSI colours keyed by the level character ESP_LOG prints.
COLOR = {"E": "\033[31m", "W": "\033[33m", "I": "\033[36m", "D": "\033[2m"}
## @brief The colour the source column is printed in.
SRC_COLOR = "\033[35m"
## @brief End of any of the above.
RESET = "\033[0m"


def ip_str(src_ip):
    """
    @brief Format a src_ip field as dotted quad.
    @param src_ip  The wire bytes, which are network order read out
                   little-endian.
    @return The address.
    """
    return socket.inet_ntoa(struct.pack("<I", src_ip))


def parse_console_line(line):
    """
    @brief Split an ESP_LOG console line `L (ts) tag: msg` into its parts.

    The same shape the firmware emits on UART, read off the bridge's serial
    port. A leading ANSI colour escape is skipped, since the bridge is a
    separate build and may have CONFIG_LOG_COLORS on.

    @param line  One line as it arrived.
    @return (level, tag, msg); ('?', '', line) when the shape is not met, so a
            line that does not parse is still kept whole.
    """
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
    """
    @brief Thread-safe merged sink: coloured console, jsonl and csv.
    """

    def __init__(self, out_dir):
        """
        @brief Open both files in append mode and write the CSV header once.
        @param out_dir  The session directory; created if it does not exist.
        """
        os.makedirs(out_dir, exist_ok=True)
        ## @brief One JSON record per log line, line-buffered.
        self.jsonl = open(os.path.join(out_dir, "all.jsonl"), "a", buffering=1)
        ## @brief One row per health record, line-buffered.
        self.csvf = open(os.path.join(out_dir, "health.csv"), "a", newline="", buffering=1)
        ## @brief Writer over csvf.
        self.csv = csv.writer(self.csvf)
        # Append mode: a re-run against the same --out directory continues the
        # file, so the header goes in only when it is empty.
        if self.csvf.tell() == 0:
            self.csv.writerow(["ts", "source", "role", "clock_src", "uptime_s"] + HEALTH_COLS)
        ## @brief Held across every write, so the UDP and serial threads
        #         cannot interleave mid-line.
        self.lock = threading.Lock()
        ## @brief When the session started.
        self.t0 = time.monotonic()

    def _ts(self):
        """
        @brief Wall clock, to the second, as it appears in both files.
        @return The formatted time.
        """
        return time.strftime("%H:%M:%S")

    def log(self, source, level, tag, msg):
        """
        @brief Record one log line and echo it to the console in colour.
        @param source  Which unit; "hub", "bridge", or "sat-<ip>".
        @param level   The ESP_LOG level character.
        @param tag     The ESP_LOG tag, possibly empty.
        @param msg     The line itself.
        """
        ts = self._ts()
        rec = {"ts": ts, "src": source, "level": level, "tag": tag, "msg": msg}
        ## @cond
        # Doxygen's Python scanner reads `with self.<attr>:` as a member
        # declaration and attaches the next identifier it parses to this class;
        # hiding the block costs nothing, since statements inside a method are
        # not documentation.
        with self.lock:
            self.jsonl.write(json.dumps(rec, ensure_ascii=False) + "\n")
            col = COLOR.get(level, "")
            tagstr = f"{tag}: " if tag else ""
            print(f"{col}[{ts}]{RESET} {SRC_COLOR}{source:<14}{RESET} "
                  f"{col}{level}{RESET} {tagstr}{msg}")
        ## @endcond

    def health(self, source, role, clock_src, uptime_s, values):
        """
        @brief Record one health record and print a one-line summary.
        @param source     Which unit.
        @param role       LOG_ROLE_HUB or LOG_ROLE_SAT.
        @param clock_src  Which clock that unit is following.
        @param uptime_s   Its uptime.
        @param values     The counters, in HEALTH_COLS order.
        """
        ## @cond
        with self.lock:
            self.csv.writerow([self._ts(), source, role, clock_src, uptime_s] + list(values))
            # A short line, so a live console also shows health arriving.
            print(f"\033[32m[{self._ts()}]{RESET} {SRC_COLOR}{source:<14}{RESET} "
                  f"HEALTH up {uptime_s}s heap {values[0]} (win {values[2]}) "
                  f"underruns {values[6]}")
        ## @endcond


def udp_loop(sock, session):
    """
    @brief Read relayed logs and health records until the socket closes.
    @param sock     Bound to PORT, with a one second timeout.
    @param session  Where the records go.
    """
    while True:
        try:
            pkt, _ = sock.recvfrom(2048)
        except (socket.timeout, TimeoutError):
            # The socket's own timeout, not an error: quiet units are the
            # normal case. TimeoutError is an OSError subclass, so it must be
            # caught first or the clause below would end the capture on the
            # first idle second.
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
    """
    @brief Read the bridge's console over USB, reconnecting for as long as the
           session runs.

    The bridge has no WiFi, so this is the only way its logs reach the laptop.
    Retires quietly if pyserial is missing, since the UDP half still works.

    @param port     The device path.
    @param baud     Its console rate.
    @param session  Where the lines go.
    """
    try:
        import serial
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
    """
    @brief Open the socket, start both reader threads, and re-subscribe on a
           timer until Ctrl-C.
    """
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


## @cond
# The entry point, not API.
if __name__ == "__main__":
    main()
## @endcond
