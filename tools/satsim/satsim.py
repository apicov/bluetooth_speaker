#!/usr/bin/env python3
"""
N fake satellites on a laptop, to load the hub's unicast fan-out.

This is a LOAD GENERATOR, not an instrument. It makes the hub believe N more
speakers are on the floor so it fans audio and analysis frames out to N more
destinations; what that does to the real satellites is the measurement, and it
is taken on them.

N SOURCE IPs, NOT N PORTS. client_seen() (hub_s3/main/clients.c) matches on
sin_addr.s_addr alone, so N sockets sharing the laptop's one address collapse
into a single client slot -- the hub would fan out to one destination and this
would look like it was working. Each fake satellite therefore needs its own
address, added as an alias on the interface already associated to the SoftAP.
The script says which commands to run; they need root, so it does not run them.

Registration is by probe: a unit that has probed within CLIENT_TIMEOUT_US is on
the send list. So each fake satellite sends the same MSG_TIME_REQ a real one
does, at the same 250 ms period, and that is the whole of what puts it on the
list.
"""

import argparse
import errno
import ipaddress
import select
import socket
import struct
import subprocess
import sys
import time

SYNC_PORT = 5001                # sync_proto.h
PROBE_PERIOD = 0.250            # PROBE_PERIOD_MS, satellite/main/sat.h

MSG_TIME_REQ = 1
MSG_TIME_RSP = 2
MSG_AUDIO = 4
MSG_FRAME = 8

# All packed, little-endian. See components/dancefloor_sync/include/sync_proto.h
TIME_MSG = struct.Struct("<BIqqq")              # type, seq, t1, t2, t3
AUDIO_HDR = struct.Struct("<BBBBHIIIq")         # ..., payload_len, seq, rate, frames, play_at
FRAME_HDR = struct.Struct("<BBB")               # type, len, count


def now_us():
    return time.monotonic_ns() // 1000


class Sat:
    """One fake satellite: a socket on its own address, and what it has seen."""

    def __init__(self, ip):
        self.ip = ip
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((ip, SYNC_PORT))
        self.sock.setblocking(False)
        self.probe_seq = 0
        self.audio = 0
        self.frames = 0
        self.lost = 0
        self.replies = 0
        self.last_seq = None

    def probe(self, hub):
        self.probe_seq += 1
        msg = TIME_MSG.pack(MSG_TIME_REQ, self.probe_seq, now_us(), 0, 0)
        try:
            self.sock.sendto(msg, (hub, SYNC_PORT))
        except OSError:
            pass                # a probe lost to a full queue is a probe lost

    def recv(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
            except BlockingIOError:
                return
            except OSError:
                return
            if not data:
                return
            kind = data[0]
            if kind == MSG_AUDIO and len(data) >= AUDIO_HDR.size:
                self.audio += 1
                seq = AUDIO_HDR.unpack_from(data)[5]
                if self.last_seq is not None:
                    gap = (seq - self.last_seq) & 0xFFFFFFFF
                    if 1 < gap < 1000:
                        self.lost += gap - 1
                self.last_seq = seq
            elif kind == MSG_FRAME and len(data) >= FRAME_HDR.size:
                self.frames += 1
            elif kind == MSG_TIME_RSP:
                self.replies += 1


def local_v4():
    """Every IPv4 address on this machine, as (addr, iface)."""
    out = subprocess.run(["ip", "-o", "-4", "addr", "show"],
                         capture_output=True, text=True).stdout
    found = []
    for line in out.splitlines():
        f = line.split()
        if len(f) >= 4:
            found.append((f[3].split("/")[0], f[1]))
    return found


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-n", type=int, default=8, help="fake satellites (default 8)")
    ap.add_argument("--hub", default="192.168.4.1", help="hub address")
    ap.add_argument("--base", default="192.168.4.100",
                    help="first alias address; the DHCP pool starts at .2 and "
                         "holds 15, so stay well above it (default .100)")
    ap.add_argument("--report", type=float, default=5.0, help="seconds between lines")
    args = ap.parse_args()

    base = ipaddress.IPv4Address(args.base)
    ips = [str(base + i) for i in range(args.n)]

    # Ask the kernel rather than parsing `ip addr`: binding is the thing that has
    # to work, and an address that cannot be bound is the failure worth naming
    # whatever the reason.
    missing = []
    sats = []
    for ip in ips:
        try:
            sats.append(Sat(ip))
        except OSError as e:
            if e.errno != errno.EADDRNOTAVAIL:
                raise
            missing.append(ip)
    if missing:
        for s in sats:
            s.sock.close()
        iface = next((i for a, i in local_v4()
                      if ipaddress.IPv4Address(a) in ipaddress.IPv4Network("192.168.4.0/24")),
                     "<wlan>")
        print(f"{len(missing)} of {args.n} addresses are not on this machine.")
        print("Run these (needs root), then start again:\n")
        for ip in missing:
            print(f"  sudo ip addr add {ip}/24 dev {iface}")
        print("\nTo undo afterwards: same lines with 'del' instead of 'add'.")
        if iface == "<wlan>":
            print("\nNo 192.168.4.x address here -- is this machine on the hub's AP?")
        return 1

    socks = [s.sock for s in sats]
    print(f"{args.n} fake satellites on {ips[0]}..{ips[-1]}, probing {args.hub} "
          f"every {PROBE_PERIOD * 1000:.0f} ms. Ctrl-C to stop.")

    next_probe = time.monotonic()
    next_report = time.monotonic() + args.report
    prev = [(0, 0, 0)] * args.n

    try:
        while True:
            t = time.monotonic()
            if t >= next_probe:
                for s in sats:
                    s.probe(args.hub)
                next_probe += PROBE_PERIOD

            ready, _, _ = select.select(socks, [], [], max(0.0, next_probe - t))
            for sock in ready:
                sats[socks.index(sock)].recv()

            t = time.monotonic()
            if t >= next_report:
                dt = args.report
                cur = [(s.audio, s.frames, s.lost) for s in sats]
                rate = [(c[0] - p[0]) / dt for c, p in zip(cur, prev)]
                fr = [(c[1] - p[1]) / dt for c, p in zip(cur, prev)]
                lost = [c[2] - p[2] for c, p in zip(cur, prev)]
                registered = sum(1 for s in sats if s.replies)
                print(f"[{time.strftime('%H:%M:%S')}] registered {registered}/{args.n} | "
                      f"audio/s {min(rate):.0f}-{max(rate):.0f} (sum {sum(rate):.0f}) | "
                      f"frames/s {min(fr):.0f}-{max(fr):.0f} | "
                      f"lost this window {sum(lost)} | lost total {sum(s.lost for s in sats)}")
                prev = cur
                next_report += args.report
    except KeyboardInterrupt:
        print()

    print(f"{'sat':<16} {'audio':>9} {'lost':>7} {'frames':>8} {'replies':>8}")
    for s in sats:
        print(f"{s.ip:<16} {s.audio:>9} {s.lost:>7} {s.frames:>8} {s.replies:>8}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
