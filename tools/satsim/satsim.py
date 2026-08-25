#!/usr/bin/env python3

import argparse
import errno
import ipaddress
import select
import socket
import struct
import subprocess
import sys
import time

SYNC_PORT = 5001
PROBE_PERIOD = 0.250

MSG_TIME_REQ = 1
MSG_TIME_RSP = 2
MSG_AUDIO = 4
MSG_FRAME = 8
MSG_AUDIO_FEC = 14
AUDIO_FMT_SBC = 1

TIME_MSG = struct.Struct("<BIqqq")
AUDIO_HDR = struct.Struct("<BBBBHIIIq")
FRAME_HDR = struct.Struct("<BBB")
FEC_HDR = struct.Struct("<BBHI")

assert AUDIO_HDR.size == 26, "audio_msg_t's header moved; this script is stale"
assert FEC_HDR.size == 8, "AUDIO_FEC_HDR_BYTES moved; this script is stale"


def xor_bytes(a, b):
    n = max(len(a), len(b))
    return (int.from_bytes(a.ljust(n, b"\0"), "little")
            ^ int.from_bytes(b.ljust(n, b"\0"), "little")).to_bytes(n, "little")


def codeword(raw):
    plen = struct.unpack_from("<H", raw, 4)[0]
    return raw[:AUDIO_HDR.size + plen]


def fec_extract(acc, span, want_seq):
    if span < AUDIO_HDR.size or len(acc) < AUDIO_HDR.size:
        return None
    kind, fmt, _marker, _restart, plen, seq, _rate, _frames, _at = \
        AUDIO_HDR.unpack_from(acc)
    if kind != MSG_AUDIO or fmt != AUDIO_FMT_SBC or seq != want_seq:
        return None
    if AUDIO_HDR.size + plen > span:
        return None
    return acc[:AUDIO_HDR.size + plen]


def now_us():
    return time.monotonic_ns() // 1000


class Sat:

    def __init__(self, ip, verify=False):
        self.ip = ip
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
        self.rcvbuf = self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        self.sock.bind((ip, SYNC_PORT))
        self.sock.setblocking(False)
        self.probe_seq = 0
        self.audio = 0
        self.frames = 0
        self.lost = 0
        self.replies = 0
        self.parity = 0
        self.last_seq = None

        self.fec = FecVerifier() if verify else None

    def probe(self, hub):
        self.probe_seq += 1
        msg = TIME_MSG.pack(MSG_TIME_REQ, self.probe_seq, now_us(), 0, 0)
        try:
            self.sock.sendto(msg, (hub, SYNC_PORT))
        except OSError:
            pass

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
                if self.fec:
                    self.fec.note(seq, data)
            elif kind == MSG_AUDIO_FEC and len(data) >= FEC_HDR.size:
                self.parity += 1
                if self.fec:
                    self.fec.check(data)
            elif kind == MSG_FRAME and len(data) >= FRAME_HDR.size:
                self.frames += 1
            elif kind == MSG_TIME_RSP:
                self.replies += 1


class FecVerifier:

    def __init__(self):
        self.pkts = {}
        self.groups = 0
        self.checked = 0
        self.matched = 0
        self.mismatch = 0
        self.repaired = 0
        self.unverifiable = 0

    def note(self, seq, data):
        self.pkts[seq] = data
        if len(self.pkts) > 256:
            for old in sorted(self.pkts)[:128]:
                del self.pkts[old]

    def check(self, data):
        _kind, count, span, base = FEC_HDR.unpack_from(data)
        parity = data[FEC_HDR.size:FEC_HDR.size + span]
        if len(parity) < span or not 2 <= count <= 8:
            self.mismatch += 1
            return

        members = [self.pkts.get((base + i) & 0xFFFFFFFF) for i in range(count)]
        have = [m for m in members if m is not None]

        if len(have) < count - 1:
            self.unverifiable += 1
            return

        if len(have) == count - 1:
            missing = next(i for i, m in enumerate(members) if m is None)
            acc = parity
            for m in have:
                acc = xor_bytes(acc, codeword(m))
            if fec_extract(acc, span, (base + missing) & 0xFFFFFFFF):
                self.repaired += 1
            else:
                self.mismatch += 1
            return

        self.groups += 1
        for drop in range(count):
            acc = parity
            for i, m in enumerate(members):
                if i != drop:
                    acc = xor_bytes(acc, codeword(m))
            self.checked += 1
            got = fec_extract(acc, span, (base + drop) & 0xFFFFFFFF)
            if got is not None and got == codeword(members[drop]):
                self.matched += 1
            else:
                self.mismatch += 1


def _fake_group(seqs, lens, k=None):
    pkts = []
    for i, (seq, plen) in enumerate(zip(seqs, lens)):
        hdr = AUDIO_HDR.pack(MSG_AUDIO, AUDIO_FMT_SBC, i == 1, i == 2,
                             plen, seq, 44100, 882, 5_000_000 + i * 20_000)
        pkts.append(hdr + bytes((b * 7 + i * 31 + 1) & 0xFF for b in range(plen)))
    span = max(len(p) for p in pkts)
    parity = b"\0" * span
    for p in pkts:
        parity = xor_bytes(parity, p)
    head = FEC_HDR.pack(MSG_AUDIO_FEC, k or len(pkts), span, seqs[0])
    return pkts, head + parity, span


def self_test():
    fails = 0

    def check(name, ok):
        nonlocal fails
        print(f"  {name:<52s} {'PASS' if ok else 'FAIL'}")
        if not ok:
            fails += 1

    seqs = [1000, 1001, 1002, 1003]
    pkts, parity, _span = _fake_group(seqs, [851, 823, 877, 12])

    v = FecVerifier()
    for seq, p in zip(seqs, pkts):
        v.note(seq, p)
    v.check(parity)
    check("every member of a good group rebuilds byte for byte",
          v.groups == 1 and v.checked == 4 and v.matched == 4 and v.mismatch == 0)

    v = FecVerifier()
    for seq, p in list(zip(seqs, pkts))[1:]:
        v.note(seq, p)
    v.check(parity)
    check("a genuinely lost member rebuilds and validates",
          v.repaired == 1 and v.mismatch == 0)

    v = FecVerifier()
    for seq, p in list(zip(seqs, pkts))[2:]:
        v.note(seq, p)
    v.check(parity)
    check("two lost members are reported unverifiable, not rebuilt",
          v.unverifiable == 1 and v.matched == 0 and v.mismatch == 0)

    bad = bytearray(parity)
    bad[FEC_HDR.size + 40] ^= 0x01
    v = FecVerifier()
    for seq, p in zip(seqs, pkts):
        v.note(seq, p)
    v.check(bytes(bad))
    check("a single flipped parity bit is caught", v.mismatch > 0)

    head = FEC_HDR.pack(MSG_AUDIO_FEC, 4, 9999, seqs[0])
    v = FecVerifier()
    for seq, p in zip(seqs, pkts):
        v.note(seq, p)
    v.check(head + parity[FEC_HDR.size:])
    check("a parity shorter than its own span is refused", v.mismatch > 0)

    print("\nself-test: " + ("FAILURES PRESENT" if fails else "all passed"))
    return 1 if fails else 0


def local_v4():
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
    ap.add_argument("--self-test", action="store_true",
                    help="check the parity verifier against a synthetic group "
                         "and exit. Needs no hub, socket or interface alias.")
    ap.add_argument("--verify", type=int, default=1, metavar="N",
                    help="satellites that also VERIFY the hub's XOR parity, "
                         "rebuilding every member of each group and comparing it "
                         "byte for byte (default 1; 0 disables). Every client "
                         "gets the same parity, so more than one proves nothing "
                         "extra and costs CPU this script would rather spend "
                         "keeping up with the hub.")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    base = ipaddress.IPv4Address(args.base)
    ips = [str(base + i) for i in range(args.n)]

    missing = []
    sats = []
    for i, ip in enumerate(ips):
        try:
            sats.append(Sat(ip, verify=i < args.verify))
        except OSError as e:
            if e.errno != errno.EADDRNOTAVAIL:
                raise
            missing.append(ip)
    if missing:
        for s in sats:
            s.sock.close()
        hub_net = ipaddress.IPv4Network(f"{args.hub}/24", strict=False)
        iface = next((i for a, i in local_v4()
                      if ipaddress.IPv4Address(a) in hub_net), None)

        if iface is None:
            here = [f"{a} on {i}" for a, i in local_v4() if i != "lo"]
            print(f"This machine has no {hub_net} address, so it is not on the "
                  f"hub's SoftAP.")
            print("Join the hub's network first (SSID and password are "
                  "DANCEFLOOR_AP_SSID / DANCEFLOOR_AP_PASS in")
            print("components/dancefloor_sync/Kconfig), then run this again.")
            if here:
                print("\nCurrently: " + ", ".join(here))
            return 1

        print(f"{len(missing)} of {args.n} addresses are not on this machine.")
        print(f"Run these (needs root) on {iface}, then start again:\n")
        for ip in missing:
            print(f"  sudo ip addr add {ip}/24 dev {iface}")
        print("\nTo undo afterwards: same lines with 'del' instead of 'add'.")
        return 1

    socks = [s.sock for s in sats]
    print(f"{args.n} fake satellites on {ips[0]}..{ips[-1]}, probing {args.hub} "
          f"every {PROBE_PERIOD * 1000:.0f} ms. Ctrl-C to stop.")
    got = min(s.rcvbuf for s in sats)
    print(f"receive buffer {got // 1024} kB per socket"
          + ("" if got >= (1 << 20) else
             "  -- LOW. Loss reported below may be this machine, not the air."
             " Raise it with: sudo sysctl -w net.core.rmem_max=8388608"))

    if args.verify:
        print(f"parity verification on {min(args.verify, args.n)} of them: every"
              f" member of every group is rebuilt from the parity and compared"
              f" byte for byte.")

    next_probe = time.monotonic()
    next_report = time.monotonic() + args.report
    prev = [(0, 0, 0, 0)] * args.n

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
                cur = [(s.audio, s.frames, s.lost, s.parity) for s in sats]
                rate = [(c[0] - p[0]) / dt for c, p in zip(cur, prev)]
                fr = [(c[1] - p[1]) / dt for c, p in zip(cur, prev)]
                lost = [c[2] - p[2] for c, p in zip(cur, prev)]
                par = [(c[3] - p[3]) / dt for c, p in zip(cur, prev)]
                registered = sum(1 for s in sats if s.replies)
                fec = ""
                v = [s.fec for s in sats if s.fec]
                if v:
                    bad = sum(f.mismatch for f in v)
                    fec = (f" | parity/s {min(par):.1f}-{max(par):.1f}"
                           f" | rebuilt {sum(f.matched for f in v):,}"
                           f" bad {bad}" + ("  ** MISMATCH **" if bad else ""))
                print(f"[{time.strftime('%H:%M:%S')}] registered {registered}/{args.n} | "
                      f"audio/s {min(rate):.0f}-{max(rate):.0f} (sum {sum(rate):.0f}) | "
                      f"frames/s {min(fr):.0f}-{max(fr):.0f} | "
                      f"lost this window {sum(lost)} | lost total {sum(s.lost for s in sats)}"
                      f"{fec}")
                prev = cur
                next_report += args.report
    except KeyboardInterrupt:
        print()

    print(f"{'sat':<16} {'audio':>9} {'lost':>7} {'parity':>8} {'frames':>8} {'replies':>8}")
    for s in sats:
        print(f"{s.ip:<16} {s.audio:>9} {s.lost:>7} {s.parity:>8} "
              f"{s.frames:>8} {s.replies:>8}")

    v = [s.fec for s in sats if s.fec]
    if v:
        groups = sum(f.groups for f in v)
        checked = sum(f.checked for f in v)
        matched = sum(f.matched for f in v)
        mismatch = sum(f.mismatch for f in v)
        repaired = sum(f.repaired for f in v)
        unver = sum(f.unverifiable for f in v)
        print(f"\nparity: {groups:,} complete groups, {checked:,} members rebuilt,"
              f" {matched:,} byte-for-byte")
        if repaired:
            print(f"        {repaired:,} genuinely lost member(s) rebuilt and"
                  f" validated -- real loss, so no byte comparison was possible")
        if unver:
            print(f"        {unver:,} group(s) lost two or more members, which one"
                  f" parity cannot repair by construction")
        if mismatch:
            print(f"        ** {mismatch:,} MISMATCHES -- the hub's encoder and this"
                  f" script disagree about the wire. **")
            print(f"        That is a real fault in one of them; neither is the"
                  f" radio. Compare against the host suite:")
            print(f"        components/dancefloor_sync/test/test_sync_proto.c")
            return 1
        if checked:
            print(f"        no mismatches -- the hub's parity reconstructs every"
                  f" member of every complete group.")
        elif not groups:
            print(f"        nothing verified: no parity arrived. Is"
                  f" DANCEFLOOR_AUDIO_FEC_K 0 in the hub's build?")
    return 0


if __name__ == "__main__":
    sys.exit(main())
