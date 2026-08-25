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

IT ALSO VERIFIES THE HUB'S XOR PARITY, which is the one thing here that IS an
instrument. Every group is rebuilt K times -- each member withheld from the XOR
in turn and reconstructed from the parity and the other K-1 -- and compared byte
for byte against the packet that actually arrived. The drops are simulated
rather than waited for, because a clean link loses nothing and waiting for real
loss would test the repair path roughly never; withholding covers all K
positions on every group instead.

That checks the HUB'S ENCODER and the wire format against an implementation
sharing no code with it. The host suite in components/dancefloor_sync/test
checks the codec against itself, which cannot catch the codec and the sender
agreeing on something wrong; this can. `--self-test` proves the checker itself
both passes a good parity and fails a corrupted one, with no hub needed.

What it CANNOT test is the satellite's receive path: there is no ring, no hold
and no re-anchoring here, so fec-held, fec-hold-max and the repair-into-the-ring
logic only ever run on real firmware against real loss.

What it is uniquely good for is the OTHER half. N fake satellites is N more
unicast destinations, which is the direct route to exhausting the hub's transmit
pool -- and parity is supposed to stand down under that pressure rather than
displace the audio it protects. Watch fec-cong rise on the hub's status line
while tx-fail (audio) stays flat. If both move together, that design premise is
wrong.
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
MSG_AUDIO_FEC = 14
AUDIO_FMT_SBC = 1

# All packed, little-endian. See components/dancefloor_sync/include/sync_proto.h
TIME_MSG = struct.Struct("<BIqqq")              # type, seq, t1, t2, t3
AUDIO_HDR = struct.Struct("<BBBBHIIIq")         # ..., payload_len, seq, rate, frames, play_at
FRAME_HDR = struct.Struct("<BBB")               # type, len, count
FEC_HDR = struct.Struct("<BBHI")                # type, count, span, base_seq

# AUDIO_HDR.size is AUDIO_MSG_BYTES(0). The codeword a group XORs is this header
# followed by the payload, zero-padded to the longest member -- see
# audio_fec_msg_t in sync_proto.h.
assert AUDIO_HDR.size == 26, "audio_msg_t's header moved; this script is stale"
assert FEC_HDR.size == 8, "AUDIO_FEC_HDR_BYTES moved; this script is stale"


def xor_bytes(a, b):
    """XOR of two byte strings, zero-padded to the longer.

    Via int rather than a loop: a group is ~900 bytes and this runs four times
    per group per satellite, and the script's job is to LOAD the hub rather than
    to become the thing that cannot keep up with it.
    """
    n = max(len(a), len(b))
    return (int.from_bytes(a.ljust(n, b"\0"), "little")
            ^ int.from_bytes(b.ljust(n, b"\0"), "little")).to_bytes(n, "little")


def codeword(raw):
    """The bytes of one audio datagram that its group's parity covers."""
    plen = struct.unpack_from("<H", raw, 4)[0]
    return raw[:AUDIO_HDR.size + plen]


def fec_extract(acc, span, want_seq):
    """Rebuild one member from a completed XOR, or None if it is not a packet.

    The same four checks audio_fec_extract() applies in sync_proto.c, written
    independently against the header rather than shared with it: the point of a
    second implementation is that it can disagree.
    """
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
    """One fake satellite: a socket on its own address, and what it has seen."""

    def __init__(self, ip, verify=False):
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
        self.parity = 0
        self.last_seq = None

        # Parity verification, on one satellite by default -- see main(). Every
        # client receives the SAME parity, so checking it N times costs N times
        # as much and proves nothing extra, and this script's first duty is to
        # be a load generator rather than the thing that stops keeping up.
        self.fec = FecVerifier() if verify else None

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
                if self.fec:
                    self.fec.note(seq, data)
            elif kind == MSG_AUDIO_FEC and len(data) >= FEC_HDR.size:
                # Counted whatever else happens to it: the hub unicasts parity to
                # every registered client, fake ones included, so a run that does
                # not count it under-reports its own fan-out by 1/K.
                self.parity += 1
                if self.fec:
                    self.fec.check(data)
            elif kind == MSG_FRAME and len(data) >= FRAME_HDR.size:
                self.frames += 1
            elif kind == MSG_TIME_RSP:
                self.replies += 1


class FecVerifier:
    """Checks the hub's XOR parity against an implementation it shares no code
    with. Separate from Sat so it can be exercised without a socket -- see
    self_test()."""

    def __init__(self):
        self.pkts = {}          # seq -> raw datagram, for the group check below
        self.groups = 0         # parity packets whose whole group was in hand
        self.checked = 0        # member reconstructions attempted
        self.matched = 0        # ...that came back byte for byte
        self.mismatch = 0       # ...that did not: a real encoder or format fault
        self.repaired = 0       # genuinely lost members rebuilt and validated
        self.unverifiable = 0   # group had real loss, so no byte comparison

    def note(self, seq, data):
        self.pkts[seq] = data
        if len(self.pkts) > 256:
            for old in sorted(self.pkts)[:128]:
                del self.pkts[old]

    def check(self, data):
        """Rebuild every member of this group from the parity and compare.

        THE DROPS ARE SIMULATED, NOT WAITED FOR. A clean desktop link loses
        nothing, so waiting for real loss would test the repair path roughly
        never. Withholding each member in turn from the XOR reconstructs it from
        the parity and the other K-1 -- exactly what a satellite does for a
        packet that never arrived -- and the withheld packet is still in hand to
        compare against. Every group therefore exercises all K positions,
        including the first-member and last-member cases that differ on the
        satellite.

        This checks the HUB's encoder and the wire format against an
        implementation that shares no code with it. The host suite checks the
        codec against itself; this is the half that can catch the codec and the
        sender agreeing on something wrong.
        """
        _kind, count, span, base = FEC_HDR.unpack_from(data)
        parity = data[FEC_HDR.size:FEC_HDR.size + span]
        if len(parity) < span or not 2 <= count <= 8:
            self.mismatch += 1          # a parity we cannot even read
            return

        members = [self.pkts.get((base + i) & 0xFFFFFFFF) for i in range(count)]
        have = [m for m in members if m is not None]

        if len(have) < count - 1:
            self.unverifiable += 1      # two or more genuinely missing
            return

        if len(have) == count - 1:
            # A real loss. No byte comparison possible -- the packet never
            # arrived -- but the rebuild must still validate, which is the same
            # test the satellite applies before it trusts a repair.
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
    """A group of audio datagrams and the parity a correct hub would send.

    Built from the header rather than from satsim's own helpers, so the test
    exercises codeword()/xor_bytes()/fec_extract() rather than agreeing with
    them by construction.
    """
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
    """Prove the verifier can both pass a good parity and fail a bad one.

    A checker that only ever says PASS is worth nothing, so the second half
    corrupts the parity and requires a MISMATCH. Runs without a hub, a socket or
    an interface alias: `satsim.py --self-test`.
    """
    fails = 0

    def check(name, ok):
        nonlocal fails
        print(f"  {name:<52s} {'PASS' if ok else 'FAIL'}")
        if not ok:
            fails += 1

    # Unequal payloads on purpose: the zero padding to the longest member is the
    # part of the wire format with nothing else to hold it up.
    seqs = [1000, 1001, 1002, 1003]
    pkts, parity, _span = _fake_group(seqs, [851, 823, 877, 12])

    v = FecVerifier()
    for seq, p in zip(seqs, pkts):
        v.note(seq, p)
    v.check(parity)
    check("every member of a good group rebuilds byte for byte",
          v.groups == 1 and v.checked == 4 and v.matched == 4 and v.mismatch == 0)

    # One member genuinely absent: no byte comparison is possible, but the
    # rebuild must still validate -- the satellite's own test before it trusts
    # a repair.
    v = FecVerifier()
    for seq, p in list(zip(seqs, pkts))[1:]:
        v.note(seq, p)
    v.check(parity)
    check("a genuinely lost member rebuilds and validates",
          v.repaired == 1 and v.mismatch == 0)

    # Two gone is out of one parity's reach by construction, and must be said
    # rather than guessed at.
    v = FecVerifier()
    for seq, p in list(zip(seqs, pkts))[2:]:
        v.note(seq, p)
    v.check(parity)
    check("two lost members are reported unverifiable, not rebuilt",
          v.unverifiable == 1 and v.matched == 0 and v.mismatch == 0)

    # AND THE HALF THAT MATTERS: a wrong parity must not pass.
    bad = bytearray(parity)
    bad[FEC_HDR.size + 40] ^= 0x01
    v = FecVerifier()
    for seq, p in zip(seqs, pkts):
        v.note(seq, p)
    v.check(bytes(bad))
    check("a single flipped parity bit is caught", v.mismatch > 0)

    # A span that claims more than the datagram carries is a truncated parity,
    # and reading it as a short packet is how a repair goes wrong quietly.
    head = FEC_HDR.pack(MSG_AUDIO_FEC, 4, 9999, seqs[0])
    v = FecVerifier()
    for seq, p in zip(seqs, pkts):
        v.note(seq, p)
    v.check(head + parity[FEC_HDR.size:])
    check("a parity shorter than its own span is refused", v.mismatch > 0)

    print("\nself-test: " + ("FAILURES PRESENT" if fails else "all passed"))
    return 1 if fails else 0


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

    # Ask the kernel rather than parsing `ip addr`: binding is the thing that has
    # to work, and an address that cannot be bound is the failure worth naming
    # whatever the reason.
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
                # parity/s should be audio/s over K, and the pair is the
                # one-glance check that the lane is alive at all.
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
