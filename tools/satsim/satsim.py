#!/usr/bin/env python3
"""
N fake satellites on a laptop, to load the hub's unicast fan-out.

A LOAD GENERATOR, not an instrument. It makes the hub believe N more speakers
are on the floor, so it fans audio and analysis frames out to N more
destinations; what that does to the real satellites is the measurement, and it
is taken on them.

N SOURCE IPs, NOT N PORTS. client_seen() in hub_s3/main/clients.c matches on
sin_addr.s_addr alone, so N sockets sharing the laptop's one address collapse
into a single client slot -- the hub would fan out to one destination and this
would look like it was working. Each fake satellite therefore needs its own
address, added as an alias on the interface already associated to the SoftAP.
This script prints the commands and does not run them, since they need root.

Registration is by probe: a unit that has probed within CLIENT_TIMEOUT_US is on
the send list. So each fake satellite sends the same MSG_TIME_REQ a real one
does, at the same period, and that is the whole of what puts it on the list.

IT ALSO VERIFIES THE HUB'S XOR PARITY, which is the one thing here that IS an
instrument. Every group is rebuilt K times -- each member withheld from the XOR
in turn and reconstructed from the parity and the other K-1 -- and compared byte
for byte against the packet that actually arrived. That checks the HUB'S ENCODER
and the wire format against an implementation sharing no code with it. The host
suite in components/dancefloor_sync/test checks the codec against itself, which
cannot catch the codec and the sender agreeing on something wrong; this can.
`--self-test` proves the checker itself both passes a good parity and fails a
corrupted one, with no hub needed.

What it CANNOT test is the satellite's receive path. There is no ring, no hold
and no re-anchoring here, so fec-held, fec-hold-max and the repair-into-the-ring
logic only ever run on real firmware against real loss.

What it is uniquely good for is the other half. N fake satellites is N more
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

## @brief The sync protocol's UDP port. See sync_proto.h.
SYNC_PORT = 5001
## @brief Seconds between probes, matching PROBE_PERIOD_MS in
#         satellite/main/sat.h. It is what keeps a client registered.
PROBE_PERIOD = 0.250

## @brief A satellite's clock probe; also what registers it.
MSG_TIME_REQ = 1
## @brief The hub's reply to one.
MSG_TIME_RSP = 2
## @brief One audio datagram.
MSG_AUDIO = 4
## @brief One analysis frame.
MSG_FRAME = 8
## @brief One group's XOR parity.
MSG_AUDIO_FEC = 14
## @brief The only audio format on this wire.
AUDIO_FMT_SBC = 1

# All packed, little-endian. See
# components/dancefloor_sync/include/sync_proto.h.

## @brief type, seq, t1, t2, t3.
TIME_MSG = struct.Struct("<BIqqq")
## @brief type, fmt, marker, restart, payload_len, seq, rate, frames, play_at.
AUDIO_HDR = struct.Struct("<BBBBHIIIq")
## @brief type, len, count.
FRAME_HDR = struct.Struct("<BBB")
## @brief type, count, span, base_seq.
FEC_HDR = struct.Struct("<BBHI")

# AUDIO_HDR.size is AUDIO_MSG_BYTES(0). The codeword a group XORs is that
# header followed by the payload, zero-padded to the longest member -- see
# audio_fec_msg_t in sync_proto.h. Asserted so a drifted format fails here
# rather than silently unpacking garbage off the wire.
assert AUDIO_HDR.size == 26, "audio_msg_t's header moved; this script is stale"
assert FEC_HDR.size == 8, "AUDIO_FEC_HDR_BYTES moved; this script is stale"


def xor_bytes(a, b):
    """
    @brief XOR of two byte strings, zero-padded to the longer.

    Via int rather than a loop: a group is most of a kilobyte and this runs
    several times per group per satellite, and the script's job is to LOAD the
    hub rather than to become the thing that cannot keep up with it.

    @param a  One operand.
    @param b  The other.
    @return The XOR, as long as the longer input.
    """
    n = max(len(a), len(b))
    return (int.from_bytes(a.ljust(n, b"\0"), "little")
            ^ int.from_bytes(b.ljust(n, b"\0"), "little")).to_bytes(n, "little")


def codeword(raw):
    """
    @brief The bytes of one audio datagram that its group's parity covers.
    @param raw  The whole datagram as it arrived.
    @return Header plus payload, with nothing after it.
    """
    plen = struct.unpack_from("<H", raw, 4)[0]
    return raw[:AUDIO_HDR.size + plen]


def fec_extract(acc, span, want_seq):
    """
    @brief Rebuild one member from a completed XOR, or reject it.

    The same four checks audio_fec_extract() applies in sync_proto.c, written
    independently against the header rather than shared with it: the point of a
    second implementation is that it can disagree.

    @param acc       The XOR of the parity and every other member.
    @param span      The parity's span, from its header.
    @param want_seq  The sequence number the rebuilt packet must carry.
    @return The rebuilt datagram, or None if it is not a well-formed packet.
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
    """
    @brief Monotonic microseconds, for a probe's t1.
    @return Microseconds since an arbitrary origin.
    """
    return time.monotonic_ns() // 1000


class Sat:
    """
    @brief One fake satellite: a socket on its own address, and what it has seen.
    """

    def __init__(self, ip, verify=False):
        """
        @brief Open and bind the socket. Raises OSError if the address is not
               on this machine, which main() turns into instructions.
        @param ip      The alias address to bind.
        @param verify  Also check the hub's parity on this one.
        """
        ## @brief The address this satellite binds and probes from.
        self.ip = ip
        ## @brief Its UDP socket, non-blocking.
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # ASK FOR A BIG RECEIVE BUFFER, because the default one makes this
        # script invent packet loss that never happened. N fake satellites are
        # N unicast streams arriving at ONE laptop radio and one IP stack, and
        # a load generator whose own receive path is the bottleneck reports the
        # hub as broken when it is fine -- which is worse than not measuring.
        #
        # Parity suffers worst, and the shape says why: it is the LAST datagram
        # of each group, so it arrives when the socket buffer is at its fullest
        # and is the first thing the kernel discards.
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
        ## @brief What the kernel actually granted, which main() prints.
        self.rcvbuf = self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        self.sock.bind((ip, SYNC_PORT))
        self.sock.setblocking(False)
        ## @brief Sequence number of the next probe.
        self.probe_seq = 0
        ## @brief Audio datagrams received.
        self.audio = 0
        ## @brief Analysis frames received.
        self.frames = 0
        ## @brief Audio datagrams missing from the sequence.
        self.lost = 0
        ## @brief Probe replies received; a non-zero count means registered.
        self.replies = 0
        ## @brief Parity datagrams received.
        self.parity = 0
        ## @brief Last audio sequence number seen, for the loss count.
        self.last_seq = None

        ## @brief The parity checker, on one satellite by default.
        #
        # Every client receives the SAME parity, so checking it N times costs N
        # times as much and proves nothing extra -- and this script's first
        # duty is to be a load generator rather than the thing that stops
        # keeping up.
        self.fec = FecVerifier() if verify else None

    def probe(self, hub):
        """
        @brief Send one MSG_TIME_REQ, which is what keeps this client
               registered with the hub.
        @param hub  The hub's address.
        """
        self.probe_seq += 1
        msg = TIME_MSG.pack(MSG_TIME_REQ, self.probe_seq, now_us(), 0, 0)
        try:
            self.sock.sendto(msg, (hub, SYNC_PORT))
        except OSError:
            pass                # a probe lost to a full queue is a probe lost

    def recv(self):
        """
        @brief Drain the socket, counting and classifying everything on it.
        """
        ## @cond
        # Doxygen's Python scanner reads identifiers out of a loop body like
        # this one and attaches them to the class as phantom members; hiding
        # the block costs nothing, since statements inside a method are not
        # documentation.
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
                # Counted whatever else happens to it: the hub unicasts
                # parity to every registered client, fake ones included, so a
                # run that does not count it under-reports its own fan-out.
                self.parity += 1
                if self.fec:
                    self.fec.check(data)
            elif kind == MSG_FRAME and len(data) >= FRAME_HDR.size:
                self.frames += 1
            elif kind == MSG_TIME_RSP:
                self.replies += 1
        ## @endcond


class FecVerifier:
    """
    @brief Checks the hub's XOR parity against an implementation it shares no
           code with.

    Separate from Sat so it can be exercised without a socket. @see self_test
    """

    def __init__(self):
        """
        @brief Start with nothing seen.
        """
        ## @brief seq -> raw datagram, for the group check below.
        self.pkts = {}
        ## @brief Parity packets whose whole group was in hand.
        self.groups = 0
        ## @brief Member reconstructions attempted.
        self.checked = 0
        ## @brief ...that came back byte for byte.
        self.matched = 0
        ## @brief ...that did not: a real encoder or wire-format fault.
        self.mismatch = 0
        ## @brief Genuinely lost members rebuilt and validated.
        self.repaired = 0
        ## @brief Groups with real loss, so no byte comparison was possible.
        self.unverifiable = 0

    def note(self, seq, data):
        """
        @brief Remember one audio datagram for the group check.

        The window is bounded and pruned oldest-first: a run of hours would
        otherwise hold every packet it ever saw.

        @param seq   Its sequence number.
        @param data  The whole datagram.
        """
        self.pkts[seq] = data
        if len(self.pkts) > 256:
            for old in sorted(self.pkts)[:128]:
                del self.pkts[old]

    def check(self, data):
        """
        @brief Rebuild every member of this group from the parity and compare.

        THE DROPS ARE SIMULATED, NOT WAITED FOR. A clean desktop link loses
        nothing, so waiting for real loss would test the repair path roughly
        never. Withholding each member in turn from the XOR reconstructs it
        from the parity and the other K-1 -- exactly what a satellite does for
        a packet that never arrived -- and the withheld packet is still in hand
        to compare against. Every group therefore exercises all K positions,
        including the first-member and last-member cases that differ on the
        satellite.

        A group that really is missing one member cannot be byte-compared, but
        the rebuild must still validate, which is the same test the satellite
        applies before it trusts a repair. Two missing is out of one parity's
        reach by construction, and is counted as unverifiable rather than
        guessed at.

        @param data  The parity datagram.
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
    """
    @brief A group of audio datagrams and the parity a correct hub would send.

    Built from the header rather than from this file's own helpers, so
    self_test() exercises codeword(), xor_bytes() and fec_extract() rather than
    agreeing with them by construction.

    @param seqs  One sequence number per member.
    @param lens  One payload length per member.
    @param k     The count to write into the parity header; defaults to the
                 real member count, and differs only when a test wants it to.
    @return (packets, parity datagram, span).
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
    """
    @brief Prove the verifier can both pass a good parity and fail a bad one.

    A checker that only ever says PASS is worth nothing, so two of the five
    cases require a MISMATCH. Runs without a hub, a socket or an interface
    alias.

    @return 0 if every check passed, 1 otherwise.
    """
    fails = 0

    def check(name, ok):
        """
        @brief Print one result and count a failure.
        @param name  What was checked.
        @param ok    Whether it held.
        """
        nonlocal fails
        print(f"  {name:<52s} {'PASS' if ok else 'FAIL'}")
        if not ok:
            fails += 1

    # Unequal payloads on purpose: the zero padding to the longest member is
    # the part of the wire format with nothing else to hold it up.
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
    """
    @brief Every IPv4 address on this machine.
    @return A list of (address, interface).
    """
    out = subprocess.run(["ip", "-o", "-4", "addr", "show"],
                         capture_output=True, text=True).stdout
    found = []
    for line in out.splitlines():
        f = line.split()
        if len(f) >= 4:
            found.append((f[3].split("/")[0], f[1]))
    return found


def main():
    """
    @brief Open N sockets, probe the hub, and report what arrives.

    An address that is not on the machine is the common first failure, and the
    two causes have different fixes: not being on the hub's SoftAP at all, or
    being on it without the aliases. Both are reported with the commands to
    run, and neither is attempted here, since adding an address needs root.

    @return 0, or 1 for a setup failure or a parity mismatch.
    """
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

    # Ask the kernel rather than parsing `ip addr`: binding is the thing that
    # has to work, and an address that cannot be bound is the failure worth
    # naming whatever the reason.
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

        # THE ASSOCIATION FIRST, and the aliases only once there is something
        # to add them to. An alias on the wrong interface would not reach the
        # hub anyway, and not being on its AP is the whole fault.
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
    # The kernel silently caps SO_RCVBUF at net.core.rmem_max, so say what was
    # actually granted rather than what was asked for: a small buffer here is
    # the difference between measuring the hub and measuring this laptop.
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


## @cond
# The entry point, not API.
if __name__ == "__main__":
    sys.exit(main())
## @endcond
