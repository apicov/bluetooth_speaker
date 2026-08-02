#!/usr/bin/env python3
"""
Desktop satellite -- listens to the hub's multicast audio and plays it.

Exists because there is no DAC yet, so nothing in this project has ever actually
been *heard*. Every conclusion so far rests on counters. This turns the stream
into sound.

The hub sends SBC rather than PCM, so this pipes the payload through ffmpeg to
decode it -- the same job the ESP32 satellites do with the vendored decoder.

Time-sync probes are what register this client with the hub: it unicasts audio
to whatever has probed recently. That matters more than the clock measurement --
multicast is never acknowledged or retried and lost ~20% of packets at every PHY
rate tried, while unicast gets link-layer retransmission and measures clean.

Usage:

    # join the hub's WiFi first: SSID "dancefloor", password "dancefloor"
    python3 desktop_satellite.py | aplay -f S16_LE -r 44100 -c 2 -

    # or, if you prefer ffplay
    python3 desktop_satellite.py | ffplay -f s16le -ar 44100 -ch_layout stereo -i - -nodisp

PCM goes to stdout, statistics to stderr, so the pipe stays clean. The only
dependency is ffmpeg, which needs its `sbc` decoder (check with
`ffmpeg -decoders | grep sbc`).
"""

import argparse
import socket
import struct
import subprocess
import sys
import threading
import time

PORT = 5001

MSG_TIME_REQ = 1
MSG_TIME_RSP = 2
MSG_AUDIO = 4
AUDIO_FMT_PCM = 0
AUDIO_FMT_SBC = 1

MASTER_IP = "192.168.4.1"      # esp_netif SoftAP default
PROBE_PERIOD_S = 0.25
SYNC_WINDOW = 10               # probes retained, as in the firmware

# time_msg_t: uint8_t type; uint32_t seq; int64_t t1, t2, t3
TIME = struct.Struct("<BIqqq")

# Mirrors audio_msg_t in components/dancefloor_sync/include/sync_proto.h:
#   uint8_t type; uint8_t format; uint16_t payload_len; uint32_t seq;
#   uint32_t sample_rate; uint32_t frames; int64_t play_at; uint8_t payload[]
HDR = struct.Struct("<BBHIIIq")
CHANNELS = 2


def log(msg):
    print(msg, file=sys.stderr, flush=True)


def now_us():
    """Monotonic microseconds -- the desktop's equivalent of esp_timer_get_time()."""
    return time.monotonic_ns() // 1000


def probe_loop(sock, stop):
    """Announce ourselves and measure the clock offset.

    Registration is the important part: the hub unicasts audio to whatever has
    probed in the last 10 s. Stop probing and it falls back to multicast, which
    loses packets.
    """
    seq = 0
    while not stop.is_set():
        msg = TIME.pack(MSG_TIME_REQ, seq, now_us(), 0, 0)
        try:
            sock.sendto(msg, (MASTER_IP, PORT))
        except OSError as e:
            log(f"probe failed: {e}")
        seq += 1
        stop.wait(PROBE_PERIOD_S)


def best_offset(samples):
    """Offset from the lowest-RTT probe, matching sync_proto.c.

    A fast round trip had little queuing in either direction, so its two path
    delays were closest to equal -- and that asymmetry is the only error the
    estimator cannot see. Picking the best sample beats averaging them.
    """
    if not samples:
        return None, None
    off, rtt = min(samples, key=lambda s: s[1])
    return off, rtt


def start_ffmpeg():
    """SBC frames in on stdin, raw s16le out on stdout."""
    try:
        return subprocess.Popen(
            ["ffmpeg", "-hide_banner", "-loglevel", "error",
             "-f", "sbc", "-i", "pipe:0",
             "-f", "s16le", "-ar", "44100", "-ac", "2", "pipe:1"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    except FileNotFoundError:
        log("ffmpeg not found -- needed to decode SBC")
        sys.exit(1)


def pump(src, dst, prebuffer_bytes):
    """Shovel decoded PCM to stdout on its own thread, so decode latency never
    stalls the receive loop and costs us packets.

    Holds back `prebuffer_bytes` first: without it the player starts on the very
    first sample and underruns on the first network hiccup.
    """
    buf = bytearray()
    started = False
    while True:
        chunk = src.read(4096)
        if not chunk:
            return
        if started:
            dst.write(chunk)
            dst.flush()
            continue
        buf += chunk
        if len(buf) >= prebuffer_bytes:
            log(f"prebuffered {len(buf) * 1000 // (44100 * CHANNELS * 2)} ms, playing")
            dst.write(buf)
            dst.flush()
            buf.clear()
            started = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--buffer-ms", type=int, default=400,
                    help="PCM held back before playback starts")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", PORT))
    sock.settimeout(2.0)

    log(f"listening on port {PORT}; audio arrives by unicast once probing starts")

    ff = start_ffmpeg()
    prebuffer = args.buffer_ms * 44100 * CHANNELS * 2 // 1000
    threading.Thread(target=pump, args=(ff.stdout, sys.stdout.buffer, prebuffer),
                     daemon=True).start()

    stop = threading.Event()
    threading.Thread(target=probe_loop, args=(sock, stop), daemon=True).start()
    log(f"probing {MASTER_IP} every {PROBE_PERIOD_S * 1000:.0f} ms "
        "(registers for unicast audio)")

    expect_seq = None
    pkts = gaps = bad = 0
    payload_bytes = 0
    sync_samples = []
    last_report = time.time()

    while True:
        try:
            data, _ = sock.recvfrom(2048)
        except socket.timeout:
            log("no packets for 2 s -- is the hub streaming, and are you on its AP?")
            continue

        if data and data[0] == MSG_TIME_RSP and len(data) >= TIME.size:
            t4 = now_us()
            _t, _seq, t1, t2, t3 = TIME.unpack_from(data, 0)
            # NTP-style: offset = ((t2-t1) + (t3-t4)) / 2, rtt = (t4-t1) - (t3-t2)
            sync_samples.append((((t2 - t1) + (t3 - t4)) // 2, (t4 - t1) - (t3 - t2)))
            del sync_samples[:-SYNC_WINDOW]
            continue

        if len(data) < HDR.size or data[0] != MSG_AUDIO:
            bad += 1
            continue

        _type, fmt, plen, seq, rate, frames, _play_at = HDR.unpack_from(data, 0)
        payload = data[HDR.size:HDR.size + plen]
        if len(payload) != plen:
            bad += 1
            continue

        if fmt != AUDIO_FMT_SBC:
            bad += 1
            if bad == 1:
                log(f"hub is sending format {fmt}, this expects SBC ({AUDIO_FMT_SBC})")
            continue

        if expect_seq is not None and seq != expect_seq:
            missing = (seq - expect_seq) & 0xFFFFFFFF
            if missing > 0xF0000000:
                # Backwards: a duplicate or reorder. Decoding it again would
                # play the same audio twice, so drop it silently.
                continue
            if 0 < missing < 100:
                gaps += missing
                # Nothing useful to insert: a dropped SBC frame cannot be
                # synthesised, and ffmpeg would reject padding. The gap simply
                # shortens playback slightly, which is what the ESP32 satellites
                # avoid by inserting silence -- they can, because they decode
                # themselves and know the frame count.
        expect_seq = (seq + 1) & 0xFFFFFFFF
        pkts += 1
        payload_bytes += plen

        try:
            ff.stdin.write(payload)
            ff.stdin.flush()
        except BrokenPipeError:
            log("ffmpeg exited")
            return

        now = time.time()
        if now - last_report >= 5.0:
            off, rtt = best_offset(sync_samples)
            sync = f" | offset {off} us rtt {rtt} us" if off is not None else ""
            log(f"pkts {pkts} | gaps {gaps} | bad {bad} | "
                f"{payload_bytes / (now - last_report) / 1024:.0f} kB/s SBC{sync}")
            pkts = gaps = bad = payload_bytes = 0
            last_report = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
