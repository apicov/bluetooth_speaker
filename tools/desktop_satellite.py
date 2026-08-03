#!/usr/bin/env python3
"""
Desktop satellite -- listens to the hub's audio, plays it, and records it.

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
    #   nmcli device wifi connect dancefloor password dancefloor

    # listen
    python3 desktop_satellite.py | aplay -f S16_LE -r 44100 -c 2 -

    # listen, and keep one WAV per track for tuning beat detection
    python3 desktop_satellite.py --record ~/dancefloor-tracks \
        | aplay -f S16_LE -r 44100 -c 2 --buffer-time=1000000 -

    # or play through ffplay instead
    python3 desktop_satellite.py | ffplay -f s16le -ar 44100 -ch_layout stereo -i - -nodisp

Stop with Ctrl-C rather than killing it: that is what writes the final WAV
header, without which the last recording of a session is unreadable.

Spotify ad breaks arrive as ordinary tracks and are recognised by their artist
field, so they play but are not saved. See --skip.

PCM goes to stdout, statistics to stderr, so the pipe stays clean. The only
dependency is ffmpeg, which needs its `sbc` decoder (check with
`ffmpeg -decoders | grep sbc`).
"""

import argparse
import os
import re
import socket
import struct
import subprocess
import sys
import threading
import time
import wave

PORT = 5001

MSG_TIME_REQ = 1
MSG_TIME_RSP = 2
MSG_AUDIO = 4
MSG_META = 5
# Master's 802.11 TSF against its own clock, sent to anything that probes.
# Measurement traffic for the ESP32 satellites; nothing here wants it, and it is
# named only so it does not land in the malformed-packet count.
MSG_TSF = 7
AUDIO_FMT_PCM = 0
AUDIO_FMT_SBC = 1

MASTER_IP = "192.168.4.1"      # esp_netif SoftAP default
PROBE_PERIOD_S = 0.25
SYNC_WINDOW = 10               # probes retained, as in the firmware

# time_msg_t: uint8_t type; uint32_t seq; int64_t t1, t2, t3
TIME = struct.Struct("<BIqqq")

# meta_msg_t: uint8_t type; then link_meta_t --
#   uint32_t track_id; char title[64]; char artist[64]; char album[64]
META = struct.Struct("<B I 64s 64s 64s")
RATE = 44100
SAMPLE_WIDTH = 2

# Mirrors audio_msg_t in components/dancefloor_sync/include/sync_proto.h:
#   uint8_t type; uint8_t format; uint8_t marker; uint8_t restart;
#   uint16_t payload_len; uint32_t seq; uint32_t sample_rate; uint32_t frames;
#   int64_t play_at; uint8_t payload[]
#
# `marker` and `restart` were added to the C struct after this was written and
# not mirrored here, which shifted every later field two bytes early. It failed
# quietly rather than loudly: payload_len was read from marker/restart, which are
# 0 in almost every packet, so plen came out 0, the "did we get the whole
# payload" check compared 0 against 0 and passed, and an empty payload went to
# ffmpeg. Packets counted, statistics looked healthy, and nothing made a sound.
HDR = struct.Struct("<BBBBHIIIq")
CHANNELS = 2


def log(msg):
    print(msg, file=sys.stderr, flush=True)


def now_us():
    """Monotonic microseconds -- the desktop's equivalent of esp_timer_get_time()."""
    return time.monotonic_ns() // 1000


def probe_loop(sock, stop):
    """Announce ourselves and measure the clock offset.

    Registration is the important part: the hub unicasts audio to whatever has
    probed in the last 10 s, and to nothing else. Stop probing and the audio
    stops within 10 s.
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


def clean(raw):
    """Trim a fixed-size C string and drop anything awkward in a filename."""
    txt = raw.split(b"\x00", 1)[0].decode("utf-8", "replace").strip()
    return txt


def safe_name(txt):
    return re.sub(r'[<>:"/\\|?*\x00-\x1f]', "_", txt).strip(". ")[:80]


# Spotify labels ads in the artist field: "Werbung . 1 von 2" in German,
# "Advertisement . 1 of 2" in English, and so on. Locale-dependent and therefore
# imperfect, which is why it is a command-line option rather than hard-wired.
DEFAULT_SKIP = r"werbung|advertisement|publicidad|pubblicit|an[uú]ncio|reclame|annonce"


class Recorder:
    """Writes decoded PCM to one WAV per track.

    Files open with a provisional name because metadata arrives one attribute at
    a time -- title first, then artist, then album -- so the full name is not
    known until well after the audio has started. They are renamed on close.

    The split lands wherever the decode pipeline happens to be, so it is off by
    a few hundred ms from the true boundary. Irrelevant for beat analysis, which
    is what these recordings are for.
    """

    def __init__(self, directory, skip_re):
        self.dir = directory
        self.skip_re = skip_re
        self.lock = threading.Lock()
        self.wav = None
        self.path = None
        self.meta = ("", "", "")
        self.track_id = None
        if directory:
            os.makedirs(directory, exist_ok=True)

    def write(self, data):
        if not self.dir:
            return
        with self.lock:
            if self.wav:
                self.wav.writeframes(data)

    def set_meta(self, title, artist, album):
        with self.lock:
            self.meta = (title, artist, album)
            if not self.wav or not self.skip_re:
                return
            # The ad marker usually arrives after recording has started, since
            # the artist field is not in the first metadata response. So the
            # file has to be discarded rather than never opened.
            if self.skip_re.search(artist) or self.skip_re.search(title):
                self.wav.close()
                self.wav = None
                try:
                    os.remove(self.path)
                except OSError:
                    pass
                log(f"    (advert -- not saved)")

    def _close_locked(self):
        if not self.wav:
            return
        self.wav.close()
        self.wav = None
        title, artist, _ = self.meta
        label = " - ".join(x for x in (artist, title) if x)
        if label:
            new = os.path.join(self.dir, f"{self.track_id:03d} - {safe_name(label)}.wav")
            try:
                os.replace(self.path, new)
                log(f"saved {new}")
                return
            except OSError as e:
                log(f"could not rename: {e}")
        log(f"saved {self.path}")

    def new_track(self, track_id):
        if not self.dir:
            return
        with self.lock:
            self._close_locked()
            self.track_id = track_id
            self.path = os.path.join(self.dir, f"{track_id:03d}.wav")
            self.wav = wave.open(self.path, "wb")
            self.wav.setnchannels(CHANNELS)
            self.wav.setsampwidth(SAMPLE_WIDTH)
            self.wav.setframerate(RATE)

    def close(self):
        with self.lock:
            self._close_locked()


def pump(src, dst, prebuffer_bytes, recorder):
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
            recorder.write(chunk)
            continue
        buf += chunk
        if len(buf) >= prebuffer_bytes:
            log(f"prebuffered {len(buf) * 1000 // (44100 * CHANNELS * 2)} ms, playing")
            dst.write(buf)
            dst.flush()
            recorder.write(buf)
            buf.clear()
            started = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--buffer-ms", type=int, default=400,
                    help="PCM held back before playback starts")
    ap.add_argument("--record", metavar="DIR",
                    help="save one WAV per track here, for offline analysis")
    ap.add_argument("--skip", metavar="REGEX", default=DEFAULT_SKIP,
                    help="discard recordings whose title or artist matches "
                         "(case-insensitive); pass an empty string to keep everything")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", PORT))
    sock.settimeout(2.0)

    log(f"listening on port {PORT}; audio arrives by unicast once probing starts")

    ff = start_ffmpeg()
    skip_re = re.compile(args.skip, re.I) if args.skip else None
    recorder = Recorder(args.record, skip_re)
    if args.record:
        log(f"recording one WAV per track into {args.record}")
    prebuffer = args.buffer_ms * RATE * CHANNELS * SAMPLE_WIDTH // 1000
    threading.Thread(target=pump,
                     args=(ff.stdout, sys.stdout.buffer, prebuffer, recorder),
                     daemon=True).start()

    stop = threading.Event()
    threading.Thread(target=probe_loop, args=(sock, stop), daemon=True).start()
    log(f"probing {MASTER_IP} every {PROBE_PERIOD_S * 1000:.0f} ms "
        "(registers for unicast audio)")

    expect_seq = None
    last_track = None
    pkts = gaps = bad = 0
    payload_bytes = 0
    sync_samples = []
    last_report = time.time()

    try:
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

        if data and data[0] == MSG_META and len(data) >= META.size:
            _t, track_id, t_raw, a_raw, al_raw = META.unpack_from(data, 0)
            title, artist, album = clean(t_raw), clean(a_raw), clean(al_raw)
            if track_id != last_track:
                last_track = track_id
                log(f"--- track #{track_id} ---")
                recorder.new_track(track_id)
            recorder.set_meta(title, artist, album)
            desc = " - ".join(x for x in (artist, title) if x) or "(no metadata yet)"
            log(f"    {desc}" + (f"  [{album}]" if album else ""))
            continue

        if data and data[0] == MSG_TSF:
            continue                    # not ours; see MSG_TSF above

        if len(data) < HDR.size or data[0] != MSG_AUDIO:
            bad += 1
            continue

        _type, fmt, _marker, _restart, plen, seq, rate, frames, _play_at = \
            HDR.unpack_from(data, 0)
        payload = data[HDR.size:HDR.size + plen]
        # plen == 0 is rejected explicitly: an empty payload is never legitimate,
        # and treating it as valid is what hid the header mismatch above.
        if not plen or len(payload) != plen:
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
    finally:
        # wave.close() is what writes the header -- without it the last file of
        # a session is unreadable, which is exactly the one you just recorded.
        recorder.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
