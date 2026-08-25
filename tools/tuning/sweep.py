#!/usr/bin/env python3
"""
sweep -- drive pattern_lab over the corpus and reduce its output to the tuning
tables.

This exists because the previous retune left no manifest. The booms-per-minute
ladder in components/dancefloor_leds/analysis.cpp is the only record of it, and
that comment does not say which tracks, which hop, or which command -- so
reproducing it meant guessing at all three. Every number the retune rested on is
produced by a subcommand here, so the command IS the provenance.

Stdlib only, deliberately: this box has no numpy on either interpreter, and a
tool whose job is to still run in a year should not acquire a dependency to
compute a percentile.

  ./sweep.py manifest                       the corpus, with sha256s
  ./sweep.py control                        (re)generate the negative control
  ./sweep.py baseline                       reproduce the hop-1024 record
  ./sweep.py sweep --detector boom          the flux floor ladder
  ./sweep.py hist                           BEAT_HIST, both axes
  ./sweep.py instants                       boom instants, hop 1024 vs 512
"""

import argparse
import concurrent.futures
import hashlib
import math
import os
import random
import shutil
import struct
import subprocess
import sys
import tempfile
import wave
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent                      # the dancefloor tree
LAB = ROOT / "tools" / "pattern_lab"

ANCHOR_DIR = Path.home() / "dancefloor-tracks"
BREADTH_DIR = Path.home() / "Music" / "forro"
ZABUMBA_DIR = Path.home() / "Music" / "zabumba"
CACHE = Path(os.environ.get("DF_TUNING_CACHE", Path.home() / ".cache" / "dancefloor-tuning"))

# The zabumba material is cut into labelled segments rather than used whole:
# two of the six recordings are lessons, and one of those carries half a minute
# of pure talking that is worth keeping on purpose as a negative control. The
# table is checked in beside this file because a segment boundary is a tuning
# input like any other. ./sweep.py segments shows how it was proposed; what is
# checked in is what a human then confirmed by ear.
ZAB_SEGMENTS = HERE / "zabumba_segments.csv"
ZAB_LABELS = HERE / "zabumba_labels.csv"

RATE = 44100                                    # df::RATE

# ffmpeg is pinned to the firmware's rate and channel count rather than left to
# follow the source: pattern_lab analyses whatever rate the file carries, and a
# 48 kHz decode would move every band edge relative to the tuning.
FFMPEG = ["ffmpeg", "-v", "error", "-i", "{src}",
          "-ar", str(RATE), "-ac", "2", "-c:a", "pcm_s16le", "-y", "{dst}"]

# The firmware's own marginal test, verbatim from visualiser.cpp: within 10% of
# the threshold, counted PER FRAME. Per boom it would read as a 3x regression
# where there is none -- the frame count doubled at hop 512 while the 200 ms
# refractory held the boom count nearly constant.
MARGIN = 0.1


# --------------------------------------------------------------------------
# corpus

class Track:
    def __init__(self, label, source, wav, group, span=None):
        self.label, self.source, self.wav, self.group = label, source, wav, group
        self.span = span               # (start_s, end_s) for a cut segment, else None


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def wav_info(path):
    with wave.open(str(path), "rb") as w:
        return w.getframerate(), w.getnframes() / w.getframerate()


def decode(src, dst):
    dst.parent.mkdir(parents=True, exist_ok=True)
    cmd = [a.format(src=str(src), dst=str(dst)) for a in FFMPEG]
    subprocess.run(cmd, check=True)


def corpus(groups, quiet=False):
    """The tracks, decoding and caching the lossy half on first use."""
    out = []
    if "anchor" in groups:
        # Top level only. xotes/ is excluded: roughly 60 of its 129 files are
        # 44-byte headers with no audio and about 20 are advertising, neither of
        # which is forro, and an average over them would not mean anything.
        for p in sorted(ANCHOR_DIR.glob("*.wav")):
            out.append(Track(p.stem, p, p, "anchor"))
    if "breadth" in groups:
        wavdir = CACHE / "wav"
        todo = []
        for p in sorted(BREADTH_DIR.glob("*.mp3")):
            dst = wavdir / (p.stem + ".wav")
            if not dst.exists():
                todo.append((p, dst))
            out.append(Track(p.stem, p, dst, "breadth"))
        if todo:
            if not quiet:
                print(f"decoding {len(todo)} tracks into {wavdir} ...", file=sys.stderr)
            with concurrent.futures.ThreadPoolExecutor(max_workers=os.cpu_count()) as ex:
                list(ex.map(lambda a: decode(*a), todo))
    if "control" in groups:
        p = CACHE / "control.wav"
        if not p.exists():
            make_control(p)
        out.append(Track("control (synthetic, no drum)", p, p, "control"))
    for g in ("zabumba", "zabumba-speech"):
        if g in groups:
            out.extend(zabumba_tracks("drum" if g == "zabumba" else "speech", g))
    return out


# --------------------------------------------------------------------------
# the zabumba set: isolated drumming, cut into labelled segments
#
# This is the only group with positive ground truth behind it, and the only one
# on which "did it fire on a drum stroke" is answerable at all -- on a full mix
# a rate ladder cannot tell a suppressed stroke from an absent one. It does not
# replace the anchor set, which is lossless and carries the comparison against
# the record; these six are lossy rips through room and camera mics and answer a
# different question. Do not read one group's ladder as the other's.

def zab_sources():
    """The six recordings, whatever container they arrived in."""
    return sorted(p for p in ZABUMBA_DIR.iterdir()
                  if p.suffix.lower() in (".opus", ".mp4", ".m4a", ".wav", ".mp3"))


def zab_full_wav(src):
    """The whole recording at the firmware's rate, decoded once and cached."""
    dst = CACHE / "wav" / "zabumba" / (src.stem + ".full.wav")
    if not dst.exists():
        decode(src, dst)
    return dst


def zab_cut(src, start, end):
    """
    One segment, cut from the cached full decode with the stdlib wave module.

    Cut here rather than with ffmpeg -ss so the boundary is an exact sample
    index rather than whatever the container's seek lands on. The labels in
    zabumba_labels.csv are instants within these cuts, so a boundary that moved
    by a few milliseconds between runs would silently shift every one of them.
    """
    dst = CACHE / "wav" / "zabumba" / f"{src.stem}.{start:g}-{end:g}.wav"
    if dst.exists():
        return dst
    full = zab_full_wav(src)
    dst.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(full), "rb") as r:
        rate, ch, w = r.getframerate(), r.getnchannels(), r.getsampwidth()
        a, b = int(start * rate), int(end * rate)
        b = min(b, r.getnframes())
        r.setpos(a)
        frames = r.readframes(max(0, b - a))
    with wave.open(str(dst), "wb") as o:
        o.setnchannels(ch)
        o.setsampwidth(w)
        o.setframerate(rate)
        o.writeframes(frames)
    return dst


def zab_segment_rows():
    """The checked-in segment table, or an empty list before it is written."""
    if not ZAB_SEGMENTS.exists():
        return []
    rows = []
    for line in ZAB_SEGMENTS.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or line.startswith("file,"):
            continue
        stem, start, end, label = line.split(",")
        rows.append((stem, float(start), float(end), label))
    return rows


def zabumba_tracks(label, group):
    by_stem = {p.stem: p for p in zab_sources()}
    out = []
    for stem, start, end, lab in zab_segment_rows():
        if lab != label:
            continue
        src = by_stem.get(stem)
        if src is None:
            raise SystemExit(f"{ZAB_SEGMENTS}: no recording named {stem!r} in {ZABUMBA_DIR}")
        out.append(Track(f"{stem} [{start:g}-{end:g}]", src,
                         zab_cut(src, start, end), group, (start, end)))
    return out


# --------------------------------------------------------------------------
# the negative control

def make_control(path, seconds=30):
    """
    Dither, a sustained accordion-ish chord that changes, and a triangle. No
    drum anywhere in it.

    This is the acceptance test at the low end of the ladder, and without it
    "lower floor -> more booms" has no cost attached. analysis.cpp records that
    0.02 produces exactly zero booms on such a passage and 0.012 produces about
    three a minute; that is the only measurement in the original sweep that says
    what the extra sensitivity costs.

    It is not silence with a beep. The bass CHANGES -- a left hand moving every
    two bars gives band 0 genuine rises to be wrong about -- and the triangle
    puts continuous energy in the top band at eighth-note rate, which is exactly
    the thing the boom detector exists to ignore. A floor that fires here is
    following the music rather than the drum.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    rng = random.Random(20260804)              # seeded: the sha256 is in the manifest
    n = seconds * RATE
    bass_notes = [55.0, 61.74, 49.0, 65.41]    # A1, B1, G1, C2 -- inside band 0
    frames = bytearray()
    # Phase is integrated rather than computed as f*t, and the triangle gets a
    # 3 ms attack rather than being switched on. Both are the same bug, and the
    # first draft of this file had both: a step discontinuity is BROADBAND, so a
    # note change and a gated 6.3 kHz sine were each dumping energy straight
    # into band 0 -- 43 to 129 Hz, the only band the boom detector sees. It fired
    # 66 times in 30 seconds on what was supposed to be material with no drum in
    # it, and it was right to. The control has to be free of transients the
    # detector is CORRECT to find, or it measures the synthesis and not the floor.
    phase_bass = 0.0
    phase_tri = 0.0
    for i in range(n):
        t = i / RATE
        # A note every 500 ms, each with a 30 ms attack and a slow decay. This is
        # the part that gives the control teeth: a bass note starting IS a rise
        # in band 0, and it is not a drum. A drone would have been silent at
        # every floor and would have proved nothing. 30 ms is a plucked or bowed
        # bass; a zabumba's mallet stroke is nearer 5, and telling those apart is
        # what the floor is being asked to do.
        note_t = t % 0.5
        f0 = bass_notes[int(t / 0.5) % len(bass_notes)]
        note_env = min(1.0, note_t / 0.030) * math.exp(-note_t * 1.2)
        phase_bass += 2 * math.pi * f0 / RATE
        phase_tri += 2 * math.pi * 6300.0 / RATE
        # Calibrated, not chosen: this puts band 0's median at ~0.03, which is
        # where the anchor corpus actually sits (Chororo 0.031, Alumiar 0.029)
        # and what analysis.cpp records for real forro. The first draft used
        # 0.20, which put band 0 at 0.31 -- ten times the music. At that level a
        # steady sine's own leakage ripple, from a 55 Hz period that does not
        # divide the window, swings the band by +-0.02 frame to frame, which is
        # the whole flux floor. The control was measuring its own synthesis.
        env = 0.0137 * (0.95 + 0.05 * math.sin(2 * math.pi * 0.3 * t)) * note_env
        v = env * math.sin(phase_bass)
        v += 0.5 * env * math.sin(3 * phase_bass)            # reedy, not pure
        # Right hand, well above band 0.
        for h in (392.0, 493.88, 587.33):
            v += 0.05 * math.sin(2 * math.pi * h * t)
        # Triangle: eighths at 120 BPM, 3 ms attack then decay, 6.3 kHz.
        ph = t % 0.25
        if ph < 0.12:
            attack = min(1.0, ph / 0.003)
            v += 0.10 * attack * math.exp(-ph * 30.0) * math.sin(phase_tri)
        v += rng.uniform(-1.0, 1.0) * 0.0005                 # dither
        s = max(-32768, min(32767, int(v * 20000)))
        frames += struct.pack("<hh", s, s)
    with wave.open(str(path), "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(bytes(frames))
    print(f"wrote {path} ({seconds} s)", file=sys.stderr)


# --------------------------------------------------------------------------
# running pattern_lab

def newest_source(*dirs):
    """
    mtime of the most recently touched file that goes into a host binary.

    The cache below is keyed on (hop, hist) alone, which was silently wrong the
    first time the component grew a feature: pattern_lab acquired --dump in
    ac311de and every cached binary predated it, so a sweep run afterwards was
    measuring the previous month's detector and reporting it as today's. A
    cached artefact older than its sources is the one failure mode a tuning
    harness cannot be allowed to have, since nothing about the output says which
    code produced it.
    """
    newest = 0.0
    for d in dirs:
        for pat in ("*.c", "*.cpp", "*.h", "*.hpp", "Makefile"):
            for f in list(d.glob(pat)) + list((d / "include").glob(pat)):
                newest = max(newest, f.stat().st_mtime)
    return newest


def build_lab(hop, hist=None):
    """One binary per (hop, hist), cached, because a sweep rebuilds otherwise."""
    tag = f"hop{hop}" + (f".hist{hist}" if hist else "")
    out = CACHE / "bin" / f"pattern_lab.{tag}"
    if out.exists() and out.stat().st_mtime >= newest_source(LAB, ROOT / "components" / "dancefloor_leds"):
        return out
    out.parent.mkdir(parents=True, exist_ok=True)
    args = ["make", "-C", str(LAB), f"HOP={hop}"]
    if hist:
        args.append(f"HIST={hist}")
    subprocess.run(["make", "-C", str(LAB), "clean"], check=True, stdout=subprocess.DEVNULL)
    subprocess.run(args, check=True, stdout=subprocess.DEVNULL)
    shutil.copy2(LAB / "pattern_lab", out)
    subprocess.run(["make", "-C", str(LAB), "clean"], check=True, stdout=subprocess.DEVNULL)
    return out


def run_lab(binary, wav, csv, flags=()):
    subprocess.run([str(binary), str(wav), "--no-tty", "--csv", str(csv), *flags],
                   check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def run_dump(binary, wav, prefix):
    subprocess.run([str(binary), str(wav), "--no-tty", "--dump", str(prefix)],
                   check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def read_band_dump(prefix):
    """
    Frame::band per frame, from pattern_lab --dump.

    The reader half of this format is tools/pattern_lab/dump_load.py, which
    needs numpy and says so. This one does not: four float32 per frame is a
    struct.unpack, and everything here has to keep running on the stdlib.
    Both read the same .meta sidecar, so a truncated file is caught here for
    the same reason it is caught there.
    """
    meta = {}
    for line in Path(str(prefix) + ".meta").read_text().splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, v = line.split("=", 1)
            meta[k.strip()] = v.strip()
    frames, bins = int(meta["frames"]), int(meta["band_bins"])
    raw = Path(str(prefix) + ".band.bin").read_bytes()
    want = frames * bins * 4
    if len(raw) != want:
        raise SystemExit(f"{prefix}.band.bin: {len(raw)} bytes, expected {want}")
    flat = struct.unpack(f"<{frames * bins}f", raw)
    return meta, [flat[i * bins:(i + 1) * bins] for i in range(frames)]


def metrics(csv):
    """
    Everything a table needs, from one pass over the trace.

    Counted from the CSV rather than from pattern_lab's stderr summary so that
    the boom count and the marginal rate are known to come from the same frames.
    """
    frames = 0
    booms = onsets = 0
    marg_boom = marg_beat = 0
    boom_flux = []
    boom_thr = []
    boom_at = []
    onset_at = []
    with open(csv) as f:
        for line in f:
            if line[0] == "#" or line[0] == "b":       # header comment, column header
                continue
            c = line.rstrip("\n").split(",")
            if len(c) < 14:
                continue
            frames += 1
            t = float(c[1])
            flux, thr = float(c[6]), float(c[7])
            bflux, bthr = float(c[10]), float(c[11])
            if c[8] == "1":
                onsets += 1
                onset_at.append(t)
            if c[12] == "1":
                booms += 1
                boom_at.append(t)
            if bthr > 0 and abs(bflux - bthr) < MARGIN * bthr:
                marg_boom += 1
            if thr > 0 and abs(flux - thr) < MARGIN * thr:
                marg_beat += 1
            boom_flux.append(bflux)
            boom_thr.append(bthr)
    return {
        "frames": frames,
        "booms": booms,
        "onsets": onsets,
        "marginal_boom": marg_boom / frames if frames else 0.0,
        "marginal_beat": marg_beat / frames if frames else 0.0,
        "flux_p50": pctile(boom_flux, 0.50),
        "flux_p90": pctile(boom_flux, 0.90),
        "flux_p99": pctile(boom_flux, 0.99),
        "thr_sd": sd(boom_thr),
        "boom_at": boom_at,
        "onset_at": onset_at,
    }


def pctile(v, p):
    if not v:
        return 0.0
    s = sorted(v)
    i = max(0, math.ceil(p * len(s)) - 1)
    return s[i]


def sd(v):
    if len(v) < 2:
        return 0.0
    m = sum(v) / len(v)
    return math.sqrt(sum((x - m) ** 2 for x in v) / len(v))


def per_min(count, wav):
    _, secs = wav_info(wav)
    return count * 60.0 / secs if secs else 0.0


def measure(track, binary, flags=()):
    with tempfile.NamedTemporaryFile(suffix=".csv", delete=False) as tf:
        csv = Path(tf.name)
    try:
        run_lab(binary, track.wav, csv, flags)
        m = metrics(csv)
    finally:
        csv.unlink(missing_ok=True)
    _, secs = wav_info(track.wav)
    m["secs"] = secs
    m["booms_min"] = m["booms"] * 60.0 / secs if secs else 0.0
    m["onsets_min"] = m["onsets"] * 60.0 / secs if secs else 0.0
    m["label"] = track.label
    m["group"] = track.group
    return m


def measure_all(tracks, binary, flags=(), jobs=None):
    jobs = jobs or max(1, os.cpu_count() - 2)
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as ex:
        return list(ex.map(lambda t: measure(t, binary, flags), tracks))


# --------------------------------------------------------------------------
# reporting helpers

def rng_str(vals, fmt="{:.0f}"):
    if not vals:
        return "-"
    return f"{fmt.format(min(vals))}-{fmt.format(max(vals))}"


def med(vals):
    return pctile(vals, 0.50)


# --------------------------------------------------------------------------
# subcommands

def cmd_manifest(args):
    tracks = corpus(args.groups)
    print("| Track | Set | Source | s | Hz | sha256 |")
    print("|---|---|---|---|---|---|")
    for t in tracks:
        rate, secs = wav_info(t.wav)
        if t.span:
            # A segment's source sha256 is shared with its siblings, so it does
            # not identify the material a label is keyed to. The cut does.
            kind = f"{t.source.suffix.lstrip('.')}, cut {t.span[0]:g}-{t.span[1]:g} s"
            digest = sha256(t.wav)
        else:
            kind = "wav" if t.source.suffix == ".wav" else "mp3, decoded"
            digest = sha256(t.source)
        print(f"| {t.label} | {t.group} | {kind} | {secs:.0f} | {rate} | {digest} |")
    if any(t.span for t in tracks):
        print()
        print("Segment sha256s are of the cut WAV, since the labels are instants")
        print("inside it. Source recordings:")
        print()
        print("| Recording | s | sha256 (source) |")
        print("|---|---|---|")
        for src in zab_sources():
            _, secs = wav_info(zab_full_wav(src))
            print(f"| {src.name} | {secs:.0f} | {sha256(src)} |")
    ver = subprocess.run(["ffmpeg", "-version"], capture_output=True, text=True)
    print()
    print(f"decoded with: {' '.join(FFMPEG)}")
    print(f"ffmpeg: {ver.stdout.splitlines()[0]}")


def cmd_control(args):
    make_control(CACHE / "control.wav")
    print(f"sha256 {sha256(CACHE / 'control.wav')}")


def cmd_baseline(args):
    """
    The gate. analysis.cpp records, at hop 1024 over the ten anchor tracks:
    median low-band flux 0.0000, p90 0.016-0.019, p99 0.038-0.131, and 68-134
    booms/min at floor 0.02. If this does not land there the harness is not
    measuring what the record measured, and nothing after it means anything.
    """
    lab = build_lab(1024)
    rows = measure_all(corpus(["anchor"]), lab)
    print("| Track | booms/min | onsets/min | marginal/frame | flux p50 | p90 | p99 |")
    print("|---|---|---|---|---|---|---|")
    for r in sorted(rows, key=lambda r: r["label"]):
        print(f"| {r['label']} | {r['booms_min']:.1f} | {r['onsets_min']:.1f} | "
              f"{100*r['marginal_boom']:.1f}% | {r['flux_p50']:.4f} | "
              f"{r['flux_p90']:.4f} | {r['flux_p99']:.4f} |")
    print()
    print(f"booms/min range   {rng_str([r['booms_min'] for r in rows], '{:.0f}')}   "
          f"(record: 68-134)")
    print(f"flux p90 range    {rng_str([r['flux_p90'] for r in rows], '{:.4f}')}   "
          f"(record: 0.016-0.019)")
    print(f"flux p99 range    {rng_str([r['flux_p99'] for r in rows], '{:.4f}')}   "
          f"(record: 0.038-0.131)")
    print(f"flux p50 max      {max(r['flux_p50'] for r in rows):.4f}   (record: 0.0000)")


def zab_seconds(binary, wav, window=1.0):
    """
    Per-window statistics for segmenting a recording into drumming and talking.

    Three numbers, all from Frame::band so that nothing here can measure
    something a satellite could not -- a rule rather than a coincidence:

      b0      median band 0 -- 43-129 Hz, the mallet stroke's own band
      rise    p90 of the frame-to-frame rise in band 0 -- transients, not level,
              which is what separates drumming from a sustained low note
      b1      median band 1 -- 172-1034 Hz, where a voice's vowels are
      b2      median band 2 -- 1034-5039 Hz, where a voice's consonants are and
              where the drum has almost nothing

    A drum passage is high `rise`; a talking passage is low `rise` with band 1
    still present, which is also what separates talking from the silence
    between takes. Level alone would not do it: a ringing drum head and a
    speaking voice can share a band-0 median, and only the rise tells them
    apart. b2 was the obvious choice for the speech half and is the wrong one --
    on these recordings it sits at 0.001-0.008 whether anyone is talking or
    not, while b1 moves by a factor of ten.
    """
    with tempfile.TemporaryDirectory() as td:
        prefix = Path(td) / "seg"
        run_dump(binary, wav, prefix)
        meta, band = read_band_dump(prefix)
    hop, rate = int(meta["hop"]), int(meta["rate"])
    per = max(1, int(window * rate / hop))
    out = []
    for i in range(0, len(band), per):
        w = band[i:i + per]
        if len(w) < per // 2:
            break
        rises = [max(0.0, w[j][0] - w[j - 1][0]) for j in range(1, len(w))]
        out.append({
            "t": i * hop / rate,
            "b0": pctile([f[0] for f in w], 0.50),
            "b1": pctile([f[1] for f in w], 0.50),
            "b2": pctile([f[2] for f in w], 0.50),
            "rise": pctile(rises, 0.90),
        })
    return out


def cmd_segments(args):
    """
    Propose the segment table, which is then confirmed by ear and checked in.

    Proposed rather than written: this classifier is a convenience for finding
    the boundaries, not evidence. What is checked in is what a human confirmed,
    and that distinction is the whole point: a tuning input with no provenance
    is worth nothing.
    """
    lab = build_lab(args.hop)
    print("# proposed segments -- confirm by ear before checking in")
    print(f"# rise >= {args.rise} -> drum;  rise < {args.rise} and b1 >= {args.b1} -> speech")
    print("file,start_s,end_s,label")
    for src in zab_sources():
        wav = zab_full_wav(src)
        secs = zab_seconds(lab, wav, args.window)
        if args.raw:
            print(f"# {src.stem}", file=sys.stderr)
            for r in secs:
                print(f"#   {r['t']:6.1f}  b0={r['b0']:.4f} b1={r['b1']:.4f} b2={r['b2']:.4f} "
                      f"rise={r['rise']:.4f}", file=sys.stderr)
        klass = []
        for r in secs:
            if r["rise"] >= args.rise:
                klass.append("drum")
            elif r["b1"] >= args.b1:
                klass.append("speech")
            else:
                klass.append("quiet")
        # Runs, with short interruptions absorbed: a bar's rest inside a groove
        # is not a segment boundary, and one struck drum inside a sentence is
        # not a drum segment.
        runs = []
        for i, k in enumerate(klass):
            if runs and runs[-1][0] == k:
                runs[-1][2] = i + 1
            else:
                runs.append([k, i, i + 1])
        changed = True
        while changed:
            changed = False
            for i in range(1, len(runs) - 1):
                if (runs[i][2] - runs[i][1]) * args.window < args.min_run and \
                        runs[i - 1][0] == runs[i + 1][0]:
                    runs[i - 1][2] = runs[i + 1][2]
                    del runs[i:i + 2]
                    changed = True
                    break
        for k, a, b in runs:
            span = (b - a) * args.window
            if k == "quiet" or span < args.min_seg:
                continue
            print(f"{src.stem},{a * args.window:g},{b * args.window:g},{k}")


# --------------------------------------------------------------------------
# ground truth
#
# The corpus has never had any. Every table the sweep produces is a rate -- booms
# per minute against a floor -- plus a synthetic negative control, and a
# rate says how OFTEN the detector fired, never whether it fired on a drum
# stroke. On a full mix that gap cannot be closed: a suppressed stroke and an
# absent one look identical from the outside. On isolated drumming it can, and
# this is the machinery for closing it.
#
# The candidates below are generated deliberately over-sensitively. Rejecting is
# quick and hunting is not, so the human pass should be reading a list that is
# too long rather than searching the audio for what is missing.

CAND_REFRACTORY_S = 0.060      # two candidates closer than this are one stroke
CAND_RISE = 0.004              # deliberately far under any shipped flux floor


def zab_candidates(binary, wav, offset=0.0):
    """
    Every plausible stroke instant in one segment, with a guess at its class.

    Peaks in band 0's rise and in band 2's rise, merged. Both are needed: band 0
    finds the mallet stroke on the big head, band 2 finds the stick stroke on
    the thin one, and the second is not a distraction but the point -- the
    tapa-confusion figure in cmd_zabumba is the number the band-0-only design
    exists to keep small, and it cannot be computed without tapas labelled.

    The guess is which of the two bands rose more. It is a guess; the ear
    decides.
    """
    with tempfile.TemporaryDirectory() as td:
        prefix = Path(td) / "cand"
        run_dump(binary, wav, prefix)
        meta, band = read_band_dump(prefix)
    hop, rate = int(meta["hop"]), int(meta["rate"])
    rise = [[0.0] * 4]
    for i in range(1, len(band)):
        rise.append([max(0.0, band[i][k] - band[i - 1][k]) for k in range(4)])

    peaks = []
    for k in (0, 2):
        for i in range(2, len(rise) - 2):
            v = rise[i][k]
            if v < CAND_RISE:
                continue
            # A local maximum over the window either side, so one stroke's
            # attack ramp does not become four candidates.
            if any(rise[j][k] > v for j in range(i - 2, i + 3) if j != i):
                continue
            peaks.append((i, k, v))
    peaks.sort()

    out = []
    for i, k, v in peaks:
        t = offset + i * hop / rate
        if out and t - out[-1][0] < CAND_REFRACTORY_S:
            # Same stroke seen in both bands, or the tail of the previous one.
            # Keep the stronger, and let a mallet stroke win a tie: a boom
            # mislabelled tapa is the error that costs a real detection.
            if v > out[-1][2] or k == 0:
                out[-1] = (out[-1][0], k if v > out[-1][2] else out[-1][1], max(v, out[-1][2]))
            continue
        out.append((t, k, v))
    return [(t, "boom" if k == 0 else "tapa") for t, k, _ in out]


def write_click_track(wav, cands, offset, dst):
    """
    The segment in one ear, a click at each candidate in the other.

    This is the whole verification interface, and it is one file rather than a
    UI because the judgement being asked for is "did that click land on a
    stroke, and was it the right kind" -- which is a listening task. Boom
    guesses click high, tapa guesses click low, so the class is audible too and
    a mislabelled stroke does not need to be looked up.
    """
    with wave.open(str(wav), "rb") as r:
        rate, ch, width = r.getframerate(), r.getnchannels(), r.getsampwidth()
        n = r.getnframes()
        raw = r.readframes(n)
    if width != 2:
        raise SystemExit(f"{wav}: expected 16-bit PCM")
    pcm = list(struct.unpack(f"<{len(raw) // 2}h", raw))
    left = [pcm[i * ch] for i in range(n)] if ch > 1 else pcm[:]

    right = [0] * n
    dur = int(0.004 * rate)
    for t, klass in cands:
        start = int((t - offset) * rate)
        f = 2400.0 if klass == "boom" else 5200.0
        for j in range(dur):
            i = start + j
            if 0 <= i < n:
                env = 1.0 - j / dur
                right[i] = max(-32768, min(32767,
                              right[i] + int(9000 * env * math.sin(2 * math.pi * f * j / rate))))
    with wave.open(str(dst), "wb") as o:
        o.setnchannels(2)
        o.setsampwidth(2)
        o.setframerate(rate)
        o.writeframes(struct.pack(f"<{2 * n}h",
                                  *[v for pair in zip(left, right) for v in pair]))


def read_labels():
    """file -> [(t_s, class)], from the checked-in hand-verified table."""
    out = {}
    if not ZAB_LABELS.exists():
        return out
    for line in ZAB_LABELS.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or line.startswith("file,"):
            continue
        stem, t, klass = line.rsplit(",", 2)
        out.setdefault(stem, []).append((float(t), klass))
    for v in out.values():
        v.sort()
    return out


def cmd_label(args):
    """
    Generate candidates and a click track for the human pass.

    Instants are absolute seconds into the full recording, not offsets into a
    segment, so that moving a segment boundary later does not silently shift
    every label inside it.
    """
    lab = build_lab(args.hop)
    have = read_labels()
    rows = [r for r in zab_segment_rows() if r[3] == "drum"]
    if args.track:
        rows = [r for r in rows if args.track.lower() in r[0].lower()]
    if not rows:
        raise SystemExit("no drum segments match --track")
    by_stem = {p.stem: p for p in zab_sources()}

    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)
    written = []
    for stem, start, end, _ in rows:
        if stem in have and not args.force:
            print(f"skip {stem} [{start:g}-{end:g}] -- already in {ZAB_LABELS.name} "
                  f"({len(have[stem])} rows); --force to regenerate", file=sys.stderr)
            continue
        src = by_stem[stem]
        wav = zab_cut(src, start, end)
        cands = zab_candidates(lab, wav, offset=start)
        click = outdir / f"{stem}.{start:g}-{end:g}.click.wav"
        write_click_track(wav, cands, start, click)
        written.append((stem, start, end, cands, click))
        print(f"{stem} [{start:g}-{end:g}]  {len(cands)} candidates  -> {click}",
              file=sys.stderr)

    if not written:
        return
    print("file,t_s,class")
    for stem, _, _, cands, _ in written:
        for t, klass in cands:
            print(f"{stem},{t:.3f},{klass}")


def cmd_sweep(args):
    lab = build_lab(args.hop, args.hist)
    tracks = corpus(args.groups)
    control = corpus(["control"])[0]
    key = "boom" if args.detector == "boom" else "beat"
    print(f"# {args.detector} flux floor, hop {args.hop}"
          + (f", BEAT_HIST {args.hist}" if args.hist else ""))
    print()
    print("| Floor | Set | rate/min | marginal/frame (median) | marginal range | control |")
    print("|---|---|---|---|---|---|")
    for floor in args.floors:
        flag = ["--boom-floor", str(floor)] if key == "boom" else ["--beat-floor", str(floor)]
        rows = measure_all(tracks, lab, flag)
        cr = measure(control, lab, flag)
        for group in args.groups:
            g = [r for r in rows if r["group"] == group]
            if not g:
                continue
            rate = [r["booms_min"] if key == "boom" else r["onsets_min"] for r in g]
            marg = [100 * r[f"marginal_{key}"] for r in g]
            cnt = cr["booms"] if key == "boom" else cr["onsets"]
            print(f"| {floor} | {group} | {rng_str(rate)} | {med(marg):.1f}% | "
                  f"{rng_str(marg, '{:.1f}')}% | {cnt} |")


def cmd_hist(args):
    """
    BEAT_HIST on both of its axes at once, because it is the only constant here
    whose meaning changed rather than its scale, and because the second axis is
    invisible to the corpus.

    Left half: what the music says -- rate, marginal frames, and how noisy the
    threshold estimate is. Right half: what a lost frame costs, from converge,
    which is the sync axis the third source mode will care about.
    """
    tracks = corpus(args.groups)
    print(f"# BEAT_HIST at hop {args.hop}")
    print()
    print("| HIST | span | onsets/min | booms/min | marginal beat | marginal boom "
          "| threshold sd | converge p50 | p95 | p95 ms | clean trials |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    probe_csv = None
    for hist in args.hists:
        lab = build_lab(args.hop, hist)
        rows = measure_all(tracks, lab)
        span_ms = hist * args.hop * 1000.0 / RATE

        # One trace for the probe, written once and reused at every HIST. The
        # bands are an output of the FFT and the band edges, neither of which
        # BEAT_HIST touches, so re-deriving them per row would only introduce a
        # second thing that moves.
        probe_csv = CACHE / f"probe.hop{args.hop}.csv"
        if not probe_csv.exists():
            run_lab(lab, tracks[0].wav, probe_csv)
        conv = subprocess.run(
            [str(build_converge(hist, args.hop)), str(probe_csv),
             "--detector", "beat", "--gap", str(args.gap)],
            capture_output=True, text=True, check=True).stdout.split()
        c = dict(kv.split("=", 1) for kv in conv)

        print(f"| {hist} | {span_ms:.0f} ms | "
              f"{med([r['onsets_min'] for r in rows]):.1f} | "
              f"{med([r['booms_min'] for r in rows]):.1f} | "
              f"{med([100*r['marginal_beat'] for r in rows]):.1f}% | "
              f"{med([100*r['marginal_boom'] for r in rows]):.1f}% | "
              f"{med([r['thr_sd'] for r in rows]):.5f} | "
              f"{c['converge_p50']} | {c['converge_p95']} | {c['converge_p95_ms']} | "
              f"{c['clean_trials']} |")


def build_converge(hist, hop):
    out = CACHE / "bin" / f"converge.hop{hop}.hist{hist}"
    if out.exists() and out.stat().st_mtime >= newest_source(HERE, ROOT / "components" / "dancefloor_leds"):
        return out
    out.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(["make", "-C", str(HERE), "clean"], check=True, stdout=subprocess.DEVNULL)
    subprocess.run(["make", "-C", str(HERE), f"HIST={hist}", f"HOP={hop}"],
                   check=True, stdout=subprocess.DEVNULL)
    shutil.copy2(HERE / "converge", out)
    subprocess.run(["make", "-C", str(HERE), "clean"], check=True, stdout=subprocess.DEVNULL)
    return out


def cmd_instants(args):
    """
    Whether it is the same detector on a finer grid, or a different one with a
    similar count. Counts alone cannot tell those apart -- a matched count with
    shifted timing is a different detector -- so every hop-1024 boom is matched
    to the nearest hop-512 boom and the distances are reported.
    """
    flags = []
    if args.boom_floor is not None:
        flags = ["--boom-floor", str(args.boom_floor)]
    coarse = build_lab(1024, args.hist)
    fine = build_lab(512, args.hist)
    tracks = corpus(["anchor"])
    print("| Track | booms 1024 | booms 512 | within 12 ms | within 23 ms | median dt | "
          "unmatched 1024 | unmatched 512 |")
    print("|---|---|---|---|---|---|---|---|")
    tot = [0, 0, 0, 0, 0, 0]
    for t in tracks:
        a = measure(t, coarse, flags)["boom_at"]
        b = measure(t, fine, flags)["boom_at"]
        d = [min((abs(x - y) for y in b), default=9.9) for x in a]
        w12 = sum(1 for x in d if x <= 0.012)
        w23 = sum(1 for x in d if x <= 0.023)
        # Unmatched in the other direction: a hop-512 boom with no hop-1024 boom
        # near it is extra sensitivity, which is expected and is not a failure.
        e = [min((abs(y - x) for x in a), default=9.9) for y in b]
        u512 = sum(1 for x in e if x > 0.023)
        print(f"| {t.label} | {len(a)} | {len(b)} | {100*w12/max(1,len(a)):.0f}% | "
              f"{100*w23/max(1,len(a)):.0f}% | {1000*med(d):.0f} ms | "
              f"{len(a)-w23} | {u512} |")
        tot[0] += len(a); tot[1] += len(b); tot[2] += w12; tot[3] += w23
        tot[4] += len(a) - w23; tot[5] += u512
    print()
    print(f"totals: {tot[0]} booms at 1024, {tot[1]} at 512; "
          f"{100*tot[2]/max(1,tot[0]):.1f}% within one hop, "
          f"{100*tot[3]/max(1,tot[0]):.1f}% within one window; "
          f"{tot[4]} hop-1024 booms unmatched, {tot[5]} hop-512 booms new")


# --------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    def groups_arg(sp, default=("anchor", "breadth")):
        sp.add_argument("--groups", nargs="+", default=list(default),
                        choices=["anchor", "breadth", "control",
                                 "zabumba", "zabumba-speech"])

    m = sub.add_parser("manifest"); groups_arg(m); m.set_defaults(fn=cmd_manifest)
    c = sub.add_parser("control"); c.set_defaults(fn=cmd_control)
    b = sub.add_parser("baseline"); b.set_defaults(fn=cmd_baseline)

    g = sub.add_parser("segments")
    g.add_argument("--hop", type=int, default=512)
    g.add_argument("--window", type=float, default=1.0)
    g.add_argument("--rise", type=float, default=0.02)
    g.add_argument("--b1", type=float, default=0.004)
    g.add_argument("--min-run", type=float, default=3.0)
    g.add_argument("--min-seg", type=float, default=5.0)
    g.add_argument("--raw", action="store_true")
    g.set_defaults(fn=cmd_segments)

    l = sub.add_parser("label")
    l.add_argument("--track", help="substring of the recording name; default all")
    l.add_argument("--hop", type=int, default=512)
    l.add_argument("--out", default=str(CACHE / "clicks"))
    l.add_argument("--force", action="store_true",
                   help="regenerate candidates for a recording that already has labels")
    l.set_defaults(fn=cmd_label)

    s = sub.add_parser("sweep"); groups_arg(s)
    s.add_argument("--detector", choices=["boom", "beat"], default="boom")
    s.add_argument("--hop", type=int, default=512)
    s.add_argument("--hist", type=int)
    s.add_argument("--floors", type=float, nargs="+",
                   default=[0.012, 0.016, 0.02, 0.024, 0.028, 0.03, 0.06])
    s.set_defaults(fn=cmd_sweep)

    h = sub.add_parser("hist"); groups_arg(h)
    h.add_argument("--hop", type=int, default=512)
    h.add_argument("--hists", type=int, nargs="+", default=[22, 43, 64, 86])
    h.add_argument("--gap", type=int, default=10)
    h.set_defaults(fn=cmd_hist)

    i = sub.add_parser("instants")
    i.add_argument("--boom-floor", type=float)
    i.add_argument("--hist", type=int)
    i.set_defaults(fn=cmd_instants)

    args = p.parse_args()
    CACHE.mkdir(parents=True, exist_ok=True)
    args.fn(args)


if __name__ == "__main__":
    main()
