from __future__ import annotations

import pathlib
import subprocess
import sys
import tempfile
import wave

import numpy as np

import dump_load

FFT_N = 1024
HOP_N = 512
RATE = 44100
BINS = FFT_N // 2
SPEC_BINS = 64
SPEC_LO_HZ = 40.0
SPEC_HI_HZ = 16000.0
BAND_EDGE_HZ = (43, 172, 1034, 5039)
BAND_GAIN = 12.0


def band_bin(hz: int, rate: int) -> int:
    return (hz * FFT_N + rate // 2) // rate


def band_bins(rate: int = RATE):
    lo = [min(max(band_bin(e, rate), 1), BINS - 1) for e in BAND_EDGE_HZ]
    hi = [lo[b + 1] - 1 if b + 1 < len(lo) else BINS - 1 for b in range(len(lo))]
    return lo, [max(h, l) for h, l in zip(hi, lo)]


def spec_bins(rate: int = RATE):
    ratio = np.power(np.float32(SPEC_HI_HZ / SPEC_LO_HZ),
                     np.float32(1.0 / SPEC_BINS), dtype=np.float32)
    edge = np.float32(SPEC_LO_HZ)
    los, his = [], []
    for _ in range(SPEC_BINS):
        nxt = np.float32(edge * ratio)
        lo = min(max(band_bin(int(edge + np.float32(0.5)), rate), 1), BINS - 1)
        hi = min(max(band_bin(int(nxt + np.float32(0.5)), rate) - 1, lo), BINS - 1)
        los.append(lo)
        his.append(hi)
        edge = nxt
    return los, his


def spec_edges_hz(rate: int = RATE):
    ratio = np.power(np.float32(SPEC_HI_HZ / SPEC_LO_HZ),
                     np.float32(1.0 / SPEC_BINS), dtype=np.float32)
    edge = np.float32(SPEC_LO_HZ)
    lo_hz, hi_hz = [], []
    for _ in range(SPEC_BINS):
        nxt = np.float32(edge * ratio)
        lo_hz.append(float(edge))
        hi_hz.append(float(nxt))
        edge = nxt
    return lo_hz, hi_hz


def hann(n: int = FFT_N) -> np.ndarray:
    i = np.arange(n, dtype=np.float32)
    m = np.float32(1.0) / np.float32(n - 1)
    t = i * np.float32(2.0)
    t = t * np.float32(np.pi)
    t = t * m
    return np.float32(0.5) * (np.float32(1.0) - np.cos(t, dtype=np.float32))


def read_wav(path):
    with wave.open(str(path), "rb") as w:
        if w.getsampwidth() != 2 or w.getcomptype() != "NONE":
            raise ValueError(f"{path}: only 16-bit PCM is supported")
        channels, rate = w.getnchannels(), w.getframerate()
        if channels not in (1, 2):
            raise ValueError(f"{path}: only mono or stereo")
        raw = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2")
    if channels == 1:
        pcm = np.empty(raw.size * 2, dtype=np.int16)
        pcm[0::2] = raw
        pcm[1::2] = raw
        return pcm.reshape(-1, 2), rate
    return raw[: raw.size // 2 * 2].reshape(-1, 2), rate


def _seq_band_sums(mag: np.ndarray, lo: int, hi: int) -> np.ndarray:
    out = mag[:, lo].copy()
    for k in range(lo + 1, hi + 1):
        out += mag[:, k]
    return out


def _band_value(mag: np.ndarray, lo: int, hi: int) -> np.ndarray:
    sums = _seq_band_sums(mag, lo, hi)
    v = (sums / np.float32(hi - lo + 1)) * np.float32(BAND_GAIN)
    v = (v / np.float32(FFT_N)) * np.float32(2.0)
    return np.where(v > 0, v / (np.float32(1.0) + v), np.float32(0.0)).astype(np.float32)


def analyse(pcm: np.ndarray, rate: int = RATE) -> dict:
    mono = (pcm[:, 0].astype(np.float32) + pcm[:, 1].astype(np.float32)) \
        / np.float32(65536.0)
    if mono.size < FFT_N:
        raise ValueError("too short to analyse")
    blocks = (mono.size - FFT_N) // HOP_N + 1

    windows = np.lib.stride_tricks.sliding_window_view(mono, FFT_N)[::HOP_N]
    spec_x = np.fft.rfft(windows * hann(), axis=1)
    mag = np.abs(spec_x)[:, :BINS].astype(np.float32)

    band_lo, band_hi = band_bins(rate)
    band = np.stack([_band_value(mag, band_lo[b], band_hi[b])
                     for b in range(len(band_lo))], axis=1)

    spec_lo, spec_hi = spec_bins(rate)
    spec = np.empty((blocks, SPEC_BINS), dtype=np.uint8)
    for s in range(SPEC_BINS):
        sums = _seq_band_sums(mag, spec_lo[s], spec_hi[s])
        v = (sums / np.float32(spec_hi[s] - spec_lo[s] + 1)) * np.float32(BAND_GAIN)
        v = (v / np.float32(FFT_N)) * np.float32(2.0)
        n = np.where(v > 0, v / (np.float32(1.0) + v), np.float32(0.0)).astype(np.float32)
        spec[:, s] = (n * np.float32(255.0) + np.float32(0.5)).astype(np.uint8)

    index = np.arange(blocks, dtype=np.int64)
    due_us = index * np.int64(HOP_N) * np.int64(1_000_000) // np.int64(rate)
    return {"mag": mag, "band": band, "spec": spec, "index": index, "due_us": due_us}


def analyse_wav(path, rate_check: bool = True) -> dict:
    pcm, rate = read_wav(path)
    if rate_check and rate != RATE:
        print(f"note: {path} is {rate} Hz -- analysed at that rate; the tuning "
              f"defaults were measured at {RATE}", file=sys.stderr)
    return analyse(pcm, rate)


def firmware_frames(wav_path, keep_prefix=None):
    here = pathlib.Path(__file__).resolve().parent
    lab = here / "pattern_lab"
    if not lab.exists():
        subprocess.run(["make", "-C", str(here)], check=True)
    if keep_prefix is not None:
        prefix = pathlib.Path(keep_prefix)
        prefix.parent.mkdir(parents=True, exist_ok=True)
        cmd = [str(lab), str(wav_path), "--no-tty", "--dump", str(prefix)]
        subprocess.run(cmd, check=True)
        return dump_load.load(prefix)
    with tempfile.TemporaryDirectory() as tmp:
        prefix = pathlib.Path(tmp) / "x"
        subprocess.run([str(lab), str(wav_path), "--no-tty", "--dump", str(prefix)],
                       check=True)
        return dump_load.load(prefix)


def _validate(wav_path) -> int:
    dump = firmware_frames(wav_path)
    ref = analyse_wav(wav_path)

    failures = []

    if dump.meta["hop"] != str(HOP_N):
        print(f"FAIL pattern_lab was built with hop={dump.meta['hop']} but this "
              f"module assumes {HOP_N} -- rebuild it (`make HOP={HOP_N}`)")
        return 1
    if dump.spec.shape != ref["spec"].shape:
        print(f"FAIL frame count: firmware {dump.spec.shape[0]}, reference "
              f"{ref['spec'].shape[0]}")
        return 1
    if not np.array_equal(dump.due_us, ref["due_us"]):
        failures.append("due_us timings differ")

    agree = (dump.spec == ref["spec"]).mean()
    spec_ok = agree >= 0.999
    print(f"{'PASS' if spec_ok else 'FAIL'} spec: {agree * 100:.4f}% of bytes "
          f"identical (>= 99.9%)")
    if not spec_ok:
        mismatches = (dump.spec != ref["spec"]).sum(axis=0)
        hot = np.nonzero(mismatches)[0]
        print(f"     per-bin mismatches concentrate at bins {hot.tolist()} -- "
              "the same bins every frame means a drifted spec edge, not noise")
        failures.append("spec byte agreement below threshold")

    floor = 0.01 * dump.mag.max()
    mask = dump.mag > floor
    rel = np.abs(dump.mag[mask].astype(np.float64) - ref["mag"][mask]) / dump.mag[mask]
    mag_ok = rel.max() <= 1e-4
    print(f"{'PASS' if mag_ok else 'FAIL'} mag: max relative error "
          f"{rel.max():.2e} over bins above {floor:.1f} (<= 1e-4)")
    if not mag_ok:
        failures.append("mag relative error above 1e-4")

    band_diff = np.abs(dump.band.astype(np.float64) - ref["band"]).max()
    band_ok = band_diff <= 5e-6
    print(f"{'PASS' if band_ok else 'FAIL'} band: max absolute difference "
          f"{band_diff:.2e} (<= 5e-6)")
    if not band_ok:
        failures.append("band difference above 5e-6")

    for f in failures:
        print(f"FAIL {f}")
    print("all checks passed" if not failures else f"{len(failures)} check(s) failed")
    return 0 if not failures else 1


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} TRACK.wav\n"
              "       compares this module's numpy pipeline against the "
              "firmware's own\n       compiled code over the same file",
              file=sys.stderr)
        sys.exit(2)
    sys.exit(_validate(sys.argv[1]))
