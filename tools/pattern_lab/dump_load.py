"""Load pattern_lab --dump output into numpy arrays.

The other half of the format contract lives in tools/pattern_lab/main.cpp
(--dump): four files per run, row-major, one row per analysis block, native
little-endian --

    PREFIX.spec.bin   uint8    frames x 64   Frame::spec, the bytes a
                                            satellite's analyser lane receives
    PREFIX.mag.bin    float32  frames x 512  Frame::mag, raw FFT magnitudes,
                                            bin k = k * rate / window Hz
    PREFIX.band.bin   float32  frames x 4    Frame::band, normalised 0..1
    PREFIX.meta       text                    key=value grid provenance

This module needs numpy and is meant for the notebook environment, which is
why it lives here rather than beside the stdlib-only tools/tuning scripts: it
is the reader half of a format whose writer is main.cpp, and the two change
together or not at all.
"""

from __future__ import annotations

import dataclasses
import pathlib

import numpy as np


@dataclasses.dataclass
class Dump:
    """One pattern_lab run.

    spec/mag/band are indexed [frame, bin]; index is 0..N-1; due_us is the
    firmware's own timing (int64 microseconds, exact integer arithmetic);
    time_s is due_us / 1e6 for plotting.
    """

    spec: np.ndarray    # (N, spec_bins)  uint8
    mag: np.ndarray     # (N, mag_bins)   float32
    band: np.ndarray    # (N, band_bins)  float32
    meta: dict
    index: np.ndarray   # (N,) int64
    due_us: np.ndarray  # (N,) int64
    time_s: np.ndarray  # (N,) float64


def _read_meta(path: pathlib.Path) -> dict:
    meta = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        meta[key.strip()] = value.strip()
    return meta


def _load_array(path: pathlib.Path, dtype: str, frames: int, bins: int, mmap: bool = False):
    # A binary format's one failure mode is silence: a truncated file loads as
    # a short array or a reshape error far from the cause. Size is known
    # exactly from the sidecar, so anything else is an error here, at the file.
    expect = frames * bins * np.dtype(dtype).itemsize
    actual = path.stat().st_size
    if actual != expect:
        raise ValueError(
            f"{path}: {actual} bytes, expected {expect} "
            f"({frames} frames x {bins} bins x {np.dtype(dtype).itemsize} B) "
            "-- truncated or stale dump?"
        )
    if mmap:
        return np.memmap(path, dtype=dtype, mode="r", shape=(frames, bins))
    return np.fromfile(path, dtype=dtype, count=frames * bins).reshape(frames, bins)


def load(prefix, mmap: bool = False) -> Dump:
    """Load a --dump run. `prefix` is the PREFIX passed to --dump.

    mmap=True maps mag instead of reading it, so a corpus of long tracks can
    be paged frame by frame without holding every float in RAM.
    """
    prefix = pathlib.Path(prefix)
    meta_path = prefix if str(prefix).endswith(".meta") else pathlib.Path(str(prefix) + ".meta")
    meta = _read_meta(meta_path)

    frames = int(meta["frames"])
    hop, rate = int(meta["hop"]), int(meta["rate"])

    spec = _load_array(pathlib.Path(str(prefix) + ".spec.bin"), meta["spec_dtype"], frames, int(meta["spec_bins"]))
    mag = _load_array(pathlib.Path(str(prefix) + ".mag.bin"), meta["mag_dtype"], frames, int(meta["mag_bins"]), mmap)
    band = _load_array(pathlib.Path(str(prefix) + ".band.bin"), meta["band_dtype"], frames, int(meta["band_bins"]))

    # Exactly main.cpp's due_us -- int64_t(b) * HOP_N * 1000000LL / rate --
    # so notebook timings and firmware timings are the same numbers, not
    # merely close ones.
    index = np.arange(frames, dtype=np.int64)
    due_us = index * np.int64(hop) * np.int64(1_000_000) // np.int64(rate)

    return Dump(spec=spec, mag=mag, band=band, meta=meta,
                index=index, due_us=due_us, time_s=due_us / 1e6)
