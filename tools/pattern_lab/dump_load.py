"""Load pattern_lab --dump output into numpy arrays.

The writer half of this format is main.cpp, and the two change together or not
at all. Four files per run, row-major, one row per analysis block, native
little-endian:

    PREFIX.spec.bin   uint8    frames x 64   Frame::spec, the bytes a
                                             satellite's analyser lane receives
    PREFIX.mag.bin    float32  frames x 512  Frame::mag, raw FFT magnitudes;
                                             bin k is k * rate / window Hz
    PREFIX.band.bin   float32  frames x 4    Frame::band, normalised 0..1
    PREFIX.meta       text                   key=value grid provenance

Needs numpy, which is why it lives here beside the notebook work rather than
with the stdlib-only scripts under tools/tuning.
"""

from __future__ import annotations

import dataclasses
import pathlib

import numpy as np


@dataclasses.dataclass
class Dump:
    """
    @brief One pattern_lab run, loaded.
    """


    ## @brief (N, spec_bins) uint8, indexed [frame, bin].
    spec: np.ndarray
    ## @brief (N, mag_bins) float32, indexed [frame, bin].
    mag: np.ndarray
    ## @brief (N, band_bins) float32, indexed [frame, band].
    band: np.ndarray
    ## @brief The .meta sidecar, parsed; every value is still a string.
    meta: dict
    ## @brief (N,) int64, 0..N-1.
    index: np.ndarray
    ## @brief (N,) int64. The firmware's own timing, in exact integer microseconds.
    due_us: np.ndarray
    ## @brief (N,) float64. due_us / 1e6, for plotting against seconds.
    time_s: np.ndarray


def _read_meta(path: pathlib.Path) -> dict:
    """
    @brief Parse the .meta sidecar into a dict of strings.
    @param path  The sidecar.
    @return Every key=value line, comments and blanks skipped. Values are not
            converted -- the caller knows which are integers.
    """
    meta = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        meta[key.strip()] = value.strip()
    return meta


def _load_array(path: pathlib.Path, dtype: str, frames: int, bins: int, mmap: bool = False):
    """
    @brief Read one of the three binary files, checking its size first.

    A binary format's one failure mode is silence: a truncated file otherwise
    loads as a short array, or raises a reshape error far from the cause. The
    size is known exactly from the sidecar, so anything else is refused here,
    at the file.

    @param path    The .bin to read.
    @param dtype   Numpy dtype string, from the sidecar.
    @param frames  Rows expected.
    @param bins    Columns expected.
    @param mmap    Map the file instead of reading it.
    @return A (frames, bins) array.
    """
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
    """
    @brief Load a whole --dump run.

    @param prefix  The PREFIX that was passed to --dump, with or without the
                   trailing ".meta".
    @param mmap    Map mag rather than reading it, so a corpus of long tracks
                   can be paged frame by frame instead of held in RAM.
    @return The run.
    """
    prefix = pathlib.Path(prefix)
    meta_path = prefix if str(prefix).endswith(".meta") else pathlib.Path(str(prefix) + ".meta")
    meta = _read_meta(meta_path)

    frames = int(meta["frames"])
    hop, rate = int(meta["hop"]), int(meta["rate"])

    spec = _load_array(pathlib.Path(str(prefix) + ".spec.bin"), meta["spec_dtype"], frames, int(meta["spec_bins"]))
    mag = _load_array(pathlib.Path(str(prefix) + ".mag.bin"), meta["mag_dtype"], frames, int(meta["mag_bins"]), mmap)
    band = _load_array(pathlib.Path(str(prefix) + ".band.bin"), meta["band_dtype"], frames, int(meta["band_bins"]))

    # Exactly main.cpp's due_us -- int64_t(b) * HOP_N * 1000000LL / rate -- so
    # notebook timings and firmware timings are the same numbers rather than
    # merely close ones.
    index = np.arange(frames, dtype=np.int64)
    due_us = index * np.int64(hop) * np.int64(1_000_000) // np.int64(rate)

    return Dump(spec=spec, mag=mag, band=band, meta=meta,
                index=index, due_us=due_us, time_s=due_us / 1e6)
