from __future__ import annotations

import dataclasses
import pathlib

import numpy as np


@dataclasses.dataclass
class Dump:

    spec: np.ndarray
    mag: np.ndarray
    band: np.ndarray
    meta: dict
    index: np.ndarray
    due_us: np.ndarray
    time_s: np.ndarray


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
    prefix = pathlib.Path(prefix)
    meta_path = prefix if str(prefix).endswith(".meta") else pathlib.Path(str(prefix) + ".meta")
    meta = _read_meta(meta_path)

    frames = int(meta["frames"])
    hop, rate = int(meta["hop"]), int(meta["rate"])

    spec = _load_array(pathlib.Path(str(prefix) + ".spec.bin"), meta["spec_dtype"], frames, int(meta["spec_bins"]))
    mag = _load_array(pathlib.Path(str(prefix) + ".mag.bin"), meta["mag_dtype"], frames, int(meta["mag_bins"]), mmap)
    band = _load_array(pathlib.Path(str(prefix) + ".band.bin"), meta["band_dtype"], frames, int(meta["band_bins"]))

    index = np.arange(frames, dtype=np.int64)
    due_us = index * np.int64(hop) * np.int64(1_000_000) // np.int64(rate)

    return Dump(spec=spec, mag=mag, band=band, meta=meta,
                index=index, due_us=due_us, time_s=due_us / 1e6)
