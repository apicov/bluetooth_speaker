"""The firmware's audio analysis, in numpy -- and the check that it stays true.

Two ways to get features out of a WAV, both chunked exactly as the strips are:

    pipeline.firmware_frames("song.wav")   run the firmware's own compiled code
                                           through pattern_lab --dump and load
                                           it; ground truth, costs a subprocess
    pipeline.analyse_wav("song.wav")       this module's numpy port of the same
                                           recipe; fast, for loops over a corpus
                                           and for tinkering

They agree to float rounding, and running this file measures that agreement
against a real track:

    pipeline.py TRACK.wav

so a change on either side that broke parity is caught rather than silently
trained on. The pipeline being mirrored is Analysis::process() in
components/dancefloor_leds/analysis.cpp, with beat_normalise() from
beat_detect.c and the constants from analysis_config.h and analysis.hpp.

Each stage below notes where a "clean" numpy version would silently diverge
from the C, and every one of those notes is load-bearing:

  mono     (L + R) / 65536.0 in float32. Exact -- the int16 sum fits a float32
           and the divide is a power of two. Not /32768 and not a mean, which
           land elsewhere on off-centre stereo.
  window   Symmetric Hann evaluated left to right in float32 like the C:
           float32(pi) is not pi, and the product order moves the last bit of
           every entry.
  fft      numpy's double-precision rfft. The firmware transforms float32
           throughout, so this stage carries relative slack that nothing
           downstream can remove; the check measures it rather than assuming it.
  banding  a sum over bins [lo, hi] accumulated SEQUENTIALLY in float32, then
           (sum/count) * BAND_GAIN / FFT_N * 2 in that order, one rounding per
           operation. np.sum and np.add.reduceat reassociate pairwise, which
           shows up in the low bits.
  compress v/(1+v) with beat_normalise's !(v > 0) -> 0 guard, NaN included.
           Soft compression: no log and no dB, anywhere.
  spec     quantised last, uint8(n*255 + 0.5); truncation is floor for the
           non-negative values this sees.
  edges    the 64 log-spaced bins are built by COMPOUNDING edge *= ratio in
           float32 sixty-four times, not by the closed form. The closed form
           drifts a fraction of a Hz and occasionally flips a bin.

Deliberately NOT ported: beat_det_update. It is sequential stateful C, and the
rule here is to run the real code rather than copy it -- onset, flux and
threshold come from pattern_lab --csv, which compiles beat_detect.c itself.
"""

from __future__ import annotations

import pathlib
import subprocess
import sys
import tempfile
import wave

import numpy as np

import dump_load

# The block below mirrors analysis_config.h, analysis.hpp and analysis.cpp. A
# value here that disagrees with the headers makes every figure this module
# produces describe a pipeline the firmware does not run -- which is what
# _validate() exists to catch, so change them together.

## @brief Window length, in samples. analysis_config.h's DF_FFT_N.
FFT_N = 1024
## @brief Samples between window starts. DF_HOP_N's default; a pattern_lab
#         built with `make HOP=` other than this fails _validate().
HOP_N = 512
## @brief The rate the tuning was measured at. Analysis follows the file.
RATE = 44100
## @brief Magnitude bins kept: DC is computed, Nyquist never is.
BINS = FFT_N // 2
## @brief Bands in the portable spectrum. DF_SPEC_BINS.
SPEC_BINS = 64
## @brief Bottom of the portable spectrum, in Hz.
SPEC_LO_HZ = 40.0
## @brief Top of the portable spectrum, in Hz.
SPEC_HI_HZ = 16000.0
## @brief Lower edge of each of the four detector bands, in Hz.
BAND_EDGE_HZ = (43, 172, 1034, 5039)
## @brief The gain applied to a band mean before compression.
BAND_GAIN = 12.0


def band_bin(hz: int, rate: int) -> int:
    """
    @brief analysis.hpp's bin for a frequency: nearest bin, rounding half up.
    @param hz    The frequency.
    @param rate  The sample rate the analysis is running at.
    @return The bin index, unclamped.
    """
    return (hz * FFT_N + rate // 2) // rate


def band_bins(rate: int = RATE):
    """
    @brief The four detector bands as bin ranges.

    Carries Analysis::init()'s clamps: never DC, never past Nyquist-1, never
    empty.

    @param rate  The sample rate the analysis is running at.
    @return ([lo...], [hi...]), both inclusive.
    """
    lo = [min(max(band_bin(e, rate), 1), BINS - 1) for e in BAND_EDGE_HZ]
    hi = [lo[b + 1] - 1 if b + 1 < len(lo) else BINS - 1 for b in range(len(lo))]
    return lo, [max(h, l) for h, l in zip(hi, lo)]


def spec_bins(rate: int = RATE):
    """
    @brief The 64 portable-spectrum bands as bin ranges.

    Reproduces the C's compounding loop rather than the closed form: edge
    starts at SPEC_LO_HZ and is multiplied by the ratio sixty-four times, each
    step in float32. So these are the firmware's bins at any rate -- several
    low bands sharing one FFT bin included, which happens there too.

    @param rate  The sample rate the analysis is running at.
    @return ([lo...], [hi...]), both inclusive.
    """
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
    """
    @brief The Hz edges of the 64 spec bins, for axis labels and training
           targets.

    The same compounding loop as spec_bins(), kept in float32 so a label
    always describes the bin beside it.

    @param rate  Accepted for symmetry with spec_bins(); the edges do not
                 depend on it.
    @return ([lo_hz...], [hi_hz...]).
    """
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
    """
    @brief Symmetric Hann window, in the C's evaluation order.

    i*2 in float32, times float32(pi), times float32(1/(N-1)), cos, 1-, 0.5*.
    Any other order moves the last bit of every entry.

    @param n  Window length.
    @return (n,) float32.
    """
    i = np.arange(n, dtype=np.float32)
    m = np.float32(1.0) / np.float32(n - 1)
    t = i * np.float32(2.0)
    t = t * np.float32(np.pi)
    t = t * m
    return np.float32(0.5) * (np.float32(1.0) - np.cos(t, dtype=np.float32))


def read_wav(path):
    """
    @brief Read a WAV the way wav.cpp reads it.

    16-bit PCM only; a mono file is duplicated into both channels so the mono
    mixdown below sees the same thing either way.

    @param path  The file.
    @return (pcm, rate), pcm being (frames, 2) int16.
    """
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
    """
    @brief Per-frame float32 sum over bins [lo, hi], one bin at a time.

    The C loop's `sum += mag_[k]`, with one rounding per step. np.sum and
    np.add.reduceat reassociate pairwise, which moves the low bits by more
    than _validate()'s band check is allowed to excuse.

    @param mag  (frames, bins) float32.
    @param lo   First bin, inclusive.
    @param hi   Last bin, inclusive.
    @return (frames,) float32.
    """
    out = mag[:, lo].copy()
    for k in range(lo + 1, hi + 1):
        out += mag[:, k]
    return out


def _band_value(mag: np.ndarray, lo: int, hi: int) -> np.ndarray:
    """
    @brief One band's normalised value: mean, gain, then beat_normalise.

    mean * BAND_GAIN / FFT_N * 2, one float32 rounding per operation in the
    C's order, then the v/(1+v) compression.

    @param mag  (frames, bins) float32.
    @param lo   First bin, inclusive.
    @param hi   Last bin, inclusive.
    @return (frames,) float32 in 0..1.
    """
    sums = _seq_band_sums(mag, lo, hi)
    v = (sums / np.float32(hi - lo + 1)) * np.float32(BAND_GAIN)
    v = (v / np.float32(FFT_N)) * np.float32(2.0)
    return np.where(v > 0, v / (np.float32(1.0) + v), np.float32(0.0)).astype(np.float32)


def analyse(pcm: np.ndarray, rate: int = RATE) -> dict:
    """
    @brief The whole firmware pipeline over interleaved int16 stereo.

    Windows are FFT_N long and start every HOP_N, exactly main.cpp's block
    loop.

    @param pcm   (frames, 2) int16, as read_wav() returns it.
    @param rate  The rate to analyse at; it cuts the bands.
    @return {'mag', 'band', 'spec', 'index', 'due_us'}, each indexed by frame.
    """
    mono = (pcm[:, 0].astype(np.float32) + pcm[:, 1].astype(np.float32)) \
        / np.float32(65536.0)
    if mono.size < FFT_N:
        raise ValueError("too short to analyse")
    blocks = (mono.size - FFT_N) // HOP_N + 1

    windows = np.lib.stride_tricks.sliding_window_view(mono, FFT_N)[::HOP_N]
    spec_x = np.fft.rfft(windows * hann(), axis=1)          # double precision
    mag = np.abs(spec_x)[:, :BINS].astype(np.float32)       # ...to here

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
    """
    @brief read_wav() then analyse().
    @param path        The WAV.
    @param rate_check  Print a note to stderr when the file is not at RATE.
                       The analysis follows the file either way.
    @return As analyse().
    """
    pcm, rate = read_wav(path)
    if rate_check and rate != RATE:
        print(f"note: {path} is {rate} Hz -- analysed at that rate; the tuning "
              f"defaults were measured at {RATE}", file=sys.stderr)
    return analyse(pcm, rate)


def firmware_frames(wav_path, keep_prefix=None):
    """
    @brief Run the firmware's own analysis over a WAV and return its frames.

    pattern_lab --dump under the hood: the binary beside this module, built on
    first use, compiling analysis.cpp straight from the component. Ground
    truth for _validate() and for anything else that must not trust the numpy
    port above.

    @param wav_path     The track.
    @param keep_prefix  Persist the four dump files at this path instead of in
                        a scratch directory that is deleted on return.
    @return A dump_load.Dump.
    """
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
    """
    @brief Compare this module against the firmware over one track.

    Three thresholds, each loose enough to pass on float rounding and tight
    enough to fail on a real divergence: spec bytes must agree on at least
    99.9% of entries, mag must stay within 1e-4 relative over bins above 1% of
    the peak, and band within 5e-6 absolute. A hop mismatch or a frame-count
    mismatch is fatal before any of that, since it would make the comparison
    meaningless.

    @param wav_path  The track to compare over.
    @return 0 if every check passed, 1 otherwise.
    """
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
        # Which bins, not just how many: the same bins every frame is a
        # drifted spec edge, a scatter is float noise.
        print(f"     per-bin mismatches concentrate at bins {hot.tolist()} -- "
              "the same bins every frame means a drifted spec edge, not noise")
        failures.append("spec byte agreement below threshold")

    # Relative error is meaningless on a bin that is nearly zero, so the
    # comparison is made only where there is signal to compare.
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


## @cond
# The entry point, not API: running this file validates the module above
# against the firmware over one track.
if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} TRACK.wav\n"
              "       compares this module's numpy pipeline against the "
              "firmware's own\n       compiled code over the same file",
              file=sys.stderr)
        sys.exit(2)
    sys.exit(_validate(sys.argv[1]))
## @endcond
