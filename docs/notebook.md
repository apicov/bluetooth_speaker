# Analysing audio in a notebook, the way a speaker does

`tools/pattern_lab/pipeline.py` puts the firmware's audio analysis in reach of a
Python notebook. You hand it a WAV; you get back the same frames a satellite
computes from the same audio — cut on the same grid, with the same windows, the
same bands and the same quantised spectrum.

Two ways to get those frames, and the whole point of the module is that they
agree:

| | What it is | Cost |
|---|---|---|
| `pipeline.firmware_frames(wav)` | Runs the firmware's own compiled code (`pattern_lab --dump`) and loads the result. **Ground truth.** | A subprocess per track |
| `pipeline.analyse_wav(wav)` | This module's numpy port of the same recipe. | ~90 ms for 30 s of audio |

`python3 pipeline.py TRACK.wav` measures the agreement, so a change on either side that
broke parity is caught rather than silently trained on. See
[§13 The analysis grid](architecture.md#13-the-analysis-grid) for what the grid
is and [§14 Onsets](architecture.md#14-onsets) for what the detector does with
it.

## Contents

[Setup](#setup) ·
[The grid](#the-grid-what-a-chunk-is) ·
[One call over a file](#one-call-over-a-whole-file) ·
[Chunk by chunk](#chunk-by-chunk-as-the-speaker-gets-them) ·
[Ground truth](#ground-truth-the-firmwares-own-frames) ·
[Onsets and booms](#onsets-and-booms-the-csv-route) ·
[Plotting](#plotting) ·
[Frequency axes](#frequency-axes) ·
[Checking parity](#checking-parity) ·
[Gotchas](#gotchas)

## Setup

`pipeline.py` imports `dump_load` as a sibling, so the module's own directory
has to be on `sys.path` — importing it from elsewhere by path alone fails on
that import. Notebooks are usually started outside the tree, so put this in the
first cell:

```python
import sys, pathlib

LAB = pathlib.Path.home() / "code/esp32_projects/dancefloor/tools/pattern_lab"
sys.path.insert(0, str(LAB))

import numpy as np
import pipeline, dump_load
```

Needs `numpy` only. `pandas` and `matplotlib` are used further down and are not
imported by the module itself. Nothing needs ESP-IDF.

The compiled half is built on first use — `firmware_frames()` runs `make` if the
binary is missing — but building it once by hand shows the compiler errors
where you can read them:

```sh
cd tools/pattern_lab && make
```

**WAV in, 16-bit PCM, mono or stereo.** Anything else is refused at
`read_wav()`. Convert first, pinned to the rate the tuning was measured at, the
way `tools/tuning/sweep.py` does it:

```sh
ffmpeg -v error -i track.flac -ar 44100 -ac 2 -c:a pcm_s16le -y track.wav
```

## The grid: what a chunk is

The firmware transforms a window of `FFT_N` = 1024 stereo frames and advances
`HOP_N` = 512 between windows — 50 % overlap, so each window carries 512 frames
of the previous one. At 44100 Hz that is one analysis frame every 11.6 ms,
about 86 a second.

```python
pipeline.FFT_N      # 1024   window length, in frames
pipeline.HOP_N      # 512    frames between window starts
pipeline.RATE       # 44100  what the tuning was measured at
pipeline.BINS       # 512    magnitude bins kept (DC computed, Nyquist not)
pipeline.SPEC_BINS  # 64     bands in the portable spectrum
```

Every frame carries two numbers that come from *counting*, never from a clock:
`index`, the block number from an origin every unit shares, and `due_us`, the
master-clock instant that window's first sample is heard —
`index * HOP_N * 1000000 // rate` in int64, exactly as `visualiser.cpp` derives
it. The notebook computes the identical integers, so a time you read off a plot
is the time the boards used.

## One call over a whole file

```python
r = pipeline.analyse_wav("track.wav")

r["mag"]     # (frames, 512) float32 — raw FFT magnitudes, bin k is k*rate/1024 Hz
r["band"]    # (frames, 4)   float32 — the detector's four bands, 0..1
r["spec"]    # (frames, 64)  uint8   — the portable spectrum; what travels on the wire
r["index"]   # (frames,)     int64
r["due_us"]  # (frames,)     int64   — microseconds
```

`spec` is worth singling out: those 64 bytes per frame are **byte for byte what
a satellite's analyser lane receives** over WiFi. A model trained on this array
is trained on the real input.

The four `band` values are the detector's, split at 43, 172, 1034 and 5039 Hz,
normalised by `v/(1+v)` — soft compression, no log and no dB anywhere in this
pipeline.

To work from samples you already have in memory rather than from a file:

```python
pcm, rate = pipeline.read_wav("track.wav")   # (frames, 2) int16, mono duplicated
r = pipeline.analyse(pcm, rate)              # the rate cuts the bands
```

## Chunk by chunk, as the speaker gets them

`analyse()` over a whole file is the same arithmetic vectorised. When what you
want is the *arrival* shape — a chunk lands, a window completes, a frame comes
out — mirror `visualiser.cpp`: fill a buffer to `FFT_N`, analyse it, slide the
tail down by `HOP_N` and keep it.

```python
def windows(pcm, hop=pipeline.HOP_N, n=pipeline.FFT_N):
    """Yield (n, 2) int16 windows from arriving hop-sized chunks."""
    buf = np.zeros((0, 2), np.int16)
    for i in range(0, len(pcm) - hop + 1, hop):
        buf = np.concatenate([buf, pcm[i:i + hop]])   # a chunk arrives
        if len(buf) < n:
            continue                                  # not a window yet
        yield buf[:n].copy()
        buf = buf[hop:]                               # carry the tail

pcm, rate = pipeline.read_wav("track.wav")
for k, w in enumerate(windows(pcm)):
    f = pipeline.analyse(w, rate)         # one frame: f["spec"][0], f["band"][0]
    due_us = k * pipeline.HOP_N * 1_000_000 // rate
    if k == 200:
        break
```

Frame *k* out of this loop is identical to row *k* of the whole-file call —
`spec` byte for byte, `band` to float rounding. Substitute a socket, a
microphone or a decoder for the slice and the framing does not change.

One thing this loop cannot give you is onsets: the detector is sequential
stateful C and is deliberately *not* ported to numpy. Onsets come from the
firmware's own code, below.

**Where a real speaker starts its first window** is the one part the notebook
does not reproduce, because it has no shared clock to align to. On a board the
first sample position is rounded *up* to the hop grid from an instant every unit
shares, which is what makes two speakers cut identical windows
([§13](architecture.md#13-the-analysis-grid)). From a file, block 0 is simply
sample 0.

## Ground truth: the firmware's own frames

```python
d = pipeline.firmware_frames("track.wav")

d.spec, d.mag, d.band      # same shapes and dtypes as above
d.index, d.due_us, d.time_s  # time_s = due_us / 1e6, for plotting
d.meta                     # {'window': '1024', 'hop': '512', 'rate': '44100', 'frames': ...}
```

This shells out to `pattern_lab --dump`, which compiles `analysis.cpp` straight
out of `components/dancefloor_leds`. The four files it writes land in a scratch
directory that is deleted on return; pass `keep_prefix` to keep them and reload
without rerunning:

```python
d = pipeline.firmware_frames("track.wav", keep_prefix="dumps/track")
d = dump_load.load("dumps/track")            # later, no subprocess
d = dump_load.load("dumps/track", mmap=True) # page mag instead of holding it
```

`mmap=True` maps the magnitudes rather than reading them — a corpus of long
tracks at 2 KB of `mag` per frame adds up (a four-minute track is 20 297 frames,
40 MB). `dump_load` checks each file's size against the sidecar first, so a
truncated or stale dump raises at the file instead of reshaping into nonsense
later.

## Onsets and booms: the CSV route

Beat detection is stateful, so it comes from `pattern_lab --csv` — the same
`beat_detect.c` the boards run:

```sh
./pattern_lab track.wav --no-tty --csv trace.csv
```

From a notebook, run it and read it in one cell — and take the dump at the same
time, since both describe the same frames row for row:

```python
import subprocess, pandas as pd

pathlib.Path("dumps").mkdir(exist_ok=True)   # --dump does not create it; keep_prefix does

subprocess.run([str(LAB / "pattern_lab"), "track.wav", "--no-tty",
                "--csv", "trace.csv", "--dump", "dumps/track"],
               check=True, capture_output=True)      # stderr carries the summary

df = pd.read_csv("trace.csv", comment="#")   # the '#' line carries window/hop/rate
d  = dump_load.load("dumps/track")           # df.block indexes d.spec directly
```

| Column | What it is |
|---|---|
| `block`, `time_s` | The grid; `block` is `index`, `time_s` is `due_us / 1e6` |
| `band0`…`band3` | The four bands, 0..1 |
| `flux`, `threshold`, `onset`, `strength` | Wideband detector: weighted flux across all four bands |
| `boom_flux`, `boom_threshold`, `boom`, `boom_strength` | The low band alone — the zabumba's stroke, which is what the lights actually follow |
| `ml0_<name>_label/score`, `ml1_<name>_label/score` | One pair per analyser slot, named after the analyser that produced it. Empty where a slow-lane slot had not answered yet |

```python
booms = df[df.boom == 1]
print(len(booms), "booms;", len(df[df.onset == 1]), "onsets")

# how near the threshold each boom was — the firmware's own marginal test
margin = (booms.boom_flux - booms.boom_threshold) / booms.boom_threshold
print((margin < 0.1).mean(), "of booms are within 10 % of threshold")
```

Detector tuning is exposed as flags — `--boom-floor`, `--boom-k`, `--boom-refr`,
`--beat-floor`; anything left unset keeps the firmware's value, deliberately, so
sweeping one parameter cannot silently revert another. The hop and the flux
history are compile-time: `make clean && make HOP=256`, `make clean && make
HIST=86`.

## Plotting

The spectrum, with an axis that says Hz rather than bin number:

```python
import matplotlib.pyplot as plt

r = pipeline.analyse_wav("track.wav")
t = r["due_us"] / 1e6
lo_hz, _ = pipeline.spec_edges_hz()

fig, ax = plt.subplots(figsize=(12, 4))
ax.imshow(r["spec"].T, origin="lower", aspect="auto", cmap="magma",
          extent=[t[0], t[-1], 0, pipeline.SPEC_BINS])
ticks = list(range(0, pipeline.SPEC_BINS, 8))
ax.set_yticks(ticks)
ax.set_yticklabels([f"{lo_hz[i]:.0f}" for i in ticks])
ax.set_xlabel("s"); ax.set_ylabel("Hz")
ax.set_xlim(30, 45)          # fifteen seconds is about as much as reads
```

The low bands look chunky because they are: several of the 64 log-spaced bands
share one FFT bin down there, which happens on the boards too and is reproduced
rather than smoothed over.

The boom detector against what it had to beat:

```python
w = df[(df.time_s >= 30) & (df.time_s <= 45)]
fig, ax = plt.subplots(figsize=(12, 3))
ax.plot(w.time_s, w.boom_flux, lw=0.8, label="boom_flux")
ax.plot(w.time_s, w.boom_threshold, lw=0.8, label="threshold")
ax.vlines(w.time_s[w.boom == 1], 0, w.boom_flux.max(), color="r", lw=0.6, label="boom")
ax.legend(); ax.set_xlabel("s")
```

For the lights themselves rather than the numbers, `--png` renders the whole
track as an image, one row of LEDs per frame:

```sh
./pattern_lab track.wav --png out.png --pattern pulse --leds 8
./pattern_lab --list                     # what patterns exist
```

## Frequency axes

```python
pipeline.band_bin(1034, 44100)   # nearest FFT bin to a frequency, rounding half up
pipeline.band_bins(44100)        # ([lo...], [hi...]) for the four detector bands
pipeline.spec_bins(44100)        # ...for the 64 portable-spectrum bands
pipeline.spec_edges_hz()         # ([lo_hz...], [hi_hz...]), for labels and targets
pipeline.hann()                  # the window, in the C's evaluation order
```

All of these take the rate, because the bands are defined in Hz and the bins
follow whatever rate the file carries — a 48 kHz file moves every edge relative
to the tuning, which is why decodes are pinned to 44100.

## Checking parity

```sh
cd tools/pattern_lab && python3 pipeline.py track.wav
```

```
PASS spec: 100.0000% of bytes identical (>= 99.9%)
PASS mag: max relative error 2.30e-06 over bins above 0.2 (<= 1e-4)
PASS band: max absolute difference 2.98e-08 (<= 5e-6)
all checks passed
```

Three thresholds, each loose enough to pass on float rounding and tight enough
to fail on a real divergence. The firmware transforms in float32 and numpy's
`rfft` is double precision, so `mag` carries slack that nothing downstream can
remove — the check measures it rather than assuming it. Run this after touching
`analysis.cpp`, `analysis_config.h` or the numpy port, and before trusting a
figure from a long notebook session.

On a `spec` failure the output names the bins that disagreed: the same bins
every frame is a drifted spec edge, a scatter is float noise.

## Gotchas

- **Import from the module's directory.** `sys.path` must contain
  `tools/pattern_lab`, or `import pipeline` dies on `import dump_load`.
- **The hop is compiled in.** A `pattern_lab` built with `make HOP=256` produces
  a different number of frames than `pipeline.HOP_N` assumes; `_validate()`
  refuses outright, and the `--csv` and `.meta` files both record the hop so a
  file cannot be misread afterwards. The dump's own `d.meta["hop"]` is the
  authority for a dump.
- **The rate follows the file, the tuning does not.** `analyse_wav()` prints a
  note to stderr on anything other than 44100 and analyses at the file's rate
  anyway. The flux floors were swept at 44100.
- **`mag` is not on the wire.** It exists only inside the analysis and dies at
  the next `process()`; the dump is written by the loop that produces it, which
  is the last place it exists. A satellite never sees it — only `spec` travels.
- **The constants are duplicated on purpose, and must be changed together.**
  `FFT_N`, `HOP_N`, `BAND_EDGE_HZ`, `SPEC_*` and `BAND_GAIN` at the top of
  `pipeline.py` mirror `analysis_config.h` and `analysis.hpp`. A value that
  disagrees makes every figure the module produces describe a pipeline the
  firmware does not run — which is what `python3 pipeline.py track.wav` catches.
- **The numpy port is not "cleaner" by accident.** Sequential float32 band sums,
  a Hann window evaluated in the C's order, spec edges built by compounding —
  each is a place a tidier version silently diverges. The docstrings in
  `pipeline.py` say which and why; leave them alone unless the validator agrees.
