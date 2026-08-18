# Analysing songs in a notebook, exactly as the firmware does

The ML analysers on the strips are trained against features that must be the
numbers the firmware computes, not a reimplementation of them. This is the
offline workflow for that: `tools/pattern_lab` compiles the analysis sources
straight out of the component and cuts the frames; a small Python layer loads
them into numpy for Jupyter.

The parity is structural, not approximate. `analysis.cpp` is compiled verbatim
for the laptop, and the hop is 512 everywhere — the boards pick it in Kconfig,
the host build gets the same number from `analysis_config.h`'s fallback, which
exists so the two cannot silently differ. One pipeline, whichever unit runs it.

The broader picture (who analyses, who runs analysers, and why) is in
[architecture.md](architecture.md); the detector tuning measured with these
tools is in [tuning-corpus.md](tuning-corpus.md).

## Building pattern_lab

```sh
cd tools/pattern_lab && make
```

That compiles `analysis.cpp`, `analysers.cpp` and `patterns.cpp` from
`components/dancefloor_leds` alongside a host FFT (`fft_host.c`) — the one
substitution, which agrees with the board's esp-dsp to float rounding. If an
IDF export has ever run in the shell, the Makefile's pinned `PATH` handles it;
no other setup exists.

Useful on its own, no notebook required:

```sh
./pattern_lab track.wav                    # live in the terminal
./pattern_lab track.wav --png out.png      # whole track as an image
./pattern_lab track.wav --csv trace.csv    # bands, flux, onsets for tuning
./pattern_lab track.wav --no-tty --dump s  # per-frame spec/mag/band (below)
./pattern_lab --list                       # available patterns
```

`make HOP=512` / `make HOP=1024` rebuild on a different hop grid (a rebuild,
not a flag — the hop is a compile-time constant). Every `--dump` sidecar
records which grid produced it, so files cut at different hops cannot be
quietly compared.

## From a notebook

One import line, then two ways to get features out of a WAV:

```python
import sys
sys.path.insert(0, "/home/pico/code/esp32_projects/dancefloor/tools/pattern_lab")
import pipeline, dump_load
```

**Ground truth — the firmware's own compiled code:**

```python
d = pipeline.firmware_frames("song.wav")   # builds pattern_lab if absent
d.spec    # (N, 64)  uint8   the portable spectrum: the exact bytes a
          #                   satellite's analyser lane is handed
d.mag     # (N, 512) float32 raw FFT magnitudes, bin k = k * rate / 1024 Hz
d.band    # (N, 4)   float32 the detector bands, as the wire carries them
d.time_s  # (N,)     frame times; d.due_us is the firmware's own int64 timing
```

The dump happens inside pattern_lab's block loop, where `Frame::mag` is still
alive — no other consumer could ever recover it. `keep_prefix=PATH` persists
the four dump files for reuse instead of a scratch directory.

**Fast iteration — the numpy port of the same recipe:**

```python
out = pipeline.analyse_wav("song.wav")     # dict: mag, band, spec, index, due_us
```

Same stages, same float32 evaluation order, same compounding spec edges — but
no subprocess, so it is the one to loop over a corpus with. The two agree to
float rounding, and the validator below is what keeps that true.

Useful extras: `pipeline.spec_edges_hz()` labels the 64 bins in Hz,
`pipeline.hann()` is the window, `pipeline.read_wav` mirrors pattern_lab's
WAV reader (mono duplicated to stereo, exactly).

## A dumped corpus, loaded directly

Dump once from the shell — a few seconds a track:

```sh
for f in ~/dancefloor-tracks/*.wav; do
  tools/pattern_lab/pattern_lab "$f" --no-tty \
    --dump ~/dancefloor-tracks/dumps/"$(basename "$f" .wav)"
done
```

Then each notebook load is a file read, not an analysis:

```python
d = dump_load.load("~/dancefloor-tracks/dumps/song")
d = dump_load.load("~/dancefloor-tracks/dumps/song", mmap=True)  # page mag on demand
```

## What the files may be

`pattern_lab` reads 16-bit PCM WAV, mono or stereo, at any rate (44.1 kHz is
what the tuning was measured against). Anything else converts first, with the
line the corpus doc pins:

```sh
ffmpeg -v error -i SRC -ar 44100 -ac 2 -c:a pcm_s16le -y DST
```

## The parity check

```sh
python3 tools/pattern_lab/pipeline.py TRACK.wav
```

Runs the firmware's own code and the numpy port over the same file and
compares: spec ≥ 99.9% of bytes identical (100.0000% on the corpus tracks),
mag within 1e-4 relative, band within 5e-6 absolute. Run it after touching
either `analysis.cpp` or `pipeline.py` — its whole job is to fail loudly when
a laptop figure would describe a pipeline the boards do not run.

## Fidelity notes, so they are re-derived never

**SBC.** Live audio reaches every unit SBC-decoded; a WAV has not been. Measured
(161 s track, 328 kbps ≈ the bridge's bitpool-53 ceiling, codec delay aligned
out): the codec moves `spec` by at most one LSB, on ~1% of bytes, and changes
no detector decision. Train on plain WAVs; the round-trip is not worth the
indeterminacy. The one caveat is level, not codec: `x/(1+x)` is monotone, so
quiet masters give genuinely quiet specs — augment gain, or mix in the
`~/dancefloor-tracks` recordings, which carry the real phone-encoded SBC at
real playback levels.

**What is not in numpy.** The beat and boom detectors are sequential stateful
C, and the rule is to run the real code, not copy it — their per-frame output
(flux, threshold, onset, strength) is in `--csv`, not in a dump.
