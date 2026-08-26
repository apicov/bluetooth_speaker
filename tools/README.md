# Dancefloor tools

Everything that runs on the laptop rather than on a board: building and
flashing, designing patterns, tuning the beat detector, loading the hub, and
reading what a long run did.

[`README.md`](../README.md) is the hardware and the operating story;
[`docs/architecture.md`](../docs/architecture.md) is what the firmware does.
This is what runs beside it.

**Where a tool needs the firmware's own logic, it compiles the firmware's own
source.** `pattern_lab` and `converge` build `analysis.cpp`, `patterns.cpp`,
`analysers.cpp`, `beat_detect.c` and `fft_host.c` straight out of
`components/dancefloor_leds`; `satsim.py` deliberately does *not*, and that is
the point of it. Nothing here reimplements a wire format or a detector by
accident.

| | |
|---|---|
| [`sat.py`](sat.py) | Build, flash and monitor either satellite. One source tree, two images, and one flag that must be on every invocation. |
| [`syntax_check.py`](syntax_check.py) | Syntax-check firmware sources from `compile_commands.json`, with no working IDF environment. Reaches Kconfig branches the last build did not compile. |
| [`gen_vol_table.py`](gen_vol_table.py) | Regenerate the volume taper table in `audio_out.h`, and check the dB it realises. |
| [`pattern_lab/`](pattern_lab/) | The LED pipeline on a laptop, against a WAV. Live in the terminal, or as a PNG, a CSV or a binary dump. |
| [`tuning/`](tuning/) | The beat detector's tuning harness: a corpus sweep driven through `pattern_lab`, and a convergence probe for `BEAT_HIST`. |
| [`soak/`](soak/) | Long-run capture and analysis. Reads every unit's console at once, then says what happened. |
| [`satsim/`](satsim/) | N fake satellites, to load the hub's unicast fan-out. Also verifies the hub's XOR parity against an independent implementation. |
| [`log_collector/`](log_collector/) | Logs over WiFi from every unit, plus the bridge's console over USB, merged into one stream. |
| [`Doxyfile`](Doxyfile) | The documentation coverage check for this folder. |

## What each one needs

Most of this is stdlib Python 3 and nothing else. The exceptions are worth
knowing before a session starts rather than after.

| tool | needs |
|---|---|
| `sat.py` | an active IDF environment (`get_idf`) |
| `syntax_check.py` | a previous `idf.py build`, for its `compile_commands.json` |
| `gen_vol_table.py`, `satsim/satsim.py`, `tuning/sweep.py` | stdlib only |
| `pattern_lab/` | a host compiler and zlib |
| `pattern_lab/pipeline.py`, `pattern_lab/dump_load.py` | numpy |
| `soak/capture.py` | pyserial; `nmcli` too, for the air sweep |
| `soak/analyse.py` | pandas and numpy |
| `soak/ab.py` | pandas |
| `log_collector/collect.py` | pyserial, but only for `--bridge` |
| `tuning/sweep.py` | `ffmpeg` on PATH, and a corpus |
| `satsim/satsim.py` | root, once, to add interface aliases |

`sweep.py` is stdlib-only on purpose. The box it runs on has no numpy on either
interpreter, and a tool whose job is to still run in a year should not acquire a
dependency in order to compute a percentile.

## Building and flashing

The satellite is one source tree that produces two images which coexist
permanently — a classic ESP32 and a XIAO S3 — and the S3 one needs
`-DSDKCONFIG=sdkconfig.s3` on **every** invocation, not just the first. Omit it
and the S3 build directory is silently reconfigured from the classic config.

```
tools/sat.py build   classic|s3
tools/sat.py flash   classic|s3 [--port ...]
tools/sat.py monitor classic|s3
```

It prints the full command line every time. The point is that the long form is
easy to get wrong, not that it should become invisible.

`syntax_check.py` is the fast check between builds. It reuses the exact compile
command the last build recorded and runs it with `-fsyntax-only`, so it catches
everything short of linking without needing a working IDF virtualenv:

```
tools/syntax_check.py satellite
tools/syntax_check.py satellite/main/play.c
tools/syntax_check.py satellite --with CONFIG_DANCEFLOOR_OUT_MONO \
                                --without CONFIG_DANCEFLOOR_OUT_STEREO
```

The `--with` / `--without` half matters more than it looks: a normal build only
compiles the branches its own sdkconfig selected, which is exactly where an edit
rots unnoticed.

## Working on the lights

```
cd tools/pattern_lab && make
./pattern_lab track.wav                    # live in the terminal
./pattern_lab track.wav --png out.png      # the whole track as an image
./pattern_lab track.wav --csv trace.csv    # per-frame numbers
./pattern_lab --list                       # available patterns
```

The image is the one to reach for first: one row per analysis frame, one column
per LED, time running downward, so a missed beat is a gap in the stripes and a
dead pixel is a blank column.

`--dump` writes the same frames as binary for a notebook, and
[`pipeline.py`](pattern_lab/pipeline.py) is a numpy port of the same analysis
for looping over a corpus — [`docs/notebook.md`](../docs/notebook.md) is the
tutorial for both. The two are kept honest against each other:

```
tools/pattern_lab/pipeline.py track.wav
```

runs both over one file and fails if they have drifted apart.

## Tuning the detector

[`tuning/sweep.py`](tuning/sweep.py) drives `pattern_lab` over a corpus and
reduces the output to tables. Every number the detector tuning rests on comes
out of a subcommand here, so **the command is the provenance** — nothing is
written down that cannot be regenerated by re-running the line that produced it.

```
tools/tuning/sweep.py manifest       # the corpus, with sha256s
tools/tuning/sweep.py baseline       # the gate: reproduce the record
tools/tuning/sweep.py sweep          # the flux floor ladder
tools/tuning/sweep.py hist           # BEAT_HIST, on both of its axes
```

Run `baseline` first. If it does not land on `BASELINE_RECORD`, the harness is
not measuring what those figures measured and nothing after it means anything.

Corpora are read from `~/dancefloor-tracks`, `~/Music/forro` and
`~/Music/zabumba`; decodes and built binaries are cached under
`~/.cache/dancefloor-tuning`, which `DF_TUNING_CACHE` overrides.
`zabumba_segments.csv` is checked in beside the script because a segment
boundary is a tuning input like any other.

`converge.cpp` answers the half of `BEAT_HIST` the corpus cannot see: how long a
detector that missed frames takes to agree with one that did not.

## Reading a running system

Each board needs its own USB-serial adapter. **Baud is per unit and the hub is
not the default** — the hub's console is 921600 on a custom UART, the satellites
and the bridge are on the IDF default of 115200, and reading a board at the
wrong rate prints garbage rather than an error.

```
tools/soak/capture.py --unit hub=/dev/ttyUSB0:921600 \
                      --unit sat=/dev/ttyUSB1 \
                      --unit bridge=/dev/ttyUSB2
tools/soak/analyse.py logs-soak-*/
```

`capture.py` writes three files per session. `raw.log` is the record; the two
CSVs beside it are derived, and `capture.py --replay <dir>` rebuilds them from
`raw.log` with the current extraction — which is what lets an improvement to the
parsing reach the sessions it was measured against.

`analyse.py` answers nine questions in the order they matter, from "did anything
break" to "was it the source rather than a unit". [`ab.py`](soak/ab.py) is the
narrow version for a two-run comparison where one setting changed. For what a
counter means once you have found it,
[§17](../docs/architecture.md#17-what-each-counter-was-born-from) and
[§20](../docs/architecture.md#20-symptom-to-counter-to-file) of the
architecture guide go from a symptom to the file that owns it.

`satsim.py` makes the hub believe N more speakers are on the floor:

```
tools/satsim/satsim.py --self-test    # no hub, socket or alias needed
tools/satsim/satsim.py -n 8
```

It is a load generator, not an instrument — the measurement is taken on the real
satellites. The one thing it *does* measure is the hub's XOR parity, rebuilding
every member of every group from an implementation that shares no code with the
firmware's.

## Documentation

The comments here are Doxygen, and the run is a **check** rather than a
renderer:

```
cd tools && doxygen Doxyfile
```

`EXTRACT_ALL` is off and `WARN_IF_UNDOCUMENTED` is on, so a clean
`doc-gen/doxygen.warn` is the coverage test. It reads empty today, and the same
holds for each firmware project and component, which carry their own Doxyfiles.
