# Tuning corpus — what the detector was measured against, and how to do it again

## 1. Why this document exists

The onset detector has five tuned numbers. Until now the only record of how any of
them was chosen was a comment block in `components/dancefloor_leds/analysis.cpp`,
and that comment does not say which recordings, which analysis hop, or which
command produced its figures. When the hop moved from 1024 to 512 in `03770c8`
and the detector had to be re-measured, reproducing the original sweep meant
guessing at all three. It took a day to establish that the recordings still
existed.

So this file is a manifest first and a conclusion second. Every table below names
the subcommand that produced it, every recording is listed with its sha256, and
the harness is checked in at `tools/tuning/`. Re-running the whole retune is four
commands.

The conclusion, stated up front because it is short: **at hop 512, none of the
three constants in question needs to move.** `BOOM_FLUX_FLOOR` stays 0.02,
`BEAT_FLUX_FLOOR` stays 0.02, `BEAT_HIST` stays 43. Sections 5 to 7 are the
ladders that show why, and section 8 is a finding about the hop change itself
that matters more than any of them.

## 2. What was measured, and what needs no measuring

`FFT_N` is unchanged at 1024, so the bin width, the band edges, the Hann window
and the magnitude normalisation are all exactly what every existing tuning figure
was measured against. What changed is how often a window is taken, and therefore
how far apart the two frames a flux value is differenced across sit.

| Constant | Where | Measured? |
|---|---|---|
| `BOOM_FLUX_FLOOR` 0.02 | `include/analysis.hpp` | yes -- section 5 |
| `BEAT_FLUX_FLOOR` 0.02 | `include/beat_detect.h` | yes -- section 6 |
| `BEAT_HIST` 43 | `include/beat_detect.h` | yes -- section 7 |
| `BOOM_REFRACTORY_US` 200000 | `include/analysis.hpp` | no -- already microseconds |
| `BEAT_REFRACTORY_US` 120000 | `include/beat_detect.h` | no -- already microseconds |
| `BOOM_THRESHOLD_K` 1.4 | `include/analysis.hpp` | no -- see below |
| `BEAT_THRESHOLD_K` 1.8 | `beat_detect.c` | no -- see below |

The two `threshold_k` values are provably invariant and were deliberately not
swept. The threshold is `mean + k * sd` over the flux history, so scaling every
flux value by any constant scales the mean and the standard deviation by the same
constant and the comparison `flux > mean + k * sd` is unchanged. A hop change
scales flux; it does not reshape that inequality. Measuring them would have
produced a table proving arithmetic.

`BEAT_HIST` is the one whose *meaning* changed rather than its scale. It is a
frame count, so the wall-clock it spans halved with the hop, from 998 ms to
499 ms. It also sizes **both** detectors, not just the wideband one --
`Analysis` holds `beat_` and `boom_` as two `beat_det_t`, and `hist[BEAT_HIST]`
is inside that struct.

## 3. The corpus

Two sets, kept separate throughout because they answer different questions.

**Anchor set — the ten WAVs in `~/dancefloor-tracks`.** These are the recordings
the `analysis.cpp` ladder was swept against, which makes them the only material
whose hop-512 numbers can be laid beside that record. Section 4 is the check that
they still reproduce it.

| Track | s | Hz | sha256 |
|---|---|---|---|
| 006 - Anastácia - Doce Cachaça | 193 | 44100 | cc43f5fa9d7eaa7014e17386e1fcd9953fe881cd4c926a806649c30d7a991690 |
| 009 - Flávio José - Pra Virar Lobizome | 190 | 44100 | 740075c7eaa65a17db0e7cc4c3a901dfddcd5a72a17dbf73f5787e0f9e0192b4 |
| 010 - Borrachinha - Como Demorou | 162 | 44100 | fb6c4787419ae74cb07e04ed5dbd90360a6407e6ca38691aad7f723218cc1725 |
| 011 - Targino Gondim - Chororô | 236 | 44100 | 60bc6ce4fe03a851da06223ac3be3b26c3ab13b2944d92449a614d1c1d2f41df |
| 012 - Forró Bemtivi - Caprichada no Baile | 250 | 44100 | 38be4ef118b7421d463a0f2c0ae12ab6908936d0c61908bc88bc717f770601aa |
| 014 - Alisone - Es ser | 160 | 44100 | b1dcc59af44ce12d32365c72e103e6c3864b126495f6a538ff3f94b6a25f5208 |
| 015 - Forroçacana - Anunciação - Ao vivo | 253 | 44100 | f67784047517ce13a3c9d91cf527e301e78cea5c4bfd874d80aaaf17d10a0d79 |
| 016 - Maria Gadú - Shimbalaiê | 198 | 44100 | ead46ec564ccb878078cf51964b766a43e2445b0ea566ef6cfa01745cbdbda05 |
| 033 - Nicolas Krassik e Cordestinos - Evelise | 229 | 44100 | da3932835275dd15bfa45323e145287d295938f7397dc0bc7fb33b0033c322b3 |
| 034 - Forró Bemtivi - Alumiar | 215 | 44100 | cc3a0c18e155a0680691e33e870a796bdb9c9e851a80e4be7c0198c157d9b446 |

**Breadth set — the 196 MP3s in `~/Music/forro`.** Twenty times the material, and
the reason a ten-track median is never trusted below without the 196-track figure
beside it. Section 5 is where that matters: the two sets disagree, and the large
one is right. Listed in full in section 10. Decoded once and cached:

```
ffmpeg -v error -i "$src" -ar 44100 -ac 2 -c:a pcm_s16le -y "$dst"
```

ffmpeg 7.1.2. `-ar 44100` is not optional. `pattern_lab` analyses whatever rate
the file carries, and a 48 kHz decode would move every band edge relative to the
tuning it is measuring.

Caveat on record: these are lossy rips, lowpassed near 16 kHz. Band 0 is
43-129 Hz and is the entire boom detector, so that detector is unaffected; the
wideband detector weights the top band at 0.15, so the influence there is small
but real. The anchor set is lossless and carries the comparison against the
record.

**Excluded: `~/dancefloor-tracks/xotes`.** 129 files, of which 61 are 44-byte
WAV headers with no audio at all and 26 are advertising under 40 seconds. 42 are
usable music. Averaging over the directory as it stands would not mean anything,
and the filter that would fix it is not worth defending when 196 clean tracks are
already available.

**Negative control — synthetic, no drum.** 30 s, sha256
`4507cb4648db4c09f3d5d204e9dd36ce389a451a9260e64e39f5ea6b65936db6`, regenerated
by `./sweep.py control` from a seeded RNG. Dither, a bass line at 120 BPM with a
30 ms attack, a sustained accordion chord and a triangle on eighths. It is the
only instrument here that says what extra sensitivity *costs*: a floor that fires
on it is following the music rather than the drum.

Three things about it are worth recording, because each was a wrong version of
this file first:

- Its bass level is calibrated, not chosen. Band 0's median is 0.0300, against
  0.031 for Chororô and 0.029 for Alumiar. The first draft sat at 0.31 -- ten
  times the music -- and at that level a steady sine's own spectral leakage,
  from a 55 Hz period that does not divide the window, swung band 0 by ±0.02
  frame to frame. That is the entire flux floor. The control was measuring its
  own synthesis.
- Nothing in it is switched on or off. A step discontinuity is broadband, so an
  abruptly gated 6.3 kHz triangle and an instantaneous note change were both
  dumping energy straight into band 0. Phase is integrated and the triangle has a
  3 ms attack.
- It has bass note attacks on purpose. A drone would have been silent at every
  floor and would have proved nothing. A bass note starting is a real rise in
  band 0 that is not a drum, and telling that from a mallet stroke -- 30 ms
  against about 5 -- is what the floor is being asked to do.

It is **not** the control the original sweep used. That one was never checked in,
which is the whole problem this document exists to fix. Its absolute counts are
therefore not comparable with the record's "produces exactly zero"; what it
provides is a cost gradient across a ladder, and a floor below which the detector
starts following an accordion.

## 4. Does the harness still reproduce the record?

The gate. If this fails, nothing after it means anything.

```
tools/tuning/sweep.py baseline
tools/tuning/sweep.py sweep --detector boom --hop 1024 --groups anchor
```

`analysis.cpp` records, at hop 1024 over the anchor set, booms per minute against
the boom flux floor. Measured now, beside it:

| Floor | Recorded | Measured |
|---|---|---|
| 0.06 | 0-80 | 0-107 |
| 0.03 | 50-114 | 51-117 |
| 0.02 | 68-134 | **68-134** |
| 0.012 | 82-139 | **82-139** |

Two rungs reproduce exactly and a third to within rounding. The 0.06 row differs
at its top end, which is where the detector is nearly dark and one track's count
swings the range. **Gate passed.**

The flux distribution fingerprint reproduces less cleanly, and the discrepancy is
recorded rather than smoothed over:

| | Recorded | Measured (per track, min-max) |
|---|---|---|
| median low-band flux | 0.0000 | 0.0000 |
| p90 | 0.016-0.019 | 0.0156-0.0380 |
| p99 | 0.038-0.131 | 0.0378-0.1560 |

Six of the ten tracks fall inside the recorded p90 band and nine inside the
recorded p99 band; the odd one out is *Evelise*, at p90 0.038 and p99 0.156. The
recorded figures read as a typical range rather than a true minimum and maximum.
Since the booms/min ladder -- which is what the floor was actually chosen on --
reproduces exactly, the harness is sound and the comment was loose. This is
precisely the class of ambiguity a manifest removes.

## 5. `BOOM_FLUX_FLOOR` — decision: unchanged at 0.02

```
tools/tuning/sweep.py sweep --detector boom --hop 512
tools/tuning/sweep.py sweep --detector boom --hop 1024
```

Anchor set, ten tracks:

| Floor | 1024 booms/min | 512 booms/min | 1024 marginal | 512 marginal | control (30 s) |
|---|---|---|---|---|---|
| 0.012 | 82-139 | 104-158 | 1.7% | 2.6% | 40 / 7 |
| 0.016 | 73-137 | 90-153 | 1.7% | 2.6% | 26 / 0 |
| 0.02 | 68-134 | 82-151 | 1.7% | 2.2% | 5 / 0 |
| 0.024 | 66-128 | 80-150 | 1.6% | 1.9% | 0 / 0 |
| 0.028 | 61-117 | 68-147 | 1.5% | 1.8% | 0 / 0 |
| 0.03 | 51-117 | 59-147 | 1.5% | 1.6% | 0 / 0 |
| 0.06 | 0-107 | 1-115 | 0.6% | 0.5% | 0 / 0 |

Breadth set, 196 tracks:

| Floor | 1024 booms/min | 512 booms/min | 1024 marginal | 512 marginal |
|---|---|---|---|---|
| 0.012 | 98-192 | 103-204 | 2.3% | 3.0% |
| 0.016 | 98-191 | 101-202 | 2.2% | 3.0% |
| 0.02 | 97-189 | 98-201 | 2.2% | 2.9% |
| 0.024 | 96-189 | 98-200 | 2.2% | 2.9% |
| 0.028 | 96-188 | 97-200 | 2.2% | 2.9% |
| 0.03 | 96-188 | 97-200 | 2.2% | 2.9% |
| 0.06 | 89-187 | 96-198 | 2.1% | 2.8% |

`marginal` is the firmware's own statistic from `visualiser.cpp`: a frame whose
boom flux is within 10% of its boom threshold, counted **per frame**. Per boom it
would read as a threefold regression where there is none -- the frame count
doubled at hop 512 while the 200 ms refractory held the boom count nearly
constant.

### Why nothing moves

**The floor cannot steer the marginal rate.** Over 196 tracks, sweeping the floor
across a factor of five moves the marginal rate by 0.2 points, from 3.0% to 2.8%.
The hop change alone moved it by 0.7 points, from 2.2% to 2.9%, at every rung.
The rise is a property of overlapping windows correlating adjacent flux samples,
and the floor is not the lever for it. No value of this constant restores the
hop-1024 marginal rate, so "hold the marginal rate flat" is not an achievable
acceptance test for it.

**The apparent preference for a higher floor is a small-sample artefact.** On the
anchor set the median marginal rate does fall usefully -- 2.2% at 0.02 against
1.9% at 0.024 and 1.8% at 0.028 -- which argues for a trim upward. But the
anchor set's *range* barely moves at all across those rungs (1.3-3.4%, 1.3-3.4%,
1.3-3.3%). What is moving is a median over ten samples hopping between adjacent
tracks. The 196-track median, which does not hop, is 2.9% at all three.

**Nothing in 0.016 to 0.028 is distinguishable on real music.** Breadth booms/min
is 98-201 at 0.02 and 97-200 at 0.028. The control is silent at every rung from
0.016 up. Moving a shipped constant on that evidence would be false precision,
and the constant's own comment already says why the floor is not where the
selectivity lives: *"the thing that makes this detector selective is not the
floor at all. It is the single-band input and the adaptive threshold above it."*
The sweep confirms that directly.

**What did change, in the right direction.** At hop 1024 the shipped floor lets
five booms through on 30 seconds of drumless material. At hop 512 it lets none
through. The overlap improved the false-positive behaviour at the shipped value.

### The premise this replaced

The retune was begun on the understanding that the per-frame marginal rate was
flat across the hop change, sampled on two tracks: Chororô 2.1% to 2.0%, Alumiar
2.6% to 2.4%. Both reproduce exactly. Over all ten they do not generalise:

| Track | 1024 | 512 | delta |
|---|---|---|---|
| 006 - Doce Cachaça | 1.0% | 1.5% | +0.5 |
| 009 - Pra Virar Lobizome | 1.5% | 2.7% | +1.2 |
| 010 - Como Demorou | 1.8% | 2.2% | +0.4 |
| 011 - Chororô | 2.1% | 2.0% | -0.1 |
| 012 - Caprichada no Baile | 2.6% | 2.5% | -0.1 |
| 014 - Es ser | 1.7% | 3.4% | +1.7 |
| 015 - Anunciação | 1.8% | 2.1% | +0.3 |
| 016 - Shimbalaiê | 1.1% | 1.3% | +0.2 |
| 033 - Evelise | 1.5% | 2.6% | +1.1 |
| 034 - Alumiar | 2.6% | 2.4% | -0.2 |

Mean +0.50 points. The three tracks that did not rise are the three with the
highest hop-1024 rate to begin with, and two of them were the sample. Worth
remembering the next time two tracks look like enough.

## 6. `BEAT_FLUX_FLOOR` — decision: unchanged at 0.02

```
tools/tuning/sweep.py sweep --detector beat --hop 512
tools/tuning/sweep.py sweep --detector beat --hop 1024
```

This value had never had a sweep of its own. It was set to match the boom
detector's floor and has been carried since. It has one now.

| Floor | anchor 1024 | anchor 512 | breadth 1024 | breadth 512 | control 1024 / 512 |
|---|---|---|---|---|---|
| 0.012 | 94-148 | 130-190 | 95-183 | 123-234 | 45 / 14 |
| 0.016 | 88-146 | 113-185 | 95-182 | 123-234 | 30 / 0 |
| 0.02 | 77-145 | 99-179 | 95-181 | 123-234 | 11 / 0 |
| 0.024 | 72-143 | 94-176 | 95-181 | 123-233 | 1 / 0 |
| 0.028 | 69-138 | 89-174 | 95-181 | 122-233 | 0 / 0 |
| 0.03 | 68-135 | 83-173 | 95-181 | 122-233 | 0 / 0 |
| 0.06 | 1-119 | 1-142 | 95-180 | 122-232 | 0 / 0 |

Onsets per minute; the control column is onsets on the drumless clip.

Same shape and the same conclusion. On 196 tracks the floor moves the rate by
under 1% across a factor of five (123-234 down to 122-232) and the marginal rate
not at all (2.5% at every rung). The only rung the control rejects at hop 512 is
0.012. 0.02 sits comfortably inside the accepted range.

The one figure worth carrying forward: at hop 1024 this detector fires 11 times
in 30 seconds on material with no drum in it, at the shipped floor. At hop 512 it
fires zero times. If either floor were to be revisited it is this one at hop
1024, and the hop change has already made that moot.

## 7. `BEAT_HIST` — decision: unchanged at 43, and which axis decided it

```
tools/tuning/sweep.py hist --hop 512
```

The only constant here whose meaning changed rather than its scale, and the only
one with two independent axes. Both were measured.

| HIST | span | onsets/min | booms/min | marginal beat | marginal boom | threshold sd | converge p50 | p95 | p95 ms | clean trials |
|---|---|---|---|---|---|---|---|---|---|---|
| 22 | 255 ms | 219.4 | 183.0 | 2.7% | 3.1% | 0.04593 | 0 | 3 | 35 | 32/40 |
| 43 | 499 ms | 199.3 | 172.2 | 2.5% | 2.9% | 0.03588 | 0 | 37 | 430 | 30/40 |
| 64 | 743 ms | 193.3 | 168.5 | 2.5% | 2.8% | 0.03176 | 0 | 36 | 418 | 30/40 |
| 86 | 998 ms | 191.4 | 166.5 | 2.4% | 2.8% | 0.02900 | 0 | 68 | 790 | 25/40 |

**Axis one, adaptation against noise.** `threshold sd` is the standard deviation
of the adaptive threshold itself -- how much the bar moves when the music does
not. 43 sits at the knee. Going from 22 to 43 buys a 22% quieter threshold
estimate; going from 43 to 86 buys a further 19% but takes twice the history to
do it, and the onset rate has almost stopped responding by then (199 to 191 per
minute across a doubling).

**Axis two, convergence after loss.** Measured by `tools/tuning/converge.cpp`,
which replays a trace's `band[]` through two detectors -- one fed every frame,
one given a gap -- and reports how long until their decisions agree and stay
agreed. This is the axis the coming third source mode will care about: under it
every unit runs this detector on the same received bands, so identical decisions
need identical state, and the state is `BEAT_HIST` frames of flux history plus a
refractory instant.

The pure state figure needs no measuring and confirms itself: the history turns
over in exactly `BEAT_HIST` frames, and the probe's `state_p50` reads 43 at
`BEAT_HIST` 43. What needed measuring is how much of that is visible, since a
threshold difference only changes a decision when flux lands between the two
thresholds. It is much less -- the median trial never disagrees at all -- but it
scales the way the state does. Across five tracks and both detectors, p95 frames
to agreement:

| | 22 | 43 | 64 | 86 |
|---|---|---|---|---|
| wideband, mean over 5 tracks | 16 | 24 | 42 | 62 |
| boom, mean over 5 tracks | 31 | 31 | 37 | 52 |

**Which axis decided it: neither had to.** They agree. The corpus puts 43 at the
knee of the noise curve, and the sync axis independently prefers the shorter
window, so the answer is the same from both directions and there was no
trade-off to adjudicate.

What that makes worth stating is the road not taken. Scaling `BEAT_HIST` to 86 to
hold the original 998 ms span constant across the hop change is the obvious
mechanical move, and it is wrong on **both** axes at once: it buys 19% less
threshold noise and costs 84% more time to reconverge after a lost frame, in a
detector that a future mode requires two units to agree about. Holding the frame
count rather than the wall-clock span is the right answer here, and it is the
answer for a reason, not by default.

A note for whoever writes the two-units-with-different-loss-histories test that
mode needs: at `BEAT_HIST` 43 and hop 512, a unit that misses ten frames is
back in agreement within 37 frames at the 95th percentile, and never disagrees at
all in three trials out of four. `test_pattern_sync.cpp` already sizes its
convergence allowance at `BEAT_HIST * 2`, which the measurement says is ample.

## 8. The finding that matters more than the constants

Boom instants, hop 1024 against hop 512, both at the shipped floor:

```
tools/tuning/sweep.py instants
```

| Track | booms 1024 | booms 512 | within 12 ms | within 23 ms | median dt |
|---|---|---|---|---|---|
| 006 - Doce Cachaça | 385 | 457 | 56% | 79% | 11 ms |
| 009 - Pra Virar Lobizome | 338 | 450 | 44% | 65% | 12 ms |
| 010 - Como Demorou | 303 | 346 | 48% | 71% | 12 ms |
| 011 - Chororô | 431 | 512 | 38% | 55% | 12 ms |
| 012 - Caprichada no Baile | 556 | 599 | 43% | 59% | 12 ms |
| 014 - Es ser | 284 | 396 | 37% | 53% | 23 ms |
| 015 - Anunciação | 424 | 534 | 49% | 74% | 12 ms |
| 016 - Shimbalaiê | 224 | 271 | 48% | 70% | 12 ms |
| 033 - Evelise | 451 | 575 | 45% | 66% | 12 ms |
| 034 - Alumiar | 368 | 411 | 33% | 45% | 35 ms |

Totals: 3764 booms at hop 1024, 4551 at hop 512. **44.2% fall within one hop of
each other and 63.5% within one window.** 1373 hop-1024 booms have no hop-512
boom within a window of them at all, and 2160 hop-512 booms are new.

That number means nothing without knowing what a small change looks like in the
same metric, so here is the calibration:

| Comparison | booms | within 12 ms | within 23 ms |
|---|---|---|---|
| hop 1024, floor 0.02 vs 0.024 | 3764 -> 3576 | **90.6%** | 91.1% |
| hop 1024 vs hop 512, both 0.02 | 3764 -> 4551 | 44.2% | 63.5% |
| hop 1024 at 0.02 vs hop 512 at 0.028 | 3764 -> 3896 | 41.8% | 59.8% |

A floor change at the same hop preserves nine instants in ten. Any hop change
preserves four in ten, and matching the boom *rates* by raising the floor at hop
512 makes the instant match slightly worse rather than better.

**So hop 512 is not the same detector on a refined grid. It is a materially
different detector**, and no value of any constant in this document changes that.
The reason is structural rather than a tuning error: flux is a frame-to-frame
difference, so at hop 512 it is differenced across 11.6 ms of audio and at hop
1024 across 23.2 ms. Those are different measurements of the same music. The
median offset being exactly one hop says the finer grid systematically prefers
the intermediate windows -- the ones whose predecessor is half a window back,
where a stroke landing mid-window produces the largest change.

This is recorded rather than acted on. Whether hop 512 is the right hop is a
question about which strokes the strip should follow, and it wants its own
evidence and its own session; it is not a retune. What 4c can say is that it was
measured, that the retune did not cause it and cannot fix it, and that anyone
comparing a hop-512 capture against a hop-1024 one should expect roughly a third
of the booms to have moved.

## 9. Reproducing all of it

```
cd tools/tuning
./sweep.py control                              # regenerate the negative control
./sweep.py manifest                             # sha256 every recording
./sweep.py baseline                             # the gate: reproduce analysis.cpp
./sweep.py sweep --detector boom --hop 512      # section 5
./sweep.py sweep --detector beat --hop 512      # section 6
./sweep.py hist  --hop 512                      # section 7
./sweep.py instants                             # section 8
```

The whole set is about twenty minutes on twelve cores, most of it the first run's
one-off decode of the breadth set into `~/.cache/dancefloor-tuning` (7.3 GB;
override with `DF_TUNING_CACHE`). Do **not** put that cache on `/tmp` here, which
is a 6.6 GB tmpfs.

Two traps that will cost an afternoon otherwise:

- Never source ESP-IDF's `export.sh` in a shell that then builds any of this. It
  puts the xtensa cross-assembler ahead of the host one and every native build
  dies with `as: unrecognized option '--64'`. All three host makefiles pin
  `PATH=/usr/bin:/bin` on every recipe; anything ad hoc needs the same.
- `pattern_lab --csv` output begins with `# window=1024 hop=512 rate=44100`. Skip
  lines starting with `#` when parsing, and check that line before comparing two
  traces -- every flux figure in a trace is a function of the hop.

`--boom-floor`, `--boom-k`, `--boom-refr` and `--beat-floor` are runtime flags,
and any left unset keeps the firmware's own value rather than a second set of
literals. `BEAT_HIST` is the length of an array inside `beat_det_t` and needs a
rebuild: `make clean && make HIST=86`, in `tools/pattern_lab`,
`tools/tuning` or `components/dancefloor_leds/test`.

### Everything here is derivable from `Frame::band`

Deliberately, and it is a constraint rather than a coincidence. The coming third
source mode -- the hub does the FFT, each satellite runs its own detector on the
bands it is sent -- can only apply a tuning value that such a unit can compute.
`Frame::band` is four floats at full precision and already travels. `Frame::mag`
is 512 floats, local-only and null on any received frame. `Frame::spec` is not a
substitute for `band`: it is quantised to 8 bits through `x/(1+x)`, and a flux
floor of 0.02 is about five counts of 255, so the quantisation lands directly on
the signal the detector runs on.

`converge.cpp` reads a trace's `band0..band3` columns and nothing else, which
makes that constraint executable rather than a promise. Any future candidate
tuning that cannot be measured from those four columns cannot run in that mode.

## 10. Appendix — the breadth set

`~/Music/forro`, 196 files, sha256 of the source MP3 before decoding.

| Track | s | Hz | sha256 (source mp3) |
|---|---|---|---|
| A Barca - Dona Mariquinha (Forró Stream)-Xt7enAPvPm4 | 256 | 44100 | a6cf35421825c917a627621ddf637c8bb863bc0cc3b728040884b195a3ab5be2 |
| Alcalyno - Lilith (Forró Stream)-L182VmVf9ao | 255 | 44100 | f598af4191786ff64df0b9d53abad77e79974d24252d39daaf7b97cb80e9b230 |
| Alcalyno - Medéia (Forró Stream)-AmiebS3SJqI | 235 | 44100 | 7f66a6b604fb18b60ae885ec079f645c3e0c1eec4aea758a6a4367063943a2fc |
| Alcalyno - Pão de Queijo e Forró (Forró Stream)-PLASbu0A4UY | 198 | 44100 | ad9221f73549a0e54cc17bc26b7ce3fa67620ae4bc814603adabf5ad28e49b41 |
| Alcalyno e Dominguinhos - Menino Angola (Forró Stream)-IJasXqBrI3Q | 290 | 44100 | c70982952945ca388420501ee4cf55ad94e56dd84ebebd51e9c7cb8c050d44ed |
| Alceu Valença - Cabelo no Pente (Forró Stream)-8hOr5ejQKdo | 226 | 44100 | f1c7d838673eadb8f0f20a43bc04223c551c320efe0b32d2414f474b8775b909 |
| Alceu Valença - Pétalas (Forró Stream)-JsFJLKQP2G4 | 198 | 44100 | f3e2ed0895ea3b93805ce7af5d9767288b97b61cdf8ea0401f6a61c719d47a98 |
| Aloisio Gomes - Minha Alencarina (Forró Stream)-OD47XsRwXTo | 199 | 44100 | 04400d7072dc29030e29992dbee9cd9d8075aec2770b132e1a874fc92d2bd9be |
| Amelinha - Frevo Mulher (Forró Stream)-N8_FJmWniP8 | 240 | 44100 | 67ecc2cb50f9b13321d9dc67904d3378ecc0ca622d7652c69e2d2e5273135960 |
| Amelinha - Gemedeira (Forró Stream)-O0oDfEY2jMc | 214 | 44100 | 90241b5702a90aab56fd1513ffa0d75365bfc45fe939b1ea5cd42ca46ff97478 |
| Aninha Vidal - Trem Bala (Forró Stream)-IuF5PEx33OY | 229 | 44100 | 5fc45a73b46818be0d53964ea6616fb98c723ef7410e5daae6daeb20a8808c2c |
| Aninha Vidal - Trevo (Tu) (Forró Stream)-aUQomokYR0Q | 226 | 44100 | 4aab06a066d8c96a337c16860abba80be390093e75cb2555f85c9eb375f12cbc |
| Arleno Farias - Anjo Querubim (Forró Stream)-kNYzE9iZDeU | 298 | 44100 | 12226d39e78ddad0e51f4bdb993da2ef37db016f8a2c173e191f630ae6e7efbe |
| Arleno Farias - Choque (Forró Stream)-O-SVDyF-3mo | 200 | 44100 | 69bd01b52d55eb0163d9209470aa8524114844c574e9838edfdb7116d52335c9 |
| Arleno Farias - Clarinha (Forró Stream)-qmUWmHRjGps | 232 | 44100 | afbe6e0fb0d38864c7a3b6b7fa6c573bdf5af62a5792b8d1bf87ffbc60d5c044 |
| Ary Lobos - Faca de Ponta (Forró Stream)-yn-35ifsph0 | 115 | 44100 | 4054dc4dafeff0109f81a5d719e21b584b1d54fb4efa009d8eec02a8a0d6a2c8 |
| As Bastianas - Esquisitice (Forró Stream)-ArlAauuijZQ | 332 | 44100 | a1a36a6479403a4438454854ae0b5429cff44b6c70b8e39075b446eef829a1b4 |
| Assisão - Forró no Sertão (Forró Stream)-_CqLH1msIo8 | 188 | 44100 | 3b8b0e0f25b04b35089bf71fe03d2a26f755ddf1baddcf287ded186f38b69423 |
| Aureliah Milagres - Saudade Que Vem (Forró Stream)-2i2vdkECZR4 | 247 | 44100 | 68b2786e17e8b17608894964e1b1c12338b80f6755229d47390a982760066fe7 |
| Aureliah Milagres - Saudade Que Vem (Forró Stream)-OReDz7bK54E | 247 | 44100 | 1c2057fbdc774683f6e6284d2a90d8dd4d3643d4ffd37ed9744348ed5750d0c2 |
| Aureliah Milagres - Voz (Forró Stream)-4YLBqlQQgsk | 187 | 44100 | 96bb768c30a018b8f4e82a17f5a48cce81a3a688dfe1b8cc08fb6d04fc8fff1b |
| Azulão - Ainda Sou Romeiro (Forró Stream)-y85MO4UPzFA | 166 | 44100 | f8911903fc5a6aaec060b308944794356cbed41cba7671058c24aa732c7f938e |
| Baião Brasil - Morena Mía (Forró Stream)-7e8n9z73IjM | 228 | 44100 | 5ce1f5fe3233cfee4fdb2b7f952e73b74045d97adc0771481dad54527b29d939 |
| Baião de Dois - Lua Menina Flor (Forró Stream)-3NzP_-dfAus | 277 | 44100 | b93334c5b0e1ad40f9bd10302b9b008bc4e7727be905c17a78ff472efc377304 |
| Banda de Pau e Corda - Caminhada (Forró Stream)-7saloM_LraQ | 170 | 44100 | 856b61011ae1862bbaa0a567c267f9b97221e4c45e59ae2d97c194fb57aa4769 |
| Banda de Pau e Corda - Me Diga Homem (Forró Stream)-nil60v_E3t4 | 191 | 44100 | 6c9a96db32f63cd4f893638d6b60574818f96b8ed7caaef3e051042214618cc7 |
| Barbatuques - Baião Destemperado (Forró Stream)-euewFsMy0Pc | 166 | 44100 | 548a22bac94cb1408e852aac8312e0393e60f7ed067f6f1fa239fa13f3c54362 |
| Barbatuques - Do Mangue a Manga (Forró Stream)-jyS9YFVaJKU | 301 | 44100 | b8d4e09a2adb15e2b2410fee358b6db2e0a90b66d4e90e2d56ae74b24f3c8a86 |
| Bequadros - Sugestão (Forró Stream)-brEhsDk54CE | 270 | 44100 | 0a2ddfac3a0a61292d5a0e7924e065b2b5eefc069a024753ad73b131d1ec5efa |
| Bezerra da Silva - Rapa Cuia (Forró Stream)-WYG0Ml9SGPs | 162 | 44100 | 6c312aa465d39d3221493736f65470563308302a62daee1744d42b17a4264e0d |
| Bicho de Pé - Eu e Você (Forró Stream)--D8TqlsqrsY | 305 | 44100 | ceeff752c96b749210efd8fb350ae57e430a0b9788b7ae29be68ae34f76e3c0a |
| Bicho de Pé - Jangadeiro (Forró Stream)-xskmeeUbC4Q | 250 | 44100 | c95ee659a9b8fd1ae38cf5af0a6e23ad785f624835ead73f92cebb4872089753 |
| Bicho de Pé - Nosso Xote (Forró Stream)-hwP14jW2L3U | 224 | 44100 | 42b70f86d5dd6e446a3992fb9acf422d73bb146cf38d94496f63d48b15ca0af6 |
| Bicho de Pé - Platonismo (Forró Stream)-ip4d1dldUZk | 269 | 44100 | 3318b9e1058cc32cae2c7188235c7ca89a471a7a25f5a1a450765d4fe83a4b0f |
| Bicho de Pé - Solidão (Forró Stream)-w-Y4J8VCME8 | 367 | 44100 | f81f0262e38c795561bc71fb2366e30d79529d5990647b1213f59b6f8e32e6df |
| Black do Acordeon - Casinha Pronta (Forró Stream)-iTEMIwkbi-w | 179 | 44100 | 465c37e59d70c2b0f993ed0f92e393b79ac3a46caee7e509bb92eadcf4f08db7 |
| Black do Acordeon - Erro Eu (Forró Stream)-imC0Kgsy5WY | 204 | 44100 | 45eefda9c24389f2fec2eb0c5856a1552aaca6e6bbd372c1ab7feacf6a50413b |
| Black do Acordeon - Lambada Chamego (Forró Stream)-VSZsaPA66LE | 208 | 44100 | 5726731a8a2f4cc4b3281a511b6e19238372aab3631c183a5080fe0a0016dec2 |
| Black do Acordeon - Legado (Forró Stream)-nUCs5bogw5c | 202 | 44100 | b38a4067c84dff890320964da6237e0398aed0e386ef5576c6c938902b6ba434 |
| Black do Acordeon e Bárbara Greco - Acalanto (Forró Stream)-g37OBzfqUkU | 218 | 44100 | 6887391cdb7a00baebe7d170ced652889c3882194c81df4e989642e1a20097e1 |
| Boi de Lata - Nem Mesmo Eu (Forró Stream)-1phrn_GfOto | 171 | 44100 | 0fb62583a3aa4b4c568f18f83a97fd1be6c5e939b08ce364e0bcc9175f07ac3a |
| Buchicho - Chineses (Forró Stream)-Wx3EGTk5Afs | 236 | 44100 | 457be08623b143005fbc3be98dfac17297238d65f6cfa555f0fb47706b13fc84 |
| Buchicho - Lindo Lago do Amor (Forró Stream)-0iaiFmjiKnE | 266 | 44100 | 13f8a951902171bd6fbe69f690f731dc8e80f4952dd9507d019de666fedb0077 |
| Buchicho - Paixão (Forró Stream)-GdGuAyF2aEI | 274 | 44100 | ef2a51d6a38051ae165a7be9771cb5ccfa427b7392c84f70b8eb377adfc0d091 |
| Candango Doido - Só Pra Te Beijar (Forró Stream)-mJK_m_BFpy8 | 244 | 44100 | 74af4b23fabd8b752e26c3c5aeef6912c211fb7560665cc97c88165432e7761c |
| Caraivana - Cabaceira Mon Amour (Forró Stream)-xFu85vk5nl4 | 226 | 44100 | 574fdd1a4f5528a7f93724c78b4b1c39f6242b9b0596282eec105f47cd40da8e |
| Carrapeta - Nortista Infeliz (Forró Stream)-h2trFCL4yUM | 121 | 44100 | 86368ba8331e99688decd422282636a4180302227619e91965460efaec14f490 |
| Carrapeta - Vapor do Una (Forró Stream)-WJ7WdC2agWA | 136 | 44100 | ab9d6b5e85ae71aa3e8046d523bb008aa67e9e96b5c9a3e69bb99c71ab9d1e25 |
| Caruá - Festa (Forró Stream)-7HI7txZanVo | 145 | 44100 | ff2597324500d0b7745d97f1fd4729618006c4df3e1e351de5fc8c9470674aaa |
| Chama Chuva - Menina de Itaúnas (Forró Stream)-sO6kmgYkqrI | 180 | 44100 | 01e3903f5f9762b182718ca05279df1aa5fcb0f505f47cc3e47756af104f7ecc |
| Chico Chagas - Swan Lake (Forró Stream)-DZdMAzgmUOI | 327 | 44100 | 76c1cc352e626a34f438c0803c49035f46c34f0979ce83c6eb65ccb740751443 |
| Chico Chagas - Um Chopin no Bach Ouvindo Forró (Forró Stream)-rSSlJwDibYE | 253 | 44100 | 329e2ea38776c4be10af18420fbd6feec5efa0ab759ac66907d8553cb83d9ba9 |
| Chico Pessoa - Os Dois Lados (Forró Stream)-2r7JS44orYk | 190 | 44100 | 4b2afdbf842461cab3ad0da61e0015833e16f73c23683172559aaf80576d3d72 |
| Chico Science & Nação Zumbi - Baião Ambiental (Forró Stream)-As3tEs2R58s | 153 | 44100 | 6659af3feda79c2a8d1d2c43f85e7aa82063730af4a30fbe24ba56c7830aeb05 |
| Chico Science & Nação Zumbi - Coco Dub (Afrociberdelia) (Forró Stream)-JHu6S2yhJlc | 406 | 44100 | 5d6f8f139b31f76d87ba556217c1bf183b2ef49217bfa0d9343d98eb366b460d |
| Chico Science e Nação Zumbi - Coco Dub (Afrociberdelia) [remix] (Forró Stream)-_wZK9lNlyVQ | 221 | 44100 | 49337bcf206c82303405225ae93ade419c4861f6138fc34a09f1064c144120c6 |
| Cipó Cravo - Te Encontrar (Forró Stream)-_DtUfVWxn60 | 209 | 44100 | c69c7fb22f4a74650551da6e56a19a20ee7ee0620b56cb1f837872a588f707f3 |
| Circuladô de Fulô - Ao Meu Lado é Seu Lugar (Forró Stream)-R8kpassr4OQ | 202 | 44100 | ad106ffa442cc4dbaa8572ee940e08acc487c1aebe65697e9c3401e86483748d |
| Circuladô de Fulô - Levitar (Forró Stream)-z_EygiFnXJY | 226 | 44100 | 9c0c1b77e770f2f33f899f449fbc79bfc93b2138299eee4d2b72a0035c31d7b4 |
| Circuladô de Fulô - Moinho de Vento (Forró Stream)-QxQaeu3PZUE | 192 | 44100 | 3c30ec93f3e85de86186b21ae6ce2ed35bc5d545a3d4a754f87e84d2db008d2d |
| Circuladô de Fulô - Pescador de Ilusões (Forró Stream)-upInQCUtXo4 | 239 | 44100 | a6081f81861643a5a30a014863b1be470b75ea95b9f7f3cdf3bd4de7a09ce3d5 |
| Circuladô de Fulô - Praia dos Pescadores (Forró Stream)-Zup-g2jLf-s | 225 | 44100 | 05c920b948aec8debe92438dc64dd72d31cb5cd4bc328aaca6ec317a75d5ff15 |
| Circuladô de Fulô - Segue o Seco (Forró Stream)-674vqexEBEc | 388 | 44100 | cfc2a252b38d98bcc3b11743e1469bfd6d90c546bd1a332dc0a7ce34936f0b3c |
| Circuladô de Fulô - Som de Bob (Forró Stream)-m-TXSKyxfes | 215 | 44100 | 157486b53e13d454d8719fe428cac7ee826fed2c841bb781f758ab4ee53f0833 |
| Clara Nunes - Feira de Mangaio (Forró Stream)-TcP5ih3bk1U | 199 | 44100 | d2ca45a6896f93fcc8482d5ccd1ffe723f56beb9fc2cc02daba8b5ad06348298 |
| Devagarinho - Sorriso Certo (Forró Stream)-2d5kWqx-3V8 | 250 | 44100 | 225112b3242b62ca5d36de58869a6090ff0a2cfcfee92a283f2aa62b38d104da |
| Devagarinho - Vida de Brasileiro (Forró Stream)-7CSZgIAqyWE | 351 | 44100 | c712d361fccaf7bc3c65a7d96b776d79a48cc1758c09936ebd4e55eaf4f7f854 |
| Diego Oliveira - Mais Um Homem Apaixonado (ao vivo) (Forró Stream)-jfMX-n-TUm0 | 167 | 44100 | 743c1d31655215eb94190b48f07f0e360855461cfb87fe352c1f966e6363bb80 |
| Dominguinhos - Desilusão (Forró Stream)-M0bJZnZ7oLA | 186 | 44100 | a0cb3f9d164f100bb96ecf3ee6bcc8fceb03b082072c2fdc2ac71271bb9b6e4e |
| Dominguinhos - Meu Pião (Forró Stream)-bKtevDlntMU | 194 | 44100 | 3a714d339c5d99f2e934f38a7b37d0b5d19b887f166491e9e33d4d82ac6c8560 |
| Dominguinhos - Minhas Desculpas (Forró Stream)-1MVSTycGxfs | 191 | 44100 | 7806cbfe8ce58d5c3d51acacfdcb7b539afd60404172e62aa691c39b416fc6c0 |
| Dominguinhos - Saudade Imprudente (Forró Stream)-c5z5AqRui08 | 185 | 44100 | b78967f433a20f805f967ed40541364ef884790996a3bc6c57efe6ddde915902 |
| Dominguinhos - Toque de Pife (Forró Stream)-cKNMmdjfgGY | 141 | 44100 | 483e9fa5e732e2bed3a1e595cf96e7af5d5834840613232d394762e06c6c0e97 |
| Dominguinhos, Sivuca e Oswaldinho - Feira de Mangaio (Forró Stream)-zB3UQezH514 | 250 | 44100 | 85bfc67962f6298c1e0afd43b802b890d4d908ce021967609c6fea7e03ed9c83 |
| Dona Zaíra - Tome Forró (Forró Stream)-Zt5SfXe4g6Q | 184 | 44100 | d73456cfe795f9651ec363732b70e7906e833bf908809ef8621eafd8043dba83 |
| Elton Raville - Agora Você Sabe-pT8C7T8jkUg | 243 | 44100 | 95c625b6d1f9f8737125c23f6cc6e3ea9358a92961f449e16bf9f6b004b88d66 |
| Estakazero - Algo Especial (Forró Stream)-6jTJRM5c31w | 263 | 44100 | 2a707a26f3ec2ee333a0deee7623ee4a076a3c847aa5d4684edee47deacdc39e |
| Estakazero - Botando o Pé na Estrada (Forró Stream)-yWpsweADfcU | 232 | 44100 | 9d1209ee1281c35e49dc7c2ea9b719192fd63eb7a1990305b0b7cf3ee89decc6 |
| Estakazero - Lua Minha (Forró Stream)-ui2T3r8aZQo | 224 | 44100 | 5668f658d081024c1673f322c145d3b3da58f84006968ce5869a92af1cfc9d32 |
| Fagner - Baião da Rua (Forró Stream)-FLfBMoBO4U0 | 224 | 44100 | 1715aaa0a065500029f150ba39baa701f35979c74a747eb31de2f5c6e138df96 |
| Falamansa - Amigo Velho (Forró Stream)-0YRcHoB1LuA | 241 | 44100 | 398183a2b7bd41112dbef62e573f707d9d685e8461d0b574ab79bd82b4a06e79 |
| Falamansa - Amor é Amor (Forró Stream)-qGlQsh_ZIU0 | 205 | 44100 | d86bc5d905610965a39fd075208223d823e220b56e69cd7ab063c3e48ce74673 |
| Falamansa - As Sanfonas do Rei (Forró Stream)-TS8ygNYkQjo | 242 | 44100 | dbfd69bf59dc0933c340039b313190a1234280b038bdb4fc3e0991eacc4ea3ff |
| Falamansa - Conselhos (Forró Stream)-5YROX8Is0E4 | 178 | 44100 | f5c73b00dc3138d6f89efb7768dd5374a2b3bbc5d75867a5a4ecb981875bfdf3 |
| Falamansa - Miou (Forró Stream)-RgKZ-Kt08XE | 236 | 44100 | 87cbfa1a7b201e07af41baea932ab31c2805e7e6412964c11f2c9fc4a7590cae |
| Falamansa - O Sol de Hiroshima (Forró Stream)-BZDIrzoJIEk | 208 | 44100 | 43002360cbec424181a9181b94f3b1151358645eeed3ee3a3ab5d60922e9ac97 |
| Falamansa - Simples Mortais (Foró Stream)-IoivxwH9w3w | 206 | 44100 | 2cc51ff568dfe08c35f4cf778db95dffbe2debeb74e70f09f7f314f59bfd8a08 |
| Falamansa e Dominguinhos - Sete Meninas e Forró do Bole-Bole (Forró Stream)-1oikHHpV1MY | 221 | 44100 | 597bbe8de466f90a97f0f76131b8a8cc8319d1e14b3fe1757824225efdad7b65 |
| Forrobamba - Balance o Berco (Forró Stream)--dsJGfw4qBM | 265 | 44100 | 68de85fc626e7e393346248066bd946e623b88ec8aca1e5c2569580548a2526d |
| Forró Fuá - Chão Batido (Forró Stream)-LUMs9Z3sQ4Y | 428 | 44100 | 81a3ef255c154a0a8a13bdc1dc2f9c5dd622de1deeae8fae1dc17dc15598cfc7 |
| Forró Fuá - Lavadeira do Rio (Forró Stream)-yF6S550V408 | 349 | 44100 | a12c14aa9be326c5d8864768ae5bc15e4a175de432ad94302f3a24f4e70e7065 |
| Forró Lunar - Xote na Lua (Forró Stream)-lN3KfGnMdtc | 201 | 44100 | ef12a924a6207c7397983a407577c959b95be7386bf7f5e4650c3e55b413b0e7 |
| Forró Massapê - A Dança dos Feixes de Luz (Forró Stream)-jGl738vo4KU | 250 | 44100 | c432c4471058542290fb0c109a290aaf8e803e0da3448a2cfa36334822f916f2 |
| Forró Massapê - Conto de Fadas (Forró Stream)-QvDnNQd7cXo | 200 | 44100 | 63d122dc0090d12cc38a3e00aa5263d9d21b15ab8b3af065c179e42fdbb06afd |
| Forró Nativo - Lembranças (Forró Stream)-MYvzLHkZMpY | 216 | 44100 | e884810e235fd2d505acfdfec79837ce0c7333fbeb7305b92b51058cbc1a46a2 |
| Forró na Contramão - Diariamente (Forró Stream)-BFDRykDAAUc | 279 | 44100 | a872bc769e07724c49d5cda891a4039517b2441c01862094215fb70b74af1265 |
| Forró na Contramão - E aí, Zé (Forró Stream)-FxeuhPOOc_U | 202 | 44100 | d2dca6916c745421e34b255fb5f15805988facce7ba9ce6c52852b4222d77ffa |
| Forró na Contramão - Fino Trato (Forró Stream)-N4MSQtnMoRY | 246 | 44100 | 893eb502d1fd637da19ff699d0ac6e75866f7037e7c9b6c887ae0155cd5b7ca1 |
| Forróçacana - Forró Horizontal (Forró Stream)-XvUWh5lB7nI | 203 | 44100 | 0f108140300cfdceb03411cb1473b5c95197d05312922b7b1c426e06ff16f5d4 |
| Forróçacana - Forró no Malagueta (Forró Stream)-hFxT4_2I2y8 | 224 | 44100 | dd3391dcad0866ed1abf09feca98c6f62af92a4a1e15bb79e88fc90d2da52cad |
| Forróçacana - Matilde (Forró Stream)-DNaT5dElaN0 | 235 | 44100 | e274f27b3b0a5119c696a1b8f28de4a876c964d36cf88cb215102b8114df6d66 |
| Forróçacana - Menina Mulher da Pele Preta (Forró Stream)-lw-iNGB3F68 | 236 | 44100 | 32b9d9e9df878990ac3911cfa30db6f31e9fcb11421232dd2bb56af4912bba7d |
| Forróçacana - O Melhor Forró do Mundo (Forró Stream)-U9TjIP4JhUk | 182 | 44100 | bd9191e33c4b18c7ca771d21c3bef2b160ef89fe27e91f4c002ec811c436d850 |
| Forrueiros - Céu e Mar (Forró Stream)-kQ3AR2VIkQY | 184 | 44100 | 09d8faa46d58d68051b326bf163eed7782ca72e2d02c553c0c4f6f3f78f7c65c |
| Forrueiros - Imaginação de Anjo (Forró Stream)-BaKzTG9xnic | 238 | 44100 | a5327b3f1a95612b1b34076127bed841211d92faa5ca78848accd13e5264a11a |
| Forrueiros - Segue o Seco (Forró Stream)-ZRTjNmGoGM8 | 201 | 44100 | 296245b3e4180c40cf8e96ce856db86571bc1b0e84cd0ec673d62d37fe515563 |
| Forró Comichão - Aconchego Perfeito (Forró Stream)-9hRUFiGxiuA | 235 | 44100 | 0fe887fd3a966849a1c06db65a5c2c9833ac6c1ced5a1480e50553c696bc5b97 |
| Forró In The Dark - Índios do Norte (Forró Stream)-UR0_oEaC_bU | 172 | 44100 | ef5d10192d8e95a60b9159312d5d26c2815cee0632aedd4dbcf8685c18b34afc |
| Frank Aguiar - Prenda (Forró Stream)-pdleZ7h0lPY | 223 | 44100 | 45b76b914addd3f162ce1d657543161b552b37adab241d6217ff80350d65f24b |
| Fúba de Taperoá - Mulheres Que Sabem Amar (Forró Stream)-vk_Qs0y3oBI | 163 | 44100 | 68b6c9d5f211fd953c948ed36fa01710d8298b48a7466504cb5054afa82fa383 |
| Geraldo Azevedo, Elba Ramalho e Zé Ramalho - Ai Que Saudade D'ocê (Forró Stream)-79wAh4lpVSI | 171 | 44100 | a9940d20b85c8f85af6f0acf39df52ffd999131468f3bfd45d74f66095f2073c |
| Gilberto Gil - De Onde Que Vem o Baião (Forró Stream)-gZO6QvNHQnM | 189 | 44100 | fc73d92109af9a52ec6adcdd6db1f29ffe8157527f70bca182aba6077002f000 |
| Ivete Sangalo e Zé Ramalho - Amar Quem Eu Já Amei (Forró Stream)-hTNvaZECElg | 168 | 44100 | 481e8058d57e04c66d722892d7b87b9c18d77e28a7803d697ad306a82fc9fa44 |
| José Patrocínio - Tenente Bezerra (Forró Stream)-KjHLh6Ax-kA | 178 | 44100 | 7e16bd657b9db87d5ed93fbde396cee365e2cb5648911fd9d9c6e96f5730abc6 |
| Jussara Silveira - Baião de Quatro Toques (Forró Stream)-aTcmNPJfYWU | 193 | 44100 | 679630e4e38338a7cc8ef2905646e810b1acbecf677a3aebdd21383e67385546 |
| Kayamaré - Lua (Forró Stream)-21AR2ljnnfE | 135 | 44100 | 796fa04a982774291d0c9e7a0ef38cd93a2ff6123a7ffa3fbcd03b20d6435055 |
| Kayapo - Sinais (Forró Stream)-k5-OsdPCzwM | 227 | 44100 | 902cc719339f908dfcb1439aa0e9bf5643b08a73307dc9cb93d2b861658c872c |
| Lampião Elétrico - A Foto (Forró Stream)-bwCRyMWtS-U | 243 | 44100 | c8dc139f47e0787cfa864038d36d2fb17f2f9c0caf7502183568290fb7e096de |
| Lampião Elétrico - Rosas Negras (Forró Stream)-p0nFNkY-iBA | 183 | 44100 | dc71d3d198eb2410d68403eda22d40f03737b0c2ec22568f0cbaa121cedf566f |
| Lenine e Frejat - Pagode Russo (Forró Stream)-AbvG5zFcPeM | 231 | 44100 | bd36a62c6feeb46443c6407eddd2ceefea7e79dc194cd3de3eedca99e29b22c1 |
| Luiz Gonzaga - Nem Se Despediu de Mim (Forró Stream)-3pcaABppEX8 | 227 | 44100 | ba97620b87d497b45ce9075bed80bbc41e9b57f1214234efaf016b878040b6f8 |
| Luiz Gonzaga - Onde o Nordeste Garoa (Forró Stream)-Abbo0BxZXkk | 143 | 44100 | 48715d0e9730aea5148632ab73eaa0785b704116bc686437b862b59f1c326562 |
| Luso Baião - Bailarina (Forró Stream)-Grb97uzVhvw | 270 | 44100 | 9541b93e0b113ca9b7d581593b8b763b9403eafa2a7d1ce47994b8e20c1560af |
| Luso Baião - Cheira Bem (Forró Stream)-zjKOmkrs6_Q | 273 | 44100 | 91ed93aacaa9361b024599d2ab46632b1dd87066d2f2ba52f8fa72d52c5ecc60 |
| Manitu - Ócio Criativo (Forró Stream)-Vnv5tsoEU7o | 232 | 44100 | e3777893c5721e47d84c8ae23f131688bcfd0bffd1ff483b87219b6da9962431 |
| Meketréfe - Relógio Baião (Forró Stream)-1VuCWz9-7pk | 235 | 44100 | 271d7cd28f01eb435ec801560a83f9384054ce0bf04f27eaad4d0a9352848996 |
| Mestre Ambrósio - Forró de Primeira (Forró Stream)-B-_XUYD_WLE | 138 | 44100 | 41f8217466afab62940f9e612b54ced81aff2cfe972f90a2caaa88cc4dcb6aee |
| Mestre Ambrósio - Pescador (Forró Stream)-Xh-b5IqgHIs | 325 | 44100 | e4c3761d8cebb7e2a0b55d8fdebb3e490588393dc143982f09052977c09e1733 |
| Mestre Ambrósio - Pé-de-Calçada (Forró Stream)-1Qe9tfOudm4 | 168 | 44100 | e8906bd1b2163a40ee1c805e1e887c95482842e774ac312ddd71f722aa6319ea |
| Mestrinho - Vou Te Matar de Cheiro (Forró Stream)-UIpuJw8vmcQ | 238 | 44100 | 960df9baa5b3a28088ecbb8d0167bde4c20b9e524890b7a1d1dcd03d9bf85c33 |
| Nicolas Krassik - Cordestinos (Forró Stream)-ITP2bDifhoU | 281 | 44100 | 9e9465da287cf9cb409b485640d8b942f705369d222e08eb6ee06f1464916b97 |
| Nicolas Krassik - Lamento Sertanejo e Último Pau de Arara (Forró Stream)-vWMoSJZT1xM | 241 | 44100 | 895dac6e6f34d7a0ae53fcb30bf11a8ad0efd00c2b167b00b1be234a85c7978a |
| Nicolas Krassik - Sanfona Sentida e Cheirinho de Mulher (Forró Stream)-KlIoGPttvMQ | 316 | 44100 | 2ff7b1981fef3d5df7e17612a693cef45bf23d36b328e3ab94709f53096603fa |
| Nó Cego - Sabor Colorido (Forró Stream)-su5RMZHHUDA | 212 | 44100 | 42d3b1bb9ba05ded0c5a7d5906f1473b3aad82a30d316b4dbd6709fd7a140f11 |
| Nó de Forró - Zabumba (Forró Stream)-OymLmVlKaJQ | 243 | 44100 | 399529aa4466b5b5826a206376e57cb28fda0e0d0b9104abc12a96aa87ed8743 |
| O Bando de Maria - Carinha de Leão (Forró Stream)-7SfQlDJx5Gw | 242 | 44100 | 51472027302e0bc4411640cf7dc25ef50826b7b9f88e050c6c1b4e94a0dda0cf |
| O Bando de Maria - Rosalina (Forró Stream)-Xn2UWqh-M-A | 186 | 44100 | c63ba8f2753297dc22d2e72292aca3a8d31f4c5b3848d91f13a58878c89bbd93 |
| O Karaíva - Cabrobró (Forró Stream)-DSZpjCUxdLc | 230 | 44100 | fd40fde0dc7cd0835592f7ab50f95999b7d6d052c6b58e566bd44dd1189b568a |
| Os Fulano - Ponto Sem Nó (Forró Stream)-YrTcEAj39RE | 142 | 44100 | 72d0e70241e37a2bff8b58548826f7048774569fc859da08a79654b900a7a72e |
| Os Fulano - Pra Alegrar o Meu Forró (Forró Stream)-v9Hsw10oPT8 | 155 | 44100 | ec83ff29bf7d1a80eec69c3449d7a6ec30da5e1bc1a37c333156c23480a5eb43 |
| Pé de Mulambo - Negócio Bom é de Dois (Forró Stream)-YoRyQPJDiTg | 205 | 44100 | 5131b5565f383a5c28504952876955a181131b25a4c306c5043fe245fca1217a |
| Pé de Mulambo - Pescador (Forró Stream)-8G0OHrxKFis | 223 | 44100 | 63f879bc89d59d3f10d9b59562657e44c45a2869688cf772bb49adbd68f7b183 |
| Praieira e Arleno Farias - É Só Imaginar (Forró Stream)-ZqzD8UEXgQc | 223 | 44100 | 996fc89113a6d4541a11a4aa8ebd851e1380726be782fe0b481b61d7c813eb18 |
| Quarteto Olinda - Xinxim no Xenhenhem (Forró Stream)--gMNZEBEyd4 | 198 | 44100 | d09554f2db61722d3273163cdc5ffa25acdd5b44f5b5c467776ebd0af15dd4ee |
| Raiz do Sana - Cabeça D'Água (Forró Stream)-d8xa0gCe22Y | 158 | 44100 | 0160745a9a9720863b9946878502f049fd3aa9fc9dacdd1ea95118cc9746d240 |
| Raiz do Sana - Ovni (Forró Stream)-KIEkLHdFCZ4 | 324 | 44100 | 6dbf7be32557c641bbdde0e7e50dd7bf1e639c9d4985189f8b62994b685d1e0e |
| Raiz do Sana e Marcelo D2 - Saudades do Futuro (Forró Stream)-fMuRFEKCUGQ | 219 | 44100 | 92fb582f5c8e72bdf18eb0ff7350056e534e741373976fd5dd43a8db869b9e6c |
| Rastapé - Colo de Menina (Forró Stream)-ArVoXhg6z7U | 205 | 44100 | 79d7143691464cb5a34115d61b8251a13cc35d564e3b40abf924c5785fa0607a |
| Rastapé - Embalo do Forró (Forró Stream)-FCkwrbkP_rA | 217 | 44100 | dad722ae9f7ab576f5f3e43a97c405dd13d598e841fcaf2e39e8d467a149aef4 |
| Rastapé - Foi Deus Quem Fez Você (Forró Stream)-47DfcLNQbwo | 215 | 44100 | 6446675f7b0bfd9c45c1831db400a2909afbf4bd72bd8e45ec374071df74938c |
| Rastapé - Tarde Quente (Forró Stream)-hKThxgAEWcM | 228 | 44100 | f32177a7d0136b3feda7719eabeb06c0463ada9a88d2c938112d0c512c397164 |
| Rastapé e Zé Ramalho - Segredo (Forró Stream)-U5KNp_vtBvI | 225 | 44100 | e135450ca55182812ad8428594aac3405386f1dd8afcdf5bb34bb12ddd35f580 |
| Rastapé - Namoro (Forró Stream)-HNhsePqDFfQ | 231 | 44100 | 06c4a5273d3350ad6255cd1c966e1efca518566d736d882a20f227246a1263dd |
| Relô Rolô - Mãe Liberdade (Forró Stream)-QMXucsCnXjI | 296 | 44100 | d36584af777740d91a0b219d696088991eb0f3d2a219c4842cf4526dd16a06fc |
| Renata Cabral - Ouro e Urucum (Forró Stream)-IQTDL7nFml4 | 274 | 44100 | 6d50c7382746ad78b6a20cbb4404073ce798593f4a0f02804f71b6894644ac77 |
| Rueiros - Filhos de Lampião (Forró Stream)-ZYjeoc52LB4 | 188 | 44100 | 9c39b50b43473968076b8d5fcf0fa66da5f7078fd461886c47f3c336aa3acbc0 |
| Santanna - Doidim Por Você (Forró Stream)-ZRcKy9y0_Rk | 211 | 44100 | 95c6b30e6fee6ccd43cd79cce08aad17e8e8bed3d72c50045445aef90c063894 |
| Santanna e Cezzinha - Lápis de Cor (Forró Stream)-nKohyJuuSBE | 221 | 44100 | 5f6ae5326aab54faa4498c40510bcb8efa3af2ebb9d50224d21b837fa0cb8f53 |
| Saulo e Mestrinho - Nosso Amor (Forró Stream)-Jtr-Yx8nPxo | 291 | 44100 | 4ad688969a9e1b46a71f1d50e05c2577db808a45d1d73b3fea5d0b665d086004 |
| Sivuca - Forró em Santa Luzia (Forró Stream)-PJkuhZVQaqQ | 139 | 44100 | dcad1489b134a2fbd69661e423826afd28e74b25e5cb19351a0cc842b564fd7e |
| Swing de Palha - Toque Xote (Forró Stream)-9B8bp4UlsaE | 236 | 44100 | 3b854d02d16153582027f685d411f874deef245d0045b026526afe42831ce6a6 |
| Trio Alvorada - De Rosto Colado (Forró Stream)-Ufg7WcA-BD4 | 230 | 44100 | 469fc19f9b752302f00f2f4d755fabfbea882f2ea32ea421f40faa49296333be |
| Trio Alvorada - Minhas Distâncias (Forró Stream)-HXD9RmOv27M | 233 | 44100 | f389a5c554589263e1186924b50f976a8d447ce9d550f2e7b6f8e54cd7c3f95f |
| Trio Balancê e Trio Xamego - Castigo (Forró Stream)-RnGpP7Sor3Y | 317 | 44100 | 336a1c514dffe02a8979dc466fb849a8c49c3cb1d653a03fd8fcebf2757b81b1 |
| Trio Bastião - Simplesmente Sangrar (Forró Stream)-FEZn_J_1LuA | 221 | 44100 | 7878525bbbc078ec88cbf97e1d28c01ff453d4e26f69e5cddfa7638b5c73b9c8 |
| Trio Bodocó - Bodocando (Forró Stream)-z52qcguEGgY | 172 | 44100 | 17fa9cddc8933cc446627a2f7f97e063b24343e510f0b04846ec6e4265d7ebde |
| Trio Cangaço - Cigano Terrorista (Forró Stream)-eV1GYuuCwxc | 231 | 44100 | b7c67d2e14fe2ba57af5ae0b0260bb3d6a9bccfd7ace11cb78f09f862197191c |
| Trio Dona Zefa - Beijo Bom (Forró Stream)-tRzNGRubgiI | 174 | 44100 | 00728f38df443443277272a464387ea4a95595c2d8e91954105c3d1f0c5635bd |
| Trio Forrozão - Em Plena Lua de Mel (Forró Stream)-vMU8zBreXEM | 241 | 44100 | 582b8e4eb507cfe8e7f752ac1db1c0acdb9ad0aea374844334bcdb1944ca8ba6 |
| Trio Forrozão - Zé Esteves (Forró Stream)-fSqGODxwlj0 | 158 | 44100 | 4989f09b80add762847e921b548fc740ef8b2f46c830a63fe1c96204f55ba071 |
| Trio Juriti - Cara a Cara (Forró Stream)-78ZE-YmXWVY | 183 | 44100 | a610fde9003f3b8e11c337c448ce850658960ccc0bd5f87371f2b1dff72c36a4 |
| Trio Macaíba - Sossego do Meu Sonhar (Forró Stream)-2zWyR35I09s | 203 | 44100 | 1149835c7829eee63f398fae81b282af88475d3052a18a135391ac376d5e4236 |
| Trio Macaíba - Xote Torto (Forró Stream)-hpvjG10E1u4 | 202 | 44100 | 26cb349d677e73505d0b3b12bb0f713888d16c3c4c3c6f936555cc1a45f74cb7 |
| Trio Potiguá - Agradar Você (Forró Stream)-R--iat8Ck90 | 325 | 44100 | 6b6e9807edb85b9f667bd18d931d92ed813af6739666c367160cb80b77584c04 |
| Trio Virgulino -  Sonhando em Itaúnas (Forró Stream)-ETn-s_iI9u0 | 233 | 44100 | 4725046984ce6ddfdbc956b656d1854c3a07c058c3155d758f32aa0fb07f4bc6 |
| Trio Virgulino - Forró do Rei (Forró Stream)-H4nVaofqCAk | 231 | 44100 | b1e744f65c67ba2a5bc301e10e9bfd0a5209ec43ea128c19f0a63a46b0d85846 |
| Trio Virgulino e Falamansa - A Peleja do Diabo Com o Dono do Céu (Forró Stream)-YIbeMj67RvM | 271 | 44100 | 9ae14469e641d3054f9fe13404975163a3aaefc29bda35110a73f4f9397b029b |
| Triô Buritís - Deixa a Lua Clarear (Forró Stream)-HJhO0LG3edA | 185 | 44100 | dbf43f6327686f64d6e072d04355c29dadf5c4877211589fea3963d2bc42a752 |
| Vanildo dos Pombos - Um Galope Galopado (Forró Stream)-xI_QOITQi7M | 218 | 44100 | 9ccae7ad44083e044c107688affa1a14cdef1f2a1e6d6087ee92134f0b46c33b |
| Verso e Prosa - Acerola (Forró Stream)-s7xbuCamX6s | 231 | 44100 | f6bf86bd16fac1fa3f5a0f1b4e9ee954b238c8475cf063ab7aae3a838b2c8706 |
| Verso e Prosa - Cataventos (Forró Stream)-r_YRAoyC16I | 252 | 44100 | 58682341401ec06f9d85358b0c88af09f0681971a2992e94444997c7beb7f6eb |
| Verso e Prosa - Moinho (Forró Stream)-qrFY6MEaN-8 | 206 | 44100 | 46071e1cf610def4bdc19ccd1a3d86985231a53ce6c388edeeb83539b0022937 |
| Verso e Prosa - Sinal de Amor (Forró Stream)-xMmeSVFG9w8 | 168 | 44100 | 5c14cebae31816ad0278138fae534f1e984eb860113ba1f007abd346c0c0ddbb |
| Verso e Prosa - Vida Sem Sentido (Forró Stream)-kJvUmrI87X0 | 203 | 44100 | 0d1c1a903fd34330a9086e31fd1c7fe3d2081ab55e4bc48e8208bb0e59f2643d |
| Xote de Colo - Baião das Fofoqueiras (Forró Stream)-NgItWI3iVSk | 184 | 44100 | 15e455f372ffb596dc8d14d723da4826fe73ea67b5a302f28b241a5e0a8b134c |
| Xote de Colo - Consciência (Forró Stream)-Z87E29dN8GM | 218 | 44100 | 3bfb9409268e06f7ccfbe5d3b8d37a46f044e66773fd45ae2dec0fb4ff7c38a3 |
| Xote de Colo - Lenda do Dragão (Forró Stream)-FvsJ_vMfRrM | 205 | 44100 | 425ea64618aae140608d998aea8a7541692ae91f04cb9ad53157a4cf4d183f01 |
| Xupanomanga - No Balanço do Mar (Forró Stream)-Lvw6CnQAjAY | 233 | 44100 | c4ceee799dfd7872d19a2c96c261a67b73989b3c57780aa1f1530366f3902fae |
| Zabumbazul - Calabouço do Amor (Forró Stream)-PWPfSZpPT1w | 130 | 44100 | 10e444e4dc8911a15a965786fac6ab14dec273f94c414848cef6817d8ce49b46 |
| Zeca Baleiro - Xote do Edifício (Forró Stream)-tsc7ggeZ4eI | 261 | 44100 | af672476f6bc340481f9a8fd1ff6600f3a249943f508aefeb4f91fc1efd75baf |
| Zeca Balero e Elino Julião - Rela Bucho (Forró Stream)-d4k3Ya3NQ7c | 215 | 44100 | fb9c10767a47e02dad914b2dbbb1315d3693920a24426bbeae922ffdd3c9462e |
| Zito Borborema - Forró do Alecrim (Forró Stream)-Qgt4kMO_duc | 159 | 44100 | 1048c977f96c17eb4225fcc144cf6aa1711e305eb27c21bf662c82040c7a0909 |
| Ó do Forró - A Vida é Uma Dança (Forró Stream)-lasG9rJpHeg | 242 | 44100 | dd0f34bc129c3edf9ecdf2df394d35f07d192071c6747cb35c61d52c6a14be8a |
| Ó do Forró - Dom do Amor (Forró Stream)-G54neIii4P4 | 297 | 44100 | 7e7f1b545843b7c554f12d3d570ebd7f5c407a3153e80d7215802b31e2711cbc |
| Ó do Forró - Me Chamo Forró (Forró Stream)-fKeFdjxYE74 | 243 | 44100 | abb156e72259f6b54b9daa898223f2e0a8bc589f857d111606676bd79c6dc2a5 |
| Ó do Forró - Voz do Coração (Forró Stream)-QqIvYAV_f6c | 264 | 44100 | 0047c0206d9f5c353d1a86a5aa133d62e017e1e3e35daff0051d7afd68aeb058 |

## 11. Code map

| | |
|---|---|
| `components/dancefloor_leds/include/analysis.hpp` | `BOOM_THRESHOLD_K`, `BOOM_FLUX_FLOOR`, `BOOM_REFRACTORY_US`, `set_boom_tuning`, `set_beat_floor` |
| `components/dancefloor_leds/include/beat_detect.h` | `BEAT_HIST`, `BEAT_FLUX_FLOOR`, `BEAT_REFRACTORY_US`, and the note on these becoming a cross-unit agreement |
| `components/dancefloor_leds/beat_detect.c` | `BEAT_THRESHOLD_K`, the threshold and the refractory |
| `components/dancefloor_leds/analysis.cpp` | `Analysis::init()` -- the reasoning behind each value, and the original hop-1024 ladder |
| `components/dancefloor_leds/visualiser.cpp` | the `marginal` counter, which is the same statistic section 5 uses |
| `components/dancefloor_leds/include/analysis_config.h` | `DF_FFT_N`, `DF_HOP_N` |
| `components/dancefloor_leds/Kconfig` | `DANCEFLOOR_LED_HOP` |
| `tools/pattern_lab/` | the pipeline on the host, and the `--boom-*` / `--beat-floor` flags |
| `tools/tuning/sweep.py` | every table above |
| `tools/tuning/converge.cpp` | section 7's second axis |
