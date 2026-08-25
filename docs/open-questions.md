# Open questions

Things this firmware does not yet know the answer to, each with the test that
would settle it and the decision that follows from each outcome.

The point of writing them down is that they are all questions where *doing
nothing* looks identical to *the question being closed*: the code runs, the
logs look fine, and the uncertainty is invisible until someone re-derives it.
Each entry names the instrument that already exists to answer it, so the
answer costs a soak rather than a rebuild.

---

## Should the satellite's rate servo run on the median or the raw phase?

**Status:** open. Both units log what is needed to decide; nobody has run the
condition that discriminates.

### What differs today

The two units treat the phase reading differently, and the difference is
deliberate rather than an oversight:

| | rate correction (EMA input) | catch-up arm |
|---|---|---|
| Hub — `hub_s3/main/servo.c` | **median** (`err_in`) | median |
| Satellite — `satellite/main/servo.c` | **raw** `phase_err_us` | median |

The hub moved its rate correction to the median because its phase reading
scattered by milliseconds under load and the servo was chasing the scatter. The
satellite never followed, and the question of whether it should has been open
since.

Both units compute the median from `sync_phase_median()` over a
`SYNC_PHASE_HIST` of recent readings, and on both units it is the play task
that computes it and publishes it for the servo — the history is play-task-only
and is reset under that task's feet at every splice and re-anchor.

### Why it is not obvious

Under clean conditions the two agree: raw minus median is tens of microseconds,
and switching would change nothing. The median earns its place only when the
reading scatters, and scatter comes from load — retry bursts, a busy channel, a
unit at the edge of its range. Every clean soak is therefore consistent with
both answers, which is exactly why the question has stayed open.

### The instrument already exists

`satellite/main/servo.c` prints raw, median and smoothed side by side, once per
`CONFIG_DANCEFLOOR_LOG_PERIOD_S`:

```
buffer 352 ms | phase +1180 us (median +1090, smoothed +1035 us) | fec-k 4
```

The hub prints the same three in the same format. `tools/soak/capture.py`
already turns both into metrics columns, because the `key then number` shape is
what its `KEYNUM` pass parses.

**`raw - median` is the scatter.** That is the whole measurement.

### The test

One soak with both units playing, under load rather than on a quiet bench —
several satellites, or a unit deliberately taken toward the edge of its range,
so that retries and delivery bursts actually happen. A clean run cannot answer
this.

Then compare, per unit, the distribution of `raw - median` against the hub's.

### The decision, fixed in advance

| Outcome | What to do |
|---|---|
| Satellite scatter under load is comparable to the hub's | Adopt the median: change the `df_servo_ema()` input in `satellite/main/servo.c` from the raw reading to `med`. One line. |
| Satellite stays clean while the hub scatters | The divergence is justified on evidence. Record why here and close the question. |
| Neither unit scatters under load | Drop the median from the hub too, and `phase_hist` comes out of the audio path on both units. |

Writing the rule down first is the part that matters. Without it, every
inconclusive clean run leaves the question exactly where it was.

### Why it has not simply been changed

It is a one-line change and the mechanism to support it is already in place on
both units. It is held because it is a *tuning* decision: it changes how a
speaker corrects its rate, and there is no measurement showing it is better on
this unit. Making it on the strength of "the hub does it" would be guessing —
and guessing of that kind is what put the wrong numbers in this project's
comments before.
