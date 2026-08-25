#!/usr/bin/env python3
"""
Read a soak session and say what happened.

Takes one or more directories written by capture.py and answers the questions
a long run exists to answer, in the order they matter:

  1. did anything break        the counters that must stay at zero
  2. cross-unit divergence     the number this project measures itself on
  3. the rate trim             is it running, at the rate it claims, and stable
  4. phase                     where each unit settled and how far it wandered
  5. buffer depth              a slow drift is what a continuous trim can cause
  6. delivery                  whether audio arrived evenly, and what a hole cost
  7. the hub's refused sends   where a hole in the sound is most often made
  8. heap                      what only a run of hours can answer
  9. the source                stalls, which look like unit faults and are not

REBOOTS ARE HANDLED, and this is the part worth knowing before trusting a
number. Every counter on a HEALTH line is cumulative SINCE BOOT, so a board
that resets mid-soak returns all of them to zero, and a maximum would then
silently report the larger of the two runs instead of their sum. A reset shows
up as the board's own uptime going backwards, so the run is split into boot
segments at those points and each segment's gain is added.

THE HUB'S WINDOW SUMS ARE LOWER BOUNDS. Its serial capture can drop whole log
lines, so anything summed over its per-window `status` gauges is short by
however many windows never arrived. Ratios between two such sums survive it,
since both terms are undercounted by the same sampling; absolute totals and
anything divided by wall-clock time do not. window_cover() measures the
shortfall and cover_note() prints it wherever it matters.

Requires pandas.

Usage:
    tools/soak/analyse.py logs-soak-20260814-120000/
    tools/soak/analyse.py logs-soak-*/ --wide wide.csv
"""

import argparse
import os
import sys

try:
    import numpy as np
    import pandas as pd
except ImportError:
    sys.exit("pandas is missing: pip install pandas")


## @brief Metric names that are cumulative since boot. Anything not listed
#         here is treated as a gauge.
#
# A counter has to be baselined per boot segment and summed; a gauge is read
# where it stands. Getting one wrong turns a total into a difference between
# unrelated numbers.
COUNTERS = {
    "underruns", "restarts", "reanchors", "anchors", "splices", "retunes",
    "retunes_refused", "gaps", "ring-full", "dma-starve", "short-reads",
    "short-resync", "wifi-drops", "alloc-fail", "phase-drop", "seq-drop",
    "decode-err", "recv-err", "sta-left", "wifi-over",
    # The piggyback FEC scheme's truncation count. The scheme is gone and so
    # is the counter, but a capture taken before XOR parity still carries it
    # and must total correctly rather than be read as a gauge.
    "fec-trunc",
    "refill-withheld", "anchors-refused", "upgrades", "dropped", "dup",
    "wide-span", "sta-timeout",
    # The faded catch-up's own pair, kept apart from the trim's dropped/dup
    # because a drain runs at over a thousand skipped frames a second by
    # design and would drown the trim's own arithmetic if mixed in.
    # See components/dancefloor_sync/audio_shift.h.
    "catchup-drops", "catchup-dups",
    # A seqlock reader that gave up and fell back to the estimator. Must be
    # zero: the unbounded version of that loop can hang a satellite with a
    # full ring and a starving DAC. See tsf_read() in satellite/main/sat.h.
    "tsf-read-fail",
    # A satellite that took itself off the hub's send list because it could
    # not play. Not a fault in itself -- it is the unit protecting the floor --
    # but it means one speaker was silent, and that must never be a mystery.
    "self-mutes",
}

## @brief What must stay at zero for a soak to have gone well, and what a
#         non-zero reading means. Roughly worst-first.
FAULTS = [
    ("underruns",    "playback ran dry"),
    ("ring-full",    "audio dropped, ring could not take it"),
    ("dma-starve",   "the DAC ran out of data (~81 per underrun is that underrun)"),
    ("gaps",         "packets lost on the air"),
    ("short-reads",  "ring came up short of a chunk"),
    ("short-resync", "a gap fill did not fit and forced a re-anchor"),
    ("alloc-fail",   "an allocation failed"),
    ("retunes_refused", "a retune was refused as insane"),
    ("seq-drop",     "packets arrived older than expected"),
    ("tsf-read-fail", "a TSF seqlock read gave up -- the writer could not run (sat.h)"),
    ("self-mutes",   "this unit went silent to stop starving the other speakers of airtime"),
    ("decode-err",   "SBC frames that would not decode"),
    ("recv-err",     "recvfrom() errors"),
    ("wifi-drops",   "lost association with the hub"),
    ("phase-drop",   "phase queue full, servo starved of input"),
]

## @brief capture.py's pseudo-unit for the 2.4 GHz sweep.
#
# Kept in step with AIR_UNIT there; the two files agree on this name and on
# nothing else. Dropped from every per-unit loop below, since it is not a board
# and would otherwise collect an uptime and a FAULTS verdict that mean nothing.
AIR_UNIT = "air"


# A refused send arrives in bursts rather than at a rate, and that difference
# decides what a soak can be judged on. Nearly all of a run's refusals fall
# inside one or two episodes lasting a couple of minutes, in runs that are
# otherwise clean throughout. So a per-hour mean spans a quiet run and a storm
# and describes neither, and a short run reads zero often enough to look like a
# fix. What a change has to be judged on is how many episodes a run had and how
# bad they were, which is what section 7 reports.

## @brief Quiet time that separates one episode from the next.
EPISODE_GAP_S = 120
## @brief Refused sends that make an episode worth counting as a major one.
EPISODE_MAJOR = 50

## @brief How far either side of an episode the air sweep is read.
#
# capture.py sweeps every 30 s by default, so a short blackout is caught by AT
# MOST ONE sweep -- and not necessarily the one nearest the episode, which is
# what nearest() alone would find. Two sweeps wide, so an adjacent blackout is
# found from either direction and one missing sweep does not hide it.
AIR_LEAD_S = 60


def load(paths):
    """
    @brief Concatenate one or more session directories.
    @param paths  Session directories written by capture.py.
    @return (metrics, events) as DataFrames; events may be empty but always
            has its columns.
    """
    mparts, eparts = [], []
    for p in paths:
        m = os.path.join(p, "metrics.csv")
        e = os.path.join(p, "events.csv")
        if not os.path.exists(m):
            sys.exit(f"{p}: no metrics.csv -- is it a capture.py session directory?")
        mparts.append(pd.read_csv(m))
        if os.path.exists(e) and os.path.getsize(e) > 0:
            eparts.append(pd.read_csv(e))
    met = pd.concat(mparts, ignore_index=True).sort_values("wall_s", kind="stable")
    ev = (pd.concat(eparts, ignore_index=True).sort_values("wall_s", kind="stable")
          if eparts else pd.DataFrame(columns=["wall_s", "unit", "kind", "text"]))
    return met, ev


def boot_segments(df):
    """
    @brief Label each row with which boot it belongs to, per unit.

    A board's own uptime only ever increases within a boot, so a decrease is a
    reset. Detected per unit, because different lines are emitted on different
    cadences and interleave.

    ONLY THE INSTRUMENTED KINDS VOTE ON A RESET. capture.py's generic
    word-then-number scan lifts "metrics" out of prose too -- a build banner's
    date, an address line -- and those carry whatever esp_ms their line had,
    which for a boot banner is single digits. A false split re-baselines every
    counter mid-run, so the test has to be narrow.

    @param df  The metrics table.
    @return A copy sorted by (unit, wall_s) with a `boot` column added.
    """
    df = df.sort_values(["unit", "wall_s"], kind="stable").copy()
    real = df["kind"].isin(["health", "trim", "status"])
    step = df["esp_ms"].where(real).groupby(df["unit"]).diff()
    df["boot"] = (step.fillna(0) < 0).groupby(df["unit"]).cumsum().astype(int)
    return df


def counter_total(df, unit, metric):
    """
    @brief What a counter gained across every boot segment of one unit.

    Counters are cumulative since boot, and a capture usually opens on boards
    that booted hours ago: the first line already carries all that history, so
    a raw total would report the past rather than the soak. Each boot segment
    is baselined at its first observed value instead.

    Read PER KIND, then the larger of the two totals. HEALTH and TRIM are
    different lines, and one name can appear on both meaning two different
    things; pooling them would take the maximum from whichever series happens
    to be higher and the baseline from whichever sample happens to be earliest,
    so the answer would be a difference between unrelated counters. Taking the
    larger of two independently computed totals keeps the real counter and
    cannot double-count the impostor. A name carried by only one kind -- nearly
    all of them -- is unaffected.

    @param df      Metrics, already through boot_segments().
    @param unit    Which board.
    @param metric  Which counter.
    @return The total, or None when the run never recorded it.
    """
    s = df[(df["unit"] == unit) & (df["metric"] == metric)
           & df["kind"].isin(["health", "trim"])]
    if s.empty:
        return None
    totals = []
    for _, part in s.groupby("kind"):
        per_boot = part.groupby("boot")["value"]
        totals.append(int((per_boot.max() - per_boot.first()).sum()))
    return max(totals)


def fmt_or(v, spec, dash="   --"):
    """
    @brief A number, or a dash when the run never recorded it.

    The same rule the satellite's lead-min follows on the wire: a value that
    was not measured must not render as a plausible zero.

    @param v     The value, or None.
    @param spec  Format spec.
    @param dash  What to print instead; sized to the column.
    @return The formatted string.
    """
    return dash if v is None else format(v, spec)


def gauge(df, unit, kind, metric):
    """
    @brief One gauge's samples, in time order.
    @param df      The metrics table.
    @param unit    Which board.
    @param kind    Which line kind carries it.
    @param metric  Which number on that line.
    @return A frame of wall_s and value, or None when nothing matched.
    """
    s = df[(df["unit"] == unit) & (df["kind"] == kind) & (df["metric"] == metric)]
    return s[["wall_s", "value"]].reset_index(drop=True) if not s.empty else None


def window_sum(df, unit, metric):
    """
    @brief Sum a per-window `status` gauge over the windows that were captured.

    tx-fail and the ENOMEM shape beside it are CLEARED by the window that
    prints them -- see tx_fail_summary() and tx_burst_summary() in
    hub_s3/main/net.c -- so summing windows is the right shape of arithmetic.
    counter_total() is the wrong helper for them twice over: it baselines each
    boot segment against its first value, which is meaningless for a series
    that returns to zero every window, and it reads only health and trim lines
    while these live on the hub's `status` line.

    A LOWER BOUND, NOT A TOTAL, and callers must say so. See the note in the
    module docstring; window_cover() gives callers what they need to qualify
    the difference.

    @param df      The metrics table.
    @param unit    Which board.
    @param metric  Which gauge.
    @return The sum, or 0 when the series is absent.
    """
    s = gauge(df, unit, "status", metric)
    return 0 if s is None or s.empty else int(s["value"].sum())


def window_cover(df, unit, metric="txdone"):
    """
    @brief How much of the run a window sum actually saw.

    The window length is the median of the SHORTEST intervals between captured
    lines: dropped lines only ever create multiples of the real period, so the
    short end of the distribution is the period itself. The median of the short
    ones rather than the outright minimum, since the minimum is one line's
    arrival jitter.

    @param df      The metrics table.
    @param unit    Which board.
    @param metric  A gauge that appears once per window.
    @return (windows captured, window seconds, seconds those windows cover);
            (0, None, 0.0) when the series is too short to say anything.
    """
    s = gauge(df, unit, "status", metric)
    if s is None or len(s) < 3:
        return (0 if s is None else len(s)), None, 0.0
    d = s["wall_s"].diff().dropna()
    d = d[d > 0]
    if d.empty:
        return len(s), None, 0.0
    win_s = round(d[d <= d.min() * 1.5].median())
    return len(s), win_s, float(len(s) * win_s)


def cover_note(n, win_s, covered_s, span_s):
    """
    @brief One line saying how much of the run a window sum saw, or nothing.

    @param n          Windows captured.
    @param win_s      Seconds per window.
    @param covered_s  Seconds those windows cover.
    @param span_s     Seconds the run lasted.
    @return The note, or "" when coverage is good enough not to need one.
    """
    if not n or not win_s or span_s <= 0:
        return ""
    pct = 100.0 * covered_s / span_s
    if pct >= 95.0:
        return ""
    return (f"    (over {n} captured {win_s:.0f} s windows = {pct:.0f}% of the run"
            f" -- the hub's serial capture flaps, so counts here are LOWER BOUNDS)")


def tx_faults(df, unit):
    """
    @brief The hub's refused sends, formatted as FAULTS lines.

    Here rather than in the FAULTS table because these are window gauges rather
    than cumulative counters, and here AT ALL because a refused send is not a
    hub-local inconvenience: the packet is never transmitted and never retried
    -- send_audio_to_clients() in hub_s3/main/timeline.c drops it -- so it
    reaches the floor as a hole in the sound.

    @param df    The metrics table.
    @param unit  Which board.
    @return Lines to print, empty for a unit with none.
    """
    total = window_sum(df, unit, "tx-fail")
    if not total:
        return []
    out = [f"      {'tx-fail':18s} {total:>8,}   "
           f"sendto() refused (lower bound) -- see HUB TX below"]
    audio = window_sum(df, unit, "tx_fail_audio")
    if audio:
        out.append(f"      {'of which audio':18s} {audio:>8,}   "
                   f"never reached the air (~12 ms of starvation each)")
    return out


def error_faults(ev, unit):
    """
    @brief Level-E console lines, formatted as FAULTS lines.

    Not in the FAULTS table because these are log lines and not counters, and
    because the case they exist for is the one where HEALTH lines stop
    arriving altogether: a board that hard-hangs prints its panic and then goes
    silent, so every counter freezes at its last good value and the run reads
    clean.

    Grouped by tag with one example each, since several hundred identical
    watchdog lines need one line here rather than several hundred.

    @param ev    The events table.
    @param unit  Which board.
    @return Lines to print, empty for a unit with none.
    """
    if ev is None or ev.empty or "level" not in ev.columns:
        return []
    e = ev[(ev["unit"] == unit) & (ev["level"] == "E")]
    if e.empty:
        return []
    out = [f"      {'error lines':18s} {len(e):>8,}   "
           f"ESP_LOGE on the console -- read raw.log, not this summary"]
    for tag, n in e["tag"].value_counts().items():
        first = " ".join(str(e[e["tag"] == tag]["text"].iloc[0]).split())
        out.append(f"          {tag}: {n:,} -- {first[:66]}")
    return out


def episodes(windows, gap_s=EPISODE_GAP_S):
    """
    @brief Group non-zero tx-fail windows into bursts separated by quiet time.
    @param windows  The non-zero windows, in time order.
    @param gap_s    Quiet time that starts a new episode.
    @return A list of episodes, each a list of rows, in time order.
    """
    out = []
    for _, r in windows.iterrows():
        if not out or r["wall_s"] - out[-1][-1]["wall_s"] > gap_s:
            out.append([])
        out[-1].append(r)
    return out


def nearest(df, wall_s):
    """
    @brief Sample a series at the moment closest to an instant.

    The lines being compared are printed by different tasks on different
    boards, so they never share a timestamp. Matching on the nearest one is
    what lets a starved window on a satellite be read against the hub's
    fan-out gap for the same few seconds.

    @param df      A gauge frame, or None.
    @param wall_s  The instant to sample at.
    @return The value, or nan when there is nothing to sample.
    """
    if df is None or df.empty:
        return float("nan")
    i = (df["wall_s"] - wall_s).abs().values.argmin()
    return df["value"].iloc[i]


def fmt_dur(seconds):
    """
    @brief Seconds as `1h02m03s`, dropping the hours when there are none.
    @param seconds  The duration.
    @return The formatted string.
    """
    h, rem = divmod(int(seconds), 3600)
    m, s = divmod(rem, 60)
    return f"{h}h{m:02d}m{s:02d}s" if h else f"{m}m{s:02d}s"


def head(title):
    """
    @brief Print a section heading with a rule under it.
    @param title  The heading.
    """
    print(f"\n{title}")
    print("-" * len(title))


def main():
    """
    @brief Load the sessions, then print the nine sections in order.
    """
    ap = argparse.ArgumentParser(description="Summarise a soak captured by capture.py")
    ap.add_argument("session", nargs="+", help="session directory/directories")
    ap.add_argument("--wide", metavar="CSV",
                    help="also write a resampled wide table for your own plotting")
    ap.add_argument("--bin", type=int, default=60,
                    help="--wide resample interval in seconds (default 60)")
    args = ap.parse_args()

    met, ev = load(args.session)
    met = boot_segments(met)
    # AIR_UNIT is not a board; it has its own section below. Left in the
    # per-unit list it would collect a SESSION row, an uptime and a FAULTS
    # verdict that all mean nothing.
    units = sorted(u for u in met["unit"].unique() if u != AIR_UNIT)

    t0, t1 = met["wall_s"].min(), met["wall_s"].max()
    head("SESSION")
    print(f"  wall clock   {fmt_dur(t1 - t0)}  ({len(met):,} metric rows)")
    for u in units:
        um = met[met["unit"] == u]
        boots = um["boot"].nunique()
        up = counter_total(met, u, "up")
        note = "" if boots == 1 else f"  ** {boots} boots -- counters summed per segment **"
        print(f"  {u:8s}     uptime {fmt_dur(up) if up else '?':>10s}"
              f"  {len(um):,} rows{note}")
        if boots > 5:
            print(f"      ** {boots} boots is not {boots} resets. Almost certainly two"
                  f" boards captured under one --unit name (or a corrupt capture);")
            print(f"         every '{u}' number below mixes both streams and is"
                  f" unreliable. Re-capture with unique names per board. **")

    # ---- 1. faults ----------------------------------------------------------
    head("FAULTS  (every one of these should read 0)")
    any_fault = False
    for u in units:
        bad = []
        for metric, meaning in FAULTS:
            n = counter_total(met, u, metric)
            if n:
                bad.append(f"      {metric:18s} {n:>8,}   {meaning}")
        bad += tx_faults(met, u)
        bad += error_faults(ev, u)
        if bad:
            any_fault = True
            print(f"  {u}:")
            print("\n".join(bad))
        else:
            print(f"  {u}: clean")
    if not any_fault:
        print("\n  Nothing broke.")

    # ---- 2. cross-unit divergence -------------------------------------------
    head("TRACK DIVERGENCE  (the number that decides PHASE_DEADBAND_US)")
    d = met[(met["kind"] == "divergence") & (met["metric"] == "apart_ms")]
    if d.empty:
        print("  No track boundaries captured. This is the metric the soak is for --")
        print("  it needs several track changes, so play a playlist rather than a track.")
    else:
        v = d["value"].abs()
        print(f"  {len(v)} boundaries")
        print(f"  median {v.median():.1f} ms   mean {v.mean():.1f} ms"
              f"   p90 {v.quantile(0.9):.1f} ms   max {v.max():.0f} ms")
        print(f"  under 3 ms: {(v <= 3).mean() * 100:.0f}%    under 6 ms:"
              f" {(v <= 6).mean() * 100:.0f}%")
        print()
        print("  Reference: 0.5-2.5 ms is the best this project has recorded.")
        if v.median() > 4:
            print("  Sitting well above that -- tightening the deadband is the lever,")
            print("  and the rate trim removed both things that made it unaffordable.")

    # ---- 3. the rate trim ---------------------------------------------------
    head("RATE TRIM")
    for u in units:
        drops = counter_total(met, u, "dropped") or 0
        dups = counter_total(met, u, "dup") or 0
        cu_drops = counter_total(met, u, "catchup-drops") or 0
        cu_dups = counter_total(met, u, "catchup-dups") or 0
        t = gauge(met, u, "trim", "trim_hz")
        if (t is None and drops == 0 and dups == 0
                and cu_drops == 0 and cu_dups == 0):
            print(f"  {u}: no trim data")
            continue
        span = max(t1 - t0, 1.0)
        line = f"  {u}: dropped {drops:,}  duplicated {dups:,}"
        if t is not None and len(t):
            line += (f"   trim {t['value'].min():+.0f}..{t['value'].max():+.0f} Hz"
                     f" (median {t['value'].median():+.0f})")
        print(line)
        print(f"      combined {(drops + dups) / span:.2f} frames/s over the run")
        # The drain's own pair, kept out of the frames/s line above because a
        # drain pays a knock in seconds at over a thousand frames/s and would
        # drown the trim's own arithmetic. Healthy shape: a short burst per
        # knock, drops-side or nothing.
        if cu_drops or cu_dups:
            print(f"      catch-up: {cu_drops:,} skipped / {cu_dups:,} replayed")
            if cu_dups > max(64, drops):
                print("      ** catchup replay dominates -- the slower-and-not-"
                      "continuous signature")

        # The frame rate must equal |trim_hz|: that is the mechanism's own
        # arithmetic, and a mismatch means the trim is not doing what it says.
        if t is not None and len(t) > 1:
            expect = t["value"].abs().mean()
            actual = (drops + dups) / span
            if expect > 0.2:
                err = abs(actual - expect) / expect * 100
                verdict = "matches" if err < 35 else "** DOES NOT MATCH **"
                print(f"      mean |trim| {expect:.1f} Hz vs {actual:.2f} frames/s"
                      f"  -> {verdict}")
        # Both directions climbing means it crossed zero repeatedly.
        if drops and dups:
            weak = min(drops, dups) / max(drops, dups)
            if weak > 0.25:
                print(f"      ** both directions active ({drops:,} vs {dups:,}) --"
                      f" the servo is hunting across zero **")
        rt = counter_total(met, u, "retunes") or 0
        print(f"      clock retunes {rt}"
              + ("  (coarse only, as intended)" if rt <= 2 else
                 "  ** more than a coarse match; the fine path should not retune **"))

    # ---- 4. phase -----------------------------------------------------------
    head("PHASE  (+ = playing late)")
    for u in units:
        p = gauge(met, u, "status", "phase")
        if p is None or p.empty:
            continue
        v = p["value"]
        settled = v[p["wall_s"] > t0 + 120]
        use = settled if len(settled) > 5 else v
        print(f"  {u}: median {use.median():+.0f} us   p05..p95"
              f" {use.quantile(0.05):+.0f}..{use.quantile(0.95):+.0f} us"
              f"   |max| {use.abs().max():.0f} us   n={len(use)}")

    # ---- 5. buffer depth ----------------------------------------------------
    head("BUFFER DEPTH  (a steady drift is what a continuous trim can cause)")
    for u in units:
        # The hub prints "local ring N bytes (N ms)", the satellite
        # "buffer N ms" -- two names for the same depth.
        b = gauge(met, u, "status", "ring_ms")
        if b is None or b.empty:
            b = gauge(met, u, "status", "buffer")
        if b is None or len(b) < 3:
            continue
        v, x = b["value"], b["wall_s"] - t0
        slope = np.polyfit(x, v, 1)[0] * 3600.0
        flag = "" if abs(slope) < 20 else "   ** drifting **"
        print(f"  {u}: {v.min():.0f}..{v.max():.0f} ms  median {v.median():.0f}"
              f"   trend {slope:+.1f} ms/hour{flag}")

    # ---- 6. delivery --------------------------------------------------------
    #
    # Not the buffer depth above, and the difference is the point. A hole in
    # delivery empties the ring, the DAC plays silence for the length of it,
    # and that is time the timeline does not give back -- so the catch-up drain
    # arms and the room hears a semitone of pitch on one speaker and not the
    # other. None of the loss counters need move for this to happen: packets
    # can be held and released in a lump with nothing lost at all.
    #
    # Four numbers say where the lump formed:
    #
    #   gap-max   longest silence between two arrivals
    #   lead-min  least of play_at minus arrival
    #   ring-low  shallowest the play task found
    #   starved   ms of digital zero the DAC emitted
    #
    # A gap with the lead COLLAPSED means the hub stamped on time and the
    # transport held them. The same gap with the lead still near its target
    # means they were stamped late, and the hub's own fanout-gap-max says so
    # from the other end.
    head("DELIVERY  (did the audio arrive evenly, and what it cost when it did not)")
    arr = met[met["kind"] == "arrival"]
    if arr.empty:
        print("  No ARRIVAL lines -- the units predate this instrument.")
    else:
        for u in sorted(arr["unit"].unique()):
            g = gauge(met, u, "arrival", "gap-max")
            ld = gauge(met, u, "arrival", "lead-min")
            rl = gauge(met, u, "arrival", "ring-low")
            st = gauge(met, u, "arrival", "starved")
            bm = gauge(met, u, "arrival", "burst-max")
            if g is None or g.empty:
                continue
            print(f"  {u}:")
            print(f"      gap-max    median {g['value'].median():.0f} ms"
                  f"   worst {g['value'].max():.0f} ms")
            if bm is not None and not bm.empty:
                print(f"      burst-max  median {bm['value'].median():.0f}"
                      f"   worst {bm['value'].max():.0f} packets released together")
            if ld is not None and not ld.empty:
                print(f"      lead-min   median {ld['value'].median():.0f} ms"
                      f"   worst {ld['value'].min():.0f} ms")
            if rl is not None and not rl.empty:
                print(f"      ring-low   median {rl['value'].median():.0f} ms"
                      f"   worst {rl['value'].min():.0f} ms")
            if st is None or st.empty:
                continue
            starved = st[st["value"] > 0]
            if starved.empty:
                print(f"      starved    never -- the DAC was fed for the whole run")
                continue
            print(f"      starved    {st['value'].sum():.0f} ms over"
                  f" {len(starved)} window(s)  ** this is what the room heard **")
            # Attribution, one line per starved window. The hub's fan-out gap
            # in the same window is the other end of the comparison; without it
            # the satellite's gap alone cannot tell a late send from a late
            # arrival. The hub's lead at the moment it STAMPED against the
            # satellite's lead when the packet ARRIVED is the transit time,
            # which is the one number that says which side of sendto() a hole
            # happened on. See n_lead_min_us in hub_s3/main/hub.h.
            fan = met[(met["kind"] == "status") &
                      (met["metric"] == "fanout-gap-max")][["wall_s", "value"]]
            hub_ld = met[(met["kind"] == "status") &
                         (met["metric"] == "lead-min")][["wall_s", "value"]]
            print("        when          starved   gap-max   sat lead   hub lead"
                  "   transit   hub fan-out")
            for _, r in starved.iterrows():
                hl = nearest(hub_ld, r["wall_s"])
                sl = nearest(ld, r["wall_s"])
                # nan means the gauge was not in that build; say so rather
                # than printing it, since "nan ms" reads like a measurement.
                hl_s = f"{hl:8.0f} ms" if hl == hl else "     n/a"
                transit = f"{hl - sl:7.0f} ms" if hl == hl and sl == sl else "    n/a"
                print(f"        +{r['wall_s'] - t0:7.0f}s   {r['value']:6.0f} ms"
                      f"  {nearest(g, r['wall_s']):7.0f} ms"
                      f"  {sl:8.0f} ms  {hl_s}"
                      f"  {transit}  {nearest(fan, r['wall_s']):9.0f} ms")
            if hub_ld.empty:
                print("        (hub lead-min absent -- the hub predates that gauge,"
                      " so transit cannot be computed)")
            print("        transit large (hub lead healthy, sat lead collapsed)"
                  " -> held AFTER sendto: driver queue, AP buffering, or the air.")
            print("        transit small (both leads collapsed together)"
                  " -> stamped late; the fault is upstream of the radio.")
            print("        Check the hub's sbc_in `max gap` for the same window"
                  " first: a source stall makes both collapse and is not a")
            print("        delivery fault at all -- it was half the starvation"
                  " on the 2026-08-19 20:04 soak.")

    fec_units = sorted({u for u in met[met["kind"] == "rx5s"]["unit"].unique()})
    fec_rows = []
    for u in fec_units:
        g = gauge(met, u, "rx5s", "gaps")
        f = gauge(met, u, "rx5s", "fec")
        if g is None or f is None or g.empty:
            continue
        def total(name):
            x = gauge(met, u, "rx5s", name)
            return 0 if x is None or x.empty else int(x["value"].sum())
        fec_rows.append((u, int(g["value"].sum()), total("fec"),
                         total("fec-lost"), total("fec-held"), total("fec-bad")))
    if fec_rows:
        # ---- 6b. XOR parity ----------------------------------------------------
        #
        # Redundancy can only be judged where losses happen, so the first thing
        # printed here is whether the run had any. A run with none says nothing
        # about parity at all, however good the other numbers look.
        #
        # The pairing that matters is `gaps` against `fec` on the same unit:
        #
        #   gaps       what the AIR lost -- counted at detection, pre-repair
        #   fec        of those, how many came back WHOLE
        #   fec-lost   ...and how many did not (two in a group, or no parity)
        #   fec-held   how often packets waited behind a hole for a parity
        #   fec-bad    parity that arrived and could not be trusted -- must be 0
        #
        # gaps being pre-repair is deliberate: it is what lets a quiet channel
        # be told apart from a working scheme. So `gaps 40, fec 38` is the
        # scheme working, not forty holes in the sound.
        head("XOR PARITY  (of what the air lost, how much came back whole)")
        k = gauge(met, "hub", "status", "fec-k")
        k_val = int(k["value"].iloc[-1]) if k is not None and not k.empty else None
        txs = gauge(met, "hub", "status", "fec-tx")
        nwin = 0 if txs is None or txs.empty else len(txs)
        # A MEDIAN over observed windows, not a sum: parity sent is a
        # per-window gauge on the hub's status line, and summing windows that
        # happened to arrive presents a sample as a total. The median survives
        # the dropped lines, and is the figure worth reading anyway -- parity
        # should be exactly the audio packet rate divided by K, which is a
        # ratio rather than a total.
        tx_med = None if not nwin else txs["value"].median()

        # The status window, recovered as the shortest interval between two
        # observed lines. @see window_cover
        win_s = None
        if nwin > 2:
            d = txs["wall_s"].diff().dropna()
            d = d[d > 0]
            if not d.empty:
                win_s = round(d[d <= d.min() * 1.5].median())

        rate = f"  =  {tx_med / win_s:.1f}/s" if tx_med and win_s else ""
        print(f"  hub: K={k_val if k_val is not None else '--'}"
              f"   parity {fmt_or(tx_med, '.0f')} per"
              f" {f'{win_s:.0f} s' if win_s else 'window'}{rate}")

        def hub_hits(name):
            g = gauge(met, "hub", "status", name)
            return 0 if g is None or g.empty else int((g["value"] > 0).sum())

        print(f"       withheld under backoff: {hub_hits('fec-cong')} of {nwin}"
              f" windows   ungroupable: {hub_hits('fec-skip')} of {nwin} windows")
        print("       (windows OBSERVED, not elapsed. The hub's serial capture"
              " flaps -- see window_sum() -- so these are lower bounds,")
        print("        and the satellite rows below are the side to trust.)")
        cong_hits = hub_hits("fec-cong")
        if k_val == 0:
            print("       ** parity is switched OFF in this build "
                  "(DANCEFLOOR_AUDIO_FEC_K=0) -- the rows below are the "
                  "unprotected baseline **")
        elif not tx_med:
            print("       ** no parity was sent all run -- check the build and "
                  "the hub's own status line before reading anything below **")
        if cong_hits:
            print(f"       parity stood down for the transmit pool in {cong_hits}"
                  " window(s). That is the design working: read it beside")
            print("       tx-fail (audio), which must NOT have risen with it -- if"
                  " both moved, parity is not what is holding the pool.")
        for u, gaps, rec, lost, held, bad in fec_rows:
            if gaps == 0:
                print(f"  {u}: no losses this run -- parity had nothing to repair,"
                      " and this run cannot judge it.")
                print("       Re-run under contention: a large download on a nearby"
                      " machine, not associated to the hub's AP.")
                continue
            par = gauge(met, u, "arrival", "fec-parity")
            par_n = int(par["value"].sum()) if par is not None and not par.empty else 0
            if tx == 0 and par_n == 0:
                print(f"  {u}: {gaps:,} lost on the air, all of them silence --"
                      " this run had no parity to repair them.")
                continue
            pct = 100.0 * rec / gaps
            print(f"  {u}: {gaps:,} lost on the air -> {rec:,} rebuilt whole"
                  f" ({pct:.0f}%), {lost:,} left as silence")
            hm = gauge(met, u, "arrival", "fec-hold-max")
            hold_s = ""
            if hm is not None and not hm.empty:
                worst = hm["value"].max()
                budget = (k_val - 2) * 20 if k_val else None
                flag = ("   ** past the (K-2) x 20 ms budget **"
                        if budget and worst > budget * 1.5 else "")
                hold_s = (f"   longest hold {worst:.0f} ms"
                          + (f" of ~{budget} ms expected" if budget else "")
                          + flag)
            print(f"       held {held:,} times   parity received {par_n:,}"
                  f"   bad {bad}{hold_s}")
            if bad:
                print("       ** fec-bad is not a radio fault. The hub and this"
                      " satellite disagree about the wire or about K;")
                print("          reflash both from the same tree. **")
            if lost > rec and gaps > 10:
                print("       More losses went unrepaired than repaired, which"
                      " means they are arriving in bursts rather than singly.")
                print("       Parity covers one loss per group of K; a burst"
                      " inside one group is out of its reach by construction.")

    air_gap = gauge(met, "hub", "status", "air-gap-max")
    if air_gap is not None and not air_gap.empty:
        # ---- 6c. the air's own reading, BEFORE the refusals ---------------------
        #
        # Every other hub-side stall number is stamped at sendto() return, so
        # ESP_WIFI_CACHE_TX_BUFFER_NUM silences all of them at once by queueing
        # in PSRAM instead of refusing. air-gap-max comes from the driver's
        # tx-done callback -- the radio finishing a frame -- so it is the one
        # that stays honest with a queue in front of it. Read it first, or a
        # quiet tx-fail reads as a fixed hub when it may only be a hidden one.
        # See tx_done_cb() in hub_s3/main/net.c.
        head("AIR GAP  (what the RADIO did -- a cache queue cannot hide this)")
        g = air_gap["value"]
        done = window_sum(met, "hub", "txdone")
        fail = window_sum(met, "hub", "txdone-fail")
        print(f"  widest silence between two transmitted frames, per window:")
        print(f"    median {g.median():.0f} ms   p90 {g.quantile(0.9):.0f} ms"
              f"   p99 {g.quantile(0.99):.0f} ms   max {g.max():.0f} ms")
        over = int((g >= 350).sum())
        print(f"    {over} of {len(g)} windows ({over / max(len(g), 1) * 100:.1f}%)"
              f" had a silence >= LEAD_US (350 ms) -- long enough that anything"
              f" queued through it plays late or not at all")
        if done:
            n_w, win_s, cov = window_cover(met, "hub", "txdone")
            rate = f"  = {done / cov:.0f}/s" if cov else ""
            print(f"  frames done {done:,}{rate}   never acked {fail:,}"
                  f"  ({fail / max(done, 1) * 100:.2f}% -- a ratio, so the"
                  f" sampling below does not affect it)")
            note = cover_note(n_w, win_s, cov, t1 - t0)
            if note:
                print(note)
            print("    a large air-gap WITH acks failing is the medium -- frames went"
                  " and died on the air.")
            print("    a large air-gap with acks CLEAN means frames are not being LOST;"
                  " retry-exhaustion is ruled out.")
            print("    it does NOT separate a driver that never dequeued from a busy"
                  " medium deferring us in CCA --")
            print("    a frame that waits for the air and then succeeds is not an ack"
                  " failure. Both look like this.")
        print()

    up = gauge(met, "hub", "status", "rssi-min")
    downs = {u: gauge(met, u, "arrival", "hub-rssi") for u in units if u != "hub"}
    downs = {u: d for u, d in downs.items() if d is not None and not d.empty}
    if up is not None and not up.empty and downs:
        # ---- 6d. the link, in both directions -----------------------------------
        #
        # The hub's rssi-min is what it HEARS FROM the satellites; the
        # satellites' hub-rssi is what they hear from IT. Antenna gain is
        # reciprocal, so a healthy pair of radios reads roughly symmetric. A
        # persistent gap is a transmit-chain fault at the weaker end, and it is
        # the one thing txdone-fail cannot see: a chain of retries that all
        # SUCCEED holds a buffer for its whole length and never counts.
        head("LINK  (uplink vs downlink -- a gap is one end's transmit chain)")
        u_med = up["value"].median()
        print(f"  uplink   hub hears satellites   median {u_med:6.0f} dBm"
              f"   min {up['value'].min():.0f}")
        worst = 0.0
        for u, d in sorted(downs.items()):
            d_med = d["value"].median()
            worst = max(worst, u_med - d_med)
            print(f"  downlink {u:12s} hears hub  median {d_med:6.0f} dBm"
                  f"   min {d['value'].min():.0f}   asymmetry {u_med - d_med:+.0f} dB")
        print()
        if worst >= 10:
            print(f"  ** {worst:.0f} dB ASYMMETRIC -- the satellites hear the hub much more"
                  f" faintly than it hears them.")
            print("     Reciprocity says a shared path cannot do that, so it is the HUB'S"
                  " TRANSMIT chain:")
            print("     antenna mismatch/VSWR, a bad u.FL seat, or the wrong"
                  " antenna-select resistor.")
        elif worst >= 5:
            print(f"  {worst:.0f} dB of asymmetry -- suggestive but within what different"
                  " radios and placements give.")
            print("  Worth a second run before reading anything into it.")
        else:
            print("  Symmetric within a few dB: both transmit chains look healthy, and a"
                  " hub antenna fault")
            print("  is NOT the explanation for held frames. Look past the radio.")
        print()

    # ---- 7. the hub's refused sends -----------------------------------------
    #
    # DELIVERY above says a hole was held AFTER sendto. This says whether the
    # hub refused to make the send at all, which is the other candidate and the
    # one that turned out to matter.
    #
    # The shape matters as much as the size, and the four burst-gap buckets
    # carry it: back-to-back, sub-beacon, beacon-locked, or long stretches far
    # apart are four different faults with four different fixes. The buckets
    # are documented where they are measured, in hub_s3/main/net.c.
    head("HUB TX  (a refused send is a hole in the sound, not a hub-local event)")
    if air_gap is not None and not air_gap.empty:
        print("  NOTE: read AIR GAP above first. With ESP_WIFI_CACHE_TX_BUFFER_NUM"
              " non-zero the driver")
        print("  queues instead of refusing, so a LOW number here is not by itself"
              " an improvement.")
    tx_any = False
    for u in units:
        tf = gauge(met, u, "status", "tx-fail")
        if tf is None or tf.empty:
            continue
        tx_any = True
        total = int(tf["value"].sum())
        span_h = max(t1 - t0, 1.0) / 3600.0
        if not total:
            print(f"  {u}: nothing refused -- every send reached the driver")
            continue
        audio = window_sum(met, u, "tx_fail_audio")
        enomem = window_sum(met, u, "space")
        n_w, win_s, cov = window_cover(met, u, "tx-fail")
        # Per hour OF WHAT WAS CAPTURED, not of the run: dividing a sampled
        # sum by wall-clock time understates the rate by the sampling factor on
        # top of the sum already being short.
        per_h = total / (cov / 3600.0) if cov else total / span_h
        print(f"  {u}: {total:,} refused in the captured windows"
              f" ({per_h:.0f}/hour while observed)")
        note = cover_note(n_w, win_s, cov, t1 - t0)
        if note:
            print(note)
        if audio:
            print(f"      {audio:,} of them audio  -> ~{audio * 12 / 1000.0:.1f} s of"
                  f" starvation expected at the measured ~12 ms each")
        else:
            print("      (audio share not recorded in this session --"
                  " `capture.py --replay <dir>` rebuilds it from raw.log)")
        # ENOMEM and EHOSTUNREACH are different faults with different fixes: a
        # pool or load problem against an ARP-seeding one. net.c keeps the
        # errno tally precisely so a bare count cannot conflate them.
        if enomem and enomem < total:
            print(f"      errno split: {enomem:,} ENOMEM, {total - enomem:,} other"
                  f" -- raw.log carries the tally, and EHOSTUNREACH is an"
                  f" ARP-seeding fault, not a pool one")

        windows = tf[tf["value"] > 0]
        rmax = gauge(met, u, "status", "refuse-max")
        cong = gauge(met, u, "status", "cong-skip")
        fan = gauge(met, u, "status", "fanout-gap-max")
        aud = gauge(met, u, "status", "tx_fail_audio")
        buckets = [gauge(met, u, "status", f"burst-gap-{b}")
                   for b in ("lt25", "25-75", "75-150", "gt150")]
        have_buckets = all(b is not None and not b.empty for b in buckets)

        # Episodes first, because this is the figure a verdict rests on and
        # the per-hour number above is the one that misleads. See EPISODE_GAP_S.
        eps = episodes(windows)
        # The status line's own cadence, so an episode's duration counts its
        # last window rather than ending when that window started. The FASTEST
        # cadence, not the median: the hub prints this line at one rate when
        # quiet and a faster one once it has something to report, so the median
        # is the quiet rate while every episode is sampled at the other.
        gaps = tf["wall_s"].diff()
        gaps = gaps[gaps > 0.5]
        period = gaps.min() if not gaps.empty else 5.0
        majors = []
        print(f"      {len(eps)} episode(s) in {span_h:.2f} h"
              f"  (major = {EPISODE_MAJOR}+ refused)")
        for e in eps:
            n = int(sum(r["value"] for r in e))
            a = sum(x for x in (nearest(aud, r["wall_s"]) for r in e) if x == x)
            a_s = f"{a:6,.0f}" if aud is not None else "   n/a"
            dur = e[-1]["wall_s"] - e[0]["wall_s"] + period
            if n >= EPISODE_MAJOR:
                majors.append((e[0]["wall_s"], dur))
            print(f"        +{e[0]['wall_s'] - t0:7.0f}s  {dur:6.0f}s"
                  f"  {n:6,} refused ({a_s} audio)"
                  f"  {'MAJOR' if n >= EPISODE_MAJOR else 'minor'}")
        if majors:
            cov = sum(d for _, d in majors)
            print(f"      major episodes cover {cov:.0f}s ="
                  f" {100 * cov / max(t1 - t0, 1.0):.1f}% of the run;"
                  f" clean for {fmt_dur(majors[0][0] - t0)} before the first")

        print("        when        refused    audio   refuse-max   cong-skip"
              "   fan-out   burst gaps")
        for _, r in windows.iterrows():
            w = r["wall_s"]
            a = nearest(aud, w)
            a_s = f"{a:7.0f}" if a == a else "    n/a"
            gaps = ("  " + "/".join(f"{nearest(b, w):.0f}" for b in buckets)
                    if have_buckets else "   n/a")
            print(f"        +{w - t0:7.0f}s  {r['value']:8.0f}  {a_s}"
                  f"   {nearest(rmax, w):8.0f} ms"
                  f"   {nearest(cong, w):9.0f}"
                  f"   {nearest(fan, w):6.0f} ms{gaps}")
        if have_buckets:
            tot = [int(b["value"].sum()) for b in buckets]
            print(f"        burst gaps <25 / 25-75 / 75-150 / >150 ms:"
                  f"  {tot[0]} / {tot[1]} / {tot[2]} / {tot[3]}")
            print("        a pile in <25 is a stall that never lets go; in 75-150"
                  " the DTIM hold; in >150 isolated")
            print("        long stretches far apart -- the drain, not the fill."
                  " See hub_s3/main/net.c.")
            if tot[2] <= max(1, sum(tot) // 20):
                print("        75-150 is the 102.4 ms beacon. Near-empty here is"
                      " the DTIM drain STAYING fixed;")
                print("        the group lanes went unicast and the signature"
                      " went with them. Do not re-open it.")

        near = gauge(met, "hub", "status", "refuse-near-frame")
        rtry = gauge(met, "hub", "status", "audio-retry")
        rok = gauge(met, "hub", "status", "audio-retry-ok")
        # ---- who held the pool, and whether the retry caught anything -------
        #
        # Both gauges are absent from a hub built before they were added, so
        # their absence is reported as "not measured" rather than as a zero.
        if near is None or near.empty:
            print("\n      No refuse-near-frame gauge -- this hub predates the"
                  " 2026-08-23 instrumentation.")
        else:
            n_near, n_ref = near["value"].sum(), windows["value"].sum()
            print(f"\n      WHO HELD THE POOL: {n_near:,.0f} of {n_ref:,.0f}"
                  f" refusals had a frame batch in flight"
                  f" ({100 * n_near / max(n_ref, 1):.0f}%).")
            if n_near >= 0.5 * max(n_ref, 1):
                print("      The frame lane is still the competitor and"
                      " TX_FRAME_PACE_US is not enough -- pace it harder or"
                      " give audio its own headroom.")
            else:
                print("      The frame lane is mostly NOT in flight, so the pool"
                      " is drained by something that is not us:")
                print("      the driver's own retries, which is the air. A"
                      " hub-side lane fix cannot reach that.")
        # Recoverable, and worth recovering: a retry that SUCCEEDS skips
        # tx_fail_note_audio, so the audio lane's failure count is exactly the
        # retries that did not go. Reported as derived, because that holds only
        # while every audio refusal is ENOMEM.
        if rtry is not None and not rtry.empty and (rok is None or rok.empty):
            n_t = rtry["value"].sum()
            aud_f = window_sum(met, "hub", "tx_fail_audio")
            print(f"\n      THE RETRY: {n_t:,.0f} attempts, and audio-retry-ok"
                  " was TRUNCATED off the status line by servo.c's 96-byte")
            print(f"      burst buffer (fixed since). Derived instead:"
                  f" {max(n_t - aud_f, 0):,.0f} went"
                  f" -- attempts minus the {aud_f:,.0f} audio refusals still"
                  " counted.")
        elif rtry is not None and not rtry.empty:
            n_t = rtry["value"].sum()
            n_o = rok["value"].sum()
            print(f"      THE RETRY: {n_o:,.0f} of {n_t:,.0f} second attempts"
                  f" went ({100 * n_o / max(n_t, 1):.0f}%)"
                  f" -- {n_o:,.0f} holes that would have reached the floor.")
            if n_t and n_o < 0.1 * n_t:
                print("      Near zero: the pool stays empty longer than a"
                      " syscall, so the retry is not earning its place.")
    if not tx_any:
        print("  No tx-fail gauge -- the hub predates this instrument.")

    chose = gauge(met, "hub", "other", "chose")
    if chose is not None and not chose.empty:
        head("CHANNEL CHOICE  (the hub's survey against capture.py's sweep)")
        cands = [1, 6, 11]

        def near(unit, name, when, tol=5.0):
            g = gauge(met, unit, "other", name)
            if g is None or g.empty:
                return None
            d = (g["wall_s"] - when).abs()
            return None if d.min() > tol else g.loc[d.idxmin(), "value"]

        surveys = chose.reset_index(drop=True)
        for i, row in surveys.iterrows():
            ch, when = int(row["value"]), row["wall_s"]
            label = (f"  survey {i + 1} of {len(surveys)}  (+{when - t0:.0f}s)"
                     if len(surveys) > 1 else "  survey")
            print(f"{label}   chose ch{ch}")
            busy = {c: near("hub", f"ch{c}-busy", when) for c in cands}
            for c in cands:
                n = near("hub", f"ch{c}-nets", when)
                mark = "  <-" if c == ch else ""
                if busy[c] is not None:
                    f = near("hub", f"ch{c}-frames", when)
                    print(f"      ch{c:<3} busy {fmt_or(busy[c], '4.0f')}"
                          f"   frames {fmt_or(f, '5.0f')}"
                          f"   nets {fmt_or(n, '3.0f')}{mark}")
                else:
                    print(f"      ch{c:<3} nets {fmt_or(n, '3.0f')}"
                          f"   (no occupancy dwell in this build){mark}")
        if len(surveys) > 1:
            picks = sorted({int(v) for v in surveys["value"]})
            if len(picks) > 1:
                print(f"\n  ** {len(surveys)} surveys in this capture chose DIFFERENT"
                      f" channels {picks}. **")
                print("     The band moved between them, or the survey is not"
                      " reproducible. Either way this")
                print("     run is not on one channel throughout -- read it per boot"
                      " segment, not whole.")

        ran_on = int(surveys["value"].iloc[-1])
        sweep = {}
        for c in cands:
            g = gauge(met, AIR_UNIT, "other", f"ch{c}-nets")
            gd = gauge(met, AIR_UNIT, "other", f"ch{c}-dbm")
            sweep[c] = (None if g is None or g.empty else g["value"].median(),
                        None if gd is None or gd.empty else gd["value"].median())
        if any(n is not None for n, _ in sweep.values()):
            print("\n  capture.py sweep, the receiver that does NOT change between runs:")
            for c in cands:
                n, d = sweep[c]
                mark = "  <- ran on this" if c == ran_on else ""
                print(f"    ch{c:<3} nets {fmt_or(n, '4.0f')}"
                      f"   {fmt_or(d, '5.0f')} dBm{mark}")
            counts = [n for n, _ in sweep.values() if n is not None]
            mine = sweep.get(ran_on, (None, None))[0]
            if mine is not None and len(counts) > 1 and mine == max(counts):
                print(f"\n  ** THE HUB RAN ON THE BUSIEST CHANNEL THE SWEEP CAN SEE"
                      f" (ch{ran_on}, {mine:.0f} nets). **")
                print("     The two receivers disagree, and the sweep is the one that"
                      " did not change.")
                print("     Expect contention: frames LOST (txdone-fail) rather than"
                      " held (air-gap-max).")
            elif mine is not None and counts and mine == min(counts):
                print(f"\n  Agrees: ch{ran_on} is also the quietest the sweep can see.")
        print()

    head("AIR  (what else was on the band, from capture.py's sweep)")
    air = met[met["unit"] == AIR_UNIT]
    if air.empty:
        print("  No air data -- this session predates the sweep, or ran with")
        print("  --air-interval 0. capture.py records it as the pseudo-unit"
              f" '{AIR_UNIT}'.")
    else:
        chans = sorted({int(m[2:m.index("-")]) for m in air["metric"].unique()
                        if m.startswith("ch") and "-dbm" in m})
        sweeps = len(air[air["metric"] == f"ch{chans[0]}-dbm"]) if chans else 0
        by_sweep = air.pivot_table(index="wall_s", columns="metric",
                                   values="value").sort_index()

        print(f"  {sweeps:,} sweeps")
        print("  channel    median      min      max   nets   blind")
        run_median, dbm, nets = {}, {}, {}
        for c in chans:
            d = gauge(met, AIR_UNIT, "other", f"ch{c}-dbm")
            n = gauge(met, AIR_UNIT, "other", f"ch{c}-nets")
            if d is None or d.empty:
                continue
            dbm[c], nets[c] = d, n
            ncol = f"ch{c}-nets"
            heard = (by_sweep[by_sweep[ncol] > 0][f"ch{c}-dbm"]
                     if ncol in by_sweep else d["value"])
            blind = sweeps - len(heard)
            if heard.empty:
                print(f"  ch{c:<7d}    deaf for every sweep"
                      f"                 {blind:5d}")
                continue
            run_median[c] = heard.median()
            nm = n["value"].median() if n is not None and not n.empty else float("nan")
            print(f"  ch{c:<7d} {heard.median():6.0f} dBm"
                  f" {heard.min():6.0f}   {heard.max():6.0f}"
                  f"   {nm:4.0f}   {blind:5d}")

        tf = gauge(met, "hub", "status", "tx-fail")
        eps = (episodes(tf[tf["value"] > 0])
               if tf is not None and not tf.empty else [])
        majors = [e for e in eps if sum(r["value"] for r in e) >= EPISODE_MAJOR]

        blinds = []
        for w, row in by_sweep.iterrows():
            gone = [c for c in chans if row.get(f"ch{c}-nets", 1) == 0]
            if gone:
                blinds.append((w, gone, row))

        def episode_near(w, lo, hi):
            for e in eps:
                off = e[0]["wall_s"] - w
                if lo <= off <= hi:
                    return e, sum(r["value"] for r in e), off
            return None, 0.0, 0.0

        if not blinds:
            print(f"\n  No blind sweep: every channel decoded at least one"
                  f" network in all {sweeps:,} sweeps.")
        else:
            print(f"\n  BLIND SWEEPS  ({len(blinds)} of {sweeps:,})  -- a"
                  " channel that decoded NOTHING while")
            print("  the others did -- possibly somebody loud on it, possibly"
                  " one marginal AP")
            print("  dropping out. The counts either side say which.")
            hits = 0
            idx = list(by_sweep.index)
            for w, gone, row in blinds:
                deaf = "/".join(f"ch{c}" for c in gone)
                others = "  ".join(f"ch{c} {row.get(f'ch{c}-nets', np.nan):.0f}"
                                   for c in chans)
                print(f"    +{w - t0:7.0f}s  {deaf} blind   total nets"
                      f" {row.get('nets', np.nan):.0f}   ({others})")
                i = idx.index(w)
                for c in gone:
                    col = f"ch{c}-nets"
                    def side(j):
                        return (f"{by_sweep[col].iloc[j]:.0f}"
                                if 0 <= j < len(idx) else "no sweep")
                    print(f"                 ch{c} read {side(i - 1)} before"
                          f" and {side(i + 1)} after, median"
                          f" {by_sweep[col].median():.0f} for the run")
                e, n, off = episode_near(w, 0.0, AIR_LEAD_S)
                if e is None:
                    print("                 -> no hub-TX episode followed")
                else:
                    kind = "MAJOR" if n >= EPISODE_MAJOR else "minor"
                    hits += kind == "MAJOR"
                    print(f"                 -> {kind} episode {off:.0f}s later"
                          f", {n:.0f} refused")
            def blind_before(e):
                return any(0.0 <= e[0]["wall_s"] - w <= AIR_LEAD_S
                           for w, _, _ in blinds)

            back = sum(blind_before(e) for e in majors)
            minors = [e for e in eps if sum(r["value"] for r in e)
                      < EPISODE_MAJOR]
            print(f"  {hits} of {len(blinds)} blind sweeps were followed by a"
                  f" MAJOR episode within {AIR_LEAD_S}s;")
            print(f"  {back} of {len(majors)} major episodes had a blind sweep"
                  f" in the {AIR_LEAD_S}s before.")
            print(f"  The control: {len(minors)} minor episodes,"
                  f" {sum(blind_before(e) for e in minors)} of them with a"
                  " blind sweep before.")

        if not majors:
            print("\n  No major hub-TX episode in this run, so nothing to"
                  " correlate against.")
        else:
            print(f"\n  during each MAJOR hub-TX episode, against the run"
                  f" median  (nets = fewest in the {AIR_LEAD_S}s before):")
            for e in majors:
                mid = e[len(e) // 2]["wall_s"]
                lead = by_sweep[(by_sweep.index >= e[0]["wall_s"] - AIR_LEAD_S)
                                & (by_sweep.index <= mid)]
                cells = []
                for c in chans:
                    if c not in dbm:
                        continue
                    ncol = f"ch{c}-nets"
                    nv = (lead[ncol].min() if ncol in lead and not lead.empty
                          else nearest(nets[c], mid))
                    if nearest(nets[c], mid) == 0:
                        cell = f"ch{c} no decode"
                    else:
                        v = nearest(dbm[c], mid)
                        cell = f"ch{c} {v:.0f} dBm ({v - run_median.get(c, v):+.0f})"
                    cells.append(cell + f" nets {nv:.0f}"
                                 + (" BLIND" if nv == 0 else ""))
                print(f"    +{e[0]['wall_s'] - t0:7.0f}s  " + "  ".join(cells))

        if blinds or majors:
            print("  Read this for DISAPPEARANCE, not occupancy: a passive"
                  " scan counts what it can")
            print("  decode, so an interferer makes a channel go quiet HERE"
                  " rather than loud. A channel")
            print("  that LOSES its networks is the thing to look at, not one"
                  " that gets busier.")
            print("  BUT DO NOT CALL IT THE CAUSE. The 2026-08-23 01:38 run"
                  " went 8.42 h and 1,008")
            print("  sweeps without a single blind sweep and still had two"
                  " major episodes, larger")
            print("  than either of the run that suggested this. A blackout is"
                  " not necessary for an")
            print("  episode; check whether the deaf channel had more than one"
                  " or two networks to")
            print("  lose before reading anything into a zero. All three"
                  " channels moving together is")
            print("  the whole band. A few dB is sweep noise -- capture.py"
                  " measures ~10 dB of it on a")
            print("  busy channel; see air_scan() there.")

    # ---- 8. heap ------------------------------------------------------------
    head("HEAP  (the long-run question: does anything leak)")
    for u in units:
        h = gauge(met, u, "health", "heap")
        if h is None or len(h) < 3:
            continue
        v, x = h["value"], h["wall_s"] - t0
        slope = np.polyfit(x, v, 1)[0] * 3600.0
        mn = gauge(met, u, "health", "min")
        flag = "" if slope > -2000 else "   ** trending down **"
        line = (f"  {u}: {v.iloc[-1]:,.0f} B free at the end"
                f"   trend {slope:+,.0f} B/hour{flag}")
        if mn is not None and len(mn):
            line += f"   lowest since boot {mn['value'].min():,.0f} B"
        print(line)
        i = gauge(met, u, "mem", "internal")
        if i is not None and len(i):
            print(f"      internal pool: min {i['value'].min():,.0f} B"
                  f"  (the one that constrains the hub)")

    # ---- 9. the source ------------------------------------------------------
    head("SOURCE  (a stall here looks like a unit fault and is not one)")
    pk = met[(met["kind"] == "sbc_in") & (met["metric"] == "pkts")]
    if pk.empty:
        print("  No sbc_in lines -- the hub's console was not captured.")
    else:
        stalls = pk[pk["value"] == 0]
        print(f"  {len(pk)} windows, {pk['value'].median():.0f} packets/window median")
        if len(stalls):
            print(f"  ** {len(stalls)} window(s) with ZERO packets -- the phone or the"
                  f" SPI link stopped **")
            print(f"     first at +{stalls['wall_s'].iloc[0] - t0:.0f}s into the run")

        eff = met[(met["kind"] == "sbc_in") & (met["metric"] == "eff")]
        if not eff.empty:
            nominal = eff["value"].median()
            weak = eff[eff["value"] < 0.9 * nominal]
            if len(weak):
                print(f"  ** {len(weak)} window(s) UNDER-DELIVERED -- source fed"
                      f" below 90% of its own {nominal:.0f} Hz median **")
                print(f"     worst {weak['value'].min():.0f} Hz"
                      f" ({100 * weak['value'].min() / nominal:.0f}%)"
                      f" at +{weak.loc[weak['value'].idxmin(), 'wall_s'] - t0:.0f}s")
                gap = met[met["metric"] == "audio-gap-ms"]
                drops = met[met["metric"] == "bridge-dropped"]
                bounds = ev[(ev["kind"] == "boundary") & (ev["unit"] == "hub")] \
                    if not ev.empty and "kind" in ev.columns else pd.DataFrame()
                print("     window     fed   nearest track boundary   what bt_bridge saw")
                for w, v in zip(weak["wall_s"], weak["value"]):
                    near = "     --"
                    if not bounds.empty:
                        dt = (bounds["wall_s"] - w)
                        i = dt.abs().values.argmin()
                        near = f"{dt.iloc[i]:+.0f}s"
                    if gap.empty and drops.empty:
                        saw = "not captured -- add it as a --unit"
                    else:
                        near_gap = gap[(gap["wall_s"] > w - 30) & (gap["wall_s"] < w + 30)]
                        nd = drops[(drops["wall_s"] > w - 30) & (drops["wall_s"] < w + 30)]
                        saw = (f"audio gap {near_gap['value'].max():.0f} ms"
                               if not near_gap.empty else "no audio gap")
                        saw += (f", {nd['value'].max():.0f} dropped"
                                if not nd.empty else ", no drops")
                    print(f"     +{w - t0:7.0f}s  {100 * v / nominal:3.0f}%"
                          f"   {near:>10s}             {saw}")
                print("     A source that halves starves the hub's own ring, and the"
                      " timeline jump that")
                print("     follows re-anchors every satellite. Gap logged on the"
                      " bridge with no drops")
                print("     = the phone stopped; no gap but the hub short = it went"
                      " missing after the bridge.")
        if not len(stalls) and (eff.empty or not len(weak)):
            print("  No stalls: the source never stopped, and never slowed.")
        g = met[(met["kind"] == "sbc_in") & (met["metric"] == "gap")]
        if not g.empty:
            print(f"  longest silence between packets: {g['value'].max() / 1000:.0f} ms"
                  f"   (median window max {g['value'].median() / 1000:.0f} ms)")

    head("EVENTS")
    if ev.empty:
        print("  none recorded")
    else:
        interesting = ev[ev["kind"].isin(
            ["underrun", "timeline_restart", "timeline_start", "alloc_fail",
             "crippled", "retune"])]
        counts = ev.groupby(["unit", "kind"]).size()
        for u in units:
            if u not in counts.index.get_level_values(0):
                print(f"  {u}: none")
                continue
            per = counts.loc[u].sort_values(ascending=False)
            print(f"  {u}: " + ", ".join(f"{k} {int(n)}" for k, n in per.items()))
        if not interesting.empty:
            print("\n  when the notable ones happened:")
            for _, r in interesting.head(25).iterrows():
                print(f"    +{r['wall_s'] - t0:7.0f}s  {r['unit']:6s} {r['kind']}")
            if len(interesting) > 25:
                print(f"    ... and {len(interesting) - 25} more (see events.csv)")

    if args.wide:
        w = met.copy()
        w["bin"] = ((w["wall_s"] - t0) // args.bin) * args.bin
        wide = w.pivot_table(index="bin", columns=["unit", "metric"],
                             values="value", aggfunc="last")
        wide.columns = [f"{u}.{m}" for u, m in wide.columns]
        wide.to_csv(args.wide)
        print(f"\nwide table -> {args.wide}  ({wide.shape[0]} rows x "
              f"{wide.shape[1]} cols, {args.bin}s bins)")

    print()


## @cond
# The entry point, not API.
if __name__ == "__main__":
    main()
## @endcond
