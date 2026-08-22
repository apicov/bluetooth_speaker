#!/usr/bin/env python3
"""
Read a soak session and say what happened.

Takes a directory written by capture.py and answers the questions a long run
exists to answer, in the order they matter:

  1. did anything break        the counters that must stay at zero
  2. cross-unit divergence     the number this project measures itself on
  3. the rate trim             is it running, at the rate it claims, and stable
  4. phase                     where each unit settled and how far it wandered
  5. buffer depth              a slow drift is what a continuous trim can cause
  6. delivery                  whether audio arrived evenly, and what a hole cost
  7. the hub's refused sends    where a hole in the sound is most often made
  8. heap                      the long-run question architecture.md 17 names
  9. the source                stalls, which look like unit faults and are not

REBOOTS ARE HANDLED, and this is the part worth knowing about before trusting a
number. Every counter on a HEALTH line is cumulative SINCE BOOT, so if a board
resets mid-soak they all return to zero. Taking a maximum would then silently
report the larger of the two runs instead of their sum. A reset shows up as the
board's own uptime going backwards, so the run is split into boot segments at
those points and each segment's final value is added.

Requires pandas. `pip install pandas`.

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


# Cumulative-since-boot counters. Anything not here is treated as a gauge.
COUNTERS = {
    "underruns", "restarts", "reanchors", "anchors", "splices", "retunes",
    "retunes_refused", "gaps", "ring-full", "dma-starve", "short-reads",
    "short-resync", "wifi-drops", "alloc-fail", "phase-drop", "seq-drop",
    "decode-err", "recv-err", "sta-left", "wifi-over", "fec-trunc",
    "refill-withheld", "anchors-refused", "upgrades", "dropped", "dup",
    "wide-span", "sta-timeout",
    # The faded catch-up's own pair, apart from the trim's dropped/dup because
    # a drain runs at ~1376 skip / ~688 replay frames/s by design and would
    # drown the trim's own frames/s arithmetic if mixed in. Expected: flat in
    # a clean run, a burst of a few seconds per knock.
    # See components/dancefloor_sync/audio_shift.h.
    "catchup-drops", "catchup-dups",
}

# What must stay at zero for a soak to have gone well, and what a non-zero
# reading means. Order is roughly worst-first.
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
    ("decode-err",   "SBC frames that would not decode"),
    ("recv-err",     "recvfrom() errors"),
    ("wifi-drops",   "lost association with the hub"),
    ("phase-drop",   "phase queue full, servo starved of input"),
]


def load(paths):
    """Concatenate one or more session directories into (metrics, events)."""
    mparts, eparts = [], []
    for p in paths:
        m = os.path.join(p, "metrics.csv")
        e = os.path.join(p, "events.csv")
        if not os.path.exists(m):
            sys.exit(f"{p}: no metrics.csv -- is it a capture.py session directory?")
        mparts.append(pd.read_csv(m))
        if os.path.exists(e) and os.path.getsize(e) > 0:
            eparts.append(pd.read_csv(e))
    # Stable: lines that arrived in the same millisecond (HEALTH/TRIM/MEM print
    # back-to-back) must keep their arrival order, or esp_ms flips backwards at
    # every burst and one boot shatters into thousands of fake ones.
    met = pd.concat(mparts, ignore_index=True).sort_values("wall_s", kind="stable")
    ev = (pd.concat(eparts, ignore_index=True).sort_values("wall_s", kind="stable")
          if eparts else pd.DataFrame(columns=["wall_s", "unit", "kind", "text"]))
    return met, ev


def boot_segments(df):
    """Label each row with which boot it belongs to, per unit.

    The board's own uptime only ever increases within a boot, so a decrease is a
    reset. Detected per (unit, kind) because different lines are emitted on
    different cadences and interleave.
    """
    df = df.sort_values(["unit", "wall_s"], kind="stable").copy()
    df["boot"] = (df.groupby("unit")["esp_ms"].diff().fillna(0) < 0) \
        .groupby(df["unit"]).cumsum().astype(int)
    return df


def counter_total(df, unit, metric):
    """Sum what `metric` gained across every boot segment of `unit`, in this window.

    Counters are cumulative since boot, and a capture usually opens on boards
    that booted hours ago: the first line already carries all that history, so
    a raw total reports the past, not the soak. Each boot segment is therefore
    baselined at its first observed value -- for a boot watched from esp=0 that
    baseline is ~0 and nothing changes; for one joined mid-life it subtracts
    everything that happened before the capture started.
    """
    # Only HEALTH and TRIM lines carry cumulative counters. The per-window RX 5s
    # lines repeat metric names ('gaps 1', 'ring-full 0') as gauges, and mixing
    # those into a cumulative series makes the baseline meaningless.
    s = df[(df["unit"] == unit) & (df["metric"] == metric)
           & df["kind"].isin(["health", "trim"])]
    if s.empty:
        return None
    per_boot = s.groupby("boot")["value"]
    return int((per_boot.max() - per_boot.first()).sum())


def gauge(df, unit, kind, metric):
    s = df[(df["unit"] == unit) & (df["kind"] == kind) & (df["metric"] == metric)]
    return s[["wall_s", "value"]].reset_index(drop=True) if not s.empty else None


def window_sum(df, unit, metric):
    """Total of a per-window `status` gauge over the run.

    tx-fail and the ENOMEM shape beside it are CLEARED by the window that
    prints them -- see tx_fail_summary() and tx_burst_summary() in
    hub_s3/main/net.c -- so the run total is a plain sum over windows.

    counter_total() is the wrong helper for them twice over: it baselines each
    boot segment against its first value, which is meaningless for a series
    that returns to zero every window, and it reads only health and trim lines
    while these live on the hub's `status` line.
    """
    s = gauge(df, unit, "status", metric)
    return 0 if s is None or s.empty else int(s["value"].sum())


def tx_faults(df, unit):
    """The hub's refused sends, as FAULTS lines. Empty for a unit with none.

    Here rather than in the FAULTS table because these are window gauges rather
    than cumulative counters, and here AT ALL because a refused send is not a
    hub-local inconvenience. The packet is never transmitted and never retried
    -- send_audio_to_clients() in hub_s3/main/timeline.c drops it -- so it
    reaches the floor as a hole.

    Across the three 2026-08-22 soaks every one of the 20 starved windows had a
    hub ENOMEM window within 25 s, none occurred without one, and the cost ran
    at 11.2 and 12.6 ms of satellite starvation per refused audio packet. The
    run that refused nothing starved not at all, and it was the longest of the
    three. Before this was reported the analyser called that hub "clean".
    """
    total = window_sum(df, unit, "tx-fail")
    if not total:
        return []
    out = [f"      {'tx-fail':18s} {total:>8,}   "
           f"sendto() refused -- see HUB TX below"]
    # Absent from sessions captured before the lane split was extracted;
    # `capture.py --replay <dir>` rebuilds those from their raw.log.
    audio = window_sum(df, unit, "tx_fail_audio")
    if audio:
        out.append(f"      {'of which audio':18s} {audio:>8,}   "
                   f"never reached the air (~12 ms of starvation each)")
    return out


def nearest(df, wall_s):
    """The value of `df` sampled closest in time to `wall_s`, or nan.

    The lines being compared are printed by different tasks on different boards,
    so they never share a timestamp; matching on the nearest one is what lets a
    starved window on the satellite be read against the hub's fan-out gap for
    the same few seconds.
    """
    if df is None or df.empty:
        return float("nan")
    i = (df["wall_s"] - wall_s).abs().values.argmin()
    return df["value"].iloc[i]


def fmt_dur(seconds):
    h, rem = divmod(int(seconds), 3600)
    m, s = divmod(rem, 60)
    return f"{h}h{m:02d}m{s:02d}s" if h else f"{m}m{s:02d}s"


def head(title):
    print(f"\n{title}")
    print("-" * len(title))


def main():
    ap = argparse.ArgumentParser(description="Summarise a soak captured by capture.py")
    ap.add_argument("session", nargs="+", help="session directory/directories")
    ap.add_argument("--wide", metavar="CSV",
                    help="also write a resampled wide table for your own plotting")
    ap.add_argument("--bin", type=int, default=60,
                    help="--wide resample interval in seconds (default 60)")
    args = ap.parse_args()

    met, ev = load(args.session)
    met = boot_segments(met)
    units = sorted(met["unit"].unique())

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
        print("  Reference: 0.5-2.5 ms is the best this project has recorded;")
        print("  clock-sync.md documents 2-9 ms as the expected deadband-bound range.")
        if v.median() > 4:
            print("  Sitting high in that band -- tightening the deadband is the lever,")
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
        if cu_drops or cu_dups:
            # The drain's own pair, kept out of the frames/s line above because
            # a drain pays a knock in seconds at ~1376/~688 frames/s and would
            # drown the trim's own arithmetic. Healthy shape: a short burst per
            # knock, drops-side or nothing.
            print(f"      catch-up: {cu_drops:,} skipped / {cu_dups:,} replayed")
            if cu_dups > max(64, drops):
                print("      ** catchup replay dominates -- the slower-and-not-"
                      "continuous signature")

        # The rate must equal |trim_hz|: that is the mechanism's own arithmetic.
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
        settled = v[p["wall_s"] > t0 + 120]           # ignore the startup walk-in
        use = settled if len(settled) > 5 else v
        print(f"  {u}: median {use.median():+.0f} us   p05..p95"
              f" {use.quantile(0.05):+.0f}..{use.quantile(0.95):+.0f} us"
              f"   |max| {use.abs().max():.0f} us   n={len(use)}")

    # ---- 5. buffer depth ----------------------------------------------------
    head("BUFFER DEPTH  (a steady drift is what a continuous trim can cause)")
    for u in units:
        # The hub prints "local ring N bytes (N ms)", the satellite "buffer N ms".
        b = gauge(met, u, "status", "ring_ms")
        if b is None or b.empty:
            b = gauge(met, u, "status", "buffer")
        if b is None or len(b) < 3:
            continue
        v, x = b["value"], b["wall_s"] - t0
        slope = np.polyfit(x, v, 1)[0] * 3600.0        # ms per hour
        flag = "" if abs(slope) < 20 else "   ** drifting **"
        print(f"  {u}: {v.min():.0f}..{v.max():.0f} ms  median {v.median():.0f}"
              f"   trend {slope:+.1f} ms/hour{flag}")

    # ---- 6. delivery --------------------------------------------------------
    head("DELIVERY  (did the audio arrive evenly, and what it cost when it did not)")
    #
    # The question this section exists to answer, and why it is not the buffer
    # depth above:
    #
    # On the 2026-08-19 soak a satellite took nine phase steps of +35 to +205 ms
    # while the hub stayed inside +-6 ms with tx-fail 0. Every step reported an
    # empty ring, and dma-starve moved by the step size -- so the DAC played
    # auto_clear silence for the length of the hole, which is time the timeline
    # does not give back, which arms the catch-up drain the room hears as a
    # semitone of pitch on one speaker and not the other.
    #
    # Nothing was LOST: seq-drop, decode-err, recv-err, wifi-drops and fec-err
    # all read zero, and the ring went from empty to 425 of its 464 ms inside one
    # window. So packets were held and released in a lump, and the only open
    # question was where the lump formed. These four numbers answer it:
    #
    #   gap-max   longest silence between two arrivals   steady ~20 ms
    #   lead-min  least of play_at minus arrival         steady ~LEAD_US (250 ms)
    #   ring-low  shallowest the play task found         steady ~RING_TARGET_MS
    #   starved   ms of digital zero the DAC emitted     steady 0
    #
    # A gap with the lead COLLAPSED means the hub stamped on time and the
    # transport held them. The same gap with the lead still near 250 means they
    # were stamped late, and the hub's own fanout-gap-max says so from the other
    # end.
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
            # Attribution, one line per starved window. The hub's fan-out gap in
            # the same window is the other end of the comparison; without it the
            # satellite's gap alone cannot tell a late send from a late arrival.
            fan = met[(met["kind"] == "status") &
                      (met["metric"] == "fanout-gap-max")][["wall_s", "value"]]
            # The hub's lead at the moment it STAMPED, against the satellite's
            # lead when the packet ARRIVED. The difference is the transit time,
            # and it is the one number that says which side of sendto() a hole
            # happened on. See n_lead_min_us in hub_s3/main/hub.h.
            hub_ld = met[(met["kind"] == "status") &
                         (met["metric"] == "lead-min")][["wall_s", "value"]]
            print("        when          starved   gap-max   sat lead   hub lead"
                  "   transit   hub fan-out")
            for _, r in starved.iterrows():
                hl = nearest(hub_ld, r["wall_s"])
                sl = nearest(ld, r["wall_s"])
                # nan means the gauge was not in that build; say so rather than
                # printing it, since "nan ms" reads like a measurement.
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

    # ---- 7. the hub's refused sends -----------------------------------------
    #
    # DELIVERY above says a hole was held AFTER sendto. This says whether the
    # hub refused to make the send at all, which is the other candidate and the
    # one that turned out to matter: on 2026-08-22 the hub read "clean" through
    # three runs while refusing 684, 0 and 63 sends, because nothing looked.
    #
    # The shape matters as much as the size, and the four burst-gap buckets are
    # what carry it -- back-to-back, sub-beacon, beacon-locked, or long
    # stretches far apart are four different faults with four different fixes.
    # The buckets are documented where they are measured, in hub_s3/main/net.c.
    head("HUB TX  (a refused send is a hole in the sound, not a hub-local event)")
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
        print(f"  {u}: {total:,} refused over the run ({total / span_h:.0f}/hour)")
        if audio:
            print(f"      {audio:,} of them audio  -> ~{audio * 12 / 1000.0:.1f} s of"
                  f" starvation expected at the measured ~12 ms each")
        else:
            print("      (audio share not recorded in this session --"
                  " `capture.py --replay <dir>` rebuilds it from raw.log)")
        # ENOMEM and EHOSTUNREACH are different faults with different fixes:
        # a pool/load problem against an ARP-seeding problem. net.c keeps the
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
    if not tx_any:
        print("  No tx-fail gauge -- the hub predates this instrument.")

    # ---- 8. heap ------------------------------------------------------------
    head("HEAP  (the long-run question: does anything leak)")
    for u in units:
        h = gauge(met, u, "health", "heap")
        if h is None or len(h) < 3:
            continue
        v, x = h["value"], h["wall_s"] - t0
        slope = np.polyfit(x, v, 1)[0] * 3600.0        # bytes per hour
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
        else:
            print("  No stalls: the source never stopped.")
        g = met[(met["kind"] == "sbc_in") & (met["metric"] == "gap")]
        if not g.empty:
            print(f"  longest silence between packets: {g['value'].max() / 1000:.0f} ms"
                  f"   (median window max {g['value'].median() / 1000:.0f} ms)")

    # ---- 10. events ----------------------------------------------------------
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

    # ---- optional wide table ------------------------------------------------
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


if __name__ == "__main__":
    main()
