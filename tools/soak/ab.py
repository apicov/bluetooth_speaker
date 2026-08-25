#!/usr/bin/env python3
"""
The few numbers an A/B turns on, one block per capture, side by side.

    tools/soak/ab.py logs-soak-K4-n4/ logs-soak-K0-n4/

analyse.py is the full picture and stays the thing to read for a soak. This is
for the narrow question "same load, one setting changed -- what moved?", where
forty lines of context between the two figures being compared is the enemy.

MEDIANS PER WINDOW, NEVER TOTALS, and that is not a stylistic preference. The
hub's serial capture drops lines -- 122 open/lost cycles in 21 minutes on the
2026-08-25 runs, leaving 61% of its status windows -- so a sum over what arrived
is a sample presented as a population, and two runs with different capture luck
are not comparable at all. A median over captured windows is unaffected by how
many were missed, which is what makes the two halves of an A/B mean the same
thing. The count is printed so a run with too few is obvious.

ONE CAPTURE PER LOAD POINT. Every figure here is taken over the whole session,
so starting satsim halfway through a capture averages the loaded and unloaded
halves into a number describing neither. Stop the capture, change one thing,
start a new one.
"""

import sys

try:
    import pandas as pd
except ImportError:
    sys.exit("pandas is missing: pip install pandas")


def series(m, unit, kind, metric):
    s = m[(m.unit == unit) & (m.kind == kind) & (m.metric == metric)]["value"]
    return s if len(s) else None


def med(s, spec="6.0f"):
    return "    --" if s is None else format(s.median(), spec)


def main(dirs):
    for d in dirs:
        m = pd.read_csv(f"{d.rstrip('/')}/metrics.csv")
        span = (m.wall_s.max() - m.wall_s.min()) / 60.0
        print(f"\n=== {d.rstrip('/').split('/')[-1]}  ({span:.0f} min) ===")

        txd = series(m, "hub", "status", "txdone")
        n = 0 if txd is None else len(txd)
        print(f"  hub / status window (median of {n} captured)")
        if n < 8:
            print("    ** too few windows to compare -- run longer, or fix the"
                  " hub's serial link **")
        print(f"    tx-fail {med(series(m, 'hub', 'status', 'tx-fail'))}"
              f"  of which audio {med(series(m, 'hub', 'status', 'tx_fail_audio'))}"
              f"   cong-skip {med(series(m, 'hub', 'status', 'cong-skip'))}"
              f"   txdone {med(txd)}")
        print(f"    fec-tx  {med(series(m, 'hub', 'status', 'fec-tx'))}"
              f"  fec-cong       {med(series(m, 'hub', 'status', 'fec-cong'))}"
              f"   air-gap {med(series(m, 'hub', 'status', 'air-gap-max'))} ms")

        for u in ("sat_classic", "sat_s3"):
            pk = series(m, u, "arrival", "pkts")
            if pk is None:
                continue
            st = series(m, u, "arrival", "starved")
            # Starvation is the one figure summed rather than taken as a median:
            # it is what the room heard, and a median hides a short outage
            # entirely. gaps is the same -- what the air lost, in total.
            gaps = series(m, u, "rx5s", "gaps")
            print(f"  {u:<12s} pkts {pk.median():.0f}"
                  f"  parity {med(series(m, u, 'arrival', 'fec-parity'), '.0f')}"
                  f"  hold-max {med(series(m, u, 'arrival', 'fec-hold-max'), '.0f')} ms"
                  f"  gap-max {series(m, u, 'arrival', 'gap-max').median():.0f} ms"
                  f"  ring-low {series(m, u, 'arrival', 'ring-low').median():.0f} ms")
            print(f"  {'':<12s} starved {0 if st is None else st.sum():.0f} ms"
                  f"   gaps {0 if gaps is None else gaps.sum():.0f} (whole run)")
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    sys.exit(main(sys.argv[1:]))
