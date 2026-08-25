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
