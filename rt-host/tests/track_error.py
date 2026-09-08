#!/usr/bin/env python3
"""track_error.py --cmd fake_sender.csv --ee state_sink_ee.csv [--lag-ms 60] [--start-skip 5]
Reports EE-vs-commanded tracking error (m): max, p95, mean; and EE drift if --cmd omitted (hold test)."""
import argparse, csv, sys
import numpy as np

def load(path):
    rows = list(csv.DictReader(open(path)))
    t = np.array([float(r["t_real"]) for r in rows]); p = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in rows])
    return t, p

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ee", required=True); ap.add_argument("--cmd", default="")
    ap.add_argument("--lag-ms", type=float, default=60.0, help="servo filter+impedance lag compensated before comparing")
    ap.add_argument("--start-skip", type=float, default=5.0, help="seconds skipped at start (settling)")
    ap.add_argument("--max-err-m", type=float, default=0.01); ap.add_argument("--max-drift-m", type=float, default=0.001)
    a = ap.parse_args()
    te, pe = load(a.ee)
    keep = te >= te[0] + a.start_skip; te, pe = te[keep], pe[keep]
    if not a.cmd:
        d = np.linalg.norm(pe - pe[0], axis=1)
        print(f"HOLD: samples={len(pe)} span={te[-1]-te[0]:.0f}s drift_max={d.max()*1e3:.2f}mm drift_final={d[-1]*1e3:.2f}mm "
              f"pos_std=({pe[:,0].std()*1e3:.2f},{pe[:,1].std()*1e3:.2f},{pe[:,2].std()*1e3:.2f})mm")
        ok = d.max() <= a.max_drift_m; print("RESULT", "PASS" if ok else "FAIL"); return 0 if ok else 1
    tc, pc = load(a.cmd)
    lo, hi = max(te[0], tc[0]), min(te[-1], tc[-1]); m = (te >= lo) & (te <= hi); te, pe = te[m], pe[m]
    cmd_at = np.stack([np.interp(te - a.lag_ms / 1e3, tc, pc[:, i]) for i in range(3)], axis=1)
    err = np.linalg.norm(pe - cmd_at, axis=1)
    print(f"TRACK: samples={len(err)} span={hi-lo:.0f}s err_max={err.max()*1e3:.2f}mm err_p95={np.percentile(err,95)*1e3:.2f}mm "
          f"err_mean={err.mean()*1e3:.2f}mm (lag comp {a.lag_ms}ms)")
    ok = err.max() <= a.max_err_m; print("RESULT", "PASS" if ok else "FAIL"); return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
