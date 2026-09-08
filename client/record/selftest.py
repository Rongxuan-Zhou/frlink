#!/usr/bin/env python3
"""Recording-pipeline self test / camera-only recording entry point.

Runs without teleop and without the robot: records N seconds from every camera in
record/config/cameras.yaml into one HDF5 file and prints the hardware-timestamp statistics plus
a content check (at least 20 frames per second per camera at the configured 30 fps).

Usage:
    python3 record/selftest.py [seconds] [output.h5] [--cams role1,role2]
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_HERE = Path(__file__).resolve().parent            # <repo>/client/record
_CLIENT = _HERE.parent                             # <repo>/client
sys.path.insert(0, str(_CLIENT))

from record.cameras.grabber import load_camera_config   # noqa: E402
from record.cameras.recorder import MultiCamRecorder   # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("seconds", nargs="?", type=float, default=3.0)
    ap.add_argument("output", nargs="?", default=None)
    ap.add_argument("--config", default=str(_HERE / "config" / "cameras.yaml"))
    ap.add_argument("--cams", default=None, help="comma-separated roles (default: all)")
    args = ap.parse_args()
    secs = args.seconds
    out = args.output or f"/tmp/franka_record_selftest_{time.strftime('%Y%m%d_%H%M%S')}.h5"

    cfg = load_camera_config(Path(args.config))
    roles = [r.strip() for r in args.cams.split(",")] if args.cams else list(cfg["cameras"].keys())
    cams = [(r, str(cfg["cameras"][r]["serial"])) for r in roles]
    d = cfg.get("defaults", {})
    fps = d.get("fps", 30)
    rec = MultiCamRecorder(cams, width=d.get("width", 640), height=d.get("height", 480), fps=fps)
    started = rec.start()
    if not started:
        print("❌ no camera started (check the serials in record/config/cameras.yaml)")
        return 1

    print(f"\n[record] {secs}s ...")
    rec.begin_episode()
    time.sleep(secs)
    rec.end_episode()

    st = rec.stats()
    meta = rec.write_hdf5(out, attrs={"selftest": True, "duration_s": secs})
    rec.stop()

    print("\n=== recording result ===")
    ok = True
    for role in started:
        s = st[role]
        shape = meta.get(role)
        got_fps = (shape[0] / secs) if shape else 0
        good = shape is not None and shape[0] >= secs * (fps * 2 / 3)
        ok &= good
        print(f"  {'✅' if good else '❌'} {role:12s} SN={s['serial']} "
              f"buffered={s['buffered']} shape={shape} ~{got_fps:.1f}fps")

    # hardware-timestamp consistency
    print("\n=== HDF5 check ===")
    import h5py
    import numpy as np
    with h5py.File(out, "r") as f:
        for role in f["observations"]:
            g = f["observations"][role]
            ts = g["hw_timestamp_ms"][:]
            wt = g["wall_timestamp_s"][:]
            dt = np.diff(ts)
            print(f"  {role}: image{g['image'].shape} "
                  f"hw_ts span={ts[-1]-ts[0]:.0f}ms dt mean={dt.mean():.1f}ms std={dt.std():.1f}ms "
                  f"wall span={(wt[-1]-wt[0])*1000:.0f}ms "
                  f"locked_exposure={g.attrs.get('locked_exposure')}")
    print(f"\n→ {out}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
