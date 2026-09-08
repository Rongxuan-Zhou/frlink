#!/usr/bin/env python3
"""Robot-state reader for the episode recorder: reads the files written by franka_state_mirror.

The RT host publishes the servo state as FRST1 datagrams; bin/franka_state_mirror writes each
body atomically to <dir>/franka_*.txt (default /tmp). This module turns those files into one
dict per poll for MultiCamRecorder.attach_state_fn:

    {"ee_pose": float32[16], "ee_ok": bool}                       # always
    {"wrench": float32[6], "wrench_ok": bool}                     # with include_wrench=True
    {"joint_state": float32[28], "joint_ok": bool}                # with include_joints=True

ee_pose is O_T_EE, column-major 4x4 (translation at indices 12, 13, 14), exactly the token
order of the file. ee_ok is False (and ee_pose all zeros) when the EE file is missing,
malformed, or its mtime is older than STALE_MS (250 ms = five missed 20 Hz samples): the servo
or the link is down and the sample must not be used as a label.

Usage:
    from state_reader import make_state_fn
    rec.attach_state_fn(make_state_fn())

    # self test while the mirror receives state (or tests/fake_frst1.py feeds it):
    python3 record/state_reader.py --selftest --secs 5 [--dir /tmp]
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Callable, Optional

import numpy as np

DEFAULT_DIR = "/tmp"
EE_NAME = "franka_current_ee.txt"       # 16 tokens @ 20 Hz
WRENCH_NAME = "franka_wrench.txt"       # 6 tokens  @ 20 Hz (O_F_ext_hat_K)
JOINT_NAME = "franka_joint_state.txt"   # 28 tokens @ 10 Hz (q dq tau_J tau_ext_hat_filtered)
STALE_MS = 250.0                        # older than this = servo dead / link silent


def _read_doubles(path: str, n: int) -> Optional[np.ndarray]:
    try:
        with open(path) as f:
            toks = f.read().split()
        if len(toks) != n:
            return None
        return np.array([float(t) for t in toks], dtype=np.float64)
    except Exception:
        return None


def _age_ms(path: str) -> float:
    try:
        return (time.time() - os.path.getmtime(path)) * 1e3
    except OSError:
        return float("inf")


def read_state_file(path: str, n: int, stale_ms: float = STALE_MS) -> tuple[np.ndarray, bool]:
    """(float32[n], ok). Missing / stale / malformed -> zeros + False."""
    if _age_ms(path) > stale_ms:
        return np.zeros(n, dtype=np.float32), False
    v = _read_doubles(path, n)
    if v is None:
        return np.zeros(n, dtype=np.float32), False
    return v.astype(np.float32), True


def read_franka_ee(ee_path: str = os.path.join(DEFAULT_DIR, EE_NAME),
                   stale_ms: float = STALE_MS) -> tuple[np.ndarray, bool]:
    """(ee_4x4_colmajor_flat[16], ok) with the staleness rule applied."""
    return read_state_file(ee_path, 16, stale_ms)


def make_state_fn(state_dir: str = DEFAULT_DIR, stale_ms: float = STALE_MS,
                  include_wrench: bool = False, include_joints: bool = False) -> Callable[[], dict]:
    """Factory for recorder.attach_state_fn. Each call reads the files once."""
    ee_path = os.path.join(state_dir, EE_NAME)
    wr_path = os.path.join(state_dir, WRENCH_NAME)
    jt_path = os.path.join(state_dir, JOINT_NAME)
    joint_stale = max(stale_ms, 2.5 * 100.0)   # joint file is 10 Hz: allow 250 ms as well

    def state_fn() -> dict:
        e, e_ok = read_state_file(ee_path, 16, stale_ms)
        out = {"ee_pose": e, "ee_ok": bool(e_ok)}
        if include_wrench:
            w, w_ok = read_state_file(wr_path, 6, stale_ms)
            out["wrench"] = w
            out["wrench_ok"] = bool(w_ok)
        if include_joints:
            j, j_ok = read_state_file(jt_path, 28, joint_stale)
            out["joint_state"] = j
            out["joint_ok"] = bool(j_ok)
        return out
    return state_fn


def _selftest(secs: float, state_dir: str, hz: float = 20.0) -> int:
    fn = make_state_fn(state_dir, include_wrench=True, include_joints=True)
    ee_path = os.path.join(state_dir, EE_NAME)
    t0 = time.time()
    n = ok = w_ok = j_ok = 0
    print(f"[selftest] polling {ee_path} for {secs:.0f}s @{hz:.0f}Hz ...")
    last = None
    first_pos = None
    while time.time() - t0 < secs:
        st = fn()
        n += 1
        ok += st["ee_ok"]
        w_ok += st["wrench_ok"]
        j_ok += st["joint_ok"]
        if st["ee_ok"]:
            last = st["ee_pose"]
            if first_pos is None:
                first_pos = last[12:15].copy()
        time.sleep(1.0 / hz)
    print(f"  samples={n}  ee_ok={ok}/{n}  wrench_ok={w_ok}/{n}  joint_ok={j_ok}/{n}")
    if last is not None:
        # column-major 4x4: translation = elements [12, 13, 14]
        moved = float(np.linalg.norm(last[12:15] - first_pos)) * 1000.0
        print(f"  last EE position = ({last[12]:.3f}, {last[13]:.3f}, {last[14]:.3f}) m; moved {moved:.1f} mm since first sample")
    if ok == 0:
        print(f"  ❌ EE path dead: {ee_path} missing or stale (> {STALE_MS:.0f} ms). Is the servo running and the mirror active?")
        print("     check: franka_state_mirror --check; franka-teleop status")
        return 1
    print("  ✅ EE path alive; recording can proceed")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--secs", type=float, default=5.0)
    ap.add_argument("--dir", default=DEFAULT_DIR, help="directory the mirror writes to (default /tmp)")
    args = ap.parse_args()
    if args.selftest:
        return _selftest(args.secs, args.dir)
    print("import me: from state_reader import make_state_fn")
    return 0


if __name__ == "__main__":
    sys.exit(main())
