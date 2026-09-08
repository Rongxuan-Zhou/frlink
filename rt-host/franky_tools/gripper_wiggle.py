#!/usr/bin/env python
"""gripper_wiggle.py — franky version; Franka Hand open/close 2-cycle diagnostic.
Counterpart of scripts/gripper_wiggle.cpp. ⚠️ Requires the Franka Hand to be mounted.

Usage: python gripper_wiggle.py <ip> [--cycles 2] [--speed 0.05]
"""
import argparse, sys
import franky


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ip")
    ap.add_argument("--cycles", type=int, default=2)
    ap.add_argument("--speed", type=float, default=0.05)
    ap.add_argument("--closed", type=float, default=0.01, help="closed width in m (leaves some margin)")
    args = ap.parse_args()
    try:
        g = franky.Gripper(args.ip)
        mw = g.max_width
        print(f"Gripper: max_width={mw*1000:.1f}mm current={g.width*1000:.1f}mm "
              f"is_grasped={g.is_grasped}")
        print(f"Plan: close→open ×{args.cycles}")
        for i in range(1, args.cycles+1):
            print(f"[Cycle {i}] Closing to {args.closed*1000:.0f}mm...")
            if not g.move(args.closed, args.speed): print("  WARN: move() false")
            print(f"[Cycle {i}] Opening to {mw*1000:.0f}mm...")
            if not g.move(mw, args.speed): print("  WARN: move() false")
        print(f"Done. Final width: {g.width*1000:.1f}mm")
        return 0
    except Exception as e:
        print(f"{type(e).__name__}: {str(e)[:120]}", file=sys.stderr); return -1


if __name__ == "__main__":
    sys.exit(main())
