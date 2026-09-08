#!/usr/bin/env python
"""wiggle_joint7.py — franky 版,J7 左右摆动诊断。对标 scripts/wiggle_joint7.cpp。
Ruckig 平滑替代 quintic。从当前姿态出发,只转 J7(手腕),无桌面交互。

用法: python wiggle_joint7.py <ip> [--cycles 3] [--amplitude 1.5] [--speed 0.3factor]
"""
import argparse, sys
import franky
from franky import JointMotion
import franky_helpers as fh


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ip", nargs="?", default=fh.ROBOT_IP)
    ap.add_argument("--cycles", type=int, default=3)
    ap.add_argument("--amplitude", type=float, default=1.5, help="rad (limit ±2.897)")
    ap.add_argument("--factor", type=float, default=0.3, help="relative_dynamics_factor")
    args = ap.parse_args()

    robot = fh.connect(args.ip)
    fh.setup(robot, load_mass=0.25, collision=20, factor=args.factor)
    q_init = list(robot.current_joint_state.position)
    A = args.amplitude
    print(f"Initial q[6]={q_init[6]:.3f} rad ({q_init[6]*57.296:.1f} deg)")
    print(f"Plan: {args.cycles} cycles of (0→+{A}→0→-{A}→0)")

    def move_j7(delta):
        q = list(q_init); q[6] = q_init[6] + delta
        robot.move(JointMotion(q))

    try:
        for c in range(args.cycles):
            for delta in (A, 0.0, -A, 0.0):
                move_j7(delta)
            print(f"  cycle {c+1}/{args.cycles} done")
        print(f"✅ Done. mode={robot.state.robot_mode}")
        return 0
    except franky.ControlException as e:
        print(f"❌ {str(e)[:120]}", file=sys.stderr); return -1


if __name__ == "__main__":
    sys.exit(main())
