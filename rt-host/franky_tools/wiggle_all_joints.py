#!/usr/bin/env python
"""wiggle_all_joints.py — franky 版,依次 wiggle J1..J7 ±0.3rad 系统诊断。
对标 scripts/wiggle_all_joints.cpp。每关节 0→+A→0→-A→0,Ruckig 平滑。
末尾可选夹爪开闭(无 Franka Hand 时自动跳过)。

⚠️ 全关节运动:确保机器人周围无障碍、远离桌面(建议先 goto_home)。

用法: python wiggle_all_joints.py <ip> [--amplitude 0.3] [--factor 0.25] [--no-gripper]
"""
import argparse, sys
import franky
from franky import JointMotion
import franky_helpers as fh


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ip", nargs="?", default=fh.ROBOT_IP)
    ap.add_argument("--amplitude", type=float, default=0.3, help="rad/关节")
    ap.add_argument("--factor", type=float, default=0.25)
    ap.add_argument("--no-gripper", action="store_true")
    args = ap.parse_args()

    robot = fh.connect(args.ip)
    fh.setup(robot, load_mass=0.25, collision=20, factor=args.factor)
    q_init = list(robot.current_joint_state.position)
    A = args.amplitude
    print(f"Plan: wiggle J1..J7 each (0→+{A}→0→-{A}→0)")
    try:
        for j in range(7):
            for delta in (A, 0.0, -A, 0.0):
                q = list(q_init); q[j] = q_init[j] + delta
                robot.move(JointMotion(q))
            print(f"  J{j+1} done")
        if not args.no_gripper:
            try:
                g = franky.Gripper(args.ip)
                print("  gripper close→open...")
                g.move(0.01, 0.05); g.open(0.05)
            except Exception as e:
                print(f"  (跳过夹爪: {type(e).__name__})")
        print(f"✅ Done. mode={robot.state.robot_mode}")
        return 0
    except franky.ControlException as e:
        print(f"❌ {str(e)[:120]}", file=sys.stderr); return -1


if __name__ == "__main__":
    sys.exit(main())
