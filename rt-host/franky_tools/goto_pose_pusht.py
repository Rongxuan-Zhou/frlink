#!/usr/bin/env python
"""goto_pose_pusht.py — franky 版,把末端慢速移到 PushT 起始姿。

对标 teleop/goto_pose_pusht.cpp(C++ 原版保留不动,本脚本并排提供)。
区别:Ruckig 平滑轨迹替代手写 quintic+slerp;贴桌起步自动先抬 Z 清桌。

用法:
  ~/franka/.venv_franky/bin/python goto_pose_pusht.py \
      [--ip 172.16.0.2] [--x 0.50] [--y 0.00] [--z 0.098] \
      [--mode top|side] [--speed 0.05] [--no-lift]

默认目标 (0.50, 0.00, 0.098) = launch_pusht.sh 的当前 PushT home(安全弯肘区)。
  top (默认): 俯视 Rx180, EE-z 朝下, 推杆垂直
  side      : 侧推, EE-z 朝 +x, 推杆水平
"""
import argparse
import sys
import franky
import franky_helpers as fh


def main():
    ap = argparse.ArgumentParser(description="franky 版 goto_pose_pusht")
    ap.add_argument("--ip", default=fh.ROBOT_IP)
    ap.add_argument("--x", type=float, default=0.50)
    ap.add_argument("--y", type=float, default=0.00)
    ap.add_argument("--z", type=float, default=fh.PLANAR_Z)
    ap.add_argument("--mode", choices=["top", "side"], default="top")
    ap.add_argument("--speed", type=float, default=0.05,
                    help="平移线速度 m/s(映射到 relative_dynamics_factor)")
    ap.add_argument("--no-lift", action="store_true",
                    help="不自动抬 Z 清桌(仅当确认当前已离桌时用)")
    args = ap.parse_args()

    # speed(m/s) → factor:FR3 max 平移 ~1.7m/s,保守映射并夹紧
    factor = min(max(args.speed / 1.7, 0.01), 0.3)
    R = fh.R_SIDE if args.mode == "side" else fh.R_TOP
    p = [args.x, args.y, args.z]

    print(f"[goto_pose_pusht/franky] mode={args.mode} 目标={p} "
          f"speed={args.speed}m/s(factor={factor:.3f}) lift={'off' if args.no_lift else 'on'}")
    robot = fh.connect(args.ip)
    fh.setup(robot, load_mass=0.0, collision=40, factor=factor)

    t0, _ = fh.current_ee(robot)
    print(f"  当前 EE = ({t0[0]:.4f}, {t0[1]:.4f}, {t0[2]:.4f})")
    try:
        tf, _ = fh.goto_pose(robot, R, p, factor=factor, lift_first=not args.no_lift)
        err = ((tf[0]-p[0])**2 + (tf[1]-p[1])**2 + (tf[2]-p[2])**2) ** 0.5
        print(f"✅ 到位 EE = ({tf[0]:.4f}, {tf[1]:.4f}, {tf[2]:.4f})  位置误差 {err*1000:.2f} mm")
        print(f"   mode={robot.state.robot_mode}")
        return 0
    except franky.ControlException as e:
        print(f"❌ reflex/control 异常: {str(e)[:120]}", file=sys.stderr)
        print(f"   mode={robot.state.robot_mode}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
