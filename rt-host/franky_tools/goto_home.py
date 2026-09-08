#!/usr/bin/env python
"""goto_home.py — franky 版,安全回 factory-ready home。

对标 teleop/goto_home.cpp(--mode home)。贴桌起步自动先抬 Z 清桌 +
FK 路径预检,避免关节路径起步扎桌(见 franky_helpers 模块注释)。

用法: ~/franka/.venv_franky/bin/python goto_home.py [--ip ...] [--speed 0.07]
"""
import argparse
import sys
import franky
import franky_helpers as fh


def main():
    ap = argparse.ArgumentParser(description="franky 版 goto_home(factory ready)")
    ap.add_argument("--ip", default=fh.ROBOT_IP)
    ap.add_argument("--speed", type=float, default=0.07,
                    help="近似关节速度档,映射到 relative_dynamics_factor")
    args = ap.parse_args()
    factor = min(max(args.speed / 1.7, 0.01), 0.3)

    print(f"[goto_home/franky] → factory ready, factor={factor:.3f}")
    robot = fh.connect(args.ip)
    fh.setup(robot, load_mass=0.0, collision=30, factor=factor)
    t0, _ = fh.current_ee(robot)
    print(f"  当前 EE z={t0[2]:.4f}（贴桌则先抬升清桌）")
    try:
        qf = fh.goto_home(robot, factor=factor)
        err = max(abs(qf[i] - fh.HOME_Q[i]) for i in range(7)) * 1000
        print(f"✅ 到位 home, 最大关节误差 {err:.2f} mrad, mode={robot.state.robot_mode}")
        return 0
    except (franky.ControlException, RuntimeError) as e:
        print(f"❌ {str(e)[:140]}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
