#!/usr/bin/env python
"""goto_home.py — franky version; safely return to the factory-ready home.

Counterpart of teleop/goto_home.cpp (--mode home). When starting at the table it first lifts Z to clear the table +
FK path pre-check, so the joint-space path does not drive into the table at the start (see the franky_helpers module notes).

Usage: ~/franka/.venv_franky/bin/python goto_home.py [--ip ...] [--speed 0.07]
"""
import argparse
import sys
import franky
import franky_helpers as fh


def main():
    ap = argparse.ArgumentParser(description="franky version of goto_home (factory ready)")
    ap.add_argument("--ip", default=fh.ROBOT_IP)
    ap.add_argument("--speed", type=float, default=0.07,
                    help="approximate joint-speed level, mapped to relative_dynamics_factor")
    args = ap.parse_args()
    factor = min(max(args.speed / 1.7, 0.01), 0.3)

    print(f"[goto_home/franky] → factory ready, factor={factor:.3f}")
    robot = fh.connect(args.ip)
    fh.setup(robot, load_mass=0.0, collision=30, factor=factor)
    t0, _ = fh.current_ee(robot)
    print(f"  current EE z={t0[2]:.4f} (if at table level, lifts to clear the table first)")
    try:
        qf = fh.goto_home(robot, factor=factor)
        err = max(abs(qf[i] - fh.HOME_Q[i]) for i in range(7)) * 1000
        print(f"✅ Reached home, max joint error {err:.2f} mrad, mode={robot.state.robot_mode}")
        return 0
    except (franky.ControlException, RuntimeError) as e:
        print(f"❌ {str(e)[:140]}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
