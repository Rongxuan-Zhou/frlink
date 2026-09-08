#!/usr/bin/env python
"""gripper_cmd.py — franky 版一次性夹爪命令。对标 teleop/gripper_cmd.cpp(1:1)。

⚠️ 需 Franka Hand 在位(PushT 推杆构型无夹爪时连接会报 FCI refused,与 C++ 原版一致)。
夹爪连接独立于机械臂 FCI motion control,可与 servo 共存。

用法:
  python gripper_cmd.py <ip> read
  python gripper_cmd.py <ip> open
  python gripper_cmd.py <ip> close [--force 60]
  python gripper_cmd.py <ip> width <w_m> [--speed 0.1]
  python gripper_cmd.py <ip> grasp <w_m> [--speed 0.1] [--force 60]
  python gripper_cmd.py <ip> homing
"""
import sys
import franky


def main():
    if len(sys.argv) < 3:
        print("Usage: gripper_cmd.py <ip> open|close|width <w>|grasp <w>|homing|read "
              "[--speed v] [--force f]", file=sys.stderr)
        return 1
    ip, cmd = sys.argv[1], sys.argv[2]
    speed, force = 0.1, 60.0   # 同 C++:60N 持续力(Franka Hand max 70N,UMI 软指需持续施力)
    rest = sys.argv[3:]
    pos = [a for a in rest if not a.startswith("--")]
    i = 0
    while i < len(rest):
        if rest[i] == "--speed": speed = float(rest[i+1]); i += 2
        elif rest[i] == "--force": force = float(rest[i+1]); i += 2
        else: i += 1
    try:
        g = franky.Gripper(ip)
        mw = g.max_width
        if cmd == "read":
            print(f'{{"width":{g.width},"max_width":{mw},"is_grasped":{str(g.is_grasped).lower()}}}')
        elif cmd == "open":
            g.open(speed)                                  # 张到 max
        elif cmd == "close":
            # 同 C++:grasp(0, ..., eps_inner=eps_outer=max_width) → 任意终宽视为成功,持续施力
            g.grasp(0.0, speed, force, epsilon_inner=mw, epsilon_outer=mw)
        elif cmd == "homing":
            g.homing()
        elif cmd in ("width", "grasp"):
            if not pos:
                print("missing width", file=sys.stderr); return 2
            w = float(pos[0])
            g.move(w, speed) if cmd == "width" else g.grasp(w, speed, force)
        else:
            print(f"unknown cmd: {cmd}", file=sys.stderr); return 3
        return 0
    except franky.GripperException as e:
        print(f"GripperException: {str(e)[:120]}", file=sys.stderr); return 4
    except Exception as e:
        print(f"{type(e).__name__}: {str(e)[:120]}", file=sys.stderr); return 4


if __name__ == "__main__":
    sys.exit(main())
