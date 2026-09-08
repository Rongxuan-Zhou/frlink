#!/usr/bin/env python3
"""
WebXR (Quest 3) → local pose reader (decoupled from the robot)

Link-verification script: it only starts the SpesRobotics `teleop` HTTPS server (:4443) and
prints the 4x4 pose / raw quaternion / move / gripper / button state of every Quest
controller callback. No robot, no UDP: use it to prove the Quest ↔ PC link end to end before
starting the bridge (teleop/02_webxr_to_franka.py through bin/franka-teleop).

Start:
    $FRANKA_PY teleop/01_webxr_pose_reader.py
    # or with a simulated initial TCP pose so the accumulated pose is visible:
    $FRANKA_PY teleop/01_webxr_pose_reader.py --init-tcp 0.5,0.0,0.4

Quest side:
    open https://<this PC's LAN IP>:4443/ (or $FRANKA_TELEOP_URL) in the Meta Browser,
    accept the self-signed certificate, Enter VR, hold Trigger
    (swapped frontend: index finger = move, middle finger = gripper).
"""
from __future__ import annotations

import argparse
import os
import signal
import socket
import sys
import threading
import time
from typing import Optional

import numpy as np


FRONTEND_SWAPPED = os.path.join(os.path.dirname(os.path.abspath(__file__)), "frontend_swapped")
FUNNEL_URL = os.environ.get("FRANKA_TELEOP_URL", "")


def get_local_ip() -> str:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


def fmt_pose_4x4(T) -> str:
    p = T[:3, 3]
    R = T[:3, :3]
    trace = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    angle = float(np.arccos(trace))
    if abs(angle) < 1e-6:
        rotvec = np.zeros(3)
    elif abs(angle - np.pi) < 1e-3:
        diag = np.diag(R)
        axis = np.sqrt(np.maximum((diag + 1.0) / 2.0, 0.0))
        if R[2, 1] - R[1, 2] < 0: axis[0] = -axis[0]
        if R[0, 2] - R[2, 0] < 0: axis[1] = -axis[1]
        if R[1, 0] - R[0, 1] < 0: axis[2] = -axis[2]
        rotvec = axis * angle
    else:
        rotvec = np.array([R[2, 1] - R[1, 2],
                           R[0, 2] - R[2, 0],
                           R[1, 0] - R[0, 1]]) / (2.0 * np.sin(angle)) * angle
    return (
        f"pos=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f})m  "
        f"rotvec=({rotvec[0]:+.2f},{rotvec[1]:+.2f},{rotvec[2]:+.2f})  "
        f"|θ|={np.degrees(angle):5.1f}°"
    )


def fmt_quest_raw(msg: dict) -> str:
    pos = msg.get("position") or {}
    ori = msg.get("orientation") or {}
    return (
        f"raw_pos=({pos.get('x',0):+.3f},{pos.get('y',0):+.3f},{pos.get('z',0):+.3f})  "
        f"raw_quat=(x={ori.get('x',0):+.3f},y={ori.get('y',0):+.3f},"
        f"z={ori.get('z',0):+.3f},w={ori.get('w',1):+.3f})"
    )


def fmt_state(msg: dict) -> str:
    btn = []
    if msg.get("reservedButtonA"): btn.append("A")
    if msg.get("reservedButtonB"): btn.append("B")
    btn_str = "+".join(btn) if btn else "-"
    return (
        f"dev={msg.get('device','?')} move={msg.get('move')} "
        f"grip={msg.get('gripper')} btn={btn_str} "
        f"scale={msg.get('scale','?')} fps={msg.get('fps','?')}"
    )


class State:
    n: int = 0
    last_msg: Optional[dict] = None
    last_pose: Optional[np.ndarray] = None
    first: bool = False
    move_first_seen: bool = False
    last_print_t: float = 0.0
    stop: bool = False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--init-tcp", type=str, default=None,
                    help="initial TCP 'x,y,z' (m); with it the callback pose accumulates while move=True")
    ap.add_argument("--hz", type=float, default=2.0, help="print rate (default 2 Hz)")
    ap.add_argument("--duration", type=float, default=0, help="exit after N seconds (0 = Ctrl-C)")
    ap.add_argument("--default-frontend", action="store_true",
                    help="use the frontend bundled with the teleop package (default: teleop/frontend_swapped)")
    args = ap.parse_args()

    from teleop import Teleop

    state = State()
    period = 1.0 / args.hz

    def callback(pose: np.ndarray, message: dict) -> None:
        state.n += 1
        state.last_pose = pose
        state.last_msg = message
        if not state.first:
            state.first = True
            print(f"\n🎯 first callback (n=1): the link is up", flush=True)
        if message.get("move") and not state.move_first_seen:
            state.move_first_seen = True
            print(f"\n🚀 MOVE=True seen for the first time (n={state.n})", flush=True)

    if not args.default_frontend and os.path.isdir(FRONTEND_SWAPPED):
        teleop = Teleop(frontend_dir=FRONTEND_SWAPPED)
        print(f"  ✅ swapped frontend: {FRONTEND_SWAPPED}")
        print(f"     Trigger (index finger) = move dead-man   Grip (middle finger) = gripper toggle")
    else:
        teleop = Teleop()
        print(f"  ⚠️ default frontend (grip = move, trigger = gripper)")

    if args.init_tcp:
        x, y, z = (float(v) for v in args.init_tcp.split(","))
        init = np.eye(4)
        init[:3, 3] = [x, y, z]
        teleop.set_pose(init)
        print(f"  ✅ set_pose initial = ({x:+.3f},{y:+.3f},{z:+.3f})")

    teleop.subscribe(callback)
    threading.Thread(target=teleop.run, daemon=True).start()

    ip = get_local_ip()
    print("=" * 64)
    print("🚀 WebXR bridge ready")
    print("=" * 64)
    print(f"  LAN:         https://{ip}:4443")
    if FUNNEL_URL:
        print(f"  Funnel:      {FUNNEL_URL}  (FRANKA_TELEOP_URL; the funnel must be up)")
    print()
    print("  On the Quest:")
    print("    1) open one of the URLs above in the Meta Browser")
    print("    2) Enter VR")
    print("    3) hold Trigger (index finger) → move=True, the pose accumulates")
    print("    4) press Grip (middle finger) → gripper toggles open/close")
    print()
    print(f"  printing at {args.hz} Hz | callback ≈ 90 Hz | Ctrl-C to quit")
    print("=" * 64)

    def _sig(*_): state.stop = True
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    t_start = time.time()
    waited_warned = False
    try:
        while not state.stop:
            now = time.time()
            if not state.first:
                if not waited_warned and now - t_start > 5.0:
                    print(f"⏳ no callback after 5 s. Check:"
                          f"\n   • did the Quest enter the immersive session"
                          f"\n   • is TCP 4443 open in this PC's firewall"
                          f"\n   • is the Quest on the same network (or the funnel online)")
                    waited_warned = True
            elif now - state.last_print_t >= period:
                state.last_print_t = now
                msg = state.last_msg or {}
                p = state.last_pose
                elapsed = now - t_start
                print(f"\n[t={elapsed:6.2f}s n={state.n:5d}]  {fmt_state(msg)}")
                print(f"  QUEST  {fmt_quest_raw(msg)}")
                if p is not None:
                    print(f"  TCP    {fmt_pose_4x4(p)}")
            if args.duration and (now - t_start) >= args.duration:
                print(f"\n⏱  ran the full {args.duration}s, exiting")
                break
            time.sleep(0.05)
    finally:
        print("\n→ exit")
    return 0


if __name__ == "__main__":
    sys.exit(main())
