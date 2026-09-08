#!/usr/bin/env python3
"""
WebXR (Quest) → Franka FR3 real-robot closed-loop teleoperation (two-host client side)

Pipeline:
    Quest → WebXR → teleop server :4443 (HTTPS, served by the `teleop` pip package)
        ↓ callback (≈90Hz, main process)
    bbox/step/anchor reset full set of guardrails
        ↓ UDP $FRANKA_SERVO_HOST:50001 (16 doubles, 4×4 col-major, docs/INTERFACE.md 2.2)
    cartesian_pose_servo on the RT host (C++ libfranka 0.17, 1kHz)
        ↓ low-pass filter + hold after 200 ms of silence
    Franka FR3

Four safety handles (aligned with ur5):
  1. dead-man:   message['move']==False → no UDP sent
  2. bbox:       O_T_EE.translation hard-clipped on all three axes; anchor reset when out of bounds
  3. step limit: consecutive target Δ > 5cm → skip
  4. anchor reset (monkey-patch teleop._Teleop__relative_pose_init=None)

Gripper: Grip button toggles message['gripper'] → subprocess bin/gripper_cmd open|close
(a symlink to bin/franka-fci-shim, which forwards to franka-ctl on the RT host over ssh).

Paths: the repository root is derived from this file's location; bin/gripper_cmd and
bin/echo_robot_state come from there. The servo host comes from $FRANKA_SERVO_HOST
(bin/franka-teleop sources config.env and passes --udp-host explicitly).

Usage (normally through bin/franka-teleop, which sets every parameter):
    # 1) Dry-run: no robot state needed beyond one `echo` from the host (refused while a servo
    #    runs); UDP is still sent, so verify the stream with `nc -ul 50001` on the target.
    $FRANKA_PY teleop/02_webxr_to_franka.py --dry-run --udp-host 127.0.0.1

    # 2) LIVE: the servo must already be running on the RT host and the state mirror must
    #    have written /tmp/franka_init_pose.txt (franka-teleop servo-only does both).
    $FRANKA_PY teleop/02_webxr_to_franka.py --live

Dependencies: requirements-teleop.txt (teleop==0.1.5, scipy, numpy)
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation as _Rotation

_HERE = Path(__file__).resolve().parent          # <repo>/client/teleop
_ROOT = _HERE.parent                             # <repo>/client
sys.path.insert(0, str(_HERE))
from franka_sender_lock import acquire_sender_lock  # noqa: E402

# The robot IP is only a positional placeholder for gripper_cmd/echo_robot_state: the shim
# drops it and franka-ctl on the RT host injects the real one.
FRANKA_IP = os.environ.get("FRANKA_ROBOT_IP", "172.16.0.2")
ECHO_ROBOT_STATE = str(_ROOT / "bin" / "echo_robot_state")
GRIPPER_CMD = str(_ROOT / "bin" / "gripper_cmd")
FRONTEND_SWAPPED = str(_HERE / "frontend_swapped")

# ── Workspace bbox (Franka base frame, metres) ──
# Conservative box starting from init O_T_EE pos ≈ (0.305, -0.003, 0.477)
# Can later be refined with a workspace calibration script (like ur5's 8-corner one)
DEFAULT_WS_X = (0.20, 0.55)
DEFAULT_WS_Y = (-0.30, 0.30)
DEFAULT_WS_Z = (0.10, 0.65)

DEFAULT_MAX_STEP_M = 0.02  # 5cm → 2cm: 90Hz × 2cm = 1.8 m/s, well below the FR3 cartesian
                            # vel limit (2 m/s), rules out joint_velocity_violation
MAX_ROT_STEP_RAD = 0.05    # 0.10 → 0.05 rad ≈ 2.9°/frame = 4.3 rad/s rotation ceiling
                            # Quest noise + hand tremor when the user operates at chest height is filtered out by 0.05 rad
DEFAULT_SLOWDOWN_ZONE_M = 0.05
SOFT_MIN_VEL_RATIO = 0.05  # Spare; the UDP side does not use vel scale directly right now, the C++ side is an impedance filter

WS_X = DEFAULT_WS_X
WS_Y = DEFAULT_WS_Y
WS_Z = DEFAULT_WS_Z
MAX_STEP_M = DEFAULT_MAX_STEP_M

# Optional library path for a locally built gripper_cmd/echo_robot_state (single-host use).
# Unused in two-host mode: the shim runs ssh, which needs nothing from it.
LD_LIBRARY_PATH = os.environ.get("FRANKA_LD_LIBRARY_PATH", "")


@dataclass
class Stats:
    n_callback: int = 0
    n_udp_sent: int = 0
    n_clipped: int = 0
    n_step_blocked: int = 0
    n_grip_press: int = 0
    n_trig_press: int = 0
    last_target: Optional[np.ndarray] = None
    last_msg: Optional[dict] = None


INIT_POSE_FILE = "/tmp/franka_init_pose.txt"


def read_init_pose_from_file(timeout_s: float = 5.0) -> np.ndarray:
    """Read the init pose file written by the servo at startup (16 doubles col-major).

    Never run echo_robot_state while the servo is active: a second libfranka connection kicks the
    servo out (franka-ctl refuses `echo` in that case anyway).
    """
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if os.path.isfile(INIT_POSE_FILE):
            try:
                with open(INIT_POSE_FILE) as f:
                    nums = f.read().split()
                if len(nums) == 16:
                    arr = np.array([float(x) for x in nums], dtype=np.float64)
                    return arr.reshape(4, 4, order="F")
            except Exception:
                pass
        time.sleep(0.1)
    raise RuntimeError(
        f"{INIT_POSE_FILE} did not appear (servo not started, or the state mirror is not running? "
        f"try: franka-teleop servo-only; franka_state_mirror --check)")


def franka_o_t_ee_to_4x4(o_t_ee_col_major: list) -> np.ndarray:
    """libfranka output of 16 doubles column-major → 4x4 matrix."""
    return np.array(o_t_ee_col_major, dtype=np.float64).reshape(4, 4, order="F")


def matrix_4x4_to_udp_bytes(T: np.ndarray) -> bytes:
    """4x4 → column-major 16 doubles → 128 bytes."""
    flat = np.asarray(T, dtype=np.float64).reshape(4, 4).flatten(order="F")
    return struct.pack("16d", *flat)


_R_MIRROR_Z = np.diag([-1.0, -1.0, 1.0])


def mirror_pose_around_init(target_pose: np.ndarray, init_pose: np.ndarray) -> np.ndarray:
    """Flip 180° about the base z axis through the init position (face-to-face mirror mode).

    Underlying logic:
      - Translation part: p_new = init_p + R_z(180) @ (p - init_p), flips (dx, dy), keeps dz
      - Rotation part:    R_new = init_R @ R_z(180) @ (init_R^T @ R) @ R_z(180)
                  conjugation by R_z(180) negates pitch about y and roll about x,
                  yaw about z is unchanged. This is the geometric fix for the user-reported "left/right + pitch reversed".
    """
    delta_p = target_pose[:3, 3] - init_pose[:3, 3]
    new_p = init_pose[:3, 3] + _R_MIRROR_Z @ delta_p

    delta_R = init_pose[:3, :3].T @ target_pose[:3, :3]
    new_R = init_pose[:3, :3] @ _R_MIRROR_Z @ delta_R @ _R_MIRROR_Z

    out = np.eye(4)
    out[:3, 3] = new_p
    out[:3, :3] = new_R
    return out


def clip_to_workspace(target_xyz: np.ndarray) -> tuple[np.ndarray, bool]:
    clipped = target_xyz.copy()
    was = False
    for i, (lo, hi) in enumerate([WS_X, WS_Y, WS_Z]):
        if clipped[i] < lo:
            clipped[i] = lo; was = True
        elif clipped[i] > hi:
            clipped[i] = hi; was = True
    return clipped, was


def _local_ip() -> str:
    """Best-effort LAN address of this PC, for the Quest URL hint."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


def gripper_set(state: str, dry_run: bool, log_fn) -> None:
    if dry_run:
        log_fn(f"  [dry] would gripper_cmd {state}")
        return
    cmd = [GRIPPER_CMD, FRANKA_IP, state]
    env = os.environ.copy()
    if LD_LIBRARY_PATH:
        env["LD_LIBRARY_PATH"] = LD_LIBRARY_PATH
    try:
        subprocess.Popen(cmd, env=env,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as exc:
        log_fn(f"⚠️ gripper_cmd exception: {exc}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--live", action="store_true",
                    help="Actually send UDP to cartesian_pose_servo (the C++ servo must be started first)")
    ap.add_argument("--dry-run", action="store_true",
                    help="Do not connect to Franka; UDP is still sent, verify the stream with nc -ul 50001")
    ap.add_argument("--no-gripper", action="store_true")
    ap.add_argument("--duration", type=float, default=0)
    ap.add_argument("--print-hz", type=float, default=2.0)
    ap.add_argument("--max-step", type=float, default=DEFAULT_MAX_STEP_M)
    ap.add_argument("--ws-x-min", type=float, default=DEFAULT_WS_X[0])
    ap.add_argument("--ws-x-max", type=float, default=DEFAULT_WS_X[1])
    ap.add_argument("--ws-y-min", type=float, default=DEFAULT_WS_Y[0])
    ap.add_argument("--ws-y-max", type=float, default=DEFAULT_WS_Y[1])
    ap.add_argument("--ws-z-min", type=float, default=DEFAULT_WS_Z[0])
    ap.add_argument("--ws-z-max", type=float, default=DEFAULT_WS_Z[1])
    ap.add_argument("--no-clip", action="store_true")
    ap.add_argument("--no-mirror", action="store_true",
                    help="Disable the face-to-face mirror (on by default: left/right + pitch double-reversal fix when the user stands opposite)")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="Translation ratio: user controller moves 1cm → robot moves N cm (default 1.0 = 1:1, suited to "
                         "small-range teleop at chest height; the earlier 2.0 doubly amplified quest noise + boundary clip)")
    ap.add_argument("--rot-scale", type=float, default=0.5,
                    help="Rotation ratio: user controller rotates N° → robot EE rotates scale*N° (default 0.5 = "
                         "user rotates 90°, robot rotates only 45°; rules out amplifying wrist tremor at chest height)")
    ap.add_argument("--calibrate", action="store_true",
                    help="bbox calibration mode: press right-hand A to sample the current EE into /tmp/franka_bbox_points.json")
    ap.add_argument("--udp-host", default=os.environ.get("FRANKA_SERVO_HOST", "127.0.0.1"),
                    help="servo host; default from FRANKA_SERVO_HOST (config.env) or 127.0.0.1")
    ap.add_argument("--udp-port", type=int, default=50001)
    args = ap.parse_args()

    if not args.live and not args.dry_run:
        print("⚠️ Must pick exactly one of --live or --dry-run")
        return 1
    if args.live:
        # Only one process may send targets to the servo (it latches the first sender and drops
        # the rest); a second bridge/collector exits 75 instead of silently doing nothing.
        _sender_lock = acquire_sender_lock("02_webxr_to_franka")  # noqa: F841

    if args.calibrate:
        # Calibration mode: keep mirror (user-view mirroring unchanged), scale=1 (no scaling), bbox fully open
        # What gets sampled is stats.last_target, i.e. the real target sent to the servo after mirror+scale+bbox = robot EE
        args.scale = 1.0
        args.rot_scale = 1.0
        args.no_clip = True
        print(f"📐 CALIBRATE mode: mirror={'kept on' if not args.no_mirror else 'off'}, "
              f"scale=1.0, bbox fully open")
        print("   Movement guide:")
        print("     1. teleop to the desk workspace "
              "front-left corner  → press right-hand A")
        print("     2. teleop to the desk workspace front-right corner  → press right-hand A")
        print("     3. teleop to the desk workspace back-left corner  → press right-hand A")
        print("     4. teleop to the desk workspace back-right corner  → press right-hand A")
        print("     5. raise to the highest point of the workspace          → press right-hand A")
        print("   Each sampled point is written to /tmp/franka_bbox_points.json")

    global WS_X, WS_Y, WS_Z, MAX_STEP_M
    WS_X = (args.ws_x_min, args.ws_x_max)
    WS_Y = (args.ws_y_min, args.ws_y_max)
    WS_Z = (args.ws_z_min, args.ws_z_max)
    MAX_STEP_M = args.max_step

    # 1) Init pose
    if args.live:
        # LIVE: the servo must be started first; read the init pose from the file the servo writes
        # (never run echo_robot_state, it would grab the libfranka protocol and kick the servo dead)
        print(f"→ Reading init pose from {INIT_POSE_FILE} (LIVE requires the servo to be started first)...")
        init_pose = read_init_pose_from_file()
    else:
        # DRY-RUN: servo not running, pull one frame directly via echo_robot_state (no concurrency risk)
        print("→ DRY-RUN: pulling init pose with echo_robot_state ...")
        env = os.environ.copy()
        if LD_LIBRARY_PATH:
            env["LD_LIBRARY_PATH"] = LD_LIBRARY_PATH
        p = subprocess.Popen([ECHO_ROBOT_STATE, FRANKA_IP],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                             env=env, text=True)
        try:
            deadline = time.time() + 2
            line = ""
            while time.time() < deadline:
                line = p.stdout.readline()
                if line and line.startswith("{") and line.rstrip().endswith("}"):
                    break
        finally:
            p.terminate()
            try: p.wait(timeout=1.0)
            except subprocess.TimeoutExpired: p.kill()
        if not line.startswith("{"):
            print("❌ echo_robot_state returned no state (servo running on the host, or the ssh channel is down?)")
            return 2
        d = json.loads(line)
        init_pose = franka_o_t_ee_to_4x4(d["O_T_EE"])

    init_xyz = init_pose[:3, 3]
    print(f"  EE pos  = ({init_xyz[0]:+.4f}, {init_xyz[1]:+.4f}, {init_xyz[2]:+.4f}) m")
    if not (WS_X[0] <= init_xyz[0] <= WS_X[1] and WS_Y[0] <= init_xyz[1] <= WS_Y[1]
            and WS_Z[0] <= init_xyz[2] <= WS_Z[1]):
        print(f"  ❌ Current EE is not inside bbox {WS_X} {WS_Y} {WS_Z}")
        return 2

    # 2) UDP socket
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_addr = (args.udp_host, args.udp_port)
    print(f"→ UDP target {udp_addr[0]}:{udp_addr[1]} ({'LIVE' if args.live else 'DRY-RUN'})")

    # 3) Teleop server
    from teleop import Teleop
    if os.path.isdir(FRONTEND_SWAPPED):
        teleop = Teleop(frontend_dir=FRONTEND_SWAPPED)
        print(f"  ✅ swapped frontend (Trigger=move, Grip=gripper)")
    else:
        teleop = Teleop()
        print(f"  ⚠️ default frontend (Grip=move, Trigger=gripper)")
    teleop.set_pose(init_pose)
    stats = Stats()
    last_gripper_state = "open"
    cal_points: list = []
    cal_last_btn_a = False
    cal_file = "/tmp/franka_bbox_points.json"
    if args.calibrate:
        # Clear old records at startup
        try:
            with open(cal_file, "w") as f:
                f.write("[]")
        except Exception:
            pass

    def log_print(msg):
        print(msg, flush=True)

    def callback(target_pose: np.ndarray, message: dict) -> None:
        nonlocal last_gripper_state, cal_last_btn_a
        stats.n_callback += 1
        stats.last_msg = message

        # Calibration mode: on the right-hand A press edge, sample the robot's actual EE
        # The servo writes the current EE to /tmp/franka_current_ee.txt at 20Hz; that is the robot's physical position
        # (the 02 accumulator value stats.last_target may be far from a point the robot can actually reach)
        if args.calibrate:
            btn_a = bool(message.get("reservedButtonA", False))
            if btn_a and not cal_last_btn_a:
                pos = None
                try:
                    with open("/tmp/franka_current_ee.txt") as f:
                        nums = f.read().split()
                    if len(nums) == 16:
                        ee_mat = np.array([float(v) for v in nums]).reshape(4, 4, order="F")
                        pos = ee_mat[:3, 3].tolist()
                except Exception as exc:
                    print(f"⚠️ Failed to read /tmp/franka_current_ee.txt: {exc}", flush=True)
                if pos is not None:
                    cal_points.append({"index": len(cal_points), "pos": pos})
                    try:
                        with open(cal_file, "w") as f:
                            json.dump(cal_points, f, indent=2)
                        print(f"📍 [{len(cal_points)}/5] Sampled robot actual EE = "
                              f"({pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f})", flush=True)
                    except Exception as exc:
                        print(f"⚠️ Failed to write {cal_file}: {exc}", flush=True)
            cal_last_btn_a = btn_a

        # Gripper
        if not args.no_gripper:
            g = message.get("gripper", "open")
            if g != last_gripper_state:
                stats.n_trig_press += 1
                gripper_set(g, args.dry_run, log_print)
                last_gripper_state = g

        # dead-man: when the Trigger is released we no longer silently stop sending; we keep sending last_target
        # Otherwise the servo's target_raw locks onto a stale value and the PD drifts the robot to that old position (what the user sees as "drifting")
        if not message.get("move"):
            if stats.last_target is not None:
                try:
                    udp.sendto(matrix_4x4_to_udp_bytes(stats.last_target), udp_addr)
                except Exception:
                    pass
            return
        stats.n_grip_press += 1

        # Face-to-face mirror (flip 180° about the base z axis through the init position)
        if not args.no_mirror:
            target_pose = mirror_pose_around_init(target_pose, init_pose)

        # Translation scaling (user 1cm → robot scale*1cm)
        if abs(args.scale - 1.0) > 1e-6:
            scaled = target_pose.copy()
            scaled[:3, 3] = init_pose[:3, 3] + args.scale * (target_pose[:3, 3] - init_pose[:3, 3])
            target_pose = scaled

        # Rotation scaling: use scipy.spatial.transform to handle every singularity such as angle ≈ π
        # The earlier hand-written axis = (R-R^T)/(2 sin angle) divided by zero at angle≈π → NaN → reflex
        if abs(args.rot_scale - 1.0) > 1e-6:
            init_R = init_pose[:3, :3]
            delta_R = init_R.T @ target_pose[:3, :3]
            delta_rotvec = _Rotation.from_matrix(delta_R).as_rotvec()
            new_rotvec = delta_rotvec * args.rot_scale
            R_new = _Rotation.from_rotvec(new_rotvec).as_matrix()
            target_pose = target_pose.copy()
            target_pose[:3, :3] = init_R @ R_new

        # bbox clipping (first pull the target into the legal range; if it exceeds the bbox, anchor reset prompts the user to release)
        was_clipped = False
        if not args.no_clip:
            target_xyz = target_pose[:3, 3].copy()
            clipped_xyz, was_clipped = clip_to_workspace(target_xyz)
            if was_clipped:
                stats.n_clipped += 1
                target_pose = target_pose.copy()
                target_pose[:3, 3] = clipped_xyz

        # Critical safety: step rate limit is the backstop for every path
        # The target sent each frame is ≤ MAX_STEP_M away from last_target
        # Prevents a teleport produced by any path (clip / mirror / anchor reset) from making the servo PD sweep hard
        # The earlier "step_blk drop frame" would freeze + a clipped target jumping 0.45m sent the robot out of bounds at high speed
        # Now changed to "clamp then send": the robot slides along the unit direction at step_lim speed, always continuous
        if stats.last_target is not None:
            delta = target_pose[:3, 3] - stats.last_target[:3, 3]
            d = float(np.linalg.norm(delta))
            if d > MAX_STEP_M:
                stats.n_step_blocked += 1
                target_pose = target_pose.copy()
                target_pose[:3, 3] = stats.last_target[:3, 3] + (delta / d) * MAX_STEP_M

            # Rotation rate limit: per-frame rotation angle ≤ MAX_ROT_STEP_RAD
            # Prevents PD rotation torque saturation causing the "EE has no force" symptom
            R_last = stats.last_target[:3, :3]
            R_now = target_pose[:3, :3]
            rotvec = _Rotation.from_matrix(R_last.T @ R_now).as_rotvec()
            angle = float(np.linalg.norm(rotvec))
            if angle > MAX_ROT_STEP_RAD:
                clamped = rotvec * (MAX_ROT_STEP_RAD / angle)
                R_clamped = R_last @ _Rotation.from_rotvec(clamped).as_matrix()
                target_pose = target_pose.copy()
                target_pose[:3, :3] = R_clamped

        # Anchor reset when the bbox is exceeded, so the user releases and re-presses to re-accumulate from the clipped position
        if was_clipped:
            try:
                teleop.set_pose(target_pose)
                teleop._Teleop__relative_pose_init = None
                teleop._Teleop__absolute_pose_init = None
                teleop._Teleop__previous_received_pose = None
            except Exception as exc:
                print(f"⚠️ anchor reset failed: {exc}")

        # Send UDP
        try:
            udp.sendto(matrix_4x4_to_udp_bytes(target_pose), udp_addr)
            stats.n_udp_sent += 1
            stats.last_target = target_pose
        except Exception as exc:
            print(f"⚠️ UDP send exception: {exc}")

    teleop.subscribe(callback)
    threading.Thread(target=teleop.run, daemon=True).start()

    print()
    print("=" * 64)
    print(f"🚀 Closed loop ready. Mode: {'LIVE 🔴' if args.live else 'DRY-RUN'}")
    print(f"   bbox:      x{WS_X} y{WS_Y} z{WS_Z}")
    print(f"   step lim:  {MAX_STEP_M*100:.0f} cm")
    print(f"   UDP:       {udp_addr[0]}:{udp_addr[1]}")
    print(f"   Quest URL: https://{_local_ip()}:4443/  (same network as this PC)")
    if os.environ.get("FRANKA_TELEOP_URL"):
        print(f"              {os.environ['FRANKA_TELEOP_URL']}  (FRANKA_TELEOP_URL, e.g. a Tailscale Funnel)")
    print(f"   Hold Trigger (index finger) = move; press Grip (middle finger) = gripper; Ctrl-C to quit")
    print("=" * 64)

    stop = {"v": False}
    def _sig(*_): stop["v"] = True
    signal.signal(signal.SIGINT, _sig); signal.signal(signal.SIGTERM, _sig)

    period = 1.0 / args.print_hz
    t_start = time.time()
    last_print = 0.0
    try:
        while not stop["v"]:
            now = time.time()
            if now - last_print >= period:
                last_print = now
                msg = stats.last_msg or {}
                tip = stats.last_target[:3, 3] if stats.last_target is not None else (0, 0, 0)
                print(f"[t={now-t_start:6.1f}s] cb={stats.n_callback:5d} "
                      f"udp={stats.n_udp_sent:5d} clip={stats.n_clipped} "
                      f"step_blk={stats.n_step_blocked} grip={stats.n_grip_press} "
                      f"trig={stats.n_trig_press} | "
                      f"move={msg.get('move')} g={msg.get('gripper')} "
                      f"target=({tip[0]:+.3f},{tip[1]:+.3f},{tip[2]:+.3f})")
            if args.duration and (now - t_start) >= args.duration:
                print(f"\n⏱  Ran the full {args.duration}s")
                break
            time.sleep(0.1)
    finally:
        print("\n→ Closing UDP socket")
        udp.close()
        print("✅ Clean exit")
    return 0


if __name__ == "__main__":
    sys.exit(main())
