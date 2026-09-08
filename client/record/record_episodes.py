#!/usr/bin/env python3
"""Task-agnostic episode recorder: cameras + robot state from the mirror -> one HDF5 per episode.

Keyboard control in the terminal (must run in the FOREGROUND, it reads stdin):
  s = start an episode
  e = end and save the current episode
  d = discard the episode saved last
  q = quit (an episode still recording is discarded)

The teleop bridge (bin/franka-teleop live) is started separately by the operator; this script
only observes: RealSense frames through record/cameras/recorder.py (RECORD profile) and the
robot state polled at --poll-hz from the files written by bin/franka_state_mirror
(record/state_reader.py, 250 ms staleness rule -> ee_ok). Nothing here sends commands.

Each episode is written in a background thread (gzip level 1) so the next `s` is immediate;
`d` removes the file, or marks it for removal if it is still being written.

Usage:
    python3 record/record_episodes.py --out-dir ~/datasets/my_task --prefix my_task
    python3 record/record_episodes.py --auto-end-secs 30 --cams wrist_d455 --with-wrench

Dataset layout: record/README.md.
"""
from __future__ import annotations

import argparse
import os
import queue
import re
import select
import socket
import sys
import termios
import threading
import time
import tty
from pathlib import Path

_HERE = Path(__file__).resolve().parent            # <repo>/client/record
_CLIENT = _HERE.parent                             # <repo>/client
sys.path.insert(0, str(_CLIENT))
sys.path.insert(0, str(_HERE))

from record.cameras.grabber import load_camera_config   # noqa: E402
from record.cameras.recorder import MultiCamRecorder   # noqa: E402
from state_reader import make_state_fn                 # noqa: E402


def _next_episode_index(out_dir: Path, prefix: str) -> int:
    """Continue numbering after the highest <prefix>_epNN.h5 already in out_dir."""
    pat = re.compile(re.escape(prefix) + r"_ep(\d+)\.h5$")
    idx = [int(m.group(1)) for p in out_dir.glob(f"{prefix}_ep*.h5") if (m := pat.search(p.name))]
    return (max(idx) + 1) if idx else 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out-dir", default=str(Path.home() / "franka_episodes"))
    ap.add_argument("--prefix", default="episode", help="file name prefix: <prefix>_epNN.h5")
    ap.add_argument("--config", default=str(_HERE / "config" / "cameras.yaml"))
    ap.add_argument("--cams", default=None,
                    help="comma-separated roles to record (default: every role in the config)")
    ap.add_argument("--poll-hz", type=float, default=20.0, help="robot-state poll rate (the EE file is 20 Hz)")
    ap.add_argument("--state-dir", default="/tmp", help="directory the state mirror writes to")
    ap.add_argument("--with-wrench", action="store_true", help="also record franka_wrench.txt (6)")
    ap.add_argument("--with-joints", action="store_true", help="also record franka_joint_state.txt (28)")
    ap.add_argument("--auto-end-secs", type=float, default=0.0,
                    help="end and save automatically N seconds after `s` (0 = manual `e`)")
    ap.add_argument("--exposure", action="append", default=[], metavar="ROLE=US",
                    help="lock a role's exposure manually instead of the converged AE value, e.g. wrist_d455=25")
    ap.add_argument("--allow-dead-state", action="store_true",
                    help="start even if the EE path is dead (camera-only recording)")
    args = ap.parse_args()

    if not sys.stdin.isatty():
        print("❌ run in a terminal foreground (keyboard s/e/d/q); not under nohup or in the background")
        return 2

    # --- robot state --------------------------------------------------------------------------
    sfn = make_state_fn(args.state_dir, include_wrench=args.with_wrench, include_joints=args.with_joints)
    st = sfn()
    if not st["ee_ok"]:
        print(f"⚠️ EE path dead ({args.state_dir}/franka_current_ee.txt missing or stale). "
              "Is the servo running and franka-state-mirror active? ee_pose will be zeros with ee_ok=False.")
        if not args.allow_dead_state:
            print("   pass --allow-dead-state for camera-only recording")
            return 1
    else:
        ee = st["ee_pose"]
        print(f"[preflight] EE path alive: pos=({ee[12]:.3f},{ee[13]:.3f},{ee[14]:.3f})")

    # --- cameras ------------------------------------------------------------------------------
    cfg = load_camera_config(Path(args.config))
    roles = [r.strip() for r in args.cams.split(",")] if args.cams else list(cfg["cameras"].keys())
    cams = []
    for r in roles:
        if r not in cfg["cameras"]:
            print(f"❌ role '{r}' not in {args.config}; have {list(cfg['cameras'])}")
            return 2
        cams.append((r, str(cfg["cameras"][r]["serial"])))
    d = cfg.get("defaults", {})
    rec = MultiCamRecorder(cams, width=d.get("width", 640), height=d.get("height", 480), fps=d.get("fps", 30))
    started = rec.start()
    if not started:
        print("❌ no camera started (serials in record/config/cameras.yaml plugged in?)")
        return 1
    for spec in args.exposure:
        role, _, us = spec.partition("=")
        if role not in started:
            print(f"[exposure] ⚠️ role {role} not started, ignored")
            continue
        try:
            import pyrealsense2 as rs
            cam = rec._cams[role]  # noqa: SLF001
            cam.sensor.set_option(rs.option.enable_auto_exposure, 0)
            cam.sensor.set_option(rs.option.exposure, float(us))
            cam.locked["exposure"] = float(us)
            print(f"[exposure] {role} locked at {us} us")
        except Exception as exc:
            print(f"[exposure] ⚠️ {role}: {exc}")
    rec.attach_state_fn(sfn)
    print(f"[rec] cameras: {started}")

    out_dir = Path(args.out_dir).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    ep_idx = _next_episode_index(out_dir, args.prefix)
    if ep_idx:
        print(f"[rec] resuming numbering at ep{ep_idx:02d} ({out_dir})")
    root_attrs = {
        "poll_hz": args.poll_hz, "state_dir": args.state_dir, "ee_stale_ms": 250.0,
        "client_host": socket.gethostname(), "servo_host": os.environ.get("FRANKA_SERVO_HOST", ""),
        "config": str(Path(args.config).resolve()),
    }
    print("\n=== keys ===  s=start  e=end+save  d=discard last  q=quit\n")

    # --- background writer --------------------------------------------------------------------
    write_q: "queue.Queue" = queue.Queue()
    lock = threading.Lock()
    written: list[Path] = []
    discarded: set = set()

    def _writer():
        while True:
            job = write_q.get()
            if job is None:
                write_q.task_done()
                break
            fn, snap, attrs = job
            try:
                meta = rec.write_hdf5(str(fn), compression_opts=1, snapshot=snap, attrs=attrs)
                with lock:
                    if fn in discarded:
                        discarded.discard(fn)
                        try:
                            os.remove(fn)
                        except Exception:
                            pass
                        print(f"\n🗑️ {fn.name} discarded after write", flush=True)
                    else:
                        written.append(fn)
                        frames = {k: v[0] for k, v in meta.items() if k != "state_steps"}
                        print(f"\n💾 {fn.name} written: frames={frames} state={meta.get('state_steps')}", flush=True)
            except Exception as exc:
                print(f"\n⚠️ writing {fn.name} failed: {exc}", flush=True)
            write_q.task_done()

    worker = threading.Thread(target=_writer, daemon=True)
    worker.start()

    recording = False
    last_saved: Path | None = None
    saved = 0
    ep_start = 0.0
    poll_dt = 1.0 / args.poll_hz

    def save_current():
        nonlocal saved, last_saved
        rec.end_episode()
        fn = out_dir / f"{args.prefix}_ep{ep_idx:02d}.h5"
        sd = rec._state_buf  # noqa: SLF001
        eok = (sum(1 for s in sd if s.get("ee_ok")) / len(sd)) if sd else 0.0
        attrs = dict(root_attrs, episode=ep_idx, duration_s=time.time() - ep_start,
                     ee_ok_fraction=eok, t_start_s=ep_start, t_end_s=time.time())
        write_q.put((fn, rec.snapshot_episode(), attrs))
        last_saved = fn
        saved += 1
        print(f"⏹️ ep{ep_idx:02d} ended after {attrs['duration_s']:.1f}s, ee_ok={eok*100:.0f}% "
              f"(writing in background, queue {write_q.qsize()})", flush=True)
        if eok < 0.95:
            print("   ⚠️ ee_ok below 95 %: the robot state was stale for part of the episode", flush=True)

    def discard_last():
        nonlocal saved, last_saved
        if last_saved is None:
            print("⚠️ nothing to discard")
            return
        fn = last_saved
        with lock:
            if fn in written:
                written.remove(fn)
                try:
                    os.remove(fn)
                except Exception:
                    pass
                print(f"🗑️ {fn.name} deleted")
            else:
                discarded.add(fn)
                print(f"🗑️ {fn.name} will be deleted once written")
        last_saved = None
        saved = max(0, saved - 1)

    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            if recording:
                rec.poll_state()
                if args.auto_end_secs > 0 and (time.time() - ep_start) >= args.auto_end_secs:
                    save_current()
                    recording = False
                    ep_idx += 1
                    print(f"⏱️ {args.auto_end_secs:.0f}s reached, saved automatically; s = next episode, q = quit", flush=True)
                    continue
            r, _, _ = select.select([sys.stdin], [], [], poll_dt)
            if not r:
                continue
            c = sys.stdin.read(1).lower()
            if c == "s":
                if recording:
                    print("⚠️ already recording (e to end first)")
                    continue
                rec.begin_episode()
                recording = True
                ep_start = time.time()
                print(f"🔴 ep{ep_idx:02d} recording ..." +
                      (f" (auto end after {args.auto_end_secs:.0f}s)" if args.auto_end_secs > 0 else " (e = end and save)"))
            elif c == "e":
                if not recording:
                    print("⚠️ not recording (s to start)")
                    continue
                save_current()
                recording = False
                ep_idx += 1
                print("⏸️ idle: s = next episode, d = discard last, q = quit", flush=True)
            elif c == "d":
                if recording:
                    print("⚠️ recording; e to end first, then d")
                    continue
                discard_last()
            elif c == "q":
                if recording:
                    print("\n⚠️ quitting while recording: the current episode is NOT saved")
                    rec.end_episode()
                break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        rec.stop()
        qn = write_q.qsize()
        if qn:
            print(f"\n⏳ waiting for {qn} episode(s) still being written ...", flush=True)
        write_q.put(None)
        worker.join()
    print(f"\n=== done: {saved} episode(s) saved in {out_dir} ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
