#!/usr/bin/env python3
"""fake_sender.py - send 128-byte 4x4 col-major pose targets like the WebXR bridge/deploy do.

Modes: constant | circle (XY plane, fixed z, top-down R_DOWN) | burst (on/off windows -> exercises 200 ms hold)
Examples:
  ./fake_sender.py --mode constant --duration 10
  ./fake_sender.py --mode circle --radius 0.03 --period 20 --duration 600 --center-from /tmp/franka_mirror/franka_init_pose.txt
  ./fake_sender.py --mode burst --burst-on 3 --burst-off 2 --cycles 2
  ./fake_sender.py --src-ip 127.0.0.2 --src-port 40003 --duration 5      # non-allowlisted source
"""
import argparse, math, os, signal, socket, struct, sys, time
import numpy as np

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])

def pack_pose(R, p):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p
    return struct.pack("16d", *T.flatten(order="F"))   # 128 bytes, col-major like rog senders

def read_pose_file(path):
    toks = open(path).read().split()
    if len(toks) != 16:
        raise SystemExit(f"{path}: expected 16 tokens, got {len(toks)}")
    T = np.array([float(t) for t in toks]).reshape(4, 4, order="F")
    return T[:3, :3].copy(), T[:3, 3].copy()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dst", default="127.0.0.1:50001")
    ap.add_argument("--src-ip", default="")
    ap.add_argument("--src-port", type=int, default=0)
    ap.add_argument("--rate", type=float, default=90.0)
    ap.add_argument("--duration", type=float, default=10.0, help="seconds (burst mode ignores; uses cycles)")
    ap.add_argument("--mode", choices=["constant", "circle", "burst"], default="constant")
    ap.add_argument("--x", type=float, default=0.50); ap.add_argument("--y", type=float, default=0.00)
    ap.add_argument("--z", type=float, default=0.098)
    ap.add_argument("--radius", type=float, default=0.03); ap.add_argument("--period", type=float, default=20.0)
    ap.add_argument("--burst-on", type=float, default=3.0); ap.add_argument("--burst-off", type=float, default=2.0)
    ap.add_argument("--cycles", type=int, default=2)
    ap.add_argument("--center-from", default="", help="16-token pose file; circle passes through it at t=0, R taken from it")
    ap.add_argument("--force-down", action="store_true", help="use R_DOWN even with --center-from")
    ap.add_argument("--log", default="")
    ap.add_argument("--print-every", type=float, default=1.0)
    a = ap.parse_args()

    host, port = a.dst.rsplit(":", 1); port = int(port)
    R = R_DOWN; x0, y0, z0 = a.x, a.y, a.z
    if a.center_from:
        Rf, pf = read_pose_file(a.center_from)
        x0, y0, z0 = float(pf[0]), float(pf[1]), float(pf[2])
        if not a.force_down: R = Rf
    cx, cy = (x0 - a.radius, y0) if a.mode == "circle" else (x0, y0)   # circle starts exactly at (x0,y0)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if a.src_ip or a.src_port:
        sock.bind((a.src_ip or "0.0.0.0", a.src_port))
    src = sock.getsockname()
    log = open(a.log, "w") if a.log else None
    if log: log.write("t_real,x,y,z\n")

    stop = False
    def on_sig(*_):
        nonlocal stop; stop = True
    signal.signal(signal.SIGINT, on_sig); signal.signal(signal.SIGTERM, on_sig)

    dt = 1.0 / a.rate
    total = a.duration if a.mode != "burst" else a.cycles * (a.burst_on + a.burst_off)
    t_start = time.perf_counter(); next_t = t_start
    sent = errors = 0; max_late = 0.0; last_print = t_start; sent_at_print = 0
    print(f"[fake_sender] {src[0]}:{src[1]} -> {host}:{port} mode={a.mode} rate={a.rate}Hz "
          f"start=({x0:.4f},{y0:.4f},{z0:.4f}) r={a.radius} T={a.period} total={total:.1f}s", flush=True)
    while not stop:
        now = time.perf_counter(); t = now - t_start
        if t >= total: break
        if a.mode == "circle":
            w = 2 * math.pi * t / a.period
            p = (cx + a.radius * math.cos(w), cy + a.radius * math.sin(w), z0)
        else:
            p = (x0, y0, z0)
        send = True
        if a.mode == "burst":
            phase = t % (a.burst_on + a.burst_off)
            send = phase < a.burst_on
        if send:
            try:
                sock.sendto(pack_pose(R, p), (host, port)); sent += 1
                if log: log.write(f"{time.time():.6f},{p[0]:.6f},{p[1]:.6f},{p[2]:.6f}\n")
            except OSError as e:
                errors += 1
                if errors <= 3: print(f"[fake_sender] sendto error: {e}", flush=True)
        if now - last_print >= a.print_every:
            rate = (sent - sent_at_print) / (now - last_print)
            print(f"[fake_sender] t={t:6.1f}s sent={sent} rate={rate:5.1f}/s max_late={max_late*1e3:.2f}ms "
                  f"errors={errors} pos=({p[0]:.4f},{p[1]:.4f},{p[2]:.4f}) {'ON' if send else 'OFF'}", flush=True)
            last_print = now; sent_at_print = sent
        next_t += dt
        late = time.perf_counter() - next_t
        if late > max_late: max_late = late
        if late < 0: time.sleep(-late)
    el = time.perf_counter() - t_start
    print(f"[fake_sender] DONE sent={sent} elapsed={el:.2f}s avg_rate={sent/el if el else 0:.2f}/s "
          f"max_late={max_late*1e3:.2f}ms errors={errors}", flush=True)
    if log: log.close()
    return 0 if errors == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
