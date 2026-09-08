#!/usr/bin/env python3
"""state_sink.py - receive FRST1 state datagrams, mirror bodies atomically, detect gaps.
Exit 0 iff: no inter-arrival gap above threshold for any name, no bad headers/bodies, no seq LOSS
(reordered datagrams are tolerated; only holes never filled count), and (unless --allow-epoch-change) no epoch change.
"""
import argparse, json, os, signal, socket, sys, time

EXPECT = {"franka_init_pose.txt": 16, "franka_current_ee.txt": 16, "franka_wrench.txt": 6,
          "franka_joint_state.txt": 28, "franka_link.txt": "kv"}
SHORT = {"franka_init_pose.txt": "init", "franka_current_ee.txt": "ee", "franka_wrench.txt": "wrench",
         "franka_joint_state.txt": "joint", "franka_link.txt": "link"}
LINK_KEYS = ["cmd_age_ms", "cmd_pkts_last_s", "cmd_drop_size", "cmd_drop_allow", "cmd_drop_latch",
             "latched_sender", "missed_cycles_total", "max_consecutive_missed", "freeze", "recovering",
             "tick_over_1p2ms_1s", "tick_max_us_1s", "reflex_count"]

def parse_link(body):
    d = {}
    for tok in body.split():
        if "=" in tok:
            k, v = tok.split("=", 1); d[k] = v
    return d

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bind", default="0.0.0.0"); ap.add_argument("--port", type=int, default=50002)
    ap.add_argument("--mirror", default="/tmp/franka_mirror")
    ap.add_argument("--duration", type=float, default=0, help="0 = until SIGINT/SIGTERM")
    ap.add_argument("--gap-ee-ms", type=float, default=250); ap.add_argument("--gap-joint-ms", type=float, default=500)
    ap.add_argument("--gap-wrench-ms", type=float, default=250); ap.add_argument("--gap-link-ms", type=float, default=1000)
    ap.add_argument("--allow-epoch-change", action="store_true")
    ap.add_argument("--report", default=""); ap.add_argument("--record-ee", default="")
    ap.add_argument("--gap-log", default="", help="CSV of every gap > 100 ms")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()
    thr = {"franka_current_ee.txt": a.gap_ee_ms, "franka_joint_state.txt": a.gap_joint_ms,
           "franka_wrench.txt": a.gap_wrench_ms, "franka_link.txt": a.gap_link_ms, "franka_init_pose.txt": None}
    os.makedirs(a.mirror, exist_ok=True)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    sock.bind((a.bind, a.port)); sock.settimeout(0.2)
    rec = open(a.record_ee, "w") if a.record_ee else None
    if rec: rec.write("t_real,x,y,z\n")
    glog = open(a.gap_log, "w") if a.gap_log else None
    if glog: glog.write("t_real,name,gap_ms\n")

    st = {}   # name -> stats
    def S(name):
        if name not in st:
            st[name] = dict(count=0, last_rx=None, max_gap_ms=0.0, over_thr=0,
                            body_bad=0, rate_count=0, max_gap_1s=0.0)
        return st[name]
    tot = dict(rx=0, hdr_bad=0, unknown=0, epoch_changes=0, epochs=[],
               seq_first=None, seq_max=None, seq_rx=0, seq_reorders=0, seq_loss=0)
    def seq_finalize(t):   # holes never filled within one process epoch = lost datagrams
        if t["seq_first"] is not None:
            t["seq_loss"] += (t["seq_max"] - t["seq_first"] + 1) - t["seq_rx"]
        t["seq_first"] = None; t["seq_max"] = None; t["seq_rx"] = 0
    last_link = {}
    stop = False
    def on_sig(*_):
        nonlocal stop; stop = True
    signal.signal(signal.SIGINT, on_sig); signal.signal(signal.SIGTERM, on_sig)

    t0 = time.monotonic(); next_print = t0 + 1.0
    while not stop:
        now = time.monotonic()
        if a.duration and now - t0 >= a.duration: break
        try:
            data, addr = sock.recvfrom(65535)
        except socket.timeout:
            data = None
        if data is not None:
            rx = time.monotonic(); tot["rx"] += 1
            nl = data.find(b"\n")
            hdr = data[:nl].decode("ascii", "replace").split() if nl > 0 else []
            body = data[nl + 1:].decode("ascii", "replace") if nl > 0 else ""
            ok = len(hdr) == 6 and hdr[0] == "FRST1"
            if ok:
                try:
                    seq, epoch, t_real_ns, t_mono_ns = int(hdr[1]), int(hdr[2]), int(hdr[3]), int(hdr[4])
                except ValueError:
                    ok = False
            if not ok:
                tot["hdr_bad"] += 1; continue
            name = hdr[5]
            if name not in EXPECT:
                tot["unknown"] += 1; continue
            s = S(name); s["count"] += 1; s["rate_count"] += 1
            if tot["epochs"] and epoch != tot["epochs"][-1]:
                tot["epoch_changes"] += 1
                seq_finalize(tot)   # new process: seq restarts
                print(f"[sink] EPOCH CHANGE {tot['epochs'][-1]} -> {epoch} (name={name})", flush=True)
            if not tot["epochs"] or epoch != tot["epochs"][-1]: tot["epochs"].append(epoch)
            # seq is a per-PROCESS counter across all names (PART B contract). The network may
            # reorder datagrams, so reorders are only counted; holes never filled = seq_loss.
            if tot["seq_first"] is None: tot["seq_first"] = seq
            if tot["seq_max"] is not None and seq < tot["seq_max"]: tot["seq_reorders"] += 1
            elif tot["seq_max"] is None or seq > tot["seq_max"]: tot["seq_max"] = seq
            tot["seq_rx"] += 1
            if s["last_rx"] is not None:
                gap = (rx - s["last_rx"]) * 1e3
                s["max_gap_ms"] = max(s["max_gap_ms"], gap); s["max_gap_1s"] = max(s["max_gap_1s"], gap)
                if thr[name] is not None and gap > thr[name]:
                    s["over_thr"] += 1; print(f"[sink] GAP {SHORT[name]} {gap:.0f} ms > {thr[name]:.0f} ms", flush=True)
                if glog and gap > 100: glog.write(f"{time.time():.6f},{name},{gap:.1f}\n")
            s["last_rx"] = rx
            toks = body.split(); exp = EXPECT[name]
            if exp == "kv":
                good = [t.split("=", 1)[0] for t in toks] == LINK_KEYS
                if good: last_link = parse_link(body)
            else:
                good = len(toks) == exp
                if good:
                    try: [float(t) for t in toks]
                    except ValueError: good = False
            if not good:
                s["body_bad"] += 1; print(f"[sink] BAD BODY {name}: {len(toks)} tokens", flush=True); continue
            if rec and name == "franka_current_ee.txt":
                rec.write(f"{t_real_ns/1e9:.6f},{toks[12]},{toks[13]},{toks[14]}\n")
            tmp = os.path.join(a.mirror, name + ".tmp"); dst = os.path.join(a.mirror, name)
            with open(tmp, "w") as f: f.write(body if body.endswith("\n") else body + "\n")
            os.replace(tmp, dst)
        if now >= next_print:
            next_print += 1.0
            parts = []
            for name in EXPECT:
                if name in st:
                    s = st[name]; parts.append(f"{SHORT[name]} {s['rate_count']}/s g{s['max_gap_1s']:.0f}")
                    s["rate_count"] = 0; s["max_gap_1s"] = 0.0
            lk = " ".join(f"{k}={last_link.get(k,'?')}" for k in
                          ("cmd_age_ms", "cmd_pkts_last_s", "latched_sender", "missed_cycles_total", "freeze", "reflex_count"))
            if not a.quiet: print(f"[sink] t={now-t0:5.0f}s | " + " | ".join(parts) + f" | {lk}", flush=True)
    seq_finalize(tot)
    fail = False
    if tot["rx"] == 0: fail = True
    missing_names = [n for n in ("franka_current_ee.txt", "franka_wrench.txt", "franka_joint_state.txt", "franka_link.txt")
                      if n not in st]
    if missing_names: fail = True
    fail = fail or tot["hdr_bad"] > 0 or tot["unknown"] > 0 or tot["seq_loss"] > 0
    if tot["epoch_changes"] > 0 and not a.allow_epoch_change: fail = True
    for name, s in st.items():
        if s["over_thr"] > 0 or s["body_bad"] > 0: fail = True
    rep = dict(bind=f"{a.bind}:{a.port}", duration_s=round(time.monotonic() - t0, 1), totals=tot,
               per_name={SHORT[n]: {k: v for k, v in s.items() if k not in ("last_rx", "rate_count", "max_gap_1s")}
                         for n, s in st.items()},
               thresholds_ms={SHORT[n]: v for n, v in thr.items()}, last_link=last_link, missing_names=missing_names,
               result="PASS" if not fail else "FAIL")
    print("[sink] FINAL " + json.dumps(rep, indent=1), flush=True)
    if a.report:
        with open(a.report, "w") as f: json.dump(rep, f, indent=1)
    if rec: rec.close()
    if glog: glog.close()
    return 0 if not fail else 1

if __name__ == "__main__":
    sys.exit(main())
