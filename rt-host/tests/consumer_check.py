#!/usr/bin/env python3
"""consumer_check.py - emulate rog consumers on the mirror dir.
A  03 bridge start-up:   franka_init_pose.txt appears with 16 tokens within --init-timeout (5 s)
B  client state reader:   20 Hz poll for --secs: age<=250 ms AND exactly 16 tokens  -> ee_ok ratio >= 0.995, token errors 0
C  deploy watchdog:       10 Hz poll: age>0.5 s streak; PASS if max streak < 5 and max age < 0.5 s
D  03 dead-man:           90 Hz reads of the EE file for --secs: 0 parse failures (16 float tokens)
E  joint/wrench:          28/6 tokens, age <= 0.5 s / 0.25 s at end (WARN only)
F  --bounds:              SafetyMonitor: z <= 0.20 m and hypot(x,y) <= 0.85 m on every EE sample (robot day only)
"""
import argparse, json, os, sys, time

def age(p):
    try: return time.time() - os.path.getmtime(p)
    except OSError: return float("inf")

def toks(p):
    try:
        with open(p) as f: return f.read().split()
    except OSError: return None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mirror", default="/tmp/franka_mirror"); ap.add_argument("--secs", type=float, default=20.0)
    ap.add_argument("--init-timeout", type=float, default=5.0); ap.add_argument("--bounds", action="store_true")
    ap.add_argument("--report", default="")
    a = ap.parse_args()
    EE = os.path.join(a.mirror, "franka_current_ee.txt"); INIT = os.path.join(a.mirror, "franka_init_pose.txt")
    JOINT = os.path.join(a.mirror, "franka_joint_state.txt"); WR = os.path.join(a.mirror, "franka_wrench.txt")
    res = {}
    t0 = time.time(); ok = False
    while time.time() - t0 < a.init_timeout:
        t = toks(INIT)
        if t is not None and len(t) == 16: ok = True; break
        time.sleep(0.1)
    res["A_init_pose"] = dict(pass_=ok, waited_s=round(time.time() - t0, 2))
    n = okc = tokerr = 0; c_streak = c_max_streak = 0; c_max_age = 0.0; d_fail = d_n = 0; f_viol = 0
    t0 = time.time(); nb = nc = nd = t0
    while time.time() - t0 < a.secs:
        now = time.time()
        if now >= nb:                       # B @20 Hz
            nb += 0.05; n += 1; ag = age(EE); t = toks(EE)
            if ag <= 0.25 and t is not None and len(t) == 16: okc += 1
            elif t is not None and len(t) != 16: tokerr += 1
        if now >= nc:                       # C @10 Hz
            nc += 0.1; ag = age(EE); c_max_age = max(c_max_age, ag)
            if ag > 0.5: c_streak += 1; c_max_streak = max(c_max_streak, c_streak)
            else: c_streak = 0
        if now >= nd:                       # D @90 Hz
            nd += 1 / 90; d_n += 1; t = toks(EE)
            try:
                v = [float(x) for x in t] if t is not None else None
                if v is None or len(v) != 16: d_fail += 1
                elif a.bounds and (v[14] > 0.20 or (v[12] ** 2 + v[13] ** 2) ** 0.5 > 0.85): f_viol += 1
            except ValueError: d_fail += 1
        time.sleep(0.002)
    ratio = okc / n if n else 0.0
    res["B_state_fn"] = dict(pass_=(ratio >= 0.995 and tokerr == 0), ee_ok=okc, samples=n, ratio=round(ratio, 4), token_errors=tokerr)
    res["C_deploy_watchdog"] = dict(pass_=(c_max_streak < 5 and c_max_age < 0.5), max_streak=c_max_streak, max_age_s=round(c_max_age, 3))
    res["D_deadman_reads"] = dict(pass_=(d_fail == 0), reads=d_n, parse_failures=d_fail)
    jt, wt = toks(JOINT), toks(WR)
    res["E_joint_wrench"] = dict(warn=not (jt and len(jt) == 28 and age(JOINT) <= 0.5 and wt and len(wt) == 6 and age(WR) <= 0.25),
                                 joint_tokens=len(jt) if jt else None, joint_age_s=round(age(JOINT), 3),
                                 wrench_tokens=len(wt) if wt else None, wrench_age_s=round(age(WR), 3))
    if a.bounds: res["F_bounds"] = dict(pass_=(f_viol == 0), violations=f_viol)
    allpass = all(v.get("pass_", True) for v in res.values())
    for k, v in res.items():
        tag = "PASS" if v.get("pass_", True) else "FAIL"
        if "warn" in v: tag = "WARN" if v["warn"] else "OK"
        print(f"[consumer_check] {k:20s} {tag}  {json.dumps({kk: vv for kk, vv in v.items() if kk not in ('pass_',)})}")
    print("[consumer_check] RESULT", "PASS" if allpass else "FAIL")
    if a.report:
        with open(a.report, "w") as f: json.dump(dict(result="PASS" if allpass else "FAIL", checks=res), f, indent=1)
    return 0 if allpass else 1

if __name__ == "__main__":
    sys.exit(main())
