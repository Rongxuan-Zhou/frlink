#!/usr/bin/env python
"""record_baseline.py — franky 版,记录静止姿态 tau_ext/F_ext baseline。
对标 scripts/record_baseline.cpp。采 1000 帧 q/tau_ext/F_ext 均值 → JSON。
只读(+setLoad),不动机器人。可与 servo 关闭时运行。

用法: python record_baseline.py <ip> [output.json]
"""
import sys, time, json
import numpy as np
import franky
from franky import Robot, RealtimeConfig


def main():
    if not (2 <= len(sys.argv) <= 3):
        print("Usage: record_baseline.py <robot-ip> [output.json]", file=sys.stderr); return -1
    ip = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) == 3 else "baseline.json"
    r = Robot(ip, realtime_config=RealtimeConfig.Ignore)
    r.set_load(0.25, [0.0, 0.0, 0.05], [1e-4,0,0, 0,1e-4,0, 0,0,1e-4])
    time.sleep(0.5)

    N = 1000
    q_sum = np.zeros(7); tau_sum = np.zeros(7); F_sum = np.zeros(6)
    print(f"Sampling {N} frames @ ~500 Hz...")
    for _ in range(N):
        s = r.state
        q_sum += np.array(s.q); tau_sum += np.array(s.tau_ext_hat_filtered)
        F_sum += np.array(s.O_F_ext_hat_K)
        time.sleep(0.002)
    q_b, tau_b, F_b = q_sum/N, tau_sum/N, F_sum/N
    F_norm = float(np.linalg.norm(F_b[:3]))
    data = {"timestamp_unix": int(time.time()), "n_samples": N, "m_load_used": 0.25,
            "F_x_Cload": [0.0, 0.0, 0.05],
            "q_baseline": [round(float(x),8) for x in q_b],
            "tau_ext_baseline": [round(float(x),8) for x in tau_b],
            "F_ext_K_baseline": [round(float(x),8) for x in F_b]}
    with open(out_path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"✅ Baseline written to: {out_path}")
    print(f"   ‖F_ext‖ baseline : {F_norm:.4f} N")
    print(f"   tau_ext_filtered : {[round(float(x),3) for x in tau_b]} Nm")
    return 0


if __name__ == "__main__":
    sys.exit(main())
