#!/usr/bin/env python
"""calibrate_payload.py — franky 版,sweep m_load 找最优抵消残余 F_ext。
对标 scripts/calibrate_payload.cpp。只读(+setLoad sweep),不动机器人。

用法: python calibrate_payload.py <robot-ip>
"""
import sys, time
import numpy as np
import franky
from franky import Robot, RealtimeConfig

COM = [0.0, 0.0, 0.05]
INERTIA = [1e-4,0,0, 0,1e-4,0, 0,0,1e-4]


def sample(r, n=500):
    s6 = np.zeros(6)
    for _ in range(n):
        s6 += np.array(r.state.O_F_ext_hat_K); time.sleep(0.002)
    avg = s6 / n
    return avg, float(np.linalg.norm(avg[:3]))


def main():
    if len(sys.argv) != 2:
        print("Usage: calibrate_payload.py <robot-ip>", file=sys.stderr); return -1
    r = Robot(sys.argv[1], realtime_config=RealtimeConfig.Ignore)

    print("=========== Phase 1: Baseline(无 setLoad)===========")
    r.set_load(0.0, COM, INERTIA); time.sleep(0.3)
    base_avg, base_F = sample(r, 500)
    print(f"  F(N): [{base_avg[0]:.4f}, {base_avg[1]:.4f}, {base_avg[2]:.4f}]  ‖F‖={base_F:.4f}")
    print(f"\n→ Suggested m_extra (‖F‖/g): {base_F/9.81:.4f} kg")

    print("\n=========== Phase 2: 多 m_load 实测 ===========")
    best_F, best_m = base_F, 0.0
    for m in [0.05,0.10,0.15,0.20,0.25,0.30,0.35,0.40]:
        r.set_load(m, COM, INERTIA); time.sleep(0.3)
        _, F = sample(r, 200)
        print(f"  m_load={m:.2f} kg → ‖F‖={F:.4f} N")
        if F < best_F: best_F, best_m = F, m

    print("\n=========== Phase 3: 结果 ===========")
    print(f"  原始 ‖F‖   : {base_F:.4f} N")
    print(f"  最优 m_load: {best_m:.2f} kg")
    print(f"  最优 ‖F‖   : {best_F:.4f} N")
    print(f"  改善       : {base_F-best_F:.4f} N ({(1-best_F/base_F)*100:.1f}%)")
    r.set_load(best_m, COM, INERTIA)
    print("→ 已应用最优 m_load(本会话有效)")
    if best_F < 0.5:
        print("✅ ‖F‖ < 0.5 N — payload 偏差已消除")
    elif best_F < base_F*0.5:
        print(f"🟡 改善 >50% 但残余 {best_F:.3f} N,建议加软件 baseline 减法")
    else:
        print("⚠️  改善有限 — 残余可能来自 COM 偏移或 model bias")
    print(f"\n要永久生效: Desk → Settings → End Effector → Load mass 填 {best_m} kg")
    return 0


if __name__ == "__main__":
    sys.exit(main())
