#!/usr/bin/env python
"""calibrate_payload_v2.py — franky 版,COM 坐标下降标定(m_load=0.25 固定)。
对标 scripts/calibrate_payload_v2.cpp。只读(+setLoad sweep),不动机器人。

策略: Phase1 sweep X → Phase2 sweep Y(X固定) → Phase3 sweep Z(X,Y固定) → 验证。

用法: python calibrate_payload_v2.py <robot-ip>
"""
import sys, time
import numpy as np
import franky
from franky import Robot, RealtimeConfig

M_LOAD = 0.25
INERTIA = [1e-4,0,0, 0,1e-4,0, 0,0,1e-4]


def measure_F_norm(r, cx, cy, cz, n=200):
    r.set_load(M_LOAD, [cx, cy, cz], INERTIA); time.sleep(0.4)
    s = np.zeros(3)
    for _ in range(n):
        s += np.array(r.state.O_F_ext_hat_K)[:3]; time.sleep(0.002)
    return float(np.linalg.norm(s / n))


def sweep_axis(r, axis, bx, by, bz, vals):
    print(f"\n  Sweep {axis} (m_load=0.25 fixed):")
    best_v = {'X': bx, 'Y': by, 'Z': bz}[axis]; best_F = 1e9
    for v in vals:
        if axis == 'X':   F = measure_F_norm(r, v, by, bz)
        elif axis == 'Y': F = measure_F_norm(r, bx, v, bz)
        else:             F = measure_F_norm(r, bx, by, v)
        print(f"    {axis} = {v:+.3f} m  →  ‖F‖ = {F:.4f} N")
        if F < best_F: best_F, best_v = F, v
    print(f"  → best {axis} = {best_v} (‖F‖={best_F:.4f})")
    return best_v


def main():
    if len(sys.argv) != 2:
        print("Usage: calibrate_payload_v2.py <robot-ip>", file=sys.stderr); return -1
    r = Robot(sys.argv[1], realtime_config=RealtimeConfig.Ignore)
    print("===== Phase 0: Baseline (m_load=0.25, COM=[0,0,0.05]) =====")
    F0 = measure_F_norm(r, 0.0, 0.0, 0.05)
    print(f"  ‖F‖ = {F0:.4f} N")
    bX, bY, bZ = 0.0, 0.0, 0.05
    print("\n===== Phase 1: Sweep X =====")
    bX = sweep_axis(r, 'X', bX, bY, bZ, [-0.10,-0.05,0.00,0.05,0.10])
    print("\n===== Phase 2: Sweep Y (X fixed) =====")
    bY = sweep_axis(r, 'Y', bX, bY, bZ, [-0.10,-0.05,0.00,0.05,0.10])
    print("\n===== Phase 3: Sweep Z (X,Y fixed) =====")
    bZ = sweep_axis(r, 'Z', bX, bY, bZ, [0.00,0.025,0.050,0.075,0.100])
    print("\n===== Final: 应用最优 + 验证 =====")
    F_final = measure_F_norm(r, bX, bY, bZ, 500)
    print(f"  Optimal F_x_Cload : [{bX}, {bY}, {bZ}] m")
    print(f"  Final ‖F‖         : {F_final:.4f} N (500 frames)")
    print(f"  Improvement vs P0 : {F0-F_final:.4f} N ({(1-F_final/F0)*100:.1f}%)")
    msg = ("✅ <0.5N payload bias 完全消除" if F_final<0.5 else
           "🟢 <1.0N 达到目标" if F_final<1.0 else
           "🟡 显著改善但>1N,残余可能 model bias" if F_final<F0*0.7 else
           "⚠️ 改善有限,建议二轮 sweep")
    print(f"  {msg}")
    print(f"\nTo persist (Desk): Load mass={M_LOAD}kg, Load center=[{bX}, {bY}, {bZ}] m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
