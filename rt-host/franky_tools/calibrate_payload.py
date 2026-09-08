#!/usr/bin/env python
"""calibrate_payload.py — franky version; sweep m_load to find the value that best cancels the residual F_ext.
Counterpart of scripts/calibrate_payload.cpp. Read-only (+setLoad sweep); does not move the robot.

Usage: python calibrate_payload.py <robot-ip>
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

    print("=========== Phase 1: Baseline (no setLoad) ===========")
    r.set_load(0.0, COM, INERTIA); time.sleep(0.3)
    base_avg, base_F = sample(r, 500)
    print(f"  F(N): [{base_avg[0]:.4f}, {base_avg[1]:.4f}, {base_avg[2]:.4f}]  ‖F‖={base_F:.4f}")
    print(f"\n→ Suggested m_extra (‖F‖/g): {base_F/9.81:.4f} kg")

    print("\n=========== Phase 2: Measure multiple m_load values ===========")
    best_F, best_m = base_F, 0.0
    for m in [0.05,0.10,0.15,0.20,0.25,0.30,0.35,0.40]:
        r.set_load(m, COM, INERTIA); time.sleep(0.3)
        _, F = sample(r, 200)
        print(f"  m_load={m:.2f} kg → ‖F‖={F:.4f} N")
        if F < best_F: best_F, best_m = F, m

    print("\n=========== Phase 3: Results ===========")
    print(f"  Original ‖F‖ : {base_F:.4f} N")
    print(f"  Best m_load  : {best_m:.2f} kg")
    print(f"  Best ‖F‖     : {best_F:.4f} N")
    print(f"  Improvement  : {base_F-best_F:.4f} N ({(1-best_F/base_F)*100:.1f}%)")
    r.set_load(best_m, COM, INERTIA)
    print("→ Best m_load applied (valid for this session only)")
    if best_F < 0.5:
        print("✅ ‖F‖ < 0.5 N — payload bias eliminated")
    elif best_F < base_F*0.5:
        print(f"🟡 Improvement >50% but residual {best_F:.3f} N; consider adding software baseline subtraction")
    else:
        print("⚠️  Limited improvement — residual may come from COM offset or model bias")
    print(f"\nTo make it permanent: Desk → Settings → End Effector → Load mass = {best_m} kg")
    return 0


if __name__ == "__main__":
    sys.exit(main())
