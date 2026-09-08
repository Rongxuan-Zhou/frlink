#!/usr/bin/env python
"""snapshot_setload.py — franky version; after setLoad(0.25), dump N frames of JSON state.
Counterpart of scripts/snapshot_setload.cpp (format matches what analyze_diagnose.py expects).
Read-only (+setLoad); one JSON line per frame to stdout.

Usage: python snapshot_setload.py <robot-ip> <n_frames>
"""
import sys, time
import franky
from franky import Robot, RealtimeConfig


def main():
    if len(sys.argv) != 3:
        print("Usage: snapshot_setload.py <robot-ip> <n_frames>", file=sys.stderr); return -1
    ip, N = sys.argv[1], int(sys.argv[2])
    r = Robot(ip, realtime_config=RealtimeConfig.Ignore)
    r.set_load(0.25, [0.0, 0.0, 0.05], [1e-4,0,0, 0,1e-4,0, 0,0,1e-4])
    time.sleep(0.5)
    for _ in range(N):
        s = r.state
        q = list(s.q); tj = list(s.tau_J); te = list(s.tau_ext_hat_filtered); F = list(s.O_F_ext_hat_K)
        fee = list(s.F_x_Cee)
        row = ('{"q":[%s],"tau_J":[%s],"tau_ext_hat_filtered":[%s],"O_F_ext_hat_K":[%s],'
               '"m_ee":%.8f,"F_x_Cee":[%.8f,%.8f,%.8f],"m_load":%.8f,"m_total":%.8f,'
               '"current_errors":[],"last_motion_errors":[],"robot_mode":"Idle"}') % (
            ",".join("%.8f" % x for x in q), ",".join("%.8f" % x for x in tj),
            ",".join("%.8f" % x for x in te), ",".join("%.8f" % x for x in F),
            s.m_ee, fee[0], fee[1], fee[2], s.m_load, s.m_total)
        print(row)
        time.sleep(0.002)
    return 0


if __name__ == "__main__":
    sys.exit(main())
