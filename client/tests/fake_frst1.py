#!/usr/bin/env python3
"""fake_frst1.py -- emit FRST1 state datagrams like the RT-host servo, for testing the client-side state mirror.

Rates follow the contract (docs/INTERFACE.md 2.3): current_ee 20 Hz, wrench 20 Hz,
joint_state 10 Hz, link 5 Hz, init_pose once at start and re-sent every 1 s. seq is one counter for
the whole process; epoch is CLOCK_REALTIME at start (or at each simulated restart).

    tests/fake_frst1.py --dst 127.0.0.1:50002 --duration 10 --epoch-bump-after 5
    tests/fake_frst1.py --dst 127.0.0.1:50002 --bad        # one malformed datagram of each kind, then exit

The EE pose moves along a slow circle so a consumer can see the file change.
"""
import argparse
import math
import socket
import sys
import time

LINK_KEYS = [
    "cmd_age_ms", "cmd_pkts_last_s", "cmd_drop_size", "cmd_drop_allow", "cmd_drop_latch",
    "latched_sender", "missed_cycles_total", "max_consecutive_missed", "freeze", "recovering",
    "tick_over_1p2ms_1s", "tick_max_us_1s", "reflex_count",
]


def fmt(vals):
    """Same formatting as the servo: default ostream precision (6 significant digits), one line."""
    return (" ".join("%g" % v for v in vals) + "\n").encode()


def pose(t):
    x, y, z = 0.45 + 0.03 * math.cos(t / 3.0), 0.03 * math.sin(t / 3.0), 0.098
    return [1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, x, y, z, 1]


class Sender:
    def __init__(self, dst, src_port):
        host, port = dst.rsplit(":", 1)
        self.dst = (host, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if src_port:
            self.sock.bind(("", src_port))
        self.seq = 0
        self.epoch = time.time_ns()

    def send(self, name, body, raw_header=None):
        hdr = raw_header if raw_header is not None else b"FRST1 %d %d %d %d %s\n" % (
            self.seq, self.epoch, time.time_ns(), time.monotonic_ns(), name.encode())
        self.sock.sendto(hdr + body, self.dst)
        self.seq += 1

    def restart(self):
        """Simulate a servo restart: new epoch, seq back to 0."""
        self.seq = 0
        self.epoch = time.time_ns()


def run(args):
    s = Sender(args.dst, args.src_port)
    if args.bad:
        s.send("franka_current_ee.txt", fmt(pose(0))[:-1] + b"", raw_header=b"FRST1 1 2 3\n")   # 4-token header
        s.send("franka_current_ee.txt", fmt(pose(0)), raw_header=b"FRSTX 1 2 3 4 franka_current_ee.txt\n")
        s.send("franka_unknown.txt", fmt(pose(0)))                                              # unknown name
        s.send("franka_current_ee.txt", fmt(pose(0)[:15]))                                      # 15 tokens
        s.send("franka_link.txt", b"cmd_age_ms=1 bogus=2\n")                                    # bad key order
        print("sent 5 malformed datagrams")
        return 0
    t0 = time.monotonic()
    init_sent_at = -1.0
    next_ee = next_joint = next_link = t0
    bumped = False
    n = 0
    while True:
        t = time.monotonic() - t0
        if args.duration and t >= args.duration:
            break
        if args.epoch_bump_after and not bumped and t >= args.epoch_bump_after:
            s.restart()
            init_sent_at = -1.0
            bumped = True
            print("epoch bump -> %d" % s.epoch)
        if init_sent_at < 0 or t - init_sent_at >= 1.0:
            s.send("franka_init_pose.txt", fmt(pose(0)))
            init_sent_at = t
        now = time.monotonic()
        if now >= next_ee:
            s.send("franka_current_ee.txt", fmt(pose(t)))
            s.send("franka_wrench.txt", fmt([0.1, -0.2, 9.8, 0.01, 0.02, 0.03]))
            next_ee += 0.05
            n += 1
        if now >= next_joint:
            s.send("franka_joint_state.txt", fmt([0.0] * 28))
            next_joint += 0.1
        if now >= next_link:
            vals = [-1, 0, 0, 0, 0, "none", 0, 0, 0, 0, 0, 120, 0]
            body = (" ".join("%s=%s" % kv for kv in zip(LINK_KEYS, vals)) + "\n").encode()
            s.send("franka_link.txt", body)
            next_link += 0.2
        time.sleep(0.005)
    print("sent seq=%d ee=%d epoch=%d" % (s.seq, n, s.epoch))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--dst", default="127.0.0.1:50002")
    ap.add_argument("--src-port", type=int, default=0)
    ap.add_argument("--duration", type=float, default=10.0, help="seconds; 0 = until Ctrl-C")
    ap.add_argument("--epoch-bump-after", type=float, default=0.0, help="simulate a servo restart at t seconds")
    ap.add_argument("--bad", action="store_true", help="send malformed datagrams only")
    return run(ap.parse_args())


if __name__ == "__main__":
    sys.exit(main())
