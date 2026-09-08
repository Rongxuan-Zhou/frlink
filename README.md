English | [中文](README.zh-CN.md)

# frlink

A two-machine setup for a Franka FR3 with the Franka Hand. A small, pinned real-time host owns
the robot and runs the 1 kHz Cartesian impedance servo in a container on an isolated CPU. Any
client PC connects to it over one direct Ethernet cable. Three narrow channels cross that cable:
a 128-byte UDP target-pose stream in, a plain-text UDP state stream out, and a handful of
control verbs over an ssh forced command. The client never links against libfranka and never
sees the robot's network, so it can be replaced, updated or rebooted without touching the robot
side. This repository holds both halves plus the frozen interface between them. The documented
use is 6-DOF teleoperation from a WebXR headset with the gripper.

```
 client PC (yours)                                 RT host "alienware"                  FR3
 =================                                 ===================                  ===
 02_webxr_to_franka.py ---- UDP 128 B O_T_EE ---> :50001  cartesian_pose_servo -------FCI-----> 172.16.0.2
 (franka-teleop)                                         docker franka-rt, CPU 2, FIFO 99      1 kHz
                                                          |
 /tmp/franka_*.txt <-- state mirror <-- UDP FRST1 <-------+ publisher threads (CPUs 4-15)
                       (:50002)
 franka-remote <verb> ---- ssh forced command ----> franka-ctl  -> systemd franka-servo@pose
                                                                -> goto home / gripper / echo / comm-test
 NIC 10.10.0.1/24 <=========== direct cable ============> frlink0 10.10.0.2/24
                                                          nftables: only 10.10.0.0/24, only ssh + udp 50001
```

## Where to go next

| you want to | read |
|---|---|
| drive the robot from your own PC: principles, code walk-through, session procedure | [`docs/GUIDE.md`](docs/GUIDE.md) |
| unlock, activate, hand-guide or recover the robot in its own console | [`docs/DESK.md`](docs/DESK.md) |
| the wire formats, addresses, rates and exit codes (frozen) | [`docs/INTERFACE.md`](docs/INTERFACE.md) |
| the host CPU and thread layout and the safety envelope, as tables | [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) |
| the acceptance procedure and the measured numbers | [`docs/ACCEPTANCE.md`](docs/ACCEPTANCE.md) |
| install the client half | [`client/README.md`](client/README.md) |
| provision or administer the host, authorize a key | [`rt-host/README.md`](rt-host/README.md) |

## Connecting your PC

1. Plug a straight Ethernet cable from your PC into the host's `frlink0` port (the ASIX USB port
   on the dock, not the built-in RJ45, which goes to the robot). No switch.
2. Give your NIC `10.10.0.1/24`, no gateway. The host is `10.10.0.2`; `ping` it. The address is
   fixed: the servo publishes state to one client and accepts commands from one source, both
   `10.10.0.1` (see
   [`rt-host/README.md`](rt-host/README.md#one-client-at-a-time-why-the-visiting-pc-must-be-101001)).
3. Generate an ed25519 key pair. Hand the public half to the host's owner, who appends it to
   `authorized_keys` on the host with the `franka-ctl` forced-command prefix (format in
   `rt-host/README.md`). Test: `ssh -i <key> rongxuan_zhou@10.10.0.2 status`.
4. Install the client half as described in [`client/README.md`](client/README.md), then run a
   session as described in [`docs/GUIDE.md`](docs/GUIDE.md), section 5. The Desk steps that
   come first (unlock joints, Activate FCI, press Enable) are in [`docs/DESK.md`](docs/DESK.md).

## Safety

Keep the **user stop in hand** for every step that can move the arm. `comm-test`, `goto home`
and `restart pose home` move the arm on their own; a running servo moves it wherever the sender
points. Clear the workspace before `goto home`, `comm-test` and before starting the servo.

**The servo itself has no workspace guard.** `cartesian_pose_servo` follows the target it is
given. Its only limits are a 200 ms hold when commands stop, a 1.5 s freeze on external force
above 35 N / 20 Nm, a per-joint torque cap, libfranka rate limiting and the robot's own
collision reflex (30 N / 30 Nm). The bridge on the client enforces the workspace box, the
0.0055 m per-frame step cap (0.5 m/s at 90 Hz), the 0.5 rotation scale and the dead-man
trigger; the values are tabled in [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md). Any other
sender you write must implement equivalent guards; the host will not do it for you.

**Never run `echo`, `goto`, `comm-test` or any direct-FCI tool while the servo is active.**
`franka-ctl` refuses them (exit 75). Tools run by hand on the host bypass that check, and a
second libfranka connection ends the control loop.

One sender at a time. The servo latches the first sender and drops the rest; the client package
holds a local lock so a second local program fails loudly. Do not work around either.

Keep GPU and CPU jobs, browsers and desktop applications off the RT host during sessions;
thermal throttling shows up as missed cycles.

## Repository layout

```
README.md                 this file
CONTRIBUTING.md           conventions for changes
docs/
  GUIDE.md                using the robot from your own PC: principles, code, session procedure
  DESK.md                 the robot's own console: states, lights, hand guiding, error recovery
  INTERFACE.md            the frozen two-host contract (addresses, formats, verbs, exit codes)
  ARCHITECTURE.md         two-host design, the three flows, host thread/CPU layout, safety envelope
  ACCEPTANCE.md           day-1 / day-2 procedures and the 2026-09-08 day-1 numbers
rt-host/                  RT host half (export of ~/franka on the host)
  README.md               what the host does, provisioning order, franka-ctl verbs, key authorization
  bin/                    franka-ctl, franka-preflight, franka-run, servo unit helpers
  host/                   provision-rt.sh, verify-rt-host.sh, franka-rt-tune.sh, etc/ tree
  docker/                 Dockerfile + build scripts for franka-rt:0.17.0-jazzy
  teleop/                 servo and motion-helper sources, rt/ header units, tests/
  tests/                  benches, FRST1 sink, fake sender, robot-day checklists
  franky_tools/           franky-based host-native scripts: goto_home.py (used by `goto home`), calibration, wiggles (no servo active)
  docs/                   INTERFACE, RUNBOOK, ASSESSMENT as kept on the host
client/                   client half (state mirror, franka-remote, preflight, bridge, franka-teleop), see client/README.md
```

## License

MIT, see [`LICENSE`](LICENSE). libfranka (Apache-2.0) and the other dependencies keep their own
licenses.
