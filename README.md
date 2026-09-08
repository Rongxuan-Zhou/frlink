# frlink

A two-machine setup for a Franka FR3 with the Franka Hand: a small, pinned real-time host
owns the robot and runs the 1 kHz Cartesian impedance servo in a container on an isolated CPU;
any client PC connects to it over one direct Ethernet cable and drives the arm through three
narrow channels — a 128-byte UDP target-pose stream in, a plain-text UDP state stream out, and
a handful of control verbs over an ssh forced command. The client never links against
libfranka, never sees the robot's network, and can be replaced, updated or rebooted without
touching the robot side. This repository holds both halves plus the frozen interface between
them. The documented use is 6-DOF teleoperation from a WebXR headset with the gripper.

```
 client PC (yours)                                 RT host "alienware"                  FR3
 =================                                 ===================                  ===
 02_webxr_to_franka.py ---- UDP 128 B O_T_EE ---> :50001  cartesian_pose_servo -------FCI-----> 172.16.0.2
 (launch_live.sh)                                         docker franka-rt, CPU 2, FIFO 99      1 kHz
                                                          |
 /tmp/franka_*.txt <-- state mirror <-- UDP FRST1 <-------+ publisher threads (CPUs 4-15)
                       (:50002)
 franka-remote <verb> ---- ssh forced command ----> franka-ctl  -> systemd franka-servo@pose
                                                                -> goto home / gripper / echo / comm-test
 NIC 10.10.0.1/24 <=========== direct cable ============> frlink0 10.10.0.2/24
                                                          nftables: only 10.10.0.0/24, only ssh + udp 50001
```

## Use this RT host from your own PC

Full guide (principles, code walk-through, session procedure): [`docs/GUIDE.md`](docs/GUIDE.md).
The robot's own console, hand guiding and error recovery: [`docs/DESK.md`](docs/DESK.md).


1. **Cable.** Plug a straight Ethernet cable from your PC into the host's `frlink0` port (the
   ASIX USB port on the dock, not the built-in RJ45 — that one goes to the robot). No switch.
2. **Address.** Give your NIC `10.10.0.1/24`, no gateway. The host is `10.10.0.2`; `ping` it.
   The address is not negotiable: the servo publishes state to exactly one client and accepts
   commands from exactly one source, both fixed at `10.10.0.1` (see
   [`rt-host/README.md`](rt-host/README.md#one-client-at-a-time-why-the-visiting-pc-must-be-101001)).
3. **Key.** Generate an ed25519 key pair on your PC. Hand the public half to the host's owner,
   who appends it to `authorized_keys` on the host with the `franka-ctl` forced-command prefix
   (format in `rt-host/README.md`). Test: `ssh -i <key> rongxuan_zhou@10.10.0.2 status`.
4. **Client package.** Install the client half from [`client/`](client/) and follow
   [`client/README.md`](client/README.md): it provides the state mirror daemon, the `franka-remote`
   ssh wrapper, the client preflight, the environment switch that points the bridge at the
   host, and `launch_live.sh`, which orchestrates a session.
5. **Session.** Desk (`https://172.16.0.2/desk/`, reachable from the host): unlock joints,
   Activate FCI, press Enable. Then `franka-remote preflight`, `franka-remote goto home`,
   `franka-remote start pose`, and start the bridge (`launch_live.sh live` does the last two
   and waits for the mirrored state). `franka-remote stop` (or `launch_live.sh stop`) when done.

The wire formats, addresses, rates and exit codes are frozen in
[`docs/INTERFACE.md`](docs/INTERFACE.md). The design and its safety envelope are in
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md); the acceptance procedure and the measured
numbers are in [`docs/ACCEPTANCE.md`](docs/ACCEPTANCE.md).

## Safety

- **User stop in hand** for every step that can move the arm. `comm-test`, `goto home` and
  `restart pose home` move the arm on their own; a running servo moves it wherever the sender
  points.
- **Workspace clear** before `goto home`, `comm-test` and before starting the servo.
- **The servo itself has no workspace guard.** `cartesian_pose_servo` follows the target it is
  given, limited by a 200 ms hold when commands stop, a 1.5 s freeze on external force above
  35 N / 20 Nm, a per-joint torque cap, libfranka rate limiting and the robot's own collision
  reflex (30 N / 30 Nm). Workspace limits are enforced by the bridge on the client:
  - translation hard-clipped to the base-frame box x [0.150, 0.700] m, y [-0.550, 0.500] m,
    z [0.100, 0.656] m, with an anchor reset when the controller leaves the box;
  - per-frame step capped at 0.0055 m (at 90 Hz: 0.5 m/s, the ISO/TS 15066 collaborative speed);
  - rotation scaled by 0.5 (controller wrist angles map to half on the robot);
  - dead-man: the target is sent only while the Trigger is held; releasing it stops the stream
    and the servo holds within 200 ms;
  - gripper: the Grip button toggles open/close through `gripper_cmd`.
  Any other sender you write must implement equivalent guards; the host will not do it for you.
- **Never run `echo`, `goto`, `comm-test` or any direct-FCI tool while the servo is active.**
  `franka-ctl` refuses them (exit 75), but tools run by hand on the host bypass that check and
  a second libfranka connection ends the control loop.
- **One sender at a time.** The servo latches the first sender and drops the rest; the client
  package holds a local lock so a second local program fails loudly. Do not work around either.
- Keep GPU and CPU jobs, browsers and desktop applications off the RT host during sessions;
  thermal throttling shows up as missed cycles.

## Repository layout

```
README.md                 this file
CONTRIBUTING.md           conventions for changes
docs/
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
client/                   client half (state mirror, franka-remote, preflight, bridge, launch_live.sh) — see client/README.md
```

## License

MIT, see [`LICENSE`](LICENSE). libfranka (Apache-2.0) and the other dependencies keep their own licenses.
