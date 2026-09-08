English | [中文](GUIDE.zh-CN.md)

# Using the FR3 + RT-host combination from your own computer

This guide is for driving the Franka FR3 behind its real-time host ("the RT host", an Alienware
m18 laptop) from your own Linux PC: Quest teleoperation, episode recording, or your own controller
sending target poses. Provisioning a new RT host: [`../rt-host/README.md`](../rt-host/README.md).

Contents

1. [What you are connecting to](#1-what-you-are-connecting-to)
2. [Principles: why two machines and how they talk](#2-principles-why-two-machines-and-how-they-talk)
3. [The RT host, code level](#3-the-rt-host-code-level)
4. [The client package, code level](#4-the-client-package-code-level)
5. [Running a session, step by step](#5-running-a-session-step-by-step)
6. [Recording data](#6-recording-data)
7. [Writing your own sender or consumer](#7-writing-your-own-sender-or-consumer)
8. [What can go wrong and what it looks like](#8-what-can-go-wrong-and-what-it-looks-like)
9. [Numbers you can expect](#9-numbers-you-can-expect)
10. [Software stack and libraries](#10-software-stack-and-libraries)

---

## 1. What you are connecting to

```
                 FCI cable (1 kHz, hard real-time)          direct cable (control plane, 10.10.0.0/24)
 Franka FR3 <=================================> RT host <===============================================> your PC
 172.16.0.2      built-in NIC enp110s0        Alienware      USB port frlink0 10.10.0.2        NIC 10.10.0.1
                                              m18 R2
```

Provided by the lab:

- The FR3 arm with the Franka Hand, system image 5.7.2, FCI licence. It talks only to the RT host.
  Its web console ("Desk", `https://172.16.0.2/desk/`) is reachable from the RT host only; see
  [`DESK.md`](DESK.md).
- The RT host: Ubuntu 22.04, kernel 6.8 pinned, CPUs 2-3 isolated for the control thread, the servo in
  a Docker image (`franka-rt:0.17.0-jazzy`, libfranka 0.17.0) under systemd. One Ethernet port for one
  client at a time, and an ssh key-only control channel.
- The frozen interface contract between the two machines, [`INTERFACE.md`](INTERFACE.md). This
  repository implements it.

You bring:

- A Linux PC with a free Ethernet port (NetworkManager, systemd user sessions, Python 3).
- One Ethernet cable. Your NIC is `10.10.0.1/24`; the host is `10.10.0.2`.
- An ssh public key. The host admin adds it to the host's `authorized_keys` with a forced command that
  allows only the `franka-ctl` verbs.
- Optional: a Meta Quest (teleoperation), Intel RealSense cameras (recording).

Everything you install is in [`../client/`](../client/README.md).

## 2. Principles: why two machines and how they talk

### 2.1 Why the servo is not on your PC

libfranka's torque interface wants a torque command every 1 ms and gives up after 20 consecutive
misses. A desktop kernel stalls for several milliseconds a few times a minute (measured on the
original workstation: ~6 ms stalls on every core). The RT host removes that variance: `preempt=full`,
`isolcpus=2,3`, NIC interrupts pinned to cpu 3, C-states limited on the isolated cores, control thread
pinned to cpu 2 with `mlockall`. Measured wake-up jitter on the isolated core: p99 = 11 us,
p99.9 = 49 us. Your PC has only soft deadlines: a target pose every ~11 ms, a state read every 50 ms.

### 2.2 The three flows

| flow | direction | transport | what |
|---|---|---|---|
| A. command | PC -> host | UDP `10.10.0.2:50001` | one datagram = 128 bytes = 16 float64 (little-endian), column-major 4x4 homogeneous `O_T_EE` target |
| B. state | host -> PC | UDP `10.10.0.1:50002` | `FRST1 <seq> <epoch_ns> <t_real_ns> <t_mono_ns> <name>\n<body>`; body is the text of one state file |
| C. control | PC -> host | ssh forced command | `franka-ctl status / preflight / start pose / stop / restart pose [home] / goto home / gripper ... / echo / comm-test --yes` |

Flow A has no sequence numbers or acknowledgements. The servo accepts a datagram only from the
allow-listed source IP (`10.10.0.1`) and only at exactly 128 bytes. It latches the first (ip, port)
that gets through and drops every other sender until the latched one has been silent for 1 s.

Flow B is a one-way stream of five named messages at fixed rates:

| name | tokens | rate | content |
|---|---|---|---|
| `franka_init_pose.txt` | 16 | once at servo start, re-sent every 1 s | `O_T_EE` at the moment the servo took over |
| `franka_current_ee.txt` | 16 | 20 Hz | live `O_T_EE` |
| `franka_wrench.txt` | 6 | 20 Hz | `O_F_ext_hat_K` (Fx Fy Fz Tx Ty Tz) |
| `franka_joint_state.txt` | 28 | 10 Hz | q[7] dq[7] tau_J[7] tau_ext_hat_filtered[7] |
| `franka_link.txt` | 13 key=value | 5 Hz | link and health counters (see 4.1) |

`epoch_ns` is the wall-clock start time of the servo process; a new epoch means a new servo instance.
Bodies are plain ASCII with default C++ stream formatting (6 significant digits), readable with
`float()` in any language.

Flow C exists so that a torque controller can never start by accident: the host runs only the verbs
above, each takes a file lock on the robot, and every verb that moves the arm refuses while a servo is
active.

### 2.3 The safety envelope (what the servo does regardless of what you send)

- Hold after 200 ms without an accepted command: the target snaps to the current pose each tick, so
  the arm stops and stays compliant. A restarted sender resumes control immediately.
- Force/torque freeze above 35 N or 20 Nm: the target is frozen for 1.5 s and commands are dropped
  (counted as accepted, not passed to the controller).
- Torque cap of 0.85 x [87 87 87 87 12 12 12] Nm, libfranka's own rate limiter, and collision reflex
  thresholds of 30 N / 30 Nm set on the robot.
- Reflex recovery: a reflex (collision, joint limit, velocity) throws inside `robot.control()`. The
  servo runs `automaticErrorRecovery`, re-reads the pose, freezes for 2 s and re-enters control, up
  to 50 times. `reflex_count` and `recovering` in `franka_link.txt` show it.
- Network guards: the allow-list and latch on port 50001 (2.2), host nftables accepting 50001 only
  from the client port, and the servo start precheck (3.2).
- Bridge guards on the client (`franka-teleop`): workspace box, per-frame step limit of 5.5 mm
  (0.5 m/s at 90 Hz), rotation scale 0.5, dead-man trigger.

### 2.4 Why the client writes files

Every consumer on the client (bridge, recorder, your scripts) reads robot state from
`/tmp/franka_*.txt`, not from the network; the `franka_state_mirror` daemon writes them from flow B
with atomic renames. Liveness is file mtime: `franka_current_ee.txt` older than 250 ms means the servo
is dead or the link silent. A state reader cannot tell single-host from two-host setups, and fits in
ten lines.

### 2.5 Clocks

State datagrams carry the host's wall clock. Episodes align camera frames (client clock) with robot
poses (host clock), and action labels are differences of consecutive poses, so the two clocks must
agree to better than 10 ms. Both machines run NTP; the mirror measures the skew continuously and
`franka-client-preflight` reports it.

## 3. The RT host, code level

All paths under [`../rt-host/`](../rt-host/). A visitor cannot change any of it.

### 3.1 `bin/franka-ctl`: the only entry point

```
verb -> [refuse_if_servo] -> [with_robot_lock: flock -n /run/lock/franka-robot.lock] -> action
```

- `start pose`: `sudo systemctl start franka-servo@pose` (sudoers allows exactly these units), waits
  2 s, confirms `active`. Exit codes: 0, 64 (bad verb or token; the forced command rejects any
  argument that is not `[A-Za-z0-9._:=-]+`), 75 (busy: lock held or servo active).
- `stop`: `systemctl stop` -> SIGTERM -> `MotionFinished` -> clean exit within ~1 s.
- `restart pose home`: stop, home sequence, start, under one robot lock. A failed home (e.g. a
  reflex) is exit 1 and the servo stays down.
- `goto home`: the franky version (`bin/franka-py franky_tools/goto_home.py`). It lifts the end
  effector to 0.15 m first if near the table, checks the joint path with forward kinematics
  (`franky_helpers.path_min_z`) and refuses if the path dips below 0.105 m.
- `gripper <argv>`: `gripper_cmd` in a one-shot container, over the gripper's own TCP channel
  (port 1338), so it is allowed while the servo runs.
- `echo`: one JSON robot state; refused while a servo is active (it would steal the FCI channel).

### 3.2 `franka-servo@.service`: how the servo is started

`ExecStartPre=bin/franka-servo-precheck` refuses the start if `/run/franka/rt-tune.ok` is missing
(RT tuning did not run), the nftables table is not loaded, AC power is off, the 10 s median package
temperature is >= 85 C, another robot container is running, the bind address is not configured, or the
image/binary is missing. `ExecStart` = `systemd-inhibit` (no sleep/lid) + `bin/franka-servo-run` =

```
docker run --rm --init --privileged --network host --ipc host --ulimit rtprio=99 --ulimit memlock=-1
    franka-rt:0.17.0-jazzy /franka/teleop/cartesian_pose_servo 172.16.0.2
    --cmd-bind 10.10.0.2 --cmd-allow 10.10.0.1 --state-dst 10.10.0.1:50002
    --rt-cpu 2 --aux-cpus 4-15 --state-files 1 --state-dir /tmp/franka
```

Log: `/tmp/franka/servo-pose.log` on the host; counters in `/tmp/franka/franka_link.txt`.

### 3.3 `teleop/cartesian_pose_servo.cpp`: the controller

One process, seven threads:

| thread | cpu | job |
|---|---|---|
| main (control) | 2, SCHED_FIFO 99 (set by libfranka) | `robot.control(torque_cb)` at 1 kHz |
| `udp_cmd` | 4-15 | `recvfrom` with 100 ms timeout -> `CmdFilter` -> `TripleBuffer<target>` |
| `ee_writer`, `wrench_writer`, `joint_writer` | 4-15 | 20/20/10 Hz: format the latest sample, write the file, send the datagram |
| `init_resend`, `link_writer` | 4-15 | 1 Hz init pose re-send, 5 Hz link counters |

The 1 kHz callback does no allocation, I/O or locking. Per tick:

1. `tick_stats.on_tick()` (interval instrumentation -> `missed_cycles_total`, `tick_max_us_1s`).
2. Read the newest target from the lock-free `TripleBuffer` (writer: udp thread). After 200 ms
   without an accepted packet, or during a freeze, a tick-local override substitutes the current
   pose; a new packet cancels it.
3. Two-stage low-pass filter on the target (translation EMA alpha 0.10 then 0.05; rotation by
   quaternion slerp with hemisphere correction).
4. Cartesian impedance: `tau = J^T (-K e - D J dq) + coriolis`, K_t = 1000 N/m, K_r = 80 Nm/rad, damping
   critically matched to an effective mass of 5 kg / 0.3 kg m^2. Near contact (|F| between 10 and 25 N)
   stiffness blends down to 25 %; near a singularity (manipulability w < 0.015) it is scaled down; a
   null-space term pushes joints away from their limits (0.5 rad margin).
5. Clamp to the torque cap, `franka::limitRate`, return. `MotionFinished` once SIGTERM was seen.

Network and RT plumbing is header-only in `teleop/rt/`: `udp_cmd.hpp` (socket + allow/size/latch
filter), `state_pub.hpp` (one formatter for file and datagram, so the bytes are identical),
`triple_buffer.hpp`, `rt_setup.hpp` (mlockall, stack prefault, affinity), `writers.hpp`.
`tests/servo_net_stub.cpp` is the same program without libfranka, for testing clients without a robot.

## 4. The client package, code level

All paths under [`../client/`](../client/). One config file, `config.env` (copied from
`config.env.example`): host address and user, key path, your IP and NIC name, the bridge's Python
interpreter. Every script derives the repo root from its own location; nothing is hard-coded.

### 4.1 `bin/franka_state_mirror`: flow B to files

A stdlib-only Python daemon run by the user unit `franka-state-mirror` (installed by `install.sh`):

```
recvfrom 0.0.0.0:50002 -> source must be $FRANKA_SERVO_HOST
  -> header: 6 tokens, "FRST1", four ints         (else drop_hdr)
  -> name in the five known names                 (else drop_name)
  -> body: N floats, or the 13 link keys in order (else drop_body)
  -> seq must not go backwards for that name      (else drop_reorder)
  -> epoch changed? unlink franka_init_pose.txt, reset per-name seq, log "EPOCH CHANGE"
  -> write /tmp/.franka_<name>.tmp, os.replace() -> /tmp/<name>
```

Nothing is deleted when the stream stops (consumers use mtime). `franka_init_pose.txt` is written once
per epoch, so its mtime means "this servo instance started" (4.4 relies on that). A status line goes
to `/tmp/franka_mirror.txt` once a second:
`epoch seq src age_ms skew_ms rx drop_src drop_hdr drop_name drop_body drop_reorder ...`.
`--check` is the health probe used by preflight (status fresher than 2 s, EE file fresher than 250 ms).

`franka_link.txt` keys, in order: `cmd_age_ms` (-1 until the first accepted command), `cmd_pkts_last_s`,
`cmd_drop_size`, `cmd_drop_allow`, `cmd_drop_latch`, `latched_sender`, `missed_cycles_total`,
`max_consecutive_missed`, `freeze`, `recovering`, `tick_over_1p2ms_1s`, `tick_max_us_1s`, `reflex_count`.

### 4.2 `bin/franka-remote`: flow C

`ssh -i $FRANKA_CTL_KEY -o BatchMode=yes -o ConnectTimeout=3 user@host <verb...>` under `timeout`
(20 s for status/gripper/echo, 45 s start/stop, 120 s goto/restart, 90 s comm-test). Exit codes pass
through untouched: 75 (busy), 255 (no link), 124 (timeout).

### 4.3 `bin/franka-fci-shim`: the old binary names

`bin/gripper_cmd`, `bin/goto_home`, `bin/echo_robot_state` are symlinks to one script that maps the
classic argument shape (`gripper_cmd <ip> close`) onto `franka-remote gripper close`, so the bridge
calls `gripper_cmd` as it did with a local robot. Anything the host does not offer (custom joint
targets) exits 64 rather than being mapped to a different motion.

### 4.4 `bin/franka-teleop`: the orchestrator

```
live    = servo_start -> wait_state_ready -> bridge_start
restart = bridge_kill -> franka-remote restart pose home (retry only on 75) -> wait_state_ready 15 -> bridge_start
stop    = bridge_kill -> franka-remote stop
```

`wait_state_ready` replaces "sleep 5": the mirror must report a new epoch, `franka_init_pose.txt`
must have 16 tokens and `franka_current_ee.txt` must be fresher than 250 ms. Before starting a servo
it deletes the old `franka_init_pose.txt`, so the bridge can never read the previous servo's pose.
Workspace box, scale and step limit are parameters at the top of the script (`FRANKA_WS_*`,
`FRANKA_SCALE_*`, `FRANKA_MAX_STEP` env overrides).

### 4.5 `teleop/02_webxr_to_franka.py`: the bridge

- Serves the WebXR page (`frontend_swapped/`) over HTTPS on port 4443 through the `teleop` pip package
  and receives one JSON message per headset frame (~90 Hz): controller position/orientation, `move`
  (Trigger held), `gripper` (Grip toggled), scale.
- On start it reads `franka_init_pose.txt` (waits up to 5 s). The first controller pose with Trigger
  held is the hand origin. Every later pose is a delta from that origin, mirrored about the robot base
  z axis (`_R_MIRROR_Z`: "towards the robot" for you is "towards you" for the arm facing you), scaled
  (translation 1.0, rotation 0.5), then applied to the init pose: `p = p_init + M delta_p`,
  `R = R_init M delta_R M`.
- Guards in order: workspace box clip, per-frame step clamp (5.5 mm), rotation step clamp (0.05 rad),
  slow-down zone near the box. After a clip the hand origin is re-anchored so the arm does not jump
  when you come back.
- Dead-man: Trigger released -> the last target is re-sent (the servo holds it); the servo's own 200 ms
  rule covers a dead bridge.
- Packet: `struct.pack("16d", *T.flatten(order="F"))` to `--udp-host:50001`.
- Gripper: a Grip edge spawns `gripper_cmd <ip> open|close` (through the shim, i.e. `franka-remote`).
- One sender at a time on the PC: `franka_sender_lock.acquire_sender_lock()` takes `flock` on
  `/tmp/franka_sender.lock`; a second live sender exits 75.

### 4.6 `record/`: episodes to HDF5

- `cameras/grabber.py`: one thread per RealSense camera from `config/cameras.yaml` (serial -> role),
  640x480 @ 30 fps colour (+ aligned depth), auto-exposure locked after convergence, hardware
  timestamp kept per frame.
- `state_reader.py`: `make_state_fn()` returns a function that reads `/tmp/franka_current_ee.txt`
  (optionally wrench/joints), applies the 250 ms staleness rule and returns
  `{"ee_pose": (16,), "ee_ok": bool, ...}`.
- `record_episodes.py`: keyboard-driven (`s` start, `e` end, `d` discard, `q` quit) or
  `--auto-end-secs`; polls the state at 20 Hz, buffers frames, writes each episode in a background
  thread. Layout per file:

```
observations/<role>/image            (N, 480, 640, 3) uint8
observations/<role>/hw_timestamp_ms  (N,) float64      camera hardware clock
observations/<role>/wall_timestamp_s (N,) float64      client wall clock at frame receipt
state/ee_pose                        (M, 16) float32   column-major O_T_EE, from the mirror
state/ee_ok                          (M,) bool         False = stale sample (zeros)
state/_t                             (M,) float64      client wall clock at the poll
```

Align on the wall clocks (both on the client) and treat `ee_ok == False` samples as gaps.

## 5. Running a session, step by step

### 5.1 One-time setup on your PC

```
git clone <this repo>; cd client
cp config.env.example config.env; $EDITOR config.env      # FRANKA_CTL_USER, FRANKA_LINK_IFACE, FRANKA_PY
nmcli con add type ethernet ifname <your NIC> con-name franka-link ipv4.method manual \
    ipv4.addresses 10.10.0.1/24 ipv4.never-default yes ipv6.method disabled connection.autoconnect yes
./install.sh          # bin -> ~/.local/bin, ssh key + config block, mirror user unit; prints your authorized_keys line
pip install -r requirements-teleop.txt      # into $FRANKA_PY's environment
pip install -r requirements-record.txt      # only if you record
```

Hand the printed `authorized_keys` line to the host admin. Until it is installed, `franka-remote
status` exits 255.

### 5.2 Every session

```
source config.env
franka-client-preflight        # every line PASS or an explained WARN
```
Ask the admin (or do it on the host's screen): Desk -> unlock joints -> Activate FCI -> press the
enabling button. Required after every robot power cycle; nothing on your PC can do it. Desk steps,
hand guiding and error recovery: [`DESK.md`](DESK.md).

```
franka-remote status           # franka-servo@pose inactive, fci link up
franka-remote goto home        # arm moves to factory-ready (~10 s), user stop in hand
franka-teleop live             # servo on the host, state ready, bridge up (~7 s)
franka-teleop status
```

### 5.3 Teleoperate

Open `https://<your PC IP>:4443/` on the Quest (same network as your PC, or a tunnel you set up; the
page uses a self-signed certificate, accept it once). Details and the controller map:
[`../client/teleop/README.md`](../client/teleop/README.md).

### 5.4 Operator procedure in the headset

1. **Stand directly in front of the robot, facing it, before you press Enter VR.** The WebXR frame is
   captured at that moment and every later hand motion is interpreted in it.
2. After that you may move around, to the side or behind the arm on the same side. The mapping does
   not rotate with you; the arm keeps moving in the calibrated directions.
3. Hold Trigger to move; release it before walking. Press Grip to toggle the gripper.
4. **When you finish, press Exit VR in the web page first**, then `franka-teleop stop` on the PC,
   then `franka-remote goto home` if you leave the arm.

### 5.5 Stop

```
franka-teleop stop             # bridge, then the servo (clean MotionFinished, ~1 s)
franka-remote goto home
```
`franka-teleop restart` (stop, home, start, bridge again, ~8 s) if the bridge or headset got confused
mid-session.

## 6. Recording data

```
python3 record/state_reader.py --selftest --secs 3                  # expect ee_ok 60/60 while the servo runs
python3 record/record_episodes.py --out-dir ~/datasets/<task> --prefix <task> [--with-wrench --with-joints]
```
Run it in a second terminal while `franka-teleop live` is up. Cameras are looked up by serial in
`record/config/cameras.yaml`; find yours with `rs-enumerate-devices | grep Serial`. The recorder refuses
to start if the state path is dead (override with `--allow-dead-state` for camera-only tests) and warns
when an episode has less than 95 % `ee_ok`.

## 7. Writing your own sender or consumer

Sender (any language; Python shown). Read the current pose and send 128-byte targets at 10-100 Hz; a
pause of more than 200 ms makes the arm hold:

```python
import socket, struct, time, numpy as np
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("10.10.0.1", 0))                       # source must be 10.10.0.1 (allow-list)
T = np.array([float(x) for x in open("/tmp/franka_current_ee.txt").read().split()]).reshape(4, 4, order="F")
for k in range(1000):                            # 10 s at 100 Hz
    T[2, 3] = T[2, 3] + 0.0001                   # creep up 0.1 mm per step (10 mm/s)
    sock.sendto(struct.pack("16d", *T.flatten(order="F")), ("10.10.0.2", 50001))
    time.sleep(0.01)
```
Take the sender lock first if you share the PC with the bridge (`teleop/franka_sender_lock.py`). Keep
steps small: the servo filters and clamps torque but not your target; a 10 cm jump is a 10 cm jump. In
`franka_link.txt`, `latched_sender` must be your `ip:port`, `cmd_pkts_last_s` your rate, and
`cmd_drop_allow` must stay 0.

Consumer: read the files.

```python
from record.state_reader import make_state_fn
state = make_state_fn()          # dict with ee_pose (16,), ee_ok; add with_wrench/with_joints
```

Or stop the mirror unit and bind 50002 yourself; only one process can own the port, and the mirror is
the recommended owner.

Extending the host (admin only): a new verb is a new `case` in `franka-ctl`; a new servo mode is a
new systemd instance name in `franka-servo-run` plus its sudoers lines. Any change to flow A/B bumps
`FRST1` to `FRST2` per the contract.

## 8. What can go wrong and what it looks like

| you see | meaning | do |
|---|---|---|
| `franka-remote` exit 255 | no ssh: cable, IP, key not authorized, host down | `ping 10.10.0.2`, `nmcli con show franka-link`, ask the admin about your key |
| `franka-remote start pose` exit 1, journal says precheck FAIL | host gate: thermal, RT tuning, image, robot busy | `franka-remote status`; the admin runs `franka-ctl preflight` on the host |
| exit 75 | robot lock held or a servo already active | `franka-remote status`; stop what runs first |
| bridge: "init pose timeout" | mirror not receiving: unit down, link down, servo publishing elsewhere | `franka_state_mirror --check`, `systemctl --user status franka-state-mirror` |
| arm holds while you move | your packets are not accepted: `cmd_drop_allow` (wrong source IP), `cmd_drop_latch` (another sender latched), `cmd_pkts_last_s`=0 (bridge dead) | read `/tmp/franka_link.txt` |
| arm freezes ~1.5 s after a bump | F/T freeze (35 N / 20 Nm) | expected; ease off |
| `reflex_count` incremented, arm re-enters control after ~2 s | robot reflex (collision, joint limit) | expected; if it repeats, stop and check the workspace |
| `restart` fails with "home sequence failed" | a reflex during the home motion (obstacle) | clear the workspace, `franka-remote goto home`, `franka-teleop live` |
| preflight: clock skew WARN/FAIL | NTP drift | fix NTP on your PC; recorded data is only trustworthy under 10 ms |
| second sender exits 75 | sender lock held | stop the other one |

## 9. Numbers you can expect

From the acceptance runs of 2026-09-08 ([`ACCEPTANCE.md`](ACCEPTANCE.md)):

| quantity | value |
|---|---|
| control-plane RTT (direct cable) | 0.32 ms avg, 0.5 ms max |
| FCI link (host to robot) | 10 000 pings at 1 kHz, 0 loss, 0.48 ms max |
| `communication_test` success rate | 1.00 / 1.00 / 1.00 (three runs) |
| 5-minute hold, no sender | drift 0.10 mm, 0 missed cycles, 0 reflex |
| `franka-teleop live` from cold | ~7 s to "state ready" |
| `franka-teleop restart` | ~8 s |
| gripper command through the shim | ~2.5 s round trip |
| slow circle tracking (3 cm radius, 9 mm/s) | 8 mm mean error: impedance compliance, not latency (command age 5 ms) |
| link outage of 10 s | servo unaffected, mirror resumes < 0.2 s after link up |
| teleop session (Quest, ~90 Hz) | 0 missed cycles; one F/T freeze and one reflex on contact, both self-recovered |

## 10. Software stack and libraries

Versions are the ones the acceptance runs used; the pins are in `client/requirements-*.txt` and
`rt-host/docker/`.

### RT host

| library | version | role | where |
|---|---|---|---|
| libfranka | 0.17.0 (commit `4448c390`, `libfranka-common` `cd38d0ec`) | the FCI client: 1 kHz `robot.control()` torque loop, `Model` for Jacobian/Coriolis, `Gripper` over TCP 1338, `automaticErrorRecovery` | built from source inside the Docker image; the servo, `gripper_cmd`, `echo_robot_state`, `fk_probe` link it |
| pinocchio | 3.9.0 (`ros-jazzy-pinocchio` from the ROS 2 Jazzy snapshot of 2026-04-13) | rigid-body kinematics and dynamics libfranka's model uses | Docker image; the snapshot is pinned because the live repo moved to pinocchio 4.0 |
| Eigen | 3.4 | matrices and quaternions in the controller | Docker image |
| Poco | 1.11 | libfranka's network layer | Docker image |
| fmt | 9.1 | logging inside libfranka | Docker image |
| Docker image `franka-rt:0.17.0-jazzy` | Ubuntu 24.04 + the above, gcc 13 | reproducible user space for the servo regardless of the host OS; run with `--privileged --network host --ulimit rtprio=99 --ulimit memlock=-1` | `rt-host/docker/Dockerfile`, `build-image.sh`, `build-inside.sh` |
| franky (`franky-control`) | 1.1.3, the wheel built against libfranka 0.17.0, Python 3.10 venv on the host | point-to-point motions with libfranka underneath: `franka-ctl goto home` runs `franky_tools/goto_home.py`, which lifts clear of the table and checks the joint path with forward kinematics before moving; also payload calibration and gripper tools | `rt-host/franky_tools/`, run through `rt-host/bin/franka-py`; only while no servo is active |
| systemd, nftables, NetworkManager, DKMS `r8126` | Ubuntu 22.04 stock | service supervision, the port firewall, the two NIC profiles, the robot NIC driver | `rt-host/host/` |

The servo itself (`teleop/cartesian_pose_servo.cpp` plus the header-only `teleop/rt/`) has no other
dependency: the UDP, triple-buffer and real-time setup code is plain POSIX.

### Client

| library | version | role | where |
|---|---|---|---|
| `teleop` (Spes Robotics) | 0.1.5 | serves the WebXR page over HTTPS and turns the headset's controller pose into a Python callback at the headset frame rate; the swapped frontend in `client/teleop/frontend_swapped/` overrides its UI to give the Trigger/Grip button map | `teleop/02_webxr_to_franka.py` |
| FastAPI + uvicorn | 0.136.1 / 0.46.0 | the web server underneath `teleop` (port 4443, WebSocket `/ws`) | pulled in by `teleop` |
| numpy, scipy, transforms3d | 2.4.4 / 1.17.1 / 0.4.2 | 4x4 pose algebra, rotation scaling (`scipy.spatial.transform.Rotation`), quaternion conversions | bridge |
| Python stdlib only | 3.x | the state mirror, the sender lock, the FRST1 test generator, the preflight helpers | `client/bin/franka_state_mirror`, `client/teleop/franka_sender_lock.py`, `client/tests/fake_frst1.py` |
| pyrealsense2 | 2.57.7 | Intel RealSense capture with hardware timestamps, exposure lock | `client/record/cameras/grabber.py` |
| h5py, opencv-python, PyYAML | 3.16.0 / 4.13.0 / 6.0.3 | HDF5 episode files, image handling, `cameras.yaml` | `client/record/` |
| OpenSSH, NetworkManager, systemd user units | stock | the control channel, the private-link profile, the mirror service | `client/install.sh` |

libfranka is pinned to 0.17.0 because the robot's system image (5.7.2) speaks research-interface
version 9, which only libfranka 0.15 to 0.17 implement; upgrading either side alone breaks the
connection. franky does the slow motions and the C++ servo the fast one: franky's Python API makes the
safety checks around a point-to-point move easy to write, while the 1 kHz impedance loop needs the
compiled controller.
