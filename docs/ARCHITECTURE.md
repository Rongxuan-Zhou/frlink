# Architecture reference

The reasoning behind the split, the code walk-through and the session procedure are in
[`GUIDE.md`](GUIDE.md), sections 2 to 4; the wire formats are frozen in
[`INTERFACE.md`](INTERFACE.md). This page keeps what those two do not spell out: the host
kernel and thread layout, the safety envelope as tables, and the control-channel details.

## Why two hosts

A torque-controlled FR3 needs a 1 kHz control loop that never misses a cycle; a teleoperation or
policy workstation needs a GPU, a browser, cameras and a current kernel. On the previous
single-host setup that combination produced 6 ms stalls on every core, and a kernel update broke
the NIC driver. So one small, pinned real-time host owns the robot, and interchangeable client
PCs talk to it over a direct cable through a frozen, minimal interface. GUIDE.md section 2.1 has
the jitter numbers.

```
 client PC (10.10.0.1)                              RT host (10.10.0.2)                 FR3
 --------------------                               -------------------                 ---
 02_webxr_to_franka.py ----- UDP 50001 ----------> cartesian_pose_servo -------------FCI-> 172.16.0.2
 /tmp/franka_*.txt <-- state mirror <-- UDP 50002 <-- FRST1 publisher (same process)  1 kHz
 franka-remote <verb> ---- ssh forced command ----> franka-ctl -> systemd / goto / gripper
 NIC 10.10.0.1/24 <======= direct cable ========> frlink0 10.10.0.2/24     enp110s0 172.16.0.1/24
```

## The three flows

All three cross the same cable. A change to any of them bumps the protocol tag
(`FRST1` -> `FRST2`). Formats, message names and rates: GUIDE.md section 2.2.

| flow | direction | transport | format |
|---|---|---|---|
| command | client -> host | UDP `10.10.0.2:50001` | 128 bytes = 16 little-endian float64 = column-major 4x4 `O_T_EE` (translation at indices 12, 13, 14) |
| state | host -> client | UDP `10.10.0.1:50002` | `FRST1 <seq> <epoch_ns> <t_real_ns> <t_mono_ns> <name>\n<body>` |
| control | client -> host | ssh forced command | `status`, `preflight`, `start pose`, `stop`, `restart pose [home]`, `goto home`, `gripper ...`, `echo`, `comm-test --yes` |

The command format predates the split and was kept so that existing senders work unchanged;
any other length is dropped and counted (`cmd_drop_size`). The impedance law (K_t 1000 N/m,
K_r 80 Nm/rad, both softened to 25 % on contact) uses the Franka Hand load model (0.25 kg,
COM z 0.05 m). These are compiled defaults; `franka-servo@pose` passes no task parameters.

In the state stream, `body` is byte-identical to what the servo writes into
`/tmp/franka/<name>` on the host, and `epoch_ns` is `CLOCK_REALTIME` at servo process start.

The control verbs arrive through an `authorized_keys` entry of the form
`command="/home/rongxuan_zhou/franka/bin/franka-ctl",no-pty,...,from="10.10.0.0/24"`; sshd
passes the verb in `SSH_ORIGINAL_COMMAND`. Token validation, exit codes and the robot lock are
in GUIDE.md section 3.1.

## Host kernel, CPU and thread layout

Kernel command line: `preempt=full isolcpus=domain,managed_irq,2,3 nohz_full=2,3 rcu_nocbs=2,3
irqaffinity=0-1,4-31 threadirqs`. CPUs 2 and 3 are removed from the scheduler's domains and
from the default IRQ mask. `franka-rt-tune` sets them to the performance governor, disables
C-states deeper than C1, and pins every `enp110s0-*` MSI-X vector to CPU 3 with its IRQ thread
at `SCHED_FIFO 85`.

The servo runs inside the `franka-rt:0.17.0-jazzy` container (`--privileged --network host
--ulimit rtprio=99 --ulimit memlock=-1`), started by the `franka-servo@pose` unit under
`systemd-inhibit` (no sleep, no lid action while it runs).

| thread | CPU | policy | role |
|---|---|---|---|
| `main` (libfranka `robot.control()`) | 2 | `SCHED_FIFO 99` (set by libfranka) | 1 kHz torque callback: reads the target from a triple buffer, computes the Cartesian impedance torque, publishes current EE / wrench / joint state into triple buffers |
| `udp_cmd` | 4-15 | `SCHED_OTHER` | `recvfrom` loop with 100 ms timeout: allowlist, size, latch; writes `target_raw` |
| `ee_writer`, `wrench_writer`, `joint_writer` | 4-15 | `SCHED_OTHER` | periodic file rewrite + FRST1 send, one formatting function for both |
| `init_resend` | 4-15 | `SCHED_OTHER` | re-sends `franka_init_pose.txt` every 1 s |
| `link_writer` | 4-15 | `SCHED_OTHER` | `franka_link.txt` at 5 Hz |

Order in `main()`: `mlockall(MCL_CURRENT|MCL_FUTURE)`, 8 MB stack prefault, set the aux CPU
mask, spawn the helpers (they inherit the mask), pin `main` to CPU 2, hand control to
libfranka. The only state shared with the tick is lock-free single-producer/single-consumer
triple buffers.

`missed_cycles_total` counts callback intervals longer than 1.5 ms; the acceptance gate is at
most 20 over a session. `tick_max_us_1s` on a healthy host sits near 1100-1140 us (the 1 kHz
period plus jitter).

The control-plane NIC (`frlink0`, a USB gigabit port) is not real-time: it carries only the
90 Hz command stream, the 20 Hz state stream and ssh. The robot NIC (`enp110s0`, RTL8126 with
`r8126`) is the only hard-real-time path, and nftables drops everything on it except traffic to
and from `172.16.0.2`.

## Safety envelope

These constants live in the servo source and were verified identical to the pre-split servo.

| mechanism | value | behaviour |
|---|---|---|
| Command hold | 200 ms without an accepted packet | target := current `O_T_EE`; torque control continues (the arm stays compliant, does not fall, does not drift). Cleared by the next accepted packet. |
| F/T emergency freeze | \|F\| > 35 N or \|T\| > 20 Nm on `O_F_ext_hat_K` | target frozen for 1500 ms, accepted packets are counted but dropped (`freeze=1` in `franka_link.txt`), then released |
| Torque cap | 0.85 x [87, 87, 87, 87, 12, 12, 12] Nm per joint, then `franka::limitRate` | hard clamp before the command leaves the callback |
| Reflex retry | up to 50 automatic recoveries | on a libfranka reflex (collision, joint limit) the servo calls `automaticErrorRecovery` and re-enters `robot.control()`; `recovering=1` between the two, `reflex_count` increments. Beyond 50 the process exits and the unit stops. |
| Robot collision thresholds | 30 N / 30 Nm (set by the servo on start) | lower than the software freeze, so a real collision normally shows up as a reflex first |
| Sender allowlist | `--cmd-allow` (servo) and the nftables `cmd_allow` set | datagrams from any other source IP never reach the servo (`cmd_drop_allow`) |
| Sender latch | first accepted `(ip, port)`; released after 1 s of silence | a second sender cannot inject targets while the first is alive (`cmd_drop_latch`); hand-over takes at most 1 s plus the hold |
| Servo precheck | `ExecStartPre` of the unit | refuses to start without `rt-tune.ok`, nftables loaded, AC power, package temp < 85 C, no other servo/goto/echo container, bind address configured, robot link up, image and binaries present |
| Mutual exclusion | `flock /run/lock/franka-robot.lock`; `goto`/`echo`/`comm-test` refused while a servo is active | one libfranka connection at a time |
| Thermal guard | 1 Hz timer, `/run/franka/temp_c` | `status` and precheck read it; the RUNBOOK's failure table lists the stall symptoms seen near 100 C |

`cartesian_pose_servo` has no workspace guard, no bounding box and no joint-limit check beyond
what the robot enforces; it follows the target it is given. Workspace safety is enforced one hop
earlier, in the client's bridge (`02_webxr_to_franka.py`):

| guard | value | effect |
|---|---|---|
| Workspace box (base frame) | x [0.150, 0.700] m, y [-0.550, 0.500] m, z [0.100, 0.656] m | the target translation is hard-clipped on all three axes; when the controller leaves the box the anchor is reset so the operator does not carry an offset back in |
| Step cap | 0.0055 m per frame at 90 Hz = 0.5 m/s | caps Cartesian speed at the ISO/TS 15066 collaborative limit; 0.5 m/s is about 0.7 rad/s at the joints, 35 % of the FR3 joint-velocity limit |
| Rotation scale | 0.5 | controller wrist angles map to half the angle on the robot; kills wrist-jitter amplification |
| Dead-man | Trigger held | the bridge sends a target only while the trigger is held; releasing it stops the stream and the servo holds within 200 ms |
| Gripper | Grip button toggles open / close | via `gripper_cmd` on the host (`franka-ctl gripper`), which may run next to the servo |

A different sender (a policy, a script) must implement the same box and step cap itself. The
user-stop button in the operator's hand is the last line.

## Client side

What the state mirror validates and how it writes the files is in GUIDE.md section 4.1. Two
details matter for the design. `skew_ms` in the mirror's status line (receive wall-clock time
minus the datagram's `t_real_ns`) is the only clock comparison in the system. The client
preflight flags a skew above 10 ms, because anything the client timestamps against the mirrored
state (logs, recordings) would otherwise be silently off. And the local `flock` on
`/tmp/franka_sender.lock` exists so that a second local sender fails loudly instead of being
silently dropped by the servo latch.
