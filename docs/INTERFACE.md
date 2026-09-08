English | [中文](INTERFACE.zh-CN.md)

# FR3 two-host interface contract
Frozen 2026-09-02; any change bumps FRST1→FRST2 and the cmd_allow set.
2026-09-08: task-specific variants removed from this public copy; the interface itself is unchanged.

## 2. Two-host interface contract (frozen by this plan; the rog side implements against it)

### 2.1 Physical links and addresses
| Link | Interface on this host | Peer | Addresses | Purpose |
|---|---|---|---|---|
| A Robot link (hard real-time) | `enp110s0` (RTL8126, r8126) | FR3 Control | this host 172.16.0.1/24 ↔ robot 172.16.0.2 | FCI UDP 1 kHz + Desk HTTPS |
| B Control plane (non-real-time) | `frlink0` (USB dock port, ASIX AX88179A, MAC 00:0a:cd:48:1e:dd, driver ax88179_178a), direct cable to rog `eno1`, no switch | rog | private subnet 10.10.0.2/24 ↔ rog 10.10.0.1 | command / state / ssh / chrony |
| C Internet access | WiFi | campus network | DHCP | ssh/apt, never in the control path |

### 2.2 Command stream (rog → this host): UDP `10.10.0.2:50001`
- Payload: **128 bytes** = 16 × float64 little-endian, column-major 4x4 homogeneous transform `O_T_EE` (translation at indices 12,13,14), identical to the existing `struct.pack("16d", *T.flatten(order="F"))`.
- Any other length is dropped and counted. No sequence number, no checksum (kept for compatibility).
- Only source IPs in the `--allow-src` whitelist are accepted; the first valid sender is latched, packets from any other source are dropped and counted, and the latch is released after 1 s of silence.
- Semantics unchanged: > 200 ms without a packet → the servo locks the target to the current `O_T_EE` (hold; torque control continues).

### 2.3 State stream (this host → rog): UDP `10.10.0.1:50002`
Each datagram = one ASCII header line + the file body as-is (byte-identical to how the /tmp files are written; the rog-side mirror program just writes it to disk unchanged):
```
FRST1 <seq> <epoch> <t_real_ns> <t_mono_ns> <name>\n<file body>
```
- `name` ∈ {`franka_init_pose.txt` (once at startup, then re-sent every 1 s), `franka_current_ee.txt` (20 Hz), `franka_wrench.txt` (20 Hz), `franka_joint_state.txt` (10 Hz), `franka_link.txt` (5 Hz, new)}.
- `seq` increases monotonically within the process (uint64); `epoch` = `CLOCK_REALTIME` ns at servo process startup; when the rog mirror sees the epoch change → it immediately rewrites `franka_init_pose.txt` (fixes the stale reference pose problem after a B-key restart).
- **File body byte format (copied verbatim from the rog source, `teleop/cartesian_pose_servo.cpp:213-248`)**: plain `std::ofstream`, **no `setprecision`/`fixed` whatsoever**, i.e. `defaultfloat` with 6 significant digits (`%g` style; `e±0X` exponent outside `[1e-5,1e6)`); written as `f << v[i] << (i + 1 < N ? ' ' : '\n')`: single line, single-space separated, one trailing `\n`, no header, no trailing whitespace. N = 16 (init_pose / current_ee, column-major `O_T_EE`, translation at 12/13/14), 6 (wrench = `O_F_ext_hat_K`: Fx Fy Fz Tx Ty Tz), 28 (joint = q[7] dq[7] tau_J[7] tau_ext_hat_filtered[7]).
- rog-side consumers only count tokens via `f.read().split()` (16/6/28) + `float()`; the client state reader requires the EE file's **mtime to advance within 250 ms**, deploy/auto_collect require < 0.5 s; the 03 bridge waits at startup for `franka_init_pose.txt` for at most 5 s and **never re-reads it** within the process.
- `franka_link.txt` (new, 5 Hz): one line of space-separated `key=value`: `cmd_age_ms cmd_pkts_last_s cmd_drop_size cmd_drop_allow cmd_drop_latch latched_sender missed_cycles_total max_consecutive_missed freeze recovering tick_over_1p2ms_1s tick_max_us_1s reflex_count`.
- The file write and the network send must be produced by the **same formatting function** yielding the same string (byte identity guaranteed by construction).

### 2.3a Servo production parameters (copied from the launch scripts in the rog working tree)
- Regular teleoperation: `cartesian_pose_servo 172.16.0.2` (`launch_live.sh:39`, all defaults: port 50001, alpha 0.05, K_t 1000, K_r 80, load 0.25 kg / COM z 0.05).
- Home: `goto_home 172.16.0.2 --speed 0.10` (factory-ready joint configuration).
- Servo behavior constants unchanged: hold after 200 ms without a packet; F/T emergency freeze at |F|>35 N / |T|>20 Nm, 1500 ms; torque clamp 0.85×[87,87,87,87,12,12,12]; `franka::limitRate`; reflex retries ≤ 50.
- The **binaries running on rog come from the working tree, not HEAD** (the working tree additionally has: wrench/joint file writes, `--load-com-*`, `goto_home --q/--mode`).

### 2.4 Control verbs (rog → this host): ssh forced-command `franka-ctl`
Only the following are allowed: `status` · `preflight` · `start pose` · `stop` · `restart pose [home]` (default = stop → start, the arm is not moved; with `home` = stop → the return-home sequence → start, i.e. what the B key on rog does today) · `goto home` (rejected while the servo is running) · `gripper <argv...>` (forwarded to `gripper_cmd` on this host, identical to today's CLI) · `echo` (`echo_robot_state`, rejected while the servo is running) · `comm-test --yes` (kIgnore variant of `communication_test`; it first moves the arm to the factory pose, `--yes` is mandatory). All verbs that touch the robot share `flock -n /run/lock/franka-robot.lock` (exception: `gripper`, which may coexist with the servo and uses `/run/lock/franka-gripper.lock`). Exit codes: the program's exit code is passed through; 64 = usage / rejected verb; 75 = busy (lock held or servo running).

### 2.5 Minimal changes needed on the rog side (out of scope for this plan; listed only for alignment)
`franka_state_mirror` (receives on 50002 → atomically writes the four /tmp files; stops writing after 250 ms without packets); `gripper_cmd`/`goto_*`/`echo_robot_state` wrappers at the same paths (execute locally if `FRANKA_SERVO_HOST` is unset, otherwise ssh franka-ctl); the three senders read `FRANKA_SERVO_HOST`; the client launch script becomes an orchestrator.
