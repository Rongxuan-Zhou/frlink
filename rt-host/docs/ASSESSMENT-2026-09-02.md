# FR3 Control Stack Two-Host Split Assessment (2026-09-02)

> Historical document (hand-over assessment, kept as-is). It mentions a task-specific PushT servo variant and its client scripts; those were removed from this public copy on 2026-09-08 and are not part of the exported host. The interface is unchanged.

Question: can the Alienware m18 R2 (this machine) take over libfranka 1 kHz real-time control, with the rog desktop handling script execution / policy inference / data collection? Or is a different combination a better fit?

---

## 1. Current state of the two machines

| Item | rog (desktop, current full-stack host) | Alienware (this machine) |
|---|---|---|
| OS / kernel | Ubuntu 24.04, 6.17.0-23 `PREEMPT_DYNAMIC` (default voluntary) | Ubuntu 22.04.5, 6.8.0-124 `PREEMPT_DYNAMIC` (default voluntary) |
| Kernel tuning available | `preempt=full`/`isolcpus`/`nohz_full`/`irqaffinity`/`threadirqs` all available, HZ=1000 | Same |
| CPU | Ultra 9 285K: 8 P-cores (cpu0-7, no hyper-threading) + 16 E-cores | i9-14900HX: 8 P-cores with hyper-threading (cpu0-15, siblings 0-1/2-3…) + 16 E-cores (16-31) |
| GPU | RTX 5080 16 GB (nvidia 595 open) | RTX 4070 Laptop 8 GB (nvidia-dkms 580, hybrid mode) |
| Real-time permissions | `rtprio 99` + `memlock unlimited` (limits.d), user in the `realtime` group | `rtprio 0`, no realtime group (**needs sudo to configure**) |
| Power / idle | governor powersave, C3 exit latency 1048 µs | Same; on AC power; GNOME **lid close = suspend** |
| Robot NIC | `eno1` RTL8125, in-tree `r8169`, static 172.16.0.1/24 (`franka-fci`), **link currently DOWN** | Only wired port `enp110s0` RTL8126 (10ec:5000), **supported only by DKMS `r8126`**, currently the campus-network uplink |
| Second NIC | USB RTL8153 gigabit (campus network) | None (has TB4 and USB-C ports, an external adapter is possible) |
| Auto-updates | — | `unattended-upgrades` enabled → a kernel upgrade triggers r8126/nvidia DKMS rebuilds |
| Ubuntu Pro RT kernel | Not attached (RESUME.md P2 plan) | Not attached; the free personal tier can install `linux-realtime-hwe-22.04` (6.8 RT) |
| libfranka | 0.17.0 built from source into `~/franka/local` (pinocchio 3.9 from ros-jazzy, fmt 9, Poco 1.11) | None; needs a rebuild on 22.04 (fmt 8, `ros-humble-pinocchio` 3.5, `libpoco-dev`), or run a 24.04 userspace in Docker |
| franky | `.venv_franky` py3.13, franky-control 1.1.3 (wheel specific to libfranka-0.17.0) | Can install the cp310 wheel of the same version; it bundles libfranka, no system library needed |

Robot: FR3, System Image **5.7.2 locked** → libfranka must be in [0.15, 0.18), research-interface server v9.

## 2. Code architecture (rog `~/franka`, read-only survey)

- **Real-time executor** = C++ torque servo `teleop/cartesian_pose_servo.cpp` / `_pusht.cpp`: `robot.control(torque cb)` + hand-written Cartesian impedance PD (calls `franka::Model::coriolis/zeroJacobian` every tick; pinocchio is a hard runtime dependency), `franka::limitRate`, F/T emergency freeze (35 N / 20 Nm, 1.5 s), automatic reflex recovery (50 attempts), null-space joint-limit avoidance. All `RealtimeConfig::kIgnore`. The C++ has **no** `sched_setscheduler`/`mlockall`/CPU pinning; real-time priority relies entirely on the launch script `ulimit -r 99; chrt -f 99 taskset -c 0-3`.
- **Command entry point**: UDP **bound to 127.0.0.1:50001** (`INADDR_LOOPBACK`; only `--port`, no `--bind`); one packet = 128 bytes = 16 doubles, column-major 4x4 target pose; no sequence number/timestamp/checksum; >200 ms without a packet → target locks to the current EE (hold, control keeps running). Senders: WebXR bridge ~90 Hz (`--udp-host` configurable), `deploy_pusht_lewm.py` 10 Hz (**hard-coded** 127.0.0.1), `auto_collect/io_utils.py` 10 Hz (hard-coded). Single-sender is enforced by discipline only.
- **State feedback**: **no network channel**. The servo writes `/tmp/franka_init_pose.txt` (once at startup; the bridge waits 5 s, otherwise exits), `/tmp/franka_current_ee.txt` at 20 Hz, `_wrench.txt` at 20 Hz, `_joint_state.txt` at 10 Hz; six Python consumers rely on local files + mtime for liveness (250/500 ms thresholds).
- **Gripper**: the Python bridge directly spawns `teleop/gripper_cmd` (libfranka TCP to 172.16.0.2, hard-coded `LD_LIBRARY_PATH`); Quest B button → `launch_pusht.sh restart`, local pkill/relaunch.
- **Teleoperation input chain**: Quest 3 → Tailscale Funnel (bound to the rog node identity, because of campus WiFi client isolation) → uvicorn:4443 → WebSocket ~90 Hz → Python callback (mirroring/scaling/bbox/radial clipping/step clamping) → UDP.
- **Cameras**: 2×D455 + 1×D405 on rog USB; recording uses RealSense hardware timestamps + `time.time()`; EE state is polled from the /tmp file at 20 Hz and aligned by the same machine's wall clock.
- **Policy**: `deploy_pusht_lewm.py` runs LeWM/PRISM-MPPI on CUDA (0.3–0.7 s per call, 5 steps), in the same thread as the UDP sender; during planning the servo relies on the 200 ms hold. Needs a 5080-class GPU.
- **franky**: used only in the 11 standalone tools under `franky_tools/` (in-process libfranka, slow point-to-point, requires the servo to be off); **not a blocker for the split**. ROS 2 is not used by any control path.
- The servo source has uncommitted changes (+63/+115 lines: wrench/joint file writing, null-space posture term, 4-step return to home); the UDP protocol is byte-for-byte unchanged; the "UDP 6000" and the NaN return-to-home packet in `technical.md` are outdated documentation.

## 3. Measured: 1 kHz wake-up latency (`~/rt_probe/jit`, clock_nanosleep absolute timing, mlockall)

| Machine / condition | p50 | p99 | p99.9 | max | count >1 ms |
|---|---|---|---|---|---|
| rog FIFO80 P-core cpu1 (30 s) | 12 µs | 194 | 247 | **5903** | 6 |
| rog FIFO80 P-core cpu3 (20 s) | 8 | 19 | 33 | **6138** | 6 |
| rog FIFO80 P-core cpu7 (20 s) | 149 | 260 | 285 | **5679** | 5 |
| rog FIFO80 E-core cpu12 (30 s) | 3 | 4 | 95 | 215 | 0 |
| rog FIFO80 E-core cpu20 (20 s, two runs) | 3 | 6 | 149 | 225 / **5893** | 0 / 6 |
| Alienware OTHER (no rtprio) P-core cpu2 | 56 | 236 | 2660 | 2664 | 134 |
| **Alienware FIFO80 P-core cpu2 (sudo, 30 s)** | 6 | 9 | 87 | **416** | **0** |
| Alienware FIFO80 E-core cpu20 | 8 | 16 | 117 | 283 | 0 |
| Alienware FIFO80 unpinned | 8 | 10 | 109 | 747 | 0 |
| Alienware FIFO80 cpu2 + `cpu_dma_latency=0` | 1 | 4 | 18 | 533 | 0 |
| Alienware FIFO80 cpu2, **under load** (20 busy loops + dd, load 8, package 101 °C) | 1 | 15 | 197 | 575 | 0 |
| Alienware FIFO80 E-core cpu20, under load | 4 | 14 | 31 | 503 | 0 |

- Alienware's SMI count stayed at 0 throughout (MSR 0x34). hwlat trace for 40 s (right after the load test, about 100 °C): 4 hardware stalls >50 µs, max 671 µs, not SMI, likely thermal/frequency events.
- rog's ≈6 ms stalls **appear intermittently on any core** (once every 3~5 s), not P-core specific; without root, hwlat cannot be run and the SMI count cannot be read, so the cause is undetermined. Per the Franka docs, "the robot stops only after 20 consecutive lost packets"; 6 ms ≈ 6 packets, **not fatal but it lowers the success rate**; the only explanation for the `communication_test` Avg 1.00 recorded in May coexisting with this is that the stall was absent at the time, or the 10 s window did not catch it.
- Conclusion: **once given SCHED_FIFO, Alienware is cleaner than today's rog**; the earlier bad numbers in OTHER mode were entirely due to the missing real-time priority, with preemption by Claude Code/Docker and the like.

## 4. Measured: network between the two machines

| Path | Result |
|---|---|
| Campus wired, UDP echo @1 kHz, 20000 packets | 0 loss; RTT min 127 µs, p50 1045, p99 1738, max 2477 µs |
| Campus wired, UDP @100 Hz | p50 1612, max 3300 µs |
| ICMP 2000 packets | avg 1.45, max 2.19 ms |
| WiFi (wlp109s0f0) @100 Hz | p50 3.1 ms, **p99 40 ms, max 317 ms** → unusable |
| Tailscale | 2~51 ms jitter → unusable |
| ZeroTier | 175 ms → unusable |

Clocks on both machines: both run systemd-timesyncd against ntp.ubuntu.com, mutual offset ≈0.3 ms (±0.6 ms), far below the 20 Hz state sampling interval.

## 5. Option comparison

(To be filled in with the Workflow 2 results)

## 6. Recommendations and acceptance criteria

(To be added)
