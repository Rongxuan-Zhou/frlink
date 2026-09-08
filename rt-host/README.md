# rt-host — the dedicated real-time host for the Franka FR3

This directory is the export of the RT host repository (`~/franka` on the machine
`alienware`). The host owns the robot: it holds the FCI cable, runs the 1 kHz torque
servo inside a Docker container on an isolated CPU, and exposes exactly three things to a
client PC over a direct Ethernet cable (see [`../docs/INTERFACE.md`](../docs/INTERFACE.md)):

| flow | direction | transport |
|---|---|---|
| target poses | client -> host | UDP `10.10.0.2:50001`, 128-byte `O_T_EE` datagrams |
| robot state | host -> client | UDP `10.10.0.1:50002`, `FRST1` datagrams |
| control verbs | client -> host | ssh forced command `franka-ctl` |

Nothing else runs on the host during a robot session. In particular it does not run the
teleop bridge, cameras, data collection or policy inference; those live on the client.

## What is in here

| path | contents |
|---|---|
| `bin/` | `franka-ctl` (the only control entry point, local CLI and ssh forced command), `franka-preflight`, `franka-run` (docker run wrapper with the RT-capable container config), `franka-servo-run` / `franka-servo-precheck` (ExecStart / ExecStartPre of the servo unit), `franka-py` |
| `host/` | `provision-rt.sh` (one-shot host provisioning), `verify-rt-host.sh` (post-reboot check), `franka-rt-tune.sh` (per-boot CPU/IRQ/NIC tuning), `franka-thermal-guard.sh`, `franka-docker-user.sh`, and `host/etc/` with every file installed under `/etc` (GRUB is edited by the script, not shipped) |
| `docker/` | `Dockerfile` for the `franka-rt:0.17.0-jazzy` image (Ubuntu 24.04 + ROS Jazzy pinocchio snapshot), `build-image.sh`, `build-inside.sh` (builds libfranka 0.17.0 and the teleop binaries inside the container), the ROS snapshot keyring |
| `teleop/` | the servo (`cartesian_pose_servo.cpp`, 6-DOF Cartesian impedance, Franka Hand load model), motion helpers (`goto_home.cpp`, `gripper_cmd.cpp`, `fk_probe.cpp`), `build.sh`, and `rt/` — the header-only RT/network units (UDP command filter, FRST1 publisher, triple buffer, tick statistics, RT setup). `teleop/rt/README.md` documents the flags and the wire format. `cartesian_pose_servo_pusht.cpp`, `pusht_pose_servo.cpp`, `goto_pose_pusht.cpp`: PushT variant, task-specific, not covered by this documentation. |
| `teleop/tests/` | `test_rt_units.cpp` + `rt_units/*.inc` (236 checks, run by `build.sh`), `servo_net_stub.cpp` (robot-free stand-in for the servo's network side) |
| `tests/` | `bench_net.sh`, `rt_bench.sh` (no-robot benches), `state_sink.py` (FRST1 receiver with gap gates), `fake_sender.py` (128-byte pose sender: hold / circle), `consumer_check.py`, `track_error.py`, and the robot-day checklists `robot_day1.md`, `robot_day2.md` |
| `franky_tools/` | franky-based calibration and wiggle scripts (payload calibration, joint wiggles). Direct-FCI tools: run only while no servo is active. |
| `docs/` | `INTERFACE.md` (the frozen contract), `RUNBOOK.md` (docking checklist, verbs, failure table), `ASSESSMENT-2026-09-02.md` (state of the host at hand-over) |

Not exported: compiled binaries, `tests/results/`, `libfranka/` (cloned at build time),
`local/` (build output), virtualenvs, and `keys/` (private keys never leave the host).

## Host facts the scripts assume

- Ubuntu 22.04, kernel `6.8.0-124-generic` pinned via apt holds and GRUB default.
- Kernel command line: `preempt=full isolcpus=domain,managed_irq,2,3 nohz_full=2,3 rcu_nocbs=2,3 irqaffinity=0-1,4-31 threadirqs nmi_watchdog=0 skew_tick=1`.
  CPU 2 runs the servo control thread; CPU 3 takes all robot-NIC IRQs (threads at FIFO 85).
- Robot NIC `enp110s0` (RTL8126, `r8126` DKMS, ASPM and EEE off), `172.16.0.1/24`, robot at `172.16.0.2`.
- Control-plane NIC `frlink0`: a USB ASIX AX88179A port renamed by MAC in
  `host/etc/systemd/network/10-franka-link.link`, `10.10.0.2/24`, direct cable to the client, no switch.
- Docker image `franka-rt:0.17.0-jazzy`; the source tree is bind-mounted at `/franka` and state
  files live in `/tmp/franka`.
- systemd template `franka-servo@<mode>`; the documented instance is `franka-servo@pose`, which runs
  `cartesian_pose_servo` with its compiled defaults (port 50001, alpha 0.05, K_t 1000 N/m,
  K_r 80 Nm/rad, load 0.25 kg / COM z 0.05 m = the Franka Hand). Network and CPU parameters come
  from `host/etc/franka/servo.env`. A `pusht` instance exists for the task-specific variant.
- The install path is hard-coded as `/home/rongxuan_zhou/franka` in the units and scripts (see Known gaps).

## Provisioning order

Run once on a fresh host, in this order. Each step has its own verification.

1. `sudo host/provision-rt.sh` — apt holds (kernel, nvidia), `realtime` + `docker` groups,
   `limits.d/99-realtime.conf`, GRUB default + RT cmdline, unattended-upgrades blacklist,
   `r8126` module options + initramfs, irqbalance off, logind lid/idle + masked sleep targets,
   sysctl, tmpfiles.
2. Reboot.
3. `host/verify-rt-host.sh` from a fresh login — expects `ALL PASS` (cmdline, isolated CPUs,
   kernel version, `ulimit -Hr 99`, groups, DKMS for r8126 and nvidia, sleep masked, tmpfiles).
4. `docker/build-image.sh` — builds `franka-rt:0.17.0-jazzy` and records `last-build.txt`.
5. `teleop/build.sh` — runs `docker/build-inside.sh` inside the image as the calling user:
   libfranka 0.17.0 into `local/`, the teleop targets (`cartesian_pose_servo`, `goto_home`,
   `gripper_cmd`, `fk_probe`, plus the PushT variants), the `kIgnore` communication test,
   the RT unit tests (must report 236 checks passed) and `tests/servo_net_stub`.
6. Install and enable `franka-rt-tune.service` (+ the NetworkManager dispatcher hook
   `90-franka-rt-tune`): governor/EPP performance and deep C-states off on CPUs 2-3, robot-NIC
   IRQs pinned to CPU 3, coalescing off. It writes `/run/franka/rt-tune.ok` only when every hard
   step passed; the servo precheck refuses to start without that file.
7. Install `host/etc/nftables.conf` and `systemctl enable --now nftables`; install
   `franka-docker-user.service` so Docker never forwards through the robot or client link.
   UDP 50001 is reachable only from `frlink0` (and `lo`), from the `cmd_allow` set.
8. Install `franka-servo@.service`, `franka-thermal-guard.{service,timer}` and
   `/etc/franka/servo.env`. The servo unit is intentionally not enabled at boot; it is only
   started through `franka-ctl start pose`.
9. `bin/franka-ctl preflight` — must print `PREFLIGHT PASS`. Then run the no-robot benches
   `tests/rt_bench.sh` and `tests/bench_net.sh`, and finally the robot checklists in `tests/`.

## `franka-ctl` verbs

`franka-ctl` is the only control entry point, both for local use and as the ssh forced command.
Everything that touches the robot takes `flock -n /run/lock/franka-robot.lock`; `gripper` has its
own lock and may run next to an active servo. Exit codes: the program's code is passed through,
`64` = usage / refused verb, `75` = busy (lock held or a servo is active).

| verb | what it does |
|---|---|
| `status` | units active/inactive, link/fci carrier, latest `franka_link.txt` counters, package temp (image presence is reported by `preflight`) |
| `preflight` | isolcpus, rtprio 99/memlock, IRQ affinity enp110s0-* = cpu3, NM profiles up, nftables loaded, timesyncd, image, robot ping |
| `start pose` | `systemctl start franka-servo@pose` -> docker run --privileged --network host --ulimit rtprio=99 ... `cartesian_pose_servo` with its compiled defaults |
| `stop` | `systemctl stop franka-servo@*` (SIGTERM, 10 s grace) |
| `restart pose [home]` | stop + start; with `home` the arm is driven to the factory-ready pose in between (this is what the client's restart path calls) |
| `goto home` | `goto_home` to the factory-ready pose at speed 0.10. Refuses if a servo is active. |
| `gripper open\|close\|width w\|grasp w\|homing\|read` | `gripper_cmd` on the Franka Hand; rc 0 ok, 4 franka::Exception; `read` prints JSON. May run while the servo is active (own lock). |
| `echo` | `echo_robot_state` once. Refuses if a servo is active. |
| `comm-test --yes` | `communication_test` (kIgnore build) with Enter fed; MOVES THE ARM to factory-ready at speed 0.5, then 10 s zero-torque; expect Avg >= 0.99 |

`start pusht`, `restart pusht [home]` and `goto pusht` are also accepted by the script: PushT
variant, task-specific, not covered here.

Over ssh, `franka-ctl` reads `SSH_ORIGINAL_COMMAND` and rejects any token outside
`[A-Za-z0-9._:=-]`, so a client cannot chain shell commands through the forced key.

## Authorizing a new client key

Every client PC gets its own ed25519 key pair; only the public half goes on the host. Append one
line to `~/.ssh/authorized_keys` on the host (user `rongxuan_zhou`):

```
command="/home/rongxuan_zhou/franka/bin/franka-ctl",no-port-forwarding,no-X11-forwarding,no-agent-forwarding,no-pty,from="10.10.0.0/24" ssh-ed25519 AAAA...your-public-key... client-name
```

- `command=` makes the key a forced command: whatever the client types after `ssh` becomes
  `SSH_ORIGINAL_COMMAND` and is parsed as a `franka-ctl` verb. The key cannot open a shell.
- `from="10.10.0.0/24"` restricts the key to the direct cable subnet. Add another CIDR only if
  you deliberately want admin access from elsewhere; the nftables input chain also only accepts
  TCP 22 on `frlink0` from `10.10.0.0/24`.
- Test from the client: `ssh -i <key> -o IdentitiesOnly=yes rongxuan_zhou@10.10.0.2 status`
  must print the status block; `ssh ... 'status; id'` must be refused with exit 64.

The key files under `keys/` on the host are private material and are not part of this export.

## One client at a time: why the visiting PC must be 10.10.0.1

The servo publishes state to exactly one destination and accepts commands from exactly one
source address. Both are fixed in `/etc/franka/servo.env`:

```
FRANKA_CMD_BIND=10.10.0.2
FRANKA_CMD_ALLOW=10.10.0.1
FRANKA_STATE_DST=10.10.0.1:50002
```

The nftables `cmd_allow` set holds the same single address. There is no discovery and no
multi-client fan-out: a visiting client PC plugs into the host's `frlink0` port, configures
`10.10.0.1/24` on its own NIC, and is then the client. Within that address, the servo latches
the first `(ip, port)` that sends a valid 128-byte datagram and drops every other sender until
the latched one has been silent for 1 s (`cmd_drop_latch` in `franka_link.txt` counts the drops).

If you must run a test client on the host itself (as the day-1 checklist does), override the
three variables for that session and restore the file afterwards; do not commit the override.

The pose servo takes no task parameters: `FRANKA_POSE_ARGS` in `servo.env` is empty and the
servo runs with port 50001, alpha 0.05, K_t 1000 N/m, K_r 80 Nm/rad and the Franka Hand load
model (0.25 kg, COM z 0.05 m). It sets the robot's collision behaviour to 30 N / 30 Nm and
freezes on 35 N / 20 Nm external force for 1500 ms.

## Known gaps

- Files that still contain Chinese comments and need an English pass (not translated in this
  export; the code is unchanged from the host):
  - `teleop/cartesian_pose_servo.cpp`, `teleop/cartesian_pose_servo_pusht.cpp`, `teleop/pusht_pose_servo.cpp`
  - `teleop/goto_home.cpp`, `teleop/goto_pose_pusht.cpp`, `teleop/gripper_cmd.cpp`, `teleop/fk_probe.cpp`
  - `franky_tools/*.py` (all eleven scripts)
  - `docs/ASSESSMENT-2026-09-02.md`
- The install prefix `/home/rongxuan_zhou/franka` is hard-coded in `bin/franka-ctl`,
  `bin/franka-run`, `bin/franka-servo-run`, `bin/franka-preflight`, `bin/franka-servo-precheck`,
  `host/provision-rt.sh` and the systemd units. Deploying under another user requires a
  search-and-replace or a `FRANKA_ROOT` variable (not done yet).
- `host/provision-rt.sh` embeds the host's root-filesystem UUID and the exact kernel version for
  the GRUB default entry; adapt both before running it on other hardware.
- The MAC address in `10-franka-link.link` and the udev rule `80-franka-ax88179.rules` are
  specific to the dock port used on this host.
- `teleop/notes/` (camera and workspace notes) and `teleop/scripts/` (client-side bridge scripts
  that historically lived on the host) were left out of this export on purpose; the client
  package supersedes them.
- The PushT sources (`cartesian_pose_servo_pusht.cpp`, `pusht_pose_servo.cpp`,
  `goto_pose_pusht.cpp`) and the `pusht` paths in `franka-ctl`, `franka-servo-run` and
  `servo.env` are exported unchanged but are task-specific and undocumented here.
