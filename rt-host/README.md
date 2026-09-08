English | [中文](README.zh-CN.md)

# rt-host: the dedicated real-time host for the Franka FR3

Export of the RT host repository (`~/franka` on `alienware`). The host owns the robot: it
holds the FCI cable and runs the 1 kHz torque servo in a Docker container on an isolated CPU.
It exposes three things to a client PC over a direct Ethernet cable. Contract:
[`../docs/INTERFACE.md`](../docs/INTERFACE.md). Reasoning and servo internals:
[`../docs/GUIDE.md`](../docs/GUIDE.md), sections 2 and 3.

| flow | direction | transport |
|---|---|---|
| target poses | client -> host | UDP `10.10.0.2:50001`, 128-byte `O_T_EE` datagrams |
| robot state | host -> client | UDP `10.10.0.1:50002`, `FRST1` datagrams |
| control verbs | client -> host | ssh forced command `franka-ctl` |

Nothing else runs on the host during a session; the teleop bridge, cameras, data collection
and policy inference live on the client.

## What is in here

| path | contents |
|---|---|
| `bin/` | `franka-ctl` (the only control entry point), `franka-preflight`, `franka-run` (docker run wrapper with the RT container config), `franka-servo-run` / `franka-servo-precheck` (ExecStart / ExecStartPre of the servo unit), `franka-py` |
| `host/` | `provision-rt.sh`, `verify-rt-host.sh`, `franka-rt-tune.sh` (per-boot CPU/IRQ/NIC tuning), `franka-thermal-guard.sh`, `franka-docker-user.sh`, and `host/etc/` with every file installed under `/etc` (GRUB is edited by the script, not shipped) |
| `docker/` | `Dockerfile` for `franka-rt:0.17.0-jazzy` (Ubuntu 24.04 + ROS Jazzy pinocchio snapshot), `build-image.sh`, `build-inside.sh` (libfranka 0.17.0 and the teleop binaries, built in the container), the ROS snapshot keyring |
| `teleop/` | the servo (`cartesian_pose_servo.cpp`, 6-DOF Cartesian impedance, Franka Hand load model), motion helpers (`goto_home.cpp`, `gripper_cmd.cpp`, `fk_probe.cpp`), `build.sh`, and `rt/`, the header-only RT/network units; flags and wire format in `teleop/rt/README.md` |
| `teleop/tests/` | `test_rt_units.cpp` + `rt_units/*.inc` (236 checks, run by `build.sh`), `servo_net_stub.cpp` (the servo's network side without a robot) |
| `tests/` | `bench_net.sh`, `rt_bench.sh` (no-robot benches), `state_sink.py` (FRST1 receiver with gap gates), `fake_sender.py` (128-byte pose sender: hold / circle), `consumer_check.py`, `track_error.py`, and the robot-day checklists `robot_day1.md`, `robot_day2.md` |
| `franky_tools/` | franky scripts run through `bin/franka-py`: `goto_home.py` (what `franka-ctl goto home` runs), payload calibration, joint and gripper wiggles. Direct FCI: only while no servo is active. |
| `docs/` | `INTERFACE.md` (the frozen contract), `RUNBOOK.md` (docking checklist, failure table), `ASSESSMENT-2026-09-02.md` (state of the host at hand-over) |

Not exported: compiled binaries, `tests/results/`, `libfranka/` (cloned at build time),
`local/` (build output), virtualenvs, and `keys/` (private keys never leave the host).

## Host facts the scripts assume

- Ubuntu 22.04, kernel `6.8.0-124-generic` pinned via apt holds and GRUB default.
- Kernel command line: `preempt=full isolcpus=domain,managed_irq,2,3 nohz_full=2,3 rcu_nocbs=2,3 irqaffinity=0-1,4-31 threadirqs nmi_watchdog=0 skew_tick=1`.
  CPU 2 runs the servo control thread; CPU 3 takes all robot-NIC IRQs (threads at FIFO 85).
- Robot NIC `enp110s0` (RTL8126, `r8126` DKMS, ASPM and EEE off), `172.16.0.1/24`, robot at `172.16.0.2`.
- Control-plane NIC `frlink0`: a USB ASIX AX88179A port renamed by MAC in
  `host/etc/systemd/network/10-franka-link.link`, `10.10.0.2/24`, direct cable to the client, no switch.
- Docker image `franka-rt:0.17.0-jazzy`; the source tree is bind-mounted at `/franka`, state
  files live in `/tmp/franka`.
- systemd template `franka-servo@<mode>`; the documented instance is `franka-servo@pose`. It runs
  `cartesian_pose_servo` with its compiled defaults (port 50001, alpha 0.05, K_t 1000 N/m,
  K_r 80 Nm/rad, load 0.25 kg / COM z 0.05 m = the Franka Hand). Network and CPU parameters come
  from `host/etc/franka/servo.env`.
- The install path is hard-coded as `/home/rongxuan_zhou/franka` in the units and scripts (see Known gaps).

## Provisioning order

Run once on a fresh host, in this order. Each step has its own verification.

1. `sudo host/provision-rt.sh`: apt holds (kernel, nvidia), `realtime` + `docker` groups,
   `limits.d/99-realtime.conf`, GRUB default + RT cmdline, unattended-upgrades blacklist,
   `r8126` module options + initramfs, irqbalance off, logind lid/idle + masked sleep targets,
   sysctl, tmpfiles.
2. Reboot.
3. `host/verify-rt-host.sh` from a fresh login: expects `ALL PASS` (cmdline, isolated CPUs,
   kernel version, `ulimit -Hr 99`, groups, DKMS for r8126 and nvidia, sleep masked, tmpfiles).
4. `docker/build-image.sh`: builds `franka-rt:0.17.0-jazzy` and records `last-build.txt`.
5. `teleop/build.sh`: runs `docker/build-inside.sh` in the image as the calling user:
   libfranka 0.17.0 into `local/`, the teleop targets (`cartesian_pose_servo`, `goto_home`,
   `gripper_cmd`, `fk_probe`), the `kIgnore` communication test, the RT unit tests (must report
   236 checks passed) and `tests/servo_net_stub`.
6. Install and enable `franka-rt-tune.service` (+ the NetworkManager dispatcher hook
   `90-franka-rt-tune`): performance governor/EPP and deep C-states off on CPUs 2-3, robot-NIC
   IRQs on CPU 3, coalescing off. It writes `/run/franka/rt-tune.ok` only when every hard step
   passed; the servo precheck refuses to start without that file.
7. Install `host/etc/nftables.conf`, `systemctl enable --now nftables`, and
   `franka-docker-user.service` (Docker must never forward through the robot or client link).
   UDP 50001 is reachable only from `frlink0` (and `lo`), from the `cmd_allow` set.
8. Install `franka-servo@.service`, `franka-thermal-guard.{service,timer}` and
   `/etc/franka/servo.env`. The servo unit is not enabled at boot on purpose; only
   `franka-ctl start pose` starts it.
9. `bin/franka-ctl preflight` must print `PREFLIGHT PASS`. Then `tests/rt_bench.sh` and
   `tests/bench_net.sh` (no robot), then the robot checklists in `tests/`.

## `franka-ctl` verbs

`franka-ctl` is the only control entry point, locally and as the ssh forced command.
Everything that touches the robot takes `flock -n /run/lock/franka-robot.lock`; `gripper` has
its own lock and may run next to an active servo. Exit codes pass through; `64` = usage /
refused verb, `75` = busy (lock held or a servo active).

| verb | what it does |
|---|---|
| `status` | units active/inactive, link/fci carrier, latest `franka_link.txt` counters, package temp (image presence is reported by `preflight`) |
| `preflight` | isolcpus, rtprio 99/memlock, IRQ affinity enp110s0-* = cpu3, NM profiles up, nftables loaded, timesyncd, image, robot ping |
| `start pose` | `systemctl start franka-servo@pose` -> docker run --privileged --network host --ulimit rtprio=99 ... `cartesian_pose_servo` with its compiled defaults |
| `stop` | `systemctl stop franka-servo@*` (SIGTERM, 10 s grace) |
| `restart pose [home]` | stop + start; with `home` the arm is driven to the factory-ready pose in between (what the client's restart path calls) |
| `goto home` | `franky_tools/goto_home.py` (via `bin/franka-py`) to the factory-ready pose at speed 0.10: lifts to the table-clearance height first when starting near the table and FK-checks the joint path (`franky_helpers.path_min_z`) before moving. Refuses if a servo is active. |
| `gripper open\|close\|width w\|grasp w\|homing\|read` | `gripper_cmd` on the Franka Hand; rc 0 ok, 4 franka::Exception; `read` prints JSON. May run while the servo is active (own lock). |
| `echo` | `echo_robot_state` once. Refuses if a servo is active. |
| `comm-test --yes` | `communication_test` (kIgnore build) with Enter fed; MOVES THE ARM to factory-ready at speed 0.5, then 10 s zero-torque; expect Avg >= 0.99 |

Over ssh, `franka-ctl` reads `SSH_ORIGINAL_COMMAND` and rejects any token outside
`[A-Za-z0-9._:=-]`, so a client cannot chain shell commands through the forced key.

## Authorizing a new client key

Every client PC gets its own ed25519 key pair; only the public half goes on the host. Append
one line to `~/.ssh/authorized_keys` on the host (user `rongxuan_zhou`):

```
command="/home/rongxuan_zhou/franka/bin/franka-ctl",no-port-forwarding,no-X11-forwarding,no-agent-forwarding,no-pty,from="10.10.0.0/24" ssh-ed25519 AAAA...your-public-key... client-name
```

- `command=` makes the key a forced command: whatever follows `ssh` on the client becomes
  `SSH_ORIGINAL_COMMAND` and is parsed as a `franka-ctl` verb. The key cannot open a shell.
- `from="10.10.0.0/24"` restricts the key to the direct cable subnet. Add another CIDR only
  for deliberate admin access from elsewhere; nftables also accepts TCP 22 only on `frlink0`
  from `10.10.0.0/24`.
- Test from the client: `ssh -i <key> -o IdentitiesOnly=yes rongxuan_zhou@10.10.0.2 status`
  must print the status block; `ssh ... 'status; id'` must be refused with exit 64.

The key files under `keys/` on the host are private and not part of this export.

## One client at a time: the visiting PC must be 10.10.0.1

The servo publishes state to one destination and accepts commands from one source address,
both fixed in `/etc/franka/servo.env`:

```
FRANKA_CMD_BIND=10.10.0.2
FRANKA_CMD_ALLOW=10.10.0.1
FRANKA_STATE_DST=10.10.0.1:50002
```

The nftables `cmd_allow` set holds the same address. No discovery, no multi-client fan-out: a
visiting PC plugs into `frlink0`, sets `10.10.0.1/24` on its own NIC, and is the client. The
sender latch within that address is described in [`../docs/GUIDE.md`](../docs/GUIDE.md),
section 2.2. To run a test client on the host itself (the day-1 checklist does), override the
three variables for that session and restore the file afterwards; do not commit the override.

The pose servo takes no task parameters: `FRANKA_POSE_ARGS` in `servo.env` is empty and the
compiled defaults are the ones listed under Host facts. It sets the robot's collision
behaviour to 30 N / 30 Nm and freezes on 35 N / 20 Nm external force for 1500 ms.

## Known gaps

- The install prefix `/home/rongxuan_zhou/franka` is hard-coded in `bin/franka-ctl`,
  `bin/franka-run`, `bin/franka-servo-run`, `bin/franka-preflight`, `bin/franka-servo-precheck`,
  `host/provision-rt.sh` and the systemd units. Another user means a search-and-replace or a
  `FRANKA_ROOT` variable (not done yet).
- `host/provision-rt.sh` embeds the host's root-filesystem UUID and the exact kernel version for
  the GRUB default entry; adapt both before running it elsewhere.
- The MAC address in `10-franka-link.link` and the udev rule `80-franka-ax88179.rules` are
  specific to the dock port on this host.
- `teleop/notes/` and `teleop/scripts/` (client-side bridge scripts that used to live on the
  host) were left out on purpose; the client package supersedes them.
- The host repository also carries task-specific servo variants and their `franka-ctl` modes;
  they are not exported. Everything here is the regular 6-DOF pose servo with the Franka Hand,
  and `../docs/INTERFACE.md` is unchanged by that omission.
