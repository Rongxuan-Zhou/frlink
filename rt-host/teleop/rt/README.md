# teleop/rt — RT / network units for the FR3 torque servos

Header-only units linked by `cartesian_pose_servo.cpp` and
`tests/servo_net_stub.cpp`. Unit tests: `tests/test_rt_units.cpp` (+ `tests/rt_units/*.inc`).
Build everything with `teleop/build.sh` (runs inside the PART A image via `bin/franka-run`).

| header | contents |
|---|---|
| `clock.hpp` | `steady_now_ms`, `mono_now_ns`, `real_now_ns`, `process_epoch_ns` |
| `triple_buffer.hpp` | `TripleBuffer<T>` lock-free SPSC (writer `write`, reader `read`) |
| `tick_stats.hpp` | `TickStats::on_tick(now_ns)` callback-interval instrumentation |
| `state_pub.hpp` | `format_doubles`, `LinkStats::format`, `StatePublisher` (FRST1), `StateSink` |
| `udp_cmd.hpp` | `CommandSocket` (recvfrom+MSG_TRUNC), `CmdFilter`, `run_cmd_listener` |
| `rt_setup.hpp` | mlockall / stack prefault / affinity / `--check-rt` report |
| `cli.hpp` | `prescan_rt_args`: strips the flags below, leaves legacy argv untouched |
| `writers.hpp` | `run_periodic_emit`, `run_init_pose_resender`, `run_link_writer` |

## Servo flags added (both servos; legacy flags unchanged)

```
--cmd-bind <ip>          UDP command bind address (default 127.0.0.1; 0.0.0.0 allowed)
--cmd-port <port>        alias of --port (default 50001)
--cmd-allow <ip[,ip..]>  accepted source IPs (default 127.0.0.1)
--state-files 0|1        write franka_*.txt state files (default 1)
--state-dir <dir>        state file dir (default $FRANKA_STATE_DIR, else /tmp)
--state-dst <ip:port>    also publish every state body as a FRST1 UDP datagram (default none)
--rt-cpu <n>             pin the control thread (main) to cpu n after helpers are spawned
--aux-cpus <list>        helper threads' cpus, e.g. 4-15 or 4,5,6 (applied before spawning)
--check-rt               apply rt setup, print affinity/policy/mlock report, exit 0 (no robot)
--help                   usage, exit 0
```
Production (PART A `franka-servo-run`): `<servo> 172.16.0.2 <legacy args> --cmd-bind 10.10.0.2
--cmd-allow 10.10.0.1 --state-dst 10.10.0.1:50002 --rt-cpu 2 --aux-cpus 4-15 --state-files 1`
with `FRANKA_STATE_DIR=/tmp/franka`.

## Command input (unchanged wire format)

One UDP datagram = 128 bytes = 16 little-endian float64, column-major 4x4 `O_T_EE`.
Filter order per datagram: source IP in `--cmd-allow` (else `cmd_drop_allow`) -> length == 128
(else `cmd_drop_size`) -> sender latch. The first accepted `(ip, port)` is latched; datagrams from
any other source are dropped (`cmd_drop_latch`) until 1000 ms pass without a packet from the
latched sender (checked on every packet and every 100 ms recv timeout). Packets accepted while
the servo is frozen (F/T emergency, reflex recovery) are still counted as accepted but do not
reach the controller (legacy `g_freeze_drops`). The 200 ms hold (no packet -> target := current
EE) is unchanged.

## State output

Files in `--state-dir` (truncate-rewritten, byte-identical to the legacy `/tmp/*.txt`):
`franka_init_pose.txt` (16, once at start), `franka_current_ee.txt` (16, 20 Hz),
`franka_wrench.txt` (6, 20 Hz), `franka_joint_state.txt` (28, 10 Hz), `franka_link.txt` (5 Hz).
Numbers use default ostream formatting (6 significant digits), space separated, one `\n`.

With `--state-dst`, every file body is ALSO sent as one datagram:
```
FRST1 <seq> <epoch_ns> <t_real_ns> <t_mono_ns> <name>\n<body bytes>
```
`seq` = per-process counter starting at 0, +1 per datagram; `epoch_ns` = CLOCK_REALTIME at
process start; `t_real_ns`/`t_mono_ns` = CLOCK_REALTIME/CLOCK_MONOTONIC at send time; `name` =
one of the five file names. `franka_init_pose.txt` is additionally re-sent every 1000 ms
(datagram only; the file is written once).

`franka_link.txt` = one line of `key=value` tokens in this order:
`cmd_age_ms` (-1 before the first accepted packet, else ms since the last accepted one, never
clamped), `cmd_pkts_last_s`, `cmd_drop_size`, `cmd_drop_allow`, `cmd_drop_latch`,
`latched_sender` (`ip:port` or `none`), `missed_cycles_total` (callback interval > 1.5 ms counts
round(interval/1 ms)-1), `max_consecutive_missed`, `freeze` (0/1), `recovering` (0/1: between a
reflex and the next `robot.control()`), `tick_over_1p2ms_1s` and `tick_max_us_1s` (last completed
1 s window), `reflex_count`.

## Threads / RT layout

`main` runs `robot.control()`; libfranka itself makes it SCHED_FIFO/99 (control_tools.cpp:76).
main(): `mlockall(MCL_CURRENT|MCL_FUTURE)` + 8 MB stack prefault + `--aux-cpus` mask, THEN spawn
helpers (`udp_cmd`, `ee_writer`, `joint_writer`, `wrench_writer`, `init_resend`, `link_writer`,
all SCHED_OTHER, inheriting the aux mask), THEN pin main to `--rt-cpu`. The tick does no
allocation/I/O/locks for any PART B feature: `TripleBuffer` for `target_raw` (udp -> tick) and
for `current_ee/wrench/joint` (tick -> writers); the hold/emergency/recovery target overrides
are tick-local and cancelled by the next packet that reaches `target_raw`.

## Robot-free testing

```
teleop/build.sh                         # builds + runs tests/test_rt_units (236 checks)
tests/servo_net_stub --cmd-bind 127.0.0.1 --cmd-allow 127.0.0.1 --cmd-port 50001 \
    --state-dst 127.0.0.1:50002 --state-dir /tmp/stub --rt-cpu 2 --aux-cpus 4-15
<servo> --check-rt --rt-cpu 2 --aux-cpus 4-15   # rt report without touching the robot
```
