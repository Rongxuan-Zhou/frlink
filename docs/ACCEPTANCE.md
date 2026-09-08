English | [中文](ACCEPTANCE.zh-CN.md)

# Acceptance

Two robot days. Day 1 drives the arm from the RT host alone, with a test sender standing in for
the client: host, servo network side, safety envelope. Day 2 puts a real client on the cable:
headset teleop through the bridge, restarts, gripper, thermal soak, rollback.

The checklists as run are `rt-host/tests/robot_day1.md` and `rt-host/tests/robot_day2.md`; the
tools they call are in `rt-host/tests/`. Results stay on the host under
`tests/results/<date>/robot_day{1,2}/` and are not part of this repository. Before either day,
`rt-host/tests/rt_bench.sh` (6 checks) and `rt-host/tests/bench_net.sh` (5 checks) must pass on
the same kernel, driver and image that will run the robot.

For every step that moves the arm: **user stop in hand, workspace clear, one person at the
keyboard.** `comm-test` moves the arm to the factory-ready pose at speed 0.5 without asking
twice.

## Day 1: RT host alone

Unless marked otherwise every command runs on the RT host. `RES` is the results directory.

### 0. Pre-conditions
1. Today's `bench_net` and `rt_bench` results all PASS or SKIP.
2. Day-1 override in `/etc/franka/servo.env` so that the host can be its own client:
   `FRANKA_STATE_DST=10.10.0.2:50002`, `FRANKA_CMD_ALLOW=10.10.0.1,10.10.0.2,127.0.0.1`.
   Revert with `sudo install -m 0644 ~/franka/host/etc/franka/servo.env /etc/franka/servo.env` at the end.
3. AC power on; package temperature < 80 C idle.

### 1. Cabling and Desk
1. FCI cable on the host's `enp110s0`; `franka-fci` profile active with `172.16.0.1/24`.
2. Desk: unlock joints, Activate FCI, press the Enable button.
3. `franka-ctl status`: fci link up, both `franka-servo@*` inactive.

### 2. Preflight and raw link
1. `franka-ctl preflight` -> every line PASS (WARN only for nftables-without-sudo).
2. `sudo ping -i 0.001 -c 10000 172.16.0.2` -> 0 % loss, max < 0.5 ms.
3. Snapshot `/proc/interrupts` for `enp110s0-*` before and after -> only the CPU 3 column changes.

### 3. Communication test x3 and echo
1. `franka-ctl comm-test --yes` three times -> each `Avg >= 0.99`, no "lost robot states" line.
2. `franka-ctl echo` -> one JSON robot state.

### 4. Home and hold with the pose servo
1. `franka-ctl goto home` -> exit 0.
2. Start the sink on the host: `state_sink.py --bind 10.10.0.2 --mirror /tmp/franka_mirror --duration <s> --record-ee hold_ee.csv --gap-log hold_gaps.csv --report hold_sink.json`.
3. `franka-ctl start pose` -> unit active within 10 s; the mirror's `franka_init_pose.txt` has 16 tokens.
4. `consumer_check.py --secs 20 --bounds` -> PASS (see the note on check F in the results).
5. Hold with no sender (30 min in the full checklist, 5 min in the core run). Sample `franka_link.txt` every 5 min.
   Gate: `missed_cycles_total <= 20`, `max_consecutive_missed <= 3`, `reflex_count 0`, `freeze 0`, `cmd_age_ms -1`.
6. `track_error.py --ee hold_ee.csv` -> `drift_max <= 1.00 mm`.

### 5. Slow circle
1. `fake_sender.py --dst 10.10.0.2:50001 --src-ip 10.10.0.2 --src-port 40001 --mode circle --radius 0.03 --period 20 --duration <s> --center-from /tmp/franka_mirror/franka_current_ee.txt --log circle_cmd.csv`
   (the circle passes through the current EE at t = 0, so there is no jump; 9.4 mm/s).
2. While running: `cmd_pkts_last_s` 88-92, `latched_sender=10.10.0.2:40001`, `cmd_age_ms < 30`.
3. `track_error.py --ee hold_ee.csv --cmd circle_cmd.csv --start-skip 5` -> `err_max < 10 mm`, `reflex_count` still 0.

### 6. Command loss and latch hand-over
1. With a circle running, `kill -STOP` the sender -> the arm stops immediately; after 1 s `cmd_age_ms > 800`; `kill -CONT` -> resumes, `cmd_age_ms < 30` within 1 s. No reflex; EE moved < 2 mm during the stop.
2. `kill -9` the sender, start a second one on `--src-port 40002` -> `latched_sender` flips to `:40002` within 2 s.

### 7. F/T emergency by hand (full checklist; moved to the day-2 soak in the core run)
Push the holding EE steadily sideways. Either the software freeze (`freeze=1` for 1.5 s, then released) or a collision reflex (`reflex_count=1`, `recovering=1` then 0) is acceptable. The servo must still be active after 5 s, the sink must show no EE gap > 250 ms, and a new sender must regain control.

### 8. Control-plane cable pull (full checklist; moved to day 2 in the core run)
Pull the `frlink0` cable, not the FCI cable, for 10 s with a circle running. The servo stays active, no reflex, `missed_cycles_total` unchanged; `sendto()` failures in the publisher must not affect the tick.

### 9. Stop
`franka-ctl stop` -> exit 0, the journal ends with the `exited (tick=... udp=... total_reflex=0)` line, the unit is inactive within 3 s, no stray containers or libfranka processes. Revert the servo.env override.

## Day 1 results, 2026-09-08 (core run, 02:41-03:08 EDT)

Core subset: comm-test x3, home, 5-min hold, 3-min circle, command-loss hold, latch hand-over,
stop. Moved to the day-2 soak: 30-min hold, F/T push test, cable pull.

| step | result | numbers |
|---|---|---|
| Control-plane link `frlink0` (ASIX AX88179A, `ax88179_178a`) to the client NIC | PASS | ping 0 % loss, RTT avg 0.32 ms. The same port under the kernel's default `cdc_ncm` binding measured 1.48 ms; the udev rule `80-franka-ax88179.rules` forces the `ax88179_178a` driver for that reason. |
| `franka-ctl preflight` over ssh on 10.10.0.2 | PASS | rc = 0 |
| FCI link, 10 000 pings at 1 kHz | PASS | 0 % loss, RTT min/avg/max 0.047 / 0.122 / 0.481 ms; IRQ delta only on CPU 3 (20 552 interrupts) |
| comm-test x3 | PASS | Max / Avg / Min = 1.00 / 1.00 / 1.00 in each of the three runs |
| echo, goto home | PASS | JSON state received; goto rc = 0 |
| pose servo start | PASS | precheck: package 67 C, bind 10.10.0.2; init pose with 16 tokens in the mirror |
| `consumer_check --secs 20 --bounds` | A-E PASS, F FAIL | Check F is a table-height bound (z <= 0.20 m) inherited from a task-specific checklist; the arm was at factory-ready z = 0.49 m, so F cannot pass at home. Checklist inconsistency, not a system fault. |
| 5-min hold | PASS | drift_max 0.10 mm, drift_final 0.09 mm, position std (0.01, 0.01, 0.02) mm; `missed_cycles_total` 0, reflex 0, freeze 0, `tick_max_us_1s` 1119-1139 us; sink report PASS |
| circle r = 0.03 m, 20 s period, 180 s | link PASS, tracking marginal FAIL | link: `cmd_age_ms` 5, `cmd_pkts_last_s` 90, missed 0, reflex 0. tracking: err_max 11.44 mm, p95 9.95 mm, mean 8.28 mm against the 10 mm gate. |
| command-loss hold | PASS | STOP + 1 s: `cmd_age_ms` 849; + 3 s: 2851; arm holds, freeze 0, reflex 0. CONT + 1 s: `cmd_age_ms` 7 |
| latch hand-over | PASS | old sender killed, new sender on :40002 latched within 1.5 s (`cmd_drop_latch` 85 during the 1 s release window) |
| `franka-ctl stop` | PASS | rc = 0 in 0.30 s; log ends `exited (tick=659744 udp=19441 total_reflex=0)`; unit inactive |
| stray processes / containers after stop | PASS | none |

The 11.4 mm circle error is the steady-state compliance of the pose servo (K_t = 1000 N/m with
the Franka Hand load model, 0.25 kg) driving a 30 mm radius at z = 0.485 m. At that height
gravity-model error and the arm's own dynamics load the impedance spring. It is not transport latency:
`cmd_age_ms` stayed at 5 ms, and the same binary on the single-host setup behaves the same. The
10 mm gate predates any measurement of the compliance at that height. It stays in the checklist
as a regression bound; a future run should compare against this baseline, not the absolute
value. The impedance is soft on purpose: in teleoperation the operator closes the loop visually.

Temperature during the run: package 67-70 C baseline with 95-100 C turbo spikes from desktop
applications (single-core bursts; core temperatures 64-68 C). The servo precheck samples once,
so such a spike can refuse a start; keep GUI load off the host during sessions.

Fixes made during the run and now in the tree: `10-franka-link.link` matches the port by MAC
(the dock has two identical ports); `80-franka-ax88179.rules` binds `ax88179_178a`;
`franka-thermal-guard.service` gets `StartLimitIntervalSec=0` so the 1 Hz timer is not
rate-limited into a stale temperature file.

## Day 2: a real client on the cable

Not yet run at the time of this export. Procedure adapted from `rt-host/tests/robot_day2.md` to
the pose-servo path.

### 0. Client prerequisites
1. Client NIC has `10.10.0.1/24`; `ssh -i <key> rongxuan_zhou@10.10.0.2 status` prints the status block.
2. The client's state mirror is running: `/tmp/franka_current_ee.txt` mtime advances every 50 ms.
3. Host `/etc/franka/servo.env` is back at production values (`FRANKA_STATE_DST=10.10.0.1:50002`, `FRANKA_CMD_ALLOW=10.10.0.1`); `franka-ctl restart pose` and confirm the client mirror updates.
4. Desk: FCI active, Enable pressed. `franka-ctl preflight` PASS on the host, the client preflight PASS on the client. Package < 80 C.

### 1. 10-minute headset teleop
`franka-ctl goto home`, `franka-ctl start pose`; on the client `franka-teleop bridge-only` (or `franka-teleop live`, which also issues the two verbs). The bridge must print the init pose from the mirror within 5 s. Sample `franka-ctl status` at 1 Hz for 10 min.
Gate: reflex 0, freeze 0 unless a real collision, `missed_cycles_total <= 20`, `cmd_pkts_last_s` 85-92 while the trigger is held, dead-man release holds without drift, motion as smooth as on the single-host setup, box clipping visibly engages at the workspace edges without a jump.

### 2. Ten restarts
Ten times, trigger the client's restart path (`franka-teleop restart`, which runs `franka-remote restart pose home`: stop, goto factory-ready, start). Time each from `Stopping` to the new init-pose line in the host journal.
Gate: all ten <= 90 s with the home sequence (<= 15 s for a plain `restart pose`), the bridge reconnects every time, 0 reflex.

### 3. Twenty gripper commands
Ten `gripper close` / `gripper open` pairs through the client's wrapper while the servo is active, then ten more from the headset's Grip button. Gate: 20 x rc 0 on the wrapper path; `gripper read` shows width near max after open; no missed-cycle increase on the host while the gripper moves.

### 4. 60-minute thermal soak
Servo active, bridge idle (trigger released) or a client-side `fake_sender.py --mode hold`. Sample package temperature and `franka-ctl status` every 10 s.
Gate: package <= 85 C at every sample, `missed_cycles_total <= 20` over the hour, `max_consecutive_missed <= 3`, reflex 0, `tick_over_1p2ms_1s` 0 in >= 99 % of samples.
Mid-soak: the F/T push test from day 1 step 7, and a 10 s control-plane cable pull. The servo holds through the pull and the client mirror resumes within 2 s of replug.

### 5. Rollback rehearsal (timed)
`franka-ctl stop`, move the FCI cable back to the client, bring up the client's local servo in single-host mode (`franka-teleop servo-only` without the remote environment), confirm `/tmp/franka_init_pose.txt` appears from the local servo. Gate: under 15 minutes end to end. Decide the steady-state host; if staying on the RT host, move the cable back and redo step 0.
