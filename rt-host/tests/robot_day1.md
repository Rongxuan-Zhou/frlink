# Robot day 1 — FR3 driven from alienware (pose servo, fake_sender as the only client)
Safety: user-stop button in hand for every move; workspace clear (comm-test moves the arm at speed 0.5 to factory-ready).
Every command below runs on alienware unless marked (rog). RES=~/franka/tests/results/$(date +%F)/robot_day1 ; mkdir -p $RES

## 0. Pre-conditions (10 min)
- [ ] C4 bench_net summary all PASS/SKIP and C5 rt_bench all PASS today (attach paths).
- [ ] Day-1 servo overrides (state to alienware, allow alienware as sender), via A6's env file:
      sudo sed -i 's/^FRANKA_STATE_DST=.*/FRANKA_STATE_DST=10.10.0.2:50002/; s/^FRANKA_CMD_ALLOW=.*/FRANKA_CMD_ALLOW=10.10.0.1,10.10.0.2,127.0.0.1/' /etc/franka/servo.env
      If franka-link is not up yet, substitute 127.0.0.1 for 10.10.0.2 everywhere in this checklist, and also
      run `sudo sed -i 's/^FRANKA_CMD_BIND=.*/FRANKA_CMD_BIND=127.0.0.1/' /etc/franka/servo.env` so the bind
      address follows the substitution.
- [ ] AC power connected; `cat /sys/class/power_supply/AC*/online` → 1. Lid may stay open; do NOT close it unless
      `systemctl is-enabled sleep.target suspend.target` prints masked masked.
- [ ] `sensors -j | python3 -c 'import json,sys;print(json.load(sys.stdin)["coretemp-isa-0000"]["Package id 0"]["temp1_input"])'` → < 80 (idle).

## 1. Cabling + Desk (10 min)
- [ ] Move the FCI Ethernet cable from rog eno1 to alienware enp110s0. `nmcli -g GENERAL.STATE con show franka-fci` → activated;
      `ip -4 addr show enp110s0 | grep 172.16.0.1/24`.
- [ ] Desk https://172.16.0.2/desk/ → unlock joints, then Activate FCI; press the physical Enable button.
- [ ] `~/franka/bin/franka-ctl status` → fci link up, franka-servo@* inactive (image presence is checked by `preflight` in §2).

## 2. Preflight + raw link (10 min)
- [ ] `~/franka/bin/franka-ctl preflight | tee $RES/preflight.txt` → every line OK. PASS = exit 0.
- [ ] `sudo ping -i 0.001 -c 10000 172.16.0.2 | tail -3 | tee $RES/ping_fci.txt` → `0% packet loss`, rtt max < 0.500 ms. (rog reference 0.16 ms avg.)
      FAIL → stop here: r8126 path is the biggest risk; check `ethtool -S enp110s0 | grep -i err`, `ethtool -c enp110s0` (rx-usecs 0), fall back to an Intel I225 Thunderbolt NIC.
- [ ] `grep enp110s0- /proc/interrupts > $RES/irq0.txt; sleep 30; grep enp110s0- /proc/interrupts > $RES/irq1.txt` during the ping → only the cpu3 column changes (`diff`). PASS = no other column changed.

## 3. communication_test ×3 (15 min)  — MOVES THE ARM to factory-ready at speed 0.5 first
- [ ] Clear the workspace, hand on user-stop. `~/franka/bin/franka-ctl comm-test --yes | tee $RES/commtest_1.txt`.
      Expected: `#100 Current success rate: 1.00` … `#10000`, then `Max: 1.00 Avg: 1.00 Min: ≥0.98` and NO "lost robot states" line.
      PASS = Avg ≥ 0.99 and no `WARNING`. Repeat → commtest_2.txt, commtest_3.txt. All three must pass. If Avg < 0.99 see RUNBOOK "comm-test Avg < 0.99".
- [ ] `~/franka/bin/franka-ctl echo | tee $RES/echo.json` → one JSON robot state (O_T_EE, q, robot_mode). Never run echo while a servo is active.

## 4. Home + 30-min hold with the pose servo (45 min)
- [ ] `~/franka/bin/franka-ctl goto home` → `✅ at [factory ready]`, exit 0.
- [ ] Sink on alienware bound to the link IP: `python3 ~/franka/tests/state_sink.py --bind 10.10.0.2 --mirror /tmp/franka_mirror --duration 1860 --record-ee $RES/hold_ee.csv --gap-log $RES/hold_gaps.csv --report $RES/hold_sink.json > $RES/hold_sink.log 2>&1 &`
- [ ] `~/franka/bin/franka-ctl start pose` → `systemctl is-active franka-servo@pose.service` = active within 10 s; `journalctl -u franka-servo@pose.service -n 20` shows the init pose line and `UDP listener`. `cat /tmp/franka_mirror/franka_init_pose.txt` has 16 tokens.
- [ ] `python3 ~/franka/tests/consumer_check.py --secs 20 --bounds` → RESULT PASS.
- [ ] Wait 30 min (no sender). Every 5 min: `cat /tmp/franka_mirror/franka_link.txt`. Record the last line in the report.
      PASS at end: missed_cycles_total ≤ 20, max_consecutive_missed ≤ 3, reflex_count 0, freeze 0, tick_over_1p2ms_1s 0 in most samples, cmd_age_ms = -1 (no sender ever).
- [ ] Drift: `python3 ~/franka/tests/track_error.py --ee $RES/hold_ee.csv` → `drift_max ≤ 1.00 mm` → RESULT PASS. Keep the sink running.
- [ ] Temperature every 5 min in the report; package ≤ 85 C expected while holding.

## 5. Slow circle 10 min (15 min)
- [ ] `python3 ~/franka/tests/fake_sender.py --dst 10.10.0.2:50001 --src-ip 10.10.0.2 --src-port 40001 --mode circle --radius 0.03 --period 20 --duration 600 --center-from /tmp/franka_mirror/franka_current_ee.txt --log $RES/circle_cmd.csv | tee $RES/circle_sender.log`
      (circle passes through the current EE at t=0, R copied from the robot → no jump; speed 9.4 mm/s). Watch: smooth 6-cm-diameter circle.
- [ ] While running: `franka_link.txt` shows cmd_pkts_last_s 88–92, latched_sender=10.10.0.2:40001, cmd_age_ms < 30.
- [ ] After: `python3 ~/franka/tests/track_error.py --ee $RES/hold_ee.csv --cmd $RES/circle_cmd.csv --start-skip 5` → err_max < 10.00 mm → PASS; reflex_count still 0.

## 6. Command-loss / hold behaviour (10 min)
- [ ] Start a 120 s circle (`--duration 120`). After 20 s: `kill -STOP $(pgrep -f fake_sender.py)`; arm stops within a blink.
      `sleep 1; cat /tmp/franka_mirror/franka_link.txt` → cmd_age_ms > 800. `kill -CONT ...` → arm resumes; cmd_age_ms < 30 within 1 s.
      PASS = both observed, no reflex, EE moved < 2 mm during the 3-s stop (hold_ee.csv rows around that time).
- [ ] Latch hand-over: `kill -9 $(pgrep -f fake_sender.py)`; start a second sender with `--src-port 40002` → latched_sender flips to :40002 within 2 s → PASS.

## 7. F/T emergency by hand (10 min)
- [ ] With the arm holding (no sender), push the EE steadily sideways. Two acceptable outcomes:
      (a) software freeze: journal `🚨 F/T emergency: |F|=3x N ... freeze 1500ms` then `🟢 emergency hold released`; link freeze=1 for 1.5 s, then 0.
      (b) robot collision reflex first (collision thresholds 30 N < 35 N): journal reflex/recovered lines; link reflex_count=1, recovering=1 then 0.
      PASS = servo still active after 5 s, sink shows no EE gap > 250 ms, a new constant sender regains control (cmd_age_ms < 30). Record which outcome in $RES/ft_test.txt.

## 8. Private-link cable pull (5 min) — pull the USB-C dongle cable, NOT the FCI cable
- [ ] With a circle running (both ends on alienware): unplug the dongle→switch cable for 10 s, replug.
      PASS = franka-servo@pose stays active, no reflex, missed_cycles_total unchanged (±1); publisher sendto() failures must not crash the servo. Record in $RES/link_pull.txt.

## 9. Stop + logs (5 min)
- [ ] `~/franka/bin/franka-ctl stop` → journal ends with `✅ exited (tick=... udp=... total_reflex=0)`; `systemctl show -p ExecMainStatus franka-servo@pose.service` → 0; stop → inactive ≤ 3 s.
- [ ] `journalctl -u 'franka-servo@*' --since today > $RES/journal_servo.txt`; final link line → $RES/link_final.txt; sink report attached.
- [ ] Revert the day-1 env overrides (step 0). Fill robot_day1_report.md, commit.
