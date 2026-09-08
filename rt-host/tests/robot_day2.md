# Robot day 2 — rog as the client (Quest teleop, recorder, deploy), thermal soak, rollback rehearsal
RES=~/franka/tests/results/$(date +%F)/robot_day2 (on alienware); rog commands are marked (rog) and are read/run only — nothing writes into rog's repo.

## rog prerequisites (out of this plan's scope — verify, do not implement here)
- [ ] (rog) `ip -4 addr show | grep 10.10.0.1/24` (franka-link on rog eno1; free because the FCI cable is on alienware).
- [ ] (rog) `ssh -i ~/.ssh/franka_ctl rongxuan_zhou@10.10.0.2 status` → franka-ctl status output (forced-command key from A7).
- [ ] (rog) `echo $FRANKA_SERVO_HOST` → 10.10.0.2 in the shell that launches bridge/deploy/auto_collect.
- [ ] (rog) state mirror running: `ls -la --time-style=full-iso /tmp/franka_current_ee.txt` mtime advancing (< 0.1 s old on two consecutive `ls`).
      Alienware: `/etc/franka/servo.env` back to FRANKA_STATE_DST=10.10.0.1:50002, FRANKA_CMD_ALLOW=10.10.0.1 (revert day-1: `sudo install -m 0644 ~/franka/host/etc/franka/servo.env /etc/franka/servo.env`), then `franka-ctl restart pose` and check the rog mirror updates.
- [ ] Desk: FCI active, Enable pressed. `franka-ctl preflight` PASS. Package temp < 80 C.

## 1. 10-min Quest teleop (20 min)
- [ ] `franka-ctl goto pusht` → 4-step home OK. `franka-ctl start pusht`.
- [ ] (rog) launch the bridge as launch_pusht.sh does (03_webxr_to_franka_pusht.py --live … --udp-host 10.10.0.2); it must print the init pose from the mirror within 5 s.
- [ ] Alienware: `for i in $(seq 600); do franka-ctl status | grep -E 'cmd_age_ms|missed|reflex|freeze'; sleep 1; done > $RES/teleop_link_samples.txt`
- [ ] Teleop 10 min. PASS: reflex_count 0, freeze 0 unless a real collision, missed_cycles_total ≤ 20, cmd_pkts_last_s ≈ 85–92 while moving, dead-man holds without drift, motion as smooth as on rog.

## 2. 10 B-button restarts (15 min)
- [ ] For i in 1..10: press B; rog bridge calls `ssh alienware-rt restart pusht home` (4-step home in between); on alienware `journalctl -u franka-servo@pusht.service -f -o short-precise` — time from `Stopping` to the new init-pose line. Record (i,seconds,ok) in $RES/restarts.csv.
      PASS: all 10 ≤ 90 s with the home sequence (≤ 15 s if rog calls plain `restart pusht`), the bridge reconnects each time, 0 reflex.

## 3. 20 gripper commands (10 min) — only with the gripper mounted; SKIP if the push-rod is mounted
- [ ] `for i in $(seq 10); do franka-ctl gripper close; echo rc=$?; franka-ctl gripper open; echo rc=$?; done | tee $RES/gripper.txt`
      PASS: 20× rc=0, `franka-ctl gripper read` JSON with width ≈ max_width after open; rc=4 → note the franka::Exception text.

## 4. 5 recorded episodes + assemble (30 min)
- [ ] (rog) `python scripts/collect_pusht_lewm.py --task pusht --auto-end-secs 20` ×5. Each episode `ee_ok` ratio ≥ 0.99. Record in $RES/episodes.txt.
- [ ] (rog) assemble step → 5 episodes, 0 frames dropped for staleness, exit 0.

## 5. 5 deploy rollouts (20 min)
- [ ] (rog) deploy_pusht_lewm.py ×5 with deploy_obs_publisher.py running. PASS: pre-check passes; no WATCHDOG pause; alienware link shows cmd_pkts_last_s ≈ 10; reflex_count 0. Record in $RES/deploy.txt.

## 6. 60-min thermal soak (65 min)
- [ ] Servo active with the rog bridge idle (dead-man) or a rog-side sender; alienware: `for i in $(seq 360); do echo "$(date +%s),$(sensors -j | python3 -c 'import json,sys;print(json.load(sys.stdin)["coretemp-isa-0000"]["Package id 0"]["temp1_input"])'),$(franka-ctl status | tr '\n' ' ')"; sleep 10; done > $RES/soak_temps.csv`
      PASS: package ≤ 85 C at every sample, missed_cycles_total ≤ 20 over the hour, max_consecutive_missed ≤ 3, reflex 0, tick_over_1p2ms_1s 0 in ≥ 99% of samples.
- [ ] Real link-loss test (once, mid-soak, bridge idle): pull the dongle→switch cable 10 s. Expected: rog consumers go stale, servo holds (cmd_age_ms grows), replug → rog mirror resumes within 2 s. PASS = servo active throughout, 0 reflex.

## 7. Rollback rehearsal (< 15 min, timed)
- [ ] t0=$(date +%s). `franka-ctl stop`. Move the FCI cable back to rog eno1.
- [ ] (rog) `nmcli con up franka-fci`; `ping -c 3 172.16.0.2` → 0 loss.
- [ ] (rog) `unset FRANKA_SERVO_HOST`, stop the rog mirror, `cd ~/franka/teleop && ./launch_pusht.sh servo-only` → `/tmp/franka_servo.log` shows the init pose; `python3 scripts/franka_pusht_state_fn.py --selftest --secs 5` OK.
- [ ] `echo "rollback_seconds=$(( $(date +%s) - t0 ))" | tee $RES/rollback.txt` → PASS < 900. Decide the steady-state host; if staying on alienware, move the cable back and re-run step 0.
- [ ] Fill $RES/robot_day2_report.md, commit.
