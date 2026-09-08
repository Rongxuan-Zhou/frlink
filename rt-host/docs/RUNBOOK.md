# FR3 on alienware: runbook

Checklists and failure table for the RT host. Verbs: [`../README.md`](../README.md#franka-ctl-verbs).
Why the setup looks like this: [`../../docs/GUIDE.md`](../../docs/GUIDE.md).

## Docking checklist (every session, 3 min)
1. AC plugged (`cat /sys/class/power_supply/AC*/online` -> 1). On battery the CPU frequency drops and cycles are missed.
2. Two cables: FCI Ethernet -> enp110s0 (built-in RJ45, r8126); USB dock port frlink0 (ASIX AX88179A, MAC 00:0a:cd:48:1e:dd) -> direct cable -> rog eno1 (private link 10.10.0.0/24, no switch). **Never swap them**: nftables only allows 172.16.0.2 on enp110s0 and the private subnet on the link.
3. Lid: closing is safe only if `systemctl is-enabled sleep.target suspend.target hibernate.target` -> masked for all three and `systemd-analyze cat-config systemd/logind.conf | grep HandleLidSwitch` -> ignore. Otherwise keep it open.
4. `~/franka/bin/franka-ctl preflight` -> exit 0. **Do not start a servo if any line is FAIL.**
5. Desk (https://172.16.0.2/desk/): unlock joints, Activate FCI, press Enable. Needed after EVERY robot boot.

## Daily start / stop
- Start: `franka-ctl preflight && franka-ctl goto home && franka-ctl start pose`. `franka-ctl status` must show the servo active and the cmd_age_ms/missed counters.
- rog side: mirror running (`/tmp/franka_current_ee.txt` mtime advancing), `FRANKA_SERVO_HOST=10.10.0.2`, then bridge / collect / deploy as before.
- Stop: `franka-ctl stop` (SIGTERM -> MotionFinished -> exit 0 within ~1 s). Then Desk -> lock joints if leaving.
- **Nothing else may talk to 172.16.0.2 while a servo runs** (no `echo`, no `goto`, no `comm-test`; the gripper TCP path is separate and allowed).

## Failure table
| symptom | likely cause | action |
|---|---|---|
| `preflight` FAIL "enp110s0 missing" / no 172.16.0.1 after a reboot | kernel update rebuilt/removed r8126 DKMS | `dkms status`; `sudo dkms autoinstall`; `sudo modprobe r8126`; if it will not build, boot the previous kernel from GRUB (`apt-mark showhold` should have prevented this). Fallback NIC: Intel I225 over Thunderbolt, re-point `franka-fci` (`nmcli con modify franka-fci connection.interface-name <new>`). |
| Arm freezes in place while teleoperating; `status` shows cmd_age_ms > 200 | hold engaged: no command packets (bridge dead-man, sender crashed, wrong FRANKA_SERVO_HOST, allowlist drop) | `status`: cmd_drop_allow rising -> sender IP not in CMD_ALLOW; cmd_drop_latch rising -> another sender holds the latch; cmd_pkts_last_s = 0 -> sender dead, restart it. The hold is safe; torque control continues. |
| rog scripts say EE file STALE / ee_ok false / recorder drops frames | rog mirror not running, link down, or state-dst wrong | alienware `status` -> publisher counters; `ip -br addr` on both ends; `ping 10.10.0.1`; restart the rog mirror; `ls --full-time /tmp/franka_current_ee.txt` on rog must advance every 50 ms. |
| comm-test Avg < 0.99 | NIC coalescing/IRQ on wrong CPU, thermal throttling, another RT load, switch in the FCI path | `ethtool -c enp110s0` (rx-usecs 0), `/proc/interrupts` enp110s0-* only cpu3, `sensors` < 85 C, `ps -eLo cls,rtprio,psr,comm | grep FF`, direct cable to the robot; retry 3 times; if still < 0.99 use the I225 NIC. |
| Servo log reflex repeatedly, or robot light red/yellow | collision or joint-limit reflex; user-stop pressed = needs Desk | Release user-stop, Desk -> acknowledge error, re-Activate FCI if it dropped; `franka-ctl stop`, `goto home`, `start pose`. Collision thresholds (30 N/30 Nm) are below the 35 N software freeze. |
| `status` median temp >= 85 C, missed cycles growing, tick_max_us_1s > 800 | thermal throttling / hardware stalls (0.3 to 0.7 ms hwlat events seen at ~100 C) | Stop GPU/CPU jobs on alienware, `docker stats` for stray containers, raise the laptop, check fans; consider `intel_pstate/no_turbo=1`; never train on this machine while it is the RT host. |
| New sender is ignored, cmd_drop_latch rises, latched_sender is an old ip:port | latch held by a dead/hung sender (releases 1 s after its last packet) | Kill the old sender on rog; wait 1 s; if it persists `franka-ctl restart pose`. |
| `start` fails: `Unable to find image 'franka-rt:0.17.0-jazzy'` | image pruned or Docker reinstalled | `docker images | grep franka-rt`; rebuild with `~/franka/docker/build-image.sh` (then `teleop/build.sh` for binaries). |
| `ulimit -r` prints 0 in a new shell | not in `realtime` group / limits.d missing / GUI session predates the change / Tailscale SSH (bypasses PAM, so limits.d never applies; a tmux server started from such a login inherits 0) | `groups`, `cat /etc/security/limits.d/99-realtime.conf`, reboot; log in via OpenSSH (port 22) or fix the current shell with `sudo prlimit --pid $$ --rtprio=99:99 --memlock=unlimited:unlimited`. Container servos are unaffected (--ulimit rtprio=99). |
| Link cable pulled / USB dock re-enumerated | NM dropped 10.10.0.2; the port lost its frlink0 name (MAC match in /etc/systemd/network/10-franka-link.link) | `nmcli con up franka-link`; `ip -br link`; if renamed, `nmcli con modify franka-link connection.interface-name <name>`. The FCI side is unaffected. |

## Where results live
`~/franka/tests/results/<date>/...` (bench_net, rt_bench, robot_day1, robot_day2). Re-run `tests/bench_net.sh` and `tests/rt_bench.sh` after any kernel, driver, Docker or servo change.
Quick RT sanity on the isolated core: `sudo taskset -c 2 cyclictest -m -p 80 -t 1 -i 1000 -l 30000 -q` (2026-09-03 baseline after isolation: Max 30 µs). `cyclictest -a 2` fails with EINVAL because isolcpus removes cpu2 from the inherited affinity mask.
