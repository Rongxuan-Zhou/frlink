# client — teleoperate and record a Franka FR3 through the RT host from your own Linux PC

This directory is everything a visiting PC needs to use the FR3 behind the RT host described
in [`../rt-host/`](../rt-host/README.md). The PC never talks to the robot directly: it sends
target poses over UDP, receives the robot state over UDP, and issues control verbs over an ssh
key that can only run `franka-ctl` (see [`../docs/INTERFACE.md`](../docs/INTERFACE.md)).

```
 your PC (10.10.0.1)                                            RT host (10.10.0.2)
 -------------------                                            -------------------
 bin/franka-teleop -> teleop/02_webxr_to_franka.py --UDP 50001--> cartesian_pose_servo (1 kHz)
 /tmp/franka_*.txt  <-- bin/franka_state_mirror  <--UDP 50002-- FRST1 state publisher
 record/record_episodes.py (cameras + /tmp/franka_*.txt -> HDF5)
 bin/franka-remote <verb> ------- ssh forced command ---------> franka-ctl (start/stop/goto/gripper)
 NIC $FRANKA_LINK_IFACE <============ direct cable ===========> frlink0
```

In scope: Quest/WebXR teleoperation of the arm and the Franka Hand, recording camera + robot
state episodes, robot data collection. Policy deployment and task-specific pipelines are not
part of this package.

## Requirements

- A Linux PC with a free Ethernet port, NetworkManager, systemd user sessions, Python 3.
- The RT host powered, its `frlink0` port free (one client at a time; the host is configured
  for exactly `10.10.0.1`).
- A Meta Quest for teleoperation; Intel RealSense cameras for recording (optional).
- The RT host admin: they append your public key to the host's `authorized_keys`.

## Quickstart

1. **Cable and address.** Plug a cable from your NIC into the host's client port. Fill in
   `config.env`:
   ```
   cp config.env.example config.env
   $EDITOR config.env        # FRANKA_CTL_USER, FRANKA_LINK_IFACE (ip -br link), FRANKA_PY
   ```
   Then create the link profile (printed again by `install.sh`):
   ```
   nmcli con add type ethernet ifname <iface> con-name franka-link ipv4.method manual \
       ipv4.addresses 10.10.0.1/24 ipv4.never-default yes ipv6.method disabled connection.autoconnect yes
   nmcli con up franka-link && ping -c 3 10.10.0.2
   ```
2. **Install.** `./install.sh` symlinks `bin/` into `~/.local/bin`, generates
   `~/.ssh/franka_ctl`, writes a `Host alienware-rt` block into `~/.ssh/config`, records the
   host key, installs and starts the `franka-state-mirror` user unit, and prints the
   `authorized_keys` line. Hand that line to the host admin. Re-run it whenever `config.env`
   changes.
3. **Python environments.** `pip install -r requirements-teleop.txt` into the interpreter
   named by `FRANKA_PY` (a venv or conda env); `pip install -r requirements-record.txt` for
   recording (same or another env). The mirror only uses the system `python3`.
4. **Preflight.** `franka-client-preflight` must end with `CLIENT PREFLIGHT PASS`: link,
   ssh channel (`franka-remote status`), mirror, clock skew, no competing sender. Run it at the
   start of every session.
5. **Robot on.** On the host's Desk (`https://172.16.0.2/desk/`, reachable from the host or
   the host admin): unlock the joints, **Activate FCI**, press the enabling button. The host
   admin does this; nothing on your PC can.
6. **Home, then teleoperate.**
   ```
   franka-remote goto home          # arm moves to the factory-ready pose (~10 s)
   franka-teleop live               # start the servo on the host, wait for state, start the bridge
   franka-teleop status             # bridge PID, mirror health, host units
   ```
   Open `https://<your PC>:4443/` on the Quest. **Stand directly in front of the robot, facing it,
   then press Enter VR** - the frame you calibrate there is the one every later motion uses, and you
   may then move around (even behind the arm on the same side) and keep operating. Hold Trigger to
   move, press Grip for the gripper. **When you finish, press Exit VR in the web page first**, then
   `franka-teleop stop`. Details, mapping and safety limits: [`teleop/README.md`](teleop/README.md).
7. **Record.** In a second terminal, with the teleop running:
   ```
   python3 record/state_reader.py --selftest --secs 3        # ee_ok 60/60
   python3 record/record_episodes.py --out-dir ~/datasets/<task> --prefix <task>
   ```
   `s` / `e` / `d` / `q` on the keyboard. Dataset layout: [`record/README.md`](record/README.md).
8. **Stop.** `franka-teleop stop` (bridge + servo). `franka-teleop restart` = stop, go home,
   relaunch (~60 s, the arm moves). Quit the recorder with `q`, never `kill -9` (the last
   episode is written in the background).

## Commands

| command | purpose |
|---|---|
| `franka-client-preflight` | PASS/WARN/FAIL checklist; exit 1 on any FAIL |
| `franka-remote <verb>` | `status`, `preflight`, `start pose`, `stop`, `restart pose [home]`, `goto home`, `gripper open\|close\|width w\|grasp w\|homing\|read`, `echo`, `comm-test --yes`. Exit codes pass through: 64 refused, 75 busy, 124 local timeout, 255 ssh/link |
| `franka-teleop home\|servo-only\|bridge-only\|live\|restart\|stop\|status\|wait-ready [s]` | session orchestrator; teleop parameters live in the script and in `config.env` |
| `franka_state_mirror --check` | mirror health (daemon alive, EE file fresh, 16 tokens); the daemon itself runs as the user unit |
| `bin/gripper_cmd`, `bin/goto_home`, `bin/echo_robot_state` | shim aliases (`franka-fci-shim`) that forward to `franka-ctl`; they exist so the bridge keeps the original call shape |
| `tests/fake_frst1.py --dst 127.0.0.1:50002` | local FRST1 generator to exercise the mirror without the host |
| `record/selftest.py [secs]` | cameras only, HDF5 + timestamp check |

Logs: `/tmp/franka_live.log` (bridge), `/tmp/franka_servo.log` (host verbs),
`journalctl --user -u franka-state-mirror`, `/tmp/franka_mirror.txt` (1 Hz status line:
`epoch seq src age_ms skew_ms rx drop_*`).

## Files the mirror writes (`/tmp`)

`franka_init_pose.txt` (16, once per servo instance; its mtime means "this servo started"),
`franka_current_ee.txt` (16, 20 Hz), `franka_wrench.txt` (6, 20 Hz), `franka_joint_state.txt`
(28, 10 Hz), `franka_link.txt` (13 `key=value`, 5 Hz: command age, packet rate, drop counters,
latched sender, missed cycles, freeze/recovering, tick statistics, reflex count). Files stop
advancing when the servo or the link is silent; nothing is deleted. Consumers apply their own
staleness rule (250 ms).

## Failure table

| symptom | check | action |
|---|---|---|
| `franka-remote` exits 255 | `ping 10.10.0.2`; `nmcli con show franka-link`; key at `~/.ssh/franka_ctl`; host key in `known_hosts` | cable / `nmcli con up franka-link`; ask the admin whether the key is authorized; `ssh-keyscan -t ed25519 10.10.0.2 >> ~/.ssh/known_hosts` |
| `franka-remote` exits 64 | the verb is not in the allowed set, or extra tokens | only the verbs listed above; no shell syntax |
| `franka-remote` exits 75 | `franka-remote status` | the host is busy (another verb running or a servo active); wait or `stop` |
| bridge says init pose timeout, `/tmp/franka_current_ee.txt` stale | `franka_state_mirror --check`; `systemctl --user status franka-state-mirror`; `franka-remote status` (servo active? `FRANKA_STATE_DST` on the host = your IP?) | restart the unit; the host's `/etc/franka/servo.env` must point `FRANKA_STATE_DST` at `10.10.0.1:50002` |
| arm holds while teleoperating; `franka_link.txt` `cmd_age_ms` > 200 | `cmd_drop_allow` rising -> your IP is not allowed; `cmd_drop_latch` rising -> another sender is latched; `cmd_pkts_last_s`=0 -> bridge dead | fix the sender; the hold is safe |
| second bridge/recorder exits 75 | `cat /tmp/franka_sender.lock` | stop the holder first |
| preflight clock skew WARN/FAIL | `timedatectl` on both machines | both must be NTP-synchronised; > 10 ms degrades recorded action labels |
| bridge exits 2 "EE not inside bbox" | the arm is not at a pose inside the workspace box | `franka-remote goto home` (or `franka-teleop restart`), or widen `FRANKA_WS_*` |
| Quest shows nothing at `https://<PC>:4443` | firewall, same network, certificate warning not accepted | open TCP 4443; accept the self-signed certificate; or use a Tailscale Funnel (`FRANKA_TELEOP_URL`) |
| stray `fake_frst1`/`state_sink`/second mirror in preflight | `ps`, `ss -lunp \| grep 5000` | kill it |
| `pkill -f <pattern>` killed your own shell | the pattern matched the `bash -c` command line | anchor patterns; the scripts here use `ps \| grep -v` for that reason |
| servo exits on the host (reflex, `TCP interrupted`) | `franka-remote status`; Desk shows an error | clear the error on Desk, re-activate FCI, `franka-teleop servo-only` (bridge keeps running) or `franka-teleop restart` |

## Layout

```
client/
  README.md                    this file
  config.env.example           copy to config.env (local, not committed)
  install.sh                   idempotent client setup
  requirements-teleop.txt      bridge environment (FRANKA_PY)
  requirements-record.txt      recorder environment
  bin/                         franka_state_mirror, franka-remote, franka-fci-shim (+ aliases),
                               franka-client-preflight, franka-teleop
  systemd/                     franka-state-mirror.service template (@ROOT@ substituted by install.sh)
  teleop/                      02_webxr_to_franka.py (bridge), 01_webxr_pose_reader.py (link test),
                               franka_sender_lock.py, frontend_swapped/ (Quest page), README.md
  record/                      record_episodes.py, selftest.py, state_reader.py, cameras/, config/cameras.yaml, README.md
  tests/fake_frst1.py          FRST1 generator for the mirror
```

Every script derives the repository root from its own location, so the directory can live
anywhere; only `config.env` is machine-specific.
