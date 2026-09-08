English | [中文](README.zh-CN.md)

# Quest / WebXR teleoperation

`bin/franka-teleop live` starts the pose servo on the RT host, then `02_webxr_to_franka.py`
on this PC. The bridge serves the WebXR page over HTTPS on port 4443 (`teleop` pip package,
page from `frontend_swapped/`) and takes controller poses at about 90 Hz. It applies the
guardrails below and sends 128-byte `O_T_EE` targets to `FRANKA_SERVO_HOST:50001`. Per-frame
math: [`../../docs/GUIDE.md`](../../docs/GUIDE.md), section 4.5.

```
Quest (Meta Browser, WebXR) --HTTPS/WebSocket :4443--> this PC: 02_webxr_to_franka.py
                                                            |  UDP 50001, 128 B, ~90 Hz
                                                            v
                                                 RT host: cartesian_pose_servo (1 kHz)
```

## Reaching the bridge from the headset

The Quest opens `https://<this PC>:4443/`. Two working options:

1. Same WiFi: the bridge prints `https://<LAN IP>:4443/`. The certificate is self-signed
   (`cert.pem` in the `teleop` package): accept the warning once in the Meta Browser, then
   press Enter VR. Allow TCP 4443 in this PC's firewall.
2. Tailscale Funnel (Quest on another network): `tailscale funnel 4443` on this PC, put the
   URL into `FRANKA_TELEOP_URL` in `config.env`, and the bridge prints it. Funnel has a
   valid certificate, so no warning.

WebXR needs a secure origin, hence HTTPS only. `$FRANKA_PY teleop/01_webxr_pose_reader.py`
prints every callback (pose, buttons) without the robot or the RT host.

### Operator procedure in the headset

1. **Stand directly in front of the robot, facing it, before you press Enter VR.** The session
   origin is captured then; every later controller motion is interpreted in that frame
   ("forward" = towards the robot as seen from that spot).
2. After that you may move around. The mapping stays in the calibrated frame: from the side
   or from behind the arm, it still moves in the directions you calibrated.
3. **When you are done, press "Exit VR" in the web page first**, then stop the bridge and the
   servo from the PC. A bridge killed under an open session leaves the headset streaming into
   nothing and confuses the next start.
4. Hold Trigger to move (dead-man), press Grip to toggle the gripper. Release Trigger before
   walking.

## Controller map (`frontend_swapped/`)

| control | function |
|---|---|
| Trigger (index finger), held | dead-man: the robot follows the controller only while held. Releasing keeps sending the last target, so the arm holds its pose instead of drifting |
| Grip (middle finger), press | toggles the Franka Hand open / close (`bin/gripper_cmd` -> `franka-ctl gripper` on the host) |
| A | reserved; only used by `--calibrate` to sample the current EE into `/tmp/franka_bbox_points.json` |
| B | reserved (no function in this bridge) |

The stock `teleop` frontend has Grip = move and Trigger = gripper; the swap is in
`frontend_swapped/index.html` (comment "Trigger <-> Grip swapped"). Without that directory
the bridge falls back to the stock page and says so.

## Mapping and guardrails (all set in `bin/franka-teleop`)

| parameter | default | meaning |
|---|---|---|
| `FRANKA_SCALE_TRANS` (`--scale`) | 1.0 | controller displacement to robot displacement. 1.0 = 1:1, comfortable at chest height; 2.0 doubles Quest noise too |
| `FRANKA_SCALE_ROT` (`--rot-scale`) | 0.5 | controller rotation to EE rotation about the init pose; 0.5 halves wrist tremor |
| `FRANKA_MAX_STEP` (`--max-step`) | 0.0055 m | max translation per 90 Hz frame (0.5 m/s). The target is clamped along its direction, never dropped, so the arm keeps moving. 0.02 m (1.8 m/s) tripped the FR3 joint-velocity limit |
| rotation step | 0.05 rad/frame (constant `MAX_ROT_STEP_RAD`) | rotation rate cap |
| `FRANKA_WS_{X,Y,Z}_{MIN,MAX}` (`--ws-*`) | the reference table box | axis-aligned workspace box in the robot base frame (metres). Targets are clipped to it and the WebXR anchor is reset, so the operator must release and re-press Trigger at the edge |
| mirror mode | on (`--no-mirror` to disable) | the operator faces the robot: the target is rotated 180 degrees about the base z axis through the init pose, so "left" on the controller is "left" as the operator sees it |

The init pose comes from `/tmp/franka_init_pose.txt` (written by the mirror once per servo
instance). If it lies outside the workspace box the bridge exits 2: run
`franka-remote goto home` (or `franka-teleop restart`) first, or widen the box.

To measure your own box: `franka-teleop servo-only`, then
`$FRANKA_PY teleop/02_webxr_to_franka.py --live --calibrate --udp-host $FRANKA_SERVO_HOST`.
Drive the arm to the four table corners and the highest useful point and press A at each.
Read `/tmp/franka_bbox_points.json` and put the min/max (with a margin) into `config.env`.

## What the servo side does with the stream

Servo-side guards (the 200 ms hold the Trigger row relies on, force/torque freeze, torque
cap, reflex recovery, the sender latch): [`../../docs/GUIDE.md`](../../docs/GUIDE.md),
section 2.3. Only `FRANKA_CLIENT_IP` is accepted; a second bridge on this PC exits 75 through
`franka_sender_lock.py` rather than being silently ignored.

The servo has no workspace guard; the box in this bridge is the only software limit before
the robot's own joint / velocity limits. **Keep the user-stop in reach.**
