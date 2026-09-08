# Quest / WebXR teleoperation

`bin/franka-teleop live` starts the pose servo on the RT host and then `02_webxr_to_franka.py`
on this PC. The bridge serves the WebXR page over HTTPS on port **4443** (the `teleop` pip
package, with the page from `frontend_swapped/`), receives controller poses at about 90 Hz,
applies the guardrails below and sends 128-byte `O_T_EE` targets to
`FRANKA_SERVO_HOST:50001`.

```
Quest (Meta Browser, WebXR) --HTTPS/WebSocket :4443--> this PC: 02_webxr_to_franka.py
                                                            |  UDP 50001, 128 B, ~90 Hz
                                                            v
                                                 RT host: cartesian_pose_servo (1 kHz)
```

## Reaching the bridge from the headset

The Quest must open `https://<this PC>:4443/`. Two working options:

1. **Same WiFi**: the bridge prints `https://<LAN IP>:4443/`. The certificate is self-signed
   (`cert.pem` inside the `teleop` package): open the URL in the Meta Browser, accept the
   warning once, then press **Enter VR**. Allow TCP 4443 in this PC's firewall.
2. **Tailscale Funnel** (Quest on another network): `tailscale funnel 4443` on this PC and put
   the resulting URL into `FRANKA_TELEOP_URL` in `config.env`; the bridge prints it. Funnel
   terminates TLS with a valid certificate, so no browser warning.

WebXR needs a secure origin, which is why the server is HTTPS only. Test the link without the
robot first: `$FRANKA_PY teleop/01_webxr_pose_reader.py` prints every callback (pose, buttons)
and needs nothing from the RT host.

## Controller map (`frontend_swapped/`)

| control | function |
|---|---|
| **Trigger** (index finger), held | dead-man: the robot follows the controller only while held. Releasing keeps sending the last target, so the arm holds its pose instead of drifting |
| **Grip** (middle finger), press | toggles the Franka Hand open / close (`bin/gripper_cmd` -> `franka-ctl gripper` on the host) |
| **A** | reserved; only used by `--calibrate` to sample the current EE into `/tmp/franka_bbox_points.json` |
| **B** | reserved (no function in this bridge) |

The stock `teleop` frontend has Grip = move and Trigger = gripper; the swap lives in
`frontend_swapped/index.html` (comment "Trigger <-> Grip swapped"). If the directory is
missing the bridge falls back to the stock page and says so.

## Mapping and guardrails (all set in `bin/franka-teleop`)

| parameter | default | meaning |
|---|---|---|
| `FRANKA_SCALE_TRANS` (`--scale`) | 1.0 | controller displacement to robot displacement. 1.0 = 1:1, comfortable when operating at chest height; 2.0 doubles Quest noise too |
| `FRANKA_SCALE_ROT` (`--rot-scale`) | 0.5 | controller rotation to EE rotation about the init pose; 0.5 halves wrist tremor |
| `FRANKA_MAX_STEP` (`--max-step`) | 0.0055 m | maximum translation per 90 Hz frame = 0.5 m/s. The target is clamped along its direction, never dropped, so the arm moves continuously. 0.02 m (1.8 m/s) tripped the FR3 joint-velocity limit |
| rotation step | 0.05 rad/frame (constant `MAX_ROT_STEP_RAD`) | rotation rate cap |
| `FRANKA_WS_{X,Y,Z}_{MIN,MAX}` (`--ws-*`) | the reference table box | axis-aligned workspace box in the robot base frame (metres). Targets are clipped to it and the WebXR anchor is reset, so the operator has to release and re-press Trigger at the edge |
| mirror mode | on (`--no-mirror` to disable) | the operator stands facing the robot: the target is rotated 180 degrees about the base z axis through the init pose, so "left" on the controller is "left" as seen by the operator |

The init pose is read from `/tmp/franka_init_pose.txt` (written by the state mirror once per
servo instance). The bridge refuses to start (exit 2) if that pose is outside the workspace
box, so bring the arm home first (`franka-remote goto home` or `franka-teleop restart`) or
widen the box.

Measuring your own box: `franka-teleop servo-only`, then
`$FRANKA_PY teleop/02_webxr_to_franka.py --live --calibrate --udp-host $FRANKA_SERVO_HOST`,
drive the arm to the four table corners and the highest useful point, press A at each, read
`/tmp/franka_bbox_points.json` and put the min/max (with a margin) into `config.env`.

## What the servo side does with the stream

- A sender that stops for more than 200 ms triggers the servo's **hold** (target := current
  pose, torque control continues). The bridge keeps sending after Trigger release for this
  reason.
- Only `FRANKA_CLIENT_IP` is accepted, and the first `(ip, port)` that sends is **latched**
  until it has been silent for 1 s. A second bridge on this PC exits 75 through
  `franka_sender_lock.py` instead of being silently ignored.
- The servo itself has no workspace guard: the box in this bridge is the only software limit
  before the robot's own joint / velocity limits. Keep the user-stop in reach.
