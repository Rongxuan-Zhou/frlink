English | [中文](README.zh-CN.md)

# Episode recording

`record_episodes.py` writes RealSense frames plus the robot state from `/tmp/franka_*.txt`
(the files `franka_state_mirror` keeps) to one HDF5 file per episode. It is task agnostic: no
commands, no knowledge of the task, no derived labels. Start teleoperation separately
(`franka-teleop live`), run the recorder in a terminal foreground, and drive episodes with
the keyboard (`s` start, `e` end + save, `d` discard last, `q` quit).

```
python3 record/state_reader.py --selftest --secs 3        # EE path alive? (ee_ok=60/60 expected)
python3 record/selftest.py 3                              # cameras only, 3 s -> /tmp/*.h5
python3 record/record_episodes.py --out-dir ~/datasets/pick --prefix pick [--auto-end-secs 30]
```

Environment: `requirements-record.txt` (pyrealsense2, h5py, numpy, opencv-python, PyYAML).
Cameras are declared in `config/cameras.yaml` (serial -> role); a role that is not plugged in
is skipped with a warning. `--cams role1,role2` records a subset.

## Camera profile

`cameras/recorder.py` applies the RECORD profile when a stream opens: auto exposure and white
balance converge for 60 frames, then exposure, gain and white balance are frozen for the
session and stored as file attributes. Contrast, gamma and saturation stay at sensor defaults
and brightness is 0, so no tone curve is baked into training images. Exposure is clamped to
60 % of the frame period (20 ms at 30 fps), gain compensates; otherwise a dark scene drops
below the target frame rate. `--exposure role=microseconds` locks a role by hand (when the
converged value overexposes a bright object). `cameras/grabber.py` is the preview / snapshot
variant; the recorder does not use it.

RealSense behaviour seen here. A D455 `hardware_reset()` on a host without the librealsense
udev rules can leave the device in `Protocol error` until the USB cable is re-plugged (hence
`hardware_reset_on_start: false` in the YAML). Rapidly restarting a pipeline can wedge a
camera the same way. One camera can only be opened by one process at a time.

## Dataset format

One file per episode, `<prefix>_epNN.h5` (numbering continues from the files already in the
output directory), gzip level 1, written in a background thread.

| path | shape / dtype | meaning |
|---|---|---|
| `observations/<role>/image` | `(T, H, W, 3)` uint8, BGR | colour frames of one camera, chunked per frame |
| `observations/<role>/hw_timestamp_ms` | `(T,)` float64 | RealSense hardware timestamp of each frame (ms; the camera's own clock, `frame.get_timestamp()`) |
| `observations/<role>/wall_timestamp_s` | `(T,)` float64 | this PC's wall clock (`time.time()`) when the frame was received |
| `observations/<role>` attrs | `serial`, `locked_exposure`, `locked_gain`, `locked_white_balance`, `locked_contrast`, `locked_gamma`, `locked_saturation`, `locked_brightness` | frozen sensor settings (-1 = not supported by that sensor) |
| `state/ee_pose` | `(N, 16)` float32 | `O_T_EE` column-major 4x4 (translation at 12, 13, 14) as published by the servo at 20 Hz |
| `state/ee_ok` | `(N,)` bool | validity of the sample (see below) |
| `state/_t` | `(N,)` float64 | this PC's wall clock (`time.time()`) at the poll |
| `state/wrench`, `state/wrench_ok` | `(N, 6)` float32, `(N,)` bool | `O_F_ext_hat_K` (Fx Fy Fz Tx Ty Tz), only with `--with-wrench` |
| `state/joint_state`, `state/joint_ok` | `(N, 28)` float32, `(N,)` bool | `q dq tau_J tau_ext_hat_filtered` (7 each), only with `--with-joints` |
| root attrs | `episode`, `duration_s`, `t_start_s`, `t_end_s`, `poll_hz`, `state_dir`, `ee_stale_ms`, `ee_ok_fraction`, `client_host`, `servo_host`, `config` | bookkeeping |

`T` differs per camera (own clock, nominally 30 fps); `N` = poll rate x duration (20 Hz by
default). Verified on the reference client in two-host mode. Nothing is resampled or aligned
in the file. Align afterwards on the wall clock (`wall_timestamp_s` per frame, `_t` per state
sample, same `time.time()` clock). `hw_timestamp_ms` finds dropped frames within one camera
(`dt` should be a constant 33.3 ms).

### `ee_ok`

`state_reader.py` reads `/tmp/franka_current_ee.txt` at every poll. `ee_ok = True` only if the
file's mtime is younger than 250 ms (five missed 20 Hz samples) and it holds exactly 16
floats. Otherwise `ee_pose` is all zeros and `ee_ok = False`: servo stopped, link silent or
mirror down, and that poll carries no robot information. Drop those samples when building
labels; do not interpolate across them. The recorder prints the `ee_ok` fraction per episode
and warns below 95 %.

### Clock synchronisation between client and host

Action labels are usually EE differences between consecutive samples
(`ee_pose[k+1] - ee_pose[k]`) paired with the nearest image. The EE sample is stamped on this
PC at the poll, but the RT host produced it on its own clock; the mirror's `skew_ms` (host
`t_real_ns` vs. client receive time) is the gap. At 20 Hz a 10 ms skew already misplaces a
sample by a fifth of a step, and the label with it. Keep both machines NTP-synchronised so
that `franka-client-preflight` reports `clock skew < 10 ms`; `/tmp/franka_mirror.txt`, field
`skew_ms`, shows the current value. Background: [`../../docs/GUIDE.md`](../../docs/GUIDE.md),
section 2.5.
