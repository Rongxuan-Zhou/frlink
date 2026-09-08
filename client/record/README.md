# Episode recording

`record_episodes.py` records RealSense frames plus the robot state that `franka_state_mirror`
writes to `/tmp/franka_*.txt` into one HDF5 file per episode. It is task agnostic: it does not
send commands, does not know what the robot is doing, and stores no derived labels. Start the
teleoperation separately (`franka-teleop live`), then run the recorder in a terminal
foreground and drive episodes with the keyboard (`s` start, `e` end + save, `d` discard last,
`q` quit).

```
python3 record/state_reader.py --selftest --secs 3        # EE path alive? (ee_ok=60/60 expected)
python3 record/selftest.py 3                              # cameras only, 3 s -> /tmp/*.h5
python3 record/record_episodes.py --out-dir ~/datasets/pick --prefix pick [--auto-end-secs 30]
```

Environment: `requirements-record.txt` (pyrealsense2, h5py, numpy, opencv-python, PyYAML).
Cameras are declared in `config/cameras.yaml` (serial -> role); a role that is not plugged in
is skipped with a warning. `--cams role1,role2` records a subset.

## Camera profile

`cameras/recorder.py` applies the RECORD profile when a stream opens: auto exposure and auto
white balance are allowed to converge for 60 frames, then exposure, gain and white balance
are **frozen** for the session and written into the file as attributes. Contrast, gamma and
saturation stay at the sensor defaults and brightness is 0, so no tone curve is baked into
training images. Exposure is clamped to 60 % of the frame period (20 ms at 30 fps) with gain
compensating, otherwise a dark scene drops below the target frame rate. `--exposure
role=microseconds` locks a role manually instead (useful when the converged value overexposes
a bright object).

`cameras/grabber.py` is the preview / snapshot variant (per-role options from the YAML,
optional depth); it is not used by the recorder.

Known RealSense behaviour worth knowing: a D455 `hardware_reset()` on a host without the
librealsense udev rules can leave the device in `Protocol error` until the USB cable is
re-plugged (`hardware_reset_on_start: false` in the YAML); rapidly restarting a pipeline can
wedge a camera the same way; one camera can only be opened by one process at a time.

## Dataset format

One file per episode, `<prefix>_epNN.h5` (numbering continues from the files already in the
output directory). Written with gzip level 1 in a background thread.

| path | shape / dtype | meaning |
|---|---|---|
| `observations/<role>/image` | `(T, H, W, 3)` uint8, **BGR** | colour frames of one camera, chunked per frame |
| `observations/<role>/hw_timestamp_ms` | `(T,)` float64 | RealSense hardware timestamp of each frame (ms; the camera's own clock, `frame.get_timestamp()`) |
| `observations/<role>/wall_timestamp_s` | `(T,)` float64 | this PC's wall clock (`time.time()`) when the frame was received |
| `observations/<role>` attrs | `serial`, `locked_exposure`, `locked_gain`, `locked_white_balance`, `locked_contrast`, `locked_gamma`, `locked_saturation`, `locked_brightness` | frozen sensor settings (-1 = not supported by that sensor) |
| `state/ee_pose` | `(N, 16)` float32 | `O_T_EE` column-major 4x4 (translation at 12, 13, 14) as published by the servo at 20 Hz |
| `state/ee_ok` | `(N,)` bool | validity of the sample (see below) |
| `state/_t` | `(N,)` float64 | this PC's wall clock (`time.time()`) at the poll |
| `state/wrench`, `state/wrench_ok` | `(N, 6)` float32, `(N,)` bool | `O_F_ext_hat_K` (Fx Fy Fz Tx Ty Tz), only with `--with-wrench` |
| `state/joint_state`, `state/joint_ok` | `(N, 28)` float32, `(N,)` bool | `q dq tau_J tau_ext_hat_filtered` (7 each), only with `--with-joints` |
| root attrs | `episode`, `duration_s`, `t_start_s`, `t_end_s`, `poll_hz`, `state_dir`, `ee_stale_ms`, `ee_ok_fraction`, `client_host`, `servo_host`, `config` | bookkeeping |

`T` differs per camera (each runs on its own clock, nominally 30 fps); `N` = poll rate x
duration (20 Hz by default). This is the layout verified on the reference client in two-host
mode: `observations/<role>/{image, hw_timestamp_ms, wall_timestamp_s}` and
`state/{ee_pose (N,16) float32, ee_ok (N,) bool, _t (N,) float64}` at 20 Hz. Nothing is
resampled or aligned in the file: align afterwards on the wall clock (`wall_timestamp_s` per
frame, `_t` per state sample, same `time.time()` clock), and use `hw_timestamp_ms` to detect
dropped frames within one camera (`dt` should be a constant 33.3 ms).

### `ee_ok`

`state_reader.py` reads `/tmp/franka_current_ee.txt` at every poll. The sample is `ee_ok =
True` only if the file's mtime is younger than **250 ms** (five missed 20 Hz samples) and it
holds exactly 16 floats. Otherwise `ee_pose` is all zeros and `ee_ok = False`: the servo
stopped, the link went silent or the mirror is down, and that poll carries no robot
information. Drop those samples (do not interpolate across them) when building labels; the
recorder prints the `ee_ok` fraction of every episode and warns below 95 %.

### Clock synchronisation between client and host

Action labels for imitation learning are usually EE differences between consecutive samples
(`ee_pose[k+1] - ee_pose[k]`) paired with the image nearest in time. The EE sample is stamped
on this PC when polled, but it was produced on the RT host at the servo's clock: the mirror's
`skew_ms` (host `t_real_ns` vs. client receive time) tells how far apart the two clocks are.
With a 20 Hz state stream a 10 ms skew already misplaces a sample by a fifth of a step, and
the pairing of frame and EE difference (hence the label) shifts with it. Keep both machines
NTP-synchronised so that `franka-client-preflight` reports `clock skew < 10 ms`; the mirror
status line (`/tmp/franka_mirror.txt`, field `skew_ms`) shows the current value.
