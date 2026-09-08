"""Dataset-grade multi-camera recording pipeline (imitation-learning data collection).

Design points (what matters for image -> action learning):
  - RECORD profile: let auto exposure converge, then freeze exposure / gain / white balance.
    The session adapts to the current lighting and stays constant within the session (no
    frame-to-frame brightness drift), without hard-coding an exposure value.
  - Neutral colour: contrast / gamma / saturation at the sensor defaults, brightness 0, so no
    non-linear tone curve is baked into the training data (object edges and contact shadows
    keep their gradients).
  - Every frame carries the **hardware timestamp** frame.get_timestamp() (ms) and the client
    wall clock, for aligning several cameras and the robot state afterwards.
  - Robot state plug-in: attach_state_fn(callable) -> dict. While teleop runs, the state
    reader feeds EE pose / ok flag at poll time; without it the recorder is camera-only.

HDF5 layout is documented in record/README.md.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

import numpy as np
import pyrealsense2 as rs

# Neutral profile (measured factory defaults of the D455 / D405)
NEUTRAL = {
    "contrast": 50.0,
    "gamma": 300.0,
    "saturation": 64.0,
    "brightness": 0.0,
}
AE_CONVERGE_FRAMES = 60   # frames observed while auto exposure converges


def _color_sensor(dev: "rs.device") -> "rs.sensor":
    """D455 -> 'RGB Camera'; D405 -> 'Stereo Module'. Fall back to whichever sensor supports exposure."""
    for s in dev.query_sensors():
        nm = s.get_info(rs.camera_info.name)
        if "RGB" in nm or "Color" in nm:
            return s
    # D405 and similar: colour comes from the sensor that exposes the exposure option
    for s in dev.query_sensors():
        if s.supports(rs.option.exposure):
            return s
    raise RuntimeError("no colour / exposure-capable sensor found")


def _set(s: "rs.sensor", opt, val) -> None:
    try:
        if s.supports(opt):
            s.set_option(opt, float(val))
    except Exception:
        pass


@dataclass
class _Cam:
    role: str
    serial: str
    pipeline: "rs.pipeline"
    sensor: "rs.sensor"
    width: int
    height: int
    locked: dict = field(default_factory=dict)
    frames: list = field(default_factory=list)   # [(hw_ts_ms, wall_ts_s, np.ndarray BGR)]
    last_color: Optional[np.ndarray] = None
    last_hw_ts: float = 0.0
    n_grab: int = 0
    lock: threading.Lock = field(default_factory=threading.Lock)


class MultiCamRecorder:
    """Dataset-grade multi-camera recorder with the RECORD profile (neutral + frozen AE).

    Usage:
        rec = MultiCamRecorder([("wrist_d455", "239622301879"),
                                 ("wrist_d405", "335122270512")])
        rec.start()                       # open streams, converge AE, freeze
        rec.attach_state_fn(get_state)    # optional robot-state hook
        rec.begin_episode()
        ... operate ...
        rec.end_episode()
        rec.write_hdf5("demo_0001.h5")
        rec.stop()
    """

    def __init__(self, cams: list[tuple[str, str]],
                 width: int = 640, height: int = 480, fps: int = 30) -> None:
        self.spec = cams
        self.w, self.h, self.fps = width, height, fps
        self._cams: dict[str, _Cam] = {}
        self._threads: list[threading.Thread] = []
        self._running = False
        self._recording = False
        self._state_fn: Optional[Callable[[], dict]] = None
        self._state_buf: list = []

    # --------------------------------------------------------------- profile
    def _apply_record_profile(self, cam: _Cam) -> None:
        """Converge AE, then freeze exposure / gain / WB; neutral colour."""
        s = cam.sensor
        _set(s, rs.option.enable_auto_exposure, 1)
        _set(s, rs.option.enable_auto_white_balance, 1)
        for k, v in NEUTRAL.items():
            _set(s, getattr(rs.option, k), v)
        # Feed AE_CONVERGE_FRAMES frames so AE / AWB settle
        for _ in range(AE_CONVERGE_FRAMES):
            try:
                cam.pipeline.wait_for_frames(timeout_ms=3000)
            except Exception:
                break
        time.sleep(0.5)
        # Read the converged values
        exp = s.get_option(rs.option.exposure) if s.supports(rs.option.exposure) else None
        gain = s.get_option(rs.option.gain) if s.supports(rs.option.gain) else None
        wb = s.get_option(rs.option.white_balance) if s.supports(rs.option.white_balance) else None
        # Clamp exposure to protect the target fps: the exposure time must stay well below the
        # frame period. 30 fps -> 33.3 ms period, ceiling = 60 % = 20000 us; brightness lost
        # above the ceiling is compensated with gain.
        if exp is not None:
            ceil_us = 0.6 * 1e6 / self.fps
            if exp > ceil_us and gain is not None:
                ratio = exp / ceil_us
                rng = s.get_option_range(rs.option.gain)
                gain = float(min(rng.max, gain * ratio))
                exp = ceil_us
                print(f"     · {cam.role} exposure {s.get_option(rs.option.exposure):.0f}"
                      f"->{exp:.0f}us to keep {self.fps} fps, gain up to {gain:.0f}")
        _set(s, rs.option.enable_auto_exposure, 0)
        if gain is not None:
            _set(s, rs.option.gain, gain)
        if exp is not None:
            _set(s, rs.option.exposure, exp)
        _set(s, rs.option.enable_auto_white_balance, 0)
        if wb is not None and wb > 0:
            _set(s, rs.option.white_balance, wb)
        cam.locked = {"exposure": exp, "gain": gain, "white_balance": wb,
                      **{k: v for k, v in NEUTRAL.items()}}

    # ----------------------------------------------------------------- start
    def start(self) -> list[str]:
        present = {d.get_info(rs.camera_info.serial_number): d
                   for d in rs.context().query_devices()}
        started = []
        for role, sn in self.spec:
            if sn not in present:
                print(f"  ⚠️  role={role} SN={sn} not connected, skipped")
                continue
            pipe = rs.pipeline()
            cfg = rs.config()
            cfg.enable_device(sn)
            cfg.enable_stream(rs.stream.color, self.w, self.h,
                              rs.format.bgr8, self.fps)
            prof = pipe.start(cfg)
            cam = _Cam(role=role, serial=sn, pipeline=pipe,
                       sensor=_color_sensor(prof.get_device()),
                       width=self.w, height=self.h)
            print(f"  → {role} SN={sn} converging AE ...", flush=True)
            self._apply_record_profile(cam)
            lk = cam.locked
            print(f"  ✅ {role} frozen: exposure={lk.get('exposure')} "
                  f"gain={lk.get('gain')} wb={lk.get('white_balance')} "
                  f"| neutral contrast/gamma/saturation defaults")
            self._cams[role] = cam
            started.append(role)
        self._running = True
        for role in started:
            t = threading.Thread(target=self._loop, args=(role,),
                                 daemon=True, name=f"rec-{role}")
            t.start()
            self._threads.append(t)
        return started

    def _loop(self, role: str) -> None:
        cam = self._cams[role]
        while self._running:
            try:
                fr = cam.pipeline.wait_for_frames(timeout_ms=2000)
                c = fr.get_color_frame()
                if not c:
                    continue
                hw_ts = c.get_timestamp()              # ms, hardware timestamp
                wall_ts = time.time()                  # s, wall clock (same clock as the state samples)
                img = np.asanyarray(c.get_data())
                with cam.lock:
                    cam.last_color = img
                    cam.last_hw_ts = hw_ts
                    cam.n_grab += 1
                    if self._recording:
                        cam.frames.append((hw_ts, wall_ts, img.copy()))
            except Exception:
                continue
        if self._recording:
            pass

    # ------------------------------------------------------------ recording
    def attach_state_fn(self, fn: Callable[[], dict]) -> None:
        """Robot-state hook: a callable returning e.g. {'ee_pose': ..., 'ee_ok': ...}."""
        self._state_fn = fn

    def begin_episode(self) -> None:
        for cam in self._cams.values():
            with cam.lock:
                cam.frames = []
        self._state_buf = []
        self._recording = True

    def end_episode(self) -> None:
        self._recording = False

    def poll_state(self) -> None:
        """Call once per loop step while recording: appends one timestamped state sample."""
        if self._state_fn and self._recording:
            try:
                st = dict(self._state_fn())
                st["_t"] = time.time()
                self._state_buf.append(st)
            except Exception:
                pass

    # ------------------------------------------------------------- snapshot
    def snapshot(self, role: str) -> Optional[np.ndarray]:
        cam = self._cams.get(role)
        if cam is None:
            return None
        with cam.lock:
            return None if cam.last_color is None else cam.last_color.copy()

    def stats(self) -> dict:
        out = {}
        for role, cam in self._cams.items():
            with cam.lock:
                out[role] = {"serial": cam.serial, "grabbed": cam.n_grab,
                             "buffered": len(cam.frames),
                             "locked": cam.locked}
        return out

    # ----------------------------------------------------------------- hdf5
    def snapshot_episode(self) -> dict:
        """Shallow copy of the current episode's frames / states (array references, no deep
        copy). For asynchronous writing: after the snapshot, begin_episode clears cam.frames
        while the snapshot keeps the old arrays alive; a background thread writes from the
        snapshot and the main loop returns immediately."""
        cams = {}
        for role, cam in self._cams.items():
            with cam.lock:
                frames = list(cam.frames)   # shallow: array references only
            cams[role] = {"frames": frames, "height": cam.height,
                          "width": cam.width, "serial": cam.serial,
                          "locked": dict(cam.locked or {})}
        return {"cams": cams, "state": list(self._state_buf)}

    def write_hdf5(self, path: str | Path, compression_opts: int = 4,
                   snapshot: dict | None = None, attrs: dict | None = None) -> dict:
        """compression_opts: gzip level (1 = fast, 4 = default).
        snapshot: if given (a snapshot_episode() result) write from it (background thread);
        otherwise write from the recorder's current buffers.
        attrs: extra root attributes (e.g. poll_hz, state_dir)."""
        import h5py
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        if snapshot is not None:
            cams_items = [(r, c["frames"], c["height"], c["width"],
                           c["serial"], c["locked"]) for r, c in snapshot["cams"].items()]
            state_buf = snapshot["state"]
        else:
            cams_items = [(r, list(c.frames), c.height, c.width, c.serial,
                           (c.locked or {})) for r, c in self._cams.items()]
            state_buf = self._state_buf
        meta = {}
        with h5py.File(path, "w") as f:
            g = f.create_group("observations")
            for role, frames, height, width, serial, locked in cams_items:
                if not frames:
                    continue
                hw_ts = np.array([t for t, _, _ in frames], dtype=np.float64)
                wall_ts = np.array([w for _, w, _ in frames], dtype=np.float64)
                imgs = np.stack([im for _, _, im in frames]).astype(np.uint8)
                cg = g.create_group(role)
                cg.create_dataset("image", data=imgs, compression="gzip",
                                  compression_opts=compression_opts,
                                  chunks=(1, height, width, 3))
                cg.create_dataset("hw_timestamp_ms", data=hw_ts)
                cg.create_dataset("wall_timestamp_s", data=wall_ts)
                cg.attrs["serial"] = serial
                for k, v in (locked or {}).items():
                    cg.attrs[f"locked_{k}"] = (v if v is not None else -1)
                meta[role] = imgs.shape
            if state_buf:
                sg = f.create_group("state")
                keys = set().union(*[s.keys() for s in state_buf])
                for k in keys:
                    try:
                        arr = np.array([s.get(k, np.nan) for s in state_buf])
                        sg.create_dataset(k, data=arr)
                    except Exception:
                        pass
                meta["state_steps"] = len(state_buf)
            for k, v in (attrs or {}).items():
                f.attrs[k] = v
        return meta

    # ------------------------------------------------------------------ stop
    def stop(self) -> None:
        self._running = False
        time.sleep(0.3)
        for cam in self._cams.values():
            try:
                cam.pipeline.stop()
            except Exception:
                pass

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *a):
        self.stop()
