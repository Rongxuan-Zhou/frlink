"""Multi-RealSense frame grabber (preview / snapshot use).

Design:
  - streams are addressed by **role name** (e.g. wrist_d455 / wrist_d405), not by device index
  - serial -> role binding is explicit in record/config/cameras.yaml
  - RGB and depth are both supported (depth aligned to the colour plane by default)
  - optional hardware_reset before start to recover a wedged USB device (off by default, see the
    comment in cameras.yaml)

Usage:
    from record.cameras import CameraGrabber
    cam = CameraGrabber()  # reads record/config/cameras.yaml
    cam.start()
    color = cam.snapshot("wrist_d455")            # H x W x 3 BGR uint8 copy
    depth = cam.snapshot_depth("wrist_d405")      # H x W   uint16 mm
    cam.stop()

For dataset recording use recorder.MultiCamRecorder instead (frozen exposure, hardware
timestamps, HDF5 output).
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

import pyrealsense2 as rs


_RECORD_ROOT = Path(__file__).resolve().parents[1]
_DEFAULT_CONFIG = _RECORD_ROOT / "config" / "cameras.yaml"


def load_camera_config(config_path: Optional[Path] = None) -> dict:
    path = Path(config_path) if config_path else _DEFAULT_CONFIG
    if not path.exists():
        raise FileNotFoundError(f"camera config not found: {path}")
    with path.open() as f:
        return yaml.safe_load(f)


@dataclass
class _CamRuntime:
    role: str
    serial: str
    pipeline: rs.pipeline
    align: Optional[rs.align]
    last_color: Optional[np.ndarray] = None
    last_depth: Optional[np.ndarray] = None
    last_ts: float = 0.0
    frame_count: int = 0
    lock: threading.Lock = field(default_factory=threading.Lock)


class CameraGrabber:
    """Background per-camera grab threads; latest frame by role name (copy). Thread safe."""

    def __init__(self, config: Optional[dict] = None,
                 config_path: Optional[Path] = None) -> None:
        self.config = config or load_camera_config(config_path)
        self._defaults = self.config.get("defaults", {})
        self._cam_specs = self.config.get("cameras", {})
        self._cams: dict[str, _CamRuntime] = {}
        self._threads: list[threading.Thread] = []
        self._running = False

    # ------------------------------------------------------------------ start
    def start(self) -> list[str]:
        """Start every configured camera; return the list of roles that came up."""
        if self._defaults.get("hardware_reset_on_start", True):
            self._hardware_reset_all()

        ctx = rs.context()
        present = {d.get_info(rs.camera_info.serial_number): d
                   for d in ctx.query_devices()}

        started: list[str] = []
        for role, spec in self._cam_specs.items():
            sn = str(spec["serial"])
            if sn not in present:
                print(f"  ⚠️  role={role} SN={sn} not connected, skipped")
                continue
            try:
                cam = self._start_one(role, sn)
                self._cams[role] = cam
                started.append(role)
                w = self._defaults.get("width", 640)
                h = self._defaults.get("height", 480)
                fps = self._defaults.get("fps", 30)
                print(f"  ✅ role={role:<14s} SN={sn} {w}x{h}@{fps}Hz")
            except Exception as exc:
                print(f"  ❌ role={role} SN={sn} failed to start: {exc}")

        self._running = True
        for role in started:
            t = threading.Thread(target=self._loop, args=(role,), daemon=True,
                                 name=f"cam-{role}")
            t.start()
            self._threads.append(t)
        return started

    def _start_one(self, role: str, sn: str) -> _CamRuntime:
        d = self._defaults
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(sn)

        fmt = rs.format.bgr8 if d.get("color_format", "bgr8") == "bgr8" \
            else rs.format.rgb8
        cfg.enable_stream(rs.stream.color,
                          d.get("width", 640), d.get("height", 480),
                          fmt, d.get("fps", 30))
        if d.get("enable_depth", True):
            cfg.enable_stream(rs.stream.depth,
                              d.get("width", 640), d.get("height", 480),
                              rs.format.z16, d.get("depth_fps", 30))

        profile = pipe.start(cfg)

        # Apply per-camera options to the RGB sensor. This must happen after pipeline.start
        # (the sensor has no context before that). Exposure can only be written after auto
        # exposure is switched off, so AE is handled first.
        opts = self._cam_specs[role].get("options") or {}
        if opts:
            self._apply_color_options(profile.get_device(), role, opts)

        align = rs.align(rs.stream.color) \
            if (d.get("enable_depth", True) and d.get("align_depth_to_color", True)) \
            else None
        return _CamRuntime(role=role, serial=sn, pipeline=pipe, align=align)

    @staticmethod
    def _apply_color_options(device: "rs.device", role: str,
                              opts: dict) -> None:
        # Find the RGB sensor
        color_sensor = None
        for s in device.query_sensors():
            name = s.get_info(rs.camera_info.name)
            if "RGB" in name or "Color" in name:
                color_sensor = s
                break
        if color_sensor is None:
            print(f"  ⚠️  {role}: no RGB sensor found, options skipped")
            return

        opt_map = {
            "enable_auto_exposure":      rs.option.enable_auto_exposure,
            "exposure":                  rs.option.exposure,
            "gain":                      rs.option.gain,
            "brightness":                rs.option.brightness,
            "contrast":                  rs.option.contrast,
            "gamma":                     rs.option.gamma,
            "saturation":                rs.option.saturation,
            "white_balance":             rs.option.white_balance,
            "enable_auto_white_balance": rs.option.enable_auto_white_balance,
            "backlight_compensation":    rs.option.backlight_compensation,
            "auto_exposure_priority":    rs.option.auto_exposure_priority,
        }

        # Order matters: AE must be off before exposure can be written, and AWB must be off
        # before white_balance can be written.
        ordered = [k for k in ("enable_auto_exposure",
                                "enable_auto_white_balance") if k in opts]
        ordered += [k for k in opts if k not in ordered]

        for key in ordered:
            if key not in opt_map:
                print(f"  ⚠️  {role}: unknown option '{key}', skipped")
                continue
            opt = opt_map[key]
            if not color_sensor.supports(opt):
                print(f"  ⚠️  {role}: sensor does not support {key}, skipped")
                continue
            try:
                color_sensor.set_option(opt, float(opts[key]))
                actual = color_sensor.get_option(opt)
                print(f"     · {role}.{key} = {actual}")
            except Exception as exc:
                print(f"  ⚠️  {role}: set {key}={opts[key]} failed: {exc}")

    def _hardware_reset_all(self) -> None:
        # Only reset the serials bound in the config. Never touch other RealSense devices on
        # the same bus (a viewer running on one of them would crash together with us).
        want_sns = {str(spec["serial"]) for spec in self._cam_specs.values()}
        ctx = rs.context()
        pre = [d for d in ctx.query_devices()
               if d.get_info(rs.camera_info.serial_number) in want_sns]
        if not pre:
            return
        expected = len(pre)
        for d in pre:
            try:
                d.hardware_reset()
            except Exception:
                pass
        # query_devices occasionally raises a named_mutex error while USB re-enumerates;
        # tolerate it. Only the serials we care about are counted.
        deadline = time.time() + float(self._defaults.get("reset_timeout_s", 10.0))
        last_seen = 0
        while time.time() < deadline:
            time.sleep(0.5)
            try:
                sns = {d.get_info(rs.camera_info.serial_number)
                       for d in rs.context().query_devices()}
                last_seen = len(want_sns & sns)
            except Exception:
                continue
            if last_seen >= expected:
                time.sleep(1.5)  # let the firmware settle
                return
        print(f"  ⚠️  only {last_seen}/{expected} devices enumerated after hardware_reset "
              f"(wanted SN={want_sns})")

    # ------------------------------------------------------------------- loop
    def _loop(self, role: str) -> None:
        cam = self._cams[role]
        while self._running:
            try:
                frames = cam.pipeline.wait_for_frames(timeout_ms=2000)
                if cam.align is not None:
                    frames = cam.align.process(frames)
                color = frames.get_color_frame()
                color_np = np.asanyarray(color.get_data()) if color else None
                depth_np = None
                if self._defaults.get("enable_depth", True):
                    depth = frames.get_depth_frame()
                    if depth:
                        depth_np = np.asanyarray(depth.get_data())
                if color_np is not None:
                    with cam.lock:
                        cam.last_color = color_np
                        cam.last_depth = depth_np
                        cam.last_ts = time.time()
                        cam.frame_count += 1
            except Exception:
                continue

    # ----------------------------------------------------------------- access
    def roles(self) -> list[str]:
        return list(self._cams.keys())

    def serial_of(self, role: str) -> str:
        return self._cams[role].serial

    def snapshot(self, role: str) -> Optional[np.ndarray]:
        """Latest colour frame copy (H,W,3) BGR uint8; None if nothing arrived yet."""
        cam = self._cams.get(role)
        if cam is None:
            raise KeyError(f"unknown role: {role}; have {self.roles()}")
        with cam.lock:
            return None if cam.last_color is None else cam.last_color.copy()

    def snapshot_depth(self, role: str) -> Optional[np.ndarray]:
        """Latest depth frame copy (H,W) uint16 mm; None if nothing arrived yet."""
        cam = self._cams.get(role)
        if cam is None:
            raise KeyError(f"unknown role: {role}; have {self.roles()}")
        with cam.lock:
            return None if cam.last_depth is None else cam.last_depth.copy()

    def stats(self) -> dict[str, dict]:
        out = {}
        now = time.time()
        for role, cam in self._cams.items():
            with cam.lock:
                out[role] = {
                    "serial": cam.serial,
                    "frames": cam.frame_count,
                    "age_s": (now - cam.last_ts) if cam.last_ts else None,
                    "has_color": cam.last_color is not None,
                    "has_depth": cam.last_depth is not None,
                }
        return out

    # ------------------------------------------------------------------- stop
    def stop(self) -> None:
        self._running = False
        time.sleep(0.3)
        for cam in self._cams.values():
            try:
                cam.pipeline.stop()
            except Exception:
                pass

    # context manager sugar
    def __enter__(self) -> "CameraGrabber":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()
