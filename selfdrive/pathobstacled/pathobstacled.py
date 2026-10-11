#!/usr/bin/env python3
"""Camera helper for the path-obstacle chime. Little cores, nice 19.

radard owns the radar scan and publishes pathObstacleNAP. This process
blocks on that socket (no poll loop) and opens the camera only while a
candidate is active, at most 5 times a second. A crash here must not
disengage: manager starts this process as optional.
"""
from __future__ import annotations

import math
import os
import time
from dataclasses import replace

from openpilot.selfdrive.pathobstacled.cone_cam import (
  BUDGET_S as CONE_BUDGET_S,
  CamGeom,
  ConeCamState,
  WINDOWS_M as CONE_WINDOWS_M,
  score_line,
  uv_planes_from_nv12,
)
from openpilot.selfdrive.controls.lib.path_obstacle import (
  PARAM_OBSTACLE_CHIME,
  PARAM_OBSTACLE_LOG,
  PathObstacleDetector,
  hit_from_msg,
  is_night_lighting,
  lighting_score,
  patch_change_score,
  patch_signature,
  project_road_point,
  projection_box,
  prune_thumbs,
  save_ppm,
  score_row_patches,
)

# comma 3X road camera (OS04C10). Wide focal is the stock fisheye guess.
ROAD_W, ROAD_H, ROAD_F = 1344, 760, 1141.5
WIDE_F = 425.25
AR_W, AR_H, AR_F, AR_WIDE_F = 1928, 1208, 2648.0, 567.0
VISION_HZ = 5.0
VISION_BUDGET_S = 0.008
RECV_TIMEOUT_MS = 20
CONNECT_BACKOFF_S = 0.50
CONNECT_BACKOFF_MAX_S = 5.0
STALE_FRAME_NS = 500_000_000
VISION_FAIL_REASONS = (
  "no_connection",
  "no_frame",
  "stale_frame",
  "roi_out_of_frame",
  "budget",
  "model_error",
)
# Cone camera confirm (log only). At most 5 Hz, one shared 8 ms budget per
# grab+score, SCHED_IDLE while scoring, and a backoff while modeld drops frames.
CONE_HZ = 5.0
CONE_DROP_BACKOFF_S = 0.5
CONE_DROP_BACKOFF_MAX_S = 5.0
CONE_DROP_PERC = 0.0
THUMB_DIR = "/data/media/0/realdata/path_obstacle_thumbs"
THUMB_GAP_S = 2.0
NICE = 19
CORES = [0, 1, 2, 3]
# Block in the kernel until radard publishes, or this long for a param refresh.
RADAR_TIMEOUT_MS = 1500
# CalibrationParams is written by calibrationd every few blocks, not every frame.
CAL_REFRESH_S = 10.0
# roll, pitch, yaw, height, wide-from-device euler. Height 1.22 m is the stock fallback.
_DEFAULT_CALIB = (0.0, 0.0, 0.0, 1.22, 0.0, 0.0, 0.0)


def _bool_param(params, key: str, default: bool) -> bool:
  try:
    return bool(params.get_bool(key))
  except Exception:
    return default


def _cam_size(sensor) -> tuple[int, int, float, float]:
  name = str(sensor or "").lower()
  if "ar0231" in name or "ox03" in name:
    return AR_W, AR_H, AR_F, AR_WIDE_F
  return ROAD_W, ROAD_H, ROAD_F, WIDE_F


def _own(plane):
  """Copy a crop out of camerad's live VisionIPC buffer before reading it."""
  if plane is None:
    return None
  try:
    return plane.copy()
  except Exception:
    return plane


def _lighting_from(body, cam, gps) -> float:
  """Lane and path std come from radard. Exposure and sun are local."""
  try:
    lane = float(body.laneProbMin)
  except Exception:
    lane = 1.0
  if not math.isfinite(lane):
    lane = 1.0
  try:
    std = float(body.pathYStd3s)
  except Exception:
    std = float("nan")
  path_std = std if math.isfinite(std) else None
  sun = False
  try:
    from openpilot.selfdrive.controls.lib.lat_low_visibility import sun_ahead_from_fix
    if gps is not None:
      got = sun_ahead_from_fix(
        latitude=gps.latitude,
        longitude=gps.longitude,
        unix_timestamp_millis=gps.unixTimestampMillis,
        bearing_deg=gps.bearingDeg,
        horizontal_accuracy_m=gps.horizontalAccuracy,
      )
      sun = bool(got)
  except Exception:
    sun = False
  integ = gain = grey = None
  if cam is not None:
    try:
      integ = float(cam.integLines)
    except Exception:
      integ = None
    try:
      gain = float(cam.gain)
    except Exception:
      gain = None
    try:
      grey = float(cam.measuredGreyFraction)
    except Exception:
      grey = None
  return lighting_score(
    lane_left=lane,
    lane_right=lane,
    path_y_std=path_std,
    integ_lines=integ,
    gain=gain,
    grey=grey,
    sun_ahead=sun,
    headlights=False,
  )


def _finite_list(values) -> list[float]:
  try:
    seq = list(values)
  except TypeError:
    return []
  out = []
  for value in seq:
    try:
      num = float(value)
    except (TypeError, ValueError):
      continue
    if math.isfinite(num):
      out.append(num)
  return out


def _calibration_from_params(params):
  """rpy, height, and wide euler from CalibrationParams. No liveCalibration socket."""
  try:
    raw = params.get("CalibrationParams")
  except Exception:
    raw = None
  if not raw:
    return _DEFAULT_CALIB
  try:
    from cereal import log
    with log.Event.from_bytes(raw) as msg:
      cal = msg.liveCalibration
      rpy = _finite_list(cal.rpyCalib)
      wide = _finite_list(cal.wideFromDeviceEuler)
      heights = _finite_list(cal.height)
  except Exception:
    return _DEFAULT_CALIB
  while len(rpy) < 3:
    rpy.append(0.0)
  while len(wide) < 3:
    wide.append(0.0)
  height = heights[0] if heights and heights[0] > 0.2 else 1.22
  return (rpy[0], rpy[1], rpy[2], height, wide[0], wide[1], wide[2])


def _v_plane_from_nv12(buf):
  """NV12 V plane at half resolution. None when chroma is missing.

  High V is red. Used only to spot a taillight blob in the crop.
  """
  try:
    import numpy as np
    width = int(buf.width)
    height = int(buf.height)
    stride = int(buf.stride) if getattr(buf, "stride", 0) else width
    if width < 4 or height < 4 or stride < width:
      return None
    data = buf.data
    uv_off = stride * height
    uv_rows = height // 2
    need = uv_off + stride * uv_rows
    if data is None or len(data) < need:
      return None
    uv = np.frombuffer(data, dtype=np.uint8, count=stride * uv_rows, offset=uv_off)
    return uv.reshape(uv_rows, stride)[:, 1:width:2]
  except Exception:
    return None


def _crop(plane, box):
  u0, v0, u1, v1 = box
  height, width = plane.shape[:2]
  x0 = max(0, int(u0))
  x1 = min(width, int(math_ceil(u1)))
  y0 = max(0, int(v0))
  y1 = min(height, int(math_ceil(v1)))
  if x1 - x0 < 4 or y1 - y0 < 4:
    return None, None
  patch = plane[y0:y1, x0:x1]
  road_h = max(4, (y1 - y0) // 3)
  ry0 = min(height - 1, y1)
  ry1 = min(height, ry0 + road_h)
  road = plane[ry0:ry1, x0:x1] if ry1 - ry0 >= 2 else patch[-road_h:, :]
  return _own(patch), _own(road)


def _crop_v(v_plane, box):
  """Half-resolution V crop for the same pixel box. None if it is too small."""
  if v_plane is None:
    return None
  try:
    height, width = v_plane.shape[:2]
  except Exception:
    return None
  u0, v0, u1, v1 = box
  x0 = max(0, int(u0) // 2)
  x1 = min(width, int(math_ceil(u1)) // 2)
  y0 = max(0, int(v0) // 2)
  y1 = min(height, int(math_ceil(v1)) // 2)
  if x1 - x0 < 2 or y1 - y0 < 2:
    return None
  return _own(v_plane[y0:y1, x0:x1])


def math_ceil(value: float) -> int:
  iv = int(value)
  return iv if iv == value or value < 0 else iv + 1


def _buf_stale(buf) -> bool:
  """True when timestamp_eof is a boot-time ns value older than half a second.

  A missing or zero timestamp is not stale: tests and a fresh subscribe
  sometimes have no clock, and those frames are still scored.
  """
  ts = getattr(buf, "timestamp_eof", None)
  if ts is None:
    return False
  try:
    stamp = int(ts)
  except (TypeError, ValueError):
    return False
  if stamp <= 0:
    return False
  try:
    now = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
  except Exception:
    return False
  age = now - stamp
  if age < 0:
    return False
  return age > STALE_FRAME_NS


class _Cameras:
  """VisionIPC for the road and wide cameras.

  connect() is checked. A failure is not cached as a live client, and the
  next try waits on a backoff so a down camerad cannot spin this process.
  """

  def __init__(self):
    self.road = None
    self.wide = None
    self._vipc = None
    self._next_connect = {"road": 0.0, "wide": 0.0}
    self._backoff = {"road": CONNECT_BACKOFF_S, "wide": CONNECT_BACKOFF_S}
    self._fail = {"road": "", "wide": ""}
    self.last_thumb = 0.0
    self.last_reason = ""
    self.patch_sig: dict[int, tuple] = {}
    self.chroma_v = None
    self.chroma_uv = None

  def _types(self):
    if self._vipc is None:
      from msgq.visionipc import VisionIpcClient, VisionStreamType
      self._vipc = (VisionIpcClient, VisionStreamType)
    return self._vipc

  def _kind(self, stream: str):
    _client, VisionStreamType = self._types()
    if stream == "wide":
      return VisionStreamType.VISION_STREAM_WIDE_ROAD
    return VisionStreamType.VISION_STREAM_ROAD

  def _slot(self, stream: str):
    return self.wide if stream == "wide" else self.road

  def _set_slot(self, stream: str, client) -> None:
    if stream == "wide":
      self.wide = client
    else:
      self.road = client

  def _healthy(self, client) -> bool:
    if client is None:
      return False
    ask = getattr(client, "is_connected", None)
    if ask is not None:
      try:
        if not bool(ask()):
          return False
      except Exception:
        return False
    buffers = getattr(client, "num_buffers", None)
    if buffers is None:
      return True
    try:
      return int(buffers) > 0
    except (TypeError, ValueError):
      return False

  def _streams(self, prefer: str) -> list[str]:
    """Preferred camera first, then the other one when it is actually up.

    An empty availability list means camerad has not advertised yet, so
    both streams are still tried. A list that names only one camera is
    respected.
    """
    other = "road" if prefer == "wide" else "wide"
    try:
      VisionIpcClient, VisionStreamType = self._types()
      ask = getattr(VisionIpcClient, "available_streams", None)
      avail = ask("camerad", block=False) if ask is not None else None
    except Exception:
      return [prefer, other]
    if not avail:
      return [prefer, other]
    wanted = {
      "road": VisionStreamType.VISION_STREAM_ROAD,
      "wide": VisionStreamType.VISION_STREAM_WIDE_ROAD,
    }
    have = [name for name in ("road", "wide") if wanted[name] in list(avail)]
    if not have:
      return []
    if prefer in have:
      have.remove(prefer)
      have.insert(0, prefer)
    return have

  def _ensure(self, stream: str, now: float) -> str | None:
    """None when the stream is connected. Otherwise a reason code."""
    client = self._slot(stream)
    if self._healthy(client):
      self._backoff[stream] = CONNECT_BACKOFF_S
      self._fail[stream] = ""
      return None
    if now < self._next_connect[stream]:
      return self._fail[stream] or "no_connection"
    self._set_slot(stream, None)
    self._next_connect[stream] = now + self._backoff[stream]
    self._backoff[stream] = min(CONNECT_BACKOFF_MAX_S, max(CONNECT_BACKOFF_S, self._backoff[stream] * 2.0))
    try:
      VisionIpcClient, _kind = self._types()
      client = VisionIpcClient("camerad", self._kind(stream), True)
      ok = client.connect(False)
    except Exception:
      self._fail[stream] = "no_connection"
      return "no_connection"
    if ok is not True or not self._healthy(client):
      self._fail[stream] = "no_connection"
      return "no_connection"
    self._set_slot(stream, client)
    self._backoff[stream] = CONNECT_BACKOFF_S
    self._fail[stream] = ""
    return None

  def _recv(self, stream: str, timeout_ms: int):
    client = self._slot(stream)
    if client is None:
      return None, "no_connection"
    try:
      buf = client.recv(timeout_ms=timeout_ms)
    except Exception:
      self._set_slot(stream, None)
      self._fail[stream] = "no_frame"
      return None, "no_frame"
    if buf is None:
      if not self._healthy(client):
        self._set_slot(stream, None)
        self._fail[stream] = "no_connection"
        return None, "no_connection"
      return None, "no_frame"
    if _buf_stale(buf):
      try:
        nxt = client.recv(timeout_ms=0)
      except Exception:
        nxt = None
      if nxt is not None and not _buf_stale(nxt):
        buf = nxt
      else:
        return None, "stale_frame"
    try:
      from openpilot.selfdrive.speedsignd.nv12 import y_plane_from_nv12
      plane = y_plane_from_nv12(buf)
    except Exception:
      return None, "model_error"
    if plane is None:
      return None, "no_frame"
    try:
      self.chroma_v = _v_plane_from_nv12(buf)
    except Exception:
      self.chroma_v = None
    try:
      self.chroma_uv = uv_planes_from_nv12(buf)
    except Exception:
      self.chroma_uv = None
    return plane, ""

  def grab(self, wide: bool, now: float, deadline: float | None = None):
    """Return (Y plane or None, reason). Reason is empty when a frame is ready."""
    self.chroma_v = None
    self.chroma_uv = None
    if deadline is not None and now >= deadline:
      self.last_reason = "budget"
      return None, "budget"
    prefer = "wide" if wide else "road"
    try:
      streams = self._streams(prefer)
    except Exception:
      self.last_reason = "no_connection"
      return None, "no_connection"
    if not streams:
      self.last_reason = "no_connection"
      return None, "no_connection"
    remain_s = RECV_TIMEOUT_MS / 1000.0 if deadline is None else deadline - now
    if remain_s <= 0.0:
      self.last_reason = "budget"
      return None, "budget"
    timeout_ms = max(0, min(RECV_TIMEOUT_MS, int(remain_s * 1000.0)))
    if timeout_ms <= 0:
      self.last_reason = "budget"
      return None, "budget"
    reason = "no_connection"
    for stream in streams:
      reason = self._ensure(stream, now) or ""
      if reason:
        continue
      plane, reason = self._recv(stream, timeout_ms)
      if plane is not None:
        self.last_reason = ""
        return plane, ""
      if reason in ("no_frame", "stale_frame", "model_error"):
        self.last_reason = reason
        return None, reason
    self.last_reason = reason or "no_connection"
    return None, self.last_reason


def _project(hit, roll, pitch, yaw, height, focal, width, cam_h):
  cx, cy = width / 2.0, cam_h / 2.0
  ground = project_road_point(
    hit.x, hit.y, 0.0, roll=roll, pitch=pitch, yaw=yaw,
    height=height, focal=focal, cx=cx, cy=cy,
  )
  top = project_road_point(
    hit.x, hit.y, 1.0, roll=roll, pitch=pitch, yaw=yaw,
    height=height, focal=focal, cx=cx, cy=cy,
  )
  return ground, top


def _vision_for(hit, cam, calib, cams: _Cameras, now: float):
  """Return (score or None, microseconds, patch or None, fail reason)."""
  started = time.monotonic()

  def elapsed():
    return (time.monotonic() - started) * 1e6

  roll, pitch, yaw, height, wr, wp, wy = calib if calib is not None else _DEFAULT_CALIB
  try:
    sensor = "" if cam is None else cam.sensor
  except Exception:
    sensor = ""
  width, cam_h, road_f, wide_f = _cam_size(sensor)
  use_wide = hit.x < 20.0 or abs(hit.lateral) > 2.0
  ground, top = _project(hit, roll, pitch, yaw, height, road_f, width, cam_h)
  if ground is not None and (ground[0] < width * 0.08 or ground[0] > width * 0.92):
    use_wide = True
  focal = road_f
  if use_wide:
    focal = wide_f
    ground, top = _project(hit, roll + wr, pitch + wp, yaw + wy, height, focal, width, cam_h)
  if ground is None or top is None:
    return None, elapsed(), None, "roi_out_of_frame"
  plane, reason = cams.grab(use_wide, now, now + VISION_BUDGET_S)
  if plane is None:
    return None, elapsed(), None, reason or "no_frame"
  box = projection_box(ground[0], top[1], ground[1], focal, hit.x)
  try:
    patch, road = _crop(plane, box)
  except Exception:
    return None, elapsed(), None, "model_error"
  if patch is None:
    return None, elapsed(), None, "roi_out_of_frame"
  v_crop = None
  try:
    v_crop = _crop_v(cams.chroma_v, box)
  except Exception:
    v_crop = None
  try:
    score = score_row_patches(patch, road, v_crop)
  except Exception:
    return None, elapsed(), None, "model_error"
  if score is None or not getattr(score, "evaluated", False):
    return None, elapsed(), None, "model_error"
  return score, elapsed(), patch, ""


def _maybe_thumb(patch, now: float, last: float) -> float:
  if now - last < THUMB_GAP_S:
    return last
  try:
    os.makedirs(THUMB_DIR, exist_ok=True)
  except OSError:
    return last
  path = os.path.join(THUMB_DIR, f"{int(now * 1000)}.ppm")
  if save_ppm(path, patch):
    prune_thumbs(THUMB_DIR)
    return now
  return last


def _finite_or_nan(value) -> float:
  try:
    num = float(value)
  except (TypeError, ValueError):
    return float("nan")
  if not math.isfinite(num):
    return float("nan")
  return num


def _publish_vision(pm, messaging, sample, vision_us: float) -> None:
  msg = messaging.new_message("pathObstacleVisionNAP", valid=True)
  body = msg.pathObstacleVisionNAP
  body.trackId = int(sample.track_id) & 0xFFFFFFFFFFFFFFFF
  body.visionEvaluated = bool(sample.vision_evaluated)
  body.visionConf = _finite_or_nan(sample.vision_conf)
  body.visionConfHuman = _finite_or_nan(sample.vision_conf_human)
  body.visionConfAnimal = _finite_or_nan(sample.vision_conf_animal)
  body.visionConfObstacle = _finite_or_nan(sample.vision_conf_obstacle)
  body.lightingScore = float(sample.lighting_score)
  body.wRadar = float(sample.w_radar)
  body.wVision = float(sample.w_vision)
  body.fusedScore = float(sample.fused_score)
  body.agree = bool(sample.agree)
  obj = sample.object_class if sample.object_class in ("unknown", "human", "animal", "obstacle") else "unknown"
  zone = sample.zone if sample.zone in ("none", "inPath", "entering", "roadside") else "none"
  body.objectClass = obj
  body.zone = zone
  body.brakeGate = bool(sample.brake_gate)
  body.chimed = bool(sample.chimed)
  body.chimeReason = str(sample.chime_reason)
  body.livelyScore = float(sample.lively_score)
  body.visionUs = float(vision_us)
  reason = str(getattr(sample, "vision_fail_reason", "") or "")
  if reason and reason not in VISION_FAIL_REASONS:
    reason = "model_error"
  body.visionFailReason = reason
  body.driveOverReason = str(getattr(sample, "drive_over_reason", "") or "")
  pm.send("pathObstacleVisionNAP", msg)


class Helper:
  """Blocks on radard. VisionIPC stays closed until a candidate is active."""

  def __init__(self, det: PathObstacleDetector | None = None):
    self.det = det or PathObstacleDetector()
    self.cams = _Cameras()
    self.log_on = True
    self.chime_on = True
    self.param_t = 0.0
    self.calib_t = 0.0
    self.next_vision = 0.0
    self.cam = None
    self.gps = None
    self.calib = _DEFAULT_CALIB
    self._fail_logged = ""
    self.cone = ConeCamState()
    self.next_cone = 0.0
    self.cone_backoff = 0.0
    self.cone_hold_until = 0.0
    self.cone_start = 0

  def refresh_params(self, params, now: float) -> None:
    if now - self.param_t > 1.0:
      prev_log = self.log_on
      self.log_on = _bool_param(params, PARAM_OBSTACLE_LOG, True)
      self.chime_on = _bool_param(params, PARAM_OBSTACLE_CHIME, True)
      self.param_t = now
      if self.log_on and not prev_log:
        self.calib_t = 0.0
    if now - self.calib_t > CAL_REFRESH_S:
      self.calib = _calibration_from_params(params)
      self.calib_t = now

  def absorb(self, cam, gps) -> None:
    if cam is not None:
      self.cam = cam
    if gps is not None:
      self.gps = gps

  def on_radar(self, msg, now: float):
    """Fusion and chime for one active radar hit. None if there is nothing to do.

    The camera is grabbed only when the 5 Hz budget allows it.
    """
    if not self.log_on or msg is None:
      if not self.log_on:
        self.det.reset()
      return None
    try:
      body = msg.pathObstacleNAP
    except Exception:
      return None
    if not bool(getattr(body, "active", False)):
      return None
    hit = replace(hit_from_msg(body), lighting=_lighting_from(body, self.cam, self.gps))
    # Daytime lead: same as the old vehicle exclusion. The camera stays
    # asleep and nothing is published. At night this flag is what lets
    # a stalled car chime, so the grab below still runs.
    if hit.model_lead and not is_night_lighting(hit.lighting):
      return None
    vision = None
    vision_us = 0.0
    motion = 0.0
    patch = None
    fail = ""
    attempted = now >= self.next_vision
    if attempted:
      vision, vision_us, patch, fail = _vision_for(hit, self.cam, self.calib, self.cams, now)
      if fail and fail != self._fail_logged:
        self._fail_logged = fail
        try:
          from openpilot.common.swaglog import cloudlog
          cloudlog.warning("pathobstacled vision %s", fail)
        except Exception:
          pass
      elif not fail:
        self._fail_logged = ""
      gap = 1.0 / VISION_HZ
      if vision_us > VISION_BUDGET_S * 1e6:
        gap = max(gap, 0.40)
      self.next_vision = now + gap
      if patch is not None and hit.track_id:
        sig = patch_signature(patch)
        prev = self.cams.patch_sig.get(int(hit.track_id))
        if prev is not None:
          motion = patch_change_score(prev, sig)
        self.cams.patch_sig[int(hit.track_id)] = sig
        if len(self.cams.patch_sig) > 32:
          self.cams.patch_sig.pop(next(iter(self.cams.patch_sig)))
      if vision is not None and patch is not None and vision.conf >= 0.30:
        self.cams.last_thumb = _maybe_thumb(patch, now, self.cams.last_thumb)
    sample = self.det.commit(hit, vision, chime_enabled=self.chime_on, now=now, vision_motion=motion)
    if attempted and vision is None:
      sample = replace(sample, vision_fail_reason=fail or "no_frame")
    return sample, vision_us


def _cone_geom(cam, calib) -> CamGeom:
  roll, pitch, yaw, height, _wr, _wp, _wy = calib if calib is not None else _DEFAULT_CALIB
  try:
    sensor = "" if cam is None else cam.sensor
  except Exception:
    sensor = ""
  width, cam_h, road_f, _wide_f = _cam_size(sensor)
  return CamGeom(width=width, height=cam_h, focal=road_f, roll=roll, pitch=pitch, yaw=yaw, cam_height=height)


class _IdleSched:
  """SCHED_IDLE for the cone scoring only, then back to SCHED_OTHER (nice 19 stays).

  Leaving SCHED_IDLE needs privilege: an unprivileged thread cannot come back
  (EPERM), which would leave the obstacle chime path idle-scheduled too. So
  the switch is made only as root; otherwise scoring stays at nice 19, the
  lowest SCHED_OTHER priority, under the same 8 ms budget.
  """

  def __enter__(self):
    self.ok = False
    try:
      if os.geteuid() != 0:
        return self
      os.sched_setscheduler(0, os.SCHED_IDLE, os.sched_param(0))
      self.ok = True
    except (AttributeError, OSError):
      pass
    return self

  def __exit__(self, *exc):
    if self.ok:
      try:
        os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
      except (AttributeError, OSError):
        pass
    return False


def cone_frame_drop(driving_model) -> bool:
  """True when modeld reports dropped frames (drivingModelData.frameDropPerc)."""
  if driving_model is None:
    return False
  try:
    perc = float(driving_model.frameDropPerc)
  except Exception:
    return False
  return math.isfinite(perc) and perc > CONE_DROP_PERC


def on_cone(helper: Helper, body, now: float, driving_model=None, frame=None):
  """Score the radar cone line in one ROAD frame. Returns the log dict or None.

  `frame` (y, u, v) is for tests; on device the frame comes from VisionIPC.
  Nothing reads the result: it is only logged.
  """
  try:
    side = int(body.side)
    count = int(body.count)
    lats = (float(body.latNear), float(body.latMid), float(body.latFar))
    radar_active = bool(body.active)
  except Exception:
    return None
  if side == 0 or count <= 0 or not all(math.isfinite(v) for v in lats):
    return None
  if cone_frame_drop(driving_model):
    helper.cone_backoff = min(CONE_DROP_BACKOFF_MAX_S, max(CONE_DROP_BACKOFF_S, helper.cone_backoff * 2.0))
    helper.cone_hold_until = now + helper.cone_backoff
    return None
  if now < helper.cone_hold_until or now < helper.next_cone:
    return None
  helper.cone_backoff = 0.0
  helper.next_cone = now + 1.0 / CONE_HZ
  started = time.monotonic()
  deadline = started + CONE_BUDGET_S
  reason = ""
  scores = []
  over = False
  with _IdleSched():
    if frame is not None:
      y, u, v = frame
    else:
      y, reason = helper.cams.grab(False, now, deadline)
      uv = helper.cams.chroma_uv
      u, v = uv if uv is not None else (None, None)
      if y is not None and u is None:
        reason = "no_frame"
    if not reason and y is not None:
      geom = _cone_geom(helper.cam, helper.calib)
      try:
        scores, over = score_line(y, u, v, geom, lats, deadline=deadline, start=helper.cone_start)
      except Exception:
        scores, reason = [], "model_error"
      if over:
        reason = "budget"
        helper.cone_start = (helper.cone_start + max(1, len(scores))) % len(CONE_WINDOWS_M)
      else:
        helper.cone_start = 0
  ms = (time.monotonic() - started) * 1e3
  if ms > CONE_BUDGET_S * 1e3:
    helper.next_cone = now + max(1.0 / CONE_HZ, 0.40)
  confirmed = helper.cone.update(side, scores, now)
  if not helper.cone.should_log(now):
    return None
  return {
    "side": side,
    "radar_active": radar_active,
    "radar_count": count,
    "confirmed": confirmed,
    "n_hits": sum(1 for sc in scores if sc.hit),
    "n_scored": len(scores),
    "hits": [int(sc.x0) for sc in scores if sc.hit],
    "conf": round(max((sc.conf for sc in scores), default=0.0), 3),
    "ms": round(ms, 2),
    "reason": reason,
  }


def _log_cone(event: dict) -> None:
  try:
    from openpilot.common.swaglog import cloudlog
    cloudlog.event("cone_cam", **event)
  except Exception:
    pass


def _latest_body(messaging, sock):
  last = None
  while True:
    msg = messaging.recv_one_or_none(sock)
    if msg is None:
      return last
    try:
      last = getattr(msg, msg.which())
    except Exception:
      last = msg


def _run() -> None:
  import cereal.messaging as messaging
  from openpilot.common.params import Params

  params = Params()
  from openpilot.common.gps import get_gps_location_service
  # No carState, modelV2, radarState, liveTracks, deviceState, or
  # liveCalibration. Those services are at or near the msgq reader cap.
  # Sun uses whichever GPS this device publishes. Calibration is the param.
  gps_service = get_gps_location_service(params)
  # coneLineNAP (radard, 8 Hz) and drivingModelData (modeld frame drops) are
  # far below the msgq reader cap; test_msgq_reader_budget counts them.
  sock = messaging.sub_sock("pathObstacleNAP", timeout=RADAR_TIMEOUT_MS)
  cone_sock = messaging.sub_sock("coneLineNAP", conflate=True)
  dmd_sock = messaging.sub_sock("drivingModelData", conflate=True)
  cam_sock = messaging.sub_sock("roadCameraState", conflate=True)
  gps_sock = messaging.sub_sock(gps_service, conflate=True)
  pm = messaging.PubMaster(["pathObstacleVisionNAP"])
  poller = messaging.Poller()
  poller.registerSocket(sock)
  poller.registerSocket(cone_sock)
  helper = Helper()
  driving_model = None

  while True:
    ready = poller.poll(RADAR_TIMEOUT_MS)
    now = time.monotonic()
    helper.refresh_params(params, now)
    if not helper.log_on:
      helper.det.reset()
      _latest_body(messaging, sock)
      _latest_body(messaging, cone_sock)
      continue
    if not ready:
      continue
    helper.absorb(
      _latest_body(messaging, cam_sock),
      _latest_body(messaging, gps_sock),
    )
    dmd = _latest_body(messaging, dmd_sock)
    if dmd is not None:
      driving_model = dmd
    # Obstacle first, unchanged: every queued pathObstacleNAP in order.
    while True:
      msg = messaging.recv_one_or_none(sock)
      if msg is None:
        break
      try:
        active = bool(msg.pathObstacleNAP.active)
      except Exception:
        continue
      if not active:
        continue
      try:
        result = helper.on_radar(msg, time.monotonic())
        if result is None:
          continue
        sample, vision_us = result
        _publish_vision(pm, messaging, sample, vision_us)
      except Exception:
        continue
    cone = _latest_body(messaging, cone_sock)
    if cone is None:
      continue
    try:
      event = on_cone(helper, cone, time.monotonic(), driving_model)
      if event is not None:
        _log_cone(event)
    except Exception:
      continue


def main() -> None:
  from openpilot.common.realtime import drop_realtime, set_core_affinity
  drop_realtime()
  set_core_affinity(CORES)
  try:
    os.nice(NICE)
  except OSError:
    pass
  _run()


if __name__ == "__main__":
  main()
