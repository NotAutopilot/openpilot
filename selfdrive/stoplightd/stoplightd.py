#!/usr/bin/env python3
"""Stoplight-green chime helper. Little cores, nice 19, camera only.

While the car is stopped in Drive, look for lit signal lamps in the top of
the road camera frame about 4 times a second. A confirmed red raises the
HUD red flag. When that red goes out and a new green comes on in the same
signal head, publish stoplightNAP.chime once; soundd plays the ding.

Gate inputs are livePose (speed) and onroadEvents (gear), not carState,
so this process does not add a carState subscriber. Nothing here brakes,
steers, or changes engagement. Manager starts it onroad only when
NAPStoplightChime is On, and restarts it if it crashes.
"""
from __future__ import annotations

import os
import time

from openpilot.selfdrive.stoplightd.detect import find_lamps
from openpilot.selfdrive.stoplightd.gate import StoppedInDrive, speed_from_pose
from openpilot.selfdrive.stoplightd.tracker import REF_WIDTH, StoplightTracker

PARAM = "NAPStoplightChime"
HZ = 4.0
IDLE_HZ = 1.0
RECV_TIMEOUT_MS = 20
CONNECT_BACKOFF_S = 0.5
CONNECT_BACKOFF_MAX_S = 5.0
STALE_FRAME_NS = 500_000_000
PARAM_REFRESH_S = 2.0
NICE = 19
CORES = [0, 1, 2, 3]


def _bool_param(params, key: str, default: bool) -> bool:
  try:
    value = params.get_bool(key)
  except Exception:
    return default
  return bool(value)


def nv12_uv_offset(buf, stride: int, height: int) -> int:
  """UV plane start. 3X Venus NV12 is page-aligned; not always stride*height.

  camerad sets VisionBuf.uv_offset = stride * ALIGN(height, 32). Fall back
  to stride * visible height when the buffer does not carry uv_offset.
  """
  raw = getattr(buf, "uv_offset", None)
  try:
    off = int(raw) if raw is not None else 0
  except (TypeError, ValueError):
    off = 0
  if off > 0:
    return off
  return int(stride) * int(height)


def planes_from_nv12(buf):
  """(y_half, u, v) on the half-res grid from an NV12 VisionBuf, or None.

  The UV plane starts at buf.uv_offset when camerad sets it (3X Venus
  buffers are page aligned), else stride * height.
  """
  try:
    import numpy as np
    width = int(buf.width)
    height = int(buf.height)
    stride = int(buf.stride) if getattr(buf, "stride", 0) else width
    if width < 8 or height < 8 or stride < width:
      return None
    data = buf.data
    uv_off = nv12_uv_offset(buf, stride, height)
    uv_rows = height // 2
    if data is None or len(data) < max(stride * height, uv_off + stride * uv_rows):
      return None
    y = np.frombuffer(data, dtype=np.uint8, count=stride * height).reshape(height, stride)
    uv = np.frombuffer(data, dtype=np.uint8, count=stride * uv_rows, offset=uv_off).reshape(uv_rows, stride)
    y_half = y[0:uv_rows * 2:2, 0:width:2]
    return y_half, uv[:, 0:width:2], uv[:, 1:width:2]
  except Exception:
    return None


def _buf_stale(buf) -> bool:
  ts = getattr(buf, "timestamp_eof", None)
  try:
    stamp = int(ts) if ts is not None else 0
  except (TypeError, ValueError):
    return False
  if stamp <= 0:
    return False
  try:
    age = time.clock_gettime_ns(time.CLOCK_BOOTTIME) - stamp
  except Exception:
    return False
  return age > STALE_FRAME_NS


class RoadCamera:
  """VisionIPC road stream. Connects lazily, only once the gate opens."""

  def __init__(self):
    self.client = None
    self._next = 0.0
    self._backoff = CONNECT_BACKOFF_S

  def _ensure(self, now: float) -> bool:
    if self.client is not None:
      try:
        if self.client.is_connected():
          return True
      except Exception:
        pass
      self.client = None
    if now < self._next:
      return False
    self._next = now + self._backoff
    self._backoff = min(CONNECT_BACKOFF_MAX_S, self._backoff * 2.0)
    try:
      from msgq.visionipc import VisionIpcClient, VisionStreamType
      client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, True)
      if client.connect(False) is not True:
        return False
    except Exception:
      return False
    self.client = client
    self._backoff = CONNECT_BACKOFF_S
    return True

  def grab(self, now: float):
    """((y_half, u, v), width) or (None, reason)."""
    if not self._ensure(now):
      return None, "no_connection"
    try:
      buf = self.client.recv(timeout_ms=RECV_TIMEOUT_MS)
    except Exception:
      self.client = None
      return None, "no_frame"
    if buf is None:
      return None, "no_frame"
    if _buf_stale(buf):
      return None, "stale_frame"
    planes = planes_from_nv12(buf)
    if planes is None:
      return None, "no_frame"
    return (planes, int(buf.width)), ""


class Stoplight:
  """Gate + detector + tracker. step() is pure enough to test with fakes."""

  def __init__(self):
    self.gate = StoppedInDrive()
    self.tracker = StoplightTracker()
    self.was_gated = False
    self.chime_count = 0

  def step(self, now: float, enabled: bool, frame_fn):
    """Returns a dict for the stoplightNAP message.

    frame_fn(now) -> ((y_half, u, v), width) or (None, reason); only
    called while the gate is open.
    """
    gated = bool(enabled) and self.gate.update(now)
    out = {"enabled": bool(enabled), "gated": gated, "redLight": False, "green": False, "chime": False,
           "redCount": 0, "greenCount": 0, "redX": -1.0, "redY": -1.0, "reason": "",
           "frameReason": "", "scanMs": 0.0, "speed": float(self.gate.speed or 0.0)}
    if not gated:
      if self.was_gated:
        self.tracker.reset()
      self.was_gated = False
      out["chimeCount"] = self.chime_count
      return out
    self.was_gated = True
    frame, reason = frame_fn(now)
    if frame is None:
      out["frameReason"] = reason
      lamps = None
    else:
      (y_half, u, v), width = frame
      if width > 0:
        self.tracker.scale = width / REF_WIDTH
        self.tracker.height = float(y_half.shape[0] * 2)
      t0 = time.monotonic()
      try:
        lamps = find_lamps(y_half, u, v)
      except Exception:
        lamps = None
        out["frameReason"] = "model_error"
      out["scanMs"] = (time.monotonic() - t0) * 1e3
    if lamps is not None:
      res = self.tracker.update(now, lamps)
      if res.chime:
        self.chime_count += 1
      out.update(redLight=res.red, green=res.green, chime=res.chime, redCount=min(res.red_count, 255),
                 greenCount=min(res.green_count, 255), redX=res.red_x, redY=res.red_y, reason=res.reason)
    else:
      # No frame this tick: hold the last flags for the HUD, never ding.
      last = self.tracker.last_out
      out.update(redLight=last.red, green=last.green)
    out["chimeCount"] = self.chime_count
    return out


def _publish(pm, messaging, out: dict) -> None:
  msg = messaging.new_message("stoplightNAP", valid=True)
  body = msg.stoplightNAP
  for key, value in out.items():
    setattr(body, key, value)
  pm.send("stoplightNAP", msg)


def _run() -> None:
  import cereal.messaging as messaging
  from openpilot.common.params import Params

  params = Params()
  # livePose (speed) and onroadEvents (gear); do not subscribe to carState.
  pose_sock = messaging.sub_sock("livePose", conflate=True)
  events_sock = messaging.sub_sock("onroadEvents", conflate=True)
  pm = messaging.PubMaster(["stoplightNAP"])
  core = Stoplight()
  cam = RoadCamera()
  enabled = _bool_param(params, PARAM, True)
  next_param = 0.0
  next_idle_pub = 0.0

  while True:
    start = time.monotonic()
    if start >= next_param:
      enabled = _bool_param(params, PARAM, True)
      next_param = start + PARAM_REFRESH_S
    pose = messaging.recv_one_or_none(pose_sock)
    if pose is not None:
      try:
        vel = pose.livePose.velocityDevice
        core.gate.on_pose(start, speed_from_pose(vel.x, vel.y, bool(vel.valid)))
      except Exception:
        core.gate.on_pose(start, None)
    events = messaging.recv_one_or_none(events_sock)
    if events is not None:
      try:
        core.gate.on_events(start, [str(e.name) for e in events.onroadEvents])
      except Exception:
        pass
    out = core.step(start, enabled, cam.grab)
    if out["gated"] or start >= next_idle_pub:
      try:
        _publish(pm, messaging, out)
      except Exception:
        pass
      next_idle_pub = start + 1.0 / IDLE_HZ
    period = 1.0 / HZ
    time.sleep(max(0.0, period - (time.monotonic() - start)))


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
