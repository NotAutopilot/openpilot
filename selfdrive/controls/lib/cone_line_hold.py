"""Hold the driver's line near a cone line, and apply the cone shift.

Oct 13 (pass E, 08:06): the old hold captured only 0.2–0.3 m of the ~1.0 m
Justin held and faded in ~6 s, so he steered by hand for ~55 s. Now:
  * the hold captures his ACTUAL lane offset (modelV2 lane lines, car vs
    lane centre) when each wheel push ends, and re-captures on every push;
  * it holds that offset with no fade while the cone line is active, plus
    CLEAR_S (3 s) after it clears, then releases over RELEASE_S;
  * wheel torque only. The stalk / blinker is never an input;
  * when he is not holding a line, the radar cone shift (coneLineNAP
    wouldSteer, in-lane, cap 1.0 m, ramped in radard) is applied.

Acts only after the driver has already pushed away from the cones. While
lateral is yielded this does not steer and does not change the yield.
On the re-take the curvature target is the driver's line (their offset
from the model path) instead of the model path that bends back toward
the cones. The hold stays until the cone line has been gone for a few
seconds, or the driver steers back toward it. It does not follow a
logged would-steer offset; the cone line never steers on its own.

Never commands a path closer to the cone line than the line the driver
was holding. Longitudinal is not an input or an output.

Frames:
  cone.side           +1 left of the path, -1 right
  steeringTorque      +left. Away from left cones is negative torque.
  model path y, curvature   +right (negative is a left bend). Toward
                      left cones is negative y / curvature.

Thresholds:
  push          >= 0.55 Nm away from the cones for 0.85 s (a 0.40 s gap resets).
                A 0.9 s push is enough.
  model pull    path is >= 0.12 m toward the cone line at the lookahead
  lookahead     vEgo * 1.6 s, clamped to 12–40 m, frozen when the line is captured
  min offset    0.15 m; shifts larger than 1.5 m are clipped
  cone clear    line inactive for 3.0 s, then the offset decays over 2.5 s
  re-capture    any push >= 0.55 Nm; the offset is re-read when it ends
                (a push back toward the cones re-captures a smaller offset)
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from openpilot.selfdrive.controls.lib.radar_path_gate import path_y_at_x

PARAM_CONE_LINE_HOLD = "NAPConeLineHold"
PARAM_CONE_LINE_LOG = "NAPConeLineLog"

PUSH_ARM_S = 0.85
PUSH_NM = 0.55
PUSH_GAP_S = 0.40
PULL_Y_M = 0.12
PULL_K = 0.0005
MIN_OFFSET_M = 0.15
MAX_OFFSET_M = 1.5
MAX_CURVATURE_DELTA = 0.02
CLEAR_S = 3.0
RELEASE_S = 2.5
STEER_BACK_NM = 0.55
STEER_BACK_S = 0.30
X_LA_S = 1.6
X_LA_MIN_M = 12.0
X_LA_MAX_M = 40.0
K_FILTER_S = 0.30


def lookahead_m(v_ego: float) -> float:
  try:
    v = float(v_ego)
  except (TypeError, ValueError):
    v = 0.0
  if not math.isfinite(v):
    v = 0.0
  return min(X_LA_MAX_M, max(X_LA_MIN_M, abs(v) * X_LA_S))


def _smoothstep(t: float) -> float:
  t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
  return t * t * (3.0 - 2.0 * t)


def _finite(value, default: float = 0.0) -> float:
  try:
    out = float(value)
  except (TypeError, ValueError):
    return default
  return out if math.isfinite(out) else default


@dataclass(frozen=True)
class ConeHoldOutput:
  active: bool
  offset_m: float  # applied path shift, +right; 0 when not steering it
  curvature: float
  source: str = ""  # "hold", "shift" or ""


class ConeLineHold:
  def __init__(self) -> None:
    self._reset()

  def _reset(self) -> None:
    self._push_s = 0.0
    self._push_gap_s = 0.0
    self._push_dir = 0
    self._armed = False
    self._target = 0.0          # car offset vs lane centre, +right
    self._target_rel = None     # fallback: path-relative shift when lanes are unknown
    self._side = 0
    self._gain = 0.0
    self._clear_s = 0.0
    self._release_s = 0.0
    self._releasing = False
    self._x_la = X_LA_MIN_M
    self.busy = False

  def reset(self) -> None:
    self._reset()

  def update(self, *, enabled: bool, engaged: bool, cone, torque_nm: float,
             measured_k: float, model_k: float, path_x, path_y, v_ego: float,
             yielded: bool, lat_active: bool, dt: float,
             lane_offset: float | None = None, lane_center_la: float | None = None,
             cone_shift: float = 0.0) -> ConeHoldOutput:
    model_k = _finite(model_k)
    if not enabled or not engaged:
      self._reset()
      return ConeHoldOutput(False, 0.0, model_k)

    dt = min(max(_finite(dt), 0.0), 0.2)
    torque = _finite(torque_nm)
    measured = _finite(measured_k)
    side = int(getattr(cone, "side", 0) or 0) if cone is not None else 0
    cone_active = bool(getattr(cone, "active", False)) and side in (-1, 1)
    offset = None if lane_offset is None else _finite(lane_offset, float("nan"))
    if offset is not None and not math.isfinite(offset):
      offset = None
    centre = None if lane_center_la is None else _finite(lane_center_la, float("nan"))
    if centre is not None and not math.isfinite(centre):
      centre = None

    x_now = lookahead_m(v_ego)
    if not self._armed:
      self._x_la = x_now
    y_model = path_y_at_x(path_x, path_y, self._x_la)

    # Wheel push tracking. Torque is +left; side is +1 left.
    pushing = abs(torque) >= PUSH_NM
    direction = (1 if torque > 0 else -1) if pushing else 0
    if pushing:
      self._push_gap_s = 0.0
      if self._push_dir not in (0, direction):
        self._push_s = 0.0
      self._push_dir = direction
      self._push_s += dt
    else:
      self._push_gap_s += dt
      if self._push_gap_s > PUSH_GAP_S:
        self._push_s = 0.0
        self._push_dir = 0

    # (Re-)capture his actual offset while a qualifying push is on. The last
    # value read before he lets go is the one held.
    if pushing and self._armed and self._push_s + 1e-12 >= STEER_BACK_S:
      self._capture(offset, measured, y_model, self._side)
    elif pushing and not self._armed and cone_active and direction == -side \
         and self._push_s + 1e-12 >= PUSH_ARM_S:
      # First capture: a push away from the cones (side +1 left: rightward, torque < 0).
      self._capture(offset, measured, y_model, side)

    line_here = cone_active and side == self._side
    if self._armed:
      if line_here:
        self._clear_s = 0.0
        self._releasing = False
        self._release_s = 0.0
        self._gain = 1.0
      else:
        self._clear_s += dt
        if self._clear_s + 1e-12 >= CLEAR_S:
          self._releasing = True
      if self._releasing:
        self._release_s += dt
        self._gain = 1.0 - _smoothstep(self._release_s / RELEASE_S)
        if self._gain <= 1e-3:
          self._reset()

    self.busy = self._armed
    can_steer = lat_active and not yielded and not pushing
    if self._armed and can_steer and y_model is not None:
      shift = self._hold_shift(y_model, centre)
      return self._out(shift * self._gain, model_k, "hold")
    if self._armed:
      return ConeHoldOutput(False, 0.0, model_k)

    shift = _finite(cone_shift)
    if cone_active and can_steer and abs(shift) > 1e-3:
      # Never toward the line: side +1 (left) only shifts right (+).
      if shift * side > 0.0:
        self._x_la = x_now
        return self._out(max(-MAX_OFFSET_M, min(MAX_OFFSET_M, shift)), model_k, "shift")
    return ConeHoldOutput(False, 0.0, model_k)

  def _capture(self, offset, measured: float, y_model, side: int) -> None:
    """Read the driver's actual offset as the new target (each push)."""
    if side not in (-1, 1):
      return
    if offset is not None:
      target = max(-MAX_OFFSET_M, min(MAX_OFFSET_M, offset))
      target_rel = None
    else:
      if y_model is None:
        return
      y_driver = 0.5 * measured * self._x_la * self._x_la
      target_rel = max(-MAX_OFFSET_M, min(MAX_OFFSET_M, y_driver - y_model))
      target = 0.0
    away = (target if target_rel is None else target_rel) * side
    if not self._armed and away < MIN_OFFSET_M:
      return
    if self._armed and away < MIN_OFFSET_M:
      # He steered back to (or past) the middle: let go.
      self._reset()
      return
    self._armed = True
    self._side = side
    self._target = target
    self._target_rel = target_rel
    self._gain = 1.0
    self._clear_s = 0.0
    self._releasing = False
    self._release_s = 0.0

  def _hold_shift(self, y_model: float, centre) -> float:
    if self._target_rel is not None:
      shift = self._target_rel
    elif centre is not None:
      shift = (centre + self._target) - y_model
    else:
      shift = self._target
    # Never command a path closer to the cones than the model path.
    if shift * self._side < 0.0:
      shift = 0.0
    return max(-MAX_OFFSET_M, min(MAX_OFFSET_M, shift))

  def _out(self, shift: float, model_k: float, source: str) -> ConeHoldOutput:
    x = max(self._x_la, 1.0)
    delta_k = 2.0 * shift / (x * x)
    delta_k = max(-MAX_CURVATURE_DELTA, min(MAX_CURVATURE_DELTA, delta_k))
    return ConeHoldOutput(True, shift, model_k + delta_k, source)


def lane_offset_from_model(model, x_la: float) -> tuple[float | None, float | None]:
  """(car offset vs lane centre at the car, lane centre y at x_la), both +right.

  Uses the two ego lane lines (laneLines[1], [2]). None when either is weak.
  """
  try:
    lines = list(model.laneLines)
    probs = list(model.laneLineProbs)
  except (TypeError, AttributeError):
    return None, None
  if len(lines) < 3 or len(probs) < 3 or float(probs[1]) < 0.4 or float(probs[2]) < 0.4:
    return None, None
  try:
    lx, ly = [float(v) for v in lines[1].x], [float(v) for v in lines[1].y]
    rx, ry = [float(v) for v in lines[2].x], [float(v) for v in lines[2].y]
  except (TypeError, ValueError, AttributeError):
    return None, None
  l0, r0 = path_y_at_x(lx, ly, 0.0), path_y_at_x(rx, ry, 0.0)
  la, ra = path_y_at_x(lx, ly, x_la), path_y_at_x(rx, ry, x_la)
  if None in (l0, r0, la, ra):
    return None, None
  if not (2.0 <= r0 - l0 <= 6.0):
    return None, None
  centre0 = 0.5 * (l0 + r0)
  return -centre0, 0.5 * (la + ra)
