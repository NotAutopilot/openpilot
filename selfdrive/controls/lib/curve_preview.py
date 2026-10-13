"""Curve preview slowing and exit release from the model path.

Justin (Sep 20–27 qlogs) starts slowing 6–9 s before the tightest point
of a bend, while lateral accel is still small, and is back on the gas at
the apex. The reactive curve cap (current curvature) only starts once the
car is already in the bend, and the stock turn clip (1.7 m/s² total on a
raw steer model) held +a at 0 until the bend had unwound.

Preview: over the model path 0–5 s ahead, each point's comfort speed
(speed-dependent lateral target on its curvature) sets a kinematic accel
ceiling that reaches that speed ~1.5 s before the point: negative when
the car must slow, a small positive allowance when it is just under the
comfort speed (no speeding up into the bend and braking again). The
strongest one, bounded below by a speed-dependent comfort decel and
jerk-limited, is a min() on the longitudinal command: it can only lower
the command, never raise it. MAX stays the hard ceiling. Points near the
horizon edge fade in, so a bend entering the horizon does not step.

Exit: the turn clip is a friction circle on true cornering force, using
the smaller of the current lateral accel and the path's 1 s ahead, so +a
returns as the road straightens instead of after it has.
"""
from __future__ import annotations

import math

from openpilot.selfdrive.controls.lib.curve_max_hold import curve_speed_for_curvature
from openpilot.selfdrive.modeld.constants import ModelConstants

# Oct 13: the bend was seen only 3–5 s ahead, so slowing began 0.6–6 s AFTER
# turn-in and plateaued at -0.35…-0.43, shedding 5–8 mph mid-curve. Look out
# 7 s so slowing starts ~2.5–3 s before turn-in (Justin's lift point).
PREVIEW_HORIZON_S = 7.0
# Actuator lag compensation: pedal/regen response trails the command by
# ~1.5 s, so the command finishes 1.5 s before the tight point and the car
# itself is done slowing by the apex.
PREVIEW_LAG_S = 1.5
PREVIEW_LEAD_S = PREVIEW_LAG_S
# Room is never shorter than this much travel, so a small overspeed near
# the car asks for a proportional ease, not a wall.
PREVIEW_MIN_ROOM_S = 2.0
PREVIEW_MIN_ROOM_M = 5.0
PREVIEW_MIN_V_MS = 5.0
# Comfort decel and build jerk vs speed: firm in town, a lift on the highway.
PREVIEW_DECEL_BP_MS = [10.0, 25.0]
# Oct 13: about -0.8 m/s² cap (~60 % of his -1.4 lift-off), every speed.
PREVIEW_DECEL_MS2 = [0.8, 0.8]
PREVIEW_JERK_MS3 = [1.0, 0.8]
PREVIEW_RELEASE_JERK_MS3 = 1.0
PREVIEW_TAU_S = 0.30
PREVIEW_MAX_ABS_CURVATURE = 0.2  # ignore nonsense samples (r < 5 m)
# No constraint (straight road): the ceiling is this, i.e. nothing binds.
PREVIEW_FREE_A_MS2 = 2.0
# Points in the last second of the horizon fade in.
PREVIEW_FADE_S = 1.0

# Exit: total accel allowed through a bend (true cornering force).
TURN_TOTAL_BP_MS = [8.0, 20.0, 30.0, 40.0]
TURN_TOTAL_MS2 = [2.4, 2.3, 2.4, 3.2]
EXIT_LOOKAHEAD_S = 1.0


def _interp(x: float, xp: list[float], fp: list[float]) -> float:
  if x <= xp[0]:
    return float(fp[0])
  for i in range(1, len(xp)):
    if x <= xp[i]:
      t = (x - xp[i - 1]) / (xp[i] - xp[i - 1])
      return float(fp[i - 1] + t * (fp[i] - fp[i - 1]))
  return float(fp[-1])


def preview_decel_limit_ms2(v_ego: float) -> float:
  return _interp(float(v_ego), PREVIEW_DECEL_BP_MS, PREVIEW_DECEL_MS2)


def path_curvature(model) -> tuple[list[float], list[float], list[float]] | None:
  """(t, distance along path, curvature) from modelV2, or None if unusable.

  Curvature is the path yaw rate over its speed (orientationRate.z / v).
  """
  try:
    xs = [float(v) for v in model.position.x]
    vs = [float(v) for v in model.velocity.x]
    wz = [float(v) for v in model.orientationRate.z]
    ts = [float(v) for v in model.position.t] if len(model.position.t) == len(xs) else list(ModelConstants.T_IDXS)
  except (AttributeError, TypeError, ValueError):
    return None
  n = len(xs)
  if n < 2 or len(vs) != n or len(wz) != n or len(ts) != n:
    return None
  if not all(math.isfinite(v) for v in xs + vs + wz + ts):
    return None
  ks = []
  for w, v in zip(wz, vs, strict=True):
    k = w / max(v, 1.0)
    ks.append(max(-PREVIEW_MAX_ABS_CURVATURE, min(PREVIEW_MAX_ABS_CURVATURE, k)))
  return ts, xs, ks


def _smooth01(x: float) -> float:
  if x <= 0.0:
    return 0.0
  if x >= 1.0:
    return 1.0
  return x * x * (3.0 - 2.0 * x)


def curve_preview_accel(v_ego: float, ts, dists, kappas) -> float:
  """Unslewed accel ceiling from the path ahead (PREVIEW_FREE_A_MS2 = none).

  Negative when a bend ahead needs a slower speed, bounded by the comfort
  decel; small positive when the car is just under a bend's comfort speed.
  """
  v = float(v_ego)
  if v < PREVIEW_MIN_V_MS or ts is None:
    return PREVIEW_FREE_A_MS2
  a = PREVIEW_FREE_A_MS2
  min_room = max(PREVIEW_MIN_ROOM_M, PREVIEW_MIN_ROOM_S * v)
  for t, d, k in zip(ts, dists, kappas, strict=True):
    if t < 0.0 or t > PREVIEW_HORIZON_S:
      continue
    v_t = curve_speed_for_curvature(k)
    if v_t is None:
      continue
    room = max(min_room, float(d) - PREVIEW_LEAD_S * v)
    a_i = (v_t * v_t - v * v) / (2.0 * room)
    fade = _smooth01((float(t) - (PREVIEW_HORIZON_S - PREVIEW_FADE_S)) / PREVIEW_FADE_S)
    a_i += (PREVIEW_FREE_A_MS2 - a_i) * fade
    a = min(a, a_i)
  return min(PREVIEW_FREE_A_MS2, max(a, -preview_decel_limit_ms2(v)))


def path_lat_accel_ahead(v_ego: float, ts, kappas, t_ahead: float = EXIT_LOOKAHEAD_S) -> float | None:
  """|a_y| the path asks for t_ahead seconds from now, at today's speed."""
  if ts is None or not ts:
    return None
  if t_ahead <= ts[0]:
    k = kappas[0]
  elif t_ahead >= ts[-1]:
    k = kappas[-1]
  else:
    k = kappas[-1]
    for i in range(1, len(ts)):
      if t_ahead <= ts[i]:
        f = (t_ahead - ts[i - 1]) / max(1e-6, ts[i] - ts[i - 1])
        k = kappas[i - 1] + f * (kappas[i] - kappas[i - 1])
        break
  return abs(float(k)) * float(v_ego) ** 2


def turn_total_accel_ms2(v_ego: float) -> float:
  return _interp(float(v_ego), TURN_TOTAL_BP_MS, TURN_TOTAL_MS2)


def turn_accel_limit(v_ego: float, a_y_now: float, a_y_ahead: float | None = None) -> float:
  """+a allowed in a bend: friction circle on the smaller of now / 1 s ahead."""
  ay = abs(float(a_y_now))
  if a_y_ahead is not None and math.isfinite(float(a_y_ahead)):
    ay = min(ay, abs(float(a_y_ahead)))
  total = turn_total_accel_ms2(v_ego)
  return math.sqrt(max(total * total - ay * ay, 0.0))


class CurvePreview:
  """Low-pass the preview ceiling; jerk-limit its braking part.

  The braking part (<= 0) builds and releases at comfort jerk, and a
  braking ceiling never rises faster than the release jerk. The positive
  allowance passes through the low-pass only; it can only hold back +a,
  never add it.
  """

  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self._f = PREVIEW_FREE_A_MS2
    self._brake = 0.0
    self.a = PREVIEW_FREE_A_MS2

  def update(self, v_ego: float, a_raw: float, dt: float) -> float:
    dt = max(1e-3, float(dt))
    target = min(PREVIEW_FREE_A_MS2, float(a_raw))
    self._f += (dt / (PREVIEW_TAU_S + dt)) * (target - self._f)
    build = _interp(float(v_ego), PREVIEW_DECEL_BP_MS, PREVIEW_JERK_MS3) * dt
    release = PREVIEW_RELEASE_JERK_MS3 * dt
    delta = min(0.0, self._f) - self._brake
    if delta < -build:
      delta = -build
    elif delta > release:
      delta = release
    self._brake = min(0.0, self._brake + delta)
    out = self._brake + max(0.0, self._f)
    # Past the bend the positive allowance returns fast; while the ceiling
    # is still braking, it rises no faster than the release jerk.
    if self.a < 0.0:
      out = min(out, self.a + release)
    self.a = out
    return self.a
