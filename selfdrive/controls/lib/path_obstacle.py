"""Radar-triggered obstacle log, plus an animal/person chime.

This module never brakes and never steers. A later stage can call
`future_brake_intent` (humans in or entering the path, and only when
radar and vision agree). Nothing in controlsd calls it.

Radar scans every track. Vision runs only after a candidate has
persisted, and only on that spot. The driving model has no animal or
pedestrian head, so the checker is a luma contrast/shape heuristic on
the projected patch, not a network.

The radar scan runs only at or above 15 mph (6.7 m/s), with a
13 mph (5.81 m/s) floor so the gate does not flap. Below that the
stage publishes an inactive lowSpeed heartbeat and the camera helper
stays asleep.

Thresholds (device frame, y +right, path-relative lateral):
  range                         5–120 m ahead of the camera
  in the path                   |lateral| <= 1.70 m
  human band                    in the path, or within 0.9 m (3 ft)
                                of that corridor on either side
  animal band                   in the path, or within 6.1 m (20 ft)
                                of that corridor on either side
  entering                      reaches the path within 3.2 s
                                (4.5 s for a person, 5 s for a slow
                                walker within 1 m of the lane edge)
                                and is moving toward it at >= 0.55 m/s
  roadside                      past the lane edge, inside the animal
                                band, and inside a 52° radar half-angle
  slow along the road           |ground speed| <= 3 m/s
  traffic, do not chime         ground speed above 5 m/s with
                                little lateral speed (cars and
                                motorcycles in the flow). Also any
                                radar point on a model lead (every
                                leadsV3 entry) or a radard lead, and
                                a parked-car cluster
  lateral motion                |v_lat| >= 0.45 m/s, or toward the path
  persist                       0.40 s before active; confidence
                                saturates near 0.80 s
  agree                         radar >= 0.55 and vision >= 0.45,
                                and only while the target is within 85 m
  lighting weights              good light wR = 0.50, poor wR = 0.78,
                                linear in the lighting score
  chime                         radar and vision agree, and 2 of the
                                last 3 camera looks score >= 0.60
                                overall with >= 0.40 on that same class.
                                Humans only inside the human band.
                                Animals out to the animal band. Solid
                                debris only in the path or entering it.
                                Radar alone cannot call animal or human:
                                a camera score under 0.30 for that class
                                becomes an obstacle. A stationary radar
                                "human" outside the human band is dropped
                                before the camera runs. Once per track,
                                9 s global cooldown
  yaw                           v_lat = -yvRel + yaw_right * x, the same
                                model yaw the cone-line scan already uses
  night drive-over              lighting score <= 0.55 (the same exposure
                                and lane signals the camera helper already
                                folds into lighting). A stationary in-lane
                                return that is not a person or animal, and
                                is not moving across the path, chimes only
                                when a model or radar lead sits on it, or
                                the camera crop within 60 m is solid and
                                not a headlight or taillight blob. Anything
                                else is suppressed. Daytime, living things,
                                and crossing objects are unchanged. On any
                                error this rule is skipped and the chime
                                follows the rules above.
  lively                        0 still, 1 has moved in the last 2 s.
                                Ground speed, lateral speed, or
                                position wander past radar noise.
                                Dead-still cuts animal/person class
                                confidence to 0.45, not to zero.

Road-surface pop-in (first seen inside 14 m, stationary, not spanning)
and a thin wide curtain (span >= 9 m, depth < 2.5 m) are rejected.
Cone lines, guardrails and parked cars use coneLineNAP plus the same
stationary-cluster shapes.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, replace

from openpilot.selfdrive.controls.lib.radar_path_gate import RADAR_TO_CAMERA_M, path_y_at_x

PARAM_OBSTACLE_LOG = "NAPObstacleLog"
PARAM_OBSTACLE_CHIME = "NAPObstacleChime"

# radard runs the scan. One iteration past this is the most a cycle may run.
BUDGET_S = 0.004
MAX_POINTS = 48
BREAKER_TRIPS = 3
BREAKER_WINDOW_S = 10.0
HEARTBEAT_S = 1.0

# Solids chime in the path or entering it. Living classes also chime
# on the shoulder, inside their own lateral band.

LIVING_CLASSES = frozenset({"animal", "human"})
PATH_CHIME_ZONES = frozenset({"inPath", "entering"})
# Future braking. Any agreed object in or entering the path. Not wired.
# Roadside never.
BRAKE_ZONES = frozenset({"inPath", "entering"})

X_MIN_M = 5.0
X_MAX_M = 120.0
PATH_HALF_M = 1.70
# Justin: animals ~20 ft either side of the corridor, humans ~3 ft.
HUMAN_BEYOND_M = 0.9
ANIMAL_BEYOND_M = 6.1
HUMAN_OUTER_M = PATH_HALF_M + HUMAN_BEYOND_M
ANIMAL_OUTER_M = PATH_HALF_M + ANIMAL_BEYOND_M
FOV_HALF_DEG = 52.0
SLOW_ALONG_MPS = 3.0
# Radar lateral noise on a post is below this. A real step-in is not.
ENTER_TOWARD_MPS = 0.55
# Scan only while moving. Enable at 15 mph, drop below ~13 mph.
SPEED_ENABLE_MPS = 6.7
SPEED_DISABLE_MPS = 5.81
# A stopped in-path return at this speed must be closing, or it is a ghost.
HIGHWAY_MPS = 20.0
HUMAN_ALONG_MAX_MPS = 9.0
# With the flow of traffic: too fast to be a walker, and not darting sideways.
TRAFFIC_ALONG_MPS = 5.0
TRAFFIC_VLAT_MPS = 1.2
LATERAL_MOVE_MPS = 0.45
ENTER_HORIZON_S = 3.2
HUMAN_ENTER_HORIZON_S = 4.5
WALKER_ENTER_HORIZON_S = 5.0
WALKER_EDGE_M = 2.70
PERSIST_S = 0.40
PERSIST_FULL_S = 0.80
RADAR_AGREE = 0.55
VISION_AGREE = 0.45
# Radar animal/human needs the camera to score that class at least this high.
VISION_LIVING_MIN = 0.30
# Past this the night patch is ~20 px and the path lateral is loose.
# Radar keeps the track; the camera verdict is not used.
VISION_TRUST_M = 85.0
# Two of the last three looks, same class. One lucky frame does not chime.
VISION_LOOK_MIN = 0.60
VISION_CLASS_MIN = 0.40
VISION_LOOKS = 3
VISION_LOOKS_NEED = 2
WR_GOOD = 0.50
WR_POOR = 0.78
CHIME_COOLDOWN_S = 9.0
CHIME_HOLD_S = 0.35
LIVELY_WINDOW_S = 2.0
# Below this, a Bosch return is noise. Above the full value, the cue is 1.
SPEED_NOISE_MPS = 0.40
SPEED_FULL_MPS = 1.40
VLAT_NOISE_MPS = 0.30
VLAT_FULL_MPS = 1.10
WANDER_NOISE_M = 0.40
WANDER_FULL_M = 1.15
# A frozen deer keeps this fraction of its animal/person class score.
LIVELY_FLOOR = 0.45
# Roadside chime when the track has clearly moved, even if class is unknown.
LIVELY_CHIME = 0.50
CLUSTER_DX_M = 5.5
CLUSTER_DY_M = 3.2
LANE_SPAN_M = 1.8
OVERHEAD_SPAN_M = 9.0
OVERHEAD_DEPTH_M = 2.5
POPIN_X_M = 14.0
LEAD_X_M = 4.5
LEAD_Y_M = 1.6
LEAD_PROB = 0.40
# Night is the logged lighting band (about 0.40–0.43 on the Oct 7 drive).
# Daytime scores sit near 1. Missing or non-finite lighting is not night.
NIGHT_LIGHTING_MAX = 0.55
# Low-beam range. Farther than this, a stationary in-lane return at night
# needs a model lead; the crop is too small to call solid.
DRIVE_OVER_SOLID_M = 60.0
# Lead agreement for that night exception. Wider in range than the
# exclusion box, same lateral tolerance, and a higher probability.
DRIVE_OVER_LEAD_PROB = 0.50
DRIVE_OVER_LEAD_X_M = 5.0
DRIVE_OVER_LEAD_X_FRAC = 0.15
DRIVE_OVER_LEAD_Y_M = LEAD_Y_M
# Crop light test. Y is 0..255. Red is either an RGB pixel or a high NV12 V.
NEAR_SAT_Y = 200.0
SAT_Y = 235.0
LIGHT_FRAC_REJECT = 0.12
SAT_FRAC_REJECT = 0.05
RED_FRAC_REJECT = 0.06
BLOB_SHARE_REJECT = 0.55
RED_V_MIN = 165.0
RED_Y_MIN = 160.0

CLASS_ORDER = ("unknown", "human", "animal", "obstacle")
ZONE_ORDER = ("none", "inPath", "entering", "roadside")


class _OverBudget(Exception):
  """Raised inside the scan when the per-cycle deadline has passed."""


def _check_deadline(deadline: float | None) -> None:
  if deadline is not None and time.monotonic() > deadline:
    raise _OverBudget


def _enum_name(raw, order: tuple[str, ...]) -> str:
  name = _class_name(raw)
  if name.isdigit():
    idx = int(name)
    return order[idx] if 0 <= idx < len(order) else order[0]
  return name if name in order else order[0]


def _clamp(value: float, lo: float, hi: float) -> float:
  return lo if value < lo else hi if value > hi else value


def _nan() -> float:
  return float("nan")


def _finite(value) -> float | None:
  try:
    out = float(value)
  except (TypeError, ValueError):
    return None
  if not math.isfinite(out):
    return None
  return out


def _toward(lat: float, v_lat: float) -> float:
  """Speed toward the path center, m/s. Already-in-path uses |v_lat|."""
  if abs(lat) < 0.25:
    return abs(v_lat)
  return max(0.0, -math.copysign(1.0, lat) * v_lat)


@dataclass(frozen=True)
class VisionScore:
  """Patch verdict. conf is 'something solid above the road', 0..1.

  light_dominated is the night drive-over veto: a large share of the crop
  is saturated, or a red taillight pattern, or a clipped blob owns the
  contrast. It does not change the conf used in daytime.
  """

  conf: float = 0.0
  human: float = 0.0
  animal: float = 0.0
  obstacle: float = 0.0
  evaluated: bool = True
  light_dominated: bool = False
  light_frac: float = 0.0
  red_frac: float = 0.0


@dataclass(frozen=True)
class ConeHint:
  """The bits of coneLineNAP this scan needs. Tests build it directly."""

  active: bool = False
  side: int = 0
  lat_near: float = 0.0
  lat_mid: float = 0.0
  lat_far: float = 0.0
  barrier: bool = False
  parked: bool = False


@dataclass(frozen=True)
class BrakeIntent:
  """Not consumed by any controller. decel is always 0 in this stage."""

  allowed: bool
  reason: str
  decel: float = 0.0


@dataclass(frozen=True)
class ObstacleSample:
  active: bool = False
  track_id: int = 0
  range_m: float = 0.0
  lateral: float = 0.0
  v_rel: float = 0.0
  v_lat: float = 0.0
  radar_conf: float = 0.0
  vision_conf: float = _nan()
  vision_evaluated: bool = False
  lighting_score: float = 1.0
  w_radar: float = WR_GOOD
  w_vision: float = 1.0 - WR_GOOD
  fused_score: float = 0.0
  agree: bool = False
  reject_reason: str = "no_candidate"
  in_path: bool = False
  time_to_reach: float = _nan()
  object_class: str = "unknown"
  vision_conf_human: float = _nan()
  vision_conf_animal: float = _nan()
  vision_conf_obstacle: float = _nan()
  cluster_count: int = 0
  span_m: float = 0.0
  time_to_enter: float = _nan()
  entering: bool = False
  zone: str = "none"
  brake_gate: bool = False
  chimed: bool = False
  chime_reason: str = "no_candidate"
  lively_score: float = 0.0
  scan_us: float = 0.0
  vision_us: float = 0.0
  member_ids: tuple[int, ...] = ()
  vision_fail_reason: str = ""
  # Empty, or why the night drive-over rule suppressed this chime:
  # light, no_solid, beyond_60.
  drive_over_reason: str = ""

  def with_chime(self, chimed: bool, reason: str) -> ObstacleSample:
    return replace(self, chimed=bool(chimed), chime_reason=str(reason))


@dataclass(frozen=True)
class _Hit:
  """One radar candidate after the scan, before vision and the chime."""

  active: bool
  track_id: int
  member_ids: tuple[int, ...]
  x: float
  y: float
  lateral: float
  v_rel: float
  v_lat: float
  along: float
  radar_conf: float
  radar_class: str
  zone: str
  time_to_reach: float
  time_to_enter: float
  cluster_count: int
  span_m: float
  reject_reason: str
  lighting: float
  lively: float = 0.0
  v_ego: float = 0.0
  # Stationary in-lane return that a model or radar lead agrees with.
  # Kept through the scan so the night rule can chime. Daytime still
  # excludes it as a vehicle.
  model_lead: bool = False


def project_road_point(x: float, y: float, z: float, *,
                       roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
                       height: float = 1.22, focal: float = 1141.5,
                       cx: float = 672.0, cy: float = 380.0) -> tuple[float, float] | None:
  """Project a road-frame point (x forward, y left, z up) to pixels.

  Matches `get_view_frame_from_road_frame`: device y is right, z is down,
  then the view frame is x right, y down, z forward. Returns None when
  the point is behind the camera.
  """
  try:
    x, y, z = float(x), float(y), float(z)
    roll, pitch, yaw = float(roll), float(pitch), float(yaw)
    height, focal = float(height), float(focal)
    cx, cy = float(cx), float(cy)
  except (TypeError, ValueError):
    return None
  if not all(math.isfinite(v) for v in (x, y, z, roll, pitch, yaw, height, focal, cx, cy)):
    return None
  if focal <= 1.0:
    return None
  cr, sr = math.cos(roll), math.sin(roll)
  cp, sp = math.cos(pitch), math.sin(pitch)
  cyaw, syaw = math.cos(yaw), math.sin(yaw)
  # R = Rz @ Ry @ Rx, then diag(1, -1, -1) on the road point.
  rx, ry, rz = x, -y, -z
  # Rx
  ry, rz = cr * ry - sr * rz, sr * ry + cr * rz
  # Ry
  rx, rz = cp * rx + sp * rz, -sp * rx + cp * rz
  # Rz
  rx, ry = cyaw * rx - syaw * ry, syaw * rx + cyaw * ry
  # view = (y_right, z_down, x_forward) + (0, height, 0)
  vx, vy, vz = ry, rz + height, rx
  if vz <= 0.5:
    return None
  return cx + focal * (vx / vz), cy + focal * (vy / vz)


def projection_box(u: float, v_top: float, v_ground: float, focal: float, x: float,
                   half_m: float = 0.55) -> tuple[float, float, float, float]:
  """Pixel box (u0, v0, u1, v1) around the above-road segment."""
  half = focal * half_m / max(float(x), 1.0)
  half = _clamp(half, 3.0, 180.0)
  top, bot = (v_top, v_ground) if v_top <= v_ground else (v_ground, v_top)
  if bot - top < 4.0:
    bot = top + 4.0
  return u - half, top, u + half, bot


def fusion_weights(lighting: float) -> tuple[float, float]:
  """Good light shares the vote. Poor light leans on radar. Linear, no step."""
  score = _clamp(lighting if math.isfinite(lighting) else 1.0, 0.0, 1.0)
  w_radar = WR_POOR + (WR_GOOD - WR_POOR) * score
  return w_radar, 1.0 - w_radar


def fuse_scores(radar_conf: float, vision_conf: float | None, lighting: float) -> tuple[float, float, float, bool]:
  """Return fused, w_radar, w_vision, agree.

  Vision NaN does not vote. The fused score is then the radar score, and
  agree stays false because the camera has not confirmed the spot.
  """
  w_r, w_v = fusion_weights(lighting)
  radar = _clamp(radar_conf if math.isfinite(radar_conf) else 0.0, 0.0, 1.0)
  if vision_conf is None or not math.isfinite(vision_conf):
    return radar, w_r, w_v, False
  vision = _clamp(vision_conf, 0.0, 1.0)
  fused = _clamp(w_r * radar + w_v * vision, 0.0, 1.0)
  agree = radar >= RADAR_AGREE and vision >= VISION_AGREE
  return fused, w_r, w_v, agree


def lighting_score(*, lane_left: float = 1.0, lane_right: float = 1.0,
                   path_y_std: float | None = None, integ_lines: float | None = None,
                   gain: float | None = None, grey: float | None = None,
                   sun_ahead: bool = False, headlights: bool = False) -> float:
  """0 is a camera that cannot see, 1 is a bright clear frame. Continuous."""
  left = _finite(lane_left)
  right = _finite(lane_right)
  lane = _clamp(min(left if left is not None else 1.0, right if right is not None else 1.0), 0.0, 1.0)
  if path_y_std is None or not math.isfinite(path_y_std):
    path = 0.75
  else:
    path = _clamp(1.0 - (path_y_std - 0.4) / 2.6, 0.0, 1.0)
  exposure = 1.0
  if gain is not None and math.isfinite(gain) and gain > 0.0:
    exposure *= _clamp(1.2 - gain / 5.5, 0.1, 1.0)
  if integ_lines is not None and math.isfinite(integ_lines) and integ_lines > 0.0:
    exposure *= _clamp(1.35 - integ_lines / 2800.0, 0.15, 1.0)
  if grey is not None and math.isfinite(grey):
    if grey < 0.04 or grey > 0.55:
      exposure *= 0.50
    elif grey < 0.08 or grey > 0.40:
      exposure *= 0.75
  if sun_ahead:
    exposure *= 0.55
  if headlights:
    exposure *= 0.62
  model = 0.55 * lane + 0.45 * path
  return _clamp(0.42 * model + 0.58 * exposure, 0.0, 1.0)


def is_night_lighting(lighting: float) -> bool:
  """True when the existing lighting score is the night band.

  Non-finite or missing scores are daytime, so a broken exposure reading
  keeps the previous chime rules.
  """
  try:
    score = float(lighting)
  except (TypeError, ValueError):
    return False
  return math.isfinite(score) and score <= NIGHT_LIGHTING_MAX


def _row_lists(value) -> list[list]:
  """Rows from a list of lists or a numpy image. Empty when there is nothing to read."""
  if value is None:
    return []
  try:
    iterator = iter(value)
  except TypeError:
    return []
  rows = []
  for row in iterator:
    try:
      if len(row) == 0:
        continue
      rows.append(list(row))
    except TypeError:
      continue
  return rows


def _mean_std(vals: list[float]) -> tuple[float, float]:
  n = len(vals)
  if n == 0:
    return 0.0, 0.0
  mean = sum(vals) / n
  var = sum((v - mean) ** 2 for v in vals) / n
  return mean, math.sqrt(var)


def _np_plane(value):
  """2D numeric numpy array, or None (then the list path runs)."""
  try:
    import numpy as np
  except Exception:
    return None
  if not isinstance(value, np.ndarray) or value.ndim != 2 or value.dtype.kind not in "uifb":
    return None
  if value.shape[0] == 0 or value.shape[1] == 0:
    return None
  return value


def _metrics_np(obj, road) -> dict:
  """Vectorized metrics_from_rows. Same sampling, bins and fallbacks."""
  import numpy as np
  height, width = obj.shape
  cells = height * width
  step = 1
  while cells / (step * step) > 2400 and step < 8:
    step += 1
  samp = obj[::step, :width:step].astype(np.float64)
  vals = samp.ravel()
  mean = float(vals.mean())
  std = float(np.sqrt(((vals - mean) ** 2).mean()))
  ys = np.arange(samp.shape[0]) * step
  band_of = np.where(ys < height / 3, 0, np.where(ys < 2 * height / 3, 1, 2))
  xs = np.arange(samp.shape[1])
  col_of = np.where(xs < width / 3, 0, np.where(xs < 2 * width / 3, 1, 2))
  cols = []
  for c in range(3):
    sel = samp[:, col_of == c]
    cols.append(float(sel.mean()) if sel.size else mean)
  band_means = []
  for b in range(3):
    sel = samp[band_of == b, :]
    band_means.append(float(sel.mean()) if sel.size else None)
  bands = [m if m is not None else mean for m in band_means]
  road_mean = None
  if road is not None:
    rs = road[::step, :width:step].astype(np.float64).ravel()
    rs = rs[np.isfinite(rs)]
    if rs.size:
      road_mean = float(rs.mean())
  if road_mean is None:
    road_mean = (band_means[2] if band_means[2] is not None else 0.0) or mean
  return {"mean": mean, "std": std, "cols": cols, "bands": bands, "road": road_mean}


def metrics_from_rows(obj_rows, road_rows=None) -> dict:
  """Mean/std plus 3 column and 3 row means. Caps the sample count."""
  obj_np = _np_plane(obj_rows)
  if obj_np is not None and (road_rows is None or _np_plane(road_rows) is not None) and \
      (obj_np.dtype.kind != "f" or bool(__import__("numpy").isfinite(obj_np).all())):
    return _metrics_np(obj_np, _np_plane(road_rows) if road_rows is not None else None)
  rows = _row_lists(obj_rows)
  if not rows:
    return {"mean": 0.0, "std": 0.0, "cols": [0.0, 0.0, 0.0], "bands": [0.0, 0.0, 0.0], "road": 0.0}
  height = len(rows)
  width = min(len(r) for r in rows)
  cells = height * width
  step = 1
  while cells / (step * step) > 2400 and step < 8:
    step += 1
  vals: list[float] = []
  cols = [[], [], []]
  bands = [[], [], []]
  for yi, row in enumerate(rows[::step]):
    y_full = yi * step
    band = 0 if y_full < height / 3 else 1 if y_full < 2 * height / 3 else 2
    for x_full, value in enumerate(row[:width:step]):
      try:
        pix = float(value)
      except (TypeError, ValueError):
        continue
      vals.append(pix)
      col = 0 if x_full < width / 3 else 1 if x_full < 2 * width / 3 else 2
      cols[col].append(pix)
      bands[band].append(pix)
  mean, std = _mean_std(vals)
  road_vals: list[float] = []
  for row in _row_lists(road_rows)[::step]:
    for value in list(row)[:width:step]:
      num = _finite(value)
      if num is not None:
        road_vals.append(num)
  road = _mean_std(road_vals)[0] if road_vals else (bands[2] and _mean_std(bands[2])[0]) or mean
  return {
    "mean": mean,
    "std": std,
    "cols": [_mean_std(c)[0] if c else mean for c in cols],
    "bands": [_mean_std(b)[0] if b else mean for b in bands],
    "road": road,
  }


def vision_from_regions(obj_mean: float, obj_std: float, road_mean: float,
                        cols: list[float], bands: list[float]) -> VisionScore:
  """Turn patch stats into a solid-above-road score and weak class scores.

  The class split is a shape prior (tall vs mid vs wide vs a thin post).
  It is not a trained detector. A raccoon-sized target often never clears
  the solid-object bar.
  """
  contrast = abs(obj_mean - road_mean)
  if contrast < 8.0 and obj_std < 12.0:
    solid = 0.08
  else:
    solid = _clamp(contrast / 42.0, 0.0, 1.0) * 0.75 + _clamp(obj_std / 28.0, 0.0, 1.0) * 0.25
  c0, c1, c2 = (cols + [obj_mean, obj_mean, obj_mean])[:3]
  b0, b1, b2 = (bands + [obj_mean, obj_mean, obj_mean])[:3]
  side = 0.5 * (c0 + c2)
  center_delta = abs(c1 - side)
  needle = center_delta > 14.0 and center_delta > abs(c0 - c2) + 8.0
  wide = abs(c0 - road_mean) > 12.0 and abs(c2 - road_mean) > 12.0 and abs(c1 - road_mean) > 12.0
  top_c, mid_c, bot_c = abs(b0 - road_mean), abs(b1 - road_mean), abs(b2 - road_mean)
  tall = top_c > 12.0 and mid_c > 10.0
  low_only = bot_c > 12.0 and top_c < 8.0 and mid_c < 10.0
  mid_body = mid_c > 12.0 and top_c < mid_c * 0.75
  # A thin post (delineator, mailbox) is not a person or an animal.
  human = solid * (0.85 if tall and not wide and not needle else 0.20 if tall and not needle else 0.05)
  animal = solid * (0.80 if mid_body and not needle and not wide else 0.22 if mid_body and not needle else 0.08)
  obstacle = solid * (0.85 if wide or low_only or needle else 0.35)
  human = _clamp(human, 0.0, solid if solid > 0 else 1.0)
  animal = _clamp(animal, 0.0, 1.0)
  obstacle = _clamp(obstacle, 0.0, 1.0)
  return VisionScore(conf=_clamp(solid, 0.0, 1.0), human=human, animal=animal, obstacle=obstacle, evaluated=True)


def _as_rgb(value):
  """(r, g, b) when the pixel is a color triple. None for a luma sample."""
  if isinstance(value, (bytes, str)) or isinstance(value, (int, float)):
    return None
  try:
    seq = value.tolist() if hasattr(value, "tolist") else value
    if isinstance(seq, (int, float)):
      return None
    seq = list(seq)
  except TypeError:
    return None
  if len(seq) < 3:
    return None
  r, g, b = _finite(seq[0]), _finite(seq[1]), _finite(seq[2])
  if r is None or g is None or b is None:
    return None
  return r, g, b


def _luma_of(value) -> tuple[float, bool] | None:
  """(luma 0..255, red-saturated?). None when the sample is not a pixel."""
  color = _as_rgb(value)
  if color is not None:
    r, g, b = color
    luma = 0.299 * r + 0.587 * g + 0.114 * b
    # Saturated red, not a white headlight. Luma can stay moderate when
    # G and B are near zero; the NV12 path catches the same pixels via V.
    red = r >= 200.0 and r >= g + 40.0 and r >= b + 40.0
    return luma, red
  luma = _finite(value)
  if luma is None:
    return None
  return luma, False


def _light_stats_np(obj, v_plane) -> tuple[float, float, float, float]:
  """Vectorized crop_light_stats for a 2D luma crop. Same sampling and V lookup."""
  import numpy as np
  height, width = obj.shape
  if width < 2 or height < 2:
    return 0.0, 0.0, 0.0, 0.0
  step = 1
  cells = height * width
  while cells / (step * step) > 2000 and step < 8:
    step += 1
  samp = obj[::step, :width:step].astype(np.float64)
  red = np.zeros(samp.shape, dtype=bool)
  if v_plane is not None:
    nv_rows, nv_cols = v_plane.shape
    fy = (np.arange(samp.shape[0]) * step) / max(height - 1, 1)
    vy = np.minimum(nv_rows - 1, (fy * nv_rows).astype(np.int64))
    fx = (np.arange(samp.shape[1]) * step) / max(width - 1, 1)
    vx = np.minimum(nv_cols - 1, (fx * nv_cols).astype(np.int64))
    v_vals = v_plane.astype(np.float64)[vy[:, None], vx[None, :]]
    red = (samp >= RED_Y_MIN) & np.isfinite(v_vals) & (v_vals >= RED_V_MIN)
  finite = np.isfinite(samp)
  lumas = samp[finite]
  n = int(lumas.size)
  if n == 0:
    return 0.0, 0.0, 0.0, 0.0
  near = float(np.count_nonzero(lumas >= NEAR_SAT_Y)) / n
  sat = float(np.count_nonzero(lumas >= SAT_Y)) / n
  red_frac = float(np.count_nonzero(red[finite])) / n
  median = float(np.partition(lumas, n // 2)[n // 2])
  excess = np.maximum(0.0, lumas - median)
  total = float(excess.sum())
  if total <= 1.0:
    blob = 0.0
  else:
    blob = float(excess[lumas >= NEAR_SAT_Y].sum()) / total
  return near, sat, red_frac, blob


def crop_light_stats(obj_rows, v_rows=None) -> tuple[float, float, float, float]:
  """(near_sat_frac, sat_frac, red_frac, blob_share) for one crop.

  near_sat is luma >= 200, sat is luma >= 235. red is an RGB taillight
  pixel or, when v_rows is the NV12 V crop, a bright pixel with high V.
  blob_share is how much of the contrast above the median sits in the
  clipped pixels. Empty crops return zeros. Any bad pixel is skipped.
  A 2D numpy luma crop (the on-device case) takes the vectorized path.
  """
  obj_np = _np_plane(obj_rows)
  if obj_np is not None and (v_rows is None or _np_plane(v_rows) is not None):
    return _light_stats_np(obj_np, _np_plane(v_rows) if v_rows is not None else None)
  rows = _row_lists(obj_rows)
  if not rows:
    return 0.0, 0.0, 0.0, 0.0
  height = len(rows)
  width = min(len(row) for row in rows)
  if width < 2 or height < 2:
    return 0.0, 0.0, 0.0, 0.0
  step = 1
  cells = height * width
  while cells / (step * step) > 2000 and step < 8:
    step += 1
  vdata = _row_lists(v_rows) if v_rows is not None else []
  lumas: list[float] = []
  reds = 0
  for yi, row in enumerate(rows[::step]):
    y_full = yi * step
    fy = y_full / max(height - 1, 1)
    for xi, value in enumerate(row[:width:step]):
      parsed = _luma_of(value)
      if parsed is None:
        continue
      luma, red = parsed
      if vdata:
        vy = min(len(vdata) - 1, int(fy * len(vdata)))
        vrow = vdata[vy]
        if vrow:
          fx = (xi * step) / max(width - 1, 1)
          vx = min(len(vrow) - 1, int(fx * len(vrow)))
          v_val = _finite(vrow[vx])
          if v_val is not None and luma >= RED_Y_MIN and v_val >= RED_V_MIN:
            red = True
      lumas.append(luma)
      if red:
        reds += 1
  n = len(lumas)
  if n == 0:
    return 0.0, 0.0, 0.0, 0.0
  near = sum(1 for v in lumas if v >= NEAR_SAT_Y) / n
  sat = sum(1 for v in lumas if v >= SAT_Y) / n
  red_frac = reds / n
  ordered = sorted(lumas)
  median = ordered[n // 2]
  excess = [max(0.0, v - median) for v in lumas]
  total = sum(excess)
  if total <= 1.0:
    blob = 0.0
  else:
    bright = sum(ex for v, ex in zip(lumas, excess) if v >= NEAR_SAT_Y)
    blob = bright / total
  return near, sat, red_frac, blob


def light_dominated(near: float, sat: float, red: float, blob: float) -> bool:
  """True when a headlight or taillight blob owns the crop."""
  if near >= LIGHT_FRAC_REJECT or sat >= SAT_FRAC_REJECT or red >= RED_FRAC_REJECT:
    return True
  return blob >= BLOB_SHARE_REJECT and sat >= 0.015


def score_row_patches(obj_rows, road_rows, v_rows=None) -> VisionScore:
  stats = metrics_from_rows(obj_rows, road_rows)
  score = vision_from_regions(stats["mean"], stats["std"], stats["road"], stats["cols"], stats["bands"])
  try:
    near, sat, red, blob = crop_light_stats(obj_rows, v_rows)
    dominated = light_dominated(near, sat, red, blob)
  except Exception:
    near, red, dominated = 0.0, 0.0, False
  return replace(score, light_dominated=dominated, light_frac=near, red_frac=red)


def _class_name(raw) -> str:
  return str(raw).split(".")[-1]


def should_raise_chime(chimed, object_class=None, enabled: bool = True) -> bool:
  """selfdrived uses this. The detector already chose whether to chime."""
  del object_class
  if not enabled or not chimed:
    return False
  return True


def chime_banner(object_class, zone) -> str:
  """One line for the driver. Class stays in the log, not on screen."""
  del object_class
  zone_name = _class_name(zone)
  if zone_name.isdigit():
    idx = int(zone_name)
    zone_name = ZONE_ORDER[idx] if 0 <= idx < len(ZONE_ORDER) else "none"
  if zone_name == "roadside":
    return "Object near road"
  return "Object ahead"


def _beyond_corridor(lat: float) -> float:
  return max(0.0, abs(float(lat)) - PATH_HALF_M)


def _class_zone_block(sample: ObstacleSample) -> str | None:
  """Why this sample must not chime. None means the class is inside its zone.

  Humans count in the path and within 0.9 m of it. Animals count out to
  6.1 m. Solid debris counts only in the path or entering it.
  """
  beyond = _beyond_corridor(sample.lateral)
  cls = sample.object_class
  if cls == "human" and sample.zone != "inPath" and beyond > HUMAN_BEYOND_M:
    return "off_zone"
  if cls == "animal" and sample.zone != "inPath" and beyond > ANIMAL_BEYOND_M:
    return "off_zone"
  if cls in LIVING_CLASSES:
    if sample.zone in PATH_CHIME_ZONES or sample.zone == "roadside":
      return None
    return "zone"
  if sample.zone in PATH_CHIME_ZONES:
    return None
  if sample.zone == "roadside":
    return "inanimate"
  return "zone"


def _highway_still_unconfirmed(sample: ObstacleSample, v_ego: float) -> bool:
  """Stopped in the lane at highway speed, and vision did not name it.

  A weak solid score with class unknown is not agreement. A deer or a
  piece of debris that vision actually classifies still can chime.
  """
  try:
    speed = float(v_ego)
  except (TypeError, ValueError):
    return False
  if not math.isfinite(speed) or speed < HIGHWAY_MPS:
    return False
  if sample.zone != "inPath":
    return False
  if float(sample.lively_score) >= 0.35:
    return False
  if not sample.vision_evaluated:
    return True
  return sample.object_class not in ("human", "animal", "obstacle")


def future_brake_intent(sample: ObstacleSample) -> BrakeIntent | None:
  """Any agreed object in or entering the path. Roadside never.

  Not consumed by any controller. decel stays 0.
  """
  if not sample.agree or sample.zone not in BRAKE_ZONES:
    return None
  return BrakeIntent(True, "agree_in_path", 0.0)


class ChimeGate:
  """Once per track id, plus a global gap. Every class shares it."""

  def __init__(self, cooldown_s: float = CHIME_COOLDOWN_S, hold_s: float = CHIME_HOLD_S):
    self.cooldown_s = float(cooldown_s)
    self.hold_s = float(hold_s)
    self._ids: set[int] = set()
    self._last = -1e9
    self._hold_until = -1e9
    self._hold_ids: set[int] = set()

  def reset(self) -> None:
    self._ids.clear()
    self._last = -1e9
    self._hold_until = -1e9
    self._hold_ids.clear()

  def consider(self, sample: ObstacleSample, enabled: bool, now: float) -> tuple[bool, str]:
    ids = set(sample.member_ids) or ({sample.track_id} if sample.track_id else set())
    if not enabled:
      return False, "disabled"
    if ids and ids & self._hold_ids and now <= self._hold_until:
      return True, "chimed"
    if not sample.active:
      if sample.reject_reason == "not_persistent":
        return False, "not_persistent"
      return False, sample.reject_reason or "no_candidate"
    blocked = _class_zone_block(sample)
    if blocked is not None:
      return False, blocked
    if not sample.agree:
      return False, "no_agree"
    if ids and ids & self._ids:
      return False, "already_tracked"
    if now - self._last < self.cooldown_s:
      return False, "cooldown"
    self._ids.update(ids)
    if len(self._ids) > 64:
      self._ids = set(list(self._ids)[-64:])
    self._last = now
    self._hold_until = now + self.hold_s
    self._hold_ids = set(ids)
    return True, "chimed"


def _point(raw) -> dict | None:
  if isinstance(raw, dict):
    src = raw
    get = src.get
  else:
    def get(name, default=None):
      return getattr(raw, name, default)
  d_rel = _finite(get("dRel", get("d_rel")))
  y_rel = _finite(get("yRel", get("y_rel")))
  v_rel = _finite(get("vRel", get("v_rel")))
  if d_rel is None or y_rel is None or v_rel is None:
    return None
  if not bool(get("measured", True)):
    return None
  track = get("trackId", get("track_id", 0))
  try:
    track_id = int(track)
  except (TypeError, ValueError):
    track_id = 0
  yv = _finite(get("yvRel", get("yv_rel")))
  rcs = _finite(get("rcs", get("rcsDb", get("amplitude"))))
  return {
    "id": track_id,
    "x": d_rel + RADAR_TO_CAMERA_M,
    "y": -y_rel,
    "v_rel": v_rel,
    "yv_rel": yv,
    "rcs": rcs,
  }


def _iter_items(value):
  if value is None:
    return
  try:
    iterator = iter(value)
  except TypeError:
    return
  yield from iterator


def _field(obj, *names):
  if isinstance(obj, dict):
    for name in names:
      if name in obj:
        return obj[name]
    return None
  for name in names:
    try:
      return getattr(obj, name)
    except Exception:
      continue
  return None


def _xy_prob(item, xy_name: str | None = None):
  """Device-frame (x, y, prob) from a model lead, or None."""
  if isinstance(item, (tuple, list)):
    if len(item) < 3:
      return None
    x, y, prob = _finite(item[0]), _finite(item[1]), _finite(item[2])
    if x is None or y is None or prob is None or prob < LEAD_PROB:
      return None
    if len(item) > 3:
      try:
        return x, y, prob, int(item[3])
      except (TypeError, ValueError):
        return x, y, prob
    return x, y, prob
  prob = _finite(_field(item, "prob"))
  if prob is None or prob < LEAD_PROB:
    return None
  if xy_name:
    xyva = _field(item, xy_name)
    try:
      x, y = _finite(xyva[0]), _finite(xyva[1])
    except Exception:
      return None
  else:
    raw_x, raw_y = _field(item, "x"), _field(item, "y")
    try:
      x = _finite(raw_x if isinstance(raw_x, (int, float)) else raw_x[0])
      y = _finite(raw_y if isinstance(raw_y, (int, float)) else raw_y[0])
    except Exception:
      return None
  if x is None or y is None:
    return None
  return x, y, prob


def vehicle_exclusion_points(leads_v3=(), leads_v2=(), radar_leads=()):
  """Every tracked vehicle the chime must not treat as an obstacle.

  Model leadsV3 (lead0, lead1, and any further entries) and legacy
  leads V2 are device-frame points. A radard lead (leadOne and leadTwo)
  is included when status is set: its track id, and its position with
  y flipped from left-positive yRel to device +right. Prob under 0.40
  is ignored for the model. A selected radar lead is kept even if
  modelProb is low.

  Returns (track_ids, points). Points are (x, y, prob) or
  (x, y, prob, track_id).
  """
  ids: list[int] = []
  points: list[tuple] = []
  for item in _iter_items(leads_v3):
    parsed = _xy_prob(item)
    if parsed is not None:
      points.append(parsed)
  for item in _iter_items(leads_v2):
    parsed = _xy_prob(item, "xyva") if not isinstance(item, (tuple, list)) else _xy_prob(item)
    if parsed is not None:
      points.append(parsed)
  for lead in _iter_items(radar_leads):
    if lead is None or not bool(_field(lead, "status")):
      continue
    d_rel = _finite(_field(lead, "dRel", "d_rel"))
    y_rel = _finite(_field(lead, "yRel", "y_rel"))
    if d_rel is None or y_rel is None:
      continue
    prob = _finite(_field(lead, "modelProb", "prob"))
    if prob is None or prob < LEAD_PROB:
      prob = 1.0
    track = _field(lead, "radarTrackId", "radar_track_id")
    try:
      track_id = int(track) if track is not None else -1
    except (TypeError, ValueError):
      track_id = -1
    x = d_rel + RADAR_TO_CAMERA_M
    y = -y_rel
    if track_id >= 0:
      ids.append(track_id)
      points.append((x, y, prob, track_id))
    else:
      points.append((x, y, prob))
  return ids, points


def _line_lat(cone: ConeHint, x: float) -> float | None:
  pts = ((15.0, cone.lat_near), (30.0, cone.lat_mid), (45.0, cone.lat_far))
  if x <= pts[0][0]:
    return pts[0][1]
  if x >= pts[-1][0]:
    return pts[-1][1]
  for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
    if x0 <= x <= x1 and x1 > x0:
      t = (x - x0) / (x1 - x0)
      return y0 + t * (y1 - y0)
  return pts[-1][1]


def _cone_reject(lat: float, x: float, cone: ConeHint | None) -> str | None:
  if cone is None:
    return None
  line = _line_lat(cone, x)
  if cone.barrier and line is not None and abs(lat - line) < 1.15:
    return "barrier"
  # lat is +right. cone.side is the real side: +1 left, -1 right,
  # so a left line has negative lat. side +1 agrees with lat < 0.
  same_side = cone.side == 0 or (lat > 0.0) == (cone.side < 0) or abs(lat) < 0.4
  if cone.active and line is not None and abs(lat - line) < 0.80 and same_side:
    return "cone_line"
  if cone.parked and 1.5 < abs(lat) < 4.5:
    return "exclVehicle"
  return None


def _humanish(lat: float, v_lat: float, along: float, toward: float) -> bool:
  if 0.30 <= toward <= 3.4 and abs(along) < 4.0:
    return True
  if PATH_HALF_M < abs(lat) <= WALKER_EDGE_M and abs(along) < 2.2 and toward >= 0.15:
    return True
  if abs(lat) <= PATH_HALF_M + 0.3 and 2.0 < along <= HUMAN_ALONG_MAX_MPS and abs(v_lat) < 2.0:
    return True
  return False


def _zone_of(lat: float, v_lat: float, along: float) -> tuple[str, float]:
  if abs(lat) <= PATH_HALF_M:
    return "inPath", 0.0
  toward = _toward(lat, v_lat)
  dist = abs(lat) - PATH_HALF_M
  t_enter = dist / toward if toward >= ENTER_TOWARD_MPS else float("inf")
  horizon = HUMAN_ENTER_HORIZON_S if _humanish(lat, v_lat, along, toward) else ENTER_HORIZON_S
  if _humanish(lat, v_lat, along, toward) and PATH_HALF_M < abs(lat) <= WALKER_EDGE_M and toward >= 0.15:
    horizon = max(horizon, WALKER_ENTER_HORIZON_S)
  if math.isfinite(t_enter) and t_enter <= horizon:
    return "entering", t_enter
  if PATH_HALF_M < abs(lat) <= ANIMAL_OUTER_M:
    return "roadside", t_enter if math.isfinite(t_enter) else _nan()
  return "none", _nan()


def _fov_ok(x: float, lat: float) -> bool:
  return math.degrees(math.atan2(abs(lat), max(x, 0.5))) <= FOV_HALF_DEG


def _closing_time(x: float, v_rel: float) -> float:
  closing = -v_rel
  if closing <= 0.4:
    return _nan()
  return x / closing


def _ramp(value: float, noise: float, full: float) -> float:
  span = full - noise
  if span <= 1e-6:
    return 0.0
  return _clamp((abs(value) - noise) / span, 0.0, 1.0)


def lively_from_history(hist) -> float:
  """Max living-motion evidence in the last ~2 s. 0 is dead still.

  Ground speed, lateral speed, and position wander each count. One real
  cue in the window is enough. Jitter inside radar noise stays at 0.
  """
  if not hist:
    return 0.0
  now = float(hist[-1][0])
  window = [row for row in hist if now - float(row[0]) <= LIVELY_WINDOW_S]
  if not window:
    window = [hist[-1]]
  best = 0.0
  xs: list[float] = []
  ys: list[float] = []
  for _age, x, y, along, v_lat in window:
    xs.append(float(x))
    ys.append(float(y))
    speed = math.hypot(float(along), float(v_lat))
    best = max(best, _ramp(speed, SPEED_NOISE_MPS, SPEED_FULL_MPS), _ramp(v_lat, VLAT_NOISE_MPS, VLAT_FULL_MPS))
  span_t = float(window[-1][0]) - float(window[0][0])
  if span_t >= 1.0 and len(window) >= 4:
    mx = sum(xs) / len(xs)
    my = sum(ys) / len(ys)
    wander = max(math.hypot(x - mx, y - my) for x, y in zip(xs, ys))
    best = max(best, _ramp(wander, WANDER_NOISE_M, WANDER_FULL_M))
  return best


def scale_living_vision(vision: VisionScore, lively: float) -> VisionScore:
  """Lively raises animal/person class scores. Still lowers them, not to 0."""
  scale = LIVELY_FLOOR + (1.0 - LIVELY_FLOOR) * _clamp(lively if math.isfinite(lively) else 0.0, 0.0, 1.0)
  return replace(
    vision,
    human=_clamp(vision.human * scale, 0.0, 1.0),
    animal=_clamp(vision.animal * scale, 0.0, 1.0),
  )


def patch_signature(rows, grid: int = 8) -> tuple[float, ...]:
  """Coarse mean grid so a 5 Hz check can see the patch move."""
  data = _row_lists(rows)
  if not data:
    return ()
  height = len(data)
  width = min(len(row) for row in data)
  if width < 2 or height < 2:
    return ()
  cells = []
  for gy in range(grid):
    y0 = gy * height // grid
    y1 = max(y0 + 1, (gy + 1) * height // grid)
    for gx in range(grid):
      x0 = gx * width // grid
      x1 = max(x0 + 1, (gx + 1) * width // grid)
      acc = 0.0
      count = 0
      for row in data[y0:y1]:
        for value in row[x0:x1]:
          num = _finite(value)
          if num is not None:
            acc += num
            count += 1
      cells.append(acc / count if count else 0.0)
  return tuple(cells)


def patch_change_score(prev, rows) -> float:
  """0 when the patch matches, 1 when the gray levels really moved.

  About 12 counts of change is camera noise. Around 40 counts is motion.
  """
  old = prev if isinstance(prev, tuple) else patch_signature(prev)
  new = rows if isinstance(rows, tuple) else patch_signature(rows)
  if not old or not new or len(old) != len(new):
    return 0.0
  delta = sum(abs(a - b) for a, b in zip(old, new)) / len(old)
  return _clamp((delta - 12.0) / 28.0, 0.0, 1.0)


def _vision_in_range(x: float) -> bool:
  """Camera verdicts count only out to VISION_TRUST_M. Beyond that, radar only."""
  try:
    dist = float(x)
  except (TypeError, ValueError):
    return False
  return math.isfinite(dist) and dist <= VISION_TRUST_M


def _vision_class_score(vision: VisionScore, cls: str) -> float:
  if cls == "human":
    value = vision.human
  elif cls == "animal":
    value = vision.animal
  elif cls == "obstacle":
    value = vision.obstacle
  else:
    return 0.0
  try:
    score = float(value)
  except (TypeError, ValueError):
    return 0.0
  if not math.isfinite(score):
    return 0.0
  return score


def _unrotate_y(x: float, y: float, psi: float) -> float:
  """Lateral position with ego yaw removed. psi is +right radians since track start."""
  return math.sin(psi) * x + math.cos(psi) * y


def _fuse_class(radar_class: str, span: float, count: int, vision: VisionScore | None,
                raw: VisionScore | None = None) -> str:
  if count >= 2 and span >= LANE_SPAN_M:
    return "obstacle"
  # Camera score for the radar's living class, before the lively scale.
  # Below 0.30 the radar guess is not a person or an animal.
  camera = raw if raw is not None else vision
  if (
    camera is not None and camera.evaluated and radar_class in LIVING_CLASSES
    and _vision_class_score(camera, radar_class) < VISION_LIVING_MIN
  ):
    return "obstacle"
  if vision is None or not vision.evaluated or vision.conf < VISION_AGREE:
    return radar_class
  scores = (("human", vision.human), ("animal", vision.animal), ("obstacle", vision.obstacle))
  best, best_s = max(scores, key=lambda item: item[1])
  ordered = sorted((s for _, s in scores), reverse=True)
  margin = best_s - (ordered[1] if len(ordered) > 1 else 0.0)
  if radar_class in ("human", "animal") and best_s < 0.45:
    return radar_class
  if best_s >= 0.30 and margin >= 0.08:
    return best
  return radar_class


def _radar_class(members: list[dict], span: float, zone: str) -> str:
  count = len(members)
  lats = [m["lat"] for m in members]
  if count >= 2 and span >= LANE_SPAN_M and min(lats) < 0.6 and max(lats) > -0.6:
    return "obstacle"
  best = "unknown"
  for m in members:
    toward = _toward(m["lat"], m["v_lat"])
    along = m["along"]
    if _humanish(m["lat"], m["v_lat"], along, toward) and (count == 1 or span < 1.5):
      return "human"
    animalish = (
      (2.0 <= abs(m["v_lat"]) <= 12.0 and abs(along) < 5.0)
      or (toward >= 1.6 and abs(along) < 4.5 and toward > 3.2)
    )
    if animalish and (count == 1 or span < 1.6):
      best = "animal"
  if best == "animal":
    return best
  if zone == "inPath" and count >= 2 and span >= LANE_SPAN_M:
    return "obstacle"
  return "unknown"


def _cluster_reject(members: list[dict]) -> str | None:
  xs = [m["x"] for m in members]
  lats = [m["lat"] for m in members]
  count = len(members)
  if count < 2:
    return None
  x_span = max(xs) - min(xs)
  lat_span = max(lats) - min(lats)
  if lat_span >= OVERHEAD_SPAN_M and x_span < OVERHEAD_DEPTH_M and count >= 3:
    return "overhead"
  mean_lat = sum(lats) / count
  var = sum((v - mean_lat) ** 2 for v in lats) / count
  if x_span >= 10.0 and math.sqrt(var) <= 0.55 and count >= 4 and 0.5 <= abs(mean_lat) <= 3.0 and x_span > lat_span:
    return "cone_line"
  straddles = min(lats) < -0.2 and max(lats) > 0.2
  if x_span < 8.0 and lat_span >= 1.0 and abs(mean_lat) > 2.0 and not straddles and count >= 3:
    return "exclVehicle"
  return None


def _still_members(members: list[dict]) -> bool:
  """Ground speed and lively score both say this cluster has not moved."""
  for member in members:
    if abs(float(member["along"])) > SLOW_ALONG_MPS:
      return False
    if float(member.get("lively") or 0.0) >= LIVELY_CHIME:
      return False
  return True


def _group_zone(members: list[dict]) -> str:
  lats = [m["lat"] for m in members]
  span = (max(lats) - min(lats)) if len(lats) > 1 else 0.0
  straddles = min(lats) < -0.2 and max(lats) > 0.2 and span >= 1.2
  zones = [_zone_of(m["lat"], m["v_lat"], m["along"])[0] for m in members]
  if straddles or any(zone == "inPath" for zone in zones):
    return "inPath"
  if any(zone == "entering" for zone in zones):
    return "entering"
  if any(zone == "roadside" for zone in zones):
    return "roadside"
  return "none"


def _phantom_in_path(members: list[dict], v_ego: float) -> bool:
  """Stationary in-path return at highway speed that is not being closed.

  A real stopped object approaches at about ego speed and is gone after
  the detection window. A 100 s 'in path' track at 25 m/s is a ghost or
  a vehicle the lead logic missed, and it must not wake the camera.
  """
  if v_ego < HIGHWAY_MPS:
    return False
  if not any(abs(m["lat"]) <= PATH_HALF_M for m in members):
    return False
  if not all(abs(m["along"]) <= SLOW_ALONG_MPS for m in members):
    return False
  age = max(float(m["age"]) for m in members)
  if age < 0.35:
    return False
  closed = max(float(m["first_x"]) - float(m["x"]) for m in members)
  need = min(8.0, 0.35 * v_ego * age)
  if closed + 1e-6 < need:
    return True
  window = (X_MAX_M - X_MIN_M) / max(v_ego, 1.0)
  return age > window + 2.0


def _candidate_block(members: list[dict], v_ego: float) -> str | None:
  """Drop a cluster before vision. None means the camera may look."""
  if _phantom_in_path(members, v_ego):
    return "not_closing"
  zone = _group_zone(members)
  if zone == "none":
    return "off_path"
  lats = [m["lat"] for m in members]
  span = (max(lats) - min(lats)) if len(lats) > 1 else 0.0
  radar_class = _radar_class(members, span, zone)
  beyond = _beyond_corridor(min(abs(m["lat"]) for m in members))
  # Radar-only "human" on a stationary tall roadside return. Posts and
  # delineators outside 3 ft of the corridor never reach the camera.
  if radar_class == "human" and _still_members(members) and beyond > HUMAN_BEYOND_M:
    return "off_zone"
  if radar_class == "animal" and beyond > ANIMAL_BEYOND_M:
    return "off_zone"
  return None


def _popin(members: list[dict]) -> bool:
  if any(m["first_x"] >= POPIN_X_M + 4.0 for m in members):
    return False
  if any(m["x"] > POPIN_X_M + 2.0 for m in members):
    return False
  if max(m["age"] for m in members) >= 0.55:
    return False
  lats = [m["lat"] for m in members]
  if len(members) >= 2 and max(lats) - min(lats) >= LANE_SPAN_M:
    return False
  return all(abs(m["v_lat"]) < 0.40 and abs(m["along"]) < SLOW_ALONG_MPS for m in members)


def _still_in_lane(ret) -> bool:
  """Stationary and already in the path. Crossing and roadside are not."""
  try:
    along = float(ret["along"])
    v_lat = float(ret["v_lat"])
    lat = float(ret["lat"])
  except (TypeError, ValueError, KeyError):
    return False
  if not all(math.isfinite(v) for v in (along, v_lat, lat)):
    return False
  if abs(along) > SLOW_ALONG_MPS or abs(v_lat) >= ENTER_TOWARD_MPS:
    return False
  zone, _t = _zone_of(lat, v_lat, along)
  return zone == "inPath"


def _lead_flags(ret, leads, model_leads) -> tuple[bool, bool]:
  """(old vehicle exclusion, night drive-over lead agreement)."""
  try:
    x = float(ret["x"])
    y = float(ret["y"])
    tid = int(ret["id"])
  except (TypeError, ValueError, KeyError):
    return False, False
  lead_ids = set()
  for raw in leads or ():
    try:
      lead_ids.add(int(raw))
    except (TypeError, ValueError):
      continue
  old = tid in lead_ids
  agree = old
  for lead in model_leads or ():
    try:
      lx, ly, prob = float(lead[0]), float(lead[1]), float(lead[2])
    except (TypeError, ValueError, IndexError):
      continue
    if not all(math.isfinite(v) for v in (lx, ly, prob)) or prob < LEAD_PROB:
      continue
    lead_id = None
    if len(lead) > 3 and lead[3] is not None:
      try:
        lead_id = int(lead[3])
      except (TypeError, ValueError):
        lead_id = None
    same = lead_id is not None and lead_id == tid
    dx = abs(x - lx)
    dy = abs(y - ly)
    if same or (dx < LEAD_X_M and dy < LEAD_Y_M):
      old = True
    tol = max(DRIVE_OVER_LEAD_X_M, DRIVE_OVER_LEAD_X_FRAC * abs(x))
    if prob >= DRIVE_OVER_LEAD_PROB and (same or (dx <= tol and dy <= DRIVE_OVER_LEAD_Y_M)):
      agree = True
  return old, agree


def _lead_keep(ret, leads, model_leads) -> str | None:
  """'exclVehicle', or None when the return stays in the scan.

  A stationary in-lane lead is kept and flagged so a stalled car can
  chime at night. Every other lead match is still excluded. A failure
  excludes a known lead id and otherwise leaves the return alone.
  """
  try:
    old, agree = _lead_flags(ret, leads, model_leads)
  except Exception:
    try:
      if int(ret.get("id", -1)) in set(leads or ()):
        return "exclVehicle"
    except (TypeError, ValueError):
      return None
    return None
  if agree and _still_in_lane(ret):
    ret["model_lead"] = True
    return None
  if old or agree:
    return "exclVehicle"
  return None


def _drive_over_case(sample: ObstacleSample, hit: _Hit) -> bool:
  """Night, stationary, already in the lane, and not a person or animal."""
  if not is_night_lighting(hit.lighting):
    return False
  if not sample.active or sample.zone != "inPath":
    return False
  if sample.object_class in LIVING_CLASSES:
    return False
  if abs(float(hit.along)) > SLOW_ALONG_MPS:
    return False
  if abs(float(hit.v_lat)) >= ENTER_TOWARD_MPS:
    return False
  if float(sample.lively_score) >= LIVELY_CHIME:
    return False
  return True


def _night_lead_may_chime(sample: ObstacleSample, hit: _Hit) -> bool:
  """Model-lead matches stay excluded except this night stalled-car case."""
  try:
    return bool(hit.model_lead) and _drive_over_case(sample, hit)
  except Exception:
    return False


def _look_light(item) -> bool:
  return len(item) > 3 and bool(item[3])


def _look_confirms(item, cls: str) -> bool:
  try:
    conf, look_cls, score = float(item[0]), item[1], float(item[2])
  except (TypeError, ValueError, IndexError):
    return False
  return look_cls == cls and conf >= VISION_LOOK_MIN and score >= VISION_CLASS_MIN


def _drive_over_block(sample: ObstacleSample, vision: VisionScore | None, hist) -> str:
  """Why this night in-lane return must not chime. Empty means it may."""
  if float(sample.range_m) > DRIVE_OVER_SOLID_M:
    return "beyond_60"
  dominated = bool(vision is not None and getattr(vision, "light_dominated", False))
  cls = sample.object_class
  clean = [item for item in hist if _look_confirms(item, cls) and not _look_light(item)]
  lit = dominated or any(_look_light(item) and _look_confirms(item, cls) for item in hist)
  if dominated or (lit and len(clean) < VISION_LOOKS_NEED):
    return "light"
  conf = None if vision is None else _finite(getattr(vision, "conf", None))
  evaluated = bool(vision is not None and getattr(vision, "evaluated", False))
  if not evaluated or conf is None or conf < VISION_AGREE or len(clean) < VISION_LOOKS_NEED:
    return "no_solid"
  return ""


def _as_vehicle(sample: ObstacleSample) -> ObstacleSample:
  """Daytime (and any non-night) lead match: same quiet result as before."""
  return replace(
    sample,
    active=False,
    agree=False,
    brake_gate=False,
    in_path=False,
    entering=False,
    zone="none",
    reject_reason="exclVehicle",
    chimed=False,
    chime_reason="exclVehicle",
    drive_over_reason="",
  )


class PathObstacleDetector:
  """One candidate per radar frame. State is track age and the chime memory."""

  def __init__(self) -> None:
    self._tracks: dict[int, dict] = {}
    self._chime = ChimeGate()
    self._vision_looks: dict[int, list] = {}

  def reset(self) -> None:
    self._tracks.clear()
    self._chime.reset()
    self._vision_looks.clear()

  def begin(self, points, v_ego: float, path_x, path_y, dt: float, *,
            lead_ids=(), cone: ConeHint | None = None, model_leads=(),
            lighting: float = 1.0, deadline: float | None = None,
            yaw_rate: float = 0.0) -> _Hit:
    dt = 0.0 if dt is None else float(dt)
    if not math.isfinite(dt) or dt < 0.0:
      dt = 0.0
    dt = min(dt, 0.5)
    try:
      v_ego_f = float(v_ego)
    except (TypeError, ValueError):
      v_ego_f = 0.0
    if not math.isfinite(v_ego_f):
      v_ego_f = 0.0
    try:
      yaw = float(yaw_rate)
    except (TypeError, ValueError):
      yaw = 0.0
    if not math.isfinite(yaw):
      yaw = 0.0
    light = lighting_score() if lighting is None else float(lighting)
    if not math.isfinite(light):
      light = 1.0

    try:
      _check_deadline(deadline)
      parsed = []
      try:
        iterator = list(points) if points is not None else []
      except TypeError:
        iterator = []
      for raw in iterator:
        _check_deadline(deadline)
        item = _point(raw)
        if item is not None:
          parsed.append(item)
      if len(parsed) > MAX_POINTS:
        parsed.sort(key=lambda item: item["x"])
        del parsed[MAX_POINTS:]
      returns = self._advance(parsed, dt, v_ego_f, path_x, path_y, deadline, yaw)
      leads = {int(i) for i in lead_ids if i is not None}
      near_reason = "no_candidate"
      near_ret = None
      kept = []
      for ret in returns:
        reason = self._keep_reason(ret, leads, cone, model_leads)
        if reason is None:
          kept.append(ret)
        elif near_ret is None or ret["x"] < near_ret["x"]:
          near_ret = ret
          near_reason = reason
      # A stopped lead is its own candidate. Clustering it with a neighbor
      # would hide that neighbor behind the lead, which daytime still ignores.
      lead_pts = [ret for ret in kept if ret.get("model_lead")]
      other_pts = [ret for ret in kept if not ret.get("model_lead")]
      clusters = _cluster(other_pts)
      clusters.extend([pt] for pt in lead_pts)
      best = None
      best_key = None
      best_lead = None
      best_lead_key = None
      rejected = near_reason
      rejected_members = [near_ret] if near_ret is not None else []
      for group in clusters:
        _check_deadline(deadline)
        why = _cluster_reject(group)
        if why is None and _popin(group):
          why = "road_surface"
        if why is None:
          why = _candidate_block(group, v_ego_f)
        if why is not None:
          if best is None and best_lead is None:
            rejected = why
            rejected_members = group
          continue
        hit = self._score_group(group, light, v_ego_f)
        if hit is None:
          continue
        key = (
          1 if hit.zone in ("inPath", "entering") else 0,
          1 if hit.radar_class in ("human", "animal") else 0,
          hit.radar_conf,
          -hit.x,
        )
        # Anything that is not a lead keeps the slot, as it did before a
        # lead match was left in the scan. The lead is used only when it
        # is the only thing here, so a daytime obstacle is not hidden.
        if hit.model_lead:
          if best_lead_key is None or key > best_lead_key:
            best_lead = hit
            best_lead_key = key
        elif best_key is None or key > best_key:
          best = hit
          best_key = key
      if best is not None:
        return best
      if best_lead is not None:
        return best_lead
      return self._miss(rejected_members, rejected, light)
    except _OverBudget:
      return self._miss([], "over_budget", light)

  def commit(self, hit: _Hit, vision: VisionScore | None, *,
             chime_enabled: bool = True, now: float = 0.0,
             vision_motion: float = 0.0) -> ObstacleSample:
    try:
      motion = float(vision_motion)
    except (TypeError, ValueError):
      motion = 0.0
    if not math.isfinite(motion):
      motion = 0.0
    lively = _clamp(max(float(hit.lively), _clamp(motion, 0.0, 1.0)), 0.0, 1.0)
    use_vision = bool(hit.active and vision is not None and vision.evaluated)
    trust_vision = bool(use_vision and vision is not None and _vision_in_range(hit.x))
    scored = scale_living_vision(vision, lively) if trust_vision and vision is not None else None
    if trust_vision:
      final_class = _fuse_class(hit.radar_class, hit.span_m, hit.cluster_count, scored, raw=vision)
    else:
      final_class = _fuse_class(hit.radar_class, hit.span_m, hit.cluster_count, None)
    if not hit.cluster_count:
      final_class = "unknown"
    if trust_vision and vision is not None:
      self._note_vision_look(
        hit.track_id, vision.conf, final_class, _vision_class_score(vision, final_class),
        bool(getattr(vision, "light_dominated", False)),
      )
    vision_conf = vision.conf if trust_vision and vision is not None else None
    fused, w_r, w_v, agree = fuse_scores(hit.radar_conf, vision_conf, hit.lighting)
    agree = bool(agree and hit.active and trust_vision)
    zone = hit.zone if hit.reject_reason in ("none", "not_persistent") else "none"
    in_path = zone == "inPath"
    entering = zone == "entering"
    t_enter = 0.0 if in_path else hit.time_to_enter
    brake = bool(agree and zone in BRAKE_ZONES)
    if trust_vision and scored is not None and vision is not None:
      vh, va, vo = scored.human, scored.animal, scored.obstacle
      vconf = vision.conf
    elif use_vision and vision is not None:
      vh, va, vo = vision.human, vision.animal, vision.obstacle
      vconf = vision.conf
    else:
      vh = va = vo = vconf = _nan()
    sample = ObstacleSample(
      active=bool(hit.active),
      track_id=hit.track_id,
      range_m=hit.x,
      lateral=hit.lateral,
      v_rel=hit.v_rel,
      v_lat=hit.v_lat,
      radar_conf=hit.radar_conf if hit.cluster_count else 0.0,
      vision_conf=vconf,
      vision_evaluated=use_vision,
      lighting_score=hit.lighting,
      w_radar=w_r,
      w_vision=w_v,
      fused_score=fused if hit.cluster_count else 0.0,
      agree=agree,
      reject_reason="none" if hit.active else hit.reject_reason,
      in_path=in_path,
      time_to_reach=hit.time_to_reach,
      object_class=final_class,
      vision_conf_human=vh,
      vision_conf_animal=va,
      vision_conf_obstacle=vo,
      cluster_count=hit.cluster_count,
      span_m=hit.span_m,
      time_to_enter=t_enter,
      entering=entering,
      zone=zone,
      brake_gate=brake,
      lively_score=lively if hit.cluster_count else 0.0,
      member_ids=hit.member_ids,
    )
    if hit.model_lead and not _night_lead_may_chime(sample, hit):
      return _as_vehicle(sample)
    if sample.active and _highway_still_unconfirmed(sample, hit.v_ego):
      return sample.with_chime(False, "no_agree")
    if sample.active:
      blocked = _class_zone_block(sample)
      if blocked is not None:
        return sample.with_chime(False, blocked)
      drive_over = self._drive_over_reason(sample, vision, hit)
      if drive_over:
        # Log only. brakeGate stays closed so a later consumer cannot
        # treat a headlight or a rail as something to stop for.
        return replace(
          sample, chimed=False, chime_reason="drive_over", drive_over_reason=drive_over,
          brake_gate=False, agree=False,
        )
      if sample.agree and not self._vision_confirmed(sample.track_id, sample.object_class):
        return sample.with_chime(False, "no_agree")
    chimed, reason = self._chime.consider(sample, chime_enabled, now)
    return sample.with_chime(chimed, reason)

  def _drive_over_reason(self, sample: ObstacleSample, vision: VisionScore | None, hit: _Hit) -> str:
    """Suppression token, or "" to leave the chime on the existing rules.

    Errors return "" so a broken crop or a bad field falls back to the
    chime behavior from before this rule.
    """
    try:
      if hit.model_lead or not _drive_over_case(sample, hit):
        return ""
      return _drive_over_block(sample, vision, self._vision_looks.get(int(hit.track_id)) or ())
    except Exception:
      return ""

  def _note_vision_look(self, track_id: int, conf: float, cls: str, score: float,
                        light: bool = False) -> None:
    """Keep the last three trusted camera looks for this track."""
    try:
      tid = int(track_id)
    except (TypeError, ValueError):
      return
    if tid == 0:
      return
    try:
      conf_f = float(conf)
    except (TypeError, ValueError):
      conf_f = 0.0
    if not math.isfinite(conf_f):
      conf_f = 0.0
    hist = self._vision_looks.pop(tid, [])
    hist.append((conf_f, cls, float(score), bool(light)))
    if len(hist) > VISION_LOOKS:
      del hist[:-VISION_LOOKS]
    self._vision_looks[tid] = hist
    while len(self._vision_looks) > 32:
      self._vision_looks.pop(next(iter(self._vision_looks)))

  def _vision_confirmed(self, track_id: int, cls: str) -> bool:
    """True when 2 of the last 3 looks clear 0.60 overall and 0.40 on cls."""
    try:
      tid = int(track_id)
    except (TypeError, ValueError):
      return False
    hist = self._vision_looks.get(tid) or ()
    good = 0
    for item in hist:
      conf, look_cls, score = item[0], item[1], item[2]
      if look_cls == cls and conf >= VISION_LOOK_MIN and score >= VISION_CLASS_MIN:
        good += 1
    return good >= VISION_LOOKS_NEED

  def update(self, points, v_ego: float, path_x, path_y, dt: float, *,
             lead_ids=(), cone: ConeHint | None = None, model_leads=(),
             vision: VisionScore | None = None, lighting: float = 1.0,
             chime_enabled: bool = True, now: float = 0.0,
             vision_motion: float = 0.0, yaw_rate: float = 0.0) -> ObstacleSample:
    hit = self.begin(
      points, v_ego, path_x, path_y, dt,
      lead_ids=lead_ids, cone=cone, model_leads=model_leads, lighting=lighting,
      yaw_rate=yaw_rate,
    )
    return self.commit(hit, vision, chime_enabled=chime_enabled, now=now, vision_motion=vision_motion)

  def _advance(self, parsed, dt, v_ego, path_x, path_y, deadline=None, yaw: float = 0.0) -> list[dict]:
    seen = set()
    out = []
    for item in parsed:
      _check_deadline(deadline)
      seen.add(item["id"])
      prev = self._tracks.get(item["id"])
      if prev is None or prev["gap"] > 0.35:
        state = {
          "age": dt, "first_x": item["x"], "gap": 0.0, "y": item["y"],
          "v_lat": 0.0, "hist": [], "yaw_int": 0.0,
        }
      else:
        state = prev
        state["age"] = float(state["age"]) + dt
        state["gap"] = 0.0
      # A fixed target at range x slides sideways at about -yaw*x while the
      # car turns. Take that out of v_lat and out of the wander history.
      prev_y = float(state["y"])
      measured = None
      if item["yv_rel"] is not None:
        measured = -float(item["yv_rel"])
      elif prev is not None and prev["gap"] <= 0.35 and dt > 1e-4:
        measured = (item["y"] - prev_y) / dt
      if measured is not None:
        state["v_lat"] = measured + yaw * float(item["x"])
      state["y"] = item["y"]
      state["x"] = item["x"]
      dpsi = yaw * dt
      if dpsi > 0.25:
        dpsi = 0.25
      elif dpsi < -0.25:
        dpsi = -0.25
      state["yaw_int"] = float(state.get("yaw_int", 0.0)) + dpsi
      y_fix = _unrotate_y(float(item["x"]), float(item["y"]), float(state["yaw_int"]))
      hist = state.setdefault("hist", [])
      hist.append((
        float(state["age"]), float(item["x"]), y_fix,
        float(item["v_rel"]) + v_ego, float(state["v_lat"]),
      ))
      state["hist"] = [row for row in hist if float(state["age"]) - float(row[0]) <= LIVELY_WINDOW_S + 0.05][-24:]
      self._tracks[item["id"]] = state
      py = path_y_at_x(path_x, path_y, item["x"])
      lat = item["y"] if py is None else item["y"] - py
      out.append({
        "id": item["id"],
        "x": item["x"],
        "y": item["y"],
        "lat": lat,
        "v_rel": item["v_rel"],
        "v_lat": float(state["v_lat"]),
        "along": item["v_rel"] + v_ego,
        "rcs": item["rcs"],
        "age": float(state["age"]),
        "first_x": float(state["first_x"]),
        "lively": lively_from_history(state.get("hist") or ()),
      })
    for track_id in list(self._tracks):
      if track_id in seen:
        continue
      self._tracks[track_id]["gap"] = float(self._tracks[track_id]["gap"]) + dt
      if self._tracks[track_id]["gap"] > 0.50:
        del self._tracks[track_id]
    return out

  def _keep_reason(self, ret, leads, cone, model_leads) -> str | None:
    x, lat = ret["x"], ret["lat"]
    if x < X_MIN_M or x > X_MAX_M:
      return "out_of_range"
    if not _fov_ok(x, ret["y"]):
      return "fov"
    lead_why = _lead_keep(ret, leads, model_leads)
    if lead_why is not None:
      return lead_why
    cone_why = _cone_reject(lat, x, cone)
    if cone_why is not None:
      return cone_why
    along, v_lat = ret["along"], ret["v_lat"]
    # Cars and motorcycles going with traffic. A stopped car the model
    # has not tagged is left for the chime. A sideways dart is not traffic.
    if along > TRAFFIC_ALONG_MPS and abs(v_lat) <= TRAFFIC_VLAT_MPS:
      return "exclVehicle"
    toward = _toward(lat, v_lat)
    human = _humanish(lat, v_lat, along, toward)
    moving = abs(v_lat) >= LATERAL_MOVE_MPS
    slow = abs(along) <= SLOW_ALONG_MPS
    if along < -2.5 and abs(v_lat) <= TRAFFIC_VLAT_MPS and not human:
      return "exclVehicle"
    if not (slow or moving or human) or abs(along) > HUMAN_ALONG_MAX_MPS + 0.5:
      return "exclVehicle"
    zone, _t = _zone_of(lat, v_lat, along)
    if zone == "none":
      return "off_path"
    return None

  def _score_group(self, members: list[dict], lighting: float, v_ego: float = 0.0) -> _Hit | None:
    members = sorted(members, key=lambda m: m["x"])
    rep = members[0]
    lats = [m["lat"] for m in members]
    span = max(lats) - min(lats) if len(lats) > 1 else 0.0
    straddles = min(lats) < -0.2 and max(lats) > 0.2 and span >= 1.2
    zones = []
    for m in members:
      zone, t_enter = _zone_of(m["lat"], m["v_lat"], m["along"])
      zones.append((zone, t_enter, m))
    if straddles or any(z == "inPath" for z, _, _ in zones):
      zone, t_enter = "inPath", 0.0
    elif any(z == "entering" for z, _, _ in zones):
      entering = [(t, m) for z, t, m in zones if z == "entering"]
      t_enter, _m = min(entering, key=lambda item: item[0])
      zone = "entering"
    elif any(z == "roadside" for z, _, _ in zones):
      zone = "roadside"
      finite = [t for z, t, _m in zones if z == "roadside" and math.isfinite(t)]
      t_enter = min(finite) if finite else _nan()
    else:
      return None
    radar_class = _radar_class(members, span, zone)
    age = max(m["age"] for m in members)
    persist = _clamp(age / PERSIST_FULL_S, 0.0, 1.0)
    active = age + 1e-9 >= PERSIST_S
    if zone == "inPath":
      place = 1.0
    elif zone == "entering":
      horizon = HUMAN_ENTER_HORIZON_S if radar_class == "human" else ENTER_HORIZON_S
      t_use = t_enter if math.isfinite(t_enter) else horizon
      place = _clamp(1.0 - t_use / horizon, 0.35, 0.95)
      t_reach = _closing_time(rep["x"], rep["v_rel"])
      if radar_class == "human" and math.isfinite(t_reach) and math.isfinite(t_enter) and t_enter < t_reach:
        place = min(1.0, place + 0.12)
    else:
      place = 0.72
    penalty = 0.0
    if zone == "roadside" and all(abs(m["v_lat"]) < 0.35 and abs(m["along"]) < 1.0 for m in members):
      penalty += 0.22
    rcs_vals = [m["rcs"] for m in members if m["rcs"] is not None]
    if rcs_vals and max(rcs_vals) < -10.0:
      penalty += 0.30
    if rcs_vals and max(rcs_vals) < -15.0:
      penalty += 0.25
    clutter = _clamp(1.0 - penalty, 0.20, 1.0)
    conf = _clamp(0.18 + 0.78 * persist * place * clutter, 0.0, 1.0)
    if len(members) >= 2 and span >= LANE_SPAN_M and zone == "inPath":
      conf = min(1.0, conf + 0.06)
    if not active:
      conf = min(conf, 0.49)
    v_lat = max(members, key=lambda m: _toward(m["lat"], m["v_lat"]))["v_lat"]
    return _Hit(
      active=active,
      track_id=rep["id"],
      member_ids=tuple(m["id"] for m in members),
      x=rep["x"],
      y=rep["y"],
      lateral=rep["lat"],
      v_rel=rep["v_rel"],
      v_lat=v_lat,
      along=rep["along"],
      radar_conf=conf,
      radar_class=radar_class,
      zone=zone,
      time_to_reach=_closing_time(rep["x"], rep["v_rel"]),
      time_to_enter=0.0 if zone == "inPath" else t_enter,
      cluster_count=len(members),
      span_m=span,
      reject_reason="none" if active else "not_persistent",
      lighting=lighting,
      lively=max(float(m.get("lively", 0.0)) for m in members),
      v_ego=float(v_ego),
      model_lead=any(bool(m.get("model_lead")) for m in members),
    )

  def _miss(self, members, reason: str, lighting: float) -> _Hit:
    if not members or members[0] is None:
      return _Hit(
        False, 0, (), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "unknown", "none",
        _nan(), _nan(), 0, 0.0, reason, lighting, 0.0,
      )
    group = [m for m in members if m is not None]
    rep = min(group, key=lambda m: m["x"])
    lats = [m["lat"] for m in group]
    return _Hit(
      False, rep["id"], tuple(m["id"] for m in group), rep["x"], rep["y"], rep["lat"],
      rep["v_rel"], rep["v_lat"], rep["along"], 0.0, "unknown", "none",
      _closing_time(rep["x"], rep["v_rel"]), _nan(), len(group),
      (max(lats) - min(lats)) if len(lats) > 1 else 0.0, reason, lighting,
      max(float(m.get("lively", 0.0)) for m in group),
    )


def _obstacle_body(msg):
  if msg is None:
    return None
  try:
    which = msg.which()
  except Exception:
    return msg
  if which == "pathObstacleNAP":
    return getattr(msg, which)
  return msg


def hit_to_msg(hit: _Hit, dest) -> None:
  """Write every _Hit field onto a pathObstacleNAP builder."""
  dest.active = bool(hit.active)
  dest.trackId = int(hit.track_id) & 0xFFFFFFFFFFFFFFFF
  ids = tuple(int(i) & 0xFFFFFFFFFFFFFFFF for i in hit.member_ids)
  slots = dest.init("memberIds", len(ids))
  for i, mid in enumerate(ids):
    slots[i] = mid
  dest.range = float(hit.x)
  dest.y = float(hit.y)
  dest.lateral = float(hit.lateral)
  dest.vRel = float(hit.v_rel)
  dest.vLat = float(hit.v_lat)
  dest.along = float(hit.along)
  dest.radarConf = float(hit.radar_conf) if hit.cluster_count else 0.0
  radar_class = _enum_name(hit.radar_class, CLASS_ORDER)
  dest.radarClass = radar_class
  dest.objectClass = radar_class
  zone = _enum_name(hit.zone, ZONE_ORDER)
  dest.zone = zone
  dest.inPath = zone == "inPath"
  dest.entering = zone == "entering"
  dest.timeToReach = float(hit.time_to_reach)
  dest.timeToEnter = float(hit.time_to_enter)
  dest.clusterCount = int(min(255, max(0, hit.cluster_count)))
  dest.spanM = float(hit.span_m)
  dest.rejectReason = str(hit.reject_reason)
  dest.lightingScore = float(hit.lighting)
  dest.livelyScore = float(hit.lively) if hit.cluster_count else 0.0
  w_r, w_v = fusion_weights(hit.lighting)
  dest.wRadar = w_r
  dest.wVision = w_v
  dest.visionEvaluated = False
  dest.visionConf = _nan()
  dest.visionConfHuman = _nan()
  dest.visionConfAnimal = _nan()
  dest.visionConfObstacle = _nan()
  dest.modelLeadAgree = bool(hit.model_lead)
  dest.driveOverReason = ""


def hit_from_msg(msg) -> _Hit:
  """Inverse of hit_to_msg. Accepts the event or the struct."""
  body = _obstacle_body(msg)
  try:
    member_ids = tuple(int(i) for i in body.memberIds)
  except Exception:
    member_ids = ()
  try:
    model_lead = bool(body.modelLeadAgree)
  except Exception:
    model_lead = False
  return _Hit(
    active=bool(body.active),
    track_id=int(body.trackId),
    member_ids=member_ids,
    x=float(body.range),
    y=float(body.y),
    lateral=float(body.lateral),
    v_rel=float(body.vRel),
    v_lat=float(body.vLat),
    along=float(body.along),
    radar_conf=float(body.radarConf),
    radar_class=_enum_name(body.radarClass, CLASS_ORDER),
    zone=_enum_name(body.zone, ZONE_ORDER),
    time_to_reach=float(body.timeToReach),
    time_to_enter=float(body.timeToEnter),
    cluster_count=int(body.clusterCount),
    span_m=float(body.spanM),
    reject_reason=str(body.rejectReason or ""),
    lighting=float(body.lightingScore),
    lively=float(body.livelyScore),
    model_lead=model_lead,
  )


def _quantize(value, digits: int):
  try:
    num = float(value)
  except (TypeError, ValueError):
    return None
  if not math.isfinite(num):
    return None
  return round(num, digits)


class ObstacleStage:
  """Radar half of the detector. radard calls step() after radarState is sent.

  Returns a hit to publish, or None when this cycle should stay silent.
  commit() stays on the camera helper; this object only scans.
  """

  def __init__(self) -> None:
    self.det = PathObstacleDetector()
    self.heartbeat = False
    self._log_on = True
    self._log_override: bool | None = None
    self._log_check_t = -1.0
    self._skip = 0
    self._exc_times: list[float] = []
    self._disabled = False
    self._sig = None
    self._last_pub = -1e9
    self._reset_done = False
    self._speed_on = False
    self._speed_reset = False

  def note_failure(self) -> None:
    """Count a failure outside begin(), such as a publish error."""
    self._note_exception()

  def _note_exception(self) -> None:
    if self._disabled:
      return
    now = time.monotonic()
    self._exc_times = [t for t in self._exc_times if now - t <= BREAKER_WINDOW_S]
    self._exc_times.append(now)
    if len(self._exc_times) < BREAKER_TRIPS:
      return
    self._disabled = True
    try:
      from openpilot.common.swaglog import cloudlog
      cloudlog.exception("path obstacle detector disabled until next drive")
    except Exception:
      pass

  def _logging(self, now: float) -> bool:
    if self._log_override is not None:
      return bool(self._log_override)
    if self._log_check_t >= 0.0 and now - self._log_check_t < 1.0:
      return self._log_on
    self._log_check_t = now
    try:
      from openpilot.common.params import Params
      self._log_on = bool(Params().get_bool(PARAM_OBSTACLE_LOG))
    except Exception:
      self._log_on = True
    return self._log_on

  def _at_speed(self, v_ego) -> bool:
    """Hysteresis: arm at 15 mph, drop below about 13 mph."""
    try:
      speed = float(v_ego)
    except (TypeError, ValueError):
      speed = 0.0
    if not math.isfinite(speed):
      speed = 0.0
    if self._speed_on:
      if speed < SPEED_DISABLE_MPS:
        self._speed_on = False
    elif speed >= SPEED_ENABLE_MPS:
      self._speed_on = True
    return self._speed_on

  def _idle(self, now: float, reason: str) -> _Hit | None:
    if now - self._last_pub < HEARTBEAT_S:
      return None
    self._last_pub = now
    self.heartbeat = True
    return self.det._miss([], reason, 1.0)

  def step(self, points, v_ego, path_x, path_y, dt, lead_ids, cone, model_leads, now,
           yaw_rate: float = 0.0) -> _Hit | None:
    """Scan one radar frame. None means nothing to publish."""
    self.heartbeat = False
    try:
      now_f = float(now)
    except (TypeError, ValueError):
      now_f = time.monotonic()
    if not math.isfinite(now_f):
      now_f = time.monotonic()
    if self._disabled:
      return self._idle(now_f, "disabled")
    if not self._logging(now_f):
      if not self._reset_done:
        self.det.reset()
        self._reset_done = True
        self._sig = None
      return self._idle(now_f, "disabled")
    self._reset_done = False
    if not self._at_speed(v_ego):
      self._skip = 0
      if not self._speed_reset:
        self.det.reset()
        self._sig = None
        self._speed_reset = True
      return self._idle(now_f, "lowSpeed")
    self._speed_reset = False
    if self._skip > 0:
      self._skip -= 1
      return self._idle(now_f, "over_budget")
    started = time.monotonic()
    try:
      hit = self.det.begin(
        points, v_ego, path_x, path_y, dt,
        lead_ids=lead_ids, cone=cone, model_leads=model_leads,
        lighting=1.0, deadline=started + BUDGET_S, yaw_rate=yaw_rate,
      )
    except Exception:
      self._note_exception()
      try:
        self.det.reset()
      except Exception:
        pass
      return self._idle(now_f, "error")
    if hit.reject_reason == "over_budget":
      elapsed = max(0.0, time.monotonic() - started)
      self._skip = max(1, math.ceil(elapsed / BUDGET_S))
    sig = (
      bool(hit.active), int(hit.track_id), tuple(hit.member_ids),
      hit.zone, hit.radar_class, hit.reject_reason,
      _quantize(hit.x, 1), _quantize(hit.lateral, 2),
    )
    changed = sig != self._sig
    due = now_f - self._last_pub >= HEARTBEAT_S
    if not (hit.active or changed or due):
      return None
    self._sig = sig
    self._last_pub = now_f
    self.heartbeat = bool(due and not hit.active and not changed)
    return hit


def _cluster(members: list[dict]) -> list[list[dict]]:
  n = len(members)
  if n == 0:
    return []
  parent = list(range(n))

  def find(i: int) -> int:
    while parent[i] != i:
      parent[i] = parent[parent[i]]
      i = parent[i]
    return i

  for i in range(n):
    for j in range(i):
      if abs(members[i]["x"] - members[j]["x"]) <= CLUSTER_DX_M and abs(members[i]["lat"] - members[j]["lat"]) <= CLUSTER_DY_M:
        ri, rj = find(i), find(j)
        if ri != rj:
          parent[rj] = ri
  groups: dict[int, list[dict]] = {}
  for i, member in enumerate(members):
    groups.setdefault(find(i), []).append(member)
  return list(groups.values())


def save_ppm(path: str, rows) -> bool:
  """Write a grayscale P5 thumbnail. False on any I/O problem."""
  try:
    data = _row_lists(rows)
    if not data:
      return False
    height = len(data)
    width = min(len(r) for r in data)
    if width < 2 or height < 2:
      return False
    raw = bytearray()
    for row in data:
      for value in row[:width]:
        try:
          pix = int(value)
        except (TypeError, ValueError):
          pix = 0
        raw.append(max(0, min(255, pix)))
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    with open(path, "wb") as handle:
      handle.write(header)
      handle.write(raw)
    return True
  except OSError:
    return False


def prune_thumbs(directory: str, max_files: int = 24, max_bytes: int = 2_000_000) -> None:
  """Drop the oldest thumbnails until count and bytes are inside the cap."""
  import os
  try:
    names = [n for n in os.listdir(directory) if n.endswith(".ppm")]
  except OSError:
    return
  files = []
  for name in names:
    full = os.path.join(directory, name)
    try:
      files.append((os.path.getmtime(full), os.path.getsize(full), full))
    except OSError:
      continue
  files.sort()
  total = sum(size for _, size, _ in files)
  while files and (len(files) > max_files or total > max_bytes):
    _mtime, size, full = files.pop(0)
    total -= size
    try:
      os.remove(full)
    except OSError:
      pass
