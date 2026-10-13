"""Roundabout Steering Assist (Pre-AP, NAPRoundaboutAssist, default Off).

Sep 28 (R126, R128): through the Soco / Dellwood / Jonathan Creek ring the model's curvature was a wide, slow S. Sep 29 the
curvature blend curled left at 1.8x the model and the driver overrode it. Sep 30: once latched the car FOLLOWS THE MAP'S LANE
CIRCLE; the blend, the left-curl ramp and the map-only auto-exit are gone.

  * Pose: 1 Hz GNSS (0.66 s latency, odometry-compensated) fused with wheel speed + yaw rate, local frame at the ring center.
  * Entry (unchanged): farther than D_TURN_M from the ring edge the target can only hold / add right. Inside it the target is the
    circle-follow curvature (1 s preview), circulation-side curl capped at 2.2 m/s² and held while > 3 mph over the ring speed.
  * Latch: tangent in the band -> lane circle R -/+ half-width/2 (inner if on the island side of R and the pose is trustworthy).
  * Ring (latched): the circle follower at that lane radius; output <= RING_CAP_RATIO (1.15) x the model's circulation-side
    curvature and <= RING_A_LAT_MAX (2.2 m/s²), >= RING_FLOOR_RATIO x the model, correction slew RING_SLEW (0.035), whatever the pose.
  * Release: only a held same-side stalk (StalkTipTurn.is_turn), a steering pull toward the exit, or an override (steeringPressed /
    lat off). No map-only exit: with no blinker the car stays on the ring. Any held stalk / pressed / torque: weight 0 at once, the
    driver always wins. ALC tips are suppressed while latched (NAPRoundaboutLatched Param). Camera edge limits still apply.

Curvature sign is openpilot's (positive = right). Pure functions and one small state class, replayable without cereal.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib import roundabout_ring_hold as RH
from openpilot.selfdrive.controls.lib.roundabout_ring_log import LOG_RAW_X, _interp_xy, raw_ring_summary  # noqa: F401
from openpilot.selfdrive.controls.lib.stalk_tip_turn import PARAM_RING_LATCHED, StalkTipTurn
from openpilot.selfdrive.mapd.roundabout import roundabout_target_ms
from openpilot.selfdrive.mapd.roundabout_map import RingGeometry, local_xy, roundabout_comfort_speed_ms

PARAM_ROUNDABOUT_ASSIST = "NAPRoundaboutAssist"

GPS_LATENCY_S = 0.66         # qcom gpsLocation vs logMonoTime, measured on the Sep 29 route (R126/R128 xcorr gave 0.9–1.0 s)
GPS_EXT_LATENCY_S = 0.2      # ublox gpsLocationExternal (not in the Sep 28 logs; unvalidated)
FIX_FRESH_S = 1.5
FIX_STALE_S = 3.0
HISTORY_S = 4.0
INNOV_GOOD_M = 3.0
INNOV_BAD_M = 8.0
INNOV_JUMP_M = 15.0
INNOV_POOR_M = 6.0
SPEED_TOL_MS = 2.5            # GNSS speed lags ~1 s like position; loose check
POS_GAIN = 0.5
HEADING_GAIN = 0.3
GOOD_FIXES = 3

V_MIN = 2.0                  # m/s
V_MAX = 13.5                 # m/s (~30 mph); the ring slow-down should be well under this
ENGAGE_EDGE_M = 25.0         # start blending (weight ramps) this far from the ring edge
FULL_EDGE_M = 12.0
D_TURN_M = 9.0               # hold the right entry until this far from the ring edge
LEAVE_EDGE_M = 60.0          # forget a finished ring once this far away
A_LAT_MAX = 3.0            # m/s^2: cap on the guide target away from the circulation direction (right entry)
A_LAT_CIRC_MAX = 2.2       # m/s^2: cap on the circulation-side (left on a CCW ring) transition target before the latch, at actual speed
FOLLOW_OMEGA = 0.7            # rad/s, circle-follow bandwidth
FOLLOW_ZETA = 0.9
PRE_LATCH_HW = 0.75          # entry aims R + 0.75 hw (outer lane, away from the island)
OUT_SLEW = 0.035            # 1/m per s, max change of the blended output while active (poor / unverified pose)
OUT_SLEW_GOOD = 0.07         # ...with a good pose (R126/R128 fixtures need it to stay in the lane band)
HANDOFF_DK = 0.00005         # output must be this close to the model before handing back (well under one slew step)
PREVIEW_S = 1.0             # s, circle-follow preview (R126/R128 replay: 0.6-1.2 s trade inner vs outer drift)
TARGET_SLEW = 0.035          # 1/m per s, transition rate of the guide target (poor / unverified pose)
TARGET_SLEW_GOOD = 0.07
W_MAX = 0.8
W_RATE = 2.0                 # phase weight ramp, 1/s
DK_MAX = 0.045               # max |correction| (1/m)
TANGENT_DEG = 25.0
RING_CAP_RATIO = 1.15        # latched output <= this x the model's circulation-side curvature
RING_A_LAT_MAX = 2.2         # m/s²: lateral-accel cap of the latched output (the 18 mph ring itself needs ~3.0)
RING_CAP_FLOOR = 0.0         # 1/m: allowance on top of the ratio (0 = strictly 1.15x the model)
RING_SLEW = 0.035            # 1/m per s, correction slew once latched (any pose quality)
RING_REF_HOLD_S = 0.0        # cap reference = the model's circulation-side PEAK over this long; 0 = the model now (strict 1.15x)
RING_FLOOR_RATIO = 0.85      # ...and never less than this x the model's circulation-side curvature
LANE_CIRCLE_FRAC = 0.5       # lane circle = R -/+ this x half-width
PULL_TORQUE_NM = 1.5         # driver torque: either side = weight 0; toward the exit for PULL_RELEASE_S = release
PULL_RELEASE_S = 0.3
OVERRIDE_S = 0.5             # steeringPressed (or lat off) this long: override, release
FAILSAFE_EDGE_M = 15.0       # safety only: far outside the ring band
POSE_LOST_S = 8.0            # safety only: no GNSS fix this long
EDGE_MARGIN_M = 0.3
HALF_CAR_M = 0.98
APPROACH_HALF_W_M = 4.0
MATCH_COS = 0.9              # approach segment (stored in travel direction) within ~25° of the heading
MATCH_MAX_M = 12.0
LANE_SNAP_M = 1.75           # one lane off the OSM centerline is not a GNSS error
BIAS_GAIN = 0.3
BIAS_MAX_M = 8.0
BEARING_ACC_UNRELIABLE_DEG = 30.0   # no latch / no inner clamp beyond this (Sep 29: 30-50°)
BIAS_UNRELIABLE_M = 2.0
BEARING_ACC_POOR_DEG = 20.0         # poor: latch allowed on the outer lane circle only, slow pre-latch slews
BIAS_DRIFT_M = 1.0                  # bias moved more than this over BIAS_WINDOW_S: poor pose
BIAS_LATCH_DRIFT_M = 1.0            # ...and more than this: not bias-consistent, do not latch yet
BIAS_WINDOW_S = 3.0
SPEED_HOLD_MS = 3.0 * 0.44704       # hold the circulation-side curl until within 3 mph of the ring speed
LOG_PERIOD_S = 1.0
RAW_LOG_EDGE_M = 60.0         # raw-model summary from this far from the ring edge until released
LOG_RAW_PERIOD_S = 0.8        # ~1.25 Hz raw-model summary near / in a ring (qlog via errorLogMessage)


def _interp(x: float, xp: tuple[float, float], fp: tuple[float, float]) -> float:
  if x <= xp[0]:
    return fp[0]
  if x >= xp[1]:
    return fp[1]
  return fp[0] + (fp[1] - fp[0]) * (x - xp[0]) / (xp[1] - xp[0])


def _hint_on_ring(hint) -> bool:
  try:
    return bool(hint is not None and getattr(hint, "on_roundabout", False))
  except Exception:
    return False


def _wrap(a: float) -> float:
  return (a + math.pi) % (2.0 * math.pi) - math.pi


def _rot(x: float, y: float, d: float) -> tuple[float, float]:
  """Rotate a vector clockwise by d (bearing sense): heading b → b + d."""
  c, s = math.cos(d), math.sin(d)
  return x * c + y * s, -x * s + y * c


class PoseTracker:
  """GNSS + odometry pose in a local east/north frame (m) with bearing (rad, cw from north)."""

  def __init__(self) -> None:
    self.origin: tuple[float, float] | None = None
    self.reset()

  def reset(self) -> None:
    self.xo = self.yo = self.psio = 0.0
    self.t = None
    self.hist: deque = deque()
    self.initialized = False
    self.dpsi = 0.0
    self.tx = self.ty = 0.0
    self.innov = INNOV_BAD_M
    self.n_good = 0
    self.last_fix_t = -1e9
    self.bacc = 0.0     # last reported bearing accuracy (deg); 0 = unknown

  def set_origin(self, lat: float, lon: float) -> None:
    if self.origin is None or abs(self.origin[0] - lat) > 1e-7 or abs(self.origin[1] - lon) > 1e-7:
      self.origin = (float(lat), float(lon))
      keep_hist, keep_t = self.hist, self.t
      xo, yo, psio = self.xo, self.yo, self.psio
      self.reset()
      self.hist, self.t, self.xo, self.yo, self.psio = keep_hist, keep_t, xo, yo, psio

  def predict(self, t: float, v: float, yaw_rate: float) -> None:
    if not (math.isfinite(t) and math.isfinite(v) and math.isfinite(yaw_rate)):
      return   # a NaN would poison the odometry for the rest of the drive (seen in R126 qlog yaw)
    if self.t is not None:
      dt = min(max(t - self.t, 0.0), 0.2)
      self.psio += yaw_rate * dt
      self.xo += v * math.sin(self.psio) * dt
      self.yo += v * math.cos(self.psio) * dt
    self.t = t
    self.hist.append((t, self.xo, self.yo, self.psio, v))
    while self.hist and self.hist[0][0] < t - HISTORY_S:
      self.hist.popleft()

  def _odo_at(self, t: float):
    h = self.hist
    if h and h[-1][0] < t <= h[-1][0] + 0.05:
      t = h[-1][0]   # fix processed just before this cycle's predict
    if not h or t < h[0][0] or t > h[-1][0]:
      return None
    lo, hi = 0, len(h) - 1
    while hi - lo > 1:
      mid = (lo + hi) // 2
      if h[mid][0] <= t:
        lo = mid
      else:
        hi = mid
    a, b = h[lo], h[hi]
    f = 0.0 if b[0] <= a[0] else (t - a[0]) / (b[0] - a[0])
    return tuple(a[k] + f * (b[k] - a[k]) for k in range(1, 5))

  def _world(self, xo: float, yo: float) -> tuple[float, float]:
    x, y = _rot(xo, yo, self.dpsi)
    return x + self.tx, y + self.ty

  def fix(self, t_log: float, lat: float, lon: float, bearing_deg: float | None,
          gps_speed: float | None = None, bearing_acc_deg: float | None = None,
          latency_s: float = GPS_LATENCY_S) -> None:
    if self.origin is None or not all(math.isfinite(z) for z in (t_log, lat, lon)):
      return
    self.bacc = float(bearing_acc_deg) if (bearing_acc_deg is not None and math.isfinite(bearing_acc_deg)
                                           and bearing_acc_deg > 0.0) else 0.0
    odo = self._odo_at(t_log - latency_s)
    if odo is None:
      return
    xo, yo, psio, v_fix = odo
    fx, fy = local_xy(lat, lon, self.origin[0], self.origin[1])
    b_ok = (bearing_deg is not None and math.isfinite(bearing_deg) and v_fix > 3.0
            and (bearing_acc_deg is None or bearing_acc_deg <= 0.0 or bearing_acc_deg < 10.0))
    speed_ok = gps_speed is None or not math.isfinite(gps_speed) or abs(gps_speed - v_fix) < max(SPEED_TOL_MS, 0.25 * v_fix)
    if not self.initialized:
      if not b_ok:
        return
      self.dpsi = _wrap(math.radians(bearing_deg) - psio)
      wx, wy = _rot(xo, yo, self.dpsi)
      self.tx, self.ty = fx - wx, fy - wy
      self.initialized = True
      self.innov = INNOV_GOOD_M
      self.n_good = 1 if speed_ok else 0
      self.last_fix_t = t_log
      return
    px, py = self._world(xo, yo)
    ex, ey = fx - px, fy - py
    e = math.hypot(ex, ey)
    if e > INNOV_JUMP_M:
      # GNSS jump or a bad track: re-anchor, confidence starts over.
      self.initialized = False
      self.n_good = 0
      self.innov = INNOV_BAD_M
      self.fix(t_log, lat, lon, bearing_deg, gps_speed, bearing_acc_deg, latency_s)
      self.n_good = 0
      return
    self.innov = 0.7 * self.innov + 0.3 * e
    ax, ay = px + POS_GAIN * ex, py + POS_GAIN * ey
    if b_ok:
      self.dpsi += HEADING_GAIN * _wrap(math.radians(bearing_deg) - (psio + self.dpsi))
    wx, wy = _rot(xo, yo, self.dpsi)
    self.tx, self.ty = ax - wx, ay - wy
    # One poor fix costs two good ones (a hard reset made the assist drop out mid-ring on R126).
    self.n_good = self.n_good + 1 if (e < INNOV_POOR_M and speed_ok) else max(self.n_good - 2, 0)
    self.last_fix_t = t_log

  def pose(self) -> tuple[float, float, float] | None:
    if not self.initialized:
      return None
    x, y = self._world(self.xo, self.yo)
    return x, y, self.psio + self.dpsi

  def confidence(self, t: float) -> float:
    if not self.initialized:
      return 0.0
    c_fix = _interp(t - self.last_fix_t, (FIX_FRESH_S, FIX_STALE_S), (1.0, 0.0))
    c_innov = _interp(self.innov, (INNOV_GOOD_M, INNOV_BAD_M), (1.0, 0.0))
    c_count = min(1.0, self.n_good / GOOD_FIXES)
    return min(c_fix, c_innov, c_count)


def _arc_ahead(x: float, y: float, psi: float, v: float, yaw_rate: float, dt: float) -> tuple[float, float, float]:
  """Pose after dt s at constant speed and yaw rate (x east, y north, psi cw from north)."""
  n = 5
  h = dt / n
  for _ in range(n):
    psi_m = psi + 0.5 * yaw_rate * h
    x += v * math.sin(psi_m) * h
    y += v * math.cos(psi_m) * h
    psi += yaw_rate * h
  return x, y, psi


def circle_follow_curvature(x: float, y: float, psi: float, v: float, r_ref: float, ccw: bool) -> float:
  """Curvature (positive right) that follows the circle r_ref (center 0,0) in the circulation direction.

  Feed-forward 1/r_ref plus a critically-damped correction on radial offset
  e = r - r_ref and outward heading angle phi (radial speed = v sin phi):
    k_center = 1/r_ref + (2 ZETA OMEGA v sin(phi) + OMEGA^2 e) / v^2
  which makes e'' + 2 ZETA OMEGA e' + OMEGA^2 e ~= 0. Moving against the
  circulation counts as fully outward (turn toward the ring direction).
  """
  r = math.hypot(x, y)
  if r < 1e-3 or r_ref <= 0.0:
    return 0.0
  sense = 1.0 if ccw else -1.0
  hx, hy = math.sin(psi), math.cos(psi)            # heading unit vector (x east, y north)
  radial = (hx * x + hy * y) / r                    # sin(phi), outward positive
  tangential = sense * (-hx * y + hy * x) / r       # along the circulation
  sin_phi = radial if tangential > 0.0 else math.copysign(1.0, radial if radial != 0.0 else 1.0)
  vv = max(v, 3.0)
  k_center = 1.0 / r_ref + (2.0 * FOLLOW_ZETA * FOLLOW_OMEGA * vv * sin_phi + FOLLOW_OMEGA ** 2 * (r - r_ref)) / (vv * vv)
  return -sense * k_center


def _dist_to_polyline(x: float, y: float, pts: list[tuple[float, float]]) -> float:
  best = 1e9
  for (ax, ay), (bx, by) in zip(pts, pts[1:], strict=False):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    f = 0.0 if L2 <= 1e-9 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
    best = min(best, math.hypot(x - (ax + f * dx), y - (ay + f * dy)))
  return best


def map_match_confidence(ring: RingGeometry, x: float, y: float) -> float:
  """1 on the mapped ring band / an approach road, 0 once ≥ 4 m outside every mapped road."""
  r = math.hypot(x, y)
  excess = max(0.0, abs(r - ring.radius_m) - ring.half_width_m)
  on_approach_side = r > ring.radius_m + ring.half_width_m
  if on_approach_side:
    if ring.approaches:
      d_app = min(_dist_to_polyline(x, y, a) for a in ring.approaches if len(a) >= 2)
      excess = min(excess, max(0.0, d_app - APPROACH_HALF_W_M))
    else:
      return 0.7 * _interp(excess, (1.0, 45.0), (1.0, 0.0)) if excess < 45.0 else 0.0
  return _interp(excess, (1.0, 5.0), (1.0, 0.0))


def map_match_offset(ring: RingGeometry, x: float, y: float, psi: float) -> tuple[float, float, float] | None:
  """GNSS offset from the mapped road that no lane can explain: (nx, ny, excess_m) or None.

  On an approach road: signed perpendicular offset from the nearest roughly
  parallel segment, minus LANE_SNAP_M (a car can legitimately be one lane off
  the OSM centerline). On the ring: radial offset beyond the lane centers.
  """
  r = math.hypot(x, y)
  if r < 8.0:
    return None
  R, hw = ring.radius_m, ring.half_width_m
  if r <= R + hw + 2.0:
    e = r - R
    ex = math.copysign(max(abs(e) - 0.5 * hw, 0.0), e)
    return x / r, y / r, ex
  best = None
  hx, hy = math.sin(psi), math.cos(psi)
  for pts in ring.approaches:
    for (ax, ay), (bx, by) in zip(pts, pts[1:], strict=False):
      dx, dy = bx - ax, by - ay
      L = math.hypot(dx, dy)
      if L < 1e-3 or (dx * hx + dy * hy) / L < MATCH_COS:   # travel direction (see ring_geometry)
        continue
      f = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (L * L)))
      px, py = ax + f * dx, ay + f * dy
      d = math.hypot(x - px, y - py)
      if d <= MATCH_MAX_M and (best is None or d < best[0]):
        nx, ny = dy / L, -dx / L        # right-hand normal of the segment
        best = (d, nx, ny, (x - px) * nx + (y - py) * ny)
  if best is None:
    return None
  _, nx, ny, e = best
  return nx, ny, math.copysign(max(abs(e) - LANE_SNAP_M, 0.0), e)


def ring_confidence(ring: RingGeometry) -> float:
  c = min(_interp(ring.rms_m, (1.0, 3.0), (1.0, 0.0)), _interp(ring.coverage_deg, (200.0, 300.0), (0.0, 1.0)))
  return c if ring.tagged else 0.7 * c


@dataclass
class EdgeInfo:
  """Camera lateral bounds at lookahead x (m). y positive right (device frame)."""
  x: float
  path_y: float
  left_y: float | None = None
  right_y: float | None = None


def limit_correction_by_edges(dk: float, edges: EdgeInfo | None) -> float:
  """Scale the correction so the predicted path keeps a margin to lane lines / road edges."""
  if edges is None or abs(dk) < 1e-9:
    return dk
  x = max(edges.x, 3.0)
  dy = 0.5 * dk * x * x
  margin = HALF_CAR_M + EDGE_MARGIN_M
  if dk < 0.0 and edges.left_y is not None:
    allowed = edges.left_y + margin - edges.path_y   # most-negative lateral move allowed
    dy = max(dy, min(0.0, allowed))
  if dk > 0.0 and edges.right_y is not None:
    allowed = edges.right_y - margin - edges.path_y
    dy = min(dy, max(0.0, allowed))
  return 2.0 * dy / (x * x)


def lat_limited(k: float, v: float, sense: float) -> float:
  """Clip a target curvature: A_LAT_CIRC_MAX toward the circulation side (left on CCW), A_LAT_MAX away from it (right entry)."""
  v2 = max(v * v, 1.0)
  k_circ, k_other = A_LAT_CIRC_MAX / v2, A_LAT_MAX / v2
  lo, hi = (-k_circ, k_other) if sense > 0.0 else (-k_other, k_circ)
  return min(max(k, lo), hi)


def hold_curl(k_raw: float, model_k: float, sense: float) -> float:
  """Never add more circulation-side curvature than the model asks for (hold / add right only on CCW)."""
  return max(model_k, k_raw) if sense > 0.0 else min(model_k, k_raw)


def ring_cap(ref_k: float, model_k: float, v: float, sense: float) -> float:
  """Most circulation-side curvature (-sense * k) the latched output may carry: RING_CAP_RATIO x ref_k (+ floor), limited to
  RING_A_LAT_MAX at the actual speed, never below the model itself. ref_k = the model's circulation-side peak over RING_REF_HOLD_S."""
  s_model = -sense * model_k
  if s_model <= 0.0:
    return s_model          # model not curving with the circulation: the assist never steers against it
  return max(s_model, min(RING_CAP_RATIO * max(ref_k, s_model) + RING_CAP_FLOOR, RING_A_LAT_MAX / max(v * v, 1.0)))


def ring_floor(ref_k: float, v: float) -> float:
  """Least circulation-side curvature the latched output may carry: RING_FLOOR_RATIO x ref_k."""
  return min(RING_FLOOR_RATIO * ref_k, RING_A_LAT_MAX / max(v * v, 1.0)) if ref_k > 0.0 else -1e9


@dataclass
class DriverInput:
  """What the driver is doing with the wheel / stalk (controlsd -> guide)."""
  stalk_dir: int = 0          # turnSignalStalkState: 1 left, 2 right
  stalk_held: bool = False    # StalkTipTurn.is_turn: held > 0.40 s (driver turn, not an ALC tip)
  pressed: bool = False       # carState.steeringPressed
  torque: float = 0.0         # carState.steeringTorque (+ left)


def pose_quality(bearing_acc_deg: float, bias_m: float, bias_drift_m: float, innov_m: float) -> tuple[bool, bool]:
  """(unreliable, poor). Unreliable: never latch (bearing accuracy > 30°, or bias > 2 m with bearing > 20°: Sep 29 11:57:48).
  Poor: latch on the outer lane only, slow pre-latch slews."""
  unreliable = bearing_acc_deg > BEARING_ACC_UNRELIABLE_DEG or \
    (bias_m > BIAS_UNRELIABLE_M and bearing_acc_deg > BEARING_ACC_POOR_DEG)
  poor = unreliable or bearing_acc_deg > BEARING_ACC_POOR_DEG or innov_m > INNOV_POOR_M
  return unreliable, poor


CAP_FALL = 0.3               # x RING_SLEW: how fast the widened ring cap is given up
IDLE, ACTIVE, EXITING, DONE = "idle", "active", "exiting", "done"


class RoundaboutGuide:
  def __init__(self) -> None:
    self.pose = PoseTracker()
    self.ring: RingGeometry | None = None
    self.ring_v = 0.0
    self.exits: list[float] = []
    self.reset_state()
    self.debug: dict = {}

  def reset_state(self) -> None:
    self.phase = IDLE
    self.phase_w = 0.0
    self.k_tgt = None
    self.latched = False
    self.r_ref = None
    self.theta_entry = 0.0
    self.travel = 0.0
    self.r_lane = None
    self.k_ref = 0.0
    self.model_hist: deque = deque()
    self.release = ""
    self.override_s = 0.0
    self.pull_s = 0.0
    self.drv_s = 0.0
    self.cap_extra = 0.0
    self.last_t = None
    self.turning = False
    self.bias = (0.0, 0.0)
    self.dk_out = 0.0
    self.prev_out: float | None = None
    self.prev_model: float | None = None
    self.bias_hist: deque = deque()
    self.latch_t: float | None = None
    self.latch_poor = False
    self.d_edge: float | None = None
    self.events: list[str] = []
    self._blocked = ""
    self._holding = False

  def _ev(self, msg: str) -> None:
    if len(self.events) < 50:
      self.events.append(msg)

  def set_ring(self, ring: RingGeometry | None) -> None:
    if ring is None:
      return
    if self.ring is not None and set(ring.way_ids) == set(self.ring.way_ids):
      return
    self.ring = ring
    self.exits = RH.exit_angles(ring)
    self.ring_v = float(roundabout_target_ms(roundabout_comfort_speed_ms(ring.radius_m, ring.lanes, ring.maxspeed_ms)))
    self.pose.set_origin(ring.lat, ring.lon)
    self.reset_state()

  def has_way(self, way_id: int) -> bool:
    return self.ring is not None and int(way_id) in self.ring.way_ids

  def gnss(self, t_log: float, lat: float, lon: float, bearing_deg: float | None,
           speed: float | None = None, bearing_acc_deg: float | None = None,
           latency_s: float = GPS_LATENCY_S) -> None:
    if self.pose.origin is None:
      self.pose.set_origin(lat, lon)
    self.pose.fix(t_log, lat, lon, bearing_deg, speed, bearing_acc_deg, latency_s)
    self._update_bias()

  def _update_bias(self) -> None:
    """Map-matched GNSS bias: the part of the offset from the mapped road no lane explains."""
    raw = self.pose.pose()
    if self.ring is None or raw is None or self.pose.n_good < 1:
      return
    m = map_match_offset(self.ring, raw[0], raw[1], raw[2])
    if m is None:
      return
    nx, ny, ex = m
    bx, by = self.bias
    bn = bx * nx + by * ny
    bn += BIAS_GAIN * (ex - bn)
    bx, by = bx + (bn - (bx * nx + by * ny)) * nx, by + (bn - (bx * nx + by * ny)) * ny
    mag = math.hypot(bx, by)
    if mag > BIAS_MAX_M:
      bx, by = bx * BIAS_MAX_M / mag, by * BIAS_MAX_M / mag
    self.bias = (bx, by)

  def _release_reason(self, drv: DriverInput, sense: float, lat_active: bool, dt: float, v: float, d_edge: float,
                      fix_age: float, theta: float = 0.0) -> str:
    """Why a latched ring is released ('' = stay). Only the driver (stalk / steering pull / override) or a hard
    failsafe: there is no map-only exit, with no blinker the car stays on the ring."""
    exit_dir = 2 if sense > 0.0 else 1                      # exit side: right on a CCW ring
    if drv.stalk_held and drv.stalk_dir == exit_dir:
      return "stalk"
    if drv.stalk_dir == exit_dir and RH.on_exit_arm(theta, sense, self.exits, d_edge):
      return "exit_blinker"                               # blinker on while on the exit arm: hand back to the model
    self.pull_s = self.pull_s + dt if -sense * drv.torque >= PULL_TORQUE_NM else 0.0
    if self.pull_s >= PULL_RELEASE_S:
      return "pull"
    hands = RH.counts_as_press(drv.pressed, drv.torque, sense) or (not lat_active and not drv.stalk_held)
    self.override_s = self.override_s + dt if hands else 0.0
    if self.override_s >= OVERRIDE_S:
      return "override"
    if v > V_MAX + 2.0:
      return "speed"
    if d_edge > FAILSAFE_EDGE_M:
      return "far_from_ring"
    if fix_age > POSE_LOST_S:
      return "pose_lost"
    return ""

  def corrected_pose(self) -> tuple[float, float, float] | None:
    raw = self.pose.pose()
    if raw is None:
      return None
    return raw[0] - self.bias[0], raw[1] - self.bias[1], raw[2]

  def _passthrough(self, model_k: float) -> float:
    self.dk_out = 0.0
    self.prev_out = self.prev_model = None
    return model_k

  def update(self, t: float, v: float, yaw_rate: float, model_k: float, *,
             enabled: bool, lat_active: bool, edges: EdgeInfo | None = None,
             driver: DriverInput | None = None) -> float:
    """Return the (possibly blended) curvature. Exactly model_k whenever inactive."""
    dt = 0.0 if self.last_t is None else min(max(t - self.last_t, 0.0), 0.2)
    self.last_t = t
    self.pose.predict(t, v, yaw_rate)
    self.debug = {"phase": self.phase, "w": 0.0}
    model_k = float(model_k)
    ring = self.ring
    pose = self.corrected_pose()
    if not enabled or ring is None or pose is None or not all(math.isfinite(z) for z in (model_k, v, yaw_rate)):
      self.phase_w, self.k_tgt, self.dk_out = 0.0, None, 0.0
      return self._passthrough(model_k)
    x, y, psi = pose
    R, hw = ring.radius_m, ring.half_width_m
    r = math.hypot(x, y)
    d_edge = r - (R + hw)
    self.d_edge = d_edge
    if self.phase == DONE:
      if d_edge > LEAVE_EDGE_M:
        self.reset_state()
      return self._passthrough(model_k)
    theta = math.atan2(y, x)
    sense = 1.0 if ring.ccw else -1.0
    # Tangent (circulation) bearing vs heading.
    tx, ty = -sense * y / max(r, 1e-3), sense * x / max(r, 1e-3)
    tan_brg = math.atan2(tx, ty)
    align = abs(math.degrees(_wrap(psi - tan_brg)))
    radial_v = v * (math.sin(psi) * x + math.cos(psi) * y) / max(r, 1e-3)

    if self.phase == IDLE:
      if d_edge <= ENGAGE_EDGE_M and radial_v < -0.5 and V_MIN <= v <= V_MAX:
        self.phase = ACTIVE
      else:
        self.phase_w, self.k_tgt, self.dk_out = 0.0, None, 0.0
        return self._passthrough(model_k)

    # Pose quality: an unreliable pose (bearing accuracy > 30° / bias > 2 m) or a still-moving bias never latches.
    bias_m = math.hypot(*self.bias)
    self.bias_hist.append((t, self.bias[0], self.bias[1]))
    while self.bias_hist and self.bias_hist[0][0] < t - BIAS_WINDOW_S:
      self.bias_hist.popleft()
    drift = math.hypot(self.bias[0] - self.bias_hist[0][1], self.bias[1] - self.bias_hist[0][2])
    unreliable, poor = pose_quality(self.pose.bacc, bias_m, drift, self.pose.innov)
    can_latch = not unreliable and drift <= BIAS_LATCH_DRIFT_M
    ready = RH.latch_ready(align, d_edge, TANGENT_DEG)
    if not self.latched and ready and not can_latch:
      kind = "unreliable" if unreliable else "bias_unsettled"
      if kind != self._blocked:
        self._blocked = kind
        self._ev(f"latch blocked {kind} bacc={self.pose.bacc:.0f} bias={bias_m:.1f} drift={drift:.1f} r={r:.1f} d_edge={d_edge:.1f}")
    if not self.latched and ready and can_latch:
      self.latched = True
      self.latch_t = t
      self.latch_poor = poor
      inner = r <= R and not (poor or bias_m > BIAS_UNRELIABLE_M)           # doubtful pose: never the inner lane
      if inner:
        self.r_lane = R - LANE_CIRCLE_FRAC * hw
      else:     # doubtful pose: the outer lane circle if the car reads outside R, else the center line (never the island side)
        self.r_lane = R + LANE_CIRCLE_FRAC * hw if r > R else R
      self.r_ref = self.r_lane
      self.theta_entry = theta
      self.travel = 0.0
      self._ev(f"latched r={r:.1f} r_ref={self.r_ref:.1f} bias={bias_m:.1f} bacc={self.pose.bacc:.0f} poor={int(poor)} v={v:.1f}")
    elif self.latched:
      self.travel += math.degrees(sense * _wrap(theta - self.theta_entry))
      self.theta_entry = theta
    self.model_hist.append((t, -sense * model_k))
    while self.model_hist and self.model_hist[0][0] < t - RING_REF_HOLD_S:
      self.model_hist.popleft()
    self.k_ref = max(0.0, max(m for _, m in self.model_hist))
    drv = driver or DriverInput()
    drv_active, self.drv_s = RH.driver_wins(drv.stalk_held, drv.pressed, drv.torque, sense, self.drv_s, dt)
    if self.latched and self.phase != EXITING:
      why = self._release_reason(drv, sense, lat_active, dt, v, d_edge, t - self.pose.last_fix_t, theta)
      if why:
        self.release = why
        self._ev(f"released {why} r={r:.1f} travel={self.travel:.0f} v={v:.1f}")
        if why in ("stalk", "pull", "override", "exit_blinker"):      # the driver: hand back at once
          self.phase, self.k_tgt, self.phase_w = DONE, None, 0.0
          return self._passthrough(model_k)
        self.phase = EXITING                          # safety release: fade to the model (W_RATE, RING_SLEW)
    r_ref = self.r_ref if self.latched else R + PRE_LATCH_HW * hw   # outer part of the ring until tangent, then hold the lane
    # Evaluate the circle follower at the pose PREVIEW_S ahead (constant current yaw rate): leads the
    # target slew + actuator delay so the left transition starts before the car is already tangent.
    px, py, ppsi = _arc_ahead(x, y, psi, v, yaw_rate, PREVIEW_S)
    k_pp = circle_follow_curvature(px, py, ppsi, v, r_ref, ring.ccw)

    if d_edge > D_TURN_M and not self.turning:
      k_raw = max(model_k, k_pp)   # hold / add right only; never an early left
      phase_target = _interp(d_edge, (FULL_EDGE_M, ENGAGE_EDGE_M), (1.0, 0.0))
    else:
      self.turning = True
      k_raw = k_pp
      # Unreliable pose and not latched: fade the assist out, the model keeps this pass (Sep 29 11:57:48).
      phase_target = 0.0 if (unreliable and not self.latched) else 1.0
    lim, plain = (0.0, 0.0), True
    if self.latched:
      # Circulating: command the lane circle, within [ring_floor, ring_cap] whatever the pose quality.
      holding_now = v > self.ring_v + SPEED_HOLD_MS
      plain = drv.stalk_dir == (2 if sense > 0.0 else 1) or d_edge > RH.EXIT_ARM_EDGE_M or holding_now   # the old 1.15x-model limits
      lim = RH.ring_limits(ring_cap(self.k_ref, model_k, v, sense), ring_floor(self.k_ref, v), -sense * model_k, -sense * k_pp, v,
                           RING_A_LAT_MAX, RH.in_exit_window(theta, sense, self.exits), plain)
      s_cmd = max(min(-sense * k_pp, lim[0]), lim[1])
      k_raw = -sense * s_cmd
    else:
      k_raw = lat_limited(k_raw, v, sense)
    holding = v > self.ring_v + SPEED_HOLD_MS
    if holding:
      k_raw = hold_curl(k_raw, model_k, sense)
    if holding != self._holding:
      self._holding = holding
      self._ev(f"curl {'held' if holding else 'released'} v={v:.1f} ring_v={self.ring_v:.1f}")

    if self.phase == ACTIVE and not self.latched:
      leaving = d_edge > 0.0 and radial_v > 1.0 and self.turning
      if leaving or not (V_MIN <= v <= V_MAX + 2.0) or d_edge > ENGAGE_EDGE_M + 10.0:
        self.phase = EXITING
    if self.phase == EXITING:
      phase_target = 0.0

    step = W_RATE * dt
    self.phase_w = min(max(phase_target, self.phase_w - step), self.phase_w + step)

    slew = RING_SLEW if self.latched else (TARGET_SLEW if poor else TARGET_SLEW_GOOD)
    if self.k_tgt is None or not lat_active:
      self.k_tgt = model_k
    else:
      self.k_tgt += min(max(k_raw - self.k_tgt, -slew * dt), slew * dt)

    c_bias = _interp(bias_m, (3.0, BIAS_MAX_M), (1.0, 0.4))
    conf = min(self.pose.confidence(t), map_match_confidence(ring, x, y), ring_confidence(ring), c_bias)
    w = self.phase_w if self.latched else W_MAX * conf * self.phase_w     # latched: the cap, not the confidence, bounds the assist
    if drv_active:
      w = 0.0
    self.debug = {"phase": self.phase, "w": w, "conf": conf, "bias": bias_m, "bacc": self.pose.bacc, "poor": poor,
                  "unreliable": unreliable, "hold": holding, "innov": self.pose.innov,
                  "n_good": self.pose.n_good, "r": r, "d_edge": d_edge, "k_pp": k_pp, "k_tgt": self.k_tgt, "r_ref": r_ref,
                  "align": align, "travel": self.travel, "latched": self.latched, "drv": int(drv_active)}
    if not lat_active or drv_active:
      # Driver has the wheel / the stalk (or lateral is off): no correction, nothing carried over.
      self.dk_out = 0.0
      if self.phase == EXITING and self.phase_w <= 0.0:
        self.phase, self.k_tgt = DONE, None
      return self._passthrough(model_k)
    dk = w * (self.k_tgt - model_k)
    dk = min(max(dk, -DK_MAX), DK_MAX)
    if self.latched and self.phase != EXITING:
      # Ring cap / floor on what is actually sent. Over the ring speed the curl hold wins (no floor, nothing above the model's).
      floor = -1e9 if holding else lim[1]
      base = -sense * model_k if holding else ring_cap(self.k_ref, model_k, v, sense)
      self.cap_extra, cap = RH.step_cap(self.cap_extra, base, lim[0], v, RING_A_LAT_MAX, RING_SLEW * dt, CAP_FALL, plain, holding)
      dk = -sense * min(max(-sense * (model_k + dk), floor), cap) - model_k
    # The correction moves at <= RING_SLEW once latched: an over-cap entry curl unwinds at the slew, no step.
    oslew = RING_SLEW if self.latched else (OUT_SLEW if poor else OUT_SLEW_GOOD)
    self.dk_out += min(max(dk - self.dk_out, -oslew * dt), oslew * dt)
    # ...and the sum never moves faster than max(slew, the model's own rate).
    if self.prev_out is not None and self.prev_model is not None:
      allowed = max(oslew * dt, abs(model_k - self.prev_model))
      out = min(max(model_k + self.dk_out, self.prev_out - allowed), self.prev_out + allowed)
      self.dk_out = out - model_k
    if self.latched and self.phase != EXITING and -sense * model_k > 0.0:
      # Hard clamp while the model curves with the circulation (the slews can lag a model that drops): never above the cap.
      out = -sense * min(-sense * (model_k + self.dk_out), cap)
      self.dk_out = out - model_k
    if self.latched and self.phase != EXITING and self.cap_extra > 0.0:
      a_cap = RING_A_LAT_MAX / max(v * v, 1.0)           # the model follows the exit branch (right) while the guide holds the circle: 2.2 m/s² still binds
      self.dk_out = -sense * min(-sense * (model_k + self.dk_out), max(a_cap, -sense * model_k)) - model_k
    self.prev_model = model_k
    self.prev_out = model_k + self.dk_out
    if self.phase == EXITING and self.phase_w <= 0.0 and abs(self.dk_out) < HANDOFF_DK:
      self.phase, self.k_tgt, self.dk_out = DONE, None, 0.0
      return self._passthrough(model_k)
    # Camera authority on lane lines / road edges applies to what is actually sent, immediately.
    return model_k + limit_correction_by_edges(self.dk_out, edges)


# --------------------------------------------------------------------------------------------
# controlsd glue (kept here so it is testable with a fake SubMaster / Params)

HINT_HOLD_S = 10.0
PARAM_READ_FRAMES = 100       # ~1 s at 100 Hz
LOOK_T_S = 1.0


def edges_from_model(model_v2, v_ego: float) -> EdgeInfo | None:
  """Nearest camera bound on each side of the model path at ~1 s (device frame, y right)."""
  try:
    x = min(max(float(v_ego) * LOOK_T_S, 5.0), 20.0)
    path_y = _interp_xy(model_v2.position.x, model_v2.position.y, x)
    if path_y is None:
      return None
    lefts, rights = [], []
    lines, probs = model_v2.laneLines, model_v2.laneLineProbs
    for i in (1, 2):
      if len(lines) > i and len(probs) > i and float(probs[i]) > 0.5:
        yy = _interp_xy(lines[i].x, lines[i].y, x)
        if yy is not None:
          (lefts if yy < path_y else rights).append(yy)
    edges, stds = model_v2.roadEdges, model_v2.roadEdgeStds
    for i in (0, 1):
      if len(edges) > i and len(stds) > i and float(stds[i]) < 1.0:
        yy = _interp_xy(edges[i].x, edges[i].y, x)
        if yy is not None:
          (lefts if yy < path_y else rights).append(yy)
    return EdgeInfo(x=x, path_y=path_y, left_y=max(lefts) if lefts else None,
                    right_y=min(rights) if rights else None)
  except Exception:
    return None


class RoundaboutAssist:
  """Pre-AP controlsd wrapper: toggle, GNSS feed, ring param, hint gating, edges, driver inputs, ring logging."""

  def __init__(self, preap: bool, params=None) -> None:
    self.preap = bool(preap)
    self.params = params
    self.guide = RoundaboutGuide()
    self.enabled = False
    self._frame = -1
    self._last_hint_t = -1e9
    self._last_ring_read_t = -1e9
    self._gps_frames: dict[str, int] = {}
    self._last_log_t = -1e9
    self._last_raw_t = -1e9
    self._last_t: float | None = None
    self._stalk = StalkTipTurn()
    self._latched_param = False
    self.active = False

  @property
  def latched(self) -> bool:
    return bool(self.guide.latched and self.guide.phase != DONE)

  def _read_toggle(self) -> None:
    if self._frame < 0 or self._frame >= PARAM_READ_FRAMES:
      self._frame = 0
      on = False
      if self.params is not None:
        try:
          on = bool(self.params.get_bool(PARAM_ROUNDABOUT_ASSIST))
        except Exception:
          on = False
      self.enabled = on
    self._frame += 1

  def _load_ring(self, t: float, way_id: int) -> None:
    if self.params is None or self.guide.has_way(way_id) or t - self._last_ring_read_t < 1.0:
      return
    self._last_ring_read_t = t
    try:
      from openpilot.selfdrive.mapd.roundabout_map import PARAM_RING
      ring = RingGeometry.from_json(self.params.get(PARAM_RING))
    except Exception:
      ring = None
    if ring is not None and int(way_id) in ring.way_ids:
      self.guide.set_ring(ring)

  def _feed_gnss(self, sm) -> None:
    for sock, lat_s in (("gpsLocationExternal", GPS_EXT_LATENCY_S), ("gpsLocation", GPS_LATENCY_S)):
      try:
        frame = int(sm.recv_frame[sock])
      except Exception:
        continue
      if frame <= 0 or frame == self._gps_frames.get(sock):
        continue
      self._gps_frames[sock] = frame
      g = sm[sock]
      lat, lon = float(g.latitude), float(g.longitude)
      if not (math.isfinite(lat) and math.isfinite(lon)) or (abs(lat) < 1e-6 and abs(lon) < 1e-6):
        continue
      if hasattr(g, "hasFix") and not bool(g.hasFix):
        continue
      self.guide.gnss(float(sm.logMonoTime[sock]) * 1e-9, lat, lon, float(g.bearingDeg), float(g.speed),
                      float(getattr(g, "bearingAccuracyDeg", 0.0) or 0.0), lat_s)
      return   # one source per cycle; external preferred when present

  def _sync_latched_param(self, on_ring: bool = False) -> None:
    """modeld reads this (a different process) to suppress ALC tips while the car is on the ring.

    Oct 13: also set whenever the map says we are on a roundabout, assist on
    or off, so the exit blinker in a ring never arms an automatic lane change.
    """
    latched = bool(self.latched or on_ring)
    if latched == self._latched_param:
      return
    self._latched_param = latched
    try:
      self.params.put_bool(PARAM_RING_LATCHED, latched)
    except Exception:
      pass

  def update(self, sm, *, t: float, v_ego: float, yaw_rate: float, model_k: float, lat_active: bool,
             maneuver_active: bool, lane_change_active: bool, hint=None, model_v2=None,
             stalk_state: int = 0, steering_pressed: bool = False, steering_torque: float = 0.0) -> float:
    """Return the curvature to use in place of model_k (identical when Off / non-Pre-AP / idle)."""
    self._read_toggle()
    self.active = False
    if not self.preap:
      return float(model_k)
    if not self.enabled:
      self.guide.update(t, v_ego, yaw_rate, model_k, enabled=False, lat_active=False)
      if self.guide.latched:
        self.guide.reset_state()      # toggled Off on the ring: nothing carried over
      self._sync_latched_param(on_ring=_hint_on_ring(hint))
      return float(model_k)
    dt = 0.0 if self._last_t is None else min(max(t - self._last_t, 0.0), 0.1)
    self._last_t = t
    self._stalk.update(stalk_state, dt)
    driver = DriverInput(stalk_dir=int(self._stalk.direction), stalk_held=bool(self._stalk.is_turn),
                         pressed=bool(steering_pressed), torque=float(steering_torque))
    self._feed_gnss(sm)
    if hint is not None and getattr(hint, "way_id", 0):
      self._last_hint_t = t
      self._load_ring(t, int(hint.way_id))
    busy = self.guide.phase in (ACTIVE, EXITING)
    on = (t - self._last_hint_t) <= HINT_HOLD_S or busy
    if maneuver_active or lane_change_active:
      on = False
    edges = edges_from_model(model_v2, v_ego) if (on and model_v2 is not None) else None
    out = self.guide.update(t, v_ego, yaw_rate, model_k, enabled=on, lat_active=lat_active, edges=edges, driver=driver)
    self.active = self.guide.phase in (ACTIVE, EXITING)
    self._sync_latched_param(on_ring=_hint_on_ring(hint))
    self._log(t, model_k, out, v_ego, model_v2, driver, on)
    return out

  def _log(self, t: float, model_k: float, out: float, v_ego: float = 0.0, model_v2=None,
           driver: DriverInput | None = None, on: bool = False) -> None:
    """Ring state through swaglog (no capnp). ERROR level on purpose: logMessage is rlog-only, errorLogMessage is in qlog.

    Events as they happen, a state line at ~1 Hz while the assist runs, and (near / in a ring) a compact summary of the
    model's raw lane lines / road edges / path at ~1.25 Hz, so a qlog says whether the model's output still holds the ring.
    """
    g = self.guide
    ev, g.events = g.events, []
    for e in ev:
      cloudlog.error(f"roundabout_assist {e}")
    d = g.debug
    if self.active and t - self._last_log_t >= LOG_PERIOD_S:
      self._last_log_t = t
      r_ref = d.get("r_ref")
      fields = [f"phase={g.phase}", f"latched={int(bool(d.get('latched')))}", f"w={d.get('w', 0.0):.2f}", f"conf={d.get('conf', 0.0):.2f}",
                f"bias={d.get('bias', 0.0):.1f}", f"bacc={d.get('bacc', 0.0):.0f}", f"poor={int(bool(d.get('poor')))}",
                f"hold={int(bool(d.get('hold')))}", f"r={d.get('r', 0.0):.1f}", f"r_ref={'-' if r_ref is None else f'{r_ref:.1f}'}",
                f"model={model_k:+.4f}", f"out={out:+.4f}"]
      cloudlog.error("roundabout_assist state " + " ".join(fields))
    near = on and g.d_edge is not None and g.d_edge < RAW_LOG_EDGE_M and g.phase != DONE
    if near and model_v2 is not None and t - self._last_raw_t >= LOG_RAW_PERIOD_S:
      self._last_raw_t = t
      cloudlog.error("roundabout_raw " + raw_ring_summary(g, t, v_ego, model_k, out, model_v2, driver))
