"""Low-visibility lateral back-off.

When the driving model is unsure where to go, or the road camera exposure
collapses (sun low and ahead makes a milder drop count), ease lateral
toward the driver's current steering and raise one alert: "Low visibility".

Openpilot stays engaged. This module does not touch longitudinal control
and does not clear latActive — it only scales how far the lateral command
may sit from the wheel. Settings → NAP → Driving Mannerisms can turn it
Off (NAPLowVisBackoff, default On). Off is identity: authority stays 1
and the alert stays down.

Enter (model): the model is unsure where to go, held 0.40 s. All of
these, not lane lines: path lateral std at ~3 s above 2 m, desired
curvature jumping inside a short window, and no usable road edge on
either side. Missing paint on a gravel road or a city street does not
qualify when the path is confident and stable. The timer runs only
above 20 mph, with no blinker, while lateral is not yielded, only
after the car has been out of a turn for the last 1 s (measured
curvature under 0.01 1/m, radius over ~100 m), and not in or within
40 m of a roundabout (the same ``roundabout_yield_context`` signal
the lateral yield uses). A turn or a roundabout holds the timer at 0.
Desired-curvature jumps are uncertainty, not a turn.

Exit: the path is confident again (lateral std at ~3 s under 1.2 m and
desired curvature no longer jumping), held 0.80 s. Outside a roundabout
the car also has to be out of the turn, so an alert that is already up
when a turn starts stays up through the turn. Inside a roundabout that
turn hold does not apply: a confident path releases on the circle.
Lane lines are not required. A single frame cannot flip the latch.

Enter (camera): integration lines fall below 0.45× a recent bright
baseline (0.65× when the sun is within 12° of the horizon and 22° of
the heading), held 0.30 s, and ONLY when the sun is low and ahead or
the model path is also unsure (lateral std at ~3 s over 1.2 m). A plain
shade-to-sun exposure change on a clear day never counts. Exit when
lines are back above 0.70× for 1 s, or when no ratio can be computed
(baseline under 280 lines, i.e. a normal bright daylight exposure) for
1 s. The sun-glare check does not use the turn gate.

Backstop: a latch held up only by the camera releases once the model
path has been confident and stable for 3 s, and the camera cannot
re-latch until the exposure has recovered once.

Authority: never zero. A camera-only latch with a confident path keeps
following the path with a gentle cap (0.85). Model uncertainty lowers
authority in proportion to the path std (1.0 at 1.2 m down to a 0.30
floor at 4 m). Lateral never holds a heading or a fixed direction.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.model_sight import path_y_std_at as _shared_path_y_std_at

# --- model confidence -------------------------------------------------------
# laneLineProbs are already 0..1. Road-edge std (m) maps through
# exp(-std / EDGE_STD_SCALE): ~0.1 m → ~0.83, ~0.7 m → ~0.28, ~1.5 m → ~0.07.
LANE_CONF_ENTER = 0.30
LANE_CONF_EXIT = 0.55
EDGE_STD_SCALE_M = 0.55
PATH_T_S = 3.0
PATH_Y_STD_ENTER_M = 2.0
PATH_Y_STD_EXIT_M = 1.2
MODEL_ENTER_S = 0.40
MODEL_EXIT_S = 0.80

# Model-poor timer. Turns, blinkers, a yielded wheel, and town creep
# are not low visibility: lane lines vanish in intersections.
ENTRY_MIN_V_MS = 20.0 * CV.MPH_TO_MS  # strictly above 20 mph
TURN_CURVATURE = 0.01                  # 1/m, measured radius ~100 m
STRAIGHT_HOLD_S = 1.0                  # out of the turn this long before the timer runs
# Desired-curvature chatter. Peak-to-peak inside this window, not a
# smooth bend. One spike ages out before the 0.40 s enter dwell.
CURV_JUMP = 0.008                      # 1/m
CURV_WINDOW_S = 0.15

# --- camera exposure --------------------------------------------------------
EXPOSURE_WINDOW_S = 12.0
EXPOSURE_EXCLUDE_S = 1.0       # baseline ignores the newest second
EXPOSURE_FAST_S = 0.40
EXPOSURE_MIN_BASELINE = 280    # a camera that was never bright cannot "collapse"
EXPOSURE_COLLAPSE_RATIO = 0.45
EXPOSURE_SUN_RATIO = 0.65      # earlier heads-up when the sun is low and ahead
EXPOSURE_RECOVER_RATIO = 0.70
EXPOSURE_ENTER_S = 0.30
EXPOSURE_EXIT_S = 1.00

# --- sun (optional). Incident was ~0.3–1° up and ~7–8° right of ahead. -----
SUN_ELEV_MIN_DEG = -1.0
SUN_ELEV_MAX_DEG = 12.0
SUN_AHEAD_DEG = 22.0

# --- authority slew ---------------------------------------------------------
AUTHORITY_DOWN_PER_S = 1.0     # full fade in about 1 s
AUTHORITY_UP_PER_S = 0.80      # recovery a little slower than the drop
AUTHORITY_FLOOR = 0.30         # never zero: lateral keeps following the path
CAMERA_ONLY_AUTHORITY = 0.85   # gentle cap while only the camera is latched
AUTHORITY_FLOOR_STD_M = 4.0    # path std where authority reaches the floor

# Glare corroboration: the model path must be at least this unsure when the
# sun is not low and ahead. Same number as the confident-path exit.
CAMERA_MODEL_UNSURE_M = PATH_Y_STD_EXIT_M
# Camera-only backstop: confident, stable path this long releases the latch.
CAMERA_BACKSTOP_S = 3.0
# Any lateral degrade latched this long raises the louder prolonged alert.
PROLONGED_S = 45.0

PARAM_LOW_VIS_BACKOFF = "NAPLowVisBackoff"


@dataclass(frozen=True)
class LowVisibilityOutput:
  authority: float
  active: bool
  alert: bool
  model_poor: bool
  camera_blind: bool


def edge_confidence(std_m: float) -> float:
  """Road-edge lateral std (m) → 0..1 confidence. Unknown std is not low."""
  try:
    std = float(std_m)
  except (TypeError, ValueError):
    return 1.0
  if not math.isfinite(std) or std < 0.0:
    return 1.0
  return math.exp(-std / EDGE_STD_SCALE_M)


def _finite(value, default: float = 1.0) -> float:
  try:
    v = float(value)
  except (TypeError, ValueError):
    return default
  return v if math.isfinite(v) else default


def side_confidences(lane_probs, edge_stds) -> tuple[float, float]:
  """(left, right) confidence. Missing signals count as seen (1), not blind.

  Lane lines: index 1 left, 2 right. Road edges: index 0 left, 1 right.
  A side is the worse of its lane probability and its road-edge confidence.
  """
  probs = list(lane_probs or [])
  edges = list(edge_stds or [])

  def lane(i: int) -> float:
    if i >= len(probs):
      return 1.0
    return _finite(probs[i], 1.0)

  def edge(i: int) -> float:
    if i >= len(edges):
      return 1.0
    return edge_confidence(edges[i])

  return min(lane(1), edge(0)), min(lane(2), edge(1))


def side_confidences_exit(lane_probs, edge_stds) -> tuple[float, float]:
  """(left, right) for the exit check only. Missing signals count as seen.

  Same indexes as side_confidences. A side is the better of its lane
  probability and its road-edge confidence, so a healthy lane line can
  leave low-visibility even when the road-edge std is still wide.
  """
  probs = list(lane_probs or [])
  edges = list(edge_stds or [])

  def lane(i: int) -> float:
    if i >= len(probs):
      return 1.0
    return _finite(probs[i], 1.0)

  def edge(i: int) -> float:
    if i >= len(edges):
      return 1.0
    return edge_confidence(edges[i])

  return max(lane(1), edge(0)), max(lane(2), edge(1))


def path_y_std_at(times, y_stds, t_query: float = PATH_T_S) -> float | None:
  """Interpolate lateral path std (m) at ``t_query`` seconds. None if unusable.

  Shared with the sight-distance cap: see model_sight.path_y_std_at.
  """
  return _shared_path_y_std_at(times, y_stds, t_query)


def edges_both_unusable(edge_stds) -> bool:
  """True only when both road edges are reported and both are wide.

  A missing edge is not blind: unknown std stays "seen", so a parse
  glitch cannot fade steering. One usable curb or shoulder is enough
  to keep the trigger down. Lane-line probabilities are not read.
  """
  edges = list(edge_stds or [])
  if len(edges) < 2:
    return False
  return (edge_confidence(edges[0]) < LANE_CONF_ENTER
          and edge_confidence(edges[1]) < LANE_CONF_ENTER)


def model_is_poor(y_std, edges_unusable, curvature_unstable) -> bool:
  """Unsure where to go: wide plan, jumping curvature, and no edges.

  Lane-line presence is not an input. A tight path on an unmarked road
  is not poor.
  """
  if y_std is None or not math.isfinite(y_std) or y_std <= PATH_Y_STD_ENTER_M:
    return False
  return bool(edges_unusable) and bool(curvature_unstable)


def model_is_clear(y_std, curvature_stable) -> bool:
  """Path is confident again: settled lateral std and stable curvature.

  Lane lines are not required. ``curvature_stable`` is the caller saying
  desired curvature has not jumped inside ``CURV_WINDOW_S``.
  """
  if y_std is None or not math.isfinite(y_std) or y_std >= PATH_Y_STD_EXIT_M:
    return False
  return bool(curvature_stable)


def _abs_curvature(value) -> float | None:
  try:
    k = float(value)
  except (TypeError, ValueError):
    return None
  return abs(k) if math.isfinite(k) else None


def in_turn(measured_k) -> bool:
  """True when the car itself is turning (measured radius under ~100 m).

  Measured curvature is ``self.curvature`` in controlsd. Desired
  curvature is not a turn: a jump there is the uncertainty signal.
  Unusable measured curvature does not start the timer.
  """
  measured = _abs_curvature(measured_k)
  if measured is None:
    return True
  return measured >= TURN_CURVATURE


def entry_speed_ok(v_ego) -> bool:
  """Model-poor entry counts only above 20 mph."""
  try:
    v = float(v_ego)
  except (TypeError, ValueError):
    return False
  return math.isfinite(v) and v > ENTRY_MIN_V_MS


def fade_curvature(model_k: float, measured_k: float, authority: float) -> float:
  """Blend the lateral target toward the wheel. Authority 1 is the model."""
  try:
    a = float(authority)
    mk = float(model_k)
    meas = float(measured_k)
  except (TypeError, ValueError):
    return float(measured_k) if measured_k is not None else 0.0
  if not math.isfinite(a):
    a = 1.0
  if not math.isfinite(mk):
    mk = meas if math.isfinite(meas) else 0.0
  if not math.isfinite(meas):
    meas = 0.0
  a = min(1.0, max(0.0, a))
  if a >= 1.0:
    return mk
  if a <= 0.0:
    return meas
  return a * mk + (1.0 - a) * meas


def angle_delta_deg(a_deg: float, b_deg: float) -> float:
  return abs((float(a_deg) - float(b_deg) + 180.0) % 360.0 - 180.0)


def sun_low_and_ahead(*, elevation_deg: float, heading_deg: float, azimuth_deg: float) -> bool:
  """Sun near the horizon and roughly in front of the camera heading."""
  try:
    elev = float(elevation_deg)
    heading = float(heading_deg)
    azimuth = float(azimuth_deg)
  except (TypeError, ValueError):
    return False
  if not all(math.isfinite(v) for v in (elev, heading, azimuth)):
    return False
  if not SUN_ELEV_MIN_DEG <= elev <= SUN_ELEV_MAX_DEG:
    return False
  return angle_delta_deg(heading, azimuth) <= SUN_AHEAD_DEG


def solar_position(lat_deg: float, lon_deg: float, unix_s: float) -> tuple[float, float]:
  """Approximate (elevation_deg, azimuth_deg). Azimuth is clockwise from north.

  About a degree, which is enough for "low and ahead" versus overhead or behind.
  """
  lat = float(lat_deg)
  lon = float(lon_deg)
  # Days since 2000-01-01 12:00 UTC (J2000-ish).
  days = (float(unix_s) - 946728000.0) / 86400.0
  mean_long = (280.460 + 0.9856474 * days) % 360.0
  mean_anom = math.radians((357.528 + 0.9856003 * days) % 360.0)
  lam = math.radians((mean_long + 1.915 * math.sin(mean_anom) + 0.020 * math.sin(2.0 * mean_anom)) % 360.0)
  eps = math.radians(23.439 - 0.0000004 * days)
  dec = math.asin(max(-1.0, min(1.0, math.sin(eps) * math.sin(lam))))
  ra = math.atan2(math.cos(eps) * math.sin(lam), math.cos(lam))
  gmst = (280.46061837 + 360.98564736629 * days) % 360.0
  lmst = math.radians((gmst + lon) % 360.0)
  ha = lmst - ra
  lat_r = math.radians(lat)
  sin_el = (math.sin(lat_r) * math.sin(dec)
            + math.cos(lat_r) * math.cos(dec) * math.cos(ha))
  elev = math.degrees(math.asin(max(-1.0, min(1.0, sin_el))))
  # Azimuth clockwise from north. -sin(ha) puts morning sun in the east.
  az = math.degrees(math.atan2(
    -math.sin(ha),
    math.tan(dec) * math.cos(lat_r) - math.sin(lat_r) * math.cos(ha),
  ))
  az = (az + 360.0) % 360.0
  return elev, az


def sun_ahead_from_fix(*, latitude: float, longitude: float, unix_timestamp_millis: int,
                       bearing_deg: float, horizontal_accuracy_m: float = 0.0) -> bool | None:
  """True/False when the fix is usable. None when sun position should be ignored."""
  try:
    lat = float(latitude)
    lon = float(longitude)
    ts_ms = int(unix_timestamp_millis)
    bearing = float(bearing_deg)
    acc = float(horizontal_accuracy_m or 0.0)
  except (TypeError, ValueError):
    return None
  if not all(math.isfinite(v) for v in (lat, lon, bearing, acc)):
    return None
  if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
    return None
  if ts_ms < 1_000_000_000_000:  # need real unix milliseconds
    return None
  if acc > 50.0:
    return None
  elev, az = solar_position(lat, lon, ts_ms / 1000.0)
  return sun_low_and_ahead(elevation_deg=elev, heading_deg=bearing, azimuth_deg=az)


class _Exposure:
  def __init__(self):
    self._samples: deque[tuple[float, float]] = deque()
    self._t = 0.0
    self.blind = False
    self._enter_s = 0.0
    self._exit_s = 0.0
    self.suppressed = False
    self.recovered = True

  def backstop_release(self):
    """Drop the latch and refuse to re-latch until exposure recovers once."""
    self.clear_latch()
    self.suppressed = True
    self.recovered = False

  def update(self, integ_lines, sun_ahead: bool, dt: float, corroborated: bool | None = None) -> bool:
    """``corroborated``: the sun is low and ahead or the model path is unsure.

    None keeps the old meaning (sun_ahead only). A drop that is not
    corroborated never enters. No computable ratio counts as clear.
    """
    self._t += dt
    try:
      integ = float(integ_lines) if integ_lines is not None else None
    except (TypeError, ValueError):
      integ = None
    if integ is not None and math.isfinite(integ) and integ > 0.0:
      self._samples.append((self._t, integ))
    horizon = self._t - EXPOSURE_WINDOW_S
    while self._samples and self._samples[0][0] < horizon:
      self._samples.popleft()

    baseline = self._baseline()
    current = self._current()
    ratio = None
    if baseline is not None and current is not None and baseline >= EXPOSURE_MIN_BASELINE:
      ratio = current / baseline
    enter_ratio = EXPOSURE_SUN_RATIO if sun_ahead else EXPOSURE_COLLAPSE_RATIO
    if corroborated is None:
      corroborated = bool(sun_ahead)
    raw = ratio is not None and ratio < enter_ratio and bool(corroborated)
    # Baseline under 280 (or no samples): a normal bright exposure, not blind.
    clear = ratio is None or ratio > EXPOSURE_RECOVER_RATIO
    if clear:
      self.recovered = True
    if self.suppressed and not self.recovered:
      raw = False
    if self.suppressed and self.recovered:
      self.suppressed = False
    if raw:
      self._enter_s += dt
      self._exit_s = 0.0
      if self._enter_s + 1e-12 >= EXPOSURE_ENTER_S:
        self.blind = True
    elif clear:
      self._exit_s += dt
      self._enter_s = 0.0
      if self._exit_s + 1e-12 >= EXPOSURE_EXIT_S:
        self.blind = False
    else:
      self._enter_s = 0.0
      self._exit_s = 0.0
    return self.blind

  def _baseline(self) -> float | None:
    old = [v for t, v in self._samples if self._t - t >= EXPOSURE_EXCLUDE_S]
    if len(old) < 3:
      return None
    return max(old)

  def _current(self) -> float | None:
    recent = [v for t, v in self._samples if self._t - t <= EXPOSURE_FAST_S]
    if not recent:
      return None
    return sum(recent) / len(recent)

  def clear_latch(self):
    self.blind = False
    self._enter_s = 0.0
    self._exit_s = 0.0
    self.suppressed = False
    self.recovered = True


def authority_target(active: bool, model_poor: bool, camera: bool, y_std) -> float:
  """Never zero. Degrade only in proportion to genuine path uncertainty."""
  if not active:
    return 1.0
  target = 1.0
  if camera:
    target = CAMERA_ONLY_AUTHORITY
  if y_std is not None and math.isfinite(y_std) and y_std > PATH_Y_STD_EXIT_M:
    span = AUTHORITY_FLOOR_STD_M - PATH_Y_STD_EXIT_M
    frac = min(1.0, (y_std - PATH_Y_STD_EXIT_M) / span)
    target = min(target, AUTHORITY_FLOOR + (1.0 - frac) * (1.0 - AUTHORITY_FLOOR))
  elif model_poor:
    target = min(target, CAMERA_ONLY_AUTHORITY)
  return max(AUTHORITY_FLOOR, target)


class LowVisibility:
  """Filter model + exposure into one latch and a slewed lateral authority."""

  def __init__(self):
    self.authority = 1.0
    self.active = False
    self._model = False
    self._poor_s = 0.0
    self._clear_s = 0.0
    # No turn observed yet, so a straight at speed can enter immediately.
    # A turn zeros this; the timer stays at 0 until it climbs back to 1 s.
    self._straight_s = STRAIGHT_HOLD_S
    self._k_t = 0.0
    self._k_hist: deque[tuple[float, float]] = deque()
    self._exposure = _Exposure()
    self._cam_confident_s = 0.0
    self._active_s = 0.0

  def reset_latch(self):
    self.active = False
    self._model = False
    self._poor_s = 0.0
    self._clear_s = 0.0
    self._straight_s = STRAIGHT_HOLD_S
    self._k_t = 0.0
    self._k_hist.clear()
    self.authority = 1.0
    self._exposure.clear_latch()
    self._cam_confident_s = 0.0
    self._active_s = 0.0

  @property
  def prolonged(self) -> bool:
    """Lateral degrade latched longer than PROLONGED_S."""
    return self.active and self._active_s + 1e-9 >= PROLONGED_S

  def _curvature_unstable(self, model_k, dt: float) -> bool:
    """Desired curvature peak-to-peak inside the recent window is a jump."""
    self._k_t += dt
    try:
      k = float(model_k)
    except (TypeError, ValueError):
      return False
    if not math.isfinite(k):
      return False
    self._k_hist.append((self._k_t, k))
    cutoff = self._k_t - CURV_WINDOW_S
    while self._k_hist and self._k_hist[0][0] < cutoff:
      self._k_hist.popleft()
    if len(self._k_hist) < 2:
      return False
    ks = [sample for _, sample in self._k_hist]
    return (max(ks) - min(ks)) >= CURV_JUMP

  def update(self, *, enabled: bool, lane_probs=None, edge_stds=None,
             path_t=None, path_y_std=None, integ_lines=None,
             sun_ahead: bool = False, dt: float = 0.01,
             v_ego: float = 15.0, blinker: bool = False, yielded: bool = False,
             measured_curvature: float = 0.0, model_curvature: float = 0.0,
             near_roundabout: bool = False) -> LowVisibilityOutput:
    """``v_ego`` defaults above 20 mph and both curvatures default straight.

    Confidence tests that omit the car state keep the previous straight-road
    behavior. controlsd always passes the live speed, blinker, yield, and
    the curvatures it already uses for the lateral command.
    """
    dt = float(dt) if dt and math.isfinite(float(dt)) else 0.01
    # Lane probabilities are intentionally unused. Missing paint is normal.
    del lane_probs
    turning = in_turn(measured_curvature)
    if turning:
      self._straight_s = 0.0
    else:
      self._straight_s = min(STRAIGHT_HOLD_S, self._straight_s + dt)
    straight_for_hold = self._straight_s + 1e-12 >= STRAIGHT_HOLD_S
    entry_ok = (
      straight_for_hold
      and entry_speed_ok(v_ego)
      and not bool(blinker)
      and not bool(yielded)
      and not bool(near_roundabout)
    )
    unstable = self._curvature_unstable(model_curvature, dt)
    y_std = path_y_std_at(path_t, path_y_std, PATH_T_S)
    poor = model_is_poor(y_std, edges_both_unusable(edge_stds), unstable)
    # Out of the turn, a settled path with calm desired curvature releases
    # even when lane lines never come back. In a turn the exit hold does
    # not run, so an alert that is already up stays up. A roundabout is
    # the exception: release as soon as the path is confident, still circling.
    hold_through_turn = turning and not bool(near_roundabout)
    clear = (not hold_through_turn) and model_is_clear(y_std, not unstable)
    if self._model:
      if clear:
        self._clear_s += dt
        self._poor_s = 0.0
        if self._clear_s + 1e-12 >= MODEL_EXIT_S:
          self._model = False
      else:
        self._poor_s = 0.0
        self._clear_s = 0.0
    elif poor and entry_ok:
      self._poor_s += dt
      self._clear_s = 0.0
      if self._poor_s + 1e-12 >= MODEL_ENTER_S:
        self._model = True
    else:
      # Turn, roundabout, blinker, yield, or under 20 mph: timer stays at 0.
      self._poor_s = 0.0
      self._clear_s = 0.0

    model_unsure = y_std is not None and math.isfinite(y_std) and y_std > CAMERA_MODEL_UNSURE_M
    camera = self._exposure.update(integ_lines, bool(sun_ahead), dt,
                                   corroborated=bool(sun_ahead) or model_unsure)
    # Backstop: camera-only latch with a confident, stable path.
    confident = model_is_clear(y_std, not unstable)
    if camera and not self._model and confident:
      self._cam_confident_s += dt
      if self._cam_confident_s + 1e-12 >= CAMERA_BACKSTOP_S:
        self._exposure.backstop_release()
        camera = False
        self._cam_confident_s = 0.0
    else:
      self._cam_confident_s = 0.0
    if not enabled:
      self._model = False
      self._poor_s = 0.0
      self._clear_s = 0.0
      self._exposure.clear_latch()
      self._cam_confident_s = 0.0
      self._active_s = 0.0
      self.active = False
      self.authority = 1.0
      return LowVisibilityOutput(1.0, False, False, False, False)

    self.active = bool(self._model or camera)
    self._active_s = self._active_s + dt if self.active else 0.0
    target = authority_target(self.active, self._model, camera, y_std)
    if self.authority > target:
      self.authority = max(target, self.authority - AUTHORITY_DOWN_PER_S * dt)
    elif self.authority < target:
      self.authority = min(target, self.authority + AUTHORITY_UP_PER_S * dt)
    return LowVisibilityOutput(
      float(self.authority), bool(self.active), bool(self.active),
      bool(self._model), bool(camera))
