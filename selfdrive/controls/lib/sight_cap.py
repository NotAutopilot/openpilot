"""Sight-distance speed cap (Pre-AP). LOG ONLY: nothing reads its output.

Idea (Justin, Oct 11 2026): when the model suddenly cannot push its path out
confidently, treat that like low visibility and slow so the car could still
handle a tight bend inside the distance it can see.

Sight distance is model_sight.sight_distance_m: where the path's lateral std
reaches 2 m (shared with low-visibility steering, which reads the same std).

Would fire when all of these hold:
- sight drops below SHRINK_RATIO (0.5) x its own median over the 2-12 s
  before, held SHRINK_HOLD_S (0.5 s);
- 25-55 mph;
- confidence was not already red with a short path over that baseline window
  (red for half of it AND baseline sight under 7 s of travel). Red alone is the
  model's normal state for long stretches; with a long confident path it does
  not count as "already unsure";
- another cue: path curvature rising, or a map roundabout ahead whose funnel
  is not live yet;
- no tracked vehicle explains it (Justin, Oct 11): vehicles never count as
  sight loss. A lead in our lane, moving or stopped, belongs to the lead /
  stopped-object logic; parked cars and cars in the next lane do not change
  our road. Any radar/model vehicle within VEHICLE_RANGE_M suppresses it,
  with the reason logged: lead / stopped_lead / adjacent / parked. It fires
  only when the ROAD is hidden or uncertain (a bend, a crest, a junction);
- the roundabout map funnel is not live (it owns entry).

The cap speed is the speed from which the car can slow, inside the sight
distance less MARGIN_M, to a TURN_R_M (30 m) bend at TURN_LAT_MS2 (15 mph)
at DECEL_MS2 (1.5 m/s²). Its would-be accel command is jerk limited at
JERK_MS3 (2 m/s³). Low-visibility glare (camera-only) never feeds this: it
reads only the model path.

Log: "sightcap {json}" through cloudlog.error (errorLogMessage, kept in qlog),
on every would-fire edge, at 2 Hz while it would be firing, and once when a
held sight drop is suppressed by a vehicle (why = lead / stopped_lead /
adjacent / parked).
"""
from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import dataclass

from openpilot.common.constants import CV

PARAM_KEY = "NAPSightCap"
MODE_OFF = 0
MODE_SHADOW = 1
DEFAULT_MODE = MODE_SHADOW

SHRINK_RATIO = 0.5
RECOVER_RATIO = 0.8
BASE_WIN_S = (2.0, 12.0)    # baseline = median sight over [t-12, t-2]
BASE_MIN_SAMPLES = 10
SHRINK_HOLD_S = 0.5
V_MIN_MS = 25.0 * CV.MPH_TO_MS
V_MAX_MS = 55.0 * CV.MPH_TO_MS
RED_BASE_FRAC = 0.5          # "already red": red for at least half the baseline window
# ...and the path was already short then. Red is the model's normal state for
# long stretches (a third of Oct 4-10); a red model whose path still reached
# BASE_LONG_S of travel was not unsure. The Oct 10 5:40:45 PM corner is red
# throughout with a ~200 m (9 s) baseline.
BASE_LONG_S = 7.0

# Cue: rising curvature (max |k| over the path) within the last CURV_RISE_S.
CURV_RISE_S = 1.0
CURV_RISE_K = 0.002
CURV_MIN_K = 0.003

# Lead classification (path-relative lateral, m).
OUR_LANE_M = 1.8
ADJ_LANE_M = 5.4
MOVING_LEAD_MS = 3.0
VEHICLE_RANGE_M = 150.0
VEHICLE_HOLD_S = 1.5        # a radar track that flickers out still explains the drop for this long

# Cap.
TURN_R_M = 30.0
TURN_LAT_MS2 = 1.5
V_TURN_MS = math.sqrt(TURN_R_M * TURN_LAT_MS2)   # 6.7 m/s, 15 mph
DECEL_MS2 = 1.5
JERK_MS3 = 2.0
MARGIN_M = 10.0
CAP_GAIN_1_S = 0.6
FREE_A_MS2 = 2.0

# Crest: path height second difference over 50/100 m (m).
CREST_Z_M = -1.0

LOG_PERIOD_S = 0.5
LOG_PREFIX = "sightcap "


def read_sight_cap_mode(params) -> int:
  try:
    mode = int(params.get(PARAM_KEY, return_default=True))
  except Exception:
    return DEFAULT_MODE
  return mode if mode in (MODE_OFF, MODE_SHADOW) else DEFAULT_MODE


def cap_speed_ms(sight_m: float) -> float:
  """Fastest speed that can still slow to a 30 m bend at 1.5 m/s² inside the sight distance."""
  room = max(0.0, float(sight_m) - MARGIN_M)
  return math.sqrt(V_TURN_MS * V_TURN_MS + 2.0 * DECEL_MS2 * room)


def crest_ahead(xs, zs) -> tuple[bool, float | None]:
  """(crest, z second difference) from the model path height at 50 and 100 m."""
  try:
    pts = [(float(x), float(z)) for x, z in zip(xs, zs, strict=False)]
    if len(pts) < 2 or pts[-1][0] < 100.0:
      return False, None

    def z_at(d):
      for (x0, z0), (x1, z1) in zip(pts, pts[1:], strict=False):
        if x0 <= d <= x1 and x1 > x0:
          return z0 + (d - x0) / (x1 - x0) * (z1 - z0)
      return pts[-1][1]
    dd = z_at(100.0) - 2.0 * z_at(50.0) + pts[0][1]
    return dd < CREST_Z_M, round(dd, 2)
  except Exception:
    return False, None


@dataclass
class LeadInfo:
  present: bool = False
  lane: str = "none"        # "ours" | "adjacent" | "other" | "none"
  v: float | None = None
  d: float | None = None
  lat: float | None = None

  @property
  def moving_in_our_lane(self) -> bool:
    return self.present and self.lane == "ours" and self.v is not None and self.v > MOVING_LEAD_MS

  @property
  def kind(self) -> str:
    """lead / stopped_lead / adjacent / parked, or "" when nothing within range."""
    if not self.present or (self.d is not None and self.d > VEHICLE_RANGE_M):
      return ""
    moving = self.v is None or self.v > MOVING_LEAD_MS
    if self.lane == "ours":
      return "lead" if moving else "stopped_lead"
    return "adjacent" if moving else "parked"


def classify_lead(present: bool, path_lat, v_lead, d_rel) -> LeadInfo:
  if not present:
    return LeadInfo()
  lane = "other"
  if path_lat is None:
    lane = "ours"            # radard picked it as leadOne and no path to say otherwise
  elif abs(float(path_lat)) <= OUR_LANE_M:
    lane = "ours"
  elif abs(float(path_lat)) <= ADJ_LANE_M:
    lane = "adjacent"
  return LeadInfo(True, lane, None if v_lead is None else float(v_lead),
                  None if d_rel is None else float(d_rel), None if path_lat is None else float(path_lat))


class SightCap:
  """Trigger state and the would-be cap. update() never raises out to the planner."""

  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self._hist: deque = deque()     # (t, sight, red)
    self._kmax: deque = deque()     # (t, max |k|)
    self._shrink_t = 0.0
    self.latched = False
    self._base_at_fire = None
    self._a = FREE_A_MS2
    self.sight = None
    self.base = None
    self.v_cap = None
    self.a_cap = FREE_A_MS2
    self.would_fire = False
    self.reason = "idle"
    self.cue = ""
    self.fired_edge = False
    self.suppressed_edge = False
    self._veh_blocked = False
    self._veh_kind = ""
    self._veh_age = 1e9

  def _baseline(self, t: float):
    lo, hi = t - BASE_WIN_S[1], t - BASE_WIN_S[0]
    vals = [s for (tt, s, _r) in self._hist if lo <= tt <= hi]
    reds = [r for (tt, _s, r) in self._hist if lo <= tt <= hi]
    if len(vals) < BASE_MIN_SAMPLES:
      return None, False
    vals.sort()
    m = vals[len(vals) // 2]
    return m, (sum(reds) / len(reds)) >= RED_BASE_FRAC

  def update(self, *, t: float, dt: float, v_ego: float, sight_m, ks, conf_red: bool,
             lead: LeadInfo, rb_live: bool, rb_ahead: bool, others=()) -> bool:
    """One planner cycle. Returns would_fire."""
    self.fired_edge = False
    self.suppressed_edge = False
    v = float(v_ego)
    self.sight = None if sight_m is None or not math.isfinite(sight_m) else float(sight_m)
    kmax = max((abs(float(k)) for k in (ks or []) if math.isfinite(float(k))), default=0.0)
    self._kmax.append((t, kmax))
    while self._kmax and self._kmax[0][0] < t - CURV_RISE_S:
      self._kmax.popleft()
    if self.sight is not None:
      self._hist.append((t, self.sight, bool(conf_red)))
    while self._hist and self._hist[0][0] < t - BASE_WIN_S[1] - 1.0:
      self._hist.popleft()
    base, red_before = self._baseline(t)
    self.base = base

    k_old = self._kmax[0][1] if self._kmax else kmax
    curv_rising = kmax >= CURV_MIN_K and (kmax - k_old) >= CURV_RISE_K
    cue = "curv" if curv_rising else ("rb_map" if (rb_ahead and not rb_live) else "")

    shrink = self.sight is not None and base is not None and base > 0.0 and self.sight < SHRINK_RATIO * base
    self._shrink_t = self._shrink_t + dt if shrink else 0.0

    block = ""
    if rb_live:
      block = "rb_funnel"
    else:
      veh = next((k for k in (x.kind for x in (lead, *others)) if k), "")
      if veh:
        self._veh_kind, self._veh_age = veh, 0.0
      else:
        self._veh_age += dt
      if self._veh_kind and self._veh_age <= VEHICLE_HOLD_S:
        block = self._veh_kind
    if not block and not (V_MIN_MS <= v <= V_MAX_MS):
      block = "speed"

    veh_blocked = block not in ("", "rb_funnel", "speed") and self._shrink_t >= SHRINK_HOLD_S
    self.suppressed_edge = veh_blocked and not self._veh_blocked
    self._veh_blocked = veh_blocked
    if self.latched:
      recovered = self.sight is not None and self._base_at_fire is not None and self.sight >= RECOVER_RATIO * self._base_at_fire
      if block or recovered or self.sight is None:
        self.latched = False
        self.reason = block or ("recovered" if recovered else "no_path")
    else:
      if block:
        self.reason = block
      elif self._shrink_t < SHRINK_HOLD_S:
        self.reason = "idle"
      elif red_before and base < BASE_LONG_S * max(v, 1.0):
        self.reason = "red_before"
      elif not cue:
        self.reason = "no_cue"
      else:
        self.latched = True
        self.fired_edge = True
        self._base_at_fire = base
        self.cue = cue
        self.reason = "fire"

    self.v_cap = cap_speed_ms(self.sight) if self.sight is not None else None
    target = FREE_A_MS2
    if self.latched and self.v_cap is not None:
      target = max(-DECEL_MS2, min(FREE_A_MS2, CAP_GAIN_1_S * (self.v_cap - v)))
    # Would-be command, jerk limited on the way down.
    if target < self._a:
      self._a = max(target, self._a - JERK_MS3 * dt)
    else:
      self._a = target
    self.a_cap = self._a
    self.would_fire = self.latched
    return self.would_fire


def format_log_line(rec: dict) -> str:
  return LOG_PREFIX + json.dumps(rec, separators=(",", ":"))
