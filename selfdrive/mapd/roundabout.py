"""Roundabout detect, speed ease, and outer path bias.

Map `junction=roundabout` (or a closed circulating way already in the OSM
pack) — not steer. Decel onset ~140 m (was 200 m, Oct 1) so 35 mph can hit 18 by the
ring. Kinematic a (not Lookahead comfort −0.55) toward OSM maxspeed when
present. Clamp aTarget ≤ 0 while approaching / on the ring.

Lateral: the legacy ~3.2 m outer path bias (#223) is applied only when the
turn geometry correction (NAPLatTurnGeom, default On for Pre-AP) is off.
With the correction on, modeld owns turn-in timing / rear-axle reference and
roundabouts get no extra lateral bias (no double compensation).

Yield-before-merge, continue-circulate desire, and UI chip are later tips.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.constants import LOOKAHEAD_NORMAL

# Speed-based decel onset (Oct 11, Watertown SD): the fixed 140 m was tuned
# for 35->18 mph and was late / abrupt from a 55 mph approach (needs ~340 m
# at 0.7 m/s²). Onset = (v² − v_ring²)/(2·0.7) + 30 m, clamped 140–450 m.
RB_ONSET_A_MS2 = 0.7
RB_ONSET_MARGIN_M = 30.0
RB_ONSET_MIN_M = 140.0
RB_ONSET_MAX_M = 450.0
# MAX profile reaches ring speed this far before the ring line.
RB_PROFILE_BUFFER_M = 15.0
# Kinematic −a ceiling: never brick-wall.
RB_A_MAX_DECEL_MS2 = -1.5
# Planner jerk limit on the ring −a onset (m/s³).
RB_DECEL_JERK_MS3 = 1.5
# Detect / start ease by this distance (the decel onset). Must still have fired by MIN.
# Oct 1: 200 m (first fire 187 m in the field) braked ~0.2 m/s² earlier than needed and had the car crawling at 18 mph well before the ring
# (0000013c); 140 m keeps 35→18 mph at ~0.64 m/s² average. 100 m was too late (Willmar 10:11 / 10:14).
RB_DECEL_ONSET_M = RB_ONSET_MIN_M  # low-speed onset (35->18 mph); see rb_decel_onset_m
# Search / latch reach: the longest speed-based onset.
RB_FUNNEL_M = RB_ONSET_MAX_M
RB_FUNNEL_MIN_M = 80.0
# ~0.0065° ≈ 510 m EW at 45°N (720 m N-S) so a 450 m approach is in the rtree.
RB_SEARCH_PAD_DEG = 0.0065

# Circulating target. OSM maxspeed on the RB way wins when present (often 20).
# Sep 30: entry was a little slow. Soco ring pass speeds were 17-19 mph (R126 17.8, R128 19.3, R130 17.5 median); the
# 2.5 m/s² comfort speed (15.8 mph) is now floored here, so the planner's ring speed (and the guide's) is 18 mph.
RB_RING_SPEED_MPH = 18.0
RB_V_MIN_MS = RB_RING_SPEED_MPH * CV.MPH_TO_MS
RB_V_MAX_MS = 20.0 * CV.MPH_TO_MS
RB_V_DEFAULT_MS = 20.0 * CV.MPH_TO_MS

# Kinematic ease. Lookahead Early comfort is only 0.55 — that is why tip
# aTarget sat at −0.55 at 37–45 mph. Plan to ring speed from remaining d.
RB_A_MIN_MS2 = RB_A_MAX_DECEL_MS2
RB_A_FLOOR_MS2 = -1.0
RB_A_FLOOR_V_MS = 40.0 * CV.MPH_TO_MS
RB_NEAR_SPEED_MS = 2.5 * CV.MPH_TO_MS
RB_ON_RING_DECEL_M = 28.0
RB_DECEL_D_MIN_M = 12.0

# Legacy outer path bias (turn geometry correction OFF only). openpilot +y is left. EP1: OP lat R=14.9 vs OSM 18.2
# (−3.3 m inside); 0.45 m was far too small. 3.2 m counters that cut and
# sits near the outer half of the circulating lane / ring, not the island.
RB_OUTER_OFFSET_M = 3.2
RB_OUTER_OFFSET_MIN_M = 2.8
RB_OUTER_OFFSET_MAX_M = 3.6
# Preview length for a parallel-path curvature offset (κ ≈ 2 y / L²).
RB_PATH_LOOKAHEAD_M = 18.0
# Apply the same outer bias on the last stretch of approach (not 200 m out).
RB_ENTRY_BIAS_M = 50.0

# Closed-loop geometry for published packs that do not store `junction`.
RB_RADIUS_MIN_M = 6.0
RB_RADIUS_MAX_M = 55.0
RB_CLOSED_GAP_M = 8.0
RB_ON_WAY_M = 25.0
# Service / path / residential loops (cul-de-sac bulbs, Maritime/Seaway) are
# not circulating RBs unless tagged junction=roundabout|circular.
_UNTAGGED_SKIP_HIGHWAY = frozenset({
  "service", "footway", "path", "cycleway", "pedestrian", "steps", "track",
  "residential", "living_street",
})
_RB_JUNCTIONS = frozenset({"roundabout", "circular"})
# Freeway / trunk mainline. A side-street loop beside I-74 is not an upcoming
# roundabout. Town rings are not on these classes (Willmar stays on residential).
_RB_SUPPRESS_HIGHWAY = frozenset({
  "motorway", "motorway_link", "trunk", "trunk_link",
})

EARTH_R = 6371000.0


@dataclass(frozen=True)
class RoundaboutHint:
  on_roundabout: bool = False
  approaching: bool = False
  distance_m: float = 0.0
  speed_limit_ms: float = 0.0
  way_id: int = 0


def rb_decel_onset_m(v_ego_ms: float | None, v_ring_ms: float | None = None) -> float:
  """Distance before the ring to start easing: (v² − v_ring²)/(2·0.7) + 30 m, 140–450 m."""
  if v_ego_ms is None:
    return RB_ONSET_MIN_M
  vr = RB_V_MIN_MS if v_ring_ms is None else float(v_ring_ms)
  v = max(float(v_ego_ms), 0.0)
  d = max(0.0, v * v - vr * vr) / (2.0 * RB_ONSET_A_MS2) + RB_ONSET_MARGIN_M
  return max(RB_ONSET_MIN_M, min(RB_ONSET_MAX_M, d))


def rb_profile_v_ms(distance_m: float, target_ms: float) -> float:
  """MAX ceiling on approach: reach target RB_PROFILE_BUFFER_M before the ring at 0.7 m/s²."""
  d = max(0.0, float(distance_m) - RB_PROFILE_BUFFER_M)
  return math.sqrt(float(target_ms) ** 2 + 2.0 * RB_ONSET_A_MS2 * d)


def junction_is_roundabout(junction: str | None) -> bool:
  return (junction or "").strip().lower() in _RB_JUNCTIONS


def roundabout_suppressed_for_highway(highway: str | None) -> bool:
  """True when the ego match is freeway / trunk, so RB hints must not publish.

  I-74 (highway=motorway, posted 65) latched MAX to 20 mph off untagged
  residential loops beside the carriageway. Real town roundabouts are not
  matched onto these classes.
  """
  return (highway or "").strip().lower() in _RB_SUPPRESS_HIGHWAY


def _local_xy(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
  x = math.radians(lon - lon0) * math.cos(math.radians(lat0)) * EARTH_R
  y = math.radians(lat - lat0) * EARTH_R
  return x, y


def way_radius_m(coords: list[tuple[float, float]] | tuple[tuple[float, float], ...]) -> float | None:
  if len(coords) < 3:
    return None
  lat0 = sum(c[0] for c in coords) / len(coords)
  lon0 = sum(c[1] for c in coords) / len(coords)
  rs = [math.hypot(*_local_xy(c[0], c[1], lat0, lon0)) for c in coords]
  r = sum(rs) / len(rs)
  return r if r > 0.5 else None


def way_is_closed_loop(coords: list[tuple[float, float]] | tuple[tuple[float, float], ...]) -> bool:
  if len(coords) < 4:
    return False
  gap = math.hypot(*_local_xy(coords[-1][0], coords[-1][1], coords[0][0], coords[0][1]))
  return gap <= RB_CLOSED_GAP_M


def way_is_roundabout(
  coords: list[tuple[float, float]] | tuple[tuple[float, float], ...],
  junction: str = "",
  highway: str = "",
) -> bool:
  """True for OSM circulating rings — not a sharp corner or cross street.

  Tagged `junction=roundabout|circular` is authoritative (multi-way rings).
  Published speed-limit packs may omit that column; a compact closed loop
  of typical RB radius is the fallback. Signalized intersections are
  crossing polylines, not a closed circulating way.
  """
  tagged = junction_is_roundabout(junction)
  r = way_radius_m(coords)
  if tagged:
    if r is None:
      return True
    return r <= (RB_RADIUS_MAX_M * 2.5)
  hw = (highway or "").strip().lower()
  if hw in _UNTAGGED_SKIP_HIGHWAY:
    return False
  if not way_is_closed_loop(coords) or r is None:
    return False
  return RB_RADIUS_MIN_M <= r <= RB_RADIUS_MAX_M


def roundabout_target_ms(speed_limit_ms: float = 0.0) -> float:
  """Soft circulating v. OSM ring maxspeed when present, else 20 mph."""
  if speed_limit_ms is not None and float(speed_limit_ms) > 0.5:
    return max(RB_V_MIN_MS, min(RB_V_MAX_MS, float(speed_limit_ms)))
  return RB_V_DEFAULT_MS


def roundabout_hint_active(hint: RoundaboutHint | None) -> bool:
  return hint is not None and (hint.on_roundabout or hint.approaching)


def roundabout_ease_v_ms(
  hint: RoundaboutHint | None,
  v_ego_ms: float,
  current_ms: float = 0.0,
  lookahead: int = LOOKAHEAD_NORMAL,
) -> float | None:
  """Ring target as soon as the funnel is live. None if no RB.

  Do not interpolative-ease MAX over another 100 m — that left aTarget at
  Lookahead comfort (−0.55) after long enabled inside the funnel. `_` args
  kept so card / planner call sites stay unchanged.
  """
  _ = v_ego_ms, current_ms, lookahead
  if not roundabout_hint_active(hint):
    return None
  target = roundabout_target_ms(hint.speed_limit_ms)
  if hint.on_roundabout or float(hint.distance_m) <= 0.0:
    return target
  # Oct 11: smooth MAX ramp down the 0.7 m/s² profile (no 35->18 step).
  return rb_profile_v_ms(hint.distance_m, target)


def roundabout_decel_ms2(hint: RoundaboutHint | None, v_ego_ms: float) -> float | None:
  """Kinematic −a to hit ring speed by remaining d, or 0 to clamp +a.

  None if no RB hint. While approaching / on-ring, never returns +a.
  From ≥40 mph, |a| is at least 1.0 until near ring speed (Willmar 10:11
  / 10:14: −0.55 at 37–45 mph was about half of 37→20 over 80–95 m).
  """
  if not roundabout_hint_active(hint):
    return None
  target = roundabout_target_ms(hint.speed_limit_ms)
  v0 = max(float(v_ego_ms), 0.0)
  if hint.on_roundabout or float(hint.distance_m) <= 0.0:
    d = RB_ON_RING_DECEL_M
  else:
    d = max(float(hint.distance_m), RB_DECEL_D_MIN_M)
  if v0 <= target + RB_NEAR_SPEED_MS:
    return 0.0
  if not hint.on_roundabout and float(hint.distance_m) > 0.0:
    d = max(d - RB_PROFILE_BUFFER_M, RB_DECEL_D_MIN_M)
  a_kin = (target * target - v0 * v0) / (2.0 * d)
  a = max(RB_A_MIN_MS2, min(0.0, a_kin))
  # Willmar floor only for a late hint (inside the speed-based onset by
  # more than the margin). On a normal early onset the 0.7 profile rules.
  late = float(hint.distance_m) < rb_decel_onset_m(v0, target) - RB_ONSET_MARGIN_M
  if v0 >= RB_A_FLOOR_V_MS and (hint.on_roundabout or late):
    a = min(a, RB_A_FLOOR_MS2)
  return max(a, RB_A_MAX_DECEL_MS2)


def apply_roundabout_plan(
  v_ego_ms: float,
  v_cruise_ms: float,
  v_hud_ms: float,
  output_a_target: float,
  hint: RoundaboutHint | None,
  lookahead: int = LOOKAHEAD_NORMAL,
) -> tuple[float, float, float, float | None]:
  """Cap cruise / HUD to ring speed and min kinematic −a. No-op if no hint.

  Returns (v_cruise, v_hud, a_target, rb_v or None). a_target is ≤ 0 while
  the funnel is live (no +a rebound after a lead clears). Long-enable in
  the funnel is the same call — full ease on the first frame.
  """
  rb_v = roundabout_ease_v_ms(hint, v_ego_ms, v_hud_ms, lookahead)
  if rb_v is None:
    return float(v_cruise_ms), float(v_hud_ms), float(output_a_target), None
  v_cruise_ms = min(float(v_cruise_ms), float(rb_v))
  v_hud_ms = min(float(v_hud_ms), float(rb_v))
  a_rb = roundabout_decel_ms2(hint, v_ego_ms)
  if a_rb is not None:
    output_a_target = min(float(output_a_target), a_rb)
  output_a_target = min(float(output_a_target), 0.0)
  return v_cruise_ms, v_hud_ms, float(output_a_target), float(rb_v)


def live_map_roundabout_hint(md) -> RoundaboutHint | None:
  """Read LiveMapDataNAP RB fields. Missing / old messages → None."""
  if md is None:
    return None
  try:
    on_rb = bool(getattr(md, "onRoundabout", False))
    approaching = bool(getattr(md, "approachingRoundabout", False))
    if not on_rb and not approaching:
      return None
    return RoundaboutHint(
      on_roundabout=on_rb,
      approaching=approaching,
      distance_m=float(getattr(md, "roundaboutDistance", 0.0) or 0.0),
      speed_limit_ms=float(getattr(md, "roundaboutSpeedLimit", 0.0) or 0.0),
      way_id=int(getattr(md, "roundaboutWayId", 0) or 0),
    )
  except Exception:
    return None


def _outer_bias_active(*, on_roundabout: bool, approaching: bool, distance_m: float) -> bool:
  if on_roundabout:
    return True
  return bool(approaching) and 0.0 < float(distance_m) <= RB_ENTRY_BIAS_M


def roundabout_outer_path_offset_m(
  *,
  on_roundabout: bool,
  approaching: bool = False,
  distance_m: float = 0.0,
  is_rhd: bool = False,
) -> float:
  """Outer-lane path offset in openpilot y (left +). 0 off the ring / far approach.

  RHT / US: outer is right → negative y. RHD: outer is left → positive y.
  Magnitude counters an EP1-class ~3 m inside cut and holds the outer half
  of the circulating lane. Applied on-ring and in the last 50 m of approach
  (not 200 m out on a straight).
  """
  if not _outer_bias_active(
    on_roundabout=on_roundabout, approaching=approaching, distance_m=distance_m,
  ):
    return 0.0
  mag = max(RB_OUTER_OFFSET_MIN_M, min(RB_OUTER_OFFSET_MAX_M, RB_OUTER_OFFSET_M))
  return mag if is_rhd else -mag


def roundabout_outer_curvature_bias(offset_m: float, lookahead_m: float = RB_PATH_LOOKAHEAD_M) -> float:
  """Curvature add-on that tracks a parallel path offset_m to the left.

  Positive offset (left) → positive curvature. 3.2 m right ≈ −0.020 /m at
  the 18 m preview — enough to walk a 3 m inside model path back to the
  OSM ring / outer half of the lane.
  """
  if abs(float(offset_m)) < 1e-6:
    return 0.0
  L = max(8.0, float(lookahead_m))
  return 2.0 * float(offset_m) / (L * L)


def roundabout_lateral_curvature_bias(hint: RoundaboutHint | None, *, is_rhd: bool = False,
                                      turn_geometry_active: bool = False) -> float:
  """Curvature add-on controlsd applies to the model / maneuver curvature.

  Turn geometry correction ON (NAPLatTurnGeom, Pre-AP default): 0. The
  3.2 m outer bias was tuned against the EP1 inside cut, which included
  early turn-in / corner clipping that the correction now handles in
  modeld; keeping both would double-compensate (run wide on entry).
  Correction OFF: legacy #223 outer bias, unchanged from 1119c0f.
  Roundabout detection and speed ease are not affected either way.
  """
  if turn_geometry_active:
    return 0.0
  return roundabout_outer_curvature_bias(
    roundabout_outer_path_offset_m(
      on_roundabout=bool(hint.on_roundabout) if hint is not None else False,
      approaching=bool(hint.approaching) if hint is not None else False,
      distance_m=float(hint.distance_m) if hint is not None else 0.0,
      is_rhd=is_rhd,
    ),
  )
