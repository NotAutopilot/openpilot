"""Select cruise/MAX set speed from the existing driver target and an OSM limit.

This is not a second longitudinal controller. It only returns the kph value that
card.py already publishes as CS.vCruise / vCruiseCluster (HUD "MAX").
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.constants import (
  ACCEL_DEFAULT, ACCEL_MAX, ACCEL_MIN, LOOKAHEAD_NORMAL, LOOKAHEAD_OFF,
  LOOKAHEAD_TUNING, MANUAL_SET_EPS_KPH, MIN_DECREASE_MS, MODE_CAP, MODE_DISPLAY,
  MODE_FOLLOW, MODE_OFF, POSTED_LIMIT_EPS_KPH, TRACK_DEADBAND_MS, TRACK_TAPER_MS,
  map_accel_a_ms2, map_brake_a_ms2, map_track_taper_ms,
)

# Oct 13 map limits. Untagged state defaults (OSM way with no maxspeed, filled
# by the pack) are estimates: Ridgewood Rd (tertiary, Sartell/St Joseph city
# polygon) went 55 -> 30 and the car braked 45 -> 33 mph at -1.6 m/s².
MAP_SOURCE_TAGGED = "osm"
MAP_SOURCE_DEFAULT = "osm_default"
# A drop to a default limit never brakes harder than this.
DEFAULT_DROP_A_MS2 = 0.5
# Untagged rural minor roads keep 55 along the same road.
RURAL_MINOR_HIGHWAYS = frozenset({"secondary", "secondary_link", "tertiary", "tertiary_link"})
RURAL_KEEP_MS = 55.0 * CV.MPH_TO_MS


def map_source_for(estimated: bool, sign: bool = False) -> str:
  """liveMapDataNAP.source value: sign override, tagged OSM, or state default."""
  if sign:
    return "sign"
  return MAP_SOURCE_DEFAULT if estimated else MAP_SOURCE_TAGGED


def keep_rural_default_ms(limit_ms: float, *, estimated: bool, highway: str, road_name: str,
                          prev: tuple[str, str, float] | None) -> float:
  """Keep 55 on an untagged secondary/tertiary when the same road was already 55+.

  prev is (road_name, highway, published limit) from the last cycle. A tagged
  limit, another road, or another class is returned unchanged.
  """
  lim = float(limit_ms)
  hw = str(highway or "").strip().lower()
  if not estimated or hw not in RURAL_MINOR_HIGHWAYS or lim >= RURAL_KEEP_MS - 0.3:
    return lim
  if prev is None:
    return lim
  p_name, p_hw, p_lim = prev
  base = hw.replace("_link", "")
  if not road_name or p_name != road_name or str(p_hw or "").replace("_link", "") != base:
    return lim
  if float(p_lim) >= RURAL_KEEP_MS - 0.3:
    return RURAL_KEEP_MS
  return lim


def default_drop_cap_ms(cap_ms: float, prev_ms: float | None, v_ego_ms: float, dt: float) -> float:
  """Ceiling that follows a lower DEFAULT limit down at DEFAULT_DROP_A_MS2 at most."""
  cap = float(cap_ms)
  start = max(float(v_ego_ms), cap) if prev_ms is None else float(prev_ms)
  return max(cap, start - DEFAULT_DROP_A_MS2 * max(0.0, float(dt)))


# Keep in sync with openpilot.selfdrive.car.cruise (avoid importing cereal here).
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255


def _clamp_accel_level(level: int) -> int:
  return max(ACCEL_MIN, min(ACCEL_MAX, int(level)))


def read_map_speed_params(params) -> tuple[int, float, int, int]:
  """Return (mode, offset_kph, lookahead, accel). Unknown keys → off / 0 / normal / 5."""
  try:
    mode = int(params.get("NAPMapSpeedMode", return_default=True) or 0)
    offset_mph = float(params.get("NAPMapSpeedOffsetMph", return_default=True) or 0)
    lookahead = int(params.get("NAPMapSpeedLookahead", return_default=True) or LOOKAHEAD_NORMAL)
    if lookahead not in LOOKAHEAD_TUNING:
      lookahead = LOOKAHEAD_NORMAL
    accel = _clamp_accel_level(int(params.get("NAPMapSpeedAccel", return_default=True) or ACCEL_DEFAULT))
    return mode, offset_mph * CV.MPH_TO_KPH, lookahead, accel
  except Exception:
    return MODE_OFF, 0.0, LOOKAHEAD_NORMAL, ACCEL_DEFAULT


def anticipatory_limit_ms(
  current_ms: float,
  next_ms: float,
  next_dist_m: float,
  v_ego_ms: float,
  lookahead: int,
  accel: int = ACCEL_DEFAULT,
) -> float | None:
  """Decrease-only cruise ceiling for an upcoming lower limit, or None.

  Uses comfort decel + margin so MAX eases down and the car is near the new
  limit as the lag-corrected position enters that way. A higher nextSpeedLimit
  far ahead never raises this ceiling; Follow still raises when the posted
  match at v_ego * 1.5 s is already the faster way (GNSS lag, not lookahead).
  """
  if lookahead <= LOOKAHEAD_OFF or lookahead not in LOOKAHEAD_TUNING:
    return None
  if current_ms <= 0 or next_ms <= 0:
    return None
  if next_ms >= current_ms - MIN_DECREASE_MS:
    return None
  _a_base, margin_m, horizon_m = LOOKAHEAD_TUNING[lookahead]
  if horizon_m <= 0:
    return None
  # Brake / anticipatory decreases are locked at Accel 5. `accel` is unused
  # (climb-only); kept so call sites can pass NAPMapSpeedAccel unchanged.
  _ = accel
  a_comfort = map_brake_a_ms2(lookahead)
  if a_comfort <= 0:
    return None
  v0 = max(float(v_ego_ms), float(current_ms), 0.0)
  vt = float(next_ms)
  if v0 <= vt:
    return None
  if next_dist_m <= 0:
    return vt
  need_m = (v0 * v0 - vt * vt) / (2.0 * a_comfort) + margin_m
  need_m = min(need_m, horizon_m)
  if next_dist_m > need_m:
    return None
  # Fall from v0 at need_m to vt at the sign. Using v² = vt² + 2 a d alone
  # wasted the margin: MAX stayed at the old limit until kinematic d, so a
  # 50→30 still arrived at the sign ~10 mph hot. Interpolate over the full window.
  span = max(need_m, 1e-6)
  v_cmd = math.sqrt(max(0.0, vt * vt + (v0 * v0 - vt * vt) * (float(next_dist_m) / span)))
  return max(vt, min(float(current_ms), v_cmd))


def effective_map_limit_ms(
  current_ms: float | None,
  next_ms: float = 0.0,
  next_dist_m: float = 0.0,
  v_ego_ms: float = 0.0,
  lookahead: int = LOOKAHEAD_NORMAL,
  accel: int = ACCEL_DEFAULT,
  sticky: bool = False,
) -> float | None:
  """Posted limit, optionally eased down for a closer lower limit ahead.

  A sticky stalk set must hold until posted `a` changes. Do not ease toward
  an upcoming lower OSM limit while that set is active — that fights the hold.
  """
  if current_ms is None or current_ms <= 0:
    return None
  if sticky:
    return float(current_ms)
  anticipated = anticipatory_limit_ms(
    float(current_ms), float(next_ms or 0.0), float(next_dist_m or 0.0),
    float(v_ego_ms or 0.0), lookahead, accel,
  )
  if anticipated is None:
    return float(current_ms)
  return min(float(current_ms), anticipated)


def map_slew_a_ms2(prev_ms: float, target_ms: float, lookahead: int, accel_level: int) -> float:
  """Slew rate for HUD/planner MAX: brake-locked a on decreases, Accel 1–10 on rises."""
  if float(target_ms) < float(prev_ms):
    return map_brake_a_ms2(lookahead)
  return map_accel_a_ms2(lookahead, accel_level)


def slew_map_speed_ms(prev_ms: float, target_ms: float, dt: float, a_ms2: float) -> float:
  """Rate-limit map-driven MAX (m/s) so a new limit does not cliff the HUD."""
  if dt <= 0 or a_ms2 <= 0:
    return float(target_ms)
  max_dv = float(a_ms2) * float(dt)
  delta = float(target_ms) - float(prev_ms)
  if abs(delta) <= max_dv:
    return float(target_ms)
  return float(prev_ms) + math.copysign(max_dv, delta)


def map_track_decel_ms2(v_ego_ms: float, v_cruise_ms: float, a_comfort: float) -> float | None:
  """Comfort decel (negative m/s²) to track a lower map MAX, or None.

  LongitudinalMpc's cruise column is a virtual lead at
  get_safe_obstacle_distance(v_ego) with V_EGO_COST=0. Holding 70 mph for
  10 s after MAX drops to 45 never violates that obstacle, so aTarget stays
  ~0 unless we command this. Lead/MPC may still request more braking via min().
  Callers must pass Accel-5 comfort a (map_brake_a_ms2); Accel 1–10 does not
  change brake rate.
  """
  if a_comfort <= 0 or v_ego_ms <= 0 or v_cruise_ms <= 0:
    return None
  dv = float(v_ego_ms) - float(v_cruise_ms)
  if dv <= TRACK_DEADBAND_MS:
    return None
  span = max(1e-6, TRACK_TAPER_MS - TRACK_DEADBAND_MS)
  scale = min(1.0, (dv - TRACK_DEADBAND_MS) / span)
  return -float(a_comfort) * scale


def map_climb_replaces_mpc(a_up, output_a_target, has_valid_lead: bool) -> bool:
  """True when map climb may replace a non-negative MPC aTarget.

  A valid radar lead owns follow. Replacing ~0 / slight+ MPC with Accel 1–10
  toward MAX punches through that lead, then overshoots and will not rematch.
  map_track_decel when above MAX is a separate min() and is not gated here.
  """
  return a_up is not None and float(output_a_target) >= 0.0 and not bool(has_valid_lead)


def map_track_accel_ms2(v_ego_ms: float, v_cruise_ms: float, a_comfort: float,
                        accel_level=None) -> float | None:
  """Comfort accel (positive m/s²) when catching a higher Follow MAX, or None.

  MPC cruise_obstacle will not climb to a higher MAX (V_EGO_COST=0). The
  planner commands this a when ego is below MAX and MPC is not braking
  *and* there is no valid radar lead. Accel 1–10 sets the climb rate *and*
  the last-mph taper (Accel 1 baby-steps ~5 mph out; Accel 7–10 stays brisk
  to the target). Lead gap-close uses this same envelope. A slower lead
  (negative aTarget) still wins.
  """
  if a_comfort <= 0 or v_ego_ms <= 0 or v_cruise_ms <= 0:
    return None
  dv = float(v_cruise_ms) - float(v_ego_ms)
  if dv <= TRACK_DEADBAND_MS:
    return None
  taper = map_track_taper_ms(accel_level)
  span = max(1e-6, float(taper) - TRACK_DEADBAND_MS)
  scale = min(1.0, (dv - TRACK_DEADBAND_MS) / span)
  return float(a_comfort) * scale


def map_in_track_deadband(v_ego_ms: float, v_set_ms: float) -> bool:
  """True when ego is close enough to MAX to hold, not climb or map-brake."""
  if v_ego_ms <= 0 or v_set_ms <= 0:
    return False
  return abs(float(v_ego_ms) - float(v_set_ms)) <= TRACK_DEADBAND_MS


def posted_limits_same(a_kph: float | None, b_kph: float | None) -> bool:
  if a_kph is None or b_kph is None:
    return False
  return abs(float(a_kph) - float(b_kph)) < POSTED_LIMIT_EPS_KPH


def is_manual_set_change(prev_kph: float, cur_kph: float) -> bool:
  """True when set speed moved (not engage 0↔set). Includes ego jumps."""
  if prev_kph in (0, V_CRUISE_UNSET) or cur_kph <= 0 or cur_kph >= V_CRUISE_UNSET:
    return False
  return abs(float(cur_kph) - float(prev_kph)) > MANUAL_SET_EPS_KPH


# Tesla stalk: tip/bump = 1 mph or 1 kph; full press/hold = 5 mph or 5 kph.
# Metric 1 kph and imperial 1 mph are both single-step (~1.0 / MPH_TO_KPH).
STALK_STEP_EPS_KPH = 0.55
CRUISE_STALK_TIP_STEPS_KPH = (1.0, CV.MPH_TO_KPH)
CRUISE_STALK_HOLD_STEPS_KPH = (5.0, 5.0 * CV.MPH_TO_KPH)


def _matches_cruise_stalk_steps(prev_kph: float, cur_kph: float, steps: tuple[float, ...]) -> bool:
  if not is_manual_set_change(prev_kph, cur_kph):
    return False
  delta = abs(float(cur_kph) - float(prev_kph))
  return any(abs(delta - step) < STALK_STEP_EPS_KPH for step in steps)


def is_cruise_stalk_tip_step(prev_kph: float, cur_kph: float) -> bool:
  """True if delta matches a Tesla stalk tip / bump (1 mph or 1 kph)."""
  return _matches_cruise_stalk_steps(prev_kph, cur_kph, CRUISE_STALK_TIP_STEPS_KPH)


def is_cruise_stalk_hold_step(prev_kph: float, cur_kph: float) -> bool:
  """True if delta matches a Tesla full press (5 mph or 5 kph)."""
  return _matches_cruise_stalk_steps(prev_kph, cur_kph, CRUISE_STALK_HOLD_STEPS_KPH)


def is_cruise_stalk_step(prev_kph: float, cur_kph: float) -> bool:
  """True if delta matches Tesla pedal stalk +/- (1 or 5 mph / kph).

  An ego / DI_digitalSpeed jump is not a stalk step and must not arm sticky.
  """
  return is_cruise_stalk_tip_step(prev_kph, cur_kph) or is_cruise_stalk_hold_step(prev_kph, cur_kph)


def _ref_posted_kph(hold: MapCruiseHold, posted_kph: float | None) -> float | None:
  if posted_kph is not None and posted_kph > 0:
    return float(posted_kph)
  if hold.last_posted_kph is not None and hold.last_posted_kph > 0:
    return float(hold.last_posted_kph)
  return None


def is_below_posted(set_kph: float, posted_kph: float | None) -> bool:
  if posted_kph is None or posted_kph <= 0:
    return False
  return float(set_kph) < float(posted_kph) - MANUAL_SET_EPS_KPH


@dataclass
class MapCruiseHold:
  """Engage-seed + sticky manual set while the posted OSM limit is unchanged.

  Follow holds the stalk set (above or below `a`) until posted leaves `a`.
  Cap still never exceeds the posted sign. No 10s raise-above timer.

  `held_max_kph` is the HUD MAX to resume after a long pause (brake).
  It survives `enableLongControl` dropping; only a full disengage
  (cruiseEnabled down) or a double SET (take-speed-now) forgets it.
  `last_posted_kph` is the last *known* OSM posted (+ offset). GPS / match
  drop must not clear it or invent a replacement.
  """
  last_posted_kph: float | None = None
  last_raw_kph: float = V_CRUISE_UNSET
  policy_kph: float | None = None
  sticky_set_kph: float | None = None
  held_max_kph: float | None = None
  follow_override_until: float = 0.0

  def reset(self) -> None:
    self.last_posted_kph = None
    self.last_raw_kph = V_CRUISE_UNSET
    self.policy_kph = None
    self.sticky_set_kph = None
    self.held_max_kph = None
    self.follow_override_until = 0.0


@dataclass
class MapCruiseDecision:
  """Cruise overlay for one card.py cycle.

  `seed_kph` is a one-shot pedal write (engage, posted-limit raise, or a
  real stalk step). None on a continuing sticky hold — writing every
  sticky frame undoes CI.update's 1/5 mph stalk step.
  """
  driver_kph: float
  follow_override: bool
  seed_kph: float | None
  sticky: bool


def _sticky_decision(hold: MapCruiseHold, mode: int, posted_kph: float | None,
                     write_pedal: bool = False) -> MapCruiseDecision:
  held = float(hold.sticky_set_kph)
  if mode == MODE_CAP and posted_kph is not None and posted_kph > 0:
    held = min(held, float(posted_kph))
  hold.policy_kph = held
  hold.held_max_kph = held
  # follow_override True so apply_map_speed cannot Follow-raise if a caller
  # ignores `sticky` and only passes the override flag.
  return MapCruiseDecision(held, True, held if write_pedal else None, True)


def _apply_posted_change(hold: MapCruiseHold, posted_kph: float, prev: float, *,
                         take_now: bool, resume_held: bool) -> MapCruiseDecision | None:
  """Rebase MAX onto a new posted limit and forget a stalk set.

  Known a → known b and none → valid share this path, including how a
  recent stalk / set-speed override is dropped. A raise seeds the pedal
  immediately. A decrease does not cliff-seed. Returns None when
  take-speed-now or one-SET resume still owns the pedal write.
  """
  hold.sticky_set_kph = None
  hold.follow_override_until = 0.0
  hold.last_posted_kph = posted_kph
  hold.held_max_kph = float(posted_kph)
  hold.last_raw_kph = float(posted_kph)
  hold.policy_kph = float(posted_kph)
  raised = float(posted_kph) > float(prev) + POSTED_LIMIT_EPS_KPH
  if take_now or resume_held:
    return None
  return MapCruiseDecision(
    float(posted_kph) if raised else float(prev),
    False,
    float(posted_kph) if raised else None,
    False,
  )


def _traveled_kph(traveled_kph: float | None, raw_kph: float) -> float:
  """Current traveled speed. Never invent a posted value."""
  if traveled_kph is not None and 0.0 < float(traveled_kph) < V_CRUISE_UNSET:
    return float(traveled_kph)
  return float(raw_kph)


def decide_map_cruise(
  hold: MapCruiseHold,
  *,
  engaged: bool,
  mode: int,
  raw_kph: float,
  posted_kph: float | None,
  engage_rising: bool,
  now: float,
  stalk_pressed: bool | None = None,
  take_speed_now: bool = False,
  resume_held: bool = False,
  traveled_kph: float | None = None,
  long_active: bool = True,
  self_wrote_pedal: bool = False,
) -> MapCruiseDecision:
  """Engage seed + sticky hold. posted_kph is OSM current maxspeed + offset.

  `engaged` is the OP session (`cruiseEnabled`), not just pedal-long. A
  brake long pause must keep sticky / held MAX. Full disengage
  (cancel / door / gear / steer fault) resets.

  Double SET / initial engage is `take_speed_now` (or `engage_rising` when
  not `resume_held` and no held MAX yet): forget sticky; MAX = current
  posted if maps+posted known, else current traveled speed. Never invent
  a posted value.

  One SET after a long pause is `resume_held`: restore held MAX, which may
  already have rebased if posted changed under maps. `resume_held` is a
  one-shot; `engage_rising` can arrive a frame later (`pedalLongActive`
  lags `enableLongControl`). That delayed rising edge must not take-now
  and overwrite a held MAX with current traveled speed.

  Maps on: MAX rebases when the posted value changes (known a → known b)
  and when a missing limit becomes a posted limit (none → valid), including
  while long-paused. Both drop a stalk set the same way a 55 → 70 change
  does. A posted limit becoming missing (valid → invalid) holds the
  current MAX: no drop and no reseed to ego, and sticky is left alone.
  The same posted value coming back after a dropout is not a change.

  Maps off / display: never auto-rebase.

  `stalk_pressed` is extra (button edge). A 1/5 mph pedal_speed step always
  counts as a stalk while long is active; an ego jump does not. Do not treat
  `stalk_pressed=False` as "ignore pedal_speed" — pre-AP button events are
  not reliable. Pause frames must not arm sticky from cruiseState.speed
  falling back to ego.

  `self_wrote_pedal` is the frame that echoes a pedal_speed this code just
  wrote. That step is not a driver stalk press.

  Pedal write (`seed_kph`) only on take-speed-now, resume-held, a real 1/5
  mph stalk step, or a posted-limit *raise*. A continuing sticky hold must
  not write — that overwrites CI.update's stalk step on the same frame.
  """
  _ = now
  # engage_rising without a held MAX is initial engage (take current).
  # A held MAX means this session already has a MAX — one SET after a
  # pause must resume it even if resume_held was consumed last frame.
  take_now = bool(take_speed_now) or (
    bool(engage_rising) and not bool(resume_held) and hold.held_max_kph is None
  )
  maps_control = mode in (MODE_CAP, MODE_FOLLOW)

  if not engaged:
    hold.reset()
    return MapCruiseDecision(raw_kph, False, None, False)

  if not maps_control:
    # Maps off / display / unknown posted: never invent, never auto-rebase.
    if take_now:
      hold.sticky_set_kph = None
      seed = _traveled_kph(traveled_kph, raw_kph)
      hold.held_max_kph = seed
      hold.last_raw_kph = seed
      hold.policy_kph = seed
      return MapCruiseDecision(seed, False, seed, False)
    if long_active:
      # Pedal/HUD MAX is the source of truth while long is active. Stalk
      # +/- must update the MAX one SET will resume. Pause publishes
      # cruiseState.speed as ego — never latch that into held.
      stalk_step = (not self_wrote_pedal) and is_cruise_stalk_step(hold.last_raw_kph, raw_kph)
      if 0.0 < raw_kph < V_CRUISE_UNSET and (
        hold.held_max_kph is None or stalk_step or bool(stalk_pressed)
      ):
        hold.held_max_kph = float(raw_kph)
      hold.last_raw_kph = raw_kph
    if hold.held_max_kph is not None:
      held = float(hold.held_max_kph)
      hold.policy_kph = held
    else:
      # Paused with no latch: report raw for this frame only. Do not store
      # it — pause raw is ego.
      held = float(raw_kph)
    seed = float(held) if resume_held and hold.held_max_kph is not None else None
    return MapCruiseDecision(float(held), False, seed, False)

  posted_ok = posted_kph is not None and posted_kph > 0
  # Pause: cruiseState.speed is ego, not MAX. Do not treat that as a stalk.
  stalk_step = (not self_wrote_pedal) and bool(long_active) and is_cruise_stalk_step(hold.last_raw_kph, raw_kph)
  manual = bool(long_active) and (stalk_step or bool(stalk_pressed))
  if long_active:
    hold.last_raw_kph = raw_kph
  # valid → invalid. Keep the MAX we already have. cruiseState.speed
  # falling back to ego (including a 1 or 5 mph gap) is not a stalk and
  # must not reseed.
  if (
    not posted_ok
    and hold.last_posted_kph is not None
    and not take_now
    and traveled_kph is not None
    and abs(float(raw_kph) - float(traveled_kph)) <= MANUAL_SET_EPS_KPH
  ):
    manual = False
    stalk_step = False
    anchor = hold.held_max_kph if hold.held_max_kph is not None else hold.policy_kph
    if anchor is not None:
      hold.last_raw_kph = float(anchor)

  posted_changed = (
    posted_ok
    and hold.last_posted_kph is not None
    and not posted_limits_same(hold.last_posted_kph, posted_kph)
  )
  if posted_changed:
    # Posted value itself changed (known a → known b), including while
    # long-paused. Forget sticky and rebase MAX to the new posted. A raise
    # seeds immediately. A decrease does not cliff-seed — Cap/Follow ease
    # via map_kph (kin+110 m) — unless this frame is one SET resume, which
    # must write the already-rebased held MAX onto pedal.
    changed = _apply_posted_change(
      hold, posted_kph, float(hold.last_posted_kph),
      take_now=take_now, resume_held=resume_held,
    )
    if changed is not None:
      return changed

  if take_now:
    hold.sticky_set_kph = None
    hold.follow_override_until = 0.0
    if posted_ok:
      seed = float(posted_kph)
      hold.last_posted_kph = posted_kph
    else:
      # Unknown posted: never invent a limit. Hold current traveled speed
      # until a real posted value appears. That later none → valid step is
      # a normal limit change, not a timed engage exception.
      seed = _traveled_kph(traveled_kph, raw_kph)
      hold.sticky_set_kph = seed
    hold.held_max_kph = seed
    hold.last_raw_kph = seed
    hold.policy_kph = seed
    return MapCruiseDecision(seed, hold.sticky_set_kph is not None, seed,
                             hold.sticky_set_kph is not None)

  if posted_ok and hold.last_posted_kph is None:
    # none → valid while long is already engaged. Same rebase as a posted
    # change, at any time, including a stalk set made during the dropout.
    # If MAX is already the returning limit, leave it: a same-value return
    # must not write the pedal again.
    prev = float(hold.held_max_kph if hold.held_max_kph is not None else raw_kph)
    if not posted_limits_same(prev, posted_kph):
      changed = _apply_posted_change(
        hold, posted_kph, prev, take_now=False, resume_held=resume_held,
      )
      if changed is not None:
        return changed

  if posted_ok:
    # GPS return: record posted without rebasing (not a value change).
    hold.last_posted_kph = posted_kph

  if resume_held:
    held = hold.sticky_set_kph if hold.sticky_set_kph is not None else hold.held_max_kph
    if held is None:
      held = float(raw_kph)
    if mode == MODE_CAP and posted_ok:
      held = min(float(held), float(posted_kph))
    hold.held_max_kph = float(held)
    hold.policy_kph = float(held)
    sticky = hold.sticky_set_kph is not None
    return MapCruiseDecision(float(held), sticky, float(held), sticky)

  ref_posted = _ref_posted_kph(hold, posted_kph)

  if manual:
    hold.policy_kph = float(raw_kph)
    hold.follow_override_until = 0.0
    if mode == MODE_FOLLOW:
      hold.sticky_set_kph = float(raw_kph)
    elif is_below_posted(raw_kph, ref_posted) or ref_posted is None:
      hold.sticky_set_kph = float(raw_kph)
    else:
      # Cap: stalk at or above posted is not sticky; apply_map_speed caps.
      hold.sticky_set_kph = None
    hold.held_max_kph = float(raw_kph) if hold.sticky_set_kph is not None else hold.held_max_kph

  if hold.policy_kph is None:
    hold.policy_kph = float(raw_kph)

  if hold.sticky_set_kph is not None:
    # Write once when pedal_speed actually stepped. stalk_pressed with
    # unchanged raw must not write the old hold over CI's in-flight step.
    return _sticky_decision(hold, mode, posted_kph, write_pedal=stalk_step)

  if posted_ok and mode == MODE_FOLLOW:
    hold.held_max_kph = float(posted_kph)
  elif hold.held_max_kph is None:
    hold.held_max_kph = float(hold.policy_kph)

  return MapCruiseDecision(float(hold.policy_kph), False, None, False)


def should_write_preap_pedal(seed_kph: float | None, hud_kph: float,
                             last_pedal_kph: float | None) -> bool:
  """When to write HUD MAX onto pre-AP pedal_speed.

  Write on engage/posted seed or a real stalk step (`seed_kph`). Also write
  when MAX *rose* vs the last pedal write (Follow posted raise backup).
  Do not write every sticky/Follow frame — that ate stalk +/-.
  """
  if seed_kph is not None:
    return True
  if last_pedal_kph is None:
    return False
  return float(hud_kph) > float(last_pedal_kph) + MANUAL_SET_EPS_KPH


def apply_map_speed_kph(
  driver_set_kph: float,
  map_limit_kph: float | None,
  *,
  mode: int,
  offset_kph: float = 0.0,
  engaged: bool = False,
  op_long_software_cruise: bool = False,
  driver_override: bool = False,
  v_min: float = V_CRUISE_MIN,
  v_max: float = V_CRUISE_MAX,
) -> float:
  """Return the kph target for vCruise / HUD MAX.

  Control (cap/follow) only runs when NAP owns software cruise — pre-AP pedal
  mode (`openpilotLongitudinalControl` and not `pcmCruise`). Display and off
  never change the driver/stock target.
  """
  if driver_set_kph <= 0 or driver_set_kph >= V_CRUISE_UNSET:
    return driver_set_kph

  driver = float(driver_set_kph)
  if mode in (MODE_OFF, MODE_DISPLAY) or not engaged:
    return driver
  if not op_long_software_cruise:
    return driver
  if map_limit_kph is None or map_limit_kph <= 0:
    return driver

  map_target = max(v_min, min(v_max, float(map_limit_kph) + float(offset_kph)))

  if mode == MODE_CAP:
    return min(driver, map_target)

  if mode == MODE_FOLLOW:
    # driver_override is a sticky hold backup. Card usually applies sticky
    # via seed_kph before this function. No 10s snap-back to posted.
    if driver_override:
      return max(v_min, min(v_max, driver))
    return map_target

  return driver


def cap_planner_v_cruise_ms(
  v_cruise_ms: float,
  map_limit_ms: float | None,
  *,
  mode: int,
  offset_ms: float = 0.0,
) -> float:
  """HUD MAX is already card policy.

  Trust HUD for Cap and Follow. min() with posted snapped MAX when GPS or
  the 1.5 s offset entered a lower zone, skipping kin+110 m ease. Card Cap
  still never exceeds the eased ceiling. Lead still wins via mpc.update.
  """
  _ = map_limit_ms, mode, offset_ms
  return v_cruise_ms


# Same column order as LongitudinalMpc.update:
# x_obstacles = [lead0, lead1, cruise_obstacle(v_cruise)]
SOURCE_LEAD0 = 0
SOURCE_LEAD1 = 1
SOURCE_CRUISE = 2


def longitudinal_obstacle_source(lead0_m: float, lead1_m: float, cruise_m: float) -> int:
  """Return which MPC obstacle is tightest at t=0 (0=lead0, 1=lead1, 2=cruise).

  Map speed only changes the cruise column. A closer/slower lead must still win.
  """
  stacked = (float(lead0_m), float(lead1_m), float(cruise_m))
  return min(range(3), key=lambda i: stacked[i])
