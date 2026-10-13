#!/usr/bin/env python3
import math
import time

import numpy as np

import cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  LongitudinalMpc,
  LongitudinalPlanSource,
  get_safe_obstacle_distance,
  get_stopped_equivalence_factor,
  get_T_FOLLOW,
)
from openpilot.selfdrive.controls.lib.curve_max_hold import curve_speed_for_curvature
from openpilot.selfdrive.controls.lib.curve_preview import (
  PREVIEW_FREE_A_MS2,
  CurvePreview,
  curve_preview_accel,
  path_curvature,
  path_lat_accel_ahead,
  turn_accel_limit,
)
from openpilot.selfdrive.controls.lib.curve_follow import (
  LOG_PERIOD_CYCLES,
  LOG_QUIET_PERIOD_CYCLES,
  MODE_ACTIVE,
  MODE_OFF,
  CurveFollow,
  format_log_line,
  path_is_relevant,
  read_curve_follow_mode,
)
from openpilot.selfdrive.controls.lib.radar_path_gate import model_path_xy, path_lateral_m
from openpilot.selfdrive.controls.lib.model_sight import model_sight
from openpilot.selfdrive.controls.lib.sight_cap import (
  MODE_OFF as SIGHT_MODE_OFF, SightCap, classify_lead, crest_ahead, read_sight_cap_mode,
)
from openpilot.selfdrive.controls.lib.sight_cap import LOG_PERIOD_S as SIGHT_LOG_PERIOD_S
from openpilot.selfdrive.controls.lib.sight_cap import format_log_line as sight_log_line
from openpilot.selfdrive.controls.lib.lead_leaving import LeadLeavingEstimator
from openpilot.selfdrive.controls.lib.unified_lead import UnifiedLeadController, unified_follow_desired
from openpilot.selfdrive.controls.lib.gap_lock import (
  GapLockLatch,
  gap_lock_param_enabled,
  stalk_follow_exit,
)
from openpilot.selfdrive.controls.lib.lead_approach import (
  lead_approach_track_ok,
  lead_close_should_cap,
  resolve_lead_close_hold,
)
from openpilot.selfdrive.controls.lib.follow_distance import FollowDistanceBlend, NAP_FOLLOW_DISTANCE_RANGE
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.selfdrive.mapd.constants import MODE_CAP, MODE_FOLLOW, map_accel_a_ms2, map_brake_a_ms2
from openpilot.selfdrive.mapd.map_speed_policy import (
  MAP_SOURCE_DEFAULT, default_drop_cap_ms,
  cap_planner_v_cruise_ms, map_climb_replaces_mpc, map_in_track_deadband, map_track_accel_ms2,
  map_track_decel_ms2, read_map_speed_params,
)
from openpilot.selfdrive.mapd.roundabout import RB_DECEL_JERK_MS3, RingExitGate, apply_roundabout_plan, live_map_roundabout_hint
from openpilot.selfdrive.controls.lib.hill_climb import (
  apply_hill_climb, read_hypermile_hill_climb,
)
from openpilot.selfdrive.controls.lib.hypermile import (
  effective_nap_follow_dist, read_hypermile_params,
)
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog


A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]

# Pre-AP follow-mode accel cap: imported lazily to avoid circular deps
_preap_follow_cache = None
def _get_preap_follow_limit(v_ego):
  global _preap_follow_cache
  if _preap_follow_cache is None:
    try:
      from opendbc.car.tesla.preap.constants import ACCEL_PREAP_BP, ACCEL_PREAP_FOLLOW
      _preap_follow_cache = (ACCEL_PREAP_BP, ACCEL_PREAP_FOLLOW)
    except ImportError:
      _preap_follow_cache = (None, None)
  bp, v = _preap_follow_cache
  if bp is None:
    return None
  return float(np.interp(v_ego, bp, v))


def get_preap_follow_cap_strength(v_ego, lead_distance, lead_speed, t_follow):
  lead_obstacle_distance = lead_distance + get_stopped_equivalence_factor(max(lead_speed, 0.0))
  safe_obstacle_distance = get_safe_obstacle_distance(v_ego, t_follow)
  equivalent_ratio = lead_obstacle_distance / max(safe_obstacle_distance, 1.0)
  return float(np.clip(1.0 - (equivalent_ratio - 1.2) / 0.3, 0.0, 1.0))


CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py

def limit_accel_in_turns(v_ego, angle_steers, a_target, CP, a_y=None, a_y_ahead=None):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns.

  Pre-AP HUD MAX snapshot/restore through a bend lives in card.py
  (CurveMaxHold). This clip is temporary +a only — it must not rebase
  vCruise / sticky MAX.

  With true cornering force (`a_y`, vehicle-model curvature) the clip is a
  friction circle on the smaller of now and the model path 1 s ahead, so
  +a returns near the apex. Without it, the stock steer-model table stays.
  """
  if a_y is not None:
    return [a_target[0], min(a_target[1], turn_accel_limit(v_ego, a_y, a_y_ahead))]
  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]



# First-frame brake head start for a new lead (m/s^2 below the lead-free command).
SEED_BRAKE_STEP_MS2 = 0.7
CYCLE_LOG_EVERY_N = 600  # ~30 s at the 20 Hz model rate


class CycleTimeLog:
  """Running planner update() cost, logged every CYCLE_LOG_EVERY_N cycles.

  The unified controller is the only lead-follow path, so this is the number
  to compare against older builds when checking plannerd CPU.
  """

  def __init__(self, every_n: int = CYCLE_LOG_EVERY_N) -> None:
    self.every_n = int(every_n)
    self.reset()

  def reset(self) -> None:
    self.n = 0
    self.total_s = 0.0
    self.max_s = 0.0

  def record(self, elapsed_s: float):
    """Add one cycle. Returns (n, mean_ms, max_ms) when a window closes, else None."""
    self.n += 1
    self.total_s += float(elapsed_s)
    self.max_s = max(self.max_s, float(elapsed_s))
    if self.n < self.every_n:
      return None
    out = (self.n, 1000.0 * self.total_s / self.n, 1000.0 * self.max_s)
    self.reset()
    return out


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL, params=None):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self._is_preap = (CP.brand == "tesla" and CP.carFingerprint == "TESLA_MODEL_S_PREAP"
                       and CP.openpilotLongitudinalControl and not CP.pcmCruise)
    self._params = Params() if params is None else params
    self.nap_follow_dist = self._params.get("NAPFollowDistance", return_default=True) if self._is_preap else None
    self.nap_adaptive_accel = self._params.get_bool("NAPAdaptiveAccel") if self._is_preap else False
    self._hypermile_on = read_hypermile_params(self._params) if self._is_preap else False
    self._hypermile_hill_climb = read_hypermile_hill_climb(self._params) if self._is_preap else False
    self._hill_pitch = 0.0
    self._map_speed_mode, self._map_speed_offset_kph, self._map_speed_lookahead, self._map_speed_accel = (
      read_map_speed_params(self._params) if self._is_preap else (0, 0.0, 0, 5)
    )
    self._follow_blend = FollowDistanceBlend()
    if self._is_preap:
      self._follow_blend.read_setpoints(self._params)
    self.active_nap_follow_dist = effective_nap_follow_dist(self._is_preap, self.nap_follow_dist)
    self.t_follow = get_T_FOLLOW(nap_follow_dist=self.active_nap_follow_dist)
    self._frame = 0
    # Last in-window radar lead, held through a brief status drop.
    self._lead_close_hold_d = None
    self._lead_close_hold_v = None
    self._lead_close_hold_a = None
    self._lead_close_hold_age = 0.0
    self._lead_y_rel = None
    self._corner_curvature = 0.0
    self._curve_preview = CurvePreview()
    self.curve_preview_a = PREVIEW_FREE_A_MS2
    # Curve-follow (NAPCurveFollow 0 off / 1 shadow / 2 active). Shadow computes
    # and logs only; the old preview and CurveMaxHold stay in force.
    self._cf_mode = read_curve_follow_mode(self._params) if self._is_preap else MODE_OFF
    self._curve_follow = CurveFollow()
    self.curve_follow_a = PREVIEW_FREE_A_MS2
    self._cf_trusted = False
    self._cf_faulted = False
    self._cf_cycle = 0
    # Sight-distance cap (NAPSightCap 0 off / 1 shadow). LOG ONLY: nothing reads it.
    self._sc_mode = read_sight_cap_mode(self._params) if self._is_preap else SIGHT_MODE_OFF
    self._sight_cap = SightCap()
    self._sc_faulted = False
    self._sc_last_log = -1e9
    self._sc_clock = 0.0
    self._turn_a_max = None

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_should_stop = False
    # Continuous lead follow: the one longitudinal controller for a lead on Pre-AP.
    self.unified_a_target = 0.0
    self._unified = UnifiedLeadController()
    self._gap_lock = GapLockLatch()
    self._gap_lock_enabled = False
    self._lead_leave = LeadLeavingEstimator()
    self.lead_leave_w = 0.0
    self._unified_v_cap_ms = 0.0
    self._unified_lead_id = None
    self._unified_following = False
    # A compute failure latches until process restart. It must not escape update().
    self._unified_faulted = False
    self._unified_fault_count = 0
    self._unified_fault_logged = False
    self._cycle_log = CycleTimeLog()

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  @staticmethod
  def parse_model(model_msg):
    if (len(model_msg.position.x) == ModelConstants.IDX_N and
      len(model_msg.velocity.x) == ModelConstants.IDX_N and
      len(model_msg.acceleration.x) == ModelConstants.IDX_N):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    if len(model_msg.meta.disengagePredictions.gasPressProbs) > 1:
      throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1]
    else:
      throttle_prob = 1.0
    return x, v, a, j, throttle_prob

  def _update_corner_state(self, sm) -> None:
    """Vehicle-model curvature for the lead path weighting and the curve ceiling."""
    curv = getattr(sm['controlsState'], "curvature", 0.0)
    self._corner_curvature = 0.0 if curv is None else float(curv)

  def _track_lead(self, sm) -> None:
    """Hold the last in-window radar lead through a brief status drop.

    The continuous controller reads this: a live lead, or the held one while
    radard flickers, with its aLead and lateral offset.
    """
    if not self._is_preap:
      return
    lead = sm['radarState'].leadOne
    d_cap, _, self._lead_close_hold_d, self._lead_close_hold_v, self._lead_close_hold_age = (
      resolve_lead_close_hold(
        lead.status, lead.dRel, lead.vLead,
        self._lead_close_hold_d, self._lead_close_hold_v, self._lead_close_hold_age,
        self.dt, model_prob=lead.modelProb, radar=lead.radar,
      )
    )
    if d_cap is not None:
      if lead.status and lead_close_should_cap(lead.dRel, lead.modelProb, lead.radar, active=True):
        self._lead_close_hold_a = float(lead.aLeadK)
    else:
      self._lead_close_hold_a = None
    held = self._lead_close_hold_d is not None and self._lead_close_hold_v is not None
    if bool(lead.status) and lead_close_should_cap(lead.dRel, lead.modelProb, lead.radar, active=False):
      self._lead_y_rel = float(lead.yRel)
    elif not held:
      self._lead_y_rel = None

  def update(self, sm):
    t0 = time.perf_counter()
    self._update(sm)
    window = self._cycle_log.record(time.perf_counter() - t0)
    if window is not None:
      cloudlog.info("longitudinal planner cycle: n=%d mean=%.3f ms max=%.3f ms", *window)

  def _update(self, sm):
    self._frame += 1
    if self._is_preap:
      # Stalk Follow Distance 1–7 must land on the next plan.
      self.nap_follow_dist = self._params.get("NAPFollowDistance", return_default=True)
      self._hypermile_on = read_hypermile_params(self._params)
      self._hypermile_hill_climb = read_hypermile_hill_climb(self._params)
      self.nap_adaptive_accel = self._params.get_bool("NAPAdaptiveAccel")
      self._map_speed_mode, self._map_speed_offset_kph, self._map_speed_lookahead, self._map_speed_accel = (
        read_map_speed_params(self._params)
      )
      self._follow_blend.read_setpoints(self._params)
      self._cf_mode = read_curve_follow_mode(self._params)
      self._sc_mode = read_sight_cap_mode(self._params)
      self._gap_lock_enabled = gap_lock_param_enabled(self._params)

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    # HUD MAX after card's map overlay (seed / sticky / Follow override).
    v_hud_ms = v_cruise
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    reset_state = reset_state or not v_cruise_initialized

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    accel_clip = [ACCEL_MIN, get_max_accel(v_ego)]
    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
    # Pre-AP: true cornering force (vehicle-model curvature) and the model
    # path ahead for the turn clip and the curve preview.
    curve_path = path_curvature(sm['modelV2']) if self._is_preap else None
    a_y_now = None
    a_y_ahead = None
    if self._is_preap:
      a_y_now = abs(float(getattr(sm['controlsState'], "curvature", 0.0) or 0.0)) * v_ego * v_ego
      if curve_path is not None:
        a_y_ahead = path_lat_accel_ahead(v_ego, curve_path[0], curve_path[2])
    accel_clip = limit_accel_in_turns(
      v_ego, steer_angle_without_offset, accel_clip, self.CP, a_y=a_y_now, a_y_ahead=a_y_ahead,
    )
    self._turn_a_max = float(accel_clip[1]) if self._is_preap else None

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])
      self._lead_close_hold_d = None
      self._lead_close_hold_v = None
      self._lead_close_hold_a = None
      self._lead_close_hold_age = 0.0
      self._lead_y_rel = None
      self._corner_curvature = 0.0
      self._unified.reset()
      self._unified_lead_id = None
      self._unified_following = False
      self._curve_preview.reset()

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    _, _, _, _, throttle_prob = self.parse_model(sm['modelV2'])
    # Don't clip at low speeds since throttle_prob doesn't account for creep
    # VDAS takes acceleration targets literally, so the model's negative coast
    # ceiling commands regen even when the MPC is trying to hold cruise speed.
    self.allow_throttle = self._is_preap or throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_clip[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [accel_clip[1], clipped_accel_coast])
      accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)

    if force_slow_decel:
      v_cruise = 0.0

    # OSM map speed: trust card HUD MAX (eased decreases, lag-corrected raises).
    # Do not min() with posted — that snapped when GPS entered a lower zone.
    # Roundabout funnel: cap cruise to ring speed and command kinematic −a
    # (not Lookahead comfort). aTarget stays ≤ 0 until the ring is exited.
    rb_hint = None
    a_rb_plan = 0.0
    if self._is_preap:
      try:
        md = sm['liveMapDataNAP']
      except Exception:
        md = None
      rb_hint = live_map_roundabout_hint(md)
      try:
        self._map_default = str(getattr(md, "source", "") or "") == MAP_SOURCE_DEFAULT
      except Exception:
        self._map_default = False
      # Oct 13: right blinker on the ring + leaving it ends the ring clamp
      # (same gate as card's MAX). The blinker alone does nothing.
      if not hasattr(self, "_rb_exit_gate"):
        self._rb_exit_gate = RingExitGate()
      try:
        rb_curv = float(getattr(sm['controlsState'], "curvature", 0.0))
        rb_right = bool(getattr(sm['carState'], "rightBlinker", False))
      except Exception:
        rb_curv, rb_right = 0.0, False
      if self._rb_exit_gate.update(rb_hint, rb_right, rb_curv, self.dt):
        rb_hint = None
    if (not force_slow_decel) and self._is_preap and self._map_speed_mode in (MODE_CAP, MODE_FOLLOW):
      v_cruise = cap_planner_v_cruise_ms(v_hud_ms, None, mode=self._map_speed_mode)
    if not force_slow_decel:
      v_cruise, v_hud_ms, a_rb_plan, rb_v = apply_roundabout_plan(
        v_ego, v_cruise, v_hud_ms, 0.0, rb_hint, self._map_speed_lookahead,
      )
    else:
      rb_v = None
    self._unified_v_cap_ms = float(v_hud_ms)

    self.active_nap_follow_dist = effective_nap_follow_dist(self._is_preap, self.nap_follow_dist)
    self.t_follow = get_T_FOLLOW(sm['selfdriveState'].personality, self.active_nap_follow_dist)
    long_engaged = self._is_preap and (not reset_state)
    has_lead_status = bool(sm['radarState'].leadOne.status)
    if self._is_preap:
      lead_tf = sm['radarState'].leadOne
      # Unset (255) is clamped to V_CRUISE_MAX above, which would look
      # like a highway MAX. 0 is not a city set speed. Ego is the fallback.
      follow_max_ms = v_hud_ms if v_cruise_initialized and float(v_hud_ms) > 0.0 else None
      t_blended, active_dist, _ = self._follow_blend.update(
        v_ego, self.dt,
        engaged=long_engaged,
        has_lead=has_lead_status,
        v_lead=float(lead_tf.vLead) if has_lead_status else None,
        v_cruise=follow_max_ms,
      )
      if active_dist in NAP_FOLLOW_DISTANCE_RANGE:
        self.active_nap_follow_dist = active_dist
        self.nap_follow_dist = active_dist
        self.t_follow = float(t_blended)

    # Pre-AP adaptive accel: only limit accel when the lead's obstacle-equivalent
    # distance is close. Above 1.5x the full cruise profile applies. Below 1.2x,
    # cap to follow limits so the command the lead-follow controller starts
    # from does not overshoot. Blend in between.
    if self.CP.carFingerprint == "TESLA_MODEL_S_PREAP" and self.nap_adaptive_accel and sm['radarState'].leadOne.status:
      follow_limit = _get_preap_follow_limit(v_ego)
      if follow_limit is not None:
        lead = sm['radarState'].leadOne
        cap_strength = get_preap_follow_cap_strength(v_ego, lead.dRel, lead.vLead, self.t_follow)
        if cap_strength > 0:
          blended = accel_clip[1] * (1.0 - cap_strength) + follow_limit * cap_strength
          accel_clip[1] = min(accel_clip[1], blended)

    self._update_corner_state(sm)
    self._track_lead(sm)

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    self.mpc.update(sm['radarState'], v_cruise, t_follow=self.t_follow)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                                                        action_t=action_t, vEgoStopping=self.CP.vEgoStopping)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    if sm['selfdriveState'].experimentalMode:
      output_a_target = min(output_a_target_e2e, output_a_target_mpc)
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc
      if output_a_target < output_a_target_mpc:
        self.mpc.source = LongitudinalPlanSource.e2e
    else:
      output_a_target = output_a_target_mpc
      self.output_should_stop = output_should_stop_mpc
    if self._is_preap:
      self._update_lead_leave(sm, float(v_ego), reset_state)

    # Map MAX is a set speed; MPC cruise_obstacle will not track it.
    # Climb at Accel 1–10 until the deadband, then hold so we do not surge
    # past MAX and map_track_decel below it. Brake is locked Accel 5.
    # A valid radar lead owns follow: do not replace ~0 / slight+ MPC with
    # map climb toward MAX (punch + overshoot + sluggish re-match).
    # map_track_decel when above MAX still mins in.
    # Hypermile Hill Climb (IMU pitch only — no maps-elevation lookahead)
    # then raises +a on a real uphill *under* MAX when there is no lead.
    # Crest / downhill ease only at or above MAX (not TRACK_TAPER).
    # Deadband leaves hold 0 — no +g·sin past MAX. It never writes vCruise / MAX.
    has_valid_lead = bool(sm['radarState'].leadOne.status)
    if len(sm['carControl'].orientationNED) == 3:
      hill_pitch = float(sm['carControl'].orientationNED[1])
    else:
      hill_pitch = 0.0
    if self._is_preap and self._map_speed_mode in (MODE_CAP, MODE_FOLLOW):
      if map_in_track_deadband(v_ego, v_hud_ms):
        if float(output_a_target) >= 0.0:
          output_a_target = 0.0
        pre_hill = float(output_a_target)
        output_a_target = apply_hill_climb(
          pitch_rad=hill_pitch,
          prev_pitch_rad=self._hill_pitch,
          v_ego_ms=v_ego,
          v_cruise_ms=v_hud_ms,
          a_cmd=float(output_a_target),
          in_deadband=True,
          hypermile_on=self._hypermile_on,
          hill_climb_on=self._hypermile_hill_climb,
        )
        if has_valid_lead and float(output_a_target) > pre_hill:
          output_a_target = pre_hill
      else:
        a_brake = map_track_decel_ms2(
          v_ego, v_hud_ms, map_brake_a_ms2(self._map_speed_lookahead),
        )
        if a_brake is not None:
          output_a_target = min(float(output_a_target), a_brake)
        else:
          a_up = map_track_accel_ms2(
            v_ego, v_hud_ms, map_accel_a_ms2(self._map_speed_lookahead, self._map_speed_accel),
            accel_level=self._map_speed_accel,
          )
          if map_climb_replaces_mpc(a_up, output_a_target, has_valid_lead):
            # min() alone never created climb (MPC holds ~0). Command Accel 1–10
            # toward MAX only when no radar lead is following.
            output_a_target = a_up
        pre_hill = float(output_a_target)
        output_a_target = apply_hill_climb(
          pitch_rad=hill_pitch,
          prev_pitch_rad=self._hill_pitch,
          v_ego_ms=v_ego,
          v_cruise_ms=v_hud_ms,
          a_cmd=float(output_a_target),
          in_deadband=False,
          hypermile_on=self._hypermile_on,
          hill_climb_on=self._hypermile_hill_climb,
        )
        if has_valid_lead and float(output_a_target) > pre_hill:
          # Do not add +g·sin punch toward MAX while a lead constrains.
          output_a_target = pre_hill
    self._hill_pitch = hill_pitch

    # Maps-off / display still eases into an RB (card may not have dropped MAX).
    # Kinematic a from apply_roundabout_plan — not map_track_decel comfort.
    # Oct 11: jerk-limit the onset of the ring −a (no brick-wall step).
    if rb_v is not None:
      a_jl_prev = float(getattr(self, "_rb_a_jl", 0.0))
      a_rb_jl = max(float(a_rb_plan), a_jl_prev - RB_DECEL_JERK_MS3 * self.dt)
      self._rb_a_jl = a_rb_jl
      output_a_target = min(float(output_a_target), a_rb_jl)
    else:
      self._rb_a_jl = 0.0

    # Curve preview: accel ceiling from the model path (0–5 s) that slows
    # ahead of a bend, finished ~1.5 s before the tight point, and holds
    # back +a just under a bend's comfort speed. min() only: never raises
    # the command. A lead, if any, is then followed by the continuous
    # controller, which takes this ceiling as its own map/curve term.
    self.curve_preview_a = PREVIEW_FREE_A_MS2
    if self._is_preap:
      if reset_state or curve_path is None:
        self._curve_preview.reset()
      else:
        a_prev_raw = curve_preview_accel(v_ego, *curve_path)
        self.curve_preview_a = float(self._curve_preview.update(v_ego, a_prev_raw, self.dt))
      # Curve-follow: computed and logged in shadow (1) and active (2). Only
      # active replaces the old preview's ceiling; the lead law still takes it
      # through the same a_map slot, so nothing else changes.
      old_preview_a = float(self.curve_preview_a)
      self._update_curve_follow(sm, float(v_ego), curve_path, old_preview_a, rb_v is not None,
                                float(output_a_target))
      self._update_sight_cap(sm, float(v_ego), curve_path, rb_v is not None, rb_hint)
      output_a_target = min(float(output_a_target), self.curve_preview_a)

    for idx in range(2):
      accel_clip[idx] = np.clip(accel_clip[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
    self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
    self.prev_accel_clip = accel_clip
    self._apply_lead_follow(sm, float(v_ego))

  def _update_curve_follow(self, sm, v_ego: float, curve_path, old_preview_a: float, rb_active: bool,
                           a_target: float) -> None:
    """Curve-follow step. Never raises; a fault latches it off (old preview stays)."""
    self._cf_trusted = False
    self.curve_follow_a = PREVIEW_FREE_A_MS2
    if self._cf_mode == MODE_OFF or self._cf_faulted:
      return
    try:
      model = sm['modelV2']
      meta = model.meta
      ts, ds, ks = curve_path if curve_path is not None else (None, None, None)
      y_std = list(model.position.yStd)
      y_std = y_std if (ts is not None and len(y_std) == len(ts)) else None
      car_state = sm['carState']
      blinker = bool(car_state.leftBlinker or car_state.rightBlinker)
      lane_change = str(meta.laneChangeState) != "off"
      frame_drop = float(model.frameDropPerc)
      a_cf = float(self._curve_follow.step(
        dt=self.dt, v_ego=v_ego, ts=ts, ds=ds, ks=ks, y_std=y_std, lane_change=lane_change,
        blinker=blinker, frame_drop_pct=frame_drop, rb_active=rb_active, valid=curve_path is not None,
      ))
      self.curve_follow_a = a_cf
      self._cf_trusted = bool(self._curve_follow.trusted)
      if self._cf_mode == MODE_ACTIVE:
        self.curve_preview_a = a_cf
      self._cf_cycle += 1
      if v_ego >= 2.0 and ts is not None and self._cf_cycle % LOG_PERIOD_CYCLES == 0:
        arrays = (path_is_relevant(ks, a_cf, old_preview_a, self._curve_follow.conf)
                  or self._cf_cycle % LOG_QUIET_PERIOD_CYCLES == 0)
        self._log_curve_follow(sm, model, v_ego, a_target, old_preview_a, ts, ds, ks, y_std, blinker,
                               lane_change, frame_drop, rb_active, arrays)
    except Exception:
      self._cf_faulted = True
      self._cf_trusted = False
      self.curve_follow_a = PREVIEW_FREE_A_MS2
      cloudlog.exception("curve_follow fault: old preview stays in force")

  def _update_sight_cap(self, sm, v_ego: float, curve_path, rb_live: bool, rb_hint) -> None:
    """Sight-distance cap in shadow: compute and log. Never touches the command."""
    if self._sc_mode == SIGHT_MODE_OFF or self._sc_faulted:
      return
    try:
      model = sm['modelV2']
      self._sc_clock += self.dt
      t_s = self._sc_clock
      sight = model_sight(model)
      radar_state = sm['radarState']
      lead_one = radar_state.leadOne
      live = bool(lead_one.status)
      path_x = path_y = None
      lat = None
      if live:
        try:
          path_x, path_y = model_path_xy(model)
          lat = path_lateral_m(lead_one, path_x, path_y)
        except Exception:
          lat = None
      lead = classify_lead(live, lat, float(lead_one.vLead) if live else None, float(lead_one.dRel) if live else None)
      others = []
      lead_two = radar_state.leadTwo
      if bool(lead_two.status):
        try:
          lat2 = path_lateral_m(lead_two, *model_path_xy(model))
        except Exception:
          lat2 = None
        others.append(classify_lead(True, lat2, float(lead_two.vLead), float(lead_two.dRel)))
      ks = curve_path[2] if curve_path is not None else None
      sc = self._sight_cap
      sc.update(t=t_s, dt=self.dt, v_ego=v_ego, sight_m=sight, ks=ks,
                conf_red=str(model.confidence) == "red", lead=lead, rb_live=rb_live,
                rb_ahead=bool(rb_hint is not None and getattr(rb_hint, "approaching", False)), others=others)
      if sc.fired_edge or sc.suppressed_edge or (sc.would_fire and t_s - self._sc_last_log >= SIGHT_LOG_PERIOD_S):
        self._sc_last_log = t_s
        self._log_sight_cap(sm, model, t_s, v_ego, lead, live, others)
    except Exception:
      self._sc_faulted = True
      cloudlog.exception("sight_cap fault: shadow logging stopped")

  def _log_sight_cap(self, sm, model, t_s: float, v_ego: float, lead, live: bool, others) -> None:
    """Would-fire record through errorLogMessage (qlog), like curvefollow. No capnp fields."""
    try:
      sc = self._sight_cap
      cs = sm['carState']
      two = sm['radarState'].leadTwo
      two_info = None
      if bool(two.status):
        try:
          lat2 = path_lateral_m(two, *model_path_xy(model))
        except Exception:
          lat2 = None
        t2 = classify_lead(True, lat2, float(two.vLead), float(two.dRel))
        two_info = [t2.lane, round(float(t2.v), 1), round(float(t2.d), 1)]
      crest, z2 = crest_ahead(list(model.position.x), list(model.position.z))
      pitch = None
      try:
        if len(sm['carControl'].orientationNED) == 3:
          pitch = round(float(sm['carControl'].orientationNED[1]), 4)
      except Exception:
        pitch = None
      night = None
      sun_el = None
      try:
        md = sm['liveMapDataNAP']
        lat_deg, lon_deg = float(md.latitude), float(md.longitude)
        if lat_deg != 0.0 or lon_deg != 0.0:
          from openpilot.selfdrive.controls.lib.lat_low_visibility import solar_position
          # Wall clock for the sun position only (log field; never control).
          sun_el, _az = solar_position(lat_deg, lon_deg, time.time())  # noqa: TID251
          night = bool(sun_el < -6.0)
          sun_el = round(float(sun_el), 1)
      except Exception:
        night = None
      cf_a = float(self.curve_follow_a)
      try:
        t_log = float(sm.logMonoTime['modelV2']) * 1e-9
      except Exception:
        t_log = t_s
      rec = {
        "t": round(t_log, 2), "edge": int(sc.fired_edge), "fire": int(sc.would_fire), "why": sc.reason, "cue": sc.cue,
        "v": round(v_ego, 2), "sd": None if sc.sight is None else round(sc.sight, 1),
        "sb": None if sc.base is None else round(sc.base, 1),
        "vc": None if sc.v_cap is None else round(sc.v_cap, 2), "ac": round(sc.a_cap, 3),
        "cf": round(cf_a, 3), "cmb": round(min(cf_a, sc.a_cap), 3),
        "lead": [int(live), lead.lane, None if lead.v is None else round(lead.v, 1),
                 None if lead.d is None else round(lead.d, 1), None if lead.lat is None else round(lead.lat, 2)],
        "lead2": two_info, "veh": next((k for k in (lead.kind, *(o.kind for o in others)) if k), ""),
        "pitch": pitch, "crest": int(crest), "z2": z2,
        "night": night, "sun": sun_el,
        "llp": [round(float(p), 2) for p in list(model.laneLineProbs)[:4]],
        "bl": int(bool(cs.leftBlinker)) + 2 * int(bool(cs.rightBlinker)),
        "cn": str(model.confidence),
        "lv": int(bool(getattr(sm['controlsState'], 'lowVisibility', False))),
      }
      cloudlog.error(sight_log_line(rec))
    except Exception:
      pass  # a sight-cap log failure must never reach the plan

  def _log_curve_follow(self, sm, model, v_ego, a_target, old_preview_a, ts, ds, ks, y_std, blinker,
                        lane_change, frame_drop, rb_active, arrays) -> None:
    """2 Hz JSON line through errorLogMessage, which qlog keeps (same route the roundabout summary uses)."""
    try:
      desire = list(model.meta.desireState)
      d_idx = max(range(len(desire)), key=lambda i: desire[i]) if desire else -1
      meta = {
        "bl": int(bool(sm['carState'].leftBlinker)) + 2 * int(bool(sm['carState'].rightBlinker)),
        "lc": str(model.meta.laneChangeState), "ds": [d_idx, round(float(desire[d_idx]), 2) if desire else 0.0],
        "df": round(frame_drop, 1), "cn": str(model.confidence), "rb": int(rb_active),
        "llp": [round(float(p), 2) for p in list(model.laneLineProbs)[:4]],
        "res": [round(float(p), 2) for p in list(model.roadEdgeStds)[:2]],
      }
      t_s = float(sm.logMonoTime['modelV2']) * 1e-9
      cloudlog.error(format_log_line(
        mode=self._cf_mode, t=t_s, v=v_ego, a_ego=float(sm['carState'].aEgo),
        v_cruise_kph=float(sm['carState'].vCruise), a_target=a_target, cf=self._curve_follow,
        old_a=old_preview_a, ts=ts, ds=ds, ks=ks, y_std=y_std, meta=meta, arrays=arrays,
      ))
    except Exception:
      pass  # logging must never touch the command

  def _update_lead_leave(self, sm, v_ego: float, reset_state: bool) -> None:
    """Leaving-path weight for radard's leadOne (feeds the lead-follow controller)."""
    radar_state = sm['radarState']
    lead = radar_state.leadOne
    live = bool(lead.status)
    held = (not live) and self._lead_close_hold_d is not None and self._lead_close_hold_v is not None
    suppress = bool(reset_state or self.fcw or self.mpc.crash_cnt > 0 or self.output_should_stop
                    or bool(sm['controlsState'].forceDecel))
    path_lat = None
    two_lat = None
    if live:
      try:
        path_x, path_y = model_path_xy(sm['modelV2'])
        path_lat = path_lateral_m(lead, path_x, path_y)
        two = radar_state.leadTwo
        if bool(two.status) and int(getattr(two, 'radarTrackId', -1)) != int(getattr(lead, 'radarTrackId', -1)):
          two_lat = path_lateral_m(two, path_x, path_y)
          if two_lat is None:
            two_lat = 0.0
      except Exception:
        path_lat = None
        two_lat = 0.0
    self.lead_leave_w = float(self._lead_leave.update(
      dt=self.dt,
      present=live or held,
      held=held,
      lead_id=int(getattr(lead, 'radarTrackId', 0) or 0) if live else None,
      path_lat=path_lat,
      gap=float(lead.dRel) if live else 0.0,
      v_ego=v_ego,
      v_lead=float(lead.vLead) if live else None,
      a_lead=float(lead.aLeadK) if live else 0.0,
      model_prob=float(lead.modelProb) if live else None,
      lead_two_path_lat=two_lat,
      suppress=suppress,
    ))

  def _note_unified_fault(self) -> None:
    """Latch the controller off until this process restarts.

    cloudlog.exception runs for the first failure only. While latched the
    published command is the MPC command with the map, curve and roundabout
    ceilings (the lead-free path), which still brakes for the lead.
    """
    self._unified_fault_count += 1
    self._unified_faulted = True
    if self._unified_fault_logged:
      return
    self._unified_fault_logged = True
    cloudlog.exception(
      "unified lead-follow compute failed (count=%d); MPC command stays in force",
      self._unified_fault_count,
    )

  def _refresh_gap_lock(self, sm, lead) -> None:
    """Latch or clear. A throw here must not latch the unified fault."""
    cs = sm["carState"]
    long_on = False
    try:
      long_on = (cs.enableLongControl is True) and (cs.cruiseState.enabled is True)
    except Exception:
      long_on = False
    try:
      seq = int(cs.gapLockArmSeq)
    except (TypeError, ValueError, AttributeError):
      seq = 0
    holding = False
    try:
      holding = bool(cs.gapLockHold)
    except (TypeError, ValueError, AttributeError):
      holding = False
    status = False
    radar = False
    track_id = -1
    d_rel = None
    y_rel = None
    try:
      status = bool(lead.status)
      radar = bool(lead.radar)
      track_id = int(lead.radarTrackId)
      if status:
        d_rel = float(lead.dRel)
    except (TypeError, ValueError, AttributeError):
      status = False
      radar = False
      track_id = -1
      d_rel = None
    if status:
      try:
        y_rel = float(lead.yRel)
      except (TypeError, ValueError, AttributeError):
        y_rel = None
    self._gap_lock.update(
      self.dt,
      enabled=self._gap_lock_enabled is True,
      long_on=long_on,
      seq=seq,
      status=status,
      radar=radar,
      track_id=track_id,
      d_rel=d_rel,
      stalk_exit=stalk_follow_exit(cs),
      holding=holding,
      y_rel=y_rel,
    )

  def _gap_set_override(self, lead, live: bool, held: bool):
    """Locked meters, or the in-hold median, for this radar id.

    The hold publishes the running median before the latch so the ease
    toward the time gap stops as soon as the stalk hold starts.
    """
    try:
      gap_m = self._gap_lock.gap_m
      track = self._gap_lock.track_id
      if gap_m is None:
        gap_m = self._gap_lock.preview_m
        track = self._gap_lock.preview_track
      if gap_m is None or track is None:
        return None
      if live:
        # A brief vision-only dip of the locked track keeps the meters.
        # The in-hold preview still wants a radar id.
        if self._gap_lock.gap_m is None and lead.radar is not True:
          return None
        if int(lead.radarTrackId) != int(track):
          return None
        return float(gap_m)
      if held:
        return float(gap_m)
    except (TypeError, ValueError, AttributeError):
      return None
    return None

  def _apply_lead_follow(self, sm, v_ego: float) -> None:
    """The one lead-follow controller (Pre-AP).

    Runs every frame. With a live or held lead its command replaces the
    lead-free command in output_a_target. With none, the lead-free command
    (MPC, map, hill, roundabout, curve preview) stands. FCW, should-stop and
    force-decel keep that command if it is deeper. A compute exception
    latches the controller off and leaves the lead-free command in force.
    """
    if not self._is_preap:
      self.unified_a_target = 0.0
      return

    lead = sm["radarState"].leadOne
    try:
      self._refresh_gap_lock(sm, lead)
    except Exception:
      pass
    live = bool(lead.status) and lead_close_should_cap(
      lead.dRel, lead.modelProb, lead.radar, active=False,
    )
    held = self._lead_close_hold_d is not None and self._lead_close_hold_v is not None
    if live:
      gap = float(lead.dRel)
      v_lead = float(lead.vLead)
      a_lead = float(lead.aLeadK)
      y_rel = float(getattr(lead, "yRel", 0.0) or 0.0)
      self._unified_lead_id = ("radar", int(getattr(lead, "radarTrackId", 0) or 0))
      lead_id = self._unified_lead_id
    elif held:
      gap = float(self._lead_close_hold_d)
      v_lead = float(self._lead_close_hold_v)
      a_lead = 0.0 if self._lead_close_hold_a is None else float(self._lead_close_hold_a)
      y_rel = 0.0 if self._lead_y_rel is None else float(self._lead_y_rel)
      lead_id = self._unified_lead_id if self._unified_lead_id is not None else ("hold",)
    else:
      gap = None
      v_lead = 0.0
      a_lead = 0.0
      y_rel = 0.0
      lead_id = None

    v_lat = 0.0
    long_on = True
    if live:
      try:
        v_lat = float(lead.vLat)
      except (TypeError, ValueError, AttributeError):
        v_lat = 0.0
    try:
      cs = sm["carState"]
      long_on = (cs.enableLongControl is True) and (cs.cruiseState.enabled is True)
    except Exception:
      long_on = True
    map_cap = float(self._unified_v_cap_ms)
    # Oct 13: a drop to an untagged state default follows down at <= 0.5 m/s².
    if getattr(self, "_map_default", False) and map_cap > 0.0:
      self._default_cap_ms = default_drop_cap_ms(map_cap, getattr(self, "_default_cap_ms", None), v_ego, self.dt)
      map_cap = float(self._default_cap_ms)
    else:
      self._default_cap_ms = None
    v_cap = map_cap
    # True cornering (vehicle-model curvature) with the speed-dependent
    # lateral target, not the steer model (which reads ~12% high at speed).
    v_curve = curve_speed_for_curvature(float(self._corner_curvature))
    # Active curve-follow already covers the present curvature (its point 0);
    # the reactive cap is the dead-camera fallback, used when it is not trusted.
    cf_owns_curve = self._cf_mode == MODE_ACTIVE and self._cf_trusted and not self._cf_faulted
    curve_cap = None
    if v_curve is not None and v_ego >= 5.0 and not cf_owns_curve:
      v_cap = min(v_cap, float(v_curve))
      curve_cap = float(v_curve)
    # Locked meters are the setpoint, including above MAX. Map-speed
    # braking back to the cap is skipped while the lock is on. A curve
    # cap is passed separately so a bend still slows the car.
    override = self._gap_set_override(lead, live, held)
    a_map = None
    if override is None and self._map_speed_mode in (MODE_CAP, MODE_FOLLOW):
      a_map = map_track_decel_ms2(
        v_ego, map_cap, map_brake_a_ms2(self._map_speed_lookahead),
      )
    if self.curve_preview_a < PREVIEW_FREE_A_MS2:
      a_map = self.curve_preview_a if a_map is None else min(float(a_map), self.curve_preview_a)
    v_curve_cap = curve_cap if override is not None else None
    a_max_u = float(get_max_accel(v_ego))
    if self._turn_a_max is not None:
      a_max_u = min(a_max_u, float(self._turn_a_max))
    # Lead weighting: lateral offset from the model's planned path (bends
    # stay on-path, adjacent lanes do not) and reading quality.
    path_lat = None
    model_prob = None
    radar = None
    if live:
      try:
        path_lat = path_lateral_m(lead, *model_path_xy(sm["modelV2"]))
        model_prob = float(lead.modelProb)
        radar = bool(lead.radar)
      except Exception:
        path_lat = model_prob = radar = None
    existing = float(self.output_a_target)
    seed = existing
    command = None
    if not self._unified_faulted:
      try:
        if gap is not None and not self._unified_following:
          # First lead after the lead-free command. That command knows nothing
          # of this lead (cruise or map-climb +a), so it may not accelerate
          # harder than the law itself asks. A far closing lead is never
          # rematched with +a.
          wanted = unified_follow_desired(
            gap, v_ego, v_lead, a_lead, float(self.t_follow),
            v_ceiling=v_cap, a_map=a_map, y_rel=y_rel,
            curvature=float(self._corner_curvature), path_lat=path_lat,
            leave_w=float(self.lead_leave_w), model_prob=model_prob, radar=radar,
            gap_set_override_m=override, v_curve_cap=v_curve_cap,
          )
          wanted = min(wanted, a_max_u)
          if wanted >= 0.0:
            seed = min(existing, max(wanted, 0.0))
          else:
            # The law wants a brake the lead-free command does not: start up to
            # SEED_BRAKE_STEP_MS2 firmer than it (a hard close must not wait on
            # the jerk-limited build-up), never past what the law itself asks.
            seed = min(existing, max(wanted, min(existing, 0.0) - SEED_BRAKE_STEP_MS2))
        command = float(self._unified.step(
          dt=self.dt,
          present=gap is not None,
          gap=0.0 if gap is None else gap,
          v_ego=v_ego,
          v_lead=v_lead,
          a_lead=a_lead,
          t_follow=float(self.t_follow),
          lead_id=lead_id,
          seed_a=seed,
          v_ceiling=v_cap,
          a_map=a_map,
          y_rel=y_rel,
          curvature=float(self._corner_curvature),
          a_max=a_max_u,
          path_lat=path_lat,
          model_prob=model_prob,
          radar=radar,
          leave_w=float(self.lead_leave_w),
          gap_set_override_m=override,
          v_curve_cap=v_curve_cap,
          v_lat=v_lat,
          long_on=long_on,
        ))
      except Exception:
        self._note_unified_fault()

    if self._unified_faulted or command is None:
      self.unified_a_target = 0.0
      self._unified_following = False
      return
    self._unified_following = gap is not None
    self.unified_a_target = command
    if gap is None:
      return
    if self.fcw or self.output_should_stop or bool(sm["controlsState"].forceDecel):
      command = min(command, existing)
    self.output_a_target = float(command)

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState', 'radarState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    # Chevron / HUD lead: live status, or the planner still owns a held
    # in-window lead through a brief flicker.
    live_lead = bool(sm['radarState'].leadOne.status)
    if self._is_preap and live_lead:
      live_lead = lead_approach_track_ok(
        sm['radarState'].leadOne.dRel,
        sm['radarState'].leadOne.modelProb,
        sm['radarState'].leadOne.radar,
        active=False,
      )
    longitudinalPlan.hasLead = live_lead or (
      self._is_preap and self._lead_close_hold_d is not None
    )
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.unifiedATarget = float(self.unified_a_target)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)
    longitudinalPlan.napFollowDistance = self.active_nap_follow_dist or 0
    longitudinalPlan.tFollow = self.t_follow
    longitudinalPlan.gapLockM = float(self._gap_lock.gap_m or 0.0)
    longitudinalPlan.gapLockEvent = int(self._gap_lock.hud) & 0xFF

    pm.send('longitudinalPlan', plan_send)
