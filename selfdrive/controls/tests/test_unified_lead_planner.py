"""Planner integration for the unified lead-follow controller (the sole path).

On Pre-AP, a live or held lead hands the published accel to the continuous
controller. Without a lead the lead-free command (MPC, map, curve, roundabout)
stays in force. A compute exception is logged once, latches the controller off
for the rest of the process, and the lead-free command (which still brakes for
the lead) is published. Other cars never run it.
"""
import math
import time

import numpy as np
import pytest

from cereal import car, log, messaging
from opendbc.car.tesla.preap.constants import PEDAL_LONG_K_BP, PEDAL_LONG_KI_V, PEDAL_LONG_KP_V
from openpilot.common.constants import CV
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalPlanSource, T_IDXS
from openpilot.selfdrive.controls.lib.longitudinal_planner import SEED_BRAKE_STEP_MS2, CycleTimeLog, LongitudinalPlanner
from openpilot.selfdrive.modeld.constants import ModelConstants


class _PlannerInputs(dict):
  logMonoTime = {"modelV2": 0}

  @staticmethod
  def all_checks(service_list):
    return set(service_list) == {"carState", "controlsState", "selfdriveState", "radarState"}


class _CapturingPubMaster:
  def send(self, service, message):
    assert service == "longitudinalPlan"
    self.message = message


class _MutablePlannerParams:
  """Same unknown-key behavior as the following-suite double.

  An unknown key raises, so the planner may only read keys it knows.
  """

  def __init__(self, nap_follow_dist, adaptive_accel=False, map_speed_accel=5):
    self.nap_follow_dist = nap_follow_dist
    self.adaptive_accel = adaptive_accel
    self.map_speed_accel = map_speed_accel
    self.city = nap_follow_dist
    self.hwy = nap_follow_dist
    self.migrated = True
    self.curve_follow = 1
    self.sight_cap = 1

  def __bool__(self):
    return False

  def get(self, key, return_default=False):
    assert return_default
    if key == "NAPFollowDistance":
      return self.nap_follow_dist
    if key == "NAPFollowDistanceCity":
      return self.city
    if key == "NAPFollowDistanceHwy":
      return self.hwy
    if key == "NAPMapSpeedAccel":
      return self.map_speed_accel
    if key == "NAPMapSpeedMode":
      return 0
    if key == "NAPMapSpeedOffsetMph":
      return 0
    if key == "NAPMapSpeedLookahead":
      return 2
    if key == "NAPCurveFollow":
      return self.curve_follow
    if key == "NAPSightCap":
      return self.sight_cap
    raise AssertionError(key)

  def get_bool(self, key):
    if key == "NAPAdaptiveAccel":
      return self.adaptive_accel
    if key == "NAPHypermile":
      return False
    if key == "NAPHypermileHillClimb":
      return True
    if key == "NAPFollowDistanceSplitMigrated":
      return self.migrated
    raise AssertionError(key)

  def put_bool(self, key, value):
    if key == "NAPFollowDistanceSplitMigrated":
      self.migrated = bool(value)
    elif key == "NAPAdaptiveAccel":
      self.adaptive_accel = bool(value)


class _ConstantAccelerationMpc:
  def __init__(self, speed_mps, acceleration_mps2):
    self.v_solution = speed_mps + acceleration_mps2 * T_IDXS
    self.a_solution = np.full(len(T_IDXS), acceleration_mps2)
    self.j_solution = np.zeros(len(T_IDXS) - 1)
    self.params = np.zeros((len(T_IDXS), 6))
    self.source = LongitudinalPlanSource.cruise
    self.crash_cnt = 0
    self.solve_time = 0.0

  @staticmethod
  def set_weights(prev_accel_constraint, personality):
    pass

  @staticmethod
  def set_cur_state(speed_mps, acceleration_mps2):
    pass

  def update(self, radar_state, cruise_speed_mps, t_follow):
    self.params[:, 4] = t_follow


def _make_preap_params():
  params = car.CarParams.new_message()
  params.brand = "tesla"
  params.carFingerprint = "TESLA_MODEL_S_PREAP"
  params.openpilotLongitudinalControl = True
  params.pcmCruise = False
  params.steerRatio = 15.75
  params.wheelbase = 2.959
  params.longitudinalTuning.kpBP = PEDAL_LONG_K_BP
  params.longitudinalTuning.kpV = PEDAL_LONG_KP_V
  params.longitudinalTuning.kiBP = PEDAL_LONG_K_BP
  params.longitudinalTuning.kiV = PEDAL_LONG_KI_V
  params.longitudinalTuning.kf = 1.0
  params.vEgoStarting = 0.1
  return params


def _set_v_cruise_ms(inputs, v_cruise_ms):
  inputs["carState"].vCruise = float(v_cruise_ms) * CV.MS_TO_KPH


def _make_planner_inputs(speed_mps):
  radar = messaging.new_message("radarState").radarState
  controls = messaging.new_message("controlsState").controlsState
  selfdrive = messaging.new_message("selfdriveState").selfdriveState
  car_state = messaging.new_message("carState").carState
  car_control = messaging.new_message("carControl").carControl
  live_parameters = messaging.new_message("liveParameters").liveParameters
  model = messaging.new_message("modelV2").modelV2

  controls.longControlState = LongCtrlState.pid
  selfdrive.personality = log.LongitudinalPersonality.standard
  car_state.vEgo = speed_mps
  car_state.vCruise = speed_mps * 3.6
  car_control.orientationNED = [0.0, 0.0, 0.0]

  model.position.x = (speed_mps * np.array(ModelConstants.T_IDXS)).tolist()
  model.velocity.x = (speed_mps * np.ones_like(ModelConstants.T_IDXS)).tolist()
  model.acceleration.x = np.zeros_like(ModelConstants.T_IDXS).tolist()
  model.meta.disengagePredictions.gasPressProbs = [1.0] * 6

  return _PlannerInputs({
    "radarState": radar,
    "controlsState": controls,
    "selfdriveState": selfdrive,
    "carState": car_state,
    "carControl": car_control,
    "liveParameters": live_parameters,
    "modelV2": model,
  })


def _planner(v_ego, *, nap_follow_dist=2, accel=-3.5):
  params = _MutablePlannerParams(nap_follow_dist=nap_follow_dist)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=accel)
  planner.prev_accel_clip = [-3.5, 2.0]
  inputs = _make_planner_inputs(v_ego)
  _set_v_cruise_ms(inputs, max(v_ego + 8.0, 40.0))
  return planner, inputs, params


def _own_lead(inputs, d_rel, v_lead, a_lead=0.0, y_rel=0.0, track=1):
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = float(d_rel)
  lead.vLead = float(v_lead)
  lead.aLeadK = float(a_lead)
  lead.yRel = float(y_rel)
  lead.modelProb = 1.0
  lead.radar = True
  lead.radarTrackId = int(track)
  return lead


def _run(planner, inputs, n, mpc_accel=None):
  if mpc_accel is not None:
    crash = float(getattr(planner.mpc, "crash_cnt", 0) or 0)
    planner.mpc = _ConstantAccelerationMpc(inputs["carState"].vEgo, acceleration_mps2=mpc_accel)
    planner.mpc.crash_cnt = crash
  for _ in range(n):
    planner.update(inputs)
  return float(planner.output_a_target)


def _guarded_updates(planner, inputs, n):
  """Turn a propagating shadow exception into an assertion failure.

  The mutation gate only counts AssertionError as a killed mutant. If the
  planner guard is removed, update() must fail these tests that way.
  """
  escaped = []
  for _ in range(n):
    try:
      planner.update(inputs)
    except Exception as exc:
      escaped.append(f"{type(exc).__name__}: {exc}")
  return escaped


def _spy_cloudlog_exception(monkeypatch):
  calls = []

  def _record(*args, **kwargs):
    calls.append(args)

  monkeypatch.setattr(cloudlog, "exception", _record)
  return calls


def _capture_lead_free(monkeypatch, planner):
  """Lead-free command for each frame, before the lead-follow step runs."""
  captured = []
  original = planner._apply_lead_follow

  def _wrap(sm, v_ego):
    captured.append(float(planner.output_a_target))
    return original(sm, v_ego)

  monkeypatch.setattr(planner, "_apply_lead_follow", _wrap)
  return captured


def test_sole_path_follows_the_continuous_command_for_sep23():
  """23:31 geometry: firm match to a braking lead, not the raw MPC -3.5."""
  v_ego = 30.0
  planner, inputs, _ = _planner(v_ego, accel=-3.5)
  _own_lead(inputs, 35.0, v_ego - 1.13, a_lead=-1.18)
  _run(planner, inputs, 80)
  assert float(planner.output_a_target) == pytest.approx(float(planner.unified_a_target), abs=1e-6)
  assert -2.2 < float(planner.output_a_target) <= -0.90
  publisher = _CapturingPubMaster()
  planner.publish(inputs, publisher)
  assert publisher.message.longitudinalPlan.unifiedATarget == pytest.approx(planner.unified_a_target)


def test_first_far_closing_lead_is_never_rematched_with_plus_a():
  """The lead-free command knows nothing of a new lead: cruise +a must not pass through."""
  v_ego = 25.0
  planner, inputs, _ = _planner(v_ego, nap_follow_dist=4, accel=1.5)
  _own_lead(inputs, 160.0, v_ego - 1.6, a_lead=0.0)
  planner.update(inputs)
  assert planner._unified_following is True
  assert float(planner.output_a_target) <= 0.05
  assert _run(planner, inputs, 15, mpc_accel=1.5) <= 0.05

  # A far same-speed lead (the law wants +a): the cruise +1.5 is bounded by what
  # the law itself asks (about +0.8 here), never passed through.
  same, same_in, _ = _planner(v_ego, nap_follow_dist=4, accel=1.5)
  _own_lead(same_in, 160.0, v_ego, a_lead=0.0)
  same.update(same_in)
  assert 0.0 < float(same.output_a_target) <= 0.9


def test_first_hard_close_frame_starts_firmer_than_the_lead_free_command():
  """07:55 Sep 20 class: the law asks for far more brake than the lead-free command.

  The new lead starts up to SEED_BRAKE_STEP_MS2 firmer than that command
  (not waiting on the jerk-limited build-up) and never passes the law's own ask.
  """
  v_ego = 18.5
  planner, inputs, _ = _planner(v_ego, accel=-0.5)
  _own_lead(inputs, 25.0, v_ego - 4.44, a_lead=-1.04)
  planner.update(inputs)
  first = float(planner.output_a_target)
  assert first <= -0.5 - 0.5 * SEED_BRAKE_STEP_MS2
  assert first >= -0.5 - SEED_BRAKE_STEP_MS2 - 0.8
  assert _run(planner, inputs, 40, mpc_accel=-0.5) <= -2.0


def test_no_lead_keeps_the_lead_free_command(monkeypatch):
  v_ego = 20.0
  planner, inputs, _ = _planner(v_ego, accel=0.40)
  captured = _capture_lead_free(monkeypatch, planner)
  _run(planner, inputs, 10)
  assert inputs["radarState"].leadOne.status is False
  assert float(planner.output_a_target) == pytest.approx(captured[-1], abs=1e-9)
  assert float(planner.output_a_target) > 0.1
  assert float(planner.unified_a_target) == pytest.approx(0.0, abs=1e-9)
  assert planner._unified_following is False


def test_faster_lead_at_max_does_not_overrun():
  v_max = 60.0 * CV.MPH_TO_MS
  v_lead = v_max + 3.0 * CV.MPH_TO_MS
  planner, inputs, _ = _planner(v_max, nap_follow_dist=4, accel=1.2)
  _set_v_cruise_ms(inputs, v_max)
  _own_lead(inputs, 80.0, v_lead, a_lead=0.2)
  _run(planner, inputs, 30, mpc_accel=1.2)
  assert float(planner.output_a_target) <= 0.05


def test_fcw_is_not_weaker_than_the_lead_free_command(monkeypatch):
  v_ego = 30.0
  planner, inputs, _ = _planner(v_ego, accel=-3.5)
  captured = _capture_lead_free(monkeypatch, planner)
  _own_lead(inputs, 35.0, v_ego - 1.13, a_lead=-1.18)
  planner.mpc.crash_cnt = 3
  _run(planner, inputs, 8, mpc_accel=-3.5)
  assert planner.fcw is True
  assert float(planner.output_a_target) <= captured[-1] + 1e-6


def test_curve_ceiling_brakes_with_a_lead_present():
  v_ego = 25.0
  curve, inputs, _ = _planner(v_ego, accel=0.0)
  straight, straight_inputs, _ = _planner(v_ego, accel=0.0)
  _own_lead(inputs, 50.0, v_ego + 1.0, a_lead=0.0)
  _own_lead(straight_inputs, 50.0, v_ego + 1.0, a_lead=0.0)
  inputs["carState"].steeringAngleDeg = 15.0
  # The curve ceiling reads true cornering (vehicle-model curvature).
  inputs["controlsState"].curvature = 15.0 * math.pi / 180.0 / (15.75 * 2.959)
  _run(curve, inputs, 40, mpc_accel=0.0)
  _run(straight, straight_inputs, 40, mpc_accel=0.0)
  assert float(curve.output_a_target) <= -0.50
  assert float(straight.output_a_target) > float(curve.output_a_target)


@pytest.mark.parametrize("brand,fingerprint,openpilot_long,pcm_cruise", [
  ("tesla", "TESLA_MODEL_3", True, False),       # other Tesla
  ("tesla", "TESLA_MODEL_S_PREAP", False, False),  # Pre-AP without openpilot long
  ("tesla", "TESLA_MODEL_S_PREAP", True, True),    # Pre-AP on stock cruise
  ("toyota", "TOYOTA_COROLLA_TSS2", True, False),
])
def test_other_cars_never_run_the_unified_controller(brand, fingerprint, openpilot_long, pcm_cruise):
  v_ego = 30.0
  cp = _make_preap_params()
  cp.brand = brand
  cp.carFingerprint = fingerprint
  cp.openpilotLongitudinalControl = openpilot_long
  cp.pcmCruise = pcm_cruise
  planner = LongitudinalPlanner(cp, init_v=v_ego, params=_MutablePlannerParams(nap_follow_dist=2))
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=-1.0)
  inputs = _make_planner_inputs(v_ego)
  _set_v_cruise_ms(inputs, 40.0)
  _own_lead(inputs, 35.0, v_ego - 1.13, a_lead=-1.18)
  for _ in range(40):
    planner.update(inputs)
  assert planner._is_preap is False
  assert planner._unified_following is False
  assert float(planner.unified_a_target) == 0.0
  publisher = _CapturingPubMaster()
  planner.publish(inputs, publisher)
  assert publisher.message.longitudinalPlan.unifiedATarget == 0.0


def test_unified_fault_latches_and_publishes_the_lead_free_command(monkeypatch):
  """An exception must not escape update() or be logged more than once.

  The mutation gate only counts AssertionError as a killed mutant, so a
  propagating exception is turned into an assertion failure here.
  """
  v_ego = 30.0
  planner, inputs, _ = _planner(v_ego, accel=-3.5)
  _own_lead(inputs, 35.0, v_ego - 1.13, a_lead=-1.18)
  logs = _spy_cloudlog_exception(monkeypatch)
  _run(planner, inputs, 80)
  assert planner._unified_faulted is False
  assert float(planner.unified_a_target) <= -0.5
  assert logs == []

  captured = _capture_lead_free(monkeypatch, planner)
  step_calls = {"n": 0}

  def _boom(*_args, **_kwargs):
    step_calls["n"] += 1
    raise RuntimeError("unified compute failed")

  monkeypatch.setattr(planner._unified, "step", _boom)
  escaped = _guarded_updates(planner, inputs, 1)
  assert escaped == [], escaped
  assert planner._unified_faulted is True
  assert planner._unified_fault_count == 1
  assert planner._unified_following is False
  assert planner.unified_a_target == 0.0
  assert planner.output_a_target == pytest.approx(captured[-1], abs=1e-9)
  assert len(logs) == 1
  assert logs[0][1] == 1

  escaped = _guarded_updates(planner, inputs, 19)
  assert escaped == [], escaped
  assert planner.output_a_target == pytest.approx(captured[-1], abs=1e-9)
  assert planner._unified_fault_count == 1
  assert step_calls["n"] == 1
  assert len(logs) == 1

  publisher = _CapturingPubMaster()
  planner.publish(inputs, publisher)
  assert publisher.message.longitudinalPlan.aTarget == pytest.approx(planner.output_a_target, abs=1e-9)
  assert publisher.message.longitudinalPlan.unifiedATarget == 0.0

  sentinel_calls = {"n": 0}

  def _sentinel(*_args, **_kwargs):
    sentinel_calls["n"] += 1
    return 4.0

  monkeypatch.setattr(planner._unified, "step", _sentinel)
  escaped = _guarded_updates(planner, inputs, 5)
  assert escaped == [], escaped
  assert sentinel_calls["n"] == 0
  assert planner._unified_faulted is True
  assert planner.output_a_target == pytest.approx(captured[-1], abs=1e-9)
  assert planner.output_a_target != pytest.approx(4.0, abs=0.5)

  inputs["controlsState"].longControlState = LongCtrlState.off
  assert _guarded_updates(planner, inputs, 1) == []
  inputs["controlsState"].longControlState = LongCtrlState.pid
  assert _guarded_updates(planner, inputs, 3) == []
  assert planner._unified_faulted is True
  assert sentinel_calls["n"] == 0
  assert len(logs) == 1


def test_cycle_time_log_reports_mean_and_max_per_window():
  log_ = CycleTimeLog(every_n=4)
  assert [log_.record(t) for t in (0.001, 0.002, 0.003)] == [None, None, None]
  window = log_.record(0.010)
  assert window is not None
  n, mean_ms, max_ms = window
  assert n == 4
  assert mean_ms == pytest.approx(4.0)
  assert max_ms == pytest.approx(10.0)
  assert log_.record(0.001) is None
  assert log_.n == 1


def test_planner_update_cycle_stays_cheap_with_a_lead():
  """Guard rail only (stub MPC): the lead-follow step must stay well under a model frame."""
  v_ego = 25.0
  planner, inputs, _ = _planner(v_ego, accel=0.0)
  _own_lead(inputs, 45.0, v_ego - 0.5, a_lead=-0.3)
  _run(planner, inputs, 20)
  frames = 400
  t0 = time.perf_counter()
  _run(planner, inputs, frames)
  mean_ms = 1000.0 * (time.perf_counter() - t0) / frames
  assert mean_ms < 5.0
