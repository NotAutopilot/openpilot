from types import SimpleNamespace
from typing import Any, cast

import numpy as np
import pytest

from openpilot.cereal import log, messaging
from opendbc.car import structs
from opendbc.car.tesla.preap import virtual_das
from opendbc.car.tesla.preap.boot import apply_preap_hardware_snapshot, hardware_snapshot_from_values, pedal_pipeline_enabled
from opendbc.car.tesla.preap.constants import (
  PEDAL_LONG_K_BP, PEDAL_LONG_KI_V, PEDAL_LONG_KP_V,
  PEDAL_RAMP_RATE_DOWN, PEDAL_RAMP_RATE_UP,
  PREAP_T_FOLLOW,
)
from opendbc.car.tesla.preap.virtual_das import GRAVITY, VirtualDAS
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
import openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc as long_mpc_mod
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  LongitudinalPlanSource,
  T_IDXS,
  get_T_FOLLOW,
)
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.modeld.constants import ModelConstants


NAP_FOLLOW_SETTINGS = range(1, 8)
NAP_FOLLOW_TIMES_S = PREAP_T_FOLLOW
FOLLOW_TEST_SPEED_MPS = 25.0
STOP_DISTANCE_M = 6.0
FULL_LOOP_DT_S = 0.01
FULL_LOOP_PLANNER_DT_S = 0.05
FULL_LOOP_VDAS_DT_S = 0.02
FULL_LOOP_PLANT_DELAY_S = 0.40
FULL_LOOP_PLANT_TAU_S = 0.25
FULL_LOOP_PEDAL_DI_BP = [-5.0, -2.0, 0.0, 3.0, 8.0, 15.0, 22.0, 27.0, 35.0, 50.0]
FULL_LOOP_NET_ACCEL_BP = [-1.05, -0.62, -0.50, -0.36, 0.0, 0.50, 1.15, 1.65, 2.10, 2.45]
FULL_LOOP_RECOVERY_END_S = 70.0
FULL_LOOP_UPHILL_RAMP_END_S = 72.0
FULL_LOOP_UPHILL_HOLD_END_S = 78.0
FULL_LOOP_CREST_RAMP_END_S = 80.0
FULL_LOOP_CREST_HOLD_END_S = 86.0
FULL_LOOP_ROLLING_RAMP_END_S = 88.0
FULL_LOOP_DURATION_S = 100.0
FULL_LOOP_UPHILL_PITCH_RAD = float(np.deg2rad(4.0))
FULL_LOOP_CREST_PITCH_RAD = float(np.deg2rad(-3.0))
FULL_LOOP_ROLLING_PITCH_RAD = float(np.deg2rad(2.0))
FULL_LOOP_GRADE_SETTLING_S = (
  FULL_LOOP_PLANT_DELAY_S + FULL_LOOP_PLANT_TAU_S + 5.0 * virtual_das.PITCH_LP_RC
)


class _FollowParams:
  def __init__(self, nap_follow_dist):
    self.nap_follow_dist = nap_follow_dist

  def get(self, key, return_default=False):
    assert return_default
    assert key == "NAPFollowDistance"
    return self.nap_follow_dist

  def get_bool(self, key):
    raise AssertionError(f"NAPAdaptiveAccel must not be read; got {key}")


class _ConstantAccelerationMpc:
  def __init__(self, speed_mps, acceleration_mps2):
    self.v_solution = speed_mps + acceleration_mps2 * T_IDXS
    self.a_solution = np.full(len(T_IDXS), acceleration_mps2)
    self.j_solution = np.zeros(len(T_IDXS) - 1)
    self.params = np.zeros((len(T_IDXS), 6))
    self.source = LongitudinalPlanSource.cruise
    self.crash_cnt = 0
    self.solve_time = 0.0
    self.captured_t_follow = None

  @staticmethod
  def set_weights(prev_accel_constraint, personality):
    pass

  @staticmethod
  def set_cur_state(speed_mps, acceleration_mps2):
    pass

  def update(self, radar_state, cruise_speed_mps, personality=None):
    t_follow = long_mpc_mod.get_T_FOLLOW(personality)
    self.captured_t_follow = t_follow
    self.params[:, 4] = t_follow


def _make_preap_params():
  CP = structs.CarParams()
  CP.brand = "tesla"
  CP.carFingerprint = "TESLA_MODEL_S_PREAP"
  CP.openpilotLongitudinalControl = True
  CP.pcmCruise = False
  CP.steerRatio = 15.75
  CP.wheelbase = 2.959
  CP.longitudinalTuning.kpBP = list(PEDAL_LONG_K_BP)
  CP.longitudinalTuning.kpV = list(PEDAL_LONG_KP_V)
  CP.longitudinalTuning.kiBP = list(PEDAL_LONG_K_BP)
  CP.longitudinalTuning.kiV = list(PEDAL_LONG_KI_V)
  try:
    CP.longitudinalTuning.kf = 1.0
  except AttributeError:
    pass
  CP_SP = structs.CarParamsSP()
  apply_preap_hardware_snapshot(CP, CP_SP, hardware_snapshot_from_values(
    pedal_enabled=True,
    pedal_bus=2,
    pedal_calib_done=True,
    pedal_calib_factor=0.035,
    pedal_calib_zero=0.25,
    pedal_calib_min=-3.0,
    pedal_calib_max=99.6,
  ))
  return CP, CP_SP


def _make_planner_inputs(speed_mps):
  radar = messaging.new_message("radarState").radarState
  controls = messaging.new_message("controlsState").controlsState
  selfdrive = messaging.new_message("selfdriveState").selfdriveState
  car_state = messaging.new_message("carState").carState
  car_state_sp = messaging.new_message("carStateSP").carStateSP
  car_control = messaging.new_message("carControl").carControl
  live_parameters = messaging.new_message("liveParameters").liveParameters
  gps_location = messaging.new_message("gpsLocation").gpsLocation
  live_map_data_sp = messaging.new_message("liveMapDataSP").liveMapDataSP
  model = messaging.new_message("modelV2").modelV2

  controls.longControlState = LongCtrlState.pid
  selfdrive.personality = log.LongitudinalPersonality.standard
  car_state.vEgo = speed_mps
  car_state.vCruise = speed_mps * 3.6

  position = log.XYZTData.new_message()
  position.x = ((speed_mps + 0.5) * np.array(ModelConstants.T_IDXS)).tolist()
  model.position = position
  velocity = log.XYZTData.new_message()
  velocity.x = ((speed_mps + 0.5) * np.ones_like(ModelConstants.T_IDXS)).tolist()
  velocity.x[0] = speed_mps
  model.velocity = velocity
  acceleration = log.XYZTData.new_message()
  acceleration.x = np.zeros_like(ModelConstants.T_IDXS).tolist()
  model.acceleration = acceleration
  model.meta.disengagePredictions.gasPressProbs = [1.0] * 6

  return {
    "radarState": radar,
    "controlsState": controls,
    "selfdriveState": selfdrive,
    "carState": car_state,
    "carStateSP": car_state_sp,
    "carControl": car_control,
    "liveParameters": live_parameters,
    "modelV2": model,
    "gpsLocation": gps_location,
    "liveMapDataSP": live_map_data_sp,
  }


def _full_loop_pitch(elapsed_s):
  if elapsed_s < FULL_LOOP_RECOVERY_END_S:
    return 0.0
  if elapsed_s < FULL_LOOP_UPHILL_RAMP_END_S:
    return FULL_LOOP_UPHILL_PITCH_RAD * (
      (elapsed_s - FULL_LOOP_RECOVERY_END_S) /
      (FULL_LOOP_UPHILL_RAMP_END_S - FULL_LOOP_RECOVERY_END_S)
    )
  if elapsed_s < FULL_LOOP_UPHILL_HOLD_END_S:
    return FULL_LOOP_UPHILL_PITCH_RAD
  if elapsed_s < FULL_LOOP_CREST_RAMP_END_S:
    ramp_fraction = (
      (elapsed_s - FULL_LOOP_UPHILL_HOLD_END_S) /
      (FULL_LOOP_CREST_RAMP_END_S - FULL_LOOP_UPHILL_HOLD_END_S)
    )
    return FULL_LOOP_UPHILL_PITCH_RAD + ramp_fraction * (
      FULL_LOOP_CREST_PITCH_RAD - FULL_LOOP_UPHILL_PITCH_RAD
    )
  if elapsed_s < FULL_LOOP_CREST_HOLD_END_S:
    return FULL_LOOP_CREST_PITCH_RAD
  if elapsed_s < FULL_LOOP_ROLLING_RAMP_END_S:
    ramp_fraction = (
      (elapsed_s - FULL_LOOP_CREST_HOLD_END_S) /
      (FULL_LOOP_ROLLING_RAMP_END_S - FULL_LOOP_CREST_HOLD_END_S)
    )
    return FULL_LOOP_CREST_PITCH_RAD + ramp_fraction * (
      FULL_LOOP_ROLLING_PITCH_RAD - FULL_LOOP_CREST_PITCH_RAD
    )
  return FULL_LOOP_ROLLING_PITCH_RAD


def _run_full_closed_loop_following(
    monkeypatch,
    nap_follow_dist=7,
    plant_aligned_feedforward=False,
    grade_compensation_scale=1.0,
):
  monkeypatch.setattr(
    virtual_das,
    "PEDAL_MAX_VALUES",
    [50.0] * len(virtual_das.PEDAL_BP),
  )
  monkeypatch.setattr(
    virtual_das,
    "get_zero_torque",
    lambda: SimpleNamespace(get=lambda _speed_mps: 3.0),
  )

  speed_mps = FOLLOW_TEST_SPEED_MPS
  acceleration_mps2 = 0.0
  ego_distance_m = 0.0
  lead_distance_m = 20.0
  lead_speed_mps = FOLLOW_TEST_SPEED_MPS
  pedal_di = 8.0
  planner_target_mps2 = 0.0
  vdas_target_mps2 = 0.0

  params = _FollowParams(nap_follow_dist)
  car_params, car_params_sp = _make_preap_params()
  car_params.longitudinalActuatorDelay = FULL_LOOP_PLANT_DELAY_S
  planner = LongitudinalPlanner(car_params, car_params_sp, init_v=speed_mps, params=params)
  long_control = LongControl(car_params, car_params_sp)
  vdas = VirtualDAS(dt=FULL_LOOP_VDAS_DT_S)
  vdas.reset(measured_accel=acceleration_mps2, commanded_accel=0.0, pedal_di_init=pedal_di)
  if plant_aligned_feedforward:
    vdas._feedforward = lambda acceleration_effort_mps2, _speed_mps: float(np.interp(
      acceleration_effort_mps2,
      FULL_LOOP_NET_ACCEL_BP,
      FULL_LOOP_PEDAL_DI_BP,
    ))
  if grade_compensation_scale != 1.0:
    grade_estimator_update = vdas.grade_estimator.update

    def scaled_grade_estimator_update(orientation_ned):
      steady_compensation, transient_compensation = grade_estimator_update(orientation_ned)
      return (
        grade_compensation_scale * steady_compensation,
        grade_compensation_scale * transient_compensation,
      )

    vdas.grade_estimator.update = scaled_grade_estimator_update

  delay_steps = round(FULL_LOOP_PLANT_DELAY_S / FULL_LOOP_DT_S)
  delayed_pedals_di = [pedal_di] * delay_steps
  plant_alpha = FULL_LOOP_DT_S / (FULL_LOOP_PLANT_TAU_S + FULL_LOOP_DT_S)
  planner_interval_steps = round(FULL_LOOP_PLANNER_DT_S / FULL_LOOP_DT_S)
  vdas_interval_steps = round(FULL_LOOP_VDAS_DT_S / FULL_LOOP_DT_S)
  samples = []
  pedal_samples = []

  for step in range(round(FULL_LOOP_DURATION_S / FULL_LOOP_DT_S)):
    elapsed_s = step * FULL_LOOP_DT_S
    pitch_rad = _full_loop_pitch(elapsed_s)
    gap_m = lead_distance_m - ego_distance_m

    if step % planner_interval_steps == 0:
      inputs = _make_planner_inputs(float(speed_mps))
      inputs["carState"].aEgo = float(acceleration_mps2)
      inputs["carState"].vCruise = lead_speed_mps * 3.6
      inputs["carControl"].orientationNED = [0.0, pitch_rad, 0.0]
      lead = inputs["radarState"].leadOne
      lead.present = True
      lead.dRel = float(max(gap_m, 0.0))
      lead.vRel = float(lead_speed_mps - speed_mps)
      lead.vLead = lead_speed_mps
      lead.vLeadK = lead_speed_mps
      lead.aLeadK = 0.0
      lead.aLeadTau = 1.5
      lead.modelProb = 1.0
      planner.update(inputs)
      planner_target_mps2 = float(planner.output_a_target)

    state = structs.CarState()
    state.vEgo = float(speed_mps)
    state.aEgo = float(acceleration_mps2)
    state.brakePressed = False
    state.cruiseState.standstill = False
    vdas_target_mps2 = float(long_control.update(
      active=True,
      CS=state,
      a_target=planner_target_mps2,
      should_stop=planner.output_should_stop,
      accel_limits=(-1.5, 0.8),
    ))

    if step % vdas_interval_steps == 0:
      pedal_di = vdas.update(
        vdas_target_mps2,
        v_ego=speed_mps,
        prev_pedal_di=pedal_di,
        a_ego=acceleration_mps2,
        freeze_integrator=False,
        orientation_ned=[0.0, pitch_rad, 0.0],
      )
      pedal_samples.append(pedal_di)

    applied_pedal_di = delayed_pedals_di.pop(0)
    delayed_pedals_di.append(pedal_di)
    grade_acceleration_mps2 = GRAVITY * np.sin(pitch_rad)
    plant_target_mps2 = float(np.interp(
      applied_pedal_di,
      FULL_LOOP_PEDAL_DI_BP,
      FULL_LOOP_NET_ACCEL_BP,
    )) - grade_acceleration_mps2
    acceleration_mps2 += plant_alpha * (plant_target_mps2 - acceleration_mps2)
    speed_mps = max(0.0, speed_mps + acceleration_mps2 * FULL_LOOP_DT_S)
    ego_distance_m += speed_mps * FULL_LOOP_DT_S
    lead_distance_m += lead_speed_mps * FULL_LOOP_DT_S

    samples.append((
      elapsed_s,
      lead_distance_m - ego_distance_m,
      speed_mps,
      acceleration_mps2,
      planner_target_mps2,
      vdas_target_mps2,
      pitch_rad,
    ))

  return np.array(samples), np.array(pedal_samples)



def test_planner_publishes_the_follow_policy_used_by_mpc():
  CP, CP_SP = _make_preap_params()
  assert pedal_pipeline_enabled(CP, CP_SP)
  params = _FollowParams(1)
  planner = LongitudinalPlanner(CP, CP_SP, init_v=FOLLOW_TEST_SPEED_MPS, params=params)
  planner.mpc = cast(Any, _ConstantAccelerationMpc(FOLLOW_TEST_SPEED_MPS, 0.0))
  inputs = _make_planner_inputs(FOLLOW_TEST_SPEED_MPS)
  params.nap_follow_dist = 7
  planner._frame = 19
  planner.update(inputs)
  assert planner.t_follow == pytest.approx(1.9)
  assert planner.mpc.captured_t_follow == pytest.approx(1.9)
  assert np.all(planner.mpc.params[:, 4] == planner.t_follow)


@pytest.mark.parametrize("nap_follow_dist", [-1, 0, 8])
def test_invalid_nap_follow_setting_uses_personality(nap_follow_dist):
  CP, CP_SP = _make_preap_params()
  planner = LongitudinalPlanner(CP, CP_SP, init_v=FOLLOW_TEST_SPEED_MPS, params=_FollowParams(nap_follow_dist))
  planner.mpc = cast(Any, _ConstantAccelerationMpc(FOLLOW_TEST_SPEED_MPS, 0.0))
  inputs = _make_planner_inputs(FOLLOW_TEST_SPEED_MPS)
  planner.update(inputs)
  assert planner.t_follow == get_T_FOLLOW(log.LongitudinalPersonality.standard)
  assert planner.mpc.captured_t_follow == planner.t_follow


def test_non_preap_planner_uses_personality():
  CP = structs.CarParams()
  CP.brand = "honda"
  CP.carFingerprint = "HONDA_CIVIC"
  CP.steerRatio = 15.75
  CP.wheelbase = 2.959
  CP_SP = structs.CarParamsSP()
  planner = LongitudinalPlanner(CP, CP_SP, init_v=FOLLOW_TEST_SPEED_MPS)
  planner.mpc = cast(Any, _ConstantAccelerationMpc(FOLLOW_TEST_SPEED_MPS, 0.0))
  inputs = _make_planner_inputs(FOLLOW_TEST_SPEED_MPS)
  inputs["selfdriveState"].personality = log.LongitudinalPersonality.relaxed
  planner.update(inputs)
  assert not planner._pedal_preap
  assert planner.t_follow == get_T_FOLLOW(log.LongitudinalPersonality.relaxed)


def test_nap_follow_setting_map_is_strictly_monotonic():
  actual_follow_times = [PREAP_T_FOLLOW[setting - 1] for setting in NAP_FOLLOW_SETTINGS]
  physical_gaps = [t_follow * FOLLOW_TEST_SPEED_MPS + STOP_DISTANCE_M for t_follow in actual_follow_times]
  assert actual_follow_times == list(NAP_FOLLOW_TIMES_S)
  assert np.all(np.diff(physical_gaps) > 0.0)


def test_max_follow_full_closed_loop_recovers_gap_with_production_fallback(monkeypatch):
  samples, pedal_samples = _run_full_closed_loop_following(monkeypatch, nap_follow_dist=7)
  disabled_grade_samples, _ = _run_full_closed_loop_following(
    monkeypatch,
    nap_follow_dist=7,
    grade_compensation_scale=0.0,
  )
  elapsed_s, gaps_m, speeds_mps, accelerations_mps2, planner_targets_mps2, vdas_targets_mps2, _ = samples.T
  disabled_grade_speeds_mps = disabled_grade_samples[:, 2]
  desired_gap_m = PREAP_T_FOLLOW[6] * FOLLOW_TEST_SPEED_MPS + STOP_DISTANCE_M
  recovery_window = (elapsed_s >= FULL_LOOP_RECOVERY_END_S - 5.0) & (elapsed_s < FULL_LOOP_RECOVERY_END_S)
  grade_window = elapsed_s >= FULL_LOOP_RECOVERY_END_S
  settled_rolling_window = elapsed_s >= FULL_LOOP_ROLLING_RAMP_END_S + FULL_LOOP_GRADE_SETTLING_S
  final_speed_window = elapsed_s >= FULL_LOOP_DURATION_S - 2.0

  assert np.min(gaps_m) >= 19.5
  assert np.mean(gaps_m[recovery_window]) == pytest.approx(desired_gap_m, abs=2.0)
  assert np.mean(speeds_mps[recovery_window]) == pytest.approx(FOLLOW_TEST_SPEED_MPS, abs=0.2)
  assert gaps_m[-1] >= desired_gap_m - 2.0
  assert np.mean(speeds_mps[final_speed_window]) == pytest.approx(FOLLOW_TEST_SPEED_MPS, abs=0.2)
  assert np.max(speeds_mps[settled_rolling_window]) <= FOLLOW_TEST_SPEED_MPS + 0.1

  assert np.min(accelerations_mps2) >= -1.0
  assert np.max(accelerations_mps2) <= 0.8
  assert np.max(np.abs(np.diff(accelerations_mps2) / FULL_LOOP_DT_S)) <= 1.5
  assert np.max(np.diff(pedal_samples)) <= PEDAL_RAMP_RATE_UP + 1e-9
  assert np.min(np.diff(pedal_samples)) >= -PEDAL_RAMP_RATE_DOWN - 1e-9
  assert np.min(pedal_samples) >= FULL_LOOP_PEDAL_DI_BP[0]
  assert np.max(pedal_samples) <= FULL_LOOP_PEDAL_DI_BP[-1]
  assert np.max(np.abs(planner_targets_mps2 - vdas_targets_mps2)) <= 1e-9

  compensated_speed_error = np.trapezoid(
    np.abs(speeds_mps[grade_window] - FOLLOW_TEST_SPEED_MPS),
    elapsed_s[grade_window],
  )
  disabled_speed_error = np.trapezoid(
    np.abs(disabled_grade_speeds_mps[grade_window] - FOLLOW_TEST_SPEED_MPS),
    elapsed_s[grade_window],
  )
  assert np.max(np.abs(speeds_mps[grade_window] - FOLLOW_TEST_SPEED_MPS)) <= 1.25
  assert compensated_speed_error <= 0.5 * disabled_speed_error


def test_plant_aligned_full_closed_loop_grade_compensation_holds_speed(monkeypatch):
  samples, _ = _run_full_closed_loop_following(
    monkeypatch,
    nap_follow_dist=7,
    plant_aligned_feedforward=True,
  )
  elapsed_s, gaps_m, speeds_mps, accelerations_mps2, _, vdas_targets_mps2, pitches_rad = samples.T
  desired_gap_m = PREAP_T_FOLLOW[6] * FOLLOW_TEST_SPEED_MPS + STOP_DISTANCE_M
  uphill_window = (
    (elapsed_s >= FULL_LOOP_UPHILL_RAMP_END_S + FULL_LOOP_GRADE_SETTLING_S)
    & (elapsed_s < FULL_LOOP_UPHILL_HOLD_END_S)
  )
  crest_window = (
    (elapsed_s >= FULL_LOOP_CREST_RAMP_END_S + FULL_LOOP_GRADE_SETTLING_S)
    & (elapsed_s < FULL_LOOP_CREST_HOLD_END_S)
  )
  rolling_window = elapsed_s >= FULL_LOOP_ROLLING_RAMP_END_S + FULL_LOOP_GRADE_SETTLING_S

  assert np.min(gaps_m) >= 19.5
  assert gaps_m[-1] >= desired_gap_m - 2.0
  for window, expected_pitch_rad in (
    (uphill_window, FULL_LOOP_UPHILL_PITCH_RAD),
    (crest_window, FULL_LOOP_CREST_PITCH_RAD),
    (rolling_window, FULL_LOOP_ROLLING_PITCH_RAD),
  ):
    assert pitches_rad[window] == pytest.approx(np.full(np.count_nonzero(window), expected_pitch_rad))
    assert np.min(speeds_mps[window]) >= FOLLOW_TEST_SPEED_MPS - 0.5
    assert np.max(speeds_mps[window]) <= FOLLOW_TEST_SPEED_MPS + 0.5

  for phase_end_s in (
    FULL_LOOP_UPHILL_HOLD_END_S,
    FULL_LOOP_CREST_HOLD_END_S,
    FULL_LOOP_DURATION_S,
  ):
    tracking_window = (elapsed_s >= phase_end_s - 2.0) & (elapsed_s < phase_end_s)
    assert np.mean(np.abs(
      accelerations_mps2[tracking_window] - vdas_targets_mps2[tracking_window]
    )) <= 0.12
