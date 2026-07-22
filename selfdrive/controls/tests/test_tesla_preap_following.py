from types import SimpleNamespace

import numpy as np
import pytest

from cereal import car, log, messaging
from opendbc.car.tesla.preap import virtual_das
from opendbc.car.tesla.preap.constants import PEDAL_LONG_K_BP, PEDAL_LONG_KI_V, PEDAL_LONG_KP_V
from opendbc.car.tesla.preap.virtual_das import GRAVITY, VirtualDAS
from opendbc.car.tesla.pedal.controller import PEDAL_RAMP_RATE_DOWN, PEDAL_RAMP_RATE_UP
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.controls.lib import longitudinal_planner
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  LongitudinalPlanSource,
  T_IDXS,
  get_safe_obstacle_distance,
  get_stopped_equivalence_factor,
  get_T_FOLLOW,
)
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.tests.test_following_distance import run_following_distance_simulation
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.test.longitudinal_maneuvers.maneuver import Maneuver
from openpilot.selfdrive.test.process_replay.process_replay import get_process_config


NAP_FOLLOW_SETTINGS = range(1, 8)
NAP_FOLLOW_TIMES_S = (0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9)
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
  def __init__(self, nap_follow_dist, adaptive_accel=False):
    self.nap_follow_dist = nap_follow_dist
    self.adaptive_accel = adaptive_accel

  def __bool__(self):
    return False

  def get(self, key, return_default=False):
    assert return_default
    assert key == "NAPFollowDistance"
    return self.nap_follow_dist

  def get_bool(self, key):
    assert key == "NAPAdaptiveAccel"
    return self.adaptive_accel


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

  def update(self, radar_state, cruise_speed_mps, t_follow):
    self.captured_t_follow = t_follow
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


def _physical_lead_distance(v_ego, v_lead, t_follow, obstacle_ratio):
  safe_obstacle_distance = get_safe_obstacle_distance(v_ego, t_follow)
  lead_obstacle_distance = obstacle_ratio * safe_obstacle_distance
  return lead_obstacle_distance - get_stopped_equivalence_factor(max(v_lead, 0.0))


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
    "nap_conf",
    SimpleNamespace(get_pedal_profile_values=lambda: [50.0] * len(virtual_das.PEDAL_BP)),
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

  params = _MutablePlannerParams(nap_follow_dist=nap_follow_dist, adaptive_accel=True)
  car_params = _make_preap_params()
  car_params.longitudinalActuatorDelay = FULL_LOOP_PLANT_DELAY_S
  planner = LongitudinalPlanner(car_params, init_v=speed_mps, params=params)
  long_control = LongControl(car_params)
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
      lead.status = True
      lead.dRel = float(max(gap_m, 0.0))
      lead.vRel = float(lead_speed_mps - speed_mps)
      lead.vLead = lead_speed_mps
      lead.vLeadK = lead_speed_mps
      lead.aLeadK = 0.0
      lead.aLeadTau = 1.5
      lead.modelProb = 1.0
      planner.update(inputs)
      planner_target_mps2 = float(planner.output_a_target)

    state = car.CarState.new_message()
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


@pytest.mark.parametrize(("lead_speed", "obstacle_ratio", "expected_strength"), [
  (30.0, 1.0, 1.0),
  (30.0, 1.2, 1.0),
  (30.0, 1.35, 0.5),
  (30.0, 1.5, 0.0),
  (20.0, 1.35, 0.5),
  (-5.0, 1.35, 0.5),
])
def test_preap_follow_cap_uses_obstacle_equivalent_distance(lead_speed, obstacle_ratio, expected_strength):
  speed_mps = 30.0
  t_follow = 1.9
  lead_distance = _physical_lead_distance(speed_mps, lead_speed, t_follow, obstacle_ratio)

  cap_strength = longitudinal_planner.get_preap_follow_cap_strength(
    speed_mps,
    lead_distance,
    lead_speed,
    t_follow,
  )

  assert cap_strength == pytest.approx(expected_strength)


def test_planner_adaptive_cap_changes_the_delivered_acceleration_for_unequal_speed_lead():
  speed_mps = 30.0
  lead_speed_mps = 35.0
  obstacle_ratio = 1.35
  t_follow = 1.9
  params = _MutablePlannerParams(nap_follow_dist=7, adaptive_accel=True)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=speed_mps, params=params)
  planner.mpc = _ConstantAccelerationMpc(speed_mps, acceleration_mps2=1.5)
  inputs = _make_planner_inputs(speed_mps)
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = _physical_lead_distance(speed_mps, lead_speed_mps, t_follow, obstacle_ratio)
  lead.vLead = lead_speed_mps

  for _ in range(32):
    planner.update(inputs)

  open_road_limit = longitudinal_planner.get_max_accel(speed_mps)
  follow_limit = longitudinal_planner._get_preap_follow_limit(speed_mps)
  cap_strength = longitudinal_planner.get_preap_follow_cap_strength(
    speed_mps,
    lead.dRel,
    lead_speed_mps,
    t_follow,
  )
  expected_adaptive_limit = open_road_limit * (1.0 - cap_strength) + follow_limit * cap_strength

  assert cap_strength == pytest.approx(0.5)
  assert planner.mpc.captured_t_follow == t_follow
  assert planner.output_a_target == pytest.approx(expected_adaptive_limit)
  assert planner.output_a_target < open_road_limit


def test_planner_publishes_the_follow_policy_used_by_mpc():
  params = _MutablePlannerParams(nap_follow_dist=1)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=FOLLOW_TEST_SPEED_MPS, params=params)
  inputs = _make_planner_inputs(FOLLOW_TEST_SPEED_MPS)
  publisher = _CapturingPubMaster()

  params.nap_follow_dist = 7
  planner._frame = 19
  planner.update(inputs)
  planner.publish(inputs, publisher)

  plan = publisher.message.longitudinalPlan
  assert plan.napFollowDistance == 7
  assert plan.tFollow == pytest.approx(1.9)
  assert planner.t_follow == 1.9
  assert np.all(planner.mpc.params[:, 4] == planner.t_follow)
  assert plan.tFollow == pytest.approx(planner.t_follow, abs=1e-6)


@pytest.mark.parametrize("nap_follow_dist", [-1, 0, 8])
def test_invalid_nap_follow_setting_publishes_zero_and_uses_personality(nap_follow_dist):
  params = _MutablePlannerParams(nap_follow_dist)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=FOLLOW_TEST_SPEED_MPS, params=params)
  inputs = _make_planner_inputs(FOLLOW_TEST_SPEED_MPS)
  publisher = _CapturingPubMaster()

  planner.update(inputs)
  planner.publish(inputs, publisher)

  plan = publisher.message.longitudinalPlan
  assert plan.napFollowDistance == 0
  assert planner.t_follow == get_T_FOLLOW(log.LongitudinalPersonality.standard)
  assert np.all(planner.mpc.params[:, 4] == planner.t_follow)
  assert plan.tFollow == pytest.approx(planner.t_follow, abs=1e-6)


def test_non_preap_planner_publishes_zero_and_uses_personality():
  planner_params = _make_preap_params()
  planner_params.brand = "honda"
  planner_params.carFingerprint = "HONDA_CIVIC"
  planner = LongitudinalPlanner(planner_params, init_v=FOLLOW_TEST_SPEED_MPS)
  inputs = _make_planner_inputs(FOLLOW_TEST_SPEED_MPS)
  inputs["selfdriveState"].personality = log.LongitudinalPersonality.relaxed
  publisher = _CapturingPubMaster()

  planner.update(inputs)
  planner.publish(inputs, publisher)

  plan = publisher.message.longitudinalPlan
  assert plan.napFollowDistance == 0
  assert planner.t_follow == get_T_FOLLOW(log.LongitudinalPersonality.relaxed)
  assert np.all(planner.mpc.params[:, 4] == planner.t_follow)
  assert plan.tFollow == pytest.approx(planner.t_follow, abs=1e-6)


def test_process_replay_ignores_additive_follow_policy_telemetry():
  ignored_fields = get_process_config("plannerd").ignore

  assert "longitudinalPlan.napFollowDistance" in ignored_fields
  assert "longitudinalPlan.tFollow" in ignored_fields


def test_nap_follow_setting_map_and_physical_gaps_are_strictly_monotonic():
  actual_follow_times = [get_T_FOLLOW(nap_follow_dist=setting) for setting in NAP_FOLLOW_SETTINGS]
  physical_gaps = [t_follow * FOLLOW_TEST_SPEED_MPS + STOP_DISTANCE_M for t_follow in actual_follow_times]

  assert actual_follow_times == list(NAP_FOLLOW_TIMES_S)
  assert np.all(np.diff(physical_gaps) > 0.0)


def test_nap_follow_settings_control_monotonic_maneuver_gaps():
  steady_gaps = [
    run_following_distance_simulation(
      FOLLOW_TEST_SPEED_MPS,
      t_end=80.0,
      nap_follow_dist=nap_follow_dist,
    )
    for nap_follow_dist in NAP_FOLLOW_SETTINGS
  ]
  expected_gaps = [
    t_follow * FOLLOW_TEST_SPEED_MPS + STOP_DISTANCE_M
    for t_follow in NAP_FOLLOW_TIMES_S
  ]

  assert steady_gaps == pytest.approx(expected_gaps, abs=0.5)
  assert np.all(np.diff(steady_gaps) > 0.0)


def test_max_follow_setting_recovers_from_a_close_lead_without_closing_first():
  maneuver = Maneuver(
    "max follow recovery",
    duration=60.0,
    initial_speed=FOLLOW_TEST_SPEED_MPS,
    lead_relevancy=True,
    initial_distance_lead=20.0,
    speed_lead_values=[FOLLOW_TEST_SPEED_MPS],
    breakpoints=[0.0],
    e2e=False,
    nap_follow_dist=7,
  )

  valid, output = maneuver.evaluate()
  assert valid
  assert np.min(output[:, 6]) >= 19.5
  assert output[-1, 6] == pytest.approx(53.5, abs=0.5)
  assert output[-1, 3] == pytest.approx(FOLLOW_TEST_SPEED_MPS, abs=0.1)


def test_max_follow_full_closed_loop_recovers_gap_with_production_fallback(monkeypatch):
  samples, pedal_samples = _run_full_closed_loop_following(monkeypatch, nap_follow_dist=7)
  disabled_grade_samples, _ = _run_full_closed_loop_following(
    monkeypatch,
    nap_follow_dist=7,
    grade_compensation_scale=0.0,
  )
  elapsed_s, gaps_m, speeds_mps, accelerations_mps2, planner_targets_mps2, vdas_targets_mps2, _ = samples.T
  disabled_grade_speeds_mps = disabled_grade_samples[:, 2]
  desired_gap_m = get_T_FOLLOW(nap_follow_dist=7) * FOLLOW_TEST_SPEED_MPS + STOP_DISTANCE_M
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
  desired_gap_m = get_T_FOLLOW(nap_follow_dist=7) * FOLLOW_TEST_SPEED_MPS + STOP_DISTANCE_M
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
