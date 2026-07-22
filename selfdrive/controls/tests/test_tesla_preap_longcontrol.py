"""Pre-AP planner-target contracts at the LongControl output seam."""

from dataclasses import dataclass
from types import SimpleNamespace

from cereal import car
from opendbc.car.tesla.preap.constants import (
  PEDAL_LONG_K_BP,
  PEDAL_LONG_KI_V,
  PEDAL_LONG_KP_V,
  VDAS_ACCEL_JERK_MAX,
  VDAS_DECEL_JERK_MAX,
)
from opendbc.car.tesla.preap.nap_conf import PEDAL_MAX_VALUES
from opendbc.car.tesla.preap.virtual_das import VirtualDAS
from opendbc.car.tesla.preap import virtual_das

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.longcontrol import LongControl


ACCEL_LIMITS_MPS2 = (-1.5, 0.8)
PASSTHROUGH_TOLERANCE_MPS2 = 1e-9
VDAS_DT_S = 0.02
PEDAL_PLANT_DELAY_S = 0.40
PEDAL_PLANT_TAU_S = 0.25
PEDAL_PLANT_ACCEL_PER_DI_MPS2 = 0.063
SETTLED_REGEN_WINDOW_S = 1.0


@dataclass(frozen=True)
class _TracePhase:
  duration_s: float
  planner_target_end_mps2: float
  road_load_end_mps2: float


def _make_preap_params():
  params = car.CarParams.new_message()
  params.brand = "tesla"
  params.carFingerprint = "TESLA_MODEL_S_PREAP"
  params.openpilotLongitudinalControl = True
  params.pcmCruise = False
  params.longitudinalTuning.kpBP = PEDAL_LONG_K_BP
  params.longitudinalTuning.kpV = PEDAL_LONG_KP_V
  params.longitudinalTuning.kiBP = PEDAL_LONG_K_BP
  params.longitudinalTuning.kiV = PEDAL_LONG_KI_V
  params.longitudinalTuning.kf = 1.0
  params.vEgoStarting = 0.1
  return params


def _make_car_state(speed_mps):
  state = car.CarState.new_message()
  state.vEgo = speed_mps
  state.brakePressed = False
  state.cruiseState.standstill = False
  return state


def _run_continuous_trace(long_control, state, phases):
  planner_target_mps2 = 0.0
  road_load_mps2 = 0.0
  planner_targets_mps2 = []
  vdas_commands_mps2 = []

  for phase in phases:
    frame_count = round(phase.duration_s / DT_CTRL)
    phase_start_target_mps2 = planner_target_mps2
    phase_start_road_load_mps2 = road_load_mps2
    planner_jerk_mps3 = (
      (phase.planner_target_end_mps2 - phase_start_target_mps2) / (frame_count * DT_CTRL)
    )
    assert -VDAS_DECEL_JERK_MAX <= planner_jerk_mps3 <= VDAS_ACCEL_JERK_MAX, (
      f"planner trace jerk {planner_jerk_mps3:.3f} m/s^3 exceeds VDAS command bounds"
    )

    for frame_index in range(1, frame_count + 1):
      phase_fraction = frame_index / frame_count
      planner_target_mps2 = phase_start_target_mps2 + (
        phase.planner_target_end_mps2 - phase_start_target_mps2
      ) * phase_fraction
      road_load_mps2 = phase_start_road_load_mps2 + (
        phase.road_load_end_mps2 - phase_start_road_load_mps2
      ) * phase_fraction
      state.aEgo = planner_target_mps2 - road_load_mps2

      vdas_command_mps2 = long_control.update(
        active=True,
        CS=state,
        a_target=planner_target_mps2,
        should_stop=False,
        accel_limits=ACCEL_LIMITS_MPS2,
      )
      planner_targets_mps2.append(planner_target_mps2)
      vdas_commands_mps2.append(vdas_command_mps2)

      state.vEgo += state.aEgo * DT_CTRL

  return planner_targets_mps2, vdas_commands_mps2


def _run_continuous_trace_through_pedal(long_control, vdas, state, phases, coast_pedal_di):
  planner_target_mps2 = 0.0
  road_load_mps2 = 0.0
  pedal_di = coast_pedal_di
  delayed_pedal_di = [coast_pedal_di] * round(PEDAL_PLANT_DELAY_S / VDAS_DT_S)
  plant_alpha = VDAS_DT_S / (PEDAL_PLANT_TAU_S + VDAS_DT_S)
  pedal_samples = []
  control_frame = 0

  for phase in phases:
    frame_count = round(phase.duration_s / DT_CTRL)
    phase_start_target_mps2 = planner_target_mps2
    phase_start_road_load_mps2 = road_load_mps2
    planner_jerk_mps3 = (
      (phase.planner_target_end_mps2 - phase_start_target_mps2) / (frame_count * DT_CTRL)
    )
    assert -VDAS_DECEL_JERK_MAX <= planner_jerk_mps3 <= VDAS_ACCEL_JERK_MAX, (
      f"planner trace jerk {planner_jerk_mps3:.3f} m/s^3 exceeds VDAS command bounds"
    )

    for frame_index in range(1, frame_count + 1):
      phase_fraction = frame_index / frame_count
      planner_target_mps2 = phase_start_target_mps2 + (
        phase.planner_target_end_mps2 - phase_start_target_mps2
      ) * phase_fraction
      road_load_mps2 = phase_start_road_load_mps2 + (
        phase.road_load_end_mps2 - phase_start_road_load_mps2
      ) * phase_fraction

      vdas_command_mps2 = long_control.update(
        active=True,
        CS=state,
        a_target=planner_target_mps2,
        should_stop=False,
        accel_limits=ACCEL_LIMITS_MPS2,
      )
      if control_frame % round(VDAS_DT_S / DT_CTRL) == 0:
        pedal_di = vdas.update(
          vdas_command_mps2,
          v_ego=state.vEgo,
          prev_pedal_di=pedal_di,
          a_ego=state.aEgo,
          freeze_integrator=False,
          orientation_ned=[0.0, 0.0, 0.0],
        )
        applied_pedal_di = delayed_pedal_di.pop(0)
        delayed_pedal_di.append(pedal_di)
        plant_target_accel_mps2 = (
          (applied_pedal_di - coast_pedal_di) * PEDAL_PLANT_ACCEL_PER_DI_MPS2
          - road_load_mps2
        )
        state.aEgo += plant_alpha * (plant_target_accel_mps2 - state.aEgo)
        pedal_samples.append((planner_target_mps2, pedal_di))

      state.vEgo = max(0.0, state.vEgo + state.aEgo * DT_CTRL)
      control_frame += 1

  return pedal_samples


def test_road_load_history_cannot_reverse_finite_jerk_negative_planner_target():
  long_control = LongControl(_make_preap_params())
  state = _make_car_state(speed_mps=25.0)
  phases = (
    _TracePhase(duration_s=0.5, planner_target_end_mps2=0.35, road_load_end_mps2=0.55),
    _TracePhase(duration_s=4.0, planner_target_end_mps2=0.35, road_load_end_mps2=0.55),
    _TracePhase(duration_s=0.4, planner_target_end_mps2=-0.25, road_load_end_mps2=0.55),
  )

  planner_targets_mps2, vdas_commands_mps2 = _run_continuous_trace(long_control, state, phases)
  final_planner_target_mps2 = planner_targets_mps2[-1]
  final_vdas_command_mps2 = vdas_commands_mps2[-1]

  failure_message = (
    f"planner reached {final_planner_target_mps2:.3f} m/s^2 through a finite-jerk transition after road load, "
    + f"but VDAS received a positive {final_vdas_command_mps2:.3f} m/s^2 command"
  )
  assert final_vdas_command_mps2 <= 0.0, failure_message


def test_vdas_receives_route_shaped_planner_target_trace_unchanged():
  long_control = LongControl(_make_preap_params())
  state = _make_car_state(speed_mps=22.0)
  route_phases = (
    _TracePhase(duration_s=1.0, planner_target_end_mps2=0.40, road_load_end_mps2=0.20),
    _TracePhase(duration_s=4.0, planner_target_end_mps2=0.40, road_load_end_mps2=0.50),
    _TracePhase(duration_s=1.0, planner_target_end_mps2=0.00, road_load_end_mps2=0.20),
    _TracePhase(duration_s=1.0, planner_target_end_mps2=-0.30, road_load_end_mps2=-0.10),
    _TracePhase(duration_s=2.0, planner_target_end_mps2=-0.10, road_load_end_mps2=-0.20),
    _TracePhase(duration_s=1.0, planner_target_end_mps2=0.30, road_load_end_mps2=0.00),
    _TracePhase(duration_s=3.0, planner_target_end_mps2=0.30, road_load_end_mps2=0.45),
  )

  planner_targets_mps2, vdas_commands_mps2 = _run_continuous_trace(long_control, state, route_phases)
  command_target_deltas_mps2 = [
    abs(command_mps2 - target_mps2)
    for target_mps2, command_mps2 in zip(planner_targets_mps2, vdas_commands_mps2, strict=True)
  ]
  worst_sample_index = max(range(len(command_target_deltas_mps2)), key=command_target_deltas_mps2.__getitem__)
  max_command_target_delta_mps2 = command_target_deltas_mps2[worst_sample_index]
  planner_target_mps2 = planner_targets_mps2[worst_sample_index]
  vdas_command_mps2 = vdas_commands_mps2[worst_sample_index]

  failure_message = (
    f"LongControl changed in-range planner target {planner_target_mps2:.6f} m/s^2 "
    + f"to {vdas_command_mps2:.6f} m/s^2 before VDAS at trace sample {worst_sample_index}; "
    + f"delta {max_command_target_delta_mps2:.6f} m/s^2"
  )
  assert max_command_target_delta_mps2 <= PASSTHROUGH_TOLERANCE_MPS2, failure_message


def test_negative_planner_target_reaches_regen_side_of_coast_anchor(monkeypatch):
  coast_pedal_di = 3.0
  zero_torque = SimpleNamespace(get=lambda _v_ego: coast_pedal_di)
  controller_conf = SimpleNamespace(get_pedal_profile_values=lambda: PEDAL_MAX_VALUES)
  monkeypatch.setattr(virtual_das, "get_zero_torque", lambda: zero_torque)
  monkeypatch.setattr(virtual_das, "nap_conf", controller_conf)

  long_control = LongControl(_make_preap_params())
  vdas = VirtualDAS(dt=VDAS_DT_S)
  state = _make_car_state(speed_mps=25.0)
  phases = (
    _TracePhase(duration_s=0.5, planner_target_end_mps2=0.0, road_load_end_mps2=0.443),
    _TracePhase(duration_s=12.0, planner_target_end_mps2=0.0, road_load_end_mps2=0.443),
    _TracePhase(duration_s=0.8, planner_target_end_mps2=-0.20, road_load_end_mps2=0.20),
    _TracePhase(duration_s=1.0, planner_target_end_mps2=-0.20, road_load_end_mps2=0.20),
  )
  vdas.reset(measured_accel=0.0, commanded_accel=0.0, pedal_di_init=coast_pedal_di)

  pedal_samples = _run_continuous_trace_through_pedal(
    long_control,
    vdas,
    state,
    phases,
    coast_pedal_di,
  )
  settled_sample_count = round(SETTLED_REGEN_WINDOW_S / VDAS_DT_S)
  settled_pedal_samples = pedal_samples[-settled_sample_count:]

  assert all(target_mps2 <= -0.15 for target_mps2, _pedal_di in settled_pedal_samples)
  maximum_settled_pedal_di = max(pedal_di for _target_mps2, pedal_di in settled_pedal_samples)
  assert maximum_settled_pedal_di <= coast_pedal_di - 0.05, (
    f"settled negative planner window reached {max(pedal_di for _, pedal_di in settled_pedal_samples):.3f} DI, "
    + f"without moving meaningfully below the {coast_pedal_di:.3f} DI coast anchor"
  )
