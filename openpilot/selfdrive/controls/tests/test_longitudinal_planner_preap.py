import numpy as np
import pytest

from openpilot.cereal import log, messaging
from opendbc.car import structs
from opendbc.car.tesla.preap.boot import (
  apply_preap_hardware_snapshot,
  hardware_snapshot_from_values,
  pedal_pipeline_enabled,
)
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.modeld.constants import ModelConstants


def _pedal_snapshot():
  return hardware_snapshot_from_values(
    pedal_enabled=True,
    pedal_bus=2,
    pedal_calib_done=True,
    pedal_calib_factor=0.035,
    pedal_calib_zero=0.25,
    pedal_calib_min=-3.0,
    pedal_calib_max=99.6,
  )


def make_preap_params(pedal=True):
  CP = structs.CarParams()
  CP.brand = "tesla"
  CP.carFingerprint = "TESLA_MODEL_S_PREAP"
  CP.steerRatio = 15.75
  CP.wheelbase = 2.959
  CP.openpilotLongitudinalControl = False
  CP.pcmCruise = True
  CP_SP = structs.CarParamsSP()
  if pedal:
    apply_preap_hardware_snapshot(CP, CP_SP, _pedal_snapshot())
  return CP, CP_SP


def make_planner_inputs(*, v_ego, v_cruise, pitch, throttle_probability):
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
  car_state.vEgo = v_ego
  car_state.vCruise = v_cruise * 3.6
  car_control.orientationNED = [0.0, pitch, 0.0]

  position = log.XYZTData.new_message()
  position.x = ((v_ego + 0.5) * np.array(ModelConstants.T_IDXS)).tolist()
  model.position = position
  velocity = log.XYZTData.new_message()
  velocity.x = ((v_ego + 0.5) * np.ones_like(ModelConstants.T_IDXS)).tolist()
  velocity.x[0] = v_ego
  model.velocity = velocity
  acceleration = log.XYZTData.new_message()
  acceleration.x = np.zeros_like(ModelConstants.T_IDXS).tolist()
  model.acceleration = acceleration
  model.meta.disengagePredictions.gasPressProbs = [throttle_probability] * 6

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


class _FollowParams:
  def __init__(self, nap_follow_dist):
    self.nap_follow_dist = nap_follow_dist

  def get(self, key, return_default=False):
    assert return_default
    assert key == "NAPFollowDistance"
    return self.nap_follow_dist

  def get_bool(self, key):
    raise AssertionError(f"NAPAdaptiveAccel must not be read; got {key}")


def test_preap_cruise_ignores_model_throttle_suppression():
  CP, CP_SP = make_preap_params(pedal=True)
  assert pedal_pipeline_enabled(CP, CP_SP)
  planner = LongitudinalPlanner(CP, CP_SP, init_v=20.9, params=_FollowParams(3))
  inputs = make_planner_inputs(
    v_ego=20.9,
    v_cruise=21.0,
    pitch=0.046,
    throttle_probability=0.15,
  )

  for _ in range(60):
    planner.update(inputs)

  assert planner.output_a_target > -0.05
  assert planner.allow_throttle
  assert planner._pedal_preap


@pytest.mark.parametrize(("brand", "fingerprint", "openpilot_longitudinal", "pcm_cruise"), [
  ("honda", "HONDA_CIVIC", True, False),
  ("tesla", "TESLA_MODEL_S_PREAP", False, True),
])
def test_non_vdas_modes_keep_model_throttle_suppression(
  brand, fingerprint, openpilot_longitudinal, pcm_cruise,
):
  CP = structs.CarParams()
  CP.brand = brand
  CP.carFingerprint = fingerprint
  CP.openpilotLongitudinalControl = openpilot_longitudinal
  CP.pcmCruise = pcm_cruise
  CP.steerRatio = 15.75
  CP.wheelbase = 2.959
  CP_SP = structs.CarParamsSP()
  assert not pedal_pipeline_enabled(CP, CP_SP)
  planner = LongitudinalPlanner(CP, CP_SP, init_v=20.9)
  inputs = make_planner_inputs(
    v_ego=20.9,
    v_cruise=21.0,
    pitch=0.046,
    throttle_probability=0.15,
  )

  for _ in range(60):
    planner.update(inputs)

  assert not planner._pedal_preap
  assert not planner.allow_throttle
  assert planner.output_a_target < -0.5
