import numpy as np
import pytest

from cereal import car, log, messaging
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.modeld.constants import ModelConstants


def make_preap_params():
  params = car.CarParams.new_message()
  params.brand = "tesla"
  params.carFingerprint = "TESLA_MODEL_S_PREAP"
  params.openpilotLongitudinalControl = True
  params.pcmCruise = False
  params.steerRatio = 15.75
  params.wheelbase = 2.959
  return params


def make_planner_inputs(*, v_ego, v_cruise, pitch, throttle_probability):
  radar = messaging.new_message("radarState").radarState
  controls = messaging.new_message("controlsState").controlsState
  selfdrive = messaging.new_message("selfdriveState").selfdriveState
  car_state = messaging.new_message("carState").carState
  car_control = messaging.new_message("carControl").carControl
  live_parameters = messaging.new_message("liveParameters").liveParameters
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
    "carControl": car_control,
    "liveParameters": live_parameters,
    "modelV2": model,
  }


def test_preap_cruise_ignores_model_throttle_suppression():
  planner = LongitudinalPlanner(make_preap_params(), init_v=20.9)
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


@pytest.mark.parametrize(("brand", "fingerprint", "openpilot_longitudinal", "pcm_cruise"), [
  ("honda", "HONDA_CIVIC", True, False),
  ("tesla", "TESLA_MODEL_S_PREAP", False, True),
])
def test_non_vdas_modes_keep_model_throttle_suppression(
  brand, fingerprint, openpilot_longitudinal, pcm_cruise,
):
  params = make_preap_params()
  params.brand = brand
  params.carFingerprint = fingerprint
  params.openpilotLongitudinalControl = openpilot_longitudinal
  params.pcmCruise = pcm_cruise
  planner = LongitudinalPlanner(params, init_v=20.9)
  inputs = make_planner_inputs(
    v_ego=20.9,
    v_cruise=21.0,
    pitch=0.046,
    throttle_probability=0.15,
  )

  for _ in range(60):
    planner.update(inputs)

  assert not planner.allow_throttle
  assert planner.output_a_target < -0.5
