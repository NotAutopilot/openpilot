from types import SimpleNamespace
from typing import Any, cast

from openpilot.cereal import log
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode


LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection


def _car_state(*, left=True, right=False, lever=0):
  return SimpleNamespace(
    vEgo=30.0,
    leftBlinker=left,
    rightBlinker=right,
    turnSignalStalkState=lever,
    leftBlindspot=False,
    rightBlindspot=False,
    steeringPressed=False,
    steeringTorque=0.0,
    brakePressed=False,
  )


def _helper():
  helper = DesireHelper()
  cast(Any, helper.alc).update_params = lambda: None
  helper.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGE
  cast(Any, helper.lane_turn_controller).update_params = lambda: None
  return helper


def test_opposite_physical_tap_cancels_armed_lane_change():
  helper = _helper()
  helper.update(_car_state(lever=1), True, 1.0)
  assert helper.lane_change_state == LaneChangeState.preLaneChange
  assert helper.lane_change_direction == LaneChangeDirection.left

  helper.update(_car_state(lever=0), True, 1.0)
  helper.update(_car_state(lever=2), True, 1.0)
  assert helper.lane_change_state == LaneChangeState.off
  assert helper.lane_change_direction == LaneChangeDirection.none


def test_held_opposite_lever_does_not_rearm():
  helper = _helper()
  helper.update(_car_state(lever=1), True, 1.0)
  helper.update(_car_state(lever=2), True, 1.0)
  helper.update(_car_state(lever=2), True, 1.0)
  assert helper.lane_change_state == LaneChangeState.off
