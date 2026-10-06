from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import (
  DesireHelper as _NapDesireHelper,
  LANE_CHANGE_ARM_TIME,
  MAX_QUEUED_LANE_CHANGES,
  LaneChangeDirection,
  LaneChangeState,
)
from opendbc.car import structs

# sunnypilot: DesireHelper takes CP and only uses the nap-dev lane change for
# the Pre-AP car. These tests are the nap-dev tests, so hand them that CP.
_PREAP_CP = structs.CarParams()
_PREAP_CP.carFingerprint = "TESLA_MODEL_S_PREAP"


def DesireHelper(CP=None):
  return _NapDesireHelper(_PREAP_CP if CP is None else CP)


class FakeCarState:
  def __init__(self, v_ego=30.0, left=False, right=False,
               steering_pressed=False, steering_torque=0.0,
               left_blindspot=False, right_blindspot=False, lever=0):
    self.vEgo = v_ego
    self.leftBlinker = left
    self.rightBlinker = right
    self.steeringPressed = steering_pressed
    self.steeringTorque = steering_torque
    self.leftBlindspot = left_blindspot
    self.rightBlindspot = right_blindspot
    self.turnSignalStalkState = lever


def _tick(dh, cs, n=1, lane_change_prob=0.0):
  for _ in range(n):
    dh.update(cs, lateral_active=True, lane_change_prob=lane_change_prob)


def _arm_left(dh):
  dh.update(FakeCarState(), True, 0.0)
  dh.update(FakeCarState(left=True, lever=1), True, 0.0)
  dh.update(FakeCarState(left=True), True, 0.0)


def _lever_tap(dh, lever, lamp_left=True, hold_ticks=3):
  for _ in range(hold_ticks):
    dh.update(FakeCarState(left=lamp_left, lever=lever), True, 0.0)
  dh.update(FakeCarState(left=lamp_left), True, 0.0)


def _nudge_left():
  return FakeCarState(left=True, steering_pressed=True, steering_torque=1.0)


def _advance_to_finishing(dh):
  _tick(dh, _nudge_left(), n=int(0.6 / DT_MDL), lane_change_prob=0.0)
  assert dh.lane_change_state == LaneChangeState.laneChangeFinishing


def _complete_maneuver(dh, cs_after):
  _advance_to_finishing(dh)
  for _ in range(int(1.5 / DT_MDL)):
    dh.update(cs_after, True, 0.0)
    if dh.lane_change_state != LaneChangeState.laneChangeFinishing:
      break


def test_tap_arms_pre_lane_change():
  dh = DesireHelper()
  _arm_left(dh)

  assert dh.lane_change_state == LaneChangeState.preLaneChange
  assert dh.lane_change_direction == LaneChangeDirection.left
  assert dh.queued_changes == 1


def test_latch_survives_blinker_lamp_off():
  dh = DesireHelper()
  _arm_left(dh)
  dh.update(FakeCarState(), True, 0.0)

  assert dh.lane_change_state == LaneChangeState.preLaneChange


def test_below_speed_does_not_arm():
  dh = DesireHelper()
  slow = 10 * CV.MPH_TO_MS
  dh.update(FakeCarState(v_ego=slow), True, 0.0)
  dh.update(FakeCarState(v_ego=slow, left=True, lever=1), True, 0.0)

  assert dh.lane_change_state == LaneChangeState.off


def test_arming_times_out_without_nudge():
  dh = DesireHelper()
  _arm_left(dh)
  _tick(dh, FakeCarState(left=True), n=int(LANE_CHANGE_ARM_TIME / DT_MDL) + 5)

  assert dh.lane_change_state == LaneChangeState.off
  assert dh.lane_change_direction == LaneChangeDirection.none
  assert dh.queued_changes == 0


def test_wheel_nudge_starts_lane_change():
  dh = DesireHelper()
  _arm_left(dh)
  dh.update(_nudge_left(), True, 0.0)

  assert dh.lane_change_state == LaneChangeState.laneChangeStarting


def test_nudge_during_hands_pause_is_remembered_without_command_desire():
  dh = DesireHelper()
  _arm_left(dh)
  dh.update(_nudge_left(), False, 0.0, hands_on_paused=True)
  assert dh.lane_change_state == LaneChangeState.preLaneChange
  assert dh.desire == log.Desire.none
  for _ in range(int(1.1 / DT_MDL)):
    dh.update(FakeCarState(left=True), False, 0.0, hands_on_paused=True)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    assert dh.desire == log.Desire.none
  dh.update(FakeCarState(left=True), True, 0.0)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting


def test_hands_pause_freezes_maneuver_progress_but_not_timeout():
  dh = DesireHelper()
  _arm_left(dh)
  dh.update(_nudge_left(), True, 0.0)
  prob = dh.lane_change_ll_prob
  dh.update(FakeCarState(left=True), False, 0.0, hands_on_paused=True)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting
  assert dh.lane_change_ll_prob == prob
  assert dh.desire == log.Desire.none
  for _ in range(int(11 / DT_MDL)):
    dh.update(FakeCarState(left=True), False, 0.0, hands_on_paused=True)
  assert dh.lane_change_state == LaneChangeState.off


def test_blindspot_cancels_frozen_maneuver_during_pause_or_on_resume():
  for phase in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing):
    for right in (False, True):
      for occupied_on_resume in (False, True):
        dh = DesireHelper()
        direction = LaneChangeDirection.right if right else LaneChangeDirection.left
        clear = FakeCarState(left=not right, right=right)
        dh.update(FakeCarState(), True, 0.0)
        dh.update(clear, True, 0.0)
        nudge = FakeCarState(left=not right, right=right, steering_pressed=True, steering_torque=-1.0 if right else 1.0)
        dh.update(nudge, True, 0.0)
        if phase == LaneChangeState.laneChangeFinishing:
          _tick(dh, clear, n=int(0.6 / DT_MDL))
        assert dh.lane_change_state == phase
        assert dh.lane_change_direction == direction
        dh.update(clear, False, 0.0, hands_on_paused=True)
        occupied = FakeCarState(left=not right, right=right, left_blindspot=not right, right_blindspot=right)
        dh.update(occupied, occupied_on_resume, 0.0, hands_on_paused=not occupied_on_resume)
        assert dh.lane_change_state == LaneChangeState.off
        assert dh.desire == log.Desire.none
        assert dh.queued_changes == 0
        dh.update(clear, True, 0.0)
        assert dh.lane_change_state == LaneChangeState.off


def test_lost_hands_pause_reason_cancels_retained_nudge():
  # A hard disable, brake pause, or stale state is NOT hands_on_paused.
  dh = DesireHelper()
  _arm_left(dh)
  dh.update(_nudge_left(), False, 0.0, hands_on_paused=True)
  dh.update(FakeCarState(left=True), False, 0.0)
  dh.update(FakeCarState(left=True), True, 0.0)
  assert dh.lane_change_state == LaneChangeState.off
  assert dh.queued_changes == 0


def test_blindspot_invalidates_nudge_during_pause():
  dh = DesireHelper()
  _arm_left(dh)
  dh.update(_nudge_left(), False, 0.0, hands_on_paused=True)
  dh.update(FakeCarState(left=True, left_blindspot=True), False, 0.0, hands_on_paused=True)
  dh.update(FakeCarState(left=True), True, 0.0)
  assert dh.lane_change_state == LaneChangeState.preLaneChange
  dh.update(_nudge_left(), True, 0.0)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting


def test_opposite_tap_cancels_nudge_during_hands_pause():
  dh = DesireHelper()
  _arm_left(dh)
  dh.update(_nudge_left(), False, 0.0, hands_on_paused=True)
  dh.update(FakeCarState(left=True, lever=2), False, 0.0, hands_on_paused=True)
  dh.update(FakeCarState(left=True), True, 0.0)
  assert dh.lane_change_state == LaneChangeState.off


def test_pause_without_nudge_does_not_start_lane_change():
  dh = DesireHelper()
  _arm_left(dh)
  dh.update(FakeCarState(left=True), False, 0.0, hands_on_paused=True)
  dh.update(FakeCarState(left=True), True, 0.0)
  assert dh.lane_change_state == LaneChangeState.preLaneChange


def test_opposite_lever_tap_cancels_while_arming():
  dh = DesireHelper()
  _arm_left(dh)
  dh.update(FakeCarState(left=True, lever=2), True, 0.0)

  assert dh.lane_change_state == LaneChangeState.off
  assert dh.lane_change_direction == LaneChangeDirection.none
  assert dh.queued_changes == 0


def test_opposite_lever_tap_cancels_while_starting():
  dh = DesireHelper()
  _arm_left(dh)
  dh.update(_nudge_left(), True, 0.0)
  dh.update(FakeCarState(left=True, lever=2), True, 0.0)

  assert dh.lane_change_state == LaneChangeState.off
  assert dh.lane_change_direction == LaneChangeDirection.none


def test_opposite_lever_tap_cancels_while_finishing():
  dh = DesireHelper()
  _arm_left(dh)
  _advance_to_finishing(dh)
  dh.update(FakeCarState(left=True, lever=2), True, 0.0)

  assert dh.lane_change_state == LaneChangeState.off
  assert dh.lane_change_direction == LaneChangeDirection.none


def test_opposite_tap_does_not_rearm_new_direction():
  dh = DesireHelper()
  _arm_left(dh)
  dh.update(FakeCarState(left=True, right=False, lever=2), True, 0.0)

  assert dh.lane_change_state == LaneChangeState.off


def test_held_lever_counts_once():
  dh = DesireHelper()
  _arm_left(dh)
  _tick(dh, FakeCarState(left=True, lever=1), n=40)

  assert dh.queued_changes == 2


def test_same_direction_tap_during_maneuver_queues_change():
  dh = DesireHelper()
  _arm_left(dh)
  dh.update(_nudge_left(), True, 0.0)
  _lever_tap(dh, 1)

  assert dh.queued_changes == 2


def test_held_lamp_without_lever_does_not_queue():
  dh = DesireHelper()
  _arm_left(dh)
  dh.update(_nudge_left(), True, 0.0)
  _tick(dh, FakeCarState(left=True), n=20)

  assert dh.queued_changes == 1


def test_lamp_edges_without_lever_do_not_queue():
  dh = DesireHelper()
  _arm_left(dh)
  for _ in range(3):
    dh.update(FakeCarState(), True, 0.0)
    dh.update(FakeCarState(left=True), True, 0.0)

  assert dh.queued_changes == 1


def test_queue_capped_at_three():
  dh = DesireHelper()
  _arm_left(dh)
  for _ in range(5):
    _lever_tap(dh, 1)

  assert dh.queued_changes == MAX_QUEUED_LANE_CHANGES == 3


def test_queued_change_rearms_after_first_completes():
  dh = DesireHelper()
  _arm_left(dh)
  _lever_tap(dh, 1)
  dh.update(_nudge_left(), True, 0.0)
  _complete_maneuver(dh, FakeCarState(left=True))

  assert dh.lane_change_state == LaneChangeState.preLaneChange
  assert dh.lane_change_direction == LaneChangeDirection.left
  assert dh.queued_changes == 1


def test_single_change_resets_after_completion():
  dh = DesireHelper()
  _arm_left(dh)
  dh.update(_nudge_left(), True, 0.0)
  _complete_maneuver(dh, FakeCarState())

  assert dh.lane_change_state == LaneChangeState.off
  assert dh.lane_change_direction == LaneChangeDirection.none
  assert dh.queued_changes == 0


def test_queued_change_timeout_resets_everything():
  dh = DesireHelper()
  _arm_left(dh)
  _lever_tap(dh, 1)
  dh.update(_nudge_left(), True, 0.0)
  _complete_maneuver(dh, FakeCarState(left=True))
  _tick(dh, FakeCarState(left=True), n=int(LANE_CHANGE_ARM_TIME / DT_MDL) + 5)

  assert dh.lane_change_state == LaneChangeState.off
  assert dh.lane_change_direction == LaneChangeDirection.none
  assert dh.queued_changes == 0
