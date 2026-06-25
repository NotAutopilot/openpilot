from openpilot.selfdrive.controls.lib.desire_helper import (
    DesireHelper, LaneChangeState, LaneChangeDirection, LANE_CHANGE_FLASH_BUDGET,
)
from openpilot.common.constants import CV


class FakeCarState:
    def __init__(self, v_ego=30.0, left=False, right=False,
                 steering_pressed=False, steering_torque=0.0,
                 left_blindspot=False, right_blindspot=False):
        self.vEgo = v_ego
        self.leftBlinker = left
        self.rightBlinker = right
        self.steeringPressed = steering_pressed
        self.steeringTorque = steering_torque
        self.leftBlindspot = left_blindspot
        self.rightBlindspot = right_blindspot


def _pulse_blinker(dh, cs, on):
    """Drive one blinker on/off transition through update() to count a flash."""
    cs.leftBlinker = on and (dh.lane_change_direction == LaneChangeDirection.left)
    cs.rightBlinker = on and (dh.lane_change_direction == LaneChangeDirection.right)
    dh.update(cs, lateral_active=True, lane_change_prob=0.0)


def test_tap_arms_pre_lane_change():
    dh = DesireHelper()
    # rising edge of one_blinker (a tap)
    dh.update(FakeCarState(left=False), True, 0.0)
    dh.update(FakeCarState(left=True), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    assert dh.lane_change_direction == LaneChangeDirection.left


def test_latch_survives_blinker_off():
    # Core fix: after the tap, blinker physically goes off but arming persists.
    dh = DesireHelper()
    dh.update(FakeCarState(left=False), True, 0.0)
    dh.update(FakeCarState(left=True), True, 0.0)
    # blinker now off (stock 3-flash ended), no wheel nudge
    dh.update(FakeCarState(left=False), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.preLaneChange


def test_flashes_remaining_counts_down():
    dh = DesireHelper()
    dh.update(FakeCarState(left=False), True, 0.0)
    cs = FakeCarState(left=True)
    dh.update(cs, True, 0.0)  # enter preLaneChange, first rising edge = 1 flash
    assert dh.flashes_remaining == LANE_CHANGE_FLASH_BUDGET - 1
    # toggle blinker off then on -> second flash
    _pulse_blinker(dh, cs, False)
    _pulse_blinker(dh, cs, True)
    assert dh.flashes_remaining == LANE_CHANGE_FLASH_BUDGET - 2


def test_cancels_when_budget_exhausted_without_nudge():
    dh = DesireHelper()
    dh.update(FakeCarState(left=False), True, 0.0)
    cs = FakeCarState(left=True)
    dh.update(cs, True, 0.0)
    # Generate 7 flashes with no wheel nudge
    for _ in range(LANE_CHANGE_FLASH_BUDGET):
        _pulse_blinker(dh, cs, False)
        _pulse_blinker(dh, cs, True)
    assert dh.lane_change_state == LaneChangeState.off
    assert dh.flashes_remaining == LANE_CHANGE_FLASH_BUDGET


def test_wheel_nudge_starts_lane_change():
    dh = DesireHelper()
    dh.update(FakeCarState(left=False), True, 0.0)
    dh.update(FakeCarState(left=True), True, 0.0)
    # driver nudges wheel left (positive torque, pressed) while armed left
    nudge = FakeCarState(left=True, steering_pressed=True, steering_torque=1.0)
    dh.update(nudge, True, 0.0)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting


def test_opposite_tap_cancels():
    dh = DesireHelper()
    dh.update(FakeCarState(left=False), True, 0.0)
    dh.update(FakeCarState(left=True), True, 0.0)   # armed left
    # opposite tap: right blinker rising edge
    dh.update(FakeCarState(left=False, right=False), True, 0.0)  # blinker off
    dh.update(FakeCarState(right=True), True, 0.0)  # right rising edge
    assert dh.lane_change_state == LaneChangeState.off


def test_below_speed_does_not_arm():
    dh = DesireHelper()
    slow = 10 * CV.MPH_TO_MS
    dh.update(FakeCarState(v_ego=slow, left=False), True, 0.0)
    dh.update(FakeCarState(v_ego=slow, left=True), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.off
