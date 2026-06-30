import math

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import (
    DesireHelper, LaneChangeState, LaneChangeDirection,
    LANE_CHANGE_ARM_TIME, MAX_QUEUED_LANE_CHANGES,
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


def _tick(dh, cs, n=1, lane_change_prob=0.0):
    for _ in range(n):
        dh.update(cs, lateral_active=True, lane_change_prob=lane_change_prob)


def _arm_left(dh):
    """Drive a single left tap (rising edge of one_blinker) to arm preLaneChange."""
    dh.update(FakeCarState(left=False), True, 0.0)
    dh.update(FakeCarState(left=True), True, 0.0)


def _nudge_left():
    return FakeCarState(left=True, steering_pressed=True, steering_torque=1.0)


# ---- arming -------------------------------------------------------------

def test_tap_arms_pre_lane_change():
    dh = DesireHelper()
    _arm_left(dh)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    assert dh.lane_change_direction == LaneChangeDirection.left


def test_latch_survives_blinker_off():
    dh = DesireHelper()
    _arm_left(dh)
    # blinker physically off, no nudge — still armed (well within 7s)
    dh.update(FakeCarState(left=False), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.preLaneChange


def test_below_speed_does_not_arm():
    dh = DesireHelper()
    slow = 10 * CV.MPH_TO_MS
    dh.update(FakeCarState(v_ego=slow, left=False), True, 0.0)
    dh.update(FakeCarState(v_ego=slow, left=True), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.off


# ---- time-based countdown ----------------------------------------------

def test_signals_remaining_starts_at_budget():
    dh = DesireHelper()
    _arm_left(dh)
    assert dh.signals_remaining == math.ceil(LANE_CHANGE_ARM_TIME)


def test_signals_remaining_counts_down_with_time():
    dh = DesireHelper()
    _arm_left(dh)
    start = dh.signals_remaining
    # advance ~2 seconds of ticks, blinker off the whole time (no edges)
    _tick(dh, FakeCarState(left=False), n=int(2.0 / DT_MDL))
    assert dh.signals_remaining < start
    assert dh.signals_remaining > 0


def test_cancels_after_time_budget_without_nudge():
    dh = DesireHelper()
    _arm_left(dh)
    # advance past the 7s budget with no wheel nudge
    _tick(dh, FakeCarState(left=False), n=int(LANE_CHANGE_ARM_TIME / DT_MDL) + 5)
    assert dh.lane_change_state == LaneChangeState.off
    assert dh.lane_change_direction == LaneChangeDirection.none


# ---- nudge / exits ------------------------------------------------------

def test_wheel_nudge_starts_lane_change():
    dh = DesireHelper()
    _arm_left(dh)
    dh.update(_nudge_left(), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting


def test_opposite_tap_cancels():
    dh = DesireHelper()
    _arm_left(dh)
    dh.update(FakeCarState(left=False, right=False), True, 0.0)  # blinker off
    dh.update(FakeCarState(right=True), True, 0.0)               # opposite rising edge
    assert dh.lane_change_state == LaneChangeState.off
    assert dh.lane_change_direction == LaneChangeDirection.none


# ---- full reset after single change (stuck-toast fix) -------------------

def _complete_maneuver(dh, cs_after):
    """Drive laneChangeStarting -> Finishing -> done. cs_after is the carstate
    presented once the maneuver finishes (controls which post-state we land in)."""
    # In starting: lane_change_prob low + ll_prob fades to push to finishing
    _tick(dh, _nudge_left(), n=int(0.6 / DT_MDL), lane_change_prob=0.0)
    assert dh.lane_change_state == LaneChangeState.laneChangeFinishing
    # In finishing: ll_prob fades back in over ~1s
    for _ in range(int(1.5 / DT_MDL)):
        dh.update(cs_after, True, 0.0)
        if dh.lane_change_state != LaneChangeState.laneChangeFinishing:
            break


def test_single_change_resets_to_off_after_completion():
    dh = DesireHelper()
    _arm_left(dh)
    dh.update(_nudge_left(), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting
    # After maneuver, blinker is off (op stops driving it). Expect full reset.
    _complete_maneuver(dh, FakeCarState(left=False))
    assert dh.lane_change_state == LaneChangeState.off
    assert dh.lane_change_direction == LaneChangeDirection.none
    assert dh.lane_changes_remaining == 0


# ---- multi-lane-change queue -------------------------------------------

def test_double_tap_queues_two_changes():
    dh = DesireHelper()
    # first tap arms
    dh.update(FakeCarState(left=False), True, 0.0)
    dh.update(FakeCarState(left=True), True, 0.0)
    # second same-direction tap within window: blinker off then on again
    dh.update(FakeCarState(left=False), True, 0.0)
    dh.update(FakeCarState(left=True), True, 0.0)
    assert dh.queued_changes == 2
    # one in progress + one remaining
    assert dh.lane_changes_remaining == 1


def test_queue_capped_at_three():
    dh = DesireHelper()
    dh.update(FakeCarState(left=False), True, 0.0)
    for _ in range(5):  # 5 taps, cap is 3
        dh.update(FakeCarState(left=True), True, 0.0)
        dh.update(FakeCarState(left=False), True, 0.0)
    assert dh.queued_changes == MAX_QUEUED_LANE_CHANGES == 3


def test_second_change_rearms_after_first_completes():
    dh = DesireHelper()
    # queue two
    dh.update(FakeCarState(left=False), True, 0.0)
    dh.update(FakeCarState(left=True), True, 0.0)
    dh.update(FakeCarState(left=False), True, 0.0)
    dh.update(FakeCarState(left=True), True, 0.0)
    assert dh.queued_changes == 2
    # nudge + complete first; signal stays on (left=True) between changes
    dh.update(_nudge_left(), True, 0.0)
    _complete_maneuver(dh, FakeCarState(left=True))
    # Should re-arm in same direction, NOT reset to off
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    assert dh.lane_change_direction == LaneChangeDirection.left
    assert dh.lane_changes_remaining == 0  # one left, now in the waiting window
    assert dh.queued_changes == 1


def test_midsequence_timeout_full_resets():
    dh = DesireHelper()
    # queue two
    dh.update(FakeCarState(left=False), True, 0.0)
    dh.update(FakeCarState(left=True), True, 0.0)
    dh.update(FakeCarState(left=False), True, 0.0)
    dh.update(FakeCarState(left=True), True, 0.0)
    dh.update(_nudge_left(), True, 0.0)
    _complete_maneuver(dh, FakeCarState(left=True))
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    # now let the 2nd window time out with no nudge
    _tick(dh, FakeCarState(left=True), n=int(LANE_CHANGE_ARM_TIME / DT_MDL) + 5)
    assert dh.lane_change_state == LaneChangeState.off
    assert dh.lane_change_direction == LaneChangeDirection.none
    assert dh.queued_changes == 0


# ---- taps DURING the maneuver (not just the pre-nudge window) -----------

def _tap_same_dir_left(dh):
    """A same-direction (left) tap: blinker off for a tick, then on — a rising
    edge of one_blinker — without applying wheel torque."""
    dh.update(FakeCarState(left=False), True, 0.0)
    dh.update(FakeCarState(left=True), True, 0.0)


def test_same_dir_tap_during_starting_queues_change():
    dh = DesireHelper()
    _arm_left(dh)
    dh.update(_nudge_left(), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting
    assert dh.queued_changes == 1
    # tap again while the maneuver is underway
    _tap_same_dir_left(dh)
    assert dh.queued_changes == 2


def test_same_dir_tap_during_finishing_queues_change():
    dh = DesireHelper()
    _arm_left(dh)
    # drive into laneChangeFinishing
    _tick(dh, _nudge_left(), n=int(0.6 / DT_MDL), lane_change_prob=0.0)
    assert dh.lane_change_state == LaneChangeState.laneChangeFinishing
    assert dh.queued_changes == 1
    _tap_same_dir_left(dh)
    assert dh.queued_changes == 2


def test_tap_during_maneuver_offers_second_change_after_completion():
    # The reported bug: tap during the first change -> a second change should be
    # offered (re-armed, awaiting its own nudge) once the first completes.
    dh = DesireHelper()
    _arm_left(dh)
    dh.update(_nudge_left(), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting
    _tap_same_dir_left(dh)               # queue a second mid-maneuver
    assert dh.queued_changes == 2
    _complete_maneuver(dh, FakeCarState(left=True))
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    assert dh.lane_change_direction == LaneChangeDirection.left
    assert dh.queued_changes == 1
    # second change still needs its own wheel nudge
    dh.update(_nudge_left(), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting


def test_opposite_tap_during_starting_cancels():
    dh = DesireHelper()
    _arm_left(dh)
    dh.update(_nudge_left(), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting
    # opposite (right) tap while the left maneuver is underway -> full cancel
    dh.update(FakeCarState(left=False, right=False), True, 0.0)
    dh.update(FakeCarState(right=True), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.off
    assert dh.lane_change_direction == LaneChangeDirection.none
    assert dh.queued_changes == 0


def test_opposite_tap_during_finishing_cancels():
    dh = DesireHelper()
    _arm_left(dh)
    _tick(dh, _nudge_left(), n=int(0.6 / DT_MDL), lane_change_prob=0.0)
    assert dh.lane_change_state == LaneChangeState.laneChangeFinishing
    dh.update(FakeCarState(left=False, right=False), True, 0.0)
    dh.update(FakeCarState(right=True), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.off
    assert dh.lane_change_direction == LaneChangeDirection.none


def test_held_blinker_during_maneuver_does_not_queue():
    # Guard: a continuously-held blinker (no rising edge) must NOT be miscounted
    # as repeated taps while the maneuver runs.
    dh = DesireHelper()
    _arm_left(dh)
    dh.update(_nudge_left(), True, 0.0)
    assert dh.queued_changes == 1
    _tick(dh, FakeCarState(left=True), n=20)  # blinker held on, no edges
    assert dh.queued_changes == 1
