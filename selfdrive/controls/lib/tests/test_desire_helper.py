import math

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import (
    DesireHelper, LaneChangeState, LaneChangeDirection, ButtonType,
    LANE_CHANGE_ARM_TIME, MAX_QUEUED_LANE_CHANGES,
)
from openpilot.common.constants import CV


class FakeButtonEvent:
    def __init__(self, btn_type, pressed=True):
        self.type = btn_type
        self.pressed = pressed


def _left_tap_events():
    return [FakeButtonEvent(ButtonType.leftBlinker)]


def _right_tap_events():
    return [FakeButtonEvent(ButtonType.rightBlinker)]


class FakeCarState:
    """Models the Pre-AP signal reality:
    - leftBlinker/rightBlinker = indicator LAMP status (BC_indicatorLStatus),
      which latches ON while openpilot drives the blinker.
    - buttonEvents = turn-signal LEVER transitions (TurnIndLvr_Stat), the only
      reliable driver-tap signal while the lamp is op-driven.
    """
    def __init__(self, v_ego=30.0, left=False, right=False,
                 steering_pressed=False, steering_torque=0.0,
                 left_blindspot=False, right_blindspot=False,
                 button_events=()):
        self.vEgo = v_ego
        self.leftBlinker = left
        self.rightBlinker = right
        self.steeringPressed = steering_pressed
        self.steeringTorque = steering_torque
        self.leftBlindspot = left_blindspot
        self.rightBlindspot = right_blindspot
        self.buttonEvents = list(button_events)


def _tick(dh, cs, n=1, lane_change_prob=0.0):
    for _ in range(n):
        dh.update(cs, lateral_active=True, lane_change_prob=lane_change_prob)


def _arm_left(dh):
    """Initial arming: a tap raises the stock lamp -> one_blinker rising edge.
    (The lever event fires too, but arming keys off the lamp edge.)"""
    dh.update(FakeCarState(left=False), True, 0.0)
    dh.update(FakeCarState(left=True, button_events=_left_tap_events()), True, 0.0)


def _nudge_left():
    return FakeCarState(left=True, steering_pressed=True, steering_torque=1.0)


def _complete_maneuver(dh, cs_after):
    """Drive laneChangeStarting -> Finishing -> done. cs_after is the carstate
    presented once the maneuver finishes."""
    _tick(dh, _nudge_left(), n=int(0.6 / DT_MDL), lane_change_prob=0.0)
    assert dh.lane_change_state == LaneChangeState.laneChangeFinishing
    for _ in range(int(1.5 / DT_MDL)):
        dh.update(cs_after, True, 0.0)
        if dh.lane_change_state != LaneChangeState.laneChangeFinishing:
            break


# ---- arming -------------------------------------------------------------

def test_tap_arms_pre_lane_change():
    dh = DesireHelper()
    _arm_left(dh)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    assert dh.lane_change_direction == LaneChangeDirection.left
    assert dh.queued_changes == 1


def test_latch_survives_blinker_off():
    # Arming persists even if the lamp momentarily reads off (stock 3-flash gap).
    dh = DesireHelper()
    _arm_left(dh)
    dh.update(FakeCarState(left=False), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.preLaneChange


def test_below_speed_does_not_arm():
    dh = DesireHelper()
    slow = 10 * CV.MPH_TO_MS
    dh.update(FakeCarState(v_ego=slow, left=False), True, 0.0)
    dh.update(FakeCarState(v_ego=slow, left=True, button_events=_left_tap_events()), True, 0.0)
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
    # ~2s pass; lamp is op-driven (held on), no lever events
    _tick(dh, FakeCarState(left=True), n=int(2.0 / DT_MDL))
    assert dh.signals_remaining < start
    assert dh.signals_remaining > 0


def test_cancels_after_time_budget_without_nudge():
    dh = DesireHelper()
    _arm_left(dh)
    _tick(dh, FakeCarState(left=True), n=int(LANE_CHANGE_ARM_TIME / DT_MDL) + 5)
    assert dh.lane_change_state == LaneChangeState.off
    assert dh.lane_change_direction == LaneChangeDirection.none


# ---- nudge / exits ------------------------------------------------------

def test_wheel_nudge_starts_lane_change():
    dh = DesireHelper()
    _arm_left(dh)
    dh.update(_nudge_left(), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting


def test_opposite_lever_tap_cancels_while_arming():
    # Lamp is op-driven (held on, left); driver taps the lever RIGHT.
    dh = DesireHelper()
    _arm_left(dh)
    dh.update(FakeCarState(left=True, button_events=_right_tap_events()), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.off
    assert dh.lane_change_direction == LaneChangeDirection.none
    assert dh.queued_changes == 0


# ---- full reset after single change (stuck-toast fix) -------------------

def test_single_change_resets_to_off_after_completion():
    dh = DesireHelper()
    _arm_left(dh)
    dh.update(_nudge_left(), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting
    _complete_maneuver(dh, FakeCarState(left=False))
    assert dh.lane_change_state == LaneChangeState.off
    assert dh.lane_change_direction == LaneChangeDirection.none
    assert dh.lane_changes_remaining == 0


# ---- multi-lane-change queue (lever taps; lamp stays latched ON) ---------

def test_double_lever_tap_queues_two_changes():
    dh = DesireHelper()
    _arm_left(dh)
    # second same-direction lever tap; lamp stays ON the whole time (op-driven)
    dh.update(FakeCarState(left=True, button_events=_left_tap_events()), True, 0.0)
    assert dh.queued_changes == 2
    assert dh.lane_changes_remaining == 1


def test_queue_capped_at_three():
    dh = DesireHelper()
    _arm_left(dh)
    for _ in range(5):  # 5 more lever taps; cap is 3
        dh.update(FakeCarState(left=True, button_events=_left_tap_events()), True, 0.0)
    assert dh.queued_changes == MAX_QUEUED_LANE_CHANGES == 3


def test_same_dir_lever_tap_during_starting_queues_change():
    dh = DesireHelper()
    _arm_left(dh)
    dh.update(_nudge_left(), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting
    assert dh.queued_changes == 1
    # tap the lever again while the maneuver is underway — lamp still ON
    dh.update(FakeCarState(left=True, button_events=_left_tap_events()), True, 0.0)
    assert dh.queued_changes == 2


def test_same_dir_lever_tap_during_finishing_queues_change():
    dh = DesireHelper()
    _arm_left(dh)
    _tick(dh, _nudge_left(), n=int(0.6 / DT_MDL), lane_change_prob=0.0)
    assert dh.lane_change_state == LaneChangeState.laneChangeFinishing
    dh.update(FakeCarState(left=True, button_events=_left_tap_events()), True, 0.0)
    assert dh.queued_changes == 2


def test_tap_during_maneuver_offers_second_change_after_completion():
    # The on-car scenario: tap during the first change -> second change offered
    # (re-armed, awaiting its own nudge) once the first completes.
    dh = DesireHelper()
    _arm_left(dh)
    dh.update(_nudge_left(), True, 0.0)
    dh.update(FakeCarState(left=True, button_events=_left_tap_events()), True, 0.0)
    assert dh.queued_changes == 2
    _complete_maneuver(dh, FakeCarState(left=True))
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    assert dh.lane_change_direction == LaneChangeDirection.left
    assert dh.queued_changes == 1
    # second change still needs its own wheel nudge
    dh.update(_nudge_left(), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting


def test_opposite_lever_tap_during_starting_cancels():
    dh = DesireHelper()
    _arm_left(dh)
    dh.update(_nudge_left(), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting
    # opposite lever tap while the left maneuver runs; lamp still reads left-ON
    dh.update(FakeCarState(left=True, button_events=_right_tap_events()), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.off
    assert dh.lane_change_direction == LaneChangeDirection.none
    assert dh.queued_changes == 0


def test_opposite_lever_tap_during_finishing_cancels():
    dh = DesireHelper()
    _arm_left(dh)
    _tick(dh, _nudge_left(), n=int(0.6 / DT_MDL), lane_change_prob=0.0)
    assert dh.lane_change_state == LaneChangeState.laneChangeFinishing
    dh.update(FakeCarState(left=True, button_events=_right_tap_events()), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.off
    assert dh.lane_change_direction == LaneChangeDirection.none


def test_held_lamp_without_lever_events_does_not_queue():
    # The op-driven lamp (latched ON, no lever events) must never queue changes.
    dh = DesireHelper()
    _arm_left(dh)
    dh.update(_nudge_left(), True, 0.0)
    assert dh.queued_changes == 1
    _tick(dh, FakeCarState(left=True), n=20)
    assert dh.queued_changes == 1


def test_lamp_edges_alone_do_not_queue():
    # Regression guard for the on-car bug class: lamp toggling (e.g. stock
    # flash gaps or op-drive transitions) without lever events must not queue.
    dh = DesireHelper()
    _arm_left(dh)
    for _ in range(3):
        dh.update(FakeCarState(left=False), True, 0.0)
        dh.update(FakeCarState(left=True), True, 0.0)
    assert dh.queued_changes == 1


def test_second_change_rearms_after_first_completes():
    dh = DesireHelper()
    _arm_left(dh)
    dh.update(FakeCarState(left=True, button_events=_left_tap_events()), True, 0.0)  # queue 2nd
    assert dh.queued_changes == 2
    dh.update(_nudge_left(), True, 0.0)
    _complete_maneuver(dh, FakeCarState(left=True))
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    assert dh.lane_change_direction == LaneChangeDirection.left
    assert dh.lane_changes_remaining == 0  # one left, now in the waiting window
    assert dh.queued_changes == 1


def test_midsequence_timeout_full_resets():
    dh = DesireHelper()
    _arm_left(dh)
    dh.update(FakeCarState(left=True, button_events=_left_tap_events()), True, 0.0)
    dh.update(_nudge_left(), True, 0.0)
    _complete_maneuver(dh, FakeCarState(left=True))
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    # let the 2nd window time out with no nudge
    _tick(dh, FakeCarState(left=True), n=int(LANE_CHANGE_ARM_TIME / DT_MDL) + 5)
    assert dh.lane_change_state == LaneChangeState.off
    assert dh.lane_change_direction == LaneChangeDirection.none
    assert dh.queued_changes == 0
