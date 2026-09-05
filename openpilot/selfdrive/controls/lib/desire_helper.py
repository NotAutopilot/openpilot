import math

from openpilot.cereal import log, custom
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeController, AutoLaneChangeMode
from openpilot.sunnypilot.selfdrive.controls.lib.lane_turn_desire import LaneTurnController

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
TurnDirection = custom.ModelDataV2SP.TurnDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.
LANE_CHANGE_START_TIME = 0.5

# Time the lane change stays armed waiting for steering input.
LANE_CHANGE_ARM_TIME = 7.0

# Cap on how many same-direction lane changes can be queued from repeated taps.
MAX_QUEUED_LANE_CHANGES = 3

TURN_DESIRES = {
  TurnDirection.none: log.Desire.none,
  TurnDirection.turnLeft: log.Desire.turnLeft,
  TurnDirection.turnRight: log.Desire.turnRight,
}

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.none,
    LaneChangeState.laneChangeFinishing: log.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeRight,
  },
}


class DesireHelper:
  def __init__(self, CP=None):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none

    self.arm_timer = 0.0
    self.signals_remaining = math.ceil(LANE_CHANGE_ARM_TIME)

    # Lever history is independent of lane-change state. Resetting it would
    # turn a held lever into a second tap.
    self.prev_turn_lever = 0

    # Includes the lane change currently armed or in progress.
    self.queued_changes = 0
    self.lane_changes_remaining = 0

    self.alc = AutoLaneChangeController(self)
    self.lane_turn_controller = LaneTurnController(self)
    self.lane_turn_direction = TurnDirection.none
    # nap-dev lane change for the Pre-AP car, sunnypilot auto lane change for
    # everything else.
    self._preap = CP is not None and CP.carFingerprint == "TESLA_MODEL_S_PREAP"

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def _reset(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.arm_timer = 0.0
    self.queued_changes = 0
    self.lane_changes_remaining = 0

  def _update_preap(self, carstate, lateral_active, lane_change_prob):
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    # The indicator lamp stays on while openpilot drives it, so driver taps
    # must be detected from the physical lever. The lever is a level signal and
    # remains asserted long enough to survive conflated carState reads.
    lever = carstate.turnSignalStalkState
    left_tap = lever == 1 and self.prev_turn_lever != 1
    right_tap = lever == 2 and self.prev_turn_lever != 2
    self.prev_turn_lever = lever

    if self.lane_change_direction == LaneChangeDirection.left:
      same_direction_tap, opposite_direction_tap = left_tap, right_tap
    elif self.lane_change_direction == LaneChangeDirection.right:
      same_direction_tap, opposite_direction_tap = right_tap, left_tap
    else:
      same_direction_tap, opposite_direction_tap = False, False

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self._reset()
    else:
      just_cancelled = False
      if self.lane_change_state != LaneChangeState.off:
        if opposite_direction_tap:
          self._reset()
          just_cancelled = True
        elif same_direction_tap:
          self.queued_changes = min(self.queued_changes + 1, MAX_QUEUED_LANE_CHANGES)

      # LaneChangeState.off
      if not just_cancelled and self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self.get_lane_change_direction(carstate)
        # Start a fresh arming window. The first tap queues one change.
        self.arm_timer = 0.0
        self.queued_changes = 1

      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

        self.arm_timer += DT_MDL

        if below_lane_change_speed:
          self._reset()
        elif torque_applied and not blindspot_detected:
          self.lane_change_state = LaneChangeState.laneChangeStarting
        elif self.arm_timer > LANE_CHANGE_ARM_TIME:
          # Window expired with no wheel nudge — cancel everything.
          self._reset()

      # LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        # fade out over .5s
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)

        # 98% certainty
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)

        if self.lane_change_ll_prob > 0.99:
          # One change just completed.
          self.queued_changes = max(self.queued_changes - 1, 0)
          if self.queued_changes > 0:
            # More queued: keep the same direction and signal on, re-arm a fresh
            # window and wait for the next wheel nudge.
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_ll_prob = 1.0
            self.arm_timer = 0.0
          else:
            # Nothing left — full reset so the toast clears and no ALC re-arms.
            self._reset()

    # Retain the existing metadata even though the stock alert text no longer
    # displays the countdown or queue depth.
    if self.lane_change_state == LaneChangeState.preLaneChange:
      self.signals_remaining = max(math.ceil(LANE_CHANGE_ARM_TIME - self.arm_timer), 0)
    else:
      self.signals_remaining = math.ceil(LANE_CHANGE_ARM_TIME)
    self.lane_changes_remaining = max(self.queued_changes - 1, 0)

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.prev_one_blinker = one_blinker

    self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # Send keep pulse once per second during LaneChangeState.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none

  def _update_sp(self, carstate, lateral_active, lane_change_prob, left_edge_detected, right_edge_detected):
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX or self.alc.lane_change_set_timer == AutoLaneChangeMode.OFF:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
    else:
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_timer = 0.0
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self.get_lane_change_direction(carstate)

      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Update lane change direction
        self.lane_change_direction = self.get_lane_change_direction(carstate)

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        blindspot_detected = (((carstate.leftBlindspot or left_edge_detected) and self.lane_change_direction == LaneChangeDirection.left) or
                              ((carstate.rightBlindspot or right_edge_detected) and self.lane_change_direction == LaneChangeDirection.right))

        self.alc.update_lane_change(blindspot_detected, carstate.brakePressed)

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0
        elif (torque_applied or self.alc.auto_lane_change_allowed) and not blindspot_detected:
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self.lane_change_timer = 0.0

      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_timer += DT_MDL

        if lane_change_prob < 0.02 and self.lane_change_timer >= LANE_CHANGE_START_TIME:
          self.lane_change_timer = 0.0
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_direction = self.get_lane_change_direction(carstate)
          else:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none

    self.prev_one_blinker = one_blinker and lateral_active

    if self.lane_change_state == LaneChangeState.laneChangeStarting:
      if self.lane_change_direction == LaneChangeDirection.left:
        self.desire = log.Desire.laneChangeLeft
      elif self.lane_change_direction == LaneChangeDirection.right:
        self.desire = log.Desire.laneChangeRight
      else:
        self.desire = log.Desire.none
    else:
      self.desire = log.Desire.none

  def update(self, carstate, lateral_active, lane_change_prob, left_edge_detected=False, right_edge_detected=False):
    self.alc.update_params()
    self.lane_turn_controller.update_params()

    # Lane turn controller update
    self.lane_turn_controller.update_lane_turn(blindspot_left=carstate.leftBlindspot, blindspot_right=carstate.rightBlindspot,
                                               left_blinker=carstate.leftBlinker, right_blinker=carstate.rightBlinker, v_ego=carstate.vEgo)
    self.lane_turn_direction = self.lane_turn_controller.get_turn_direction()

    if self._preap:
      self._update_preap(carstate, lateral_active, lane_change_prob)
    else:
      self._update_sp(carstate, lateral_active, lane_change_prob, left_edge_detected, right_edge_detected)

    if self.lane_turn_direction != TurnDirection.none:
      self.desire = TURN_DESIRES[self.lane_turn_direction]

    self.alc.update_state()
