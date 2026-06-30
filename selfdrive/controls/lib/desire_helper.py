import math

from cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.

# Time (seconds) the lane change stays armed waiting for a wheel nudge. The
# countdown shown in the UI is the ceil of the time remaining. Time-based rather
# than flash-based because on Pre-AP the indicator-light feedback
# (BC_indicatorLStatus) does not reflect openpilot-driven DAS_bodyControls
# flashes, so a flash counter never advances on its own.
LANE_CHANGE_ARM_TIME = 7.0

# Cap on how many same-direction lane changes can be queued from repeated taps.
MAX_QUEUED_LANE_CHANGES = 3

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
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none

    # Time-based arming latch
    self.arm_timer = 0.0
    self.signals_remaining = math.ceil(LANE_CHANGE_ARM_TIME)

    # Multi-lane-change queue. queued_changes counts the lane changes still to
    # perform (the one currently armed/in-progress plus any waiting). It is set
    # from same-direction taps during the arming window (capped). lane_changes_
    # remaining is the UI value: how many are left *beyond* the current one.
    self.queued_changes = 0
    self.lane_changes_remaining = 0

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def _reset(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.arm_timer = 0.0
    self.queued_changes = 0
    self.lane_changes_remaining = 0

  def update(self, carstate, lateral_active, lane_change_prob):
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    # A tap is a rising edge of one_blinker. Detected once here so it can be
    # honored in every active state, not just preLaneChange — the driver may tap
    # to queue another change (same direction) or to cancel (opposite direction)
    # while a maneuver is already underway.
    new_tap = one_blinker and not self.prev_one_blinker
    tap_direction = self.get_lane_change_direction(carstate) if new_tap else LaneChangeDirection.none
    same_dir_tap = new_tap and tap_direction == self.lane_change_direction
    opposite_tap = new_tap and self.lane_change_direction != LaneChangeDirection.none \
                            and tap_direction != self.lane_change_direction

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self._reset()
    else:
      # Taps honored from ANY active state (arming or mid-maneuver):
      #   opposite-direction tap -> cancel everything, blinker off
      #   same-direction tap     -> queue another change (capped)
      just_cancelled = False
      if self.lane_change_state != LaneChangeState.off:
        if opposite_tap:
          self._reset()
          just_cancelled = True
        elif same_dir_tap:
          self.queued_changes = min(self.queued_changes + 1, MAX_QUEUED_LANE_CHANGES)

      # LaneChangeState.off — do NOT re-arm on the same tick an opposite tap just
      # cancelled, otherwise the cancel would immediately re-arm the new direction.
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

        # Count down the arming window. (Tap queue/cancel handled above for all
        # active states.)
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

    # Derived UI values.
    if self.lane_change_state == LaneChangeState.preLaneChange:
      self.signals_remaining = max(math.ceil(LANE_CHANGE_ARM_TIME - self.arm_timer), 0)
    else:
      self.signals_remaining = math.ceil(LANE_CHANGE_ARM_TIME)
    # How many lane changes remain beyond the one currently armed/in progress.
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
