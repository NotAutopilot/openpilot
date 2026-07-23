"""Pre-AP pedal-long regen demand check.

The planner's deceleration request is clamped to the regen envelope
(get_preap_accel_limits) before the car ever sees it, so a demand the envelope
cannot cover is invisible to the pedal controller. This check reads the
unclamped plan and prompts the driver to add friction brake when the demand
sits meaningfully below the envelope floor. The carstate pedalMaxRegen flag
covers the other failure shape: a request inside the envelope that weak
battery regen fails to deliver.
"""
import math

from openpilot.common.realtime import DT_CTRL
from opendbc.car.tesla.preap.interface import get_preap_accel_limits

# Evidence accumulates in a saturating up/down counter so a single MPC sample
# cannot flash a driver prompt, while brief dropouts do not restart the clock.
REGEN_DEMAND_EVIDENCE_COUNT = int(0.3 / DT_CTRL)
REGEN_DEMAND_TRIGGER_MARGIN = 0.2  # m/s² below the envelope floor
REGEN_DEMAND_CLEAR_MARGIN = 0.05  # m/s²
REGEN_DEMAND_MIN_SPEED = 2.0  # m/s; do not prompt for a stopped/settling car
REGEN_DEMAND_CLEAR_SPEED = 1.0  # m/s


class RegenDemandCheck:
  """Prompt when planned deceleration exceeds what the regen envelope allows."""

  def __init__(self):
    self.active = False
    self.evidence_updates = 0

  def reset(self):
    self.active = False
    self.evidence_updates = 0

  def update(self, *, pedal_long_active: bool, brake_pressed: bool,
             a_target: float, v_ego: float) -> bool:
    if not pedal_long_active or brake_pressed or not math.isfinite(a_target):
      self.reset()
      return False

    accel_floor, _ = get_preap_accel_limits(v_ego)

    if self.active:
      keep_prompting = (
        v_ego > REGEN_DEMAND_CLEAR_SPEED
        and a_target <= accel_floor - REGEN_DEMAND_CLEAR_MARGIN
      )
      if not keep_prompting:
        self.reset()
      return self.active

    demanding = (
      v_ego >= REGEN_DEMAND_MIN_SPEED
      and a_target <= accel_floor - REGEN_DEMAND_TRIGGER_MARGIN
    )
    if demanding:
      self.evidence_updates = min(self.evidence_updates + 1, REGEN_DEMAND_EVIDENCE_COUNT)
    else:
      self.evidence_updates = max(self.evidence_updates - 1, 0)
    self.active = self.evidence_updates >= REGEN_DEMAND_EVIDENCE_COUNT
    return self.active
