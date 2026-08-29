"""Pre-AP pedal-long helpers used by selfdrived.

RegenDemandCheck reads the unclamped plan and prompts when planned
deceleration sits below the regen envelope floor. The carstate
pedalMaxRegen flag covers the other failure shape: a request inside the
envelope that weak battery regen fails to deliver.

pedal_long_chime binds engage/disengage to enableLongControl edges.
Gas override drops interceptor authority (pedalLongActive) but keeps
enableLongControl, so a pedal press/release must not chime. Brake and
first-pull drop enableLongControl while cruise stays, and those must.
"""
import math

from openpilot.common.realtime import DT_CTRL
from opendbc.car.tesla.preap.interface import get_preap_accel_limits

# Evidence accumulates in a saturating up/down counter so a single MPC sample
# cannot flash a driver prompt, while brief dropouts do not restart the clock.
REGEN_DEMAND_EVIDENCE_COUNT = int(0.2 / DT_CTRL)
REGEN_DEMAND_TRIGGER_MARGIN = 0.1  # m/s² below the envelope floor
REGEN_DEMAND_CLEAR_MARGIN = 0.03  # m/s²
REGEN_DEMAND_MIN_SPEED = 2.0  # m/s; do not prompt for a stopped/settling car
REGEN_DEMAND_CLEAR_SPEED = 1.0  # m/s

PEDAL_CHIME_ENABLED = "enabled"
PEDAL_CHIME_DISABLED = "disabled"


def pedal_long_chime(*, enable_long_control: bool, prev_enable_long_control: bool) -> str | None:
  """Engage/disengage chime follows the FSM long-control flag, not authority."""
  if enable_long_control and not prev_enable_long_control:
    return PEDAL_CHIME_ENABLED
  if prev_enable_long_control and not enable_long_control:
    return PEDAL_CHIME_DISABLED
  return None


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
