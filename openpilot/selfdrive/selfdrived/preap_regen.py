"""Pre-AP pedal-long regen demand check.

The planner's deceleration request is clamped to the regen envelope
before the car ever sees it, so a demand the envelope cannot cover is
invisible to the pedal controller. This check reads the unclamped plan
and prompts the driver to add friction brake when the demand sits
meaningfully below the envelope floor. CarStateSP.pedalMaxRegen covers
the other failure shape: a request inside the envelope that weak
battery regen fails to deliver.

update_preap_chimes edge-detects lat and long engage/disengage. Long
follows FSM intent (enableLongControl), not interceptor handshake and
not gas override. enableLongControl stays true while the driver is on
the pedal, so press and release are silent.
"""
import math
from typing import NamedTuple

from openpilot.common.realtime import DT_CTRL
from opendbc.car.tesla.preap.constants import get_preap_accel_limits
from openpilot.cereal import custom, log
from opendbc.car.structs import car
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import Alert, ET, Priority
from openpilot.sunnypilot.selfdrive.selfdrived.preap_alerts import (
  ALERT_PEDAL_UNAVAILABLE,
  ALERT_PEDAL_UNAVAILABLE_SUB,
  ALERT_REGEN,
  ALERT_REGEN_SUB,
)

AlertSize = log.SelfdriveState.AlertSize
AlertStatus = log.SelfdriveState.AlertStatus
VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = log.SelfdriveState.AudibleAlert
EventNameSP = custom.OnroadEventSP.EventName

REGEN_DEMAND_EVIDENCE_COUNT = int(0.3 / DT_CTRL)
REGEN_DEMAND_TRIGGER_MARGIN = 0.2  # m/s² below the envelope floor
REGEN_DEMAND_CLEAR_MARGIN = 0.05  # m/s²
REGEN_DEMAND_MIN_SPEED = 2.0  # m/s
REGEN_DEMAND_CLEAR_SPEED = 1.0  # m/s


class PreAPChimeState(NamedTuple):
  lat_engaged: bool = False
  long_engaged: bool = False


class PreAPChimes(NamedTuple):
  lat_engage: bool = False
  lat_disengage: bool = False
  long_engage: bool = False
  long_disengage: bool = False


def update_preap_chimes(*, lat_engaged: bool, long_engaged: bool,
                        prev: PreAPChimeState) -> tuple[PreAPChimes, PreAPChimeState]:
  """Rising/falling edges for Pre-AP lat and long driver prompts."""
  chimes = PreAPChimes(
    lat_engage=lat_engaged and not prev.lat_engaged,
    lat_disengage=(not lat_engaged) and prev.lat_engaged,
    long_engage=long_engaged and not prev.long_engaged,
    long_disengage=(not long_engaged) and prev.long_engaged,
  )
  return chimes, PreAPChimeState(lat_engaged, long_engaged)


class RegenDemandCheck:
  """Prompt when planned deceleration exceeds what the regen envelope allows."""

  def __init__(self):
    self.active = False
    self.evidence_updates = 0

  def reset(self):
    self.active = False
    self.evidence_updates = 0

  def update(self, *, plan_valid: bool, pedal_long_active: bool, brake_pressed: bool,
             a_target: float, v_ego: float) -> bool:
    if not plan_valid or not pedal_long_active or brake_pressed or not math.isfinite(a_target):
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


PREAP_REGEN_ALERT = Alert(
  ALERT_REGEN,
  ALERT_REGEN_SUB,
  AlertStatus.userPrompt, AlertSize.mid,
  Priority.HIGH, VisualAlert.brakePressed, AudibleAlert.prompt, .2,
)

PREAP_PEDAL_CRUISE_ENABLED_ALERT = Alert(
  "Pedal Cruise Engaged",
  "",
  AlertStatus.normal, AlertSize.small,
  Priority.HIGH, VisualAlert.none, AudibleAlert.engage, 0.8,
)

PREAP_PEDAL_CRUISE_DISABLED_ALERT = Alert(
  "Pedal Cruise Disengaged",
  "",
  AlertStatus.normal, AlertSize.small,
  Priority.HIGH, VisualAlert.none, AudibleAlert.disengage, 0.8,
)

PREAP_PEDAL_UNAVAILABLE_ALERT = Alert(
  ALERT_PEDAL_UNAVAILABLE,
  ALERT_PEDAL_UNAVAILABLE_SUB,
  AlertStatus.userPrompt, AlertSize.mid,
  Priority.HIGH, VisualAlert.none, AudibleAlert.warningImmediate, 4.,
)


def register_preap_regen_alerts() -> None:
  from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP
  EVENTS_SP.setdefault(EventNameSP.pedalMaxRegen, {ET.WARNING: PREAP_REGEN_ALERT})
  # ET.PERMANENT so the disengage prompt still plays on the USER_DISABLE frame.
  EVENTS_SP[EventNameSP.pedalCruiseEnabled] = {ET.PERMANENT: PREAP_PEDAL_CRUISE_ENABLED_ALERT}
  EVENTS_SP[EventNameSP.pedalCruiseDisabled] = {ET.PERMANENT: PREAP_PEDAL_CRUISE_DISABLED_ALERT}
  EVENTS_SP.setdefault(EventNameSP.pedalUnavailable, {ET.WARNING: PREAP_PEDAL_UNAVAILABLE_ALERT})
