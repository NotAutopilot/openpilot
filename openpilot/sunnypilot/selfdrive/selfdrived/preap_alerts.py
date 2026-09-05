"""MADS lat-prompt wrappers and leftover CarSpecificEventsSP helpers.

NAP's EventName alert texts now live in selfdrive/selfdrived/events.py.
This file keeps the MADS-only mapping (lkasEnable/lkasDisable) because NAP
has no MADS, plus the symbols CarSpecificEventsSP still imports.
"""
from __future__ import annotations

from dataclasses import dataclass

from openpilot.cereal import log
from opendbc.car.structs import car
from opendbc.car.tesla.preap.carcontroller import PedalAuthorityState
from opendbc.car.tesla.preap.sp.platform import is_preap_platform
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import Alert, EngagementAlert, Priority

AlertSize = log.SelfdriveState.AlertSize
AlertStatus = log.SelfdriveState.AlertStatus
VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = log.SelfdriveState.AudibleAlert


@dataclass(frozen=True)
class PreAPAlertInputs:
  is_preap: bool
  pedal_present: bool
  pedal_calib_available: bool
  pedal_calib_done: bool
  pedal_available: bool
  pedal_timeout: bool
  pedal_authority_state: int
  pedal_authority_failed: bool
  interceptor_state: int
  radar_present: bool
  radar_config_invalid: bool
  radar_fault: bool
  established_authority_lost: bool = False


class PedalAuthorityLossMapper:
  """Alert immediately when established pedal authority is lost.

  Lifecycle:
    INACTIVE: not established. Clears any prior loss.
    ACQUIRING from INACTIVE: initial acquisition. Silent.
    ACTIVE: authority is established.
    ACQUIRING or FAILED after ACTIVE: alert until ACTIVE recovery or INACTIVE release.
  """

  def __init__(self) -> None:
    self._established = False
    self._prev = int(PedalAuthorityState.INACTIVE)

  def reset(self) -> None:
    self._established = False
    self._prev = int(PedalAuthorityState.INACTIVE)

  def update(self, authority_state: int) -> bool:
    state = int(authority_state)
    prev = self._prev
    self._prev = state
    if state == int(PedalAuthorityState.INACTIVE):
      self._established = False
      return False
    if state == int(PedalAuthorityState.ACTIVE):
      self._established = True
      return False
    if self._established and state in (
      int(PedalAuthorityState.ACQUIRING),
      int(PedalAuthorityState.FAILED),
    ):
      return True
    if prev == int(PedalAuthorityState.INACTIVE) and state == int(PedalAuthorityState.ACQUIRING):
      return False
    return False


def preap_alert_inputs_from_snapshot(CP, CP_SP, CS_SP=None, *, radar_fault: bool = False,
                                     established_authority_lost: bool = False) -> PreAPAlertInputs:
  is_preap = bool(CP is not None and is_preap_platform(CP))
  flags = int(getattr(CP_SP, "flags", 0) or 0) if CP_SP is not None else 0
  pedal_present = bool(flags & TeslaFlagsSP.PREAP_PEDAL_PRESENT)
  pedal_calib_available = bool(flags & TeslaFlagsSP.PREAP_PEDAL_CALIB_AVAILABLE)
  radar_present = bool(flags & TeslaFlagsSP.PREAP_RADAR_PRESENT)
  radar_config_invalid = bool(getattr(CP, "radarUnavailable", False)) if CP is not None else False
  authority_state = int(PedalAuthorityState.INACTIVE)
  authority_failed = False
  if CS_SP is not None:
    authority_state = int(getattr(CS_SP, "pedalAuthorityState", 0) or 0)
    authority_failed = bool(getattr(CS_SP, "pedalAuthorityFailed", False))
  return PreAPAlertInputs(
    is_preap=is_preap,
    pedal_present=pedal_present,
    pedal_calib_available=pedal_calib_available,
    pedal_calib_done=pedal_calib_available,
    pedal_available=True,
    pedal_timeout=False,
    pedal_authority_state=int(authority_state),
    pedal_authority_failed=authority_failed,
    interceptor_state=0,
    radar_present=radar_present,
    radar_config_invalid=radar_config_invalid,
    radar_fault=bool(radar_fault),
    established_authority_lost=bool(established_authority_lost),
  )


def select_preap_alerts(inputs: PreAPAlertInputs) -> tuple[int, ...]:
  # Pedal unavailable / not-calibrated now fire as EventName from NAP's
  # CarSpecificEvents hunk. Do not also map them through EventNameSP.
  return ()


def preap_lkas_enable_alert(CP, CS, sm, metric, soft_disable_time, personality) -> Alert:
  if is_preap_platform(CP):
    return Alert(
      "Steering Engaged",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.MID, VisualAlert.none, AudibleAlert.engage, 0.8)
  return EngagementAlert(AudibleAlert.engage)


def preap_lkas_disable_alert(CP, CS, sm, metric, soft_disable_time, personality) -> Alert:
  if is_preap_platform(CP):
    return Alert(
      "Steering Disengaged",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.MID, VisualAlert.none, AudibleAlert.disengage, 0.8)
  return EngagementAlert(AudibleAlert.disengage)


def radar_state_has_fault(radar_state) -> bool:
  """True when production RadarState.radarErrors reports a radar/config fault.

  log.RadarState.radarErrors is Car.RadarData.Error: a struct with canError,
  radarFault, wrongConfig, and radarUnavailableTemporary. It is not a list and
  there is no top-level radarFault. Temporary unavailability is a distinct
  stock event and is not treated as a Pre-AP radarFault.
  """
  if radar_state is None:
    return False
  errors = getattr(radar_state, "radarErrors", None)
  if errors is None or isinstance(errors, (list, tuple, str, bytes)):
    return False
  return bool(
    getattr(errors, "canError", False)
    or getattr(errors, "radarFault", False)
    or getattr(errors, "wrongConfig", False)
  )


def preap_radar_fault(inputs: PreAPAlertInputs) -> bool:
  return bool(inputs.is_preap and inputs.radar_present and (inputs.radar_config_invalid or inputs.radar_fault))


def register_preap_alerts() -> None:
  """Register MADS lat prompts. Pedal EventName texts live in events.py."""
  from openpilot.cereal import custom
  from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP
  from openpilot.sunnypilot.selfdrive.selfdrived.events_base import ET
  EventNameSP = custom.OnroadEventSP.EventName
  EVENTS_SP[EventNameSP.lkasEnable] = {ET.ENABLE: preap_lkas_enable_alert}
  EVENTS_SP[EventNameSP.lkasDisable] = {ET.USER_DISABLE: preap_lkas_disable_alert}
