"""Pre-AP alert policy for EventsSP.

custom.capnp is frozen: EventNameSP ends at @28. Pre-AP alert states map onto
those existing enumerants instead of adding schema.

  pedalUnavailable: uncalibrated/timeout/authority-loss/interceptor faults
  pedalMaxRegen: regen demand (registered in preap_regen)
  radarFault: stock EventName.radarFault for Pre-AP radar failures
"""
from __future__ import annotations

from dataclasses import dataclass

from openpilot.cereal import custom
from opendbc.car.tesla.preap.boot import is_preap_platform
from opendbc.car.tesla.preap.constants import PEDAL_FEEDBACK_TIMEOUT_STATE
from opendbc.car.tesla.preap.carcontroller import PedalAuthorityState
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP
from openpilot.system.ui.lib.multilang import tr_noop

EventNameSP = custom.OnroadEventSP.EventName

# Extracted via existing tr()/tr_noop PO conventions. Display strings are the
# registered EventsSP texts for the mapped EventNameSP entries, and the stock
# radarFault text dispatched for Pre-AP radar failures.
ALERT_PEDAL_UNAVAILABLE = tr_noop("Pedal Unavailable")
ALERT_PEDAL_UNAVAILABLE_SUB = tr_noop("Stock cruise required")
ALERT_REGEN = tr_noop("Regen Limit Reached")
ALERT_REGEN_SUB = tr_noop("Press Brake to Slow Down")
ALERT_RADAR_FAULT = tr_noop("Radar Error: Restart the Car")


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
  pedal_calib_done = pedal_calib_available
  pedal_timeout = False
  interceptor_state = 0
  authority_state = int(PedalAuthorityState.INACTIVE)
  authority_failed = False
  if CS_SP is not None:
    authority_state = int(getattr(CS_SP, "pedalAuthorityState", 0) or 0)
    authority_failed = bool(getattr(CS_SP, "pedalAuthorityFailed", False))
    interceptor_state = int(getattr(CS_SP, "pedalFeedbackState", 0) or 0)
    pedal_timeout = interceptor_state == PEDAL_FEEDBACK_TIMEOUT_STATE
    if pedal_timeout:
      interceptor_state = 0
  return PreAPAlertInputs(
    is_preap=is_preap,
    pedal_present=pedal_present,
    pedal_calib_available=pedal_calib_available,
    pedal_calib_done=bool(pedal_calib_done),
    pedal_available=not pedal_timeout and interceptor_state == 0,
    pedal_timeout=bool(pedal_timeout),
    pedal_authority_state=int(authority_state),
    pedal_authority_failed=authority_failed,
    interceptor_state=int(interceptor_state),
    radar_present=radar_present,
    radar_config_invalid=radar_config_invalid,
    radar_fault=bool(radar_fault),
    established_authority_lost=bool(established_authority_lost),
  )


def _pedal_unusable(inputs: PreAPAlertInputs) -> bool:
  return bool(
    not inputs.pedal_calib_done
    or not inputs.pedal_calib_available
    or inputs.pedal_authority_failed
    or inputs.pedal_authority_state == int(PedalAuthorityState.FAILED)
    or inputs.established_authority_lost
    or inputs.pedal_timeout
    or not inputs.pedal_available
    or inputs.interceptor_state != 0
  )


def select_preap_alerts(inputs: PreAPAlertInputs) -> tuple[int, ...]:
  if not inputs.is_preap:
    return ()

  events: list[int] = []
  if inputs.pedal_present and _pedal_unusable(inputs):
    events.append(EventNameSP.pedalUnavailable)
  return tuple(events)


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
  """Register mapped EventNameSP alerts before EventsSP snapshots EVENTS_SP keys.

  Extra Pre-AP pedal states (invalid calibration, timeout, authority loss,
  interceptor faults) reuse frozen EventNameSP.pedalUnavailable @28. Regen
  registration also setdefaults that enumerant plus pedalMaxRegen.
  """
  from openpilot.selfdrive.selfdrived.preap_regen import (
    PREAP_PEDAL_UNAVAILABLE_ALERT,
    register_preap_regen_alerts,
  )
  from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP
  from openpilot.sunnypilot.selfdrive.selfdrived.events_base import ET
  EVENTS_SP.setdefault(EventNameSP.pedalUnavailable, {ET.WARNING: PREAP_PEDAL_UNAVAILABLE_ALERT})
  register_preap_regen_alerts()
