"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import log, custom
from opendbc.car import structs

from opendbc.car.chrysler.values import RAM_DT
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.selfdrive.car.preap_intent import PreAPIntentConsumer
from openpilot.sunnypilot.selfdrive.selfdrived.preap_alerts import (
  PedalAuthorityLossMapper,
  preap_alert_inputs_from_snapshot,
  preap_radar_fault,
  select_preap_alerts,
)

EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
GearShifter = structs.CarState.GearShifter


class CarSpecificEventsSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.low_speed_alert = False
    self.preap_intent = PreAPIntentConsumer() if CP.carFingerprint == "TESLA_MODEL_S_PREAP" else None
    self._pedal_authority_loss = PedalAuthorityLossMapper()

  def update(self, CS: structs.CarState, events: Events, CS_SP=None, radar_fault: bool = False):
    events_sp = EventsSP()

    # Consume each new Pre-AP intent record before the standard state transition.
    if self.preap_intent is not None:
      if CS_SP is not None:
        self.preap_intent.update(
          CS_SP, events, events_sp,
          apply_longitudinal=bool(self.CP.openpilotLongitudinalControl),
        )
      lost = False
      if CS_SP is not None:
        lost = self._pedal_authority_loss.update(int(getattr(CS_SP, "pedalAuthorityState", 0) or 0))
      else:
        self._pedal_authority_loss.reset()
      inputs = preap_alert_inputs_from_snapshot(
        self.CP, self.CP_SP, CS_SP, radar_fault=radar_fault, established_authority_lost=lost,
      )
      if not inputs.pedal_present:
        self._pedal_authority_loss.reset()
        inputs = preap_alert_inputs_from_snapshot(
          self.CP, self.CP_SP, CS_SP, radar_fault=radar_fault, established_authority_lost=False,
        )
      for name in select_preap_alerts(inputs):
        events_sp.add(name)
      if preap_radar_fault(inputs):
        events.add(EventName.radarFault)

    if self.CP.brand == 'chrysler':
      if self.CP.carFingerprint in RAM_DT:
        # remove belowSteerSpeed event from CarSpecificEvents as RAM_DT uses a different logic
        if events.has(EventName.belowSteerSpeed):
          events.remove(EventName.belowSteerSpeed)

        # TODO-SP: use if/elif to have the gear shifter condition takes precedence over the speed condition
        # TODO-SP: add 1 m/s hysteresis
        if CS.vEgo >= self.CP.minEnableSpeed:
          self.low_speed_alert = False
        if self.CP.minEnableSpeed >= 14.5 and CS.gearShifter != GearShifter.drive:
          self.low_speed_alert = True
      if self.low_speed_alert:
        events.add(EventName.belowSteerSpeed)

    elif self.CP.brand == 'toyota':
      if self.CP.openpilotLongitudinalControl:
        if CS.cruiseState.standstill and not CS.brakePressed and self.CP_SP.enableGasInterceptor:
          if events.has(EventName.resumeRequired):
            events.remove(EventName.resumeRequired)

    return events_sp
