"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import time

from openpilot.cereal import log, custom

from opendbc.car import structs
from opendbc.car.tesla.preap.sp.platform import is_preap_platform
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP
from opendbc.car.hyundai.values import HyundaiFlags
from openpilot.common.params import Params
from openpilot.selfdrive.selfdrived.events import ET
from openpilot.sunnypilot.mads.helpers import (
  MadsSteeringModeOnBrake, persist_required_mads, read_steering_mode_param, resolve_mads_capabilities,
  unified_engagement_locked_off,
)
from openpilot.sunnypilot.mads.state import StateMachine, GEARS_ALLOW_PAUSED_SILENT

State = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState
ButtonType = structs.CarState.ButtonEvent.Type
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
GearShifter = structs.CarState.GearShifter
SafetyModel = structs.CarParams.SafetyModel

SET_SPEED_BUTTONS = (ButtonType.accelCruise, ButtonType.resumeCruise, ButtonType.decelCruise, ButtonType.setCruise)
IGNORED_SAFETY_MODES = (SafetyModel.silent, SafetyModel.noOutput)
HANDS_ON_PAUSE_LEVEL = 2
HANDS_ON_RESUME_US = 1_000_000
UINT32_MASK = 0xFFFFFFFF


def _mono_us() -> int:
  return (time.monotonic_ns() // 1000) & UINT32_MASK


def _elapsed_us(now: int, start: int) -> int:
  return (now - start) & UINT32_MASK


class ModularAssistiveDrivingSystem:
  def __init__(self, selfdrive):
    self.CP = selfdrive.CP
    self.CP_SP = selfdrive.CP_SP
    self.params = selfdrive.params

    self.enabled = False
    self.active = False
    self.available = False
    self.lateral_mismatch_counter = 0
    self.allow_always = False
    self.no_main_cruise = False
    self.selfdrive = selfdrive
    self.selfdrive.enabled_prev = False
    self.state_machine = StateMachine(self)
    self.events = self.selfdrive.events
    self.events_sp = self.selfdrive.events_sp
    self.disengage_on_accelerator = Params().get_bool("DisengageOnAccelerator")
    if self.CP.brand == "hyundai":
      if self.CP.flags & (HyundaiFlags.HAS_LDA_BUTTON | HyundaiFlags.CANFD):
        self.allow_always = True
    if self.CP.brand == "tesla":
      self.allow_always = True

    caps = resolve_mads_capabilities(self.CP, self.CP_SP, self.params)
    self.no_main_cruise = caps.no_main_cruise
    self._freeze_mads_snapshot = (
      caps.mads_required and getattr(self.CP_SP, "madsCapabilityContractVersion", 0) >= 1
    )
    self._hands_on_pause_available = (
      is_preap_platform(self.CP)
      and bool(getattr(self.CP_SP, "madsHandsOnPauseAvailable", False))
      and bool(int(getattr(self.CP_SP, "flags", 0) or 0) & int(TeslaFlagsSP.PREAP_HANDS_ON_PAUSE))
    )
    self._hands_on_steering_inhibited = False
    self._hands_on_clear_timing = False
    self._hands_on_clear_ts = 0

    # Required-MADS platforms consume only the frozen typed snapshot.
    if persist_required_mads(self.params, self.CP_SP):
      self.enabled_toggle = True
    else:
      self.enabled_toggle = self.params.get_bool("Mads")
    self.steering_mode_on_brake = read_steering_mode_param(self.CP, self.CP_SP, self.params)
    if self._freeze_mads_snapshot:
      self.main_enabled_toggle = caps.main_cruise_allowed
      self.unified_engagement_mode = caps.unified_engagement_mode
    else:
      self.main_enabled_toggle = self.params.get_bool("MadsMainCruiseAllowed")
      self.unified_engagement_mode = self.params.get_bool("MadsUnifiedEngagementMode")
    if unified_engagement_locked_off(self.CP, self.CP_SP):
      self.unified_engagement_mode = False

  def read_params(self):
    if persist_required_mads(self.params, self.CP_SP):
      self.enabled_toggle = True
      if self._freeze_mads_snapshot:
        if unified_engagement_locked_off(self.CP, self.CP_SP):
          self.unified_engagement_mode = False
        return
    else:
      self.enabled_toggle = self.params.get_bool("Mads")
    self.main_enabled_toggle = self.params.get_bool("MadsMainCruiseAllowed")
    if unified_engagement_locked_off(self.CP, self.CP_SP):
      self.unified_engagement_mode = False
    else:
      self.unified_engagement_mode = self.params.get_bool("MadsUnifiedEngagementMode")

  def pedal_pressed_non_gas_pressed(self, CS: structs.CarState) -> bool:
    # ignore `pedalPressed` events caused by gas presses
    if self.events.has(EventName.pedalPressed) and not (CS.gasPressed and not self.selfdrive.CS_prev.gasPressed and self.disengage_on_accelerator):
      return True

    return False

  def should_silent_lkas_enable(self, CS: structs.CarState) -> bool:
    if self._hands_on_steering_inhibited:
      return False

    if self.steering_mode_on_brake == MadsSteeringModeOnBrake.PAUSE and (CS.brakePressed or CS.regenBraking or self.pedal_pressed_non_gas_pressed(CS)):
      return False

    if self.events_sp.contains_in_list(GEARS_ALLOW_PAUSED_SILENT):
      return False

    if self._hands_on_pause_available:
      if self.events_sp.has(EventNameSP.lkasDisable):
        return False
      if self.events.has(EventName.driverUnresponsive3) or self.events.has(EventName.driverDistracted3):
        return False
      if self.events.contains(ET.NO_ENTRY) or self.events.contains(ET.IMMEDIATE_DISABLE):
        return False
      if self.events_sp.contains(ET.NO_ENTRY) or self.events_sp.contains(ET.IMMEDIATE_DISABLE):
        return False

    return True

  def block_unified_engagement_mode(self) -> bool:
    # UEM disabled
    if not self.unified_engagement_mode:
      return True

    if self.enabled:
      return True

    if self.selfdrive.enabled and self.selfdrive.enabled_prev:
      return True

    return False

  def get_wrong_car_mode(self, alert_only: bool) -> None:
    if alert_only:
      if self.events.has(EventName.wrongCarMode):
        self.replace_event(EventName.wrongCarMode, EventNameSP.wrongCarModeAlertOnly)
    else:
      self.events.remove(EventName.wrongCarMode)

  def transition_paused_state(self):
    if self.state_machine.state != State.paused:
      self.events_sp.add(EventNameSP.silentLkasDisable)

  def _reset_hands_on_clear_timer(self):
    self._hands_on_clear_timing = False
    self._hands_on_clear_ts = 0

  def _hard_disable_from_pause(self) -> None:
    self._hands_on_steering_inhibited = False
    self._reset_hands_on_clear_timer()
    if self.events_sp.has(EventNameSP.silentLkasDisable):
      self.events_sp.remove(EventNameSP.silentLkasDisable)
    if self.events_sp.has(EventNameSP.silentLkasEnable):
      self.events_sp.remove(EventNameSP.silentLkasEnable)
    if self.enabled or self.state_machine.state == State.paused:
      if not self.events_sp.has(EventNameSP.lkasDisable):
        self.events_sp.add(EventNameSP.lkasDisable)

  def _update_hands_on_pause(self, CS: structs.CarState) -> None:
    if not self._hands_on_pause_available:
      return

    tesla_preap = getattr(SafetyModel, "teslaPreap", None)
    pandas = []
    for ps in self.selfdrive.sm['pandaStates']:
      if tesla_preap is not None and ps.safetyModel == tesla_preap:
        pandas.append(ps)
      elif ps.safetyModel not in IGNORED_SAFETY_MODES and getattr(ps, "steeringControlInhibited", False):
        pandas.append(ps)

    if hasattr(self.selfdrive.sm, "all_checks"):
      panda_fresh = bool(self.selfdrive.sm.all_checks(["pandaStates"]))
    else:
      updated = getattr(self.selfdrive.sm, "updated", None)
      panda_fresh = True if updated is None else bool(updated.get("pandaStates", False))
    cs_fresh = bool(getattr(self.selfdrive, "cs_fresh", True))
    panda_inhibited = (not pandas) or any(bool(getattr(ps, "steeringControlInhibited", False)) for ps in pandas)
    panda_lat_lost = bool(pandas) and any(not bool(getattr(ps, "controlsAllowedLateral", True)) for ps in pandas)

    epas_fault = bool(getattr(CS, "steerFaultPermanent", False) or getattr(CS, "steerFaultTemporary", False))
    door_or_gear = bool(getattr(CS, "doorOpen", False) or CS.gearShifter != GearShifter.drive)
    dm_lock = bool(
      self.events.has(EventName.driverUnresponsive3) or self.events.has(EventName.driverDistracted3)
    )
    hard_event = bool(
      epas_fault
      or self.events_sp.has(EventNameSP.lkasDisable)
      or door_or_gear
      or (not cs_fresh)
      or (not pandas)
      or (not panda_fresh)
      or panda_lat_lost
      or dm_lock
      or self.events_sp.has(EventNameSP.controlsMismatchLateral)
      or self.events.contains(ET.IMMEDIATE_DISABLE)
      or self.events_sp.contains(ET.IMMEDIATE_DISABLE)
    )
    if hard_event:
      self._hard_disable_from_pause()
      return

    if not hasattr(CS, "handsOnLevel"):
      self._hands_on_steering_inhibited = True
      self._reset_hands_on_clear_timer()
      if self.enabled:
        self.transition_paused_state()
        if self.events_sp.has(EventNameSP.silentLkasEnable):
          self.events_sp.remove(EventNameSP.silentLkasEnable)
      return

    level = int(CS.handsOnLevel or 0)
    host_hands = level >= HANDS_ON_PAUSE_LEVEL
    host_clear = level < HANDS_ON_PAUSE_LEVEL

    if host_hands or panda_inhibited:
      self._hands_on_steering_inhibited = True
    if host_hands != panda_inhibited:
      self._hands_on_steering_inhibited = True

    host_clear_healthy = (
      self._hands_on_steering_inhibited and host_clear and cs_fresh and
      panda_fresh and bool(pandas) and (not door_or_gear) and (not dm_lock)
    )
    if not host_clear_healthy:
      self._reset_hands_on_clear_timer()
    else:
      now = _mono_us()
      if not self._hands_on_clear_timing:
        self._hands_on_clear_timing = True
        self._hands_on_clear_ts = now
      elif _elapsed_us(now, self._hands_on_clear_ts) >= HANDS_ON_RESUME_US and not panda_inhibited:
        self._hands_on_steering_inhibited = False
        self._reset_hands_on_clear_timer()

    if self._hands_on_steering_inhibited and self.enabled:
      if not self.events_sp.has(EventNameSP.lkasDisable):
        self.transition_paused_state()
      if self.events_sp.has(EventNameSP.silentLkasEnable):
        self.events_sp.remove(EventNameSP.silentLkasEnable)

  def replace_event(self, old_event: int, new_event: int):
    self.events.remove(old_event)
    self.events_sp.add(new_event)

  def data_sample(self):
    # When the safety and selfdrived do not agree on controls_allowed_lateral
    # we want to disengage sunnypilot. However the status from the panda goes through
    # another socket other than the CAN messages and one can arrive earlier than the other.
    # Therefore we allow a mismatch for two samples, then we trigger the disengagement.
    if not self.active or self.selfdrive.enabled:
      self.lateral_mismatch_counter = 0
    elif any(not ps.controlsAllowedLateral for ps in self.selfdrive.sm['pandaStates']
             if ps.safetyModel not in IGNORED_SAFETY_MODES):
      self.lateral_mismatch_counter += 1

  def update_events(self, CS: structs.CarState):
    if not self.selfdrive.enabled and self.enabled:
      preap = is_preap_platform(self.CP)
      if CS.standstill:
        if self.events.has(EventName.doorOpen):
          if preap:
            self.events_sp.add(EventNameSP.lkasDisable)
          else:
            self.replace_event(EventName.doorOpen, EventNameSP.silentDoorOpen)
            self.transition_paused_state()
        if self.events.has(EventName.seatbeltNotLatched):
          self.replace_event(EventName.seatbeltNotLatched, EventNameSP.silentSeatbeltNotLatched)
          self.transition_paused_state()
      if self.events.has(EventName.wrongGear) and (CS.vEgo < 2.5 or CS.gearShifter == GearShifter.reverse):
        if preap:
          self.events_sp.add(EventNameSP.lkasDisable)
        else:
          self.replace_event(EventName.wrongGear, EventNameSP.silentWrongGear)
          self.transition_paused_state()
      if self.events.has(EventName.reverseGear):
        if preap:
          self.events_sp.add(EventNameSP.lkasDisable)
        else:
          self.replace_event(EventName.reverseGear, EventNameSP.silentReverseGear)
          self.transition_paused_state()
      if self.events.has(EventName.brakeHold):
        self.replace_event(EventName.brakeHold, EventNameSP.silentBrakeHold)
        self.transition_paused_state()
      if self.events.has(EventName.parkBrake):
        self.replace_event(EventName.parkBrake, EventNameSP.silentParkBrake)
        self.transition_paused_state()

      if self.steering_mode_on_brake == MadsSteeringModeOnBrake.PAUSE:
        if self.pedal_pressed_non_gas_pressed(CS):
          self.transition_paused_state()

      self.events.remove(EventName.preEnableStandstill)
      self.events.remove(EventName.belowEngageSpeed)
      self.events.remove(EventName.speedTooLow)
      self.events.remove(EventName.cruiseDisabled)
      self.events.remove(EventName.manualRestart)
      self.events.remove(EventName.espActive)

    selfdrive_enable_events = self.events.has(EventName.pcmEnable) or self.events.has(EventName.buttonEnable)
    set_speed_btns_enable = any(be.type in SET_SPEED_BUTTONS for be in CS.buttonEvents)

    # wrongCarMode alert only or actively block control
    self.get_wrong_car_mode(selfdrive_enable_events or set_speed_btns_enable)

    if selfdrive_enable_events:
      if self.pedal_pressed_non_gas_pressed(CS):
        self.events_sp.add(EventNameSP.pedalPressedAlertOnly)

      if self.block_unified_engagement_mode():
        self.events.remove(EventName.pcmEnable)
        self.events.remove(EventName.buttonEnable)
    else:
      # Stateful MAIN only. Momentary Pre-AP must not treat cruiseState.available as a stalk.
      if self.main_enabled_toggle and not self.no_main_cruise:
        if CS.cruiseState.available and not self.selfdrive.CS_prev.cruiseState.available:
          self.events_sp.add(EventNameSP.lkasEnable)

    for be in CS.buttonEvents:
      if be.type == ButtonType.cancel:
        if not self.selfdrive.enabled and self.selfdrive.enabled_prev:
          self.events_sp.add(EventNameSP.manualLongitudinalRequired)
      if be.type == ButtonType.lkas and be.pressed and (CS.cruiseState.available or self.allow_always):
        if self.enabled:
          if self.selfdrive.enabled:
            self.events_sp.add(EventNameSP.manualSteeringRequired)
          else:
            self.events_sp.add(EventNameSP.lkasDisable)
        else:
          self.events_sp.add(EventNameSP.lkasEnable)

    if not CS.cruiseState.available and not self.no_main_cruise:
      self.events.remove(EventName.buttonEnable)
      if self.selfdrive.CS_prev.cruiseState.available:
        self.events_sp.add(EventNameSP.lkasDisable)

    if self.steering_mode_on_brake == MadsSteeringModeOnBrake.DISENGAGE:
      if self.pedal_pressed_non_gas_pressed(CS):
        if self.enabled:
          self.events_sp.add(EventNameSP.lkasDisable)
        else:
          # block lkasEnable if being sent, then send pedalPressedAlertOnly event
          if self.events_sp.contains(EventNameSP.lkasEnable):
            self.events_sp.remove(EventNameSP.lkasEnable)
            self.events_sp.add(EventNameSP.pedalPressedAlertOnly)

    if self._hands_on_pause_available and not self.enabled:
      if int(getattr(CS, "handsOnLevel", 0) or 0) >= HANDS_ON_PAUSE_LEVEL:
        if self.events_sp.has(EventNameSP.lkasEnable):
          self.events_sp.remove(EventNameSP.lkasEnable)

    self._update_hands_on_pause(CS)
    if is_preap_platform(self.CP):
      if self.events_sp.has(EventNameSP.lkasDisable) or self.events.contains(ET.IMMEDIATE_DISABLE) or self.events_sp.contains(ET.IMMEDIATE_DISABLE):
        if self.events_sp.has(EventNameSP.silentLkasDisable):
          self.events_sp.remove(EventNameSP.silentLkasDisable)
        if self.events_sp.has(EventNameSP.silentLkasEnable):
          self.events_sp.remove(EventNameSP.silentLkasEnable)


    if self.should_silent_lkas_enable(CS):
      if self.state_machine.state == State.paused:
        self.events_sp.add(EventNameSP.silentLkasEnable)

    if self.lateral_mismatch_counter >= 200:
      self.events_sp.add(EventNameSP.controlsMismatchLateral)

    self.events.remove(EventName.pcmDisable)
    self.events.remove(EventName.buttonCancel)
    self.events.remove(EventName.pedalPressed)
    self.events.remove(EventName.wrongCruiseMode)

  def update(self, CS: structs.CarState):
    if persist_required_mads(self.params, self.CP_SP):
      self.enabled_toggle = True
    if not self.enabled_toggle:
      return

    self.data_sample()

    self.update_events(CS)

    if not self.CP.passive and self.selfdrive.initialized:
      self.enabled, self.active = self.state_machine.update()

    # Copy of previous SelfdriveD states for MADS events handling
    self.selfdrive.enabled_prev = self.selfdrive.enabled
