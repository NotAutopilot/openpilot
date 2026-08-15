import unittest
from unittest.mock import MagicMock
from typing import Any, cast

from openpilot.cereal import custom
from opendbc.car import structs
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.mads.mads import HANDS_ON_RESUME_US, ModularAssistiveDrivingSystem

State = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState
EventNameSP = custom.OnroadEventSP.EventName
SafetyModel = structs.CarParams.SafetyModel
GearShifter = structs.CarState.GearShifter


class FakeCS:
  def __init__(self, hands_on_level=0, steer_fault=False, steering_pressed=False, door_open=False):
    self.handsOnLevel = hands_on_level
    self.steerFaultPermanent = steer_fault
    self.steerFaultTemporary = False
    self.steeringPressed = steering_pressed
    self.steeringDisengage = False
    self.doorOpen = door_open
    self.gearShifter = GearShifter.drive
    self.brakePressed = False
    self.regenBraking = False
    self.gasPressed = False
    self.standstill = False
    self.vEgo = 10.0
    self.cruiseState = structs.CarState().cruiseState
    self.cruiseState.available = True
    self.buttonEvents = []


class FakeSM(dict):
  def __init__(self, pandas, panda_updated=True, panda_valid=True):
    super().__init__({"pandaStates": pandas})
    self.updated = {"pandaStates": panda_updated}
    self.panda_valid = panda_valid

  def all_checks(self, services):
    return self.panda_valid and services == ["pandaStates"]


def _panda(inhibited, model=None):
  ps = MagicMock()
  ps.steeringControlInhibited = inhibited
  ps.controlsAllowedLateral = True
  ps.safetyModel = model if model is not None else getattr(SafetyModel, "teslaPreap", SafetyModel.silent)
  return ps


def make_mads(hands_on=True):
  CP = structs.CarParams()
  CP.brand = "tesla"
  CP.carFingerprint = "TESLA_MODEL_S_PREAP"
  CP.passive = False
  CP_SP = structs.CarParamsSP()
  CP_SP.madsCapabilityContractVersion = 1
  CP_SP.madsRequired = True
  CP_SP.madsHandsOnPauseAvailable = hands_on
  CP_SP.madsMainCruiseInputKind = structs.CarParamsSP.MadsMainCruiseInputKind.momentary
  params = MagicMock()
  params.get_bool.side_effect = lambda k: k in ("Mads",)
  params.get.return_value = 0
  sd = MagicMock()
  sd.CP = CP
  sd.CP_SP = CP_SP
  sd.params = params
  sd.events = Events()
  sd.events_sp = EventsSP()
  sd.enabled = False
  sd.enabled_prev = False
  sd.initialized = True
  sd.cs_fresh = True
  sd.CS_prev = structs.CarState()
  sd.sm = FakeSM([_panda(False)])
  mads = ModularAssistiveDrivingSystem(sd)
  mads.enabled_toggle = True
  mads.enabled = True
  mads.active = True
  mads.state_machine.state = State.enabled
  return mads, sd


class TestPreAPHandsOnPause(unittest.TestCase):
  def test_hands_on_level_pauses_not_steering_pressed(self):
    mads, sd = make_mads()
    cs = FakeCS(hands_on_level=0, steering_pressed=True)
    cs.steeringDisengage = True
    sd.sm = FakeSM([_panda(False)])
    mads.update_events(cs)
    self.assertFalse(mads._hands_on_steering_inhibited)
    self.assertFalse(sd.events_sp.has(EventNameSP.silentLkasDisable))

    cs = FakeCS(hands_on_level=2, steering_pressed=False)
    sd.sm = FakeSM([_panda(True)])
    mads.update_events(cs)
    self.assertTrue(mads._hands_on_steering_inhibited)
    self.assertTrue(sd.events_sp.has(EventNameSP.silentLkasDisable))

  def test_missing_hands_on_level_fail_closes(self):
    mads, sd = make_mads()
    cs = FakeCS(hands_on_level=0)
    del cs.handsOnLevel
    sd.sm = FakeSM([_panda(False)])
    mads.update_events(cs)
    self.assertTrue(mads._hands_on_steering_inhibited)
    self.assertTrue(sd.events_sp.has(EventNameSP.silentLkasDisable))

  def test_mismatch_keeps_inhibited(self):
    mads, sd = make_mads()
    cs = FakeCS(hands_on_level=2)
    sd.sm = FakeSM([_panda(False)])
    mads.update_events(cs)
    self.assertTrue(mads._hands_on_steering_inhibited)
    cs = FakeCS(hands_on_level=0)
    sd.sm = FakeSM([_panda(True)])
    mads.update_events(cs)
    self.assertTrue(mads._hands_on_steering_inhibited)
    self.assertTrue(mads._hands_on_clear_timing)

  def test_resume_requires_continuous_1s_fresh_clear(self):
    mads, sd = make_mads()
    ns = {"v": 0}

    def fake_ns():
      return ns["v"]

    import openpilot.sunnypilot.mads.mads as mads_mod
    orig = mads_mod.time.monotonic_ns
    cast(Any, mads_mod.time).monotonic_ns = fake_ns
    try:
      cs = FakeCS(hands_on_level=2)
      sd.sm = FakeSM([_panda(True)])
      mads.update_events(cs)
      self.assertTrue(mads._hands_on_steering_inhibited)

      cs = FakeCS(hands_on_level=0)
      sd.sm = FakeSM([_panda(True)])
      mads.update_events(cs)
      self.assertTrue(mads._hands_on_steering_inhibited)
      self.assertTrue(mads._hands_on_clear_timing)

      ns["v"] = (HANDS_ON_RESUME_US - 1) * 1000
      mads.update_events(cs)
      self.assertTrue(mads._hands_on_steering_inhibited)

      sd.cs_fresh = False
      ns["v"] = (HANDS_ON_RESUME_US + 1) * 1000
      mads.update_events(cs)
      self.assertTrue(mads._hands_on_steering_inhibited)
      self.assertFalse(mads._hands_on_clear_timing)

      sd.cs_fresh = True
      sd.sm = FakeSM([_panda(False)], panda_valid=False)
      mads.update_events(cs)
      self.assertTrue(mads._hands_on_steering_inhibited)

      sd.sm = FakeSM([_panda(True)], panda_valid=True)
      ns["v"] = 0
      mads.update_events(cs)
      ns["v"] = HANDS_ON_RESUME_US * 1000
      mads.update_events(cs)
      self.assertTrue(mads._hands_on_steering_inhibited)
      sd.sm = FakeSM([_panda(False)], panda_valid=True)
      mads.update_events(cs)
      self.assertFalse(mads._hands_on_steering_inhibited)
    finally:
      cast(Any, mads_mod.time).monotonic_ns = orig

  def test_epas_fault_is_not_hands_on_pause(self):
    mads, sd = make_mads()
    cs = FakeCS(hands_on_level=2, steer_fault=True)
    sd.sm = FakeSM([_panda(True)])
    mads.update_events(cs)
    self.assertFalse(mads._hands_on_steering_inhibited)
    self.assertFalse(sd.events_sp.has(EventNameSP.silentLkasDisable))

  def test_restart_reinhibits_from_panda(self):
    restarted, sd2 = make_mads()
    sd2.sm = FakeSM([_panda(True)])
    self.assertFalse(restarted._hands_on_steering_inhibited)
    restarted.enabled = True
    restarted.update_events(FakeCS(hands_on_level=0))
    self.assertTrue(restarted._hands_on_steering_inhibited)

  def test_generic_platform_unchanged(self):
    mads, sd = make_mads(hands_on=False)
    cs = FakeCS(hands_on_level=2)
    sd.sm = FakeSM([_panda(True)])
    mads.update_events(cs)
    self.assertFalse(mads._hands_on_steering_inhibited)
    self.assertFalse(sd.events_sp.has(EventNameSP.silentLkasDisable))


if __name__ == "__main__":
  unittest.main()
