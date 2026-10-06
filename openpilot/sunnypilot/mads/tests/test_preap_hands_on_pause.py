import unittest
from unittest.mock import MagicMock, patch
from typing import Any, cast

from openpilot.cereal import custom, log
from opendbc.car import structs
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.mads.mads import HANDS_ON_RESUME_US, LATERAL_MISMATCH_MAX_COUNT, ModularAssistiveDrivingSystem

State = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState
EventName = log.OnroadEvent.EventName
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


def make_mads(hands_on=True, capability=True, safety_param=None):
  CP = structs.CarParams()
  CP.brand = "tesla"
  CP.carFingerprint = "TESLA_MODEL_S_PREAP"
  CP.passive = False
  CP_SP = structs.CarParamsSP()
  CP_SP.madsCapabilityContractVersion = 1
  CP_SP.madsRequired = True
  CP_SP.madsHandsOnPauseAvailable = capability
  CP_SP.flags = int(TeslaFlagsSP.PREAP_HANDS_ON_PAUSE) if hands_on else 0
  CP_SP.madsMainCruiseInputKind = structs.CarParamsSP.MadsMainCruiseInputKind.momentary
  if safety_param is None:
    safety_param = 8 if hands_on else 0
  safety = structs.CarParams.SafetyConfig()
  safety.safetyParam = safety_param
  CP.safetyConfigs = [safety]
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
  sd.state_machine = MagicMock()
  sd.state_machine.current_alert_types = []
  sd.state_machine.soft_disable_timer = 100
  mads = ModularAssistiveDrivingSystem(sd)
  mads.enabled_toggle = True
  mads.enabled = True
  mads.active = True
  mads.state_machine.state = State.enabled
  return mads, sd

def _disengage(mads):
  mads.enabled = False
  mads.active = False
  mads.state_machine.state = State.disabled


def _step_mads(mads, sd, cs, *, allowed=True, inhibited=False, panda_valid=True, enable=False, override=False):
  ps = _panda(inhibited)
  ps.controlsAllowedLateral = allowed
  sd.sm = FakeSM([ps], panda_valid=panda_valid)
  sd.events.clear()
  sd.events_sp.clear()
  if enable:
    sd.events_sp.add(EventNameSP.lkasEnable)
  if override:
    sd.events.add(EventName.steerOverride)
  mads.update(cs)
  sd.CS_prev = cs



class TestPreAPHandsOnPause(unittest.TestCase):
  def test_deliberate_entry_with_hands_starts_paused_without_active_frame(self):
    for safety_param, level in ((11, 3), (8 | (1 << 8), 1), (8 | (3 << 8), 3)):
      with self.subTest(safety_param=safety_param, level=level):
        mads, sd = make_mads(safety_param=safety_param)
        _disengage(mads)
        _step_mads(mads, sd, FakeCS(hands_on_level=level), inhibited=True, enable=True)
        self.assertTrue(mads.enabled)
        self.assertFalse(mads.active)
        self.assertTrue(mads.hands_on_paused)
        self.assertEqual(mads.state_machine.state, State.paused)
        _step_mads(mads, sd, FakeCS(hands_on_level=level), inhibited=True, enable=True)
        self.assertFalse(mads.active)

  def test_admitted_pause_resumes_only_after_fresh_clear_hold(self):
    mads, sd = make_mads(safety_param=11)
    _disengage(mads)
    with patch("openpilot.sunnypilot.mads.mads.time.monotonic_ns") as now:
      now.return_value = 0
      _step_mads(mads, sd, FakeCS(hands_on_level=3), inhibited=True, enable=True)
      _step_mads(mads, sd, FakeCS(), inhibited=True)
      now.return_value = (HANDS_ON_RESUME_US - 1) * 1000
      _step_mads(mads, sd, FakeCS(), inhibited=False)
      self.assertFalse(mads.active)
      self.assertTrue(mads.hands_on_paused)
      now.return_value = HANDS_ON_RESUME_US * 1000
      _step_mads(mads, sd, FakeCS(), inhibited=False)
      # Resume event is evaluated before the hands-clear latch is updated.
      _step_mads(mads, sd, FakeCS(), inhibited=False)
      self.assertTrue(mads.active)
      self.assertFalse(mads.hands_on_paused)

  def test_stale_panda_blocks_new_admission_with_hands(self):
    mads, sd = make_mads()
    _disengage(mads)
    _step_mads(mads, sd, FakeCS(hands_on_level=3), inhibited=True, enable=True, panda_valid=False)
    self.assertFalse(mads.enabled)
    self.assertFalse(mads.hands_on_paused)

  def test_brake_invalidates_hands_only_reason_without_restoring_commands(self):
    mads, sd = make_mads()
    _step_mads(mads, sd, FakeCS(hands_on_level=3), inhibited=True)
    self.assertTrue(mads.hands_on_paused)
    cs = FakeCS(hands_on_level=3)
    cs.brakePressed = True
    _step_mads(mads, sd, cs, inhibited=True)
    self.assertFalse(mads.hands_on_paused)
    self.assertFalse(mads.active)

  def test_terminal_events_invalidate_hands_only_reason(self):
    for event in (EventName.driverUnresponsive3, EventName.driverDistracted3, EventName.steerUnavailable):
      with self.subTest(event=event):
        mads, sd = make_mads()
        _step_mads(mads, sd, FakeCS(hands_on_level=3), inhibited=True)
        sd.events.add(event)
        mads.update(FakeCS(hands_on_level=3))
        self.assertFalse(mads.hands_on_paused)
        self.assertFalse(mads.enabled)

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
      self.assertTrue(sd.events_sp.has(EventNameSP.silentLkasDisable))
      self.assertFalse(sd.events_sp.has(EventNameSP.lkasDisable))

      cs = FakeCS(hands_on_level=0)
      sd.sm = FakeSM([_panda(True)])
      mads.update_events(cs)
      self.assertTrue(mads._hands_on_steering_inhibited)
      self.assertTrue(mads._hands_on_clear_timing)

      ns["v"] = (HANDS_ON_RESUME_US - 1) * 1000
      mads.update_events(cs)
      self.assertTrue(mads._hands_on_steering_inhibited)

      ns["v"] = (HANDS_ON_RESUME_US + 1) * 1000
      mads.update_events(cs)
      self.assertTrue(mads._hands_on_steering_inhibited)

      sd.sm = FakeSM([_panda(False)], panda_valid=True)
      ns["v"] = 0
      mads.update_events(cs)
      ns["v"] = HANDS_ON_RESUME_US * 1000
      mads.update_events(cs)
      self.assertFalse(mads._hands_on_steering_inhibited)
      self.assertFalse(sd.events_sp.has(EventNameSP.lkasDisable))
    finally:
      cast(Any, mads_mod.time).monotonic_ns = orig

  def test_stale_cs_hard_disables_and_blocks_resume(self):
    mads, sd = make_mads()
    cs = FakeCS(hands_on_level=2)
    sd.sm = FakeSM([_panda(True)])
    mads.update_events(cs)
    self.assertTrue(sd.events_sp.has(EventNameSP.silentLkasDisable))
    sd.cs_fresh = False
    mads.update_events(FakeCS(hands_on_level=0))
    self.assertTrue(sd.events_sp.has(EventNameSP.lkasDisable))
    self.assertFalse(sd.events_sp.has(EventNameSP.silentLkasDisable))
    self.assertFalse(mads._hands_on_steering_inhibited)
    sd.cs_fresh = True
    mads.update_events(FakeCS(hands_on_level=0))
    self.assertFalse(mads.should_silent_lkas_enable(FakeCS(hands_on_level=0)))

  def test_door_open_while_paused_hard_disables(self):
    mads, sd = make_mads()
    sd.sm = FakeSM([_panda(True)])
    mads.update_events(FakeCS(hands_on_level=2))
    mads.update_events(FakeCS(hands_on_level=2, door_open=True))
    self.assertTrue(sd.events_sp.has(EventNameSP.lkasDisable))
    self.assertFalse(sd.events_sp.has(EventNameSP.silentLkasDisable))

  def test_standstill_door_and_hands_state_disables(self):
    mads, sd = make_mads()
    sd.sm = FakeSM([_panda(True)])
    sd.events.add(EventName.doorOpen)
    cs = FakeCS(hands_on_level=2, door_open=True)
    cs.standstill = True
    cs.vEgo = 0.0
    mads.update_events(cs)
    self.assertFalse(sd.events_sp.has(EventNameSP.silentLkasDisable))
    self.assertTrue(sd.events_sp.has(EventNameSP.lkasDisable))
    mads.state_machine.update()
    self.assertEqual(mads.state_machine.state, State.disabled)

  def test_immediate_disable_and_hands_state_disables(self):
    mads, sd = make_mads()
    sd.sm = FakeSM([_panda(True)])
    sd.events.add(EventName.steerUnavailable)
    cs = FakeCS(hands_on_level=2)
    mads.update_events(cs)
    self.assertFalse(sd.events_sp.has(EventNameSP.silentLkasDisable))
    mads.state_machine.update()
    self.assertEqual(mads.state_machine.state, State.disabled)

  def test_gear_not_drive_while_paused_hard_disables(self):
    mads, sd = make_mads()
    sd.sm = FakeSM([_panda(True)])
    mads.update_events(FakeCS(hands_on_level=2))
    cs = FakeCS(hands_on_level=2)
    cs.gearShifter = GearShifter.reverse
    mads.update_events(cs)
    self.assertTrue(sd.events_sp.has(EventNameSP.lkasDisable))

  def test_stale_panda_hard_disables(self):
    mads, sd = make_mads()
    sd.sm = FakeSM([_panda(True)])
    mads.update_events(FakeCS(hands_on_level=2))
    sd.sm = FakeSM([_panda(False)], panda_valid=False)
    mads.update_events(FakeCS(hands_on_level=0))
    self.assertTrue(sd.events_sp.has(EventNameSP.lkasDisable))
    self.assertFalse(mads._hands_on_steering_inhibited)

  def test_lateral_loss_hard_disables(self):
    mads, sd = make_mads()
    sd.sm = FakeSM([_panda(True)])
    mads.update_events(FakeCS(hands_on_level=2))
    ps = _panda(False)
    ps.controlsAllowedLateral = False
    sd.sm = FakeSM([ps])
    mads.update_events(FakeCS(hands_on_level=0))
    self.assertTrue(sd.events_sp.has(EventNameSP.lkasDisable))

  def test_dm_critical_blocks_resume(self):
    mads, sd = make_mads()
    sd.sm = FakeSM([_panda(True)])
    mads.update_events(FakeCS(hands_on_level=2))
    sd.events.add(log.OnroadEvent.EventName.driverUnresponsive3)
    mads.update_events(FakeCS(hands_on_level=0))
    self.assertTrue(sd.events_sp.has(EventNameSP.lkasDisable))
    self.assertFalse(mads.should_silent_lkas_enable(FakeCS(hands_on_level=0)))

  def test_epas_fault_is_not_hands_on_pause(self):
    mads, sd = make_mads()
    cs = FakeCS(hands_on_level=2, steer_fault=True)
    sd.sm = FakeSM([_panda(True)])
    mads.update_events(cs)
    self.assertFalse(mads._hands_on_steering_inhibited)
    self.assertFalse(sd.events_sp.has(EventNameSP.silentLkasDisable))
    self.assertTrue(sd.events_sp.has(EventNameSP.lkasDisable))

  def test_restart_reinhibits_from_panda(self):
    restarted, sd2 = make_mads()
    sd2.sm = FakeSM([_panda(True)])
    self.assertFalse(restarted._hands_on_steering_inhibited)
    restarted.enabled = True
    restarted.update_events(FakeCS(hands_on_level=0))
    self.assertTrue(restarted._hands_on_steering_inhibited)

  def test_default_off_does_not_pause(self):
    mads, sd = make_mads(hands_on=False)
    cs = FakeCS(hands_on_level=2)
    sd.sm = FakeSM([_panda(True)])
    mads.update_events(cs)
    self.assertFalse(mads._hands_on_pause_available)
    self.assertFalse(mads._hands_on_steering_inhibited)
    self.assertFalse(sd.events_sp.has(EventNameSP.silentLkasDisable))

  def test_default_off_standstill_door_hard_disables(self):
    mads, sd = make_mads(hands_on=False)
    sd.events.add(EventName.doorOpen)
    cs = FakeCS(hands_on_level=0, door_open=True)
    cs.standstill = True
    cs.vEgo = 0.0
    mads.update_events(cs)
    self.assertTrue(sd.events_sp.has(EventNameSP.lkasDisable))
    self.assertFalse(sd.events_sp.has(EventNameSP.silentLkasDisable))
    mads.state_machine.update()
    self.assertEqual(mads.state_machine.state, State.disabled)

  def test_preap_seatbelt_cannot_override_door_lkas_disable(self):
    mads, sd = make_mads(hands_on=False)
    sd.events.add(EventName.doorOpen)
    sd.events.add(EventName.seatbeltNotLatched)
    cs = FakeCS(hands_on_level=0, door_open=True)
    cs.standstill = True
    cs.vEgo = 0.0
    mads.update_events(cs)
    self.assertTrue(sd.events_sp.has(EventNameSP.lkasDisable))
    self.assertFalse(sd.events_sp.has(EventNameSP.silentLkasDisable))
    mads.state_machine.update()
    self.assertEqual(mads.state_machine.state, State.disabled)

  def test_non_preap_standstill_door_still_pauses(self):
    mads, sd = make_mads(hands_on=False, capability=False)
    sd.CP.brand = "honda"
    sd.CP.carFingerprint = "HONDA_CIVIC"
    mads.CP = sd.CP
    mads._hands_on_pause_available = False
    sd.events.add(EventName.doorOpen)
    cs = FakeCS(hands_on_level=0, door_open=True)
    cs.standstill = True
    cs.vEgo = 0.0
    mads.update_events(cs)
    self.assertTrue(sd.events_sp.has(EventNameSP.silentLkasDisable))
    self.assertFalse(sd.events_sp.has(EventNameSP.lkasDisable))
    mads.state_machine.update()
    self.assertEqual(mads.state_machine.state, State.paused)

  def test_generic_platform_unchanged(self):
    mads, sd = make_mads(hands_on=False, capability=False)
    cs = FakeCS(hands_on_level=2)
    sd.sm = FakeSM([_panda(True)])
    mads.update_events(cs)
    self.assertFalse(mads._hands_on_steering_inhibited)
    self.assertFalse(sd.events_sp.has(EventNameSP.silentLkasDisable))

  def test_first_pull_delayed_permission_50ms_does_not_false_disable(self):
    mads, sd = make_mads()
    _disengage(mads)
    cs = FakeCS(hands_on_level=1, steering_pressed=True)
    _step_mads(mads, sd, cs, allowed=False, enable=True, override=True)
    self.assertNotEqual(mads.state_machine.state, State.disabled)
    self.assertFalse(sd.events_sp.has(EventNameSP.lkasDisable))
    for _ in range(5):
      _step_mads(mads, sd, cs, allowed=False, override=True)
      self.assertFalse(sd.events_sp.has(EventNameSP.lkasDisable))
      self.assertNotEqual(mads.state_machine.state, State.disabled)
    _step_mads(mads, sd, cs, allowed=True, override=True)
    self.assertFalse(sd.events_sp.has(EventNameSP.lkasDisable))
    self.assertEqual(mads.state_machine.state, State.overriding)

  def test_first_pull_delayed_permission_over_100ms_does_not_false_disable(self):
    mads, sd = make_mads()
    _disengage(mads)
    cs = FakeCS(hands_on_level=1, steering_pressed=True)
    _step_mads(mads, sd, cs, allowed=False, enable=True, override=True)
    for _ in range(15):
      _step_mads(mads, sd, cs, allowed=False, override=True)
      self.assertFalse(sd.events_sp.has(EventNameSP.lkasDisable))
      self.assertNotEqual(mads.state_machine.state, State.disabled)
    _step_mads(mads, sd, cs, allowed=True, override=True)
    self.assertFalse(sd.events_sp.has(EventNameSP.lkasDisable))
    self.assertEqual(mads.state_machine.state, State.overriding)

  def test_never_granted_permission_hits_mismatch_bound(self):
    mads, sd = make_mads()
    _disengage(mads)
    cs = FakeCS(hands_on_level=1, steering_pressed=True)
    _step_mads(mads, sd, cs, allowed=False, enable=True, override=True)
    self.assertFalse(sd.events_sp.has(EventNameSP.lkasDisable))
    for _ in range(LATERAL_MISMATCH_MAX_COUNT - 2):
      _step_mads(mads, sd, cs, allowed=False, override=True)
      self.assertFalse(sd.events_sp.has(EventNameSP.lkasDisable))
      self.assertNotEqual(mads.state_machine.state, State.disabled)
    _step_mads(mads, sd, cs, allowed=False, override=True)
    self.assertTrue(sd.events_sp.has(EventNameSP.lkasDisable))
    self.assertEqual(mads.state_machine.state, State.disabled)

  def test_acquired_permission_then_lost_hard_disables(self):
    mads, sd = make_mads()
    _disengage(mads)
    cs = FakeCS(hands_on_level=1, steering_pressed=True)
    _step_mads(mads, sd, cs, allowed=True, enable=True, override=True)
    _step_mads(mads, sd, cs, allowed=True, override=True)
    self.assertFalse(sd.events_sp.has(EventNameSP.lkasDisable))
    self.assertEqual(mads.state_machine.state, State.overriding)
    _step_mads(mads, sd, cs, allowed=False, override=True)
    self.assertTrue(sd.events_sp.has(EventNameSP.lkasDisable))
    self.assertEqual(mads.state_machine.state, State.disabled)

  def test_pause_cannot_evade_acquisition_deadline(self):
    mads, sd = make_mads()
    _disengage(mads)
    cs = FakeCS(hands_on_level=1, steering_pressed=True)
    _step_mads(mads, sd, cs, allowed=False, enable=True, override=True)
    for _ in range(49):
      _step_mads(mads, sd, cs, allowed=False, override=True)
    self.assertNotEqual(mads.state_machine.state, State.disabled)
    cs_pause = FakeCS(hands_on_level=2)
    remaining = LATERAL_MISMATCH_MAX_COUNT - 50
    for _ in range(remaining - 1):
      _step_mads(mads, sd, cs_pause, allowed=False, inhibited=True)
      self.assertNotEqual(mads.state_machine.state, State.disabled)
    _step_mads(mads, sd, cs_pause, allowed=False, inhibited=True)
    self.assertTrue(sd.events_sp.has(EventNameSP.lkasDisable))
    self.assertEqual(mads.state_machine.state, State.disabled)

  def test_stale_positive_grant_during_pending_hard_disables(self):
    mads, sd = make_mads()
    _disengage(mads)
    cs = FakeCS(hands_on_level=1, steering_pressed=True)
    _step_mads(mads, sd, cs, allowed=False, enable=True, override=True)
    for _ in range(10):
      _step_mads(mads, sd, cs, allowed=False, override=True)
    _step_mads(mads, sd, cs, allowed=True, override=True, panda_valid=False)
    self.assertTrue(sd.events_sp.has(EventNameSP.lkasDisable))
    self.assertEqual(mads.state_machine.state, State.disabled)

  def test_door_during_pending_acquisition_hard_disables(self):
    mads, sd = make_mads()
    _disengage(mads)
    cs = FakeCS(hands_on_level=1, steering_pressed=True)
    _step_mads(mads, sd, cs, allowed=False, enable=True, override=True)
    self.assertFalse(sd.events_sp.has(EventNameSP.lkasDisable))
    _step_mads(mads, sd, FakeCS(hands_on_level=1, steering_pressed=True, door_open=True),
               allowed=False, override=True)
    self.assertTrue(sd.events_sp.has(EventNameSP.lkasDisable))
    self.assertEqual(mads.state_machine.state, State.disabled)

  def test_level_one_pauses_at_hands_one(self):
    mads, sd = make_mads(safety_param=8 | (1 << 8))
    sd.sm = FakeSM([_panda(False)])
    mads.update_events(FakeCS(hands_on_level=0))
    self.assertFalse(mads._hands_on_steering_inhibited)
    mads.update_events(FakeCS(hands_on_level=1))
    self.assertTrue(mads._hands_on_steering_inhibited)
    self.assertTrue(sd.events_sp.has(EventNameSP.silentLkasDisable))

  def test_level_three_stays_enabled_at_hands_two(self):
    mads, sd = make_mads(safety_param=8 | (3 << 8))
    sd.sm = FakeSM([_panda(False)])
    mads.update_events(FakeCS(hands_on_level=2))
    self.assertFalse(mads._hands_on_steering_inhibited)
    self.assertFalse(sd.events_sp.has(EventNameSP.silentLkasDisable))

  def test_level_three_pauses_at_hands_three(self):
    mads, sd = make_mads(safety_param=8 | (3 << 8))
    sd.sm = FakeSM([_panda(False)])
    mads.update_events(FakeCS(hands_on_level=3))
    self.assertTrue(mads._hands_on_steering_inhibited)
    self.assertTrue(sd.events_sp.has(EventNameSP.silentLkasDisable))

  def test_panda_inhibit_pauses_irrespective_of_host_level(self):
    mads, sd = make_mads(safety_param=8 | (3 << 8))
    sd.sm = FakeSM([_panda(True)])
    mads.update_events(FakeCS(hands_on_level=0))
    self.assertTrue(mads._hands_on_steering_inhibited)
    self.assertTrue(sd.events_sp.has(EventNameSP.silentLkasDisable))

  def test_disengage_level_frozen_at_init(self):
    mads, sd = make_mads(safety_param=8 | (3 << 8))
    sd.CP.safetyConfigs[0].safetyParam = 8 | (1 << 8)
    sd.sm = FakeSM([_panda(False)])
    mads.update_events(FakeCS(hands_on_level=2))
    self.assertFalse(mads._hands_on_steering_inhibited)

if __name__ == "__main__":
  unittest.main()
