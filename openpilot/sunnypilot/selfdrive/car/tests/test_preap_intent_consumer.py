import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from openpilot.cereal import custom, log
from opendbc.can import CANPacker
from opendbc.car import CanData, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.common.conversions import Conversions as CV
import opendbc.car.tesla.preap.nap_conf as nap_conf_mod
from opendbc.car.tesla.preap.nap_params import NAPParamKeys
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.selfdrive.car.car_specific import CarSpecificEvents
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP
from opendbc.car.tesla.preap.sp.platform import preap_radar_present
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP
from opendbc.car.tesla.preap.sp.carstate import PREAP_HANDS_ON_RESUME_MS, PreAPCarState
from opendbc.car.tesla.values import CAR, CruiseButtons
from openpilot.selfdrive.car.helpers import convert_to_capnp
from openpilot.selfdrive.selfdrived.preap_regen import PreAPChimeState, update_preap_chimes
from openpilot.selfdrive.selfdrived.state import StateMachine as OPStateMachine
from openpilot.sunnypilot.mads.mads import ModularAssistiveDrivingSystem
from opendbc.car.tesla.preap.carcontroller import PedalAuthorityState, PreAPLongController
from opendbc.car.tesla.preap.nap_conf import PEDAL_MAX_VALUES
from opendbc.car.tesla.preap.teslacan import GAS_COMMAND_ID, PEDAL_D, PEDAL_M1, TeslaCANPreAP
from openpilot.sunnypilot.selfdrive.car.preap_intent import (
  PreAPIntentConsumer, UINT32_HALF, UINT32_MASK, sequence_is_newer,
)

EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
Lateral = custom.CarStateSP.PreapLateralIntent
Longitudinal = custom.CarStateSP.PreapLongitudinalIntent


def record(sequence, lat=Lateral.none, lon=Longitudinal.none, epoch=0):
  return SimpleNamespace(
    preapIntentEpoch=epoch,
    preapIntentSequence=sequence,
    preapLateralIntent=lat,
    preapLongitudinalIntent=lon,
  )


class TestSequenceContract(unittest.TestCase):
  def test_newer_ignore_and_half_range(self):
    self.assertTrue(sequence_is_newer(1, 0))
    self.assertFalse(sequence_is_newer(0, 0))
    self.assertFalse(sequence_is_newer(0, 1))
    self.assertIsNone(sequence_is_newer(UINT32_HALF, 0))
    self.assertTrue(sequence_is_newer(0, UINT32_MASK))


class TestPreAPIntentConsumer(unittest.TestCase):
  def setUp(self):
    self.consumer = PreAPIntentConsumer()
    self.events = Events()
    self.events_sp = EventsSP()

  def apply(self, rec, apply_longitudinal=True):
    self.events.clear()
    self.events_sp.clear()
    self.consumer.update(rec, self.events, self.events_sp, apply_longitudinal=apply_longitudinal)

  def test_first_record_seeds_without_acting(self):
    self.apply(record(0))
    self.assertTrue(self.consumer.seeded)
    self.assertFalse(self.events_sp.has(EventNameSP.lkasEnable))
    self.assertFalse(self.events.has(EventName.buttonEnable))
    self.apply(record(0, Lateral.mainCruiseRequest))
    self.assertFalse(self.events_sp.has(EventNameSP.lkasEnable))

  def test_consumer_restart_seeds_current_without_acting(self):
    self.apply(record(4, Lateral.mainCruiseRequest, Longitudinal.enable))
    self.assertFalse(self.events_sp.has(EventNameSP.lkasEnable))
    self.assertFalse(self.events.has(EventName.buttonEnable))

  def test_duplicate_and_older_ignored(self):
    self.apply(record(1))
    self.apply(record(2, Lateral.mainCruiseRequest))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasEnable))
    self.apply(record(2, Lateral.mainCruiseRequest))
    self.assertFalse(self.events_sp.has(EventNameSP.lkasEnable))
    self.apply(record(1, Lateral.forceDisable))
    self.assertFalse(self.events_sp.has(EventNameSP.lkasDisable))

  def test_latched_newer_recovers_loss(self):
    self.apply(record(1))
    self.apply(record(4, Lateral.forceDisable, Longitudinal.disable))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasDisable))
    self.assertTrue(self.events.has(EventName.buttonCancel))

  def test_enable_disable_late_enable(self):
    self.apply(record(10))
    self.apply(record(11, Lateral.mainCruiseRequest))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasEnable))
    self.apply(record(12, Lateral.forceDisable))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasDisable))
    self.apply(record(11, Lateral.mainCruiseRequest))
    self.assertFalse(self.events_sp.has(EventNameSP.lkasEnable))

  def test_rollover_progression(self):
    self.apply(record(UINT32_MASK))
    self.apply(record(0, Lateral.mainCruiseRequest))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasEnable))

  def test_half_range_fail_closed(self):
    self.apply(record(0))
    self.apply(record(UINT32_HALF, Lateral.mainCruiseRequest, Longitudinal.enable))
    self.assertTrue(self.consumer.fail_closed)
    self.assertTrue(self.events_sp.has(EventNameSP.lkasDisable))
    self.assertTrue(self.events.has(EventName.buttonCancel))
    self.assertFalse(self.events_sp.has(EventNameSP.lkasEnable))
    self.apply(record(UINT32_HALF + 1, Lateral.mainCruiseRequest))
    self.assertFalse(self.events_sp.has(EventNameSP.lkasEnable))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasDisable))

  def test_disable_before_enable_same_record(self):
    self.apply(record(1))
    self.apply(record(2, Lateral.forceDisable, Longitudinal.enable))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasDisable))
    self.assertTrue(self.events.has(EventName.buttonEnable))

  def test_event_mapping(self):
    self.apply(record(1))
    self.apply(record(2, Lateral.mainCruiseRequest, Longitudinal.enable))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasEnable))
    self.assertTrue(self.events.has(EventName.buttonEnable))
    self.apply(record(3, Lateral.forceDisable, Longitudinal.disable))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasDisable))
    self.assertTrue(self.events.has(EventName.buttonCancel))

  def test_no_longitudinal_when_not_op_long(self):
    self.apply(record(1))
    self.apply(record(2, Lateral.none, Longitudinal.enable), apply_longitudinal=False)
    self.assertFalse(self.events.has(EventName.buttonEnable))

  def test_car_specific_events_uses_consumer_before_brand_logic(self):
    CP = structs.CarParams()
    CP.brand = "tesla"
    CP.carFingerprint = "TESLA_MODEL_S_PREAP"
    CP.openpilotLongitudinalControl = True
    CP_SP = structs.CarParamsSP()
    cse = CarSpecificEventsSP(CP, CP_SP)
    events = Events()
    cs = structs.CarState()
    cse.update(cs, events, record(0))
    events_sp = cse.update(cs, events, record(1, Lateral.mainCruiseRequest, Longitudinal.enable))
    self.assertTrue(events_sp.has(EventNameSP.lkasEnable))
    self.assertTrue(events.has(EventName.buttonEnable))

  def test_unaligned_none_does_not_act(self):
    CP = structs.CarParams()
    CP.brand = "hyundai"
    cse = CarSpecificEventsSP(CP, structs.CarParamsSP())
    events = Events()
    events_sp = cse.update(structs.CarState(), events, None)
    self.assertFalse(events_sp.has(EventNameSP.lkasEnable))

  def test_nopedal_radar_fault_maps_only_with_preap_radar_present(self):
    CP = structs.CarParams()
    CP.brand = "tesla"
    CP.carFingerprint = "TESLA_MODEL_S_PREAP"
    CP.openpilotLongitudinalControl = False
    CP.pcmCruise = True
    CP_SP = structs.CarParamsSP()
    CP_SP.flags = int(TeslaFlagsSP.PREAP_RADAR_PRESENT)
    self.assertTrue(preap_radar_present(CP, CP_SP))

    events = Events()
    CarSpecificEventsSP(CP, CP_SP).update(structs.CarState(), events, record(0), radar_fault=True)
    self.assertTrue(events.has(EventName.radarFault))

    events = Events()
    CarSpecificEventsSP(CP, structs.CarParamsSP()).update(
      structs.CarState(), events, record(0), radar_fault=True,
    )
    self.assertFalse(events.has(EventName.radarFault))

    hyundai = structs.CarParams()
    hyundai.brand = "hyundai"
    hyundai.carFingerprint = "HYUNDAI_KONA_NON_SCC"
    hyundai.openpilotLongitudinalControl = False
    overlap = structs.CarParamsSP()
    overlap.flags = int(TeslaFlagsSP.PREAP_RADAR_PRESENT)
    self.assertFalse(preap_radar_present(hyundai, overlap))
    events = Events()
    CarSpecificEventsSP(hyundai, overlap).update(structs.CarState(), events, None, radar_fault=True)
    self.assertFalse(events.has(EventName.radarFault))

  def test_latched_hold_survives_conflate(self):
    CP = structs.CarParams()
    CP.carFingerprint = CAR.TESLA_MODEL_S_PREAP
    cs = PreAPCarState(CP, structs.CarParamsSP())

    def publish():
      ret_sp = structs.CarStateSP()
      cs._publish_mads_intent(ret_sp)
      return convert_to_capnp(ret_sp)

    seed = publish()
    cs.engagement.cruiseEnabled = True
    pulse = publish()
    hold = publish()
    mailbox = hold
    self.assertEqual(pulse.preapIntentSequence, hold.preapIntentSequence)
    self.assertEqual(hold.preapLateralIntent, Lateral.mainCruiseRequest)

    self.apply(seed)
    self.apply(mailbox)
    self.assertTrue(self.events_sp.has(EventNameSP.lkasEnable))

  def test_duplicate_same_sequence_does_not_reenable(self):
    self.apply(record(1, epoch=1))
    self.apply(record(2, Lateral.mainCruiseRequest, epoch=1))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasEnable))
    self.apply(record(2, Lateral.mainCruiseRequest, epoch=1))
    self.assertFalse(self.events_sp.has(EventNameSP.lkasEnable))

  def test_long_only_after_disable_does_not_replay_lat(self):
    self.apply(record(1, epoch=1))
    self.apply(record(2, Lateral.mainCruiseRequest, epoch=1))
    self.apply(record(3, Lateral.forceDisable, epoch=1))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasDisable))
    self.apply(record(4, Lateral.none, Longitudinal.enable, epoch=1))
    self.assertFalse(self.events_sp.has(EventNameSP.lkasEnable))
    self.assertTrue(self.events.has(EventName.buttonEnable))

  def test_producer_epoch_restart_seeds_without_acting(self):
    self.apply(record(4, Lateral.mainCruiseRequest, epoch=1))
    self.assertFalse(self.events_sp.has(EventNameSP.lkasEnable))
    self.apply(record(0, Lateral.mainCruiseRequest, epoch=2))
    self.assertFalse(self.events_sp.has(EventNameSP.lkasEnable))
    self.apply(record(1, Lateral.mainCruiseRequest, epoch=2))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasEnable))

  def test_publisher_long_only_does_not_keep_lat_request(self):
    CP = structs.CarParams()
    CP.carFingerprint = CAR.TESLA_MODEL_S_PREAP
    cs = PreAPCarState(CP, structs.CarParamsSP())

    def publish():
      ret_sp = structs.CarStateSP()
      cs._publish_mads_intent(ret_sp)
      return ret_sp

    publish()
    cs.engagement.cruiseEnabled = True
    lat = publish()
    cs.engagement.enableLongControl = True
    lon = publish()
    self.assertEqual(lat.preapLateralIntent, structs.CarStateSP.PreapLateralIntent.mainCruiseRequest)
    self.assertEqual(lon.preapLateralIntent, structs.CarStateSP.PreapLateralIntent.none)
    self.assertEqual(lon.preapLongitudinalIntent, structs.CarStateSP.PreapLongitudinalIntent.enable)
    self.assertNotEqual(lat.preapIntentSequence, lon.preapIntentSequence)

  def test_adapter_hands_on_keeps_active_long(self):
    CP = structs.CarParams()
    CP.carFingerprint = CAR.TESLA_MODEL_S_PREAP
    CP_SP = structs.CarParamsSP()
    CP_SP.flags = int(TeslaFlagsSP.PREAP_HANDS_ON_PAUSE)
    cs = PreAPCarState(CP, CP_SP)
    cs.engagement.cruiseEnabled = True
    cs.engagement.enableLongControl = True
    cs.engagement.pedal_speed_kph = 72.0
    cs.engagement.stalk_pull_time_ms = 1500
    cs.engagement.prev_stalk_pull_time_ms = 1000
    cs.engagement.pending_enable = True
    cs._epas_hands = 2
    cs.engagement.handle_steering_disengage(True)
    self.assertTrue(cs.engagement.cruiseEnabled)
    self.assertTrue(cs.engagement.enableLongControl)
    self.assertAlmostEqual(cs.engagement.pedal_speed_kph, 72.0)
    self.assertFalse(cs.engagement.pending_enable)
    self.assertEqual(cs.engagement.stalk_pull_time_ms, 0)
    self.assertEqual(cs.engagement.prev_stalk_pull_time_ms, -1000)

  def test_adapter_epas_reject_full_disengage(self):
    CP = structs.CarParams()
    CP.carFingerprint = CAR.TESLA_MODEL_S_PREAP
    CP_SP = structs.CarParamsSP()
    CP_SP.flags = int(TeslaFlagsSP.PREAP_HANDS_ON_PAUSE)
    cs = PreAPCarState(CP, CP_SP)
    cs.engagement.cruiseEnabled = True
    cs.engagement.enableLongControl = True
    cs._epas_hands = 2
    cs._epas_rejecting = True
    cs.engagement.handle_steering_disengage(True)
    self.assertFalse(cs.engagement.cruiseEnabled)

  def test_adapter_delayed_epas_after_pause_force_disables(self):
    CP = structs.CarParams()
    CP.carFingerprint = CAR.TESLA_MODEL_S_PREAP
    CP_SP = structs.CarParamsSP()
    CP_SP.flags = int(TeslaFlagsSP.PREAP_HANDS_ON_PAUSE)
    cs = PreAPCarState(CP, CP_SP)
    cs.engagement.cruiseEnabled = True
    cs.engagement.enableLongControl = False
    cs.engagement.prev_steering_disengage = True
    cs._epas_hands = 2
    cs._epas_rejecting = True
    cs.engagement.handle_steering_disengage(True)
    self.assertFalse(cs.engagement.cruiseEnabled)

  def test_adapter_default_off_full_disengage(self):
    CP = structs.CarParams()
    CP.carFingerprint = CAR.TESLA_MODEL_S_PREAP
    cs = PreAPCarState(CP, structs.CarParamsSP())
    cs.engagement.cruiseEnabled = True
    cs._epas_hands = 2
    cs.engagement.handle_steering_disengage(True)
    self.assertFalse(cs.engagement.cruiseEnabled)

  def test_disabled_hands_on_does_not_emit_main_cruise(self):
    CP = structs.CarParams()
    CP.carFingerprint = CAR.TESLA_MODEL_S_PREAP
    CP_SP = structs.CarParamsSP()
    CP_SP.flags = int(TeslaFlagsSP.PREAP_HANDS_ON_PAUSE)
    cs = PreAPCarState(CP, CP_SP)
    cs._epas_hands = 2
    ret_sp = structs.CarStateSP()
    cs._publish_mads_intent(ret_sp)
    cs.engagement.cruiseEnabled = True
    cs._publish_mads_intent(ret_sp)
    self.assertEqual(ret_sp.preapLateralIntent, structs.CarStateSP.PreapLateralIntent.none)

  def test_unadmitted_held_hands_revokes_cruise(self):
    CP = structs.CarParams()
    CP.carFingerprint = CAR.TESLA_MODEL_S_PREAP
    CP_SP = structs.CarParamsSP()
    CP_SP.flags = int(TeslaFlagsSP.PREAP_HANDS_ON_PAUSE)
    cs = PreAPCarState(CP, CP_SP)
    cs.engagement.cruiseEnabled = True
    cs.engagement.pending_enable = True
    cs.engagement.enableJustCC = True
    ret = structs.CarState()
    ret.cruiseState.enabled = True
    cs._revoke_unadmitted_held_hands(ret)
    self.assertFalse(cs.engagement.cruiseEnabled)
    self.assertFalse(cs.engagement.pending_enable)
    self.assertFalse(cs.engagement.enableJustCC)
    self.assertFalse(ret.cruiseState.enabled)
    self.assertFalse(cs.preap_cc_cancel_needed)
    self.assertFalse(cs.preap_cc_engage_needed)
    self.assertEqual(cs.engagement.stalk_pull_time_ms, 0)
    self.assertEqual(cs.engagement.prev_stalk_pull_time_ms, -1000)

  def test_adapter_pause_swallows_main_without_starting_or_dropping_long(self):
    CP = structs.CarParams()
    CP.carFingerprint = CAR.TESLA_MODEL_S_PREAP
    CP_SP = structs.CarParamsSP()
    CP_SP.flags = int(TeslaFlagsSP.PREAP_HANDS_ON_PAUSE)
    cs = PreAPCarState(CP, CP_SP)
    cs.engagement.enableDoublePull = True
    cs.engagement.double_pull_window_ms = 750
    cs._epas_hands = 2
    cs.engagement.cruiseEnabled = True
    cs.engagement.enableLongControl = True
    cs.engagement.pedal_speed_kph = 64.0
    cs.engagement.process_buttons(
      CruiseButtons.MAIN, CruiseButtons.IDLE, 2000, 20.0, "KPH", True, True, True, False,
    )
    self.assertTrue(cs.engagement.cruiseEnabled)
    self.assertTrue(cs.engagement.enableLongControl)
    self.assertAlmostEqual(cs.engagement.pedal_speed_kph, 64.0)

    cs.engagement.enableLongControl = False
    cs.engagement.pedal_speed_kph = 0.0
    cs.engagement.process_buttons(
      CruiseButtons.MAIN, CruiseButtons.IDLE, 2800, 20.0, "KPH", True, True, True, False,
    )
    self.assertTrue(cs.engagement.cruiseEnabled)
    self.assertFalse(cs.engagement.enableLongControl)
    self.assertEqual(cs.engagement.pedal_speed_kph, 0.0)

    cs._epas_hands = 0
    cs.engagement.process_buttons(
      CruiseButtons.IDLE, CruiseButtons.IDLE, 2800, 20.0, "KPH", True, True, True, False,
    )
    cs.engagement.process_buttons(
      CruiseButtons.MAIN, CruiseButtons.IDLE, 2800 + PREAP_HANDS_ON_RESUME_MS - 1,
      20.0, "KPH", True, True, True, False,
    )
    self.assertTrue(cs.engagement.cruiseEnabled)
    self.assertFalse(cs.engagement.enableLongControl)
    self.assertEqual(cs.engagement.stalk_pull_time_ms, 0)

  def test_fresh_set_cruise_while_cruise_true_requests_lat(self):
    CP = structs.CarParams()
    CP.carFingerprint = CAR.TESLA_MODEL_S_PREAP
    cs = PreAPCarState(CP, structs.CarParamsSP())
    cs.engagement.cruiseEnabled = True
    cs._prev_cruise_enabled = True
    cs.engagement.enableLongControl = True
    cs._prev_enable_long = False
    cs._epas_hands = 0
    ret = structs.CarState()
    be = structs.CarState.ButtonEvent()
    be.pressed = True
    be.type = structs.CarState.ButtonEvent.Type.setCruise
    ret.buttonEvents = [be]
    ret_sp = structs.CarStateSP()
    cs._publish_mads_intent(ret_sp, ret)
    self.assertEqual(ret_sp.preapLateralIntent, structs.CarStateSP.PreapLateralIntent.mainCruiseRequest)
    self.assertEqual(ret_sp.preapLongitudinalIntent, structs.CarStateSP.PreapLongitudinalIntent.enable)

    hold = structs.CarStateSP()
    cs._publish_mads_intent(hold, structs.CarState())
    self.assertEqual(hold.preapIntentSequence, ret_sp.preapIntentSequence)
    self.assertEqual(hold.preapLateralIntent, structs.CarStateSP.PreapLateralIntent.mainCruiseRequest)


OPState = log.SelfdriveState.OpenpilotState


class TestPedalLongitudinalHostFlow(unittest.TestCase):
  def setUp(self):
    self.enterContext(OpenpilotPrefix())
    params = Params()
    params.put_bool(NAPParamKeys.PEDAL_ENABLED, True, block=True)
    params.put_bool(NAPParamKeys.RADAR_ENABLED, True, block=True)
    params.put_bool(NAPParamKeys.PEDAL_CALIB_DONE, True, block=True)
    params.put(NAPParamKeys.PEDAL_CALIB_FACTOR, 0.95, block=True)
    params.put_bool("Mads", True, block=True)
    self.enterContext(patch.object(nap_conf_mod, "_PARAMS_AVAILABLE", True))
    self.enterContext(patch.object(nap_conf_mod, "_params", params))

    CarInterface = interfaces[CAR.TESLA_MODEL_S_PREAP]
    fingerprint_buses = {bus: {} for bus in range(8)}
    self.CP = CarInterface.get_params(
      CAR.TESLA_MODEL_S_PREAP, fingerprint_buses, [],
      alpha_long=False, is_release=False, docs=False,
    )
    self.CP_SP = CarInterface.get_params_sp(
      self.CP, CAR.TESLA_MODEL_S_PREAP, fingerprint_buses, [],
      alpha_long=False, is_release_sp=False, docs=False,
    )

    self.cse = CarSpecificEvents(self.CP)
    self.sp = CarSpecificEventsSP(self.CP, self.CP_SP)
    self.adapter = PreAPCarState(self.CP, self.CP_SP)
    self.adapter.engagement.enableDoublePull = True
    self.adapter.engagement.double_pull_window_ms = 750
    seed = structs.CarStateSP()
    self.adapter._publish_mads_intent(seed)
    self.cs_sp = convert_to_capnp(seed)
    self.sp.update(structs.CarState(), Events(), self.cs_sp)

    self.events = Events()
    self.events_sp = EventsSP()
    self.op_sm = OPStateMachine()
    self.cc = structs.CarControl()
    self.cs_prev = self._cs()
    self.chimes_prev = PreAPChimeState()
    self.sd = SimpleNamespace(
      CP=self.CP,
      CP_SP=self.CP_SP,
      params=params,
      events=self.events,
      events_sp=self.events_sp,
      enabled=False,
      enabled_prev=False,
      initialized=True,
      cs_fresh=True,
      CS_prev=self.cs_prev,
      sm={"pandaStates": []},
      state_machine=self.op_sm,
    )
    self.mads = ModularAssistiveDrivingSystem(self.sd)

  def _pull(self, button, prev, t_ms, brake=False):
    return self.adapter.engagement.process_buttons(
      button, prev, t_ms, 20.0, "KPH", True, True, True, brake,
    )

  def _cs(self, gas=False, buttons=None):
    cs = structs.CarState()
    cs.cruiseState.available = True
    cs.cruiseState.enabled = bool(self.adapter.engagement.cruiseEnabled)
    cs.enableLongControl = bool(self.adapter.engagement.enableLongControl)
    cs.gearShifter = structs.CarState.GearShifter.drive
    cs.gasPressed = gas
    cs.vEgo = 20.0
    if cs.enableLongControl:
      cs.cruiseState.speed = self.adapter.engagement.pedal_speed_kph * CV.KPH_TO_MS
    else:
      cs.cruiseState.speed = 1e-3
    if buttons:
      cs.buttonEvents = buttons
    return cs

  def _publish(self):
    ret_sp = structs.CarStateSP()
    self.adapter._publish_mads_intent(ret_sp)
    self.cs_sp = convert_to_capnp(ret_sp)

  def _tick(self, cs):
    self.events.clear()
    self.events_sp.clear()
    common = self.cse.update(cs, self.cs_prev, self.cc)
    self.events.add_from_msg(common.to_msg())
    extra_sp = self.sp.update(cs, self.events, self.cs_sp)
    self.events_sp.add_from_msg(extra_sp.to_msg())
    op_enabled, _ = self.op_sm.update(self.events)
    self.sd.enabled = bool(op_enabled)
    self.sd.CS_prev = self.cs_prev
    pre = Events()
    pre.add_from_msg(self.events.to_msg())
    pre_sp = EventsSP()
    pre_sp.add_from_msg(self.events_sp.to_msg())
    self.mads.update(cs)
    chimes, self.chimes_prev = update_preap_chimes(
      lat_engaged=bool(cs.cruiseState.enabled),
      long_engaged=bool(cs.enableLongControl),
      prev=self.chimes_prev,
    )
    self.cs_prev = cs
    self.sd.enabled_prev = bool(op_enabled)
    return pre, pre_sp, op_enabled, chimes

  def _first_pull(self, gas=False):
    buttons = self._pull(CruiseButtons.MAIN, CruiseButtons.IDLE, 1000)
    self._publish()
    return self._tick(self._cs(gas=gas, buttons=buttons))

  def _second_pull(self, gas=True):
    buttons = self._pull(CruiseButtons.MAIN, CruiseButtons.IDLE, 1500)
    self._publish()
    return self._tick(self._cs(gas=gas, buttons=buttons))

  def test_first_pull_enables_mads_not_openpilot_long(self):
    events, events_sp, op_enabled, chimes = self._first_pull(gas=True)
    self.assertTrue(self.adapter.engagement.cruiseEnabled)
    self.assertFalse(self.adapter.engagement.enableLongControl)
    self.assertFalse(events.has(EventName.pcmEnable))
    self.assertFalse(events.has(EventName.buttonEnable))
    self.assertTrue(events_sp.has(EventNameSP.lkasEnable))
    self.assertFalse(op_enabled)
    self.assertTrue(self.mads.enabled)
    self.assertFalse(chimes.long_engage)

  def test_double_pull_gas_held_latches_long_and_set_speed(self):
    self._first_pull(gas=True)
    events, events_sp, op_enabled, chimes = self._second_pull(gas=True)
    self.assertTrue(self.adapter.engagement.enableLongControl)
    self.assertFalse(events.has(EventName.pcmEnable))
    self.assertTrue(events.has(EventName.buttonEnable))
    self.assertTrue(self.sd.events.has(EventName.buttonEnable))
    self.assertTrue(op_enabled)
    self.assertEqual(self.op_sm.state, OPState.overriding)
    self.assertTrue(events.has(EventName.gasPressedOverride))
    self.assertTrue(chimes.long_engage)
    self.assertAlmostEqual(self.adapter.engagement.pedal_speed_kph, 72.0)

  def test_gas_release_stays_enabled_without_new_long_chime(self):
    self._first_pull(gas=True)
    self._second_pull(gas=True)
    events, _, op_enabled, chimes = self._tick(self._cs(gas=False))
    self.assertTrue(self.adapter.engagement.enableLongControl)
    self.assertTrue(op_enabled)
    self.assertEqual(self.op_sm.state, OPState.enabled)
    self.assertFalse(chimes.long_engage)
    self.assertFalse(chimes.long_disengage)
    self.assertFalse(events.has(EventName.buttonEnable))

  def test_stalk_hold_adjusts_target_without_enable_events(self):
    self._first_pull(gas=True)
    self._second_pull(gas=True)
    speed0 = self.adapter.engagement.pedal_speed_kph
    buttons = self._pull(CruiseButtons.RES_ACCEL, CruiseButtons.IDLE, 1600)
    self.assertGreater(self.adapter.engagement.pedal_speed_kph, speed0)
    speed1 = self.adapter.engagement.pedal_speed_kph
    hold = self._pull(CruiseButtons.RES_ACCEL_2ND, CruiseButtons.RES_ACCEL, 1700)
    self.assertGreater(self.adapter.engagement.pedal_speed_kph, speed1)
    events, _, op_enabled, chimes = self._tick(self._cs(gas=True, buttons=buttons + hold))
    self.assertTrue(op_enabled)
    self.assertFalse(events.has(EventName.pcmEnable))
    self.assertFalse(events.has(EventName.buttonCancel))
    self.assertFalse(chimes.long_engage)

  def test_brake_drops_long_keeps_mads(self):
    self._first_pull(gas=True)
    self._second_pull(gas=True)
    self._tick(self._cs(gas=False))
    self._pull(CruiseButtons.IDLE, CruiseButtons.IDLE, 2000, brake=True)
    self.assertTrue(self.adapter.engagement.cruiseEnabled)
    self.assertFalse(self.adapter.engagement.enableLongControl)
    self._publish()
    events, events_sp, op_enabled, chimes = self._tick(self._cs(gas=False))
    self.assertTrue(events.has(EventName.buttonCancel))
    self.assertFalse(self.sd.events.has(EventName.buttonCancel))
    self.assertFalse(events_sp.has(EventNameSP.lkasDisable))
    self.assertFalse(op_enabled)
    self.assertTrue(self.mads.enabled)
    self.assertTrue(chimes.long_disengage)

  def test_cancel_full_exit(self):
    self._first_pull()
    self._second_pull(gas=False)
    buttons = self._pull(CruiseButtons.CANCEL, CruiseButtons.IDLE, 3000)
    self.assertFalse(self.adapter.engagement.cruiseEnabled)
    self.assertFalse(self.adapter.engagement.enableLongControl)
    self._publish()
    events, events_sp, op_enabled, chimes = self._tick(self._cs(buttons=buttons))
    self.assertTrue(events.has(EventName.buttonCancel))
    self.assertTrue(events_sp.has(EventNameSP.lkasDisable))
    self.assertFalse(op_enabled)
    self.assertFalse(self.mads.enabled)
    self.assertTrue(chimes.long_disengage)

  def test_hard_fault_exits_both(self):
    self._first_pull()
    self._second_pull(gas=False)
    cs = self._cs()
    cs.steerFaultPermanent = True
    events, _, op_enabled, _ = self._tick(cs)
    self.assertTrue(events.has(EventName.steerUnavailable))
    self.assertFalse(op_enabled)
    self.assertFalse(self.mads.enabled)


class _PausePandaSM(dict):
  def __init__(self, pandas, panda_valid=True):
    super().__init__({"pandaStates": pandas})
    self.updated = {"pandaStates": True}
    self.panda_valid = panda_valid

  def all_checks(self, services):
    return self.panda_valid and services == ["pandaStates"]


def _pause_panda(inhibited, lat=True, allowed=True):
  ps = MagicMock()
  ps.steeringControlInhibited = inhibited
  ps.controlsAllowedLateral = lat
  ps.controlsAllowed = allowed
  model = getattr(structs.CarParams.SafetyModel, "teslaPreap", None)
  if model is None:
    model = structs.CarParams.SafetyModel.tesla
  ps.safetyModel = model
  return ps


State = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState


class TestHandsOnPauseHostFlow(TestPedalLongitudinalHostFlow):
  def setUp(self):
    super().setUp()
    self.CP_SP.madsHandsOnPauseAvailable = True
    self.CP_SP.flags = int(getattr(self.CP_SP, "flags", 0) or 0) | int(TeslaFlagsSP.PREAP_HANDS_ON_PAUSE)
    self.adapter = PreAPCarState(self.CP, self.CP_SP)
    self.adapter.engagement.enableDoublePull = True
    self.adapter.engagement.double_pull_window_ms = 750
    seed = structs.CarStateSP()
    self.adapter._publish_mads_intent(seed)
    self.cs_sp = convert_to_capnp(seed)
    self.sp = CarSpecificEventsSP(self.CP, self.CP_SP)
    self.sp.update(structs.CarState(), Events(), self.cs_sp)
    self.sd.sm = _PausePandaSM([_pause_panda(False)])
    self.mads = ModularAssistiveDrivingSystem(self.sd)

  def _set_panda(self, inhibited, lat=True, allowed=True):
    self.sd.sm = _PausePandaSM([_pause_panda(inhibited, lat=lat, allowed=allowed)])

  def _pause_cs(self, hands=0, steering_disengage=False, **kwargs):
    cs = self._cs(**kwargs)
    cs.handsOnLevel = hands
    cs.steeringDisengage = steering_disengage
    return cs

  def _hands_override(self, hands, reject=False, fault=False):
    self.adapter._epas_hands = hands
    self.adapter._epas_rejecting = reject
    self.adapter._epas_fault = fault
    self.adapter.engagement.handle_steering_disengage(hands >= 2 or reject)

  def test_pause_keeps_healthy_long_and_pauses_lateral(self):
    self._first_pull(gas=True)
    self._second_pull(gas=True)
    self._tick(self._cs(gas=False))
    self.assertTrue(self.adapter.engagement.enableLongControl)
    self.assertTrue(self.sd.enabled)
    self.assertTrue(self.mads.enabled)
    speed = self.adapter.engagement.pedal_speed_kph

    self._set_panda(True)
    self._hands_override(2)
    self.assertTrue(self.adapter.engagement.cruiseEnabled)
    self.assertTrue(self.adapter.engagement.enableLongControl)
    self.assertAlmostEqual(self.adapter.engagement.pedal_speed_kph, speed)
    self._publish()
    events, _, op_enabled, chimes = self._tick(self._pause_cs(hands=2, gas=False))
    self.assertFalse(events.has(EventName.steerDisengage))
    self.assertFalse(events.has(EventName.buttonCancel))
    self.assertFalse(self.events_sp.has(EventNameSP.lkasDisable))
    self.assertTrue(self.events_sp.has(EventNameSP.silentLkasDisable))
    self.assertTrue(op_enabled)
    self.assertTrue(self.adapter.engagement.enableLongControl)
    self.assertEqual(self.mads.state_machine.state, State.paused)
    self.assertTrue(self.mads.enabled)
    self.assertFalse(self.mads.active)
    self.assertFalse(chimes.long_disengage)
    self.assertAlmostEqual(self.adapter.engagement.pedal_speed_kph, speed)

  def test_pause_does_not_start_inactive_long(self):
    self._first_pull(gas=False)
    self.assertFalse(self.adapter.engagement.enableLongControl)
    self._set_panda(True)
    self._hands_override(2)
    self.adapter.engagement.process_buttons(
      CruiseButtons.MAIN, CruiseButtons.IDLE, 2500, 20.0, "KPH", True, True, True, False,
    )
    self.assertTrue(self.adapter.engagement.cruiseEnabled)
    self.assertFalse(self.adapter.engagement.enableLongControl)
    self._publish()
    _, _, op_enabled, _ = self._tick(self._pause_cs(hands=2))
    self.assertFalse(self.adapter.engagement.enableLongControl)
    self.assertFalse(op_enabled)

  def test_brake_drops_long_and_pause_does_not_resume_it(self):
    self._first_pull(gas=True)
    self._second_pull(gas=True)
    self._tick(self._cs(gas=False))
    self._pull(CruiseButtons.IDLE, CruiseButtons.IDLE, 2000, brake=True)
    self.assertFalse(self.adapter.engagement.enableLongControl)
    self._publish()
    self._tick(self._cs(gas=False))
    self._set_panda(True)
    self._hands_override(2)
    self.assertFalse(self.adapter.engagement.enableLongControl)
    self._publish()
    _, _, op_enabled, _ = self._tick(self._pause_cs(hands=2, gas=False))
    self.assertFalse(self.adapter.engagement.enableLongControl)
    self.assertFalse(op_enabled)
    self._set_panda(False)
    self._hands_override(0)
    self._publish()
    self._tick(self._pause_cs(hands=0, gas=False))
    self.assertFalse(self.adapter.engagement.enableLongControl)

  def test_epas_reject_full_disengage_with_pause_on(self):
    self._first_pull(gas=True)
    self._second_pull(gas=True)
    self._tick(self._cs(gas=False))
    self._hands_override(2, reject=True)
    self.assertFalse(self.adapter.engagement.cruiseEnabled)
    self.assertFalse(self.adapter.engagement.enableLongControl)
    self._publish()
    events, events_sp, op_enabled, _ = self._tick(self._pause_cs(hands=2, steering_disengage=True, gas=False))
    self.assertTrue(events.has(EventName.steerDisengage) or events_sp.has(EventNameSP.lkasDisable) or events.has(EventName.buttonCancel))
    self.assertFalse(op_enabled)
    self.assertFalse(self.mads.active)

  def test_default_off_still_full_disengages(self):
    off = PreAPCarState(self.CP, structs.CarParamsSP())
    off.engagement.cruiseEnabled = True
    off.engagement.enableLongControl = True
    off.engagement.pedal_speed_kph = 72.0
    off._epas_hands = 2
    off.engagement.handle_steering_disengage(True)
    self.assertFalse(off.engagement.cruiseEnabled)
    self.assertFalse(off.engagement.enableLongControl)
    self.assertEqual(off.engagement.pedal_speed_kph, 0.0)
    off.engagement.enableDoublePull = True
    off.engagement.process_buttons(
      CruiseButtons.MAIN, CruiseButtons.IDLE, 1000, 20.0, "KPH", True, True, True, False,
    )
    self.assertTrue(off.engagement.cruiseEnabled)
    self.assertFalse(off.engagement.enableLongControl)

  def test_revoked_pull_is_not_false_double_pull(self):
    self.adapter._epas_hands = 2
    self.adapter.engagement.stalk_pull_time_ms = 4000
    self.adapter.engagement.prev_stalk_pull_time_ms = 3900
    self.adapter.engagement.pending_enable = True
    ret = structs.CarState()
    self.adapter._revoke_unadmitted_held_hands(ret)
    self.assertEqual(self.adapter.engagement.stalk_pull_time_ms, 0)
    self.adapter._epas_hands = 0
    self.adapter.engagement.process_buttons(
      CruiseButtons.MAIN, CruiseButtons.IDLE, 4100, 20.0, "KPH", True, True, True, False,
    )
    self.assertTrue(self.adapter.engagement.cruiseEnabled)
    self.assertFalse(self.adapter.engagement.enableLongControl)

  def test_repeated_disengage_then_double_pull(self):
    t = 1000
    for _ in range(5):
      self._pull(CruiseButtons.MAIN, CruiseButtons.IDLE, t)
      t += 500
      self._pull(CruiseButtons.MAIN, CruiseButtons.IDLE, t)
      t += 1500
      self._pull(CruiseButtons.CANCEL, CruiseButtons.IDLE, t)
      t += 1500
    self.assertFalse(self.adapter.engagement.cruiseEnabled)
    self.assertFalse(self.adapter.engagement.enableLongControl)
    self._pull(CruiseButtons.MAIN, CruiseButtons.IDLE, t)
    self.assertTrue(self.adapter.engagement.cruiseEnabled)
    self.assertFalse(self.adapter.engagement.enableLongControl)
    self._pull(CruiseButtons.MAIN, CruiseButtons.IDLE, t + 400)
    self.assertTrue(self.adapter.engagement.enableLongControl)

  def test_recovery_hold_rejects_inactive_long_pull(self):
    self._first_pull(gas=False)
    self.assertFalse(self.adapter.engagement.enableLongControl)
    self._hands_override(2)
    self._pull(CruiseButtons.IDLE, CruiseButtons.IDLE, 2000)
    self.adapter._epas_hands = 0
    self._pull(CruiseButtons.IDLE, CruiseButtons.IDLE, 2000)
    self._pull(CruiseButtons.MAIN, CruiseButtons.IDLE, 2100)
    self._pull(CruiseButtons.MAIN, CruiseButtons.IDLE, 2500)
    self.assertTrue(self.adapter.engagement.cruiseEnabled)
    self.assertFalse(self.adapter.engagement.enableLongControl)
    self._publish()
    events, _, op_enabled, _ = self._tick(self._pause_cs(hands=0))
    self.assertFalse(events.has(EventName.buttonEnable))
    self.assertFalse(op_enabled)
    self.assertFalse(self.adapter.engagement.enableLongControl)

  def test_held_main_then_fresh_pull_after_recovery(self):
    self._first_pull(gas=False)
    self._hands_override(2)
    self._pull(CruiseButtons.IDLE, CruiseButtons.IDLE, 2000)
    self.adapter._epas_hands = 0
    self._pull(CruiseButtons.MAIN, CruiseButtons.IDLE, 2000)
    self._pull(CruiseButtons.IDLE, CruiseButtons.MAIN, 2200)
    self._pull(CruiseButtons.MAIN, CruiseButtons.IDLE, 2400)
    self._pull(CruiseButtons.MAIN, CruiseButtons.MAIN, 2000 + PREAP_HANDS_ON_RESUME_MS)
    self.assertFalse(self.adapter.engagement.enableLongControl)
    self._publish()
    events, _, op_enabled, _ = self._tick(self._pause_cs(hands=0))
    self.assertFalse(events.has(EventName.buttonEnable))
    self.assertFalse(op_enabled)

    self._pull(CruiseButtons.IDLE, CruiseButtons.MAIN, 2000 + PREAP_HANDS_ON_RESUME_MS + 100)
    self._pull(CruiseButtons.MAIN, CruiseButtons.IDLE, 2000 + PREAP_HANDS_ON_RESUME_MS + 200)
    self.assertFalse(self.adapter.engagement.enableLongControl)
    self._pull(CruiseButtons.MAIN, CruiseButtons.IDLE, 2000 + PREAP_HANDS_ON_RESUME_MS + 600)
    self.assertTrue(self.adapter.engagement.enableLongControl)
    self._publish()
    events, _, op_enabled, _ = self._tick(self._cs(gas=False))
    self.assertTrue(events.has(EventName.buttonEnable))
    self.assertTrue(op_enabled)

  def test_renewed_hands_resets_recovery_hold(self):
    self._first_pull(gas=False)
    self._hands_override(2)
    self._pull(CruiseButtons.IDLE, CruiseButtons.IDLE, 2000)
    self.adapter._epas_hands = 0
    self._pull(CruiseButtons.IDLE, CruiseButtons.IDLE, 2000)
    self._hands_override(2)
    self._pull(CruiseButtons.IDLE, CruiseButtons.IDLE, 2600)
    self.adapter._epas_hands = 0
    self._pull(CruiseButtons.IDLE, CruiseButtons.IDLE, 2600)
    self._pull(CruiseButtons.MAIN, CruiseButtons.IDLE, 2600 + PREAP_HANDS_ON_RESUME_MS - 1)
    self._pull(CruiseButtons.MAIN, CruiseButtons.IDLE, 2600 + PREAP_HANDS_ON_RESUME_MS - 1 + 400)
    self.assertFalse(self.adapter.engagement.enableLongControl)
    self._pull(CruiseButtons.IDLE, CruiseButtons.IDLE, 2600 + PREAP_HANDS_ON_RESUME_MS)
    self._pull(CruiseButtons.MAIN, CruiseButtons.IDLE, 2700 + PREAP_HANDS_ON_RESUME_MS)
    self._pull(CruiseButtons.MAIN, CruiseButtons.IDLE, 3100 + PREAP_HANDS_ON_RESUME_MS)
    self.assertTrue(self.adapter.engagement.enableLongControl)

  def test_recovery_hold_keeps_active_long_target(self):
    self._first_pull(gas=True)
    self._second_pull(gas=True)
    speed = self.adapter.engagement.pedal_speed_kph
    self._hands_override(2)
    self._pull(CruiseButtons.IDLE, CruiseButtons.IDLE, 2000)
    self.adapter._epas_hands = 0
    self._pull(CruiseButtons.MAIN, CruiseButtons.IDLE, 2000)
    self._pull(CruiseButtons.MAIN, CruiseButtons.IDLE, 2400)
    self.assertTrue(self.adapter.engagement.enableLongControl)
    self.assertAlmostEqual(self.adapter.engagement.pedal_speed_kph, speed)
    self._publish()
    events, _, op_enabled, _ = self._tick(self._pause_cs(hands=0, gas=False))
    self.assertFalse(events.has(EventName.buttonEnable))
    self.assertTrue(self.adapter.engagement.enableLongControl)
    self.assertTrue(op_enabled)
    self.assertAlmostEqual(self.adapter.engagement.pedal_speed_kph, speed)

  def test_epas_fault_clears_hold_unlike_hands_pause(self):
    self._first_pull(gas=True)
    self._second_pull(gas=True)
    self._tick(self._cs(gas=False))
    self._hands_override(2)
    self._pull(CruiseButtons.IDLE, CruiseButtons.IDLE, 2000)
    self._hands_override(2, fault=True)
    self.assertFalse(self.adapter.engagement.cruiseEnabled)
    self.assertFalse(self.adapter.engagement.enableLongControl)
    self.adapter._epas_hands = 0
    self.adapter._epas_fault = False
    self._pull(CruiseButtons.MAIN, CruiseButtons.IDLE, 2100)
    self.assertTrue(self.adapter.engagement.cruiseEnabled)
    self.assertFalse(self.adapter.engagement.enableLongControl)


def _decode_pedal_enable(command):
  address, data, _bus = command
  assert address == GAS_COMMAND_ID
  return bool(data[4] & 0x80), (data[0] << 8 | data[1]) * PEDAL_M1 + PEDAL_D


class TestPausePedalOutputContinuity(unittest.TestCase):
  def setUp(self):
    conf = SimpleNamespace(
      use_pedal=True,
      pedal_factor=1.0,
      pedal_can_zero=False,
      di_to_pedal=lambda pedal_di: pedal_di,
      pedal_to_di=lambda pedal: pedal,
      get_pedal_profile_values=lambda: PEDAL_MAX_VALUES,
    )
    zero = SimpleNamespace(get=lambda _v_ego: 0.0, update=lambda *_a, **_k: None)
    self.enterContext(patch("opendbc.car.tesla.preap.carcontroller.nap_conf", conf))
    self.enterContext(patch("opendbc.car.tesla.preap.carcontroller.get_zero_torque", lambda: zero))
    self.enterContext(patch("opendbc.car.tesla.preap.virtual_das.nap_conf", conf))
    self.enterContext(patch("opendbc.car.tesla.preap.carstate.nap_conf", conf))
    self.enterContext(patch("opendbc.car.tesla.preap.pedal_feedback.nap_conf", conf))
    self.enterContext(patch("opendbc.car.tesla.preap.virtual_das.get_zero_torque", lambda: zero))
    CP = structs.CarParams()
    CP.carFingerprint = CAR.TESLA_MODEL_S_PREAP
    CP_SP = structs.CarParamsSP()
    CP_SP.flags = int(TeslaFlagsSP.PREAP_HANDS_ON_PAUSE)
    self.cs = PreAPCarState(CP, CP_SP)
    self.cs.engagement.enableDoublePull = True
    self.cs.engagement.double_pull_window_ms = 750
    self.cs.pedal.available = True
    self.cs.pedal.timeout = False
    self.cs.pedal.interceptor_state = 0
    self.cs.pedal.idx = 1
    self.cs.real_brake_pressed = False
    self.cs.out.vEgo = 20.0
    self.cs.out.aEgo = 0.0
    self.cs.out.gasPressed = False
    self.cs.out.steeringDisengage = False
    self.cs.pedal_interceptor_value = 0.0
    self.cs.cruise_buttons = 0
    self.cs.prev_cruise_buttons = 0
    self.cs.pedal_timeout = False
    self.cs.pccEvent = None
    self.cs.preap_cc_cancel_needed = False
    self.cc = SimpleNamespace(
      actuators=SimpleNamespace(accel=0.2),
      longActive=True,
      orientationNED=[],
    )
    self.controller = PreAPLongController()
    self.controller.pedal_authority.state = PedalAuthorityState.ACTIVE
    self.controller.prev_requested_long = True
    self.can = TeslaCANPreAP({})

  def _sync_bridge(self):
    self.cs.cruiseEnabled = self.cs.engagement.cruiseEnabled
    self.cs.enableLongControl = self.cs.engagement.enableLongControl
    self.cs.enableJustCC = self.cs.engagement.enableJustCC
    self.cs.pedal_speed_kph = self.cs.engagement.pedal_speed_kph

  def _engage_long(self):
    self.cs.engagement.cruiseEnabled = True
    self.cs.engagement.enableLongControl = True
    self.cs.engagement.pedal_speed_kph = 72.0
    self._sync_bridge()

  def _adapter_pause(self):
    parsers = self.cs.get_can_parsers(self.cs.CP, self.cs.CP_SP)
    packer = CANPacker("tesla_preap")
    frames = []
    messages: tuple[tuple[str, int, dict[str, float]], ...] = (
      ("EPAS_sysStatus", 0, {"EPAS_handsOnLevel": 2}),
      ("DI_torque2", 0, {"DI_gear": 4}),
      ("SDM1", 0, {"SDM_bcklDrivStatus": 1}),
      ("ESP_B", 0, {"ESP_vehicleSpeed": 72}),
      ("STW_ACTN_RQ", 0, {"DTR_Dist_Rq": 255}),
      ("GAS_SENSOR", 2, {"IDX": 2}),
    )
    for name, bus, values in messages:
      address, data, source = packer.make_can_msg(name, bus, values)
      frames.append(CanData(address, data, source))
    for parser in parsers.values():
      parser.update([(2_000_000_000, frames)])
    with patch("opendbc.car.tesla.preap.carstate._current_time_millis", return_value=2000):
      self.cs.out, _ = self.cs.update(parsers)

  def _sends(self, frame):
    return self.controller.update(self.cc, self.cs, frame, self.can, 0, now_nanos=frame * 10_000_000)

  def test_enabled_pedal_output_continues_through_hands_on_pause(self):
    self._engage_long()
    sends = self._sends(2)
    self.assertTrue(sends)
    enabled, _ = _decode_pedal_enable(sends[0])
    self.assertTrue(enabled)
    target = self.cs.engagement.pedal_speed_kph

    self._adapter_pause()
    self.assertTrue(self.cs.engagement.cruiseEnabled)
    self.assertTrue(self.cs.engagement.enableLongControl)
    self.assertAlmostEqual(self.cs.engagement.pedal_speed_kph, target)
    self.assertFalse(self.cs.out.steeringDisengage)

    sends = self._sends(4)
    self.assertTrue(sends)
    enabled, _ = _decode_pedal_enable(sends[0])
    self.assertTrue(enabled)
    self.assertTrue(self.cs.enableLongControl)
    self.assertAlmostEqual(self.cs.engagement.pedal_speed_kph, target)

  def test_inactive_long_does_not_start_pedal_during_pause(self):
    self.cs.engagement.cruiseEnabled = True
    self.cs.engagement.enableLongControl = False
    self.cs.engagement.pedal_speed_kph = 0.0
    self._sync_bridge()
    self.controller.prev_requested_long = False
    self._adapter_pause()
    self.cs.engagement.process_buttons(
      CruiseButtons.IDLE, CruiseButtons.IDLE, 1000, 20.0, "KPH", True, True, True, False,
    )
    self.cs._epas_hands = 0
    self.cs.engagement.process_buttons(
      CruiseButtons.MAIN, CruiseButtons.IDLE, 1100, 20.0, "KPH", True, True, True, False,
    )
    self.cs.engagement.process_buttons(
      CruiseButtons.MAIN, CruiseButtons.IDLE, 1500, 20.0, "KPH", True, True, True, False,
    )
    self._sync_bridge()
    self.assertFalse(self.cs.enableLongControl)
    sends = self._sends(2)
    if sends:
      enabled, _ = _decode_pedal_enable(sends[0])
      self.assertFalse(enabled)


if __name__ == "__main__":
  unittest.main()
