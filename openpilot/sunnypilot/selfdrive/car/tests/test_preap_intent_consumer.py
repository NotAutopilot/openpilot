import unittest
from types import SimpleNamespace

from openpilot.cereal import custom, log
from opendbc.car import structs
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP
from opendbc.car.tesla.preap.sp.platform import preap_radar_present
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP
from opendbc.car.tesla.preap.sp.carstate import PreAPCarState
from opendbc.car.tesla.values import CAR
from openpilot.selfdrive.car.helpers import convert_to_capnp
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

  def test_adapter_hands_on_drops_long_keeps_lat(self):
    CP = structs.CarParams()
    CP.carFingerprint = CAR.TESLA_MODEL_S_PREAP
    CP_SP = structs.CarParamsSP()
    CP_SP.flags = int(TeslaFlagsSP.PREAP_HANDS_ON_PAUSE)
    cs = PreAPCarState(CP, CP_SP)
    cs.engagement.cruiseEnabled = True
    cs.engagement.enableLongControl = True
    cs._epas_hands = 2
    cs.engagement.handle_steering_disengage(True)
    self.assertTrue(cs.engagement.cruiseEnabled)
    self.assertFalse(cs.engagement.enableLongControl)
    self.assertFalse(cs.engagement.enableJustCC)
    self.assertTrue(cs._pause_cancel_this_tick)

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

if __name__ == "__main__":
  unittest.main()
