import unittest
from types import SimpleNamespace

from openpilot.cereal import custom, log
from opendbc.car import structs
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP
from openpilot.sunnypilot.selfdrive.car.preap_intent import (
  PreAPIntentConsumer, UINT32_HALF, UINT32_MASK, sequence_is_newer,
)

EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
Lateral = custom.CarStateSP.PreapLateralIntent
Longitudinal = custom.CarStateSP.PreapLongitudinalIntent


def record(epoch, sequence, lat=Lateral.none, lon=Longitudinal.none):
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

  def test_neutral_new_epoch_seeds_without_acting(self):
    self.apply(record(7, 0))
    self.assertTrue(self.consumer.seeded)
    self.assertFalse(self.events_sp.has(EventNameSP.lkasEnable))
    self.assertFalse(self.events.has(EventName.buttonEnable))
    self.apply(record(7, 0, Lateral.mainCruiseRequest))
    self.assertFalse(self.events_sp.has(EventNameSP.lkasEnable))

  def test_consumer_restart_seeds_current_without_acting(self):
    self.apply(record(3, 4, Lateral.mainCruiseRequest, Longitudinal.enable))
    self.assertFalse(self.events_sp.has(EventNameSP.lkasEnable))
    self.assertFalse(self.events.has(EventName.buttonEnable))

  def test_duplicate_and_older_ignored(self):
    self.apply(record(1, 1))
    self.apply(record(1, 2, Lateral.mainCruiseRequest))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasEnable))
    self.apply(record(1, 2, Lateral.mainCruiseRequest))
    self.assertFalse(self.events_sp.has(EventNameSP.lkasEnable))
    self.apply(record(1, 1, Lateral.forceDisable))
    self.assertFalse(self.events_sp.has(EventNameSP.lkasDisable))

  def test_latched_newer_recovers_loss(self):
    self.apply(record(1, 1))
    self.apply(record(1, 4, Lateral.forceDisable, Longitudinal.disable))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasDisable))
    self.assertTrue(self.events.has(EventName.buttonCancel))

  def test_enable_disable_late_enable(self):
    self.apply(record(1, 10))
    self.apply(record(1, 11, Lateral.mainCruiseRequest))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasEnable))
    self.apply(record(1, 12, Lateral.forceDisable))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasDisable))
    self.apply(record(1, 11, Lateral.mainCruiseRequest))
    self.assertFalse(self.events_sp.has(EventNameSP.lkasEnable))

  def test_rollover_progression(self):
    self.apply(record(1, UINT32_MASK))
    self.apply(record(1, 0, Lateral.mainCruiseRequest))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasEnable))

  def test_half_range_fail_closed(self):
    self.apply(record(1, 0))
    self.apply(record(1, UINT32_HALF, Lateral.mainCruiseRequest, Longitudinal.enable))
    self.assertTrue(self.consumer.fail_closed)
    self.assertTrue(self.events_sp.has(EventNameSP.lkasDisable))
    self.assertTrue(self.events.has(EventName.buttonCancel))
    self.assertFalse(self.events_sp.has(EventNameSP.lkasEnable))
    self.apply(record(1, UINT32_HALF + 1, Lateral.mainCruiseRequest))
    self.assertFalse(self.events_sp.has(EventNameSP.lkasEnable))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasDisable))

  def test_producer_restart_new_epoch_requires_neutral_seed(self):
    self.apply(record(1, 1))
    self.apply(record(1, 2, Lateral.mainCruiseRequest))
    self.apply(record(9, 2, Lateral.mainCruiseRequest, Longitudinal.enable))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasDisable))
    self.assertTrue(self.events.has(EventName.buttonCancel))
    self.apply(record(9, 3, Lateral.mainCruiseRequest))
    self.assertFalse(self.events_sp.has(EventNameSP.lkasEnable))

  def test_disable_before_enable_same_record(self):
    self.apply(record(1, 1))
    self.apply(record(1, 2, Lateral.forceDisable, Longitudinal.enable))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasDisable))
    self.assertTrue(self.events.has(EventName.buttonEnable))

  def test_event_mapping(self):
    self.apply(record(1, 1))
    self.apply(record(1, 2, Lateral.mainCruiseRequest, Longitudinal.enable))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasEnable))
    self.assertTrue(self.events.has(EventName.buttonEnable))
    self.apply(record(1, 3, Lateral.forceDisable, Longitudinal.disable))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasDisable))
    self.assertTrue(self.events.has(EventName.buttonCancel))

  def test_no_longitudinal_when_not_op_long(self):
    self.apply(record(1, 1))
    self.apply(record(1, 2, Lateral.none, Longitudinal.enable), apply_longitudinal=False)
    self.assertFalse(self.events.has(EventName.buttonEnable))

  def test_zero_epoch_ignored(self):
    self.apply(record(0, 5, Lateral.mainCruiseRequest))
    self.assertFalse(self.consumer.seeded)
    self.assertFalse(self.events_sp.has(EventNameSP.lkasEnable))

  def test_car_specific_events_uses_consumer_before_brand_logic(self):
    CP = structs.CarParams()
    CP.brand = "tesla"
    CP.carFingerprint = "TESLA_MODEL_S_PREAP"
    CP.openpilotLongitudinalControl = True
    CP_SP = structs.CarParamsSP()
    cse = CarSpecificEventsSP(CP, CP_SP)
    events = Events()
    cs = structs.CarState()
    cse.update(cs, events, record(4, 0))
    events_sp = cse.update(cs, events, record(4, 1, Lateral.mainCruiseRequest, Longitudinal.enable))
    self.assertTrue(events_sp.has(EventNameSP.lkasEnable))
    self.assertTrue(events.has(EventName.buttonEnable))

  def test_unaligned_none_does_not_act(self):
    CP = structs.CarParams()
    CP.brand = "hyundai"
    cse = CarSpecificEventsSP(CP, structs.CarParamsSP())
    events = Events()
    events_sp = cse.update(structs.CarState(), events, None)
    self.assertFalse(events_sp.has(EventNameSP.lkasEnable))


if __name__ == "__main__":
  unittest.main()
