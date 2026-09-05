import unittest
from types import SimpleNamespace

from opendbc.car import structs
from openpilot.cereal import custom, log
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP
from openpilot.sunnypilot.selfdrive.car.preap_intent import PreAPIntentConsumer

EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
Lateral = custom.CarStateSP.PreapLateralIntent
Longitudinal = custom.CarStateSP.PreapLongitudinalIntent
StockCc = custom.CarStateSP.PreapStockCcTransactionState


def record(epoch, sequence, lat=Lateral.none, lon=Longitudinal.none, *, pending=False, bound=0, host_di=False, state=None):
  return SimpleNamespace(
    preapIntentEpoch=0,
    preapIntentSequence=sequence,
    preapLateralIntent=lat,
    preapLongitudinalIntent=lon,
    preapStockCcEnablePending=pending,
    preapStockCcBoundCounter=bound,
    preapStockCcHostDiConfirmed=host_di,
    preapStockCcState=state or StockCc.idle,
  )


class TestPreAPStockCcConsumer(unittest.TestCase):
  def setUp(self):
    self.consumer = PreAPIntentConsumer()
    self.events = Events()
    self.events_sp = EventsSP()

  def apply(self, rec, apply_longitudinal=False):
    self.events.clear()
    self.events_sp.clear()
    self.consumer.update(rec, self.events, self.events_sp, apply_longitudinal=apply_longitudinal)

  def test_exactly_one_button_enable_then_release(self):
    self.apply(record(1, 1))
    self.apply(record(1, 2, lon=Longitudinal.enable, pending=True, bound=5, host_di=True, state=StockCc.confirmed))
    self.assertTrue(self.events.has(EventName.buttonEnable))
    self.apply(record(1, 2, lon=Longitudinal.enable, pending=True, bound=5, host_di=True, state=StockCc.confirmed))
    self.assertFalse(self.events.has(EventName.buttonEnable))
    self.apply(record(1, 2, pending=True, bound=5, host_di=True, state=StockCc.confirmed))
    self.assertFalse(self.events.has(EventName.buttonEnable))

  def test_generic_long_intent_does_not_enable_when_not_op_long(self):
    self.apply(record(1, 1))
    self.apply(record(1, 2, lon=Longitudinal.enable), apply_longitudinal=False)
    self.assertFalse(self.events.has(EventName.buttonEnable))

  def test_pending_emits_on_repeated_sequence(self):
    self.apply(record(1, 4))
    self.apply(record(1, 5, lat=Lateral.mainCruiseRequest))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasEnable))
    self.apply(record(1, 5, pending=True, bound=9, host_di=True, state=StockCc.confirmed))
    self.assertTrue(self.events.has(EventName.buttonEnable))
    self.assertFalse(self.events_sp.has(EventNameSP.lkasEnable))

  def test_car_specific_no_pedal_emits_stock_cc_not_generic_pcm(self):
    CP = structs.CarParams()
    CP.brand = "tesla"
    CP.carFingerprint = "TESLA_MODEL_S_PREAP"
    CP.openpilotLongitudinalControl = False
    cse = CarSpecificEventsSP(CP, structs.CarParamsSP())
    events = Events()
    cs = structs.CarState()
    cs.blockPcmEnable = True
    cse.update(cs, events, record(4, 0))
    cse.update(cs, events, record(4, 1, lon=Longitudinal.enable))
    self.assertFalse(events.has(EventName.buttonEnable))
    events.clear()
    cse.update(cs, events, record(4, 1, pending=True, bound=3, host_di=True, state=StockCc.confirmed))
    self.assertTrue(events.has(EventName.buttonEnable))
    events.clear()
    cse.update(cs, events, record(4, 1, pending=True, bound=3, host_di=True, state=StockCc.confirmed))
    self.assertFalse(events.has(EventName.buttonEnable))

  def test_restart_seeds_true_pending_as_consumed(self):
    self.apply(record(1, 7, pending=True, bound=4, host_di=True, state=StockCc.confirmed))
    self.assertFalse(self.events.has(EventName.buttonEnable))
    self.apply(record(1, 7, pending=True, bound=4, host_di=True, state=StockCc.confirmed))
    self.assertFalse(self.events.has(EventName.buttonEnable))
    self.apply(record(1, 8, pending=False, bound=4, host_di=False, state=StockCc.idle))
    self.assertFalse(self.events.has(EventName.buttonEnable))
    self.apply(record(1, 9, pending=True, bound=5, host_di=True, state=StockCc.confirmed))
    self.assertTrue(self.events.has(EventName.buttonEnable))

  def test_unrelated_newer_intent_while_pending_does_not_emit(self):
    self.apply(record(1, 1))
    self.apply(record(1, 2, pending=True, bound=1, host_di=True, state=StockCc.confirmed))
    self.assertTrue(self.events.has(EventName.buttonEnable))
    self.apply(record(1, 3, lat=Lateral.mainCruiseRequest, pending=True, bound=1, host_di=True, state=StockCc.confirmed))
    self.assertFalse(self.events.has(EventName.buttonEnable))
    self.apply(record(1, 4, pending=True, bound=2, host_di=True, state=StockCc.confirmed))
    self.assertFalse(self.events.has(EventName.buttonEnable))

  def test_false_to_true_after_uint8_wrap_emits(self):
    self.apply(record(1, 1))
    self.apply(record(1, 2, pending=True, bound=255, host_di=True, state=StockCc.confirmed))
    self.assertTrue(self.events.has(EventName.buttonEnable))
    self.apply(record(1, 3, pending=False, bound=255, host_di=False, state=StockCc.idle))
    self.assertFalse(self.events.has(EventName.buttonEnable))
    self.apply(record(1, 4, pending=True, bound=0, host_di=True, state=StockCc.confirmed))
    self.assertTrue(self.events.has(EventName.buttonEnable))

  def test_cancel_requested_emits_button_cancel_when_not_op_long(self):
    self.apply(record(1, 1))
    self.apply(record(1, 2, state=StockCc.cancelRequested), apply_longitudinal=False)
    self.assertTrue(self.events.has(EventName.buttonCancel))
    self.assertFalse(self.events.has(EventName.buttonEnable))
    self.apply(record(1, 3, lon=Longitudinal.enable), apply_longitudinal=False)
    self.assertFalse(self.events.has(EventName.buttonEnable))
    self.assertFalse(self.events.has(EventName.buttonCancel))

  def test_coupled_pending_emits_lkas_and_button_enable_atomically(self):
    self.apply(record(1, 1))
    self.apply(
      record(1, 2, lat=Lateral.mainCruiseRequest, lon=Longitudinal.enable, pending=True,
             bound=5, host_di=True, state=StockCc.confirmed),
      apply_longitudinal=False,
    )
    self.assertTrue(self.events.has(EventName.buttonEnable))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasEnable))
    self.apply(
      record(1, 2, lat=Lateral.mainCruiseRequest, lon=Longitudinal.enable, pending=True,
             bound=5, host_di=True, state=StockCc.confirmed),
      apply_longitudinal=False,
    )
    self.assertFalse(self.events.has(EventName.buttonEnable))
    self.assertFalse(self.events_sp.has(EventNameSP.lkasEnable))

  def test_reorder_false_cannot_rearm_consumed_pending(self):
    self.apply(record(1, 1, pending=False))
    self.apply(record(1, 2, pending=True, bound=2, host_di=True, state=StockCc.confirmed))
    self.assertTrue(self.events.has(EventName.buttonEnable))
    self.apply(record(1, 1, pending=False))
    self.assertFalse(self.events.has(EventName.buttonEnable))
    self.apply(record(1, 2, pending=True, bound=2, host_di=True, state=StockCc.confirmed))
    self.assertFalse(self.events.has(EventName.buttonEnable))
    self.apply(record(1, 3, pending=False, state=StockCc.idle))
    self.assertFalse(self.events.has(EventName.buttonEnable))
    self.apply(record(1, 4, pending=True, bound=3, host_di=True, state=StockCc.confirmed))
    self.assertTrue(self.events.has(EventName.buttonEnable))

  def test_same_sequence_rollback_cannot_rearm(self):
    self.apply(record(1, 1))
    self.apply(record(1, 2, pending=True, bound=1, host_di=True, state=StockCc.confirmed))
    self.assertTrue(self.events.has(EventName.buttonEnable))
    self.apply(record(1, 2, pending=False))
    self.assertFalse(self.events.has(EventName.buttonEnable))
    self.apply(record(1, 2, pending=True, bound=1, host_di=True, state=StockCc.confirmed))
    self.assertFalse(self.events.has(EventName.buttonEnable))

  def test_cancelled_or_failed_emits_once_and_clears_deferred(self):
    self.apply(record(1, 1))
    self.apply(record(1, 2, lat=Lateral.mainCruiseRequest))
    self.apply(
      record(1, 3, lat=Lateral.forceDisable, lon=Longitudinal.disable, state=StockCc.cancelledOrFailed),
      apply_longitudinal=False,
    )
    self.assertTrue(self.events.has(EventName.buttonCancel))
    self.assertTrue(self.events_sp.has(EventNameSP.lkasDisable))
    self.apply(
      record(1, 3, lat=Lateral.forceDisable, lon=Longitudinal.disable, state=StockCc.cancelledOrFailed),
      apply_longitudinal=False,
    )
    self.assertFalse(self.events.has(EventName.buttonCancel))
    self.apply(record(1, 4, pending=True, bound=1, host_di=True, state=StockCc.confirmed), apply_longitudinal=False)
    self.assertTrue(self.events.has(EventName.buttonEnable))

  def test_skipped_confirmation_force_disable_cannot_emit_button_enable(self):
    self.apply(record(1, 1, state=StockCc.awaitingDiConfirmation, pending=False), apply_longitudinal=False)
    self.assertFalse(self.events.has(EventName.buttonEnable))
    self.apply(
      record(
        1, 2,
        lat=Lateral.forceDisable, lon=Longitudinal.disable,
        pending=True, bound=5, host_di=True, state=StockCc.confirmed,
      ),
      apply_longitudinal=False,
    )
    self.assertTrue(self.events_sp.has(EventNameSP.lkasDisable))
    self.assertFalse(self.events.has(EventName.buttonEnable))

  def test_skipped_confirmation_panda_loss_cannot_emit_button_enable(self):
    self.apply(record(1, 1, state=StockCc.awaitingDiConfirmation, pending=False), apply_longitudinal=False)
    self.assertFalse(self.events.has(EventName.buttonEnable))
    self.apply(
      record(
        1, 2,
        lat=Lateral.forceDisable, lon=Longitudinal.disable,
        pending=False, bound=5, host_di=False, state=StockCc.cancelledOrFailed,
      ),
      apply_longitudinal=False,
    )
    self.assertTrue(self.events_sp.has(EventNameSP.lkasDisable))
    self.assertFalse(self.events.has(EventName.buttonEnable))

  def test_force_disable_dominates_pending_and_enable_on_one_record(self):
    self.apply(record(1, 1), apply_longitudinal=False)
    self.apply(
      record(
        1, 2,
        lat=Lateral.forceDisable, lon=Longitudinal.enable,
        pending=True, bound=5, host_di=True, state=StockCc.confirmed,
      ),
      apply_longitudinal=False,
    )
    self.assertTrue(self.events_sp.has(EventNameSP.lkasDisable))
    self.assertFalse(self.events.has(EventName.buttonEnable))
    self.assertFalse(self.events_sp.has(EventNameSP.lkasEnable))


if __name__ == "__main__":
  unittest.main()
