import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car import CanData, gen_empty_fingerprint, structs
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.preap.boot import apply_preap_hardware_snapshot, hardware_snapshot_from_values
from opendbc.car.tesla.values import CAR, CruiseButtons
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
    preapIntentEpoch=epoch,
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
    # Consumer is awaiting DI confirmation with pending=false. The producer
    # confirms, but that intermediate record is not delivered. The next
    # record is a newer forceDisable that still carries pending=true.
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
    # Confirmation publication is skipped. The next observed record is the
    # post-Panda-loss terminal disable with pending and host confirmation cleared.
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


def _packet(name, values, bus=0, ts=1):
  addr, dat, bus = CANPacker("tesla_preap").make_can_msg(name, bus, values)
  return [(ts, [CanData(addr, dat, bus)])]


def _produced_nopedal_coupled_ci():
  CP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  apply_preap_hardware_snapshot(
    CP, CP_SP, hardware_snapshot_from_values(engagement_mode="cruiseCoupled"),
  )
  return CP, CP_SP, CarInterface(CP, CP_SP)


def _prime_drive(CI, ts=1_000_000):
  packets = []
  packets += _packet("ESP_B", {"ESP_vehicleSpeed": 36.0}, ts=ts)
  packets += _packet("DI_torque2", {"DI_brakePedal": 0, "DI_gear": 4, "DI_brakePedalState": 0}, ts=ts)
  packets += _packet("BrakeMessage", {"driverBrakeStatus": 1}, ts=ts)
  packets += _packet("DI_torque1", {"DI_pedalPos": 0}, ts=ts)
  packets += _packet("DI_state", {"DI_cruiseState": 0, "DI_speedUnits": 1, "DI_digitalSpeed": 20}, ts=ts)
  packets += _packet("EPAS_sysStatus", {"EPAS_internalSAS": 0, "EPAS_torsionBarTorque": 0, "EPAS_handsOnLevel": 0,
                                        "EPAS_eacStatus": 1, "EPAS_eacErrorCode": 0}, ts=ts)
  packets += _packet("STW_ANGLHP_STAT", {"StW_AnglHP_Spd": 0}, ts=ts)
  packets += _packet("GTW_carState", {
    "DOOR_STATE_FL": 0, "DOOR_STATE_FR": 0, "DOOR_STATE_RL": 0, "DOOR_STATE_RR": 0,
    "DOOR_STATE_FrontTrunk": 0, "BOOT_STATE": 0, "BC_indicatorLStatus": 0, "BC_indicatorRStatus": 0,
  }, ts=ts)
  CI.update(packets)


def _from_produced(cs_sp, epoch):
  return record(
    epoch,
    int(cs_sp.preapIntentSequence),
    lat=getattr(Lateral, cs_sp.preapLateralIntent.name),
    lon=getattr(Longitudinal, cs_sp.preapLongitudinalIntent.name),
    pending=bool(cs_sp.preapStockCcEnablePending),
    bound=int(cs_sp.preapStockCcBoundCounter),
    host_di=bool(cs_sp.preapStockCcHostDiConfirmed),
    state=getattr(StockCc, cs_sp.preapStockCcState.name),
  )



def _produced_nopedal_independent_ci():
  CP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  apply_preap_hardware_snapshot(
    CP, CP_SP, hardware_snapshot_from_values(engagement_mode="independent"),
  )
  return CP, CP_SP, CarInterface(CP, CP_SP)


def _force_confirmed(CI, stalk_counter=5):
  t = CI.CS.stock_cc
  t.state = structs.CarStateSP.PreapStockCcTransactionState.confirmed
  t.enable_pending = True
  t.host_di_confirmed = True
  t._need_release = False
  t._blocked = False
  t._panda_counter_at_bind = 0
  t.bound_counter = 1
  t.sync_counter(stalk_counter)
  t._prev_lever = 0
  return t


class TestNoPedalLogicalActiveMadsEvents(unittest.TestCase):
  def _consume(self, seed, action):
    consumer = PreAPIntentConsumer()
    events = Events()
    events_sp = EventsSP()
    consumer.update(seed, events, events_sp, apply_longitudinal=False)
    events.clear()
    events_sp.clear()
    consumer.update(action, events, events_sp, apply_longitudinal=False)
    return events, events_sp

  def test_produced_nopedal_coupled_first_pull_emits_force_disable_and_button_cancel(self):
    CP, _CP_SP, CI = _produced_nopedal_coupled_ci()
    self.assertFalse(CP.openpilotLongitudinalControl)
    self.assertTrue(CP.pcmCruise)
    self.assertTrue(CI.CS.stock_cc.active)
    _prime_drive(CI)
    CC = structs.CarControl()
    CC.enabled = True
    CC.longActive = False
    CI.apply(CC, structs.CarControlSP(), now_nanos=0)
    self.assertTrue(CI.CS.intent.long_active)

    CI.update(_packet("STW_ACTN_RQ", {"SpdCtrlLvr_Stat": 0, "MC_STW_ACTN_RQ": 0}, ts=2_000_000))
    _cs, CS_SP = CI.update(_packet("STW_ACTN_RQ", {"SpdCtrlLvr_Stat": 2, "MC_STW_ACTN_RQ": 1}, ts=2_000_001))
    self.assertEqual(CS_SP.preapLateralIntent.name, "forceDisable")
    self.assertEqual(CS_SP.preapLongitudinalIntent.name, "disable")
    events, events_sp = self._consume(record(5, 0), _from_produced(CS_SP, 5))
    self.assertTrue(events_sp.has(EventNameSP.lkasDisable))
    self.assertTrue(events.has(EventName.buttonCancel))

  def test_produced_nopedal_coupled_brake_emits_force_disable(self):
    CP, _CP_SP, CI = _produced_nopedal_coupled_ci()
    self.assertFalse(CP.openpilotLongitudinalControl)
    _prime_drive(CI)
    CC = structs.CarControl()
    CC.enabled = True
    CC.longActive = False
    CI.apply(CC, structs.CarControlSP(), now_nanos=0)
    self.assertTrue(CI.CS.intent.long_active)

    _cs, CS_SP = CI.update(_packet("BrakeMessage", {"driverBrakeStatus": 2}, ts=2_000_000))
    self.assertEqual(CS_SP.preapLateralIntent.name, "forceDisable")
    self.assertEqual(CS_SP.preapLongitudinalIntent.name, "disable")
    events, events_sp = self._consume(record(5, 0), _from_produced(CS_SP, 5))
    self.assertTrue(events_sp.has(EventNameSP.lkasDisable))

  def _confirmed_di_fall(self, CI):
    frozen = [2_000_000_000]
    CI.CS._clock_ns = lambda: frozen[0]
    _prime_drive(CI, ts=1_000_000)
    t = _force_confirmed(CI)
    CI.update(_packet("DI_state", {"DI_cruiseState": 2, "DI_speedUnits": 1, "DI_digitalSpeed": 20}, ts=1_500_000))
    self.assertEqual(t.state, structs.CarStateSP.PreapStockCcTransactionState.confirmed)
    _cs, CS_SP = CI.update(_packet(
      "DI_state", {"DI_cruiseState": 0, "DI_speedUnits": 1, "DI_digitalSpeed": 20}, ts=2_000_000,
    ))
    return CS_SP

  def test_produced_nopedal_independent_confirmed_di_fall_does_not_lkas_disable(self):
    _CP, _CP_SP, CI = _produced_nopedal_independent_ci()
    CS_SP = self._confirmed_di_fall(CI)
    self.assertEqual(CS_SP.preapLateralIntent.name, "none")
    self.assertEqual(CS_SP.preapLongitudinalIntent.name, "disable")
    self.assertEqual(CS_SP.preapStockCcState.name, "cancelledOrFailed")
    events, events_sp = self._consume(record(5, 0), _from_produced(CS_SP, 5))
    self.assertFalse(events_sp.has(EventNameSP.lkasDisable))
    self.assertTrue(events.has(EventName.buttonCancel))

  def test_produced_nopedal_coupled_confirmed_di_fall_emits_lkas_disable(self):
    _CP, _CP_SP, CI = _produced_nopedal_coupled_ci()
    CS_SP = self._confirmed_di_fall(CI)
    self.assertEqual(CS_SP.preapLateralIntent.name, "forceDisable")
    self.assertEqual(CS_SP.preapLongitudinalIntent.name, "disable")
    self.assertEqual(CS_SP.preapStockCcState.name, "cancelledOrFailed")
    events, events_sp = self._consume(record(5, 0), _from_produced(CS_SP, 5))
    self.assertTrue(events_sp.has(EventNameSP.lkasDisable))
    self.assertTrue(events.has(EventName.buttonCancel))


STW_ADDR = 0x45
IDLE = CruiseButtons.IDLE
MAIN = CruiseButtons.MAIN
CANCEL = CruiseButtons.CANCEL
SET_ACCEL = CruiseButtons.SET_ACCEL
PASSTHROUGH_LEVERS = (
  CruiseButtons.RES_ACCEL,
  CruiseButtons.RES_ACCEL_2ND,
  CruiseButtons.DECEL_SET,
  CruiseButtons.DECEL_2ND,
)
FORBIDDEN_TX_LEVERS = (MAIN, CruiseButtons.RES_ACCEL_2ND, CruiseButtons.DECEL_SET, CruiseButtons.DECEL_2ND)


def _stw(lever, counter, ts):
  return _packet("STW_ACTN_RQ", {"SpdCtrlLvr_Stat": lever, "MC_STW_ACTN_RQ": counter}, ts=ts)


def _di(enabled, ts):
  return _packet("DI_state", {
    "DI_cruiseState": 2 if enabled else 0,
    "DI_speedUnits": 1,
    "DI_digitalSpeed": 20,
  }, ts=ts)


class TestProducedNopedalCoupledDirectAdjustment(unittest.TestCase):
  def _drain_stock_cc_tx(self, CI, frames=40):
    CC = structs.CarControl()
    CC_SP = structs.CarControlSP()
    sent = []
    echo = None
    for _ in range(frames):
      _act, msgs = CI.apply(CC, CC_SP, now_nanos=0)
      for addr, dat, bus in msgs:
        if addr != STW_ADDR:
          continue
        lever = dat[0] & 0x3F
        sent.append(lever)
        echo = (addr, dat, bus)
    return sent, echo

  def _echo(self, CI, echo, ts):
    addr, dat, bus = echo
    CI.update([(ts, [CanData(addr, dat, bus)])])

  def _consume(self, seed, action):
    consumer = PreAPIntentConsumer()
    events = Events()
    events_sp = EventsSP()
    consumer.update(seed, events, events_sp, apply_longitudinal=False)
    events.clear()
    events_sp.clear()
    consumer.update(action, events, events_sp, apply_longitudinal=False)
    return events, events_sp

  def test_direct_adjustment_levers_produce_atomic_lkas_and_button_enable(self):
    for lever in PASSTHROUGH_LEVERS:
      with self.subTest(lever=lever):
        CP, _CP_SP, CI = _produced_nopedal_coupled_ci()
        self.assertFalse(CP.openpilotLongitudinalControl)
        self.assertTrue(CP.pcmCruise)
        self.assertTrue(CI.CS.stock_cc.active)
        frozen = [0]
        CI.CS._clock_ns = lambda: frozen[0]
        _prime_drive(CI, ts=1_000_000)
        CI.apply(structs.CarControl(), structs.CarControlSP(), now_nanos=0)
        self.assertFalse(CI.CS.intent.long_active)

        CI.update(_stw(IDLE, 0, 2_000_000))
        CI.update(_stw(MAIN, 1, 2_000_001))
        origin = CI.CS.intent._first_pull_ms
        self.assertEqual(origin, 0)
        CI.update(_stw(lever, 2, 2_000_002))
        self.assertEqual(CI.CS.intent._first_pull_ms, origin)

        cancel_tx, cancel_echo = self._drain_stock_cc_tx(CI)
        self.assertEqual(cancel_tx, [CANCEL])
        self.assertNotIn(MAIN, cancel_tx)
        self.assertTrue(all(tx not in FORBIDDEN_TX_LEVERS for tx in cancel_tx))
        self._echo(CI, cancel_echo, 2_100_000)

        CI.update(_di(False, 3_000_000))
        self.assertTrue(CI.CS.stock_cc._post_cancel_di)

        frozen[0] = 399_000_000
        CI.update(_stw(IDLE, 4, 4_000_000))
        _cs, CS_SP = CI.update(_stw(MAIN, 5, 4_000_001))
        self.assertTrue(CI.CS.intent._coupled_deferred)
        self.assertEqual(CS_SP.preapLateralIntent.name, "none")

        set_tx, set_echo = self._drain_stock_cc_tx(CI)
        self.assertEqual(set_tx, [SET_ACCEL])
        self.assertNotIn(MAIN, set_tx)
        self.assertTrue(all(tx not in FORBIDDEN_TX_LEVERS for tx in set_tx))
        self._echo(CI, set_echo, 4_100_000)

        CI.update(_di(True, 5_000_000))
        CI.CS.update_stock_cc_panda(SimpleNamespace(
          stockCcReengageCounter=CI.CS.stock_cc.bound_counter,
          stockCcReengageConfirmed=True,
          controlsAllowedLongitudinal=True,
        ))
        _cs, CS_SP = CI.update([])
        self.assertEqual(CS_SP.preapLateralIntent.name, "mainCruiseRequest")
        self.assertEqual(CS_SP.preapLongitudinalIntent.name, "enable")
        self.assertTrue(CS_SP.preapStockCcEnablePending)
        self.assertTrue(CS_SP.preapStockCcHostDiConfirmed)
        self.assertEqual(CS_SP.preapStockCcState.name, "confirmed")

        events, events_sp = self._consume(record(5, 0), _from_produced(CS_SP, 5))
        self.assertTrue(events_sp.has(EventNameSP.lkasEnable))
        self.assertTrue(events.has(EventName.buttonEnable))
        self.assertFalse(events_sp.has(EventNameSP.lkasDisable))
        self.assertFalse(events.has(EventName.buttonCancel))
