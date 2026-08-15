from types import SimpleNamespace
from unittest.mock import patch
from typing import Any, cast

from opendbc.car import structs
from opendbc.car.tesla.preap.stock_cc import StockCcTransaction
from opendbc.car.tesla.values import CruiseButtons
from openpilot.cereal import custom, log
from openpilot.selfdrive.car.card import Car
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.selfdrive.car.preap_intent import PreAPIntentConsumer

EventName = log.OnroadEvent.EventName
Lateral = custom.CarStateSP.PreapLateralIntent
Longitudinal = custom.CarStateSP.PreapLongitudinalIntent
StockCc = custom.CarStateSP.PreapStockCcTransactionState


def _record(sequence, lateral, longitudinal):
  record = structs.CarStateSP()
  record.preapIntentSequence = sequence
  record.preapLateralIntent = lateral
  record.preapLongitudinalIntent = longitudinal
  return record


def test_new_epoch_publishes_neutral_seed_before_latched_intent():
  lateral = structs.CarStateSP.PreapLateralIntent
  longitudinal = structs.CarStateSP.PreapLongitudinalIntent
  card = Car.__new__(Car)
  card.preap_intent_epoch = 42
  card.preap_intent_seed_pending = True

  seed = _record(7, lateral.mainCruiseRequest, longitudinal.enable)
  card.stamp_preap_intent_epoch(seed)
  action = _record(7, lateral.mainCruiseRequest, longitudinal.enable)
  card.stamp_preap_intent_epoch(action)

  assert seed.preapIntentEpoch == 42
  assert seed.preapIntentSequence == 0
  assert seed.preapLateralIntent == lateral.none
  assert seed.preapLongitudinalIntent == longitudinal.none
  assert action.preapIntentEpoch == 42
  assert action.preapIntentSequence == 7
  assert action.preapLateralIntent == lateral.mainCruiseRequest
  assert action.preapLongitudinalIntent == longitudinal.enable


def _stock_record(*, sequence, state, pending=False, confirmed=False, bound=0, lateral=None, longitudinal=None):
  record = structs.CarStateSP()
  record.preapIntentSequence = sequence
  record.preapLateralIntent = lateral or structs.CarStateSP.PreapLateralIntent.none
  record.preapLongitudinalIntent = longitudinal or structs.CarStateSP.PreapLongitudinalIntent.none
  record.preapStockCcState = state
  record.preapStockCcEnablePending = pending
  record.preapStockCcHostDiConfirmed = confirmed
  record.preapStockCcBoundCounter = bound
  return record


def _consumer_view(cs_sp):
  return SimpleNamespace(
    preapIntentEpoch=int(cs_sp.preapIntentEpoch),
    preapIntentSequence=int(cs_sp.preapIntentSequence),
    preapLateralIntent=getattr(Lateral, cs_sp.preapLateralIntent.name),
    preapLongitudinalIntent=getattr(Longitudinal, cs_sp.preapLongitudinalIntent.name),
    preapStockCcEnablePending=bool(cs_sp.preapStockCcEnablePending),
    preapStockCcBoundCounter=int(cs_sp.preapStockCcBoundCounter),
    preapStockCcHostDiConfirmed=bool(cs_sp.preapStockCcHostDiConfirmed),
    preapStockCcState=getattr(StockCc, cs_sp.preapStockCcState.name),
  )


def test_seed_neutralizes_stockcc_projection_and_sequence_1_exposes_edge():
  StockCc = structs.CarStateSP.PreapStockCcTransactionState
  card = Car.__new__(Car)
  card.preap_intent_epoch = 42
  card.preap_intent_seed_pending = True

  live_state = StockCc.cancelRequested
  seed = _stock_record(sequence=1, state=live_state, pending=True, confirmed=True, bound=4)
  card.stamp_preap_intent_epoch(seed)

  assert seed.preapIntentEpoch == 42
  assert seed.preapIntentSequence == 0
  assert seed.preapLateralIntent == structs.CarStateSP.PreapLateralIntent.none
  assert seed.preapLongitudinalIntent == structs.CarStateSP.PreapLongitudinalIntent.none
  assert seed.preapStockCcState == StockCc.idle
  assert seed.preapStockCcEnablePending is False
  assert seed.preapStockCcHostDiConfirmed is False
  assert seed.preapStockCcBoundCounter == 0

  action = _stock_record(sequence=1, state=live_state, pending=False, confirmed=False, bound=4)
  card.stamp_preap_intent_epoch(action)
  assert action.preapIntentEpoch == 42
  assert action.preapIntentSequence == 1
  assert action.preapStockCcState == live_state

  consumer = PreAPIntentConsumer()
  events = Events()
  events_sp = EventsSP()
  consumer.update(_consumer_view(seed), events, events_sp, apply_longitudinal=False)
  assert not events.has(EventName.buttonCancel)
  events.clear()
  events_sp.clear()
  consumer.update(_consumer_view(action), events, events_sp, apply_longitudinal=False)
  assert events.has(EventName.buttonCancel)


def test_producer_restart_initial_batch_exposes_cancel_edge_after_new_seed():
  StockCc = structs.CarStateSP.PreapStockCcTransactionState
  card = Car.__new__(Car)
  card.preap_intent_epoch = 7
  card.preap_intent_seed_pending = True
  first_seed = _stock_record(sequence=1, state=StockCc.cancelRequested, bound=3)
  card.stamp_preap_intent_epoch(first_seed)
  first_action = _stock_record(sequence=1, state=StockCc.cancelRequested, bound=3)
  card.stamp_preap_intent_epoch(first_action)

  # Producer restart: new epoch, live internal cancel remains, seed must not consume it.
  card.preap_intent_epoch = 8
  card.preap_intent_seed_pending = True
  restart_seed = _stock_record(sequence=1, state=StockCc.cancelRequested, bound=3)
  card.stamp_preap_intent_epoch(restart_seed)
  assert restart_seed.preapIntentEpoch == 8
  assert restart_seed.preapIntentSequence == 0
  assert restart_seed.preapStockCcState == StockCc.idle
  assert restart_seed.preapStockCcEnablePending is False
  assert restart_seed.preapStockCcHostDiConfirmed is False

  restart_action = _stock_record(sequence=1, state=StockCc.cancelRequested, bound=3)
  card.stamp_preap_intent_epoch(restart_action)
  assert restart_action.preapIntentSequence == 1
  assert restart_action.preapStockCcState == StockCc.cancelRequested

  consumer = PreAPIntentConsumer()
  events = Events()
  events_sp = EventsSP()
  consumer.update(_consumer_view(first_seed), events, events_sp, apply_longitudinal=False)
  events.clear()
  consumer.update(_consumer_view(first_action), events, events_sp, apply_longitudinal=False)
  assert events.has(EventName.buttonCancel)
  events.clear()
  consumer.update(_consumer_view(restart_seed), events, events_sp, apply_longitudinal=False)
  assert not events.has(EventName.buttonCancel)
  events.clear()
  consumer.update(_consumer_view(restart_action), events, events_sp, apply_longitudinal=False)
  assert events.has(EventName.buttonCancel)



class _FakeSM:
  def __init__(self, *, panda_states=None, alive=True, valid=True, updated=False):
    self._panda_states = [] if panda_states is None else panda_states
    self.alive = {"pandaStates": alive}
    self.valid = {"pandaStates": valid}
    self.updated = {"pandaStates": updated}

  def update(self, _n):
    return None

  def all_alive(self, service_list=None):
    keys = service_list or list(self.alive)
    return all(self.alive.get(s, False) for s in keys)

  def all_valid(self, service_list=None):
    keys = service_list or list(self.valid)
    return all(self.valid.get(s, False) for s in keys)

  def __getitem__(self, key):
    if key == "pandaStates":
      return self._panda_states
    if key == "carControl":
      return SimpleNamespace(enabled=False)
    if key == "longitudinalPlanSP":
      return SimpleNamespace()
    raise KeyError(key)


def _live_cancel_stock_cc():
  stock_cc = StockCcTransaction(True)
  stock_cc.state = structs.CarStateSP.PreapStockCcTransactionState.cancelRequested
  stock_cc.bound_counter = 4
  return stock_cc


def _card_with_producers(epoch, stock_cc):
  card = Car.__new__(Car)
  card.preap_intent_epoch = epoch
  card.preap_intent_seed_pending = True
  card.CP = SimpleNamespace(carFingerprint="TESLA_MODEL_S_PREAP")
  card.can_sock = object()
  card.can_rcv_cum_timeout_counter = 0
  card.is_metric = False
  card.experimental_mode = False
  card.dynamic_experimental_control = False
  card.CS_prev = SimpleNamespace()
  card.CC_prev = SimpleNamespace(enabled=False)
  card.v_cruise_helper = cast(Any, SimpleNamespace(
    update_speed_limit_assist=lambda *a, **k: None,
    update_v_cruise=lambda *a, **k: None,
    v_cruise_kph=0.0,
    v_cruise_cluster_kph=0.0,
  ))
  card.sm = cast(Any, _FakeSM())
  card.RI = SimpleNamespace(update=lambda _can: None)

  def ci_update(_can):
    cs = SimpleNamespace()
    cs_sp = structs.CarStateSP()
    cs_sp.preapIntentSequence = 1
    stock_cc.publish(cs_sp)
    return cs, cs_sp

  card.CI = SimpleNamespace(
    update=ci_update,
    CS=SimpleNamespace(
      update_stock_cc_panda=lambda panda: None if panda is None else stock_cc.update_panda(
        counter=int(getattr(panda, "stockCcReengageCounter", 0) or 0),
        confirmed=bool(getattr(panda, "stockCcReengageConfirmed", False)),
        controls_allowed_longitudinal=bool(getattr(panda, "controlsAllowedLongitudinal", False)),
      ),
      stock_cc=stock_cc,
    ),
  )
  return card


def _run_state_update(card):
  with patch("openpilot.selfdrive.car.card.messaging.drain_sock_raw", return_value=[b"can"]):
    with patch("openpilot.selfdrive.car.card.can_capnp_to_list", return_value=[]):
      with patch("openpilot.selfdrive.car.card.convert_to_capnp", side_effect=lambda sp: sp):
        return card.state_update()


def test_state_update_startup_seed_is_neutral_then_exposes_cancel_edge():
  StockCc = structs.CarStateSP.PreapStockCcTransactionState
  stock_cc = _live_cancel_stock_cc()
  card = _card_with_producers(42, stock_cc)

  _cs, seed, _rd = _run_state_update(card)
  assert seed.preapIntentEpoch == 42
  assert seed.preapIntentSequence == 0
  assert seed.preapLateralIntent == structs.CarStateSP.PreapLateralIntent.none
  assert seed.preapLongitudinalIntent == structs.CarStateSP.PreapLongitudinalIntent.none
  assert seed.preapStockCcState == StockCc.idle
  assert seed.preapStockCcEnablePending is False
  assert seed.preapStockCcHostDiConfirmed is False
  assert seed.preapStockCcBoundCounter == 0
  assert stock_cc.state == StockCc.cancelRequested

  _cs, action, _rd = _run_state_update(card)
  assert action.preapIntentEpoch == 42
  assert action.preapIntentSequence == 1
  assert action.preapStockCcState == StockCc.cancelRequested

  consumer = PreAPIntentConsumer()
  events = Events()
  events_sp = EventsSP()
  consumer.update(_consumer_view(seed), events, events_sp, apply_longitudinal=False)
  assert not events.has(EventName.buttonCancel)
  events.clear()
  events_sp.clear()
  consumer.update(_consumer_view(action), events, events_sp, apply_longitudinal=False)
  assert events.has(EventName.buttonCancel)


def test_state_update_producer_restart_exposes_cancel_after_new_seed():
  StockCc = structs.CarStateSP.PreapStockCcTransactionState
  first_cc = _live_cancel_stock_cc()
  first = _card_with_producers(7, first_cc)
  _cs, first_seed, _rd = _run_state_update(first)
  _cs, first_action, _rd = _run_state_update(first)

  restart_cc = _live_cancel_stock_cc()
  restart = _card_with_producers(8, restart_cc)
  _cs, restart_seed, _rd = _run_state_update(restart)
  assert restart_seed.preapIntentEpoch == 8
  assert restart_seed.preapIntentSequence == 0
  assert restart_seed.preapStockCcState == StockCc.idle
  assert restart_seed.preapStockCcEnablePending is False
  assert restart_seed.preapStockCcHostDiConfirmed is False
  assert restart_cc.state == StockCc.cancelRequested

  _cs, restart_action, _rd = _run_state_update(restart)
  assert restart_action.preapIntentSequence == 1
  assert restart_action.preapStockCcState == StockCc.cancelRequested

  consumer = PreAPIntentConsumer()
  events = Events()
  events_sp = EventsSP()
  consumer.update(_consumer_view(first_seed), events, events_sp, apply_longitudinal=False)
  events.clear()
  consumer.update(_consumer_view(first_action), events, events_sp, apply_longitudinal=False)
  assert events.has(EventName.buttonCancel)
  events.clear()
  consumer.update(_consumer_view(restart_seed), events, events_sp, apply_longitudinal=False)
  assert not events.has(EventName.buttonCancel)
  events.clear()
  consumer.update(_consumer_view(restart_action), events, events_sp, apply_longitudinal=False)
  assert events.has(EventName.buttonCancel)


def _confirmed_stock_cc():
  stock_cc = StockCcTransaction(True)
  stock_cc.state = structs.CarStateSP.PreapStockCcTransactionState.confirmed
  stock_cc.enable_pending = True
  stock_cc.host_di_confirmed = True
  stock_cc._panda_matched = True
  stock_cc._panda_counter_at_bind = 0
  stock_cc.bound_counter = 1
  return stock_cc


def _confirmed_panda():
  return SimpleNamespace(
    stockCcReengageCounter=1,
    stockCcReengageConfirmed=True,
    controlsAllowedLongitudinal=True,
  )


def _card_with_panda_dispatch(stock_cc, sm):
  card = _card_with_producers(1, stock_cc)
  card.sm = sm
  card.preap_intent_seed_pending = False

  def update_stock_cc_panda(panda):
    if panda is None:
      stock_cc.update_panda(counter=None, confirmed=False, controls_allowed_longitudinal=False)
      return
    stock_cc.update_panda(
      counter=int(getattr(panda, "stockCcReengageCounter", 0) or 0),
      confirmed=bool(getattr(panda, "stockCcReengageConfirmed", False)),
      controls_allowed_longitudinal=bool(getattr(panda, "controlsAllowedLongitudinal", False)),
    )

  card.CI.CS.update_stock_cc_panda = update_stock_cc_panda
  return card


def test_healthy_cached_panda_survives_between_updates():
  stock_cc = _confirmed_stock_cc()
  sm = _FakeSM(panda_states=[_confirmed_panda()], alive=True, valid=True, updated=False)
  card = _card_with_panda_dispatch(stock_cc, sm)
  _run_state_update(card)
  assert stock_cc.state == structs.CarStateSP.PreapStockCcTransactionState.confirmed
  assert stock_cc.enable_pending is True
  sm.updated["pandaStates"] = False
  _cs, cs_sp, _rd = _run_state_update(card)
  assert stock_cc.state == structs.CarStateSP.PreapStockCcTransactionState.confirmed
  assert stock_cc.enable_pending is True
  assert cs_sp.preapStockCcEnablePending is True
  assert cs_sp.preapStockCcHostDiConfirmed is True


def test_dead_panda_after_confirmation_fails_closed():
  stock_cc = _confirmed_stock_cc()
  sm = _FakeSM(panda_states=[_confirmed_panda()], alive=True, valid=True)
  card = _card_with_panda_dispatch(stock_cc, sm)
  _run_state_update(card)
  assert stock_cc.state == structs.CarStateSP.PreapStockCcTransactionState.confirmed
  sm.alive["pandaStates"] = False
  _cs, cs_sp, _rd = _run_state_update(card)
  assert stock_cc.state == structs.CarStateSP.PreapStockCcTransactionState.cancelledOrFailed
  assert stock_cc.enable_pending is False
  assert stock_cc.host_di_confirmed is False
  assert cs_sp.preapStockCcEnablePending is False


def test_invalid_panda_after_confirmation_fails_closed():
  stock_cc = _confirmed_stock_cc()
  sm = _FakeSM(panda_states=[_confirmed_panda()], alive=True, valid=True)
  card = _card_with_panda_dispatch(stock_cc, sm)
  _run_state_update(card)
  assert stock_cc.state == structs.CarStateSP.PreapStockCcTransactionState.confirmed
  sm.valid["pandaStates"] = False
  _cs, cs_sp, _rd = _run_state_update(card)
  assert stock_cc.state == structs.CarStateSP.PreapStockCcTransactionState.cancelledOrFailed
  assert stock_cc.enable_pending is False
  assert stock_cc.host_di_confirmed is False
  assert cs_sp.preapStockCcEnablePending is False


def test_stale_pending_after_dead_panda_cannot_enable():
  stock_cc = _confirmed_stock_cc()
  sm = _FakeSM(panda_states=[_confirmed_panda()], alive=True, valid=True)
  card = _card_with_panda_dispatch(stock_cc, sm)
  _run_state_update(card)
  sm.alive["pandaStates"] = False
  _cs, dead, _rd = _run_state_update(card)
  assert dead.preapStockCcEnablePending is False
  consumer = PreAPIntentConsumer()
  events = Events()
  events_sp = EventsSP()
  view = _consumer_view(dead)
  consumer.update(view, events, events_sp, apply_longitudinal=True)
  events.clear()
  events_sp.clear()
  view.preapIntentSequence = int(view.preapIntentSequence) + 1
  consumer.update(view, events, events_sp, apply_longitudinal=True)
  assert not events.has(EventName.buttonEnable)


def _failed_stock_cc():
  stock_cc = StockCcTransaction(True)
  stock_cc.update_stalk(CruiseButtons.IDLE, 0, 0)
  stock_cc.update_stalk(CruiseButtons.MAIN, 1, 0)
  stock_cc.update_health(blocked=True)
  stock_cc.update_health(blocked=False)
  return stock_cc


def test_state_update_terminal_seed_survives_immediate_idle_then_recovers():
  StockCc = structs.CarStateSP.PreapStockCcTransactionState
  stock_cc = _failed_stock_cc()
  card = _card_with_producers(42, stock_cc)

  _cs, seed, _rd = _run_state_update(card)
  assert seed.preapIntentEpoch == 42
  assert seed.preapIntentSequence == 0
  assert seed.preapLateralIntent == structs.CarStateSP.PreapLateralIntent.none
  assert seed.preapLongitudinalIntent == structs.CarStateSP.PreapLongitudinalIntent.none
  assert seed.preapStockCcState == StockCc.idle
  assert seed.preapStockCcEnablePending is False
  assert seed.preapStockCcHostDiConfirmed is False
  assert stock_cc.state == StockCc.cancelledOrFailed

  stock_cc.update_stalk(CruiseButtons.IDLE, 2, 10)
  assert stock_cc.state == StockCc.cancelledOrFailed

  _cs, terminal, _rd = _run_state_update(card)
  assert terminal.preapIntentEpoch == 42
  assert terminal.preapIntentSequence == 1
  assert terminal.preapStockCcState == StockCc.cancelledOrFailed
  assert terminal.preapStockCcEnablePending is False
  assert terminal.preapStockCcHostDiConfirmed is False

  consumer = PreAPIntentConsumer()
  events = Events()
  events_sp = EventsSP()
  consumer.update(_consumer_view(seed), events, events_sp, apply_longitudinal=False)
  assert not events.has(EventName.buttonCancel)
  events.clear()
  events_sp.clear()
  consumer.update(_consumer_view(terminal), events, events_sp, apply_longitudinal=False)
  assert events.has(EventName.buttonCancel)

  stock_cc.update_stalk(CruiseButtons.IDLE, 3, 20)
  assert stock_cc.state == StockCc.idle
  _cs, recovered, _rd = _run_state_update(card)
  assert recovered.preapStockCcState == StockCc.idle
