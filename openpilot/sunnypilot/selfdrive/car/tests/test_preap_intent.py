from openpilot.cereal import custom, log
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.car.preap_intent import PreAPIntentConsumer
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP


EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
Lat = custom.CarStateSP.PreapLateralIntent
Long = custom.CarStateSP.PreapLongitudinalIntent


def record(epoch, sequence, lateral=Lat.none, longitudinal=Long.none):
  msg = custom.CarStateSP.new_message()
  msg.preapIntentEpoch = epoch
  msg.preapIntentSequence = sequence
  msg.preapLateralIntent = lateral
  msg.preapLongitudinalIntent = longitudinal
  return msg


def apply(consumer, msg):
  events, events_sp = Events(), EventsSP()
  consumer.update(msg, events, events_sp)
  return events, events_sp


def test_restart_seeds_without_acting_then_accepts_newer():
  consumer = PreAPIntentConsumer()
  events, events_sp = apply(consumer, record(1, 4, Lat.mainCruiseRequest, Long.enable))
  assert not events.has(EventName.buttonEnable)
  assert not events_sp.has(EventNameSP.lkasEnable)

  events, events_sp = apply(consumer, record(1, 5, Lat.mainCruiseRequest, Long.enable))
  assert events.has(EventName.buttonEnable)
  assert events_sp.has(EventNameSP.lkasEnable)


def test_duplicate_older_and_reordered_records_are_ignored():
  consumer = PreAPIntentConsumer()
  apply(consumer, record(1, 10))
  apply(consumer, record(1, 11))
  for sequence in (11, 10, 9):
    events, events_sp = apply(consumer, record(1, sequence, Lat.forceDisable, Long.disable))
    assert not events.has(EventName.buttonCancel)
    assert not events_sp.has(EventNameSP.lkasDisable)


def test_rollover_progression_is_newer():
  consumer = PreAPIntentConsumer()
  apply(consumer, record(1, 0xFFFFFFFF))
  events, events_sp = apply(consumer, record(1, 0, Lat.mainCruiseRequest, Long.enable))
  assert events.has(EventName.buttonEnable)
  assert events_sp.has(EventNameSP.lkasEnable)


def test_half_range_and_non_neutral_new_epoch_fail_closed():
  consumer = PreAPIntentConsumer()
  apply(consumer, record(1, 0))
  events, events_sp = apply(consumer, record(1, 0x80000000, Lat.mainCruiseRequest, Long.enable))
  assert events.has(EventName.buttonCancel)
  assert events_sp.has(EventNameSP.lkasDisable)

  events, events_sp = apply(consumer, record(2, 1, Lat.mainCruiseRequest, Long.enable))
  assert events.has(EventName.buttonCancel)
  assert events_sp.has(EventNameSP.lkasDisable)


def test_new_epoch_neutral_seed_requires_fresh_record():
  consumer = PreAPIntentConsumer()
  apply(consumer, record(1, 5))
  events, events_sp = apply(consumer, record(2, 0))
  assert not events.has(EventName.buttonCancel)
  assert not events_sp.has(EventNameSP.lkasDisable)
  events, events_sp = apply(consumer, record(2, 1, Lat.mainCruiseRequest))
  assert events_sp.has(EventNameSP.lkasEnable)
