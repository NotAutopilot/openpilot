from opendbc.car import structs
from openpilot.selfdrive.car.card import Car


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
