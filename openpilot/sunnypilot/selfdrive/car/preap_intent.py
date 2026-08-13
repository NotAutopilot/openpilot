from __future__ import annotations

from openpilot.cereal import custom, log


EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
LateralIntent = custom.CarStateSP.PreapLateralIntent
LongitudinalIntent = custom.CarStateSP.PreapLongitudinalIntent

UINT32_MASK = 0xFFFFFFFF
UINT32_HALF = 0x80000000


def uint32_delta(candidate: int, current: int) -> int:
  return (int(candidate) - int(current)) & UINT32_MASK


def sequence_is_newer(candidate: int, current: int) -> bool | None:
  """Newer within an epoch iff 0 < uint32(candidate - current) < 2^31.

  Returns None for the exact half-range ambiguity (fail-closed).
  """
  delta = uint32_delta(candidate, current)
  if delta == 0:
    return False
  if delta == UINT32_HALF:
    return None
  return delta < UINT32_HALF


def _is_intent(value, enum_member) -> bool:
  return value == enum_member


class PreAPIntentConsumer:
  """Apply each newer atomic Pre-AP intent record to existing event streams once."""

  def __init__(self):
    self.epoch = 0
    self.sequence = 0
    self.seeded = False
    self.fail_closed = False

  @staticmethod
  def _neutral(record) -> bool:
    return _is_intent(record.preapLateralIntent, LateralIntent.none) and \
           _is_intent(record.preapLongitudinalIntent, LongitudinalIntent.none)

  def _seed(self, epoch: int, sequence: int) -> None:
    self.epoch = epoch
    self.sequence = sequence
    self.seeded = True
    self.fail_closed = False

  def update(self, record, events, events_sp, apply_longitudinal: bool = True) -> None:
    epoch = int(record.preapIntentEpoch)
    sequence = int(record.preapIntentSequence) & UINT32_MASK
    if epoch == 0:
      return

    # Consumer restart: seed the current record without acting.
    if not self.seeded:
      self._seed(epoch, sequence)
      return

    # A new producer epoch must begin with a neutral seed. A non-neutral
    # first record is an invalid restart and cannot grant authority.
    if epoch != self.epoch:
      self._seed(epoch, sequence)
      if not self._neutral(record):
        self.fail_closed = True
        events_sp.add(EventNameSP.lkasDisable)
        if apply_longitudinal:
          events.add(EventName.buttonCancel)
      return
    if self.fail_closed:
      events_sp.add(EventNameSP.lkasDisable)
      if apply_longitudinal:
        events.add(EventName.buttonCancel)
      return

    newer = sequence_is_newer(sequence, self.sequence)
    if newer is None:
      self.fail_closed = True
      events_sp.add(EventNameSP.lkasDisable)
      if apply_longitudinal:
        events.add(EventName.buttonCancel)
      return
    if not newer:
      return
    self.sequence = sequence

    # Disable is processed before enable when one action has multiple consequences.
    if _is_intent(record.preapLateralIntent, LateralIntent.forceDisable):
      events_sp.add(EventNameSP.lkasDisable)
    if apply_longitudinal and _is_intent(record.preapLongitudinalIntent, LongitudinalIntent.disable):
      events.add(EventName.buttonCancel)
    if _is_intent(record.preapLateralIntent, LateralIntent.mainCruiseRequest):
      events_sp.add(EventNameSP.lkasEnable)
    if apply_longitudinal and _is_intent(record.preapLongitudinalIntent, LongitudinalIntent.enable):
      events.add(EventName.buttonEnable)
