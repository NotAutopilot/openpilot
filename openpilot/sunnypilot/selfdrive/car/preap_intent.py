from __future__ import annotations

from openpilot.cereal import custom, log


EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
LateralIntent = custom.CarStateSP.PreapLateralIntent
LongitudinalIntent = custom.CarStateSP.PreapLongitudinalIntent
StockCcState = custom.CarStateSP.PreapStockCcTransactionState

UINT32_MASK = 0xFFFFFFFF
UINT32_HALF = 0x80000000


def uint32_delta(candidate: int, current: int) -> int:
  return (int(candidate) - int(current)) & UINT32_MASK


def sequence_is_newer(candidate: int, current: int) -> bool | None:
  """Newer if and only if 0 < uint32(candidate - current) < 2^31.

  Equal is not newer. The 2^31 delta is undefined and fail-closed.
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
  """Apply each newer Pre-AP intent record to MADS / long event streams once.

  Sequence increments once per emission. The emitted pair is republished until
  the next edge so conflate cannot replace it with none. A later long-only
  edge is a new sequence with lateral none, so it cannot replay a stale
  mainCruiseRequest after host disable. Epoch changes re-seed without acting
  (producer restart). Duplicates and older sequences are ignored.
  """

  def __init__(self):
    self.sequence = 0
    self.epoch = 0
    self.seeded = False
    self.fail_closed = False
    self._stock_cc_pending_seen = False
    self._stock_cc_state = StockCcState.idle

  @staticmethod
  def _stock_cc_pending(record) -> bool:
    return bool(getattr(record, "preapStockCcEnablePending", False))

  @staticmethod
  def _stock_cc_state_of(record):
    return getattr(record, "preapStockCcState", StockCcState.idle)

  def _seed(self, sequence: int, pending: bool, state, epoch: int = 0) -> None:
    self.sequence = sequence
    self.epoch = int(epoch) if epoch else 0
    self.seeded = True
    self.fail_closed = False
    self._stock_cc_pending_seen = pending
    self._stock_cc_state = state

  def _apply_stock_cc_state(self, state, events) -> None:
    if state is None:
      return
    if state != self._stock_cc_state:
      if state == StockCcState.cancelRequested:
        events.add(EventName.buttonCancel)
      elif state == StockCcState.cancelledOrFailed:
        events.add(EventName.buttonCancel)
    self._stock_cc_state = state

  def _apply_pending_edge(self, pending: bool, events, apply_longitudinal: bool, *, rearm: bool) -> None:
    if apply_longitudinal:
      if rearm:
        self._stock_cc_pending_seen = pending
      return
    if pending and not self._stock_cc_pending_seen:
      events.add(EventName.buttonEnable)
      self._stock_cc_pending_seen = True
    elif rearm:
      self._stock_cc_pending_seen = pending

  def update(self, record, events, events_sp, apply_longitudinal: bool = True) -> None:
    sequence = int(record.preapIntentSequence) & UINT32_MASK
    epoch = int(getattr(record, "preapIntentEpoch", 0) or 0)
    pending = self._stock_cc_pending(record)
    state = self._stock_cc_state_of(record)

    if not self.seeded or epoch != self.epoch:
      self._seed(sequence, pending, state, epoch)
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

    same_sequence = uint32_delta(sequence, self.sequence) == 0
    force_disable = _is_intent(record.preapLateralIntent, LateralIntent.forceDisable)
    if newer:
      self.sequence = sequence
      if force_disable:
        events_sp.add(EventNameSP.lkasDisable)
      if apply_longitudinal and _is_intent(record.preapLongitudinalIntent, LongitudinalIntent.disable):
        events.add(EventName.buttonCancel)
      if _is_intent(record.preapLateralIntent, LateralIntent.mainCruiseRequest):
        events_sp.add(EventNameSP.lkasEnable)
      if apply_longitudinal and _is_intent(record.preapLongitudinalIntent, LongitudinalIntent.enable):
        events.add(EventName.buttonEnable)
      self._apply_pending_edge(pending, events, True if force_disable else apply_longitudinal, rearm=True)
      self._apply_stock_cc_state(state, events)
    elif same_sequence:
      self._apply_pending_edge(pending, events, True if force_disable else apply_longitudinal, rearm=False)
      self._apply_stock_cc_state(state, events)
