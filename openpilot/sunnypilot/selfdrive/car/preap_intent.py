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
  """Newer within an epoch iff 0 < uint32(candidate - current) < 2^31.

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
  """Apply each newer atomic Pre-AP intent record to existing event streams once."""

  def __init__(self):
    self.epoch = 0
    self.sequence = 0
    self.seeded = False
    self.fail_closed = False
    self._stock_cc_pending_seen = False
    self._stock_cc_state = StockCcState.idle

  @staticmethod
  def _neutral(record) -> bool:
    return _is_intent(record.preapLateralIntent, LateralIntent.none) and \
           _is_intent(record.preapLongitudinalIntent, LongitudinalIntent.none)

  @staticmethod
  def _stock_cc_pending(record) -> bool:
    return bool(getattr(record, "preapStockCcEnablePending", False))

  @staticmethod
  def _stock_cc_state_of(record):
    return getattr(record, "preapStockCcState", StockCcState.idle)

  def _seed(self, epoch: int, sequence: int, pending: bool, state) -> None:
    self.epoch = epoch
    self.sequence = sequence
    self.seeded = True
    self.fail_closed = False
    # Restart/new-epoch: an already-true pending level is consumed, not an enable edge.
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
    epoch = int(record.preapIntentEpoch)
    sequence = int(record.preapIntentSequence) & UINT32_MASK
    pending = self._stock_cc_pending(record)
    state = self._stock_cc_state_of(record)
    if epoch == 0:
      return

    # Consumer restart: seed the current record without acting, and treat
    # an already-true pending level as consumed so it cannot replay as buttonEnable.
    if not self.seeded:
      self._seed(epoch, sequence, pending, state)
      return

    # A new producer epoch must begin with a neutral seed. A non-neutral
    # first record is an invalid restart and cannot grant authority.
    if epoch != self.epoch:
      self._seed(epoch, sequence, pending, state)
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

    same_sequence = uint32_delta(sequence, self.sequence) == 0
    force_disable = _is_intent(record.preapLateralIntent, LateralIntent.forceDisable)
    if newer:
      self.sequence = sequence
      # Disable is processed before enable when one action has multiple consequences.
      if force_disable:
        events_sp.add(EventNameSP.lkasDisable)
      if apply_longitudinal and _is_intent(record.preapLongitudinalIntent, LongitudinalIntent.disable):
        events.add(EventName.buttonCancel)
      if _is_intent(record.preapLateralIntent, LateralIntent.mainCruiseRequest):
        events_sp.add(EventNameSP.lkasEnable)
      if apply_longitudinal and _is_intent(record.preapLongitudinalIntent, LongitudinalIntent.enable):
        events.add(EventName.buttonEnable)
      # forceDisable never yields buttonEnable from a pending level, even if the
      # intermediate confirmation record was missed. Rearm/consume without an edge.
      self._apply_pending_edge(pending, events, True if force_disable else apply_longitudinal, rearm=True)
      self._apply_stock_cc_state(state, events)
    elif same_sequence:
      # Same-sequence false-to-true confirmation may fire; rollback cannot rearm.
      # A same-sequence forceDisable still cannot emit pending buttonEnable.
      self._apply_pending_edge(pending, events, True if force_disable else apply_longitudinal, rearm=False)
      self._apply_stock_cc_state(state, events)
    # Older/equal-non-same records cannot mutate a consumed latch.
