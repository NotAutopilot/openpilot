"""Approved Pre-AP diagnostic transport. Never grants ALLOUTPUT or driving authority."""
from __future__ import annotations

import time
from opendbc.car import structs
from opendbc.car.tesla.preap.constants import (
  GAS_COMMAND_ID,
  PREAP_FLAG_ENABLE_PEDAL,
  PREAP_FLAG_PEDAL_BUS_ZERO,
  PREAP_FLAG_PEDAL_CALIBRATION,
  PREAP_MODE_INVALID,
)

SAFETY_SILENT = int(structs.CarParams.SafetyModel.silent)
SAFETY_ELM327 = int(structs.CarParams.SafetyModel.elm327)
SAFETY_ALLOUTPUT = int(structs.CarParams.SafetyModel.allOutput)
SAFETY_TESLA_PREAP = int(structs.CarParams.SafetyModel.teslaPreap)

ALLOWED_DIAGNOSTIC_MODES = frozenset({SAFETY_SILENT, SAFETY_ELM327})
ALLOWED_SAFETY_MODES = ALLOWED_DIAGNOSTIC_MODES | frozenset({SAFETY_TESLA_PREAP})
PANDA_CONNECT_RETRIES = 5
PANDA_CONNECT_DELAY = 2.0


class TransportError(Exception):
  """Fail-closed transport or negative-response failure."""


def uds_fail_closed(exc: BaseException) -> TransportError:
  """Wrap UDS negative responses so callers fail closed."""
  name = type(exc).__name__
  if name == "NegativeResponseError" or "negative response" in str(exc).lower():
    return TransportError(f"UDS negative response (fail closed): {exc}")
  return TransportError(str(exc))


def fail_closed_negative_response(exc: BaseException) -> None:
  """Convert a UDS negative response into TransportError. Never continue the tool."""
  name = type(exc).__name__
  message = str(exc)
  if name == "NegativeResponseError" or "negative response" in message.lower() or getattr(exc, "error_code", None) is not None:
    raise TransportError(f"UDS negative response (fail closed): {exc}") from exc
  raise TransportError(f"UDS request failed closed: {exc}") from exc


class DiagnosticTransport:
  """Panda access that cannot bypass TX restrictions."""

  def __init__(self, panda=None):
    self.panda = panda
    self._mode = SAFETY_SILENT
    self._mode_set = False

  def connect(self, panda_factory=None):
    if self.panda is not None:
      return self.panda
    factory = panda_factory
    if factory is None:
      from panda import Panda
      factory = Panda
    last_exc: Exception | None = None
    for attempt in range(PANDA_CONNECT_RETRIES):
      try:
        self.panda = factory()
        return self.panda
      except Exception as exc:
        last_exc = exc
        if attempt < PANDA_CONNECT_RETRIES - 1:
          time.sleep(PANDA_CONNECT_DELAY)
    raise TransportError(f"panda connect failed: {last_exc}") from last_exc

  def set_diagnostic_session(self) -> None:
    self._set_safety_mode(SAFETY_ELM327)

  def set_silent(self) -> None:
    self._set_safety_mode(SAFETY_SILENT)

  def set_pedal_calibration_session(self, bus: int = 2) -> None:
    """Tesla Pre-AP safety with the calibration-only flag. Never ELM327/allOutput.

    Firmware 0xdf is ignored in car safety modes, so the order is: silent/non-car,
    safetyParamSP INVALID via 0xdf, then teslaPreap calibration. Calibration uses
    INVALID plus an isolated 0x551 allowlist and must not depend on mode validity.
    """
    if bus not in (0, 2):
      raise TransportError("invalid pedal bus")
    param = PREAP_FLAG_PEDAL_CALIBRATION
    if bus == 0:
      param |= PREAP_FLAG_PEDAL_BUS_ZERO
    self.set_silent()
    setter = getattr(self.panda, "set_alternative_experience", None)
    if not callable(setter):
      raise TransportError("panda cannot set safetyParamSP")
    setter(0, PREAP_MODE_INVALID)
    self._set_safety_mode(SAFETY_TESLA_PREAP, param=param)

  def _set_safety_mode(self, mode: int, param: int = 0) -> None:
    if mode == SAFETY_ALLOUTPUT or mode not in ALLOWED_SAFETY_MODES:
      raise TransportError("panda TX bypass is not permitted")
    if mode == SAFETY_TESLA_PREAP:
      if (int(param) & PREAP_FLAG_PEDAL_CALIBRATION) == 0:
        raise TransportError("teslaPreap transport requires calibration flag")
      if int(param) & PREAP_FLAG_ENABLE_PEDAL:
        raise TransportError("calibration must not grant longitudinal")
    elif int(param) != 0:
      raise TransportError("diagnostic modes cannot carry safetyParam bits")
    if self.panda is None:
      raise TransportError("panda not connected")
    self.panda.set_safety_mode(mode, param)
    self._mode = mode
    self._mode_set = True

  def can_recv(self):
    if self.panda is None:
      raise TransportError("panda not connected")
    try:
      return self.panda.can_recv()
    except Exception as exc:
      raise TransportError(f"can_recv failed: {exc}") from exc

  def can_send(self, addr: int, dat: bytes, bus: int) -> None:
    if self.panda is None:
      raise TransportError("panda not connected")
    if self._mode == SAFETY_ALLOUTPUT:
      raise TransportError("panda TX bypass is not permitted")
    if int(addr) == GAS_COMMAND_ID:
      if self._mode in (SAFETY_ELM327, SAFETY_ALLOUTPUT) or self._mode != SAFETY_TESLA_PREAP:
        raise TransportError("pedal frames require teslaPreap calibration safety")
    try:
      self.panda.can_send(addr, dat, bus)
    except Exception as exc:
      raise TransportError(f"can_send failed: {exc}") from exc

  def close(self) -> None:
    if self.panda is None:
      return
    if self._mode_set:
      try:
        self.set_silent()
      except Exception:
        pass
    try:
      self.panda.close()
    except Exception:
      pass
    self.panda = None
    self._mode_set = False
