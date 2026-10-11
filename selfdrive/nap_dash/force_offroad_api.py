"""Web mirror of the on-road Force Offroad Yes/No (selfdrive/ui/onroad/force_offroad_confirm.py).

Params only, through the same system/hardware/nap_force_offroad helpers the
native dialog uses. Yes writes NAPForceOffroadConfirmed=True (card then runs
the stock-CC handoff); No drops all Force Offroad intent. No cereal: "started"
is the IsOnroad param hardwared writes from deviceState.started.
"""
from __future__ import annotations

from openpilot.system.hardware.nap_force_offroad import (
  CONFIRM_NO,
  CONFIRM_PROMPT,
  CONFIRM_YES,
  CONFIRMED_PARAM,
  PARAM,
  cancel_force_offroad,
  confirm_force_offroad,
  needs_driver_confirm,
)


class AckError(ValueError):
  pass


def _flag(params, key: str) -> bool:
  try:
    return bool(params.get_bool(key))
  except Exception:
    return False


def read_ack(params) -> dict:
  """Same condition as maybe_show_force_offroad_confirm()."""
  pending = needs_driver_confirm(_flag(params, PARAM), _flag(params, CONFIRMED_PARAM), _flag(params, "IsOnroad"))
  return {"pending": bool(pending), "prompt": CONFIRM_PROMPT, "yes": CONFIRM_YES, "no": CONFIRM_NO}


def handle_ack(payload: dict, params) -> dict:
  answer = str(payload.get("answer") or "").strip().lower()
  if answer not in ("yes", "no"):
    raise AckError("answer must be yes or no")
  if not read_ack(params)["pending"]:
    raise AckError("Nothing to confirm: Force Offroad is not waiting for an answer.")
  if answer == "yes":
    if payload.get("confirm") is not True:
      raise AckError("confirmation required")
    confirm_force_offroad(params)
  else:
    cancel_force_offroad(params)
  return read_ack(params)
