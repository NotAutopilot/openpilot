"""Off-car Pre-AP chime replay. Product test, not the TESLA process_replay whitelist."""
from __future__ import annotations

import pytest

from openpilot.selfdrive.selfdrived.tests.preap_routes import (
  CUSTOM_PARAMS,
  FINGERPRINT,
  PINNED_SEGMENTS,
  REPLAY_PROCS,
  assert_replay_contract,
  load_rlog,
  resolve_pinned_rlog,
)
from openpilot.selfdrive.test.process_replay.preap_log_contracts import evaluate_contracts, synthetic_drive
from openpilot.selfdrive.test.process_replay.process_replay import replay_process_with_name


def _check(report, name: str):
  for check in report.checks:
    if check.name == name:
      return check
  raise AssertionError(f"missing check {name}: {[c.name for c in report.checks]}")


def test_replay_harness_is_preap_not_model_y():
  assert_replay_contract()
  assert FINGERPRINT == "TESLA_MODEL_S_PREAP"
  assert FINGERPRINT != "TESLA_MODEL_Y"
  assert CUSTOM_PARAMS["DisengageOnAccelerator"] is False
  assert REPLAY_PROCS == ("selfdrived", "radard")
  assert "card" not in REPLAY_PROCS
  assert "controlsd" not in REPLAY_PROCS


def test_synthetic_gas_override_does_not_chime():
  report = evaluate_contracts(synthetic_drive(), source="synthetic")
  assert not report.failed, report.format()
  assert _check(report, "gas_override_no_chime").status == "pass"
  assert _check(report, "long_engage_chime").status == "pass"
  assert _check(report, "long_disengage_chime").status == "pass"
  assert _check(report, "lat_engage_chime").status == "pass"
  assert _check(report, "enableLongControl").status == "pass"


def test_synthetic_gas_override_chime_is_a_failure():
  report = evaluate_contracts(synthetic_drive(override_emits_disengage=True), source="buggy")
  assert _check(report, "gas_override_no_chime").status == "fail"


@pytest.mark.parametrize(
  "pinned",
  PINNED_SEGMENTS,
  ids=[p.segment_name for p in PINNED_SEGMENTS],
)
def test_replay_pinned_rlog_chimes(pinned):
  assert_replay_contract()
  path = resolve_pinned_rlog(pinned)
  if path is None:
    pytest.skip(f"rlog not on disk: {pinned.segment_name} ({pinned.note})")

  msgs = load_rlog(path)
  outputs = replay_process_with_name(
    list(REPLAY_PROCS),
    msgs,
    fingerprint=FINGERPRINT,
    custom_params=CUSTOM_PARAMS,
    return_all_logs=True,
    disable_progress=True,
  )
  report = evaluate_contracts(outputs, source=pinned.segment_name)
  assert report.fingerprint in ("", FINGERPRINT), report.format()
  assert not report.failed, report.format()
