from pathlib import Path

import pytest

from openpilot.selfdrive.test.process_replay.preap_log_contracts import evaluate_contracts, synthetic_drive
from openpilot.selfdrive.test.process_replay.preap_offcar import main
from openpilot.selfdrive.test.process_replay.preap_route_index import (
  JACK_DONGLE,
  RouteIndexError,
  index_segments,
  parse_log_path,
  resolve_source,
)


def _check(report, name: str):
  for check in report.checks:
    if check.name == name:
      return check
  raise AssertionError(f"missing check {name}: {[c.name for c in report.checks]}")


def _signal(report, prefix: str):
  for signal in report.signals:
    if signal.name.startswith(prefix):
      return signal
  raise AssertionError(f"missing signal {prefix}: {[s.name for s in report.signals]}")


def test_synthetic_fixture_passes_chime_and_radar_contracts():
  report = evaluate_contracts(synthetic_drive(), source="fixture")
  assert not report.failed, report.format()
  assert _check(report, "gas_override_no_chime").status == "pass"
  assert _check(report, "enableLongControl").status == "pass"
  assert _check(report, "long_engage_chime").status == "pass"
  assert _check(report, "long_disengage_chime").status == "pass"
  assert _check(report, "lat_engage_chime").status == "pass"
  assert _check(report, "radar_tracks").status == "pass"
  assert _signal(report, "GAS_COMMAND").present
  assert _signal(report, "Bosch RadarPoint").present
  assert not _signal(report, "DAS_bodyControls").present


def test_gas_override_chime_fails_when_disengage_event_fires():
  report = evaluate_contracts(synthetic_drive(override_emits_disengage=True), source="buggy")
  assert report.failed
  assert _check(report, "gas_override_no_chime").status == "fail"


def test_gas_override_fails_when_enable_long_drops():
  report = evaluate_contracts(synthetic_drive(override_drops_long=True), source="drop")
  assert report.failed
  assert _check(report, "enableLongControl").status == "fail"
  assert _check(report, "gas_override_no_chime").status == "fail"


def test_missing_radar_is_skip_not_fail():
  report = evaluate_contracts(synthetic_drive(include_radar=False), source="no-radar")
  assert _check(report, "radar_tracks").status == "skip"
  assert _check(report, "radar_hw_fail").status == "skip"
  assert _check(report, "radar_donor").status == "skip"
  assert _check(report, "gas_override_no_chime").status == "pass"


def test_cli_fixture_exits_zero(capsys):
  assert main(["--fixture"]) == 0
  out = capsys.readouterr().out
  assert "gas_override_no_chime" in out
  assert "pass" in out
  assert "synthetic fixture" in out


def test_route_index_parses_explorer_name_and_resolves(tmp_path: Path):
  name = f"{JACK_DONGLE}_00000074--113899b226--28--rlog.zst"
  path = tmp_path / name
  path.write_bytes(b"not-a-real-log")
  located = parse_log_path(path)
  assert located is not None
  assert located.dongle == JACK_DONGLE
  assert located.log_id == "00000074--113899b226"
  assert located.segment == 28
  assert located.kind == "rlog"

  indexed = index_segments([tmp_path], dongle=JACK_DONGLE)
  assert [s.segment_name for s in indexed] == [f"{JACK_DONGLE}|00000074--113899b226--28"]

  resolved = resolve_source("00000074--113899b226", roots=[tmp_path], dongle=JACK_DONGLE)
  assert resolved[0].path == path
  resolved = resolve_source(str(path), roots=[tmp_path])
  assert resolved[0].path == path


def test_route_index_device_segment_dir(tmp_path: Path):
  seg_dir = tmp_path / f"{JACK_DONGLE}|00000005--fb95696ac5--0"
  seg_dir.mkdir()
  rlog = seg_dir / "rlog.zst"
  rlog.write_bytes(b"x")
  located = parse_log_path(rlog)
  assert located is not None
  assert located.segment == 0
  assert located.kind == "rlog"


def test_route_index_does_not_invent_missing_routes(tmp_path: Path):
  with pytest.raises(RouteIndexError, match="no rlog/qlog matched"):
    resolve_source("d0cdc986c5d023f5|does-not-exist--deadbeef00--0", roots=[tmp_path], dongle=JACK_DONGLE)


def test_cli_list_empty_dir(tmp_path: Path, capsys, monkeypatch):
  monkeypatch.setenv("PREAP_OFFCAR_DATA_DIR", str(tmp_path))
  # Still searches default roots; --data-dir prepends. Use a dongle that will
  # not match Jack's copies if those roots exist.
  assert main(["--list", "--data-dir", str(tmp_path), "--dongle", "ffffffffffffffff"]) == 0
  out = capsys.readouterr().out
  assert "ffffffffffffffff" in out
