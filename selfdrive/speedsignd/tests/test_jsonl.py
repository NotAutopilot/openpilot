"""JSONL log path, record shape, and enable gate (no sqlite, no cruise)."""
from __future__ import annotations

import ast
import json
import os
from pathlib import Path

from openpilot.selfdrive.speedsignd.detect import SpeedSignDetector, paint_mutcd_r2_1
from openpilot.selfdrive.speedsignd.jsonl import RECORD_KEYS, JsonlLogger, append_jsonl, make_record
from openpilot.selfdrive.speedsignd.paths import PARAM_KEY, default_log_path, default_onnx_path, nap_data_dir
from openpilot.selfdrive.speedsignd.speedsignd import process_observations, should_run_speed_sign_log
from openpilot.selfdrive.speedsignd.tests.test_detect import _scene


class _FakeParams:
  def __init__(self, enabled=False):
    self.enabled = enabled

  def get_bool(self, key):
    assert key == PARAM_KEY
    return self.enabled


def test_default_off_even_when_onroad():
  assert not should_run_speed_sign_log(True, _FakeParams(False))
  assert not should_run_speed_sign_log(False, _FakeParams(True))
  assert not should_run_speed_sign_log(False, _FakeParams(False))


def test_enable_onroad_starts_process():
  assert should_run_speed_sign_log(True, _FakeParams(True))


def test_jsonl_record_keys_and_append(tmp_path):
  path = str(tmp_path / "speed_signs.jsonl")
  rec = make_record(1.5, 45.31, -95.60, 87.0, 45, 0.91)
  assert tuple(rec.keys()) == RECORD_KEYS
  append_jsonl(path, rec)
  lines = Path(path).read_text(encoding="utf-8").strip().splitlines()
  assert len(lines) == 1
  parsed = json.loads(lines[0])
  assert parsed == {"t": 1.5, "lat": 45.31, "lon": -95.60, "bearing": 87.0, "mph": 45, "conf": 0.91}


def test_logger_dedup_same_sign(tmp_path):
  path = str(tmp_path / "speed_signs.jsonl")
  log = JsonlLogger(path, cooldown_s=8.0, radius_m=40.0)
  a = make_record(10.0, 45.0, -95.0, 0.0, 30, 0.8)
  b = make_record(11.0, 45.0001, -95.0001, 0.0, 30, 0.8)
  c = make_record(11.0, 45.0, -95.0, 0.0, 55, 0.8)
  assert log.write(a)
  assert not log.write(b)
  assert log.write(c)
  lines = Path(path).read_text(encoding="utf-8").strip().splitlines()
  assert len(lines) == 2


def test_process_observations_writes_gps_row(tmp_path):
  path = str(tmp_path / "out.jsonl")
  y = _scene()
  paint_mutcd_r2_1(y, 55, x=200, y=30, w=90, h=112)
  written = process_observations(
    y, 45.315, -95.601, 12.0, True,
    SpeedSignDetector(onnx_path=str(tmp_path / "missing.onnx")),
    JsonlLogger(path),
    now=123.0,
  )
  assert written
  assert written[0]["mph"] == 55
  assert written[0]["lat"] == 45.315
  row = json.loads(Path(path).read_text(encoding="utf-8").splitlines()[0])
  assert row["lon"] == -95.601
  assert row["bearing"] == 12.0
  assert row["t"] == 123.0


def test_no_gps_does_not_log(tmp_path):
  path = str(tmp_path / "out.jsonl")
  y = _scene()
  paint_mutcd_r2_1(y, 25, x=200, y=30, w=90, h=112)
  written = process_observations(
    y, 0.0, 0.0, None, False,
    SpeedSignDetector(onnx_path=str(tmp_path / "missing.onnx")),
    JsonlLogger(path),
    now=1.0,
  )
  assert written == []
  assert not os.path.exists(path)


def test_log_and_weight_paths_live_under_data(monkeypatch):
  monkeypatch.delenv("NAP_SPEED_SIGN_LOG", raising=False)
  monkeypatch.delenv("NAP_SPEED_SIGN_ONNX", raising=False)
  monkeypatch.setattr("openpilot.selfdrive.speedsignd.paths.os.path.isdir", lambda p: p == "/data/media/0")
  assert nap_data_dir() == "/data/media/0/nap"
  assert default_log_path() == "/data/media/0/nap/speed_signs.jsonl"
  assert default_onnx_path() == "/data/media/0/nap/speed_sign.onnx"


def test_env_overrides_log_path(monkeypatch, tmp_path):
  monkeypatch.setenv("NAP_SPEED_SIGN_LOG", str(tmp_path / "x.jsonl"))
  monkeypatch.setenv("NAP_SPEED_SIGN_ONNX", str(tmp_path / "x.onnx"))
  assert default_log_path().endswith("x.jsonl")
  assert default_onnx_path().endswith("x.onnx")


def test_module_does_not_touch_sqlite_cruise_or_osm():
  src = Path(__file__).resolve().parents[1]
  lowered_code = ""
  for p in src.glob("*.py"):
    tree = ast.parse(p.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
      if isinstance(node, ast.Import):
        lowered_code += " ".join(n.name.lower() for n in node.names) + "\n"
      elif isinstance(node, ast.ImportFrom) and node.module:
        lowered_code += node.module.lower() + "\n"
  assert "sqlite3" not in lowered_code
  assert "vcruise" not in lowered_code
  assert "osm.org" not in lowered_code
  assert "overpass" not in lowered_code
  assert "map_speed_policy" not in lowered_code
  tree = ast.parse((src / "speedsignd.py").read_text(encoding="utf-8"))
  imports = []
  for node in ast.walk(tree):
    if isinstance(node, ast.Import):
      imports.extend(n.name for n in node.names)
    elif isinstance(node, ast.ImportFrom) and node.module:
      imports.append(node.module)
  assert not any("mapd.osm" in i or "mapd.overpass" in i or "map_speed_policy" in i for i in imports)


def test_process_config_wires_default_off_gate():
  cfg = Path(__file__).resolve().parents[3] / "system" / "manager" / "process_config.py"
  text = cfg.read_text(encoding="utf-8")
  assert "speedsignd" in text
  assert "NAPSpeedSignLog" in text
  assert "selfdrive.speedsignd.speedsignd" in text


def test_settings_and_docs_cover_enable_and_log_path():
  root = Path(__file__).resolve().parents[3]
  nap = (root / "selfdrive" / "ui" / "layouts" / "settings" / "nap.py").read_text(encoding="utf-8")
  docs = (root / "docs-nap" / "speed-sign-log.md").read_text(encoding="utf-8")
  assert "Speed Sign Logger" in nap
  assert "NAPSpeedSignLog" in nap or "NAP_SPEED_SIGN_LOG" in nap
  # Logger sits at the top of NAP (before Force Offroad / Longitudinal).
  assert nap.find("Speed Sign Logger") < nap.find("Force Offroad")
  assert nap.find("Speed Sign Logger") < nap.find("Longitudinal Control")
  mici = (root / "selfdrive" / "ui" / "mici" / "layouts" / "settings" / "nap.py").read_text(encoding="utf-8")
  marker = "self._scroller.add_widgets([\n      speed_sign_log,"
  assert marker in mici
  widgets = mici[mici.find("self._scroller.add_widgets(["):]
  assert widgets.find("speed_sign_log,") < widgets.find("pedal_enabled,")
  content = (root / "selfdrive" / "ui" / "layouts" / "settings" / "nap_content.py").read_text(encoding="utf-8")
  assert "1 Hz" in content and "nice 19" in content
  assert "engaged" in content.lower()
  assert "manual" in content.lower()
  assert "WAIT" in content or "disengaged" in content.lower()
  assert "Force Offroad" in content  # weights install, not the detect gate
  assert "/data/media/0/nap/speed_signs.jsonl" in docs
  assert "NAPSpeedSignLog" in docs
  assert "speed_sign.onnx" in docs
  assert "install_speed_sign_weights" in docs
  assert "export_speed_sign_onnx" in docs
  assert "liveSpeedSignNAP" in docs
  assert "3.0" in docs
  assert "1 Hz" in docs
  assert "WAIT" in docs
  assert "engaged" in docs.lower()
  assert "controlling" in docs.lower()
  assert "swaglog" in docs.lower()
  assert "20 Hz" in docs
  assert "manual" in docs.lower()
  assert "moving" in docs.lower()
  assert "TAKE CONTROL" in docs
  assert "parked, disengaged, or Force Offroad" not in docs
  assert "modeld" in docs.lower() or "driving model" in docs.lower()
  assert "night" in docs.lower()
  assert "SHA-256" in docs or "sha256" in docs.lower()
  assert "NO WT" in docs
  assert "Yes" in docs and "No" in docs
  assert "on_confirm_accuracy" in docs
  assert "left" in docs.lower()
  assert "OSM" in docs
  assert "SIGN never lights" in docs
  assert "Install weights" in nap


def test_params_key_default_is_off():
  keys = Path(__file__).resolve().parents[3] / "common" / "params_keys.h"
  text = keys.read_text(encoding="utf-8")
  assert "NAPSpeedSignLog" in text
  # Explicit 0, or BOOL with no default (get_bool → false). Either is default-off.
  assert '{"NAPSpeedSignLog", {PERSISTENT, BOOL, "0"}}' in text or \
         '{"NAPSpeedSignLog", {PERSISTENT, BOOL}}' in text
