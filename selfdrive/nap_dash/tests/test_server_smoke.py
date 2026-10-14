"""Process wiring + HTTP smoke for the in-tree Dash. No live cereal required."""
from __future__ import annotations

import json
from http.client import HTTPConnection
from pathlib import Path
from threading import Thread

import pytest

from openpilot.selfdrive.nap_dash.server import Handler, ThreadingHTTPServer
from openpilot.selfdrive.nap_dash.tests.test_settings import FakeParams

ROOT = Path(__file__).resolve().parents[3]


def test_process_config_registers_optional_python_process():
  cfg = (ROOT / "system" / "manager" / "process_config.py").read_text(encoding="utf-8")
  assert 'PythonProcess("nap_dash"' in cfg
  assert "selfdrive.nap_dash.server" in cfg
  snippet = cfg.split('PythonProcess("nap_dash"', 1)[1][:400]
  assert "nap_dash_enabled" in snippet
  assert "restart_if_crash=True" in snippet
  assert "optional=True" in snippet
  optional_src = (ROOT / "system" / "manager" / "optional_procs.py").read_text(encoding="utf-8")
  assert "NAPDashEnabled" in optional_src
  assert "NAPDashEnabled" in (ROOT / "common" / "params_keys.h").read_text(encoding="utf-8")


def test_docs_cover_hotspot_and_mannerisms():
  docs = (ROOT / "docs-nap" / "nap-dash.md").read_text(encoding="utf-8")
  readme = (ROOT / "docs-nap" / "README.md").read_text(encoding="utf-8")
  assert "http://100.99.9.1/" in docs
  assert "100.99.9.0/24" in docs
  assert "hotspot" in docs.lower()
  assert "Driving Mannerisms" in docs
  assert "do not merge" in docs.lower()
  assert "nap-dash.md" in readme
  assert "panda" in docs.lower()
  assert "optional" in docs.lower()
  assert "processNotRunning" in docs
  assert "NAPDashEnabled" in docs
  assert "https://installer.comma.ai/jmbrunick/openpilot/cursor/mcu-web-ui-c588" in docs
  assert "/api/software" in docs
  assert "no high-rate sockets" in docs.lower()
  assert "modelV2" in docs
  assert "`can`" in docs
  assert "LTE" in docs


def test_onroad_cpu_budget_lists_nap_dash():
  onroad = (ROOT / "selfdrive" / "test" / "test_onroad.py").read_text(encoding="utf-8")
  assert '"selfdrive.nap_dash.server"' in onroad


def test_http_set_round_trip(monkeypatch):
  from openpilot.selfdrive.nap_dash import server as srv

  params = FakeParams()
  monkeypatch.setattr(srv, "PARAMS", params)
  monkeypatch.setattr(srv, "_params", lambda: params)

  httpd = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
  thread = Thread(target=httpd.serve_forever, daemon=True)
  thread.start()
  try:
    host, port = httpd.server_address
    conn = HTTPConnection(host, port, timeout=3)
    conn.request("GET", "/")
    home = conn.getresponse()
    body = home.read().decode("utf-8")
    assert home.status == 200
    assert "openpilot" in body
    assert "XMLHttpRequest" in body
    assert "fetch(" not in body
    assert "speed_trim" not in body
    conn.close()

    conn = HTTPConnection(host, port, timeout=3)
    conn.request("GET", "/api/state")
    state_resp = conn.getresponse()
    state = json.loads(state_resp.read().decode("utf-8"))
    assert state_resp.status == 200
    assert state["lite"] is True
    assert state["car"] == {}
    assert state["bms"] == {}
    assert state["lead1"] == {}
    conn.close()

    conn = HTTPConnection(host, port, timeout=5)
    conn.request("GET", "/api/manifest")
    manifest_resp = conn.getresponse()
    manifest = json.loads(manifest_resp.read().decode("utf-8"))
    assert manifest_resp.status == 200
    assert manifest["hotspot"]["url"] == "http://100.99.9.1/"
    titles = [item["title"] for item in manifest["controls"]]
    assert "Driving Mannerisms" in titles or "Soft Lateral Handoff" in titles
    assert any(item["param"] == "IsMetric" for item in manifest["controls"])
    conn.close()

    conn = HTTPConnection(host, port, timeout=5)
    conn.request("GET", "/api/settings")
    settings_resp = conn.getresponse()
    settings_body = json.loads(settings_resp.read().decode("utf-8"))
    assert settings_resp.status == 200
    assert "IsMetric" in settings_body["values"]
    assert any(item.get("param") == "IsMetric" for item in settings_body["catalog"])
    conn.close()

    conn = HTTPConnection(host, port, timeout=3)
    conn.request("GET", "/api/routes")
    routes = conn.getresponse()
    assert routes.status == 404
    routes.read()
    conn.close()

    conn = HTTPConnection(host, port, timeout=5)
    payload = json.dumps({"param": "IsMetric", "value": True}).encode()
    conn.request("POST", "/api/set", body=payload, headers={"Content-Type": "application/json"})
    resp = conn.getresponse()
    data = json.loads(resp.read().decode("utf-8"))
    assert resp.status == 200
    assert data["values"]["IsMetric"] is True
    assert params.bools["IsMetric"] is True
    conn.close()

    conn = HTTPConnection(host, port, timeout=3)
    conn.request("POST", "/api/set", body=json.dumps({"name": "speed_trim", "value": 5}).encode(),
                 headers={"Content-Type": "application/json"})
    bad = conn.getresponse()
    err = json.loads(bad.read().decode("utf-8"))
    assert bad.status == 400
    assert "rejected" in err["error"] or "unknown" in err["error"] or "not allowed" in err["error"]
    conn.close()

    conn = HTTPConnection(host, port, timeout=3)
    conn.request("POST", "/api/set", body=json.dumps({"action": "engage"}).encode(),
                 headers={"Content-Type": "application/json"})
    engage = conn.getresponse()
    engage_err = json.loads(engage.read().decode("utf-8"))
    assert engage.status == 400
    assert "not allowed" in engage_err["error"]
    conn.close()

    conn = HTTPConnection(host, port, timeout=3)
    conn.request("POST", "/api/nav/update", body=json.dumps({"route_state": "active"}).encode(),
                 headers={"Content-Type": "application/json"})
    nav = conn.getresponse()
    assert nav.status == 404
    conn.close()
  finally:
    httpd.shutdown()
    httpd.server_close()


def test_server_source_has_no_nav_write_or_settings_file():
  src = (ROOT / "selfdrive" / "nap_dash" / "server.py").read_text(encoding="utf-8")
  assert "NAP_SETTINGS_FILE" not in src
  assert "update_nap_settings_file" not in src
  assert "/api/nav" not in src
  assert "PHONE_HTML" not in src
  assert "def update_navigation" not in src
  assert "def bind_http_server" in src
  assert "idle_until_stop" in src
  assert "/api/software" in src
  assert "handle_software" in src


def test_server_has_no_high_rate_cereal():
  """Phone Hub settings/software are Params + HTTP. No cereal drain."""
  src = (ROOT / "selfdrive" / "nap_dash" / "server.py").read_text(encoding="utf-8")
  assert "SubMaster" not in src
  assert "messaging" not in src
  assert "modelV2" not in src
  assert "TELEMETRY_SVCS" not in src
  assert "def telemetry" not in src
  assert "_decode_bms" not in src
  assert "ffmpeg" not in src
  assert "/api/routes" not in src
  assert "/stream/" not in src
  assert "/export/" not in src
  assert '"can"' not in src
  assert "'can'" not in src
  for name in ("settings.py", "system_api.py", "ui_api.py", "discover.py"):
    companion = (ROOT / "selfdrive" / "nap_dash" / name).read_text(encoding="utf-8")
    assert "SubMaster" not in companion
    assert "modelV2" not in companion
    assert "import cereal" not in companion
    assert "from cereal" not in companion
  html = (ROOT / "selfdrive" / "nap_dash" / "dashboard.html").read_text(encoding="utf-8")
  assert "=>" not in html
  assert "let " not in html
  assert "const " not in html
  assert "fetch(" not in html
  assert "Phone guidance" not in html
  assert "Cruise speed trim" not in html
  assert "#2563eb" in html
  assert "overflow:hidden" in html.replace(" ", "")
  assert "float:left" not in html.replace(" ", "")


def test_nap_dash_is_optional_and_ignored_by_process_not_running():
  from types import SimpleNamespace
  from openpilot.system.manager.optional_procs import (
    OPTIONAL_PROCESS_NAMES, missing_required_processes, nap_dash_enabled,
  )

  assert "nap_dash" in OPTIONAL_PROCESS_NAMES
  assert "card" not in OPTIONAL_PROCESS_NAMES
  assert "selfdrived" not in OPTIONAL_PROCESS_NAMES
  assert "controlsd" not in OPTIONAL_PROCESS_NAMES

  dash = SimpleNamespace(name="nap_dash", running=False, shouldBeRunning=True)
  card = SimpleNamespace(name="card", running=False, shouldBeRunning=True)
  assert missing_required_processes([dash]) == []
  assert missing_required_processes([dash, card]) == ["card"]

  class _P:
    def get(self, key):
      return None
    def get_bool(self, key):
      raise AssertionError("missing NAPDashEnabled must fail open")
  assert nap_dash_enabled(True, _P(), None) is True

  class _Off:
    def get(self, key):
      return "0"
    def get_bool(self, key):
      return False
  assert nap_dash_enabled(True, _Off(), None) is False


def test_bind_failure_returns_none_and_idle_stops(monkeypatch):
  from openpilot.selfdrive.nap_dash import server as srv

  def _raise(*_args, **_kwargs):
    raise OSError("Address already in use")

  monkeypatch.setattr(srv, "DashHTTPServer", _raise)
  assert srv.bind_http_server("127.0.0.1", 1) is None

  calls = []
  stop = type("E", (), {"is_set": lambda self: len(calls) >= 2})()
  def sleeper(_dt):
    calls.append(1)
  srv.idle_until_stop(stop_event=stop, sleeper=sleeper)
  assert len(calls) == 2


def test_optional_prepare_and_dead_process_are_non_critical():
  pytest.importorskip("capnp")
  from openpilot.system.manager.process import PythonProcess

  p = PythonProcess("nap_dash", "openpilot.does.not.exist.nap_dash", lambda *_a: True, optional=True)
  p.prepare()  # must not raise / must not take down manager

  class Dead:
    pid = 1
    exitcode = 1
    def is_alive(self):
      return False

  p.proc = Dead()
  state = p.get_process_state_msg()
  assert state.running is False
  assert state.shouldBeRunning is False


def test_process_not_running_alert_omits_optional_dash():
  pytest.importorskip("capnp")
  from cereal import car, log
  from openpilot.selfdrive.selfdrived.events import process_not_running_alert

  cs = car.CarState.new_message()
  cp = car.CarParams.new_message()
  dash = log.ManagerState.ProcessState.new_message()
  dash.name = "nap_dash"
  dash.running = False
  dash.shouldBeRunning = True
  ms = log.ManagerState.new_message()
  ms.processes = [dash]
  alert = process_not_running_alert(cp, cs, {"managerState": ms}, False, 100, log.LongitudinalPersonality.standard)
  assert "nap_dash" not in (alert.alert_text_1 + " " + alert.alert_text_2)


def test_bookmark_endpoints_offroad_and_onroad():
  from openpilot.selfdrive.nap_dash.server import bookmark_status, bookmark_tap
  p = FakeParams()
  assert bookmark_status(p) == {"recording": False, "left": 0}
  out = bookmark_tap(p)
  assert out["ok"] is False and out["reason"] == "offroad"
  assert p.get("NAPBookmarkRequest") is None
  p.bools["IsOnroad"] = True
  assert bookmark_tap(p)["ok"] is True
  assert p.get("NAPBookmarkRequest") is not None
  p.put("NAPBookmarkRecording", 23)
  assert bookmark_status(p) == {"recording": True, "left": 23}


def test_dashboard_has_bookmark_button():
  html = (ROOT / "selfdrive" / "nap_dash" / "dashboard.html").read_text(encoding="utf-8")
  assert "/api/bookmark" in html and "REC " in html and '"Bookmark"' in html
