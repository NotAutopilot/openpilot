"""Web Force Offroad Yes/No mirrors selfdrive/ui/onroad/force_offroad_confirm.py."""
from __future__ import annotations

import json
from http.client import HTTPConnection
from pathlib import Path
from threading import Thread

import pytest

from openpilot.selfdrive.nap_dash.force_offroad_api import AckError, handle_ack, read_ack
from openpilot.system.hardware.nap_force_offroad import CONFIRM_PROMPT


class FakeParams:
  def __init__(self, **bools):
    self.bools = dict(bools)
    self.writes: list[tuple[str, bool]] = []

  def get_bool(self, key):
    return bool(self.bools.get(key, False))

  def put_bool(self, key, value):
    self.writes.append((key, bool(value)))
    self.bools[key] = bool(value)


def waiting():
  return FakeParams(NAPForceOffroad=True, NAPForceOffroadConfirmed=False, IsOnroad=True)


def test_prompt_shows_on_native_condition():
  assert read_ack(waiting()) == {"pending": True, "prompt": CONFIRM_PROMPT, "yes": "Yes", "no": "No"}
  assert read_ack(FakeParams(NAPForceOffroad=True, IsOnroad=False))["pending"] is False   # parked
  assert read_ack(FakeParams(NAPForceOffroad=True, NAPForceOffroadConfirmed=True, IsOnroad=True))["pending"] is False
  assert read_ack(FakeParams(IsOnroad=True))["pending"] is False


def test_yes_writes_confirmed_only_and_needs_confirm():
  p = waiting()
  with pytest.raises(AckError, match="confirmation"):
    handle_ack({"answer": "yes"}, p)
  assert p.writes == []
  snap = handle_ack({"answer": "yes", "confirm": True}, p)
  assert p.writes == [("NAPForceOffroadConfirmed", True)]
  assert snap["pending"] is False


def test_no_writes_native_resets():
  p = waiting()
  handle_ack({"answer": "no"}, p)
  assert p.writes == [("NAPForceOffroad", False), ("NAPForceOffroadConfirmed", False), ("NAPForceOffroadHandoffReady", False)]
  assert read_ack(p)["pending"] is False


def test_refuses_when_not_pending_or_bad_answer():
  for p in (FakeParams(), FakeParams(NAPForceOffroad=True, NAPForceOffroadConfirmed=True, IsOnroad=True)):
    for payload in ({"answer": "yes", "confirm": True}, {"answer": "no"}):
      with pytest.raises(AckError, match="Nothing to confirm"):
        handle_ack(payload, p)
    assert p.writes == []
  for payload in ({"answer": "engage"}, {}):
    with pytest.raises(AckError):
      handle_ack(payload, waiting())


def test_page_has_prompt_and_polls():
  html = (Path(__file__).resolve().parents[1] / "dashboard.html").read_text()
  assert 'id="foack"' in html and "setInterval(foPoll, 1500)" in html
  assert '{answer: "yes", confirm: true}' in html


def test_http_round_trip_and_engage_still_blocked(monkeypatch):
  from openpilot.selfdrive.nap_dash import server as srv
  p = waiting()
  monkeypatch.setattr(srv, "_params", lambda: p)
  httpd = srv.ThreadingHTTPServer(("127.0.0.1", 0), srv.Handler)
  Thread(target=httpd.serve_forever, daemon=True).start()
  host, port = httpd.server_address

  def call(method, path, body=None):
    conn = HTTPConnection(host, port, timeout=3)
    conn.request(method, path, body=json.dumps(body).encode() if body is not None else None,
                 headers={"Content-Type": "application/json"})
    r = conn.getresponse()
    out = (r.status, json.loads(r.read().decode()))
    conn.close()
    return out
  try:
    assert call("GET", "/api/force_offroad")[1]["pending"] is True
    assert call("POST", "/api/force_offroad", {"answer": "yes"})[0] == 400
    status, data = call("POST", "/api/force_offroad", {"answer": "yes", "confirm": True})
    assert status == 200 and data["pending"] is False
    assert call("POST", "/api/set", {"action": "engage"})[0] == 400
  finally:
    httpd.shutdown()
    httpd.server_close()


def test_force_offroad_tap_has_no_generic_confirm_and_returns_prompt():
  """Tap Force Offroad: parked applies at once; car on -> only the steering Yes/No, in the same response."""
  from openpilot.selfdrive.nap_dash import ui_api
  from openpilot.selfdrive.nap_dash.ui_api import _needs_confirm
  assert _needs_confirm(None, {"writer": "force_offroad", "confirm": True}, True, False) is False
  html = (Path(__file__).resolve().parents[1] / "dashboard.html").read_text()
  assert 'ctrl.writer !== "force_offroad"' in html and "foRender(data.force_offroad_ack)" in html

  class Full(FakeParams):
    def get(self, key, *a, **kw):
      return None

    def put(self, key, value, *a, **kw):
      self.writes.append((key, value))

  for onroad, pending in ((False, False), (True, True)):
    p = Full(IsOnroad=onroad)
    out = ui_api.write_setting(p, "NAPForceOffroad", True, confirm=False)
    assert p.bools["NAPForceOffroad"] is True
    assert p.bools["NAPForceOffroadConfirmed"] is (not onroad)
    assert read_ack(p)["pending"] is pending
    assert out is not None
