#!/usr/bin/env python3
"""Comma web UI. Params and software only.

HTTP on port 80 when the process can bind it, otherwise 7070.
This process does not open cereal sockets and does not run a live
telemetry loop. It cannot engage openpilot.

A crash, bind failure, or preimport error must not block engage
(the manager process is optional).
"""
from __future__ import annotations

import json
import os
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

try:
  from openpilot.common.swaglog import cloudlog
except Exception:  # pragma: no cover - unit tests without zmq
  class _PrintLog:
    def info(self, msg):
      print(msg)

    def warning(self, msg):
      print(msg)

    def exception(self, msg):
      print(msg)
  cloudlog = _PrintLog()

from openpilot.selfdrive.nap_dash.force_offroad_api import AckError, handle_ack, read_ack
from openpilot.selfdrive.nap_dash.settings import SettingError
from openpilot.selfdrive.nap_dash.system_api import (
  SoftwareError,
  handle_software,
  read_software,
)
from openpilot.selfdrive.nap_dash.ui_api import (
  get_manifest,
  network_status,
  public_manifest,
  read_settings,
  write_setting,
)

HOST = os.environ.get("NAP_DASH_HOST", "0.0.0.0")
DASHBOARD_PATH = Path(__file__).resolve().parent / "dashboard.html"

PARAMS = None
PM_LOCK = threading.Lock()
STOP = threading.Event()


def listen_ports():
  """Port 80 is the MCU URL. 7070 is the fallback if 80 cannot be bound."""
  raw = os.environ.get("NAP_DASH_PORT", "").strip()
  if raw:
    return [int(raw)]
  return [80, 7070]


def _params():
  global PARAMS
  if PARAMS is None:
    from openpilot.common.params import Params
    PARAMS = Params()
  return PARAMS


def locked_settings():
  with PM_LOCK:
    return read_settings(_params())


def locked_write(name, value, confirm=False):
  with PM_LOCK:
    return write_setting(_params(), name, value, confirm=confirm)


def state_snapshot():
  """Minimal JSON so an old page does not 500. No live car data."""
  return {
    "lite": True,
    "ts": 0,
    "car": {},
    "drive": {},
    "plan": {},
    "lead1": {},
    "settings": {},
    "health": {},
    "engagement": {},
    "bms": {},
  }


def dashboard_html() -> bytes:
  try:
    return DASHBOARD_PATH.read_bytes()
  except OSError:
    return b"<h1>openpilot</h1><p>dashboard.html missing</p>"


class Handler(BaseHTTPRequestHandler):
  protocol_version = "HTTP/1.1"

  def log_message(self, fmt, *args):
    cloudlog.info("nap_dash " + (fmt % args))

  def send_json(self, obj, code=200):
    body = json.dumps(obj).encode()
    self.send_response(code)
    self.send_header("Content-Type", "application/json")
    self.send_header("Cache-Control", "no-cache, no-store")
    self.send_header("Content-Length", str(len(body)))
    self.end_headers()
    self.wfile.write(body)

  def _send_html(self):
    body = dashboard_html()
    self.send_response(200)
    self.send_header("Content-Type", "text/html; charset=utf-8")
    self.send_header("Cache-Control", "no-cache, no-store")
    self.send_header("Content-Length", str(len(body)))
    self.end_headers()
    self.wfile.write(body)

  def do_GET(self):
    path = urlparse(self.path).path
    if path in ("/", "/index.html", "/phone", "/phone/"):
      return self._send_html()
    if path == "/api/manifest":
      try:
        return self.send_json(public_manifest())
      except Exception as exc:
        return self.send_json({"error": str(exc)}, 500)
    if path == "/api/state":
      return self.send_json(state_snapshot())
    if path == "/api/settings":
      try:
        snap = locked_settings()
        snap["catalog"] = public_manifest()["controls"]
        return self.send_json(snap)
      except Exception as exc:
        return self.send_json({"error": str(exc)}, 500)
    if path == "/api/network":
      try:
        return self.send_json(network_status())
      except Exception as exc:
        return self.send_json({"error": str(exc)}, 500)
    if path == "/api/force_offroad":
      try:
        return self.send_json(read_ack(_params()))
      except Exception as exc:
        return self.send_json({"error": str(exc)}, 500)
    if path == "/api/software":
      try:
        return self.send_json(read_software(_params()))
      except Exception as exc:
        return self.send_json({"error": str(exc)}, 500)
    return self.send_json({"error": "not found"}, 404)

  def do_POST(self):
    path = urlparse(self.path).path
    try:
      n = int(self.headers.get("Content-Length", "0"))
      if n < 0 or n > 16 * 1024:
        return self.send_json({"error": "payload too large"}, 413)
      payload = json.loads(self.rfile.read(n).decode() or "{}")
      if path == "/api/software":
        return self.send_json(handle_software(payload, _params()))
      if path == "/api/force_offroad":
        return self.send_json(handle_ack(payload, _params()))
      if path != "/api/set":
        return self.send_json({"error": "not found"}, 404)
      action = str(payload.get("action") or "").strip().lower()
      if action in {"engage", "disengage"}:
        return self.send_json({"error": "engaging openpilot from the web is not allowed"}, 400)
      name = str(payload.get("param") or payload.get("name") or "")
      snap = locked_write(name, payload.get("value"), bool(payload.get("confirm")))
      # Lets the page raise the steering Yes/No right after the tap, not on the next poll.
      snap["force_offroad_ack"] = read_ack(_params())
      return self.send_json(snap)
    except AckError as exc:
      return self.send_json({"error": str(exc)}, 400)
    except SoftwareError as exc:
      return self.send_json({"error": str(exc)}, 400)
    except SettingError as exc:
      return self.send_json({"error": str(exc)}, 400)
    except Exception as exc:
      return self.send_json({"error": str(exc)}, 400)


class DashHTTPServer(ThreadingHTTPServer):
  allow_reuse_address = True
  daemon_threads = True


def bind_http_server(host=HOST, port=None):
  """Bind port 80, then 7070. Returns None if every port fails.

  An explicit port (tests, NAP_DASH_PORT) tries only that port.
  """
  ports = [int(port)] if port is not None else listen_ports()
  for one in ports:
    try:
      server = DashHTTPServer((host, one), Handler)
      cloudlog.info(f"nap_dash listening on {host}:{one} (Params + software, no cereal)")
      return server
    except OSError as exc:
      cloudlog.warning(f"nap_dash failed to bind {host}:{one}: {exc}")
  return None


def idle_until_stop(stop_event=None, sleeper=time.sleep):
  """Keep the managed process alive so optional Dash cannot block engage."""
  stop_event = STOP if stop_event is None else stop_event
  while not stop_event.is_set():
    sleeper(1.0)


def main():
  try:
    hot = get_manifest().get("hotspot") or {}
    cloudlog.info("nap_dash url " + str(hot.get("url") or "http://100.99.9.1/"))
  except Exception:
    cloudlog.warning("nap_dash manifest unavailable at start")
  server = bind_http_server(HOST)
  if server is None:
    idle_until_stop()
    return
  try:
    server.serve_forever()
  except KeyboardInterrupt:
    pass
  except Exception:
    traceback.print_exc()
    idle_until_stop()
  finally:
    STOP.set()
    try:
      server.server_close()
    except Exception:
      pass


if __name__ == "__main__":
  main()
