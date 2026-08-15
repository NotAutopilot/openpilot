"""Pre-AP production tool runner seam.

Launches an approved tool module as a separate process after the user
confirms on the instruction screen. Sets NAPScriptRunning so manager
releases pandad. Development-only utilities are not registered here.
"""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import threading

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.safety import (
  DESTRUCTIVE_TOOLS,
  ToolSafetyError,
  require_preap_tool_start,
  require_runtime_path,
)

RUN_SCRIPT_MODULE = "openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.run_script"

APPROVED_TOOLS = {
  "calibrate_pedal": "openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.calibrate_pedal",
  "calibrate_radar": "openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.calibrate_radar",
  "diagnose_radar": "openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.diagnose_radar",
  "test_radar": "openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.test_radar",
  "extract_epas": "openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.extract_epas",
  "flash_epas": "openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.flash_epas",
  "restore_epas": "openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.restore_epas",
}


def approved_module(tool: str) -> str:
  if tool not in APPROVED_TOOLS:
    raise ValueError(f"unapproved tool: {tool}")
  return APPROVED_TOOLS[tool]


def _clear_tool_flags(params: Params) -> None:
  params.put_bool("NAPScriptRunning", False, block=True)
  params.put_bool("NAPEpasRiskAccepted", False, block=True)


def _reap_tool(process: subprocess.Popen, params: Params) -> None:
  try:
    process.wait()
  finally:
    _clear_tool_flags(params)


def start_tool(tool: str, *, confirmed: bool, params: Params | None = None) -> subprocess.Popen:
  require_runtime_path()
  require_preap_tool_start(params, tool=tool, confirmed=confirmed)
  module = approved_module(tool)
  params = params or Params()
  if params.get_bool("NAPScriptRunning"):
    raise ToolSafetyError("another Pre-AP tool is already running")
  params.put_bool("NAPScriptRunning", True, block=True)
  if tool in ("flash_epas", "restore_epas") and confirmed:
    params.put_bool("NAPEpasRiskAccepted", True, block=True)
  env = os.environ.copy()
  env["PYTHONPATH"] = env.get("PYTHONPATH", BASEDIR)
  cmd = [sys.executable, "-m", module]
  if confirmed and tool in DESTRUCTIVE_TOOLS:
    cmd.append("--confirm")
  try:
    process = subprocess.Popen(
      cmd,
      cwd=BASEDIR,
      start_new_session=True,
      env=env,
    )
  except Exception:
    _clear_tool_flags(params)
    raise
  threading.Thread(target=_reap_tool, args=(process, params), daemon=True).start()
  return process


def stop_tool(process: subprocess.Popen, params: Params | None = None) -> None:
  params = params or Params()
  if process.poll() is None:
    process.send_signal(signal.SIGINT)
    try:
      process.wait(timeout=5)
    except subprocess.TimeoutExpired:
      process.terminate()
      try:
        process.wait(timeout=2)
      except subprocess.TimeoutExpired:
        process.kill()
  if process.poll() is not None:
    _clear_tool_flags(params)


def is_destructive(tool: str) -> bool:
  return tool in DESTRUCTIVE_TOOLS
