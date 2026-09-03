"""Pre-AP production tool runner seam.

UI launch opens run_script on the device display. Offroad is required.
There is no exclusive NAPScriptRunning lock.
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


def launch_on_device_runner(title: str, tool: str, instructions: str,
                            params: Params | None = None) -> subprocess.Popen:
  """Take over the Comma screen with run_script. Do not spawn the tool on a pts."""
  require_runtime_path()
  params = params or Params()
  require_preap_tool_start(params, tool=tool, confirmed=True)
  module = approved_module(tool)
  env = os.environ.copy()
  env["PYTHONPATH"] = env.get("PYTHONPATH", BASEDIR)
  log_path = "/tmp/nap_script_runner.log"
  with open(log_path, "w") as log_file:
    return subprocess.Popen(
      [sys.executable, "-m", RUN_SCRIPT_MODULE, title, module, instructions],
      cwd=BASEDIR,
      start_new_session=True,
      stdout=log_file,
      stderr=log_file,
      env=env,
    )


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
