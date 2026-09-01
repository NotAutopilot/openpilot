"""Launch helper for NAP scripts via the naponsp tool runner."""
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.runner import start_tool

TOOL_BY_MODULE = {
  "scripts.nap.calibrate_pedal": "calibrate_pedal",
  "scripts.nap.extract_epas": "extract_epas",
  "scripts.nap.flash_epas": "flash_epas",
  "scripts.nap.restore_epas": "restore_epas",
  "calibrate_pedal": "calibrate_pedal",
  "extract_epas": "extract_epas",
  "flash_epas": "flash_epas",
  "restore_epas": "restore_epas",
  "calibrate_radar": "calibrate_radar",
  "diagnose_radar": "diagnose_radar",
  "test_radar": "test_radar",
}


def launch_script(title: str, instructions: str, script_module: str) -> None:
  del title, instructions
  start_tool(TOOL_BY_MODULE[script_module], confirmed=True)
