"""Launch helper for NAP scripts via the on-device run_script UI."""
from openpilot.selfdrive.ui.mici.widgets.dialog import BigDialog
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.runner import launch_on_device_runner
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.safety import ToolSafetyError
from openpilot.system.ui.lib.application import gui_app

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
  try:
    launch_on_device_runner(title, TOOL_BY_MODULE[script_module], instructions)
  except (ToolSafetyError, ValueError, KeyError) as exc:
    gui_app.push_widget(BigDialog("can't start tool", str(exc)))
