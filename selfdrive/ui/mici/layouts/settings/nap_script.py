"""Launch helper for NAP scripts (pedal calibration, EPAS flash, etc.).

Spawns scripts/nap/run_script.py as a detached process. The runner kills
the comma tmux session, takes over the screen with its own pyray window,
and uses the NAPScriptRunning param to gate manager.py into stopping
pandad/card/controlsd/etc. for the duration.

safety_class:
    "offroad_only" — refuse to open the runner and refuse Start if
        IsOnroad is true. Use for EPAS firmware operations.
    "stationary" — no IsOnroad check. Use for pedal/radar calibration
        and tests, which require ignition on so the ECU is active.
"""
import os
import subprocess

from openpilot.common.basedir import BASEDIR

SAFETY_OFFROAD_ONLY = "offroad_only"
SAFETY_STATIONARY = "stationary"


def launch_script(title: str, instructions: str, script_module: str,
                  safety_class: str = SAFETY_STATIONARY) -> None:
  """Spawn run_script.py for the given module. Detached; the runner
  takes over the display until the user exits."""
  script_path = os.path.join(BASEDIR, "scripts", "nap", "run_script.py")
  log_path = "/tmp/nap_script_runner.log"
  with open(log_path, "w") as log_file:
    subprocess.Popen(
      ["python", script_path, title, script_module, instructions, safety_class],
      cwd=BASEDIR,
      start_new_session=True,
      stdout=log_file,
      stderr=log_file,
    )
