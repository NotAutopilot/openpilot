"""Launch helper for NAP scripts (pedal calibration, EPAS flash, etc.).

Spawns scripts/nap/run_script.py as a detached process. The runner kills
the comma tmux session, takes over the screen with its own pyray window,
and uses the NAPScriptRunning param to gate manager.py into stopping
pandad/card/controlsd/etc. for the duration.

The settings-panel button is offroad-gated, which forces the user to be
parked when they tap. After the runner opens, the user wakes the car
(brake + power button) and then taps Start — same flow as the tici
panel that's been shipping on nap-release.
"""
import os
import subprocess

from openpilot.common.basedir import BASEDIR


def launch_script(title: str, instructions: str, script_module: str) -> None:
  """Spawn run_script.py for the given module. Detached; the runner
  takes over the display until the user exits."""
  script_path = os.path.join(BASEDIR, "scripts", "nap", "run_script.py")
  log_path = "/tmp/nap_script_runner.log"
  with open(log_path, "w") as log_file:
    subprocess.Popen(
      ["python", script_path, title, script_module, instructions],
      cwd=BASEDIR,
      start_new_session=True,
      stdout=log_file,
      stderr=log_file,
    )
