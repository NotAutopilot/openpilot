"""Mici (comma 4) NAP script runner.

Selected from `run_script.py:main()` when `gui_app.big_ui()` is False.

The lifecycle is identical to the tici runner — same IsOnroad gates, same
module-bound safety_class enforcement, same NAPScriptRunning param dance,
same SIGINT-then-escalate cancel with fail-closed on stuck child. Only the
UI differs: the mici display is 536×240 logical, so we use a NavScroller of
GreyBigButton cards plus a single BigPillButton action instead of the
custom-laid-out title/instructions/output/buttons of the tici runner.

The script-runner UI primitive lives at
`system/ui/mici/widgets/script_runner_app.py` so that it's reusable for any
future mici "run this with full-screen output" flow.
"""
import queue
import signal
import subprocess
import sys
import threading

from openpilot.common.params import Params
from openpilot.system.hardware import HARDWARE, PC
from openpilot.system.ui.mici.widgets.script_runner_app import MiciScriptRunnerApp, ScriptState
from scripts.nap.run_script import (
  SAFETY_OFFROAD_ONLY,
  SAFETY_STATIONARY,
  VALID_SAFETY_CLASSES,
  resolve_safety_class,
)


class _NAPMiciRunner:
  def __init__(self, title: str, module: str, instructions: str, safety_class: str):
    self._module = module
    self._safety_class = safety_class
    self._params = Params()
    self._process: subprocess.Popen | None = None
    self._reader_thread: threading.Thread | None = None
    self._output_queue: queue.Queue[str] = queue.Queue()

    self._app = MiciScriptRunnerApp(
      title=title,
      instructions=instructions,
      on_start=self._on_start,
      on_exit=self._on_exit,
    )

  def run(self) -> None:
    self._app.run("NAP Script Runner")

  # ── start ─────────────────────────────────────────

  def _on_start(self) -> None:
    if self._safety_class == SAFETY_OFFROAD_ONLY and self._params.get_bool("IsOnroad"):
      self._app.append_output("Cannot run while car is on.")
      self._app.append_output("Put the vehicle in park and turn it off,")
      self._app.append_output("then try again.")
      self._app.set_state(ScriptState.ERROR)
      return

    self._params.put_bool("NAPScriptRunning", True)
    try:
      self._process = subprocess.Popen(
        ["python", "-m", self._module],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        cwd="/data/openpilot",
        text=True,
        bufsize=1,
      )
    except Exception as e:
      self._params.put_bool("NAPScriptRunning", False)
      self._app.append_output(f"Error starting script: {e}")
      self._app.set_state(ScriptState.ERROR)
      return

    self._app.append_output("Starting script...")
    self._app.set_state(ScriptState.RUNNING)
    # EPAS scripts must not be cancelled mid-write — disable the action
    # button visibly. Calibration/test scripts stay cancellable.
    if self._safety_class == SAFETY_OFFROAD_ONLY:
      self._app.set_action_enabled(False)
    self._reader_thread = threading.Thread(target=self._read_output, daemon=True)
    self._reader_thread.start()

  def _read_output(self) -> None:
    try:
      assert self._process is not None
      if self._process.stdout is not None:
        for line in iter(self._process.stdout.readline, ''):
          if line:
            self._app.append_output(line.rstrip())
          if self._process.poll() is not None:
            break

      return_code = self._process.wait()
      if return_code == 0:
        self._app.append_output("")
        self._app.append_output("[Script completed successfully]")
        self._app.set_state(ScriptState.COMPLETED)
      else:
        self._app.append_output("")
        self._app.append_output(f"[Script exited with code {return_code}]")
        self._app.set_state(ScriptState.ERROR)
      # Re-enable the action button after the script finishes — exit
      # is allowed in both COMPLETED and ERROR states regardless of
      # safety_class.
      self._app.set_action_enabled(True)
    except Exception as e:
      self._app.append_output(f"[Error reading output: {e}]")
      self._app.set_state(ScriptState.ERROR)
      self._app.set_action_enabled(True)

  # ── exit ──────────────────────────────────────────

  def _on_exit(self) -> None:
    # Match the tici runner's hardware-safety contract.
    # offroad_only scripts (EPAS flash/extract/restore) must not be
    # interrupted while RUNNING — terminating mid-UDS-write can brick
    # the module. The MiciScriptRunnerApp action button still fires
    # this callback if the user taps Exit; we defend here.
    if self._safety_class == SAFETY_OFFROAD_ONLY and self._app.state == ScriptState.RUNNING:
      self._app.append_output("Exit blocked: EPAS scripts cannot be cancelled mid-write.")
      return

    if self._process is not None and self._process.poll() is None:
      self._process.send_signal(signal.SIGINT)
      try:
        self._process.wait(timeout=5)
      except subprocess.TimeoutExpired:
        self._process.terminate()
        try:
          self._process.wait(timeout=2)
        except subprocess.TimeoutExpired:
          self._process.kill()
          try:
            self._process.wait(timeout=2)
          except subprocess.TimeoutExpired:
            self._app.append_output("[ERROR] script did not exit; manager will stay paused")
            self._app.append_output("Reboot the device to recover.")
            self._app.set_state(ScriptState.ERROR)
            return

    self._params.put_bool("NAPScriptRunning", False)

    # request_close lives on gui_app; we touch it through the app indirectly
    # by relying on render-loop teardown after on_exit returns and reboot below.
    # On dev hosts (PC=True) we skip the reboot so the user can keep iterating.
    if not PC:
      HARDWARE.reboot()
    # Closing the runner window:
    from openpilot.system.ui.lib.application import gui_app
    gui_app.request_close()


def main():
  if len(sys.argv) < 4:
    print("Usage: run_script_mici.py <title> <module> <instructions> [safety_class]")
    sys.exit(1)

  title = sys.argv[1]
  module = sys.argv[2]
  instructions = sys.argv[3]
  requested = sys.argv[4] if len(sys.argv) > 4 else SAFETY_STATIONARY
  if requested not in VALID_SAFETY_CLASSES:
    print(f"unknown safety_class {requested!r}; expected one of {VALID_SAFETY_CLASSES}")
    sys.exit(1)

  # Module-bound resolution: unknown modules and weaker-than-required
  # requests are upgraded to the strictest class. This file is a
  # standalone entry point, so the same fail-closed policy must apply
  # here as in run_script.py — a direct invocation with a stale or
  # missing safety_class would otherwise bypass the offroad gates.
  safety_class = resolve_safety_class(module, requested)

  # Pre-window onroad gate for offroad-only scripts. Same as tici runner —
  # refuse before killing tmux so the user keeps their main UI.
  if safety_class == SAFETY_OFFROAD_ONLY and Params().get_bool("IsOnroad"):
    print("NAP runner: cannot run while car is on. Park it and try again.")
    sys.exit(0)

  # Kill the comma tmux session so we own the screen (no-op on dev hosts).
  try:
    subprocess.run(["tmux", "kill-session", "-t", "comma"], capture_output=True)
  except FileNotFoundError:
    pass

  runner = _NAPMiciRunner(title, module, instructions, safety_class)
  runner.run()


if __name__ == "__main__":
  main()
