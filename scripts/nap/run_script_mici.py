"""Mici (comma 4) NAP script runner.

Selected from `run_script.py:main()` when `gui_app.big_ui()` is False.

Matches the tici runner's lifecycle contract — `NAPScriptRunning`
param dance, SIGINT → SIGTERM → SIGKILL cancel with fail-closed on
stuck child, reboot on exit (PC=False). The settings-panel button is
offroad-gated, which forces the user to be parked at tap time; the
user is expected to wake the car (brake + power button) before
pressing Start so the targeted ECU is electrically active.

The UI primitive lives at `system/ui/mici/widgets/script_runner_app.py`
so that it can be reused for any future mici full-screen-script flow.
"""
import queue
import signal
import subprocess
import sys
import threading

from openpilot.common.params import Params
from openpilot.system.hardware import HARDWARE, PC
from openpilot.system.ui.mici.widgets.script_runner_app import MiciScriptRunnerApp, ScriptState


class _NAPMiciRunner:
  def __init__(self, title: str, module: str, instructions: str):
    self._module = module
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
    except Exception as e:
      self._app.append_output(f"[Error reading output: {e}]")
      self._app.set_state(ScriptState.ERROR)

  # ── exit ──────────────────────────────────────────

  def _on_exit(self) -> None:
    if self._process is not None and self._process.poll() is None:
      # Cooperative cancel: SIGINT first so the child's finally blocks
      # run (Panda safety reset, pedal disable). Escalate only if the
      # child doesn't cooperate. Note: by the time Exit is clickable
      # (state != RUNNING) the child is already dead, so this path is
      # mostly defense in depth.
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

    if not PC:
      HARDWARE.reboot()
    from openpilot.system.ui.lib.application import gui_app
    gui_app.request_close()


def main():
  if len(sys.argv) < 4:
    print("Usage: run_script_mici.py <title> <module> <instructions>")
    sys.exit(1)

  title = sys.argv[1]
  module = sys.argv[2]
  instructions = sys.argv[3]

  # Kill the comma tmux session so we own the screen (no-op on dev hosts).
  try:
    subprocess.run(["tmux", "kill-session", "-t", "comma"], capture_output=True)
  except FileNotFoundError:
    pass

  runner = _NAPMiciRunner(title, module, instructions)
  runner.run()


if __name__ == "__main__":
  main()
