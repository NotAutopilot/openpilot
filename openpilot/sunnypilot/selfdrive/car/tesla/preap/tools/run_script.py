#!/usr/bin/env python3
"""Production Pre-AP script runner.

Launches an approved tool module after offroad + runtime-path checks.
Sets NAPScriptRunning so manager releases pandad. Development-only
utilities are rejected. Native Tesla settings use ConfirmDialog/start_tool;
this runner remains the CLI/display process for the same approved tools.

Usage:
    python -m openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.run_script \\
      "Title" "openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.calibrate_pedal" "Instructions"
"""
from __future__ import annotations

import os
import subprocess
import sys
import threading
import queue
from pathlib import Path

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.runner import APPROVED_TOOLS
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.safety import (
  ToolSafetyError,
  require_offroad,
  require_runtime_path,
)

APPROVED_MODULES = frozenset(APPROVED_TOOLS.values())


class ScriptState:
  READY = 0
  RUNNING = 1
  COMPLETED = 2
  ERROR = 3


def prepare_run(module: str, params: Params | None = None) -> str:
  """Validate module, runtime path, and offroad before spawn."""
  require_runtime_path()
  params = params or Params()
  require_offroad(params)
  if module not in APPROVED_MODULES:
    raise ValueError(f"unapproved tool: {module}")
  return module


def spawn_approved_module(module: str, params: Params | None = None) -> subprocess.Popen:
  module = prepare_run(module, params)
  params = params or Params()
  params.put_bool("NAPScriptRunning", True, block=True)
  env = os.environ.copy()
  env["PYTHONPATH"] = env.get("PYTHONPATH", BASEDIR)
  try:
    return subprocess.Popen(
      [sys.executable, "-m", module],
      stdout=subprocess.PIPE,
      stderr=subprocess.STDOUT,
      cwd=str(Path(BASEDIR)),
      start_new_session=True,
      env=env,
      text=True,
      bufsize=1,
    )
  except Exception:
    params.put_bool("NAPScriptRunning", False, block=True)
    raise


def main(argv: list[str] | None = None) -> int:
  argv = list(sys.argv if argv is None else argv)
  if len(argv) < 4:
    print("Usage: run_script.py <title> <module> <instructions>")
    return 1

  title = argv[1]
  module = argv[2]
  instructions = argv[3]
  try:
    prepare_run(module)
  except (ToolSafetyError, ValueError) as exc:
    print(f"ERROR: {exc}")
    return 1

  import pyray as rl
  from openpilot.common.hardware import HARDWARE, PC
  from openpilot.system.ui.lib.application import gui_app, FontWeight
  from openpilot.system.ui.lib.scroll_panel import GuiScrollPanel
  from openpilot.system.ui.widgets.button import Button, ButtonStyle

  margin = 50
  title_font_size = 70
  text_font_size = 45
  output_font_size = 35
  line_height = 45
  button_width = 350
  button_height = 110
  button_spacing = 30

  class ScriptRunnerApp:
    def __init__(self):
      self._title = title
      self._module = module
      self._instructions = instructions
      self._state = ScriptState.READY
      self._output_lines: list[str] = []
      self._output_queue: queue.Queue[str] = queue.Queue()
      self._process: subprocess.Popen | None = None
      self._scroll_panel = GuiScrollPanel()
      self._instruction_lines: list[str] = []
      self._params = Params()
      self._font = None
      self._title_font = None
      self._start_button = None
      self._exit_button = None

    def init_ui(self):
      self._font = gui_app.font(FontWeight.NORMAL)
      self._title_font = gui_app.font(FontWeight.BOLD)
      self._start_button = Button("Start", click_callback=self._on_start, button_style=ButtonStyle.PRIMARY, font_size=text_font_size)
      self._exit_button = Button("Exit", click_callback=self._on_exit, button_style=ButtonStyle.TRANSPARENT_WHITE_BORDER, font_size=text_font_size)

    def _on_start(self):
      if self._state != ScriptState.READY:
        return
      self._state = ScriptState.RUNNING
      self._output_lines = ["Starting script...", ""]
      try:
        self._process = spawn_approved_module(self._module, self._params)
        threading.Thread(target=self._read_output, daemon=True).start()
      except Exception as exc:
        self._output_lines.append(f"Error starting script: {exc}")
        self._state = ScriptState.ERROR

    def _read_output(self):
      try:
        if self._process and self._process.stdout:
          for line in iter(self._process.stdout.readline, ""):
            if line:
              self._output_queue.put(line.rstrip())
        if self._process:
          code = self._process.wait()
          if code == 0:
            self._output_queue.put("\n[Script completed successfully]")
            self._state = ScriptState.COMPLETED
          else:
            self._output_queue.put(f"\n[Script exited with code {code}]")
            self._state = ScriptState.ERROR
      except Exception as exc:
        self._output_queue.put(f"\n[Error reading output: {exc}]")
        self._state = ScriptState.ERROR

    def _on_exit(self):
      if self._process and self._process.poll() is None:
        self._process.terminate()
        try:
          self._process.wait(timeout=2)
        except subprocess.TimeoutExpired:
          self._process.kill()
      self._params.put_bool("NAPScriptRunning", False, block=True)
      self._params.put_bool("NAPEpasRiskAccepted", False, block=True)
      gui_app.request_close()
      if not PC:
        HARDWARE.reboot()

    def render(self):
      rect = rl.Rectangle(0, 0, gui_app.width, gui_app.height)
      rl.draw_rectangle_rec(rect, rl.Color(20, 20, 20, 255))
      while True:
        try:
          self._output_lines.append(self._output_queue.get_nowait())
        except queue.Empty:
          break
      content_x = rect.x + margin
      current_y = rect.y + margin
      rl.draw_text_ex(self._title_font, self._title, rl.Vector2(content_x, current_y), title_font_size, 0, rl.WHITE)
      current_y += title_font_size + margin
      if self._state == ScriptState.READY:
        for i, line in enumerate(self._instructions.split("\n")):
          rl.draw_text_ex(self._font, line, rl.Vector2(content_x, current_y + i * line_height), text_font_size, 0, rl.Color(200, 200, 200, 255))
      else:
        for i, line in enumerate(self._output_lines[-40:]):
          rl.draw_text_ex(self._font, line, rl.Vector2(content_x, current_y + i * line_height), output_font_size, 0, rl.WHITE)
      button_y = rect.y + rect.height - margin - button_height
      if self._state == ScriptState.READY:
        self._start_button.render(rl.Rectangle(rect.x + rect.width - margin - button_width * 2 - button_spacing, button_y, button_width, button_height))
      self._exit_button.set_enabled(self._state != ScriptState.RUNNING)
      self._exit_button.render(rl.Rectangle(rect.x + rect.width - margin - button_width, button_y, button_width, button_height))

  subprocess.run(["tmux", "kill-session", "-t", "comma"], capture_output=True)
  gui_app.init_window("Pre-AP Script Runner")
  app = ScriptRunnerApp()
  app.init_ui()
  for _ in gui_app.render():
    app.render()
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
