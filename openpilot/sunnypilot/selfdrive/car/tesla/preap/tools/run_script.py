#!/usr/bin/env python3
"""Production Pre-AP script runner.

Launches an approved tool module after offroad + runtime-path checks.
Takes over the Comma screen. Native Tesla settings spawn this runner
instead of the tool on a pts. Development-only utilities are rejected.

Usage:
    python -m openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.run_script \\
      "Title" "openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.calibrate_pedal" "Instructions"
"""
from __future__ import annotations

import os
import queue
import subprocess
import sys
import threading
from pathlib import Path

from openpilot.common.basedir import BASEDIR
from openpilot.common.hardware import HARDWARE, PC
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.runner import (
  APPROVED_TOOLS,
  clear_tool_flags,
  mark_script_running,
  stop_child,
)
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.safety import (
  DESTRUCTIVE_TOOLS,
  ToolSafetyError,
  require_offroad,
  require_runtime_path,
)

APPROVED_MODULES = frozenset(APPROVED_TOOLS.values())


def follow_scroll_offset(line_count: int, line_height: float, bounds_height: float) -> float:
  overflow = line_count * line_height - bounds_height
  return -overflow if overflow > 0 else 0.0


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
  tool = next(name for name, path in APPROVED_TOOLS.items() if path == module)
  if tool in ("flash_epas", "restore_epas"):
    params.put_bool("NAPEpasRiskAccepted", True, block=True)
  env = os.environ.copy()
  env["PYTHONPATH"] = env.get("PYTHONPATH", BASEDIR)
  cmd = [sys.executable, "-m", module]
  if tool in DESTRUCTIVE_TOOLS:
    cmd.append("--confirm")
  mark_script_running(params)
  try:
    return subprocess.Popen(
      cmd,
      stdout=subprocess.PIPE,
      stderr=subprocess.STDOUT,
      cwd=str(Path(BASEDIR)),
      start_new_session=True,
      env=env,
      text=True,
      bufsize=1,
    )
  except Exception:
    clear_tool_flags(params)
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

      # Set NAPScriptRunning before spawn so manager stops pandad before
      # the child process tries to open Panda USB.
      mark_script_running(self._params)

      try:
        self._process = spawn_approved_module(self._module, self._params)
        threading.Thread(target=self._read_output, daemon=True).start()
      except Exception as exc:
        clear_tool_flags(self._params)
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
        if not stop_child(self._process):
          # Child still alive after SIGKILL. Bail without clearing
          # NAPScriptRunning — letting manager resume pandad while
          # a zombie script may still hold Panda USB would create
          # the exact dual-owner state this runner is meant to
          # prevent. Surface an error and keep the runner up.
          self._output_lines.append(
            "[ERROR] script did not exit; manager will stay paused")
          self._output_lines.append("Reboot the device to recover.")
          self._state = ScriptState.ERROR
          return

      # Clear NAPScriptRunning only after we've confirmed the child is gone.
      clear_tool_flags(self._params)

      gui_app.request_close()
      if not PC:
        HARDWARE.reboot()

    def render(self):
      rect = rl.Rectangle(0, 0, gui_app.width, gui_app.height)
      rl.draw_rectangle_rec(rect, rl.Color(20, 20, 20, 255))
      got_new = False
      while True:
        try:
          self._output_lines.append(self._output_queue.get_nowait())
          got_new = True
        except queue.Empty:
          break
      content_x = rect.x + margin
      current_y = rect.y + margin
      rl.draw_text_ex(self._title_font, self._title, rl.Vector2(content_x, current_y), title_font_size, 0, rl.WHITE)
      current_y += title_font_size + margin
      button_y = rect.y + rect.height - margin - button_height
      body_rect = rl.Rectangle(content_x, current_y, rect.width - margin * 2,
                               max(0.0, button_y - current_y - margin))
      if self._state == ScriptState.READY:
        lines = self._instructions.split("\n")
        font_size = text_font_size
        color = rl.Color(200, 200, 200, 255)
      else:
        lines = self._output_lines
        font_size = output_font_size
        color = rl.WHITE
      content_rect = rl.Rectangle(0, 0, body_rect.width, len(lines) * line_height)
      if got_new and self._state != ScriptState.READY:
        self._scroll_panel.set_offset(follow_scroll_offset(len(lines), line_height, body_rect.height))
      scroll = self._scroll_panel.update(body_rect, content_rect)
      rl.begin_scissor_mode(int(body_rect.x), int(body_rect.y), int(body_rect.width), int(body_rect.height))
      for i, line in enumerate(lines):
        line_y = body_rect.y + scroll + i * line_height
        if line_y + line_height < body_rect.y or line_y > body_rect.y + body_rect.height:
          continue
        rl.draw_text_ex(self._font, line, rl.Vector2(body_rect.x, line_y), font_size, 0, color)
      rl.end_scissor_mode()
      if self._state == ScriptState.READY:
        self._start_button.render(rl.Rectangle(rect.x + rect.width - margin - button_width * 2 - button_spacing,
                                               button_y, button_width, button_height))
      # Exit is disabled while RUNNING. The flash path can't be
      # safely interrupted mid-write; the calibration/test paths
      # also disable cancel for consistency with the original design.
      self._exit_button.set_enabled(self._state != ScriptState.RUNNING)
      self._exit_button.render(rl.Rectangle(rect.x + rect.width - margin - button_width,
                                            button_y, button_width, button_height))

  # tmux isn't installed on dev hosts, so swallow the FileNotFoundError.
  try:
    subprocess.run(["tmux", "kill-session", "-t", "comma"], capture_output=True)
  except FileNotFoundError:
    pass
  gui_app.init_window("Pre-AP Script Runner")
  app = ScriptRunnerApp()
  app.init_ui()
  for _ in gui_app.render():
    app.render()
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
