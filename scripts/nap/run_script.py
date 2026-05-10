#!/usr/bin/env python3
"""
NAP Script Runner

Standalone full-screen application for running scripts with live output display.
Launched as a separate process to take over the display.

Usage:
    ./run_script.py "Title" "module.path.to.script" "Instructions text..." [safety_class]

safety_class:
    "offroad_only" — refuse to launch and refuse Start if IsOnroad. For
        EPAS firmware operations (flash/extract/restore) — driving stack
        must be off, car must be parked.
    "stationary" (default) — no IsOnroad check. For pedal/radar
        calibration and tests, which require ignition on so the
        relevant ECU is electrically active.
"""

import sys
import subprocess
import threading
import queue
import pyray as rl

# Add openpilot to path
sys.path.insert(0, '/data/openpilot')

from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.scroll_panel import GuiScrollPanel
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets.button import Button, ButtonStyle
from openpilot.system.hardware import HARDWARE, PC

SAFETY_OFFROAD_ONLY = "offroad_only"
SAFETY_STATIONARY = "stationary"
VALID_SAFETY_CLASSES = (SAFETY_OFFROAD_ONLY, SAFETY_STATIONARY)

# Per-module required minimum safety class. Anything not listed is
# unknown and rejected — fail closed. Callers may *request* the same
# class or stricter; a weaker request is upgraded to the required one
# so a forgotten or stale call site can't downgrade an EPAS flash to
# a stationary classification.
MODULE_REQUIRED_SAFETY: dict[str, str] = {
  "scripts.nap.flash_epas":     SAFETY_OFFROAD_ONLY,
  "scripts.nap.extract_epas":   SAFETY_OFFROAD_ONLY,
  "scripts.nap.restore_epas":   SAFETY_OFFROAD_ONLY,
  "scripts.nap.calibrate_pedal":   SAFETY_STATIONARY,
  "scripts.nap.calibrate_radar":   SAFETY_STATIONARY,
  "scripts.nap.test_radar":     SAFETY_STATIONARY,
}


def resolve_safety_class(module: str, requested: str) -> str:
  """Resolve the effective safety class for `module`.

  Unknown module → strictest (offroad_only). Known module → max(required,
  requested) so a caller passing a weaker class is upgraded, never
  downgraded.
  """
  required = MODULE_REQUIRED_SAFETY.get(module)
  if required is None:
    return SAFETY_OFFROAD_ONLY
  # offroad_only is strictly stronger than stationary
  if required == SAFETY_OFFROAD_ONLY or requested == SAFETY_OFFROAD_ONLY:
    return SAFETY_OFFROAD_ONLY
  return SAFETY_STATIONARY

# UI Constants
MARGIN = 50
TITLE_FONT_SIZE = 70
TEXT_FONT_SIZE = 45
OUTPUT_FONT_SIZE = 35
LINE_HEIGHT = 45
BUTTON_WIDTH = 350
BUTTON_HEIGHT = 110
BUTTON_SPACING = 30


class ScriptState:
  READY = 0
  RUNNING = 1
  COMPLETED = 2
  ERROR = 3


class ScriptRunnerApp:
  def __init__(self, title: str, script_module: str, instructions: str,
               safety_class: str = SAFETY_STATIONARY):
    assert safety_class in VALID_SAFETY_CLASSES, f"unknown safety_class {safety_class!r}"
    self._title = title
    self._script_module = script_module
    self._instructions = instructions
    self._safety_class = safety_class

    self._state = ScriptState.READY
    self._output_lines: list[str] = []
    self._output_queue: queue.Queue[str] = queue.Queue()
    self._process: subprocess.Popen | None = None
    self._reader_thread: threading.Thread | None = None

    self._scroll_panel = GuiScrollPanel()
    self._instruction_lines: list[str] = []
    self._params = Params()

    # Will be initialized after gui_app.init_window()
    self._font = None
    self._title_font = None
    self._start_button = None
    self._exit_button = None

  def _init_ui(self):
    """Initialize UI components after window is created"""
    self._font = gui_app.font(FontWeight.NORMAL)
    self._title_font = gui_app.font(FontWeight.BOLD)

    self._start_button = Button(
      "Start",
      click_callback=self._on_start_clicked,
      button_style=ButtonStyle.PRIMARY,
      font_size=TEXT_FONT_SIZE
    )
    self._exit_button = Button(
      "Exit",
      click_callback=self._on_exit_clicked,
      button_style=ButtonStyle.TRANSPARENT_WHITE_BORDER,
      font_size=TEXT_FONT_SIZE
    )

  def _wrap_text(self, text: str, max_width: float, font_size: int) -> list[str]:
    lines = []
    for paragraph in text.split("\n"):
      if not paragraph.strip():
        lines.append("")
        continue

      words = paragraph.split()
      current_line = ""

      for word in words:
        test_line = f"{current_line} {word}".strip()
        text_width = measure_text_cached(self._font, test_line, font_size).x

        if text_width <= max_width:
          current_line = test_line
        else:
          if current_line:
            lines.append(current_line)
          current_line = word

      if current_line:
        lines.append(current_line)

    return lines

  def _on_start_clicked(self):
    if self._state != ScriptState.READY:
      return

    # Execution-boundary safety check for offroad-only scripts (EPAS
    # firmware ops). Catches the case where the user opened the runner
    # while parked, then the car transitioned onroad before they
    # pressed Start. Calibration/test scripts intentionally skip this
    # — they require ignition on for the ECU to be electrically active.
    if self._safety_class == SAFETY_OFFROAD_ONLY and self._params.get_bool("IsOnroad"):
      self._output_lines = [
        "Cannot run while car is on.",
        "",
        "Put the vehicle in park and turn it off, then try again.",
      ]
      self._state = ScriptState.ERROR
      return

    self._state = ScriptState.RUNNING
    self._output_lines = ["Starting script...", ""]

    # Set NAPScriptRunning before spawn so manager stops pandad before
    # the child process tries to open Panda USB.
    self._params.put_bool("NAPScriptRunning", True)

    try:
      self._process = subprocess.Popen(
        ["python", "-m", self._script_module],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        cwd="/data/openpilot",
        text=True,
        bufsize=1
      )

      self._reader_thread = threading.Thread(target=self._read_output, daemon=True)
      self._reader_thread.start()

    except Exception as e:
      self._params.put_bool("NAPScriptRunning", False)
      self._output_lines.append(f"Error starting script: {e}")
      self._state = ScriptState.ERROR

  def _read_output(self):
    try:
      if self._process and self._process.stdout:
        for line in iter(self._process.stdout.readline, ''):
          if line:
            self._output_queue.put(line.rstrip())
          if self._process.poll() is not None:
            break

      if self._process:
        return_code = self._process.wait()
        if return_code == 0:
          self._output_queue.put("\n[Script completed successfully]")
          self._state = ScriptState.COMPLETED
        else:
          self._output_queue.put(f"\n[Script exited with code {return_code}]")
          self._state = ScriptState.ERROR

    except Exception as e:
      self._output_queue.put(f"\n[Error reading output: {e}]")
      self._state = ScriptState.ERROR

  def _on_exit_clicked(self):
    if self._process and self._process.poll() is None:
      self._process.terminate()
      try:
        self._process.wait(timeout=2)
      except subprocess.TimeoutExpired:
        self._process.kill()

    # Clear NAPScriptRunning so manager resumes normal operation
    self._params.put_bool("NAPScriptRunning", False)

    gui_app.request_close()
    if not PC:
      HARDWARE.reboot()

  def _process_output_queue(self):
    while True:
      try:
        line = self._output_queue.get_nowait()
        self._output_lines.append(line)
      except queue.Empty:
        break

  def render(self):
    rect = rl.Rectangle(0, 0, gui_app.width, gui_app.height)

    # Draw background
    rl.draw_rectangle_rec(rect, rl.Color(20, 20, 20, 255))

    # Process pending output
    self._process_output_queue()

    content_x = rect.x + MARGIN
    content_width = rect.width - MARGIN * 2
    current_y = rect.y + MARGIN

    # Draw title
    title_size = measure_text_cached(self._title_font, self._title, TITLE_FONT_SIZE)
    rl.draw_text_ex(
      self._title_font,
      self._title,
      rl.Vector2(content_x, current_y),
      TITLE_FONT_SIZE,
      0,
      rl.WHITE
    )
    current_y += title_size.y + MARGIN

    # Draw separator
    rl.draw_line_ex(
      rl.Vector2(content_x, current_y),
      rl.Vector2(content_x + content_width, current_y),
      2,
      rl.Color(80, 80, 80, 255)
    )
    current_y += MARGIN

    if self._state == ScriptState.READY:
      self._render_instructions(rect, content_x, content_width, current_y)
    else:
      self._render_output(rect, content_x, content_width, current_y)

    self._render_buttons(rect)

  def _render_instructions(self, rect, content_x, content_width, start_y):
    if not self._instruction_lines:
      self._instruction_lines = self._wrap_text(self._instructions, content_width, TEXT_FONT_SIZE)

    button_area_height = BUTTON_HEIGHT + MARGIN * 2
    text_area_height = rect.height - start_y - button_area_height - rect.y

    current_y = start_y
    for line in self._instruction_lines:
      if current_y + LINE_HEIGHT > start_y + text_area_height:
        break
      rl.draw_text_ex(
        self._font,
        line,
        rl.Vector2(content_x, current_y),
        TEXT_FONT_SIZE,
        0,
        rl.Color(200, 200, 200, 255)
      )
      current_y += LINE_HEIGHT

  def _render_output(self, rect, content_x, content_width, start_y):
    button_area_height = BUTTON_HEIGHT + MARGIN * 2
    output_area_height = rect.height - start_y - button_area_height - rect.y

    output_rect = rl.Rectangle(content_x, start_y, content_width, output_area_height)

    content_height = len(self._output_lines) * LINE_HEIGHT
    content_rect = rl.Rectangle(0, 0, content_width, content_height)

    # Auto-scroll to bottom
    if content_height > output_area_height:
      self._scroll_panel._offset_filter_y.x = -(content_height - output_area_height)

    scroll = self._scroll_panel.update(output_rect, content_rect)

    rl.begin_scissor_mode(
      int(output_rect.x),
      int(output_rect.y),
      int(output_rect.width),
      int(output_rect.height)
    )

    for i, line in enumerate(self._output_lines):
      line_y = output_rect.y + scroll + i * LINE_HEIGHT

      if line_y + LINE_HEIGHT < output_rect.y or line_y > output_rect.y + output_rect.height:
        continue

      if "[Error" in line or "Error:" in line:
        color = rl.Color(255, 100, 100, 255)
      elif "[Script completed" in line:
        color = rl.Color(100, 255, 100, 255)
      elif line.startswith("***"):
        color = rl.Color(100, 200, 255, 255)
      else:
        color = rl.WHITE

      rl.draw_text_ex(
        self._font,
        line,
        rl.Vector2(output_rect.x, line_y),
        OUTPUT_FONT_SIZE,
        0,
        color
      )

    rl.end_scissor_mode()

    # Status indicator
    status_text = ""
    status_color = rl.WHITE
    if self._state == ScriptState.RUNNING:
      status_text = "Running..."
      status_color = rl.Color(255, 200, 100, 255)
    elif self._state == ScriptState.COMPLETED:
      status_text = "Completed"
      status_color = rl.Color(100, 255, 100, 255)
    elif self._state == ScriptState.ERROR:
      status_text = "Error"
      status_color = rl.Color(255, 100, 100, 255)

    if status_text:
      status_size = measure_text_cached(self._font, status_text, TEXT_FONT_SIZE)
      rl.draw_text_ex(
        self._font,
        status_text,
        rl.Vector2(rect.x + rect.width - MARGIN - status_size.x, start_y - LINE_HEIGHT),
        TEXT_FONT_SIZE,
        0,
        status_color
      )

  def _render_buttons(self, rect):
    button_y = rect.y + rect.height - MARGIN - BUTTON_HEIGHT

    if self._state == ScriptState.READY:
      start_rect = rl.Rectangle(
        rect.x + rect.width - MARGIN - BUTTON_WIDTH * 2 - BUTTON_SPACING,
        button_y,
        BUTTON_WIDTH,
        BUTTON_HEIGHT
      )
      exit_rect = rl.Rectangle(
        rect.x + rect.width - MARGIN - BUTTON_WIDTH,
        button_y,
        BUTTON_WIDTH,
        BUTTON_HEIGHT
      )
      self._start_button.render(start_rect)
      self._exit_button.render(exit_rect)
    else:
      exit_rect = rl.Rectangle(
        rect.x + rect.width - MARGIN - BUTTON_WIDTH,
        button_y,
        BUTTON_WIDTH,
        BUTTON_HEIGHT
      )
      self._exit_button.set_enabled(self._state != ScriptState.RUNNING)
      self._exit_button.render(exit_rect)


def main():
  if len(sys.argv) < 4:
    print("Usage: run_script.py <title> <module> <instructions> [safety_class]")
    print(f"  safety_class: one of {VALID_SAFETY_CLASSES} (default: {SAFETY_STATIONARY})")
    print("Example: run_script.py 'Pedal Calibration' 'scripts.nap.calibrate_pedal' 'Instructions...'")
    sys.exit(1)

  title = sys.argv[1]
  module = sys.argv[2]
  instructions = sys.argv[3]
  requested = sys.argv[4] if len(sys.argv) > 4 else SAFETY_STATIONARY
  if requested not in VALID_SAFETY_CLASSES:
    print(f"unknown safety_class {requested!r}; expected one of {VALID_SAFETY_CLASSES}")
    sys.exit(1)

  # Module-bound resolution: unknown modules and weaker-than-required
  # requests are upgraded to the strictest class. A caller cannot
  # accidentally downgrade an EPAS flash.
  safety_class = resolve_safety_class(module, requested)

  # Pre-window onroad gate for offroad-only scripts. Refuse BEFORE
  # killing the tmux session — refusing after the kill would strand the
  # user in a runner with the main UI dead.
  if safety_class == SAFETY_OFFROAD_ONLY and Params().get_bool("IsOnroad"):
    print("NAP runner: cannot run while car is on. Park it and try again.")
    sys.exit(0)

  # Kill the main openpilot UI tmux session so we can take over the screen
  subprocess.run(["tmux", "kill-session", "-t", "comma"], capture_output=True)

  gui_app.init_window("NAP Script Runner")

  app = ScriptRunnerApp(title, module, instructions, safety_class)
  app._init_ui()

  for _ in gui_app.render():
    app.render()


if __name__ == "__main__":
  main()
