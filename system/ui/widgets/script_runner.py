import subprocess
import threading
from collections import deque

import pyray as rl

from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.button import Button, ButtonStyle

# Processes to stop when running a script that needs hardware access
CONFLICTING_PROCESSES = [
  "selfdrive.pandad",
  "selfdrive.car.card",
  "selfdrive.controls.controlsd",
  "selfdrive.selfdrived",
  "selfdrive.controls.plannerd",
  "selfdrive.controls.radard",
  "selfdrive.locationd",
  "selfdrive.modeld",
]

# Layout constants
TITLE_HEIGHT = 80
BUTTON_HEIGHT = 80
BUTTON_WIDTH = 250
MARGIN = 40
OUTPUT_PADDING = 15
LINE_HEIGHT = 28
FONT_SIZE = 24
TITLE_FONT_SIZE = 50
MAX_OUTPUT_LINES = 1000

# Colors
BG_COLOR = rl.Color(20, 20, 20, 255)
OUTPUT_BG_COLOR = rl.Color(10, 10, 10, 255)
OUTPUT_TEXT_COLOR = rl.Color(0, 230, 0, 255)
STATUS_TEXT_COLOR = rl.Color(180, 180, 180, 255)
ERROR_TEXT_COLOR = rl.Color(255, 80, 80, 255)


class ScriptRunner(Widget):
  """Fullscreen modal overlay that stops openpilot, runs a subprocess,
  displays output, and restarts openpilot when done.

  Usage:
    runner = ScriptRunner("Calibrate Pedal", ["python", "-m", "calibrate_pedal"])
    gui_app.set_modal_overlay(runner, callback=on_done)
  """

  def __init__(self, title: str, cmd: list[str], stop_openpilot: bool = True):
    super().__init__()
    self._title = title
    self._cmd = cmd
    self._stop_openpilot = stop_openpilot

    self._output_lines: deque[str] = deque(maxlen=MAX_OUTPUT_LINES)
    self._process: subprocess.Popen | None = None
    self._reader_thread: threading.Thread | None = None
    self._script_running = False
    self._script_started = False
    self._exit_code: int | None = None
    self._result = DialogResult.NO_ACTION
    self._lock = threading.Lock()

    self._font = gui_app.font(FontWeight.NORMAL)
    self._title_font = gui_app.font(FontWeight.BOLD)
    self._close_btn = Button("Close", self._on_close, button_style=ButtonStyle.PRIMARY)
    self._cancel_btn = Button("Cancel", self._on_cancel, button_style=ButtonStyle.DANGER)

    self._params = Params()

  def show_event(self):
    if not self._script_started:
      self._script_started = True
      self._start()

  def _start(self):
    """Stop conflicting processes and start the script subprocess."""
    if self._stop_openpilot:
      self._params.put_bool("NAPScriptRunning", True)
      self._append_line("[NAP] Stopping openpilot processes...")

      # Kill conflicting processes directly for immediate effect.
      # The manager will also see NAPScriptRunning and avoid restarting them.
      for proc_name in CONFLICTING_PROCESSES:
        subprocess.run(['pkill', '-f', proc_name], capture_output=True)

      self._append_line("[NAP] Processes stopped.")

    self._append_line(f"[NAP] Starting: {' '.join(self._cmd)}")
    self._append_line("=" * 60)

    try:
      self._process = subprocess.Popen(
        self._cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=1,
        text=True,
      )
      self._script_running = True
      self._reader_thread = threading.Thread(target=self._read_output, daemon=True)
      self._reader_thread.start()
    except Exception as e:
      self._append_line(f"[NAP] Error starting script: {e}")
      self._script_running = False

  def _read_output(self):
    """Background thread: read subprocess stdout line by line."""
    try:
      for line in self._process.stdout:
        self._append_line(line.rstrip('\n'))
    except Exception as e:
      self._append_line(f"[Error reading output: {e}]")
    finally:
      self._process.wait()
      with self._lock:
        self._exit_code = self._process.returncode
        self._script_running = False
      self._append_line("=" * 60)
      if self._exit_code == 0:
        self._append_line("[NAP] Script finished successfully.")
      else:
        self._append_line(f"[NAP] Script exited with code {self._exit_code}")

  def _append_line(self, text: str):
    with self._lock:
      self._output_lines.append(text)

  def _on_close(self):
    self._cleanup()
    self._result = DialogResult.CONFIRM

  def _on_cancel(self):
    if self._process and self._process.poll() is None:
      self._append_line("[NAP] Cancelling script...")
      self._process.terminate()
      try:
        self._process.wait(timeout=3)
      except subprocess.TimeoutExpired:
        self._process.kill()
        self._process.wait()
    self._cleanup()
    self._result = DialogResult.CANCEL

  def _cleanup(self):
    """Restore openpilot processes by clearing the script-running flag."""
    if self._stop_openpilot:
      self._params.put_bool("NAPScriptRunning", False)

  def _render(self, rect: rl.Rectangle):
    # Full-screen dark background
    rl.draw_rectangle_rec(rect, BG_COLOR)

    # Title
    rl.draw_text_ex(
      self._title_font, self._title,
      rl.Vector2(rect.x + MARGIN, rect.y + 15),
      TITLE_FONT_SIZE, 0, rl.WHITE,
    )

    # Status indicator next to title
    with self._lock:
      running = self._script_running
      exit_code = self._exit_code

    if running:
      status = "Running..."
      status_color = STATUS_TEXT_COLOR
    elif exit_code is not None and exit_code == 0:
      status = "Completed"
      status_color = OUTPUT_TEXT_COLOR
    elif exit_code is not None:
      status = f"Failed (exit {exit_code})"
      status_color = ERROR_TEXT_COLOR
    else:
      status = "Starting..."
      status_color = STATUS_TEXT_COLOR

    title_size = measure_text_cached(self._title_font, self._title, TITLE_FONT_SIZE)
    rl.draw_text_ex(
      self._font, status,
      rl.Vector2(rect.x + MARGIN + title_size.x + 30, rect.y + 30),
      FONT_SIZE, 0, status_color,
    )

    # Button bar at bottom
    btn_y = rect.y + rect.height - BUTTON_HEIGHT - MARGIN
    btn_rect = rl.Rectangle(
      rect.x + rect.width - BUTTON_WIDTH - MARGIN,
      btn_y, BUTTON_WIDTH, BUTTON_HEIGHT,
    )

    if running:
      self._cancel_btn.render(btn_rect)
    else:
      self._close_btn.render(btn_rect)

    # Output area
    output_top = rect.y + TITLE_HEIGHT + 10
    output_bottom = btn_y - 20
    output_rect = rl.Rectangle(
      rect.x + MARGIN, output_top,
      rect.width - MARGIN * 2, output_bottom - output_top,
    )

    # Output background with rounded corners
    rl.draw_rectangle_rounded(output_rect, 0.01, 10, OUTPUT_BG_COLOR)

    # Clip output text to the output area
    rl.begin_scissor_mode(
      int(output_rect.x), int(output_rect.y),
      int(output_rect.width), int(output_rect.height),
    )

    with self._lock:
      lines = list(self._output_lines)

    # Auto-scroll: show the most recent lines that fit
    visible_lines = int((output_rect.height - OUTPUT_PADDING * 2) / LINE_HEIGHT)
    start = max(0, len(lines) - visible_lines)

    y = output_rect.y + OUTPUT_PADDING
    for i in range(start, len(lines)):
      line = lines[i]
      color = OUTPUT_TEXT_COLOR
      if line.startswith("[NAP]"):
        color = STATUS_TEXT_COLOR
      elif line.startswith("[Error"):
        color = ERROR_TEXT_COLOR
      rl.draw_text_ex(
        self._font, line,
        rl.Vector2(output_rect.x + OUTPUT_PADDING, y),
        FONT_SIZE, 0, color,
      )
      y += LINE_HEIGHT
      if y > output_rect.y + output_rect.height - OUTPUT_PADDING:
        break

    rl.end_scissor_mode()

    return self._result
