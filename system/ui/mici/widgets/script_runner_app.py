"""Mici full-screen script runner.

Standalone app: owns its own ``gui_app`` window because the device's main UI
gets torn down before the runner takes over (the consumer typically sets a
param that asks the manager to stop driving processes). It is not a widget
you embed.

The UI is plain raylib at mici-scaled sizes — title and status at the top,
a clipped scrollable text region in the body for instructions (before
start) or live output (after), and a compact action button at the bottom.
No card chrome, no NavScroller. This is a tool, not an onboarding flow.

States:
    READY      — instructions visible, primary Start action
    RUNNING    — output streaming, Exit action (may be disabled by consumer)
    COMPLETED  — output frozen, Exit action
    ERROR      — output frozen with red status, Exit action

Threading: ``append_output`` and ``set_state`` are safe to call from any
thread. Both push onto queues. The render loop drains and mutates UI
objects from one thread only.

Generic primitive: no NAP, subprocess, or hardware knowledge. The consumer
wires ``on_start`` and ``on_exit`` callbacks and decides what those mean.
"""
import enum
import queue
import threading
from collections.abc import Callable

import pyray as rl

from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.scroll_panel import GuiScrollPanel
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets.button import Button, ButtonStyle


class ScriptState(enum.IntEnum):
  READY = 0
  RUNNING = 1
  COMPLETED = 2
  ERROR = 3


# Mici-scaled constants. The logical canvas is 536x240 on a comma 4 device;
# anything larger than this stops feeling like a tool.
MARGIN_X = 16
MARGIN_TOP = 10
MARGIN_BOTTOM = 10

TITLE_FONT_SIZE = 24
STATUS_FONT_SIZE = 16
BODY_FONT_SIZE = 18
LINE_HEIGHT = 22

BUTTON_W = 120
BUTTON_H = 36

TITLE_BAR_H = TITLE_FONT_SIZE + 8
BUTTON_BAR_H = BUTTON_H + MARGIN_BOTTOM

# Lines older than this fall off the back of the output buffer. Practical
# scripts rarely emit more than a few hundred lines; the cap protects the
# render loop in pathological cases.
MAX_OUTPUT_LINES = 500


class MiciScriptRunnerApp:
  def __init__(
    self,
    title: str,
    instructions: str,
    on_start: Callable[[], None],
    on_exit: Callable[[], None],
  ):
    self._title = title
    self._instructions = instructions
    self._on_start = on_start
    self._on_exit = on_exit

    # State owned by the render thread.
    self._state = ScriptState.READY
    self._output_lines: list[str] = []
    self._instruction_lines: list[str] = []  # wrapped once after _init_ui

    # Producer threads enqueue here; render thread drains.
    self._output_queue: queue.Queue[str] = queue.Queue()
    self._state_queue: queue.Queue[ScriptState] = queue.Queue()

    # Consumer-controlled flag: action button enabled for the current
    # state. Read by the render thread, written by either thread; a bare
    # bool is safe under the GIL for this kind of write/read.
    self._action_enabled: bool = True
    self._action_enabled_lock = threading.Lock()

    # Initialised in _init_ui after gui_app.init_window.
    self._font = None
    self._title_font = None
    self._scroll_panel: GuiScrollPanel | None = None
    self._action_button: Button | None = None

  # ── public api ──────────────────────────────────────────────────────────

  def run(self, window_title: str = "Script Runner") -> None:
    gui_app.init_window(window_title)
    self._init_ui()
    for _ in gui_app.render():
      self._drain_queues()
      self._render_frame()

  def append_output(self, line: str) -> None:
    """Push a line into the output area. Safe from any thread."""
    self._output_queue.put(line)

  def set_state(self, state: ScriptState) -> None:
    """Transition state. Safe from any thread; applied on next frame."""
    self._state_queue.put(state)

  def set_action_enabled(self, enabled: bool) -> None:
    """Enable or disable the action button for the current state.

    Consumer uses this to make Exit non-clickable during a destructive
    RUNNING phase (e.g. mid-flash, where interrupting would brick).
    """
    with self._action_enabled_lock:
      self._action_enabled = enabled

  @property
  def state(self) -> ScriptState:
    return self._state

  # ── internals ───────────────────────────────────────────────────────────

  def _init_ui(self) -> None:
    self._font = gui_app.font(FontWeight.NORMAL)
    self._title_font = gui_app.font(FontWeight.BOLD)
    self._scroll_panel = GuiScrollPanel()
    self._action_button = Button(
      "start",
      click_callback=self._handle_action,
      button_style=ButtonStyle.PRIMARY,
      font_size=BODY_FONT_SIZE,
    )

    content_width = gui_app.width - MARGIN_X * 2
    self._instruction_lines = _wrap_text(
      self._font, self._instructions, content_width, BODY_FONT_SIZE
    )

  def _drain_queues(self) -> None:
    assert self._action_button is not None

    new_state = self._state
    drained_state = False
    while True:
      try:
        new_state = self._state_queue.get_nowait()
        drained_state = True
      except queue.Empty:
        break
    if drained_state and new_state != self._state:
      self._state = new_state
      self._action_button.set_text("start" if new_state == ScriptState.READY else "exit")
      # Note: _action_enabled is consumer-controlled. We deliberately
      # don't reset it on transitions — racing with the consumer's
      # set_action_enabled call would lose the consumer's preference.

    while True:
      try:
        line = self._output_queue.get_nowait()
      except queue.Empty:
        break
      self._output_lines.append(line)
    if len(self._output_lines) > MAX_OUTPUT_LINES:
      self._output_lines = self._output_lines[-MAX_OUTPUT_LINES:]

    with self._action_enabled_lock:
      self._action_button.set_enabled(self._action_enabled)

  def _render_frame(self) -> None:
    rect = rl.Rectangle(0, 0, gui_app.width, gui_app.height)
    rl.draw_rectangle_rec(rect, rl.Color(20, 20, 20, 255))

    title_y = MARGIN_TOP
    rl.draw_text_ex(self._title_font, self._title,
                    rl.Vector2(MARGIN_X, title_y),
                    TITLE_FONT_SIZE, 0, rl.WHITE)

    status = _status_text(self._state)
    if status:
      status_w = measure_text_cached(self._font, status, STATUS_FONT_SIZE).x
      status_y = title_y + (TITLE_FONT_SIZE - STATUS_FONT_SIZE) // 2
      rl.draw_text_ex(self._font, status,
                      rl.Vector2(rect.width - MARGIN_X - status_w, status_y),
                      STATUS_FONT_SIZE, 0, _status_color(self._state))

    sep_y = title_y + TITLE_FONT_SIZE + 4
    rl.draw_line_ex(
      rl.Vector2(MARGIN_X, sep_y),
      rl.Vector2(rect.width - MARGIN_X, sep_y),
      1, rl.Color(80, 80, 80, 255),
    )

    body_top = sep_y + 6
    body_bottom = rect.height - BUTTON_BAR_H - 4
    body_rect = rl.Rectangle(MARGIN_X, body_top,
                              rect.width - 2 * MARGIN_X,
                              body_bottom - body_top)

    lines = self._instruction_lines if self._state == ScriptState.READY else self._output_lines
    self._render_text_block(body_rect, lines)

    button_y = rect.height - MARGIN_BOTTOM - BUTTON_H
    button_x = rect.width - MARGIN_X - BUTTON_W
    assert self._action_button is not None
    self._action_button.render(rl.Rectangle(button_x, button_y, BUTTON_W, BUTTON_H))

  def _render_text_block(self, body_rect: rl.Rectangle, lines: list[str]) -> None:
    assert self._scroll_panel is not None

    content_h = len(lines) * LINE_HEIGHT
    content_rect = rl.Rectangle(0, 0, body_rect.width, content_h)

    # Output streams in; auto-pin to bottom when it would otherwise be
    # offscreen. Instructions don't grow after init, so this pin is mostly
    # relevant in RUNNING/COMPLETED/ERROR states.
    if content_h > body_rect.height and self._state != ScriptState.READY:
      self._scroll_panel._offset_filter_y.x = -(content_h - body_rect.height)

    scroll = self._scroll_panel.update(body_rect, content_rect)

    rl.begin_scissor_mode(
      int(body_rect.x), int(body_rect.y),
      int(body_rect.width), int(body_rect.height),
    )
    for i, line in enumerate(lines):
      y = body_rect.y + scroll + i * LINE_HEIGHT
      if y + LINE_HEIGHT < body_rect.y or y > body_rect.y + body_rect.height:
        continue
      color = _line_color(line)
      rl.draw_text_ex(self._font, line,
                      rl.Vector2(body_rect.x, y),
                      BODY_FONT_SIZE, 0, color)
    rl.end_scissor_mode()

  def _handle_action(self) -> None:
    if self._state == ScriptState.READY:
      self._on_start()
    else:
      self._on_exit()


def _wrap_text(font, text: str, max_width: float, font_size: int) -> list[str]:
  lines: list[str] = []
  for paragraph in text.split("\n"):
    if not paragraph.strip():
      lines.append("")
      continue
    words = paragraph.split()
    current = ""
    for word in words:
      test = f"{current} {word}".strip()
      width = measure_text_cached(font, test, font_size).x
      if width <= max_width:
        current = test
      else:
        if current:
          lines.append(current)
        current = word
    if current:
      lines.append(current)
  return lines


def _status_text(state: ScriptState) -> str:
  return {
    ScriptState.READY: "",
    ScriptState.RUNNING: "running",
    ScriptState.COMPLETED: "completed",
    ScriptState.ERROR: "error",
  }[state]


def _status_color(state: ScriptState) -> rl.Color:
  return {
    ScriptState.RUNNING: rl.Color(255, 200, 100, 255),
    ScriptState.COMPLETED: rl.Color(100, 255, 100, 255),
    ScriptState.ERROR: rl.Color(255, 100, 100, 255),
  }.get(state, rl.WHITE)


def _line_color(line: str) -> rl.Color:
  if "[Error" in line or "Error:" in line or line.startswith("[ERROR]"):
    return rl.Color(255, 100, 100, 255)
  if "[Script completed" in line:
    return rl.Color(100, 255, 100, 255)
  if line.startswith("***"):
    return rl.Color(100, 200, 255, 255)
  return rl.WHITE


__all__ = ["MiciScriptRunnerApp", "ScriptState"]
