"""Mici full-screen script runner.

Generic primitive: presents instructions, fires `on_start`, streams output
text into a scrolling card, fires `on_exit` on the exit action.
No subprocess, params, or hardware knowledge — those are the consumer's job.

The app owns its own `gui_app` window because it expects the device's main
UI to be torn down by the time it renders (script runners take exclusive
hardware access). It is not a widget you embed; it is a standalone screen.

States:
    READY      — instructions visible, start button at the bottom
    RUNNING    — output streaming, exit button at the bottom (cancels)
    COMPLETED  — output frozen, exit button (cleanup + close)
    ERROR      — output frozen with error tint, exit button
"""
import enum
import threading
from collections.abc import Callable

import pyray as rl

from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.mici_setup import BigPillButton, GreyBigButton
from openpilot.system.ui.widgets.scroller import NavScroller


class ScriptState(enum.IntEnum):
  READY = 0
  RUNNING = 1
  COMPLETED = 2
  ERROR = 3


# Output is rendered as a single card to keep the scroller cheap when many
# lines arrive. Lines join with newlines.
_MAX_OUTPUT_LINES = 200


class _OutputCard(GreyBigButton):
  """Read-only text card that accumulates output lines.

  Visually identical to GreyBigButton, but with public methods to
  append a line and to recolor (e.g. red on error).
  """

  def __init__(self):
    super().__init__("output", "")
    self._lines: list[str] = []

  def append(self, line: str) -> None:
    self._lines.append(line)
    if len(self._lines) > _MAX_OUTPUT_LINES:
      # Keep the newest; older output scrolls out of memory.
      self._lines = self._lines[-_MAX_OUTPUT_LINES:]
    self.set_value("\n".join(self._lines))

  def set_tint(self, color: rl.Color) -> None:
    # _sub_label is BigButton-internal; touched here because the public
    # API doesn't expose colour on the value text. If this primitive
    # graduates to upstream, push the colour setter into BigButton.
    self._sub_label.set_text_color(color)


class MiciScriptRunnerApp:
  """Standalone full-screen script-runner app.

  Construct, then call .run() — blocks until the user exits. Callers push
  output via append_output() from any thread and transition state via
  set_state(). The output buffer is thread-safe enough for the typical
  "reader thread pumps lines, render thread reads them" pattern because
  pyray text rendering only touches the buffer at render time.
  """

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
    self._state = ScriptState.READY
    self._lock = threading.Lock()
    self._pending_output: list[str] = []

    # Built in _build_ui() after gui_app.init_window.
    self._scroller: NavScroller | None = None
    self._title_card: GreyBigButton | None = None
    self._output_card: _OutputCard | None = None
    self._action_button: BigPillButton | None = None

  # ── public api ──────────────────────────────────────────

  def run(self, window_title: str = "Script Runner") -> None:
    """Open the window and render until close. Blocks."""
    gui_app.init_window(window_title)
    self._build_ui()
    for _ in gui_app.render():
      self._drain_pending_output()

  def append_output(self, line: str) -> None:
    """Push a line into the output area. Safe to call from any thread."""
    with self._lock:
      self._pending_output.append(line)

  def set_state(self, state: ScriptState) -> None:
    """Transition state. Re-layouts the action button on next render."""
    if state == self._state:
      return
    self._state = state
    if self._scroller is None:
      return
    self._rebuild_button()
    if state == ScriptState.ERROR and self._output_card is not None:
      self._output_card.set_tint(rl.Color(255, 100, 100, 255))

  @property
  def state(self) -> ScriptState:
    return self._state

  # ── internals ──────────────────────────────────────────

  def _build_ui(self) -> None:
    self._scroller = NavScroller()

    self._title_card = GreyBigButton(self._title, _initial_subtitle(self._state))
    instruction_cards = [GreyBigButton("", para) for para in _split_paragraphs(self._instructions)]
    self._output_card = _OutputCard()
    self._output_card.set_visible(False)

    self._action_button = BigPillButton(_action_label(self._state), green=(self._state == ScriptState.READY))
    self._action_button.set_click_callback(self._handle_action)

    self._scroller._scroller.add_widgets([
      self._title_card,
      *instruction_cards,
      self._output_card,
      self._action_button,
    ])

    gui_app.push_widget(self._scroller)

  def _rebuild_button(self) -> None:
    if self._action_button is None or self._output_card is None or self._title_card is None:
      return
    self._action_button.set_text(_action_label(self._state))
    self._action_button.set_green(self._state == ScriptState.READY)
    if self._state != ScriptState.READY:
      self._output_card.set_visible(True)
    self._title_card.set_value(_initial_subtitle(self._state))

  def _drain_pending_output(self) -> None:
    if self._output_card is None:
      return
    with self._lock:
      lines = self._pending_output
      self._pending_output = []
    for line in lines:
      self._output_card.append(line)

  def _handle_action(self) -> None:
    if self._state == ScriptState.READY:
      self._on_start()
    else:
      self._on_exit()


def _split_paragraphs(text: str) -> list[str]:
  """Break instructions on blank lines so the scroller shows them as
  separate cards. Falls back to one card if there are no blank lines."""
  paragraphs = [p.strip() for p in text.split("\n\n")]
  return [p for p in paragraphs if p] or [text]


def _action_label(state: ScriptState) -> str:
  return "start" if state == ScriptState.READY else "exit"


def _initial_subtitle(state: ScriptState) -> str:
  return {
    ScriptState.READY: "scroll to read, then tap start",
    ScriptState.RUNNING: "running",
    ScriptState.COMPLETED: "completed",
    ScriptState.ERROR: "error",
  }[state]


__all__ = ["MiciScriptRunnerApp", "ScriptState"]
