"""On-road camera speed-sign HUD (Yes saves a local map limit; No is log-only).

Large opaque MUTCD-style plate on the driver / left side of the onroad UI.
TICI: below MAX. Mici: top-left.

When the logger is On and ONNX weights failed to load, the plate shows
NO WT instead of staying blank (and never a fake mph). When the weights
are present but the CPU runtime is not installed, the plate shows NO RT
(Install weights also installs that runtime). When openpilot is
actively controlling, YOLO is skipped and the plate shows WAIT. If WAIT
appears while the UI looks disengaged, that is a bug — report swaglog
lines matching speedsignd. Manual driving (moving OK) still runs detect.

Yes/No ("is this accurate?") appears only with a live mph — not for NO WT
or NO RT.
Taps are logged to speedsign_feedback/ (JSONL + crop JPEG). Yes also becomes a local
map override (mapd/speed_overrides.py) shown on the regular speed-limit sign;
No is log-only. Either tap hides this popup at once (AnswerGate).
"""
from __future__ import annotations

import pyray as rl
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.speedsignd.hud import (
  AnswerGate,
  point_in_confirm_buttons,
  HUD_CONFIRM_NO,
  HUD_CONFIRM_PROMPT,
  HUD_CONFIRM_YES,
  HUD_LABEL,
  LiveSignView,
  MICI_CONFIRM_BTN_W,
  MICI_CONFIRM_GAP,
  MICI_SIGN_H,
  MICI_SIGN_W,
  TICI_CONFIRM_BTN_H,
  TICI_CONFIRM_GAP,
  TICI_CONFIRM_PROMPT_H,
  TICI_SIGN_H,
  TICI_SIGN_W,
  accuracy_button_rects,
  apply_runtime_plate,
  hit_accuracy_button,
  hud_confirm_visible,
  live_sign_from_event,
  mici_sign_origin,
  plate_digit_size,
  tici_sign_origin,
)
from openpilot.selfdrive.speedsignd.install import runtime_missing
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight, MousePos
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

# TICI digits bigger than OSM LIMIT (64pt), smaller than live speed (176pt).
TICI_W, TICI_H = TICI_SIGN_W, TICI_SIGN_H
TICI_DIGIT = 120
TICI_LABEL = 36
TICI_CONFIRM_FONT = 48
TICI_PROMPT_FONT = 28
# Mici 536x240 — still a solid plate, not a toast.
MICI_W, MICI_H = MICI_SIGN_W, MICI_SIGN_H
MICI_DIGIT = 72
MICI_LABEL = 18
MICI_CONFIRM_FONT = 22

_SIGN_BG = rl.WHITE
_SIGN_BORDER = rl.Color(180, 40, 40, 255)
_SIGN_INK = rl.BLACK
_SIGN_LABEL = rl.Color(40, 40, 40, 255)
_SIGN_MISSING_INK = rl.Color(160, 40, 40, 255)
_SIGN_PAUSED_INK = rl.Color(90, 90, 90, 255)
_YES_BG = rl.Color(36, 130, 72, 235)
_YES_BG_PRESSED = rl.Color(48, 160, 88, 255)
_NO_BG = rl.Color(170, 40, 40, 235)
_NO_BG_PRESSED = rl.Color(200, 52, 52, 255)
_BTN_INK = rl.WHITE
_PROMPT_INK = rl.Color(230, 230, 230, 255)


def live_sign_from_sm(sm, enabled: bool) -> LiveSignView:
  if not enabled:
    return LiveSignView()
  try:
    d = sm["liveSpeedSignNAP"]
  except Exception:
    return LiveSignView()
  msg_valid = False
  try:
    msg_valid = bool(sm.valid["liveSpeedSignNAP"])
  except Exception:
    msg_valid = False
  view = live_sign_from_event(True, d, msg_valid)
  # Disk, not a new cereal field. NO WT from the message still wins.
  return apply_runtime_plate(view, runtime_missing=runtime_missing())


def logger_enabled() -> bool:
  return bool(getattr(ui_state, "speed_sign_log", False))


def tici_sign_rect(rect: rl.Rectangle) -> rl.Rectangle:
  """Driver-left, below the MAX box. Same anchor for SIGN mph and NO WT."""
  x, y = tici_sign_origin(rect.x, rect.y)
  return rl.Rectangle(x, y, TICI_W, TICI_H)


def mici_sign_rect(rect: rl.Rectangle) -> rl.Rectangle:
  """Left / driver side, analogous to TICI."""
  x, y = mici_sign_origin(rect.x, rect.y)
  return rl.Rectangle(x, y, MICI_W, MICI_H)


def tici_confirm_rects(sign: rl.Rectangle) -> tuple[tuple[float, float, float, float], tuple[float, float, float, float]]:
  return accuracy_button_rects(
    sign.x, sign.y, sign.width, sign.height,
    beside=False, btn_h=TICI_CONFIRM_BTN_H, gap=TICI_CONFIRM_GAP, prompt_h=TICI_CONFIRM_PROMPT_H,
  )


def mici_confirm_rects(sign: rl.Rectangle) -> tuple[tuple[float, float, float, float], tuple[float, float, float, float]]:
  return accuracy_button_rects(
    sign.x, sign.y, sign.width, sign.height,
    beside=True, btn_w=MICI_CONFIRM_BTN_W, gap=MICI_CONFIRM_GAP,
  )


def on_confirm_accuracy(mph: int, yes: bool) -> None:
  """Yes/No feedback: append to /data/media/0/nap/speedsign_feedback/feedback.jsonl.

  Keeps the peak-size crop JPEG speedsignd saved for the plate (pending.jpg)
  and, on Yes, also appends confirmed.jsonl (future OSM confirmed readings;
  nothing is uploaded to osm.org). The row and crop are mirrored into the
  current realdata segment for athena harvest.

  Yes only: also sets NAPSpeedSignConfirmed; mapd stores it as a local
  override for the matched OSM way + heading, so the regular speed-limit sign
  and Map Speed Limit (MAX = limit + offset) use it now and on future drives
  (selfdrive/mapd/speed_overrides.py). No is log-only.
  """
  import time
  from openpilot.selfdrive.speedsignd.two_stage import record_answer
  rec = record_answer(int(mph), bool(yes))
  applied = False
  if yes:
    from openpilot.selfdrive.mapd.speed_overrides import request_override
    applied = request_override(int(mph), rec, time.monotonic())
  cloudlog.info("speed_sign_hud feedback mph=%s yes=%s saved=%s jpg=%s lat=%s lon=%s realdata=%s applied=%s",
                int(mph), bool(yes), rec is not None, (rec or {}).get("jpg"),
                (rec or {}).get("lat"), (rec or {}).get("lon"), (rec or {}).get("realdata"), applied)


def draw_speed_sign_plate(
  sign: rl.Rectangle,
  text: str,
  font_bold,
  font_label,
  digit_size: int,
  label_size: int,
  *,
  ink=None,
) -> None:
  rl.draw_rectangle_rounded(sign, 0.10, 8, _SIGN_BG)
  rl.draw_rectangle_rounded_lines_ex(sign, 0.10, 8, 6, _SIGN_BORDER)
  label_w = measure_text_cached(font_label, HUD_LABEL, label_size).x
  rl.draw_text_ex(
    font_label, HUD_LABEL,
    rl.Vector2(sign.x + (sign.width - label_w) / 2, sign.y + max(6, sign.height * 0.06)),
    label_size, 0, _SIGN_LABEL,
  )
  num = str(text)
  size = plate_digit_size(num, digit_size)
  num_w = measure_text_cached(font_bold, num, size).x
  num_h = measure_text_cached(font_bold, num, size).y
  color = _SIGN_INK if ink is None else ink
  rl.draw_text_ex(
    font_bold, num,
    rl.Vector2(sign.x + (sign.width - num_w) / 2, sign.y + sign.height * 0.38 - num_h / 4),
    size, 0, color,
  )


def _rect_from_tuple(r: tuple[float, float, float, float]) -> rl.Rectangle:
  return rl.Rectangle(r[0], r[1], r[2], r[3])


def _draw_confirm_button(r: tuple[float, float, float, float], label: str, font, font_size: int,
                         bg, pressed: bool, pressed_bg) -> None:
  box = _rect_from_tuple(r)
  rl.draw_rectangle_rounded(box, 0.18, 8, pressed_bg if pressed else bg)
  text_w = measure_text_cached(font, label, font_size).x
  text_h = measure_text_cached(font, label, font_size).y
  rl.draw_text_ex(
    font, label,
    rl.Vector2(box.x + (box.width - text_w) / 2, box.y + (box.height - text_h) / 2),
    font_size, 0, _BTN_INK,
  )


def draw_accuracy_overlay(
  sign: rl.Rectangle,
  yes: tuple[float, float, float, float],
  no: tuple[float, float, float, float],
  font,
  *,
  tici: bool,
  pressed_yes: bool = False,
  pressed_no: bool = False,
) -> None:
  font_size = TICI_CONFIRM_FONT if tici else MICI_CONFIRM_FONT
  if tici:
    prompt_w = measure_text_cached(font, HUD_CONFIRM_PROMPT, TICI_PROMPT_FONT).x
    rl.draw_text_ex(
      font, HUD_CONFIRM_PROMPT,
      rl.Vector2(sign.x + (sign.width - prompt_w) / 2, sign.y + sign.height + 6),
      TICI_PROMPT_FONT, 0, _PROMPT_INK,
    )
  _draw_confirm_button(yes, HUD_CONFIRM_YES, font, font_size, _YES_BG, pressed_yes, _YES_BG_PRESSED)
  _draw_confirm_button(no, HUD_CONFIRM_NO, font, font_size, _NO_BG, pressed_no, _NO_BG_PRESSED)


_GATE = AnswerGate()


def _now() -> float:
  import time
  return time.monotonic()


def _draw_live_sign(rect_fn, rect: rl.Rectangle, font_bold, font_label, digit_size: int, label_size: int) -> LiveSignView:
  view = _GATE.filter(live_sign_from_sm(ui_state.sm, logger_enabled()), _now())
  if not view.show:
    return view
  if view.show_missing_weights or view.show_runtime_missing:
    ink = _SIGN_MISSING_INK
  elif view.show_detect_paused:
    ink = _SIGN_PAUSED_INK
  else:
    ink = _SIGN_INK
  draw_speed_sign_plate(
    rect_fn(rect), view.plate_text, font_bold, font_label, digit_size, label_size, ink=ink,
  )
  return view


def draw_tici_speed_sign(rect: rl.Rectangle, font_bold, font_label) -> LiveSignView:
  return _draw_live_sign(tici_sign_rect, rect, font_bold, font_label, TICI_DIGIT, TICI_LABEL)


def draw_mici_speed_sign(rect: rl.Rectangle, font_bold, font_label) -> LiveSignView:
  return _draw_live_sign(mici_sign_rect, rect, font_bold, font_label, MICI_DIGIT, MICI_LABEL)


def _safe_confirm(mph: int, yes: bool) -> None:
  try:
    on_confirm_accuracy(mph, yes)
  except Exception:
    cloudlog.exception("speed_sign_hud: feedback failed")


class SpeedSignHud(Widget):
  """SIGN / NO WT plate plus stub Yes/No accuracy overlay."""

  def __init__(self, *, tici: bool):
    super().__init__()
    self._tici = tici
    self._view = LiveSignView()
    self._yes = (0.0, 0.0, 0.0, 0.0)
    self._no = (0.0, 0.0, 0.0, 0.0)
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_label: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)

  def render(self, rect: rl.Rectangle | None = None):
    # A HUD error must never break the onroad frame or its touch handling.
    try:
      return super().render(rect)
    except Exception:
      if not getattr(self, "_logged_err", False):
        cloudlog.exception("speed_sign_hud: render failed; HUD hidden this frame")
        self._logged_err = True
      self._view = LiveSignView()
      return None

  def owns_touch(self, x: float, y: float) -> bool:
    """Press is on a visible Yes/No button (onroad then skips the sidebar toggle)."""
    try:
      return point_in_confirm_buttons(x, y, self._view, self._yes, self._no)
    except Exception:
      return False

  def _update_state(self) -> None:
    self._view = _GATE.filter(live_sign_from_sm(ui_state.sm, logger_enabled()), _now())

  def _layout(self) -> None:
    sign = tici_sign_rect(self._rect) if self._tici else mici_sign_rect(self._rect)
    if self._tici:
      self._yes, self._no = tici_confirm_rects(sign)
    else:
      self._yes, self._no = mici_confirm_rects(sign)

  @property
  def _hit_rect(self) -> rl.Rectangle:
    # Only Yes/No consume touches — the plate itself does not block onroad UI.
    if not hud_confirm_visible(self._view):
      return rl.Rectangle(0, 0, 0, 0)
    yx, yy, yw, yh = self._yes
    nx, ny, nw, nh = self._no
    x = min(yx, nx)
    y = min(yy, ny)
    return rl.Rectangle(x, y, max(yx + yw, nx + nw) - x, max(yy + yh, ny + nh) - y)

  def _handle_mouse_press(self, mouse_pos: MousePos) -> None:
    # Answer on press, not release: the press is inside a drawn button right
    # now, so a later layout shift (sidebar, plate expiring) can't drop it,
    # and is_pressed is already set when onroad checks user_interacting().
    self._answer_at(mouse_pos)

  def _handle_mouse_release(self, mouse_pos: MousePos) -> None:
    pass

  def _answer_at(self, mouse_pos: MousePos) -> None:
    if not hud_confirm_visible(self._view):
      return
    hit = hit_accuracy_button(mouse_pos.x, mouse_pos.y, self._yes, self._no)
    if hit is None:
      return
    # One answer per plate: ignore repeat taps on the same mph for 3 s.
    import time as _time
    now = _time.monotonic()
    last = getattr(self, "_last_answer", (None, 0.0))
    if last[0] == int(self._view.mph) and now - last[1] < 3.0:
      return
    self._last_answer = (int(self._view.mph), now)
    mph = int(self._view.mph)
    # Hide the popup (question + prompt) this frame for Yes and No.
    _GATE.answer(mph, hit, now)
    self._view = _GATE.filter(self._view, now)
    # Disk / params work off the UI thread so a slow write can never stall frames.
    import threading
    threading.Thread(target=_safe_confirm, args=(mph, hit), daemon=True, name="speedsign-tap").start()

  def _render(self, rect: rl.Rectangle) -> None:
    if not self._view.show:
      return
    if self._tici:
      sign = tici_sign_rect(rect)
      view = draw_tici_speed_sign(rect, self._font_bold, self._font_label)
    else:
      sign = mici_sign_rect(rect)
      view = draw_mici_speed_sign(rect, self._font_bold, self._font_label)
    if not hud_confirm_visible(view):
      return
    mouse = rl.get_mouse_position()
    pressed = self.is_pressed
    draw_accuracy_overlay(
      sign, self._yes, self._no, self._font_label, tici=self._tici,
      pressed_yes=pressed and hit_accuracy_button(mouse.x, mouse.y, self._yes, self._no) is True,
      pressed_no=pressed and hit_accuracy_button(mouse.x, mouse.y, self._yes, self._no) is False,
    )
