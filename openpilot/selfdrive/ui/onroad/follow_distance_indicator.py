"""Follow-distance overlay: car outline, capped gap, car outline, integer."""
from __future__ import annotations

import time

import pyray as rl

from openpilot.selfdrive.ui.radar.radar_view import radar_hud_rect
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached

FOLLOW_DISTANCE_MIN = 1
FOLLOW_DISTANCE_MAX = 7
OVERLAY_SECONDS = 2.0

_WHITE = rl.Color(255, 255, 255, 235)
_PILL = rl.Color(0, 0, 0, 175)
_PILL_EDGE = rl.Color(255, 255, 255, 36)

# Side-profile sedan, nose right, in unit square (x right, y down).
_CAR_BODY = (
  (0.06, 0.70),
  (0.10, 0.46),
  (0.22, 0.44),
  (0.32, 0.22),
  (0.64, 0.20),
  (0.78, 0.42),
  (0.94, 0.44),
  (0.97, 0.70),
)
_WHEELS = ((0.28, 0.70), (0.80, 0.70))

# Picker taps never appear on STW DTR. Hold until the physical wheel actually moves.
_pending_tap: int | None = None
_wheel_at_tap: int = 0
_force_show: bool = False
_last_wheel: int = 0
_selection_started_frame: int | None = None


def _sync_selection_epoch() -> None:
  global _selection_started_frame
  if ui_state.started and _selection_started_frame != ui_state.started_frame:
    reset_follow_distance_tap()
    _selection_started_frame = ui_state.started_frame


def live_stalk_follow_distance(sm) -> int:
  if not (sm.alive.get("carState") and sm.valid.get("carState")):
    return 0
  value = int(sm["carState"].napStalkFollowDistance)
  if FOLLOW_DISTANCE_MIN <= value <= FOLLOW_DISTANCE_MAX:
    return value
  return 0


def note_follow_distance_tap(distance: int, wheel: int = 0) -> None:
  """Flash the overlay for a GUI tap until a new physical detent arrives."""
  global _pending_tap, _wheel_at_tap, _force_show
  value = int(distance)
  if not (FOLLOW_DISTANCE_MIN <= value <= FOLLOW_DISTANCE_MAX):
    return
  _sync_selection_epoch()
  _pending_tap = value
  _wheel_at_tap = int(wheel) if FOLLOW_DISTANCE_MIN <= int(wheel) <= FOLLOW_DISTANCE_MAX else 0
  _force_show = True


def reset_follow_distance_tap() -> None:
  global _pending_tap, _wheel_at_tap, _force_show, _last_wheel, _selection_started_frame
  _pending_tap = None
  _wheel_at_tap = 0
  _force_show = False
  _last_wheel = 0
  _selection_started_frame = None


def pop_forced_follow_distance_show() -> bool:
  global _force_show
  forced, _force_show = _force_show, False
  return forced


def selected_follow_distance(wheel: int) -> int:
  """New 1..7 detent wins; same-wheel samples leave a pending GUI tap in place."""
  global _pending_tap, _wheel_at_tap, _last_wheel
  _sync_selection_epoch()
  wheel_value = int(wheel)
  wheel_live = FOLLOW_DISTANCE_MIN <= wheel_value <= FOLLOW_DISTANCE_MAX
  if wheel_live:
    _last_wheel = wheel_value
  if _pending_tap is not None:
    detent_changed = wheel_live and (_wheel_at_tap == 0 or wheel_value != _wheel_at_tap)
    if not detent_changed:
      return _pending_tap
    _pending_tap = None
    _wheel_at_tap = 0
  return _last_wheel




def follow_distance_overlay_rect(content: rl.Rectangle) -> rl.Rectangle:
  compact = content.width < 800
  width = 168.0 if compact else 360.0
  height = 40.0 if compact else 88.0
  width = min(width, max(0.0, content.width * 0.52))
  height = min(height, max(0.0, content.height * 0.26))
  bottom = content.y + content.height
  if getattr(ui_state, "radar_hud", False):
    bottom = min(bottom, radar_hud_rect(content).y)
  margin = 6.0 if compact else 18.0
  x = content.x + (content.width - width) * 0.5
  y = bottom - height - margin
  if y < content.y + margin:
    y = content.y + margin
    height = max(0.0, min(height, bottom - margin - y))
  return rl.Rectangle(x, y, width, height)


def _draw_car_outline(x: float, y: float, width: float, height: float, color: rl.Color, thickness: float) -> None:
  pts = [rl.Vector2(x + px * width, y + py * height) for px, py in _CAR_BODY]
  if len(pts) >= 2:
    for i in range(len(pts) - 1):
      rl.draw_line_ex(pts[i], pts[i + 1], thickness, color)
    rl.draw_line_ex(pts[-1], pts[0], thickness, color)
  wheel_r = height * 0.16
  for wx, wy in _WHEELS:
    rl.draw_circle_lines(int(x + wx * width), int(y + wy * height), wheel_r, color)


def draw_follow_distance_indicator(rect: rl.Rectangle, distance: int) -> None:
  """Production drawing: two right-facing car outlines, capped gap, integer."""
  if rect.width < 8 or rect.height < 8:
    return
  if not (FOLLOW_DISTANCE_MIN <= int(distance) <= FOLLOW_DISTANCE_MAX):
    return

  compact = rect.height < 56
  roundness = 0.5 if compact else 0.4
  rl.draw_rectangle_rounded(rect, roundness, 8, _PILL)
  rl.draw_rectangle_rounded_lines_ex(rect, roundness, 8, 1.5 if compact else 2.0, _PILL_EDGE)

  pad = rect.height * 0.16
  inner_h = max(8.0, rect.height - pad * 2)
  car_h = inner_h
  car_w = car_h * 1.55
  font_size = int(inner_h * (0.72 if compact else 0.64))
  font = gui_app.font(FontWeight.BOLD)
  number = str(int(distance))
  number_w = measure_text_cached(font, number, font_size).x
  gap_room = rect.width - pad * 2 - car_w * 2 - number_w - pad
  if gap_room < inner_h * 0.2:
    scale = max(0.35, (rect.width - pad * 3 - number_w) / max(car_w * 2 + inner_h * 0.2, 1.0))
    car_w *= scale
    car_h *= scale
    gap_room = rect.width - pad * 2 - car_w * 2 - number_w - pad
  t = (int(distance) - FOLLOW_DISTANCE_MIN) / float(FOLLOW_DISTANCE_MAX - FOLLOW_DISTANCE_MIN)
  gap_w = max(inner_h * 0.18, gap_room * (0.20 + 0.72 * t))
  gap_w = min(gap_w, max(inner_h * 0.18, gap_room))

  cy = rect.y + (rect.height - car_h) * 0.5
  x = rect.x + pad
  stroke = 1.6 if compact else 2.4
  _draw_car_outline(x, cy, car_w, car_h, _WHITE, stroke)
  gap_x = x + car_w
  gap_y = cy + car_h * 0.52
  cap = car_h * 0.22
  rl.draw_line_ex(rl.Vector2(gap_x, gap_y - cap), rl.Vector2(gap_x, gap_y + cap), stroke, _WHITE)
  rl.draw_line_ex(rl.Vector2(gap_x, gap_y), rl.Vector2(gap_x + gap_w, gap_y), stroke, _WHITE)
  rl.draw_line_ex(rl.Vector2(gap_x + gap_w, gap_y - cap), rl.Vector2(gap_x + gap_w, gap_y + cap), stroke, _WHITE)
  _draw_car_outline(gap_x + gap_w, cy, car_w, car_h, _WHITE, stroke)
  num_x = gap_x + gap_w + car_w + pad * 0.4
  num_y = rect.y + (rect.height - font_size) * 0.5
  rl.draw_text_ex(font, number, rl.Vector2(num_x, num_y), font_size, 0, _WHITE)


class FollowDistanceIndicator:
  def __init__(self):
    self._started_frame: int | None = None
    self._reset()

  def _reset(self) -> None:
    self._have_sample = False
    self._last = 0
    self._hide_at = 0.0

  def reset_offroad(self) -> None:
    self._reset()
    reset_follow_distance_tap()

  def cancel_for_alert(self) -> None:
    self._hide_at = 0.0

  def note_selection(self, now: float, distance: int, *, onroad: bool, alert: bool) -> int:
    """Track selected 1..7. Returns the integer to draw, or 0 if hidden."""
    if not onroad:
      self.reset_offroad()
      return 0
    if alert:
      self.cancel_for_alert()
      if FOLLOW_DISTANCE_MIN <= distance <= FOLLOW_DISTANCE_MAX:
        self._have_sample = True
        self._last = distance
      return 0
    if not (FOLLOW_DISTANCE_MIN <= distance <= FOLLOW_DISTANCE_MAX):
      return self._last if now < self._hide_at else 0
    if not self._have_sample:
      self._have_sample = True
      self._last = distance
      return 0
    if distance != self._last:
      self._last = distance
      self._hide_at = now + OVERLAY_SECONDS
    return self._last if now < self._hide_at else 0

  def render(self, rect: rl.Rectangle, *, alert: bool) -> None:
    if not ui_state.started:
      self.reset_offroad()
      return
    if ui_state.CP is None or ui_state.CP.carFingerprint != "TESLA_MODEL_S_PREAP":
      self._reset()
      return
    if self._started_frame != ui_state.started_frame:
      self._reset()
      self._started_frame = ui_state.started_frame

    selected = selected_follow_distance(live_stalk_follow_distance(ui_state.sm))
    if pop_forced_follow_distance_show():
      self._have_sample = True
      self._last = 0
    distance = self.note_selection(time.monotonic(), selected, onroad=True, alert=alert)
    if distance:
      draw_follow_distance_indicator(follow_distance_overlay_rect(rect), distance)
