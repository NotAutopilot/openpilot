"""Bird's-eye Bosch radar plot, driving HUD, and in-session monitor dialog.

Tinkla C2's calibrate/test tools subscribed to cereal `can` and left
pandad running. NAP's script runner killed the comma session instead.
This dialog is a push_widget overlay: open while driving, close without
reboot, no NAPScriptRunning.
"""

from __future__ import annotations

from dataclasses import replace

import pyray as rl

from openpilot.selfdrive.ui.radar.bosch_status import (
  ALIGN_D_MAX,
  ALIGN_D_MIN,
  ALIGN_Y_MAX,
  BoschRadarStatus,
  RadarTrack,
)
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.button import Button, ButtonStyle

RANGE_M = 120.0
LATERAL_M = 8.0

HEALTH_COLOR = {
  "LIVE": rl.Color(70, 200, 120, 255),
  "NO TRACKS": rl.Color(180, 180, 180, 255),
  "WAIT VIN": rl.Color(230, 170, 50, 255),
  "WAIT GTW": rl.Color(230, 170, 50, 255),
  "SGU": rl.Color(230, 170, 50, 255),
  "REJECT": rl.Color(230, 120, 40, 255),
  "UNAVAIL": rl.Color(230, 170, 50, 255),
  "CAN": rl.Color(226, 44, 44, 255),
  "FROZEN": rl.Color(226, 44, 44, 255),
  "FAULT": rl.Color(226, 44, 44, 255),
}

PANEL_BG = rl.Color(12, 14, 18, 210)
PANEL_BORDER = rl.Color(255, 255, 255, 40)
ALIGN_FILL = rl.Color(70, 91, 234, 40)
ALIGN_LINE = rl.Color(70, 91, 234, 160)
TRACK_FILL = rl.Color(90, 200, 255, 230)
TRACK_COAST = rl.Color(180, 180, 180, 200)
LEAD_FILL = rl.Color(128, 216, 166, 255)
TEXT = rl.Color(230, 230, 230, 255)
MUTED = rl.Color(160, 160, 160, 255)


def health_color(label: str) -> rl.Color:
  return HEALTH_COLOR.get(label, MUTED)


def _draw_text(font, text: str, x: float, y: float, size: int, color: rl.Color):
  rl.draw_text_ex(font, text, rl.Vector2(x, y), size, 0, color)


def draw_radar_plot(rect: rl.Rectangle, status: BoschRadarStatus, align_box: bool = True) -> None:
  rl.draw_rectangle_rec(rect, rl.Color(8, 10, 14, 255))
  rl.draw_rectangle_lines_ex(rect, 2, PANEL_BORDER)

  if rect.width < 8 or rect.height < 8:
    return

  def to_px(d_rel: float, y_rel: float) -> tuple[float, float]:
    x = rect.x + rect.width * (0.5 - y_rel / (2 * LATERAL_M))
    y = rect.y + rect.height * (1.0 - d_rel / RANGE_M)
    return x, y

  if align_box:
    left, bottom = to_px(ALIGN_D_MIN, ALIGN_Y_MAX)
    right, top = to_px(ALIGN_D_MAX, -ALIGN_Y_MAX)
    box = rl.Rectangle(min(left, right), min(top, bottom), abs(right - left), abs(bottom - top))
    rl.draw_rectangle_rec(box, ALIGN_FILL)
    rl.draw_rectangle_lines_ex(box, 2, ALIGN_LINE)

  car_x, car_y = to_px(0.0, 0.0)
  rl.draw_rectangle(int(car_x - 6), int(car_y - 10), 12, 16, rl.Color(220, 220, 220, 255))

  closest = None
  for track in status.tracks:
    if closest is None or track.d_rel < closest.d_rel:
      closest = track

  for track in status.tracks:
    x, y = to_px(max(0.0, track.d_rel), track.y_rel)
    color = LEAD_FILL if track is closest else (TRACK_FILL if track.measured else TRACK_COAST)
    radius = 7 if track is closest else 5
    rl.draw_circle(int(x), int(y), radius, color)

  font = gui_app.font(FontWeight.NORMAL)
  _draw_text(font, "0m", rect.x + 8, rect.y + rect.height - 28, 18, MUTED)
  _draw_text(font, f"{int(RANGE_M)}m", rect.x + 8, rect.y + 6, 18, MUTED)


def _status_lines(status: BoschRadarStatus) -> list[str]:
  awd = "—" if status.awd is None else ("4WD" if status.awd else "2WD")
  pos = "—" if status.position is None else str(status.position)
  epas = "—" if status.epas_type is None else str(status.epas_type)
  lamps = []
  if status.hw_fail:
    lamps.append("HWFail")
  if status.sgu_fail:
    lamps.append("SGUFail")
  if status.dirty:
    lamps.append("dirty")
  if status.table_frozen:
    lamps.append("frozen")
  if status.radar_fault:
    lamps.append("radarFault")
  if status.can_error:
    lamps.append("canError")
  lamp_text = " ".join(lamps) if lamps else "none"
  alerts = ", ".join(status.alerts) if status.alerts else "none"
  vin = status.vin if len(status.vin) == 17 else (status.vin or "—")
  lines = [
    f"{status.health_label}   {len(status.tracks)} tracks   raw {status.unique_raw}",
    f"lamps  {lamp_text}",
  ]
  if len(alerts) <= 36:
    lines.append(f"alerts {alerts}")
  else:
    lines.append("alerts")
    chunk = ""
    for name in status.alerts:
      nxt = name if not chunk else chunk + ", " + name
      if len(nxt) > 36:
        if chunk:
          lines.append("  " + chunk)
        chunk = name
      else:
        chunk = nxt
    if chunk:
      lines.append("  " + chunk)
  lines.append(f"GTW    {awd}  pos {pos}  EPAS {epas}")
  lines.append(f"VIN    {vin}")
  return lines


def draw_status_text(rect: rl.Rectangle, status: BoschRadarStatus, font_size: int = 28) -> None:
  font = gui_app.font(FontWeight.MEDIUM)
  y = rect.y
  label = status.health_label
  _draw_text(gui_app.font(FontWeight.BOLD), label, rect.x, y, font_size + 10, health_color(label))
  y += font_size + 18
  for line in _status_lines(status)[1:]:
    _draw_text(font, line, rect.x, y, font_size, TEXT)
    y += font_size + 8


def _track_row(track: RadarTrack) -> str:
  flag = "m" if track.measured else "c"
  return f"{track.track_id:>4}  {track.d_rel:6.1f}  {track.y_rel:+6.2f}  {track.v_rel:+6.2f}  {flag}"


class RadarHudOverlay(Widget):
  """Compact on-road readout. Hidden unless NAPRadarHud is set."""

  def _render(self, rect: rl.Rectangle):
    if not ui_state.radar_hud:
      return
    status = ui_state.radar_status
    panel = rect
    rl.draw_rectangle_rounded(panel, 0.08, 8, PANEL_BG)
    rl.draw_rectangle_rounded_lines_ex(panel, 0.08, 8, 2, health_color(status.health_label))

    pad = 8 if panel.height < 180 else 12
    plot_w = min(150, panel.width * 0.38)
    plot = rl.Rectangle(panel.x + pad, panel.y + pad, plot_w, panel.height - pad * 2)
    draw_radar_plot(plot, status, align_box=False)
    text = rl.Rectangle(plot.x + plot.width + 10, panel.y + pad, panel.width - plot.width - pad * 3, panel.height - pad * 2)
    draw_status_text(text, status, font_size=18 if panel.height < 180 else 20)


class RadarMonitorDialog(Widget):
  """Full-screen live radar. Does not stop pandad or set NAPScriptRunning."""

  def __init__(self):
    super().__init__()
    self._close = Button("Close", click_callback=self._on_close, button_style=ButtonStyle.TRANSPARENT_WHITE_BORDER)
    self._align_only = False
    self._align_btn = Button("Align", click_callback=self._toggle_align, button_style=ButtonStyle.ACTION)

  def _on_close(self):
    gui_app.pop_widget()

  def _toggle_align(self):
    self._align_only = not self._align_only

  def _render(self, rect: rl.Rectangle):
    status = ui_state.radar_status
    if self._align_only:
      tracks = tuple(
        t for t in status.tracks
        if ALIGN_D_MIN <= t.d_rel <= ALIGN_D_MAX and abs(t.y_rel) <= ALIGN_Y_MAX
      )
      status = replace(status, tracks=tracks)

    compact = rect.height < 500
    pad = 12 if compact else 30
    title_size = 28 if compact else 52
    body = 18 if compact else 26
    btn_h = 48 if compact else 90
    btn_w = 140 if compact else 280
    header_h = 44 if compact else 110
    footer_h = btn_h + (16 if compact else 40)

    rl.draw_rectangle_rec(rect, rl.Color(0, 0, 0, 230))
    _draw_text(gui_app.font(FontWeight.BOLD), "Live radar", rect.x + pad, rect.y + (8 if compact else 20), title_size, TEXT)
    if not compact:
      _draw_text(gui_app.font(FontWeight.MEDIUM), "openpilot stays running", rect.x + pad, rect.y + 78, 24, MUTED)

    plot_w = min(rect.width * (0.40 if compact else 0.42), 720)
    plot = rl.Rectangle(rect.x + pad, rect.y + header_h, plot_w, rect.height - header_h - footer_h)
    draw_radar_plot(plot, status, align_box=True)

    info_x = plot.x + plot.width + (12 if compact else 30)
    info_w = rect.x + rect.width - info_x - pad
    font_size = 18 if compact else 32
    draw_status_text(rl.Rectangle(info_x, plot.y, info_w, 160 if compact else 220), status, font_size=font_size)

    font = gui_app.font(FontWeight.MEDIUM)
    y = plot.y + (190 if compact else 380)
    if y + 40 < rect.y + rect.height - footer_h:
      _draw_text(font, "  ID     dRel     yRel     vRel", info_x, y, body, MUTED)
      y += body + 8
      rows = sorted(status.tracks, key=lambda t: t.d_rel)[: 4 if compact else 12]
      for track in rows:
        if y + body > rect.y + rect.height - footer_h:
          break
        _draw_text(font, _track_row(track), info_x, y, body, TEXT)
        y += body + 6
      if not rows:
        _draw_text(font, "no published tracks", info_x, y, body, MUTED)

    btn_y = rect.y + rect.height - footer_h + (8 if compact else 20)
    align_w = btn_w if compact else 220
    self._close.render(rl.Rectangle(rect.x + pad, btn_y, btn_w, btn_h))
    self._align_btn.render(rl.Rectangle(rect.x + pad + btn_w + 12, btn_y, align_w, btn_h))
    mode = "align 2.5–14.5m" if self._align_only else "all tracks"
    _draw_text(font, mode, rect.x + pad + btn_w + align_w + 28, btn_y + btn_h * 0.28, body, MUTED)
