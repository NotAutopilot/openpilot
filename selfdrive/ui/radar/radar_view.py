"""Bird's-eye Bosch radar plot, driving HUD, and in-session monitor dialog.

Tinkla C2's calibrate/test tools subscribed to cereal `can` and left
pandad running. NAP's script runner killed the comma session instead.
This dialog is a push_widget overlay: open while driving, close without
reboot, no NAPScriptRunning.

C4 (536x240) is a cut-down status strip: health plus one or two reject/table
bits. The full key-on chip set lives on hidpi / the 2160x1080 modal.
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
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.button import Button, ButtonStyle

RANGE_M = 120.0
LATERAL_M = 8.0

# C4 on-road strip. Must fit inside 536x240 (content ~476x180 after 30px border).
C4_HUD_W = 460.0
C4_HUD_H = 44.0
# Hidpi on-road panel: plot + full key-on chips.
HIDPI_HUD_W = 900.0
HIDPI_HUD_H = 340.0

KEY_REJECT = ("vinValidity", "xwdValidity", "radPositionMismatch", "strRackMismatch")

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

PANEL_BG = rl.Color(12, 14, 18, 230)
PANEL_BORDER = rl.Color(255, 255, 255, 70)
ALIGN_FILL = rl.Color(70, 91, 234, 40)
ALIGN_LINE = rl.Color(70, 91, 234, 160)
TRACK_FILL = rl.Color(90, 200, 255, 230)
TRACK_COAST = rl.Color(180, 180, 180, 200)
LEAD_FILL = rl.Color(128, 216, 166, 255)
TEXT = rl.Color(230, 230, 230, 255)
MUTED = rl.Color(170, 170, 170, 255)
CHIP_MUTED = rl.Color(128, 128, 128, 255)
CHIP_FILL = rl.Color(57, 57, 57, 255)
CHIP_ON_FILL = rl.Color(70, 40, 20, 230)
OK_FILL = rl.Color(20, 50, 32, 230)


def health_color(label: str) -> rl.Color:
  return HEALTH_COLOR.get(label, MUTED)


def _draw_text(font, text: str, x: float, y: float, size: int, color: rl.Color) -> None:
  rl.draw_text_ex(font, text, rl.Vector2(x, y), size, 0, color)


def _line_h(font, text: str, size: int, gap: int = 4) -> float:
  return measure_text_cached(font, text or "Ag", size).y + gap


def is_c4_screen(rect: rl.Rectangle) -> bool:
  """True for the 536x240 C4 window / content, not the hidpi 900x340 HUD card."""
  return rect.width < 800 and rect.height < 500


def radar_hud_rect(content: rl.Rectangle) -> rl.Rectangle:
  """On-road HUD that always fits inside `content` with zero clipping.

  C4 536x240 / content ~476x180: 460x44 status strip.
  Hidpi 2160x1080 / content ~2100x1020: 900x340 chip panel.
  """
  if is_c4_screen(content):
    margin = 8.0
    w = min(C4_HUD_W, max(0.0, content.width - margin * 2))
    h = min(C4_HUD_H, max(0.0, content.height - margin))
    return rl.Rectangle(content.x + margin, content.y + content.height - h - margin, w, h)
  margin_x, margin_y = 24.0, 20.0
  w = min(HIDPI_HUD_W, max(0.0, content.width - margin_x * 2))
  h = min(HIDPI_HUD_H, max(0.0, content.height - 24.0))
  return rl.Rectangle(content.x + margin_x, content.y + content.height - h - margin_y, w, h)


def _clamp_to_window(rect: rl.Rectangle) -> rl.Rectangle:
  w_max = float(max(1, gui_app.width))
  h_max = float(max(1, gui_app.height))
  x = min(max(0.0, rect.x), w_max)
  y = min(max(0.0, rect.y), h_max)
  w = min(max(0.0, rect.width), w_max - x)
  h = min(max(0.0, rect.height), h_max - y)
  return rl.Rectangle(x, y, w, h)


def _chip_pad(font_size: int) -> tuple[int, int]:
  if font_size < 16:
    return 6, 2
  if font_size < 22:
    return 8, 3
  return 14, 6


def chip_size(text: str, font_size: int) -> tuple[float, float]:
  font = gui_app.font(FontWeight.BOLD)
  sz = measure_text_cached(font, text, font_size)
  pad_x, pad_y = _chip_pad(font_size)
  return sz.x + pad_x * 2, sz.y + pad_y * 2


def draw_chip(x: float, y: float, text: str, color: rl.Color, font_size: int,
              fill: rl.Color | None = None) -> tuple[float, float]:
  font = gui_app.font(FontWeight.BOLD)
  sz = measure_text_cached(font, text, font_size)
  pad_x, pad_y = _chip_pad(font_size)
  w = sz.x + pad_x * 2
  h = sz.y + pad_y * 2
  rec = rl.Rectangle(x, y, w, h)
  bg = fill if fill is not None else rl.Color(color.r, color.g, color.b, 48)
  rl.draw_rectangle_rounded(rec, 0.5, 8, bg)
  rl.draw_rectangle_rounded_lines_ex(rec, 0.5, 8, 1.5 if font_size < 20 else 2, color)
  _draw_text(font, text, x + pad_x, y + pad_y, font_size, color)
  return w, h


def table_bit(status: BoschRadarStatus) -> str | None:
  """Moving vs frozen from the 0x310 unique window (not liveTracks age)."""
  if status.table_frozen or status.unique_raw == 1:
    return "frozen"
  if status.unique_raw >= 2:
    return "moving"
  return None


def c4_extra_chips(status: BoschRadarStatus) -> list[tuple[str, rl.Color]]:
  """The one or two bits that matter on the C4 strip. No 0x501 dump, no VIN."""
  extras: list[tuple[str, rl.Color]] = []
  reason = next((name for name in KEY_REJECT if name in status.alerts), None)
  if reason is None and status.health_label == "FAULT":
    if status.hw_fail:
      reason = "HWFail"
    elif status.radar_fault:
      reason = "radarFault"
  if reason is None and status.health_label in ("WAIT VIN", "WAIT GTW"):
    reason = "waiting"
  if reason:
    extras.append((reason, health_color("REJECT" if reason in KEY_REJECT else status.health_label)))
  bit = table_bit(status)
  if bit:
    extras.append((bit, health_color("FROZEN" if bit == "frozen" else "LIVE")))
  return extras[:2]


def _extract_vin(raw) -> str:
  if raw is None:
    return ""
  if isinstance(raw, (bytes, bytearray)):
    raw = raw.decode("utf-8", "ignore")
  token = "".join(ch for ch in str(raw).upper() if ch.isalnum())
  if len(token) == 17:
    return token
  for i in range(0, max(0, len(token) - 16)):
    cand = token[i:i + 17]
    if len(cand) == 17:
      return cand
  return ""


def vin_refs(status: BoschRadarStatus) -> tuple[str, str, str]:
  """0x2B9 feed vs F190 (radar EEPROM) vs chassis CarVin."""
  feed = status.vin if len(status.vin) == 17 else (status.vin or "")
  f190 = _extract_vin(getattr(status, "vin_f190", "") or "")
  chassis = _extract_vin(getattr(status, "vin_chassis", "") or "")
  if not f190 or not chassis:
    try:
      from openpilot.common.params import Params
      p = Params()
      if not chassis:
        chassis = _extract_vin(p.get("CarVin"))
      if not f190:
        raw = ""
        try:
          from opendbc.car.tesla.preap.nap_params import NAPParamKeys
          raw = p.get(NAPParamKeys.RADAR_VIN_READ_STATUS, return_default=True) or ""
        except Exception:
          raw = p.get("NAPRadarVinReadStatus", return_default=True) or ""
        f190 = _extract_vin(raw)
    except Exception:
      pass
  return feed, f190, chassis


def link_label(status: BoschRadarStatus) -> str:
  return "talking" if status.gtw_live else "waiting"


def draw_radar_plot(rect: rl.Rectangle, status: BoschRadarStatus, align_box: bool = True,
                    compact: bool = False) -> None:
  rl.draw_rectangle_rec(rect, rl.Color(8, 10, 14, 255))
  rl.draw_rectangle_lines_ex(rect, 1 if compact else 2, PANEL_BORDER)
  if rect.width < 8 or rect.height < 8:
    return

  def to_px(d_rel: float, y_rel: float) -> tuple[float, float]:
    x = rect.x + rect.width * (0.5 - y_rel / (2 * LATERAL_M))
    y = rect.y + rect.height * (1.0 - d_rel / RANGE_M)
    return x, y

  rl.begin_scissor_mode(int(rect.x) + 1, int(rect.y) + 1, max(1, int(rect.width) - 2), max(1, int(rect.height) - 2))

  if align_box:
    left, bottom = to_px(ALIGN_D_MIN, ALIGN_Y_MAX)
    right, top = to_px(ALIGN_D_MAX, -ALIGN_Y_MAX)
    box = rl.Rectangle(min(left, right), min(top, bottom), abs(right - left), abs(bottom - top))
    rl.draw_rectangle_rec(box, ALIGN_FILL)
    rl.draw_rectangle_lines_ex(box, 1 if compact else 2, ALIGN_LINE)

  car_x, car_y = to_px(0.0, 0.0)
  cw, ch = (5, 8) if compact else (12, 16)
  rl.draw_rectangle(int(car_x - cw / 2), int(car_y - ch + 2), cw, ch, rl.Color(220, 220, 220, 255))

  closest = None
  for track in status.tracks:
    if closest is None or track.d_rel < closest.d_rel:
      closest = track

  for track in status.tracks:
    x, y = to_px(max(0.0, min(RANGE_M, track.d_rel)), track.y_rel)
    color = LEAD_FILL if track is closest else (TRACK_FILL if track.measured else TRACK_COAST)
    radius = (4 if track is closest else 3) if compact else (7 if track is closest else 5)
    rl.draw_circle(int(x), int(y), radius, color)

  rl.end_scissor_mode()

  font = gui_app.font(FontWeight.NORMAL)
  tick = 11 if compact else 18
  _draw_text(font, "0m", rect.x + 4, rect.y + rect.height - (14 if compact else 28), tick, MUTED)
  _draw_text(font, f"{int(RANGE_M)}m", rect.x + 4, rect.y + 3, tick, MUTED)


def draw_chip_row(x: float, y: float, max_w: float, chips: list[tuple[str, rl.Color, rl.Color | None]],
                  font_size: int, gap: float = 6) -> tuple[float, float]:
  cx, cy = x, y
  row_h = 0.0
  used_w = 0.0
  bottom = y
  for text, color, fill in chips:
    tw, th = chip_size(text, font_size)
    if cx + tw > x + max_w and cx > x:
      cx = x
      cy += row_h + 4
      row_h = 0.0
    if tw > max_w:
      continue
    draw_chip(cx, cy, text, color, font_size, fill=fill)
    cx += tw + gap
    row_h = max(row_h, th)
    used_w = max(used_w, cx - x)
    bottom = cy + row_h
  return used_w, max(0.0, bottom - y)


def _track_row(track: RadarTrack) -> str:
  flag = "m" if track.measured else "c"
  return f"{track.track_id:>4}  {track.d_rel:6.1f}  {track.y_rel:+6.2f}  {track.v_rel:+6.2f}  {flag}"


def _c4_chip_list(status: BoschRadarStatus) -> list[tuple[str, rl.Color, rl.Color | None]]:
  chips: list[tuple[str, rl.Color, rl.Color | None]] = [
    (status.health_label, health_color(status.health_label), None),
  ]
  for text, color in c4_extra_chips(status):
    fill = CHIP_FILL if text in ("moving", "frozen", "waiting") else CHIP_ON_FILL
    chips.append((text, color, fill))
  return chips


def draw_c4_strip(panel: rl.Rectangle, status: BoschRadarStatus) -> None:
  """Cut-down C4 HUD: state + at most two reject/table bits. No VIN, no 0x2A9."""
  rl.draw_rectangle_rounded(panel, 0.18, 6, PANEL_BG)
  rl.draw_rectangle_rounded_lines_ex(panel, 0.18, 6, 2, health_color(status.health_label))
  font_size = 13
  chips = _c4_chip_list(status)
  max_h = 0.0
  for text, _, _ in chips:
    _w, th = chip_size(text, font_size)
    max_h = max(max_h, th)
  pad = 6.0
  y = panel.y + max(pad, (panel.height - max_h) / 2)
  x = panel.x + pad
  draw_chip_row(x, y, panel.width - pad * 2, chips, font_size, gap=6)


def draw_hidpi_chips(rect: rl.Rectangle, status: BoschRadarStatus, font_size: int) -> float:
  """Full key-on chip set. Returns y after the last row."""
  if rect.width < 8 or rect.height < 8:
    return rect.y
  font = gui_app.font(FontWeight.MEDIUM)
  label_size = max(14, font_size - 4)
  x, y = rect.x, rect.y
  max_w = rect.width
  bottom = rect.y + rect.height
  gap = 8

  def row(label: str, chips: list[tuple[str, rl.Color, rl.Color | None]]) -> None:
    nonlocal y
    if y + 20 > bottom:
      return
    lw = 0.0
    if label:
      _draw_text(font, label, x, y + 4, label_size, MUTED)
      lw = measure_text_cached(font, label, label_size).x + 10
    _, h = draw_chip_row(x + lw, y, max_w - lw, chips, font_size, gap=6)
    y += max(h, _line_h(font, label or "Ag", label_size, 0)) + gap

  label = status.health_label
  row("", [
    (label, health_color(label), None),
    ("SGUFail", health_color("SGU") if status.sgu_fail else CHIP_MUTED,
     CHIP_ON_FILL if status.sgu_fail else CHIP_FILL),
    ("HWFail", health_color("FAULT") if status.hw_fail else CHIP_MUTED,
     CHIP_ON_FILL if status.hw_fail else CHIP_FILL),
    (link_label(status), health_color("LIVE") if status.gtw_live else health_color("WAIT VIN"),
     OK_FILL if status.gtw_live else CHIP_FILL),
  ])

  alert_chips: list[tuple[str, rl.Color, rl.Color | None]] = []
  for name in KEY_REJECT:
    on = name in status.alerts
    alert_chips.append((name, health_color("REJECT") if on else CHIP_MUTED, CHIP_ON_FILL if on else CHIP_FILL))
  for name in status.alerts:
    if name not in KEY_REJECT:
      alert_chips.append((name, health_color("SGU"), CHIP_ON_FILL))
  row("0x501", alert_chips)

  awd = "4WD" if status.awd else ("2WD" if status.awd is False else "drive —")
  pos = "—" if status.position is None else str(status.position)
  epas = "—" if status.epas_type is None else str(status.epas_type)
  row("0x2A9", [
    (awd, TEXT, CHIP_FILL),
    (f"pos {pos}", TEXT, CHIP_FILL),
    (f"EPAS {epas}", TEXT, CHIP_FILL),
  ])

  feed, f190, chassis = vin_refs(status)
  vins = [v for v in (feed, f190, chassis) if v]
  mismatch = len(set(vins)) > 1
  vin_chips: list[tuple[str, rl.Color, rl.Color | None]] = []
  if feed:
    bad = mismatch and bool(f190) and feed != f190
    vin_chips.append((f"0x2B9 {feed}", health_color("REJECT") if bad else TEXT, CHIP_ON_FILL if bad else CHIP_FILL))
  if f190:
    bad = mismatch and bool(chassis) and f190 != chassis
    vin_chips.append((f"F190 {f190}", health_color("REJECT") if bad else TEXT, CHIP_ON_FILL if bad else CHIP_FILL))
  if chassis:
    vin_chips.append((f"chassis {chassis}", health_color("REJECT") if mismatch else TEXT,
                      CHIP_ON_FILL if mismatch else CHIP_FILL))
  if vin_chips:
    row("VIN", vin_chips)

  age = "age —" if status.last_raw_age_s is None else f"age {status.last_raw_age_s:.2f}s"
  bit = table_bit(status)
  raw_chips: list[tuple[str, rl.Color, rl.Color | None]] = [
    (f"unique {status.unique_raw}", TEXT, CHIP_FILL),
    (age, TEXT, CHIP_FILL),
    (f"{len(status.tracks)} tracks", TEXT, CHIP_FILL),
  ]
  if bit:
    raw_chips.append((bit, health_color("FROZEN" if bit == "frozen" else "LIVE"),
                      CHIP_ON_FILL if bit == "frozen" else OK_FILL))
  row("0x310", raw_chips)
  return y


class RadarHudOverlay(Widget):
  """On-road readout. Hidden unless NAPRadarHud is set."""

  def _render(self, rect: rl.Rectangle):
    if not ui_state.radar_hud:
      return
    status = ui_state.radar_status
    panel = _clamp_to_window(rect)
    if panel.width < 8 or panel.height < 8:
      return
    # Strip vs card is about the HUD rect, not the window.
    if panel.height < 80:
      draw_c4_strip(panel, status)
      return

    rl.draw_rectangle_rounded(panel, 0.06, 8, PANEL_BG)
    rl.draw_rectangle_rounded_lines_ex(panel, 0.06, 8, 2, health_color(status.health_label))
    pad = 12
    plot_w = min(220.0, panel.width * 0.28)
    plot = rl.Rectangle(panel.x + pad, panel.y + pad, plot_w, panel.height - pad * 2)
    draw_radar_plot(plot, status, align_box=False, compact=False)
    chips = rl.Rectangle(plot.x + plot.width + 14, panel.y + pad,
                         panel.width - plot.width - pad * 3, panel.height - pad * 2)
    draw_hidpi_chips(chips, status, font_size=18)


class RadarMonitorDialog(Widget):
  """Full-screen live radar. Does not stop pandad or set NAPScriptRunning."""

  def __init__(self):
    super().__init__()
    compact = not gui_app.big_ui()
    font = 18 if compact else 48
    pad = 4 if compact else 20
    self._close = Button("Close", click_callback=self._on_close, font_size=font,
                         text_padding=pad, button_style=ButtonStyle.TRANSPARENT_WHITE_BORDER)
    self._align_only = False
    self._align_btn = Button("Align", click_callback=self._toggle_align, font_size=font,
                             text_padding=pad, button_style=ButtonStyle.ACTION)

  def _on_close(self):
    gui_app.pop_widget()

  def _toggle_align(self):
    self._align_only = not self._align_only

  def _tune_buttons(self, compact: bool) -> None:
    font = 18 if compact else 48
    pad = 4 if compact else 20
    radius = 8 if compact else 10
    for btn in (self._close, self._align_btn):
      btn._label.set_font_size(font)
      btn._label._text_padding = pad
      btn._border_radius = radius
    self._align_btn.set_button_style(ButtonStyle.PRIMARY if self._align_only else ButtonStyle.ACTION)

  def _render(self, rect: rl.Rectangle):
    status = ui_state.radar_status
    if self._align_only:
      tracks = tuple(
        t for t in status.tracks
        if ALIGN_D_MIN <= t.d_rel <= ALIGN_D_MAX and abs(t.y_rel) <= ALIGN_Y_MAX
      )
      status = replace(status, tracks=tracks)

    compact = is_c4_screen(rect)
    self._tune_buttons(compact)
    rl.draw_rectangle_rec(rect, rl.Color(0, 0, 0, 255))
    if compact:
      self._render_c4(rect, status)
    else:
      self._render_hidpi(rect, status)

  def _render_c4(self, rect: rl.Rectangle, status: BoschRadarStatus) -> None:
    """Cut-down 536x240 modal: title, state+bits, plot, Close. No Align, no VIN dump."""
    pad = 8
    title_size = 18
    btn_w, btn_h = 100, 34
    footer_h = btn_h + 10
    title_font = gui_app.font(FontWeight.BOLD)
    _draw_text(title_font, "Live radar", rect.x + pad, rect.y + 6, title_size, TEXT)
    chip_y = rect.y + 30
    draw_chip_row(rect.x + pad, chip_y, rect.width - pad * 2, _c4_chip_list(status), 13, gap=6)
    plot = rl.Rectangle(rect.x + pad, rect.y + 64, rect.width - pad * 2, rect.height - 64 - footer_h)
    if plot.height > 24:
      draw_radar_plot(plot, status, align_box=False, compact=True)
    btn_y = rect.y + rect.height - footer_h + 4
    self._close.render(rl.Rectangle(rect.x + pad, btn_y, btn_w, btn_h))

  def _render_hidpi(self, rect: rl.Rectangle, status: BoschRadarStatus) -> None:
    pad = 28
    title_size = 44
    body = 24
    btn_h, btn_w, align_w = 90, 280, 220
    header_h = 96
    footer_h = btn_h + 36
    title_font = gui_app.font(FontWeight.BOLD)
    body_font = gui_app.font(FontWeight.MEDIUM)

    _draw_text(title_font, "Live radar", rect.x + pad, rect.y + 16, title_size, TEXT)
    chip_x = rect.x + pad + measure_text_cached(title_font, "Live radar", title_size).x + 18
    draw_chip(chip_x, rect.y + 22, status.health_label, health_color(status.health_label), 26)
    _draw_text(body_font, "openpilot stays running", rect.x + pad, rect.y + 66, 22, MUTED)

    plot_w = min(rect.width * 0.36, 720)
    plot = rl.Rectangle(rect.x + pad, rect.y + header_h, plot_w, rect.height - header_h - footer_h)
    draw_radar_plot(plot, status, align_box=True, compact=False)

    info_x = plot.x + plot.width + 28
    info_w = rect.x + rect.width - info_x - pad
    footer_y = rect.y + rect.height - footer_h
    chips_h = min(240.0, max(80.0, (footer_y - plot.y) * 0.55))
    y = draw_hidpi_chips(rl.Rectangle(info_x, plot.y, info_w, chips_h), status, font_size=20)

    table_y = y + 12
    table_bottom = footer_y - 8
    if table_y + body < table_bottom and info_w > 40:
      _draw_text(body_font, "  ID     dRel     yRel     vRel", info_x, table_y, body, MUTED)
      table_y += _line_h(body_font, "dRel", body, 6)
      rows = sorted(status.tracks, key=lambda t: t.d_rel)[:10]
      for track in rows:
        if table_y + body > table_bottom:
          break
        _draw_text(body_font, _track_row(track), info_x, table_y, body, TEXT)
        table_y += _line_h(body_font, _track_row(track), body, 4)
      if not rows:
        empty = "no tracks in 2.5–14.5 m" if self._align_only else "no published tracks"
        if table_y + body <= table_bottom:
          _draw_text(body_font, empty, info_x, table_y, body, MUTED)

    btn_y = footer_y + 16
    self._close.render(rl.Rectangle(rect.x + pad, btn_y, btn_w, btn_h))
    self._align_btn.render(rl.Rectangle(rect.x + pad + btn_w + 16, btn_y, align_w, btn_h))
    mode = "align 2.5–14.5m" if self._align_only else "all tracks"
    mode_color = health_color("LIVE") if self._align_only else MUTED
    mw, mh = chip_size(mode, body)
    mode_x = rect.x + pad + btn_w + align_w + 24
    mode_y = btn_y + max(0, (btn_h - mh) / 2)
    if mode_x + mw <= rect.x + rect.width - pad:
      draw_chip(mode_x, mode_y, mode, mode_color, body,
                fill=ALIGN_FILL if self._align_only else CHIP_FILL)
