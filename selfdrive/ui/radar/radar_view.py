"""Bird's-eye Bosch radar plot, driving HUD, and in-session monitor dialog.

Tinkla C2's calibrate/test tools subscribed to cereal `can` and left
pandad running. NAP's script runner killed the comma session instead.
This dialog is a push_widget overlay: open while driving, close without
reboot, no NAPScriptRunning.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

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

# 0x501 bits Jack actually uses to decide "wrong radar / wrong car"
ALERT_CHIP = {
  "vinValidity": "VIN",
  "xwdValidity": "XWD",
  "radPositionMismatch": "POS",
  "strRackMismatch": "RACK",
}

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

# NAP list_view / settings language
PANEL_BG = rl.Color(12, 14, 18, 230)
PANEL_BORDER = rl.Color(255, 255, 255, 70)
ALIGN_FILL = rl.Color(70, 91, 234, 40)
ALIGN_LINE = rl.Color(70, 91, 234, 160)
TRACK_FILL = rl.Color(90, 200, 255, 230)
TRACK_COAST = rl.Color(180, 180, 180, 200)
LEAD_FILL = rl.Color(128, 216, 166, 255)
TEXT = rl.Color(230, 230, 230, 255)
MUTED = rl.Color(170, 170, 170, 255)         # ITEM_TEXT_VALUE_COLOR
CHIP_MUTED = rl.Color(170, 170, 170, 255)
CHIP_FILL = rl.Color(57, 57, 57, 255)        # LIST_ACTION
CHIP_ALERT = rl.Color(230, 120, 40, 255)
MISMATCH = rl.Color(230, 120, 40, 255)
MATCH = rl.Color(70, 200, 120, 255)
PRIMARY = rl.Color(70, 91, 234, 255)


def health_color(label: str) -> rl.Color:
  return HEALTH_COLOR.get(label, MUTED)


def _draw_text(font, text: str, x: float, y: float, size: int, color: rl.Color) -> None:
  rl.draw_text_ex(font, text, rl.Vector2(x, y), size, 0, color)


def _line_h(font, text: str, size: int, gap: int = 4) -> float:
  return measure_text_cached(font, text or "Ag", size).y + gap


def _clean_vin(raw) -> str:
  if raw is None:
    return ""
  if isinstance(raw, bytes):
    raw = raw.decode("ascii", errors="ignore")
  vin = "".join(ch for ch in str(raw).upper() if ch.isalnum())
  return vin if len(vin) == 17 else (vin if vin and vin != "." * 17 else "")


def decorate_status(status: BoschRadarStatus) -> BoschRadarStatus:
  """Fill F190 (donor param) and chassis (CarParams) when the decoder left them empty."""
  f190 = _clean_vin(getattr(status, "vin_f190", ""))
  chassis = _clean_vin(getattr(status, "vin_chassis", ""))
  if not f190:
    try:
      raw = ui_state.params.get("NAPRadarDonorVin", return_default=True) or ""
      f190 = _clean_vin(raw)
    except Exception:
      f190 = ""
  if not chassis and getattr(ui_state, "CP", None) is not None:
    try:
      chassis = _clean_vin(getattr(ui_state.CP, "carVin", "") or "")
    except Exception:
      chassis = ""
  if f190 != getattr(status, "vin_f190", "") or chassis != getattr(status, "vin_chassis", ""):
    return replace(status, vin_f190=f190, vin_chassis=chassis)
  return status


def radar_hud_rect(content: rl.Rectangle) -> rl.Rectangle:
  """On-road HUD that always fits inside `content`.

  Hidpi / tici (~2100x1020 content): 640x260 card, bottom-left.
  C4 (536x240 screen, ~476x180 after 30px border): full remaining width, ~156 tall.
  Never larger than the content rect.
  """
  compact = content.height < 500
  if compact:
    margin = 6.0
    w = max(0.0, content.width - margin * 2)
    h = min(156.0, max(0.0, content.height - margin * 2))
    return rl.Rectangle(content.x + margin, content.y + content.height - h - margin, w, h)
  margin_x, margin_y = 24.0, 16.0
  w = min(640.0, max(0.0, content.width - margin_x * 2))
  h = min(260.0, max(0.0, content.height - 24.0))
  return rl.Rectangle(content.x + margin_x, content.y + content.height - h - margin_y, w, h)


def _clamp_to_window(rect: rl.Rectangle) -> rl.Rectangle:
  w_max = float(max(1, gui_app.width))
  h_max = float(max(1, gui_app.height))
  x = min(max(0.0, rect.x), w_max)
  y = min(max(0.0, rect.y), h_max)
  w = min(max(0.0, rect.width), w_max - x)
  h = min(max(0.0, rect.height), h_max - y)
  return rl.Rectangle(x, y, w, h)


def _chip_size(text: str, font_size: int) -> tuple[float, float]:
  font = gui_app.font(FontWeight.BOLD)
  sz = measure_text_cached(font, text, font_size)
  pad_x = 6 if font_size < 20 else 12
  pad_y = 2 if font_size < 20 else 5
  return sz.x + pad_x * 2, sz.y + pad_y * 2


def draw_chip(x: float, y: float, text: str, color: rl.Color, font_size: int,
              fill: rl.Color | None = None) -> tuple[float, float]:
  """NAP pill: LIST_ACTION fill, 1.5–2px stroke, bold label."""
  font = gui_app.font(FontWeight.BOLD)
  sz = measure_text_cached(font, text, font_size)
  pad_x = 6 if font_size < 20 else 12
  pad_y = 2 if font_size < 20 else 5
  w = sz.x + pad_x * 2
  h = sz.y + pad_y * 2
  rec = rl.Rectangle(x, y, w, h)
  bg = fill if fill is not None else rl.Color(color.r, color.g, color.b, 48)
  rl.draw_rectangle_rounded(rec, 0.45, 8, bg)
  rl.draw_rectangle_rounded_lines_ex(rec, 0.45, 8, 1.5 if font_size < 20 else 2, color)
  _draw_text(font, text, x + pad_x, y + pad_y, font_size, color)
  return w, h


def _flow_chips(x: float, y: float, max_w: float, bottom: float, items: list[tuple[str, rl.Color, rl.Color | None]],
                font_size: int, gap: float = 4) -> float:
  """Wrap chips. Returns y after the last row (or y unchanged if nothing fits)."""
  if not items:
    return y
  cx = x
  row_h = 0.0
  started = False
  for text, color, fill in items:
    tw, th = _chip_size(text, font_size)
    if cx + tw > x + max_w and cx > x:
      cx = x
      y += row_h + 3
      row_h = 0.0
    if y + th > bottom:
      break
    draw_chip(cx, y, text, color, font_size, fill=fill)
    cx += tw + gap
    row_h = max(row_h, th)
    started = True
  if started:
    y += row_h + 4
  return y


def _lamp_chips(status: BoschRadarStatus) -> list[tuple[str, rl.Color, rl.Color | None]]:
  items = []
  if status.hw_fail:
    items.append(("HWFail", HEALTH_COLOR["FAULT"], None))
  if status.sgu_fail:
    items.append(("SGUFail", CHIP_ALERT, None))
  if status.dirty:
    items.append(("dirty", MUTED, CHIP_FILL))
  if status.table_frozen:
    items.append(("frozen", HEALTH_COLOR["FROZEN"], None))
  if status.radar_fault:
    items.append(("radarFault", HEALTH_COLOR["FAULT"], None))
  if status.can_error:
    items.append(("canError", HEALTH_COLOR["CAN"], None))
  return items


def _alert_chips(status: BoschRadarStatus) -> list[tuple[str, rl.Color, rl.Color | None]]:
  items = []
  named = set()
  for key, short in ALERT_CHIP.items():
    if key in status.alerts:
      items.append((short, CHIP_ALERT, None))
      named.add(key)
  for name in status.alerts:
    if name not in named:
      items.append((name, MUTED, CHIP_FILL))
  return items


def _gtw_chips(status: BoschRadarStatus) -> list[tuple[str, rl.Color, rl.Color | None]]:
  items = []
  if status.awd is None and status.position is None and status.epas_type is None:
    return items
  if status.awd is not None:
    items.append(("4WD" if status.awd else "2WD", TEXT, CHIP_FILL))
  if status.position is not None:
    items.append((f"pos {status.position}", TEXT, CHIP_FILL))
  if status.epas_type is not None:
    items.append((f"EPAS {status.epas_type}", TEXT, CHIP_FILL))
  return items


def _gate_chip(status: BoschRadarStatus) -> tuple[str, rl.Color, rl.Color | None]:
  # silent-until-VIN-complete: host 0x560 mux must finish before panda talks
  if not status.vin_stream_complete:
    return "waiting", HEALTH_COLOR["WAIT VIN"], None
  return "talking", MATCH, None


def _raw_chip(status: BoschRadarStatus) -> tuple[str, rl.Color, rl.Color | None]:
  age = getattr(status, "last_raw_age_s", None)
  if age is None:
    label = f"310 {status.unique_raw}"
  elif age < 10:
    label = f"310 {status.unique_raw}  {age:.1f}s"
  else:
    label = f"310 {status.unique_raw}  {age:.0f}s"
  frozenish = status.unique_raw <= 1 and status.table_frozen
  return label, (HEALTH_COLOR["FROZEN"] if frozenish else MUTED), CHIP_FILL


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
    x, y = to_px(max(0.0, track.d_rel), track.y_rel)
    color = LEAD_FILL if track is closest else (TRACK_FILL if track.measured else TRACK_COAST)
    radius = (4 if track is closest else 3) if compact else (7 if track is closest else 5)
    rl.draw_circle(int(x), int(y), radius, color)

  font = gui_app.font(FontWeight.NORMAL)
  tick = 11 if compact else 18
  _draw_text(font, "0m", rect.x + 4, rect.y + rect.height - (14 if compact else 28), tick, MUTED)
  _draw_text(font, f"{int(RANGE_M)}m", rect.x + 4, rect.y + 3, tick, MUTED)


def _vin_value(status: BoschRadarStatus) -> str:
  vin = getattr(status, "vin", "") or ""
  return vin if len(vin) == 17 else (vin or "—")


def draw_vin_trio(x: float, y: float, max_w: float, bottom: float, status: BoschRadarStatus,
                  font_size: int) -> float:
  """0x2B9 vs F190 vs chassis. Jack's spare / Ian donor mixups were real."""
  font = gui_app.font(FontWeight.MEDIUM)
  bold = gui_app.font(FontWeight.BOLD)
  rows = (
    ("2B9", _vin_value(status)),
    ("F190", _clean_vin(getattr(status, "vin_f190", "")) or "—"),
    ("CHS", _clean_vin(getattr(status, "vin_chassis", "")) or "—"),
  )
  present = [v for _, v in rows if v != "—"]
  mismatch = len(set(present)) > 1
  tag_w = measure_text_cached(bold, "F190", font_size).x + 6
  line = _line_h(font, "VIN", font_size, 1 if font_size < 16 else 3)
  for tag, value in rows:
    if y + line > bottom:
      break
    _draw_text(bold, tag, x, y, font_size, MUTED)
    color = TEXT
    suffix = ""
    if value != "—" and mismatch:
      others = [v for t, v in rows if t != tag and v != "—"]
      if others and value not in others:
        color = MISMATCH
        suffix = "  ≠"
      elif others and all(value == o for o in others):
        color = MATCH
    label = value + suffix
    size = font_size
    while size >= 10 and measure_text_cached(font, label, size).x > max_w - tag_w:
      size -= 1
    _draw_text(font, label, x + tag_w, y, size, color)
    y += line
  return y


def draw_status_panel(rect: rl.Rectangle, status: BoschRadarStatus, *, compact: bool,
                      font_size: int, show_health_chip: bool = True) -> float:
  """Health + lamps + 0x501 + 0x2A9 + gate/310 + VIN trio. Returns y after last line."""
  if rect.width < 8 or rect.height < 8:
    return rect.y

  x, y = rect.x, rect.y
  max_w = rect.width
  bottom = rect.y + rect.height
  chip = font_size
  body = font_size
  font = gui_app.font(FontWeight.MEDIUM)

  if show_health_chip:
    label = status.health_label
    cw, ch = draw_chip(x, y, label, health_color(label), chip)
    meta = f"{len(status.tracks)} trk"
    mx = x + cw + 6
    if mx + measure_text_cached(font, meta, body).x <= x + max_w:
      _draw_text(font, meta, mx, y + max(0, (ch - measure_text_cached(font, meta, body).y) / 2), body, MUTED)
    y += ch + (3 if compact else 6)

  gate = _gate_chip(status)
  raw = _raw_chip(status)
  y = _flow_chips(x, y, max_w, bottom, [gate, raw], max(11, chip - 1))
  y = _flow_chips(x, y, max_w, bottom, _lamp_chips(status), max(11, chip - 1))
  y = _flow_chips(x, y, max_w, bottom, _alert_chips(status), max(11, chip - 1))
  y = _flow_chips(x, y, max_w, bottom, _gtw_chips(status), max(11, chip - 1))

  if y + 14 <= bottom:
    y = draw_vin_trio(x, y, max_w, bottom, status, body)
  return y


def _track_row(track: RadarTrack) -> str:
  flag = "m" if track.measured else "c"
  return f"{track.track_id:>4}  {track.d_rel:6.1f}  {track.y_rel:+6.2f}  {track.v_rel:+6.2f}  {flag}"


@dataclass(frozen=True)
class _DialogMetrics:
  compact: bool
  pad: int
  title_size: int
  body: int
  chip: int
  btn_h: int
  btn_w: int
  align_w: int
  btn_font: int
  btn_pad: int
  btn_radius: int
  header_h: int
  footer_h: int
  plot_frac: float
  status_size: int
  table_rows: int


def _dialog_metrics(rect: rl.Rectangle) -> _DialogMetrics:
  compact = rect.height < 500
  if compact:
    btn_h = 26
    return _DialogMetrics(
      compact=True,
      pad=6,
      title_size=16,
      body=12,
      chip=12,
      btn_h=btn_h,
      btn_w=80,
      align_w=80,
      btn_font=15,
      btn_pad=3,
      btn_radius=8,
      header_h=24,
      footer_h=btn_h + 8,
      plot_frac=0.32,
      status_size=12,
      table_rows=2,
    )
  # Hidpi: NAP list_view BUTTON_FONT_SIZE=35, BUTTON_HEIGHT=100, ITEM 50/40
  btn_h = 80
  return _DialogMetrics(
    compact=False,
    pad=24,
    title_size=40,
    body=22,
    chip=22,
    btn_h=btn_h,
    btn_w=240,
    align_w=200,
    btn_font=35,
    btn_pad=16,
    btn_radius=10,
    header_h=88,
    footer_h=btn_h + 20,
    plot_frac=0.36,
    status_size=22,
    table_rows=8,
  )


class RadarHudOverlay(Widget):
  """Compact on-road readout. Hidden unless NAPRadarHud is set."""

  def _render(self, rect: rl.Rectangle):
    if not ui_state.radar_hud:
      return
    status = decorate_status(ui_state.radar_status)
    panel = _clamp_to_window(rect)
    if panel.width < 8 or panel.height < 8:
      return
    compact = panel.height < 200
    rl.draw_rectangle_rounded(panel, 0.08, 8, PANEL_BG)
    rl.draw_rectangle_rounded_lines_ex(panel, 0.08, 8, 2, health_color(status.health_label))

    pad = 6 if compact else 12
    plot_w = min(100 if compact else 160, panel.width * (0.28 if compact else 0.34))
    plot = rl.Rectangle(panel.x + pad, panel.y + pad, plot_w, panel.height - pad * 2)
    draw_radar_plot(plot, status, align_box=False, compact=compact)
    text = rl.Rectangle(
      plot.x + plot.width + (6 if compact else 12),
      panel.y + pad,
      panel.width - plot.width - pad * 3,
      panel.height - pad * 2,
    )
    draw_status_panel(text, status, compact=compact, font_size=11 if compact else 18)


class RadarMonitorDialog(Widget):
  """Full-screen live radar. Does not stop pandad or set NAPScriptRunning."""

  def __init__(self):
    super().__init__()
    # Font size is retuned in _render for C4 vs hidpi. Close is pop_widget only.
    self._close = Button("Close", click_callback=self._on_close,
                         font_size=35, text_padding=16,
                         button_style=ButtonStyle.TRANSPARENT_WHITE_BORDER)
    self._align_only = False
    self._align_btn = Button("Align", click_callback=self._toggle_align,
                             font_size=35, text_padding=16,
                             button_style=ButtonStyle.ACTION)

  def _on_close(self):
    gui_app.pop_widget()

  def _toggle_align(self):
    self._align_only = not self._align_only

  def _tune_buttons(self, m: _DialogMetrics) -> None:
    for btn in (self._close, self._align_btn):
      btn._label.set_font_size(m.btn_font)
      btn._label._text_padding = m.btn_pad
      btn._border_radius = m.btn_radius
    self._align_btn.set_button_style(ButtonStyle.PRIMARY if self._align_only else ButtonStyle.ACTION)

  def _render(self, rect: rl.Rectangle):
    status = decorate_status(ui_state.radar_status)
    if self._align_only:
      tracks = tuple(
        t for t in status.tracks
        if ALIGN_D_MIN <= t.d_rel <= ALIGN_D_MAX and abs(t.y_rel) <= ALIGN_Y_MAX
      )
      status = replace(status, tracks=tracks)

    m = _dialog_metrics(rect)
    self._tune_buttons(m)

    # Opaque fill — alpha 230 left ghost glyphs from the previous pyray frame.
    rl.draw_rectangle_rec(rect, rl.Color(0, 0, 0, 255))

    title_font = gui_app.font(FontWeight.BOLD)
    body_font = gui_app.font(FontWeight.MEDIUM)
    _draw_text(title_font, "Live radar", rect.x + m.pad, rect.y + (3 if m.compact else 14), m.title_size, TEXT)

    chip_x = rect.x + m.pad + measure_text_cached(title_font, "Live radar", m.title_size).x + (8 if m.compact else 16)
    chip_y = rect.y + (3 if m.compact else 18)
    draw_chip(chip_x, chip_y, status.health_label, health_color(status.health_label), m.chip)

    if not m.compact:
      _draw_text(body_font, "openpilot stays running", rect.x + m.pad, rect.y + 60, 20, MUTED)

    plot_w = min(rect.width * m.plot_frac, 168 if m.compact else 700)
    plot_h = rect.height - m.header_h - m.footer_h
    plot = rl.Rectangle(rect.x + m.pad, rect.y + m.header_h, plot_w, plot_h)
    draw_radar_plot(plot, status, align_box=True, compact=m.compact)

    info_x = plot.x + plot.width + (8 if m.compact else 24)
    info_w = rect.x + rect.width - info_x - m.pad
    footer_y = rect.y + rect.height - m.footer_h
    info = rl.Rectangle(info_x, plot.y, info_w, max(0, footer_y - plot.y - (4 if m.compact else 10)))
    y = draw_status_panel(info, status, compact=m.compact, font_size=m.status_size, show_health_chip=False)

    # Track table sits strictly above the footer so Align never covers rows.
    table_y = y + (3 if m.compact else 10)
    table_bottom = footer_y - (2 if m.compact else 8)
    if table_y + m.body < table_bottom and info_w > 40:
      _draw_text(body_font, "  ID     dRel     yRel     vRel", info_x, table_y, m.body, MUTED)
      table_y += _line_h(body_font, "dRel", m.body, 2 if m.compact else 5)
      rows = sorted(status.tracks, key=lambda t: t.d_rel)[: m.table_rows]
      for track in rows:
        if table_y + m.body > table_bottom:
          break
        _draw_text(body_font, _track_row(track), info_x, table_y, m.body, TEXT)
        table_y += _line_h(body_font, _track_row(track), m.body, 2 if m.compact else 4)
      if not rows:
        empty = "no tracks in 2.5–14.5 m" if self._align_only else "no published tracks"
        if table_y + m.body <= table_bottom:
          _draw_text(body_font, empty, info_x, table_y, m.body, MUTED)

    btn_y = footer_y + (4 if m.compact else 10)
    self._close.render(rl.Rectangle(rect.x + m.pad, btn_y, m.btn_w, m.btn_h))
    self._align_btn.render(rl.Rectangle(rect.x + m.pad + m.btn_w + (6 if m.compact else 14), btn_y, m.align_w, m.btn_h))

    mode = "2.5–14.5 m" if self._align_only else "all tracks"
    mode_color = PRIMARY if self._align_only else MUTED
    mode_x = rect.x + m.pad + m.btn_w + m.align_w + (8 if m.compact else 18)
    mode_y = btn_y + max(0, (m.btn_h - _chip_size(mode, m.body)[1]) / 2)
    if mode_x + 20 < rect.x + rect.width - m.pad:
      draw_chip(mode_x, mode_y, mode, mode_color, m.body,
                fill=ALIGN_FILL if self._align_only else CHIP_FILL)
