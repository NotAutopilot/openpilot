"""Live HUD hold for the camera speed-sign readout.

Display-only. Does not write sqlite or change cruise.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from openpilot.selfdrive.speedsignd.detect_types import MUTCD_MPH, SpeedSign
from openpilot.selfdrive.speedsignd.weights_manifest import YOLO_MIN_CONF
from openpilot.selfdrive.speedsignd.yolo import REFINE_OVERRIDE_CONF

# Hold a recent *accepted* mph across 1 Hz skip-on-overrun. Justin's parked
# 50 went blank after 10 s while tinygrad infer stayed 3–5 s / intermittent,
# then flashed 60 / 70 / 40. 45 s plus a soft refresh on any in-threshold
# speedLimit* box (even refine fail / junk class) keeps SIGN lit without
# inventing an mph.
HUD_HOLD_S = 45.0
# Two reads of the same mph from different frames. A class/OCR pair on one
# frame is a single vote. v8n duty gap is about 4–16 s; 12 s covers two
# reads when infer is up to ~3 s.
AGREE_READS = 2
AGREE_WINDOW_S = 12.0
HUD_LABEL = "SIGN"
# Logger On + ONNX missing: show this instead of a blank plate or a fake mph.
HUD_MISSING_WEIGHTS_TEXT = "NO WT"
# WAIT plate if detectPaused is set. Engaged driving does not set it —
# the mph hold stays up. Cereal-unknown is not WAIT.
HUD_DETECT_PAUSED_TEXT = "WAIT"
HUD_CONFIRM_PROMPT = "accurate?"
HUD_CONFIRM_YES = "Yes"
HUD_CONFIRM_NO = "No"

# TICI 2160x1080 — driver-left, below the MAX box (hud_renderer UI_CONFIG).
TICI_SIGN_W = 200
TICI_SIGN_H = 248
TICI_SIGN_LEFT = 60
TICI_MAX_TOP = 45
TICI_MAX_H = 204
TICI_BELOW_MAX_GAP = 16
TICI_CONFIRM_BTN_H = 112
TICI_CONFIRM_GAP = 10
TICI_CONFIRM_PROMPT_H = 32

# Mici 536x240 — same left / driver-side idea.
MICI_SIGN_W = 108
MICI_SIGN_H = 128
MICI_SIGN_LEFT = 8
MICI_SIGN_TOP = 6
MICI_CONFIRM_BTN_W = 68
MICI_CONFIRM_GAP = 6


def plate_digit_size(text: str, default: int) -> int:
  """Shrink 'NO WT' so it fits the mph plate; keep 2–3 digit mph large."""
  if len(text) > 3:
    return max(22, int(default * 0.42))
  return default


def hud_should_show(enabled: bool, valid: bool, mph: int) -> bool:
  return bool(enabled) and bool(valid) and int(mph) > 0


def hud_should_show_missing_weights(enabled: bool, weights_missing: bool) -> bool:
  return bool(enabled) and bool(weights_missing)


def hud_should_show_detect_paused(enabled: bool, detect_paused: bool, weights_missing: bool = False) -> bool:
  """WAIT only when detect is paused because OP is controlling — never for unknown cereal."""
  return bool(enabled) and bool(detect_paused) and not bool(weights_missing)


def hud_confirm_visible(view: LiveSignView | None = None, *, show_mph: bool = False, mph: int = 0,
                        show_missing_weights: bool = False, show_detect_paused: bool = False) -> bool:
  """Yes/No accuracy overlay: only when a real mph is on the plate, never for NO WT / WAIT."""
  if view is not None:
    show_mph = view.show_mph
    mph = view.mph
    show_missing_weights = view.show_missing_weights
    show_detect_paused = view.show_detect_paused
  return (
    bool(show_mph) and int(mph) > 0
    and not bool(show_missing_weights) and not bool(show_detect_paused)
  )


def tici_sign_origin(rect_x: float = 0.0, rect_y: float = 0.0) -> tuple[float, float]:
  """Left / driver side, below MAX so the plate does not cover MAX or the center path."""
  return rect_x + TICI_SIGN_LEFT, rect_y + TICI_MAX_TOP + TICI_MAX_H + TICI_BELOW_MAX_GAP


def mici_sign_origin(rect_x: float = 0.0, rect_y: float = 0.0) -> tuple[float, float]:
  return rect_x + MICI_SIGN_LEFT, rect_y + MICI_SIGN_TOP


def accuracy_button_rects(
  sign_x: float,
  sign_y: float,
  sign_w: float,
  sign_h: float,
  *,
  beside: bool = False,
  btn_w: float | None = None,
  btn_h: float = TICI_CONFIRM_BTN_H,
  gap: float = TICI_CONFIRM_GAP,
  prompt_h: float = 0.0,
) -> tuple[tuple[float, float, float, float], tuple[float, float, float, float]]:
  """Return (yes, no) as (x, y, w, h). TICI stacks under the plate; mici sits beside."""
  if beside:
    w = MICI_CONFIRM_BTN_W if btn_w is None else btn_w
    h = (sign_h - gap) / 2.0
    yes = (sign_x + sign_w + gap, sign_y, w, h)
    no = (sign_x + sign_w + gap, sign_y + h + gap, w, h)
    return yes, no
  y0 = sign_y + sign_h + gap + prompt_h
  yes = (sign_x, y0, sign_w, btn_h)
  no = (sign_x, y0 + btn_h + gap, sign_w, btn_h)
  return yes, no


def point_in_rect(px: float, py: float, r: tuple[float, float, float, float]) -> bool:
  x, y, w, h = r
  return x <= px <= x + w and y <= py <= y + h


def hit_accuracy_button(
  px: float,
  py: float,
  yes: tuple[float, float, float, float],
  no: tuple[float, float, float, float],
) -> bool | None:
  """True = Yes, False = No, None = miss."""
  if point_in_rect(px, py, yes):
    return True
  if point_in_rect(px, py, no):
    return False
  return None


@dataclass(frozen=True)
class LiveSignView:
  show_mph: bool = False
  mph: int = 0
  show_missing_weights: bool = False
  show_detect_paused: bool = False

  @property
  def show(self) -> bool:
    return self.show_mph or self.show_missing_weights or self.show_detect_paused

  @property
  def plate_text(self) -> str:
    if self.show_missing_weights:
      return HUD_MISSING_WEIGHTS_TEXT
    if self.show_detect_paused:
      return HUD_DETECT_PAUSED_TEXT
    if self.show_mph:
      return str(int(self.mph))
    return ""

  @property
  def confirm_visible(self) -> bool:
    return hud_confirm_visible(self)


def live_sign_from_event(enabled: bool, d, msg_valid: bool) -> LiveSignView:
  """Parse a liveSpeedSignNAP-like object for the on-road plate."""
  weights_missing = bool(getattr(d, "weightsMissing", False))
  detect_paused = bool(getattr(d, "detectPaused", False))
  return live_sign_view(
    enabled=bool(enabled),
    msg_valid=bool(msg_valid) or weights_missing or detect_paused,
    mph=int(getattr(d, "mph", 0) or 0),
    valid=bool(getattr(d, "valid", False)),
    weights_missing=weights_missing,
    detect_paused=detect_paused,
  )


def live_sign_view(
  *,
  enabled: bool,
  msg_valid: bool,
  mph: int,
  valid: bool,
  weights_missing: bool = False,
  detect_paused: bool = False,
) -> LiveSignView:
  """What the on-road SIGN plate should show. No false mph when weights are missing."""
  if not enabled:
    return LiveSignView()
  if hud_should_show_missing_weights(True, weights_missing):
    return LiveSignView(show_missing_weights=True)
  if hud_should_show_detect_paused(True, detect_paused, weights_missing):
    return LiveSignView(show_detect_paused=True)
  if not msg_valid:
    return LiveSignView()
  if hud_should_show(True, valid, mph):
    return LiveSignView(show_mph=True, mph=int(mph))
  return LiveSignView()


def apply_live_sign(
  dst, *, mph: int, conf: float, valid: bool,
  weights_missing: bool = False, detect_paused: bool = False,
) -> None:
  dst.mph = int(mph) if valid else 0
  dst.conf = float(conf) if valid else 0.0
  dst.valid = bool(valid)
  dst.weightsMissing = bool(weights_missing)
  dst.detectPaused = bool(detect_paused)


def _class_mph(sign: SpeedSign) -> int:
  class_mph = getattr(sign, "class_mph", None)
  if class_mph is not None:
    return int(class_mph)
  return int(sign.mph)


def should_replace_held_mph(
  held_mph: int, held_conf: float, new_mph: int, new_conf: float,
  refine_mph: int | None = None,
) -> bool:
  """Same mph refreshes. A different agreed mph replaces when it is at least as sure."""
  del refine_mph
  if int(held_mph) <= 0:
    return True
  if int(new_mph) == int(held_mph):
    return True
  return float(new_conf) >= float(held_conf)


def is_speed_limit_sighting(sign: SpeedSign | None) -> bool:
  """True if YOLO posted any in-threshold speedLimit* (refine may have failed).

  Used only to extend a live hold — never to invent or change mph.
  """
  if sign is None:
    return False
  try:
    conf = float(sign.conf)
  except (TypeError, ValueError):
    return False
  if conf < YOLO_MIN_CONF:
    return False
  refine = getattr(sign, "refine_mph", None)
  if refine is not None:
    try:
      if int(refine) in MUTCD_MPH:
        return True
    except (TypeError, ValueError):
      pass
  try:
    if int(sign.mph) in MUTCD_MPH:
      return True
  except (TypeError, ValueError):
    pass
  try:
    return _class_mph(sign) in MUTCD_MPH
  except (TypeError, ValueError):
    return False


def frame_reads(sign: SpeedSign | None) -> tuple[list[tuple[int, float]], tuple[int, float] | None]:
  """At most one vote from one frame. The second return is always None.

  v2 misreads are systematic (60 read as 30, 30 read as 50, 30/35/60 as
  50). A same-frame class+OCR pair used to light the HUD immediately, so a
  wrong class that OCR repeated became a 45 s plate.

  - class and OCR name the same mph: one vote. A later frame must agree.
  - class and OCR differ, both above their floors: no vote. A correct OCR
    beside a wrong class is a miss, not a misread.
  - class only: no vote. That is how route 17a posted 60 as 30.
  - OCR only (second look, or refine with no in-threshold class): one vote.
  - bare mph with neither field set (numpy tests): one vote.
  sl_unreadable and neg_* have no MUTCD mph here, so they never vote.
  """
  if sign is None:
    return [], None
  explicit_class = getattr(sign, "class_mph", None)
  class_conf = float(getattr(sign, "class_conf", 0.0) or 0.0)
  refine = getattr(sign, "refine_mph", None)
  refine_conf = float(getattr(sign, "refine_conf", 0.0) or 0.0)
  class_i = None
  class_ok = False
  if explicit_class is not None:
    try:
      class_i = int(explicit_class)
      class_ok = class_i in MUTCD_MPH and class_conf >= YOLO_MIN_CONF
    except (TypeError, ValueError):
      class_i = None
  refine_i = None
  refine_ok = False
  if refine is not None:
    try:
      refine_i = int(refine)
      refine_ok = refine_i in MUTCD_MPH and refine_conf >= REFINE_OVERRIDE_CONF
    except (TypeError, ValueError):
      refine_i = None
  if class_ok and refine_ok and class_i != refine_i:
    return [], None
  if class_ok and refine_ok and class_i == refine_i:
    return [(int(class_i), max(class_conf, refine_conf))], None
  if refine_ok and not class_ok:
    return [(int(refine_i), refine_conf)], None
  if class_ok:
    return [], None
  if explicit_class is None and refine is None:
    try:
      mph = int(sign.mph)
      conf = float(sign.conf)
    except (TypeError, ValueError):
      return [], None
    if mph in MUTCD_MPH and conf >= YOLO_MIN_CONF:
      return [(mph, conf)], None
  return [], None


def note_read(obs: list, now: float, mph: int, conf: float, extra=None) -> None:
  """Record one vote. The same timestamp and mph is the same frame, not a second read."""
  mph = int(mph)
  conf = float(conf)
  for i, row in enumerate(obs):
    if row[0] == now and int(row[1]) == mph:
      if conf > float(row[2]):
        obs[i] = (now, mph, conf) if extra is None else (now, mph, conf, extra)
      return
  obs.append((now, mph, conf) if extra is None else (now, mph, conf, extra))


def accepted_hud_sign(sign: SpeedSign | None) -> SpeedSign | None:
  """One frame never lights the HUD, even when class and OCR agree.

  40/60/65/70 are not special-cased. Callers that used to treat a returned
  sign as an immediate plate must wait for LiveSignHold.
  """
  del sign
  return None


@dataclass
class LiveSignHold:
  hold_s: float = HUD_HOLD_S
  mph: int = 0
  conf: float = 0.0
  until: float = 0.0
  _obs: list = field(default_factory=list)

  def update(self, signs, now: float) -> tuple[bool, int, float]:
    fresh_mphs: set[int] = set()
    for s in signs or []:
      pending, _pair = frame_reads(s)
      for mph, conf in pending:
        note_read(self._obs, now, int(mph), float(conf))
        fresh_mphs.add(int(mph))
    self._obs = [o for o in self._obs if now - o[0] <= AGREE_WINDOW_S]
    buckets: dict[int, list[float]] = {}
    for _t, mph, conf in self._obs:
      buckets.setdefault(int(mph), []).append(float(conf))
    winners = [(mph, max(cs)) for mph, cs in buckets.items() if len(cs) >= AGREE_READS]
    # An older mph that still has two votes must not light again just because
    # this frame voted for something else. Only a mph read this call can.
    fresh_winners = [item for item in winners if item[0] in fresh_mphs]
    holding = now < self.until and self.mph > 0
    if fresh_winners:
      new_mph, new_conf = max(fresh_winners, key=lambda item: item[1])
      if (not holding) or should_replace_held_mph(self.mph, self.conf, new_mph, new_conf):
        self.mph = int(new_mph)
        self.conf = float(new_conf)
        self.until = now + self.hold_s
      else:
        self.until = now + self.hold_s
    elif holding and any(is_speed_limit_sighting(s) for s in (signs or [])):
      self.until = now + self.hold_s
    live = now < self.until and self.mph > 0
    if not live:
      return False, 0, 0.0
    return True, self.mph, self.conf
