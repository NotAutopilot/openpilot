"""HUD hold and cereal live sample for the speed-sign logger."""
from __future__ import annotations

from types import SimpleNamespace

import pytest

from openpilot.selfdrive.speedsignd.detect import SpeedSign, SpeedSignDetector, paint_mutcd_r2_1
from openpilot.selfdrive.speedsignd.hud import (
  AGREE_READS,
  AGREE_WINDOW_S,
  HUD_DETECT_PAUSED_TEXT,
  HUD_HOLD_S,
  SOLO_CONF,
  SOLO_CONTRADICT_S,
  HUD_MISSING_WEIGHTS_TEXT,
  HUD_RUNTIME_MISSING_TEXT,
  LiveSignHold,
  is_speed_limit_sighting,
  should_replace_held_mph,
  TICI_CONFIRM_BTN_H,
  TICI_SIGN_H,
  TICI_SIGN_W,
  accepted_hud_sign,
  accuracy_button_rects,
  hit_accuracy_button,
  hud_confirm_visible,
  plate_digit_size,
  apply_live_sign,
  apply_runtime_plate,
  hud_should_show,
  hud_should_show_detect_paused,
  hud_should_show_missing_weights,
  live_sign_from_event,
  live_sign_view,
  mici_sign_origin,
  tici_sign_origin,
)
from openpilot.selfdrive.speedsignd.jsonl import JsonlLogger
from openpilot.selfdrive.speedsignd.speedsignd import live_sign_publish_fields, process_frame
from openpilot.selfdrive.speedsignd.tests.test_detect import _scene


def test_hold_keeps_mph_then_clears():
  h = LiveSignHold(hold_s=3.0)
  signs = [SpeedSign(mph=45, conf=0.60, bbox=(0, 0, 10, 10))]
  live, mph, _ = h.update(signs, 9.8)
  assert not live
  live, mph, conf = h.update(signs, 10.0)
  assert live and mph == 45 and conf == 0.60
  live, mph, _ = h.update([], 11.0)
  assert live and mph == 45
  live, mph, _ = h.update([], 12.99)
  assert live and mph == 45
  live, mph, conf = h.update([], 13.01)
  assert not live and mph == 0 and conf == 0.0


def test_new_detection_refreshes_hold():
  h = LiveSignHold(hold_s=3.0)
  h.update([SpeedSign(mph=30, conf=0.7, bbox=(0, 0, 1, 1))], 0.0)
  live, mph, _ = h.update([SpeedSign(mph=30, conf=0.7, bbox=(0, 0, 1, 1))], 0.2)
  assert live and mph == 30
  h.update([SpeedSign(mph=55, conf=0.9, bbox=(0, 0, 1, 1))], 1.0)
  live, mph, _ = h.update([SpeedSign(mph=55, conf=0.9, bbox=(0, 0, 1, 1))], 1.2)
  assert live and mph == 55
  live, mph, _ = h.update([], 3.9)
  assert live and mph == 55


def test_hud_hidden_unless_enabled_and_valid():
  assert not hud_should_show(False, True, 45)
  assert not hud_should_show(True, False, 45)
  assert not hud_should_show(True, True, 0)
  assert hud_should_show(True, True, 45)


def test_apply_live_sign_clears_when_invalid():
  d = SimpleNamespace()
  apply_live_sign(d, mph=45, conf=0.8, valid=True)
  assert d.mph == 45 and d.valid and not d.weightsMissing and not d.detectPaused
  apply_live_sign(d, mph=45, conf=0.8, valid=False)
  assert d.mph == 0 and d.conf == 0.0 and not d.valid and not d.weightsMissing and not d.detectPaused
  apply_live_sign(d, mph=45, conf=0.8, valid=False, weights_missing=True)
  assert d.mph == 0 and not d.valid and d.weightsMissing and not d.detectPaused
  apply_live_sign(d, mph=45, conf=0.8, valid=False, detect_paused=True)
  assert d.mph == 0 and not d.valid and d.detectPaused and not d.weightsMissing


def test_hud_missing_weights_plate_when_logger_on():
  assert HUD_MISSING_WEIGHTS_TEXT == "NO WT"
  assert plate_digit_size("NO WT", 120) < plate_digit_size("55", 120)
  assert hud_should_show_missing_weights(True, True)
  assert not hud_should_show_missing_weights(False, True)
  assert not hud_should_show_missing_weights(True, False)

  missing = live_sign_view(enabled=True, msg_valid=True, mph=0, valid=False, weights_missing=True)
  assert missing.show and missing.show_missing_weights
  assert missing.plate_text == "NO WT"
  assert not missing.show_mph

  # Never a false mph while weights are missing, even if numpy produced one.
  fake = live_sign_view(enabled=True, msg_valid=True, mph=55, valid=True, weights_missing=True)
  assert fake.show_missing_weights and fake.plate_text == "NO WT"
  assert not fake.show_mph

  off = live_sign_view(enabled=False, msg_valid=True, mph=0, valid=False, weights_missing=True)
  assert not off.show

  blank = live_sign_view(enabled=True, msg_valid=False, mph=0, valid=False, weights_missing=False)
  assert not blank.show and blank.plate_text == ""

  mph = live_sign_view(enabled=True, msg_valid=True, mph=55, valid=True, weights_missing=False)
  assert mph.show_mph and mph.mph == 55 and mph.plate_text == "55"

  ev = live_sign_from_event(True, SimpleNamespace(mph=0, valid=False, weightsMissing=True), True)
  assert ev.show_missing_weights and ev.plate_text == "NO WT"
  ev_blank = live_sign_from_event(True, SimpleNamespace(mph=0, valid=False, weightsMissing=False), False)
  assert not ev_blank.show
  ev_mph = live_sign_from_event(True, SimpleNamespace(mph=60, valid=True, weightsMissing=False), True)
  assert ev_mph.show_mph and ev_mph.plate_text == "60"


def test_hud_runtime_missing_plate():
  assert HUD_RUNTIME_MISSING_TEXT == "NO RT"
  assert plate_digit_size("NO RT", 120) < plate_digit_size("55", 120)
  missing_rt = live_sign_view(
    enabled=True, msg_valid=True, mph=55, valid=True, runtime_missing=True,
  )
  assert missing_rt.show_runtime_missing and missing_rt.plate_text == "NO RT"
  assert not missing_rt.show_mph
  assert not hud_confirm_visible(missing_rt)
  # NO WT wins when the ONNX itself is absent. Install gets both.
  both = live_sign_view(
    enabled=True, msg_valid=True, mph=55, valid=True,
    weights_missing=True, runtime_missing=True,
  )
  assert both.plate_text == "NO WT"
  mph_view = live_sign_view(enabled=True, msg_valid=True, mph=60, valid=True)
  overlaid = apply_runtime_plate(mph_view, runtime_missing=True)
  assert overlaid.plate_text == "NO RT"
  kept = apply_runtime_plate(
    live_sign_view(enabled=True, msg_valid=True, mph=0, valid=False, weights_missing=True),
    runtime_missing=True,
  )
  assert kept.plate_text == "NO WT"


def test_hud_wait_plate_when_detect_paused():
  assert HUD_DETECT_PAUSED_TEXT == "WAIT"
  assert plate_digit_size("WAIT", 120) < plate_digit_size("55", 120)
  assert hud_should_show_detect_paused(True, True)
  assert not hud_should_show_detect_paused(False, True)
  assert not hud_should_show_detect_paused(True, False)
  assert not hud_should_show_detect_paused(True, True, weights_missing=True)

  paused = live_sign_view(enabled=True, msg_valid=True, mph=0, valid=False, detect_paused=True)
  assert paused.show and paused.show_detect_paused
  assert paused.plate_text == "WAIT"
  assert not paused.show_mph
  assert not hud_confirm_visible(paused)

  # Never a leftover mph while YOLO is paused.
  leftover = live_sign_view(enabled=True, msg_valid=True, mph=55, valid=True, detect_paused=True)
  assert leftover.show_detect_paused and leftover.plate_text == "WAIT"
  assert not leftover.show_mph

  # Missing weights wins over WAIT so install-weights stays obvious.
  nowt = live_sign_view(
    enabled=True, msg_valid=True, mph=0, valid=False,
    weights_missing=True, detect_paused=True,
  )
  assert nowt.show_missing_weights and nowt.plate_text == "NO WT"
  assert not nowt.show_detect_paused

  ev = live_sign_from_event(True, SimpleNamespace(mph=0, valid=False, weightsMissing=False, detectPaused=True), True)
  assert ev.show_detect_paused and ev.plate_text == "WAIT"

  # Unknown / not-paused must not show WAIT — mph can still appear later.
  unknown = live_sign_view(enabled=True, msg_valid=False, mph=0, valid=False, detect_paused=False)
  assert not unknown.show_detect_paused and unknown.plate_text == ""


def test_publish_fields_flag_missing_weights_without_mph():
  h = LiveSignHold(hold_s=3.0)
  signs = [SpeedSign(mph=45, conf=0.55, bbox=(0, 0, 10, 10))]
  msg_valid, mph, conf, missing, paused = live_sign_publish_fields(h, signs, 10.0, True)
  assert msg_valid and missing and mph == 0 and conf == 0.0 and not paused
  # Hold was not primed by the numpy hit.
  live, held, _ = h.update([], 10.1)
  assert not live and held == 0

  msg_valid, mph, conf, missing, paused = live_sign_publish_fields(h, signs, 10.8, False)
  assert mph == 0
  msg_valid, mph, conf, missing, paused = live_sign_publish_fields(h, signs, 11.0, False)
  assert msg_valid and not missing and mph == 45 and not paused
  live, held, _ = h.update([], 11.5)
  assert live and held == 45

  msg_valid, mph, conf, missing, paused = live_sign_publish_fields(h, signs, 12.0, False, True)
  assert msg_valid and paused and not missing and mph == 0 and conf == 0.0
  # Paused publish does not refresh hold from a leftover sign list.
  live, held, _ = h.update([], 12.1)
  assert live and held == 45


def test_hold_shows_ocr_50_from_two_frames_not_a_contradiction():
  """Class 65 + OCR 50 is not a vote. Two OCR-only frames (second look) are."""
  h = LiveSignHold(hold_s=3.0)
  clash = [SpeedSign(
    mph=50, conf=0.71, bbox=(0, 0, 10, 10),
    class_mph=65, class_conf=0.81, refine_mph=50, refine_conf=0.71,
  )]
  assert not h.update(clash, 9.0)[0]
  assert not h.update(clash, 9.4)[0]
  signs = [SpeedSign(
    mph=50, conf=0.60, bbox=(0, 0, 10, 10),
    class_mph=None, refine_mph=50, refine_conf=0.60,
  )]
  live, mph, _ = h.update(signs, 9.8)
  assert not live
  live, mph, _ = h.update(signs, 10.0)
  assert live and mph == 50
  live, mph, _ = h.update([], 11.0)
  assert live and mph == 50


def test_process_frame_hud_lights_refined_50(tmp_path):
  from openpilot.selfdrive.speedsignd.debounce import SignDebounce

  class _Det:
    onnx = object()

    def detect(self, y, min_conf=None, rgb=None, nv12=None):
      return [SpeedSign(
        mph=50, conf=0.71, bbox=(0, 0, 8, 8),
        class_mph=50, class_conf=0.82, refine_mph=50, refine_conf=0.71,
      )]

  log = JsonlLogger(str(tmp_path / "out.jsonl"))
  debounce = SignDebounce()
  y = _scene()
  signs, written = process_frame(
    y, 45.0, -95.0, 0.0, True, _Det(), log, now=1.0, debounce=debounce,
  )
  assert signs and signs[0].mph == 50
  assert written == []


def test_process_frame_hud_blank_when_yolo_65_refine_fails(tmp_path):
  from openpilot.selfdrive.speedsignd.debounce import SignDebounce

  class _Det:
    onnx = object()

    def detect(self, y, min_conf=None, rgb=None, nv12=None):
      return [SpeedSign(
        mph=65, conf=0.73, bbox=(0, 0, 8, 8),
        class_mph=65, class_conf=0.73, refine_mph=None,
      )]

  log = JsonlLogger(str(tmp_path / "out.jsonl"))
  debounce = SignDebounce()
  y = _scene()
  signs, written = process_frame(
    y, 45.0, -95.0, 0.0, True, _Det(), log, now=1.0, debounce=debounce,
  )
  # Class-only ≥0.70 is offered to the plate. It is not a JSONL row.
  assert signs and signs[0].mph == 65
  assert written == []
  assert debounce.last_raw and debounce.last_raw[0].mph == 65
  live, mph, _ = LiveSignHold().update(signs, 1.0)
  assert live and mph == 65
  live, mph, _ = LiveSignHold().update(
    [SpeedSign(mph=65, conf=0.60, bbox=(0, 0, 8, 8), class_mph=65, class_conf=0.60)],
    2.0,
  )
  assert not live and mph == 0


def test_detect_without_gps_still_returns_signs(tmp_path):
  y = _scene()
  paint_mutcd_r2_1(y, 35, x=200, y=30, w=90, h=112)
  signs, written = process_frame(
    y, 0.0, 0.0, None, False,
    SpeedSignDetector(onnx_path=str(tmp_path / "missing.onnx")),
    JsonlLogger(str(tmp_path / "out.jsonl")),
    now=1.0,
  )
  assert written == []
  assert signs and signs[0].mph == 35


def test_hold_duration_constant():
  # Parked 50 went blank after 10 s of 3–5 s infers; 45 s covers miss gaps.
  # Two agreeing reads replace the {40, 60, 65, 70} junk gate.
  assert HUD_HOLD_S == 45.0
  assert AGREE_READS == 2
  assert AGREE_WINDOW_S == 12.0
  assert SOLO_CONF == 0.70
  assert SOLO_CONTRADICT_S == 3.0


def test_hold_last_good_across_infer_skips():
  """Empty signs between 1 Hz / multi-second infers must keep last accepted mph."""
  h = LiveSignHold()
  signs = [SpeedSign(
    mph=50, conf=0.60, bbox=(0, 0, 10, 10),
    class_mph=50, class_conf=0.60, refine_mph=50, refine_conf=0.60,
  )]
  live, mph, _ = h.update(signs, 9.5)
  assert not live
  live, mph, _ = h.update(signs, 10.0)
  assert live and mph == 50
  for t in (11.0, 12.5, 14.0, 17.5, 19.9, 25.0, 35.0, 50.0):
    live, mph, _ = h.update([], t)
    assert live and mph == 50
  live, mph, conf = h.update([], 10.0 + HUD_HOLD_S + 0.01)
  assert not live and mph == 0 and conf == 0.0


def test_hold_lasts_across_more_than_10s_empty_ticks():
  """Justin: 10 s hold expired during miss gaps. Default 45 s must survive >10 s empty."""
  h = LiveSignHold()
  sign50 = [_sign(50, 0.50, class_mph=50, refine_mph=50, refine_conf=0.50)]
  assert not h.update(sign50, 0.0)[0]
  live, mph, _ = h.update(sign50, 0.2)
  assert live and mph == 50
  live, mph, _ = h.update([], 10.5)
  assert live and mph == 50
  live, mph, _ = h.update([], 20.0)
  assert live and mph == 50
  live, mph, _ = h.update([], 44.9)
  assert live and mph == 50
  # Confirming read was at 0.2 s, so the 45 s hold ends at 45.2.
  live, mph, _ = h.update([], 45.3)
  assert not live


def test_yolo_65_refine_fail_lights_when_solo_confident():
  """One class-only read at ≥0.70 lights. Below that, the plate stays blank."""
  bad = SpeedSign(
    mph=65, conf=0.73, bbox=(0, 0, 10, 10),
    class_mph=65, class_conf=0.73, refine_mph=None,
  )
  assert accepted_hud_sign(bad) is None
  live, mph, _ = LiveSignHold().update([bad], 1.0)
  assert live and mph == 65
  low = SpeedSign(
    mph=65, conf=0.60, bbox=(0, 0, 10, 10),
    class_mph=65, class_conf=0.60, refine_mph=None,
  )
  live, mph, _ = LiveSignHold().update([low], 1.0)
  assert not live and mph == 0


def test_yolo_70_refine_fail_does_not_light_70():
  bad = SpeedSign(
    mph=70, conf=0.68, bbox=(0, 0, 10, 10),
    class_mph=70, class_conf=0.68, refine_mph=None,
  )
  assert accepted_hud_sign(bad) is None
  h = LiveSignHold()
  live, mph, _ = h.update([bad], 1.0)
  assert not live and mph == 0


def test_yolo_65_refine_fail_holds_prior_50():
  """A later unrefined 65 must not replace a refined 50; keep last-good."""
  h = LiveSignHold()
  good = [SpeedSign(
    mph=50, conf=0.60, bbox=(0, 0, 10, 10),
    class_mph=50, class_conf=0.60, refine_mph=50, refine_conf=0.60,
  )]
  live, mph, _ = h.update(good, 0.8)
  assert not live
  live, mph, _ = h.update(good, 1.0)
  assert live and mph == 50
  bad = [SpeedSign(
    mph=65, conf=0.73, bbox=(0, 0, 10, 10),
    class_mph=65, class_conf=0.73, refine_mph=None,
  )]
  live, mph, _ = h.update(bad, 2.0)
  assert live and mph == 50
  live, mph, _ = h.update([], 8.0)
  assert live and mph == 50


def test_class_ocr_contradiction_never_lights():
  """v2 often reads 60 as 30 and 30 as 50. A disagreeing OCR must not vote either way."""
  sign = SpeedSign(
    mph=50, conf=0.71, bbox=(0, 0, 10, 10),
    class_mph=65, class_conf=0.82, refine_mph=50, refine_conf=0.71,
  )
  assert accepted_hud_sign(sign) is None
  h = LiveSignHold()
  assert not h.update([sign], 5.0)[0]
  live, mph, _ = h.update([sign], 5.4)
  assert not live and mph == 0
  live, mph, _ = h.update([sign], 8.0)
  assert not live and mph == 0


def test_same_timestamp_is_one_frame():
  """Class+OCR agreement on one frame is one vote, even if update is called twice."""
  sign = SpeedSign(
    mph=65, conf=0.60, bbox=(0, 0, 10, 10),
    class_mph=65, class_conf=0.60, refine_mph=65, refine_conf=0.60,
  )
  assert accepted_hud_sign(sign) is None
  h = LiveSignHold()
  assert not h.update([sign], 1.0)[0]
  assert not h.update([sign], 1.0)[0]
  live, mph, _ = h.update([sign], 2.0)
  assert live and mph == 65


def test_yolo_65_refine_agree_needs_a_second_frame():
  """Class and OCR both saying 65 below 0.70 is one vote. The next frame lights it."""
  sign = SpeedSign(
    mph=65, conf=0.60, bbox=(0, 0, 10, 10),
    class_mph=65, class_conf=0.60, refine_mph=65, refine_conf=0.60,
  )
  assert accepted_hud_sign(sign) is None
  h = LiveSignHold()
  live, mph, _ = h.update([sign], 1.0)
  assert not live and mph == 0
  live, mph, _ = h.update([sign], 2.0)
  assert live and mph == 65


def _sign(mph, conf, *, class_mph=None, refine_mph=None, refine_conf=0.0):
  return SpeedSign(
    mph=mph, conf=conf, bbox=(0, 0, 10, 10),
    class_mph=class_mph if class_mph is not None else mph,
    class_conf=conf,
    refine_mph=refine_mph, refine_conf=refine_conf,
  )


def test_held_50_not_overturned_by_weaker_agreed_65():
  """A different mph replaces only when it is at least as sure as the hold."""
  assert should_replace_held_mph(50, 0.50, 65, 0.50)
  assert not should_replace_held_mph(50, 0.53, 65, 0.47)
  assert not should_replace_held_mph(50, 0.60, 70, 0.55)
  h = LiveSignHold()
  pair50 = [_sign(50, 0.50, class_mph=50, refine_mph=50, refine_conf=0.50)]
  h.update(pair50, 1.0)
  live, mph, _ = h.update(pair50, 1.2)
  assert live and mph == 50
  weaker = _sign(65, 0.47, class_mph=65, refine_mph=65, refine_conf=0.47)
  assert accepted_hud_sign(weaker) is None
  live, mph, _ = h.update([weaker], 1.4)
  assert live and mph == 50


def test_held_50_not_overturned_by_one_read_of_40_60_70():
  """One class-only read never flashes 40/60/70. An agreed pair below the held conf stays 50."""
  h = LiveSignHold()
  pair50 = [_sign(50, 0.90, class_mph=50, refine_mph=50, refine_conf=0.90)]
  h.update(pair50, 1.0)
  live, mph, _ = h.update(pair50, 1.2)
  assert live and mph == 50
  for mph_j, conf in ((60, 0.51), (40, 0.48), (70, 0.73)):
    lone = _sign(mph_j, conf, class_mph=mph_j, refine_mph=None)
    assert accepted_hud_sign(lone) is None
    live, mph, _ = h.update([lone], 1.4)
    assert live and mph == 50
  weak70 = _sign(70, 0.55, class_mph=70, refine_mph=70, refine_conf=0.55)
  assert accepted_hud_sign(weak70) is None
  live, mph, _ = h.update([weak70], 1.6)
  assert live and mph == 50
  # Two frames that class and OCR both name, at least as sure as the hold, do light.
  real65 = _sign(65, 0.95, class_mph=65, refine_mph=65, refine_conf=0.95)
  live, mph, _ = h.update([real65], 1.8)
  assert live and mph == 50
  live, mph, _ = h.update([real65], 1.9)
  assert live and mph == 65


def test_refine_50_refreshes_held_50():
  h = LiveSignHold()
  h.update([_sign(50, 0.43, class_mph=50, refine_mph=50, refine_conf=0.43)], 1.0)
  live, mph, _ = h.update([_sign(50, 0.53, class_mph=50, refine_mph=50, refine_conf=0.53)], 2.0)
  assert live and mph == 50
  live, mph, _ = h.update([], 46.0)
  assert live and mph == 50


def test_sl_sighting_extends_hold_without_changing_mph():
  h = LiveSignHold(hold_s=3.0)
  pair = [_sign(50, 0.50, class_mph=50, refine_mph=50, refine_conf=0.50)]
  h.update(pair, 1.0)
  h.update(pair, 1.2)
  miss = _sign(65, 0.73, class_mph=65, refine_mph=None)
  assert is_speed_limit_sighting(miss)
  live, mph, _ = h.update([miss], 3.5)
  assert live and mph == 50
  live, mph, _ = h.update([], 6.4)
  assert live and mph == 50


def test_held_50_overturns_to_agreed_65():
  h = LiveSignHold()
  pair = [_sign(50, 0.50, class_mph=50, refine_mph=50, refine_conf=0.50)]
  h.update(pair, 1.0)
  h.update(pair, 1.2)
  strong = _sign(65, 0.80, class_mph=65, refine_mph=65, refine_conf=0.80)
  assert should_replace_held_mph(50, 0.50, 65, 0.80)
  live, mph, _ = h.update([strong], 1.4)
  assert live and mph == 50
  live, mph, _ = h.update([strong], 1.6)
  assert live and mph == 65


def test_held_65_yields_to_equally_sure_ocr_50():
  """A contradiction is not a vote. Two OCR-only 50s at least as sure replace 65."""
  assert not should_replace_held_mph(65, 0.80, 50, 0.43)
  assert should_replace_held_mph(65, 0.51, 50, 0.55)
  h = LiveSignHold()
  agreed = _sign(65, 0.80, class_mph=65, refine_mph=65, refine_conf=0.80)
  h.update([agreed], 1.0)
  h.update([agreed], 1.1)
  weak = [SpeedSign(
    mph=50, conf=0.43, bbox=(0, 0, 10, 10),
    class_mph=65, class_conf=0.80, refine_mph=50, refine_conf=0.43,
  )]
  h.update(weak, 1.2)
  live, mph, _ = h.update(weak, 1.4)
  assert live and mph == 65
  sure = [SpeedSign(
    mph=50, conf=0.85, bbox=(0, 0, 10, 10),
    class_mph=None, refine_mph=50, refine_conf=0.85,
  )]
  h.update(sure, 1.6)
  live, mph, _ = h.update(sure, 1.8)
  assert live and mph == 50


def test_expired_hold_needs_two_frames_and_ignores_one_class_read():
  """One unrefined 70 stays blank. One class==OCR 70 stays blank. The next frame lights."""
  h = LiveSignHold(hold_s=1.0)
  pair = [_sign(50, 0.50, class_mph=50, refine_mph=50, refine_conf=0.50)]
  h.update(pair, 0.0)
  h.update(pair, 0.2)
  # Hold expired. Empty publish must not re-arm from votes still inside 12 s.
  live, mph, _ = h.update([], 3.0)
  assert not live
  lone = [_sign(70, 0.60, class_mph=70, refine_mph=None)]
  live, mph, _ = h.update(lone, 3.1)
  assert not live and mph == 0
  agreed = [_sign(70, 0.51, class_mph=70, refine_mph=70, refine_conf=0.51)]
  live, mph, _ = h.update(agreed, 3.2)
  assert not live and mph == 0
  live, mph, _ = h.update(agreed, 3.4)
  assert live and mph == 70


def test_publish_fields_yolo_65_refine_fail_blank_or_hold():
  """Below 0.70, one unrefined class is not published. It does not replace a held 50."""
  h = LiveSignHold()
  low = [SpeedSign(
    mph=65, conf=0.60, bbox=(0, 0, 10, 10),
    class_mph=65, class_conf=0.60, refine_mph=None,
  )]
  blank = LiveSignHold()
  msg_valid, mph, conf, missing, paused = live_sign_publish_fields(blank, low, 1.0, False)
  # No accepted detect yet: cereal mph stays 0 (plate blank), same as no-hit.
  assert not missing and not paused and mph == 0 and conf == 0.0
  assert not msg_valid

  good = [SpeedSign(
    mph=50, conf=0.60, bbox=(0, 0, 10, 10),
    class_mph=50, class_conf=0.60, refine_mph=50, refine_conf=0.60,
  )]
  msg_valid, mph, _, missing, paused = live_sign_publish_fields(h, good, 2.0, False)
  assert mph == 0
  msg_valid, mph, _, missing, paused = live_sign_publish_fields(h, good, 2.4, False)
  assert msg_valid and mph == 50
  challenger = [SpeedSign(
    mph=65, conf=0.73, bbox=(0, 0, 10, 10),
    class_mph=65, class_conf=0.73, refine_mph=None,
  )]
  msg_valid, mph, _, missing, paused = live_sign_publish_fields(h, challenger, 3.0, False)
  assert msg_valid and mph == 50
  msg_valid, mph, _, missing, paused = live_sign_publish_fields(h, [], 4.0, False)
  assert msg_valid and mph == 50


def test_tici_plate_sits_on_driver_left_below_max():
  x, y = tici_sign_origin(0, 0)
  assert x + TICI_SIGN_W < 2160 / 2
  assert x < 200
  # Below the MAX box (y=45, h=204), not covering the center path.
  assert y >= 45 + 204
  yes, no = accuracy_button_rects(x, y, TICI_SIGN_W, TICI_SIGN_H, prompt_h=32)
  assert yes[0] + yes[2] < 2160 / 2
  assert no[0] + no[2] < 2160 / 2
  assert yes[3] == TICI_CONFIRM_BTN_H
  assert yes[1] >= y + TICI_SIGN_H
  assert no[1] > yes[1]


def test_mici_plate_sits_on_left():
  x, y = mici_sign_origin(0, 0)
  assert x < 536 / 2
  assert x < 20
  yes, no = accuracy_button_rects(x, y, 108, 128, beside=True)
  assert yes[0] > x
  assert yes[0] + yes[2] < 536 / 2


def test_confirm_visible_only_when_mph_valid():
  mph = live_sign_view(enabled=True, msg_valid=True, mph=55, valid=True, weights_missing=False)
  assert hud_confirm_visible(mph) and mph.confirm_visible
  nowt = live_sign_view(enabled=True, msg_valid=True, mph=0, valid=False, weights_missing=True)
  assert nowt.show and nowt.show_missing_weights
  assert not hud_confirm_visible(nowt) and not nowt.confirm_visible
  # Missing weights wins even if a leftover mph is present.
  fake = live_sign_view(enabled=True, msg_valid=True, mph=55, valid=True, weights_missing=True)
  assert not hud_confirm_visible(fake)
  hidden = live_sign_view(enabled=True, msg_valid=False, mph=0, valid=False)
  assert not hud_confirm_visible(hidden)
  assert not hud_confirm_visible(show_mph=True, mph=0)
  assert hud_confirm_visible(show_mph=True, mph=60)


def test_hit_accuracy_button_yes_no_and_miss():
  yes = (10.0, 20.0, 80.0, 40.0)
  no = (10.0, 70.0, 80.0, 40.0)
  assert hit_accuracy_button(15, 25, yes, no) is True
  assert hit_accuracy_button(50, 90, yes, no) is False
  assert hit_accuracy_button(0, 0, yes, no) is None
  assert hit_accuracy_button(200, 25, yes, no) is None


def test_cereal_live_sign_has_weights_missing_field():
  capnp = pytest.importorskip("capnp")
  from pathlib import Path
  capnp.remove_import_hook()
  custom_path = Path(__file__).resolve().parents[3] / "cereal" / "custom.capnp"
  custom = capnp.load(str(custom_path))
  msg = custom.LiveSpeedSignNAP.new_message()
  assert msg.mph == 0
  assert not msg.weightsMissing
  assert not msg.detectPaused
  apply_live_sign(msg, mph=0, conf=0.0, valid=False, weights_missing=True)
  assert msg.weightsMissing and msg.mph == 0 and not msg.valid and not msg.detectPaused
  apply_live_sign(msg, mph=0, conf=0.0, valid=False, detect_paused=True)
  assert msg.detectPaused and not msg.weightsMissing and msg.mph == 0


def test_ui_and_cereal_wire_live_sign():
  from pathlib import Path
  root = Path(__file__).resolve().parents[3]
  custom = (root / "cereal" / "custom.capnp").read_text(encoding="utf-8")
  log = (root / "cereal" / "log.capnp").read_text(encoding="utf-8")
  services = (root / "cereal" / "services.py").read_text(encoding="utf-8")
  ui = (root / "selfdrive" / "ui" / "ui_state.py").read_text(encoding="utf-8")
  tici = (root / "selfdrive" / "ui" / "onroad" / "hud_renderer.py").read_text(encoding="utf-8")
  mici = (root / "selfdrive" / "ui" / "mici" / "onroad" / "hud_renderer.py").read_text(encoding="utf-8")
  assert "struct LiveSpeedSignNAP" in custom
  assert "weightsMissing @3" in custom
  assert "detectPaused @4" in custom
  assert "liveSpeedSignNAP @108" in log
  assert '"liveSpeedSignNAP"' in services
  assert "liveSpeedSignNAP" in ui
  assert "SpeedSignHud" in tici
  assert "SpeedSignHud" in mici
  onroad = (root / "selfdrive" / "ui" / "onroad" / "speed_sign_hud.py").read_text(encoding="utf-8")
  assert "NO WT" in onroad
  assert "NO RT" in onroad
  assert "runtime_missing" in onroad
  assert "WAIT" in onroad
  assert "bug" in onroad.lower() or "swaglog" in onroad.lower()
  assert "live_sign_from_event" in onroad
  assert "on_confirm_accuracy" in onroad
  assert "draw_tici_speed_sign" in onroad
  assert "draw_mici_speed_sign" in onroad
  # Stub only: Yes/No must not write params / JSONL / sqlite / OSM in this PR.
  assert "Params(" not in onroad
  assert "append_jsonl" not in onroad
  assert "osm.org" in onroad  # mentioned as future work in the stub comment


def test_route_181_solo_reads_light_and_a_contradiction_clears():
  """45@0.60 stays dark. 0.77 / 0.75 / 0.89 / 0.81 light unless contradicted in 3 s."""
  samples = ((45, 0.60, False), (45, 0.77, True), (30, 0.75, True), (45, 0.89, True), (30, 0.81, True))
  for mph, conf, lights in samples:
    sign = SpeedSign(mph=mph, conf=conf, bbox=(0, 0, 10, 10), class_mph=mph, class_conf=conf)
    live, got, _ = LiveSignHold().update([sign], 1.0)
    assert live is lights
    if lights:
      assert got == mph
  h = LiveSignHold()
  lone = SpeedSign(mph=45, conf=0.89, bbox=(0, 0, 10, 10), class_mph=45, class_conf=0.89)
  assert h.update([lone], 0.0)[1] == 45
  assert h.update([], 0.0 + SOLO_CONTRADICT_S + 0.05)[0]
  assert h.update([], 0.0 + HUD_HOLD_S - 0.05)[0]
  assert not h.update([], 0.0 + HUD_HOLD_S + 0.05)[0]
  cleared = LiveSignHold()
  assert cleared.update([lone], 5.0)[0]
  weak = SpeedSign(mph=30, conf=0.50, bbox=(0, 0, 10, 10), class_mph=30, class_conf=0.50)
  live, mph, _ = cleared.update([weak], 5.0 + 1.0)
  assert not live and mph == 0
  swapped = LiveSignHold()
  swapped.update([lone], 8.0)
  other = SpeedSign(mph=30, conf=0.81, bbox=(0, 0, 10, 10), class_mph=30, class_conf=0.81)
  live, mph, _ = swapped.update([other], 8.0 + 1.0)
  assert live and mph == 30
  held = LiveSignHold()
  pair = [_sign(50, 0.60, class_mph=50, refine_mph=50, refine_conf=0.60)]
  held.update(pair, 1.0)
  held.update(pair, 1.2)
  live, mph, _ = held.update([other], 2.0)
  assert live and mph == 50
  clash = SpeedSign(
    mph=50, conf=0.90, bbox=(0, 0, 10, 10),
    class_mph=45, class_conf=0.90, refine_mph=50, refine_conf=0.90,
  )
  assert not LiveSignHold().update([clash], 1.0)[0]
