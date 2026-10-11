"""JSONL confirms on two agreeing reads. One class-only read does not."""
from __future__ import annotations

from openpilot.selfdrive.speedsignd.debounce import SignDebounce
from openpilot.selfdrive.speedsignd.detect import SpeedSign
from openpilot.selfdrive.speedsignd.jsonl import JsonlLogger
from openpilot.selfdrive.speedsignd.speedsignd import SPEEDSIGND_HZ, next_detect_mono, process_frame
from openpilot.selfdrive.speedsignd.weights_manifest import DEBOUNCE_HITS, DEBOUNCE_WINDOW_S


def test_single_frame_is_not_enough():
  d = SignDebounce()
  signs = [SpeedSign(mph=55, conf=0.9, bbox=(1, 1, 10, 10))]
  assert d.update(signs, 0.0) == []


def test_two_agreeing_frames_confirm():
  d = SignDebounce()
  s = SpeedSign(mph=55, conf=0.8, bbox=(2, 2, 12, 12))
  assert d.update([s], 0.0) == []
  # 1 Hz spacing (and a skipped cycle at ~2 s) must still confirm.
  out = d.update([s], 1.0)
  assert len(out) == 1 and out[0].mph == 55


def test_two_hits_after_skip_still_confirm():
  d = SignDebounce()
  s = SpeedSign(mph=55, conf=0.85, bbox=(0, 0, 8, 8))
  assert d.update([s], 0.0) == []
  out = d.update([s], 2.0)
  assert len(out) == 1 and out[0].mph == 55


def test_disagreement_does_not_confirm():
  d = SignDebounce()
  d.update([SpeedSign(mph=55, conf=0.9, bbox=(0, 0, 1, 1))], 0.0)
  assert d.update([SpeedSign(mph=60, conf=0.9, bbox=(0, 0, 1, 1))], 1.0) == []


def test_stale_hit_expires():
  d = SignDebounce()
  s = SpeedSign(mph=55, conf=0.9, bbox=(0, 0, 1, 1))
  d.update([s], 0.0)
  assert d.update([s], DEBOUNCE_WINDOW_S + 0.01) == []


def test_low_conf_ignored():
  d = SignDebounce()
  s = SpeedSign(mph=55, conf=0.2, bbox=(0, 0, 1, 1))
  d.update([s], 0.0)
  assert d.update([s], 1.0) == []


def test_process_frame_debounce_blocks_jsonl(tmp_path):
  class _Det:
    onnx = None

    def detect(self, y, min_conf=None, rgb=None, nv12=None):
      return [SpeedSign(mph=55, conf=0.9, bbox=(0, 0, 8, 8))]

  log = JsonlLogger(str(tmp_path / "out.jsonl"))
  debounce = SignDebounce()
  y = __import__("numpy").zeros((32, 32), __import__("numpy").uint8)
  signs, written = process_frame(y, 45.0, -95.0, 0.0, True, _Det(), log, now=1.0, debounce=debounce)
  # First in-threshold hit lights HUD; JSONL still waits for the second frame.
  assert signs and signs[0].mph == 55
  assert written == []
  signs, written = process_frame(y, 45.0, -95.0, 0.0, True, _Det(), log, now=2.0, debounce=debounce)
  assert signs and signs[0].mph == 55
  assert written and written[0]["mph"] == 55


def test_update_split_hud_on_first_hit():
  d = SignDebounce()
  s = SpeedSign(mph=55, conf=0.72, bbox=(0, 0, 8, 8))
  split = d.update_split([s], 0.0)
  assert split.hud and split.hud[0].mph == 55
  assert split.confirmed == []
  split = d.update_split([s], 1.0)
  assert split.hud and split.confirmed
  assert split.confirmed[0].mph == 55


def test_duty_cycle_gap_two_yolo_reads_confirm_inside_12s():
  """A 2.1 s infer waits 8.4 s. That fits the 12 s window, so the next YOLO confirms.

  A class/OCR contradiction on the second look does not.
  """
  period, budget = 1.0 / SPEEDSIGND_HZ, 0.100
  infer_s = 2.1
  second_t = next_detect_mono(infer_s, infer_s, period, budget)
  assert abs(second_t - infer_s * 4.0) < 1e-9
  assert second_t < DEBOUNCE_WINDOW_S
  d = SignDebounce()
  first_sign = SpeedSign(
    mph=55, conf=0.8, bbox=(0, 0, 8, 8),
    class_mph=55, class_conf=0.8, refine_mph=55, refine_conf=0.8,
  )
  first = d.update_split([first_sign], 0.0)
  assert first.hud and first.confirmed == []
  clash = SpeedSign(
    mph=55, conf=0.62, bbox=(0, 0, 8, 8),
    class_mph=65, class_conf=0.70, refine_mph=55, refine_conf=0.62,
  )
  assert d.update_split([clash], 0.1).confirmed == []
  late = d.update_split([first_sign], second_t)
  assert late.confirmed and late.confirmed[0].mph == 55
  # A 4 s infer gaps 16 s, past the window: a miss, not a one-frame plate.
  slow = next_detect_mono(4.0, 4.0, period, budget)
  assert slow > DEBOUNCE_WINDOW_S


def test_debounce_constants_match_duty_gap():
  assert SPEEDSIGND_HZ == 1.0
  assert DEBOUNCE_HITS == 2
  assert DEBOUNCE_WINDOW_S == 12.0
  # Two duty-cycled reads at a ~3 s infer (12 s apart) still fit.
  assert DEBOUNCE_WINDOW_S >= 4.0 * 3.0
  assert DEBOUNCE_WINDOW_S >= 2.0 / SPEEDSIGND_HZ


def test_update_split_one_unrefined_read_does_not_confirm():
  """Class-only ≥0.70 is handed to the plate. It still does not write JSONL."""
  d = SignDebounce()
  for i, mph in enumerate((40, 60, 65, 70)):
    bad = SpeedSign(mph=mph, conf=0.73, bbox=(0, 0, 8, 8), class_mph=mph, class_conf=0.73)
    split = d.update_split([bad], float(i) * 3.0)
    assert split.hud and split.hud[0].mph == mph
    assert split.confirmed == []
  low = SpeedSign(mph=45, conf=0.60, bbox=(0, 0, 8, 8), class_mph=45, class_conf=0.60)
  assert d.update_split([low], 100.0).hud == []
  assert d.update_split([low], 101.0).confirmed == []


def test_contradiction_does_not_confirm_and_two_ocr_frames_do():
  d = SignDebounce()
  mixed = SpeedSign(
    mph=50, conf=0.71, bbox=(0, 0, 8, 8),
    class_mph=65, class_conf=0.82, refine_mph=50, refine_conf=0.71,
  )
  assert d.update_split([mixed], 0.0).confirmed == []
  assert d.update_split([mixed], 0.2).confirmed == []
  ocr = SpeedSign(
    mph=50, conf=0.71, bbox=(0, 0, 8, 8),
    class_mph=None, refine_mph=50, refine_conf=0.71,
  )
  assert d.update_split([ocr], 1.0).confirmed == []
  split = d.update_split([ocr], 1.2)
  assert split.confirmed and split.confirmed[0].mph == 50


def test_update_split_one_unrefined_65_is_not_confirmed():
  d = SignDebounce()
  bad = SpeedSign(
    mph=65, conf=0.73, bbox=(0, 0, 8, 8),
    class_mph=65, class_conf=0.73, refine_mph=None,
  )
  split = d.update_split([bad], 0.0)
  assert split.hud and split.hud[0].mph == 65 and split.confirmed == []
  assert d.last_raw and d.last_raw[0].mph == 65
  again = d.update_split([bad], 0.3)
  assert again.confirmed == []
  assert again.hud and again.hud[0].mph == 65


def test_unreadable_and_negative_never_confirm():
  d = SignDebounce()
  unread = SpeedSign(mph=0, conf=0.90, bbox=(1, 1, 20, 20), class_mph=None)
  neg = SpeedSign(mph=0, conf=0.90, bbox=(1, 1, 20, 20), class_mph=None, refine_mph=None)
  assert d.update_split([unread], 0.0).confirmed == []
  assert d.update_split([unread, neg], 1.0).confirmed == []
  assert d.last_raw == []


def test_update_split_contradiction_is_not_a_hud_vote():
  d = SignDebounce()
  s = SpeedSign(
    mph=50, conf=0.71, bbox=(0, 0, 8, 8),
    class_mph=65, class_conf=0.82, refine_mph=50, refine_conf=0.71,
  )
  split = d.update_split([s], 0.0)
  assert split.hud == []
  assert split.confirmed == []
