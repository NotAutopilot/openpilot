"""YOLOv8 ONNX interface: class map, letterbox, decode, no numpy fallback."""
from __future__ import annotations

import numpy as np
import pytest

from openpilot.selfdrive.speedsignd.detect import OnnxSpeedSignDetector, SpeedSign, SpeedSignDetector, paint_mutcd_r2_1
from openpilot.selfdrive.speedsignd.tests.test_detect import _scene
from openpilot.selfdrive.speedsignd.weights_manifest import (
  ASSET_BYTES,
  ASSET_SHA256,
  RELEASE_TAG,
  YOLO_CLASS_NAMES,
  YOLO_IMGSZ,
  YOLO_MIN_CONF,
  YOLO_OUTPUT_LAYOUT,
)
from openpilot.selfdrive.speedsignd.nv12 import Nv12DetectCrop
from openpilot.selfdrive.speedsignd.yolo import (
  PEAK_NAME_MIN,
  POSTED_LOG_MPH,
  REFINE_OVERRIDE_CONF,
  _resize_rgb,
  class_to_mph,
  decode_yolov8,
  is_negative_name,
  is_unreadable_name,
  letterbox_rgb,
  nms_xyxy,
  ocr_dropped_digit,
  prefer_refine,
  refine_mph,
  road_detect_crop,
  road_detect_crop_rect,
  speed_limit_class_indices,
  yolo_peak,
)

# v2 channel = 4 + class index. 0 sl_25 … 9 sl_70, 10 unreadable, 11/12 neg.
_SL_30 = 1
_SL_50 = 5
_SL_55 = 6
_SL_60 = 7
_SL_65 = 8
_SL_70 = 9
_SL_UNREADABLE = 10
_NEG_YELLOW = 11
_NEG_OTHER = 12
_NC = 17


def test_class_to_mph_reads_sl_and_speed_limit():
  assert class_to_mph("sl_55") == 55
  assert class_to_mph("sl_60") == 60
  assert class_to_mph("sl_25") == 25
  assert class_to_mph("speedLimit55") == 55
  assert class_to_mph("sl_unreadable") is None
  assert class_to_mph("neg_yellow") is None
  assert class_to_mph("neg_other") is None
  assert class_to_mph("sl_47") is None
  assert is_unreadable_name("sl_unreadable")
  assert is_negative_name("neg_yellow")
  assert not is_negative_name("sl_60")


def test_yolo_class_list_is_v2_speed_sign():
  assert RELEASE_TAG == "speed-sign-onnx-v2"
  assert ASSET_SHA256 == "1131200b7fe2d400596e09d7680fc3a26a08dcd8952acfdf261170bdb22d3d28"
  assert ASSET_BYTES == 12658953
  assert YOLO_OUTPUT_LAYOUT == (1, 17, 2100)
  assert YOLO_MIN_CONF == 0.40
  names = list(YOLO_CLASS_NAMES)
  assert names == [
    "sl_25", "sl_30", "sl_35", "sl_40", "sl_45", "sl_50", "sl_55",
    "sl_60", "sl_65", "sl_70", "sl_unreadable", "neg_yellow", "neg_other",
  ]
  assert len(names) == 13
  sl = speed_limit_class_indices()
  assert sl[0] == 0 and names[0] == "sl_25"
  assert sl[-1] == 9 and names[9] == "sl_70"
  assert _SL_UNREADABLE not in sl
  assert _NEG_YELLOW not in sl and _NEG_OTHER not in sl
  assert POSTED_LOG_MPH == (30, 50, 60, 65)
  assert PEAK_NAME_MIN == 0.05
  assert REFINE_OVERRIDE_CONF == 0.28


def test_letterbox_keeps_aspect_and_pads():
  rgb = np.zeros((120, 320, 3), np.uint8)
  rgb[:, :] = (10, 20, 30)
  boxed, scale, pad_x, pad_y = letterbox_rgb(rgb, 320)
  assert boxed.shape == (320, 320, 3)
  assert scale == pytest.approx(1.0)
  assert pad_x == 0
  assert pad_y == 100
  assert boxed[pad_y, 10, 0] == 10


def test_resize_rgb_is_bilinear_not_nearest():
  """Nearest index-sample of 2×2→1×1 picks a corner; bilinear averages."""
  img = np.zeros((2, 2, 3), np.uint8)
  img[0, 0] = 0
  img[0, 1] = 100
  img[1, 0] = 0
  img[1, 1] = 100
  out = _resize_rgb(img, 1, 1)
  assert 40 <= int(out[0, 0, 0]) <= 60


def test_road_detect_crop_is_the_native_right_window():
  rgb = np.zeros((1208, 1928, 3), np.uint8)
  crop, box = road_detect_crop(rgb)
  x, y, w, h = box
  assert crop.shape == (320, 628, 3)
  assert (x, y, w, h) == (1300, 520, 628, 320)
  # Width limits the letterbox. A 60 px sign stays ≥ 30 px at imgsz 320.
  assert 60 * (YOLO_IMGSZ / float(w)) >= 30.0
  assert x > 964
  assert road_detect_crop_rect(1208, 1928) == (1300, 520, 628, 320)


def test_yolo_peak_reports_below_threshold_class():
  raw = np.zeros((1, _NC, 4), np.float32)
  raw[0, 4 + _NEG_YELLOW, 0] = 0.33
  peak = yolo_peak(raw)
  assert peak.shape == (1, _NC, 4)
  assert peak.conf == pytest.approx(0.33, abs=1e-5)
  assert peak.name == "neg_yellow"
  assert peak.n_over == 0
  assert peak.sl_conf == pytest.approx(0.0, abs=1e-5)
  assert peak.n_over_sl == 0


def test_yolo_peak_splits_negative_from_speed_limit_and_logs_top3():
  """A strong neg_* must not be reported as the speed-limit peak."""
  raw = np.zeros((1, _NC, 4), np.float32)
  raw[0, 4 + _NEG_YELLOW, 0] = 0.91
  raw[0, 4 + _NEG_OTHER, 0] = 0.22
  raw[0, 4 + _SL_55, 1] = 0.12
  raw[0, 4 + 0, 1] = 0.08  # sl_25
  peak = yolo_peak(raw)
  assert peak.name == "neg_yellow"
  assert peak.conf == pytest.approx(0.91, abs=1e-5)
  assert peak.n_over == 1
  assert peak.sl_name == "sl_55"
  assert peak.sl_conf == pytest.approx(0.12, abs=1e-5)
  assert peak.n_over_sl == 0
  assert [n for n, _c in peak.top3] == ["neg_yellow", "neg_other", "sl_55"]
  assert peak.top3[0][1] == pytest.approx(0.91, abs=1e-5)
  assert peak.top3[2][1] == pytest.approx(0.12, abs=1e-5)
  assert dict(peak.posted)[30] == pytest.approx(0.0, abs=1e-5)
  assert dict(peak.posted)[50] == pytest.approx(0.0, abs=1e-5)
  assert dict(peak.posted)[60] == pytest.approx(0.0, abs=1e-5)
  assert dict(peak.posted)[65] == pytest.approx(0.0, abs=1e-5)


def test_yolo_peak_blank_name_on_argmax_noise():
  """A ~0 argmax is not a read."""
  raw = np.zeros((1, _NC, 4), np.float32)
  raw[0, 4 + _SL_65, 0] = 0.004
  raw[0, 4 + _SL_60, 0] = 0.003
  raw[0, 4 + _SL_50, 0] = 0.002
  raw[0, 4 + _SL_30, 0] = 0.001
  peak = yolo_peak(raw)
  assert peak.conf == pytest.approx(0.004, abs=1e-5)
  assert peak.name == ""
  assert peak.sl_name == ""
  assert peak.n_over == 0
  assert peak.top3 == ()
  assert dict(peak.posted) == {30: pytest.approx(0.001, abs=1e-5),
                               50: pytest.approx(0.002, abs=1e-5),
                               60: pytest.approx(0.003, abs=1e-5),
                               65: pytest.approx(0.004, abs=1e-5)}


def test_nms_keeps_highest_score():
  boxes = np.array([
    [0, 0, 10, 10],
    [1, 1, 11, 11],
    [50, 50, 60, 60],
  ], dtype=np.float32)
  scores = np.array([0.4, 0.9, 0.8], dtype=np.float32)
  keep = nms_xyxy(boxes, scores, iou_thr=0.3, max_det=3)
  assert keep[0] == 1
  assert 2 in keep
  assert 0 not in keep


def test_decode_yolov8_posted_30_50_60():
  """Class map for the v2 ONNX: sl_30, sl_50, sl_60."""
  for mph, idx in ((30, _SL_30), (50, _SL_50), (60, _SL_60)):
    raw = np.zeros((1, _NC, 4), np.float32)
    raw[0, :4, 0] = [160, 120, 80, 100]
    raw[0, 4 + idx, 0] = 0.88
    hits = decode_yolov8(raw, scale=1.0, pad_x=0, pad_y=0, src_hw=(320, 320), min_conf=0.4)
    assert len(hits) == 1 and hits[0].mph == mph
    assert hits[0].class_mph == mph


def test_decode_yolov8_speed_limit_55():
  raw = np.zeros((1, _NC, 8), np.float32)
  raw[0, 0, 0] = 160.0
  raw[0, 1, 0] = 120.0
  raw[0, 2, 0] = 80.0
  raw[0, 3, 0] = 100.0
  raw[0, 4 + _SL_55, 0] = 0.91
  hits = decode_yolov8(raw, scale=1.0, pad_x=0, pad_y=0, src_hw=(320, 320), min_conf=0.4)
  assert len(hits) == 1
  assert hits[0].mph == 55
  assert hits[0].conf == pytest.approx(0.91, abs=1e-5)
  x, y, w, h = hits[0].bbox
  assert x == 120 and y == 70
  assert w == 80 and h == 100


def test_decode_yolov8_accepts_extra_singleton_dim():
  """tinygrad OnnxRunner has returned [1,1,C,N]; old _as_cn dropped that as empty."""
  raw = np.zeros((1, 1, _NC, 8), np.float32)
  raw[0, 0, 0, 0] = 160.0
  raw[0, 0, 1, 0] = 120.0
  raw[0, 0, 2, 0] = 80.0
  raw[0, 0, 3, 0] = 100.0
  raw[0, 0, 4 + _SL_55, 0] = 0.91
  hits = decode_yolov8(raw, scale=1.0, pad_x=0, pad_y=0, src_hw=(320, 320), min_conf=0.4)
  assert len(hits) == 1 and hits[0].mph == 55


def test_decode_yolov8_ignores_neg_and_low_conf():
  raw = np.zeros((1, _NC, 4), np.float32)
  raw[0, :4, 0] = [80, 80, 40, 40]
  raw[0, 4 + _NEG_YELLOW, 0] = 0.99
  raw[0, :4, 1] = [200, 80, 40, 40]
  raw[0, 4 + _SL_60, 1] = 0.2
  assert decode_yolov8(raw, scale=1.0, pad_x=0, pad_y=0, src_hw=(320, 320), min_conf=0.4) == []


def test_decode_yolov8_keeps_speed_limit_under_stronger_negative():
  """neg_yellow must not hide an sl_55 on the same anchor."""
  raw = np.zeros((1, _NC, 2), np.float32)
  raw[0, :4, 0] = [160, 120, 80, 100]
  raw[0, 4 + _NEG_YELLOW, 0] = 0.91
  raw[0, 4 + _SL_55, 0] = 0.50
  hits = decode_yolov8(raw, scale=1.0, pad_x=0, pad_y=0, src_hw=(320, 320), min_conf=0.4)
  assert len(hits) == 1
  assert hits[0].mph == 55
  assert hits[0].conf == pytest.approx(0.50, abs=1e-5)


def test_decode_unreadable_is_a_box_not_an_mph():
  raw = np.zeros((1, _NC, 2), np.float32)
  raw[0, :4, 0] = [160, 120, 80, 100]
  raw[0, 4 + _SL_UNREADABLE, 0] = 0.80
  hits = decode_yolov8(raw, scale=1.0, pad_x=0, pad_y=0, src_hw=(320, 320), min_conf=0.4)
  assert len(hits) == 1
  assert hits[0].mph == 0
  assert hits[0].class_mph is None
  assert hits[0].bbox[2] > 0
  out = refine_mph(hits[0], np.zeros((320, 320), np.uint8), lambda _c: (55, 0.90))
  assert out.mph == 0
  assert out.refine_mph is None
  assert out.class_mph is None


def test_decode_neg_other_alone_is_dropped():
  raw = np.zeros((1, _NC, 2), np.float32)
  raw[0, :4, 0] = [160, 120, 80, 100]
  raw[0, 4 + _NEG_OTHER, 0] = 0.95
  assert decode_yolov8(raw, scale=1.0, pad_x=0, pad_y=0, src_hw=(320, 320), min_conf=0.4) == []


class _YoloSess:
  def get_inputs(self):
    return [type("I", (), {"name": "images", "shape": [1, 3, 320, 320]})()]

  def run(self, _out, feed):
    blob = next(iter(feed.values()))
    assert blob.shape == (1, 3, 320, 320)
    raw = np.zeros((1, _NC, 4), np.float32)
    raw[0, 0, 0] = 200.0
    raw[0, 1, 0] = 80.0
    raw[0, 2, 0] = 60.0
    raw[0, 3, 0] = 80.0
    raw[0, 4 + _SL_60, 0] = 0.88
    return [raw]


def test_onnx_yolo_session_returns_60():
  det = OnnxSpeedSignDetector("/tmp/fake.onnx", _YoloSess())
  y = np.zeros((320, 320), np.uint8)
  hits = det.detect(y, min_conf=0.4)
  assert hits and hits[0].mph == 60
  assert hits[0].conf == pytest.approx(0.88, abs=1e-5)


def test_yolo_crop_offsets_bbox_to_full_road_frame():
  det = OnnxSpeedSignDetector("/tmp/fake.onnx", _YoloSess(), backend="tinygrad")
  rgb = np.zeros((1208, 1928, 3), np.uint8)
  hits = det.detect(None, min_conf=0.4, rgb=rgb)
  assert hits and hits[0].mph == 60
  x, _y, _w, _h = hits[0].bbox
  assert x >= 1100
  d = det.diag_dict()
  assert d["frame_w"] == 1928 and d["frame_h"] == 1208
  assert d["crop"] == (1300, 520, 628, 320)
  assert d["backend"] == "tinygrad"


def test_onnx_records_peak_when_decode_empty():
  class NearMiss(_YoloSess):
    def run(self, _out, _feed):
      raw = np.zeros((1, _NC, 4), np.float32)
      raw[0, :4, 0] = [80, 80, 40, 40]
      raw[0, 4 + _NEG_YELLOW, 0] = 0.33
      return [raw]

  det = OnnxSpeedSignDetector("/tmp/fake.onnx", NearMiss(), backend="tinygrad")
  y = np.zeros((320, 320), np.uint8)
  assert det.detect(y, min_conf=0.4) == []
  d = det.diag_dict()
  assert d["backend"] == "tinygrad"
  assert d["frame_w"] == 320 and d["frame_h"] == 320
  assert d["letterbox"] == 320
  assert d["peak_name"] == "neg_yellow"
  assert d["peak_conf"] == pytest.approx(0.33, abs=1e-5)
  assert d["n_over"] == 0
  assert d["sl_peak_conf"] == pytest.approx(0.0, abs=1e-5)
  assert d["n_over_sl"] == 0
  assert d["top3"][0][0] == "neg_yellow"
  assert d["error"] == ""


def test_onnx_detect_surfaces_session_error():
  class Boom(_YoloSess):
    def run(self, _out, _feed):
      raise RuntimeError("tinygrad layout")

  det = OnnxSpeedSignDetector("/tmp/fake.onnx", Boom(), backend="tinygrad")
  assert det.detect(np.zeros((64, 64), np.uint8)) == []
  assert "tinygrad layout" in det.diag_dict()["error"]


def test_loaded_onnx_does_not_fall_back_to_numpy(tmp_path):
  class EmptyYolo(_YoloSess):
    def run(self, _out, _feed):
      return [np.zeros((1, _NC, 4), np.float32)]

  y = _scene()
  paint_mutcd_r2_1(y, 45, x=200, y=30, w=90, h=112)
  det = SpeedSignDetector(onnx=OnnxSpeedSignDetector(str(tmp_path / "x.onnx"), EmptyYolo()))
  assert det.onnx is not None
  assert det.detect(y) == []


def test_refine_overrides_wrong_yolo_class_on_clear_crop():
  from openpilot.selfdrive.speedsignd.detect import _read_mph
  y = _scene()
  paint_mutcd_r2_1(y, 55, x=200, y=30, w=90, h=112)
  wrong = SpeedSign(mph=65, conf=0.57, bbox=(200, 30, 90, 112))
  out = refine_mph(wrong, y, _read_mph)
  assert out.mph == 55


def test_refine_prefers_50_when_class_says_65():
  """Parked close R2-1 50: YOLO class is 65; crop digits are 50."""
  from openpilot.selfdrive.speedsignd.detect import _read_mph
  y = _scene(seed=50)
  paint_mutcd_r2_1(y, 50, x=200, y=30, w=90, h=112)
  wrong = SpeedSign(
    mph=65, conf=0.74, bbox=(200, 30, 90, 112),
    class_mph=65, class_conf=0.74, alt_mph=None, alt_conf=0.0,
  )
  out = refine_mph(wrong, y, _read_mph)
  assert out.mph == 50
  assert out.class_mph == 65
  assert out.refine_mph == 50
  assert out.refine_conf >= REFINE_OVERRIDE_CONF


def test_refine_overrides_confident_65_when_class_50_is_zero():
  """Justin's parked close 50: top=65:0.73, cls=50:0.00 — not a close race."""
  wrong = SpeedSign(
    mph=65, conf=0.73, bbox=(10, 10, 40, 50),
    class_mph=65, class_conf=0.73, alt_mph=70, alt_conf=0.11,
  )
  out = refine_mph(wrong, np.zeros((80, 80), np.uint8), lambda _c: (50, 0.40))
  assert out.mph == 50
  assert out.class_mph == 65
  assert out.refine_mph == 50
  assert prefer_refine(wrong, 50, 0.40)


def test_refine_keeps_class_when_crop_read_is_none():
  wrong = SpeedSign(mph=65, conf=0.73, bbox=(10, 10, 40, 50), class_mph=65, class_conf=0.73)
  out = refine_mph(wrong, np.zeros((80, 80), np.uint8), lambda _c: (None, 0.0))
  assert out.mph == 65
  assert out.refine_mph is None


def test_refine_prefers_any_confident_disagreement():
  """Not only 50↔65 — any confident crop mph overrides the class."""
  sign = SpeedSign(mph=35, conf=0.70, bbox=(10, 10, 40, 50), class_mph=35, class_conf=0.70)
  out = refine_mph(sign, np.zeros((80, 80), np.uint8), lambda _c: (55, 0.40))
  assert out.mph == 55
  assert out.refine_mph == 55


def test_decode_65_with_50_head_at_zero():
  """sl_65 0.74, sl_50 ~0. No class-margin rescue. HUD still needs a second frame."""
  raw = np.zeros((1, _NC, 4), np.float32)
  raw[0, :4, 0] = [160, 120, 80, 100]
  raw[0, 4 + _SL_65, 0] = 0.74
  raw[0, 4 + _SL_50, 0] = 0.004
  hits = decode_yolov8(raw, scale=1.0, pad_x=0, pad_y=0, src_hw=(320, 320), min_conf=0.4)
  assert len(hits) == 1
  assert hits[0].mph == 65
  assert hits[0].class_mph == 65
  assert hits[0].alt_mph != 50 or hits[0].alt_conf < 0.05


def test_onnx_detect_overrides_65_class_on_painted_50():
  """refine_mph still overrides a confident 65 when the crop reads 50.

  That contradiction is not a HUD vote — see test_hud.
  """
  class Sess65(_YoloSess):
    def run(self, _out, _feed):
      raw = np.zeros((1, _NC, 4), np.float32)
      raw[0, :4, 0] = [245, 86, 90, 112]
      raw[0, 4 + _SL_65, 0] = 0.74
      raw[0, 4 + _SL_50, 0] = 0.004
      raw[0, 4 + _SL_70, 0] = 0.11
      return [raw]

  y = _scene(h=320, w=320)
  paint_mutcd_r2_1(y, 50, x=200, y=30, w=90, h=112)
  det = OnnxSpeedSignDetector("/tmp/fake.onnx", Sess65(), backend="tinygrad")
  hits = det.detect(y, min_conf=0.4)
  assert hits and hits[0].mph == 50
  assert hits[0].class_mph == 65
  assert hits[0].refine_mph == 50
  d = det.diag_dict()
  assert d["refine"]
  class_mph, hud_mph, ref_mph, ref_conf = d["refine"][0]
  assert class_mph == 65 and hud_mph == 50 and ref_mph == 50
  assert ref_conf > 0.0


def test_yolo_min_conf_constant():
  assert YOLO_MIN_CONF == 0.40


def test_refine_never_turns_a_valid_class_into_a_dropped_digit():
  """Route 00000181: OCR produced 5 from a 45. The class must stay 45."""
  sign = SpeedSign(
    mph=45, conf=0.89, bbox=(10, 10, 40, 50),
    class_mph=45, class_conf=0.89,
  )
  y = np.zeros((80, 80), np.uint8)
  for bad in (5, 4, 46, 100, None):
    out = refine_mph(sign, y, lambda _c, bad=bad: (bad, 0.95))
    assert out.mph == 45
    assert out.class_mph == 45
    assert out.refine_mph is None
  assert ocr_dropped_digit(45, 5)
  assert ocr_dropped_digit(45, 4)
  assert not ocr_dropped_digit(65, 50)
  assert not ocr_dropped_digit(35, 55)
  assert not prefer_refine(sign, 5, 0.95)
  assert not prefer_refine(sign, 100, 0.95)
  kept = refine_mph(sign, y, lambda _c: (40, 0.90))
  assert kept.mph == 40 and kept.class_mph == 45 and kept.refine_mph == 40


def test_onnx_nv12_crop_path_offsets_and_chroma():
  """On-car detect uses the crop copy, not a full-frame RGB convert."""
  det = OnnxSpeedSignDetector("/tmp/fake.onnx", _YoloSess(), backend="tinygrad")
  y = np.full((320, 320), 88, np.uint8)
  u = np.full((160, 160), 128, np.uint8)
  v = np.full((160, 160), 128, np.uint8)
  uv = np.empty((160, 320), np.uint8)
  uv[:, 0::2] = u
  uv[:, 1::2] = v
  nv12 = Nv12DetectCrop(y=y, uv=uv, frame_w=1928, frame_h=1208, crop=(1100, 325, 320, 320))
  hits = det.detect(None, min_conf=0.4, nv12=nv12)
  assert hits and hits[0].mph == 60
  assert hits[0].bbox[0] >= 1100
  d = det.diag_dict()
  assert d["backend"] == "tinygrad"
  assert d["frame_w"] == 1928 and d["frame_h"] == 1208
  assert d["crop"] == (1100, 325, 320, 320)  # caller-supplied crop, not the live window
  assert d["letterbox"] == 320
  assert d["chroma"] == 1
  assert d["luma_mean"] == pytest.approx(88.0, abs=1.0)
  assert d["prep_ms"] >= 0.0
  assert d["sess_ms"] >= 0.0


@pytest.mark.skipif(
  not __import__("os").path.isfile("/tmp/speedsign/best.onnx"),
  reason="exported ONNX not in this environment",
)
def test_real_onnx_empty_road_is_quiet():
  ort = pytest.importorskip("onnxruntime")
  from openpilot.selfdrive.speedsignd.detect import OnnxSpeedSignDetector
  sess = ort.InferenceSession("/tmp/speedsign/best.onnx", providers=["CPUExecutionProvider"])
  det = OnnxSpeedSignDetector("/tmp/speedsign/best.onnx", sess)
  rgb = np.zeros((240, 320, 3), np.uint8)
  rgb[:80] = (140, 170, 200)
  rgb[80:] = (45, 45, 40)
  assert det.detect(None, min_conf=0.40, rgb=rgb) == []
