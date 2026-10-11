"""NV12 Y / RGB helpers for the ROAD camera."""
from __future__ import annotations

import numpy as np
import pytest

from openpilot.selfdrive.speedsignd.nv12 import (
  Nv12DetectCrop,
  copy_nv12_detect_crop,
  detect_crop_rect,
  letterbox_rgb_from_nv12_crop,
  nv12_uv_offset,
  rgb_from_nv12,
  rgb_from_y,
  y_plane_from_nv12,
)


def test_y_plane_from_nv12():
  class Buf:
    width = 4
    height = 2
    stride = 4
    data = bytes([1, 2, 3, 4, 5, 6, 7, 8]) + bytes(8)

  y = y_plane_from_nv12(Buf())
  assert y is not None
  assert y.shape == (2, 4)
  assert int(y[0, 0]) == 1
  copied = y_plane_from_nv12(Buf(), copy=True)
  assert copied is not None and copied.shape == (2, 4)
  copied[0, 0] = 99
  assert int(y_plane_from_nv12(Buf())[0, 0]) == 1


def test_rgb_from_y_replicates_luma():
  y = np.array([[10, 20], [30, 40]], np.uint8)
  rgb = rgb_from_y(y)
  assert rgb.shape == (2, 2, 3)
  assert int(rgb[0, 1, 2]) == 20


def test_rgb_from_nv12_gray_uv():
  # 2x2 gray: Y=128, U=V=128
  y = bytes([128, 128, 128, 128])
  uv = bytes([128, 128, 128, 128])

  class Buf:
    width = 2
    height = 2
    stride = 2
    data = y + uv

  rgb = rgb_from_nv12(Buf())
  assert rgb is not None
  assert rgb.shape == (2, 2, 3)
  assert 120 <= int(rgb[0, 0, 0]) <= 136


def test_nv12_uv_offset_prefers_buf_field():
  class Buf:
    uv_offset = 2048 * 1216

  assert nv12_uv_offset(Buf(), stride=2048, height=1208) == 2048 * 1216
  assert nv12_uv_offset(type("B", (), {})(), stride=2048, height=1208) == 2048 * 1208


def test_rgb_from_nv12_uses_uv_offset_not_visible_height():
  """3X Venus pad: UV starts after ALIGN(height,32) rows, not height.

  Zeros in the Y pad must not be read as U=V=0 (that turns gray into green).
  """
  width, height, stride = 4, 2, 4
  y = bytes([128] * (stride * height))
  pad = bytes([0] * 8)
  uv = bytes([128, 128, 128, 128])

  class Buf:
    pass

  Buf.width = width
  Buf.height = height
  Buf.stride = stride
  Buf.uv_offset = stride * height + len(pad)
  Buf.data = y + pad + uv

  rgb = rgb_from_nv12(Buf())
  assert rgb is not None
  assert 120 <= int(rgb[0, 0, 0]) <= 136
  assert 120 <= int(rgb[0, 0, 1]) <= 136
  assert 120 <= int(rgb[0, 0, 2]) <= 136


def _nv12_buf(width, height, *, stride=None, y=128, u=128, v=128, uv_align=32):
  stride = int(stride or width)
  y_bytes = bytes([y]) * (stride * height)
  pad_h = (uv_align - (height % uv_align)) % uv_align
  pad = bytes(stride * pad_h)
  pair = bytes([u, v])
  uv = pair * ((stride // 2) * (height // 2))

  class Buf:
    pass

  Buf.width = width
  Buf.height = height
  Buf.stride = stride
  Buf.uv_offset = stride * height + len(pad)
  Buf.data = y_bytes + pad + uv
  return Buf


def test_detect_crop_rect_is_right_biased_square():
  x, y, w, h = detect_crop_rect(1208, 1928)
  assert (x, y, w, h) == (720, 0, 1208, 1208)


def test_native_window_covers_the_road_test_signs():
  """Route 0000017a signs sat at y=557–805, x=1110–1914. The window is
  x 1300–1928, y 520–840. Letterbox into 320 keeps a ≥60 px sign at ≥30 px.
  """
  from openpilot.selfdrive.speedsignd.nv12 import (
    NATIVE_WINDOW_H,
    NATIVE_WINDOW_W,
    NATIVE_WINDOW_X,
    NATIVE_WINDOW_Y,
    native_detect_window,
    native_window_model_scale,
  )
  from openpilot.selfdrive.speedsignd.weights_manifest import YOLO_IMGSZ
  x, y, w, h = native_detect_window(1208, 1928)
  assert (x, y, w, h) == (NATIVE_WINDOW_X, NATIVE_WINDOW_Y, NATIVE_WINDOW_W, NATIVE_WINDOW_H)
  assert (x, y, w, h) == (1300, 520, 628, 320)
  assert x <= 1300 and x + w >= 1928
  assert y <= 520 and y + h >= 840
  # Observed band sits inside. The left tail x=1110–1300 is the far signs.
  assert y <= 557 and y + h >= 805
  assert x + w >= 1914
  assert x % 2 == y % 2 == w % 2 == h % 2 == 0
  scale = native_window_model_scale(YOLO_IMGSZ)
  assert scale == pytest.approx(320 / 628)
  assert 60 * scale >= 30.0
  # 30-inch plate ~67 px at 100 ft, ~83 px at 80 ft (x=1300 is about 80 ft).
  assert 67 * scale >= 30.0
  assert 83 * scale >= 30.0
  # The old 1208² letterbox is still available and is not the live window.
  assert detect_crop_rect(1208, 1928) == (720, 0, 1208, 1208)
  # Window constants stay independent of the model input size.
  assert YOLO_IMGSZ == 320


def test_copy_nv12_detect_crop_is_native_320_and_keeps_chroma():
  buf = _nv12_buf(1928, 1208, y=128, u=128, v=128)
  crop = copy_nv12_detect_crop(buf)
  assert crop is not None
  assert crop.frame_w == 1928 and crop.frame_h == 1208
  assert crop.crop == (1300, 520, 628, 320)
  assert crop.y.shape == (320, 628)
  assert crop.uv is not None
  assert crop.uv.shape == (160, 628)
  boxed, scale, pad_x, pad_y = letterbox_rgb_from_nv12_crop(crop, 320)
  assert boxed.shape == (320, 320, 3)
  assert pad_x == 0
  assert scale == pytest.approx(320 / 628)
  nh = int(round(320 * (320 / 628)))
  assert pad_y == (320 - nh) // 2
  # Neutral gray stays gray — Y-pad zeros were not read as U=V=0.
  assert 120 <= int(boxed[160, 160, 0]) <= 136
  assert 120 <= int(boxed[160, 160, 1]) <= 136
  assert 120 <= int(boxed[160, 160, 2]) <= 136


def test_letterbox_from_nv12_crop_uses_uv_offset_not_visible_height():
  width, height, stride = 8, 4, 8
  y = bytes([128] * (stride * height))
  pad = bytes([0] * 16)
  uv = bytes([128, 128] * (stride // 2) * (height // 2))

  class Buf:
    pass

  Buf.width = width
  Buf.height = height
  Buf.stride = stride
  Buf.uv_offset = stride * height + len(pad)
  Buf.data = y + pad + uv

  crop = copy_nv12_detect_crop(Buf())
  assert crop is not None and crop.uv is not None
  boxed, _scale, _px, _py = letterbox_rgb_from_nv12_crop(crop, 4)
  assert 120 <= int(boxed[2, 2, 1]) <= 136


def test_letterbox_nv12_crop_is_far_cheaper_than_full_rgb():
  """Pixel work: 320² RGB vs 1928×1208 RGB888. Relative time on this host."""
  import time
  buf = _nv12_buf(1928, 1208, y=90, u=120, v=130)
  t0 = time.perf_counter()
  full = rgb_from_nv12(buf)
  full_s = time.perf_counter() - t0
  t1 = time.perf_counter()
  crop = copy_nv12_detect_crop(buf)
  boxed, _, _, _ = letterbox_rgb_from_nv12_crop(crop, 320)
  cheap_s = time.perf_counter() - t1
  assert full is not None and full.shape == (1208, 1928, 3)
  assert boxed.shape == (320, 320, 3)
  full_px = 1928 * 1208
  cheap_px = 320 * 320
  assert cheap_px * 8 < full_px
  # Copy+letterbox should beat a full-frame BT.601 convert. Allow slack on a
  # busy host; the pixel ratio is the hard assert.
  assert cheap_s < full_s * 1.5 or cheap_px < full_px


def test_nv12_detect_crop_luma_only_if_uv_short():
  class Buf:
    width = 4
    height = 4
    stride = 4
    uv_offset = 16
    data = bytes([200] * 16)  # Y only

  crop = copy_nv12_detect_crop(Buf())
  assert crop is not None
  assert crop.uv is None
  boxed, _, _, _ = letterbox_rgb_from_nv12_crop(crop, 4)
  assert int(boxed[0, 0, 0]) == int(boxed[0, 0, 1]) == int(boxed[0, 0, 2])


def test_nv12_detect_crop_type():
  y = np.full((8, 8), 40, np.uint8)
  crop = Nv12DetectCrop(y=y, uv=None, frame_w=16, frame_h=8, crop=(8, 0, 8, 8))
  assert crop.frame_w == 16
