"""YOLOv8 ONNX decode for the US MUTCD speed-sign detector.

Expects an Ultralytics detect export:
  input  `images`  float32 [1,3,H,W] RGB 0..1 (letterboxed)
  output `output0` float32 [1,4+nc,N]  (or [1,N,4+nc]) = cx,cy,w,h + class scores
"""
from __future__ import annotations

import re
from dataclasses import replace
from typing import NamedTuple

import numpy as np

from openpilot.selfdrive.speedsignd.detect_types import MUTCD_MPH, SpeedSign
from openpilot.selfdrive.speedsignd.nv12 import native_detect_window
from openpilot.selfdrive.speedsignd.weights_manifest import (
  YOLO_CLASS_NAMES,
  YOLO_IMGSZ,
  YOLO_IOU,
  YOLO_MAX_DET,
  YOLO_MIN_CONF,
)

# Justin parked close 50: top=speedLimit65:0.73–0.76, raw=[(65,0.73)],
# cls=30:0.00,50:0.00,60:0.00 — the 50 head is at zero, not a close race.
# Class-margin cannot fix a confidently wrong YOLO head. Prefer any crop
# digit read that returns a different MUTCD mph at this floor.
# _read_mph already requires min digit NCC ≥ 0.25.
REFINE_OVERRIDE_CONF = 0.28
CROP_OVERRIDE_CONF = REFINE_OVERRIDE_CONF  # old name; was 0.55 and hid 50s

# v2 names are sl_25 … sl_70. speedLimitNN still parses so older logs and
# tests that name a class that way stay readable. sl_unreadable does not match.
_SPEED_RE = re.compile(r"^(?:speedLimit|sl_)(\d+)$")

# Below this, a class name is argmax-of-noise. Do not treat it as a mph read.
PEAK_NAME_MIN = 0.05
# Below YOLO_MIN_CONF. Schedules a second look only — never lights the HUD alone.
SIGN_LIKE_CONF = 0.15
# Posted heads worth logging. v2 has sl_30/50/60/65; 65 is still the common miss.
POSTED_LOG_MPH = (30, 50, 60, 65)


def class_to_mph(name: str) -> int | None:
  """MUTCD mph from sl_NN or speedLimitNN. sl_unreadable and neg_* are None."""
  m = _SPEED_RE.match(str(name))
  if not m:
    return None
  value = int(m.group(1))
  return value if value in MUTCD_MPH else None


def is_unreadable_name(name: str) -> bool:
  """Box the OCR second look may use. Not an mph."""
  return str(name) == "sl_unreadable"


def is_negative_name(name: str) -> bool:
  """Yellow advisory / other non-speed. Never a box, never an mph."""
  return str(name).startswith("neg_")


def speed_limit_class_indices(names: tuple[str, ...] = YOLO_CLASS_NAMES) -> tuple[int, ...]:
  """Indices whose names are sl_NN / speedLimitNN. Excludes sl_unreadable and neg_*."""
  return tuple(i for i, n in enumerate(names) if class_to_mph(n) is not None)


def _look_indices(names: tuple[str, ...]) -> list[int]:
  """Speed classes plus sl_unreadable. neg_* stay out of the argmax."""
  out = list(speed_limit_class_indices(names))
  for i, n in enumerate(names):
    if is_unreadable_name(n):
      out.append(i)
  return out


class YoloPeak(NamedTuple):
  """Pre-threshold snapshot: global peak, speedLimit* peak, top-3, posted heads."""
  shape: tuple
  conf: float
  name: str
  n_over: int
  sl_conf: float = 0.0
  sl_name: str = ""
  n_over_sl: int = 0
  top3: tuple[tuple[str, float], ...] = ()
  posted: tuple[tuple[int, float], ...] = ()


def letterbox_rgb(rgb: np.ndarray, size: int = YOLO_IMGSZ) -> tuple[np.ndarray, float, int, int]:
  """Resize with aspect ratio, pad to size×size. Returns image, scale, pad_x, pad_y."""
  if rgb.ndim != 3 or rgb.shape[2] != 3:
    raise ValueError("letterbox_rgb expects HxWx3")
  h, w = rgb.shape[:2]
  scale = min(size / float(h), size / float(w))
  nh = max(1, int(round(h * scale)))
  nw = max(1, int(round(w * scale)))
  resized = _resize_rgb(rgb, nh, nw)
  canvas = np.full((size, size, 3), 114, dtype=np.uint8)
  top = (size - nh) // 2
  left = (size - nw) // 2
  canvas[top:top + nh, left:left + nw] = resized
  return canvas, scale, left, top


def road_detect_crop_rect(h: int, w: int) -> tuple[int, int, int, int]:
  """Right-side native window. Same pixels the ONNX letterboxes.

  The rect is independent of YOLO_IMGSZ. A 320 model letterboxes 628×320
  at scale 320/628, which keeps a ≥60 px sign at ≥30 px.
  """
  return native_detect_window(h, w)


def road_detect_crop(rgb: np.ndarray) -> tuple[np.ndarray, tuple[int, int, int, int]]:
  """Right-side window. Width and height follow native_detect_window."""
  if rgb.ndim != 3 or rgb.shape[2] != 3:
    raise ValueError("road_detect_crop expects HxWx3")
  h, w = rgb.shape[:2]
  x, y, cw, ch = road_detect_crop_rect(h, w)
  return rgb[y:y + ch, x:x + cw], (x, y, cw, ch)


def _resize_rgb(img: np.ndarray, h: int, w: int) -> np.ndarray:
  """Bilinear resize. Nearest index-sampling destroyed distant R2-1s at 6×."""
  if img.shape[0] == h and img.shape[1] == w:
    return img
  try:
    import cv2
    return cv2.resize(img, (w, h), interpolation=cv2.INTER_LINEAR)
  except Exception:
    return _resize_rgb_bilinear(img, h, w)


def _resize_rgb_bilinear(img: np.ndarray, h: int, w: int) -> np.ndarray:
  src_h, src_w = img.shape[:2]
  if src_h < 1 or src_w < 1:
    return np.zeros((h, w, img.shape[2]), dtype=img.dtype)
  ys = (np.arange(h, dtype=np.float32) + 0.5) * (src_h / float(h)) - 0.5
  xs = (np.arange(w, dtype=np.float32) + 0.5) * (src_w / float(w)) - 0.5
  ys = np.clip(ys, 0.0, src_h - 1.0)
  xs = np.clip(xs, 0.0, src_w - 1.0)
  y0 = np.floor(ys).astype(np.int32)
  x0 = np.floor(xs).astype(np.int32)
  y1 = np.minimum(y0 + 1, src_h - 1)
  x1 = np.minimum(x0 + 1, src_w - 1)
  wy = (ys - y0).astype(np.float32)[:, None, None]
  wx = (xs - x0).astype(np.float32)[None, :, None]
  img_f = img.astype(np.float32)
  i00 = img_f[y0][:, x0]
  i01 = img_f[y0][:, x1]
  i10 = img_f[y1][:, x0]
  i11 = img_f[y1][:, x1]
  out = (i00 * (1.0 - wx) + i01 * wx) * (1.0 - wy) + (i10 * (1.0 - wx) + i11 * wx) * wy
  return np.clip(np.rint(out), 0, 255).astype(np.uint8)


def nms_xyxy(boxes: np.ndarray, scores: np.ndarray, iou_thr: float = YOLO_IOU, max_det: int = YOLO_MAX_DET) -> list[int]:
  """Greedy IoU NMS. boxes are [N,4] xyxy."""
  if len(boxes) == 0:
    return []
  order = scores.argsort()[::-1]
  keep: list[int] = []
  while order.size and len(keep) < max_det:
    i = int(order[0])
    keep.append(i)
    if order.size == 1:
      break
    rest = order[1:]
    xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
    yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
    xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
    yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])
    inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
    area_i = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
    area_r = (boxes[rest, 2] - boxes[rest, 0]) * (boxes[rest, 3] - boxes[rest, 1])
    iou = inter / np.maximum(1e-6, area_i + area_r - inter)
    order = rest[iou <= iou_thr]
  return keep


def _as_cn(raw: np.ndarray) -> np.ndarray:
  """Normalize YOLO output to [4+nc, N]. Squeeze tinygrad singleton dims."""
  arr = np.squeeze(np.asarray(raw, dtype=np.float32))
  if arr.ndim == 3 and arr.shape[0] == 1:
    arr = arr[0]
  if arr.ndim != 2:
    return np.zeros((0, 0), np.float32)
  channels = 4 + len(YOLO_CLASS_NAMES)
  if arr.shape[0] == channels:
    return arr
  if arr.shape[1] == channels:
    return arr.T
  # Unknown nc: prefer the shorter axis as channels (4+nc << anchors).
  if arr.shape[0] <= arr.shape[1] and arr.shape[0] >= 6:
    return arr
  if arr.shape[1] >= 6:
    return arr.T
  return arr


def yolo_peak(raw, names: tuple[str, ...] = YOLO_CLASS_NAMES) -> YoloPeak:
  """Pre-threshold peak: global top class, speedLimit* peak, top-3 class peaks."""
  arr = np.asarray(raw)
  shape = tuple(int(v) for v in arr.shape)
  pred = _as_cn(raw)
  empty = YoloPeak(shape, 0.0, "", 0)
  if pred.size == 0 or pred.shape[0] < 5:
    return empty
  scores = pred[4:]
  if scores.size == 0:
    return empty
  if scores.ndim != 2:
    scores = scores.reshape(-1, 1)
  conf = scores.max(axis=0)
  cls = scores.argmax(axis=0)
  if conf.size == 0:
    return empty
  i = int(conf.argmax())
  max_conf = float(conf[i])
  top = int(cls[i]) if cls.size else 0
  name = names[top] if 0 <= top < len(names) else str(top)
  if max_conf < PEAK_NAME_MIN:
    name = ""
  n_over = int(np.sum(conf >= YOLO_MIN_CONF))

  sl_conf, sl_name, n_over_sl = 0.0, "", 0
  sl_idx = [k for k in speed_limit_class_indices(names) if k < scores.shape[0]]
  if sl_idx:
    sl = scores[np.array(sl_idx, dtype=np.int32)]
    sl_per_anchor = sl.max(axis=0)
    j = int(sl_per_anchor.argmax())
    sl_conf = float(sl_per_anchor[j])
    sl_local = int(sl[:, j].argmax())
    sl_top = sl_idx[sl_local]
    sl_name = names[sl_top] if 0 <= sl_top < len(names) else str(sl_top)
    if sl_conf < PEAK_NAME_MIN:
      sl_name = ""
    n_over_sl = int(np.sum(sl_per_anchor >= YOLO_MIN_CONF))

  class_peak = scores.max(axis=1)
  order = class_peak.argsort()[::-1]
  top3 = tuple(
    (names[int(k)] if 0 <= int(k) < len(names) else str(int(k)), float(class_peak[k]))
    for k in order
    if float(class_peak[k]) >= PEAK_NAME_MIN
  )[:3]
  posted: list[tuple[int, float]] = []
  for mph in POSTED_LOG_MPH:
    idx = None
    for label in (f"sl_{mph}", f"speedLimit{mph}"):
      try:
        idx = names.index(label)
        break
      except ValueError:
        continue
    if idx is None:
      posted.append((mph, 0.0))
      continue
    posted.append((mph, float(class_peak[idx]) if idx < class_peak.shape[0] else 0.0))
  return YoloPeak(shape, max_conf, name, n_over, sl_conf, sl_name, n_over_sl, top3, tuple(posted))


def decode_yolov8(
  raw,
  *,
  scale: float,
  pad_x: int,
  pad_y: int,
  src_hw: tuple[int, int],
  names: tuple[str, ...] = YOLO_CLASS_NAMES,
  min_conf: float = YOLO_MIN_CONF,
  iou: float = YOLO_IOU,
  max_det: int = YOLO_MAX_DET,
) -> list[SpeedSign]:
  pred = _as_cn(raw)
  if pred.size == 0 or pred.shape[0] < 5:
    return []
  boxes_xywh = pred[:4].T
  scores = pred[4:].T
  if scores.size == 0:
    return []
  # Argmax over sl_* and sl_unreadable only. neg_* must not hide a plate
  # and must not become a detection. sl_unreadable is a box for the second
  # look, not an mph.
  sl_idx = [k for k in _look_indices(names) if k < scores.shape[1]]
  alt_cls = None
  alt_conf = None
  if sl_idx:
    sl = scores[:, np.array(sl_idx, dtype=np.int32)]
    local = sl.argmax(axis=1)
    conf = sl.max(axis=1)
    cls = np.array(sl_idx, dtype=np.int32)[local]
    sl2 = sl.copy()
    sl2[np.arange(sl2.shape[0]), local] = -1.0
    local2 = sl2.argmax(axis=1)
    alt_conf = sl2.max(axis=1)
    alt_cls = np.array(sl_idx, dtype=np.int32)[local2]
  else:
    cls = scores.argmax(axis=1)
    conf = scores.max(axis=1)
  mask = conf >= min_conf
  if not np.any(mask):
    return []
  boxes_xywh = boxes_xywh[mask]
  conf = conf[mask]
  cls = cls[mask]
  if alt_cls is not None and alt_conf is not None:
    alt_cls = alt_cls[mask]
    alt_conf = alt_conf[mask]

  cx, cy, bw, bh = boxes_xywh[:, 0], boxes_xywh[:, 1], boxes_xywh[:, 2], boxes_xywh[:, 3]
  xyxy = np.stack([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2], axis=1)
  keep = nms_xyxy(xyxy, conf, iou_thr=iou, max_det=max(max_det * 3, 8))

  src_h, src_w = src_hw
  inv = 1.0 / scale if scale > 1e-6 else 1.0
  found: list[SpeedSign] = []
  for i in keep:
    name = names[int(cls[i])] if 0 <= int(cls[i]) < len(names) else ""
    if is_negative_name(name):
      continue
    unreadable = is_unreadable_name(name)
    mph = None if unreadable else class_to_mph(name)
    if mph is None and not unreadable:
      continue
    x1 = (xyxy[i, 0] - pad_x) * inv
    y1 = (xyxy[i, 1] - pad_y) * inv
    x2 = (xyxy[i, 2] - pad_x) * inv
    y2 = (xyxy[i, 3] - pad_y) * inv
    x = int(round(max(0.0, min(x1, x2))))
    y = int(round(max(0.0, min(y1, y2))))
    w = int(round(max(1.0, abs(x2 - x1))))
    h = int(round(max(1.0, abs(y2 - y1))))
    if x >= src_w or y >= src_h:
      continue
    w = min(w, src_w - x)
    h = min(h, src_h - y)
    alt_mph = None
    alt_c = 0.0
    if alt_cls is not None and alt_conf is not None:
      alt_name = names[int(alt_cls[i])] if 0 <= int(alt_cls[i]) < len(names) else ""
      alt_mph = class_to_mph(alt_name)
      alt_c = float(alt_conf[i])
      if alt_mph == mph:
        alt_mph, alt_c = None, 0.0
    if unreadable:
      # mph 0 and no class_mph: HUD and JSONL cannot treat this as a plate.
      # conf stays so the second look can use the box.
      found.append(SpeedSign(
        mph=0, conf=float(conf[i]), bbox=(x, y, w, h),
        class_mph=None, class_conf=0.0,
      ))
    else:
      found.append(SpeedSign(
        mph=int(mph), conf=float(conf[i]), bbox=(x, y, w, h),
        class_mph=int(mph), class_conf=float(conf[i]),
        alt_mph=alt_mph, alt_conf=alt_c,
      ))
    if len(found) >= max_det:
      break
  found.sort(key=lambda s: s.conf, reverse=True)
  return found


def prefer_refine(_sign: SpeedSign, mph: int, conf: float) -> bool:
  """Prefer crop digits over YOLO class when the read is a confident MUTCD mph.

  Justin's parked 50 is class 65 @ 0.73 with cls=50:0.00. Do not require a
  runner-up margin — the 50 head is not in the race.
  """
  return mph in MUTCD_MPH and conf >= REFINE_OVERRIDE_CONF


def refine_mph(sign: SpeedSign, y: np.ndarray | None, read_mph) -> SpeedSign:
  """Digit read on every in-threshold speed-limit crop. Does not invent a hit.

  If the crop returns a different MUTCD mph with confidence, sign.mph becomes
  that read. The HUD still requires a second frame, and a class/OCR
  contradiction is not a vote. sl_unreadable (mph not in MUTCD, no class_mph)
  is left unchanged — the OCR second look is the only way that box becomes
  an mph. Logs keep both class_mph and refine_mph.
  """
  if sign.class_mph is None and int(sign.mph) not in MUTCD_MPH:
    return sign
  if y is None or y.ndim != 2:
    return sign
  x, yy, w, h = sign.bbox
  if w < 12 or h < 12:
    return sign
  inset = max(3, int(min(w, h) * 0.06))
  x0 = max(0, x + inset)
  y0 = max(0, yy + inset)
  x1 = min(y.shape[1], x + w - inset)
  y1 = min(y.shape[0], yy + h - inset)
  if x1 - x0 < 12 or y1 - y0 < 12:
    return sign
  crop = y[y0:y1, x0:x1]
  if crop.size == 0:
    return sign
  mph, conf = read_mph(crop)
  class_mph = sign.class_mph if sign.class_mph is not None else sign.mph
  tagged = replace(
    sign,
    class_mph=int(class_mph),
    class_conf=float(sign.class_conf or sign.conf),
    refine_mph=int(mph) if mph is not None and mph in MUTCD_MPH else None,
    refine_conf=float(conf) if mph is not None and mph in MUTCD_MPH else 0.0,
  )
  if mph is None or mph not in MUTCD_MPH:
    return tagged
  if mph == sign.mph:
    return replace(tagged, conf=float(min(1.0, max(sign.conf, conf))))
  if prefer_refine(sign, int(mph), float(conf)):
    return replace(tagged, mph=int(mph), conf=float(conf))
  return tagged
