"""MUTCD R2-1 detector: YOLO ONNX on the ROAD camera, numpy fallback for tests.

Stock modelV2 has no speedSign head. On-device detection is a compact YOLOv8
ONNX under /data (see weights_manifest). The numpy template matcher is only
for unit tests / missing weights — it does not see real roadside signs.

MUTCD R2-1 is a white rectangle (about 24x30 in, aspect ~0.8) with black
legend: SPEED LIMIT over a 1–3 digit mph value.
"""
from __future__ import annotations

import os
import time
from typing import Any

# OpenBLAS / OpenMP / tinygrad read these at import. Force one thread before numpy.
INFER_THREAD_ENV = (
  "OMP_NUM_THREADS",
  "OPENBLAS_NUM_THREADS",
  "MKL_NUM_THREADS",
  "NUMEXPR_NUM_THREADS",
  "VECLIB_MAXIMUM_THREADS",
  "BLIS_NUM_THREADS",
  "OPENCV_FOR_THREADS_NUM",
  "GOTO_NUM_THREADS",
  "TINYGRAD_NUM_THREADS",
  "KMP_NUM_THREADS",
)
for _thread_key in INFER_THREAD_ENV:
  os.environ[_thread_key] = "1"

# tinygrad's default device on a 3X is QCOM (the Adreno modeld already uses).
# DEV is read once, at import. Pin CPU/CLANG and hide every GPU before that.
# BEAM searches kernels for minutes. NOOPT compiles -O0. Both stay off.
_GPU_ENV = ("GPU", "QCOM", "CUDA", "METAL", "AMD", "NV", "CL", "WEBGPU", "DSP")


def pin_cpu_only_env() -> None:
  """Force a CPU backend. Never leaves a GPU or QCOM device enabled."""
  os.environ["DEV"] = "CPU"
  os.environ["CPU"] = "1"
  os.environ["CLANG"] = "1"
  os.environ["CPU_COUNT"] = "1"
  os.environ["BEAM"] = "0"
  os.environ["NOOPT"] = "0"
  os.environ["OMP_DYNAMIC"] = "FALSE"
  os.environ["MKL_DYNAMIC"] = "FALSE"
  os.environ["OMP_WAIT_POLICY"] = "PASSIVE"
  os.environ["OPENCV_OPENCL_RUNTIME"] = "disabled"
  os.environ["OPENCV_DNN_OPENCL"] = "0"
  for key in _GPU_ENV:
    os.environ[key] = "0"
  for key in INFER_THREAD_ENV:
    os.environ[key] = "1"


pin_cpu_only_env()

import numpy as np

from openpilot.selfdrive.speedsignd.detect_types import MUTCD_MPH, SpeedSign, shift_sign
from openpilot.selfdrive.speedsignd.nv12 import letterbox_rgb_from_nv12_crop, rgb_from_y, upscale_gray
from openpilot.selfdrive.speedsignd.paths import default_onnx_path, nap_data_dir
from openpilot.selfdrive.speedsignd.weights_manifest import (
  YOLO_CLASS_NAMES,
  YOLO_IMGSZ,
  YOLO_IOU,
  YOLO_MAX_DET,
  YOLO_MIN_CONF,
)
from openpilot.selfdrive.speedsignd.yolo import (
  SIGN_LIKE_CONF,
  decode_yolov8,
  letterbox_rgb,
  refine_mph,
  road_detect_crop,
  yolo_peak,
)

MIN_CONF = 0.42
MAX_DET = 3
DIGIT_H, DIGIT_W = 24, 16
# Downsample the ROAD frame so CC stays cheap on the 3X.
MAX_DETECT_WIDTH = 320
# tinygrad OnnxRunner / OpenBLAS will otherwise take every core.
THREADS_ENV = "NAP_SPEED_SIGN_THREADS"
THREADS_DEFAULT = 1
THREADS_MAX = 1
# Soft cap: cannot kill an in-flight tinygrad kernel; skip/pay-back uses this
# plus infer time so a 1.8 s session cannot immediately start another.
CAP_MS_ENV = "NAP_SPEED_SIGN_INFER_CAP_MS"
CAP_MS_DEFAULT = 800.0
CAP_MS_MIN = 50.0
CAP_MS_MAX = 5000.0


def _resize(img: np.ndarray, h: int, w: int) -> np.ndarray:
  if img.shape[0] == h and img.shape[1] == w:
    return img
  ys = np.linspace(0, img.shape[0] - 1, h).astype(np.int32)
  xs = np.linspace(0, img.shape[1] - 1, w).astype(np.int32)
  return img[ys][:, xs]


def _ncc(a: np.ndarray, b: np.ndarray) -> float:
  x = a.astype(np.float32).ravel()
  y = b.astype(np.float32).ravel()
  x -= x.mean()
  y -= y.mean()
  den = float(np.linalg.norm(x) * np.linalg.norm(y))
  if den < 1e-6:
    return 0.0
  return float(np.dot(x, y) / den)


def _paint_rect(t: np.ndarray, r0: int, r1: int, c0: int, c1: int, v: int = 1) -> None:
  t[r0:r1, c0:c1] = v


def _digit_templates() -> dict[int, np.ndarray]:
  """Block Highway-Gothic-ish 16x24 ink templates (1 = ink)."""
  out: dict[int, np.ndarray] = {}
  h, w = DIGIT_H, DIGIT_W
  thick = 3

  def blank() -> np.ndarray:
    return np.zeros((h, w), np.uint8)

  # 0: closed oval
  t = blank()
  _paint_rect(t, 1, 1 + thick, 3, w - 3)
  _paint_rect(t, h - 1 - thick, h - 1, 3, w - 3)
  _paint_rect(t, 1, h - 1, 2, 2 + thick)
  _paint_rect(t, 1, h - 1, w - 2 - thick, w - 2)
  out[0] = t

  # 1: right stem + top serif
  t = blank()
  _paint_rect(t, 1, h - 1, w // 2, w // 2 + thick)
  _paint_rect(t, 1, 1 + thick, w // 2 - 4, w // 2 + thick)
  _paint_rect(t, h - 1 - thick, h - 1, w // 2 - 4, w // 2 + 5)
  out[1] = t

  # 2
  t = blank()
  _paint_rect(t, 1, 1 + thick, 2, w - 2)
  _paint_rect(t, 1, h // 2, w - 2 - thick, w - 2)
  _paint_rect(t, h // 2 - 1, h // 2 - 1 + thick, 2, w - 2)
  _paint_rect(t, h // 2, h - 1, 2, 2 + thick)
  _paint_rect(t, h - 1 - thick, h - 1, 2, w - 2)
  out[2] = t

  # 3
  t = blank()
  _paint_rect(t, 1, 1 + thick, 2, w - 2)
  _paint_rect(t, h // 2 - 1, h // 2 - 1 + thick, 3, w - 3)
  _paint_rect(t, h - 1 - thick, h - 1, 2, w - 2)
  _paint_rect(t, 1, h - 1, w - 2 - thick, w - 2)
  out[3] = t

  # 4: open top, crossbar, right stem
  t = blank()
  _paint_rect(t, 1, h // 2 + 1, 2, 2 + thick)
  _paint_rect(t, h // 2 - 1, h // 2 - 1 + thick, 2, w - 2)
  _paint_rect(t, 1, h - 1, w - 5, w - 2)
  out[4] = t

  # 5
  t = blank()
  _paint_rect(t, 1, 1 + thick, 2, w - 2)
  _paint_rect(t, 1, h // 2, 2, 2 + thick)
  _paint_rect(t, h // 2 - 1, h // 2 - 1 + thick, 2, w - 2)
  _paint_rect(t, h // 2, h - 1, w - 2 - thick, w - 2)
  _paint_rect(t, h - 1 - thick, h - 1, 2, w - 2)
  out[5] = t

  # 6
  t = blank()
  _paint_rect(t, 1, 1 + thick, 2, w - 3)
  _paint_rect(t, 1, h - 1, 2, 2 + thick)
  _paint_rect(t, h // 2 - 1, h // 2 - 1 + thick, 2, w - 2)
  _paint_rect(t, h // 2, h - 1, w - 2 - thick, w - 2)
  _paint_rect(t, h - 1 - thick, h - 1, 2, w - 2)
  out[6] = t

  # 7
  t = blank()
  _paint_rect(t, 1, 1 + thick, 2, w - 2)
  _paint_rect(t, 1, h - 1, w - 2 - thick, w - 2)
  out[7] = t

  # 8
  t = blank()
  _paint_rect(t, 1, 1 + thick, 3, w - 3)
  _paint_rect(t, h // 2 - 1, h // 2 - 1 + thick, 3, w - 3)
  _paint_rect(t, h - 1 - thick, h - 1, 3, w - 3)
  _paint_rect(t, 1, h - 1, 2, 2 + thick)
  _paint_rect(t, 1, h - 1, w - 2 - thick, w - 2)
  out[8] = t

  # 9
  t = blank()
  _paint_rect(t, 1, 1 + thick, 2, w - 2)
  _paint_rect(t, 1, h // 2 + 2, 2, 2 + thick)
  _paint_rect(t, 1, h // 2 + 2, w - 2 - thick, w - 2)
  _paint_rect(t, h // 2 - 1, h // 2 - 1 + thick, 2, w - 2)
  _paint_rect(t, h // 2, h - 1, w - 2 - thick, w - 2)
  _paint_rect(t, h - 1 - thick, h - 1, 2, w - 3)
  out[9] = t
  return out


DIGIT_TEMPLATES = _digit_templates()


def parse_infer_threads(raw: str | None, default: int = THREADS_DEFAULT) -> int:
  """One inference thread. Never a pool."""
  if raw is None or str(raw).strip() == "":
    return int(default)
  try:
    n = int(float(raw))
  except (TypeError, ValueError):
    return int(default)
  return max(1, min(THREADS_MAX, n))


def parse_infer_cap_ms(raw: str | None, default: float = CAP_MS_DEFAULT) -> float:
  """0 disables the cap. Otherwise clamp to a sane skip budget."""
  if raw is None or str(raw).strip() == "":
    return float(default)
  try:
    ms = float(raw)
  except (TypeError, ValueError):
    return float(default)
  if not (ms == ms) or ms < 0:  # NaN or negative
    return float(default)
  if ms == 0.0:
    return 0.0
  return min(CAP_MS_MAX, max(CAP_MS_MIN, ms))


def _shrink_blas_pool(threads: int) -> None:
  """Resize a BLAS/OpenMP pool that already started.

  Manager forks speedsignd after numpy is imported. OpenBLAS captured
  nproc (8 on a 3X) at that init, so the env pin in this file is too late.
  The logged `threads=11` was that pool of 8 plus the main thread, the
  infer worker, and the VisionIPC client. This call is the one that
  actually drops the pool to 1 inside the child.
  """
  import ctypes
  n = max(1, int(threads))
  for soname in ("libopenblas.so.0", "libopenblas.so", "libblas.so.3", "libblas.so"):
    try:
      lib = ctypes.CDLL(soname)
    except OSError:
      continue
    for sym in ("openblas_set_num_threads", "goto_set_num_threads"):
      fn = getattr(lib, sym, None)
      if fn is None:
        continue
      try:
        fn.argtypes = [ctypes.c_int]
        fn.restype = None
        fn(n)
      except Exception:
        pass
    break
  for soname in ("libgomp.so.1", "libgomp.so", "libiomp5.so"):
    try:
      lib = ctypes.CDLL(soname)
    except OSError:
      continue
    fn = getattr(lib, "omp_set_num_threads", None)
    if fn is None:
      continue
    try:
      fn.argtypes = [ctypes.c_int]
      fn.restype = None
      fn(n)
    except Exception:
      pass
    break


def limit_infer_threads(n: int | None = None) -> int:
  """Pin BLAS / OpenMP / tinygrad / OpenCV pools. Always one thread on device."""
  pin_cpu_only_env()
  threads = parse_infer_threads(os.environ.get(THREADS_ENV) if n is None else str(n))
  n_s = str(threads)
  for key in INFER_THREAD_ENV:
    os.environ[key] = n_s
  _shrink_blas_pool(threads)
  try:
    import cv2
    cv2.setNumThreads(threads)
    ocl = getattr(cv2, "ocl", None)
    if ocl is not None and hasattr(ocl, "setUseOpenCL"):
      ocl.setUseOpenCL(False)
  except Exception:
    pass
  return threads


def paint_digit(canvas: np.ndarray, digit: int, r0: int, c0: int, scale: int = 2, ink: int = 20) -> None:
  """Blit a template onto a uint8 Y image (for tests and ONNX fixtures)."""
  t = DIGIT_TEMPLATES[int(digit)]
  h, w = t.shape
  block = np.repeat(np.repeat(t, scale, axis=0), scale, axis=1)
  r1 = r0 + block.shape[0]
  c1 = c0 + block.shape[1]
  if r0 < 0 or c0 < 0 or r1 > canvas.shape[0] or c1 > canvas.shape[1]:
    return
  canvas[r0:r1, c0:c1] = np.where(block > 0, ink, canvas[r0:r1, c0:c1])


def paint_mutcd_r2_1(canvas: np.ndarray, mph: int, x: int, y: int, w: int, h: int) -> None:
  """Draw a white R2-1-style plate with SPEED LIMIT bars and gothic-ish digits."""
  canvas[y:y + h, x:x + w] = 235
  canvas[y:y + 3, x:x + w] = 15
  canvas[y + h - 3:y + h, x:x + w] = 15
  canvas[y:y + h, x:x + 3] = 15
  canvas[y:y + h, x + w - 3:x + w] = 15
  # "SPEED LIMIT" stand-in: two dark word bars in the upper third.
  bar_h = max(3, h // 18)
  canvas[y + h // 8:y + h // 8 + bar_h, x + w // 6:x + w - w // 6] = 25
  canvas[y + h // 5:y + h // 5 + bar_h, x + w // 5:x + w - w // 5] = 25
  digits = [int(ch) for ch in str(int(mph))]
  n = len(digits)
  inner_w = max(8, w - 12)
  inner_h = max(8, h // 2 - 10)
  scale = max(1, min(inner_h // DIGIT_H, (inner_w - (n - 1) * 2) // max(1, n * DIGIT_W)))
  digit_w = DIGIT_W * scale
  digit_h = DIGIT_H * scale
  gap = max(2, scale)
  total_w = n * digit_w + (n - 1) * gap
  c0 = x + (w - total_w) // 2
  r0 = y + h // 2 + h // 16
  if r0 + digit_h > y + h - 4:
    r0 = y + h - 4 - digit_h
  for i, d in enumerate(digits):
    paint_digit(canvas, d, r0, c0 + i * (digit_w + gap), scale=scale, ink=18)


def _components(mask: np.ndarray) -> list[tuple[int, int, int, int, int]]:
  """Two-pass connected components → (x, y, w, h, area)."""
  h, w = mask.shape
  labels = np.zeros((h, w), np.int32)
  parent = [0]

  def find(a: int) -> int:
    while parent[a] != a:
      parent[a] = parent[parent[a]]
      a = parent[a]
    return a

  def union(a: int, b: int) -> None:
    ra, rb = find(a), find(b)
    if ra != rb:
      parent[rb] = ra

  n = 0
  for r in range(h):
    row = mask[r]
    lab_row = labels[r]
    lab_up = labels[r - 1] if r else None
    for c in range(w):
      if not row[c]:
        continue
      left = lab_row[c - 1] if c else 0
      up = lab_up[c] if lab_up is not None else 0
      if left and up:
        lab_row[c] = left
        union(int(left), int(up))
      elif left:
        lab_row[c] = left
      elif up:
        lab_row[c] = up
      else:
        n += 1
        parent.append(n)
        lab_row[c] = n

  remap: dict[int, int] = {}
  next_id = 1
  stats: dict[int, list[int]] = {}
  for r in range(h):
    lab_row = labels[r]
    for c in range(w):
      lab = int(lab_row[c])
      if lab == 0:
        continue
      root = find(lab)
      i = remap.get(root)
      if i is None:
        remap[root] = next_id
        i = next_id
        next_id += 1
        stats[i] = [c, r, c, r, 0]
      s = stats[i]
      if c < s[0]:
        s[0] = c
      if r < s[1]:
        s[1] = r
      if c > s[2]:
        s[2] = c
      if r > s[3]:
        s[3] = r
      s[4] += 1
  boxes = []
  for x0, y0, x1, y1, area in stats.values():
    boxes.append((x0, y0, x1 - x0 + 1, y1 - y0 + 1, area))
  return boxes


def _digit_score(patch: np.ndarray, thr: float = 90.0) -> tuple[int, float]:
  ink = (patch < thr).astype(np.uint8)
  if ink.mean() < 0.05 or ink.mean() > 0.85:
    return 0, 0.0
  templ = _resize(ink, DIGIT_H, DIGIT_W)
  best_d, best_s = 0, -1.0
  for d, t in DIGIT_TEMPLATES.items():
    s = _ncc(templ, t)
    if s > best_s:
      best_d, best_s = d, s
  return best_d, float(best_s)


def _split_digit_boxes(ink: np.ndarray) -> list[tuple[int, int, int, int]]:
  if ink.size == 0:
    return []
  col = ink.mean(axis=0)
  if col.size >= 3:
    col = np.convolve(col, np.ones(3, dtype=np.float32) / 3.0, mode="same")
  thresh = max(0.025, float(col.max()) * 0.12)
  runs = []
  in_run = False
  start = 0
  for i, v in enumerate(col):
    if v >= thresh and not in_run:
      in_run = True
      start = i
    elif v < thresh and in_run:
      in_run = False
      runs.append((start, i))
  if in_run:
    runs.append((start, len(col)))

  raw: list[tuple[int, int, int, int]] = []
  for x0, x1 in runs:
    if x1 - x0 < 2:
      continue
    strip = ink[:, x0:x1]
    rows = np.where(strip.mean(axis=1) > 0.05)[0]
    if len(rows) < 4:
      continue
    y0, y1 = int(rows[0]), int(rows[-1]) + 1
    raw.append((x0, y0, x1 - x0, y1 - y0))
  if not raw:
    return []

  # Open 4 / thin 7: merge adjacent narrow stems. Do not glue a thin stem onto a wide neighbor (4+5).
  typical = max(b[2] for b in raw)
  thin = max(4, int(typical * 0.45))
  merged: list[tuple[int, int, int, int]] = [raw[0]]
  for b in raw[1:]:
    prev = merged[-1]
    gap = b[0] - (prev[0] + prev[2])
    if prev[2] <= thin and b[2] <= thin and gap <= max(8, typical // 2):
      x0 = prev[0]
      y0 = min(prev[1], b[1])
      x1 = b[0] + b[2]
      y1 = max(prev[1] + prev[3], b[1] + b[3])
      merged[-1] = (x0, y0, x1 - x0, y1 - y0)
    else:
      merged.append(b)
  return merged[:3]


READ_MPH_MIN_DIGIT = 0.25


def _ink_thresholds(gray: np.ndarray) -> list[float]:
  """Fixed 90 plus a mid-gray split. ROAD plates wash out above 90."""
  if gray.size == 0:
    return [90.0]
  p20, p80 = np.percentile(gray.astype(np.float32), (20.0, 80.0))
  mid = float(max(40.0, min(160.0, (p20 + p80) * 0.5)))
  out: list[float] = []
  for t in (90.0, mid, 70.0, 110.0, 130.0):
    t = float(max(40.0, min(160.0, t)))
    if all(abs(t - s) > 4.0 for s in out):
      out.append(t)
  return out


def _read_mph_at(lower: np.ndarray, thr: float) -> tuple[int, float, float] | None:
  ink = (lower < thr).astype(np.uint8)
  boxes = _split_digit_boxes(ink)
  if not boxes:
    return None
  digits = []
  scores = []
  for x, y, bw, bh in boxes:
    pad = 1
    y0 = max(0, y - pad)
    x0 = max(0, x - pad)
    patch = lower[y0:min(lower.shape[0], y + bh + pad), x0:min(lower.shape[1], x + bw + pad)]
    d, s = _digit_score(patch, thr)
    digits.append(d)
    scores.append(s)
  if not scores or min(scores) < READ_MPH_MIN_DIGIT:
    return None
  value = 0
  for d in digits:
    value = value * 10 + d
  if value not in MUTCD_MPH:
    return None
  return int(value), float(sum(scores) / len(scores)), float(min(scores))


def _read_mph(crop: np.ndarray) -> tuple[int | None, float]:
  h, w = crop.shape
  if h < 12 or w < 12:
    return None, 0.0
  bands = (
    crop[int(h * 0.42):, int(w * 0.08):int(w * 0.92)],
    crop[int(h * 0.38):int(max(int(h * 0.38) + 12, h * 0.98)), int(w * 0.10):int(w * 0.90)],
  )
  best: tuple[float, float, int] | None = None  # min_digit, conf, mph
  for lower in bands:
    if lower.size == 0 or lower.shape[0] < 8 or lower.shape[1] < 8:
      continue
    for thr in _ink_thresholds(lower):
      got = _read_mph_at(lower, thr)
      if got is None:
        continue
      value, conf, min_s = got
      if best is None or min_s > best[0]:
        best = (min_s, conf, value)
  if best is None:
    return None, 0.0
  _min_s, conf, value = best
  # Extra boost when the upper third looks like stacked word bars (SPEED LIMIT).
  upper = crop[:int(h * 0.40), int(w * 0.10):int(w * 0.90)]
  if upper.size and upper.std() > 18.0:
    conf = min(1.0, conf + 0.08)
  return value, conf


def ocr_tight_y(y: np.ndarray | None) -> tuple[int | None, float]:
  """Digit OCR on a full-res plate crop. Upscale when the plate is tiny."""
  if y is None or getattr(y, "ndim", 0) != 2 or y.size == 0:
    return None, 0.0
  return _read_mph(upscale_gray(y, 96))


def _sign_candidates(y: np.ndarray) -> list[tuple[int, int, int, int]]:
  h, w = y.shape
  bright = y > 175
  boxes = _components(bright)
  out = []
  frame_area = h * w
  for x, yy, bw, bh, area in boxes:
    if bh < 18 or bw < 12:
      continue
    if area < 80:
      continue
    aspect = bw / float(bh)
    if aspect < 0.45 or aspect > 1.15:
      continue
    fill = area / float(bw * bh)
    if fill < 0.35:
      continue
    if bw * bh > 0.35 * frame_area:
      continue
    crop = y[yy:yy + bh, x:x + bw]
    if crop.std() < 22.0:
      continue
    # White plate with dark ink: mean should stay high.
    if crop.mean() < 140:
      continue
    out.append((x, yy, bw, bh))
  return out


def detect_mutcd_speed_signs(y: np.ndarray, min_conf: float = MIN_CONF) -> list[SpeedSign]:
  """Return MUTCD numeric speed signs on a uint8 Y-plane image."""
  if y is None or y.ndim != 2 or y.shape[0] < 32 or y.shape[1] < 32:
    return []
  src_h, src_w = y.shape
  work = y
  scale = 1.0
  if src_w > MAX_DETECT_WIDTH:
    scale = MAX_DETECT_WIDTH / float(src_w)
    nh = max(32, int(src_h * scale))
    work = _resize(y, nh, MAX_DETECT_WIDTH)

  found: list[SpeedSign] = []
  for x, yy, bw, bh in _sign_candidates(work):
    mph, conf = _read_mph(work[yy:yy + bh, x:x + bw])
    if mph is None or conf < min_conf:
      continue
    if scale != 1.0:
      inv = 1.0 / scale
      x, yy, bw, bh = int(x * inv), int(yy * inv), int(bw * inv), int(bh * inv)
    found.append(SpeedSign(mph=mph, conf=float(conf), bbox=(x, yy, bw, bh)))
    if len(found) >= MAX_DET:
      break
  found.sort(key=lambda s: s.conf, reverse=True)
  return found


class OnnxSpeedSignDetector:
  """Compact ONNX under /data. Missing/unloadable → None.

  Supported models:
    1. Ultralytics YOLOv8 detect (preferred):
         input  float32 [1,3,H,W] RGB 0..1 (letterboxed)
         output float32 [1,4+nc,N] or [1,N,4+nc]
    2. Legacy custom:
         input  `image`  float32 [1,1,H,W] (Y 0..1) or [1,3,H,W] (RGB 0..1)
         output `dets`   float32 [N,6] = x, y, w, h, mph, conf
         A [1,2] (mph, conf) output is also accepted.
  """

  def __init__(self, path: str, session=None, backend: str | None = None):
    self.path = path
    self.session = session
    self.backend = backend or session_backend(session)
    self.sha = _sha_short(path)
    self.last_diag: dict[str, Any] = self._empty_diag()

  def _empty_diag(self) -> dict[str, Any]:
    return {
      "backend": self.backend,
      "frame_w": 0,
      "frame_h": 0,
      "letterbox": YOLO_IMGSZ,
      "crop": (0, 0, 0, 0),
      "weights_path": self.path or "",
      "weights_sha": self.sha,
      "out_shape": (),
      "peak_conf": 0.0,
      "peak_name": "",
      "n_over": 0,
      "sl_peak_conf": 0.0,
      "sl_peak_name": "",
      "n_over_sl": 0,
      "top3": (),
      "posted": (),
      "refine": (),
      "error": "",
      "luma_mean": 0.0,
      "luma_std": 0.0,
      "chroma": 0,
      "prep_ms": 0.0,
      "sess_ms": 0.0,
    }

  def diag_dict(self) -> dict[str, Any]:
    return dict(self.last_diag)

  @classmethod
  def try_load(cls, path: str | None = None) -> OnnxSpeedSignDetector | None:
    path = path or default_onnx_path()
    if not path or not os.path.isfile(path):
      return None
    # v1 at the same path has a different class map. Refuse it so a 60 is
    # not decoded as whatever channel 7 means in the new head.
    try:
      from openpilot.selfdrive.speedsignd.install import weights_sha_matches
      if not weights_sha_matches(path):
        return None
    except Exception:
      return None
    session, backend = _onnx_session(path)
    if session is None:
      return None
    return cls(path, session, backend=backend)

  def detect(self, y: np.ndarray, min_conf: float | None = None, rgb: np.ndarray | None = None,
             nv12=None) -> list[SpeedSign]:
    diag = self._empty_diag()
    self.last_diag = diag
    if self.session is None:
      diag["error"] = "no-session"
      return []
    t_prep = time.monotonic()
    luma_src = None
    boxed = None
    work = None
    crop = (0, 0, 0, 0)
    scale = 1.0
    pad_x = pad_y = 0
    src_h = src_w = 0
    if nv12 is not None:
      y_crop = getattr(nv12, "y", None)
      if y_crop is None or getattr(y_crop, "ndim", 0) != 2:
        diag["error"] = "no-frame"
        return []
      src_w = int(getattr(nv12, "frame_w", 0) or y_crop.shape[1])
      src_h = int(getattr(nv12, "frame_h", 0) or y_crop.shape[0])
      crop = tuple(getattr(nv12, "crop", (0, 0, y_crop.shape[1], y_crop.shape[0])))
      diag["frame_w"] = src_w
      diag["frame_h"] = src_h
      diag["crop"] = crop
      diag["luma_mean"] = float(y_crop.mean())
      diag["luma_std"] = float(y_crop.std())
      uv = getattr(nv12, "uv", None)
      diag["chroma"] = 1 if uv is not None and getattr(uv, "size", 0) else 0
      luma_src = y_crop
    else:
      rgb_given = rgb is not None
      if rgb is None:
        if y is None or y.ndim != 2:
          diag["error"] = "no-frame"
          return []
        rgb = rgb_from_y(y)
      if rgb.ndim != 3 or rgb.shape[-1] != 3:
        diag["error"] = "bad-rgb"
        return []
      src_h, src_w = rgb.shape[:2]
      diag["frame_w"] = int(src_w)
      diag["frame_h"] = int(src_h)
      if y is not None and getattr(y, "ndim", 0) == 2:
        diag["luma_mean"] = float(y.mean())
        diag["luma_std"] = float(y.std())
      else:
        diag["luma_mean"] = float(rgb.mean())
        diag["luma_std"] = float(rgb.std())
      # Caller RGB may be real chroma; rgb_from_y is luma-only.
      diag["chroma"] = 1 if rgb_given else 0
    inp = _onnx_input_name(self.session)
    shape = _onnx_input_shape(self.session)
    yolo = _is_yolo_input(shape)
    try:
      if yolo:
        size = YOLO_IMGSZ
        if shape is not None and len(shape) == 4:
          h = int(shape[2]) if shape[2] not in (None, 0, -1) else YOLO_IMGSZ
          w = int(shape[3]) if shape[3] not in (None, 0, -1) else YOLO_IMGSZ
          size = h if h == w else YOLO_IMGSZ
        diag["letterbox"] = int(size)
        if nv12 is not None:
          boxed, scale, pad_x, pad_y = letterbox_rgb_from_nv12_crop(nv12, size)
          crop_h, crop_w = luma_src.shape[:2]
          cx, cy = int(crop[0]), int(crop[1])
        else:
          work, crop = road_detect_crop(rgb)
          diag["crop"] = crop
          cx, cy, _cw, _ch = crop
          boxed, scale, pad_x, pad_y = letterbox_rgb(work, size)
          crop_h, crop_w = work.shape[:2]
        blob = boxed.transpose(2, 0, 1)[None, ...].astype(np.float32) / 255.0
        diag["prep_ms"] = (time.monotonic() - t_prep) * 1000.0
        t_sess = time.monotonic()
        raw = self.session.run(None, {inp: blob})[0]
        diag["sess_ms"] = (time.monotonic() - t_sess) * 1000.0
        peak = yolo_peak(raw)
        diag["out_shape"] = peak.shape
        diag["peak_conf"] = peak.conf
        diag["peak_name"] = peak.name
        diag["n_over"] = peak.n_over
        diag["sl_peak_conf"] = peak.sl_conf
        diag["sl_peak_name"] = peak.sl_name
        diag["n_over_sl"] = peak.n_over_sl
        diag["top3"] = peak.top3
        diag["posted"] = peak.posted
        thr = YOLO_MIN_CONF if min_conf is None else min_conf
        like = decode_yolov8(
          raw, scale=scale, pad_x=pad_x, pad_y=pad_y, src_hw=(crop_h, crop_w),
          names=YOLO_CLASS_NAMES, min_conf=SIGN_LIKE_CONF, iou=YOLO_IOU, max_det=YOLO_MAX_DET,
        )
        if luma_src is not None:
          luma = luma_src
        elif y is not None and getattr(y, "ndim", 0) == 2:
          luma = y[cy:cy + crop_h, cx:cx + crop_w]
          if luma.shape[:2] != (crop_h, crop_w):
            luma = work[:, :, 1]
        else:
          luma = work[:, :, 1]
        refined_like = [refine_mph(s, luma, _read_mph) for s in like]
        if cx or cy:
          refined_like = [shift_sign(s, cx, cy) for s in refined_like]
        refined = [
          s for s in refined_like
          if float(s.conf) >= thr or (
            s.refine_mph is not None and float(s.refine_conf or 0.0) >= 0.28
          )
        ]
        if refined_like:
          best_like = max(refined_like, key=lambda s: float(s.conf))
          diag["sign_like_bbox"] = tuple(int(v) for v in best_like.bbox)
        else:
          diag["sign_like_bbox"] = None
        diag["refine"] = tuple(
          (
            int(s.class_mph if s.class_mph is not None else s.mph),
            int(s.mph),
            None if s.refine_mph is None else int(s.refine_mph),
            float(s.refine_conf),
          )
          for s in refined
        )
        return refined
      arr = (y if y is not None else rgb[:, :, 1]).astype(np.float32) / 255.0
      if shape is not None and len(shape) == 4:
        _n, c, h, w = [int(v) if v not in (None, 0, -1) else None for v in shape]
        hh = h or 256
        ww = w or 256
        resized = _resize(arr, hh, ww)
        if c == 3:
          blob = np.stack([resized, resized, resized], axis=0)[None, ...]
        else:
          blob = resized[None, None, ...]
      else:
        blob = arr[None, None, ...]
      diag["prep_ms"] = (time.monotonic() - t_prep) * 1000.0
      t_sess = time.monotonic()
      raw = self.session.run(None, {inp: blob.astype(np.float32)})[0]
      diag["sess_ms"] = (time.monotonic() - t_sess) * 1000.0
    except Exception as e:
      diag["error"] = f"{type(e).__name__}: {e}"
      diag["prep_ms"] = (time.monotonic() - t_prep) * 1000.0
      return []
    thr = MIN_CONF if min_conf is None else min_conf
    return _parse_onnx_dets(raw, (src_h, src_w), thr)


def _sha_short(path: str | None, n: int = 12) -> str:
  if not path or not os.path.isfile(path):
    return ""
  try:
    from openpilot.selfdrive.speedsignd.install import sha256_file
    return sha256_file(path)[:n]
  except Exception:
    return ""


def session_backend(session) -> str:
  if session is None:
    return "none"
  tagged = getattr(session, "_nap_backend", None)
  if tagged:
    return str(tagged)
  mod = getattr(type(session), "__module__", "")
  if "onnxruntime" in mod:
    return "onnxruntime-cpu"
  name = type(session).__name__
  if name == "_TinyOrtSession":
    return "tinygrad-cpu"
  return name


def compile_cache_dir(sha: str) -> str:
  """Per-weights compile cache under /data. One clang compile per sha."""
  root = os.environ.get("NAP_SPEED_SIGN_CACHE") or os.path.join(nap_data_dir(), "speed_sign_cache")
  digest = (sha or "unknown").strip() or "unknown"
  return os.path.join(root, digest)


def _compile_stamp_path(sha: str) -> str:
  return os.path.join(compile_cache_dir(sha), "ready")


def compile_cache_warm(sha: str) -> bool:
  """True when a previous disengaged run finished this sha.

  tinygrad also needs its sqlite kernel cache. opencv/onnxruntime only
  need the stamp: their session create is not the multi-minute clang.
  """
  if not sha or not os.path.isfile(_compile_stamp_path(sha)):
    return False
  try:
    with open(_compile_stamp_path(sha), encoding="utf-8") as f:
      text = f.read()
  except OSError:
    return False
  if "tinygrad" in text:
    db = os.path.join(compile_cache_dir(sha), "tinygrad", "cache.db")
    try:
      return os.path.isfile(db) and os.path.getsize(db) > 0
    except OSError:
      return False
  return True


def write_compile_stamp(sha: str, backend: str) -> None:
  if not sha:
    return
  path = _compile_stamp_path(sha)
  try:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
      f.write((backend or "cpu") + "\n" + sha + "\n")
    os.replace(tmp, path)
  except OSError:
    return


def compile_allowed(*, known: bool, controlling: bool, ready: bool) -> bool:
  """Warm-up realizes kernels. Start it only when known disengaged.

  Unknown cereal is not disengaged: after the startup grace, unread
  selfdriveState used to set controlling False and the first clang
  realize ran while the car was engaged. A finished cache (ready) may
  run while engaged. Offroad never reaches this process (manager starts
  it only onroad); disengaged onroad is the compile window.
  """
  if ready:
    return True
  if controlling or not known:
    return False
  return True


def dnn_ids_are_cpu(
  backend: int,
  target: int,
  *,
  gpu_backends: set[int],
  gpu_targets: set[int],
  cpu_target: int,
) -> bool:
  """OpenCV DNN must stay on the CPU target. Never CUDA, OpenCL, or Vulkan."""
  if int(backend) in gpu_backends or int(target) in gpu_targets:
    return False
  return int(target) == int(cpu_target)


def _device_is_cpu(name: str) -> bool:
  base = (name or "").split(":")[0].strip().upper()
  return base in ("CPU", "CLANG")


def assert_cpu_backend(session) -> str:
  """Startup check. Raise unless this session can only run on CPU.

  onnxruntime must be exactly CPUExecutionProvider. opencv-dnn must be the
  OpenCV CPU target. tinygrad must be the CPU/CLANG device, never QCOM,
  CUDA, METAL, or any other GPU.
  """
  if session is None:
    raise AssertionError("speedsignd session is missing")
  backend = str(getattr(session, "_nap_backend", "") or "")
  device = str(getattr(session, "_nap_device", "") or "")
  if backend == "opencv-dnn-cpu":
    if device and not _device_is_cpu(device):
      raise AssertionError("speedsignd backend is not CPU: " + backend + " device=" + device)
    return "opencv-dnn-cpu"
  get_providers = getattr(session, "get_providers", None)
  if get_providers is not None and not isinstance(session, (_TinyOrtSession, _LazyCpuSession)):
    providers = list(get_providers())
    if providers != ["CPUExecutionProvider"]:
      raise AssertionError("onnxruntime providers " + str(providers) + " are not CPU-only")
    try:
      session._nap_backend = "onnxruntime-cpu"
      session._nap_device = "CPU"
    except Exception:
      pass
    return "onnxruntime-cpu"
  if backend == "onnxruntime-cpu":
    return "onnxruntime-cpu"
  blob = (backend + " " + device).upper()
  for tok in ("QCOM", "CUDA", "METAL", "WEBGPU", "OPENCL", "GPU"):
    if tok in blob:
      raise AssertionError("speedsignd backend is not CPU: " + backend + " device=" + device)
  if not (_device_is_cpu(device) or "CPU" in blob or "CLANG" in blob):
    raise AssertionError("speedsignd backend is not CPU: " + backend + " device=" + device)
  renderer = str(getattr(session, "_nap_renderer", "") or "")
  if "CLANG" in device.upper() or "CLANG" in renderer.upper() or backend == "tinygrad-clang":
    return "tinygrad-clang"
  return "tinygrad-cpu"


def describe_infer_runtimes() -> str:
  """What this process can import. Prefer ORT, then cv2.dnn, then tinygrad."""
  pin_cpu_only_env()
  try:
    import onnxruntime as ort
    ort_s = "onnxruntime-" + str(getattr(ort, "__version__", "present"))
  except Exception as e:
    ort_s = "onnxruntime-absent(" + type(e).__name__ + ")"
  cv_s = "cv2-dnn-present" if _cv2_dnn_available() else "cv2-dnn-absent"
  try:
    import tinygrad
    tg_s = "tinygrad-" + str(getattr(tinygrad, "__version__", "present"))
  except Exception as e:
    tg_s = "tinygrad-absent(" + type(e).__name__ + ")"
  return ort_s + " " + cv_s + " " + tg_s


def _cv2_dnn_available() -> bool:
  try:
    import cv2
  except Exception:
    return False
  dnn = getattr(cv2, "dnn", None)
  return dnn is not None and hasattr(dnn, "readNetFromONNX")


def _cv_dnn_id_sets() -> tuple[set[int], set[int], int]:
  import cv2
  gpu_b: set[int] = set()
  gpu_t: set[int] = set()
  for name in (
    "DNN_BACKEND_CUDA", "DNN_BACKEND_VKCOM", "DNN_BACKEND_INFERENCE_ENGINE",
    "DNN_BACKEND_TIMVX", "DNN_BACKEND_CANN", "DNN_BACKEND_WEBNN",
  ):
    if hasattr(cv2.dnn, name):
      gpu_b.add(int(getattr(cv2.dnn, name)))
  for name in (
    "DNN_TARGET_CUDA", "DNN_TARGET_CUDA_FP16", "DNN_TARGET_OPENCL",
    "DNN_TARGET_OPENCL_FP16", "DNN_TARGET_VULKAN", "DNN_TARGET_FPGA",
    "DNN_TARGET_MYRIAD", "DNN_TARGET_HDDL", "DNN_TARGET_NPU",
  ):
    if hasattr(cv2.dnn, name):
      gpu_t.add(int(getattr(cv2.dnn, name)))
  return gpu_b, gpu_t, int(cv2.dnn.DNN_TARGET_CPU)


def _read_cv_net(path: str):
  """Built-in OpenCV CPU engine. Never ENGINE_ORT, CUDA, or OpenCL."""
  import cv2
  for name in ("ENGINE_CLASSIC", "ENGINE_OPENCV", "ENGINE_NEW"):
    engine = getattr(cv2.dnn, name, None)
    if engine is None:
      continue
    try:
      return cv2.dnn.readNetFromONNX(path, int(engine))
    except TypeError:
      break
    except Exception:
      continue
  return cv2.dnn.readNetFromONNX(path)


def _open_cv_dnn(path: str) -> _CvDnnSession:
  """OpenCV's CPU DNN runner. Never OpenCL, CUDA, or Vulkan."""
  import cv2
  cv2.setNumThreads(1)
  ocl = getattr(cv2, "ocl", None)
  if ocl is not None and hasattr(ocl, "setUseOpenCL"):
    ocl.setUseOpenCL(False)
  net = _read_cv_net(path)
  backend = int(cv2.dnn.DNN_BACKEND_OPENCV)
  target = int(cv2.dnn.DNN_TARGET_CPU)
  try:
    net.setPreferableBackend(backend)
    net.setPreferableTarget(target)
  except Exception:
    pass
  got_b, got_t = backend, target
  get_b = getattr(net, "getPreferableBackend", None)
  get_t = getattr(net, "getPreferableTarget", None)
  if get_b is not None:
    got_b = int(get_b())
  if get_t is not None:
    got_t = int(get_t())
  gpu_b, gpu_t, cpu_t = _cv_dnn_id_sets()
  if not dnn_ids_are_cpu(got_b, got_t, gpu_backends=gpu_b, gpu_targets=gpu_t, cpu_target=cpu_t):
    raise AssertionError("cv2.dnn backend=" + str(got_b) + " target=" + str(got_t) + " is not CPU")
  if ocl is not None and hasattr(ocl, "useOpenCL") and ocl.useOpenCL():
    raise AssertionError("cv2 OpenCL stayed on")
  return _CvDnnSession(net)


def _prepare_tinygrad_cache(sha: str) -> None:
  """Point tinygrad's sqlite kernel cache at /data, keyed by weights sha.

  ClangJITCompiler already passes -O2. It does not vectorize: the compile
  is freestanding C, and LLVM's SLP/loop passes are a different renderer.
  BEAM stays off (that search is the long compile). NOOPT stays off.
  The cache is what makes the clang compile once per sha instead of every boot.
  """
  pin_cpu_only_env()
  cache = compile_cache_dir(sha)
  try:
    os.makedirs(cache, exist_ok=True)
  except OSError:
    return
  os.environ["XDG_CACHE_HOME"] = cache
  os.environ["CACHEDB"] = os.path.join(cache, "tinygrad", "cache.db")
  os.environ["BEAM"] = "0"
  os.environ["NOOPT"] = "0"
  os.environ["CACHELEVEL"] = "2"
  os.environ["CPU_COUNT"] = "1"


def _open_tinygrad_cpu(sha: str = "") -> tuple[str, str]:
  """Import tinygrad only after DEV=CPU. Do not probe QCOM or any GPU."""
  _prepare_tinygrad_cache(sha)
  from tinygrad.device import Device
  from tinygrad.helpers import BEAM, CPU_COUNT, DEV, NOOPT
  DEV.value = "CPU"
  BEAM.value = 0
  NOOPT.value = 0
  CPU_COUNT.value = 1
  opened = Device["CPU"]
  name = str(getattr(opened, "device", "CPU"))
  if not _device_is_cpu(name):
    raise AssertionError("tinygrad opened " + name + ", not CPU")
  renderer = type(getattr(opened, "renderer", None)).__name__
  renderer_u = renderer.upper()
  for tok in ("QCOM", "CUDA", "METAL", "GPU", "WEBGPU"):
    if tok in renderer_u:
      raise AssertionError("tinygrad CPU renderer is " + renderer)
  if "CLANG" not in renderer_u and "LLVM" not in renderer_u and "CPU" not in renderer_u:
    raise AssertionError("tinygrad renderer " + renderer + " is not CPU/CLANG")
  return name, renderer


def _ort_session(path: str, threads: int, sha: str):
  import onnxruntime as ort
  opts = ort.SessionOptions()
  opts.intra_op_num_threads = threads
  opts.inter_op_num_threads = 1
  opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
  try:
    opts.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
  except Exception:
    pass
  optimized = os.path.join(compile_cache_dir(sha), "model.ort")
  try:
    os.makedirs(os.path.dirname(optimized), exist_ok=True)
    if not os.path.isfile(optimized):
      opts.optimized_model_filepath = optimized
  except OSError:
    optimized = ""
  model = optimized if optimized and os.path.isfile(optimized) else path
  sess = ort.InferenceSession(model, opts, providers=["CPUExecutionProvider"])
  sess._nap_backend = "onnxruntime-cpu"
  sess._nap_device = "CPU"
  sess.compiled = True
  sess._nap_sha = sha
  return sess


def _select_cpu_kind() -> str:
  """Which CPU runner this process can import. Does not load weights."""
  try:
    import onnxruntime  # noqa: F401
    return "onnxruntime-cpu"
  except Exception:
    pass
  if _cv2_dnn_available():
    return "opencv-dnn-cpu"
  try:
    import tinygrad  # noqa: F401
    return "tinygrad-clang"
  except Exception:
    return ""


def _onnx_session(path: str) -> tuple[object | None, str]:
  """Pick a CPU session. Do not realize it here — that is the compile."""
  limit_infer_threads()
  pin_cpu_only_env()
  kind = _select_cpu_kind()
  if not kind:
    return None, ""
  sha = _sha_short(path, n=64)
  sess = _LazyCpuSession(path, sha, kind)
  return sess, assert_cpu_backend(sess)


class _CvDnnSession:
  """cv2.dnn on DNN_BACKEND_OPENCV + DNN_TARGET_CPU. No OpenCL."""

  def __init__(self, net, input_name: str = "images"):
    self.net = net
    self._nap_backend = "opencv-dnn-cpu"
    self._nap_device = "CPU"
    self._nap_renderer = "opencv-dnn"
    self.compiled = False
    self._allow_compile = False
    self._inputs = [type("I", (), {"name": input_name, "shape": [1, 3, YOLO_IMGSZ, YOLO_IMGSZ]})()]

  def allow_compile(self, allow: bool) -> None:
    self._allow_compile = bool(allow)

  def get_inputs(self):
    return self._inputs

  def run(self, _outs, feed: dict):
    if not self.compiled and not self._allow_compile:
      raise RuntimeError("compile-blocked while driving")
    blob = np.ascontiguousarray(next(iter(feed.values())), dtype=np.float32)
    self.net.setInput(blob)
    out = self.net.forward()
    self.compiled = True
    return [np.asarray(out)]


class _LazyCpuSession:
  """Build the CPU net on first allowed run, not at process start.

  Session create for onnxruntime and the first tinygrad realize are the
  compile. Both used to run from SpeedSignDetector() in main(), before
  engagement was known. A warm stamp skips that wait.
  """

  def __init__(self, path: str, sha: str, kind: str):
    self.path = path
    self.sha = sha
    self.kind = kind
    self._impl = None
    self._allow_compile = False
    self._nap_backend = kind
    self._nap_device = "CPU"
    self._nap_renderer = "CLANG" if kind == "tinygrad-clang" else kind
    self.compiled = bool(sha) and compile_cache_warm(sha)
    self._inputs = [type("I", (), {"name": "images", "shape": [1, 3, YOLO_IMGSZ, YOLO_IMGSZ]})()]

  def allow_compile(self, allow: bool) -> None:
    self._allow_compile = bool(allow)
    fn = getattr(self._impl, "allow_compile", None)
    if fn is not None:
      fn(bool(allow))

  def get_inputs(self):
    return self._inputs if self._impl is None else self._impl.get_inputs()

  def _build(self):
    threads = limit_infer_threads()
    if self.kind == "onnxruntime-cpu":
      return _ort_session(self.path, threads, self.sha)
    if self.kind == "opencv-dnn-cpu":
      return _open_cv_dnn(self.path)
    device, renderer = _open_tinygrad_cpu(self.sha)
    from tinygrad import Tensor
    from tinygrad.nn.onnx import OnnxRunner
    runner = OnnxRunner(self.path)
    sess = _TinyOrtSession(
      runner, Tensor, device=device, renderer=renderer, sha=self.sha, jit=True,
    )
    if self.compiled:
      sess.compiled = True
    return sess

  def run(self, outs, feed: dict):
    if self._impl is None:
      if not self.compiled and not self._allow_compile:
        raise RuntimeError("compile-blocked while driving")
      self._impl = self._build()
      if self.compiled:
        self._impl.compiled = True
      self._nap_backend = str(getattr(self._impl, "_nap_backend", self.kind))
      self._nap_device = str(getattr(self._impl, "_nap_device", "CPU"))
      self._nap_renderer = str(getattr(self._impl, "_nap_renderer", self._nap_renderer))
      assert_cpu_backend(self._impl)
      fn = getattr(self._impl, "allow_compile", None)
      if fn is not None:
        fn(self._allow_compile)
    if not getattr(self._impl, "compiled", False) and not self._allow_compile:
      raise RuntimeError("compile-blocked while driving")
    out = self._impl.run(outs, feed)
    if getattr(self._impl, "compiled", False):
      self.compiled = True
      write_compile_stamp(self.sha, self._nap_backend)
    return out


class _TinyOrtSession:
  """onnxruntime-shaped wrapper around tinygrad OnnxRunner, CPU/CLANG only.

  Clang at -O2 is still scalar (no LLVM SLP). TinyJit keeps the captured
  graph so later runs do not retrace. The sqlite cache under /data is what
  skips clang the next boot. BEAM and NOOPT stay off.
  """

  def __init__(self, runner, tensor_cls, device: str = "CPU", renderer: str = "",
               sha: str = "", jit: bool = False):
    self.runner = runner
    self._tensor = tensor_cls
    self._nap_device = device
    self._nap_renderer = renderer
    self._nap_backend = "tinygrad-clang" if "CLANG" in renderer.upper() else "tinygrad-cpu"
    self._nap_sha = sha
    self.compiled = False
    self._allow_compile = False
    self._want_jit = bool(jit)
    self._jit = None
    names = list(getattr(runner, "graph_inputs", {}) or {"images": None})
    self._inputs = [type("I", (), {"name": names[0] if names else "images", "shape": [1, 3, YOLO_IMGSZ, YOLO_IMGSZ]})()]

  def allow_compile(self, allow: bool) -> None:
    self._allow_compile = bool(allow)

  def get_inputs(self):
    return self._inputs

  def _realize(self, feed: dict):
    tensors = {}
    for k, v in feed.items():
      tensor = self._tensor(v)
      move = getattr(tensor, "to", None)
      if move is not None:
        try:
          tensor = move("CPU")
        except Exception:
          pass
      tensors[k] = tensor
    out = self.runner(tensors)
    if isinstance(out, dict):
      val = next(iter(out.values()))
    else:
      val = out
    dev = str(getattr(val, "device", self._nap_device) or self._nap_device)
    if not _device_is_cpu(dev):
      raise AssertionError("speedsignd realized on " + dev + ", not CPU")
    self._nap_device = dev.split(":")[0].upper()
    return val.numpy() if hasattr(val, "numpy") else np.asarray(val)

  def _jit_call(self, blob):
    name = self._inputs[0].name
    return self._realize({name: blob})

  def run(self, _outs, feed: dict):
    if not self.compiled and not self._allow_compile:
      raise RuntimeError("compile-blocked while driving")
    # First realize is the compile. TinyJit captures the next call so the
    # graph is not retraced. Kernel bytes come from the /data cache.
    if self._want_jit and self.compiled:
      if self._jit is None:
        jit_cls = _tinyjit_cls()
        self._jit = jit_cls(self._jit_call) if jit_cls is not None else None
        if self._jit is None:
          self._want_jit = False
      if self._jit is not None:
        blob = next(iter(feed.values()))
        try:
          return [np.asarray(self._jit(blob))]
        except Exception:
          self._want_jit = False
          self._jit = None
    arr = self._realize(feed)
    self.compiled = True
    return [arr]


def _tinyjit_cls():
  """TinyJit captures one CLANG graph. Missing tinygrad just runs directly."""
  try:
    from tinygrad import TinyJit
    return TinyJit
  except Exception:
    pass
  try:
    from tinygrad.engine.jit import TinyJit
    return TinyJit
  except Exception:
    return None


def _is_yolo_input(shape) -> bool:
  if shape is None or len(shape) != 4:
    return False
  c = shape[1]
  return c in (3, "3")


def _onnx_input_name(session) -> str:
  try:
    return session.get_inputs()[0].name
  except Exception:
    return "image"


def _onnx_input_shape(session):
  try:
    return session.get_inputs()[0].shape
  except Exception:
    return None


def _parse_onnx_dets(raw, frame_hw: tuple[int, int], min_conf: float) -> list[SpeedSign]:
  arr = np.asarray(raw, dtype=np.float32)
  signs: list[SpeedSign] = []
  if arr.ndim == 1 and arr.size >= 2:
    arr = arr.reshape(1, -1)
  if arr.ndim == 3:
    arr = arr.reshape(-1, arr.shape[-1])
  if arr.ndim != 2:
    return []
  fh, fw = frame_hw
  for row in arr:
    if row.size == 2:
      mph, conf = float(row[0]), float(row[1])
      x, yy, bw, bh = 0, 0, fw, fh
    elif row.size >= 6:
      x, yy, bw, bh, mph, conf = (float(v) for v in row[:6])
    else:
      continue
    if conf < min_conf:
      continue
    ival = int(round(mph))
    if ival not in MUTCD_MPH:
      continue
    signs.append(SpeedSign(mph=ival, conf=float(conf), bbox=(int(x), int(yy), int(bw), int(bh))))
  signs.sort(key=lambda s: s.conf, reverse=True)
  return signs[:MAX_DET]


class SpeedSignDetector:
  """YOLO ONNX if weights exist on /data. Numpy matcher is tests/dev only."""

  def __init__(self, onnx: OnnxSpeedSignDetector | None = None, onnx_path: str | None = None):
    self.onnx_path = onnx_path
    if onnx is not None:
      self.onnx = onnx
    else:
      self.onnx = OnnxSpeedSignDetector.try_load(onnx_path)

  def weights_missing(self) -> bool:
    return self.onnx is None

  def backend_name(self) -> str:
    if self.onnx is None:
      return "numpy-mutcd"
    return self.onnx.backend or "yolo-onnx"

  def weights_sha_short(self) -> str:
    if self.onnx is not None and self.onnx.sha:
      return self.onnx.sha
    return _sha_short(self.onnx_path)

  def diag_dict(self) -> dict[str, Any]:
    if self.onnx is not None:
      return self.onnx.diag_dict()
    return {
      "backend": "numpy-mutcd",
      "frame_w": 0,
      "frame_h": 0,
      "letterbox": 0,
      "crop": (0, 0, 0, 0),
      "weights_path": self.onnx_path or "",
      "weights_sha": self.weights_sha_short(),
      "out_shape": (),
      "peak_conf": 0.0,
      "peak_name": "",
      "n_over": 0,
      "sl_peak_conf": 0.0,
      "sl_peak_name": "",
      "n_over_sl": 0,
      "top3": (),
      "posted": (),
      "refine": (),
      "error": "",
      "luma_mean": 0.0,
      "luma_std": 0.0,
      "chroma": 0,
      "prep_ms": 0.0,
      "sess_ms": 0.0,
    }

  def try_reload(self) -> bool:
    """Load ONNX if the file appeared after Settings install. True if newly loaded."""
    if self.onnx is not None:
      return False
    loaded = OnnxSpeedSignDetector.try_load(self.onnx_path)
    if loaded is None:
      return False
    self.onnx = loaded
    return True

  def compile_ready(self) -> bool:
    """False until warm-up or a per-sha cache stamp. Missing means not ready."""
    if self.onnx is None or self.onnx.session is None:
      return True
    return bool(getattr(self.onnx.session, "compiled", False))

  def allow_compile(self, allow: bool) -> None:
    sess = None if self.onnx is None else self.onnx.session
    fn = getattr(sess, "allow_compile", None)
    if fn is not None:
      fn(bool(allow))

  def detect(self, y: np.ndarray, min_conf: float | None = None, rgb: np.ndarray | None = None,
             nv12=None) -> list[SpeedSign]:
    if self.onnx is not None:
      thr = YOLO_MIN_CONF if min_conf is None else min_conf
      return self.onnx.detect(y, min_conf=thr, rgb=rgb, nv12=nv12)
    thr = MIN_CONF if min_conf is None else min_conf
    return detect_mutcd_speed_signs(y, min_conf=thr)
