"""Two-stage speed-sign reader: cheap strip proposer, IoU tracker, one sharp read.

Stage 1 (find): a 1-class "sign" YOLOv8 (width 0.125) looks at the lower-right
strip x 964-1928, y 525-875 of the 1928x1208 ROAD frame, resized to 352x128.
~0.3 GFLOP, ~110 ms on one 3X core, so 2 Hz is ~22 % of a core. Skipped below
2 m/s (GPS speed / modelV2 velocity; no carState subscriber).

Stage 2 (read): proposals are linked into tracks (IoU or center distance).
A track must be seen twice. When it is near its largest (box height >= 60 px,
near the right edge, growth has stopped, or the track was lost) we copy one
square native crop around it (3x the box, clamped 240-480 px), letterbox it to
320 and run the existing v2 detector + refine_mph digit check once (up to two
retries on later frames if nothing >= YOLO_MIN_CONF). A track whose read is
neg_yellow / neg_other is suppressed and never read again. HUD rules
(LiveSignHold / SignDebounce) are unchanged; reads feed them like before.

Weights: Release speed-sign-proposer-v1 / sign_proposer.onnx (Install weights).
When the proposer file is missing, speedsignd keeps the legacy fixed tiles.
"""
from __future__ import annotations

import json
import os
import re
import shutil
import time
from dataclasses import dataclass, field

import numpy as np

STRIP_RECT = (964, 525, 964, 350)  # x, y, w, h in the 1928x1208 frame
PROPOSER_IN_W = 352
PROPOSER_IN_H = 128
PROPOSER_CONF = 0.15
PROPOSER_IOU = 0.5
PROPOSER_HZ = 2.0
PROPOSER_MIN_SPEED_MS = 2.0

TRACK_IOU = 0.3
TRACK_MAX_GAP_S = 1.6
TRACK_MIN_HITS = 2
CAPTURE_MIN_H = 60
CAPTURE_EDGE_X = 1928 - 150
CAPTURE_MIN_SIDE = 240
CAPTURE_MAX_SIDE = 480
CAPTURE_SCALE = 3.0
MAX_READS_PER_TRACK = 3
NEG_NAMES = ("neg_yellow", "neg_other")
NEG_SUPPRESS_CONF = 0.40


def resize_area(img: np.ndarray, h: int, w: int) -> np.ndarray:
  if img.shape[0] == h and img.shape[1] == w:
    return img
  try:
    import cv2
    return cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
  except Exception:
    from openpilot.selfdrive.speedsignd.nv12 import _resize_gray_bilinear
    return _resize_gray_bilinear(img, h, w)


def strip_rgb_from_crop(crop) -> np.ndarray:
  """NV12 strip crop -> 128x352 RGB uint8 (area resize, BT.601)."""
  from openpilot.selfdrive.speedsignd.nv12 import _bt601_yuv_to_rgb
  y_s = resize_area(crop.y, PROPOSER_IN_H, PROPOSER_IN_W)
  uv = crop.uv
  if uv is None or uv.size == 0 or uv.shape[1] < 2:
    return np.stack([y_s, y_s, y_s], axis=-1)
  u = resize_area(np.ascontiguousarray(uv[:, 0::2]), PROPOSER_IN_H, PROPOSER_IN_W)
  v = resize_area(np.ascontiguousarray(uv[:, 1::2]), PROPOSER_IN_H, PROPOSER_IN_W)
  return _bt601_yuv_to_rgb(y_s, u, v)


def _nms(boxes: np.ndarray, scores: np.ndarray, iou: float) -> list[int]:
  order = list(np.argsort(-scores))
  keep: list[int] = []
  while order:
    i = order.pop(0)
    keep.append(int(i))
    order = [j for j in order if box_iou(boxes[i], boxes[j]) < iou]
  return keep


def decode_proposals(raw: np.ndarray, crop_xywh, conf: float = PROPOSER_CONF,
                     in_w: int = PROPOSER_IN_W, in_h: int = PROPOSER_IN_H) -> list[tuple[float, float, float, float, float]]:
  """[1,5,N] (cx,cy,w,h,score) at model scale -> frame (x, y, w, h, score), top-left xywh."""
  out = np.asarray(raw)
  if out.ndim == 3:
    out = out[0]
  if out.shape[0] != 5 and out.shape[-1] == 5:
    out = out.T
  sc = out[4]
  idx = np.where(sc >= conf)[0]
  if idx.size == 0:
    return []
  cx0, cy0, cw, ch = (float(v) for v in crop_xywh)
  sx = cw / float(in_w)
  sy = ch / float(in_h)
  b = out[:4, idx].T
  boxes = np.stack([
    (b[:, 0] - b[:, 2] / 2) * sx + cx0,
    (b[:, 1] - b[:, 3] / 2) * sy + cy0,
    b[:, 2] * sx,
    b[:, 3] * sy,
  ], axis=1)
  s = sc[idx].astype(np.float32)
  keep = _nms(boxes, s, PROPOSER_IOU)[:8]
  return [(float(boxes[i, 0]), float(boxes[i, 1]), float(boxes[i, 2]), float(boxes[i, 3]), float(s[i])) for i in keep]


class Proposer:
  """onnxruntime CPU, one thread. None-safe: .ok is False when weights/runtime missing."""

  def __init__(self, path: str):
    self.path = path
    self.session = None
    self.error = ""
    self.load()

  def load(self) -> bool:
    if not os.path.isfile(self.path):
      self.error = "missing"
      return False
    try:
      import onnxruntime as ort
      so = ort.SessionOptions()
      so.intra_op_num_threads = 1
      so.inter_op_num_threads = 1
      so.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
      self.session = ort.InferenceSession(self.path, so, providers=["CPUExecutionProvider"])
      self._inp = self.session.get_inputs()[0].name
      self.error = ""
      return True
    except Exception as e:
      self.session = None
      self.error = f"{type(e).__name__}: {e}"[:120]
      return False

  @property
  def ok(self) -> bool:
    return self.session is not None

  def propose(self, crop) -> list[tuple[float, float, float, float, float]]:
    if self.session is None or crop is None:
      return []
    rgb = strip_rgb_from_crop(crop)
    blob = rgb.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    raw = self.session.run(None, {self._inp: blob})[0]
    return decode_proposals(raw, crop.crop)


def box_iou(a, b) -> float:
  ax0, ay0, aw, ah = a[0], a[1], a[2], a[3]
  bx0, by0, bw, bh = b[0], b[1], b[2], b[3]
  ix = max(0.0, min(ax0 + aw, bx0 + bw) - max(ax0, bx0))
  iy = max(0.0, min(ay0 + ah, by0 + bh) - max(ay0, by0))
  inter = ix * iy
  union = aw * ah + bw * bh - inter
  return float(inter / union) if union > 0 else 0.0


def _near(a, b) -> bool:
  acx, acy = a[0] + a[2] / 2, a[1] + a[3] / 2
  bcx, bcy = b[0] + b[2] / 2, b[1] + b[3] / 2
  tol = max(a[2], a[3], b[2], b[3]) + 40
  # Signs drift outward (right/up) and grow; allow a larger forward step.
  # At 2 Hz a near sign can move ~300 px right per sample; never far left.
  return -tol < (bcx - acx) < tol * 3.0 and abs(acy - bcy) < tol


@dataclass
class Track:
  tid: int
  box: tuple[float, float, float, float]
  score: float
  first_t: float
  last_t: float
  hits: int = 1
  peak_h: float = 0.0
  reads: int = 0
  neg: bool = False
  done: bool = False
  history: list = field(default_factory=list)

  def predicted(self, t: float) -> tuple[float, float, float, float]:
    """Constant growth / drift from the last two boxes."""
    if len(self.history) < 2:
      return self.box
    (t0, b0), (t1, b1) = self.history[-2], self.history[-1]
    dt = max(1e-3, t1 - t0)
    k = min(2.0, (t - t1) / dt)
    return tuple(b1[i] + (b1[i] - b0[i]) * k for i in range(4))  # type: ignore[return-value]


class SignTracker:
  def __init__(self):
    self.tracks: list[Track] = []
    self._next = 1

  def update(self, props, t: float) -> list[Track]:
    used = set()
    for tr in self.tracks:
      if tr.done:
        continue
      best, bi = 0.0, -1
      for i, p in enumerate(props):
        if i in used:
          continue
        v = box_iou(tr.box, p)
        if v < TRACK_IOU and _near(tr.box, p):
          v = max(v, TRACK_IOU)
        if v > best:
          best, bi = v, i
      if bi >= 0 and best >= TRACK_IOU:
        p = props[bi]
        used.add(bi)
        tr.box = (p[0], p[1], p[2], p[3])
        tr.score = max(tr.score, p[4])
        tr.hits += 1
        tr.last_t = t
        tr.peak_h = max(tr.peak_h, p[3])
        tr.history = (tr.history + [(t, tr.box)])[-4:]
    for i, p in enumerate(props):
      if i in used:
        continue
      tr = Track(tid=self._next, box=(p[0], p[1], p[2], p[3]), score=p[4], first_t=t, last_t=t, peak_h=p[3])
      tr.history = [(t, tr.box)]
      self._next += 1
      self.tracks.append(tr)
    self.tracks = [tr for tr in self.tracks if t - tr.last_t <= TRACK_MAX_GAP_S * 2 and not (tr.done and t - tr.last_t > 1.0)]
    return self.tracks

  def capture_candidate(self, t: float) -> Track | None:
    """Track due for its sharp read now, or None. Biggest first."""
    due = []
    for tr in self.tracks:
      if tr.done or tr.neg or tr.hits < TRACK_MIN_HITS or tr.reads >= MAX_READS_PER_TRACK:
        continue
      x, y, w, h = tr.predicted(t)
      lost = t - tr.last_t > 0.6
      if h >= CAPTURE_MIN_H or x + w >= CAPTURE_EDGE_X or lost or (tr.reads > 0):
        due.append(tr)
    if not due:
      return None
    return max(due, key=lambda tr: tr.predicted(t)[3])

  def mark_read(self, tr: Track, *, got_mph: bool, neg: bool) -> None:
    tr.reads += 1
    if neg:
      tr.neg = True
      tr.done = True
    elif got_mph or tr.reads >= MAX_READS_PER_TRACK:
      tr.done = True


def capture_rect(box, frame_w: int = 1928, frame_h: int = 1208) -> tuple[int, int, int, int]:
  x, y, w, h = (float(v) for v in box)
  side = int(min(CAPTURE_MAX_SIDE, max(CAPTURE_MIN_SIDE, CAPTURE_SCALE * max(w, h))))
  side = min(side, frame_w, frame_h)
  side -= side % 2
  cx, cy = x + w / 2, y + h / 2
  x0 = int(min(max(0, cx - side / 2), frame_w - side))
  y0 = int(min(max(0, cy - side / 2), frame_h - side))
  x0 -= x0 % 2
  y0 -= y0 % 2
  return x0, y0, side, side


def neg_read(diag: dict | None) -> bool:
  if not diag:
    return False
  name = str(diag.get("peak_name") or "")
  try:
    conf = float(diag.get("peak_conf") or 0.0)
  except Exception:
    conf = 0.0
  return name in NEG_NAMES and conf >= NEG_SUPPRESS_CONF


def speed_from_sm(sm) -> float | None:
  """Ego speed without carState: GPS speed first, then modelV2 velocity."""
  for svc in ("gpsLocationExternal", "gpsLocation"):
    try:
      g = sm[svc]
      if bool(getattr(g, "hasFix", False)) or int(getattr(g, "flags", 0) or 0) & 1:
        return float(g.speed)
    except Exception:
      pass
  try:
    vx = sm["modelV2"].velocity.x
    if len(vx):
      return float(vx[0])
  except Exception:
    pass
  return None


def proposals_allowed(speed_ms: float | None) -> bool:
  """Skip the proposer when stopped. Unknown speed is not 'stopped'."""
  return speed_ms is None or speed_ms >= PROPOSER_MIN_SPEED_MS


# ---------------------------------------------------------------- feedback
FEEDBACK_DIRNAME = "speedsign_feedback"
PENDING_JSON = "pending.json"
PENDING_JPG = "pending.jpg"
FEEDBACK_JSONL = "feedback.jsonl"
CONFIRMED_JSONL = "confirmed.jsonl"
FEEDBACK_MAX_BYTES = 50 * 1024 * 1024


def feedback_dir() -> str:
  override = os.environ.get("NAP_SPEED_SIGN_FEEDBACK")
  if override:
    return override
  from openpilot.selfdrive.speedsignd.paths import nap_data_dir
  return os.path.join(nap_data_dir(), FEEDBACK_DIRNAME)


def crop_jpeg_bytes(crop) -> bytes | None:
  try:
    import cv2
    from openpilot.selfdrive.speedsignd.nv12 import _rgb_from_resized_nv12
    rgb = _rgb_from_resized_nv12(crop.y, crop.uv)
    ok, enc = cv2.imencode(".jpg", rgb[:, :, ::-1], [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    return bytes(enc) if ok else None
  except Exception:
    return None


def write_pending(meta: dict, jpg: bytes | None, d: str | None = None) -> bool:
  """speedsignd: the read now on the plate (overwritten each new plate). Atomic."""
  d = d or feedback_dir()
  try:
    os.makedirs(d, exist_ok=True)
    if jpg:
      with open(os.path.join(d, PENDING_JPG + ".tmp"), "wb") as f:
        f.write(jpg)
      os.replace(os.path.join(d, PENDING_JPG + ".tmp"), os.path.join(d, PENDING_JPG))
    m = dict(meta)
    m["has_jpg"] = bool(jpg)
    with open(os.path.join(d, PENDING_JSON + ".tmp"), "w") as f:
      json.dump(m, f)
    os.replace(os.path.join(d, PENDING_JSON + ".tmp"), os.path.join(d, PENDING_JSON))
    return True
  except Exception:
    return False


def _dir_bytes(d: str) -> int:
  n = 0
  try:
    for e in os.scandir(d):
      if e.is_file():
        n += e.stat().st_size
  except Exception:
    pass
  return n


REALDATA_PREFIX = "speedsign_"
_SEG_RE = re.compile(r"^[0-9a-f]{8}--[0-9a-f]{10}--\d+$")
CURRENT_SEG_MAX_AGE_S = 300.0


def current_segment_dir(root: str | None = None, now: float | None = None) -> str | None:
  """Newest loggerd segment folder under realdata, if written in the last 5 min."""
  if root is None:
    try:
      from openpilot.system.hardware.hw import Paths
      root = Paths.log_root()
    except Exception:
      return None
  now = time.time() if now is None else now  # noqa: TID251  compare with file mtimes
  best, best_m = None, -1.0
  try:
    with os.scandir(root) as it:
      for e in it:
        if not (_SEG_RE.match(e.name) and e.is_dir(follow_symlinks=False)):
          continue
        m = e.stat(follow_symlinks=False).st_mtime
        if m > best_m:
          best, best_m = e.path, m
  except OSError:
    return None
  if best is None or now - best_m > CURRENT_SEG_MAX_AGE_S:
    return None
  return best


def copy_to_realdata(rec: dict, line: str, d: str, root: str | None = None, now: float | None = None) -> str | None:
  """Mirror one feedback row (+ crop) into the current route segment folder.

  athena uploadFilesToUrls only reaches files under realdata, so harvest can
  pull speedsign_feedback.jsonl / speedsign_<stamp>_<yes|no>_<mph>.jpg per
  segment. The uploader never auto-sends these names (not qlog/qcamera).
  """
  seg = current_segment_dir(root, now=now)
  if seg is None:
    return None
  try:
    with open(os.path.join(seg, REALDATA_PREFIX + FEEDBACK_JSONL), "a") as f:
      f.write(line)
    jpg = rec.get("jpg")
    if jpg and os.path.isfile(os.path.join(d, jpg)):
      shutil.copyfile(os.path.join(d, jpg), os.path.join(seg, REALDATA_PREFIX + jpg))
    return os.path.basename(seg)
  except OSError:
    return None


def record_answer(displayed_mph: int, yes: bool, d: str | None = None, now: float | None = None) -> dict | None:
  """UI Yes/No: append feedback.jsonl (+confirmed.jsonl on Yes), keep the crop JPEG.

  Uses the pending read only when its mph matches the plate the driver saw.
  No upload, no cruise / HUD / map-speed change.
  """
  d = d or feedback_dir()
  now = time.time() if now is None else now  # noqa: TID251  wall clock for feedback rows
  try:
    os.makedirs(d, exist_ok=True)
    meta: dict = {}
    try:
      with open(os.path.join(d, PENDING_JSON)) as f:
        meta = json.load(f)
    except Exception:
      meta = {}
    if int(meta.get("mph", -1)) != int(displayed_mph):
      meta = {"mph": int(displayed_mph), "stale_pending": True}
    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(now))
    rec = {
      "t": round(now, 3),
      "answer": "yes" if yes else "no",
      "displayed_mph": int(displayed_mph),
      "class": meta.get("class"),
      "conf": meta.get("conf"),
      "refine_mph": meta.get("refine_mph"),
      "refine_conf": meta.get("refine_conf"),
      "track_id": meta.get("track_id"),
      "read_t": meta.get("t"),
      "lat": meta.get("lat"),
      "lon": meta.get("lon"),
      "bearing": meta.get("bearing"),
      "gps_ok": meta.get("gps_ok"),
      "crop": meta.get("crop"),
      "stale_pending": bool(meta.get("stale_pending", False)),
    }
    src = os.path.join(d, PENDING_JPG)
    if meta.get("has_jpg") and not rec["stale_pending"] and os.path.isfile(src) and _dir_bytes(d) < FEEDBACK_MAX_BYTES:
      name = f"{stamp}_{rec['answer']}_{int(displayed_mph)}.jpg"
      shutil.copyfile(src, os.path.join(d, name))
      rec["jpg"] = name
    line = json.dumps(rec, separators=(",", ":")) + "\n"
    with open(os.path.join(d, FEEDBACK_JSONL), "a") as f:
      f.write(line)
    if yes and not rec["stale_pending"]:
      with open(os.path.join(d, CONFIRMED_JSONL), "a") as f:
        f.write(line)
    rec["realdata"] = copy_to_realdata(rec, line, d, now=now)
    return rec
  except Exception:
    return None
