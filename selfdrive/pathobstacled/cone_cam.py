"""Camera confirm for the radar cone line. Log only: nothing reads the result.

radard publishes coneLineNAP (side plus the line's path-relative lateral at
15/30/45 m). While that line has posts, pathobstacled grabs one ROAD frame
at most 5 Hz and scores up to six ground windows along the line (8–48 m).
Each window is projected with the calibrated camera into a pixel box that
covers 0–1.2 m above the road and ±0.6 m around the line. The box is
compared with a road patch 1.3 m toward the lane:

  orange   V above the road by ORANGE_DV and U below it by ORANGE_DU
  white    luma above the road by WHITE_DY with near-neutral chroma
  banding  alternating orange / white row runs, the drum and post stripes

A window is a hit when its orange area is a real fraction of a drum or a
post at that range, plus either banding or a drum-sized orange area. The
line is camera-confirmed while, within the last CONFIRM_WINDOW_S, one frame
had two or more windows on the line's side hit. A lone orange truck or building can never make a
line: a window is only scored where radar already sees a line.

Pure numpy on the CPU. No tinygrad, onnx, or OpenCL: the GPU belongs to
modeld (Oct 10, a second GPU tenant pushed modeld exec from ~25 ms to
300–710 ms and raised commIssue).
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np


# Six ground windows (x0, x1) in metres ahead of the camera.
WINDOWS_M = ((8.0, 12.0), (12.0, 17.0), (17.0, 23.0), (23.0, 30.0), (30.0, 38.0), (38.0, 48.0))
MAX_CROPS = 6
LOOKAHEADS_M = (15.0, 30.0, 45.0)
HALF_WIDTH_M = 0.60
TOP_M = 1.20
ROAD_SHIFT_M = 1.30
ROAD_TOP_M = 0.0
ORANGE_DV = 14.0
ORANGE_DU = 8.0
ORANGE_MIN_V = 140.0
WHITE_DY = 45.0
WHITE_CHROMA = 14.0
# Visible face of a tubular post and of a drum, metres (w, h).
POST_WH = (0.10, 0.90)
DRUM_WH = (0.55, 0.95)
HIT_POST_FRAC = 0.35
HIT_DRUM_FRAC = 0.30
MIN_BANDS = 2
MIN_ORANGE_PX = 6
CONFIRM_HITS = 2
CONFIRM_WINDOW_S = 1.0
LOG_HZ = 2.0
BUDGET_S = 0.008
# Chroma samples per box. Near windows are large; they are strided, not dropped.
MAX_SAMPLES = 12000


@dataclass(frozen=True)
class CamGeom:
  width: int = 1928
  height: int = 1208
  focal: float = 2648.0
  roll: float = 0.0
  pitch: float = 0.0
  yaw: float = 0.0
  cam_height: float = 1.22


@dataclass(frozen=True)
class CropScore:
  x0: float
  x1: float
  hit: bool
  conf: float
  orange_px: int
  white_px: int
  bands: int
  box: tuple


def _lat_at(lats: tuple[float, float, float], x: float) -> float:
  """Linear in x through the three lookaheads, clamped at the ends."""
  xs = LOOKAHEADS_M
  if x <= xs[0]:
    return lats[0]
  if x >= xs[-1]:
    return lats[-1]
  for i in range(len(xs) - 1):
    if xs[i] <= x <= xs[i + 1]:
      t = (x - xs[i]) / (xs[i + 1] - xs[i])
      return lats[i] + t * (lats[i + 1] - lats[i])
  return lats[-1]


def _project_many(geom: CamGeom, xs, lat_right, zs):
  """Vectorized project_road_point (same math). Returns (u, v) or None if any point is behind."""
  x = np.asarray(xs, dtype=np.float64)
  y = -np.asarray(lat_right, dtype=np.float64)
  z = np.asarray(zs, dtype=np.float64)
  cr, sr = math.cos(geom.roll), math.sin(geom.roll)
  cp, sp = math.cos(geom.pitch), math.sin(geom.pitch)
  cyaw, syaw = math.cos(geom.yaw), math.sin(geom.yaw)
  rx, ry, rz = x, -y, -z
  ry, rz = cr * ry - sr * rz, sr * ry + cr * rz
  rx, rz = cp * rx + sp * rz, -sp * rx + cp * rz
  rx, ry = cyaw * rx - syaw * ry, syaw * rx + cyaw * ry
  vx, vy, vz = ry, rz + geom.cam_height, rx
  if not np.all(np.isfinite(vz)) or np.any(vz <= 0.5):
    return None
  return geom.width / 2.0 + geom.focal * (vx / vz), geom.height / 2.0 + geom.focal * (vy / vz)


_CORNERS = np.array([(i, j, k) for i in (0, 1) for j in (-1.0, 1.0) for k in (0, 1)], dtype=np.float64)


def ground_box(geom: CamGeom, x0: float, x1: float, lat0: float, lat1: float,
               half_w: float, z0: float, z1: float) -> tuple[int, int, int, int] | None:
  """Pixel bounding box of a ground-frame block. None when off-frame or behind."""
  sel = _CORNERS[:, 0]
  xs = np.where(sel > 0, x1, x0)
  lats = np.where(sel > 0, lat1, lat0) + _CORNERS[:, 1] * half_w
  zs = np.where(_CORNERS[:, 2] > 0, z1, z0)
  got = _project_many(geom, xs, lats, zs)
  if got is None:
    return None
  us, vs = got
  if not (np.all(np.isfinite(us)) and np.all(np.isfinite(vs))):
    return None
  u0 = max(0, int(math.floor(float(us.min()))))
  u1 = min(geom.width, int(math.ceil(float(us.max()))))
  v0 = max(0, int(math.floor(float(vs.min()))))
  v1 = min(geom.height, int(math.ceil(float(vs.max()))))
  if u1 - u0 < 4 or v1 - v0 < 4:
    return None
  return u0, v0, u1, v1


def _object_px(geom: CamGeom, x: float, wh: tuple[float, float]) -> float:
  return max(1.0, (geom.focal * wh[0] / x) * (geom.focal * wh[1] / x))


def _chroma(plane, box):
  """Half-resolution U or V crop for a full-resolution box."""
  u0, v0, u1, v1 = box
  h, w = plane.shape[:2]
  x0, x1 = max(0, u0 // 2), min(w, max(u0 // 2 + 1, u1 // 2))
  y0, y1 = max(0, v0 // 2), min(h, max(v0 // 2 + 1, v1 // 2))
  return plane[y0:y1, x0:x1]


def _bands(orange_rows: np.ndarray, white_rows: np.ndarray, width: int) -> int:
  """Alternations between orange-dominant and white-dominant row runs."""
  need = max(1, int(0.08 * width))
  labels = np.where(orange_rows >= np.maximum(need, white_rows), 1,
                    np.where(white_rows >= np.maximum(need, orange_rows), 2, 0))
  seq = labels[labels > 0]
  if seq.size < 2:
    return 0
  return int(np.count_nonzero(seq[1:] != seq[:-1]))


def score_window(y: np.ndarray, u: np.ndarray, v: np.ndarray, geom: CamGeom,
                 x0: float, x1: float, lats: tuple[float, float, float]) -> CropScore | None:
  """Score one ground window. None when it does not land in the frame."""
  la, lb = _lat_at(lats, x0), _lat_at(lats, x1)
  box = ground_box(geom, x0, x1, la, lb, HALF_WIDTH_M, 0.0, TOP_M)
  if box is None:
    return None
  # Road patch toward the lane (lat -> 0), on the ground.
  toward = -ROAD_SHIFT_M if (la + lb) > 0 else ROAD_SHIFT_M
  rbox = ground_box(geom, x0, x1, la + toward, lb + toward, 0.35, ROAD_TOP_M, 0.0)
  if rbox is None or (rbox[3] - rbox[1]) < 2:
    rbox = ground_box(geom, x0, x1, la + toward, lb + toward, 0.35, 0.0, 0.15)
  if rbox is None:
    return None
  u0, v0, u1, v1 = box
  yb = y[v0:v1, u0:u1]
  ub = _chroma(u, box)
  vb = _chroma(v, box)
  if yb.size == 0 or ub.size == 0 or vb.size == 0:
    return None
  ry = y[rbox[1]:rbox[3], rbox[0]:rbox[2]]
  ru = _chroma(u, rbox)
  rv = _chroma(v, rbox)
  if ry.size == 0 or ru.size == 0 or rv.size == 0:
    return None
  rs = max(1, int(math.sqrt(ru.size / 1500.0)))
  y_r = float(np.median(ry[::2 * rs, ::2 * rs]))
  u_r = float(np.median(ru[::rs, ::rs]))
  v_r = float(np.median(rv[::rs, ::rs]))
  # Chroma at half resolution; luma subsampled to match.
  ch, cw = min(ub.shape[0], vb.shape[0]), min(ub.shape[1], vb.shape[1])
  step = 1
  while (ch // step) * (cw // step) > MAX_SAMPLES:
    step += 1
  ub = ub[:ch:step, :cw:step].astype(np.int16)
  vb = vb[:ch:step, :cw:step].astype(np.int16)
  ys = yb[: 2 * ch : 2 * step, : 2 * cw : 2 * step].astype(np.int16)
  ch, cw = min(ub.shape[0], ys.shape[0]), min(ub.shape[1], ys.shape[1])
  ub, vb, ys = ub[:ch, :cw], vb[:ch, :cw], ys[:ch, :cw]
  orange = (vb >= v_r + ORANGE_DV) & (ub <= u_r - ORANGE_DU) & (vb >= ORANGE_MIN_V)
  white = (ys >= y_r + WHITE_DY) & (np.abs(ub - 128) <= WHITE_CHROMA) & (np.abs(vb - 128) <= WHITE_CHROMA)
  o_rows = orange.sum(axis=1)
  w_rows = white.sum(axis=1)
  scale = 4 * step * step  # one strided half-res sample -> full-res area
  orange_px = int(o_rows.sum()) * scale
  white_px = int(w_rows.sum()) * scale
  bands = _bands(o_rows, w_rows, cw)
  xm = 0.5 * (x0 + x1)
  post = _object_px(geom, xm, POST_WH)
  drum = _object_px(geom, xm, DRUM_WH)
  post_like = orange_px >= max(MIN_ORANGE_PX * 4, HIT_POST_FRAC * post) and bands >= MIN_BANDS
  drum_like = orange_px >= HIT_DRUM_FRAC * drum and white_px > 0 and bands >= 1
  hit = bool(post_like or drum_like)
  conf = min(1.0, orange_px / max(1.0, HIT_DRUM_FRAC * drum))
  if bands >= MIN_BANDS:
    conf = min(1.0, conf + 0.25)
  return CropScore(x0, x1, hit, round(conf if hit else min(conf, 0.49), 3), orange_px, white_px, bands, box)


def score_line(y, u, v, geom: CamGeom, lats: tuple[float, float, float],
               deadline: float | None = None, start: int = 0) -> tuple[list[CropScore], bool]:
  """Score up to MAX_CROPS windows, starting at `start` (rotates after a budget stop).

  Second value is True when the budget ran out before every window was scored.
  """
  out: list[CropScore] = []
  n = min(MAX_CROPS, len(WINDOWS_M))
  for k in range(n):
    x0, x1 = WINDOWS_M[(start + k) % n]
    if deadline is not None and time.monotonic() >= deadline:
      return out, True
    s = score_window(y, u, v, geom, x0, x1, lats)
    if s is not None:
      out.append(s)
  return out, False


@dataclass
class ConeCamState:
  """Last 1 s of per-frame window hits for one side, and the 2 Hz log gate.

  Posts "agree" when two or more different windows hit in the same frame.
  A single window cannot confirm across frames: one orange object slides
  through neighbouring windows as the car moves.
  """
  side: int = 0
  frames: list = field(default_factory=list)  # (t, hit window count)
  last_log: float = -1e9

  def update(self, side: int, scores: list[CropScore], now: float) -> bool:
    if side != self.side:
      self.side = side
      self.frames = []
    hit_windows = {(s.x0, s.x1) for s in scores if s.hit}
    self.frames.append((now, len(hit_windows)))
    self.frames = [f for f in self.frames if now - f[0] <= CONFIRM_WINDOW_S]
    return self.confirmed(now)

  def confirmed(self, now: float) -> bool:
    return self.side != 0 and any(now - t <= CONFIRM_WINDOW_S and n >= CONFIRM_HITS for t, n in self.frames)

  def should_log(self, now: float) -> bool:
    if now - self.last_log >= 1.0 / LOG_HZ:
      self.last_log = now
      return True
    return False


def uv_planes_from_nv12(buf):
  """(U, V) half-resolution views of an NV12 VisionBuf. None when chroma is missing."""
  try:
    width = int(buf.width)
    height = int(buf.height)
    stride = int(buf.stride) if getattr(buf, "stride", 0) else width
    if width < 4 or height < 4 or stride < width:
      return None
    data = buf.data
    uv_off = getattr(buf, "uv_offset", 0) or stride * height
    uv_rows = height // 2
    if data is None or len(data) < uv_off + stride * uv_rows:
      return None
    uv = np.frombuffer(data, dtype=np.uint8, count=stride * uv_rows, offset=uv_off).reshape(uv_rows, stride)
    return uv[:, 0:width:2], uv[:, 1:width:2]
  except Exception:
    return None
