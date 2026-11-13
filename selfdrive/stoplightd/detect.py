"""Traffic-light lamp finder for the stoplight-green chime. Pure numpy, CPU.

Works on the half-resolution grid of an NV12 road frame: luma subsampled
2x2 so it lines up with the U and V planes. A lit signal lamp is a small,
compact blob of strongly colored pixels around a bright core:

  red    V - 128 >= RED_MIN_V and the hue is red, not amber/sodium:
         V - 128 >= RED_HUE_K * (128 - U)  (amber has a much lower U)
  green  128 - V >= GREEN_MIN_V and U - 128 >= GREEN_MIN_U
         (signal green is blue-green; foliage and green signs are dull)
  core   luma >= CORE_Y somewhere inside the blob, or the blob itself is
         that bright (daytime lamps are colored edge to edge)

Calibrated on Scallywag's own drives (comma 3X road camera, 1928x1208):
red halos sit at V-128 ~ +100..+115 with U-128 ~ -20..-35, green halos at
V-128 ~ -100..-120 with U-128 ~ +30..+40, and the lamp core blows out to
white (luma 230+, neutral chroma) at night. See tests/samples/README.md.

Pixels are binned into CELL x CELL cells. A cell is colored when at least
CELL_MIN_PX of its pixels are; colored cells are joined 8-connected. Only
blobs with a lamp-like size and aspect survive. No OpenCV or scipy: they
are not on the device.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

RED = "red"
GREEN = "green"

CELL = 4                  # half-res px per cell side (8 full-res px)
CELL_MIN_PX = 3           # colored px (of 16) for a cell to count
RED_MIN_V = 40
RED_HUE_K = 1.25
RED_MAX_U = 20
GREEN_MIN_V = 40
GREEN_MIN_U = -8
COLOR_MIN_Y = 30          # ignore colored noise in black areas
CORE_Y = 170
MIN_PX = 5                # colored px per blob
MAX_CELLS_SIDE = 22       # 88 half-res px: a lamp ~6 m away; larger is a body or sign
MAX_ASPECT = 2.6
MAX_BLOBS = 24
MAX_COLORED_CELLS = 900   # sunset sky / red wall: give up on that color this frame
# Region of interest: signals hang above the horizon. Fractions of the frame height.
ROI_TOP = 0.0
ROI_BOTTOM = 0.56


@dataclass(frozen=True)
class Lamp:
  color: str
  x: float        # centroid, full-res px
  y: float
  w: float        # bbox, full-res px
  h: float
  px: int         # colored half-res px
  core: int       # bright px in the bbox
  chroma: float   # mean |V-128| of the colored px
  peak: int = -1  # brightest luma in the bbox (-1: unknown)

  @property
  def size(self) -> float:
    return max(self.w, self.h)


def color_masks(y: np.ndarray, u: np.ndarray, v: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
  """Red and green lamp pixel masks on the half-res grid. u, v are signed (minus 128)."""
  lit = y >= COLOR_MIN_Y
  red = lit & (v >= RED_MIN_V) & (u <= RED_MAX_U) & (v >= RED_HUE_K * (-u))
  green = lit & (v <= -GREEN_MIN_V) & (u >= GREEN_MIN_U)
  return red, green


def _cells(mask: np.ndarray) -> np.ndarray:
  h, w = mask.shape
  hc, wc = h // CELL, w // CELL
  m = mask[:hc * CELL, :wc * CELL].reshape(hc, CELL, wc, CELL)
  return m.sum(axis=(1, 3), dtype=np.int32)


def _components(on: np.ndarray) -> list[list[tuple[int, int]]]:
  """8-connected components of a small boolean cell grid."""
  ys, xs = np.nonzero(on)
  todo = set(zip(ys.tolist(), xs.tolist(), strict=True))
  comps = []
  while todo:
    seed = todo.pop()
    stack = [seed]
    comp = [seed]
    while stack:
      cy, cx = stack.pop()
      for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
          nb = (cy + dy, cx + dx)
          if nb in todo:
            todo.remove(nb)
            stack.append(nb)
            comp.append(nb)
    comps.append(comp)
  return comps


def _blobs(color: str, mask: np.ndarray, y: np.ndarray, v: np.ndarray, row0: int) -> list[Lamp]:
  counts = _cells(mask)
  on = counts >= CELL_MIN_PX
  if int(on.sum()) > MAX_COLORED_CELLS:
    return []
  out = []
  for comp in _components(on):
    cys = [c[0] for c in comp]
    cxs = [c[1] for c in comp]
    c0, c1 = min(cys), max(cys) + 1
    x0, x1 = min(cxs), max(cxs) + 1
    hc, wc = c1 - c0, x1 - x0
    if max(hc, wc) > MAX_CELLS_SIDE:
      continue
    # Pixel extent of the blob, one cell of margin for the core search.
    py0, py1 = c0 * CELL, c1 * CELL
    px0, px1 = x0 * CELL, x1 * CELL
    sub = mask[py0:py1, px0:px1]
    n = int(sub.sum())
    if n < MIN_PX:
      continue
    sy, sx = np.nonzero(sub)
    bw = float(sx.max() - sx.min() + 1)
    bh = float(sy.max() - sy.min() + 1)
    if max(bw, bh) / max(1.0, min(bw, bh)) > MAX_ASPECT:
      continue
    my0, my1 = max(0, py0 - CELL), min(y.shape[0], py1 + CELL)
    mx0, mx1 = max(0, px0 - CELL), min(y.shape[1], px1 + CELL)
    core = int((y[my0:my1, mx0:mx1] >= CORE_Y).sum())
    peak = int(y[py0:py1, px0:px1].max())
    chroma = float(np.abs(v[py0:py1, px0:px1][sub]).mean())
    cx = (px0 + float(sx.mean()) + 0.5) * 2.0
    cy = (row0 + py0 + float(sy.mean()) + 0.5) * 2.0
    out.append(Lamp(color, cx, cy, bw * 2.0, bh * 2.0, n, core, chroma, peak))
  out.sort(key=lambda lamp: -lamp.px)
  return out[:MAX_BLOBS]


def find_lamps(y_half: np.ndarray, u: np.ndarray, v: np.ndarray,
               roi_top: float = ROI_TOP, roi_bottom: float = ROI_BOTTOM) -> list[Lamp]:
  """Lit red / green lamps in the ROI. Inputs share the half-res grid.

  y_half: uint8 luma at half res (Y[::2, ::2]). u, v: uint8 chroma planes.
  Coordinates in the result are full-res pixels.
  """
  rows = y_half.shape[0]
  r0 = max(0, int(rows * roi_top))
  r1 = min(rows, int(rows * roi_bottom))
  if r1 - r0 < CELL:
    return []
  yy = y_half[r0:r1]
  uu = u[r0:r1].astype(np.int16) - 128
  vv = v[r0:r1].astype(np.int16) - 128
  red, green = color_masks(yy, uu, vv)
  return _blobs(RED, red, yy, vv, r0) + _blobs(GREEN, green, yy, vv, r0)


def find_lamps_nv12(y_full: np.ndarray, u: np.ndarray, v: np.ndarray, **kw) -> list[Lamp]:
  """Same, from a full-res luma plane and the half-res U / V planes."""
  return find_lamps(y_full[::2, ::2], u, v, **kw)
