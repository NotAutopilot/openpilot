"""How far ahead the driving model is confident about its own path.

One place for the path-confidence reads that more than one consumer uses:

- ``path_y_std_at``: lateral path std (m) at a time ahead. Low-visibility
  steering (lat_low_visibility) reads it at 3 s for its enter/exit tests.
- ``sight_distance_m``: the distance ahead (m) at which the lateral path std
  first reaches SIGHT_STD_M (2 m). The sight-distance speed cap
  (sight_cap) reads it.

Both come from modelV2.position (t, x, yStd). Pure functions, no state.
"""
from __future__ import annotations

import math

SIGHT_STD_M = 2.0


def path_y_std_at(times, y_stds, t_query: float = 3.0) -> float | None:
  """Interpolate lateral path std (m) at ``t_query`` seconds. None if unusable."""
  try:
    ts = [float(t) for t in (times or [])]
    ys = [float(y) for y in (y_stds or [])]
  except (TypeError, ValueError):
    return None
  n = min(len(ts), len(ys))
  pairs = [(t, y) for t, y in zip(ts[:n], ys[:n], strict=True) if math.isfinite(t) and math.isfinite(y)]
  if len(pairs) < 2:
    return None
  pairs.sort(key=lambda p: p[0])
  want = float(t_query)
  if want <= pairs[0][0]:
    return pairs[0][1]
  if want >= pairs[-1][0]:
    return pairs[-1][1]
  for (t0, y0), (t1, y1) in zip(pairs, pairs[1:], strict=False):
    if t0 <= want <= t1 and t1 > t0:
      w = (want - t0) / (t1 - t0)
      return y0 + w * (y1 - y0)
  return None


def sight_distance_m(xs, y_stds, std_m: float = SIGHT_STD_M) -> float | None:
  """Distance ahead (m) where the path's lateral std first reaches ``std_m``.

  Linear between the two path points that bracket it. The full path length
  when it never does (confident to the horizon); 0 when the first point is
  already past it. None when the arrays are unusable.
  """
  try:
    pts = [(float(x), float(y)) for x, y in zip(xs, y_stds, strict=False)]
  except (TypeError, ValueError):
    return None
  pts = [(x, y) for x, y in pts if math.isfinite(x) and math.isfinite(y)]
  if len(pts) < 2:
    return None
  if pts[0][1] >= std_m:
    return 0.0
  for (x0, y0), (x1, y1) in zip(pts, pts[1:], strict=False):
    if y1 >= std_m:
      if y1 <= y0:
        return x1
      return x0 + (std_m - y0) / (y1 - y0) * (x1 - x0)
  return pts[-1][0]


def model_sight(model, std_m: float = SIGHT_STD_M) -> float | None:
  """sight_distance_m straight from a modelV2 message."""
  try:
    pos = model.position
    return sight_distance_m(list(pos.x), list(pos.yStd), std_m)
  except Exception:
    return None
