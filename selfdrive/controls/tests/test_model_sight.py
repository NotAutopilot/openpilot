"""Shared path-confidence helper: low-visibility steering reads it unchanged."""
import math
import random

from openpilot.selfdrive.controls.lib import lat_low_visibility as lv
from openpilot.selfdrive.controls.lib.model_sight import path_y_std_at, sight_distance_m


def _frozen_path_y_std_at(times, y_stds, t_query=3.0):
  """lat_low_visibility.path_y_std_at as it was before the move (99b1d391)."""
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


def test_low_vis_reads_the_shared_helper_with_identical_results():
  rng = random.Random(11)
  cases = [([], []), (None, None), ([0.0], [1.0]), ([0.0, 1.0], [0.1, float("nan")]), (["x", 1], [0, 1])]
  for _ in range(400):
    n = rng.randint(0, 40)
    ts = sorted(rng.uniform(0, 10) for _ in range(n))
    ys = [rng.choice([rng.uniform(0, 5), float("inf"), float("nan")]) if rng.random() < 0.05 else rng.uniform(0, 5)
          for _ in range(rng.randint(max(0, n - 2), n + 2))]
    cases.append((ts, ys))
  for ts, ys in cases:
    for q in (lv.PATH_T_S, 0.0, 1.7, 3.0, 12.0):
      assert lv.path_y_std_at(ts, ys, q) == _frozen_path_y_std_at(ts, ys, q) == path_y_std_at(ts, ys, q)
    assert lv.path_y_std_at(ts, ys) == _frozen_path_y_std_at(ts, ys, lv.PATH_T_S)


def test_sight_distance_crosses_2m_linearly():
  assert sight_distance_m([0, 50, 100], [0.0, 1.0, 3.0]) == 75.0
  assert sight_distance_m([0, 50, 100], [0.0, 1.0, 1.5]) == 100.0
  assert sight_distance_m([0, 50], [2.5, 3.0]) == 0.0
  assert sight_distance_m([0], [0.0]) is None
