"""Oct 14 cone corridor (route 194 seg 25, Ridgewood Rd 08:28:47-08:29:14 CT).

Cones on the centreline and barrels on the right shoulder. The one-line
detector tracked only the barrels, shifted LEFT and drifted into the left
cones. Both sides are now tracked, the shift aims at the midpoint, nothing
moves to within 1.5 m of a tracked line, and the shift eases out after the
corridor ends.
"""
import random
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib.cone_line import (
  CORRIDOR_MARGIN_M,
  CORRIDOR_RELEASE_S,
  DROP_S,
  WOULD_STEER_CAP_M,
  ConeCorridorDetector,
  ConeLineDetector,
  corridor_bounds,
  corridor_target,
)
from openpilot.selfdrive.controls.lib.cone_line_hold import ConeLineHold

V = 13.5  # ~30 mph, the seg 25 speed
DT = 0.125
PX = [0.0, 30.0, 70.0]
PY = [0.0, 0.0, 0.0]
XS = [10, 17, 24, 31, 38, 46, 54]


def _pts(y_rel, xs=XS):
  return [SimpleNamespace(dRel=x - 1.52, yRel=y_rel, vRel=-V, measured=True) for x in xs]


def _corridor(left_m, right_m):
  # yRel +left: left cones positive, right barrels negative.
  return _pts(left_m) + _pts(-right_m, [x + 3 for x in XS])


def _run(det, pts, seconds):
  out = []
  for _ in range(int(round(seconds / DT))):
    out.append(det.update(pts, V, PX, PY, None, DT))
  return out


def test_both_sides_tracked_and_steer_to_midpoint():
  det = ConeCorridorDetector()
  s = _run(det, _corridor(1.9, 2.3), 5.0)[-1]
  assert s.active and s.corridor and s.left_active and s.right_active
  assert s.left_clear == pytest.approx(1.9, abs=0.05)
  assert s.right_clear == pytest.approx(2.3, abs=0.05)
  assert s.would_steer == pytest.approx(0.2, abs=0.03)  # midpoint, slightly right
  assert s.margin >= CORRIDOR_MARGIN_M


def test_seg25_never_steers_toward_the_untracked_or_tracked_left_cones():
  # Legacy single-line detector: picks one side and can shift toward the other.
  legacy = ConeLineDetector()
  for _ in range(40):
    old = legacy.update(_corridor(1.6, 2.3), V, PX, PY, None, DT)
  det = ConeCorridorDetector()
  outs = _run(det, _corridor(1.6, 2.3), 5.0)
  lo, hi = corridor_bounds(1.6, 2.3)
  for s in outs:
    if s.active:
      assert s.would_steer >= lo - 1e-6
  assert outs[-1].would_steer == pytest.approx(0.35, abs=0.03)
  assert outs[-1].margin >= CORRIDOR_MARGIN_M
  # The old behaviour for the record: it tracked one line only.
  assert old.side in (-1, 1)


def test_line_inside_margin_pushes_away_firmly():
  det = ConeCorridorDetector()
  outs = _run(det, _pts(1.1), 4.0)  # one left line 1.1 m from centre
  s = outs[-1]
  assert s.left_active and not s.right_active
  assert s.would_steer >= CORRIDOR_MARGIN_M - 1.1 - 1e-6
  assert s.would_steer <= WOULD_STEER_CAP_M


def test_corridor_target_hard_rule_property():
  rng = random.Random(3)
  for _ in range(5000):
    lc = rng.choice([None, rng.uniform(0.6, 4.0)])
    rc = rng.choice([None, rng.uniform(0.6, 4.0)])
    s = corridor_target(lc, rc, v_ego=13.0, room_right=rng.uniform(0, 1), room_left=rng.uniform(0, 1))
    lo, hi = corridor_bounds(lc, rc)
    assert abs(s) <= WOULD_STEER_CAP_M + 1e-9
    if lo <= hi and lo <= WOULD_STEER_CAP_M and hi >= -WOULD_STEER_CAP_M:
      assert lo - 1e-9 <= s <= hi + 1e-9


def test_release_eases_back_without_a_snap():
  det = ConeCorridorDetector()
  _run(det, _pts(1.4), 5.0)
  ws0 = det._ws
  assert ws0 > 0.3
  after = _run(det, [], DROP_S + CORRIDOR_RELEASE_S + 1.0)
  ws = [s.would_steer for s in after]
  assert all(b <= a + 1e-9 for a, b in zip(ws, ws[1:], strict=False))
  steps = [a - b for a, b in zip(ws, ws[1:], strict=False)]
  assert max(steps) <= ws0 * 1.5 * DT / CORRIDOR_RELEASE_S + 1e-6  # smoothstep peak slope 1.5
  assert any(s.releasing for s in after)
  assert ws[-1] == 0.0 and not after[-1].releasing
  # Held while the detector's ~2 s drop timer runs, then eased.
  assert ws[int(1.5 / DT)] == pytest.approx(ws0, abs=1e-6)


def test_hold_applies_the_releasing_shift_then_lets_go():
  hold = ConeLineHold()
  kw = dict(enabled=True, engaged=True, torque_nm=0.0, measured_k=0.0, model_k=0.0,
            path_x=PX, path_y=PY, v_ego=V, yielded=False, lat_active=True, dt=0.01)
  live = SimpleNamespace(active=True, side=1, releasing=False)
  assert hold.update(cone=live, cone_shift=0.5, **kw).curvature > 0.0
  easing = SimpleNamespace(active=False, side=1, releasing=True)
  out = hold.update(cone=easing, cone_shift=0.2, **kw)
  assert out.active and out.source == "shift" and out.curvature > 0.0
  done = SimpleNamespace(active=False, side=1, releasing=False)
  out = hold.update(cone=done, cone_shift=0.0, **kw)
  assert not out.active and out.curvature == 0.0
