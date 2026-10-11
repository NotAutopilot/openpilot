"""Camera cone confirm (log only) on real Oct 10, 2026 ROAD-camera frames.

Fixtures are fcamera frames (OX03C10, 1928x1208) from dongle 1c95345a3286a5db,
rows 760-1208 only (the boxes never reach above row ~800), JPEG, base64 so
they push as text. They are converted to BT.601 limited-range YUV 4:2:0, the
same planes camerad hands VisionIPC.

  r17c_173045  0000017c 17:30:45 CT  drums in the closed left lane
  r17c_173203  0000017c 17:32:03 CT  orange/white posts on the left line
  r17c_173231  0000017c 17:32:31 CT  drums on the left
  r17b_164810  0000017b 16:48:10 CT  real drums, low sun ahead: washed out,
                                    the camera abstains (radar keeps the line)
  r17b_164217  0000017b 16:42:17 CT  town: orange/white striped building, trucks
  r17b_164220  0000017b 16:42:20 CT  town: same building, box truck
  r17b_164850  0000017b 16:48:50 CT  grass and a curve, no cones
  r17c_173005  0000017c 17:30:05 CT  grass and an intersection, no cones
"""
import base64
import io
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.pathobstacled import cone_cam as cc
from openpilot.selfdrive.pathobstacled.pathobstacled import _IdleSched as _REAL_IDLE

FIX = Path(__file__).resolve().parent / "cone_frames"
TOP = 760
G17B = cc.CamGeom(roll=0.00067, pitch=-0.0791, yaw=-0.0191, cam_height=1.26)
G17C = cc.CamGeom(roll=0.00068, pitch=-0.0791, yaw=-0.0192, cam_height=1.254)
_CACHE = {}


def _frame(name):
  if name in _CACHE:
    return _CACHE[name]
  from PIL import Image
  raw = base64.b64decode((FIX / f"{name}.jpg.b64").read_bytes())
  rgb = np.asarray(Image.open(io.BytesIO(raw)).convert("RGB")).astype(np.float32)
  full = np.zeros((1208, 1928, 3), dtype=np.float32)
  full[TOP:TOP + rgb.shape[0]] = rgb
  r, g, b = full[..., 0], full[..., 1], full[..., 2]
  y = 16 + 0.257 * r + 0.504 * g + 0.098 * b
  u = 128 - 0.148 * r - 0.291 * g + 0.439 * b
  v = 128 + 0.439 * r - 0.368 * g - 0.071 * b
  u = u.reshape(604, 2, 964, 2).mean((1, 3))
  v = v.reshape(604, 2, 964, 2).mean((1, 3))

  def q(a):
    return np.clip(np.rint(a), 0, 255).astype(np.uint8)
  _CACHE[name] = (q(y), q(u), q(v))
  return _CACHE[name]


def _hits(name, geom, lats):
  y, u, v = _frame(name)
  scores, over = cc.score_line(y, u, v, geom, lats)
  assert not over
  return [s for s in scores if s.hit], scores


@pytest.mark.parametrize("name,lats", [
  ("r17c_173045", (-3.70, -3.77, -3.88)),
  ("r17c_173203", (-1.70, -1.70, -1.70)),
  ("r17c_173231", (-3.46, -3.15, -2.86)),
])
def test_oct10_left_cones_confirm(name, lats):
  hits, scores = _hits(name, G17C, lats)
  assert len({(s.x0, s.x1) for s in hits}) >= cc.CONFIRM_HITS
  assert all(s.bands >= 1 for s in hits)
  state = cc.ConeCamState()
  assert state.update(1, scores, 10.0)
  # Held for 1 s, then it needs a fresh agreeing frame.
  assert state.update(1, [], 10.9)
  assert not state.update(1, [], 11.2)


def test_oct10_cones_do_not_hit_on_the_wrong_side():
  hits, _ = _hits("r17c_173045", G17C, (3.5, 3.5, 3.5))
  assert hits == []


def test_oct10_low_sun_pass_abstains():
  # Real drums, but backlit to V~142 / U~121 against road 131 / 124: no reliable
  # chroma. The camera must not guess; the radar line stands on its own.
  hits, scores = _hits("r17b_164810", G17B, (-1.65, -1.65, -1.64))
  assert hits == []
  assert not cc.ConeCamState().update(1, scores, 0.0)


@pytest.mark.parametrize("name", ["r17b_164217", "r17b_164220"])
def test_oct10_orange_building_and_trucks_never_confirm(name):
  for lat in (1.5, 2.0, 2.5, 3.0, 3.5, 4.0, -1.5, -2.0, -2.5, -3.0, -3.5, -4.0):
    hits, scores = _hits(name, G17B, (lat, lat, lat))
    assert len(hits) < cc.CONFIRM_HITS, (name, lat, [(s.x0, s.orange_px, s.bands) for s in hits])
    assert not cc.ConeCamState().update(1 if lat < 0 else -1, scores, 0.0)


def test_oct10_town_sequence_never_confirms():
  state = cc.ConeCamState()
  t = 0.0
  for name in ("r17b_164217", "r17b_164220", "r17b_164217", "r17b_164220"):
    for lat in (3.0, 4.0):
      _, scores = _hits(name, G17B, (lat, lat, lat))
      assert not state.update(-1, scores, t)
      t += 0.2


@pytest.mark.parametrize("name,geom", [("r17b_164850", G17B), ("r17c_173005", G17C)])
def test_oct10_grass_has_no_hits(name, geom):
  for lat in (-4.0, -3.0, -2.0, 2.0, 3.0, 4.0):
    hits, _ = _hits(name, geom, (lat, lat, lat))
    assert hits == [], (name, lat)


def test_cpu_cost_per_frame():
  frames = [("r17c_173045", G17C, (-3.70, -3.77, -3.88)), ("r17c_173231", G17C, (-3.46, -3.15, -2.86)),
            ("r17b_164217", G17B, (3.0, 3.0, 3.0))]
  for name, _, _ in frames:
    _frame(name)
  # Best of 5 batches: a shared CI runner's load is noise, not our cost.
  n = 12
  batches = []
  for _ in range(5):
    t0 = time.perf_counter()
    for i in range(n):
      name, geom, lats = frames[i % len(frames)]
      y, u, v = _frame(name)
      cc.score_line(y, u, v, geom, lats)
    batches.append((time.perf_counter() - t0) * 1e3 / n)
  per_frame_ms = min(batches)
  # About 1-4 ms on a desktop core (5 Hz: 0.5-2 % of one core). The
  # device enforces the 8 ms budget with a deadline (next test).
  assert per_frame_ms < cc.BUDGET_S * 1e3, per_frame_ms


def test_budget_deadline_stops_and_rotates():
  y, u, v = _frame("r17c_173231")
  scores, over = cc.score_line(y, u, v, G17C, (-3.46, -3.15, -2.86), deadline=time.monotonic() - 1.0)
  assert over and scores == []
  scores, over = cc.score_line(y, u, v, G17C, (-3.46, -3.15, -2.86), start=3)
  assert not over and scores[0].x0 == cc.WINDOWS_M[3][0]


class _NoIdle:
  def __enter__(self):
    return self

  def __exit__(self, *exc):
    return False


@pytest.fixture(autouse=True)
def _no_idle_sched(monkeypatch):
  # SCHED_IDLE on a shared CI box starves the scorer and makes timing flaky.
  # test_cone_scoring_runs_under_sched_idle checks the real switch.
  import openpilot.selfdrive.pathobstacled.pathobstacled as po
  monkeypatch.setattr(po, "_IdleSched", _NoIdle)


def test_cone_scoring_runs_under_sched_idle(monkeypatch):
  import os
  calls = []
  monkeypatch.setattr(os, "sched_setscheduler", lambda pid, pol, param: calls.append(pol), raising=False)
  monkeypatch.setattr(os, "geteuid", lambda: 0)
  with _REAL_IDLE():
    pass
  assert calls == [os.SCHED_IDLE, os.SCHED_OTHER]
  # Unprivileged: SCHED_IDLE could not be left again, so it is not entered.
  calls.clear()
  monkeypatch.setattr(os, "geteuid", lambda: 1000)
  with _REAL_IDLE():
    pass
  assert calls == []


def _helper():
  from openpilot.selfdrive.pathobstacled.pathobstacled import Helper
  h = Helper()
  h.calib = (0.00068, -0.0791, -0.0192, 1.254, 0.0, 0.0, 0.0)
  h.cam = SimpleNamespace(sensor="ox03c10")
  return h


def _cone(side=1, count=4, lats=(-3.46, -3.15, -2.86), active=True):
  return SimpleNamespace(side=side, count=count, latNear=lats[0], latMid=lats[1], latFar=lats[2], active=active)


def test_on_cone_logs_at_most_2hz_and_confirms():
  from openpilot.selfdrive.pathobstacled.pathobstacled import on_cone
  h = _helper()
  frame = _frame("r17c_173231")
  events = []
  t = 100.0
  while t < 102.0:
    ev = on_cone(h, _cone(), t, SimpleNamespace(frameDropPerc=0.0), frame=frame)
    if ev is not None:
      events.append((t, ev))
    t += 0.05
  assert 3 <= len(events) <= 5
  gaps = [b[0] - a[0] for a, b in zip(events, events[1:], strict=False)]
  assert min(gaps) >= 0.5 - 1e-6
  assert all(ev["side"] == 1 and ev["radar_count"] == 4 for _, ev in events)
  assert any(ev["confirmed"] and ev["n_hits"] >= 2 for _, ev in events)
  assert all(ev["reason"] in ("", "budget") for _, ev in events)


def test_on_cone_ignores_empty_lines_and_backs_off_on_model_drops():
  from openpilot.selfdrive.pathobstacled.pathobstacled import on_cone
  h = _helper()
  frame = _frame("r17c_173231")
  assert on_cone(h, _cone(side=0, count=0), 1.0, None, frame=frame) is None
  # modeld dropping frames: no grab, and a growing hold-off.
  assert on_cone(h, _cone(), 2.0, SimpleNamespace(frameDropPerc=3.0), frame=frame) is None
  assert h.cone_hold_until == pytest.approx(2.5)
  assert on_cone(h, _cone(), 2.1, SimpleNamespace(frameDropPerc=3.0), frame=frame) is None
  assert h.cone_hold_until == pytest.approx(3.1)
  assert on_cone(h, _cone(), 2.6, SimpleNamespace(frameDropPerc=0.0), frame=frame) is None
  ev = on_cone(h, _cone(), 3.2, SimpleNamespace(frameDropPerc=0.0), frame=frame)
  assert ev is not None and ev["n_scored"] >= 1


def test_no_gpu_libraries_are_imported():
  code = """
import sys
import openpilot.selfdrive.pathobstacled.pathobstacled
import openpilot.selfdrive.pathobstacled.cone_cam
import openpilot.selfdrive.speedsignd.nv12
gpu = ('tinygrad', 'onnx', 'onnxruntime', 'pyopencl', 'torch')
bad = [m for m in sys.modules if m.split('.')[0] in gpu or 'opencl' in m.lower()
       or m.startswith('openpilot.selfdrive.modeld')]
print(','.join(bad))
sys.exit(1 if bad else 0)
"""
  out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=120)
  assert out.returncode == 0, out.stdout + out.stderr
  root = Path(__file__).resolve().parents[1]
  for src in ("cone_cam.py", "pathobstacled.py"):
    text = (root / src).read_text()
    for lib in ("tinygrad", "onnx", "OpenCL", "pyopencl", "torch", "GPU="):
      assert f"import {lib}" not in text and f"from {lib}" not in text, (src, lib)


def test_vectorized_patch_stats_match_the_list_path():
  import openpilot.selfdrive.controls.lib.path_obstacle as po
  rng = np.random.default_rng(7)
  for trial in range(40):
    h, w = int(rng.integers(2, 160)), int(rng.integers(2, 220))
    a = rng.integers(0, 256, (h, w)).astype(np.uint8)
    if trial % 3 == 0:
      a[int(rng.integers(0, h)):, :] = int(rng.integers(190, 256))
    road = rng.integers(0, 256, (int(rng.integers(1, 60)), int(rng.integers(1, 260)))).astype(np.uint8) if trial % 4 else None
    vp = rng.integers(100, 256, (max(1, h // 2), max(1, w // 2))).astype(np.uint8) if trial % 2 else None
    fast = po.metrics_from_rows(a, road)
    slow = po.metrics_from_rows(a.tolist(), None if road is None else road.tolist())
    for k in fast:
      assert np.allclose(fast[k], slow[k], atol=1e-6), (k, fast[k], slow[k])
    assert np.allclose(po.crop_light_stats(a, vp), po.crop_light_stats(a.tolist(), None if vp is None else vp.tolist()), atol=1e-9)


def test_selfdrived_still_ignores_path_obstacle_by_name():
  root = Path(__file__).resolve().parents[2]
  assert "'pathObstacleNAP'" in (root / "selfdrived/selfdrived.py").read_text()
