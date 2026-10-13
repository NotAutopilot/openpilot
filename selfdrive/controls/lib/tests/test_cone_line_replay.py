"""Replays of real Oct 10, 2026 drives (dongle 1c95345a3286a5db) through ConeLineDetector.

Each fixture is the radar-rate input radard hands the detector, cut from the rlog
(gzip JSON, base64 so it diffs and pushes as text):
liveTracks points (pre-filtered to near-stationary returns), vEgo, the model path,
road edges and yaw rate. `t` is seconds from the start of the cut.

  17c_cone_zone   0000017c 17:30:00–17:33:30 CT. Drums in the closed left lane,
                  then channelizers on the left line, 46–70 mph. 112907e9 saw nothing.
  17b_cone_pass   0000017b 16:47:55–16:48:45 CT. Drums 1.5–2.5 m left at 54 mph.
                  Must stay a left line with wouldSteer pointing right.
  17b_town_parked 0000017b 16:41:20–16:43:00 CT. Town, parked cars, an overpass
                  pier and sign poles; includes the 16:42:24 parked rejection.
  17b_parked_1647 / 17b_parked_1650   the 16:47:01 and 16:50:21 parked rejections.
"""
import base64
import gzip
import json
from pathlib import Path

import pytest

from openpilot.selfdrive.controls.lib.cone_line import ConeLineDetector

FIX = Path(__file__).resolve().parent / "cone_replay"


def _replay(name):
  frames = json.loads(gzip.decompress(base64.b64decode((FIX / f"{name}.json.gz.b64").read_bytes())))
  det = ConeLineDetector()
  out = []
  prev = None
  for f in frames:
    dt = 0.05 if prev is None else f["t"] - prev
    prev = f["t"]
    pts = [{"dRel": p[0], "yRel": p[1], "vRel": p[2], "measured": bool(p[3])} for p in f["pts"]]
    edges = [(e[0], e[1]) for e in f["edges"]]
    yaw = f["yaw"] if abs(f["yaw"]) < 1.5 else 0.0
    out.append((f["t"], det.update(pts, f["v"], f["px"], f["py"], edges, dt, yaw)))
  return out


def _active_s(out):
  total = 0.0
  for (t0, s0), (t1, _) in zip(out, out[1:], strict=False):
    if s0.active:
      total += t1 - t0
  return total


@pytest.fixture(autouse=True)
def _no_budget(monkeypatch):
  # Replays are about the geometry, not the 2 ms on-device budget of a slow CI box.
  import openpilot.selfdrive.controls.lib.cone_line as cl
  monkeypatch.setattr(cl, "CONE_BUDGET_S", 10.0)


def test_oct10_closed_left_lane_zone_is_detected_on_the_left():
  out = _replay("oct10_17c_cone_zone")
  active = [s for _, s in out if s.active]
  assert _active_s(out) >= 10.0
  assert all(s.side == 1 for s in active)
  # Left line: path-relative lateral is negative (+right frame), and nothing proposes
  # moving toward it.
  assert all(s.lat_near < 0.0 for s in active)
  assert all(s.would_steer >= 0.0 for s in active)


def test_oct10_1648_left_drums_stay_correct():
  out = _replay("oct10_17b_cone_pass")
  active = [(t, s) for t, s in out if s.active]
  assert active, "the 16:48 pass must still confirm"
  # 112907e9 confirmed 16.2 s into this cut (16:48:11.1); never later than that.
  assert active[0][0] <= 16.3
  assert _active_s(out) >= 18.0
  assert all(s.side == 1 for _, s in active)
  steer = [s.would_steer for _, s in active if s.would_steer != 0.0]
  assert steer and all(0.0 < w <= 1.0 + 1e-6 for w in steer)
  # Ramped slowly (0.2 m/s): no single-frame jump.
  prev = 0.0
  for _, s in out:
    assert abs(s.would_steer - prev) <= 0.2 * 0.5 + 1e-6
    prev = s.would_steer


@pytest.mark.parametrize("name", ["oct10_17b_town_parked", "oct10_17b_parked_1647", "oct10_17b_parked_1650"])
def test_oct10_town_and_parked_cars_never_become_a_line(name):
  out = _replay(name)
  assert not any(s.active for _, s in out)
  assert all(s.would_steer == 0.0 for _, s in out)
  assert any(s.parked for _, s in out), "the logged parked-car rejection must still fire"
