"""Sight-distance cap (log only) replayed on Justin's rlogs at 10 Hz.

Each fixture holds modelV2 position (x, y, yStd), path curvature, confidence,
radarState leadOne (status, dRel, vLead, yRel) and vEgo. The cap is driven
exactly as the planner drives it. For the corner, a simulated car also obeys
the would-be cap (the real car does not: it is log only) to see what speed
the cap alone would reach at the apex.
"""
from __future__ import annotations

import base64
import gzip
import json
import types
from pathlib import Path

import pytest

from openpilot.selfdrive.controls.lib.model_sight import sight_distance_m
from openpilot.selfdrive.controls.lib.radar_path_gate import path_lateral_m
from openpilot.selfdrive.controls.lib.sight_cap import DECEL_MS2, JERK_MS3, SightCap, classify_lead

HERE = Path(__file__).parent / "sight_cap_replay"
MPH = 2.23694


def _load(name):
  return json.loads(gzip.decompress(base64.b64decode((HERE / f"{name}.json.gz.b64").read_text().replace("\n", ""))))


def _lead(f):
  if not f["lead"] or not f["lead"][0]:
    return classify_lead(False, None, None, None)
  _st, d, v, y = f["lead"]
  lat = path_lateral_m(types.SimpleNamespace(dRel=d, yRel=y), f["x"], f["y"])
  return classify_lead(True, lat, v, d)


def _replay(fx, *, follow_cap=False, use_lead=True):
  sc = SightCap()
  rows = []
  prev = None
  v_sim = None
  for f in fx["frames"]:
    dt = 0.1 if prev is None else max(0.02, min(0.3, f["t"] - prev))
    prev = f["t"]
    v_real = float(f["v"])
    if v_sim is None:
      v_sim = v_real
    v = v_sim if follow_cap else v_real
    lead = _lead(f) if use_lead else classify_lead(False, None, None, None)
    sc.update(t=f["t"], dt=dt, v_ego=v, sight_m=sight_distance_m(f["x"], f["ys"]), ks=[k * 1e-4 for k in f["k"]],
              conf_red=f["cn"] == "red", lead=lead, rb_live=False, rb_ahead=False)
    rows.append(dict(t=f["t"], v=v, fire=sc.would_fire, edge=sc.fired_edge, a=sc.a_cap, why=sc.reason, vc=sc.v_cap, lead=lead))
    if follow_cap:
      # The car keeps its real speed profile unless the cap asks for less.
      a = sc.a_cap if sc.would_fire else 0.0
      v_sim = min(v_real, v_sim + a * dt) if sc.would_fire else min(v_real, v_sim + 0.8 * dt)
    rows[-1]["sup"] = sc.suppressed_edge
  return rows


def test_corner_fires_before_the_bend_and_the_cap_reaches_about_25_mph():
  rows = _replay(_load("corner_1010_174045"), follow_cap=True)
  edges = [r["t"] for r in rows if r["edge"]]
  assert edges and -8.0 <= edges[0] <= -5.0, edges     # ~6.6 s before the apex (curve follow: ~1.9 s)
  apex = min(r["v"] for r in rows if -1.0 <= r["t"] <= 1.0) * MPH
  assert 20.0 <= apex <= 30.0, apex                     # Justin: 23 mph. Curve follow alone: ~46.
  assert min(r["a"] for r in rows) >= -DECEL_MS2 - 1e-6
  worst = max((a["a"] - b["a"]) / max(1e-3, b["t"] - a["t"]) for a, b in zip(rows, rows[1:], strict=False)
              if b["a"] < a["a"] and b["a"] < 0.0)
  assert worst <= JERK_MS3 + 1e-6                      # jerk limited: not a wall


def _variant(edit):
  fx = _load("moving_lead_1007_204158")
  for f in fx["frames"]:
    if f["lead"] and f["lead"][0]:
      edit(f["lead"])
  return fx


def _set(i, val):
  def edit(lead):
    lead[i] = val if not callable(val) else val(lead[i])
  return edit


@pytest.mark.parametrize("why, edit", [
  ("lead", None),                              # as driven: moving car ahead in our lane
  ("stopped_lead", _set(2, 0.0)),              # same car, stopped in our lane
  ("adjacent", _set(3, lambda y: y + 3.6)),    # same car, one lane over, moving
  ("parked", None),                            # one lane over and stopped
])
def test_any_vehicle_explaining_the_drop_suppresses_it(why, edit):
  """Vehicles never count as sight loss (Justin, Oct 11): suppress and log why."""
  assert any(r["edge"] for r in _replay(_load("moving_lead_1007_204158"), use_lead=False)), "road-only would fire"
  if why == "parked":
    fx = _variant(lambda lead: (_set(2, 0.0)(lead), _set(3, lambda y: y + 3.6)(lead)))
  else:
    fx = _variant(edit) if edit else _load("moving_lead_1007_204158")
  rows = _replay(fx)
  assert not any(r["fire"] for r in rows)
  assert any(r["why"] == why for r in rows if -1.0 <= r["t"] <= 1.0), why


def test_lead_kinds():
  assert classify_lead(True, 0.2, 14.0, 40.0).kind == "lead"
  assert classify_lead(True, 0.2, 0.0, 40.0).kind == "stopped_lead"
  assert classify_lead(True, 3.5, 14.0, 40.0).kind == "adjacent"
  assert classify_lead(True, -3.5, 0.5, 40.0).kind == "parked"
  assert classify_lead(True, 0.2, 14.0, 200.0).kind == ""
  assert classify_lead(False, None, None, None).kind == ""


def test_straight_never_fires():
  rows = _replay(_load("straight_1010_1758"))
  assert len(rows) > 400
  assert not any(r["fire"] for r in rows)


@pytest.mark.parametrize("rb_live", [True])
def test_roundabout_funnel_live_steps_aside(rb_live):
  fx = _load("corner_1010_174045")
  sc = SightCap()
  fired = False
  prev = None
  for f in fx["frames"]:
    dt = 0.1 if prev is None else f["t"] - prev
    prev = f["t"]
    sc.update(t=f["t"], dt=dt, v_ego=float(f["v"]), sight_m=sight_distance_m(f["x"], f["ys"]),
              ks=[k * 1e-4 for k in f["k"]], conf_red=f["cn"] == "red", lead=classify_lead(False, None, None, None),
              rb_live=rb_live, rb_ahead=True)
    fired |= sc.would_fire
  assert not fired and sc.reason == "rb_funnel"
