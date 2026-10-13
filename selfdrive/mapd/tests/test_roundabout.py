"""Roundabout first slice: funnel detect, speed ease, outer bias, FP guard."""
import json
import math
import os

import pytest
from openpilot.selfdrive.mapd.roundabout import RB_ONSET_MARGIN_M, RingExitGate, RB_A_MAX_DECEL_MS2, RB_EXIT_RESTORE_MS2

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.constants import LOOKAHEAD_NORMAL
from openpilot.selfdrive.mapd.osm_db import EARTH_R, OsmSpeedLimitDB, _unpack_coords
from openpilot.selfdrive.mapd.overpass import ways_from_overpass
from openpilot.selfdrive.mapd.roundabout import (
  RB_A_FLOOR_MS2,
  RB_DECEL_JERK_MS3,
  RB_ONSET_MAX_M,
  RB_ONSET_MIN_M,
  RB_PROFILE_BUFFER_M,
  RB_ENTRY_BIAS_M,
  RB_DECEL_ONSET_M,
  RB_FUNNEL_M,
  RB_FUNNEL_MIN_M,
  RB_OUTER_OFFSET_M,
  RB_OUTER_OFFSET_MAX_M,
  RB_OUTER_OFFSET_MIN_M,
  RB_V_DEFAULT_MS,
  RB_RING_SPEED_MPH,
  RB_V_MAX_MS,
  RB_V_MIN_MS,
  RoundaboutHint,
  apply_roundabout_plan,
  junction_is_roundabout,
  live_map_roundabout_hint,
  rb_decel_onset_m,
  roundabout_decel_ms2,
  roundabout_ease_v_ms,
  roundabout_outer_curvature_bias,
  roundabout_outer_path_offset_m,
  roundabout_suppressed_for_highway,
  roundabout_target_ms,
  way_is_closed_loop,
  way_is_roundabout,
  way_radius_m,
)

# Willmar-ish 30th St SW / 19th Ave SW ring: R ≈ 22.5 m, maxspeed 20.
RB_LAT = 45.1165
RB_LON = -95.0610
RB_R_M = 22.5


def _offset(lat, lon, bearing_deg, dist_m):
  theta = dist_m / EARTH_R
  brng = math.radians(bearing_deg)
  phi1 = math.radians(lat)
  lam1 = math.radians(lon)
  phi2 = math.asin(math.sin(phi1) * math.cos(theta) + math.cos(phi1) * math.sin(theta) * math.cos(brng))
  lam2 = lam1 + math.atan2(
    math.sin(brng) * math.sin(theta) * math.cos(phi1),
    math.cos(theta) - math.sin(phi1) * math.sin(phi2),
  )
  return math.degrees(phi2), (math.degrees(lam2) + 540.0) % 360.0 - 180.0


def _circle(lat0, lon0, r_m=RB_R_M, n=20):
  coords = [_offset(lat0, lon0, 360.0 * i / n, r_m) for i in range(n)]
  coords.append(coords[0])
  return coords


def _eastbound_way(lat, lon_west, lon_east, steps=4):
  return [(lat, lon_west + (lon_east - lon_west) * i / steps) for i in range(steps + 1)]


def test_closed_loop_radius_is_roundabout():
  ring = _circle(RB_LAT, RB_LON)
  assert way_is_closed_loop(ring)
  r = way_radius_m(ring)
  assert r is not None and abs(r - RB_R_M) < 3.0
  assert way_is_roundabout(ring, junction="")
  assert way_is_roundabout(ring, junction="roundabout")
  assert junction_is_roundabout("circular")


def test_tagged_arc_counts_as_roundabout():
  # Multi-way OSM rings are 90° arcs with junction=roundabout, not closed.
  arc = [_offset(RB_LAT, RB_LON, brg, RB_R_M) for brg in (0.0, 30.0, 60.0, 90.0)]
  assert not way_is_closed_loop(arc)
  assert way_is_roundabout(arc, junction="roundabout")
  assert not way_is_roundabout(arc, junction="")


def test_sharp_town_corner_is_not_roundabout():
  # 90° town corner — the false-positive we must not treat as an RB.
  corner = [
    (RB_LAT, RB_LON - 0.002),
    (RB_LAT, RB_LON),
    (RB_LAT + 0.002, RB_LON),
  ]
  assert not way_is_closed_loop(corner)
  assert not way_is_roundabout(corner, junction="")
  assert not way_is_roundabout(corner, highway="residential")


def test_signalized_cross_is_not_roundabout():
  ns = [(RB_LAT - 0.002, RB_LON), (RB_LAT + 0.002, RB_LON)]
  ew = [(RB_LAT, RB_LON - 0.002), (RB_LAT, RB_LON + 0.002)]
  assert not way_is_roundabout(ns, junction="")
  assert not way_is_roundabout(ew, junction="")


def test_service_bulb_without_tag_is_not_roundabout():
  bulb = _circle(RB_LAT, RB_LON, r_m=10.0)
  assert not way_is_roundabout(bulb, junction="", highway="service")
  assert way_is_roundabout(bulb, junction="roundabout", highway="service")


def test_untagged_residential_loop_is_not_roundabout():
  """Cul-de-sac / Maritime-style bulbs are not RBs unless junction-tagged."""
  bulb = _circle(RB_LAT, RB_LON, r_m=20.0)
  assert way_is_closed_loop(bulb)
  assert way_is_roundabout(bulb, junction="", highway="unclassified")
  assert not way_is_roundabout(bulb, junction="", highway="residential")
  assert not way_is_roundabout(bulb, junction="", highway="living_street")
  assert way_is_roundabout(bulb, junction="roundabout", highway="residential")
  assert way_is_roundabout(bulb, junction="circular", highway="living_street")


def test_motorway_and_trunk_suppress_roundabout_hints():
  for hw in ("motorway", "motorway_link", "trunk", "trunk_link", " Motorway "):
    assert roundabout_suppressed_for_highway(hw)
  for hw in ("", "residential", "primary", "unclassified", "secondary", "tertiary"):
    assert not roundabout_suppressed_for_highway(hw)


def test_target_clamps_osm_to_15_20():
  assert abs(roundabout_target_ms(0.0) - RB_V_DEFAULT_MS) < 1e-6
  assert abs(roundabout_target_ms(20.0 * CV.MPH_TO_MS) - 20.0 * CV.MPH_TO_MS) < 1e-6
  assert abs(roundabout_target_ms(18.5 * CV.MPH_TO_MS) - 18.5 * CV.MPH_TO_MS) < 1e-6
  assert roundabout_target_ms(15.0 * CV.MPH_TO_MS) == RB_V_MIN_MS == pytest.approx(RB_RING_SPEED_MPH * CV.MPH_TO_MS)   # Sep 30: 18 mph floor
  assert RB_RING_SPEED_MPH == 18.0
  assert roundabout_target_ms(25.0 * CV.MPH_TO_MS) == RB_V_MAX_MS
  assert roundabout_target_ms(10.0 * CV.MPH_TO_MS) == RB_V_MIN_MS


def test_funnel_eases_speed():
  hint = RoundaboutHint(
    approaching=True, distance_m=90.0, speed_limit_ms=20.0 * CV.MPH_TO_MS,
  )
  v49 = 49.0 * CV.MPH_TO_MS
  eased = roundabout_ease_v_ms(hint, v49, v49, LOOKAHEAD_NORMAL)
  assert eased is not None
  assert eased < v49 - 2.0
  assert eased >= RB_V_MIN_MS - 1e-6
  on_ring = RoundaboutHint(on_roundabout=True, speed_limit_ms=20.0 * CV.MPH_TO_MS)
  assert abs(roundabout_ease_v_ms(on_ring, v49, v49) - 20.0 * CV.MPH_TO_MS) < 1e-6
  # Low-speed decel onset stays 140 m (Oct 1): 35 mph can still hit 18 by the ring at ~0.65 m/s².
  # Oct 11: the search reach (RB_FUNNEL_M) is the 450 m max of the speed-based onset.
  assert 80.0 <= RB_FUNNEL_MIN_M <= 100.0
  assert 130.0 <= RB_DECEL_ONSET_M <= 140.0 and RB_FUNNEL_M == 450.0
  assert (35.0 * CV.MPH_TO_MS) ** 2 / 2.0 - (18.0 * CV.MPH_TO_MS) ** 2 / 2.0 <= 0.7 * RB_DECEL_ONSET_M


def test_no_hint_does_not_ease():
  assert roundabout_ease_v_ms(None, 22.0, 22.0) is None
  assert roundabout_ease_v_ms(RoundaboutHint(), 22.0, 22.0) is None


def test_plan_funnel_cuts_plus_a():
  """Willmar: aTarget ~+0.4 at 45–49 mph must become kinematic −a, not −0.55."""
  v49 = 49.0 * CV.MPH_TO_MS
  v50 = 50.0 * CV.MPH_TO_MS
  hint = RoundaboutHint(approaching=True, distance_m=90.0, speed_limit_ms=20.0 * CV.MPH_TO_MS)
  _vc, _vh, a, rb = apply_roundabout_plan(v49, v50, v50, 0.4, hint)
  assert rb is not None and a <= RB_A_FLOOR_MS2
  _vc, _vh, a0, rb0 = apply_roundabout_plan(v49, v50, v50, 0.4, None)
  assert rb0 is None and a0 == 0.4


def test_outer_bias_sign_and_magnitude():
  rht = roundabout_outer_path_offset_m(on_roundabout=True, is_rhd=False)
  lhd = roundabout_outer_path_offset_m(on_roundabout=True, is_rhd=True)
  assert rht < 0.0  # right / outer in US
  assert lhd > 0.0
  assert RB_OUTER_OFFSET_MIN_M <= abs(rht) <= RB_OUTER_OFFSET_MAX_M
  assert abs(abs(rht) - RB_OUTER_OFFSET_M) < 1e-6
  assert abs(rht) >= 2.8  # EP1 cut was 3.3 m inside; 0.45 m was not enough
  assert roundabout_outer_path_offset_m(on_roundabout=False) == 0.0
  # Far approach: do not drift to the shoulder 180 m out.
  assert roundabout_outer_path_offset_m(
    on_roundabout=False, approaching=True, distance_m=180.0,
  ) == 0.0
  entry = roundabout_outer_path_offset_m(
    on_roundabout=False, approaching=True, distance_m=RB_ENTRY_BIAS_M - 1.0,
  )
  assert abs(entry - rht) < 1e-6
  kappa = roundabout_outer_curvature_bias(rht)
  assert kappa < 0.0  # less left / larger R on a CCW US ring
  assert abs(kappa) >= 0.015  # counters ~3 m inside at 18 m preview
  assert abs(kappa) < 0.05


def test_kinematic_a_from_40_45_in_funnel():
  """40–45 mph in the funnel must plan a ≤ −1.0 until near ring speed."""
  target = 20.0 * CV.MPH_TO_MS
  for v_mph, dist in ((40.0, 90.0), (45.0, 90.0), (45.0, 160.0), (37.0, 90.0)):
    hint = RoundaboutHint(approaching=True, distance_m=dist, speed_limit_ms=target)
    v = v_mph * CV.MPH_TO_MS
    a = roundabout_decel_ms2(hint, v)
    assert a is not None and a <= 0.0
    if v_mph >= 40.0:
      assert a <= RB_A_FLOOR_MS2, (v_mph, dist, a)
  # Already near ring speed: clamp +a only, do not keep −1.0.
  slow = RoundaboutHint(approaching=True, distance_m=40.0, speed_limit_ms=target)
  assert abs(roundabout_decel_ms2(slow, 21.0 * CV.MPH_TO_MS)) < 1e-6


def test_no_plus_a_while_approaching():
  """EP0 hole: after lead ease, aTarget +0.399 before/inside detect."""
  hint = RoundaboutHint(
    approaching=True, distance_m=95.0, speed_limit_ms=20.0 * CV.MPH_TO_MS,
  )
  v37 = 37.0 * CV.MPH_TO_MS
  v50 = 50.0 * CV.MPH_TO_MS
  _vc, _vh, a, rb = apply_roundabout_plan(v37, v50, v50, 0.399, hint)
  assert rb is not None and a <= 0.0
  # At ring speed, still no +a rebound.
  _vc, _vh, a0, _rb = apply_roundabout_plan(20.0 * CV.MPH_TO_MS, v50, v50, 0.399, hint)
  assert a0 <= 0.0
  on = RoundaboutHint(on_roundabout=True, speed_limit_ms=20.0 * CV.MPH_TO_MS)
  _vc, _vh, a_on, _rb = apply_roundabout_plan(18.0 * CV.MPH_TO_MS, v50, v50, 0.4, on)
  assert a_on <= 0.0


def test_live_map_hint_reads_capnp_fields():
  class _Md:
    onRoundabout = False
    approachingRoundabout = True
    roundaboutDistance = 90.0
    roundaboutSpeedLimit = 20.0 * CV.MPH_TO_MS
    roundaboutWayId = 99

  hint = live_map_roundabout_hint(_Md())
  assert hint is not None and hint.approaching and abs(hint.distance_m - 90.0) < 1e-6
  assert live_map_roundabout_hint(None) is None

  class _Empty:
    onRoundabout = False
    approachingRoundabout = False

  assert live_map_roundabout_hint(_Empty()) is None


def _db_with_ring_and_approach(tmp_path, *, tagged=False, corner=False, ring_highway="unclassified"):
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  west_lat, west_lon = _offset(RB_LAT, RB_LON, 270.0, RB_R_M)
  approach_lat, approach_lon = _offset(west_lat, west_lon, 270.0, 160.0)
  OsmSpeedLimitDB.insert_way(
    con, 1, "30th St SW", "residential", 30.0 * CV.MPH_TO_MS,
    _eastbound_way(west_lat, approach_lon, west_lon),
  )
  if corner:
    OsmSpeedLimitDB.insert_way(
      con, 2, "19th Ave SW", "residential", 30.0 * CV.MPH_TO_MS,
      [
        (west_lat, west_lon - 0.0004),
        (RB_LAT, RB_LON),
        (RB_LAT + 0.002, RB_LON),
      ],
    )
  else:
    OsmSpeedLimitDB.insert_way(
      con, 2, "", ring_highway, 20.0 * CV.MPH_TO_MS,
      _circle(RB_LAT, RB_LON),
      junction="roundabout" if tagged else "",
    )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  return db, west_lat, west_lon


def test_funnel_detects_untagged_closed_ring(tmp_path):
  db, west_lat, west_lon = _db_with_ring_and_approach(tmp_path, tagged=False)
  qlat, qlon = _offset(west_lat, west_lon, 270.0, 90.0)
  hint = db.find_roundabout(qlat, qlon, bearing_deg=90.0)
  assert hint.approaching or hint.on_roundabout
  assert 0.0 < hint.distance_m <= RB_FUNNEL_M or hint.on_roundabout
  assert abs(hint.speed_limit_ms - 20.0 * CV.MPH_TO_MS) < 0.3
  # On the ring.
  on = db.find_roundabout(RB_LAT, RB_LON + 0.0, bearing_deg=0.0)
  # Center is ~22.5 m from the way — still on.
  assert on.on_roundabout
  db.close()


def test_tagged_ring_detects_in_funnel(tmp_path):
  db, west_lat, west_lon = _db_with_ring_and_approach(tmp_path, tagged=True)
  qlat, qlon = _offset(west_lat, west_lon, 270.0, 90.0)
  hint = db.find_roundabout(qlat, qlon, bearing_deg=90.0)
  assert hint.approaching
  assert hint.distance_m <= RB_FUNNEL_M
  db.close()


def test_funnel_detects_farther_out(tmp_path):
  """Decel onset ~RB_DECEL_ONSET_M: live at 135 m out, not yet at 180 m (Oct 1: was 200 m)."""
  db, west_lat, west_lon = _db_with_ring_and_approach(tmp_path, tagged=True)
  qlat, qlon = _offset(west_lat, west_lon, 270.0, 135.0)
  hint = db.find_roundabout(qlat, qlon, bearing_deg=90.0)
  assert hint.approaching
  assert 100.0 <= hint.distance_m <= RB_FUNNEL_M
  qlat, qlon = _offset(west_lat, west_lon, 270.0, 180.0)
  # Once hinted, the ring stays latched (no mid-approach drop / early exit restore).
  assert db.find_roundabout(qlat, qlon, bearing_deg=90.0).approaching
  db._rb_latch_way = 0
  assert not db.find_roundabout(qlat, qlon, bearing_deg=90.0).approaching
  db.close()


def test_sharp_corner_does_not_trigger_funnel(tmp_path):
  db, west_lat, west_lon = _db_with_ring_and_approach(tmp_path, corner=True)
  qlat, qlon = _offset(west_lat, west_lon, 270.0, 90.0)
  hint = db.find_roundabout(qlat, qlon, bearing_deg=90.0)
  assert not hint.approaching
  assert not hint.on_roundabout
  db.close()


def test_tagged_residential_ring_still_funnels(tmp_path):
  """junction=roundabout on a residential way is still a town roundabout."""
  db, west_lat, west_lon = _db_with_ring_and_approach(
    tmp_path, tagged=True, ring_highway="residential",
  )
  qlat, qlon = _offset(west_lat, west_lon, 270.0, 90.0)
  hint = db.find_roundabout(qlat, qlon, bearing_deg=90.0)
  assert hint.approaching
  assert hint.distance_m <= RB_FUNNEL_M
  db.close()


def test_untagged_residential_ring_does_not_funnel(tmp_path):
  db, west_lat, west_lon = _db_with_ring_and_approach(
    tmp_path, tagged=False, ring_highway="residential",
  )
  qlat, qlon = _offset(west_lat, west_lon, 270.0, 90.0)
  hint = db.find_roundabout(qlat, qlon, bearing_deg=90.0)
  assert not hint.approaching
  assert not hint.on_roundabout
  db.close()


# I-74 eastbound, Indianapolis. Residential loops beside the motorway
# (Maritime Dr 17493053, Seaway Dr 17497242) published approachingRoundabout
# and latched MAX 65→20.
I74_LAT = 39.81706
I74_LON = -86.30076


def _east_line(lat, lon, start_m, end_m, step_m):
  origin = _offset(lat, lon, 270.0, abs(start_m)) if start_m < 0 else _offset(lat, lon, 90.0, start_m)
  n = max(1, int(round((end_m - start_m) / step_m)))
  return [_offset(origin[0], origin[1], 90.0, step_m * i) for i in range(n + 1)]


def test_residential_loops_beside_motorway_do_not_approach(tmp_path):
  """Maritime / Seaway shape: closed residential ways within the 200 m funnel."""
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  OsmSpeedLimitDB.insert_way(
    con, 99449506, "I 74", "motorway", 65.0 * CV.MPH_TO_MS,
    _east_line(I74_LAT, I74_LON, -400.0, 400.0, 40.0),
  )
  maritime_c = _offset(*_offset(I74_LAT, I74_LON, 90.0, 160.0), 0.0, 30.0)
  seaway_c = _offset(*_offset(I74_LAT, I74_LON, 90.0, 175.0), 180.0, 35.0)
  maritime = _circle(*maritime_c, r_m=20.0)
  seaway = _circle(*seaway_c, r_m=18.0)
  # Same geometry would be an untagged RB on a town class; residential is not.
  assert way_is_roundabout(maritime, junction="", highway="unclassified")
  assert not way_is_roundabout(maritime, junction="", highway="residential")
  assert not way_is_roundabout(seaway, junction="", highway="residential")
  OsmSpeedLimitDB.insert_way(
    con, 17493053, "Maritime Drive", "residential", 25.0 * CV.MPH_TO_MS, maritime,
  )
  OsmSpeedLimitDB.insert_way(
    con, 17497242, "Seaway Drive", "residential", 25.0 * CV.MPH_TO_MS, seaway,
  )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  hint = db.find_roundabout(I74_LAT, I74_LON, bearing_deg=90.0)
  assert not hint.approaching
  assert not hint.on_roundabout
  assert hint.way_id == 0
  # Stored bulbs are still closed loops the untagged fallback would accept
  # on a town class. Residential / living_street stay skipped without a tag,
  # and the motorway match suppresses the hint on its own.
  for way_id in (17493053, 17497242):
    row = db._con.execute("SELECT coords, highway FROM ways WHERE way_id=?", (way_id,)).fetchone()
    stored = _unpack_coords(row["coords"])
    assert way_is_roundabout(stored, junction="", highway="unclassified")
    assert not way_is_roundabout(stored, junction="", highway=row["highway"])
  off_gate = db.find_roundabout(I74_LAT, I74_LON, bearing_deg=90.0, current_highway="residential")
  assert not off_gate.approaching
  assert not off_gate.on_roundabout
  db.close()


def test_motorway_suppresses_nearby_closed_ring(tmp_path):
  """An unclassified closed ring beside a motorway match must not approach.

  The same ring still funnels when the ego match is a town street.
  """
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  OsmSpeedLimitDB.insert_way(
    con, 99449506, "I 74", "motorway", 65.0 * CV.MPH_TO_MS,
    _east_line(I74_LAT, I74_LON, -400.0, 400.0, 40.0),
  )
  center = _offset(*_offset(I74_LAT, I74_LON, 90.0, 150.0), 0.0, 40.0)
  ring = _circle(*center, r_m=RB_R_M)
  OsmSpeedLimitDB.insert_way(
    con, 2, "", "unclassified", 20.0 * CV.MPH_TO_MS, ring,
  )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  on_motorway = db.find_roundabout(I74_LAT, I74_LON, bearing_deg=90.0)
  assert not on_motorway.approaching
  assert not on_motorway.on_roundabout
  for hw in ("motorway", "motorway_link", "trunk", "trunk_link"):
    gated = db.find_roundabout(I74_LAT, I74_LON, bearing_deg=90.0, current_highway=hw)
    assert not gated.approaching and not gated.on_roundabout
  town = db.find_roundabout(I74_LAT, I74_LON, bearing_deg=90.0, current_highway="residential")
  assert town.approaching
  assert 0.0 < town.distance_m <= RB_FUNNEL_M
  assert town.way_id == 2
  db.close()


def test_overpass_keeps_junction_tag():
  payload = {
    "elements": [
      {
        "type": "way",
        "id": 77,
        "tags": {"highway": "unclassified", "maxspeed": "20 mph", "junction": "roundabout"},
        "geometry": [{"lat": p[0], "lon": p[1]} for p in _circle(RB_LAT, RB_LON)],
      }
    ]
  }
  ways = ways_from_overpass(payload)
  assert len(ways) == 1
  assert ways[0]["junction"] == "roundabout"


# --- Oct 11: speed-based onset, smooth MAX profile (Watertown SD route 00000185) ---------------------------------
MPH = CV.MPH_TO_MS
DATA = os.path.join(os.path.dirname(__file__), "data", "rb_watertown_oct11.json")


def test_onset_formula_and_clamp():
  assert rb_decel_onset_m(None) == RB_ONSET_MIN_M
  assert rb_decel_onset_m(25 * MPH) == RB_ONSET_MIN_M
  d55 = rb_decel_onset_m(55 * MPH)
  # Oct 13: final slowdown starts ~40 m earlier (margin 30 -> 70 m), clamped at the max.
  assert d55 == pytest.approx(min(((55 * MPH) ** 2 - RB_V_MIN_MS ** 2) / 1.4 + RB_ONSET_MARGIN_M, RB_ONSET_MAX_M))
  assert 440.0 <= d55 <= 460.0
  assert rb_decel_onset_m(80 * MPH) == RB_ONSET_MAX_M


def _simulate(v0_mph, start_m, ring_limit_ms=0.0, dt=0.05, a_now=0.0):
  """Planner-ish sim: v follows min(profile MAX, kinematic −a) with the planner jerk limit."""
  v = v0_mph * MPH
  d = start_m
  a_jl = 0.0
  peak = 0.0
  first_slow_d = None
  while d > 0.0:
    onset = rb_decel_onset_m(v)
    hint = None
    if d <= max(onset, start_m if first_slow_d is not None else 0.0):
      hint = RoundaboutHint(approaching=True, distance_m=d, speed_limit_ms=ring_limit_ms)
    a_cmd = a_now
    if hint is not None:
      v_max = roundabout_ease_v_ms(hint, v, v)
      a_rb = roundabout_decel_ms2(hint, v)
      a_jl = max(a_rb, a_jl - RB_DECEL_JERK_MS3 * dt)
      a_cmd = min(a_now, a_jl)
      if v > v_max + 0.3:  # MAX ceiling: cruise tracks it at ≤ comfort
        a_cmd = min(a_cmd, -0.7)
      if first_slow_d is None and a_cmd < -0.05:
        first_slow_d = d
    peak = max(peak, -a_cmd)
    v = max(0.0, v + a_cmd * dt)
    d -= v * dt
  return v, peak, first_slow_d


def test_55mph_highway_approach():
  """Starts at the onset (~415 m from 55 mph at 0.7 m/s²), reaches ring speed by entry, never above ~1.5 m/s²."""
  v_entry, peak, first = _simulate(55.0, 600.0)
  assert first is not None and 430.0 <= first <= 460.0, first
  target = roundabout_target_ms(0.0)
  assert v_entry <= target + 1.0 * MPH, v_entry / MPH
  assert peak <= 1.5 + 1e-6
  assert peak <= 1.0  # typical ~0.7, no brick wall


def test_max_profile_is_smooth_no_step():
  """MAX ramps down from ~current speed instead of stepping 35 → 18."""
  v = 35 * MPH
  onset = rb_decel_onset_m(v)
  first = roundabout_ease_v_ms(RoundaboutHint(approaching=True, distance_m=onset), v, v)
  assert first >= v - 1.0 * MPH
  prev = first
  for d in range(int(onset), 0, -5):
    cur = roundabout_ease_v_ms(RoundaboutHint(approaching=True, distance_m=float(d)), v, v)
    assert cur <= prev + 1e-9
    assert prev - cur <= 1.0  # per 5 m
    prev = cur
  assert roundabout_ease_v_ms(RoundaboutHint(approaching=True, distance_m=RB_PROFILE_BUFFER_M), v, v) == \
    pytest.approx(roundabout_target_ms(0.0))


def test_late_hint_still_capped():
  a = roundabout_decel_ms2(RoundaboutHint(approaching=True, distance_m=40.0), 50 * MPH)
  assert a >= RB_A_MAX_DECEL_MS2 - 1e-9


def test_replay_watertown_ring_oct11():
  """Route 00000185--d9fc14b285 ring (17:02:37–17:03:07 CT) replayed through the new profile."""
  with open(DATA) as f:
    rows = json.load(f)
  hit = [r for r in rows if r["appr"]]
  assert hit, "ring approach present"
  first = hit[0]
  # Old: MAX stepped 35.2 → 18.0 mph at 133 m. New profile at 133 m is near current speed.
  new_max = roundabout_ease_v_ms(RoundaboutHint(approaching=True, distance_m=first["d"],
                                                speed_limit_ms=first["rb"]), first["v"], first["v"]) / MPH
  assert new_max >= 25.0
  assert rb_decel_onset_m(first["v"], roundabout_target_ms(first["rb"])) == RB_ONSET_MIN_M
  # Required decel never above 1.5 along the logged approach, and ring target is 18 mph.
  for r in hit:
    a = roundabout_decel_ms2(RoundaboutHint(approaching=True, distance_m=r["d"], speed_limit_ms=r["rb"]), r["v"])
    assert a >= -1.5
  assert roundabout_target_ms(first["rb"]) / MPH == pytest.approx(18.0)
  assert math.isfinite(new_max)


# --- Oct 13: exit restore on right blinker + unwinding, capped braking -----------------------------------------
def _ring_hint():
  return RoundaboutHint(on_roundabout=True, way_id=1)


def test_rb_brake_cap_near_one():
  assert -1.05 <= RB_A_MAX_DECEL_MS2 <= -0.95
  assert 1.5 * MPH <= RB_EXIT_RESTORE_MS2 <= 2.0 * MPH


def test_blinker_alone_does_not_restore():
  g = RingExitGate()
  h = _ring_hint()
  for _ in range(40):  # tight ring, curvature steady, blinker on
    assert not g.update(h, True, 0.05, 0.1)


def test_unwind_without_blinker_does_not_restore():
  g = RingExitGate()
  h = _ring_hint()
  for k in [0.05] * 20 + [0.01] * 20:
    assert not g.update(h, False, k, 0.1)


def test_blinker_plus_unwind_restores_and_latches():
  g = RingExitGate()
  h = _ring_hint()
  for _ in range(20):
    g.update(h, False, 0.05, 0.1)
  rel = [g.update(h, True, 0.015, 0.1) for _ in range(20)]
  assert any(rel)
  assert g.update(h, False, 0.015, 0.1)  # latched after the blinker drops
  assert not g.update(None, False, 0.0, 0.1)  # off the ring: reset
