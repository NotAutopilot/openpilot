"""Map pack v4 roundabouts: rings + approach roads without maxspeed (rb_ways).

Fixture: Soco Rd / Dellwood Rd / Jonathan Creek Rd ring (Maggie Valley NC),
Sep 28 PM R126 16:52 / R128 17:38 CT. OSM data © OpenStreetMap contributors,
ODbL 1.0. The ring has no maxspeed and no lanes tag, so pack v3 (maxspeed-only
ways) never had it and the roundabout hint / slow-down never fired.
"""
import json
import math
import sqlite3
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.local_refresh import merge_ways_into_db
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB
from openpilot.selfdrive.mapd.overpass import overpass_query, ways_from_overpass
from openpilot.selfdrive.mapd.roundabout import RB_PROFILE_BUFFER_M, roundabout_target_ms
from openpilot.selfdrive.mapd.roundabout_map import (
  PARAM_RING, ROLE_APPROACH, ROLE_RING, RingCache, RingGeometry, local_xy, rb_rows_from_overpass,
  ring_geometry, roundabout_comfort_speed_ms,
)

FIXTURE = Path(__file__).parent / "data" / "soco_roundabout_overpass.json"
SOCO_WAY = 1514082747
SOCO_RING_WAYS = {1514082747, 1517278969, 1517278970, 1517279058, 1517279059, 1517279060}
SOCO_CENTER = (35.52208, -83.02814)
# R128 17:37:59.9 CT, first mapd hint ~192 m east of the ring (westbound on Dellwood Rd).
R128_FIRST_HINT = (35.52224, -83.02645)   # decel onset (RB_DECEL_ONSET_M = 140): ~131 m out
# A tagged speed way near the ring (synthetic, not in OSM) to prove speed matching is unchanged.
SPEED_COORDS: list[tuple[float, float]] = [(35.5240, -83.0330), (35.5240, -83.0230)]
SPEED_WAY = {"way_id": 900000001, "name": "Test Rd", "highway": "primary", "maxspeed_ms": 45 * CV.MPH_TO_MS,
             "coords": SPEED_COORDS, "junction": ""}


def _payload():
  return json.loads(FIXTURE.read_text())


def _rows():
  return rb_rows_from_overpass(_payload())


def _ring(rows=None):
  rows = _rows() if rows is None else rows
  return ring_geometry([r for r in rows if r["role"] == ROLE_RING], [r for r in rows if r["role"] == ROLE_APPROACH],
                       seed_id=SOCO_WAY)


def _db(tmp_path, *, rb=True, name="pack.sqlite"):
  path = str(tmp_path / name)
  con = OsmSpeedLimitDB.create(path)
  w = SPEED_WAY
  OsmSpeedLimitDB.insert_way(con, w["way_id"], w["name"], w["highway"], w["maxspeed_ms"], w["coords"], junction="")
  if rb:
    for r in _rows():
      OsmSpeedLimitDB.insert_rb_way(con, r)
    OsmSpeedLimitDB.recount_rb_ways(con)
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  db.open()
  return db, path


def test_fixture_is_attributed_odbl():
  assert "OpenStreetMap contributors" in _payload()["attribution"]
  assert "ODbL" in _payload()["attribution"]


def test_soco_ring_and_approaches_are_extracted_without_maxspeed():
  rows = _rows()
  rings = {r["way_id"] for r in rows if r["role"] == ROLE_RING}
  approaches = {r["way_id"] for r in rows if r["role"] == ROLE_APPROACH}
  assert rings == SOCO_RING_WAYS
  assert SOCO_WAY in rings
  assert {1514082743, 1514082744, 1514082745, 1514082746} <= approaches   # Dellwood / Soco both directions
  assert all(r["maxspeed_ms"] == 0.0 for r in rows)
  # None of these reach the speed-limit ways (no maxspeed): pack v3 never had the ring.
  assert ways_from_overpass(_payload()) == []


def test_soco_ring_fit():
  ring = _ring()
  assert ring is not None
  assert 21.0 < ring.radius_m < 22.3
  assert ring.rms_m < 0.6
  assert ring.ccw                      # US counter-clockwise circulation
  assert ring.coverage_deg > 330.0
  assert set(ring.way_ids) == SOCO_RING_WAYS
  x, y = local_xy(ring.lat, ring.lon, *SOCO_CENTER)
  assert math.hypot(x, y) < 3.0


def test_approaches_are_stored_in_travel_direction():
  ring = _ring()
  inbound = outbound = 0
  for a in ring.approaches:
    r0, r1 = math.hypot(*a[0]), math.hypot(*a[-1])
    inbound += r1 < r0
    outbound += r1 > r0
  # Oneway Dellwood/Soco pairs: one inbound + one outbound each side.
  assert inbound >= 2 and outbound >= 2
  # Westbound Dellwood (1514082743) enters the ring: its polyline must end at the ring.
  rows = [r for r in _rows() if r["way_id"] == 1514082743]
  single = ring_geometry([r for r in _rows() if r["role"] == ROLE_RING], rows, seed_id=SOCO_WAY)
  a = single.approaches[0]
  assert math.hypot(*a[-1]) < math.hypot(*a[0])


def test_ring_json_roundtrip():
  ring = _ring()
  back = RingGeometry.from_json(json.dumps(ring.to_json()))
  assert back is not None
  assert back.way_ids == ring.way_ids and back.ccw == ring.ccw
  assert abs(back.radius_m - ring.radius_m) < 0.01
  assert len(back.approaches) == len(ring.approaches)
  assert RingGeometry.from_json({"v": 999}) is None
  assert RingGeometry.from_json("not json") is None
  assert PARAM_RING == "NAPRoundaboutRing"


def test_comfort_speed_soco_about_15_mph():
  ring = _ring()
  v = roundabout_comfort_speed_ms(ring.radius_m, ring.lanes, ring.maxspeed_ms)
  assert 15.0 * CV.MPH_TO_MS <= v <= 16.5 * CV.MPH_TO_MS
  # Inner lane (2 lanes, lanes tag missing) at that speed: <= 2.5 m/s^2.
  assert v * v / (ring.radius_m - 1.75) <= 2.5 + 1e-6
  # Planner target floors it at the 18 mph ring speed (Sep 30; the comfort speed itself is unchanged).
  assert roundabout_target_ms(v) == pytest.approx(18.0 * CV.MPH_TO_MS)
  # OSM maxspeed still caps; bigger rings are faster; bad input safe.
  assert roundabout_comfort_speed_ms(21.6, 2, 5.0) == pytest.approx(5.0)
  assert roundabout_comfort_speed_ms(40.0, 2) > v
  assert roundabout_comfort_speed_ms(0.0, 2) >= 0.0


def test_v4_pack_ring_geometry_from_first_hint_distance(tmp_path):
  db, _ = _db(tmp_path)
  hint = db.find_roundabout(R128_FIRST_HINT[0], R128_FIRST_HINT[1], 270.0, current_highway="primary")
  # Hint fires from the approach, then geometry is fit around the hinted way (not the car ~190 m out).
  ring = db.ring_geometry(SOCO_WAY, *R128_FIRST_HINT)
  assert ring is not None and set(ring.way_ids) == SOCO_RING_WAYS
  assert hint.approaching and int(hint.way_id) in SOCO_RING_WAYS
  assert 120.0 < hint.distance_m < 140.0


def test_v4_pack_find_roundabout_sees_untagged_speed_ring(tmp_path):
  db, _ = _db(tmp_path)
  lat, lon = SOCO_CENTER
  x_east = local_xy(lat, lon + 0.0006, lat, lon)[0]
  assert x_east > 40
  h = db.find_roundabout(lat, lon + 0.0006, 270.0, current_highway="primary")
  assert h.approaching or h.on_roundabout
  assert int(h.way_id) in SOCO_RING_WAYS


def test_v3_pack_without_rb_tables_still_works(tmp_path):
  db, path = _db(tmp_path, rb=False, name="v3.sqlite")
  con = sqlite3.connect(path)
  con.execute("DROP TABLE IF EXISTS rb_ways")
  con.execute("DROP TABLE IF EXISTS rb_ways_rtree")
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  db.open()
  assert db.ring_geometry(SOCO_WAY, *SOCO_CENTER) is None
  m = db.lookup(35.5240, -83.0280, 90.0, v_ego_ms=20.0)
  assert m is not None and m.speed_limit_ms == pytest.approx(SPEED_WAY["maxspeed_ms"])
  h = db.find_roundabout(*SOCO_CENTER, 90.0, current_highway="primary")
  assert not h.on_roundabout


def test_speed_matching_unchanged_by_rb_rows(tmp_path):
  with_rb, _ = _db(tmp_path, rb=True, name="a.sqlite")
  without, _ = _db(tmp_path, rb=False, name="b.sqlite")
  for lat, lon, brg in ((35.5240, -83.0280, 90.0), (35.5240, -83.0300, 270.0), (35.52224, -83.02602, 270.0),
                        (35.52208, -83.02814, 0.0)):
    a = with_rb.lookup(lat, lon, brg, v_ego_ms=15.0)
    b = without.lookup(lat, lon, brg, v_ego_ms=15.0)
    assert (a is None) == (b is None)
    if a is not None:
      assert a.way_id == b.way_id and a.speed_limit_ms == b.speed_limit_ms
  # rb rows never land in the speed `ways` table.
  con = sqlite3.connect(str(tmp_path / "a.sqlite"))
  assert con.execute("SELECT COUNT(*) FROM ways").fetchone()[0] == 1
  assert con.execute("SELECT COUNT(*) FROM rb_ways WHERE role='ring'").fetchone()[0] == 6
  assert con.execute("SELECT value FROM meta WHERE key='rb_ring_way_count'").fetchone()[0] == "6"


def test_overpass_queries_include_rings_and_approaches():
  for unmarked in (False, True):
    q = overpass_query(35.0, -84.0, 36.0, -83.0, include_unmarked=unmarked)
    assert '["junction"~"^(roundabout|circular)$"]' in q
    assert "node(w.rb)->.rbn;" in q
    assert 'way(bn.rbn)["highway"]->.ap;' in q
    assert "\n  .rb;\n  .ap;" in q        # both sets are output in the union
  # Old form still available.
  q = overpass_query(35.0, -84.0, 36.0, -83.0, include_roundabouts=False)
  assert "junction" not in q and '["maxspeed"]' in q


def test_extra_rb_elements_do_not_change_speed_ways():
  base = {"elements": [{"type": "way", "id": SPEED_WAY["way_id"], "tags": {"highway": "primary", "maxspeed": "45 mph"},
                        "geometry": [{"lat": a, "lon": b} for a, b in SPEED_COORDS]}]}
  combined = {"elements": base["elements"] + _payload()["elements"]}
  assert ways_from_overpass(combined) == ways_from_overpass(base)


def test_refresh_merge_replaces_rb_rows_in_bbox(tmp_path):
  _, path = _db(tmp_path, rb=False, name="installed.sqlite")
  bbox = (35.50, -83.05, 35.55, -83.00)
  merge_ways_into_db(path, [SPEED_WAY], bbox, rb_rows=_rows())
  con = sqlite3.connect(path)
  assert con.execute("SELECT COUNT(*) FROM rb_ways WHERE role='ring'").fetchone()[0] == 6
  # Second refresh replaces (no duplicates); None leaves rb_ways alone.
  merge_ways_into_db(path, [SPEED_WAY], bbox, rb_rows=_rows())
  merge_ways_into_db(path, [SPEED_WAY], bbox, rb_rows=None)
  assert con.execute("SELECT COUNT(*) FROM rb_ways WHERE role='ring'").fetchone()[0] == 6
  con.close()
  db = OsmSpeedLimitDB(path)
  db.open()
  assert db.ring_geometry(SOCO_WAY, *R128_FIRST_HINT) is not None


def test_ring_cache_publishes_once(tmp_path):
  db, _ = _db(tmp_path)
  cache, sent = RingCache(), []
  for t in range(5):
    g = cache.update(db, SOCO_WAY, *R128_FIRST_HINT, float(t), publish=sent.append)
    assert g is not None
  cache.update(db, 1517278969, *SOCO_CENTER, 6.0, publish=sent.append)   # another way of the same ring
  assert len(sent) == 1 and sent[0]["ids"] == sorted(SOCO_RING_WAYS)
  assert cache.update(db, 0, *SOCO_CENTER, 7.0) is None


def test_builder_from_json_writes_pack_v4(tmp_path):
  from scripts.nap.build_osm_speed_limits import main as build_main
  src = tmp_path / "in.json"
  payload = _payload()
  payload["elements"].append({"type": "way", "id": SPEED_WAY["way_id"], "tags": {"highway": "primary", "maxspeed": "45 mph"},
                              "nodes": [1, 2], "geometry": [{"lat": a, "lon": b} for a, b in SPEED_COORDS]})
  src.write_text(json.dumps(payload))
  out = tmp_path / "out.sqlite"
  assert build_main(["--from-json", str(src), "--out", str(out)]) in (0, None)
  con = sqlite3.connect(str(out))
  assert con.execute("SELECT COUNT(*) FROM ways").fetchone()[0] == 1
  assert con.execute("SELECT COUNT(*) FROM rb_ways WHERE role='ring'").fetchone()[0] == 6
  assert con.execute("SELECT value FROM meta WHERE key='pack_schema'").fetchone()[0] == "4"
  con.close()
  out2 = tmp_path / "out2.sqlite"
  assert build_main(["--from-json", str(src), "--out", str(out2), "--no-roundabouts"]) in (0, None)
  con = sqlite3.connect(str(out2))
  assert con.execute("SELECT COUNT(*) FROM ways").fetchone()[0] == 1
  assert con.execute("SELECT value FROM meta WHERE key='pack_schema'").fetchone() is None
  con.close()


# --- follow-ups: ring-only message validity, missing-ring-data warning -------------------------------------------

class _Stop(Exception):
  pass


def _run_mapd(monkeypatch, db_path, fixes):
  """Drive mapd.main() over GPS fixes [(lat, lon, bearing)]; return the published messages."""
  from openpilot.selfdrive.mapd import mapd
  sent = []

  class _G:
    def __init__(self, lat, lon, brg):
      self.latitude, self.longitude, self.bearingDeg = lat, lon, brg
      self.speed, self.horizontalAccuracy = 15.0, 3.0

  class _Sm:
    def __init__(self, *_a, **_k):
      self.recv_frame, self.recv_time, self._g = {"gpsLocationExternal": 1}, {"gpsLocationExternal": 1e18}, None

    def update(self, _t):
      self._g = _G(*fixes[len(sent)])

    def __getitem__(self, key):
      return self._g

  class _Pm:
    def __init__(self, *_a, **_k):
      pass

    def send(self, _name, msg):
      sent.append(msg)

  class _Rk:
    def __init__(self, *_a, **_k):
      pass

    def keep_time(self):
      if len(sent) >= len(fixes):
        raise _Stop

  class _Params:
    def get(self, key, *_a, **_k):
      return db_path if key == "NAPMapSpeedDbPath" else None

    def put(self, *_a, **_k):
      pass

  monkeypatch.setattr(mapd.messaging, "SubMaster", _Sm)
  monkeypatch.setattr(mapd.messaging, "PubMaster", _Pm)
  monkeypatch.setattr(mapd, "Ratekeeper", _Rk)
  monkeypatch.setattr(mapd, "Params", _Params)
  monkeypatch.setattr(mapd.time, "monotonic", lambda: 100.0)
  monkeypatch.setattr(mapd, "gps_sample_from_sm", lambda sm, now=None: (*fixes[len(sent)], True))
  monkeypatch.setattr(mapd, "persist_last_gps_position", lambda *a, **k: None)
  with pytest.raises(_Stop):
    mapd.main()
  return sent


ON_SPEED_WAY = (35.5240, -83.0280, 90.0)          # matches the tagged speed way, no ring nearby
OFF_MAP = (36.5, -84.0, 90.0)                      # nothing in the pack
RING_APPROACH = (*R128_FIRST_HINT, 270.0)          # ring hint, no speed-limit match


def test_mapd_ring_hint_keeps_message_valid_without_speed_match(tmp_path, monkeypatch):
  _db(tmp_path)
  msgs = _run_mapd(monkeypatch, str(tmp_path / "pack.sqlite"), [RING_APPROACH, ON_SPEED_WAY, OFF_MAP])
  ring, speed, off = msgs
  assert ring.valid and ring.liveMapDataNAP.approachingRoundabout
  assert not ring.liveMapDataNAP.speedLimitValid and ring.liveMapDataNAP.speedLimit == 0.0
  assert ring.liveMapDataNAP.roundaboutSpeedLimit == pytest.approx(15.0 * CV.MPH_TO_MS, abs=1.5)
  assert speed.valid and speed.liveMapDataNAP.speedLimitValid
  assert not speed.liveMapDataNAP.approachingRoundabout and not speed.liveMapDataNAP.onRoundabout
  assert not off.valid and not off.liveMapDataNAP.speedLimitValid


def test_mapd_without_ring_rows_is_invalid_at_the_ring(tmp_path, monkeypatch):
  _db(tmp_path, rb=False)
  msgs = _run_mapd(monkeypatch, str(tmp_path / "pack.sqlite"), [RING_APPROACH, ON_SPEED_WAY])
  assert not msgs[0].valid and not msgs[0].liveMapDataNAP.approachingRoundabout
  assert msgs[1].valid and msgs[1].liveMapDataNAP.speedLimitValid


def test_map_msg_valid_only_changes_with_a_ring_hint():
  from openpilot.selfdrive.mapd.roundabout_status import map_msg_valid
  for gps in (False, True):
    for loaded in (False, True):
      for matched in (False, True):
        assert map_msg_valid(gps_ok=gps, db_loaded=loaded, matched=matched, ring_hint=False) == (gps and loaded and matched)
        assert map_msg_valid(gps_ok=gps, db_loaded=loaded, matched=matched, ring_hint=True) == (gps and loaded)


def test_pack_ring_count_and_summary(tmp_path, monkeypatch):
  from openpilot.selfdrive.mapd.roundabout_status import (
    RING_MISSING_TEXT, installed_ring_summary, pack_ring_count, ring_data_summary,
  )
  _db(tmp_path, name="v4.sqlite")
  _db(tmp_path, rb=False, name="norb.sqlite")
  _db(tmp_path, name="v3.sqlite")
  con = sqlite3.connect(str(tmp_path / "v3.sqlite"))
  con.execute("DROP TABLE rb_ways")
  con.execute("DROP TABLE rb_ways_rtree")
  con.commit()
  con.close()
  four, empty, v3, gone = (str(tmp_path / n) for n in ("v4.sqlite", "norb.sqlite", "v3.sqlite", "nope.sqlite"))
  assert pack_ring_count(four) == 6
  assert pack_ring_count(empty) == 0          # rb tables exist, no rows: Refresh never stored a ring
  assert pack_ring_count(v3) == 0             # published v3 pack: no rb tables at all
  assert pack_ring_count(gone) is None and pack_ring_count("") is None and pack_ring_count(None) is None
  (tmp_path / "junk.sqlite").write_bytes(b"not a database" * 100)
  assert pack_ring_count(str(tmp_path / "junk.sqlite")) is None
  assert ring_data_summary(6) == "6 rings" and ring_data_summary(1200) == "1,200 rings"
  assert ring_data_summary(0) == RING_MISSING_TEXT and "Refresh maps" in RING_MISSING_TEXT
  assert ring_data_summary(None) == "Not installed"
  assert installed_ring_summary(four) == "6 rings"
  assert installed_ring_summary(v3) == RING_MISSING_TEXT
  assert installed_ring_summary(gone) == "Not installed"
  monkeypatch.setenv("NAP_MAP_DB", four)
  assert installed_ring_summary() == "6 rings"           # default path follows the installed pack
  monkeypatch.setenv("NAP_MAP_DB", v3)
  assert installed_ring_summary() == RING_MISSING_TEXT


def test_ring_data_watch_logs_once_per_pack_state(tmp_path):
  from openpilot.selfdrive.mapd.roundabout_status import RingDataWatch
  _db(tmp_path, rb=False, name="a.sqlite")
  path = str(tmp_path / "a.sqlite")
  w = RingDataWatch()
  first = w.check(path)
  assert first is not None and first[0] == "warning" and "no roundabout ring rows" in first[1] and "Refresh maps" in first[1]
  assert w.check(path) is None and w.check(path) is None
  con = sqlite3.connect(path)                   # Refresh maps adds rings: pack changes, log the new state once
  for r in _rows():
    OsmSpeedLimitDB.insert_rb_way(con, r)
  OsmSpeedLimitDB.recount_rb_ways(con)
  con.commit()
  con.close()
  again = w.check(path)
  assert again is not None and again[0] == "info" and "6 roundabout ring rows" in again[1]
  assert w.check(path) is None
  assert w.check(str(tmp_path / "missing.sqlite")) is None


def test_mapd_logs_missing_ring_rows_once(tmp_path, monkeypatch):
  from openpilot.selfdrive.mapd import mapd
  _db(tmp_path, rb=False)
  logged = []
  monkeypatch.setattr(mapd.cloudlog, "warning", lambda msg, *a: logged.append(msg % a if a else msg))
  _run_mapd(monkeypatch, str(tmp_path / "pack.sqlite"), [ON_SPEED_WAY] * 4)
  assert [m for m in logged if "no roundabout ring rows" in m] and len([m for m in logged if "ring rows" in m]) == 1


# --- card: the ring slow-down does not need a speed-limit match --------------------------------------------------

def _card_harness(limit_mph=45.0):
  from openpilot.selfdrive.car.tests import test_preap_engage_max_after_pause as t
  h, cs, md, _eng = t._harness(limit_mph)
  return h, cs, md, t._run


def _set_ring(md, *, on=False, approaching=True, ring_mph=15.8, dist=120.0):
  md.onRoundabout, md.approachingRoundabout = on, approaching
  md.roundaboutSpeedLimit, md.roundaboutDistance, md.roundaboutWayId = ring_mph * CV.MPH_TO_MS, dist, SOCO_WAY


def _profile_mph(dist=120.0, ring_mph=15.8):
  """Oct 11: approach MAX follows the 0.7 m/s² profile down to the ring target (no 18 mph step)."""
  from openpilot.selfdrive.mapd.roundabout import rb_profile_v_ms, roundabout_target_ms
  return rb_profile_v_ms(dist, roundabout_target_ms(ring_mph * CV.MPH_TO_MS)) / CV.MPH_TO_MS


def _hud_mph(h):
  return h.v_cruise_helper.v_cruise_kph / CV.MPH_TO_KPH


def test_card_ring_hint_drops_hud_max_without_speed_match():
  """Willmar/Maggie Valley: no maxspeed on the ring, so speedLimitValid is False in the funnel."""
  h, cs, md, run = _card_harness(45.0)
  run(h, cs, 2.0)
  assert _hud_mph(h) == pytest.approx(45.0, abs=0.3)
  md.speedLimitValid, md.speedLimit = False, 0.0
  _set_ring(md)
  run(h, cs, 1.0)
  assert _hud_mph(h) == pytest.approx(_profile_mph(), abs=0.3)
  assert _hud_mph(h) < 45.0 - 5.0
  assert h._map_slew_ms == pytest.approx(_profile_mph() * CV.MPH_TO_MS, abs=0.05)
  _set_ring(md, dist=RB_PROFILE_BUFFER_M)
  run(h, cs, 0.2)
  assert _hud_mph(h) == pytest.approx(18.0, abs=0.3)


def test_card_ring_hint_same_max_with_or_without_speed_match():
  a, cs_a, md_a, run = _card_harness(45.0)
  b, cs_b, md_b, _ = _card_harness(45.0)
  run(a, cs_a, 2.0)
  run(b, cs_b, 2.0)
  _set_ring(md_a)                                # posted 45 still matched
  md_b.speedLimitValid, md_b.speedLimit = False, 0.0
  _set_ring(md_b)                                # no match
  run(a, cs_a, 1.0)
  run(b, cs_b, 1.0)
  assert _hud_mph(a) == pytest.approx(_hud_mph(b), abs=1e-6) == pytest.approx(_profile_mph(), abs=0.3)


def test_card_ring_hint_needs_a_valid_map_message():
  h, cs, md, run = _card_harness(45.0)
  run(h, cs, 2.0)
  md.speedLimitValid, md.speedLimit = False, 0.0
  h.sm.valid["liveMapDataNAP"] = False           # mapd not publishing a valid message: nothing to read
  _set_ring(md)
  run(h, cs, 1.0)
  assert _hud_mph(h) == pytest.approx(45.0, abs=0.3)


def test_card_no_ring_hint_is_identical_with_or_without_ring_fields():
  traces = []
  for extra in ({}, {"onRoundabout": False, "approachingRoundabout": False, "roundaboutDistance": 0.0,
                     "roundaboutSpeedLimit": 0.0, "roundaboutWayId": 0}):
    h, cs, md, run = _card_harness(45.0)
    md.__dict__.update(extra)
    trace = []
    for limit, valid in ((45.0, True), (25.0, True), (0.0, False), (35.0, True)):
      md.speedLimit, md.speedLimitValid = limit * CV.MPH_TO_MS, valid
      for _ in range(150):
        run(h, cs, 0.01)
        trace.append((h.v_cruise_helper.v_cruise_kph, h._map_slew_ms, cs.cruiseState.speed))
    traces.append(trace)
  assert traces[0] == traces[1]
  assert min(t[0] for t in traces[0]) < 45.0 * CV.MPH_TO_KPH


def test_card_and_mapd_never_read_the_assist_toggle():
  """The slow-down is independent of NAPRoundaboutAssist (only the steering assist reads it)."""
  import inspect
  from openpilot.selfdrive.car import card
  from openpilot.selfdrive.mapd import mapd
  for mod in (card, mapd):
    assert "NAPRoundaboutAssist" not in inspect.getsource(mod)


def _leave_ring(md):
  md.onRoundabout = False
  md.approachingRoundabout = False


def test_ring_exit_ramps_max_up_from_current_speed():
  """Oct 7 14:43:57: after the ring, MAX was 18 while ego was 25 and the car braked.

  The funnel may still show the ring speed on the way in. On the way out,
  MAX starts at least at current speed and climbs toward the posted limit.
  """
  h, cs, md, run = _card_harness(40.0)
  run(h, cs, 1.0)
  assert _hud_mph(h) == pytest.approx(40.0, abs=0.4)
  _set_ring(md, on=False, approaching=True, ring_mph=15.0, dist=135.0)
  cs.vEgo = 36.0 * CV.MPH_TO_MS
  run(h, cs, 0.4)
  assert _hud_mph(h) == pytest.approx(min(40.0, _profile_mph(135.0, 15.0)), abs=0.4)
  _set_ring(md, on=True, approaching=False, ring_mph=15.0, dist=0.0)
  run(h, cs, 0.2)
  assert _hud_mph(h) == pytest.approx(18.0, abs=0.4)

  _leave_ring(md)
  cs.vEgo = 25.0 * CV.MPH_TO_MS
  run(h, cs, 0.01)
  hud = _hud_mph(h)
  assert hud >= 24.5
  assert hud < 28.0
  run(h, cs, 1.0)
  later = _hud_mph(h)
  assert later > hud + 1.0
  assert later < 40.5
  prev = later
  for _ in range(30):
    run(h, cs, 0.01)
    step = _hud_mph(h)
    assert step > prev - 0.3
    prev = step
  run(h, cs, 25.0)
  assert _hud_mph(h) == pytest.approx(40.0, abs=0.6)


def test_set_inside_ring_does_not_latch_ring_speed():
  """A SET while on the ring keeps the ring ceiling only until the exit."""
  h, cs, md, run = _card_harness(40.0)
  run(h, cs, 1.0)
  _set_ring(md, on=True, approaching=False, ring_mph=15.0, dist=0.0)
  cs.vEgo = 22.0 * CV.MPH_TO_MS
  run(h, cs, 0.3)
  assert _hud_mph(h) == pytest.approx(18.0, abs=0.4)
  eng = h.CI.CS.engagement
  eng._nap_set_resume_long = True
  run(h, cs, 0.01)
  assert _hud_mph(h) == pytest.approx(18.0, abs=0.4)
  held = h._map_hold.held_max_kph
  assert held is None or abs(held / CV.MPH_TO_KPH - 18.0) > 2.0
  fsm = getattr(eng, "_nap_held_max_kph", None)
  assert fsm is None or abs(float(fsm) / CV.MPH_TO_KPH - 18.0) > 2.0

  _leave_ring(md)
  cs.vEgo = 25.0 * CV.MPH_TO_MS
  run(h, cs, 0.01)
  assert _hud_mph(h) >= 24.5
  run(h, cs, 2.0)
  assert 26.0 < _hud_mph(h) < 40.5
  assert h._map_hold.sticky_set_kph is None or abs(
    h._map_hold.sticky_set_kph / CV.MPH_TO_KPH - 18.0) > 2.0


def test_ring_exit_above_the_limit_eases_down():
  """Over the posted limit, MAX starts at current speed and does not step to the ring."""
  h, cs, md, run = _card_harness(30.0)
  run(h, cs, 1.0)
  _set_ring(md, approaching=True, ring_mph=15.0, dist=RB_PROFILE_BUFFER_M)
  run(h, cs, 0.2)
  assert _hud_mph(h) == pytest.approx(18.0, abs=0.4)
  _leave_ring(md)
  cs.vEgo = 40.0 * CV.MPH_TO_MS
  prev = None
  for _ in range(20):
    run(h, cs, 0.01)
    hud = _hud_mph(h)
    if prev is not None:
      assert hud > prev - 0.5
    prev = hud
  assert _hud_mph(h) > 38.0


def _ring_way(tags):
  coords = [(45.0 + 0.0002 * math.sin(a), -95.0 + 0.0003 * math.cos(a)) for a in [i * math.pi / 4 for i in range(9)]]
  return {"way_id": 1, "tags": {"highway": "tertiary", "junction": "roundabout", **tags},
          "node_ids": list(range(100, 109)), "coords": coords}


@pytest.mark.parametrize("tags, mph", [
  ({"maxspeed": "30 mph", "maxspeed:advisory": "15 mph"}, 15),
  ({"maxspeed": "20 mph", "maxspeed:advisory": "25 mph"}, 20),
  ({"maxspeed:advisory": "15 mph"}, 15),
  ({"maxspeed": "25 mph"}, 25),
])
def test_ring_advisory_uses_lower_speed(tags, mph):
  from openpilot.selfdrive.mapd.roundabout_map import roundabout_rows
  rows = roundabout_rows([_ring_way(tags)])
  assert rows[0]["role"] == ROLE_RING
  assert rows[0]["maxspeed_ms"] == pytest.approx(mph * CV.MPH_TO_MS, abs=0.05)


def test_approach_ignores_advisory():
  from openpilot.selfdrive.mapd.roundabout_map import roundabout_rows
  ring = _ring_way({})
  road = {"way_id": 2, "tags": {"highway": "primary", "maxspeed": "45 mph", "maxspeed:advisory": "15 mph"},
          "node_ids": [100, 200], "coords": [ring["coords"][0], (45.0, -94.99)]}
  app = [r for r in roundabout_rows([ring, road]) if r["role"] == ROLE_APPROACH]
  assert app and all(r["maxspeed_ms"] == pytest.approx(45 * CV.MPH_TO_MS, abs=0.05) for r in app)


def test_mini_roundabout_is_small_ring_with_approaches():
  from openpilot.selfdrive.mapd.roundabout_map import MINI_RB_RADIUS_M, mini_roundabout_way, roundabout_rows
  lat, lon = 45.12, -95.04
  mini = mini_roundabout_way(555, lat, lon, {"highway": "mini_roundabout"})
  assert mini["way_id"] == -555
  road = {"way_id": 3, "tags": {"highway": "residential"}, "node_ids": [554, 555, 556],
          "coords": [(lat - 0.001, lon), (lat, lon), (lat + 0.001, lon)]}
  rows = roundabout_rows([mini, road])
  rings = [r for r in rows if r["role"] == ROLE_RING]
  assert len(rings) == 1 and len([r for r in rows if r["role"] == ROLE_APPROACH]) == 2
  geo = ring_geometry(rings)
  assert geo is not None
  assert 6.0 <= geo.radius_m <= 8.0 and abs(geo.radius_m - MINI_RB_RADIUS_M) < 0.5
  from openpilot.selfdrive.mapd.roundabout import RB_V_MIN_MS
  # A 7 m ring is tighter than any comfort target: the funnel floor applies.
  assert roundabout_target_ms(roundabout_comfort_speed_ms(geo.radius_m, geo.lanes, geo.maxspeed_ms)) \
    == pytest.approx(RB_V_MIN_MS, abs=0.01)


# --- Oct 11: ring exit restore with / without a posted limit ------------------------------------------------------
# MAX is a hard ceiling: a driver-set pre-ring MAX is restored exactly; only a map-auto MAX (no driver set,
# no posted limit now) may rise to the speed the car had when the ring was first detected.


KPH = CV.MPH_TO_KPH


def _ring_exit_target_ms(*a):
  from openpilot.selfdrive.car.card import _ring_exit_target_ms as f
  return f(*a)


def _hold(sticky=None, held=None, policy=None):
  return SimpleNamespace(sticky_set_kph=None if sticky is None else sticky * KPH,
                         held_max_kph=None if held is None else held * KPH,
                         policy_kph=None if policy is None else policy * KPH)


def test_posted_limit_wins_when_no_driver_sticky():
  assert _ring_exit_target_ms(_hold(policy=30), 45 * CV.MPH_TO_MS, None, 26 * CV.MPH_TO_MS) == pytest.approx(45 * CV.MPH_TO_MS)


def test_no_posted_driver_set_restores_exactly_watertown():
  """Route 00000185 17:03:07: stalk-set 35.2 mph, detected at 26.5 mph, SD has no posted limit."""
  t = _ring_exit_target_ms(_hold(held=35.2), None, None, 26.5 * CV.MPH_TO_MS)
  assert t == pytest.approx(35.2 * CV.MPH_TO_MS)


def test_driver_set_is_a_ceiling_even_below_detection_speed():
  t = _ring_exit_target_ms(_hold(sticky=30), None, None, 45 * CV.MPH_TO_MS)
  assert t == pytest.approx(30 * CV.MPH_TO_MS)
  t = _ring_exit_target_ms(_hold(held=30), None, None, 45 * CV.MPH_TO_MS)
  assert t == pytest.approx(30 * CV.MPH_TO_MS)


def test_map_auto_max_rises_to_detection_speed_without_posted():
  t = _ring_exit_target_ms(_hold(policy=30), None, None, 45 * CV.MPH_TO_MS)
  assert t == pytest.approx(45 * CV.MPH_TO_MS)
  t = _ring_exit_target_ms(_hold(policy=50), None, None, 45 * CV.MPH_TO_MS)
  assert t == pytest.approx(50 * CV.MPH_TO_MS)
  assert _ring_exit_target_ms(_hold(policy=30), None, None, None) == pytest.approx(30 * CV.MPH_TO_MS)


def test_nothing_known():
  assert _ring_exit_target_ms(_hold(), None, None, 40 * CV.MPH_TO_MS) is None
