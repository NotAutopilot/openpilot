"""US state-default fills (us_statutory) for untagged OSM ways."""
import os

import pytest

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.local_refresh import refresh_local_maps
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB
from openpilot.selfdrive.mapd.overpass import resolve_way_speed, ways_from_overpass
from openpilot.selfdrive.mapd.us_statutory import IIHS, default_maxspeed_mph, fill_source, is_fill_source, state_index

WATERTOWN_SD = (44.877, -97.10)
WILLMAR_MN = (45.122, -95.043)


@pytest.mark.parametrize("pt, code", [
  (WATERTOWN_SD, "SD"), (WILLMAR_MN, "MN"), ((37.7749, -122.4194), "CA"), ((30.2672, -97.7431), "TX"),
  ((40.7128, -74.0060), "NY"), ((38.9072, -77.0369), "DC"), ((21.3069, -157.8583), "HI"), ((61.2181, -149.9003), "AK"),
])
def test_state_lookup(pt, code):
  assert state_index().state_at(*pt) == code


def test_every_state_has_limits_and_a_polygon():
  import json
  from openpilot.selfdrive.mapd.us_statutory import STATES_JSON
  with open(STATES_JSON) as f:
    codes = set(json.load(f)["states"])
  assert len(IIHS) == 51 and set(IIHS) == codes


@pytest.mark.parametrize("state, hw, tags, urban, mph", [
  ("SD", "primary", {}, False, 65),        # IIHS other roads 70, capped 65
  ("SD", "secondary", {}, False, 55),
  ("SD", "unclassified", {}, False, 55),
  ("SD", "motorway", {}, False, 80),
  ("SD", "motorway_link", {}, False, 45),
  ("SD", "primary_link", {}, True, 25),
  ("SD", "primary", {}, True, 25),
  ("SD", "trunk", {"expressway": "yes"}, False, 70),
  ("SD", "residential", {}, True, None),   # not filled outside MN (size)
  ("SD", "service", {}, False, None),
  ("TX", "primary", {}, True, 30),
  ("PA", "secondary", {}, True, 35),
  ("VT", "secondary", {}, False, 50),
  ("HI", "primary", {}, False, 45),
  ("IL", "motorway", {}, True, 55),
  ("MN", "residential", {}, True, 30),     # MN 169.14 unchanged
  ("MN", "primary", {}, False, 55),
  ("MN", "motorway", {}, False, 70),
  (None, "primary", {}, False, None),
])
def test_default_mph(state, hw, tags, urban, mph):
  assert default_maxspeed_mph(state, hw, tags, urban=urban) == mph


def test_tagged_wins_and_fill_is_marked():
  coords = [(WATERTOWN_SD[0], WATERTOWN_SD[1]), (WATERTOWN_SD[0], WATERTOWN_SD[1] + 0.01)]
  ms, src = resolve_way_speed({"highway": "primary", "maxspeed": "45 mph"}, coords, fill_unmarked=True)
  assert src == "osm" and abs(ms - 45 * CV.MPH_TO_MS) < 0.01
  ms, src = resolve_way_speed({"highway": "primary"}, coords, fill_unmarked=True)
  assert src == fill_source("SD") == "SD_default" and is_fill_source(src)
  assert abs(ms - 65 * CV.MPH_TO_MS) < 0.01
  assert resolve_way_speed({"highway": "primary"}, coords, fill_unmarked=False) == (None, "")


def _way(wid, lat, lon, hw, maxspeed=None, name="Rd"):
  tags = {"highway": hw, "name": name}
  if maxspeed:
    tags["maxspeed"] = f"{maxspeed} mph"
  return {"type": "way", "id": wid, "tags": tags, "nodes": [wid * 10, wid * 10 + 1],
          "geometry": [{"lat": lat, "lon": lon - 0.005}, {"lat": lat, "lon": lon + 0.005}]}


def test_overpass_ways_carry_estimated_flag():
  lat, lon = WATERTOWN_SD
  ways = ways_from_overpass({"elements": [_way(1, lat, lon, "primary", 45), _way(2, lat + 0.02, lon, "primary"),
                                          _way(3, lat + 0.04, lon, "residential")]}, fill_unmarked=True)
  by_id = {w["way_id"]: w for w in ways}
  assert not by_id[1]["estimated"] and by_id[2]["estimated"] and 3 not in by_id


def test_sd_refresh_fills_us212_estimated(tmp_path):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  os.makedirs(os.path.dirname(dest), exist_ok=True)
  con = OsmSpeedLimitDB.create(dest)
  OsmSpeedLimitDB.insert_way(con, 9, "Broadway", "primary", 40 * CV.MPH_TO_MS, [(40.7128, -74.006), (40.7138, -74.006)])
  con.commit()
  con.close()
  lat, lon = WATERTOWN_SD
  payload = {"elements": [_way(212, lat, lon, "primary", name="US 212"), _way(5, lat + 0.03, lon, "secondary", 45)]}
  refresh_local_maps(dest=dest, live_fix=WATERTOWN_SD, last_gps_raw=None, payload=payload)
  db = OsmSpeedLimitDB(dest)
  assert db.open()
  m = db.lookup(lat, lon, bearing_deg=90.0)
  assert m is not None and m.way_id == 212 and m.estimated
  assert abs(m.speed_limit_ms - 65 * CV.MPH_TO_MS) < 0.2
  t = db.lookup(lat + 0.03, lon, bearing_deg=90.0)
  assert t is not None and not t.estimated and abs(t.speed_limit_ms - 45 * CV.MPH_TO_MS) < 0.2
  db.close()


def test_untagged_piece_between_posted_takes_posted_limit():
  lat = 44.8901
  ways = ways_from_overpass({"elements": [
    {"type": "way", "id": 1, "tags": {"highway": "primary", "name": "9th Avenue Southeast", "maxspeed": "45 mph"},
     "geometry": [{"lat": lat, "lon": -97.070}, {"lat": lat, "lon": -97.0685}]},
    {"type": "way", "id": 2, "tags": {"highway": "primary", "name": "9th Avenue Southeast"},
     "geometry": [{"lat": lat, "lon": -97.0685}, {"lat": lat, "lon": -97.0678}]},
    {"type": "way", "id": 3, "tags": {"highway": "primary", "name": "9th Avenue Southeast", "maxspeed": "45 mph"},
     "geometry": [{"lat": lat, "lon": -97.0678}, {"lat": lat, "lon": -97.066}]},
    {"type": "way", "id": 4, "tags": {"highway": "primary", "name": "Other Rd"},
     "geometry": [{"lat": lat, "lon": -97.0678}, {"lat": lat + 0.01, "lon": -97.0678}]},
  ]}, fill_unmarked=True)
  by_id = {w["way_id"]: w for w in ways}
  assert by_id[2]["estimated"] and abs(by_id[2]["maxspeed_ms"] - 45 * CV.MPH_TO_MS) < 0.01
  assert abs(by_id[4]["maxspeed_ms"] - 65 * CV.MPH_TO_MS) < 0.01  # different road: state default


def test_mn_ramp_capped():
  assert default_maxspeed_mph("MN", "motorway_link", {}, urban=False) == 45


def test_estimated_limit_capped_at_65_on_car(tmp_path):
  from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB
  path = str(tmp_path / "cap.sqlite")
  con = OsmSpeedLimitDB.create(path)
  lat = 46.8328
  OsmSpeedLimitDB.insert_way(con, 1, "I 94", "motorway", 80 * CV.MPH_TO_MS, [(lat, -100.86), (lat, -100.84)], estimated=True)
  OsmSpeedLimitDB.insert_way(con, 2, "I 94", "motorway", 80 * CV.MPH_TO_MS, [(lat + 0.1, -100.86), (lat + 0.1, -100.84)])
  OsmSpeedLimitDB.insert_way(con, 3, "Main", "primary", 45 * CV.MPH_TO_MS, [(lat + 0.2, -100.86), (lat + 0.2, -100.84)], estimated=True)
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  est = db.lookup(lat, -100.85, bearing_deg=90)
  posted = db.lookup(lat + 0.1, -100.85, bearing_deg=90)
  low = db.lookup(lat + 0.2, -100.85, bearing_deg=90)
  assert est.estimated and abs(est.speed_limit_ms - 65 * CV.MPH_TO_MS) < 0.01
  assert not posted.estimated and abs(posted.speed_limit_ms - 80 * CV.MPH_TO_MS) < 0.01
  assert abs(low.speed_limit_ms - 45 * CV.MPH_TO_MS) < 0.01
