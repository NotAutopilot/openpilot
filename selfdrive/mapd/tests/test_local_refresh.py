"""Local Refresh maps: location, 100-mile merge, no network."""
from __future__ import annotations

import json
import os
from pathlib import Path

import pytest

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.gps_fix import (
  GPS_MAX_ACC_M,
  GPS_MAX_AGE_S,
  REFRESH_GNSS_WAIT_S,
  REFRESH_GPS_MAX_ACC_M,
  format_last_gps_position,
  gps_sample_from_sm,
  is_plausible_lat_lon,
  last_gps_from_params,
  parse_last_gps_position,
  persist_last_gps_position,
)
from openpilot.selfdrive.mapd.local_refresh import (
  NO_GPS_MESSAGE,
  REFRESH_RADIUS_KM,
  REFRESH_RADIUS_MILES,
  NoGpsError,
  RefreshMapsError,
  bbox_intersects,
  install_merged_overlay,
  main as refresh_main,
  merge_decision_for_way,
  merge_ways_into_db,
  refresh_local_maps,
  resolve_refresh_location,
)
from openpilot.selfdrive.mapd.mn_statutory import BENSON_LAT, BENSON_LON, FILL_SOURCE
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB
from openpilot.selfdrive.mapd.overpass import bbox_from_center, overpass_query, ways_from_overpass


SF = (37.7749, -122.4194)
NYC = (40.7128, -74.0060)


def _insert(con, way_id: int, lat: float, lon: float, mph: float, name: str = "Rd") -> None:
  OsmSpeedLimitDB.insert_way(
    con, way_id, name, "primary", mph * CV.MPH_TO_MS,
    [(lat, lon - 0.001), (lat, lon + 0.001)],
  )


def _tiny_us(path: str) -> None:
  con = OsmSpeedLimitDB.create(path)
  _insert(con, 1, SF[0], SF[1], 35, "Market")
  _insert(con, 2, NYC[0], NYC[1], 25, "Broadway")
  OsmSpeedLimitDB.recount_ways(con)
  con.commit()
  con.close()


def _overpass_way(way_id: int, lat: float, lon: float, mph: int, name: str) -> dict:
  return {
    "type": "way",
    "id": way_id,
    "tags": {"highway": "primary", "name": name, "maxspeed": f"{mph} mph"},
    "geometry": [
      {"lat": lat, "lon": lon - 0.001},
      {"lat": lat, "lon": lon + 0.001},
    ],
  }


def test_refresh_radius_is_100_miles():
  assert REFRESH_RADIUS_MILES == 100.0
  assert abs(REFRESH_RADIUS_KM - 160.9344) < 1e-6
  south, west, north, east = bbox_from_center(SF[0], SF[1], REFRESH_RADIUS_KM)
  # ~160.9 km → ~1.45 deg latitude
  assert abs((north - south) / 2 - REFRESH_RADIUS_KM / 111.0) < 1e-9
  assert south < SF[0] < north
  assert west < SF[1] < east
  nyc_box = bbox_from_center(NYC[0], NYC[1], 1.0)
  sf_box = bbox_from_center(SF[0], SF[1], REFRESH_RADIUS_KM)
  assert not bbox_intersects(sf_box, nyc_box)


def test_merge_decision_replaces_in_radius_keeps_outside():
  bbox = bbox_from_center(SF[0], SF[1], REFRESH_RADIUS_KM)
  assert merge_decision_for_way(SF[0], SF[0], SF[1], SF[1], bbox) == "replace"
  assert merge_decision_for_way(NYC[0], NYC[0], NYC[1], NYC[1], bbox) == "keep"
  # Corner of the bbox still intersects → replace (bbox contains the 100-mile circle).
  assert merge_decision_for_way(bbox[2], bbox[2], bbox[3], bbox[3], bbox) == "replace"


def test_no_gps_fails_without_guessing_a_city():
  with pytest.raises(NoGpsError, match="will not guess a city") as exc:
    resolve_refresh_location(live_fix=None, last_gps_raw=None)
  msg = str(exc.value)
  assert msg == NO_GPS_MESSAGE
  assert "onroad" in msg.lower()
  assert "about a minute" in msg.lower()
  assert "guess a city" in msg.lower()
  assert "drive once" not in msg.lower()
  for city in ("San Francisco", "New York", "Los Angeles", "Chicago", "default city"):
    assert city.lower() not in msg.lower()


def test_invalid_last_gps_is_no_gps():
  assert parse_last_gps_position("") is None
  assert parse_last_gps_position("not-json") is None
  assert parse_last_gps_position({"latitude": 0, "longitude": 0}) is None
  assert parse_last_gps_position({"latitude": 91, "longitude": 0}) is None
  with pytest.raises(NoGpsError):
    resolve_refresh_location(live_fix=(0.0, 0.0), last_gps_raw="{}")


def test_live_gnss_preferred_over_last_gps():
  last = format_last_gps_position(*NYC)
  loc = resolve_refresh_location(live_fix=SF, last_gps_raw=last)
  assert loc.source == "gnss"
  assert abs(loc.lat - SF[0]) < 1e-9
  loc2 = resolve_refresh_location(live_fix=None, last_gps_raw=last)
  assert loc2.source == "last_gps"
  assert abs(loc2.lon - NYC[1]) < 1e-9
  loc3 = resolve_refresh_location(lat=SF[0], lon=SF[1], live_fix=NYC, last_gps_raw=last)
  assert loc3.source == "cli"
  assert abs(loc3.lat - SF[0]) < 1e-9


def test_is_plausible_rejects_null_island():
  assert not is_plausible_lat_lon(0.0, 0.0)
  assert is_plausible_lat_lon(*SF)


class _SM:
  def __init__(self, msgs, frames, times):
    self._msgs = msgs
    self.recv_frame = frames
    self.recv_time = times

  def __getitem__(self, key):
    return self._msgs[key]


def _gps_msg(lat, lon, acc=8.0, speed=0.0, bearing=0.0):
  from types import SimpleNamespace
  return SimpleNamespace(latitude=lat, longitude=lon, horizontalAccuracy=acc, speed=speed, bearingDeg=bearing)


def test_gps_sample_prefers_external_and_rejects_stale():
  ext = _gps_msg(*SF)
  qcom = _gps_msg(*NYC, bearing=90.0)
  sm = _SM(
    {"gpsLocationExternal": ext, "gpsLocation": qcom},
    {"gpsLocationExternal": 1, "gpsLocation": 1},
    {"gpsLocationExternal": 100.0, "gpsLocation": 100.0},
  )
  lat, lon, _b, ok = gps_sample_from_sm(sm, now=100.5)
  assert ok and abs(lat - SF[0]) < 1e-9 and abs(lon - SF[1]) < 1e-9

  sm.recv_time["gpsLocationExternal"] = 90.0  # stale for mapd (2.5s)
  lat, lon, _b, ok = gps_sample_from_sm(sm, now=100.5)
  assert ok and abs(lat - NYC[0]) < 1e-9

  null = _gps_msg(0.0, 0.0, acc=1.0)
  sm._msgs["gpsLocationExternal"] = null
  sm._msgs["gpsLocation"] = null
  sm.recv_time["gpsLocationExternal"] = 100.0
  sm.recv_time["gpsLocation"] = 100.0
  _lat, _lon, _b, ok = gps_sample_from_sm(sm, now=100.5)
  assert not ok


def test_mapd_gps_sample_rejects_stale_and_coarse_accuracy():
  assert GPS_MAX_AGE_S == 2.5
  assert GPS_MAX_ACC_M == 50.0
  sm = _SM(
    {"gpsLocationExternal": _gps_msg(*SF, acc=8.0), "gpsLocation": _gps_msg(*SF, acc=8.0)},
    {"gpsLocationExternal": 1, "gpsLocation": 1},
    {"gpsLocationExternal": 90.0, "gpsLocation": 90.0},
  )
  _lat, _lon, _b, ok = gps_sample_from_sm(sm, now=100.5)
  assert not ok

  sm.recv_time["gpsLocationExternal"] = 100.0
  sm.recv_time["gpsLocation"] = 100.0
  sm._msgs["gpsLocationExternal"] = _gps_msg(*SF, acc=150.0)
  sm._msgs["gpsLocation"] = _gps_msg(*SF, acc=150.0)
  _lat, _lon, _b, ok = gps_sample_from_sm(sm, now=100.5)
  assert not ok


def test_refresh_gps_sample_accepts_stale_and_yard_accuracy():
  assert REFRESH_GNSS_WAIT_S == 10.0
  assert REFRESH_GPS_MAX_ACC_M == 200.0
  sm = _SM(
    {"gpsLocationExternal": _gps_msg(*SF, acc=150.0), "gpsLocation": _gps_msg(0.0, 0.0)},
    {"gpsLocationExternal": 1, "gpsLocation": 1},
    {"gpsLocationExternal": 90.0, "gpsLocation": 90.0},
  )
  lat, lon, _b, ok = gps_sample_from_sm(
    sm, now=100.5, max_age_s=None, max_acc_m=REFRESH_GPS_MAX_ACC_M,
  )
  assert ok and abs(lat - SF[0]) < 1e-9 and abs(lon - SF[1]) < 1e-9

  sm._msgs["gpsLocationExternal"] = _gps_msg(0.0, 0.0, acc=8.0)
  sm._msgs["gpsLocation"] = _gps_msg(0.0, 0.0, acc=8.0)
  _lat, _lon, _b, ok = gps_sample_from_sm(
    sm, now=100.5, max_age_s=None, max_acc_m=REFRESH_GPS_MAX_ACC_M,
  )
  assert not ok


def test_persist_last_gps_and_param_file_fallback(tmp_path, monkeypatch):
  stored: dict[str, str] = {}

  class FakeParams:
    def put(self, key, value):
      stored[key] = value

    def get(self, key):
      return stored.get(key)

  persist_last_gps_position(FakeParams(), *SF)
  pos = parse_last_gps_position(stored["LastGPSPosition"])
  assert pos is not None
  assert abs(pos[0] - SF[0]) < 1e-9 and abs(pos[1] - SF[1]) < 1e-9

  p = tmp_path / "LastGPSPosition"
  p.write_text(format_last_gps_position(*NYC), encoding="utf-8")
  import sys
  import types
  import openpilot.selfdrive.mapd.gps_fix as gf
  monkeypatch.setattr(gf, "_PARAM_LAST_GPS_PATHS", (str(p),))

  fake_params = types.ModuleType("openpilot.common.params")

  class EmptyParams:
    def get(self, _key):
      return None

  fake_params.Params = EmptyParams
  monkeypatch.setitem(sys.modules, "openpilot.common.params", fake_params)
  pos = last_gps_from_params()
  assert pos is not None
  assert abs(pos[0] - NYC[0]) < 1e-9
  assert abs(pos[1] - NYC[1]) < 1e-9


def test_merge_replaces_way_ids_in_radius_keeps_rest(tmp_path):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  os.makedirs(os.path.dirname(dest), exist_ok=True)
  _tiny_us(dest)
  bbox = bbox_from_center(SF[0], SF[1], REFRESH_RADIUS_KM)
  payload = {"elements": [
    _overpass_way(1, SF[0], SF[1], 45, "Market"),  # updated limit
    _overpass_way(3, SF[0] + 0.01, SF[1], 30, "New"),  # new local way
  ]}
  ways = ways_from_overpass(payload)
  deleted, inserted = merge_ways_into_db(dest, ways, bbox)
  assert deleted >= 1
  assert inserted == 2

  db = OsmSpeedLimitDB(dest)
  assert db.open()
  sf = db.lookup(SF[0], SF[1], bearing_deg=90.0)
  assert sf is not None and sf.way_id == 1
  assert abs(sf.speed_limit_ms - 45 * CV.MPH_TO_MS) < 0.2
  new = db.lookup(SF[0] + 0.01, SF[1], bearing_deg=90.0)
  assert new is not None and new.way_id == 3
  nyc = db.lookup(NYC[0], NYC[1], bearing_deg=90.0)
  assert nyc is not None and nyc.way_id == 2
  assert abs(nyc.speed_limit_ms - 25 * CV.MPH_TO_MS) < 0.2
  db.close()
  assert OsmSpeedLimitDB.way_count(dest) == 3


def test_refresh_no_gps_leaves_sqlite_and_skips_overpass(tmp_path, monkeypatch):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  os.makedirs(os.path.dirname(dest), exist_ok=True)
  _tiny_us(dest)
  prev = Path(dest).read_bytes()

  def boom(*_a, **_k):
    raise AssertionError("must not query OSM without a location")

  import openpilot.selfdrive.mapd.local_refresh as lr
  monkeypatch.setattr(lr, "fetch_overpass", boom)
  monkeypatch.setattr(lr, "download_maps", boom)

  with pytest.raises(NoGpsError, match="will not guess a city"):
    refresh_local_maps(dest=dest, live_fix=None, last_gps_raw=None)
  assert Path(dest).read_bytes() == prev


def test_cli_no_gps_prints_error(tmp_path, monkeypatch, capsys):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  os.makedirs(os.path.dirname(dest), exist_ok=True)
  _tiny_us(dest)
  prev = Path(dest).read_bytes()
  import openpilot.selfdrive.mapd.local_refresh as lr
  monkeypatch.setattr(lr, "read_live_gnss", lambda **_k: None)
  monkeypatch.setattr(lr, "last_gps_from_params", lambda: None)
  rc = refresh_main(["--out", dest])
  assert rc == 1
  out = capsys.readouterr().out
  assert "Waiting up to 10s for a satellite fix" in out
  assert "(c) OpenStreetMap contributors" in out
  assert "ERROR:" in out
  assert "guess a city" in out
  assert "onroad" in out
  assert "about a minute" in out
  assert Path(dest).read_bytes() == prev


def test_overpass_timeout_keeps_previous_sqlite(tmp_path, monkeypatch):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  os.makedirs(os.path.dirname(dest), exist_ok=True)
  _tiny_us(dest)
  prev = Path(dest).read_bytes()

  import openpilot.selfdrive.mapd.local_refresh as lr

  def timeout(*_a, **_k):
    raise RuntimeError("OSM Overpass timed out. Previous maps were left unchanged. You can retry Refresh maps.")

  monkeypatch.setattr(lr, "fetch_overpass", timeout)
  with pytest.raises(RuntimeError, match="timed out"):
    refresh_local_maps(dest=dest, live_fix=SF, last_gps_raw=None)
  assert Path(dest).read_bytes() == prev


def test_merge_exception_keeps_dest(tmp_path, monkeypatch):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  os.makedirs(os.path.dirname(dest), exist_ok=True)
  _tiny_us(dest)
  prev = Path(dest).read_bytes()
  bbox = bbox_from_center(SF[0], SF[1], REFRESH_RADIUS_KM)
  ways = ways_from_overpass({"elements": [_overpass_way(1, SF[0], SF[1], 45, "Market")]})

  import openpilot.selfdrive.mapd.local_refresh as lr

  def boom(*_a, **_k):
    raise RuntimeError("disk full during merge")

  monkeypatch.setattr(lr, "merge_ways_into_db", boom)
  with pytest.raises(RuntimeError, match="disk full"):
    install_merged_overlay(dest, ways, bbox)
  assert Path(dest).read_bytes() == prev
  assert os.path.isfile(dest)
  work = tmp_path / "osm" / ".download" / "speed_limits.merge.sqlite"
  assert not work.is_file()


def test_empty_overpass_does_not_wipe_radius(tmp_path):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  os.makedirs(os.path.dirname(dest), exist_ok=True)
  _tiny_us(dest)
  prev = Path(dest).read_bytes()
  with pytest.raises(RefreshMapsError, match="No usable highway speed limits"):
    refresh_local_maps(dest=dest, live_fix=SF, last_gps_raw=None, payload={"elements": []})
  assert Path(dest).read_bytes() == prev


def test_missing_sqlite_downloads_us_then_merges(tmp_path, monkeypatch):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  payload = {"elements": [_overpass_way(9, SF[0], SF[1], 40, "Oak")]}

  import openpilot.selfdrive.mapd.local_refresh as lr
  called = {"n": 0}

  def fake_download(dest=None, **_k):
    called["n"] += 1
    path = dest or str(tmp_path / "missing")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    _tiny_us(path)
    return path

  def no_overpass(*_a, **_k):
    raise AssertionError("payload was provided; must not hit network")

  monkeypatch.setattr(lr, "download_maps", fake_download)
  monkeypatch.setattr(lr, "fetch_overpass", no_overpass)
  refresh_local_maps(dest=dest, live_fix=SF, last_gps_raw=None, payload=payload)
  assert called["n"] == 1
  db = OsmSpeedLimitDB(dest)
  assert db.open()
  oak = db.lookup(SF[0], SF[1], bearing_deg=90.0)
  assert oak is not None and oak.way_id == 9
  nyc = db.lookup(NYC[0], NYC[1], bearing_deg=90.0)
  assert nyc is not None and nyc.way_id == 2
  db.close()


def test_refresh_never_installs_extract_only_when_download_fails(tmp_path, monkeypatch):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  import openpilot.selfdrive.mapd.local_refresh as lr

  def fail_download(*_a, **_k):
    raise RuntimeError("GitHub unreachable")

  monkeypatch.setattr(lr, "download_maps", fail_download)
  with pytest.raises(RuntimeError, match="GitHub unreachable"):
    refresh_local_maps(
      dest=dest, live_fix=SF, last_gps_raw=None,
      payload={"elements": [_overpass_way(1, SF[0], SF[1], 40, "Oak")]},
    )
  assert not os.path.isfile(dest)


def test_refresh_cli_from_json(tmp_path):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  os.makedirs(os.path.dirname(dest), exist_ok=True)
  _tiny_us(dest)
  payload_path = tmp_path / "overpass.json"
  payload_path.write_text(json.dumps({"elements": [
    _overpass_way(1, SF[0], SF[1], 55, "Market"),
  ]}), encoding="utf-8")
  rc = refresh_main([
    "--lat", str(SF[0]), "--lon", str(SF[1]),
    "--from-json", str(payload_path),
    "--out", dest,
  ])
  assert rc == 0
  db = OsmSpeedLimitDB(dest)
  assert db.open()
  m = db.lookup(SF[0], SF[1], bearing_deg=90.0)
  assert m is not None
  assert abs(m.speed_limit_ms - 55 * CV.MPH_TO_MS) < 0.2
  db.close()


def test_docs_and_ui_say_100_miles_not_published_pack():
  root = Path(__file__).resolve().parents[3]
  docs = (root / "docs-nap/map-speed.md").read_text()
  assert "100 miles" in docs
  assert "~160.9 km" in docs
  tici = (root / "selfdrive/ui/layouts/settings/map_speed.py").read_text()
  mici = (root / "selfdrive/ui/mici/layouts/settings/map_speed.py").read_text()
  nap = (root / "selfdrive/ui/layouts/settings/nap.py").read_text()
  instructions = (root / "selfdrive/ui/layouts/settings/nap_content.py").read_text()
  assert '"Refresh maps"' in tici
  assert "Check for map updates" not in tici
  assert "refresh maps" in mici
  assert "check for map updates" not in mici
  assert "scripts.nap.refresh_osm_maps" in nap
  assert "scripts.nap.refresh_osm_maps" in mici
  assert tici.index("_all_items.append(self._refresh_btn)") < tici.index("_all_items.append(self._download_btn)")
  mici_widgets = mici.split("self._scroller.add_widgets", 1)[1]
  assert mici_widgets.index("refresh_maps_btn") < mici_widgets.index("download_maps_btn")
  assert "100 miles" in instructions
  assert "guess a city" in instructions
  assert "10s" in instructions
  assert "onroad" in instructions
  assert "(c) OpenStreetMap" in instructions
  assert "10s" in docs
  assert "200 m" in docs or "200m" in docs


def test_map_scripts_do_not_reboot_on_exit():
  from scripts.nap.script_lifecycle import script_reboots_on_exit
  assert not script_reboots_on_exit("scripts.nap.refresh_osm_maps")
  assert not script_reboots_on_exit("scripts.nap.fetch_osm_maps")
  assert script_reboots_on_exit("scripts.nap.calibrate_pedal")
  assert script_reboots_on_exit("scripts.nap.flash_epas")
  assert script_reboots_on_exit("scripts.nap.extract_epas")
  assert script_reboots_on_exit("scripts.nap.restore_epas")
  root = Path(__file__).resolve().parents[3]
  for rel in ("scripts/nap/run_script.py", "scripts/nap/run_script_mici.py"):
    src = (root / rel).read_text()
    assert "script_reboots_on_exit" in src
    assert "HARDWARE.reboot()" in src
  nap = (root / "selfdrive/ui/layouts/settings/nap.py").read_text()
  mici_launch = (root / "selfdrive/ui/mici/layouts/settings/nap_script.py").read_text()
  assert "ScriptRunner" in nap
  assert "gui_app.push_widget" in nap
  assert "_MapScriptOverlay" in mici_launch
  assert "script_reboots_on_exit" in nap
  assert "script_reboots_on_exit" in mici_launch


def test_refresh_stages_merge_beside_dest_not_tmp(tmp_path):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  os.makedirs(os.path.dirname(dest), exist_ok=True)
  _tiny_us(dest)
  payload = {"elements": [_overpass_way(1, SF[0], SF[1], 40, "Market")]}
  refresh_local_maps(dest=dest, live_fix=SF, last_gps_raw=None, payload=payload)
  from openpilot.selfdrive.mapd.fetch_maps import STAGING_DIRNAME, staging_dir
  stage = staging_dir(dest)
  assert os.path.dirname(stage) == os.path.dirname(dest)
  assert os.path.basename(stage) == STAGING_DIRNAME
  assert not os.path.isfile(os.path.join(stage, "speed_limits.merge.sqlite"))


def _unmarked_way(way_id: int, lat: float, lon: float, highway: str, name: str) -> dict:
  return {
    "type": "way",
    "id": way_id,
    "tags": {"highway": highway, "name": name},
    "geometry": [
      {"lat": lat, "lon": lon - 0.001},
      {"lat": lat, "lon": lon + 0.001},
    ],
  }


def test_mn_refresh_fills_unmarked_and_does_not_wipe(tmp_path):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  os.makedirs(os.path.dirname(dest), exist_ok=True)
  _tiny_us(dest)
  payload = {"elements": [
    _overpass_way(100, BENSON_LAT, BENSON_LON, 60, "US 12"),
    _unmarked_way(101, BENSON_LAT + 0.01, BENSON_LON, "residential", "14th St"),
    _unmarked_way(102, BENSON_LAT + 0.02, BENSON_LON, "service", "Alley svc"),
  ]}
  refresh_local_maps(dest=dest, live_fix=(BENSON_LAT, BENSON_LON), last_gps_raw=None, payload=payload)
  db = OsmSpeedLimitDB(dest)
  assert db.open()
  tagged = db.lookup(BENSON_LAT, BENSON_LON, bearing_deg=90.0)
  assert tagged is not None and tagged.way_id == 100
  assert abs(tagged.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.2
  filled = db.lookup(BENSON_LAT + 0.01, BENSON_LON, bearing_deg=90.0)
  assert filled is not None and filled.way_id == 101
  assert abs(filled.speed_limit_ms - 30 * CV.MPH_TO_MS) < 0.2
  assert filled.estimated and not tagged.estimated
  skipped = db.lookup(BENSON_LAT + 0.02, BENSON_LON, bearing_deg=90.0)
  assert skipped is None
  nyc = db.lookup(NYC[0], NYC[1], bearing_deg=90.0)
  assert nyc is not None and nyc.way_id == 2
  db.close()
  assert FILL_SOURCE in (OsmSpeedLimitDB.meta_get(dest, "fill_source") or "")


def test_sf_refresh_does_not_fill_unmarked_residential(tmp_path):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  os.makedirs(os.path.dirname(dest), exist_ok=True)
  _tiny_us(dest)
  payload = {"elements": [
    _overpass_way(1, SF[0], SF[1], 40, "Market"),
    _unmarked_way(50, SF[0] + 0.01, SF[1], "residential", "Oak"),
  ]}
  refresh_local_maps(dest=dest, live_fix=SF, last_gps_raw=None, payload=payload)
  db = OsmSpeedLimitDB(dest)
  assert db.open()
  oak = db.lookup(SF[0] + 0.01, SF[1], bearing_deg=90.0)
  assert oak is None
  market = db.lookup(SF[0], SF[1], bearing_deg=90.0)
  assert market is not None and abs(market.speed_limit_ms - 40 * CV.MPH_TO_MS) < 0.2
  db.close()


def test_mn_refresh_overpass_query_includes_unmarked(monkeypatch):
  seen: dict = {}

  def fake_fetch(bbox, url="unused", **kwargs):
    seen["include_unmarked"] = kwargs.get("include_unmarked")
    seen["query"] = overpass_query(*bbox, include_unmarked=bool(kwargs.get("include_unmarked")))
    return {"elements": [_overpass_way(1, BENSON_LAT, BENSON_LON, 55, "US 12")]}

  import openpilot.selfdrive.mapd.local_refresh as lr
  monkeypatch.setattr(lr, "fetch_overpass", fake_fetch)
  dest = "/tmp/does-not-matter-mn-refresh"
  # download_maps / merge would run; stub install after fetch
  monkeypatch.setattr(lr, "_ensure_us_base", lambda dest: None)
  monkeypatch.setattr(lr, "install_merged_overlay", lambda *a, **k: dest)
  monkeypatch.setattr(lr, "installed_db_summary", lambda dest: "ok")
  monkeypatch.setattr(lr, "sqlite_present", lambda dest: True)
  refresh_local_maps(dest=dest, live_fix=(BENSON_LAT, BENSON_LON), last_gps_raw=None)
  assert seen.get("include_unmarked") is True
  assert '["maxspeed"]' not in seen["query"]
  assert "residential" in seen["query"]

