"""Refresh installed OSM speed limits from live Overpass around the vehicle.

Settings → NAP → Map Speed Limit → Refresh maps (offroad, Wi-Fi) overlays
OpenStreetMap highway ways within 100 miles (~160.9 km) of a GNSS / last-GPS
fix onto the already-installed US sqlite. Tagged numeric maxspeed is
authoritative. In Minnesota, unmarked fillable highways get Minn. Stat. 169.14
estimates (never uploaded to OSM) so a refresh does not wipe pack fills.
It does not download a published pack and never replaces the US file with
only the local extract.

OpenStreetMap data is ODbL: © OpenStreetMap contributors.
https://www.openstreetmap.org/copyright
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sqlite3
from dataclasses import dataclass
from datetime import UTC, datetime
from typing import Literal

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.db_paths import default_db_path
from openpilot.selfdrive.mapd.fetch_maps import (
  cleanup_incomplete,
  download_maps,
  installed_db_summary,
  sqlite_ok,
  sqlite_present,
  staging_dir,
)
from openpilot.selfdrive.mapd.gps_fix import (
  REFRESH_GNSS_WAIT_S,
  REFRESH_GPS_MAX_ACC_M,
  is_plausible_lat_lon,
  last_gps_from_params,
  load_last_gps_position,
  persist_last_gps_if_possible,
  read_live_gnss,
)
from openpilot.selfdrive.mapd.maps_manifest import LICENSE, LICENSE_URL
from openpilot.selfdrive.mapd.mn_statutory import FILL_NOTES, FILLABLE_HIGHWAY_REGEX as MN_FILLABLE_REGEX, bbox_intersects_minnesota
from openpilot.selfdrive.mapd.us_statutory import FILLABLE_HIGHWAY_REGEX as US_FILLABLE_REGEX, is_fill_source
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB
from openpilot.selfdrive.mapd.roundabout_map import ROLE_RING, rb_rows_from_overpass
from openpilot.selfdrive.mapd.overpass import (
  OVERPASS_URL,
  bbox_from_center,
  fetch_overpass,
  ways_from_overpass,
)

# Bounding box covering 100 miles (~160.9 km) around the vehicle. See docs-nap/map-speed.md.
REFRESH_RADIUS_MILES = 100.0
REFRESH_RADIUS_KM = REFRESH_RADIUS_MILES * CV.MPH_TO_KPH

NO_GPS_MESSAGE = (
  "No GPS location after waiting for a satellite fix. "
  + "Start openpilot onroad until the GPS icon/fix is up for about a minute, then retry Refresh maps. "
  + "The car's location is required -- maps will not guess a city."
)

_UNSET = object()

OVERPASS_HTTP_TIMEOUT_S = 360.0
OVERPASS_QUERY_TIMEOUT_S = 300


class RefreshMapsError(RuntimeError):
  pass


class NoGpsError(RefreshMapsError):
  def __init__(self, message: str = NO_GPS_MESSAGE):
    super().__init__(message)


@dataclass(frozen=True)
class RefreshLocation:
  lat: float
  lon: float
  source: Literal["gnss", "last_gps", "cli"]


def _p(msg: str) -> None:
  print(msg, flush=True)


def bbox_intersects(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> bool:
  """True when two south,west,north,east boxes overlap (inclusive)."""
  a_s, a_w, a_n, a_e = a
  b_s, b_w, b_n, b_e = b
  return a_n >= b_s and a_s <= b_n and a_e >= b_w and a_w <= b_e


def merge_decision_for_way(
  way_min_lat: float, way_max_lat: float, way_min_lon: float, way_max_lon: float,
  refresh_bbox: tuple[float, float, float, float],
) -> Literal["replace", "keep"]:
  """'replace' if the way's stored bbox intersects the refresh bbox, else 'keep'."""
  way = (way_min_lat, way_min_lon, way_max_lat, way_max_lon)
  return "replace" if bbox_intersects(way, refresh_bbox) else "keep"


def resolve_refresh_location(
  *,
  lat: float | None = None,
  lon: float | None = None,
  live_fix: tuple[float, float] | None = None,
  last_gps_raw: object | None = None,
) -> RefreshLocation:
  """Prefer CLI, then a current GNSS fix, then LastGPSPosition. Never guess a city."""
  if lat is not None and lon is not None:
    if not is_plausible_lat_lon(lat, lon):
      raise NoGpsError()
    return RefreshLocation(lat=float(lat), lon=float(lon), source="cli")
  if live_fix is not None:
    glat, glon = float(live_fix[0]), float(live_fix[1])
    if is_plausible_lat_lon(glat, glon):
      return RefreshLocation(lat=glat, lon=glon, source="gnss")
  stored = load_last_gps_position(raw=last_gps_raw)
  if stored is not None:
    return RefreshLocation(lat=stored[0], lon=stored[1], source="last_gps")
  raise NoGpsError()


def merge_ways_into_db(
  db_path: str,
  ways: list[dict],
  bbox: tuple[float, float, float, float],
  extra_meta: dict | None = None,
  rb_rows: list[dict] | None = None,
) -> tuple[int, int]:
  """Delete ways intersecting bbox, insert incoming way_ids. Returns (deleted, inserted).

  rb_rows (roundabout rings + approaches, roundabout_map.roundabout_rows) replace
  the rb_ways in the same bbox. None leaves rb_ways untouched (older callers).
  """
  con = sqlite3.connect(db_path)
  try:
    deleted = OsmSpeedLimitDB.delete_ways_intersecting_bbox(con, *bbox)
    inserted = 0
    for w in ways:
      OsmSpeedLimitDB.insert_way(
        con, w["way_id"], w["name"], w["highway"], w["maxspeed_ms"], w["coords"],
        junction=w.get("junction") or "", estimated=bool(w.get("estimated")),
      )
      inserted += 1
    OsmSpeedLimitDB.recount_ways(con)
    if rb_rows is not None:
      OsmSpeedLimitDB.ensure_rb_tables(con)
      OsmSpeedLimitDB.delete_rb_ways_intersecting_bbox(con, *bbox)
      for row in rb_rows:
        OsmSpeedLimitDB.insert_rb_way(con, row)
      OsmSpeedLimitDB.recount_rb_ways(con)
    if extra_meta:
      con.executemany(
        "INSERT OR REPLACE INTO meta(key, value) VALUES (?, ?)",
        [(str(k), str(v)) for k, v in extra_meta.items()],
      )
    con.commit()
  except Exception:
    con.rollback()
    raise
  finally:
    con.close()
  return deleted, inserted


def _ensure_us_base(dest: str) -> None:
  if sqlite_present(dest):
    return
  _p("No maps installed yet. Downloading the US pack first (Refresh maps will then overlay 100 miles)...")
  download_maps(dest=dest)
  if not sqlite_present(dest):
    raise RefreshMapsError(
      f"US pack download did not leave a usable sqlite at {dest}. "
      + "Refresh maps will not install a 100-mile-only extract."
    )


def _assert_merge_space(dest: str) -> None:
  dest_dir = os.path.dirname(os.path.abspath(dest)) or "."
  try:
    sz = os.path.getsize(dest)
  except OSError as e:
    raise RefreshMapsError(f"Could not read installed maps at {dest}: {e}") from e
  need = sz + 32 * 1024 * 1024
  free = int(shutil.disk_usage(dest_dir).free)
  _p(f"Free on {dest_dir}: {free / (1024 * 1024):.0f} MiB (need ~{need / (1024 * 1024):.0f} MiB to merge)")
  if free < need:
    raise RefreshMapsError(
      f"Not enough free space on {dest_dir} to merge maps. "
      + f"Need ~{need / (1024 * 1024):.0f} MiB, have {free / (1024 * 1024):.0f} MiB. "
      + "Previous maps were left unchanged."
    )


def install_merged_overlay(
  dest: str,
  ways: list[dict],
  bbox: tuple[float, float, float, float],
  extra_meta: dict | None = None,
  rb_rows: list[dict] | None = None,
) -> str:
  """Copy dest on the dest filesystem, merge, then atomically replace. Dest is untouched on failure."""
  dest = os.path.abspath(dest)
  if not sqlite_present(dest):
    raise RefreshMapsError(
      f"No US sqlite at {dest} to merge into. Download US Maps first. "
      + "Refresh maps will not replace the US file with a 100-mile extract."
    )
  _assert_merge_space(dest)
  cleanup_incomplete(dest)
  stage = staging_dir(dest)
  os.makedirs(stage, exist_ok=True)
  work = os.path.join(stage, "speed_limits.merge.sqlite")
  try:
    _p("Merging live OSM ways into a copy of the installed US maps...")
    shutil.copy2(dest, work)
    deleted, inserted = merge_ways_into_db(work, ways, bbox, extra_meta, rb_rows=rb_rows)
    _p(f"Replaced {deleted} ways in the 100-mile box; inserted {inserted} Overpass ways.")
    if rb_rows is not None:
      rings = sum(1 for r in rb_rows if r.get("role") == ROLE_RING)
      _p(f"Roundabouts: {rings} ring ways, {len(rb_rows) - rings} approach pieces.")
    if not sqlite_ok(work):
      raise RefreshMapsError("Merged sqlite failed to open. Previous maps were left unchanged.")
    _p("Installing...")
    os.replace(work, dest)
  except Exception:
    try:
      if os.path.isfile(work):
        os.remove(work)
    except OSError:
      pass
    raise
  finally:
    try:
      if os.path.isdir(stage) and not os.listdir(stage):
        os.rmdir(stage)
    except OSError:
      pass
  return dest


def _resolve_live_fix(
  lat: float | None,
  lon: float | None,
  live_fix: object,
) -> tuple[float, float] | None:
  if live_fix is not _UNSET:
    return live_fix if isinstance(live_fix, tuple) else None
  if lat is not None and lon is not None:
    return None
  _p(f"Waiting up to {REFRESH_GNSS_WAIT_S:.0f}s for a satellite fix...")
  return read_live_gnss(
    timeout_s=REFRESH_GNSS_WAIT_S,
    max_age_s=None,
    max_acc_m=REFRESH_GPS_MAX_ACC_M,
    progress=_p,
  )


def _resolve_last_gps_raw(lat: float | None, lon: float | None, last_gps_raw: object) -> object | None:
  if last_gps_raw is not _UNSET:
    return last_gps_raw
  if lat is not None and lon is not None:
    return None
  stored = last_gps_from_params()
  if stored is None:
    return None
  return {"latitude": stored[0], "longitude": stored[1]}


def refresh_local_maps(
  dest: str | None = None,
  *,
  lat: float | None = None,
  lon: float | None = None,
  radius_km: float = REFRESH_RADIUS_KM,
  live_fix: tuple[float, float] | None | object = _UNSET,
  last_gps_raw: object = _UNSET,
  payload: dict | None = None,
  overpass_url: str = OVERPASS_URL,
) -> str:
  """Query OSM around the vehicle and merge into dest. Network skipped when payload is given.

  Omit live_fix/last_gps_raw to look them up. Pass None to force a miss (tests).
  """
  dest = os.path.abspath(dest or default_db_path())
  _p(f"OpenStreetMap speed limits ({LICENSE}). (c) OpenStreetMap contributors")
  _p(LICENSE_URL)

  loc = resolve_refresh_location(
    lat=lat, lon=lon,
    live_fix=_resolve_live_fix(lat, lon, live_fix),
    last_gps_raw=_resolve_last_gps_raw(lat, lon, last_gps_raw),
  )
  if loc.source == "gnss":
    _p(f"Using current GNSS fix {loc.lat:.5f}, {loc.lon:.5f}")
    persist_last_gps_if_possible(loc.lat, loc.lon)
  elif loc.source == "last_gps":
    _p(f"No current GNSS fix. Using last stored GPS {loc.lat:.5f}, {loc.lon:.5f}")
    persist_last_gps_if_possible(loc.lat, loc.lon)
  else:
    _p(f"Using provided location {loc.lat:.5f}, {loc.lon:.5f}")
    persist_last_gps_if_possible(loc.lat, loc.lon)

  bbox = bbox_from_center(loc.lat, loc.lon, radius_km)
  # Every state: untagged drivable roads get the state default (us_statutory).
  # Minnesota keeps MN 169.14 incl. residential streets, so its query is wider.
  fill_unmarked = True
  in_mn = bbox_intersects_minnesota(bbox)
  _p(
    f"Refresh radius {REFRESH_RADIUS_MILES:.0f} miles ({radius_km:.1f} km). "
    + f"Overpass bbox south,west,north,east={bbox[0]:.4f},{bbox[1]:.4f},{bbox[2]:.4f},{bbox[3]:.4f}"
  )
  _p("Fetching drivable highways; unmarked ways get state default limits (estimated)"
     + (", MN 169.14 incl. residential." if in_mn else "."))

  _ensure_us_base(dest)

  if payload is None:
    _p("Querying OSM (Overpass). This can take several minutes; a timeout leaves the old maps in place.")
    payload = fetch_overpass(
      bbox, url=overpass_url,
      timeout_s=OVERPASS_HTTP_TIMEOUT_S,
      query_timeout=OVERPASS_QUERY_TIMEOUT_S,
      include_unmarked=fill_unmarked,
      unmarked_regex=MN_FILLABLE_REGEX if in_mn else US_FILLABLE_REGEX,
    )
  ways = ways_from_overpass(payload, fill_unmarked=fill_unmarked)
  rb_rows = rb_rows_from_overpass(payload)
  tagged_n = sum(1 for w in ways if not is_fill_source(w.get("source")))
  filled_n = len(ways) - tagged_n
  if fill_unmarked:
    _p(
      f"Received {len(ways)} highway ways from OSM "
      + f"({tagged_n} tagged maxspeed, {filled_n} estimated state-default fills)."
    )
  else:
    _p(f"Received {len(ways)} maxspeed ways from OSM.")
  if not ways:
    raise RefreshMapsError(
      "No usable highway speed limits found in OSM for this 100-mile area. "
      + "Previous maps were left unchanged."
    )

  extra_meta = {
    "local_refresh_lat": f"{loc.lat:.6f}",
    "local_refresh_lon": f"{loc.lon:.6f}",
    "local_refresh_radius_km": f"{radius_km:.3f}",
    "local_refresh_radius_miles": f"{REFRESH_RADIUS_MILES:.0f}",
    "local_refresh_bbox": ",".join(str(x) for x in bbox),
    "local_refresh_at": datetime.now(UTC).isoformat(),
    "local_refresh_fill_unmarked": "1" if fill_unmarked else "0",
  }
  if fill_unmarked:
    extra_meta["fill_source"] = "us_statutory (<ST>_default, MN_169.14)"
    extra_meta["fill_notes"] = FILL_NOTES
    extra_meta["local_refresh_tagged_ways"] = str(tagged_n)
    extra_meta["local_refresh_filled_ways"] = str(filled_n)
  extra_meta["local_refresh_rb_rows"] = str(len(rb_rows))
  install_merged_overlay(dest, ways, bbox, extra_meta, rb_rows=rb_rows)
  _p(installed_db_summary(dest))
  _p("mapd reloads this file within ~15s onroad. No reboot required.")
  return dest


def main(argv: list[str] | None = None) -> int:
  p = argparse.ArgumentParser(
    description="Refresh installed US OSM maps with live Overpass data within 100 miles of the vehicle",
  )
  p.add_argument("--lat", type=float, default=None, help="Override latitude (skips GNSS / LastGPSPosition)")
  p.add_argument("--lon", type=float, default=None, help="Override longitude")
  p.add_argument("--radius-km", type=float, default=REFRESH_RADIUS_KM,
                 help=f"Refresh radius km (default {REFRESH_RADIUS_KM:.1f} = 100 miles)")
  p.add_argument("--from-json", help="Saved Overpass JSON instead of the network")
  p.add_argument("--out", default=None, help="sqlite destination (default: /data/media/0/osm/speed_limits.sqlite)")
  p.add_argument("--overpass-url", default=OVERPASS_URL)
  args = p.parse_args(argv)

  if (args.lat is None) ^ (args.lon is None):
    p.error("provide both --lat and --lon")

  payload = None
  if args.from_json:
    with open(args.from_json, encoding="utf-8") as f:
      payload = json.load(f)

  try:
    refresh_local_maps(
      dest=args.out,
      lat=args.lat,
      lon=args.lon,
      radius_km=args.radius_km,
      payload=payload,
      overpass_url=args.overpass_url,
    )
  except Exception as e:
    _p(f"ERROR: {e}")
    return 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
