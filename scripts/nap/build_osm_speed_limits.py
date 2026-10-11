#!/usr/bin/env python3
"""Build a NAP OSM speed-limit sqlite from Geofabrik PBF / Overpass / GeoJSON.

OpenStreetMap data is ODbL: © OpenStreetMap contributors.
https://www.openstreetmap.org/copyright

This is a PC tool. Do not commit the sqlite to git. Attach the zstd file to
GitHub Release tag osm-us-speed-limits-v3 (asset speed_limits_us.sqlite.zst).

US tagged-maxspeed ways ~3.4M (Taginfo 2026-09). After polyline simplify,
expect roughly 0.8–1.5 GB sqlite and 200–500 MB zstd — one US-wide asset.

Examples:

  # After: osmium tags-filter us-latest.osm.pbf w/highway w/maxspeed -o us-ms.osm.pbf
  # (w/highway keeps every highway way, so roundabout rings and their approach
  # roads are in the filtered PBF; they go to the rb_ways table, pack v4+)
  python scripts/nap/build_osm_speed_limits.py --pbf us-ms.osm.pbf --out speed_limits_us.sqlite
  zstd -19 speed_limits_us.sqlite -o speed_limits_us.sqlite.zst

  # Overlay MN statutory fills for unmarked highways in the Benson 100-mile box
  # onto an existing US pack (does not upload guessed maxspeed to osm.org):
  python scripts/nap/build_osm_speed_limits.py --pbf minnesota-latest.osm.pbf \\
      --fill-mn-statutory --benson --merge-into speed_limits_us.sqlite \\
      --out speed_limits_us.sqlite --zst

  python scripts/nap/build_osm_speed_limits.py --from-json overpass.json --out speed_limits.sqlite
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sqlite3
import sys

from openpilot.selfdrive.mapd.local_refresh import merge_ways_into_db
from openpilot.selfdrive.mapd.maps_manifest import ATTRIBUTION, LICENSE, LICENSE_URL
from openpilot.selfdrive.mapd.mn_statutory import (
  BENSON_LAT,
  BENSON_LON,
  BENSON_RADIUS_MILES,
  FILL_NOTES,
  FILL_SOURCE,
  PlaceIndex,
  benson_fill_bbox,
  is_urban_place_tags,
)
from openpilot.selfdrive.mapd.us_statutory import fillable_highway, is_fill_source, state_index
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB
from openpilot.selfdrive.mapd.overpass import bbox_from_center, resolve_way_speed, ways_from_overpass
from openpilot.selfdrive.mapd.roundabout_map import (
  ROLE_RING,
  is_ring_tags,
  mini_roundabout_way,
  rb_rows_from_overpass,
  roundabout_rows,
)
from openpilot.selfdrive.mapd.speed_limit import parse_maxspeed



def _parse_bbox(text: str) -> tuple[float, float, float, float]:
  parts = [float(x.strip()) for x in text.split(",")]
  if len(parts) != 4:
    raise argparse.ArgumentTypeError("--bbox must be south,west,north,east")
  return parts[0], parts[1], parts[2], parts[3]


def _bbox_intersects_way(coords: list[tuple[float, float]], bbox: tuple[float, float, float, float]) -> bool:
  south, west, north, east = bbox
  lats = [c[0] for c in coords]
  lons = [c[1] for c in coords]
  return max(lats) >= south and min(lats) <= north and max(lons) >= west and min(lons) <= east


def _area_outer_rings(a) -> list[list[tuple[float, float]]]:
  rings: list[list[tuple[float, float]]] = []
  try:
    for ring in a.outer_rings():
      coords: list[tuple[float, float]] = []
      for n in ring:
        loc = n.location
        if not loc.valid():
          coords = []
          break
        coords.append((float(loc.lat), float(loc.lon)))
      if len(coords) >= 3:
        rings.append(coords)
  except Exception:
    return []
  return rings


def _places_from_pyosmium(pbf: str) -> PlaceIndex:
  try:
    import osmium
  except ImportError as e:
    raise RuntimeError(
      "Reading .osm.pbf needs pyosmium (pip install osmium) or convert with osmium-tool first."
    ) from e

  idx = PlaceIndex()

  class H(osmium.SimpleHandler):
    def area(self, a):
      tags = dict(a.tags)
      if not is_urban_place_tags(tags):
        return
      rings = _area_outer_rings(a)
      if rings:
        idx.add_rings(rings, tags.get("name") or "")

  print(f"Collecting MN city/town urban boundary polygons from {pbf}…", file=sys.stderr, flush=True)
  H().apply_file(pbf, locations=True)
  print(f"  {len(idx)} urban place polygons", file=sys.stderr, flush=True)
  return idx


def _ways_from_pyosmium(
  pbf: str,
  *,
  bbox: tuple[float, float, float, float] | None = None,
  fill_unmarked: bool = False,
  places: PlaceIndex | None = None,
  mn_only: bool = False,
) -> list[dict]:
  try:
    import osmium
  except ImportError as e:
    raise RuntimeError(
      "Reading .osm.pbf needs pyosmium (pip install osmium) or convert with osmium-tool first."
    ) from e

  out: list[dict] = []

  class H(osmium.SimpleHandler):
    def way(self, w):
      tags = dict(w.tags)
      highway = tags.get("highway") or ""
      if not highway:
        return
      tagged = parse_maxspeed(tags.get("maxspeed")) or parse_maxspeed(tags.get("maxspeed:forward"))
      if tagged is None and not (fill_unmarked and fillable_highway("MN", highway.lower())):
        # MN's class set is the widest (adds residential/alley); the state check is below.
        return
      coords = []
      try:
        for n in w.nodes:
          if not n.location.valid():
            return
          coords.append((float(n.lat), float(n.lon)))
      except osmium.InvalidLocationError:
        return
      if len(coords) < 2:
        return
      if bbox is not None and not _bbox_intersects_way(coords, bbox):
        return
      fill = fill_unmarked
      state = None
      if fill and tagged is None:
        state = state_index().state_of_way(coords)
        if mn_only and state != "MN":
          fill = False
        elif not fillable_highway(state, highway.lower()):
          return
      ms, source = resolve_way_speed(tags, coords, fill_unmarked=fill, places=places, state=state)
      if ms is None:
        return
      out.append({
        "way_id": int(w.id),
        "name": w.tags.get("name") or w.tags.get("ref") or "",
        "highway": highway,
        "maxspeed_ms": ms,
        "coords": coords,
        "source": source,
        "estimated": is_fill_source(source),
        "junction": tags.get("junction") or "",
      })

  print(f"Reading highway ways from {pbf}…", file=sys.stderr, flush=True)
  H().apply_file(pbf, locations=True, filters=[osmium.filter.KeyFilter("highway")])
  return out


def _rb_rows_from_pyosmium(pbf: str, *, bbox: tuple[float, float, float, float] | None = None) -> list[dict]:
  """Roundabout rings (any maxspeed) + approach roads touching them. Two passes over the PBF."""
  try:
    import osmium
  except ImportError as e:
    raise RuntimeError(
      "Reading .osm.pbf needs pyosmium (pip install osmium) or convert with osmium-tool first."
    ) from e

  generic: list[dict] = []
  ring_nodes: set[int] = set()

  def _coords(w) -> tuple[list[int], list[tuple[float, float]]] | None:
    nodes: list[int] = []
    coords: list[tuple[float, float]] = []
    try:
      for n in w.nodes:
        if not n.location.valid():
          return None
        nodes.append(int(n.ref))
        coords.append((float(n.lat), float(n.lon)))
    except osmium.InvalidLocationError:
      return None
    return nodes, coords

  class Rings(osmium.SimpleHandler):
    def node(self, n):
      tags = {t.k: t.v for t in n.tags}
      if tags.get("highway") != "mini_roundabout" or not n.location.valid():
        return
      lat, lon = float(n.location.lat), float(n.location.lon)
      if bbox is not None and not _bbox_intersects_way([(lat, lon)], bbox):
        return
      ring_nodes.add(int(n.id))
      generic.append(mini_roundabout_way(int(n.id), lat, lon, tags))

    def way(self, w):
      tags = {t.k: t.v for t in w.tags}
      if not is_ring_tags(tags):
        return
      got = _coords(w)
      if got is None or len(got[1]) < 2:
        return
      if bbox is not None and not _bbox_intersects_way(got[1], bbox):
        return
      ring_nodes.update(got[0])
      generic.append({"way_id": int(w.id), "tags": tags, "node_ids": got[0], "coords": got[1]})

  class Approaches(osmium.SimpleHandler):
    def way(self, w):
      tags = {t.k: t.v for t in w.tags}
      if not tags.get("highway") or is_ring_tags(tags):
        return
      if not any(int(n.ref) in ring_nodes for n in w.nodes):
        return
      got = _coords(w)
      if got is None or len(got[1]) < 2:
        return
      generic.append({"way_id": int(w.id), "tags": tags, "node_ids": got[0], "coords": got[1]})

  print(f"Reading roundabout rings from {pbf}…", file=sys.stderr, flush=True)
  # Only objects with a highway key reach Python (big speedup on large PBFs).
  Rings().apply_file(pbf, locations=True, filters=[osmium.filter.KeyFilter("highway")])
  if ring_nodes:
    print(f"Reading approach roads for {len(generic)} ring ways…", file=sys.stderr, flush=True)
    Approaches().apply_file(pbf, locations=True, filters=[osmium.filter.KeyFilter("highway")])
  return roundabout_rows(generic)


def _count_sources(ways: list[dict]) -> tuple[int, int]:
  filled = sum(1 for w in ways if is_fill_source(w.get("source")))
  tagged = len(ways) - filled
  return tagged, filled


def write_db(path: str, ways: list[dict], extra_meta: dict | None = None,
             rb_rows: list[dict] | None = None) -> int:
  con = OsmSpeedLimitDB.create(path)
  for row in rb_rows or []:
    OsmSpeedLimitDB.insert_rb_way(con, row)
  OsmSpeedLimitDB.recount_rb_ways(con)
  n = 0
  for w in ways:
    OsmSpeedLimitDB.insert_way(
      con, w["way_id"], w["name"], w["highway"], w["maxspeed_ms"], w["coords"],
      junction=w.get("junction") or "", estimated=bool(w.get("estimated")),
    )
    n += 1
    if n % 100000 == 0:
      print(f"  {n} ways…", file=sys.stderr, flush=True)
      con.commit()
  tagged, filled = _count_sources(ways)
  meta = {
    "attribution": ATTRIBUTION,
    "license": LICENSE,
    "license_url": LICENSE_URL,
    "way_count": str(n),
    "tagged_way_count": str(tagged),
    "filled_way_count": str(filled),
  }
  if rb_rows is not None:
    # Pack v4: rb_ways (roundabout rings + approaches, any maxspeed). Readers
    # without rb support ignore the extra tables.
    meta["pack_schema"] = "4"
  if extra_meta:
    meta.update({str(k): str(v) for k, v in extra_meta.items()})
  con.executemany("INSERT OR REPLACE INTO meta(key, value) VALUES (?, ?)", list(meta.items()))
  con.commit()
  con.close()
  return n


def _write_zst(sqlite_path: str) -> str:
  import zstandard as zstd
  zst_path = sqlite_path + ".zst"
  cctx = zstd.ZstdCompressor(level=19)
  with open(sqlite_path, "rb") as inf, open(zst_path, "wb") as outf:
    cctx.copy_stream(inf, outf)
  return zst_path


def main(argv: list[str] | None = None) -> int:
  p = argparse.ArgumentParser(description="Build NAP OSM speed-limit sqlite (ODbL)")
  p.add_argument("--pbf", help="OSM PBF (ideally pre-filtered to highway+maxspeed)")
  p.add_argument("--from-json", help="Overpass JSON")
  p.add_argument("--out", required=True, help="Output sqlite path")
  p.add_argument("--zst", action="store_true", help="Also write <out>.zst via Python zstandard")
  p.add_argument("--bbox", type=_parse_bbox, default=None, help="south,west,north,east (limit PBF ways)")
  p.add_argument("--lat", type=float, help="Center latitude (with --lon / --radius-km)")
  p.add_argument("--lon", type=float, help="Center longitude")
  p.add_argument("--radius-km", type=float, default=None)
  p.add_argument("--benson", action="store_true",
                 help="Use the ~100 mile Benson, MN bbox clipped to Minnesota")
  p.add_argument("--fill-mn-statutory", action="store_true",
                 help="Fill unmarked fillable highways with Minn. Stat. 169.14 estimates, Minnesota only (NAP pack only)")
  p.add_argument("--fill-us-defaults", action="store_true",
                 help="Fill unmarked drivable highways with each state's default limit (us_statutory; MN = 169.14)")
  p.add_argument("--merge-into", help="Existing NAP sqlite to overlay the bbox into (keeps US tagged coverage)")
  p.add_argument("--no-roundabouts", action="store_true",
                 help="Skip the rb_ways table (roundabout rings + approach roads, pack v4+)")
  args = p.parse_args(argv)

  bbox = args.bbox
  if args.benson:
    bbox = benson_fill_bbox()
  elif args.lat is not None and args.lon is not None:
    radius = args.radius_km if args.radius_km is not None else BENSON_RADIUS_MILES * 1.609344
    bbox = bbox_from_center(args.lat, args.lon, radius)
  elif (args.lat is None) ^ (args.lon is None):
    p.error("provide both --lat and --lon")

  fill = bool(args.fill_mn_statutory or args.fill_us_defaults)
  mn_only = bool(args.fill_mn_statutory and not args.fill_us_defaults)
  places: PlaceIndex | None = None
  if fill and args.pbf:
    places = _places_from_pyosmium(args.pbf)

  rb_rows: list[dict] | None = None
  if args.from_json:
    with open(args.from_json, encoding="utf-8") as f:
      payload = json.load(f)
    ways = ways_from_overpass(payload, fill_unmarked=fill)
    if not args.no_roundabouts:
      rb_rows = rb_rows_from_overpass(payload)
    source = os.path.basename(args.from_json)
  elif args.pbf:
    ways = _ways_from_pyosmium(args.pbf, bbox=bbox, fill_unmarked=fill, places=places, mn_only=mn_only)
    if not args.no_roundabouts:
      rb_rows = _rb_rows_from_pyosmium(args.pbf, bbox=bbox)
    source = os.path.basename(args.pbf)
  else:
    p.error("provide --pbf or --from-json")
  if rb_rows is not None:
    rings = sum(1 for r in rb_rows if r["role"] == ROLE_RING)
    print(f"Roundabouts: {rings} ring ways, {len(rb_rows) - rings} approach pieces (no maxspeed needed)", file=sys.stderr)

  tagged, filled = _count_sources(ways)
  print(f"Prepared {len(ways)} ways ({tagged} tagged OSM maxspeed, {filled} estimated state-default fills)", file=sys.stderr)

  extra_meta: dict[str, str] = {"source": source}
  if bbox is not None:
    extra_meta["fill_bbox"] = ",".join(f"{x:.6f}" for x in bbox)
    extra_meta["fill_center"] = f"{BENSON_LAT},{BENSON_LON}" if args.benson else ""
  if fill:
    extra_meta["fill_source"] = FILL_SOURCE if mn_only else "us_statutory (<ST>_default, MN_169.14)"
    extra_meta["fill_notes"] = FILL_NOTES
    extra_meta["tagged_way_count"] = str(tagged)
    extra_meta["filled_way_count"] = str(filled)
    extra_meta["place_polygon_count"] = str(len(places) if places is not None else 0)

  if args.merge_into:
    if bbox is None:
      p.error("--merge-into requires --bbox, --benson, or --lat/--lon")
    src = os.path.abspath(args.merge_into)
    dest = os.path.abspath(args.out)
    if not os.path.isfile(src):
      p.error(f"--merge-into file not found: {src}")
    if src != dest:
      os.makedirs(os.path.dirname(dest) or ".", exist_ok=True)
      print(f"Copying {src} → {dest}", file=sys.stderr)
      shutil.copy2(src, dest)
    print(f"Overlaying {len(ways)} ways into {dest} (delete ∩ bbox, then insert)", file=sys.stderr)
    deleted, inserted = merge_ways_into_db(dest, ways, bbox, extra_meta, rb_rows=rb_rows)
    # merge_ways_into_db recounts way_count; keep tagged/filled overlay counts in meta.
    con = sqlite3.connect(dest)
    try:
      con.executemany(
        "INSERT OR REPLACE INTO meta(key, value) VALUES (?, ?)",
        [
          ("tagged_way_count", str(tagged)),
          ("filled_way_count", str(filled)),
          ("overlay_deleted", str(deleted)),
          ("overlay_inserted", str(inserted)),
        ],
      )
      con.commit()
    finally:
      con.close()
    n = inserted
    print(f"Replaced {deleted} intersecting ways; inserted {inserted} ({tagged} tagged, {filled} filled)", file=sys.stderr)
  else:
    print(f"Writing {len(ways)} ways → {args.out}", file=sys.stderr)
    n = write_db(args.out, ways, extra_meta=extra_meta, rb_rows=rb_rows)

  sz = os.path.getsize(args.out) if os.path.isfile(args.out) else 0
  print(f"Wrote {n} overlay ways, {sz / 1e6:.0f} MB sqlite (ODbL, {ATTRIBUTION})")
  if fill:
    print(FILL_NOTES, file=sys.stderr)
  if args.zst:
    zst_path = _write_zst(args.out)
    zsz = os.path.getsize(zst_path)
    print(f"Wrote {zst_path} ({zsz / 1e6:.0f} MB). Attach to GitHub Release {os.path.basename(zst_path)}.")
  if n == 0:
    return 2
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
