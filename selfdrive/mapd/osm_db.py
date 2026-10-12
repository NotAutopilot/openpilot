"""Offline OSM speed-limit SQLite (R-tree) for comma 3X.

Schema is NAP-owned. Data is OpenStreetMap (ODbL). Query path is GPS → nearest
heading-aligned way. nextSpeedLimit follows that matched way (bearing + class),
not a nearby off-route fill. A lower limit that only lasts MIN_ZONE_LENGTH_M
(~250 ft) along heading is ignored (cross-street bleed / intersection stub).
A short first OSM piece of a longer same-limit town zone is kept. Reverse-
digitized along-route ways (Δheading ≈ 180°) stay on-route. Tagged OSM
maxspeed is authoritative; Minnesota packs may include statutory estimates
for unmarked highways (never uploaded to OSM).
"""
from __future__ import annotations

import math
import os
import sqlite3
import struct
import time
from dataclasses import dataclass, replace

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.constants import (
  HEADING_ALIGN_DEG,
  LOOKAHEAD_M,
  LOOKAHEAD_MAX_M,
  MAX_MATCH_DISTANCE_M,
  MIN_ZONE_LENGTH_M,
  SEARCH_PAD_DEG,
  osm_sign_lead_m,
)
from openpilot.selfdrive.mapd.roundabout import (
  RB_FUNNEL_M,
  rb_decel_onset_m,
  RB_FUNNEL_MIN_M,
  RB_ON_WAY_M,
  RB_SEARCH_PAD_DEG,
  RoundaboutHint,
  junction_is_roundabout,
  roundabout_suppressed_for_highway,
  way_is_roundabout,
)
from openpilot.selfdrive.mapd.roundabout_map import ROLE_RING, RingGeometry
from openpilot.selfdrive.mapd.roundabout_map import ring_geometry as fit_ring_geometry

# Ring geometry search box around the ego / hinted ring (~165 m).
RB_GEOM_PAD_DEG = 0.0015

# Functional class for route continuity. Residential fills next to a trunk
# must not count as the "next limit ahead" even when a heading ray hits them.
_HIGHWAY_RANK = {
  "motorway": 0, "motorway_link": 0,
  "trunk": 1, "trunk_link": 1,
  "primary": 2, "primary_link": 2,
  "secondary": 3, "secondary_link": 3,
  "tertiary": 4, "tertiary_link": 4,
  "unclassified": 5,
  "residential": 6, "living_street": 6, "alley": 7,
}

# Current-way hysteresis. Nearest-wins will hand a motorway to a parallel
# frontage once GPS drifts onto it. Keep the previous way while it is still
# close and aligned; a clearly closer candidate must repeat for several
# lookups. A much slower minor road also has to persist in time or distance.
STICK_KEEP_M = 35.0
STICK_SWITCH_CLOSER_M = 9.0
STICK_SWITCH_LOOKUPS = 3
STICK_PERSIST_S = 5.0
STICK_PERSIST_M = 150.0
_PLAUSIBLE_OVER_MS = 20.0 * CV.MPH_TO_MS
_MAJOR_HIGHWAY = {
  "motorway", "motorway_link",
  "trunk", "trunk_link",
  "primary", "primary_link",
}
_MINOR_HIGHWAY = {
  "tertiary", "tertiary_link",
  "residential", "living_street", "service", "unclassified",
}

# Ways a car does not drive on. A tagged `maxspeed` here (5 mph on a golf-cart
# `highway=path`) must never become the posted limit, even when it is the only
# stored way within MAX_MATCH_DISTANCE_M of the real, untagged road. Applied in
# `_best_match` (so `_next_limit` and the min-zone probes inherit it), NOT in
# `_candidates()` which roundabout ring geometry also uses.
_NON_ROAD_HIGHWAY = frozenset({
  "path", "footway", "cycleway", "bridleway", "steps", "pedestrian", "track",
})


def _is_non_road_highway(highway: str | None) -> bool:
  return (highway or "").strip().lower() in _NON_ROAD_HIGHWAY

EARTH_R = 6371000.0
_COORDS_HDR = struct.Struct("<I")
_COORD_F64 = struct.Struct("<dd")
_COORD_F32 = struct.Struct("<ff")
# Drop vertices closer than this when inserting (keeps US extract under a Release asset).
SIMPLIFY_TOL_M = 12.0


@dataclass(frozen=True)
class SpeedLimitMatch:
  speed_limit_ms: float
  way_id: int
  road_name: str
  highway: str
  distance_m: float
  next_speed_limit_ms: float = 0.0
  next_distance_m: float = 0.0
  coords: tuple[tuple[float, float], ...] = ()
  # True when the limit is a NAP state-default estimate (no posted maxspeed in OSM).
  estimated: bool = False


def _row_estimated(row) -> bool:
  try:
    return bool(row["estimated"])
  except (IndexError, KeyError):
    return False


# Estimated (state-default) limits never exceed this on the car, so a road wrongly
# classed rural (e.g. I-94 through Bismarck at ND's 80) cannot raise MAX past 65.
# Posted OSM limits are not capped.
ESTIMATED_CAP_MS = 65 * 0.44704


def _row_limit_ms(row) -> float:
  v = float(row["maxspeed_ms"] or 0.0)
  return min(v, ESTIMATED_CAP_MS) if _row_estimated(row) else v


def _pack_coords(coords: list[tuple[float, float]]) -> bytes:
  # float32 is ~1 m at US longitudes — enough for MAX_MATCH_DISTANCE_M.
  buf = bytearray(_COORDS_HDR.pack(len(coords)))
  for lat, lon in coords:
    buf += _COORD_F32.pack(float(lat), float(lon))
  return bytes(buf)


def _unpack_coords(blob: bytes) -> list[tuple[float, float]]:
  n = _COORDS_HDR.unpack_from(blob)[0]
  rest = len(blob) - _COORDS_HDR.size
  if rest == n * _COORD_F32.size:
    fmt, step = _COORD_F32, _COORD_F32.size
  elif rest == n * _COORD_F64.size:
    fmt, step = _COORD_F64, _COORD_F64.size
  else:
    return []
  coords = []
  off = _COORDS_HDR.size
  for _ in range(n):
    lat, lon = fmt.unpack_from(blob, off)
    coords.append((float(lat), float(lon)))
    off += step
  return coords


def simplify_coords(coords: list[tuple[float, float]], tol_m: float = SIMPLIFY_TOL_M) -> list[tuple[float, float]]:
  """Douglas-Peucker in local meters. Always keeps endpoints."""
  if len(coords) <= 2 or tol_m <= 0:
    return coords
  lat0, lon0 = coords[0]

  def xy(p: tuple[float, float]) -> tuple[float, float]:
    return _local_xy(p[0], p[1], lat0, lon0)

  pts = [xy(c) for c in coords]
  keep = [False] * len(coords)
  keep[0] = keep[-1] = True
  stack = [(0, len(coords) - 1)]
  while stack:
    i, j = stack.pop()
    ax, ay = pts[i]
    bx, by = pts[j]
    abx, aby = bx - ax, by - ay
    ab2 = abx * abx + aby * aby
    max_d = -1.0
    max_k = -1
    for k in range(i + 1, j):
      px, py = pts[k]
      if ab2 < 1e-6:
        d = math.hypot(px - ax, py - ay)
      else:
        t = max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / ab2))
        d = math.hypot(ax + t * abx - px, ay + t * aby - py)
      if d > max_d:
        max_d = d
        max_k = k
    if max_k >= 0 and max_d > tol_m:
      keep[max_k] = True
      stack.append((i, max_k))
      stack.append((max_k, j))
  return [c for c, k in zip(coords, keep, strict=True) if k]


def _wrap_heading_delta(a: float, b: float) -> float:
  d = abs(a - b) % 360.0
  return min(d, 360.0 - d)


def _heading_aligned(bearing_deg: float | None, seg_heading: float | None) -> bool:
  """True if the segment is along travel, including reverse-digitized OSM ways.

  OSM ways are often drawn opposite travel (Δ ≈ 180°). Use min(Δ, 180−Δ) so a
  reverse-digitized trunk continuation stays on-route. Cross streets at ~90°
  still fail (min(90, 90) = 90 > HEADING_ALIGN_DEG).
  """
  if bearing_deg is None or seg_heading is None:
    return True
  delta = _wrap_heading_delta(bearing_deg, seg_heading)
  return min(delta, 180.0 - delta) <= HEADING_ALIGN_DEG


def _bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
  phi1, phi2 = math.radians(lat1), math.radians(lat2)
  dlon = math.radians(lon2 - lon1)
  x = math.sin(dlon) * math.cos(phi2)
  y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlon)
  return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def _offset_point(lat: float, lon: float, bearing_deg: float, dist_m: float) -> tuple[float, float]:
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


def _local_xy(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
  x = math.radians(lon - lon0) * math.cos(math.radians(lat0)) * EARTH_R
  y = math.radians(lat - lat0) * EARTH_R
  return x, y


def _point_to_polyline_m(lat: float, lon: float, coords: list[tuple[float, float]]) -> tuple[float, float | None]:
  """Return (min distance m, heading of closest segment or None)."""
  if len(coords) < 2:
    return 1e9, None
  best = 1e9
  best_heading = None
  px, py = 0.0, 0.0  # query is origin
  for i in range(len(coords) - 1):
    ax, ay = _local_xy(coords[i][0], coords[i][1], lat, lon)
    bx, by = _local_xy(coords[i + 1][0], coords[i + 1][1], lat, lon)
    abx, aby = bx - ax, by - ay
    ab2 = abx * abx + aby * aby
    if ab2 < 1e-6:
      t = 0.0
    else:
      t = max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / ab2))
    dx, dy = ax + t * abx - px, ay + t * aby - py
    dist = math.hypot(dx, dy)
    if dist < best:
      best = dist
      best_heading = _bearing_deg(coords[i][0], coords[i][1], coords[i + 1][0], coords[i + 1][1])
  return best, best_heading


def _continues_route(
  bearing_deg: float | None, seg_heading: float | None,
  from_name: str, from_highway: str, to_name: str, to_highway: str,
) -> bool:
  """True if a candidate is the road ahead, not an off-route cross street / fill.

  Bearing must stay within HEADING_ALIGN_DEG of the segment or its reverse
  (OSM often digitizes the town street opposite travel). Same name may change
  class (US 12 trunk → primary in town) and is sufficient, not required —
  class continuity + bidirectional heading is enough (US 12 → Atlantic).
  A major way must not jump onto an unnamed residential / living_street fill
  just because it is geometrically closer.
  """
  if not _heading_aligned(bearing_deg, seg_heading):
    return False
  fn = (from_name or "").strip().lower()
  tn = (to_name or "").strip().lower()
  if fn and tn and fn == tn:
    return True
  fr = _HIGHWAY_RANK.get((from_highway or "").strip().lower(), 5)
  tr = _HIGHWAY_RANK.get((to_highway or "").strip().lower(), 5)
  # trunk/primary/secondary → residential 30 fill (v3 MN statutory).
  if fr <= 3 and tr >= 6:
    return False
  return True


def _same_named_road(a: str, b: str) -> bool:
  an = (a or "").strip().lower()
  bn = (b or "").strip().lower()
  return bool(an) and an == bn


def _major_to_minor(prev_highway: str, cand_highway: str) -> bool:
  prev = (prev_highway or "").strip().lower()
  cand = (cand_highway or "").strip().lower()
  return prev in _MAJOR_HIGHWAY and cand in _MINOR_HIGHWAY


def _implausible_minor(v_ego_ms: float, cand: SpeedLimitMatch, prev_highway: str) -> bool:
  """Ego is far above a lower-class candidate. A parallel frontage or a bad rematch."""
  if not _major_to_minor(prev_highway, cand.highway):
    return False
  return float(v_ego_ms) > float(cand.speed_limit_ms) + _PLAUSIBLE_OVER_MS


def _meters_between(a: tuple[float, float], b: tuple[float, float]) -> float:
  x, y = _local_xy(b[0], b[1], a[0], a[1])
  return math.hypot(x, y)


def _seg_len_m(a: tuple[float, float], b: tuple[float, float]) -> float:
  x, y = _local_xy(b[0], b[1], a[0], a[1])
  return math.hypot(x, y)


def _row_junction(row: sqlite3.Row) -> str:
  try:
    return str(row["junction"] or "")
  except (IndexError, KeyError):
    return ""


def _closest_xy_on_polyline(
  lat: float, lon: float, coords: list[tuple[float, float]],
) -> tuple[float, float, float]:
  """Local east/north of the closest point on the polyline, plus distance m."""
  if len(coords) < 2:
    return 0.0, 0.0, 1e9
  best = 1e9
  best_x = 0.0
  best_y = 0.0
  px, py = 0.0, 0.0
  for i in range(len(coords) - 1):
    ax, ay = _local_xy(coords[i][0], coords[i][1], lat, lon)
    bx, by = _local_xy(coords[i + 1][0], coords[i + 1][1], lat, lon)
    abx, aby = bx - ax, by - ay
    ab2 = abx * abx + aby * aby
    t = 0.0 if ab2 < 1e-6 else max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / ab2))
    dx, dy = ax + t * abx - px, ay + t * aby - py
    dist = math.hypot(dx, dy)
    if dist < best:
      best = dist
      best_x, best_y = ax + t * abx, ay + t * aby
  return best_x, best_y, best


def _along_heading_m(east_m: float, north_m: float, bearing_deg: float) -> float:
  br = math.radians(bearing_deg)
  return east_m * math.sin(br) + north_m * math.cos(br)


def _remaining_ahead(
  lat: float, lon: float, bearing_deg: float, coords: list[tuple[float, float]] | tuple[tuple[float, float], ...],
) -> tuple[float, tuple[float, float], float] | None:
  """Meters along the way from the closest point to the ahead endpoint.

  Ahead is the digitization direction if GPS heading agrees (<90°), else reverse.
  Returns (remaining_m, end_latlon, end_heading_deg) or None.
  """
  if len(coords) < 2:
    return None
  best = 1e12
  best_i = 0
  best_t = 0.0
  best_hdg = 0.0
  px, py = 0.0, 0.0
  for i in range(len(coords) - 1):
    ax, ay = _local_xy(coords[i][0], coords[i][1], lat, lon)
    bx, by = _local_xy(coords[i + 1][0], coords[i + 1][1], lat, lon)
    abx, aby = bx - ax, by - ay
    ab2 = abx * abx + aby * aby
    t = 0.0 if ab2 < 1e-6 else max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / ab2))
    dist = math.hypot(ax + t * abx - px, ay + t * aby - py)
    if dist < best:
      best = dist
      best_i = i
      best_t = t
      best_hdg = _bearing_deg(coords[i][0], coords[i][1], coords[i + 1][0], coords[i + 1][1])
  s = 0.0
  for i in range(best_i):
    s += _seg_len_m(coords[i], coords[i + 1])
  s += best_t * _seg_len_m(coords[best_i], coords[best_i + 1])
  total = 0.0
  for i in range(len(coords) - 1):
    total += _seg_len_m(coords[i], coords[i + 1])
  with_digitization = _wrap_heading_delta(bearing_deg, best_hdg) <= 90.0
  if with_digitization:
    remaining = max(0.0, total - s)
    end = coords[-1]
    end_hdg = _bearing_deg(coords[-2][0], coords[-2][1], coords[-1][0], coords[-1][1])
  else:
    remaining = max(0.0, s)
    end = coords[0]
    end_hdg = (_bearing_deg(coords[0][0], coords[0][1], coords[1][0], coords[1][1]) + 180.0) % 360.0
  return remaining, (float(end[0]), float(end[1])), float(end_hdg)


class OsmSpeedLimitDB:
  def __init__(self, path: str):
    self.path = path
    self._con: sqlite3.Connection | None = None
    self._rb_ok = False
    self._reset_way_memory()

  @property
  def loaded(self) -> bool:
    return self._con is not None

  def open(self) -> bool:
    if self._con is not None:
      return True
    if not self.path or not os.path.isfile(self.path):
      return False
    con = sqlite3.connect(self.path, check_same_thread=False)
    con.row_factory = sqlite3.Row
    try:
      con.execute("SELECT way_id FROM ways LIMIT 1")
    except sqlite3.Error:
      con.close()
      return False
    try:
      OsmSpeedLimitDB.ensure_junction_column(con)
    except sqlite3.Error:
      pass
    try:
      self._rb_ok = OsmSpeedLimitDB.has_rb_tables(con)
    except sqlite3.Error:
      self._rb_ok = False
    self._con = con
    return True

  def close(self) -> None:
    if self._con is not None:
      self._con.close()
      self._con = None
    self._reset_way_memory()

  @staticmethod
  def create(path: str) -> sqlite3.Connection:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    if os.path.exists(path):
      os.remove(path)
    con = sqlite3.connect(path)
    con.executescript("""
      CREATE TABLE meta (
        key TEXT PRIMARY KEY,
        value TEXT NOT NULL
      );
      CREATE TABLE ways (
        way_id INTEGER PRIMARY KEY,
        name TEXT,
        highway TEXT,
        maxspeed_ms REAL NOT NULL,
        min_lat REAL NOT NULL,
        max_lat REAL NOT NULL,
        min_lon REAL NOT NULL,
        max_lon REAL NOT NULL,
        coords BLOB NOT NULL,
        junction TEXT,
        estimated INTEGER NOT NULL DEFAULT 0
      );
      CREATE VIRTUAL TABLE ways_rtree USING rtree(
        way_id, min_lat, max_lat, min_lon, max_lon
      );
    """)
    OsmSpeedLimitDB.ensure_rb_tables(con)
    con.executemany(
      "INSERT INTO meta(key, value) VALUES (?, ?)",
      [
        ("attribution", "© OpenStreetMap contributors"),
        ("license", "ODbL"),
        ("license_url", "https://www.openstreetmap.org/copyright"),
        ("format", "nap-osm-speedlimit-v1"),
      ],
    )
    con.commit()
    return con

  @staticmethod
  def meta_get(path: str, key: str) -> str | None:
    if not path or not os.path.isfile(path):
      return None
    try:
      con = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
      try:
        row = con.execute("SELECT value FROM meta WHERE key = ?", (key,)).fetchone()
        return str(row[0]) if row else None
      finally:
        con.close()
    except sqlite3.Error:
      return None

  @staticmethod
  def way_count(path: str) -> int | None:
    raw = OsmSpeedLimitDB.meta_get(path, "way_count")
    if raw is None:
      return None
    try:
      return int(raw)
    except ValueError:
      return None

  @staticmethod
  def ensure_junction_column(con: sqlite3.Connection) -> None:
    """Published v1 packs omit junction. Refresh / new DBs store the OSM tag."""
    cols = {str(r[1]) for r in con.execute("PRAGMA table_info(ways)")}
    if "junction" not in cols:
      con.execute("ALTER TABLE ways ADD COLUMN junction TEXT")
    if "estimated" not in cols:
      # 1 = NAP state-default estimate (MN_169.14 / <ST>_default), 0 = OSM maxspeed.
      con.execute("ALTER TABLE ways ADD COLUMN estimated INTEGER NOT NULL DEFAULT 0")

  @staticmethod
  def insert_way(con: sqlite3.Connection, way_id: int, name: str, highway: str,
                 maxspeed_ms: float, coords: list[tuple[float, float]],
                 junction: str = "", estimated: bool = False) -> None:
    OsmSpeedLimitDB.ensure_junction_column(con)
    coords = simplify_coords(coords)
    if len(coords) < 2 or maxspeed_ms <= 0:
      return
    lats = [c[0] for c in coords]
    lons = [c[1] for c in coords]
    min_lat, max_lat = min(lats), max(lats)
    min_lon, max_lon = min(lons), max(lons)
    blob = _pack_coords(coords)
    con.execute(
      "INSERT OR REPLACE INTO ways(way_id, name, highway, maxspeed_ms, min_lat, max_lat, min_lon, max_lon, coords, junction, estimated) "
      + "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
      (int(way_id), name or "", highway or "", float(maxspeed_ms), min_lat, max_lat, min_lon, max_lon, blob, junction or "",
       1 if estimated else 0),
    )
    con.execute(
      "INSERT OR REPLACE INTO ways_rtree(way_id, min_lat, max_lat, min_lon, max_lon) VALUES (?, ?, ?, ?, ?)",
      (int(way_id), min_lat, max_lat, min_lon, max_lon),
    )

  @staticmethod
  def delete_ways_intersecting_bbox(con: sqlite3.Connection, south: float, west: float, north: float, east: float) -> int:
    """Remove ways whose stored bbox intersects south,west,north,east. Returns deleted count."""
    rows = con.execute(
      "SELECT way_id FROM ways_rtree WHERE max_lat >= ? AND min_lat <= ? AND max_lon >= ? AND min_lon <= ?",
      (south, north, west, east),
    ).fetchall()
    ids = [int(r[0]) for r in rows]
    if not ids:
      return 0
    for i in range(0, len(ids), 500):
      chunk = ids[i:i + 500]
      q = ",".join("?" * len(chunk))
      con.execute(f"DELETE FROM ways WHERE way_id IN ({q})", chunk)
      con.execute(f"DELETE FROM ways_rtree WHERE way_id IN ({q})", chunk)
    return len(ids)

  @staticmethod
  def recount_ways(con: sqlite3.Connection) -> int:
    n = int(con.execute("SELECT COUNT(*) FROM ways").fetchone()[0])
    con.execute("INSERT OR REPLACE INTO meta(key, value) VALUES (?, ?)", ("way_count", str(n)))
    return n

  @staticmethod
  def has_rb_tables(con: sqlite3.Connection) -> bool:
    row = con.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='rb_ways'").fetchone()
    return row is not None

  @staticmethod
  def ensure_rb_tables(con: sqlite3.Connection) -> None:
    """Roundabout rings + approach roads (no maxspeed needed). Pack v4+ / Refresh maps.

    Kept apart from `ways` so speed-limit matching never sees a 0 m/s way.
    """
    con.executescript("""
      CREATE TABLE IF NOT EXISTS rb_ways (
        id INTEGER PRIMARY KEY,
        way_id INTEGER NOT NULL,
        piece INTEGER NOT NULL DEFAULT 0,
        role TEXT NOT NULL,
        name TEXT,
        highway TEXT,
        junction TEXT,
        oneway INTEGER NOT NULL DEFAULT 0,
        lanes INTEGER NOT NULL DEFAULT 0,
        maxspeed_ms REAL NOT NULL DEFAULT 0,
        min_lat REAL NOT NULL,
        max_lat REAL NOT NULL,
        min_lon REAL NOT NULL,
        max_lon REAL NOT NULL,
        coords BLOB NOT NULL,
        UNIQUE(way_id, piece)
      );
      CREATE VIRTUAL TABLE IF NOT EXISTS rb_ways_rtree USING rtree(
        id, min_lat, max_lat, min_lon, max_lon
      );
    """)

  @staticmethod
  def insert_rb_way(con: sqlite3.Connection, row: dict) -> None:
    """Insert one roundabout_map.roundabout_rows() row (replaces the same way_id/piece)."""
    OsmSpeedLimitDB.ensure_rb_tables(con)
    coords = simplify_coords([(float(a), float(b)) for a, b in row["coords"]], tol_m=1.0)
    if len(coords) < 2:
      return
    way_id, piece = int(row["way_id"]), int(row.get("piece", 0) or 0)
    old = con.execute("SELECT id FROM rb_ways WHERE way_id = ? AND piece = ?", (way_id, piece)).fetchone()
    if old is not None:
      con.execute("DELETE FROM rb_ways WHERE id = ?", (int(old[0]),))
      con.execute("DELETE FROM rb_ways_rtree WHERE id = ?", (int(old[0]),))
    lats = [c[0] for c in coords]
    lons = [c[1] for c in coords]
    cur = con.execute(
      "INSERT INTO rb_ways(way_id, piece, role, name, highway, junction, oneway, lanes, maxspeed_ms, "
      + "min_lat, max_lat, min_lon, max_lon, coords) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
      (way_id, piece, str(row["role"]), row.get("name") or "", row.get("highway") or "",
       row.get("junction") or "", int(row.get("oneway") or 0), int(row.get("lanes") or 0),
       float(row.get("maxspeed_ms") or 0.0), min(lats), max(lats), min(lons), max(lons), _pack_coords(coords)),
    )
    con.execute(
      "INSERT INTO rb_ways_rtree(id, min_lat, max_lat, min_lon, max_lon) VALUES (?, ?, ?, ?, ?)",
      (int(cur.lastrowid), min(lats), max(lats), min(lons), max(lons)),
    )

  @staticmethod
  def delete_rb_ways_intersecting_bbox(con: sqlite3.Connection, south: float, west: float,
                                       north: float, east: float) -> int:
    if not OsmSpeedLimitDB.has_rb_tables(con):
      return 0
    ids = [int(r[0]) for r in con.execute(
      "SELECT id FROM rb_ways_rtree WHERE max_lat >= ? AND min_lat <= ? AND max_lon >= ? AND min_lon <= ?",
      (south, north, west, east),
    )]
    for i in range(0, len(ids), 500):
      chunk = ids[i:i + 500]
      q = ",".join("?" * len(chunk))
      con.execute(f"DELETE FROM rb_ways WHERE id IN ({q})", chunk)
      con.execute(f"DELETE FROM rb_ways_rtree WHERE id IN ({q})", chunk)
    return len(ids)

  @staticmethod
  def recount_rb_ways(con: sqlite3.Connection) -> int:
    if not OsmSpeedLimitDB.has_rb_tables(con):
      return 0
    n = int(con.execute("SELECT COUNT(*) FROM rb_ways WHERE role = 'ring'").fetchone()[0])
    con.execute("INSERT OR REPLACE INTO meta(key, value) VALUES (?, ?)", ("rb_ring_way_count", str(n)))
    return n

  def _rb_candidates(self, lat: float, lon: float, pad_deg: float, role: str | None = None) -> list[sqlite3.Row]:
    if self._con is None or not self._rb_ok:
      return []
    ids = [
      int(r[0]) for r in self._con.execute(
        "SELECT id FROM rb_ways_rtree WHERE max_lat >= ? AND min_lat <= ? AND max_lon >= ? AND min_lon <= ?",
        (lat - pad_deg, lat + pad_deg, lon - pad_deg, lon + pad_deg),
      )
    ]
    if not ids:
      return []
    q = f"SELECT * FROM rb_ways WHERE id IN ({','.join('?' * len(ids))})"
    rows = list(self._con.execute(q, ids))
    if role is not None:
      rows = [r for r in rows if str(r["role"]) == role]
    return rows

  def _way_bbox(self, way_id: int) -> tuple[float, float, float, float] | None:
    """(min_lat, max_lat, min_lon, max_lon) of a ring way from rb_ways or the speed ways table."""
    if self._con is None:
      return None
    rows = []
    if self._rb_ok:
      rows = list(self._con.execute("SELECT min_lat, max_lat, min_lon, max_lon FROM rb_ways WHERE way_id = ?", (way_id,)))
    if not rows:
      rows = list(self._con.execute("SELECT min_lat, max_lat, min_lon, max_lon FROM ways WHERE way_id = ?", (way_id,)))
    if not rows:
      return None
    return (min(float(r[0]) for r in rows), max(float(r[1]) for r in rows),
            min(float(r[2]) for r in rows), max(float(r[3]) for r in rows))

  def ring_geometry(self, way_id: int, lat: float, lon: float) -> RingGeometry | None:
    """Fit the ring containing way_id (rings from rb_ways and junction-tagged speed ways)."""
    if self._con is None:
      return None
    # Search around the hinted way itself (the hint fires ~150-200 m before the ring), not the car.
    pad = RB_GEOM_PAD_DEG
    box = self._way_bbox(int(way_id))
    if box is not None:
      lat, lon = 0.5 * (box[0] + box[1]), 0.5 * (box[2] + box[3])
      pad += 0.5 * max(box[1] - box[0], box[3] - box[2])
    rings: dict[int, dict] = {}
    for row in self._candidates(float(lat), float(lon), pad_deg=pad):
      if junction_is_roundabout(_row_junction(row)):
        rings[int(row["way_id"])] = {
          "way_id": int(row["way_id"]), "coords": _unpack_coords(row["coords"]), "lanes": 0,
          "maxspeed_ms": _row_limit_ms(row), "junction": _row_junction(row),
        }
    approaches = []
    for row in self._rb_candidates(float(lat), float(lon), pad):
      d = {
        "way_id": int(row["way_id"]), "coords": _unpack_coords(row["coords"]), "lanes": int(row["lanes"] or 0),
        "maxspeed_ms": float(row["maxspeed_ms"] or 0.0), "junction": str(row["junction"] or ""),
        "oneway": int(row["oneway"] or 0),
      }
      if str(row["role"]) == ROLE_RING:
        prev = rings.get(d["way_id"])
        if prev is not None:
          d["maxspeed_ms"] = max(d["maxspeed_ms"], prev["maxspeed_ms"])
        rings[d["way_id"]] = d
      else:
        approaches.append(d)
    if int(way_id) not in rings:
      return None
    return fit_ring_geometry(list(rings.values()), approaches, seed_id=int(way_id))

  def _candidates(self, lat: float, lon: float, pad_deg: float | None = None) -> list[sqlite3.Row]:
    if self._con is None:
      return []
    pad = SEARCH_PAD_DEG if pad_deg is None else float(pad_deg)
    ids = [
      int(r[0]) for r in self._con.execute(
        "SELECT way_id FROM ways_rtree WHERE max_lat >= ? AND min_lat <= ? AND max_lon >= ? AND min_lon <= ?",
        (lat - pad, lat + pad, lon - pad, lon + pad),
      )
    ]
    if not ids:
      return []
    q = f"SELECT * FROM ways WHERE way_id IN ({','.join('?' * len(ids))})"
    return list(self._con.execute(q, ids))

  def _best_match(self, lat: float, lon: float, bearing_deg: float | None,
                  along_route: SpeedLimitMatch | None = None) -> SpeedLimitMatch | None:
    best: SpeedLimitMatch | None = None
    best_score = 1e12
    for row in self._candidates(lat, lon):
      highway = row["highway"] or ""
      if _is_non_road_highway(highway):
        continue
      coords = _unpack_coords(row["coords"])
      dist, seg_heading = _point_to_polyline_m(lat, lon, coords)
      if dist > MAX_MATCH_DISTANCE_M:
        continue
      name = row["name"] or ""
      heading_pen = 0.0
      if along_route is not None:
        if not _continues_route(
          bearing_deg, seg_heading,
          along_route.road_name, along_route.highway, name, highway,
        ):
          continue
      elif bearing_deg is not None and seg_heading is not None:
        delta = _wrap_heading_delta(bearing_deg, seg_heading)
        # Opposite-direction ways: treat as a large penalty so we pick the
        # carriageway we are actually on when a dual carriageway is nearby.
        if delta > 180.0 - HEADING_ALIGN_DEG:
          heading_pen = 40.0
        elif delta > HEADING_ALIGN_DEG:
          heading_pen = 15.0
      score = dist + heading_pen
      if score < best_score:
        best_score = score
        best = SpeedLimitMatch(
          speed_limit_ms=_row_limit_ms(row),
          way_id=int(row["way_id"]),
          road_name=name,
          highway=highway,
          distance_m=float(dist),
          coords=tuple((float(c[0]), float(c[1])) for c in coords),
          estimated=_row_estimated(row),
        )
    return best

  def _same_limit_run_m(
    self, lat: float, lon: float, bearing_deg: float, match: SpeedLimitMatch,
    along_route: SpeedLimitMatch | None = None,
  ) -> float:
    """Meters of contiguous same maxspeed along the matched road from here.

    Walks onto the next on-route same-limit way so a short first OSM piece of
    a real town zone counts with the rest of the chain. Does not follow a
    heading-misaligned fill (N–S cross street while traveling E–W).
    """
    if not match.coords:
      return 0.0
    _dist, seg_hdg = _point_to_polyline_m(lat, lon, list(match.coords))
    if seg_hdg is not None and not _heading_aligned(bearing_deg, seg_hdg):
      return 0.0
    target_ms = float(match.speed_limit_ms)
    traveled = 0.0
    cur_lat, cur_lon = float(lat), float(lon)
    cur_brg = float(bearing_deg)
    cur_coords: list[tuple[float, float]] | tuple[tuple[float, float], ...] = match.coords
    route = along_route if along_route is not None else match
    seen = {int(match.way_id)}
    for _ in range(24):
      if not cur_coords:
        break
      ahead_info = _remaining_ahead(cur_lat, cur_lon, cur_brg, cur_coords)
      if ahead_info is None:
        break
      rem, end_ll, end_hdg = ahead_info
      traveled += rem
      if traveled >= MIN_ZONE_LENGTH_M:
        return traveled
      nxt = None
      plat, plon = end_ll[0], end_ll[1]
      for step_m in (12.0, 25.0, 40.0):
        plat, plon = _offset_point(end_ll[0], end_ll[1], end_hdg, step_m)
        cand = self._best_match(plat, plon, end_hdg, along_route=route)
        if cand is not None and int(cand.way_id) not in seen:
          nxt = cand
          break
      if nxt is None or abs(nxt.speed_limit_ms - target_ms) > 0.3 or not nxt.coords:
        break
      seen.add(int(nxt.way_id))
      cur_coords = nxt.coords
      cur_lat, cur_lon = plat, plon
      cur_brg = end_hdg
    return traveled

  def _limit_persists_min_zone(
    self, lat: float, lon: float, bearing_deg: float, match: SpeedLimitMatch,
    along_route: SpeedLimitMatch | None = None,
  ) -> bool:
    """True if this limit is a real along-route zone, not a cross-street blip.

    Prefer "returns to prior" over first-way length:
    - Probe MIN_ZONE_LENGTH_M along heading. Same limit → keep. A different
      limit (typically back on the highway) → ignore (stub / bleed).
    - If the probe misses (gap, or past a short first OSM piece), walk the
      contiguous same-limit on-route chain. Keep if that run is ≥ MIN_ZONE.

    Remaining-along-way alone is not enough: a long N–S fill has lots of
    remaining. The heading-ray probe and the bidirectional heading guard on
    the chain walk keep those out.
    """
    plat, plon = _offset_point(lat, lon, bearing_deg, MIN_ZONE_LENGTH_M)
    ahead = self._best_match(plat, plon, bearing_deg, along_route=along_route)
    if ahead is not None:
      return abs(ahead.speed_limit_ms - match.speed_limit_ms) <= 0.3
    return self._same_limit_run_m(lat, lon, bearing_deg, match, along_route) >= MIN_ZONE_LENGTH_M

  def _stabilize_match(
    self, lat: float, lon: float, bearing_deg: float | None, match: SpeedLimitMatch | None,
  ) -> SpeedLimitMatch | None:
    """Drop a matched lower limit that only lasts a stub along heading.

    OSM/GPS at a crossroads can snap onto a side street for ~a car length.
    Prefer the road that is still there MIN_ZONE_LENGTH_M ahead. Never snap
    posted down to a lower ahead zone — that decrease eases via next.
    """
    if match is None or bearing_deg is None:
      return match
    if self._limit_persists_min_zone(lat, lon, bearing_deg, match):
      return match
    plat, plon = _offset_point(lat, lon, bearing_deg, MIN_ZONE_LENGTH_M)
    ahead = self._best_match(plat, plon, bearing_deg)
    if ahead is None or int(ahead.way_id) == int(match.way_id):
      return match
    if ahead.speed_limit_ms + 0.3 < match.speed_limit_ms:
      return match
    # Same named road changing speed is a real zone end, not cross-street
    # bleed. Do not raise posted early; the 1.5 s GNSS offset does that.
    match_name = (match.road_name or "").strip().lower()
    ahead_name = (ahead.road_name or "").strip().lower()
    if match_name and match_name == ahead_name and ahead.speed_limit_ms > match.speed_limit_ms + 0.3:
      return match
    if not self._limit_persists_min_zone(plat, plon, bearing_deg, ahead):
      return match
    stable = self._best_match(lat, lon, bearing_deg, along_route=ahead)
    if stable is not None and abs(stable.speed_limit_ms - ahead.speed_limit_ms) <= 0.3:
      return stable
    return ahead

  def _geodesic_next(self, lat: float, lon: float, bearing_deg: float,
                     match: SpeedLimitMatch) -> tuple[float, float] | None:
    """Heading-ray probes after the matched way ends (OSM gap). Route-bearing only."""
    prev_same_d = 0.0
    current_ms = float(match.speed_limit_ms)
    for d in LOOKAHEAD_M:
      alat, alon = _offset_point(lat, lon, bearing_deg, d)
      ahead = self._best_match(alat, alon, bearing_deg, along_route=match)
      if ahead is None:
        continue
      if abs(ahead.speed_limit_ms - current_ms) <= 0.3:
        prev_same_d = d
        continue
      if not self._limit_persists_min_zone(alat, alon, bearing_deg, ahead, along_route=match):
        continue
      next_dist = 0.5 * (prev_same_d + d) if prev_same_d > 0 else d
      return float(ahead.speed_limit_ms), float(next_dist)
    return None

  def _along_way_next(self, lat: float, lon: float, bearing_deg: float,
                      match: SpeedLimitMatch) -> tuple[tuple[float, float] | None, bool]:
    """Follow the matched way to its end, then the next on-route way.

    Returns (picked, resolved). resolved means the next LOOKAHEAD_MAX_M along
    this road is known (a different limit, or the same limit continuing) — do
    not fall back to a heading ray that can hit off-route fills.

    Catches a short intermediate limit (US 12 60→50 before 30) that geodesic
    40 m heading probes skip when the road curves or the 50 way is short.
    """
    if not match.coords:
      return None, False
    current_ms = float(match.speed_limit_ms)
    traveled = 0.0
    cur_lat, cur_lon = float(lat), float(lon)
    cur_brg = float(bearing_deg)
    cur_coords: list[tuple[float, float]] | tuple[tuple[float, float], ...] = match.coords
    seen = {int(match.way_id)}
    plat, plon = cur_lat, cur_lon
    for _ in range(24):
      ahead_info = _remaining_ahead(cur_lat, cur_lon, cur_brg, cur_coords)
      if ahead_info is None:
        return None, False
      rem, end_ll, end_hdg = ahead_info
      if traveled + rem > LOOKAHEAD_MAX_M + 1.0:
        return None, True
      nxt = None
      # Step past the end; 12 m can still match the way we just left
      # (MAX_MATCH_DISTANCE_M=35).
      for step_m in (12.0, 25.0, 40.0):
        plat, plon = _offset_point(end_ll[0], end_ll[1], end_hdg, step_m)
        cand = self._best_match(plat, plon, end_hdg, along_route=match)
        if cand is not None and int(cand.way_id) not in seen:
          nxt = cand
          break
      if nxt is None:
        return None, False
      seen.add(int(nxt.way_id))
      if abs(nxt.speed_limit_ms - current_ms) > 0.3:
        if self._limit_persists_min_zone(plat, plon, end_hdg, nxt, along_route=match):
          traveled += rem
          return (float(nxt.speed_limit_ms), float(traveled)), True
        # Stub / cross-street bleed: do not walk down that way. Stay on the
        # previous geometry; `seen` prevents re-picking the stub.
        continue
      traveled += rem
      if not nxt.coords:
        return None, False
      cur_coords = nxt.coords
      cur_lat, cur_lon = plat, plon
      cur_brg = end_hdg
    return None, True

  def _next_limit(self, lat: float, lon: float, bearing_deg: float,
                  match: SpeedLimitMatch) -> tuple[float, float] | None:
    along, resolved = self._along_way_next(lat, lon, bearing_deg, match)
    if along is not None:
      return along
    if resolved:
      return None
    return self._geodesic_next(lat, lon, bearing_deg, match)

  def find_roundabout(
    self, lat: float, lon: float, bearing_deg: float | None = None,
    current_highway: str | None = None,
    v_ego_ms: float | None = None,
  ) -> RoundaboutHint:
    """Upcoming / current OSM circulating ring within the speed-based funnel.

    Funnel = rb_decel_onset_m(v_ego): 140 m at town speed, ~340 m from
    55 mph, max 450 m. Once a ring is hinted it stays latched out to the
    450 m reach so slowing down (which shrinks the onset) cannot drop the
    hint mid-approach and fire an early "exit" MAX restore.

    Does not apply MIN_ZONE_LENGTH_M — a typical RB is a short along-heading
    stub and would otherwise be dropped as cross-street bleed. Sharp corners
    and signalized crossings are not closed/tagged circulating ways.
    Motorway / trunk matches suppress the hint (a residential loop beside
    the carriageway is not the road ahead).
    """
    if self._con is None:
      return RoundaboutHint()
    if current_highway is None:
      matched = self._best_match(float(lat), float(lon), bearing_deg)
      current_highway = matched.highway if matched is not None else ""
    if roundabout_suppressed_for_highway(current_highway):
      return RoundaboutHint()
    # Default SEARCH_PAD is ~110 m; 200 m westbound at 45°N needs more.
    rows = list(self._candidates(float(lat), float(lon), pad_deg=RB_SEARCH_PAD_DEG))
    seen = {int(r["way_id"]) for r in rows}
    # Pack v4 / Refresh maps: rings without maxspeed live in rb_ways.
    rows += [r for r in self._rb_candidates(float(lat), float(lon), RB_SEARCH_PAD_DEG, role=ROLE_RING)
             if int(r["way_id"]) not in seen]

    best: RoundaboutHint | None = None
    best_dist = 1e12
    for row in rows:
      coords = _unpack_coords(row["coords"])
      if not way_is_roundabout(coords, _row_junction(row), row["highway"] or ""):
        continue
      east_m, north_m, dist = _closest_xy_on_polyline(float(lat), float(lon), coords)
      way_id = int(row["way_id"])
      latched = way_id == getattr(self, "_rb_latch_way", 0)
      if dist > (RB_FUNNEL_M if latched else rb_decel_onset_m(v_ego_ms)):
        continue
      if bearing_deg is not None and dist > RB_ON_WAY_M:
        along = _along_heading_m(east_m, north_m, float(bearing_deg))
        if along < -12.0:
          continue
        # Far funnel only: skip rings well off the travel heading.
        if dist > RB_FUNNEL_MIN_M and along < (0.5 * dist):
          continue
      on_rb = dist <= RB_ON_WAY_M
      hint = RoundaboutHint(
        on_roundabout=on_rb,
        approaching=(not on_rb),
        distance_m=0.0 if on_rb else float(dist),
        speed_limit_ms=_row_limit_ms(row),
        way_id=int(row["way_id"]),
      )
      # Prefer on-ring, then nearer ahead.
      rank = (0 if on_rb else 1, dist)
      best_rank = (0 if (best and best.on_roundabout) else 1, best_dist)
      if best is None or rank < best_rank:
        best = hint
        best_dist = dist
    self._rb_latch_way = best.way_id if best is not None else 0
    return best if best is not None else RoundaboutHint()

  def _reset_way_memory(self) -> None:
    self._stuck_match: SpeedLimitMatch | None = None
    self._reset_pending()

  def _reset_pending(self) -> None:
    self._pending_way_id: int | None = None
    self._pending_lookups = 0
    self._pending_since_s: float | None = None
    self._pending_dist_m = 0.0
    self._pending_latlon: tuple[float, float] | None = None

  def _observe_switch_candidate(
    self, way_id: int, closer_m: float, lat: float, lon: float, now: float,
  ) -> tuple[int, float, float]:
    """Track one rival way. Returns (closer-lookups, elapsed_s, traveled_m)."""
    if self._pending_way_id != int(way_id):
      self._pending_way_id = int(way_id)
      self._pending_lookups = 1 if closer_m >= STICK_SWITCH_CLOSER_M else 0
      self._pending_since_s = now
      self._pending_dist_m = 0.0
      self._pending_latlon = (lat, lon)
    else:
      if self._pending_latlon is not None:
        self._pending_dist_m += _meters_between(self._pending_latlon, (lat, lon))
      self._pending_latlon = (lat, lon)
      if closer_m >= STICK_SWITCH_CLOSER_M:
        self._pending_lookups += 1
      else:
        self._pending_lookups = 0
    elapsed = 0.0 if self._pending_since_s is None else max(0.0, now - float(self._pending_since_s))
    return self._pending_lookups, elapsed, self._pending_dist_m

  def _switch_persisted(self, elapsed_s: float, traveled_m: float) -> bool:
    return float(elapsed_s) >= STICK_PERSIST_S or float(traveled_m) >= STICK_PERSIST_M

  def _current_way_is_sticky(
    self, prev_dist_m: float, heading_aligned: bool, closer_m: float, pending_lookups: int,
  ) -> bool:
    """Keep the previous way unless a candidate is clearly closer for several lookups."""
    if prev_dist_m > STICK_KEEP_M or not heading_aligned:
      return False
    if closer_m >= STICK_SWITCH_CLOSER_M and pending_lookups >= STICK_SWITCH_LOOKUPS:
      return False
    return True

  def _accept_current_way(self, match: SpeedLimitMatch) -> SpeedLimitMatch:
    self._stuck_match = match
    self._reset_pending()
    return match

  def _keep_current_way(
    self, lat: float, lon: float, bearing_deg: float | None, v_ego_ms: float,
    cand: SpeedLimitMatch | None, now_s: float | None,
  ) -> SpeedLimitMatch | None:
    """Hysteresis + plausibility for the published current road.

    Probe matches (`_best_match` along the route) stay stateless. Only the
    road we publish as the current limit sticks.
    """
    now = time.monotonic() if now_s is None else float(now_s)
    prev = self._stuck_match
    if cand is None:
      self._reset_pending()
      return None
    if prev is None or int(prev.way_id) == int(cand.way_id) or _same_named_road(prev.road_name, cand.road_name):
      return self._accept_current_way(cand)

    prev_dist, prev_heading = _point_to_polyline_m(lat, lon, list(prev.coords))
    aligned = bearing_deg is None or _heading_aligned(bearing_deg, prev_heading)
    cand_dist, _cand_heading = _point_to_polyline_m(lat, lon, list(cand.coords))
    closer = float(prev_dist) - float(cand_dist)
    lookups, elapsed, traveled = self._observe_switch_candidate(int(cand.way_id), closer, lat, lon, now)
    if self._current_way_is_sticky(float(prev_dist), aligned, closer, lookups):
      return replace(prev, distance_m=float(prev_dist))
    if _implausible_minor(v_ego_ms, cand, prev.highway) and not self._switch_persisted(elapsed, traveled):
      if float(prev_dist) <= STICK_KEEP_M and aligned:
        return replace(prev, distance_m=float(prev_dist))
      return None
    return self._accept_current_way(cand)

  def lookup(self, lat: float, lon: float, bearing_deg: float | None = None,
             v_ego_ms: float = 0.0, now_s: float | None = None) -> SpeedLimitMatch | None:
    if self._con is None:
      return None
    gps_match = self._stabilize_match(
      float(lat), float(lon), bearing_deg,
      self._best_match(float(lat), float(lon), bearing_deg),
    )
    qlat, qlon = float(lat), float(lon)
    lead_m = osm_sign_lead_m(v_ego_ms)
    lead_match = gps_match
    if lead_m > 0.0 and bearing_deg is not None:
      qlat, qlon = _offset_point(qlat, qlon, float(bearing_deg), lead_m)
      lead_match = self._stabilize_match(
        qlat, qlon, bearing_deg,
        self._best_match(qlat, qlon, bearing_deg),
      )

    # GNSS lag: raise posted when the 1.5 s point is already in a higher zone.
    # Do not snap posted down — any decrease (10/15/20 mph, short zones) must
    # ease with kin+110 m, not jump when GPS or the offset crosses the boundary.
    raised = (
      lead_match is not None and gps_match is not None
      and float(lead_match.speed_limit_ms) > float(gps_match.speed_limit_ms) + 0.3
    )
    if raised or gps_match is None:
      match = lead_match
    else:
      match = gps_match
    match = self._keep_current_way(
      float(lat), float(lon), bearing_deg, float(v_ego_ms), match, now_s,
    )
    if match is None:
      return None
    if bearing_deg is None:
      return match
    raised = bool(
      raised and lead_match is not None and int(match.way_id) == int(lead_match.way_id)
    )

    # Next limit follows the matched road (bearing + class/name). Do not use a
    # heading ray that can pick a nearby residential fill as "ahead".
    # Remaining is from GPS minus v*1.5 s so ease starts at the lag-corrected zone.
    if raised:
      picked = self._next_limit(qlat, qlon, float(bearing_deg), match)
    else:
      picked = self._next_limit(float(lat), float(lon), float(bearing_deg), match)
      if picked is not None:
        nxt, dist = picked
        picked = (nxt, max(0.0, float(dist) - lead_m))
    if picked is None:
      return match
    next_limit, next_dist = picked
    if next_limit <= 0:
      return match
    return SpeedLimitMatch(
      speed_limit_ms=match.speed_limit_ms,
      way_id=match.way_id,
      road_name=match.road_name,
      highway=match.highway,
      distance_m=match.distance_m,
      next_speed_limit_ms=next_limit,
      next_distance_m=next_dist,
      estimated=match.estimated,
    )
