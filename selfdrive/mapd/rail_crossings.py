"""Railroad crossings on drivable roads, from the map pack (schema v5+).

Data only for now: OSM railway=level_crossing / railway=crossing nodes that sit
on a drivable highway way. No UI alert yet; ``crossings_ahead`` is the lookup a
later alert will use.
"""
from __future__ import annotations

import math
import sqlite3
from dataclasses import dataclass

TABLE = "rail_crossings"
EARTH_R_M = 6371000.0


@dataclass(frozen=True)
class RailCrossing:
  node_id: int
  lat: float
  lon: float
  way_id: int
  kind: str        # "level_crossing" | "crossing"
  barrier: str     # crossing:barrier (e.g. "full", "half", "no"), "" if untagged
  light: str       # crossing:light ("yes"/"no"), "" if untagged
  crossing: str    # crossing=* (e.g. "uncontrolled"), "" if untagged
  distance_m: float  # along the heading
  lateral_m: float   # signed, + = right of heading


def ensure_table(con: sqlite3.Connection) -> None:
  con.execute(
    f"CREATE TABLE IF NOT EXISTS {TABLE}(node_id INTEGER PRIMARY KEY, lat REAL NOT NULL, lon REAL NOT NULL," +
    " way_id INTEGER NOT NULL, kind TEXT NOT NULL, barrier TEXT NOT NULL DEFAULT '', light TEXT NOT NULL DEFAULT ''," +
    " crossing TEXT NOT NULL DEFAULT '')")
  con.execute(f"CREATE INDEX IF NOT EXISTS {TABLE}_lat_lon ON {TABLE}(lat, lon)")


def has_table(con: sqlite3.Connection) -> bool:
  return con.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name=?", (TABLE,)).fetchone() is not None


def insert(con: sqlite3.Connection, node_id: int, lat: float, lon: float, way_id: int, kind: str,
           barrier: str = "", light: str = "", crossing: str = "") -> None:
  con.execute(f"INSERT OR REPLACE INTO {TABLE} VALUES(?,?,?,?,?,?,?,?)",
              (node_id, lat, lon, way_id, kind, barrier or "", light or "", crossing or ""))


def crossings_ahead(con: sqlite3.Connection, lat: float, lon: float, bearing_deg: float,
                    horizon_m: float = 500.0, lateral_m: float = 30.0) -> list[RailCrossing]:
  """Crossings within ``horizon_m`` ahead and ``lateral_m`` of the heading line, nearest first."""
  if not has_table(con):
    return []
  dlat = math.degrees(horizon_m / EARTH_R_M)
  dlon = dlat / max(math.cos(math.radians(lat)), 1e-6)
  rows = con.execute(
    f"SELECT node_id, lat, lon, way_id, kind, barrier, light, crossing FROM {TABLE}" +
    " WHERE lat BETWEEN ? AND ? AND lon BETWEEN ? AND ?",
    (lat - dlat, lat + dlat, lon - dlon, lon + dlon)).fetchall()
  h = math.radians(bearing_deg)
  hx, hy = math.sin(h), math.cos(h)
  k = math.radians(1) * EARTH_R_M
  out = []
  for nid, la, lo, wid, kind, barrier, light, crossing in rows:
    e = (lo - lon) * k * math.cos(math.radians(lat))
    n = (la - lat) * k
    along = e * hx + n * hy
    lat_off = e * hy - n * hx
    if 0.0 <= along <= horizon_m and abs(lat_off) <= lateral_m:
      out.append(RailCrossing(nid, la, lo, wid, kind, barrier, light, crossing, along, lat_off))
  out.sort(key=lambda c: c.distance_m)
  return out
