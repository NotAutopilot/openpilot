"""OSM roundabout rings + approach roads for the NAP map pack, and ring geometry.

The US speed-limit pack only kept ways with a numeric maxspeed. Most
circulating rings (and many approach roads) carry no maxspeed, so the
roundabout funnel never fired on them (Soco Rd / Dellwood / Jonathan Creek,
way 1514082747). Rings and the roads that touch them are stored in a
separate `rb_ways` table so the speed-limit `ways` table, its matching, and
the MN statutory fills are untouched.

Ring geometry (center, radius, circulation sense, lanes) is fitted from all
connected ring ways and handed to controlsd (Roundabout Steering Assist)
through the NAPRoundaboutRing param. No cereal schema change.

OpenStreetMap data is ODbL: © OpenStreetMap contributors.
https://www.openstreetmap.org/copyright
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field

from openpilot.selfdrive.mapd.speed_limit import parse_maxspeed

EARTH_R = 6371000.0
RB_JUNCTIONS = frozenset({"roundabout", "circular"})
ROLE_RING = "ring"
ROLE_APPROACH = "approach"
# Keep this much of each approach road, measured from the node it shares with the ring.
APPROACH_KEEP_M = 250.0
# Ring ways whose endpoints are within this distance chain into one ring.
RING_JOIN_M = 3.0
# Rings outside this radius band are not fitted (bad data / huge circles).
RING_FIT_MIN_R_M = 5.0
RING_FIT_MAX_R_M = 90.0
LANE_WIDTH_M = 3.5
# Comfort lateral acceleration used for the ring speed target.
RB_COMFORT_LAT_ACCEL = 2.5
RING_JSON_VERSION = 1
# Approach polylines carried in the ring param (trimmed, local xy).
RING_JSON_APPROACH_M = 120.0
# highway=mini_roundabout is an OSM node, not a way: stored as a small synthetic ring.
MINI_RB_RADIUS_M = 7.0
MINI_RB_POINTS = 16


def is_ring_tags(tags: dict) -> bool:
  return bool(tags.get("highway")) and str(tags.get("junction") or "").strip().lower() in RB_JUNCTIONS


def _lanes(tags: dict) -> int:
  try:
    return max(0, int(str(tags.get("lanes") or "0").split(";")[0].strip()))
  except ValueError:
    return 0


def local_xy(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
  """East, north meters from (lat0, lon0)."""
  x = math.radians(lon - lon0) * math.cos(math.radians(lat0)) * EARTH_R
  y = math.radians(lat - lat0) * EARTH_R
  return x, y


def latlon_from_xy(x: float, y: float, lat0: float, lon0: float) -> tuple[float, float]:
  lat = lat0 + math.degrees(y / EARTH_R)
  lon = lon0 + math.degrees(x / (EARTH_R * math.cos(math.radians(lat0))))
  return lat, lon


def _dist_m(a: tuple[float, float], b: tuple[float, float]) -> float:
  return math.hypot(*local_xy(b[0], b[1], a[0], a[1]))


def _trim_from(coords: list[tuple[float, float]], start_at_end: bool, keep_m: float) -> list[tuple[float, float]]:
  """Keep keep_m of the polyline from the start (or the end), in original order."""
  seq = list(reversed(coords)) if start_at_end else list(coords)
  out = [seq[0]]
  run = 0.0
  for p in seq[1:]:
    d = _dist_m(out[-1], p)
    if run + d >= keep_m:
      f = (keep_m - run) / d if d > 0 else 0.0
      out.append((out[-1][0] + f * (p[0] - out[-1][0]), out[-1][1] + f * (p[1] - out[-1][1])))
      break
    out.append(p)
    run += d
  return list(reversed(out)) if start_at_end else out


def roundabout_rows(ways: list[dict], keep_m: float = APPROACH_KEEP_M) -> list[dict]:
  """Ring ways and approach roads from generic OSM ways.

  Each input dict: way_id, tags, node_ids (list[int]), coords (list[(lat, lon)]),
  same length for node_ids and coords. Output rows for OsmSpeedLimitDB.insert_rb_way.
  An approach is any highway way (not a ring) that shares a node with a ring;
  only the part within keep_m of the shared node is kept.
  """
  ring_nodes: set[int] = set()
  out: list[dict] = []
  for w in ways:
    tags = w.get("tags") or {}
    if not is_ring_tags(tags):
      continue
    coords = w.get("coords") or []
    if len(coords) < 2:
      continue
    ring_nodes.update(int(n) for n in (w.get("node_ids") or []))
    out.append(_row(w, ROLE_RING, coords))
  if not ring_nodes:
    return out
  for w in ways:
    tags = w.get("tags") or {}
    if not tags.get("highway") or is_ring_tags(tags):
      continue
    nodes = [int(n) for n in (w.get("node_ids") or [])]
    coords = w.get("coords") or []
    if len(coords) < 2 or len(nodes) != len(coords):
      continue
    hits = [i for i, n in enumerate(nodes) if n in ring_nodes]
    if not hits:
      continue
    # Split at each ring-touching node; keep both sides near it.
    pieces: list[list[tuple[float, float]]] = []
    for i in hits:
      if i > 0:
        pieces.append(_trim_from(coords[:i + 1], True, keep_m))
      if i < len(coords) - 1:
        pieces.append(_trim_from(coords[i:], False, keep_m))
    for k, piece in enumerate(pieces):
      if len(piece) >= 2:
        row = _row(w, ROLE_APPROACH, piece)
        row["piece"] = k
        out.append(row)
  return out


def _row(w: dict, role: str, coords: list[tuple[float, float]]) -> dict:
  tags = w.get("tags") or {}
  ms = parse_maxspeed(tags.get("maxspeed")) or parse_maxspeed(tags.get("maxspeed:forward")) or 0.0
  if role == ROLE_RING:
    # Advisory ring speed (yellow plate) wins when it is lower than the posted limit.
    adv = parse_maxspeed(tags.get("maxspeed:advisory")) or 0.0
    if adv > 0.5:
      ms = min(ms, adv) if ms > 0.5 else adv
  return {
    "way_id": int(w["way_id"]),
    "role": role,
    "name": tags.get("name") or tags.get("ref") or "",
    "highway": tags.get("highway") or "",
    "junction": tags.get("junction") or "",
    "oneway": 1 if str(tags.get("oneway") or "").lower() in ("yes", "1", "true") or is_ring_tags(tags) else 0,
    "lanes": _lanes(tags),
    "maxspeed_ms": float(ms),
    "coords": [(float(a), float(b)) for a, b in coords],
  }


def ways_from_overpass_elements(payload: dict) -> list[dict]:
  """Overpass `out geom` ways (with `nodes` ids) → generic ways for roundabout_rows."""
  out = []
  for el in payload.get("elements", []):
    if el.get("type") != "way":
      continue
    geom = el.get("geometry") or []
    nodes = el.get("nodes") or []
    coords = [(float(p["lat"]), float(p["lon"])) for p in geom if p and "lat" in p and "lon" in p]
    if len(coords) != len(geom) or len(coords) < 2:
      continue
    out.append({"way_id": int(el["id"]), "tags": el.get("tags") or {}, "node_ids": list(nodes), "coords": coords})
  return out


def mini_roundabout_way(node_id: int, lat: float, lon: float, tags: dict | None = None,
                        radius_m: float = MINI_RB_RADIUS_M) -> dict:
  """Synthetic closed ring way for an OSM highway=mini_roundabout node.

  way_id is the negated node id (OSM way ids are positive). node_ids are the
  real node id at every vertex so roads through the node become approaches.
  """
  tags = dict(tags or {})
  ring_tags = {"highway": "mini_roundabout", "junction": "roundabout", "oneway": "yes"}
  for k in ("name", "maxspeed", "maxspeed:advisory"):
    if tags.get(k):
      ring_tags[k] = tags[k]
  coords = []
  for i in range(MINI_RB_POINTS + 1):
    a = 2.0 * math.pi * i / MINI_RB_POINTS
    dn, de = radius_m * math.sin(a), radius_m * math.cos(a)
    coords.append((lat + math.degrees(dn / EARTH_R), lon + math.degrees(de / (EARTH_R * math.cos(math.radians(lat))))))
  return {"way_id": -int(node_id), "tags": ring_tags, "node_ids": [int(node_id)] * len(coords), "coords": coords}


def rb_rows_from_overpass(payload: dict, keep_m: float = APPROACH_KEEP_M) -> list[dict]:
  return roundabout_rows(ways_from_overpass_elements(payload), keep_m=keep_m)


# ---------------------------------------------------------------------------
# Ring geometry
# ---------------------------------------------------------------------------

@dataclass
class RingGeometry:
  lat: float
  lon: float
  radius_m: float
  rms_m: float
  ccw: bool
  lanes: int
  coverage_deg: float
  way_ids: list[int]
  tagged: bool = True
  maxspeed_ms: float = 0.0
  approaches: list[list[tuple[float, float]]] = field(default_factory=list)  # local xy (m) from center

  @property
  def half_width_m(self) -> float:
    lanes = self.lanes if self.lanes > 0 else 2
    return 0.5 * lanes * LANE_WIDTH_M

  def to_json(self) -> dict:
    return {
      "v": RING_JSON_VERSION,
      "lat": round(self.lat, 7), "lon": round(self.lon, 7),
      "r": round(self.radius_m, 2), "rms": round(self.rms_m, 2),
      "ccw": bool(self.ccw), "lanes": int(self.lanes),
      "cov": round(self.coverage_deg, 1), "ids": [int(i) for i in self.way_ids],
      "tagged": bool(self.tagged), "maxspeed": round(self.maxspeed_ms, 2),
      "app": [[[round(x, 1), round(y, 1)] for x, y in a] for a in self.approaches],
    }

  @staticmethod
  def from_json(d) -> RingGeometry | None:
    try:
      if isinstance(d, (bytes, str)):
        d = json.loads(d)
      if not isinstance(d, dict) or int(d.get("v", 0)) != RING_JSON_VERSION:
        return None
      return RingGeometry(
        lat=float(d["lat"]), lon=float(d["lon"]), radius_m=float(d["r"]), rms_m=float(d["rms"]),
        ccw=bool(d["ccw"]), lanes=int(d.get("lanes", 0)), coverage_deg=float(d.get("cov", 0.0)),
        way_ids=[int(i) for i in d.get("ids", [])], tagged=bool(d.get("tagged", True)),
        maxspeed_ms=float(d.get("maxspeed", 0.0)),
        approaches=[[(float(p[0]), float(p[1])) for p in a] for a in d.get("app", [])],
      )
    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
      return None


def _densify(pts: list[tuple[float, float]], step: float = 2.0) -> list[tuple[float, float]]:
  out = [pts[0]]
  for a, b in zip(pts, pts[1:], strict=False):
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(1, int(math.ceil(d / step)))
    for k in range(1, n + 1):
      out.append((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n))
  return out


def fit_circle(pts: list[tuple[float, float]]) -> tuple[float, float, float, float] | None:
  """Kasa least-squares circle. Returns (cx, cy, r, rms) or None."""
  n = len(pts)
  if n < 3:
    return None
  mx = sum(p[0] for p in pts) / n
  my = sum(p[1] for p in pts) / n
  u = [p[0] - mx for p in pts]
  v = [p[1] - my for p in pts]
  suu = sum(a * a for a in u)
  svv = sum(b * b for b in v)
  suv = sum(a * b for a, b in zip(u, v, strict=True))
  suuu = sum(a ** 3 for a in u)
  svvv = sum(b ** 3 for b in v)
  suvv = sum(a * b * b for a, b in zip(u, v, strict=True))
  svuu = sum(b * a * a for a, b in zip(u, v, strict=True))
  det = suu * svv - suv * suv
  if abs(det) < 1e-9:
    return None
  rhs1 = 0.5 * (suuu + suvv)
  rhs2 = 0.5 * (svvv + svuu)
  uc = (rhs1 * svv - rhs2 * suv) / det
  vc = (suu * rhs2 - suv * rhs1) / det
  cx, cy = uc + mx, vc + my
  rs = [math.hypot(p[0] - cx, p[1] - cy) for p in pts]
  r = sum(rs) / n
  rms = math.sqrt(sum((x - r) ** 2 for x in rs) / n)
  return cx, cy, r, rms


def _connected_rings(rings: list[dict], seed_id: int | None) -> list[dict]:
  if not rings:
    return []
  seed = next((w for w in rings if int(w["way_id"]) == int(seed_id or -1)), None)
  if seed is None:
    return rings
  comp = [seed]
  pending = [w for w in rings if w is not seed]
  grew = True
  while grew and pending:
    grew = False
    ends = [p for w in comp for p in (w["coords"][0], w["coords"][-1])]
    keep = []
    for w in pending:
      we = (w["coords"][0], w["coords"][-1])
      if any(_dist_m(a, b) <= RING_JOIN_M for a in we for b in ends):
        comp.append(w)
        grew = True
      else:
        keep.append(w)
    pending = keep
  return comp


def ring_geometry(rings: list[dict], approaches: list[dict] | None = None,
                  seed_id: int | None = None) -> RingGeometry | None:
  """Fit one ring from ring-way rows (role ring or junction-tagged speed ways).

  rows: dicts with way_id, coords [(lat, lon)], lanes, maxspeed_ms, junction.
  """
  comp = _connected_rings(rings, seed_id)
  if not comp:
    return None
  lat0 = sum(c[0] for w in comp for c in w["coords"]) / sum(len(w["coords"]) for w in comp)
  lon0 = sum(c[1] for w in comp for c in w["coords"]) / sum(len(w["coords"]) for w in comp)
  segs = [[local_xy(c[0], c[1], lat0, lon0) for c in w["coords"]] for w in comp]
  pts = [p for s in segs for p in _densify(s)]
  fit = fit_circle(pts)
  if fit is None:
    return None
  cx, cy, r, rms = fit
  if not (RING_FIT_MIN_R_M <= r <= RING_FIT_MAX_R_M):
    return None
  # Circulation: sign of sum (p - c) x dp over the ordered ring segments (east/north frame).
  cross = 0.0
  for s in segs:
    for a, b in zip(s, s[1:], strict=False):
      cross += (a[0] - cx) * (b[1] - a[1]) - (a[1] - cy) * (b[0] - a[0])
  angs = sorted(math.degrees(math.atan2(p[1] - cy, p[0] - cx)) % 360.0 for p in pts)
  gaps = [b - a for a, b in zip(angs, angs[1:], strict=False)] + [angs[0] + 360.0 - angs[-1]]
  coverage = 360.0 - max(gaps)
  clat, clon = latlon_from_xy(cx, cy, lat0, lon0)
  lanes = max((int(w.get("lanes") or 0) for w in comp), default=0)
  speeds = [float(w.get("maxspeed_ms") or 0.0) for w in comp if float(w.get("maxspeed_ms") or 0.0) > 0.5]
  tagged = any(str(w.get("junction") or "").strip().lower() in RB_JUNCTIONS for w in comp)
  app_xy: list[list[tuple[float, float]]] = []
  for a in approaches or []:
    xy = [local_xy(c[0], c[1], clat, clon) for c in a["coords"]]
    # Only approaches that actually touch this ring (an end within the ring band).
    if not any(abs(math.hypot(*p) - r) <= 6.0 for p in (xy[0], xy[-1])):
      continue
    inbound = abs(math.hypot(*xy[-1]) - r) <= 6.0 and abs(math.hypot(*xy[0]) - r) > 6.0
    if inbound:
      xy = list(reversed(xy))  # trim from the ring outward
    trimmed = [xy[0]]
    run = 0.0
    for p in xy[1:]:
      run += math.hypot(p[0] - trimmed[-1][0], p[1] - trimmed[-1][1])
      trimmed.append(p)
      if run >= RING_JSON_APPROACH_M:
        break
    # Stored in the direction of travel (the guide's map match needs it); two-way roads both ways.
    fwd = list(reversed(trimmed)) if inbound else trimmed
    app_xy.append(fwd)
    if not int(a.get("oneway") or 0):
      app_xy.append(list(reversed(fwd)))
  return RingGeometry(
    lat=clat, lon=clon, radius_m=r, rms_m=rms, ccw=cross > 0.0, lanes=lanes,
    coverage_deg=coverage, way_ids=sorted(int(w["way_id"]) for w in comp), tagged=tagged,
    maxspeed_ms=min(speeds) if speeds else 0.0, approaches=app_xy,
  )


def roundabout_comfort_speed_ms(radius_m: float, lanes: int = 0, maxspeed_ms: float = 0.0,
                                lat_accel: float = RB_COMFORT_LAT_ACCEL) -> float:
  """Ring speed for ≤ lat_accel on the inner lane. OSM maxspeed still caps it.

  Inner lane center ≈ R − (lanes − 1)·w/2 (R is the carriageway centerline).
  Returned value is published as liveMapDataNAP.roundaboutSpeedLimit; the
  planner clamps it to 15–20 mph (roundabout_target_ms).
  """
  lanes_n = lanes if lanes > 0 else 2
  r_inner = max(4.0, float(radius_m) - 0.5 * (lanes_n - 1) * LANE_WIDTH_M)
  v = math.sqrt(lat_accel * r_inner)
  if maxspeed_ms and maxspeed_ms > 0.5:
    v = min(v, float(maxspeed_ms))
  return v


PARAM_RING = "NAPRoundaboutRing"
RING_RETRY_S = 5.0


class RingCache:
  """mapd side: fit the hinted ring once, reuse while the hint stays on it.

  publish() is called with a JSON-able dict only when the ring changes, so the
  NAPRoundaboutRing param is written a few times per roundabout at most.
  """

  def __init__(self) -> None:
    self.geom: RingGeometry | None = None
    self._failed_id = 0
    self._failed_t = -1e9

  def update(self, db, way_id: int, lat: float, lon: float, now: float, publish=None) -> RingGeometry | None:
    way_id = int(way_id or 0)
    if way_id == 0:
      return None
    if self.geom is not None and way_id in self.geom.way_ids:
      return self.geom
    if way_id == self._failed_id and (now - self._failed_t) < RING_RETRY_S:
      return None
    try:
      geom = db.ring_geometry(way_id, lat, lon)
    except Exception:
      geom = None
    if geom is None:
      self._failed_id, self._failed_t = way_id, now
      return None
    self.geom = geom
    if publish is not None:
      publish(geom.to_json())
    return geom
