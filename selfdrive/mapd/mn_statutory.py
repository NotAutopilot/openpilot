"""Minnesota statutory speed-limit fills for NAP OSM packs.

When OSM has no usable numeric maxspeed, NAP may estimate a limit from
Minn. Stat. 169.14 / MnDOT defaults. These values are pack-only estimates
(source=MN_169.14). They must never be uploaded to osm.org.

OpenStreetMap geometries/tags remain ODbL: © OpenStreetMap contributors.
"""
from __future__ import annotations

from openpilot.common.constants import CV

# Benson, MN — the 100-mile refresh / fill region for the v3 pack overlay.
BENSON_LAT = 45.315
BENSON_LON = -95.602
BENSON_RADIUS_MILES = 100.0

# Coarse Minnesota bounds (excludes the SD sliver of a 100-mile Benson box).
MN_SOUTH = 43.499
MN_WEST = -97.239
MN_NORTH = 49.386
MN_EAST = -89.483

FILL_SOURCE = "MN_169.14"
FILL_NOTES = (
  "Unmarked fills are NAP statutory estimates (Minn. Stat. 169.14 / MnDOT), "
  + "not surveyed OSM maxspeed. Never upload these tags to osm.org."
)

ALLEY_MPH = 10
URBAN_DISTRICT_MPH = 30
RURAL_OTHER_MPH = 55
EXPRESSWAY_MPH = 65
MOTORWAY_RURAL_MPH = 70
MOTORWAY_URBANIZED_MPH = 65  # pop>50k urbanized area; unused around Benson

SKIP_HIGHWAYS = frozenset({
  "service", "track", "path", "footway", "cycleway", "pedestrian",
  "construction", "proposed", "abandoned", "disused",
  "steps", "bridleway", "corridor", "platform", "bus_stop",
  "rest_area", "services", "raceway", "escape", "busway",
  "elevator", "escalator", "via_ferrata", "emergency_bay",
})

# Overpass regex: fillable highway classes only (keeps metro payloads smaller).
FILLABLE_HIGHWAY_REGEX = (
  "^(motorway(_link)?|trunk(_link)?|primary(_link)?|secondary(_link)?"
  + r"|tertiary(_link)?|unclassified|residential|living_street|alley)$"
)

ALLEY_LIKE = frozenset({"alley", "living_street"})
ALWAYS_URBAN = frozenset({"residential"})
MOTORWAY_LIKE = frozenset({"motorway", "motorway_link"})
EXPRESSWAY_ELIGIBLE = frozenset({"trunk", "primary", "trunk_link", "primary_link"})
RURAL_OTHER = frozenset({
  "unclassified",
  "tertiary", "tertiary_link",
  "secondary", "secondary_link",
  "primary", "primary_link",
  "trunk", "trunk_link",
})
PLACE_VALUES = frozenset({"city", "town", "village"})
CITY_BORDER_TYPES = frozenset({"city", "town", "village"})
_SIDEWALK_URBAN = frozenset({"yes", "both", "left", "right", "separate", "shared"})


def in_minnesota(lat: float, lon: float) -> bool:
  return MN_SOUTH <= lat <= MN_NORTH and MN_WEST <= lon <= MN_EAST


def bbox_intersects_minnesota(bbox: tuple[float, float, float, float]) -> bool:
  south, west, north, east = bbox
  return north >= MN_SOUTH and south <= MN_NORTH and east >= MN_WEST and west <= MN_EAST


def benson_fill_bbox(radius_km: float | None = None) -> tuple[float, float, float, float]:
  """100-mile box around Benson, clipped to Minnesota so SD tagged ways stay in the US pack."""
  import math
  if radius_km is None:
    radius_km = BENSON_RADIUS_MILES * CV.MPH_TO_KPH
  dlat = radius_km / 111.0
  dlon = radius_km / (111.0 * max(0.2, abs(math.cos(math.radians(BENSON_LAT)))))
  south, west = BENSON_LAT - dlat, BENSON_LON - dlon
  north, east = BENSON_LAT + dlat, BENSON_LON + dlon
  return (
    max(south, MN_SOUTH),
    max(west, MN_WEST),
    min(north, MN_NORTH),
    min(east, MN_EAST),
  )


def is_urban_place_tags(tags: dict) -> bool:
  """True for an OSM area that should count as an urban district.

  Minnesota cities are usually boundary=administrative admin_level=8 with
  border_type=city (often no place=* on the polygon). Townships are rural.
  """
  place = str(tags.get("place") or "").strip().lower()
  if place in PLACE_VALUES:
    return True
  bt = str(tags.get("border_type") or "").strip().lower()
  if bt in CITY_BORDER_TYPES:
    return True
  if bt == "township":
    return False
  name = str(tags.get("name") or "")
  if "township" in name.lower():
    return False
  if str(tags.get("admin_level") or "") == "8" and str(tags.get("boundary") or "") == "administrative":
    return bool(name.strip())
  return False


def _truthy_yes(tag: str | None) -> bool:
  return str(tag or "").strip().lower() in ("yes", "true", "1")


def is_expressway_like(tags: dict) -> bool:
  return _truthy_yes(tags.get("expressway")) or _truthy_yes(tags.get("motorroad"))


def is_urban_way(tags: dict, coords: list[tuple[float, float]], places: PlaceIndex | None) -> bool:
  """Highway class + simple urban heuristic (place polygons / sidewalks)."""
  highway = str(tags.get("highway") or "").strip().lower()
  if highway in ALWAYS_URBAN or highway in ALLEY_LIKE:
    return True
  sidewalk = str(tags.get("sidewalk") or "").strip().lower()
  if sidewalk in _SIDEWALK_URBAN:
    return True
  if places is None or not coords:
    return False
  lat = sum(c[0] for c in coords) / len(coords)
  lon = sum(c[1] for c in coords) / len(coords)
  return places.contains(lat, lon)


def statutory_maxspeed_mph(highway: str, tags: dict, *, urban: bool) -> int | None:
  """MN default mph, or None to skip the way."""
  hw = (highway or str(tags.get("highway") or "")).strip().lower()
  if not hw or hw in SKIP_HIGHWAYS:
    return None
  if hw in ALLEY_LIKE:
    return ALLEY_MPH
  if hw in MOTORWAY_LIKE:
    # Census urbanized (>50k) would be 65 mph. Western MN around Benson is rural.
    return MOTORWAY_RURAL_MPH
  if hw in EXPRESSWAY_ELIGIBLE and is_expressway_like(tags):
    return EXPRESSWAY_MPH
  if hw in ALWAYS_URBAN:
    return URBAN_DISTRICT_MPH
  if hw in RURAL_OTHER:
    return URBAN_DISTRICT_MPH if urban else RURAL_OTHER_MPH
  return None


def statutory_maxspeed_ms(highway: str, tags: dict, *, urban: bool) -> float | None:
  mph = statutory_maxspeed_mph(highway, tags, urban=urban)
  if mph is None:
    return None
  return mph * CV.MPH_TO_MS


def _point_in_ring(lat: float, lon: float, ring: list[tuple[float, float]]) -> bool:
  """Even-odd ray cast. ring is (lat, lon) vertices."""
  n = len(ring)
  if n < 3:
    return False
  inside = False
  j = n - 1
  for i in range(n):
    yi, xi = ring[i]
    yj, xj = ring[j]
    if (xi > lon) != (xj > lon):
      denom = xj - xi
      if abs(denom) > 1e-18:
        ycross = (yj - yi) * (lon - xi) / denom + yi
        if lat < ycross:
          inside = not inside
    j = i
  return inside


class PlaceIndex:
  """Named OSM place=city/town/village polygons for the urban heuristic."""

  def __init__(self) -> None:
    self._polys: list[tuple[float, float, float, float, list[list[tuple[float, float]]]]] = []

  def __len__(self) -> int:
    return len(self._polys)

  def add_rings(self, rings: list[list[tuple[float, float]]], _name: str = "") -> None:
    outers = [r for r in rings if len(r) >= 3]
    if not outers:
      return
    lats = [p[0] for r in outers for p in r]
    lons = [p[1] for r in outers for p in r]
    self._polys.append((min(lats), min(lons), max(lats), max(lons), outers))

  def contains(self, lat: float, lon: float) -> bool:
    for min_lat, min_lon, max_lat, max_lon, rings in self._polys:
      if lat < min_lat or lat > max_lat or lon < min_lon or lon > max_lon:
        continue
      if any(_point_in_ring(lat, lon, ring) for ring in rings):
        return True
    return False


def places_from_overpass(payload: dict) -> PlaceIndex:
  idx = PlaceIndex()
  for el in payload.get("elements", []):
    tags = el.get("tags") or {}
    if not is_urban_place_tags(tags):
      continue
    name = tags.get("name") or ""
    etype = el.get("type")
    rings: list[list[tuple[float, float]]] = []
    if etype == "way":
      geom = el.get("geometry") or []
      ring = [(float(p["lat"]), float(p["lon"])) for p in geom if "lat" in p and "lon" in p]
      if len(ring) >= 3:
        rings.append(ring)
    elif etype == "relation":
      for mem in el.get("members") or []:
        if mem.get("type") != "way":
          continue
        role = str(mem.get("role") or "outer").lower()
        if role not in ("outer", ""):
          continue
        geom = mem.get("geometry") or []
        ring = [(float(p["lat"]), float(p["lon"])) for p in geom if "lat" in p and "lon" in p]
        if len(ring) >= 3:
          rings.append(ring)
    if rings:
      idx.add_rings(rings, name)
  return idx
