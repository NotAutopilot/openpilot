"""Overpass JSON → NAP OSM speed-limit ways.

OpenStreetMap data is ODbL: © OpenStreetMap contributors.
https://www.openstreetmap.org/copyright
"""
from __future__ import annotations

import json
import math
import urllib.error
import urllib.parse
import urllib.request

from openpilot.selfdrive.mapd.maps_manifest import USER_AGENT
from openpilot.selfdrive.mapd.mn_statutory import (
  FILLABLE_HIGHWAY_REGEX,
  PlaceIndex,
  places_from_overpass,
)
from openpilot.selfdrive.mapd.us_statutory import default_maxspeed_ms, inherit_posted_neighbors, is_fill_source, state_index
from openpilot.selfdrive.mapd.speed_limit import parse_maxspeed

OVERPASS_URL = "https://overpass-api.de/api/interpreter"
_KEEP = "Previous maps were left unchanged. You can retry Refresh maps."


def bbox_from_center(lat: float, lon: float, radius_km: float) -> tuple[float, float, float, float]:
  """south, west, north, east box that contains a radius_km circle around lat/lon."""
  dlat = radius_km / 111.0
  dlon = radius_km / (111.0 * max(0.2, abs(math.cos(math.radians(lat)))))
  return lat - dlat, lon - dlon, lat + dlat, lon + dlon


def overpass_query(
  south: float, west: float, north: float, east: float,
  *,
  timeout_s: int = 180,
  include_unmarked: bool = False,
  include_roundabouts: bool = True,
  unmarked_regex: str = FILLABLE_HIGHWAY_REGEX,
) -> str:
  """Bbox query. Tagged maxspeed only, or all fillable highways + place polygons (MN fill).

  include_roundabouts adds junction=roundabout|circular rings and the highway
  ways that touch them, with or without maxspeed.
  """
  bbox = f"({south},{west},{north},{east})"
  # Roundabout rings (any maxspeed) + every highway way sharing a ring node
  # (approach roads). Stored in rb_ways, not in the speed-limit ways table.
  rb_sets = "" if not include_roundabouts else f"""
way["highway"]["junction"~"^(roundabout|circular)$"]{bbox}->.rb;
node(w.rb)->.rbn;
way(bn.rbn)["highway"]->.ap;"""
  rb_union = "" if not include_roundabouts else "\n  .rb;\n  .ap;"
  if include_unmarked:
    return f"""
[out:json][timeout:{int(timeout_s)}];{rb_sets}
(
  way["highway"~"{unmarked_regex}"]{bbox};
  way["place"~"^(city|town|village)$"]{bbox};
  rel["place"~"^(city|town|village)$"]{bbox};
  way["boundary"="administrative"]["admin_level"="8"]{bbox};
  rel["boundary"="administrative"]["admin_level"="8"]{bbox};{rb_union}
);
out geom;
""".strip()
  if not include_roundabouts:
    return f"""
[out:json][timeout:{int(timeout_s)}];
way["highway"]["maxspeed"]{bbox};
out geom;
""".strip()
  return f"""
[out:json][timeout:{int(timeout_s)}];{rb_sets}
(
  way["highway"]["maxspeed"]{bbox};{rb_union}
);
out geom;
""".strip()


def fetch_overpass(
  bbox: tuple[float, float, float, float],
  url: str = OVERPASS_URL,
  *,
  timeout_s: float = 240,
  query_timeout: int = 180,
  include_unmarked: bool = False,
  include_roundabouts: bool = True,
  unmarked_regex: str = FILLABLE_HIGHWAY_REGEX,
) -> dict:
  """POST a bbox highway query. Raises RuntimeError with a retryable message on timeout/HTTP failure."""
  q = overpass_query(*bbox, timeout_s=query_timeout, include_unmarked=include_unmarked,
                     include_roundabouts=include_roundabouts, unmarked_regex=unmarked_regex)
  data = urllib.parse.urlencode({"data": q}).encode()
  req = urllib.request.Request(url, data=data, headers={"User-Agent": USER_AGENT})
  try:
    with urllib.request.urlopen(req, timeout=timeout_s) as resp:
      raw = resp.read().decode("utf-8")
  except urllib.error.HTTPError as e:
    raise RuntimeError(f"OSM Overpass failed HTTP {e.code}. {_KEEP}") from e
  except urllib.error.URLError as e:
    raise RuntimeError(f"OSM Overpass failed: {e}. Need Wi-Fi to overpass-api.de. {_KEEP}") from e
  except TimeoutError as e:
    raise RuntimeError(f"OSM Overpass timed out. {_KEEP}") from e
  try:
    payload = json.loads(raw)
  except json.JSONDecodeError as e:
    raise RuntimeError(f"OSM Overpass returned invalid JSON: {e}. {_KEEP}") from e
  if not isinstance(payload, dict):
    raise RuntimeError(f"OSM Overpass returned a non-object JSON payload. {_KEEP}")
  remark = str(payload.get("remark") or "")
  if "timed out" in remark.lower() or payload.get("timeout"):
    raise RuntimeError(f"OSM Overpass timed out ({remark or 'query timeout'}). {_KEEP}")
  return payload


def tagged_maxspeed_ms(tags: dict) -> float | None:
  """Numeric OSM maxspeed / maxspeed:forward. Non-numeric tags are not usable."""
  ms = parse_maxspeed(tags.get("maxspeed"))
  if ms is None:
    ms = parse_maxspeed(tags.get("maxspeed:forward"))
  return ms


def resolve_way_speed(
  tags: dict,
  coords: list[tuple[float, float]],
  *,
  fill_unmarked: bool = False,
  places: PlaceIndex | None = None,
  state: str | None = None,
) -> tuple[float | None, str]:
  """Return (maxspeed_ms, source). source is 'osm', MN_169.14 or '<ST>_default'. Tagged numeric wins.

  Untagged ways get the state default (us_statutory) when fill_unmarked; the
  state comes from the way's position unless given.
  """
  ms = tagged_maxspeed_ms(tags)
  if ms is not None:
    return ms, "osm"
  if not fill_unmarked:
    return None, ""
  if state is None:
    state = state_index().state_of_way(coords)
  return default_maxspeed_ms(state, str(tags.get("highway") or ""), tags, coords, places)


def ways_from_overpass(payload: dict, *, fill_unmarked: bool = False) -> list[dict]:
  places = places_from_overpass(payload) if fill_unmarked else PlaceIndex()
  out = []
  for el in payload.get("elements", []):
    if el.get("type") != "way":
      continue
    tags = el.get("tags") or {}
    if "highway" not in tags:
      continue
    geom = el.get("geometry") or []
    coords = [(float(p["lat"]), float(p["lon"])) for p in geom if "lat" in p and "lon" in p]
    if len(coords) < 2:
      continue
    ms, source = resolve_way_speed(tags, coords, fill_unmarked=fill_unmarked, places=places)
    if ms is None:
      continue
    out.append({
      "way_id": int(el["id"]),
      "name": tags.get("name") or tags.get("ref") or "",
      "highway": tags.get("highway") or "",
      "maxspeed_ms": ms,
      "coords": coords,
      "source": source,
      "estimated": is_fill_source(source),
      "junction": tags.get("junction") or "",
    })
  if fill_unmarked:
    inherit_posted_neighbors(out)
  return out
