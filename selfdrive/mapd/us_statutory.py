"""US state default speed-limit fills for NAP OSM packs (all states).

When a drivable OSM way has no usable numeric maxspeed, NAP estimates one from
the state's statutory/default limits by road class and urban vs rural context.
Fills are pack-only estimates (ways.estimated = 1, source `<ST>_default`, or
`MN_169.14` for Minnesota, whose rules live in mn_statutory.py unchanged).
They must never be uploaded to osm.org.

Sources (see docs-nap/map-speed.md, "US default fills"):
- Interstate / limited-access / other-road maximums: IIHS "Maximum posted speed
  limits" table, Oct 2026 (https://www.iihs.org/research-areas/speed/speed-limit-laws).
- Urban-district defaults: state vehicle codes, listed per state in URBAN_MPH.
Rural non-freeway fills are capped (65 mph major / 55 mph minor) below the IIHS
maximums because those maximums are ceilings, not what an unmarked road carries.

OpenStreetMap geometries/tags remain ODbL: © OpenStreetMap contributors.
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass
from functools import lru_cache

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd import mn_statutory
from openpilot.selfdrive.mapd.mn_statutory import (
  ALLEY_LIKE,
  MOTORWAY_LIKE,
  PlaceIndex,
  _point_in_ring,
  is_expressway_like,
  is_urban_way,
)

MN = "MN"
SOURCE_SUFFIX = "_default"
# Major rural roads (trunk/primary) and minor rural roads (secondary/tertiary/unclassified)
# are capped below the IIHS "other roads" ceiling.
RURAL_MAJOR_CAP_MPH = 65
RURAL_MINOR_CAP_MPH = 55
RESIDENTIAL_MPH = 25
# Unposted ramps/links: never above this (curves, merges); posted ramps keep their tag.
# Applies in Minnesota too (MN 169.14 alone would give a ramp 70).
LINK_CAP_MPH = 45
ALLEY_MPH = 15

# Fill only classes the car drives at speed. residential / living_street / alley
# are filled only inside Minnesota (existing MN_169.14 behavior), everywhere else
# they would ~triple the pack for streets that are almost always 25.
FILL_CLASSES = frozenset({
  "motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link",
  "secondary", "secondary_link", "tertiary", "tertiary_link", "unclassified",
})
FILLABLE_HIGHWAY_REGEX = (
  "^(motorway(_link)?|trunk(_link)?|primary(_link)?|secondary(_link)?"
  + r"|tertiary(_link)?|unclassified)$"
)
MAJOR = frozenset({"trunk", "trunk_link", "primary", "primary_link"})
MINOR = frozenset({"secondary", "secondary_link", "tertiary", "tertiary_link", "unclassified"})


@dataclass(frozen=True)
class StateLimits:
  rural_interstate: int
  urban_interstate: int
  limited_access: int
  other_roads: int


# IIHS maximum posted speed limits, Oct 2026 (cars). DC has no interstates
# rated rural; its limited-access value reuses the urban interstate figure.
IIHS: dict[str, StateLimits] = {
  "AL": StateLimits(70, 70, 65, 65), "AK": StateLimits(65, 55, 65, 55),
  "AZ": StateLimits(75, 65, 65, 65), "AR": StateLimits(75, 65, 75, 65),
  "CA": StateLimits(70, 65, 70, 65), "CO": StateLimits(75, 65, 65, 65),
  "CT": StateLimits(65, 55, 65, 55), "DE": StateLimits(65, 55, 65, 55),
  "DC": StateLimits(55, 55, 55, 25), "FL": StateLimits(70, 65, 70, 65),
  "GA": StateLimits(70, 70, 65, 65), "HI": StateLimits(60, 60, 55, 45),
  "ID": StateLimits(75, 75, 70, 70), "IL": StateLimits(70, 55, 65, 55),
  "IN": StateLimits(70, 55, 60, 55), "IA": StateLimits(70, 65, 70, 65),
  "KS": StateLimits(75, 75, 75, 65), "KY": StateLimits(65, 65, 65, 55),
  "LA": StateLimits(75, 70, 70, 65), "ME": StateLimits(75, 75, 75, 60),
  "MD": StateLimits(70, 70, 70, 55), "MA": StateLimits(65, 65, 65, 55),
  "MI": StateLimits(70, 70, 70, 55), "MN": StateLimits(70, 65, 65, 60),
  "MS": StateLimits(70, 70, 70, 65), "MO": StateLimits(75, 60, 75, 65),
  "MT": StateLimits(80, 65, 70, 65), "NE": StateLimits(75, 70, 70, 65),
  "NV": StateLimits(80, 65, 70, 70), "NH": StateLimits(65, 65, 55, 55),
  "NJ": StateLimits(65, 55, 65, 55), "NM": StateLimits(75, 75, 65, 55),
  "NY": StateLimits(65, 65, 65, 55), "NC": StateLimits(70, 70, 70, 55),
  "ND": StateLimits(80, 75, 70, 65), "OH": StateLimits(70, 65, 70, 55),
  "OK": StateLimits(75, 70, 70, 70), "OR": StateLimits(65, 55, 65, 65),
  "PA": StateLimits(70, 70, 70, 55), "RI": StateLimits(65, 55, 55, 55),
  "SC": StateLimits(70, 70, 60, 55), "SD": StateLimits(80, 80, 70, 70),
  "TN": StateLimits(70, 70, 70, 65), "TX": StateLimits(75, 75, 75, 75),
  "UT": StateLimits(75, 70, 75, 65), "VT": StateLimits(65, 55, 50, 50),
  "VA": StateLimits(70, 70, 65, 55), "WA": StateLimits(70, 60, 60, 60),
  "WV": StateLimits(70, 55, 65, 55), "WI": StateLimits(70, 70, 70, 55),
  "WY": StateLimits(75, 75, 70, 70),
}
# Montana night limits are lower (IIHS: limited access 75 day / 70 night,
# other roads 70 / 65); the fill uses the night value.

# Urban-district default (non-residential roads inside a city/town/village).
# 25 mph unless the state code sets another district default:
#   IL 625 ILCS 5/11-601(a) 30; IN IC 9-21-5-2 30; MN 169.14 subd 2 30;
#   NY VTL 1180(b)/1643 (city default) 30; TX Transp. Code 545.352(b)(1) 30;
#   FL 316.183(2) 30; GA O.C.G.A. 40-6-181(b)(1) 30; NC G.S. 20-141(b)(1) 35;
#   PA 75 Pa.C.S. 3362(a)(1.1) 35 (urban district).
URBAN_DEFAULT_MPH = 25
URBAN_MPH: dict[str, int] = {"IL": 30, "IN": 30, "MN": 30, "NY": 30, "TX": 30, "FL": 30, "GA": 30, "NC": 35, "PA": 35}

NEAR_PAD_DEG = 0.1
STATES_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "us_states.json")


def fill_source(state: str) -> str:
  return mn_statutory.FILL_SOURCE if state == MN else f"{state}{SOURCE_SUFFIX}"


def is_fill_source(source: str | None) -> bool:
  s = str(source or "")
  return s == mn_statutory.FILL_SOURCE or s.endswith(SOURCE_SUFFIX)


def default_maxspeed_mph(state: str | None, highway: str, tags: dict, *, urban: bool) -> int | None:
  """State default mph for an untagged way, or None to leave it unfilled."""
  if not state:
    return None
  hw = (highway or str(tags.get("highway") or "")).strip().lower()
  if state == MN:
    mph = mn_statutory.statutory_maxspeed_mph(hw, tags, urban=urban)
    return min(mph, LINK_CAP_MPH) if mph is not None and hw.endswith("_link") else mph
  lim = IIHS.get(state)
  if lim is None or hw not in FILL_CLASSES:
    return None
  mph = _state_default_mph(state, lim, hw, tags, urban=urban)
  if mph is not None and hw.endswith("_link"):
    mph = min(mph, LINK_CAP_MPH)
  return mph


def _state_default_mph(state: str, lim: StateLimits, hw: str, tags: dict, *, urban: bool) -> int | None:
  if hw in MOTORWAY_LIKE:
    return lim.urban_interstate if urban else lim.rural_interstate
  if hw in MAJOR and is_expressway_like(tags):
    return lim.limited_access
  if urban:
    return URBAN_MPH.get(state, URBAN_DEFAULT_MPH)
  if hw in MAJOR:
    return min(lim.other_roads, RURAL_MAJOR_CAP_MPH)
  if hw in MINOR:
    return min(lim.other_roads, RURAL_MINOR_CAP_MPH)
  return None


def fillable_highway(state: str | None, highway: str) -> bool:
  hw = (highway or "").strip().lower()
  if state == MN:
    return hw in FILL_CLASSES or hw in ("residential",) or hw in ALLEY_LIKE
  return hw in FILL_CLASSES


class StateIndex:
  """Point → US state postal code from simplified Census state polygons."""

  def __init__(self, path: str = STATES_JSON) -> None:
    with open(path, encoding="utf-8") as f:
      data = json.load(f)
    self._polys: list[tuple[str, float, float, float, float, list[tuple[float, float]]]] = []
    for code, rings in data["states"].items():
      for ring in rings:
        pts = [(float(a), float(b)) for a, b in ring]
        lats = [p[0] for p in pts]
        lons = [p[1] for p in pts]
        self._polys.append((code, min(lats), max(lats), min(lons), max(lons), pts))

  def state_at(self, lat: float, lon: float) -> str | None:
    for code, s, n, w, e, ring in self._polys:
      if s <= lat <= n and w <= lon <= e and _point_in_ring(lat, lon, ring):
        return code
    # Simplified coastlines miss shore roads: nearest vertex within ~0.1 deg.
    best, best_d = None, NEAR_PAD_DEG ** 2
    for code, s, n, w, e, ring in self._polys:
      if not (s - NEAR_PAD_DEG <= lat <= n + NEAR_PAD_DEG and w - NEAR_PAD_DEG <= lon <= e + NEAR_PAD_DEG):
        continue
      for plat, plon in ring:
        d = (plat - lat) ** 2 + (plon - lon) ** 2
        if d < best_d:
          best, best_d = code, d
    return best

  def state_of_way(self, coords: list[tuple[float, float]]) -> str | None:
    if not coords:
      return None
    mid = coords[len(coords) // 2]
    return self.state_at(mid[0], mid[1]) or self.state_at(*coords[0]) or self.state_at(*coords[-1])


@lru_cache(maxsize=1)
def state_index() -> StateIndex:
  return StateIndex()


def default_maxspeed_ms(state: str | None, highway: str, tags: dict, coords: list[tuple[float, float]],
                        places: PlaceIndex | None) -> tuple[float | None, str]:
  """(m/s, source) default for an untagged way, or (None, '')."""
  if not state or not fillable_highway(state, highway):
    return None, ""
  urban = is_urban_way(tags, coords, places)
  mph = default_maxspeed_mph(state, highway, tags, urban=urban)
  if mph is None:
    return None, ""
  return mph * CV.MPH_TO_MS, fill_source(state)


def _end_keys(coords: list[tuple[float, float]]) -> tuple[tuple[int, int], tuple[int, int]]:
  a, b = coords[0], coords[-1]
  return (round(a[0] * 1e6), round(a[1] * 1e6)), (round(b[0] * 1e6), round(b[1] * 1e6))


def inherit_posted_neighbors(ways: list[dict]) -> int:
  """Estimated fills that continue a posted road take the posted limit.

  An untagged piece of the same named road between two posted pieces (e.g. a
  bridge or a split way on US-212 in Watertown, SD) would otherwise get the
  rural default (70) between two 45 mph zones. If an estimated way shares an
  endpoint with posted ways of the same name, it takes the lowest of their
  limits. It stays estimated. Returns how many ways were changed.
  """
  posted: dict[tuple[str, tuple[int, int]], float] = {}
  for w in ways:
    if w.get("estimated") or len(w.get("coords") or []) < 2 or not w.get("name"):
      continue
    for k in _end_keys(w["coords"]):
      key = (str(w["name"]), k)
      posted[key] = min(posted.get(key, 1e9), float(w["maxspeed_ms"]))
  changed = 0
  for w in ways:
    if not w.get("estimated") or len(w.get("coords") or []) < 2 or not w.get("name"):
      continue
    near = [posted[(str(w["name"]), k)] for k in _end_keys(w["coords"]) if (str(w["name"]), k) in posted]
    if near:
      v = min(near)
      if abs(v - float(w["maxspeed_ms"])) > 0.01:
        w["maxspeed_ms"] = v
        changed += 1
  return changed
