"""Local speed-limit overrides from driver-confirmed camera signs.

A HUD YES tap on the speed-sign popup writes NAPSpeedSignConfirmed
({mph, seq, lat, lon, bearing}). mapd consumes it once per seq, keys it to
the OSM way it is currently matched to (same db.lookup way / heading logic
that produces liveMapDataNAP.speedLimit) plus the travel heading, and stores
it here. From then on, whenever mapd matches that way in that direction it
publishes the override as speedLimit (source "sign"), so the existing
speed-limit sign and the Map Speed Limit cruise path (MAX = limit + offset)
use it on this and future drives. The opposite direction keeps OSM.
A newer YES on the same way + direction replaces the old value. Nothing is
uploaded (OSM edits are future work). NO taps never reach this file.
"""
from __future__ import annotations

import json
import os
import time

PARAM = "NAPSpeedSignConfirmed"
FILENAME = "map_speed_overrides.json"
MPH_TO_MS = 0.44704
HEADING_MATCH_DEG = 60.0
PENDING_MAX_S = 20.0   # a YES with no OSM match yet waits this long for one
MAX_ENTRIES = 5000


def overrides_path() -> str:
  env = os.environ.get("NAP_MAP_SPEED_OVERRIDES")
  if env:
    return env
  base = "/data/media/0/nap" if os.path.isdir("/data/media/0") else os.path.join(
    os.path.expanduser("~"), ".comma", "media", "0", "nap")
  return os.path.join(base, FILENAME)


def heading_delta(a: float, b: float) -> float:
  d = abs(float(a) - float(b)) % 360.0
  return 360.0 - d if d > 180.0 else d


def parse_confirmed(raw) -> dict | None:
  if raw is None:
    return None
  try:
    d = json.loads(raw) if isinstance(raw, (str, bytes, bytearray)) else dict(raw)
    mph = int(d["mph"])
    seq = float(d["seq"])
  except Exception:
    return None
  if not (5 <= mph <= 85) or mph % 5:
    return None
  out = {"mph": mph, "seq": seq}
  for k in ("lat", "lon", "bearing"):
    try:
      if d.get(k) is not None:
        out[k] = float(d[k])
    except (TypeError, ValueError):
      pass
  return out


class SpeedOverrides:
  def __init__(self, path: str | None = None) -> None:
    self.path = path or overrides_path()
    self.entries: list[dict] = []
    self._mtime = None
    self.last_seq: float | None = None
    self.pending: dict | None = None
    self.pending_t = 0.0
    self.load()

  def load(self) -> None:
    try:
      m = os.path.getmtime(self.path)
      if m == self._mtime:
        return
      with open(self.path) as f:
        data = json.load(f)
      self.entries = [e for e in data.get("overrides", []) if "way_id" in e and "mph" in e and "heading" in e]
      self._mtime = m
    except (OSError, ValueError):
      if not os.path.exists(self.path):
        self.entries = []
        self._mtime = None

  def save(self) -> None:
    os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
    tmp = self.path + ".tmp"
    with open(tmp, "w") as f:
      json.dump({"version": 1, "overrides": self.entries[-MAX_ENTRIES:]}, f, separators=(",", ":"))
    os.replace(tmp, self.path)
    self._mtime = os.path.getmtime(self.path)

  def find(self, way_id: int, bearing: float | None) -> dict | None:
    if bearing is None:
      return None
    best = None
    for e in self.entries:
      if int(e["way_id"]) == int(way_id) and heading_delta(e["heading"], bearing) <= HEADING_MATCH_DEG:
        best = e  # newest wins (appended last)
    return best

  def limit_ms(self, way_id: int, bearing: float | None) -> float | None:
    e = self.find(way_id, bearing)
    return None if e is None else int(e["mph"]) * MPH_TO_MS

  def add(self, way_id: int, heading: float, conf: dict, road: str = "", now: float | None = None) -> dict:
    now = time.time() if now is None else now  # noqa: TID251  wall clock in the stored row
    self.entries = [e for e in self.entries
                    if not (int(e["way_id"]) == int(way_id) and heading_delta(e["heading"], heading) <= HEADING_MATCH_DEG)]
    e = {"way_id": int(way_id), "heading": round(float(heading), 1), "mph": int(conf["mph"]),
         "lat": conf.get("lat"), "lon": conf.get("lon"), "road": road, "t": round(now, 1), "source": "sign_yes"}
    self.entries.append(e)
    self.save()
    return e

  def consume(self, raw_param, match, bearing: float | None, mono_now: float) -> dict | None:
    """Turn a new YES (by seq) into an override on the current matched way.

    Returns the stored entry when one was written this call.
    """
    conf = parse_confirmed(raw_param)
    if conf is not None and conf["seq"] != self.last_seq:
      self.last_seq = conf["seq"]
      self.pending, self.pending_t = conf, mono_now
    if self.pending is None:
      return None
    if mono_now - self.pending_t > PENDING_MAX_S:
      self.pending = None
      return None
    hdg = bearing if bearing is not None else self.pending.get("bearing")
    if match is None or hdg is None:
      return None
    e = self.add(int(match.way_id), float(hdg), self.pending, road=str(getattr(match, "road_name", "") or ""))
    self.pending = None
    return e


def request_override(mph: int, rec: dict | None, seq: float, params=None) -> bool:
  """HUD YES: hand the confirmed mph (+ where it was read) to mapd."""
  payload: dict = {"mph": int(mph), "seq": float(seq)}
  for k in ("lat", "lon", "bearing"):
    if (rec or {}).get(k) is not None:
      payload[k] = (rec or {})[k]
  if parse_confirmed(payload) is None:
    return False
  try:
    if params is None:
      from openpilot.common.params import Params
      params = Params()
    params.put(PARAM, payload)
    return True
  except Exception:
    return False
