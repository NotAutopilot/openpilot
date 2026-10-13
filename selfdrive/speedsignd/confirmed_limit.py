"""Driver-confirmed camera speed limit (HUD plate YES tap only).

The UI writes NAPSpeedSignConfirmed = {"mph": int, "seq": float} only when the
driver taps YES on the speed-sign plate. card.py reads it in params_thread (no
new subscriber) and, while the hold below is active, feeds that limit into the
existing map-speed path as the posted limit, so MAX = confirmed + the
NAPMapSpeedOffsetMph offset, with the usual map slew / ease / sticky / stalk
override rules. Nothing is ever applied from an unconfirmed read.

Hold rule:
  * starts on each new YES (a new seq replaces any older confirmed limit);
  * the OSM posted limit at tap time is the anchor (if maps had no limit
    then, the first OSM limit seen afterwards becomes the anchor);
  * ends when the OSM posted limit changes to a different value than the
    anchor (we entered a new map zone), on a newer YES, or when the param
    is cleared (manager start / offroad transition).
"""
from __future__ import annotations

import json

PARAM = "NAPSpeedSignConfirmed"
MPH_TO_KPH = 1.609344
MIN_MPH = 5
MAX_MPH = 85
MAP_CHANGE_EPS_KPH = 1.5


def parse_confirmed(raw) -> tuple[int, float] | None:
  """(mph, seq) from the param value (dict, JSON str/bytes) or None."""
  if raw is None:
    return None
  try:
    d = json.loads(raw) if isinstance(raw, (str, bytes, bytearray)) else dict(raw)
    mph = int(d["mph"])
    seq = float(d["seq"])
  except Exception:
    return None
  if not (MIN_MPH <= mph <= MAX_MPH) or mph % 5:
    return None
  return mph, seq


def confirmed_payload(mph: int, seq: float) -> dict:
  return {"mph": int(mph), "seq": float(seq)}


class ConfirmedLimitHold:
  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self.kph: float | None = None
    self.seq: float | None = None
    self.anchor_kph: float | None = None
    self.released_seq: float | None = None

  @property
  def active(self) -> bool:
    return self.kph is not None

  def update(self, confirmed: tuple[int, float] | None, map_posted_kph: float | None) -> float | None:
    """Return the confirmed limit in kph while held, else None.

    map_posted_kph is the raw OSM posted limit (no offset), None if unknown.
    """
    m = map_posted_kph if (map_posted_kph is not None and map_posted_kph > 0) else None
    if confirmed is None:
      self.reset()
      return None
    mph, seq = confirmed
    if seq != self.seq:
      if seq == self.released_seq:
        return None
      self.seq = seq
      self.kph = mph * MPH_TO_KPH
      self.anchor_kph = m
      return self.kph
    if self.kph is None:
      return None
    if self.anchor_kph is None:
      self.anchor_kph = m
    elif m is not None and abs(m - self.anchor_kph) > MAP_CHANGE_EPS_KPH:
      self.released_seq = seq
      self.kph = None
      self.anchor_kph = None
      return None
    return self.kph


class ConfirmedMapData:
  """Duck-typed stand-in for liveMapDataNAP fields card.py's map overlay reads."""

  def __init__(self, kph: float, real=None) -> None:
    self.speedLimit = kph / 3.6
    self.speedLimitValid = True
    # Upcoming OSM limits stay on: a lower next limit still eases MAX down.
    nxt = float(getattr(real, "nextSpeedLimit", 0.0) or 0.0) if real is not None else 0.0
    self.nextSpeedLimit = nxt
    self.nextSpeedLimitDistance = float(getattr(real, "nextSpeedLimitDistance", 0.0) or 0.0) if nxt > 0 else 0.0
