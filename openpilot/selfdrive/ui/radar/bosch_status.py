"""Live Bosch radar health from cereal CAN + liveTracks.

The radar interface already owns track lifecycle. This module only
decodes what a driver needs to see: ECU lamps, the 0x501 alert matrix,
the GTW identity bytes we are sending, and whether the published table
has stopped changing.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

RADAR_BUS = 1
ADDR_SGU = 0x301
ADDR_POINT0 = 0x310
ADDR_ALERT = 0x501
ADDR_CAR_CONFIG = 0x2A9
ADDR_VIN_FEED = 0x2B9
ADDR_VIN_HOST = 0x560

# DBC TeslaRadarSguInfo / TeslaRadarAlertMatrix, little-endian @1+.
SGU_HW_FAIL_BIT = 45
SGU_FAIL_BIT = 46
SGU_DIRTY_BIT = 44

ALERT_BITS: tuple[tuple[int, str], ...] = (
  (3, "adjustmentNotDone"),
  (4, "adjustmentReq"),
  (5, "adjustmentNotOk"),
  (6, "sensorBlinded"),
  (8, "configMismatch"),
  (11, "espMIA"),
  (12, "gtwMIA"),
  (13, "sccmMIA"),
  (36, "vinValidity"),
  (41, "xwdValidity"),
  (60, "radPositionMismatch"),
  (61, "strRackMismatch"),
)

FREEZE_HOLD_S = 5.0
FREEZE_MIN_TRACKS = 4
ALIGN_D_MIN = 2.5
ALIGN_D_MAX = 14.5
ALIGN_Y_MAX = 1.0


def radar_bus(src: int) -> bool:
  return (int(src) & 0x7F) == RADAR_BUS


def _bit(dat: bytes, index: int) -> bool:
  byte_i, bit_i = divmod(index, 8)
  if byte_i >= len(dat):
    return False
  return bool((dat[byte_i] >> bit_i) & 1)


def decode_car_config(dat: bytes) -> tuple[bool, int, int]:
  lo = int.from_bytes(dat[:4], "little")
  hi = int.from_bytes(dat[4:8], "little") if len(dat) >= 8 else 0
  return bool(lo & 0x08), (hi >> 4) & 0x03, (hi >> 12) & 0x07


def vin_mux_chars(dat: bytes) -> tuple[int, tuple[str, ...]] | None:
  if not dat:
    return None
  rec = dat[0]
  if rec == 0x10 and len(dat) >= 8:
    return 0, tuple(chr(b) if 32 <= b < 127 else "?" for b in dat[5:8])
  if rec == 0x11 and len(dat) >= 8:
    return 3, tuple(chr(b) if 32 <= b < 127 else "?" for b in dat[1:8])
  if rec == 0x12 and len(dat) >= 8:
    return 10, tuple(chr(b) if 32 <= b < 127 else "?" for b in dat[1:8])
  return None


@dataclass(frozen=True)
class RadarTrack:
  track_id: int
  d_rel: float
  y_rel: float
  v_rel: float
  measured: bool


@dataclass(frozen=True)
class BoschRadarStatus:
  tracks: tuple[RadarTrack, ...] = ()
  hw_fail: bool = False
  sgu_fail: bool = False
  dirty: bool = False
  alerts: tuple[str, ...] = ()
  table_frozen: bool = False
  can_error: bool = False
  radar_fault: bool = False
  radar_unavailable: bool = False
  awd: bool | None = None
  position: int | None = None
  epas_type: int | None = None
  vin: str = ""
  unique_raw: int = 0
  vin_stream_complete: bool = False
  gtw_live: bool = False
  last_sgu_age_s: float | None = None
  last_tracks_age_s: float | None = None
  last_raw_age_s: float | None = None
  vin_f190: str = ""
  vin_chassis: str = ""

  @property
  def health_label(self) -> str:
    if not self.gtw_live and not self.tracks:
      return "WAIT GTW" if self.vin_stream_complete else "WAIT VIN"
    if self.radar_fault or self.hw_fail:
      return "FAULT"
    if self.table_frozen:
      return "FROZEN"
    if self.can_error:
      return "CAN"
    if "vinValidity" in self.alerts or "xwdValidity" in self.alerts or "radPositionMismatch" in self.alerts:
      return "REJECT"
    if self.sgu_fail:
      return "SGU"
    if self.radar_unavailable:
      return "UNAVAIL"
    if not self.tracks:
      return "NO TRACKS"
    return "LIVE"

  @property
  def health_ok(self) -> bool:
    return self.health_label == "LIVE"


class BoschRadarMonitor:
  def __init__(self):
    self._vin = ["."] * 17
    self._status = BoschRadarStatus()
    self._fingerprints: deque[tuple[float, tuple]] = deque()
    self._raw_seen: deque[tuple[float, bytes]] = deque()
    self._last_sgu_t: float | None = None
    self._last_tracks_t: float | None = None
    self._vin_bits = 0
    self._gtw_live = False

  @property
  def status(self) -> BoschRadarStatus:
    return self._status

  def update(self, now: float, can_messages, live_tracks=None) -> BoschRadarStatus:
    hw_fail = self._status.hw_fail
    sgu_fail = self._status.sgu_fail
    dirty = self._status.dirty
    alerts = self._status.alerts
    awd = self._status.awd
    position = self._status.position
    epas_type = self._status.epas_type
    can_error = self._status.can_error
    radar_fault = self._status.radar_fault
    radar_unavailable = self._status.radar_unavailable
    tracks = self._status.tracks

    for addr, dat, src in can_messages:
      dat = bytes(dat)
      if addr == ADDR_VIN_HOST and dat:
        rec = dat[0]
        if rec == 0:
          self._vin_bits |= 1
        elif rec == 1:
          self._vin_bits |= 2
        elif rec == 2:
          self._vin_bits |= 4
        continue
      if not radar_bus(src):
        continue
      if addr == ADDR_SGU:
        hw_fail = _bit(dat, SGU_HW_FAIL_BIT)
        sgu_fail = _bit(dat, SGU_FAIL_BIT)
        dirty = _bit(dat, SGU_DIRTY_BIT)
        self._last_sgu_t = now
      elif addr == ADDR_ALERT:
        alerts = tuple(name for bit_i, name in ALERT_BITS if _bit(dat, bit_i))
      elif addr == ADDR_CAR_CONFIG and len(dat) >= 8:
        awd, position, epas_type = decode_car_config(dat)
        self._gtw_live = True
      elif addr == ADDR_VIN_FEED:
        parsed = vin_mux_chars(dat)
        if parsed is not None:
          start, chars = parsed
          for i, ch in enumerate(chars):
            if start + i < 17:
              self._vin[start + i] = ch
      elif addr == ADDR_POINT0:
        self._raw_seen.append((now, dat[:6]))

    while self._raw_seen and now - self._raw_seen[0][0] > FREEZE_HOLD_S:
      self._raw_seen.popleft()

    if live_tracks is not None:
      errors = getattr(live_tracks, "errors", None)
      if errors is not None:
        can_error = bool(getattr(errors, "canError", False))
        radar_fault = bool(getattr(errors, "radarFault", False))
        radar_unavailable = bool(getattr(errors, "radarUnavailableTemporary", False))
      points = getattr(live_tracks, "points", ())
      tracks = tuple(
        RadarTrack(
          track_id=int(p.trackId),
          d_rel=float(p.dRel),
          y_rel=float(p.yRel),
          v_rel=float(p.vRel),
          measured=bool(p.measured),
        )
        for p in points
      )
      self._last_tracks_t = now
      fingerprint = tuple((t.track_id, round(t.d_rel, 1), round(t.v_rel, 1)) for t in tracks)
      self._fingerprints.append((now, fingerprint))

    while self._fingerprints and now - self._fingerprints[0][0] > FREEZE_HOLD_S:
      self._fingerprints.popleft()

    table_frozen = False
    if self._fingerprints and len(tracks) >= FREEZE_MIN_TRACKS:
      span = now - self._fingerprints[0][0]
      same = all(fp == self._fingerprints[-1][1] for _, fp in self._fingerprints)
      table_frozen = span >= FREEZE_HOLD_S - 1e-3 and same and bool(self._fingerprints[-1][1])

    unique_raw = len({payload for _, payload in self._raw_seen})
    last_raw_age_s = None if not self._raw_seen else now - self._raw_seen[-1][0]
    vin = "".join(self._vin)
    if vin == "." * 17:
      vin = ""

    self._status = BoschRadarStatus(
      tracks=tracks,
      hw_fail=hw_fail,
      sgu_fail=sgu_fail,
      dirty=dirty,
      alerts=alerts,
      table_frozen=table_frozen,
      can_error=can_error,
      radar_fault=radar_fault,
      radar_unavailable=radar_unavailable,
      awd=awd,
      position=position,
      epas_type=epas_type,
      vin=vin,
      unique_raw=unique_raw,
      vin_stream_complete=self._vin_bits == 7,
      gtw_live=self._gtw_live,
      last_sgu_age_s=None if self._last_sgu_t is None else now - self._last_sgu_t,
      last_tracks_age_s=None if self._last_tracks_t is None else now - self._last_tracks_t,
      last_raw_age_s=last_raw_age_s,
    )
    return self._status
