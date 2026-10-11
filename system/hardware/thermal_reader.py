"""Off-loop thermal-zone reads for hardwared.

The 2 Hz deviceState loop used to call thermal_config.get_msg() inline. A
sysfs thermal read that sleeps (Oct 11 2026: 12.7 s in one pass) then stops
deviceState and selfdrived raises commIssue. The reads now run in their own
thread; the publish loop only takes the latest snapshot from a queue and
never blocks on a sensor.

If no fresh snapshot arrives for STALE_AFTER_S, the cache reports stale.
hardwared then keeps the last temperatures and thermalStatus (never zeros
or cools them) and runs the fan at full. It escalates to at least
`overheated` (fail hot) only if the last good reading was already "yellow"
(max temp at or above the overheated band's entry temperature, or status
already above ok), or once stale time passes STALE_ESCALATE_S.
"""

import queue
import time
from dataclasses import dataclass, field, fields

from openpilot.common.realtime import DT_HW
from openpilot.common.swaglog import cloudlog

STALE_AFTER_S = 10.0
STALE_ESCALATE_S = 60.0  # stale time (after STALE_AFTER_S) before failing hot
SLOW_ZONE_S = 0.25


@dataclass
class ThermalSnapshot:
  temps: dict            # deviceState field name -> float or list[float]
  t: float               # monotonic time the read finished
  read_s: float = 0.0
  slow_zones: dict = field(default_factory=dict)  # zone name -> seconds


def _zones(thermal_config):
  for f in fields(thermal_config):
    v = getattr(thermal_config, f.name)
    if v is not None:
      yield f.name + "TempC", v


def read_thermal_snapshot(thermal_config, clock=time.monotonic, slow_zone_s=SLOW_ZONE_S) -> ThermalSnapshot:
  """Same values as ThermalConfig.get_msg(), with per-zone timing."""
  t0 = clock()
  temps: dict = {}
  slow: dict = {}

  def timed(zone):
    s = clock()
    val = zone.read()
    dt = clock() - s
    if dt > slow_zone_s:
      slow[getattr(zone, "name", "?")] = dt
    return val

  for key, v in _zones(thermal_config):
    temps[key] = [timed(z) for z in v] if isinstance(v, list) else timed(v)
  t1 = clock()
  if slow:
    ms = {k: round(v * 1000.0, 1) for k, v in slow.items()}
    cloudlog.event("hardwared slow thermal zone", error=True, zones_ms=ms, total_ms=round((t1 - t0) * 1000.0, 1))
  return ThermalSnapshot(temps=temps, t=t1, read_s=t1 - t0, slow_zones=slow)


def thermal_reader_thread(end_event, thermal_config, thermal_queue, period_s=DT_HW, clock=time.monotonic):
  while not end_event.is_set():
    start = clock()
    try:
      snap = read_thermal_snapshot(thermal_config, clock=clock)
      # keep only the newest snapshot
      try:
        thermal_queue.get_nowait()
      except queue.Empty:
        pass
      try:
        thermal_queue.put_nowait(snap)
      except queue.Full:
        pass
    except Exception:
      cloudlog.exception("Error reading thermal zones")
    end_event.wait(max(0.0, period_s - (clock() - start)))


class ThermalCache:
  """Latest thermal snapshot as seen by the publish loop. Never blocks."""

  def __init__(self, thermal_queue, clock=time.monotonic, stale_after_s=STALE_AFTER_S):
    self._q = thermal_queue
    self._clock = clock
    self._stale_after_s = stale_after_s
    self._start = clock()
    self.snapshot: ThermalSnapshot | None = None
    self.stale = False
    self.stale_since: float | None = None

  def poll(self) -> bool:
    try:
      while True:
        self.snapshot = self._q.get_nowait()
    except queue.Empty:
      pass
    ref = self.snapshot.t if self.snapshot is not None else self._start
    stale = (self._clock() - ref) > self._stale_after_s
    now = self._clock()
    if stale != self.stale:
      if stale:
        self.stale_since = now
        cloudlog.event("hardwared thermal stale start", error=True, age_s=round(now - ref, 1))
      else:
        cloudlog.event("hardwared thermal stale end", error=True,
                       stale_s=round(now - (self.stale_since or now), 1))
        self.stale_since = None
    self.stale = stale
    return stale

  @property
  def stale_s(self) -> float:
    """Seconds since the cache went stale (0 when fresh)."""
    return 0.0 if self.stale_since is None else self._clock() - self.stale_since

  @property
  def temps(self) -> dict:
    return {} if self.snapshot is None else self.snapshot.temps


def stale_thermal_status(status, stale: bool, stale_s: float, last_temp: float, yellow_temp: float,
                         ok, overheated, escalate_s: float = STALE_ESCALATE_S):
  """thermalStatus to publish given cache staleness.

  Fresh: unchanged. Stale: keep the last status (never cooler), but fail hot
  (at least `overheated`) if the last good reading was already yellow or
  worse, or once stale for longer than escalate_s.
  """
  if not stale:
    return status
  was_yellow = status > ok or last_temp >= yellow_temp
  if was_yellow or stale_s > escalate_s:
    return max(status, overheated)
  return status
