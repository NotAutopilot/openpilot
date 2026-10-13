"""Off-loop system-file reads for hardwared.

Oct 13 2026 11:25 CT (route 0000018e seg 23): the 2 Hz deviceState loop
blocked 12.6 s inside HARDWARE.get_som_power_draw() (bms sysfs read),
deviceState went silent and selfdrived raised commIssue. Same class as the
Oct 7/11 thermal hangs (moved to thermal_reader.py).

Every remaining sysfs / procfs / statvfs / sudo read the publish loop used
to make inline now runs here, in a background thread, with per-read timing.
The publish loop only reads the last cached value from SysReadCache.get(),
which never blocks on I/O. A hung read delays only fresh values; deviceState
keeps publishing with the previous ones.
"""

import threading
import time
from collections.abc import Callable

from openpilot.common.realtime import DT_HW
from openpilot.common.swaglog import cloudlog

SLOW_READ_S = 0.25


class SysReadCache:
  """Latest value per named read, shared between the reader thread and the publish loop."""

  def __init__(self, defaults: dict, clock=time.monotonic):
    self._lock = threading.Lock()
    self._values = dict(defaults)
    self._updated: dict[str, float] = {}
    self._clock = clock

  def set(self, name: str, value) -> None:
    with self._lock:
      self._values[name] = value
      self._updated[name] = self._clock()

  def get(self, name: str, default=None):
    with self._lock:
      return self._values.get(name, default)

  def age_s(self, name: str) -> float | None:
    with self._lock:
      t = self._updated.get(name)
    return None if t is None else self._clock() - t


def read_once(readers: dict[str, Callable], cache: SysReadCache, clock=time.monotonic, slow_read_s=SLOW_READ_S) -> dict:
  """Run each reader once, store results; returns {name: seconds} for slow reads."""
  t0 = clock()
  slow: dict[str, float] = {}
  for name, fn in readers.items():
    s = clock()
    try:
      cache.set(name, fn())
    except Exception:
      cloudlog.exception(f"hardwared sysfs read failed: {name}")
    dt = clock() - s
    if dt > slow_read_s:
      slow[name] = dt
  if slow:
    cloudlog.event("hardwared slow sysfs read", error=True,
                   reads_ms={k: round(v * 1000.0, 1) for k, v in slow.items()},
                   total_ms=round((clock() - t0) * 1000.0, 1))
  return slow


def sysfs_reader_thread(end_event, readers: dict[str, Callable], cache: SysReadCache, period_s=DT_HW, clock=time.monotonic):
  while not end_event.is_set():
    start = clock()
    read_once(readers, cache, clock=clock)
    end_event.wait(max(0.0, period_s - (clock() - start)))
