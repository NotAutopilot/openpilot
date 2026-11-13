"""Stopped-in-Drive gate for stoplightd. No cereal imports.

The gate reads two services and does not subscribe to carState:

  speed  livePose.velocityDevice (locationd, 20 Hz). At a stop it sits at
         |v| < 0.02 m/s on logged Pre-AP Model S drives.
  gear   onroadEvents (selfdrived, on change and 1 Hz). card raises
         wrongGear for Park / Neutral / unknown and reverseGear for R, so
         "neither event" means Drive.

Open after STILL_HOLD_S at |v| < STILL_V; close as soon as |v| > MOVE_V,
the gear leaves Drive, or either input goes stale. The hysteresis keeps a
creep in a queue from flapping it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

STILL_V = 0.25          # m/s
MOVE_V = 0.8            # m/s
STILL_HOLD_S = 1.0
POSE_STALE_S = 0.5
EVENTS_STALE_S = 3.0    # onroadEvents is sent at least once a second
NOT_DRIVE_EVENTS = frozenset({"wrongGear", "reverseGear"})


def speed_from_pose(vx: float, vy: float, valid: bool = True) -> float | None:
  if not valid:
    return None
  try:
    x, y = float(vx), float(vy)
  except (TypeError, ValueError):
    return None
  if not (math.isfinite(x) and math.isfinite(y)):
    return None
  return math.hypot(x, y)


def in_drive_from_events(names) -> bool:
  return not any(str(n) in NOT_DRIVE_EVENTS for n in names)


@dataclass
class StoppedInDrive:
  speed: float | None = None
  speed_t: float = -1e9
  drive: bool = False
  drive_t: float = -1e9
  still_since: float | None = None
  open: bool = False

  def on_pose(self, t: float, speed: float | None) -> None:
    self.speed = speed
    self.speed_t = t

  def on_events(self, t: float, names) -> None:
    self.drive = in_drive_from_events(names)
    self.drive_t = t

  def update(self, t: float) -> bool:
    fresh = (t - self.speed_t) <= POSE_STALE_S and (t - self.drive_t) <= EVENTS_STALE_S
    if not fresh or self.speed is None or not self.drive or self.speed > MOVE_V:
      self.still_since = None
      self.open = False
      return False
    if self.speed < STILL_V:
      if self.still_since is None:
        self.still_since = t
      if t - self.still_since >= STILL_HOLD_S:
        self.open = True
    elif not self.open:
      self.still_since = None
    return self.open
