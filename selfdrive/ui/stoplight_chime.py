"""Sound id for the stoplight-green ding.

Kept out of soundd so tests can import it without cereal. stoplightd sets
napVision.stoplight.chime on the one message where a red signal turned green
while the car sat stopped in Drive; soundd plays light_green.wav once.
It never interrupts a selfdrived alert sound that is still playing.
"""
from __future__ import annotations

import math

STOPLIGHT_CHIME_ID = -201
STOPLIGHT_CHIME_FILE = "light_green.wav"
# Quiet-cabin scaling cannot drop the ding below this level.
STOPLIGHT_CHIME_MIN_VOLUME = 0.7


def should_ding(msg) -> bool:
  """True for a napVision.stoplight group that carries the ding."""
  try:
    return bool(msg.enabled) and bool(msg.chime)
  except Exception:
    return False


def floor_volume(alert_id, volume: float) -> float:
  if alert_id != STOPLIGHT_CHIME_ID:
    return volume
  try:
    level = float(volume)
  except (TypeError, ValueError):
    level = 0.0
  if not math.isfinite(level):
    level = 0.0
  return max(level, STOPLIGHT_CHIME_MIN_VOLUME)
