"""HUD chip for the stoplight-green chime (napVision.stoplight). No raylib here.

  RED LIGHT   while stoplightd sees a confirmed red ahead and the car is
              stopped in Drive.
  GREEN - GO  for GREEN_FLASH_S after the red turned green (the ding).

Nothing shows while moving, with the toggle Off, or when napVision has
gone quiet for STALE_S (stoplightd not running).
"""
from __future__ import annotations

RED_TEXT = "RED LIGHT"
GREEN_TEXT = "GREEN - GO"
RED_RGBA = (255, 72, 60, 255)
GREEN_RGBA = (70, 225, 120, 255)
GREEN_FLASH_S = 3.0
STALE_S = 2.5


class StoplightChip:
  def __init__(self):
    self._green_since: float | None = None

  def update(self, msg, age_s: float, now: float) -> tuple[str, tuple[int, int, int, int]] | None:
    """(text, rgba) to draw, or None. age_s: seconds since the last napVision."""
    try:
      live = msg is not None and age_s <= STALE_S and bool(msg.enabled) and bool(msg.gated)
      green = live and bool(msg.green)
      red = live and bool(msg.redLight)
    except Exception:
      live = green = red = False
    if not live:
      self._green_since = None
      return None
    if green:
      if self._green_since is None:
        self._green_since = now
      if now - self._green_since <= GREEN_FLASH_S:
        return GREEN_TEXT, GREEN_RGBA
      return None
    self._green_since = None
    if red:
      return RED_TEXT, RED_RGBA
    return None


def chip_from_sm(chip: StoplightChip, sm, now: float):
  """Read napVision.stoplight from a SubMaster. now is time.monotonic() (SubMaster recv_time clock)."""
  try:
    if not sm.seen.get("napVision", False):
      return chip.update(None, 1e9, now)
    age = now - float(sm.recv_time["napVision"])
    vision = sm["napVision"]
    light = None if vision is None else vision.stoplight
    return chip.update(light, age, now)
  except Exception:
    return None
