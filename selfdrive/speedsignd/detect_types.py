"""Shared MUTCD mph set and detection record. No I/O."""
from __future__ import annotations

from dataclasses import dataclass, replace

# US R2-1 posted speeds. School-zone 15/25 and freeway 70/75/80 included.
MUTCD_MPH = frozenset(list(range(5, 90, 5)) + [100])


@dataclass(frozen=True)
class SpeedSign:
  mph: int
  conf: float
  bbox: tuple[int, int, int, int]  # x, y, w, h in the input frame
  # YOLO class vs crop digit-read. HUD mph may be the refine override.
  class_mph: int | None = None
  class_conf: float = 0.0
  alt_mph: int | None = None
  alt_conf: float = 0.0
  refine_mph: int | None = None
  refine_conf: float = 0.0


def shift_sign(sign: SpeedSign, dx: int, dy: int) -> SpeedSign:
  """Translate bbox; keep class/refine fields for logs."""
  x, y, w, h = sign.bbox
  return replace(sign, bbox=(x + int(dx), y + int(dy), w, h))
