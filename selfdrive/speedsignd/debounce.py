"""Temporal confirm so one frame does not write JSONL or light the HUD.

Both need two reads of the same mph from different frames. A class/OCR
contradiction and a class-only hit are not votes. The window is long
enough for two duty-cycled v8n reads when infer is a few seconds.
Empty-road YOLO max conf is ~0.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace

from openpilot.selfdrive.speedsignd.detect_types import SpeedSign
from openpilot.selfdrive.speedsignd.hud import AGREE_READS, frame_reads, note_read, solo_candidate
from openpilot.selfdrive.speedsignd.weights_manifest import DEBOUNCE_HITS, DEBOUNCE_WINDOW_S, YOLO_MIN_CONF


@dataclass(frozen=True)
class DebounceResult:
  """HUD may accept a first hit; JSONL / confirmed needs `need` hits."""
  hud: list[SpeedSign]
  confirmed: list[SpeedSign]


@dataclass
class SignDebounce:
  """Track hits. JSONL confirms after `need` votes of the same mph."""

  need: int = DEBOUNCE_HITS
  window_s: float = DEBOUNCE_WINDOW_S
  min_conf: float = YOLO_MIN_CONF
  last_raw: list[SpeedSign] = field(default_factory=list)
  _hits: list[tuple[float, int, float, tuple[int, int, int, int]]] = field(default_factory=list)

  def update(self, signs: list[SpeedSign], now: float) -> list[SpeedSign]:
    """Back-compat: two-hit / JSONL confirm only."""
    return self.update_split(signs, now).confirmed

  def update_split(self, signs: list[SpeedSign], now: float) -> DebounceResult:
    # last_raw keeps detector mph (including unconfirmed 65) for infer logs.
    self.last_raw = [s for s in signs if s.conf >= self.min_conf and s.mph > 0]
    self._hits = [h for h in self._hits if now - h[0] <= self.window_s]
    hud: list[SpeedSign] = []
    for s in self.last_raw:
      pending, _pair = frame_reads(s)
      if not pending:
        # Class-only at the solo floor reaches the plate. It is not a JSONL vote.
        if solo_candidate(s) is not None:
          hud.append(s)
        continue
      hud.append(s)
      for mph, conf in pending:
        note_read(self._hits, now, int(mph), float(conf), s.bbox)
    if not hud:
      return DebounceResult(hud=[], confirmed=[])
    counts: dict[int, list] = {}
    for hit in self._hits:
      counts.setdefault(int(hit[1]), []).append(hit)
    ready = [mph for mph, rows in counts.items() if len(rows) >= max(self.need, AGREE_READS)]
    if not ready:
      return DebounceResult(hud=hud, confirmed=[])
    best_mph = max(ready, key=lambda mph: max(row[2] for row in counts[mph]))
    top = max(counts[best_mph], key=lambda row: row[2])
    carrier = hud[-1]
    confirmed = [replace(carrier, mph=int(best_mph), conf=float(top[2]), bbox=top[3])]
    return DebounceResult(hud=hud, confirmed=confirmed)
