#!/usr/bin/env python3
"""Write light_green.wav: a soft two-note "ding-dong" up, played once.

Distinct from every other sound on the car: stock engage is one falling
~1661 Hz tone, the animal chime is G6 -> C7 twice. This is E6 (1318.5 Hz)
then A6 (1760 Hz), bell-like (fundamental plus a quiet 2.76x partial),
long decay, about 0.9 s. 48 kHz mono 16-bit, peak near full scale so the
small speaker carries it over road noise.
"""
from __future__ import annotations

import math
import struct
import wave
from pathlib import Path

SAMPLE_RATE = 48000
PEAK = 0.95


def _bell(freq: float, duration: float, decay: float) -> list[float]:
  n = int(duration * SAMPLE_RATE)
  attack = 0.004
  out = []
  for i in range(n):
    t = i / SAMPLE_RATE
    env = 0.5 * (1.0 - math.cos(math.pi * t / attack)) if t < attack else math.exp(-(t - attack) / decay)
    s = math.sin(2.0 * math.pi * freq * t)
    s += 0.35 * math.sin(2.0 * math.pi * 2.0 * freq * t) * math.exp(-t / (decay * 0.5))
    s += 0.18 * math.sin(2.0 * math.pi * 2.76 * freq * t) * math.exp(-t / (decay * 0.35))
    out.append(env * s)
  return out


def build() -> list[float]:
  first = _bell(1318.51, 0.30, 0.10)
  second = _bell(1760.00, 0.60, 0.18)
  raw = first + second
  peak = max(abs(s) for s in raw) or 1.0
  return [PEAK * s / peak for s in raw]


def write(path: Path) -> None:
  samples = build()
  pcm = b"".join(struct.pack("<h", max(-32767, min(32767, int(s * 32767.0)))) for s in samples)
  with wave.open(str(path), "w") as handle:
    handle.setnchannels(1)
    handle.setsampwidth(2)
    handle.setframerate(SAMPLE_RATE)
    handle.writeframes(pcm)


if __name__ == "__main__":
  dest = Path(__file__).with_name("light_green.wav")
  write(dest)
  audio = build()
  print(dest, len(audio) / SAMPLE_RATE, max(abs(s) for s in audio))
