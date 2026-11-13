"""Red -> green state machine for the stoplight chime. No numpy, no cereal.

Fed one list of lamps (detect.Lamp) per camera frame while the car is
stopped in Drive. The camera does not move while the car is stopped, so a
signal lamp keeps its pixel position. That is the whole trick.

Lamp sanity (lamp_filter), every frame, before tracking:
  size     MIN_LAMP_PX .. MAX_LAMP_PX (a lens, not a sign, truck or bus)
  shape    red aspect <= RED_MAX_ASPECT: a lit lens (and its halo) is round;
           brake-light bars, tail lamps and sign strokes are wide
  bright   a lit lens: peak luma >= RED_MIN_PEAK / GREEN_MIN_PEAK, and a
           red needs a bright core or chroma >= RED_DULL_CHROMA (a dull red
           surface is neither); green needs GREEN_MIN_CHROMA
  height   the lamp sits LOW_K..MAX_RISE_K lamp sizes above the horizon
           (a 20-30 cm lens hangs ~5 m up; brake and tail lights are at or
           below the camera, tower and roof lights far too high for their
           size), and its box does not touch the top edge of the frame
  pairs    two like-sized reds side by side close to the horizon are a car's
           lights, not a signal: both dropped

Heads. Each confirmed red track is one signal head. Its "centrality" is
how far it is from the middle of the frame (0 = dead ahead, 1 = frame
edge). The head most likely to be the driver's own is the most central one.

  red flag    the most central lit head is red (UI chip)
  ding        a confirmed red head went out and a new green came on in the
              SAME head (same column just below it, or beside it for a
              horizontal head, at the same spot it stayed in), within
              GREEN_AFTER_RED_S. Then it dings unless:
                * that head already dinged this cycle (one ding per change),
                * it is a side head (> SIDE_CENTER off center) and a main
                  head is still red (a turn arrow or another approach going
                  green while his light stays red),
                * a clearly more central head already dinged this stop,
                * another head dinged < DING_GAP_S ago (heads of one
                  approach change together: one ding for all).
              A side head that dinged first never blocks a more central head
              that changes later.

The caller owns the gate (Drive, ~0 m/s) and calls reset() on every
un-gate, so the next stop starts clean.

Pixel constants are for the 3X road camera (1928 x 1208). `scale` is frame
width / 1928 and `height` the frame height for other sensors.
"""
from __future__ import annotations

from dataclasses import dataclass, field

REF_WIDTH = 1928.0
REF_HEIGHT = 1208.0

RED_CONFIRM_FRAMES = 4
RED_CONFIRM_S = 0.6
RED_OFF_S = 0.35
RED_FLAG_HOLD_S = 1.2       # the red flag outlives a missed frame or two
# Mean |V-128| of a lamp's colored pixels. Real lamps on Scallywag's drives:
# red 50..115 (dull daytime LEDs sit at 50..60 and always have a bright
# core), green 62..98. Green street signs sit at 45..58.
RED_MIN_CHROMA = 48.0
RED_DULL_CHROMA = 56.0      # a red with no bright core must be this saturated
GREEN_MIN_CHROMA = 60.0
# Brightest luma in the lamp: a lit lens is bright. Real reds 115+, greens
# 157+; red price-sign digits and dull red paint sit at 73..106.
RED_MIN_PEAK = 100
GREEN_MIN_PEAK = 110
GREEN_CONFIRM_FRAMES = 2
GREEN_CONFIRM_S = 0.15
GREEN_AFTER_RED_S = 1.0     # the green comes on within this of the red going out
GREEN_PAIR_S = 2.5          # a red that went out longer ago than this is stale
GREEN_HOLD_S = 1.2
TRACK_GATE_PX = 18.0        # min association radius, full-res px
TRACK_GATE_K = 0.6          # ... or this many lamp sizes
TRACK_DROP_S = 4.0
MAX_TRACKS = 32
# Lamp sanity. Real lenses on Scallywag's drives: 12..86 px (night halos are
# the big ones), red aspect <= 1.5, green <= 1.85 (halo smear).
MIN_LAMP_PX = 10.0
MAX_LAMP_PX = 110.0
RED_MAX_ASPECT = 1.6
GREEN_MAX_ASPECT = 2.0
TOP_EDGE_PX = 4.0
# Horizon row as a fraction of the frame height (3X on Scallywag: pitch
# -0.079 rad puts it at ~row 814 of 1208). Signals sit 3.4..25 lamp sizes
# above it; a lead car's lights sit at or below it.
HORIZON_FRAC = 0.674
LOW_K = 2.5
MAX_RISE_K = 34.0           # higher than this for its size: a light on a tower / roof, not a signal
PAIR_LOW_K = 3.5            # car-light pair check only this close to the horizon
PAIR_DY_K = 0.35
PAIR_DX_MIN_K = 2.0
PAIR_DX_MAX_K = 10.0
PAIR_SIZE_RATIO = 1.5
# Stability: a head (and its green) stays put while the car is stopped.
MAX_JITTER_PX = 4.0
MAX_JITTER_K = 0.2          # mean |offset| <= max(MAX_JITTER_PX, this * size)
# Same head. Scallywag's real pairs: |dx| <= 5 px, dy 32..79 px,
# dy/size 0.5..2.65 (night halos inflate size). A digital billboard flipping
# red -> green content (route 000001f7, Nov 9) was 15 px sideways, 3 px down.
HEAD_DX_PX = 8.0
HEAD_DX_K = 0.3             # |dx| <= max(HEAD_DX_PX, this * red size)
HEAD_DY_PX = 20.0
HEAD_DY_MIN_K = 0.45        # dy >= max(HEAD_DY_PX, this * red size): green below red
HEAD_DY_MAX_K = 3.2         # dy <= this * red size + HEAD_DY_EXTRA_PX
HEAD_DY_EXTRA_PX = 12.0
SIDE_DY_PX = 6.0
SIDE_DY_K = 0.3
SIDE_DX_PX = 20.0
SIDE_DX_MIN_K = 1.2
SIDE_DX_MAX_K = 3.5
# Head arbitration.
CENTER_MARGIN = 0.12        # "clearly more central": this much of the half width
SIDE_CENTER = 0.35          # a head further out than this is a side head (arrows, cross street)
DING_GAP_S = 4.0
REARM_RED_S = 5.0           # a head that dinged must be red this long to ding again
MAX_DINGS = 4               # per stop


@dataclass
class Track:
  color: str
  x: float
  y: float
  size: float
  first_t: float
  last_t: float
  hits: int = 1
  jitter: float = 0.0
  confirmed_t: float | None = None
  dinged: bool = False
  suppressed: bool = False
  green: Track | None = None   # the green this head turned into

  def near(self, lamp, scale: float = 1.0) -> bool:
    gate = max(TRACK_GATE_PX * scale, TRACK_GATE_K * max(self.size, lamp.size))
    return abs(lamp.x - self.x) <= gate and abs(lamp.y - self.y) <= gate

  def absorb(self, lamp, t: float) -> None:
    a = 0.3
    off = abs(lamp.x - self.x) + abs(lamp.y - self.y)
    self.jitter += a * (off - self.jitter)
    self.x += a * (lamp.x - self.x)
    self.y += a * (lamp.y - self.y)
    self.size += a * (lamp.size - self.size)
    self.last_t = t
    self.hits += 1

  def steady(self, scale: float = 1.0) -> bool:
    return self.jitter <= max(MAX_JITTER_PX * scale, MAX_JITTER_K * self.size)


@dataclass
class Output:
  red: bool = False             # the driver's (most central) head is red now (UI flag)
  green: bool = False           # the driver's head turned green this stop (UI flash)
  chime: bool = False           # True on each update a ding fires
  red_count: int = 0            # red lamps that passed the sanity checks this frame
  green_count: int = 0
  red_x: float = -1.0           # full-res px of the driver's red head
  red_y: float = -1.0
  reason: str = ""


def _aspect(lamp) -> float:
  return max(lamp.w, lamp.h) / max(1.0, min(lamp.w, lamp.h))


def _rise(lamp, height: float) -> float:
  """Lamp sizes above the horizon."""
  return (HORIZON_FRAC * height - lamp.y) / max(1.0, lamp.size)


def lamp_ok(lamp, scale: float = 1.0, height: float | None = None) -> bool:
  """Single-lamp sanity: size, shape, brightness, and height in the frame."""
  h = REF_HEIGHT * scale if height is None else height
  if not (MIN_LAMP_PX * scale <= lamp.size <= MAX_LAMP_PX * scale):
    return False
  if lamp.y - lamp.h / 2.0 < TOP_EDGE_PX * scale:
    return False
  if not (LOW_K <= _rise(lamp, h) <= MAX_RISE_K):
    return False
  peak = getattr(lamp, "peak", -1)
  if lamp.color == "red":
    if _aspect(lamp) > RED_MAX_ASPECT or lamp.chroma < RED_MIN_CHROMA:
      return False
    if 0 <= peak < RED_MIN_PEAK:
      return False
    return lamp.core > 0 or lamp.chroma >= RED_DULL_CHROMA
  if 0 <= peak < GREEN_MIN_PEAK:
    return False
  return _aspect(lamp) <= GREEN_MAX_ASPECT and lamp.chroma >= GREEN_MIN_CHROMA


def car_light_pairs(reds, height: float) -> set[int]:
  """Indexes of reds that look like a vehicle's light pair: like-sized, level,
  a few sizes apart, close to the horizon."""
  out: set[int] = set()
  for i, a in enumerate(reds):
    for j in range(i + 1, len(reds)):
      b = reds[j]
      big, small = max(a.size, b.size), max(1.0, min(a.size, b.size))
      if big / small > PAIR_SIZE_RATIO:
        continue
      if abs(a.y - b.y) > PAIR_DY_K * big:
        continue
      if not (PAIR_DX_MIN_K * big <= abs(a.x - b.x) <= PAIR_DX_MAX_K * big):
        continue
      if _rise(a, height) < PAIR_LOW_K and _rise(b, height) < PAIR_LOW_K:
        out.update((i, j))
  return out


def lamp_filter(lamps, scale: float = 1.0, height: float | None = None):
  """(reds, greens) that pass every sanity check."""
  h = REF_HEIGHT * scale if height is None else height
  reds = [lp for lp in lamps if lp.color == "red" and lamp_ok(lp, scale, h)]
  greens = [lp for lp in lamps if lp.color == "green" and lamp_ok(lp, scale, h)]
  bad = car_light_pairs(reds, h)
  if bad:
    reds = [lp for i, lp in enumerate(reds) if i not in bad]
  return reds, greens


def same_head(red: Track, green: Track, scale: float = 1.0) -> bool:
  s = max(red.size, MIN_LAMP_PX * scale)
  dx = green.x - red.x
  dy = green.y - red.y
  if abs(dx) <= max(HEAD_DX_PX * scale, HEAD_DX_K * s) and \
     max(HEAD_DY_PX * scale, HEAD_DY_MIN_K * s) <= dy <= HEAD_DY_MAX_K * s + HEAD_DY_EXTRA_PX * scale:
    return True
  return abs(dy) <= max(SIDE_DY_PX * scale, SIDE_DY_K * s) and \
    max(SIDE_DX_PX * scale, SIDE_DX_MIN_K * s) <= abs(dx) <= SIDE_DX_MAX_K * s


@dataclass
class _Dinged:
  x: float
  y: float
  t: float
  center: float


@dataclass
class StoplightTracker:
  scale: float = 1.0
  height: float | None = None
  reds: list[Track] = field(default_factory=list)
  greens: list[Track] = field(default_factory=list)
  dings: list[_Dinged] = field(default_factory=list)
  green_t: float | None = None
  last_out: Output = field(default_factory=Output)

  @property
  def chimed(self) -> bool:
    return bool(self.dings)

  def reset(self) -> None:
    self.reds.clear()
    self.greens.clear()
    self.dings.clear()
    self.green_t = None
    self.last_out = Output()

  def _height(self) -> float:
    return REF_HEIGHT * self.scale if self.height is None else self.height

  def center(self, x: float) -> float:
    """0 straight ahead, 1 at the frame edge."""
    half = REF_WIDTH * self.scale / 2.0
    return min(1.0, abs(x - half) / half)

  def _assoc(self, tracks: list[Track], lamps, t: float, color: str) -> None:
    for lamp in lamps:
      best = None
      best_d = 0.0
      for tr in tracks:
        if tr.last_t < t and tr.near(lamp, self.scale):
          d = abs(tr.x - lamp.x) + abs(tr.y - lamp.y)
          if best is None or d < best_d:
            best, best_d = tr, d
      if best is not None:
        best.absorb(lamp, t)
      elif len(tracks) < MAX_TRACKS:
        tracks.append(Track(color, lamp.x, lamp.y, lamp.size, t, t))
    # Where a head dinged is kept in self.dings after its track ages out.
    tracks[:] = [tr for tr in tracks if t - tr.last_t <= TRACK_DROP_S]

  def _dinged_here(self, r: Track) -> _Dinged | None:
    for d in self.dings:
      if abs(d.x - r.x) <= max(TRACK_GATE_PX * self.scale, TRACK_GATE_K * r.size) and \
         abs(d.y - r.y) <= max(TRACK_GATE_PX * self.scale, TRACK_GATE_K * r.size):
        return d
    return None

  def _may_ding(self, r: Track, t: float, lit: list[Track]) -> str:
    """'' if head r may ding now, else why not."""
    if len(self.dings) >= MAX_DINGS:
      return "max"
    prev = self._dinged_here(r)
    if prev is not None and (r.confirmed_t is None or r.last_t - r.first_t < REARM_RED_S):
      return "same_head"
    c = self.center(r.x)
    # A side head going green while a main head (more central) stays red is
    # someone else's signal, e.g. a turn arrow. A main head never waits on
    # another head: on Scallywag's drives his light was often a little off
    # center with a red turn head dead ahead.
    if c >= SIDE_CENTER and any(o is not r and self.center(o.x) < SIDE_CENTER and
                                self.center(o.x) + CENTER_MARGIN < c for o in lit):
      return "central_red"
    if any(d.center + CENTER_MARGIN < c for d in self.dings if d is not prev):
      return "central_dinged"
    if self.dings and t - max(d.t for d in self.dings) < DING_GAP_S:
      return "gap"
    return ""

  def update(self, t: float, lamps) -> Output:
    """One camera frame at monotonic time t (s). Lamps from detect.find_lamps."""
    h = self._height()
    reds, greens = lamp_filter(lamps, self.scale, h)
    self._assoc(self.reds, reds, t, "red")
    # Every green is tracked, including ones lit while a red is still lit
    # (a sign, another approach's signal). A tracked green is never "new".
    self._assoc(self.greens, greens, t, "green")

    for tr in self.reds:
      if tr.confirmed_t is None and tr.hits >= RED_CONFIRM_FRAMES and \
         tr.last_t - tr.first_t >= RED_CONFIRM_S and tr.steady(self.scale):
        tr.confirmed_t = t
    confirmed = [tr for tr in self.reds if tr.confirmed_t is not None]
    lit = [tr for tr in confirmed if t - tr.last_t < RED_OFF_S]
    out = Output(red_count=len(reds), green_count=len(greens))

    off = [tr for tr in confirmed if not tr.dinged and not tr.suppressed and
           RED_OFF_S <= t - tr.last_t <= GREEN_PAIR_S + RED_OFF_S]
    # Most central first, so a central head wins a same-frame tie.
    off.sort(key=lambda tr: self.center(tr.x))
    used: set[int] = set()
    for r in off:
      for g in self.greens:
        if id(g) in used or g.hits < GREEN_CONFIRM_FRAMES or g.last_t != t or \
           g.last_t - g.first_t < GREEN_CONFIRM_S or not g.steady(self.scale):
          continue
        if not (r.last_t - 0.25 <= g.first_t <= r.last_t + GREEN_AFTER_RED_S):
          continue
        if not same_head(r, g, self.scale):
          continue
        used.add(id(g))
        why = self._may_ding(r, t, lit)
        r.green = g
        if why:
          r.suppressed = True
          out.reason = out.reason or f"quiet:{why} red({r.x:.0f},{r.y:.0f})"
          break
        r.dinged = True
        prev = self._dinged_here(r)
        if prev is not None:
          self.dings.remove(prev)
        self.dings.append(_Dinged(r.x, r.y, t, self.center(r.x)))
        self.green_t = t
        out.chime = True
        out.reason = f"red({r.x:.0f},{r.y:.0f})->green({g.x:.0f},{g.y:.0f})"
        break
      if out.chime:
        break

    # UI: the driver's head is the most central one that is red now or has
    # turned green this stop (and that green is still on).
    held = [tr for tr in confirmed if t - tr.last_t <= RED_FLAG_HOLD_S and not tr.suppressed]
    went_green = [tr for tr in confirmed if tr.dinged and tr.green is not None and
                  t - tr.green.last_t <= GREEN_HOLD_S]
    heads = [(self.center(tr.x), 0, tr) for tr in held if tr not in went_green] + \
            [(self.center(tr.x), 1, tr) for tr in went_green]
    if heads:
      _, kind, top = min(heads, key=lambda e: (e[0], e[1], e[2].y))
      if kind == 0:
        out.red = True
        out.red_x, out.red_y = top.x, top.y
      else:
        out.green = True
    elif self.dings and self.green_t is not None and t - self.green_t <= GREEN_HOLD_S:
      out.green = True
    self.last_out = out
    return out
