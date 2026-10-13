"""Curve-follow: the continuous curve term in the longitudinal planner (Pre-AP).

The road ahead is a speed envelope (the unchanged conservative lateral table)
and its most limiting point is a virtual lead. The term is a ceiling on
acceleration: exactly FREE on a straight, lower only when the path ahead
asks, released in proportion as the path opens, with no set speed and no latch.
"""
import json
import math
import time

import numpy as np
import pytest

import openpilot.selfdrive.controls.lib.curve_follow as cfmod
from openpilot.selfdrive.controls.lib.curve_follow import (
  CONF_FALL_1_S,
  FREE_A_MS2,
  LOG_PREFIX,
  MODE_ACTIVE,
  MODE_OFF,
  MODE_SHADOW,
  RELEASE_BEFORE_TIGHTEST_S,
  CurveFollow,
  curve_follow_raw,
  format_log_line,
  read_curve_follow_mode,
)
from openpilot.selfdrive.controls.lib.curve_max_hold import curve_speed_for_curvature
from openpilot.selfdrive.controls.tests.test_curve_preview import _bend_kappa
from openpilot.selfdrive.modeld.constants import ModelConstants

DT = 0.05
T = np.array(ModelConstants.T_IDXS)
TL = T.tolist()


def _road_k(s, bends):
  """Signed curvature at arc length s (left +, right -). bends = (start, length, kappa)."""
  return sum(_bend_kappa(s, *b) for b in bends)


def _path(v, s, bends):
  xs = (v * T).tolist()
  return TL, xs, [_road_k(s + x, bends) for x in xs]


def _step(cf, v, s, bends, **kw):
  ts, xs, ks = _path(v, s, bends)
  return cf.step(dt=DT, v_ego=v, ts=ts, ds=xs, ks=ks, **kw)


def _drive(bends, v0, seconds, v_max=40.0, cruise_gain=0.25, a_cruise_max=1.2, cf=None, **kw):
  """Kinematic car, P cruise toward v_max, curve term as a ceiling (perfect path)."""
  cf = CurveFollow() if cf is None else cf
  s, v = 0.0, float(v0)
  rows = []
  for k in range(int(round(seconds / DT))):
    a_cf = _step(cf, v, s, bends, **kw)
    a_cruise = max(-0.5, min(a_cruise_max, cruise_gain * (v_max - v)))
    a = min(a_cruise, a_cf)
    rows.append((k * DT, s, v, a, a_cf, cf))
    v = max(0.0, v + a * DT)
    s += v * DT
  return rows


def _brake_jerk(a_list):
  """Largest |da/dt| where the ceiling is braking or just released. Above ~0 it is only a bound on +a."""
  worst = 0.0
  for a0, a1 in zip(a_list, a_list[1:], strict=False):
    if a0 <= 0.05 and a1 <= 0.05:
      worst = max(worst, abs(a1 - a0) / DT)
  return worst


def _speed_at(rows, s_target):
  return next(v for _, s, v, *_ in rows if s >= s_target)


def _time_at(rows, s_target):
  return next(t for t, s, *_ in rows if s >= s_target)


# 1 ---------------------------------------------------------------------
@pytest.mark.parametrize("v", [5.0, 8.0, 15.0, 25.0, 35.0])
def test_straight_returns_exactly_free(v):
  cf = CurveFollow()
  rng = np.random.default_rng(7)
  for _ in range(200):
    ks = (rng.normal(0.0, 0.0003, len(TL))).tolist()
    a = cf.step(dt=DT, v_ego=v, ts=TL, ds=(v * T).tolist(), ks=ks, y_std=(0.3 + 0.2 * T).tolist())
    assert a == FREE_A_MS2
  assert cf.raw == FREE_A_MS2 and cf.limit_idx == -1


# 2 ---------------------------------------------------------------------
@pytest.mark.parametrize("radius, v0", [(60.0, 15.0), (100.0, 20.0), (40.0, 12.0), (80.0, 20.0)])
def test_constant_radius_bend_slows_early_gently_and_to_the_table(radius, v0):
  start = 160.0
  bends = [(start, 80.0, 1.0 / radius)]
  rows = _drive(bends, v0, 26.0, v_max=v0)
  v_env = curve_speed_for_curvature(1.0 / radius)
  t_entry = _time_at(rows, start)
  t_on = next(t for t, _, _, a, *_ in rows if a < -0.05)
  assert t_entry - t_on >= 3.0
  assert _speed_at(rows, start) <= v_env * 1.15
  assert min(a for _, _, _, a, *_ in rows) >= -1.05
  assert _brake_jerk([r[4] for r in rows]) <= 2.1
  # Inside the arc the car holds the table speed, it is not pushed below it.
  mid = _speed_at(rows, start + 50.0)
  assert v_env * 0.85 <= mid <= v_env * 1.15


# 3 ---------------------------------------------------------------------
def test_small_deficit_is_a_gentle_lift_not_a_brake():
  v_env = curve_speed_for_curvature(1.0 / 150.0)
  v0 = v_env + 1.2
  rows = _drive([(150.0, 200.0, 1.0 / 150.0)], v0, 14.0, v_max=v0)
  a_cf = [r[4] for r in rows]
  assert min(a_cf) >= -0.45
  assert min(a_cf) < -0.02
  assert _brake_jerk(a_cf) <= 1.2


# 4 ---------------------------------------------------------------------
def test_tightening_spiral_keeps_deepening_within_the_jerk_limit():
  # Radius shrinks 200 -> 35 m over the road: each later point asks for less speed.
  bends = [(120.0 + 45.0 * i, 45.0, 1.0 / r) for i, r in enumerate((200.0, 140.0, 95.0, 65.0, 45.0, 35.0))]
  rows = _drive(bends, 17.0, 45.0, v_max=17.0)
  a = [r[3] for r in rows]
  assert min(a) < -0.3
  k_min = int(np.argmin(a))
  # No release while the road is still tightening.
  rising = [a[i + 1] - a[i] for i in range(k_min)]
  assert max(rising) <= 0.12
  assert _brake_jerk([r[4] for r in rows]) <= 2.1
  # Cornering stays on the table at the tightest arc.
  assert _speed_at(rows, 120.0 + 45.0 * 5 + 20.0) <= curve_speed_for_curvature(1.0 / 35.0) * 1.2


# 5 ---------------------------------------------------------------------
def test_release_after_the_apex_lifts_and_never_brakes_into_the_exit():
  start, length = 130.0, 70.0
  rows = _drive([(start, length, 1.0 / 40.0)], 14.0, 32.0, v_max=14.0)
  end = start + length
  after = [a_cf for _, s, _, _, a_cf, _ in rows if s >= end + 8.0]
  assert after and min(after) >= -0.1
  # By the end of the exit ramp the term does not hold the cruise back.
  last = [a_cf for _, s, _, _, a_cf, _ in rows if s >= end + 40.0]
  assert last and min(last) >= 1.0


def test_proportional_release_rate_is_bounded():
  cf = CurveFollow()
  v = 14.0
  bends = [(60.0, 50.0, 1.0 / 35.0)]
  s = 0.0
  prev = None
  worst_rise = 0.0
  for _ in range(int(14.0 / DT)):
    a = _step(cf, v, s, bends)
    if prev is not None and prev < 0.0:
      worst_rise = max(worst_rise, (a - prev) / DT)
    prev = a
    s += v * DT
  assert worst_rise <= 2.6


# 6 ---------------------------------------------------------------------
def _sbend(gap_s, v=14.0, radius=50.0):
  """Left arc then a right arc whose apex is gap_s later at constant speed."""
  a1 = 90.0
  a2 = a1 + gap_s * v
  return [(a1, 40.0, 1.0 / radius), (a2, 40.0, -1.0 / radius)]


@pytest.mark.parametrize("gap_s", [4.0, 6.0, 8.0, 10.0, 14.0, 20.0])
def test_s_curve_family_regens_back_down_without_chatter_or_latch(gap_s):
  v0 = 11.0
  bends = _sbend(gap_s, v=v0)
  v_env = curve_speed_for_curvature(1.0 / 50.0)
  rows = _drive(bends, v0, 14.0 + gap_s + 8.0, v_max=18.0)
  arc1_end = bends[0][0] + bends[0][1]
  arc2_start = bends[1][0]
  between = [r for r in rows if arc1_end + 10.0 <= r[1] <= arc2_start - 20.0]
  a_cf = [r[4] for r in rows if arc1_end <= r[1] <= arc2_start + 10.0]
  # Speed at each tight arc stays on the table.
  assert _speed_at(rows, bends[0][0] + 20.0) <= v_env * 1.2
  assert _speed_at(rows, bends[1][0] + 20.0) <= v_env * 1.2
  # Sign changes of the ceiling around zero between the two bends (hysteresis band).
  state, flips = 0, 0
  for a in a_cf:
    new = 1 if a > 0.15 else (-1 if a < -0.15 else state)
    if state and new != state:
      flips += 1
    state = new
  assert flips <= 2
  if gap_s >= 10.0:
    assert between and max(r[3] for r in between) >= 0.15  # accelerates out of bend 1
  if gap_s <= 4.0:
    assert not between or max(r[3] for r in between) < 0.15


def test_s_curve_has_no_state_a_fresh_filter_agrees_mid_scene():
  """The demand is a function of the path and speed: a filter started mid-scene agrees after a moment."""
  v0 = 11.0
  bends = _sbend(12.0, v=v0)
  cf = CurveFollow()
  rows = _drive(bends, v0, 12.0, v_max=18.0, cf=cf)
  s_mid, v_mid = rows[-1][1], rows[-1][2]
  fresh = CurveFollow()
  worst = 0.0
  for k in range(int(3.0 / DT)):
    _step(fresh, v_mid, s_mid, bends)
    _step(cf, v_mid, s_mid, bends)
    if k * DT >= 0.5:
      worst = max(worst, abs(fresh.raw - cf.raw))
    s_mid += v_mid * DT
  assert worst <= 0.2


# 7 ---------------------------------------------------------------------
def test_one_frame_spike_barely_moves_the_command():
  v = 15.0
  xs = (v * T).tolist()
  quiet = [0.0] * len(TL)
  spike = list(quiet)
  spike[25] = 0.08
  cf = CurveFollow()
  lo = FREE_A_MS2
  for i in range(40):
    ks = spike if i == 20 else quiet
    lo = min(lo, cf.step(dt=DT, v_ego=v, ts=TL, ds=xs, ks=ks))
  # Sustained, the same reading is a real bend and asks for much more.
  cf2 = CurveFollow()
  hi = FREE_A_MS2
  sustained = [0.0] * 25 + [0.08] * 8
  for _ in range(40):
    hi = min(hi, cf2.step(dt=DT, v_ego=v, ts=TL, ds=xs, ks=sustained))
  assert lo > hi
  assert FREE_A_MS2 - lo <= 0.25 * (FREE_A_MS2 - hi)


def test_strong_reading_ramps_confidence_quickly_after_a_gate():
  cf = CurveFollow()
  v = 15.0
  bend = [(60.0, 80.0, 1.0 / 30.0)]
  for _ in range(30):
    _step(cf, v, 0.0, bend, lane_change=True)
  assert cf.conf < 0.1
  for _ in range(10):  # 0.5 s
    _step(cf, v, 0.0, bend, lane_change=False)
  assert cf.conf >= 0.6


def test_high_position_std_silences_far_points_low_std_trusts_them():
  v = 15.0
  xs = (v * T).tolist()
  ks = [0.0 if x < 90.0 else 1.0 / 45.0 for x in xs]
  noisy, clean = CurveFollow(), CurveFollow()
  for _ in range(40):
    a_noisy = noisy.step(dt=DT, v_ego=v, ts=TL, ds=xs, ks=ks, y_std=[40.0] * len(TL))
    a_clean = clean.step(dt=DT, v_ego=v, ts=TL, ds=xs, ks=ks, y_std=[0.25] * len(TL))
  assert a_noisy == FREE_A_MS2
  assert a_clean < -0.2


def test_lane_change_fades_the_term_out_without_a_step():
  v = 15.0
  bend = [(70.0, 80.0, 1.0 / 40.0)]
  cf = CurveFollow()
  for _ in range(40):
    a0 = _step(cf, v, 0.0, bend)
  assert a0 < -0.2
  seq = [_step(cf, v, 0.0, bend, lane_change=True) for _ in range(30)]
  assert seq[-1] > a0 + 0.5
  assert _brake_jerk([a0] + seq) <= 2.1
  assert cf.conf < 0.1


def test_dropout_while_braking_releases_without_a_surge():
  v = 15.0
  bend = [(70.0, 80.0, 1.0 / 40.0)]
  cf = CurveFollow()
  for _ in range(40):
    a0 = _step(cf, v, 0.0, bend)
  assert a0 < -0.2
  seq = [a0]
  for _ in range(30):
    seq.append(cf.step(dt=DT, v_ego=v, ts=None, ds=None, ks=None, valid=False))
  assert _brake_jerk(seq) <= 2.6
  assert cf.conf <= math.exp(-CONF_FALL_1_S * 0.5) + 0.35
  cf.reset()
  assert cf.conf == 1.0 and cf.a == FREE_A_MS2


# 8 ---------------------------------------------------------------------
@pytest.mark.parametrize("radius, v_max", [(35.0, 14.0), (70.0, 20.0)])
def test_max_stays_a_hard_ceiling_and_the_term_never_raises_the_command(radius, v_max):
  rows = _drive([(150.0, 90.0, 1.0 / radius)], 10.0, 24.0, v_max=v_max)
  for _, _, v, _a, a_cf, _ in rows:
    assert a_cf <= FREE_A_MS2
    assert v <= v_max + 0.6
  # The ceiling only ever lowers: the applied accel is never above the cruise accel.
  assert all(a <= 1.2 + 1e-9 for _, _, _, a, *_ in rows)


# 10 --------------------------------------------------------------------
def test_active_roundabout_funnel_fades_the_term_out():
  v = 12.0
  bend = [(60.0, 90.0, 1.0 / 28.0)]
  cf = CurveFollow()
  for _ in range(40):
    a0 = _step(cf, v, 0.0, bend)
  assert a0 < -0.2
  for _ in range(40):
    a_rb = _step(cf, v, 0.0, bend, rb_active=True)
  assert a_rb > a0 + 0.8 and cf.conf < 0.05


# 11 --------------------------------------------------------------------
def test_low_speed_rules():
  bend_near = [(14.0, 40.0, 1.0 / 15.0)]
  bend_far = [(40.0, 40.0, 1.0 / 15.0)]
  # Below 5 m/s: exactly FREE.
  cf = CurveFollow()
  assert all(_step(cf, 4.0, 0.0, bend_near, blinker=True) == FREE_A_MS2 for _ in range(20))
  # 6 m/s, no blinker: no anticipation at an intersection.
  cf = CurveFollow()
  assert all(_step(cf, 6.0, 0.0, bend_near) == FREE_A_MS2 for _ in range(20))
  # With the blinker a near turn is anticipated ...
  cf = CurveFollow()
  v = 7.5
  out = [_step(cf, v, 0.0, [(10.0, 40.0, 1.0 / 12.0)], blinker=True) for _ in range(40)]
  assert min(out) < -0.2
  # ... but only out to ~3 s: a turn 5 s away is not.
  cf = CurveFollow()
  v = 6.5
  out = [_step(cf, v, 0.0, [(v * 7.0, 40.0, 1.0 / 12.0)], blinker=True) for _ in range(40)]
  assert min(out) == FREE_A_MS2
  assert bend_far


# 12 --------------------------------------------------------------------
def test_mode_param_reader_defaults_to_shadow_and_clamps():
  class _P:
    def __init__(self, value):
      self.value = value

    def get(self, key, return_default=False):
      assert key == "NAPCurveFollow"
      if isinstance(self.value, Exception):
        raise self.value
      return self.value

  assert read_curve_follow_mode(_P(0)) == MODE_OFF
  assert read_curve_follow_mode(_P(1)) == MODE_SHADOW
  assert read_curve_follow_mode(_P(2)) == MODE_ACTIVE
  assert read_curve_follow_mode(_P(7)) == MODE_SHADOW
  assert read_curve_follow_mode(_P(None)) == MODE_SHADOW
  assert read_curve_follow_mode(_P(AssertionError("unknown key"))) == MODE_SHADOW


# 13 --------------------------------------------------------------------
def test_release_buffer_is_one_named_constant_and_is_wired(monkeypatch):
  assert RELEASE_BEFORE_TIGHTEST_S == pytest.approx(1.5)
  v = 15.0
  xs = (v * T).tolist()
  ks = [0.0 if x < 70.0 else 1.0 / 40.0 for x in xs]
  w = [1.0] * len(TL)
  a1 = curve_follow_raw(v, TL, xs, ks, w)[0]
  monkeypatch.setattr(cfmod, "RELEASE_BEFORE_TIGHTEST_S", 2.4)
  a24 = curve_follow_raw(v, TL, xs, ks, w)[0]
  assert a24 < a1 < 0.0


def test_cycle_time_stays_small():
  cf = CurveFollow()
  v = 15.0
  xs = (v * T).tolist()
  ks = [0.0 if x < 70.0 else 1.0 / 40.0 for x in xs]
  ys = (0.3 + 0.2 * T).tolist()
  for _ in range(50):
    cf.step(dt=DT, v_ego=v, ts=TL, ds=xs, ks=ks, y_std=ys)
  t0 = time.perf_counter()
  worst = 0.0
  n = 400
  for _ in range(n):
    t1 = time.perf_counter()
    cf.step(dt=DT, v_ego=v, ts=TL, ds=xs, ks=ks, y_std=ys)
    worst = max(worst, time.perf_counter() - t1)
  mean = (time.perf_counter() - t0) / n
  assert mean < 0.002
  assert worst < 0.01


# 14 --------------------------------------------------------------------
def test_log_line_carries_the_path_summary_and_the_full_arrays():
  cf = CurveFollow()
  v = 15.0
  ts, xs, ks = _path(v, 0.0, [(70.0, 80.0, 1.0 / 40.0)])
  ys = (0.3 + 0.2 * T).tolist()
  for _ in range(30):
    cf.step(dt=DT, v_ego=v, ts=ts, ds=xs, ks=ks, y_std=ys)
  meta = {"bl": 0, "lc": "off", "df": 0.0, "cn": "green", "rb": 0, "ds": [0, 0.97]}
  line = format_log_line(mode=1, t=123.456, v=v, a_ego=0.1, v_cruise_kph=64.4, a_target=-0.2, cf=cf, old_a=-0.3,
                         ts=ts, ds=xs, ks=ks, y_std=ys, meta=meta, arrays=True)
  assert line.startswith(LOG_PREFIX)
  rec = json.loads(line[len(LOG_PREFIX):])
  assert rec["m"] == 1 and rec["mx"] == pytest.approx(64.4) and rec["old"] == pytest.approx(-0.3)
  assert rec["cf"] == pytest.approx(cf.a, abs=1e-3) and rec["cfr"] == pytest.approx(cf.raw, abs=1e-3)
  assert len(rec["k"]) == 9 and len(rec["ys"]) == 3
  assert len(rec["K"]) == len(rec["D"]) == len(rec["T"]) == len(rec["Y"]) == 33
  assert rec["li"] == cf.limit_idx and rec["w"] == pytest.approx(cf.conf, abs=1e-3)
  quiet = format_log_line(mode=1, t=1.0, v=v, a_ego=0.0, v_cruise_kph=64.4, a_target=0.0, cf=cf, old_a=2.0,
                          ts=ts, ds=xs, ks=ks, y_std=None, meta=meta, arrays=False)
  assert "\"K\"" not in quiet and "\"k\"" in quiet


# 15 --------------------------------------------------------------------
def test_zero_mean_curvature_noise_does_not_accumulate_into_a_bend():
  """Signed accumulation: noise around zero walks, it does not build a displacement."""
  cf = CurveFollow()
  rng = np.random.default_rng(7)
  lo = FREE_A_MS2
  for _ in range(200):
    ks = rng.normal(0.0, 0.0015, len(TL)).tolist()
    lo = min(lo, cf.step(dt=DT, v_ego=20.0, ts=TL, ds=(20.0 * T).tolist(), ks=ks, y_std=(0.3 + 0.2 * T).tolist()))
  assert lo >= 1.9


def test_curvature_filter_clips_a_spike_and_opens_slowly():
  n = len(TL)
  steady = [0.0] * 10 + [0.02] * (n - 10)
  cf = CurveFollow()
  for _ in range(20):
    cf._filter_k(steady, DT)
  spike = list(steady)
  spike[25] = 0.10
  assert cf._filter_k(spike, DT)[25] < 0.03  # clipped to twice its neighbours, then filtered
  cf = CurveFollow()
  for _ in range(20):
    cf._filter_k(steady, DT)
  gone = [0.0] * n
  assert cf._filter_k(gone, DT)[25] >= 0.017  # one frame of "it opens" barely moves it
  assert cf._filter_k([0.0] * n, DT)[25] > 0.014


def test_dropped_frames_fade_the_term_out_and_light_drops_do_not():
  v = 15.0
  bend = [(60.0, 80.0, 1.0 / 30.0)]
  cf = CurveFollow()
  for _ in range(40):
    _step(cf, v, 0.0, bend, frame_drop_pct=30.0)
  assert cf.conf < 0.1 and not cf.trusted
  cf = CurveFollow()
  for _ in range(40):
    _step(cf, v, 0.0, bend, frame_drop_pct=10.0)
  assert cf.conf > 0.99 and cf.trusted


def test_near_field_term_is_a_lift_not_a_brake():
  n = len(TL)
  ts, ds = TL, (25.0 * T).tolist()
  a, idx, _, near = curve_follow_raw(25.0, ts, ds, [0.05] * n, [1.0] * n, 0.0)
  assert near is not None and near < 0.0
  assert near >= -0.35 - 1e-9


def test_apex_detection_reads_the_path_peaking():
  v = 15.0
  n = len(TL)
  ds = (v * T).tolist()
  cf = CurveFollow()
  peaking = [0.03 * max(0.0, 1.0 - float(t) / 2.0) for t in TL]  # k falls away within 2 s
  for _ in range(5):
    cf.step(dt=DT, v_ego=v, ts=TL, ds=ds, ks=peaking)
  assert cf.passed > 0.9
  cf = CurveFollow()
  for _ in range(5):
    cf.step(dt=DT, v_ego=v, ts=TL, ds=ds, ks=[0.03] * n)
  assert cf.passed == 0.0


def test_braking_never_exceeds_the_speed_dependent_decel_limit():
  from openpilot.selfdrive.controls.lib.curve_preview import preview_decel_limit_ms2
  v = 30.0
  cf = CurveFollow()
  ds = (v * T).tolist()
  for _ in range(60):
    a = cf.step(dt=DT, v_ego=v, ts=TL, ds=ds, ks=[0.0] * 3 + [0.06] * (len(TL) - 3))
    assert cf.raw >= -preview_decel_limit_ms2(v) - 1e-9
    assert a >= -preview_decel_limit_ms2(v) - 1e-9
  assert cf.raw == pytest.approx(-preview_decel_limit_ms2(v))  # the road asks for more; the limit binds
