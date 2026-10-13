"""Curve handling from true curvature and the model path (Pre-AP).

Lateral target vs speed, preview slowing 0–5 s ahead that can only lower
the command, and a turn clip that returns +a near the apex.
"""
import math

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.curve_max_hold import (
  curve_lat_target_ms2,
  curve_restore_a_ms2,
  curve_speed_for_curvature,
)
from openpilot.selfdrive.controls.lib.curve_preview import (
  PREVIEW_FREE_A_MS2,
  PREVIEW_LAG_S,
  CurvePreview,
  curve_preview_accel,
  path_curvature,
  path_lat_accel_ahead,
  turn_accel_limit,
)
from openpilot.selfdrive.controls.lib.longitudinal_planner import limit_accel_in_turns
from openpilot.selfdrive.controls.tests.test_unified_lead_planner import (
  _make_preap_params,
  _planner,
)
from openpilot.selfdrive.modeld.constants import ModelConstants

DT = 0.05
T = np.array(ModelConstants.T_IDXS)


def _bend_kappa(s, start, length, kappa, ramp=15.0):
  """Curvature at arc length s: clothoid ramps in/out around a constant arc."""
  if s < start - ramp or s > start + length + ramp:
    return 0.0
  if s < start:
    return kappa * (s - (start - ramp)) / ramp
  if s <= start + length:
    return kappa
  return kappa * (start + length + ramp - s) / ramp


def _path(v, s0, bend):
  xs = (v * T).tolist()
  ks = [_bend_kappa(s0 + x, *bend) for x in xs]
  return T.tolist(), xs, ks


def _set_model_path(model, v, s0, bend):
  _, xs, ks = _path(v, s0, bend)
  model.position.x = xs
  model.position.t = T.tolist()
  model.velocity.x = [float(v)] * len(xs)
  model.orientationRate.z = [k * v for k in ks]


def test_lateral_target_vs_speed():
  assert 2.30 <= curve_lat_target_ms2(8.0) <= 2.50
  assert 2.20 <= curve_lat_target_ms2(15.0) <= 2.30
  assert curve_lat_target_ms2(22.0) == pytest.approx(2.0, abs=0.02)
  assert curve_lat_target_ms2(30.0) == pytest.approx(1.75, abs=0.02)
  # Comfort speed solves v²|κ| = A(v).
  for r in (30.0, 60.0, 150.0, 400.0):
    v = curve_speed_for_curvature(1.0 / r)
    assert v * v / r == pytest.approx(curve_lat_target_ms2(v), rel=0.02)
  assert curve_speed_for_curvature(0.0) is None


def test_restore_rate_is_speed_dependent():
  assert curve_restore_a_ms2(10.0) == pytest.approx(0.65, abs=0.01)
  assert curve_restore_a_ms2(30.0) == pytest.approx(0.35, abs=0.01)
  assert curve_restore_a_ms2(10.0) > curve_restore_a_ms2(20.0) > curve_restore_a_ms2(30.0)


def test_path_curvature_reads_model_yaw_rate():
  planner, inputs, _ = _planner(15.0)
  _set_model_path(inputs["modelV2"], 15.0, 0.0, (0.0, 500.0, 1.0 / 50.0, 1.0))
  ts, xs, ks = path_curvature(inputs["modelV2"])
  assert len(ts) == len(xs) == len(ks)
  assert ks[5] == pytest.approx(1.0 / 50.0, rel=1e-6)
  assert path_lat_accel_ahead(15.0, ts, ks) == pytest.approx(225.0 / 50.0, rel=1e-3)


def _drive_bend(v0, bend, seconds, cruise_a=0.0):
  """Kinematic car on a straight-then-bend road, preview only (perfect path)."""
  prev = CurvePreview()
  s = 0.0
  v = v0
  out = []
  for k in range(int(round(seconds / DT))):
    ts, xs, ks = _path(v, s, bend)
    a = min(cruise_a, prev.update(v, curve_preview_accel(v, ts, xs, ks), DT))
    out.append((k * DT, s, v, a))
    v = max(0.0, v + a * DT)
    s += v * DT
  return out


@pytest.mark.parametrize("kappa, entry_ratio", [
  (1.0 / 60.0, 1.08),
  # Tighter than the town decel cap can reach inside the 5 s horizon.
  (1.0 / 40.0, 1.20),
])
def test_town_bend_preview_slows_early_and_gently(kappa, entry_ratio):
  v0 = 15.0
  start = 150.0
  trace = _drive_bend(v0, (start, 60.0, kappa), 16.0)
  v_comfort = curve_speed_for_curvature(kappa)
  t_entry = next(t for t, s, _, _ in trace if s >= start)
  v_entry = next(v for _, s, v, _ in trace if s >= start)
  t_onset = next(t for t, _, _, a in trace if a < -0.05)
  assert t_entry - t_onset >= 3.5
  assert v_entry <= v_comfort * entry_ratio
  assert v_entry <= v0 - 2.0
  accels = [a for _, _, _, a in trace]
  assert min(accels) >= -1.05
  jerks = np.diff(accels) / DT
  assert np.max(np.abs(jerks)) <= 1.2


def test_highway_bend_preview_is_a_lift():
  trace = _drive_bend(30.0, (400.0, 200.0, 1.0 / 400.0), 20.0)
  accels = [a for _, _, _, a in trace]
  # Oct 13: cap about -0.8 at every speed (was -0.35 on the highway).
  assert min(accels) >= -0.81
  assert min(accels) <= -0.1
  # The raw ceiling itself is clamped: a tight bend close ahead never asks past the cap.
  raw = curve_preview_accel(30.0, [1.0, 2.0, 3.0], [30.0, 60.0, 90.0], [1.0 / 30.0] * 3)
  assert -0.81 <= raw <= -0.7


def _drive_lagged(v0, bend, seconds, lag_s=1.5):
  """Car follows the ceiling with the ~1.5 s pedal/regen lag seen on Oct 13."""
  prev = CurvePreview()
  v, s, out, cmds = v0, 0.0, [], []
  for k in range(int(seconds / DT)):
    ts, xs, ks = _path(v, s, bend)
    cmd = min(0.0, prev.update(v, curve_preview_accel(v, ts, xs, ks), DT))
    cmds.append(cmd)
    a = cmds[max(0, len(cmds) - 1 - int(lag_s / DT))]
    out.append((k * DT, s, v, cmd, a))
    v = max(0.0, v + a * DT)
    s += v * DT
  return out


@pytest.mark.parametrize("v0, radius", [(22.0, 200.0), (24.0, 250.0), (15.0, 60.0), (30.0, 400.0)])
def test_oct13_slowing_starts_before_turn_in_and_is_done_by_the_apex(v0, radius):
  """Oct 13 18f bends: slowing began 0.6–6 s AFTER turn-in at -0.35…-0.43.

  Now the car itself (after 1.5 s lag) is slowing 2–3.5 s before turn-in,
  never harder than about -0.8, and reaches the comfort speed by the apex.
  """
  kappa, start, ramp = 1.0 / radius, 600.0, 40.0
  trace = _drive_lagged(v0, (start, 150.0, kappa, ramp), 60.0)
  t_turn_in = next(t for t, s, *_ in trace if s >= start - ramp)
  t_car = next(t for t, _, _, _, a in trace if a < -0.3)
  assert 2.0 <= t_turn_in - t_car <= 3.6
  assert min(r[3] for r in trace) >= -0.81
  v_apex = next(v for _, s, v, *_ in trace if s >= start)
  assert v_apex <= curve_speed_for_curvature(kappa) * 1.02
  assert PREVIEW_LAG_S == pytest.approx(1.5)


def test_straight_path_preview_is_free():
  ts, xs, ks = _path(25.0, 0.0, (1e6, 1.0, 0.0))
  assert curve_preview_accel(25.0, ts, xs, ks) == PREVIEW_FREE_A_MS2


def test_preview_sweep_has_no_jump():
  """A bend sliding through the horizon never steps the ceiling.

  Raw: continuous at 2 cm resolution. Filtered at 20 m/s (1 m per frame):
  the braking part moves at comfort jerk at most.
  """
  v = 20.0
  bend = (200.0, 80.0, 1.0 / 60.0)
  prev = None
  for i in range(16000):
    ts, xs, ks = _path(v, i * 0.02, bend)
    a = curve_preview_accel(v, ts, xs, ks)
    if prev is not None:
      assert abs(a - prev) < 0.04
    prev = a
  filt = CurvePreview()
  prev_brake = 0.0
  saw_brake = False
  for i in range(320):
    ts, xs, ks = _path(v, i * v * DT, bend)
    out = filt.update(v, curve_preview_accel(v, ts, xs, ks), DT)
    brake = min(0.0, out)
    assert abs(brake - prev_brake) <= 1.0 * DT + 1e-9
    saw_brake |= brake < -0.2
    prev_brake = brake
  assert saw_brake


@pytest.mark.parametrize("mpc_accel", [-0.3, 0.0, 0.8])
def test_planner_preview_only_lowers_the_command(mpc_accel):
  v = 20.0
  bend, bend_in, _ = _planner(v, accel=mpc_accel)
  plain, plain_in, _ = _planner(v, accel=mpc_accel)
  s = 0.0
  lowered = False
  for _ in range(160):
    _set_model_path(bend_in["modelV2"], v, s, (120.0, 60.0, 1.0 / 45.0))
    _set_model_path(plain_in["modelV2"], v, s, (1e6, 1.0, 0.0))
    bend.update(bend_in)
    plain.update(plain_in)
    a_b = float(bend.output_a_target)
    a_p = float(plain.output_a_target)
    assert a_b <= a_p + 1e-9
    assert bend.curve_preview_a <= PREVIEW_FREE_A_MS2
    lowered |= a_b < a_p - 0.1
    s += v * DT
  assert lowered


def test_turn_clip_returns_accel_near_the_apex():
  cp = _make_preap_params()
  v = 15.0
  # Apex: 2.3 m/s² now, path 1 s ahead already unwinding to 0.6.
  near_apex = turn_accel_limit(v, 2.3, 0.6)
  assert near_apex >= 1.0
  # Still tightening ahead: little +a.
  assert turn_accel_limit(v, 2.3, 2.4) <= 0.5
  # The path ahead can only loosen the clip relative to now, never tighten it.
  assert turn_accel_limit(v, 1.0, 2.5) == pytest.approx(turn_accel_limit(v, 1.0, None))
  # Stock steer-model table (same corner at 15.75 ratio) gave ~nothing here.
  steer = math.degrees(2.3 * cp.steerRatio * cp.wheelbase / (v * v))
  old = limit_accel_in_turns(v, steer, [-3.5, 2.0], cp)[1]
  new = limit_accel_in_turns(v, steer, [-3.5, 2.0], cp, a_y=2.3, a_y_ahead=0.6)[1]
  assert old <= 0.6
  assert new >= 1.0
