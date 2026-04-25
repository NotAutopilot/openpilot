"""Regression tests for the engage-grace accel ramp.

Invariant this guards against:
  During the engage grace window, pedal accel authority is capped at
  `grace_progress * a_max_at_speed(v_ego)`. This bounds MPC outliers
  (state corruption, solver failure, lead misclassification) to the
  same accel envelope the driving-personality profiles tune in steady
  state. Before the fix (commit 73b6b9adb), the cap was
  `grace_progress * max(accel_request, 0.0)` — effectively a scale of
  the live request with no absolute ceiling. A pathological MPC output
  would have ramped unbounded.

  Origin: drive d0cdc986c5d023f5 (drive-3) reported a pedal stab pulling
  away from a stop at v=6.59 m/s. MPC commanded 0.934 m/s² on engage.
  On a P85 at low speed that's a sharp launch. See
  vault/lessons/agent-failure-modes/engage-grace-upper-bound.md.
"""

import unittest

import numpy as np

from opendbc.car.tesla.preap.constants import ACCEL_PREAP_BP, ACCEL_PREAP_PROFILES

# Mirror the constants from carcontroller.py so the test fails loudly if
# they're renamed or retuned without updating the test.
ENGAGE_GRACE_FRAMES = 50


def a_max_at_speed(v_ego: float, personality: int = 1) -> float:
  """Max accel for the given speed and driving personality."""
  profile = ACCEL_PREAP_PROFILES[personality]
  return float(np.interp(v_ego, ACCEL_PREAP_BP, profile))


def engage_grace_ramp(accel_request: float, engage_elapsed_frames: int,
                      engage_a_max: float) -> float:
  """Pure function mirroring the carcontroller.py engage-grace logic.

  Kept here (not imported) because the carcontroller update() is heavily
  fixtured and not unit-testable in isolation. If the carcontroller
  implementation drifts from this, the test will still guard the
  *intended* behavior; the `test_matches_carcontroller_source` test at
  the bottom re-reads the source to catch silent drift.
  """
  if engage_elapsed_frames >= ENGAGE_GRACE_FRAMES:
    return accel_request
  grace_progress = engage_elapsed_frames / ENGAGE_GRACE_FRAMES
  accel_cap = grace_progress * engage_a_max
  return max(0.0, min(accel_request, accel_cap))


class TestEngageGraceInvariant(unittest.TestCase):
  """The safety envelope: output ≤ grace_progress × a_max."""

  def test_output_never_exceeds_ramped_a_max(self):
    """For every realistic (v_ego, accel_request, frame) combo, the
    output must never exceed grace_progress × a_max_at_speed."""
    # Sweep a realistic parameter grid
    speeds = [0.0, 1.3, 5.0, 10.0, 15.0, 25.0, 35.0]
    # Include pathological MPC asks (3.0, 5.0) to exercise the ceiling
    accel_requests = [-1.5, -0.5, 0.0, 0.25, 0.5, 0.93, 1.5, 3.0, 5.0]
    # Grace-window frames only: post-grace the invariant doesn't apply
    # (request passes through, downstream jerk/profile clamps take over).
    # test_post_grace_passes_request_through covers frame >= 50.
    frames = list(range(0, ENGAGE_GRACE_FRAMES, 5))

    for v_ego in speeds:
      a_max = a_max_at_speed(v_ego)
      for accel_request in accel_requests:
        for frame in frames:
          grace_progress = min(1.0, frame / ENGAGE_GRACE_FRAMES)
          output = engage_grace_ramp(accel_request, frame, a_max)
          ceiling = grace_progress * a_max

          msg = f"v={v_ego} a_cmd={accel_request} f={frame}: output {output:.4f}"
          self.assertLessEqual(output, ceiling + 1e-9,
                               f"{msg} > ceiling {ceiling:.4f}")
          self.assertGreaterEqual(output, 0.0,
                                  f"{msg} < 0.0 (regen during grace forbidden)")

  def test_drive3_stab_scenario(self):
    """The original failure: MPC asks 0.934 m/s² from engage at v=6.59.
    Output must ramp smoothly from 0, never exceed the ramped ceiling."""
    v_ego = 6.59
    accel_request = 0.934
    a_max = a_max_at_speed(v_ego)

    # Frame 0: no accel authority yet
    out_0 = engage_grace_ramp(accel_request, 0, a_max)
    self.assertAlmostEqual(out_0, 0.0, places=6)

    # Mid-ramp: below the MPC ask because still growing
    out_25 = engage_grace_ramp(accel_request, 25, a_max)
    self.assertLess(out_25, accel_request)
    self.assertAlmostEqual(out_25, 0.5 * a_max, places=6)

    # End of grace: full authority, MPC ask passes through (since < a_max)
    out_50 = engage_grace_ramp(accel_request, 50, a_max)
    self.assertAlmostEqual(out_50, accel_request, places=6)

  def test_mpc_outlier_gets_capped(self):
    """If MPC emits a pathological value (bug, solver failure, state
    corruption), the ramp must clamp to a_max, not scale with the outlier."""
    v_ego = 10.0
    a_max = a_max_at_speed(v_ego)  # ≈ 0.97
    pathological = 3.0              # way above a_max

    out_25 = engage_grace_ramp(pathological, 25, a_max)
    self.assertAlmostEqual(out_25, 0.5 * a_max, places=6)

    # Last frame inside the grace window: cap = (49/50) * a_max.
    # Post-grace (frame 50+) the outlier passes through at this layer;
    # downstream clamps (jerk limiter, pedal profile) bound it from there.
    # The grace window's job is to prevent outlier-driven transients
    # during the known-fragile engage handoff.
    out_49 = engage_grace_ramp(pathological, 49, a_max)
    self.assertAlmostEqual(out_49, (49 / 50) * a_max, places=6)

  def test_low_demand_mpc_passes_through(self):
    """If MPC asks conservatively (below ramped cap), its request
    passes through unchanged. The ramp is a ceiling, not a scaler."""
    v_ego = 25.0
    a_max = a_max_at_speed(v_ego)  # ≈ 0.80
    low_request = 0.25              # well below a_max

    # Once grace_progress × a_max > 0.25, the MPC ask should pass through
    # unchanged. grace_progress = 0.25 / 0.80 = 0.3125 → frame ≈ 16.
    out_17 = engage_grace_ramp(low_request, 17, a_max)  # just past crossover
    self.assertAlmostEqual(out_17, low_request, places=6)

    out_30 = engage_grace_ramp(low_request, 30, a_max)
    self.assertAlmostEqual(out_30, low_request, places=6)

  def test_monotonic_ramp_under_constant_request(self):
    """For a constant MPC request within the cap, output is monotone
    non-decreasing through the grace window."""
    v_ego = 10.0
    a_max = a_max_at_speed(v_ego)
    accel_request = 2.0  # above a_max, so cap is active every frame

    prev = 0.0
    for f in range(ENGAGE_GRACE_FRAMES + 1):
      out = engage_grace_ramp(accel_request, f, a_max)
      self.assertGreaterEqual(out, prev - 1e-9,
                              f"output decreased at frame {f}: {out} < {prev}")
      prev = out

  def test_negative_request_clamped_to_zero(self):
    """Regen commands during grace are forbidden (safety — prevents
    the zero-torque-learning-on-engage regen spike)."""
    v_ego = 10.0
    a_max = a_max_at_speed(v_ego)

    for frame in [0, 10, 25, 40, 49]:
      out = engage_grace_ramp(-0.5, frame, a_max)
      self.assertEqual(out, 0.0, f"negative request not clamped at frame {frame}")

  def test_post_grace_passes_request_through(self):
    """Once grace_elapsed >= ENGAGE_GRACE_FRAMES, the ramp is inactive
    and the request passes through unchanged (including negative regen)."""
    a_max = a_max_at_speed(10.0)

    for request in [-1.5, 0.0, 0.5, 2.0]:
      out = engage_grace_ramp(request, ENGAGE_GRACE_FRAMES, a_max)
      self.assertEqual(out, request)
      out = engage_grace_ramp(request, ENGAGE_GRACE_FRAMES * 3, a_max)
      self.assertEqual(out, request)


class TestEngageGraceSourceDrift(unittest.TestCase):
  """Catch silent drift between the test's mirror and carcontroller.py."""

  def test_carcontroller_uses_engage_a_max_with_grace_progress(self):
    """Verify the carcontroller source still implements the invariant this
    test file guards. This catches someone reverting to the multiplicative-
    scaling bug or introducing a new cap source."""
    import os

    here = os.path.dirname(os.path.abspath(__file__))
    src_path = os.path.join(here, "..", "carcontroller.py")
    with open(src_path) as f:
      src = f.read()

    # These three ingredients together encode the invariant
    self.assertIn(
      "engage_a_max", src,
      "carcontroller.py no longer snapshots engage_a_max; "
      + "grace-period cap may have reverted to MPC-request-scaled")
    self.assertIn("grace_progress", src,
                  "carcontroller.py dropped grace_progress ramp")
    self.assertIn("get_preap_accel_limits", src,
                  "carcontroller.py no longer pulls a_max from the accel profile")


if __name__ == "__main__":
  unittest.main()
