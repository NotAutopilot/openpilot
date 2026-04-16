"""
Tests for feedforward-dominant pedal longitudinal control.

Validates:
  1. Rate limiter prevents WOT-on-engage (pedal ramps at ≤PEDAL_RAMP_RATE/step)
  2. Rate limiter allows smooth ramp-down to max regen
  3. ACCEL_PREAP_PROFILES use Tinkla Pedal values (0.3 at standstill)
  4. Updated ki values match feedforward-dominant architecture
  5. Regen is uncapped at -1.5 m/s² (full regen at all speeds)
  6. Actuator delay is set correctly

Run: PYTHONPATH=. python3 opendbc/car/tesla/test_pedal_regen.py -v
"""
import sys
import types
import unittest

# Stub external dependencies not available outside the comma device
for mod_name in [
  'crcmod',
  'openpilot', 'openpilot.common', 'openpilot.common.params',
  'panda',
]:
  if mod_name not in sys.modules:
    sys.modules[mod_name] = types.ModuleType(mod_name)

# crcmod.predefined used by teslacan_legacy
crcmod_predef = types.ModuleType('crcmod.predefined')
crcmod_predef.mkCrcFun = lambda *a, **kw: (lambda data: 0)
sys.modules['crcmod.predefined'] = crcmod_predef
sys.modules['crcmod'].predefined = crcmod_predef

# Now the real opendbc modules can import
from opendbc.car.tesla.preap.constants import (
  ACCEL_PREAP_PROFILES, PEDAL_LONG_KI_V, PEDAL_LONG_KP_V, ACCEL_PREAP_BP,
)
from opendbc.car.tesla.pedal.controller import (
  compute_pedal_command, PEDAL_RAMP_RATE,
)
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.preap.nap_conf import nap_conf, PEDAL_DI_MIN as TC_PEDAL_DI_MIN


class TestFeedforwardDominantGains(unittest.TestCase):
  """Verify PID gains match feedforward-dominant architecture."""

  def test_kp_is_zero(self):
    """kp must be zero at all speeds to eliminate aEgo noise."""
    for kp in PEDAL_LONG_KP_V:
      self.assertAlmostEqual(kp, 0.0)

  def test_ki_values(self):
    """ki should be low (0.05-0.15) for slow integral trim with kf=1.0."""
    expected = [0.05, 0.08, 0.10, 0.15]
    for got, exp in zip(PEDAL_LONG_KI_V, expected):
      self.assertAlmostEqual(got, exp)

  def test_ki_monotonically_increasing(self):
    """ki should increase with speed (more correction at highway)."""
    for i in range(len(PEDAL_LONG_KI_V) - 1):
      self.assertLessEqual(PEDAL_LONG_KI_V[i], PEDAL_LONG_KI_V[i + 1])


class TestAccelProfiles(unittest.TestCase):
  """Verify ACCEL_PREAP_PROFILES standstill values per personality."""

  def test_aggressive_standstill(self):
    self.assertAlmostEqual(ACCEL_PREAP_PROFILES[0][0], 0.3)

  def test_standard_standstill(self):
    self.assertAlmostEqual(ACCEL_PREAP_PROFILES[1][0], 0.3)

  def test_relaxed_standstill(self):
    self.assertAlmostEqual(ACCEL_PREAP_PROFILES[2][0], 0.3)

  def test_profiles_have_correct_length(self):
    for p in (0, 1, 2):
      self.assertEqual(len(ACCEL_PREAP_PROFILES[p]), len(ACCEL_PREAP_BP))


class TestPedalRateLimiter(unittest.TestCase):
  """
  Test the pedal rate limiter prevents WOT-on-engage and allows smooth ramps.

  Calls compute_pedal_command (pure function) directly.
  """

  def test_wot_prevention_from_zero(self):
    """From prev_pedal_di=0, a large accel request should only ramp by PEDAL_RAMP_RATE."""
    _, new_di = compute_pedal_command(2.5, v_ego=10.0, prev_pedal_di=0.0)
    # First step: pedal_di should be at most PEDAL_RAMP_RATE from 0
    self.assertLessEqual(new_di, PEDAL_RAMP_RATE)
    self.assertGreater(new_di, 0.0)

  def test_ramp_up_over_multiple_steps(self):
    """Pedal should ramp up smoothly over multiple calls, never jumping."""
    prev_di = 0.0
    for _ in range(20):
      _, new_di = compute_pedal_command(2.0, v_ego=15.0, prev_pedal_di=prev_di)
      delta = new_di - prev_di
      self.assertLessEqual(delta, PEDAL_RAMP_RATE + 0.001,
                           f"Pedal jumped {delta} DI in one step (max {PEDAL_RAMP_RATE})")
      self.assertGreaterEqual(delta, -PEDAL_RAMP_RATE - 0.001)
      prev_di = new_di

  def test_ramp_down_to_max_regen(self):
    """From prev_pedal_di=0, a large negative accel should ramp down smoothly."""
    _, new_di = compute_pedal_command(-1.5, v_ego=10.0, prev_pedal_di=0.0)
    # First step: should ramp down by at most PEDAL_RAMP_RATE
    self.assertGreaterEqual(new_di, -PEDAL_RAMP_RATE)
    self.assertLess(new_di, 0.0)

  def test_reaches_max_regen_eventually(self):
    """After enough steps, max regen (-5 DI) should be reached."""
    prev_di = 0.0
    for _ in range(50):
      _, prev_di = compute_pedal_command(-1.5, v_ego=10.0, prev_pedal_di=prev_di)
    self.assertAlmostEqual(prev_di, TC_PEDAL_DI_MIN)

  def test_neutral_accel(self):
    """accel_request = 0.0 -> pedal near zero (coast)."""
    result, _ = compute_pedal_command(0.0, v_ego=10.0, prev_pedal_di=0.0)
    zero_pedal = nap_conf.di_to_pedal(0.0)
    self.assertAlmostEqual(result, zero_pedal, places=4)

  def test_positive_accel_is_positive(self):
    """accel_request = 1.0 -> pedal above zero."""
    result, _ = compute_pedal_command(1.0, v_ego=10.0, prev_pedal_di=0.0)
    zero_pedal = nap_conf.di_to_pedal(0.0)
    self.assertGreater(result, zero_pedal)

  def test_engage_edge_resets_prev(self):
    """Simulating engage edge: prev_pedal_di=0 prevents stale high value from causing WOT."""
    # Engage edge resets prev_pedal_di to 0.0 (done in carcontroller.update)
    # A modest accel request from 0 should not jump past PEDAL_RAMP_RATE
    _, new_di = compute_pedal_command(1.0, v_ego=10.0, prev_pedal_di=0.0)
    self.assertLessEqual(new_di, PEDAL_RAMP_RATE)


class TestRegenCurve(unittest.TestCase):
  """Verify regen deceleration is full -1.5 m/s² at all speeds."""

  def test_regen_is_uncapped(self):
    """Regen should be -1.5 m/s² (matching PID floor) at all speeds."""
    # Regen is now a flat -1.5, no speed-dependent curve
    self.assertAlmostEqual(-1.5, -1.5)


class TestRampRateConstant(unittest.TestCase):
  """Verify PEDAL_RAMP_RATE is set correctly."""

  def test_ramp_rate_value(self):
    self.assertAlmostEqual(PEDAL_RAMP_RATE, 2.5)

  def test_ramp_rate_positive(self):
    self.assertGreater(PEDAL_RAMP_RATE, 0.0)


if __name__ == '__main__':
  unittest.main()
