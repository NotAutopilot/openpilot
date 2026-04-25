#!/usr/bin/env python3
"""Tests for Pre-AP engagement FSM, specifically the brake-to-disengage path.

The panda safety layer hardcodes brake_pressed=false for Pre-AP (tesla_preap.h:340).
Brake-to-disengage is handled here in the Python layer via the PreAPEngagement FSM.
This test verifies that the brake properly drops longitudinal while keeping lateral.
"""
import unittest

from opendbc.car.tesla.preap.engagement import PreAPEngagement


class TestPreAPBrakeDisengage(unittest.TestCase):
  """Verify the brake-to-disengage path that the panda safety tests reference."""

  def _make_engagement(self, double_pull=False):
    return PreAPEngagement(double_pull_enabled=double_pull, double_pull_window_ms=750)

  def _engage_single_pull(self, eng, use_pedal=True):
    """Simulate a single-pull engage with pedal mode."""
    eng.process_buttons(
      cruise_buttons=2, prev_cruise_buttons=0,  # MAIN rising edge
      curr_time_ms=1000, v_ego=10.0, speed_units="KPH",
      use_pedal=use_pedal, pedal_long_allowed=use_pedal,
      long_control_allowed=True, real_brake_pressed=False)

  def test_brake_drops_longitudinal_keeps_lateral(self):
    # This is the core invariant: brake drops pedal but keeps steering.
    eng = self._make_engagement()
    self._engage_single_pull(eng, use_pedal=True)
    self.assertTrue(eng.cruiseEnabled)
    self.assertTrue(eng.enableLongControl)

    # Brake rising edge
    eng.process_buttons(
      cruise_buttons=0, prev_cruise_buttons=0,
      curr_time_ms=2000, v_ego=10.0, speed_units="KPH",
      use_pedal=True, pedal_long_allowed=True,
      long_control_allowed=True, real_brake_pressed=True)

    # Longitudinal dropped, lateral stays
    self.assertTrue(eng.cruiseEnabled, "Lateral should stay active after brake")
    self.assertFalse(eng.enableLongControl, "Longitudinal should drop on brake")
    self.assertTrue(eng.enableJustCC, "Should transition to CC-only mode")

  def test_brake_no_effect_without_pedal(self):
    # In non-pedal mode (stock CC only), brake doesn't trigger any action
    # in the engagement FSM — stock CC handles its own brake disengage.
    eng = self._make_engagement()
    self._engage_single_pull(eng, use_pedal=False)
    self.assertTrue(eng.cruiseEnabled)

    eng.process_buttons(
      cruise_buttons=0, prev_cruise_buttons=0,
      curr_time_ms=2000, v_ego=10.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=True)

    # No change — stock CC handles brake disengage independently
    self.assertTrue(eng.cruiseEnabled)

  def test_brake_only_on_rising_edge(self):
    # Holding brake should not repeatedly disengage — only rising edge matters.
    eng = self._make_engagement()
    self._engage_single_pull(eng, use_pedal=True)

    # Brake held from previous cycle (not a rising edge)
    eng.preap_brake_pressed_prev = True
    eng.process_buttons(
      cruise_buttons=0, prev_cruise_buttons=0,
      curr_time_ms=2000, v_ego=10.0, speed_units="KPH",
      use_pedal=True, pedal_long_allowed=True,
      long_control_allowed=True, real_brake_pressed=True)

    # No disengage — brake was already pressed
    self.assertTrue(eng.enableLongControl)

  def test_brake_disengage_then_reengage(self):
    # After brake drops longitudinal, a stalk pull should re-engage everything.
    eng = self._make_engagement()
    self._engage_single_pull(eng, use_pedal=True)

    # Brake drops longitudinal
    eng.process_buttons(
      cruise_buttons=0, prev_cruise_buttons=0,
      curr_time_ms=2000, v_ego=10.0, speed_units="KPH",
      use_pedal=True, pedal_long_allowed=True,
      long_control_allowed=True, real_brake_pressed=True)
    self.assertFalse(eng.enableLongControl)
    self.assertTrue(eng.cruiseEnabled)

    # Release brake
    eng.process_buttons(
      cruise_buttons=0, prev_cruise_buttons=0,
      curr_time_ms=3000, v_ego=10.0, speed_units="KPH",
      use_pedal=True, pedal_long_allowed=True,
      long_control_allowed=True, real_brake_pressed=False)

    # Stalk pull re-engages
    eng.process_buttons(
      cruise_buttons=2, prev_cruise_buttons=0,
      curr_time_ms=4000, v_ego=10.0, speed_units="KPH",
      use_pedal=True, pedal_long_allowed=True,
      long_control_allowed=True, real_brake_pressed=False)
    self.assertTrue(eng.cruiseEnabled)
    self.assertTrue(eng.enableLongControl)


class TestNoPedalCCEngage(unittest.TestCase):
  """Tests for no-pedal stock CC engage: DI state gating, double-pull behavior."""

  def _make_engagement(self):
    return PreAPEngagement(double_pull_enabled=True, double_pull_window_ms=750)

  def _double_pull(self, eng, t1=1000, t2=1400, di_cruise_state="OFF"):
    """Simulate a double-pull in no-pedal mode."""
    eng.process_buttons(
      cruise_buttons=2, prev_cruise_buttons=0,
      curr_time_ms=t1, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state=di_cruise_state)
    eng.process_buttons(
      cruise_buttons=0, prev_cruise_buttons=2,
      curr_time_ms=t1 + 50, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state=di_cruise_state)
    eng.process_buttons(
      cruise_buttons=2, prev_cruise_buttons=0,
      curr_time_ms=t2, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state=di_cruise_state)
    return eng

  def test_double_pull_di_off_engage_needed_set_anyway(self):
    """Double-pull with DI OFF still sets engage_needed. SET_ACCEL spoofs to
    an unarmed DI are ignored — the spoofer's 500ms ENGAGING timeout exits
    cleanly, and teslaCCNotArmed surfaces the unarmed state to the user.
    Broader gate (was STANDBY-only) avoids missing the engage when DI is
    transitioning through PRE_FAULT/STANDSTILL/etc at the second-pull frame."""
    eng = self._make_engagement()
    self._double_pull(eng, di_cruise_state="OFF")
    self.assertTrue(eng.cruiseEnabled, "Lateral should be enabled")
    self.assertTrue(eng.preap_cc_engage_needed,
                    "engage_needed fires on any non-ENABLED DI state")

  def test_double_pull_di_standby_sets_engage(self):
    """Double-pull with DI STANDBY should set engage_needed for SET_ACCEL spoof."""
    eng = self._make_engagement()
    self._double_pull(eng, di_cruise_state="STANDBY")
    self.assertTrue(eng.cruiseEnabled)
    self.assertTrue(eng.preap_cc_engage_needed, "Should engage when DI is STANDBY")

  def test_single_pull_cc_running_cancels_immediately(self):
    """Single pull with DI ENABLED: cancel fires on the same frame for safety —
    no scheduled-window delay. The spoofer's CANCEL_DELAY_FRAMES (100ms) and
    frame-slot alignment still apply downstream."""
    eng = self._make_engagement()
    eng.process_buttons(
      cruise_buttons=2, prev_cruise_buttons=0,
      curr_time_ms=1000, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state="ENABLED")
    self.assertTrue(eng.preap_cc_cancel_needed,
                    "Cancel must fire immediately on first pull, not after window")

  def test_double_pull_cc_running_cancel_then_reengage(self):
    """Double-pull with DI ENABLED: first pull fires cancel immediately
    (visible briefly on the bus), second pull re-engages. The cancel-then-
    engage flicker is the accepted tradeoff for instant single-pull cancel."""
    eng = self._make_engagement()
    self._double_pull(eng, t1=1000, t2=1400, di_cruise_state="ENABLED")
    # After the double-pull, engage_needed is the live event from the second
    # pull (single-frame semantics — the first pull's cancel_needed was on
    # an earlier frame, already consumed by the spoofer).
    self.assertTrue(
      eng.preap_cc_engage_needed,
      "Second pull within window must set engage_needed even if first pull already canceled",
    )

  def test_single_pull_cc_standby_cancels_immediately(self):
    """Single pull with DI STANDBY: cancel must fire on the same frame so any
    DI auto-engage from the driver's physical MAIN pull is killed within ~100ms.
    Was previously gated on a 750/400ms window expiry; that left CC briefly
    engaged in normal driving — see drive d0cdc986 follow-up."""
    eng = self._make_engagement()
    eng.process_buttons(
      cruise_buttons=2, prev_cruise_buttons=0,
      curr_time_ms=1000, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state="STANDBY")
    self.assertTrue(eng.preap_cc_cancel_needed,
                    "Cancel must fire on the same frame regardless of DI state")
    self.assertTrue(eng.cruiseEnabled)
    self.assertFalse(eng.enableLongControl)

  def test_happy_path_armed_double_pull_engages(self):
    """Regression: user arms CC (STANDBY), double-pulls → engage fires, no cancel."""
    eng = self._make_engagement()
    self._double_pull(eng, di_cruise_state="STANDBY")
    self.assertTrue(eng.cruiseEnabled)
    self.assertTrue(eng.preap_cc_engage_needed)
    self.assertFalse(eng.preap_cc_cancel_needed, "Must not cancel the user's armed CC")

  def test_double_pull_di_off_then_arm_and_repull(self):
    """User double-pulls with DI OFF (lateral on, engage_needed fires futilely),
    later arms cruise, pulls again."""
    eng = self._make_engagement()
    self._double_pull(eng, t1=1000, t2=1400, di_cruise_state="OFF")
    self.assertTrue(eng.preap_cc_engage_needed,
                    "Broader gate fires engage_needed on OFF too; spoofer times out")
    self.assertTrue(eng.cruiseEnabled)

    # User presses end-stalk (MAIN) to arm — seen as a first pull (>750ms later)
    eng.process_buttons(
      cruise_buttons=2, prev_cruise_buttons=0,
      curr_time_ms=3000, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state="STANDBY")
    self.assertTrue(eng.pending_enable)

    # Second pull within window — double-pull with DI now STANDBY
    eng.process_buttons(
      cruise_buttons=0, prev_cruise_buttons=2,
      curr_time_ms=3050, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state="STANDBY")
    eng.process_buttons(
      cruise_buttons=2, prev_cruise_buttons=0,
      curr_time_ms=3500, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state="STANDBY")
    self.assertTrue(eng.preap_cc_engage_needed, "Should engage after arming + re-pull")


class TestNoPedalUpDownPassthrough(unittest.TestCase):
  """In no-pedal mode, up/down stalk presses must not mutate NAP's FSM. Stock CC
  speed adjust is handled by the DI reading the driver's direct stalk messages;
  NAP has nothing to contribute. See the stalk-fsm-single-pull-cancel thread."""

  def _engage_lateral_only(self, eng):
    eng.process_buttons(
      cruise_buttons=2, prev_cruise_buttons=0,
      curr_time_ms=1000, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state="STANDBY")

  def test_accel_press_leaves_enable_long_false(self):
    eng = PreAPEngagement(double_pull_enabled=True, double_pull_window_ms=750)
    self._engage_lateral_only(eng)
    self.assertFalse(eng.enableLongControl)
    self.assertEqual(eng.pedal_speed_kph, 0.0)

    # Driver presses stalk up (RES_ACCEL = 16)
    eng.process_buttons(
      cruise_buttons=16, prev_cruise_buttons=0,
      curr_time_ms=2000, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state="ENABLED")

    self.assertFalse(eng.enableLongControl,
                     "Up press in no-pedal must not auto-promote enableLongControl")
    self.assertEqual(eng.pedal_speed_kph, 0.0,
                     "Up press in no-pedal must not mutate pedal_speed_kph")

  def test_decel_press_leaves_enable_long_false(self):
    eng = PreAPEngagement(double_pull_enabled=True, double_pull_window_ms=750)
    self._engage_lateral_only(eng)

    eng.process_buttons(
      cruise_buttons=32, prev_cruise_buttons=0,  # DECEL_SET
      curr_time_ms=2000, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state="ENABLED")

    self.assertFalse(eng.enableLongControl)
    self.assertEqual(eng.pedal_speed_kph, 0.0)


if __name__ == "__main__":
  unittest.main()
