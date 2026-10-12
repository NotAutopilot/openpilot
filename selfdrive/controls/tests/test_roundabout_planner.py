"""Planner hook: RB funnel eases aTarget; a sharp corner / no flag does not.

Full LongitudinalPlanner loop needs cereal/scons. The planner calls
`apply_roundabout_plan` on the liveMapDataNAP hint — that is the aTarget
path under test here (Willmar: aTarget stayed +0.4 / later only −0.55).
"""
from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.constants import LOOKAHEAD_EARLY, LOOKAHEAD_NORMAL
from openpilot.selfdrive.mapd.roundabout import (
  RB_A_FLOOR_MS2,
  RB_OUTER_OFFSET_M,
  RB_V_MAX_MS,
  RoundaboutHint,
  apply_roundabout_plan,
  rb_profile_v_ms,
  roundabout_outer_curvature_bias,
  roundabout_outer_path_offset_m,
)


def test_funnel_triggers_speed_ease():
  v_ego = 49.0 * CV.MPH_TO_MS
  v_cruise = 50.0 * CV.MPH_TO_MS
  hint = RoundaboutHint(
    approaching=True, distance_m=90.0, speed_limit_ms=20.0 * CV.MPH_TO_MS,
  )
  v_c, v_hud, a_tgt, rb_v = apply_roundabout_plan(
    v_ego, v_cruise, v_cruise, 0.4, hint, LOOKAHEAD_NORMAL,
  )
  assert rb_v is not None
  assert a_tgt <= RB_A_FLOOR_MS2, a_tgt
  assert v_c < v_cruise
  assert v_hud < v_cruise
  assert rb_v <= RB_V_MAX_MS + 8.0


def test_sharp_corner_without_flag_does_not_ease():
  v_ego = 49.0 * CV.MPH_TO_MS
  v_cruise = 50.0 * CV.MPH_TO_MS
  v_c, v_hud, a_tgt, rb_v = apply_roundabout_plan(
    v_ego, v_cruise, v_cruise, 0.4, None, LOOKAHEAD_NORMAL,
  )
  assert rb_v is None
  assert a_tgt == 0.4
  assert v_c == v_cruise
  assert v_hud == v_cruise


def test_on_roundabout_holds_soft_target():
  v_ego = 49.0 * CV.MPH_TO_MS
  v_cruise = 50.0 * CV.MPH_TO_MS
  hint = RoundaboutHint(on_roundabout=True, speed_limit_ms=20.0 * CV.MPH_TO_MS)
  v_c, _v_hud, a_tgt, rb_v = apply_roundabout_plan(
    v_ego, v_cruise, v_cruise, 0.4, hint, LOOKAHEAD_NORMAL,
  )
  assert rb_v is not None
  assert abs(rb_v - 20.0 * CV.MPH_TO_MS) < 1e-6
  assert a_tgt <= RB_A_FLOOR_MS2, a_tgt
  assert v_c <= rb_v


def test_forty_five_mph_funnel_plans_at_least_one_g_tenth():
  """(a) 40–45 mph in funnel: planned a ≤ −1.0 until near ring speed."""
  v_cruise = 50.0 * CV.MPH_TO_MS
  for v_mph, dist in ((40.0, 90.0), (45.0, 86.0), (45.0, 160.0)):
    hint = RoundaboutHint(
      approaching=True, distance_m=dist, speed_limit_ms=20.0 * CV.MPH_TO_MS,
    )
    _vc, _vh, a_tgt, rb_v = apply_roundabout_plan(
      v_mph * CV.MPH_TO_MS, v_cruise, v_cruise, 0.4, hint, LOOKAHEAD_EARLY,
    )
    assert rb_v is not None
    assert a_tgt <= RB_A_FLOOR_MS2, (v_mph, dist, a_tgt)


def test_no_plus_a_while_approaching_roundabout():
  """(b) EP0: never allow +a rebound after lead clears in the funnel."""
  hint = RoundaboutHint(
    approaching=True, distance_m=128.0, speed_limit_ms=20.0 * CV.MPH_TO_MS,
  )
  _vc, _vh, a_tgt, rb_v = apply_roundabout_plan(
    37.0 * CV.MPH_TO_MS, 50.0 * CV.MPH_TO_MS, 50.0 * CV.MPH_TO_MS, 0.399, hint,
  )
  assert rb_v is not None
  assert a_tgt <= 0.0


def test_outer_bias_counters_inside_cut():
  """(c) RHT outer is right (−y); magnitude counters EP1 ~3 m inside cut."""
  rht = roundabout_outer_path_offset_m(
    on_roundabout=True, is_rhd=False,
  )
  lhd = roundabout_outer_path_offset_m(
    on_roundabout=True, is_rhd=True,
  )
  assert rht < 0.0 and lhd > 0.0
  assert abs(rht) >= 2.8
  assert abs(abs(rht) - RB_OUTER_OFFSET_M) < 1e-6
  entry = roundabout_outer_path_offset_m(
    on_roundabout=False, approaching=True, distance_m=40.0, is_rhd=False,
  )
  assert abs(entry - rht) < 1e-6
  kappa = roundabout_outer_curvature_bias(rht)
  assert kappa < 0.0
  assert abs(kappa) >= 0.015


def test_long_enable_in_funnel_applies_ease_immediately():
  """(d) Long enable mid-funnel: full RB ease on this frame, not +0.4."""
  hint = RoundaboutHint(
    approaching=True, distance_m=86.0, speed_limit_ms=20.0 * CV.MPH_TO_MS,
  )
  v45 = 45.0 * CV.MPH_TO_MS
  v_cruise = 50.0 * CV.MPH_TO_MS
  v_c, v_hud, a_tgt, rb_v = apply_roundabout_plan(
    v45, v_cruise, v_cruise, 0.40, hint, LOOKAHEAD_EARLY,
  )
  assert rb_v is not None
  # Oct 11: MAX follows the 0.7 m/s² profile (no step); the late hint keeps the Willmar −a floor.
  assert abs(rb_v - rb_profile_v_ms(86.0, 20.0 * CV.MPH_TO_MS)) < 1e-6
  assert rb_v < v45
  assert v_c <= rb_v
  assert v_hud <= rb_v
  assert a_tgt <= RB_A_FLOOR_MS2
