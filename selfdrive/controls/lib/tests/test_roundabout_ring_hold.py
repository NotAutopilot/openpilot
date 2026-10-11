"""Roundabout ring hold (Oct 1 pm, 0000013c): cap against the map circle, exit-window floor, driver-press debounce, earlier latch, exit blinker.
NAPRoundaboutAssist stays default Off (Off bit-identical: test_roundabout_guide.py). Curvature + = right; the Sep 29 ring is CCW (circulation < 0)."""
import json
import math
from pathlib import Path

import pytest

from openpilot.selfdrive.controls.lib import roundabout_guide as RG
from openpilot.selfdrive.controls.lib import roundabout_ring_hold as RH
from openpilot.selfdrive.mapd.roundabout import RB_DECEL_ONSET_M, RB_FUNNEL_M
from openpilot.selfdrive.mapd.roundabout_map import RingGeometry

DATA = Path(__file__).parent / "data"
SEP29 = json.loads((DATA / "roundabout_sep29_1157.json").read_text())
RING = RingGeometry.from_json(SEP29["ring"])
P13 = json.loads((DATA / "roundabout_pass_oct1.json").read_text())["passes"]
DT = 0.01
A = 2.2          # m/s² lateral-accel cap


def _guide(theta_deg, off=-2.5, mk=-0.002, v=7.0):
  """Guide on the ring at polar angle theta_deg (off = meters from R, - = inside), heading along the circulation, one update in."""
  g = RG.RoundaboutGuide()
  g.set_ring(RING)
  th, r = math.radians(theta_deg), RING.radius_m + off
  g.pose.bacc, g.bias, g.pose.initialized = 5.0, (0.3, 0.0), True
  g.pose.xo, g.pose.yo, g.pose.psio, g.pose.dpsi = r * math.cos(th), r * math.sin(th), th + math.pi / 2, 0.0
  g.pose.t, g.pose.n_good, g.pose.innov, g.pose.last_fix_t = 0.0, 3, 0.5, 0.0
  g.phase = RG.ACTIVE
  g.update(0.1, v, v * mk, mk, enabled=True, lat_active=True)
  return g


def _run(g, n, driver=None, mk=-0.002, t0=0.2, v=7.0):
  return [g.update(t0 + i * DT, v, v * mk, mk, enabled=True, lat_active=True, driver=driver) for i in range(n)]


# ---- pure functions -------------------------------------------------------------------------------------------

def test_exit_angles_are_the_three_soco_branches():
  assert [round(a) for a in RH.exit_angles(RING)] == [63, 150, 308]
  assert RH.exit_angles(None) == []
  assert [round(x) for x in _guide(298.0).exits] == [63, 150, 308]          # the guide takes them from set_ring


def test_widen_cap_only_below_half_the_circle_and_bounded():
  v, circle = 8.0, 0.045
  assert RH.widen_cap(0.01, 0.03, circle, v, A) == 0.01                       # model at 67% of the circle: untouched
  assert RH.widen_cap(0.01, 0.0, circle, v, A) == pytest.approx(RH.CIRCLE_CAP_EXTRA)   # model 0: up to CIRCLE_CAP_EXTRA over it
  assert RH.widen_cap(0.0, 0.0, circle, 30.0, A) == pytest.approx(A / 900.0)   # 2.2 m/s² still binds at speed
  assert RH.widen_cap(0.0, 0.0, 0.004, v, A) == 0.004                          # never above the circle target
  assert RH.widen_cap(0.01, 0.0, -0.01, v, A) == 0.01                          # circle target on the wrong side: nothing
  edge = RH.widen_cap(0.0, 0.5 * circle - 1e-9, circle, v, A)
  assert edge == pytest.approx(0.5 * circle, abs=1e-6)                         # fades to nothing at half the circle (cap = the model): no step


def test_ring_limits_floor_only_in_the_exit_window_without_a_stalk():
  cap, floor = RH.ring_limits(0.002, 0.0, 0.0, 0.045, 8.0, A, True, False)
  assert floor == pytest.approx(min(0.045, A / 64.0)) and cap >= floor
  assert RH.ring_limits(0.002, 0.0, 0.0, 0.045, 8.0, A, True, True) == (0.002, 0.0)    # plain (stalk / off the band): old limits
  assert RH.ring_limits(0.002, 0.0, 0.0, 0.045, 8.0, A, False, False)[1] == 0.0         # outside the window: no floor


def test_exit_windows_20_deg_before_and_5_after_for_both_senses():
  ex = [150.0]
  for sense in (1.0, -1.0):
    def ahead(d, sense=sense):                           # d deg before the exit along the circulation
      return math.radians(150.0 - sense * d)
    assert RH.in_exit_window(ahead(19.0), sense, ex) and RH.in_exit_window(ahead(0.0), sense, ex)
    assert not RH.in_exit_window(ahead(21.0), sense, ex) and not RH.in_exit_window(ahead(-6.0), sense, ex)
    assert RH.in_exit_window(ahead(-4.0), sense, ex)


def test_press_debounce_and_light_torque():
  assert not RH.counts_as_press(False, 3.0, 1.0)
  assert RH.counts_as_press(True, 0.0, 1.0)
  assert not RH.counts_as_press(True, 2.0, 1.0)                   # same-side add, any magnitude, is not a press
  assert not RH.counts_as_press(True, 1.2, 1.0)                   # CCW: + (left) torque is toward the circulation
  assert RH.counts_as_press(True, -1.2, 1.0)                      # toward the exit: counts
  assert not RH.counts_as_press(True, -1.2, -1.0) and RH.counts_as_press(True, 1.2, -1.0)    # CW mirror
  t = 0.0
  for _ in range(40):
    win, t = RH.driver_wins(False, True, 0.0, 1.0, t, DT)
    assert not win                                                 # zero-torque press does not zero the curl
  assert t == pytest.approx(0.40)
  assert RH.driver_wins(False, True, 0.0, 1.0, 0.2, DT)[1] == pytest.approx(0.21)
  assert RH.driver_wins(False, False, 0.0, 1.0, 0.2, DT) == (False, 0.0)       # a gap resets the timer
  assert not RH.driver_wins(False, False, 2.0, 1.0, 0.0, DT)[0]                 # same-side hard add does not win
  assert not RH.driver_wins(False, True, 2.0, 1.0, 0.0, DT)[0]
  assert RH.driver_wins(False, False, -2.0, 1.0, 0.0, DT)[0]                    # exit-side pull: at once
  assert RH.driver_wins(True, False, 0.0, 1.0, 0.0, DT)[0]                      # held stalk, at once
  assert not RH.driver_wins(False, True, 1.2, 1.0, 0.0, DT)[0]


def test_step_cap_rises_at_once_falls_slowly_and_is_bound_by_a_lat():
  e, cap = RH.step_cap(0.0, 0.002, 0.02, 7.0, A, 0.00035, 0.3, False, False)
  assert (e, cap) == (pytest.approx(0.018), pytest.approx(0.02))
  e2, cap2 = RH.step_cap(e, 0.002, 0.002, 7.0, A, 0.00035, 0.3, False, False)
  assert e - e2 == pytest.approx(0.3 * 0.00035) and cap2 > 0.002                 # slow fall
  assert RH.step_cap(e, 0.002, 0.002, 7.0, A, 0.00035, 0.3, True, False)[0] == pytest.approx(e - 0.00035)   # plain: full slew
  assert RH.step_cap(e, 0.002, 0.02, 7.0, A, 0.00035, 0.3, False, True) == (0.0, 0.002)                    # over the ring speed
  assert RH.step_cap(0.0, 0.002, 0.2, 12.0, A, 0.00035, 0.3, False, False)[1] == pytest.approx(A / 144.0)


def test_latch_ready_earlier_at_the_ring_edge():
  assert RH.latch_ready(40.0, 0.5, 25.0) and not RH.latch_ready(50.0, 0.5, 25.0) and not RH.latch_ready(10.0, 1.5, 25.0)
  assert RH.latch_ready(10.0, 0.5, 25.0) and RH.latch_ready(30.0, -2.0, 25.0)


# ---- guide ------------------------------------------------------------------------------------------------------

def test_cap_against_the_circle_when_the_model_is_flat_but_still_2_2_and_slew_bound():
  g = _guide(298.0)
  assert g.latched
  outs = _run(g, 150)
  assert max(-o for o in outs) > 0.012 and all(o <= 0.0005 for o in outs[20:])        # left of the model's -0.002 by > 0.01
  assert all(-o * 49.0 <= A * 1.0001 for o in outs)                                    # 2.2 m/s² at 7 m/s
  assert all(abs(b - a) / DT <= RG.RING_SLEW + 1e-6 for a, b in zip(outs, outs[1:], strict=False))


def test_exit_window_floor_keeps_the_circle_target_without_a_stalk_and_not_with_one():
  def run(exits, stalk=0):
    g = _guide(298.0)
    g.exits = exits                                       # the car is ~15 deg before an exit at 330 deg (inside the 20 deg window)
    return _run(g, 200, driver=RG.DriverInput(stalk_dir=stalk) if stalk else None, mk=0.004)    # the model follows the exit branch (right)
  inside, outside = run([330.0]), run([160.0])
  assert outside[-1] > -0.012                              # no window: the widened cap only (<= model + CIRCLE_CAP_EXTRA)
  assert inside[-1] < -0.02 and inside[-1] * 49.0 >= -RG.RING_A_LAT_MAX      # in the window: up to the circle target, 2.2 m/s² at 7 m/s
  assert all(abs(b - a) / DT <= 0.07 + 1e-6 for a, b in zip(inside, inside[1:], strict=False))
  assert run([330.0], stalk=2)[-1] == 0.004                # a right stalk pending: no floor, the model's way


def test_cw_ring_mirror_of_the_window_and_the_widening():
  s_c, s_m = 0.045, 0.0
  assert RH.widen_cap(0.0, s_m, s_c, 8.0, A) > 0.0
  # a CW ring (sense -1): window is on the other side of the exit angle
  assert RH.in_exit_window(math.radians(160.0), -1.0, [150.0]) and not RH.in_exit_window(math.radians(140.0), -1.0, [150.0])


def test_exit_blinker_releases_only_on_the_exit_arm_and_only_to_the_model():
  def blink(theta, off, stalk):
    g = _guide(theta, off=off, mk=0.004)
    _run(g, 20, mk=0.004)
    return g, _run(g, 1, driver=RG.DriverInput(stalk_dir=stalk), mk=0.004, t0=0.5)[0]
  g, out = blink(300.0, 3.0, 2)                          # at the outer edge of the band (d_edge ~ -3), 8 deg before the 308 exit
  assert g.release == "exit_blinker" and g.phase == RG.DONE
  assert out == 0.004                                    # the model at once, nothing else: it does not route the path onto the exit
  g, out = blink(300.0, -2.5, 2)                         # mid band: a tip is not an exit
  assert g.release == "" and g.phase == RG.ACTIVE and out != 0.004
  g, out = blink(300.0, 3.0, 1)                          # left blinker on the exit arm: nothing
  assert g.release == "" and g.phase == RG.ACTIVE
  g, out = blink(100.0, 3.0, 2)                          # far from every exit
  assert g.release == ""


def test_latch_through_the_exit_window_without_release():
  g = _guide(298.0)
  _run(g, 600, mk=0.004)
  assert g.latched and g.phase == RG.ACTIVE and g.release == ""      # held through the window: no map-only exit


def test_press_flicker_keeps_the_correction_but_a_held_press_and_hard_torque_still_win():
  g = _guide(298.0)
  base = _run(g, 60)
  g = _guide(298.0)
  _run(g, 60 - 20)
  flick = _run(g, 20, driver=RG.DriverInput(pressed=True), t0=0.6)      # 0.2 s flicker
  assert flick[-1] != -0.002 and g.dk_out != 0.0 and g.debug["w"] > 0.0
  after = _run(g, 20, t0=0.8)
  assert abs(after[-1] - base[-1]) < 0.01                               # the correction is still there
  g = _guide(298.0)
  _run(g, 60)
  assert _run(g, 29, driver=RG.DriverInput(pressed=True), t0=0.8) and g.debug["w"] > 0.0
  still = _run(g, 10, driver=RG.DriverInput(pressed=True), t0=1.09)      # 0.39 s: debounce does not drop the curl
  assert still[-1] != -0.002 and g.debug["w"] > 0.0 and g.release == ""
  held = _run(g, 60, driver=RG.DriverInput(pressed=True), t0=1.19)
  assert held[-1] == -0.002 and g.release == "override"                  # 0.5 s zero-torque press still overrides
  g = _guide(298.0)
  _run(g, 60)
  _run(g, 1, driver=RG.DriverInput(pressed=True, torque=-2.0), t0=0.8)
  assert g.debug["w"] == 0.0                     # exit-side >= 1.5 Nm: at once
  g = _guide(298.0)
  _run(g, 60)
  same = _run(g, 60, driver=RG.DriverInput(pressed=True, torque=2.0), t0=0.8)     # same-side add keeps the curl
  assert same[-1] != -0.002 and g.release == "" and g.debug["w"] > 0.0
  g = _guide(298.0)
  _run(g, 60)
  light = _run(g, 60, driver=RG.DriverInput(pressed=True, torque=1.0), t0=0.8)    # resting hand toward the circulation: ignored
  assert light[-1] != -0.002 and g.release == ""


def test_early_latch_at_45_deg_alignment_and_not_beyond():
  for dpsi_deg, want in ((73.0, True), (90.0, False)):   # 40 / 57 deg off the tangent (the pose heading is offset 33 deg at 298)
    g = RG.RoundaboutGuide()
    g.set_ring(RING)
    th, r = math.radians(298.0), RING.radius_m + 0.5
    g.pose.bacc, g.bias, g.pose.initialized = 5.0, (0.3, 0.0), True
    g.pose.xo, g.pose.yo, g.pose.psio, g.pose.dpsi = r * math.cos(th), r * math.sin(th), th + math.pi / 2 + math.radians(dpsi_deg), 0.0
    g.pose.t, g.pose.n_good, g.pose.innov, g.pose.last_fix_t = 0.0, 3, 0.5, 0.0
    g.phase = RG.ACTIVE
    g.update(0.1, 7.0, 0.0, -0.002, enabled=True, lat_active=True)
    assert g.latched == want, dpsi_deg


# ---- the failing pass -----------------------------------------------------------------------------------------

def _p13(slow=True):
  from openpilot.selfdrive.controls.lib.tests.test_roundabout_guide import _LAST, simulate
  rows, ring = simulate("0000013c", "after", pool=P13, slow=slow)
  return rows, ring, _LAST["guide"]


def test_0000013c_holds_the_left_turn_at_the_ring_edge_in_band():
  from openpilot.selfdrive.controls.lib.tests.test_roundabout_guide import band_metrics
  rows, ring, g = _p13()
  assert g.latch_t is not None
  assert band_metrics(rows, ring, 90.0) == (0.0, 100.0)
  win = [r for r in rows if g.latch_t + 0.4 <= r[0] <= g.latch_t + 1.4]
  assert win and all(r[6] < -0.0 for r in win) and min(r[6] for r in win) < -0.02       # left of the model (which reads ~0 / right)
  assert sum(r[6] < r[5] - 0.005 for r in win) / len(win) > 0.9
  assert all(-r[6] * r[4] ** 2 <= RG.RING_A_LAT_MAX * 1.0001 for r in rows if r[0] >= g.latch_t)
  post = [r for r in rows if r[0] >= g.latch_t]
  slews = [min(abs((b[6] - b[5]) - (a[6] - a[5])), abs(b[6] - a[6])) / DT for a, b in zip(post, post[1:], strict=False)]
  assert max(slews) <= RG.RING_SLEW + 1e-6                                       # 0.035 slew (correction or held output)


# ---- decel onset ------------------------------------------------------------------------------------------------

def test_decel_onset_is_a_named_constant_near_the_ring():
  assert RB_FUNNEL_M == 450.0 and 130.0 <= RB_DECEL_ONSET_M <= 140.0  # Oct 11: funnel = speed-based reach, low-speed onset 140 m
  v0, v1 = 35 * 0.44704, 18 * 0.44704
  assert (v0 * v0 - v1 * v1) / (2.0 * RB_DECEL_ONSET_M) <= 0.7          # 35 -> 18 mph needs ~0.64 m/s² over 140 m (was 0.48 over 187 m)
