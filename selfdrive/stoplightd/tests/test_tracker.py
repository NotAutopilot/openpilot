"""Synthetic red/green sequences for the stoplight tracker and gate."""
from openpilot.selfdrive.stoplightd import tracker as tk
from openpilot.selfdrive.stoplightd.detect import GREEN, RED, Lamp
from openpilot.selfdrive.stoplightd.gate import StoppedInDrive, in_drive_from_events, speed_from_pose


def red(x=1000.0, y=300.0, size=24.0, chroma=90.0, w=None, core=40, peak=230):
  return Lamp(RED, x, y, w or size, size, 120, core, chroma, peak)


def green(x=1000.0, y=355.0, size=24.0, chroma=90.0, peak=230):
  return Lamp(GREEN, x, y, size, size, 120, 40, chroma, peak)


def run(frames, dt=0.25, t0=10.0):
  tr = tk.StoplightTracker()
  return [tr.update(t0 + i * dt, f) for i, f in enumerate(frames)]


def test_red_then_green_dings_once():
  outs = run([[red()]] * 8 + [[green()]] * 8)
  assert sum(o.chime for o in outs) == 1
  assert outs[7].red
  assert outs[-1].green and not outs[-1].red


def test_red_needs_confirm_before_flag():
  outs = run([[red()]] * 3)
  assert not any(o.red for o in outs)


def test_green_without_red_is_quiet():
  outs = run([[]] * 4 + [[green()]] * 8)
  assert not any(o.chime for o in outs)


def test_green_that_was_already_lit_is_quiet():
  # Another head (or a sign) is green the whole time; our red just goes out.
  outs = run([[red(), green(x=1300)]] * 8 + [[green(x=1300)]] * 8)
  assert not any(o.chime for o in outs)


def test_red_out_without_green_is_quiet():
  # Right turn on red / brake lights going out.
  outs = run([[red()]] * 8 + [[]] * 8)
  assert not any(o.chime for o in outs)


def test_green_elsewhere_is_quiet():
  outs = run([[red()]] * 8 + [[green(x=1600, y=300)]] * 8)
  assert not any(o.chime for o in outs)


def test_green_above_red_is_quiet():
  outs = run([[red()]] * 8 + [[green(y=245)]] * 8)
  assert not any(o.chime for o in outs)


def test_horizontal_head_dings():
  outs = run([[red()]] * 8 + [[green(x=1055, y=300)]] * 8)
  assert sum(o.chime for o in outs) == 1


def test_billboard_like_shift_is_quiet():
  outs = run([[red(size=26)]] * 8 + [[green(x=1015, y=303, size=12)]] * 8)
  assert not any(o.chime for o in outs)


def test_late_green_is_quiet():
  # Red went out, nothing for 2 s, then a green: not the same change.
  outs = run([[red()]] * 8 + [[]] * 8 + [[green()]] * 6)
  assert not any(o.chime for o in outs)


def test_flicker_does_not_ding():
  # One dropped red frame and a one-frame green blip are not a change.
  frames = [[red()]] * 6 + [[]] + [[red()]] * 3 + [[green()]] + [[red()]] * 6
  outs = run(frames)
  assert not any(o.chime for o in outs)


def test_dull_lamps_are_ignored():
  outs = run([[red(chroma=40)]] * 8 + [[green(chroma=50)]] * 8)
  assert not any(o.red for o in outs)
  assert not any(o.chime for o in outs)


def test_reset_allows_next_stop():
  tr = tk.StoplightTracker()
  t = 0.0
  chimes = 0
  for _stop in range(2):
    for f in [[red()]] * 8 + [[green()]] * 6:
      chimes += tr.update(t, f).chime
      t += 0.25
    tr.reset()
  assert chimes == 2


def test_negative_times_work():
  outs = run([[red()]] * 8 + [[green()]] * 8, t0=-30.0)
  assert sum(o.chime for o in outs) == 1


def test_scale_for_small_sensor():
  s = 1344.0 / tk.REF_WIDTH
  tr = tk.StoplightTracker(scale=s)
  outs = []
  for i, f in enumerate([[red(x=700, y=210, size=17)]] * 8 + [[green(x=700, y=248, size=17)]] * 6):
    outs.append(tr.update(i * 0.25, f))
  assert sum(o.chime for o in outs) == 1


def test_gate_opens_after_hold_and_closes_on_move():
  g = StoppedInDrive()
  t = 0.0
  states = []
  for i in range(12):
    g.on_pose(t, 0.01)
    if i % 4 == 0:
      g.on_events(t, [])
    states.append(g.update(t))
    t += 0.25
  assert not states[0] and states[-1]
  g.on_pose(t, 1.5)
  assert not g.update(t)


def test_gate_creep_hysteresis():
  g = StoppedInDrive()
  for i in range(8):
    g.on_pose(i * 0.25, 0.0)
    g.on_events(i * 0.25, [])
    g.update(i * 0.25)
  assert g.open
  g.on_pose(2.0, 0.5)   # creeping in the queue, under MOVE_V
  assert g.update(2.0)


def test_gate_needs_drive():
  g = StoppedInDrive()
  for i in range(8):
    g.on_pose(i * 0.25, 0.0)
    g.on_events(i * 0.25, ["wrongGear"])
    assert not g.update(i * 0.25)
  assert not in_drive_from_events(["reverseGear"])
  assert in_drive_from_events(["gasPressedOverride"])


def test_gate_stale_inputs_close():
  g = StoppedInDrive()
  for i in range(8):
    g.on_pose(i * 0.25, 0.0)
    g.on_events(i * 0.25, [])
    g.update(i * 0.25)
  assert g.open
  assert not g.update(10.0)


def test_speed_from_pose():
  assert speed_from_pose(3.0, 4.0) == 5.0
  assert speed_from_pose(1.0, 0.0, valid=False) is None
  assert speed_from_pose(float("nan"), 0.0) is None


# --- lamp sanity -------------------------------------------------------------

def test_sanity_rejects_non_lenses():
  assert tk.lamp_ok(red())
  assert not tk.lamp_ok(red(size=150))                 # sign / truck body, not a lens
  assert not tk.lamp_ok(red(size=6))
  assert not tk.lamp_ok(red(size=20, w=50))            # wide: brake-light bar, sign stroke
  assert not tk.lamp_ok(red(core=0, chroma=50))        # dull red paint, no bright core
  assert tk.lamp_ok(red(core=0, chroma=70))            # day LED: saturated, no blown core
  assert not tk.lamp_ok(red(peak=80))                  # dim: price-sign digits
  assert not tk.lamp_ok(green(peak=90))
  assert not tk.lamp_ok(red(y=760, size=30))           # at the horizon: car lights
  assert not tk.lamp_ok(red(y=8, size=24))             # cut by the top edge
  assert not tk.lamp_ok(red(y=60, size=12))            # tiny and far too high: tower light
  assert tk.lamp_ok(red(y=60, size=40))                # close mast-arm signal


def test_brake_light_pair_near_horizon_dropped():
  pair = [red(x=900, y=730, size=30), red(x=1100, y=732, size=28)]
  reds, _ = tk.lamp_filter(pair)
  assert reds == []
  # Two signal heads side by side, high up: kept.
  heads = [red(x=900, y=300, size=24), red(x=1100, y=301, size=24)]
  reds, _ = tk.lamp_filter(heads)
  assert len(reds) == 2


def test_wandering_red_never_confirms():
  # Billboard content slides; a lens stays put.
  frames = [[red(x=900 + 6 * i)] for i in range(16)]
  outs = run(frames)
  assert not any(o.red for o in outs)


# --- head arbitration --------------------------------------------------------

MAIN_X = 1000.0     # straight ahead
SIDE_X = 300.0      # a left-turn head well off to the side


def test_side_arrow_green_while_his_red_stays_red_is_quiet_then_his_dings():
  """Reported case: the left arrow goes green first, the driver waits; his light
  ahead goes red -> green later. Only his light dings."""
  both_red = [red(x=MAIN_X), red(x=SIDE_X)]
  arrow_green = [red(x=MAIN_X), green(x=SIDE_X)]
  main_green = [green(x=MAIN_X), green(x=SIDE_X)]
  outs = run([both_red] * 8 + [arrow_green] * 40 + [main_green] * 8)
  chimes = [i for i, o in enumerate(outs) if o.chime]
  assert len(chimes) == 1 and chimes[0] >= 48
  # While the arrow is green his light is still red: the HUD says RED.
  assert all(o.red for o in outs[12:46])
  assert outs[-1].green and not outs[-1].red


def test_side_head_ding_does_not_use_up_the_stop():
    """A side head dings first (his head not seen yet); his head ahead
    changes later and still dings."""
    frames = [[red(x=SIDE_X)]] * 8 + [[green(x=SIDE_X)]] * 8 + \
             [[green(x=SIDE_X), red(x=MAIN_X)]] * 30 + [[green(x=SIDE_X), green(x=MAIN_X)]] * 6
    outs = run(frames)
    chimes = [i for i, o in enumerate(outs) if o.chime]
    assert len(chimes) == 2 and chimes[0] < 16 <= 46 <= chimes[1]


def test_less_central_head_after_his_ding_is_quiet():
    frames = [[red(x=MAIN_X), red(x=SIDE_X)]] * 8 + [[green(x=MAIN_X), red(x=SIDE_X)]] * 30 + \
             [[green(x=MAIN_X), green(x=SIDE_X)]] * 8
    outs = run(frames)
    assert sum(o.chime for o in outs) == 1


def test_heads_of_one_approach_ding_once():
  frames = [[red(x=900), red(x=1150)]] * 8 + [[green(x=900), green(x=1150)]] * 8
  assert sum(o.chime for o in run(frames)) == 1
  # Also when the second head lags a few frames.
  frames = [[red(x=900), red(x=1150)]] * 8 + [[green(x=900), red(x=1150)]] * 3 + [[green(x=900), green(x=1150)]] * 8
  assert sum(o.chime for o in run(frames)) == 1


def test_main_head_next_to_central_red_turn_head_still_dings():
  """Stop 10 (Oct 24): his light was a little right of center while a red
  left-turn head sat dead ahead and stayed red."""
  frames = [[red(x=964), red(x=1134)]] * 8 + [[red(x=964), green(x=1134)]] * 8
  assert sum(o.chime for o in run(frames)) == 1


def test_same_head_does_not_re_ding_without_a_new_red_phase():
  frames = [[red()]] * 8 + [[green()]] * 4 + [[red()]] * 6 + [[green()]] * 4
  assert sum(o.chime for o in run(frames)) == 1


def test_same_head_dings_again_after_a_full_new_red():
  # Long stop: green for 6 s (he did not go), red again for 8 s, green again.
  frames = [[red()]] * 8 + [[green()]] * 24 + [[red()]] * 32 + [[green()]] * 6
  assert sum(o.chime for o in run(frames)) == 2
