from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.constants import (
  MODE_CAP, MODE_FOLLOW, MODE_OFF,
)
from openpilot.selfdrive.mapd.map_speed_policy import (
  MapCruiseHold, apply_map_speed_kph, decide_map_cruise, is_cruise_stalk_hold_step,
  is_cruise_stalk_step, is_cruise_stalk_tip_step,
  should_write_preap_pedal, slew_map_speed_ms,
)


def _follow_hud(dec, map_kph: float) -> float:
  """Mirror card.py: seed / sticky win; else apply_map_speed (Follow)."""
  if dec.seed_kph is not None:
    return dec.seed_kph
  if dec.sticky:
    return dec.driver_kph
  return apply_map_speed_kph(
    dec.driver_kph, map_kph, mode=MODE_FOLLOW, engaged=True,
    op_long_software_cruise=True, driver_override=dec.follow_override,
  )


def test_follow_delayed_engage_rising_keeps_sticky():
  """pedalLongActive rising after resume_held must not forget 55-in-65."""
  hold = MapCruiseHold()
  posted = 65 * CV.MPH_TO_KPH
  sticky = 55 * CV.MPH_TO_KPH
  ego = 30 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=posted, posted_kph=posted,
    engage_rising=True, now=0.0,
  )
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=sticky, posted_kph=posted,
    engage_rising=False, now=1.0, stalk_pressed=True,
  )
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=sticky, posted_kph=posted,
    engage_rising=False, now=2.0, long_active=False, traveled_kph=ego,
  )
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=sticky, posted_kph=posted,
    engage_rising=False, now=3.0, resume_held=True, long_active=False,
    traveled_kph=ego,
  )
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=sticky, posted_kph=posted,
    engage_rising=True, now=4.0, resume_held=False, long_active=True,
    traveled_kph=ego,
  )
  assert dec.sticky
  assert abs(dec.driver_kph - sticky) < 1e-6
  assert abs(hold.held_max_kph - sticky) < 1e-6
  if dec.seed_kph is not None:
    assert abs(dec.seed_kph - sticky) < 1e-6


def test_maps_double_set_uses_posted():
  hold = MapCruiseHold()
  posted = 65 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=posted, posted_kph=posted,
    engage_rising=True, now=0.0,
  )
  sticky = 55 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=sticky, posted_kph=posted,
    engage_rising=False, now=1.0, stalk_pressed=True,
  )
  traveled = 40 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=sticky, posted_kph=posted,
    engage_rising=False, now=2.0, take_speed_now=True, traveled_kph=traveled,
  )
  assert not dec.sticky
  assert dec.seed_kph is not None
  assert abs(dec.seed_kph - posted) < 1e-6
  assert abs(hold.held_max_kph - posted) < 1e-6


def test_unknown_posted_does_not_invent_or_wipe():
  hold = MapCruiseHold()
  posted = 65 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=posted, posted_kph=posted,
    engage_rising=True, now=0.0,
  )
  sticky = 55 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=sticky, posted_kph=posted,
    engage_rising=False, now=1.0, stalk_pressed=True,
  )
  last_posted = hold.last_posted_kph
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=sticky, posted_kph=None,
    engage_rising=False, now=2.0,
  )
  assert dec.sticky
  assert abs(dec.driver_kph - sticky) < 1e-6
  assert hold.last_posted_kph == last_posted
  # GPS returns same posted: still sticky 55, not Follow to 65.
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=sticky, posted_kph=posted,
    engage_rising=False, now=3.0,
  )
  assert dec.sticky
  assert abs(_follow_hud(dec, posted) - sticky) < 1e-6


def test_maps_off_never_rebases_toward_invented_posted():
  hold = MapCruiseHold()
  held = 55 * CV.MPH_TO_KPH
  fake_posted = 65 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_OFF, raw_kph=held, posted_kph=None,
    engage_rising=True, now=0.0, take_speed_now=True, traveled_kph=held,
  )
  for t in (1.0, 5.0, 20.0):
    dec = decide_map_cruise(
      hold, engaged=True, mode=MODE_OFF, raw_kph=held, posted_kph=fake_posted,
      engage_rising=False, now=t,
    )
    assert abs(dec.driver_kph - held) < 1e-6
    assert hold.last_posted_kph is None


def test_sticky_below_limit_survives_past_ten_seconds():
  """Stalk to a-5 must still be a-5 after >10s with unchanged posted limit."""
  hold = MapCruiseHold()
  a = 45 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=a,
    engage_rising=True, now=0.0,
  )
  below = a - 5 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=below, posted_kph=a,
    engage_rising=False, now=1.0, stalk_pressed=True,
  )
  assert dec.sticky
  assert hold.follow_override_until == 0.0
  for t in (11.5, 15.0, 60.0):
    dec = decide_map_cruise(
      hold, engaged=True, mode=MODE_FOLLOW, raw_kph=below, posted_kph=a,
      engage_rising=False, now=t, stalk_pressed=False,
    )
    assert dec.sticky, f"sticky lost at t={t}"
    assert hold.follow_override_until == 0.0
    assert dec.seed_kph is None
    assert abs(_follow_hud(dec, a) - below) < 1e-6
    # apply_map_speed without sticky would Follow-raise to a; card must not.
    expired = apply_map_speed_kph(
      dec.driver_kph, a, mode=MODE_FOLLOW, engaged=True,
      op_long_software_cruise=True, driver_override=False,
    )
    assert abs(expired - a) < 1e-6
    assert abs(_follow_hud(dec, a) - expired) > 1.0


def test_cruise_stalk_step_is_1_or_5_not_ego_jump():
  a = 45 * CV.MPH_TO_KPH
  assert is_cruise_stalk_step(a, a - 5 * CV.MPH_TO_KPH)
  assert is_cruise_stalk_step(a, a + 1 * CV.MPH_TO_KPH)
  assert is_cruise_stalk_step(a, a - 1.0)  # metric 1 kph
  assert is_cruise_stalk_step(a, a + 5.0)
  assert not is_cruise_stalk_step(a, 70 * CV.MPH_TO_KPH)
  assert not is_cruise_stalk_step(a, a)
  # Tip vs hold: 1 mph / 1 kph / MPH_TO_KPH are tips; 5 mph / 5 kph are holds.
  assert is_cruise_stalk_tip_step(a, a + 1 * CV.MPH_TO_KPH)
  assert is_cruise_stalk_tip_step(a, a - 1.0)
  assert not is_cruise_stalk_tip_step(a, a + 5 * CV.MPH_TO_KPH)
  assert not is_cruise_stalk_tip_step(a, a + 5.0)
  assert is_cruise_stalk_hold_step(a, a + 5 * CV.MPH_TO_KPH)
  assert is_cruise_stalk_hold_step(a, a - 5.0)
  assert not is_cruise_stalk_hold_step(a, a + 1 * CV.MPH_TO_KPH)
  assert not is_cruise_stalk_hold_step(a, a - 1.0)


def test_stalk_plus_minus_changes_set_without_button_events():
  """pre-AP buttonEvents are unreliable; a 5 mph pedal_speed step must move MAX."""
  hold = MapCruiseHold()
  a = 45 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=a,
    engage_rising=True, now=0.0,
  )
  down = a - 5 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=down, posted_kph=a,
    engage_rising=False, now=1.0, stalk_pressed=False,
  )
  assert dec.sticky
  assert abs(_follow_hud(dec, a) - down) < 1e-6
  up = down + 5 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=up, posted_kph=a,
    engage_rising=False, now=2.0, stalk_pressed=False,
  )
  assert abs(_follow_hud(dec, a) - up) < 1e-6
  above = a + 5 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=above, posted_kph=a,
    engage_rising=False, now=3.0, stalk_pressed=False,
  )
  assert dec.sticky
  assert abs(_follow_hud(dec, a) - above) < 1e-6
  # Cap: stalk up cannot exceed posted; stalk down still lowers MAX.
  hold_c = MapCruiseHold()
  decide_map_cruise(
    hold_c, engaged=True, mode=MODE_CAP, raw_kph=a, posted_kph=a,
    engage_rising=True, now=0.0,
  )
  dec = decide_map_cruise(
    hold_c, engaged=True, mode=MODE_CAP, raw_kph=above, posted_kph=a,
    engage_rising=False, now=1.0, stalk_pressed=False,
  )
  cap_out = apply_map_speed_kph(
    dec.driver_kph, a, mode=MODE_CAP, engaged=True, op_long_software_cruise=True,
    driver_override=dec.follow_override,
  )
  assert abs(cap_out - a) < 1e-6
  decide_map_cruise(
    hold_c, engaged=True, mode=MODE_CAP, raw_kph=a, posted_kph=a,
    engage_rising=False, now=2.0, stalk_pressed=False,
  )
  dec = decide_map_cruise(
    hold_c, engaged=True, mode=MODE_CAP, raw_kph=down, posted_kph=a,
    engage_rising=False, now=3.0, stalk_pressed=False,
  )
  assert dec.sticky
  assert abs(dec.driver_kph - down) < 1e-6


def test_sticky_hud_ignores_anticipatory_map_kph():
  """Card sticky path must keep 66 even if map_kph slewed toward upcoming 50."""
  hold = MapCruiseHold()
  a = 60 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=a,
    engage_rising=True, now=0.0,
  )
  above = a + 5 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=above, posted_kph=a,
    engage_rising=False, now=1.0, stalk_pressed=False,
  )
  assert dec.sticky
  anticipated = 50 * CV.MPH_TO_KPH
  assert abs(_follow_hud(dec, anticipated) - above) < 1e-6
  assert dec.seed_kph is not None  # real 5 mph stalk step writes once
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=above, posted_kph=a,
    engage_rising=False, now=2.0, stalk_pressed=False,
  )
  assert dec.sticky
  assert dec.seed_kph is None
  assert abs(_follow_hud(dec, anticipated) - above) < 1e-6
  assert not should_write_preap_pedal(dec.seed_kph, _follow_hud(dec, anticipated), above)


def test_follow_holds_absolute_set_until_posted_changes():
  """Justin: set 55 in a 50 stays 55 until posted changes; set 45 in a 50, same."""
  hold = MapCruiseHold()
  a = 50 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=a,
    engage_rising=True, now=0.0,
  )
  above = 55 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=above, posted_kph=a,
    engage_rising=False, now=1.0, stalk_pressed=True,
  )
  assert dec.sticky
  assert hold.follow_override_until == 0.0
  assert abs(_follow_hud(dec, a) - above) < 1e-6
  for t in (11.1, 15.0, 60.0):
    dec = decide_map_cruise(
      hold, engaged=True, mode=MODE_FOLLOW, raw_kph=above, posted_kph=a,
      engage_rising=False, now=t, stalk_pressed=False,
    )
    assert dec.sticky, f"55 mph sticky lost at t={t}"
    assert dec.seed_kph is None
    assert abs(_follow_hud(dec, a) - above) < 1e-6
  b = 35 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=above, posted_kph=b,
    engage_rising=False, now=61.0, stalk_pressed=False,
  )
  assert not dec.sticky
  assert dec.seed_kph is None  # decrease must not cliff MAX to posted
  assert abs(_follow_hud(dec, b) - b) < 1e-6

  hold2 = MapCruiseHold()
  decide_map_cruise(
    hold2, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=a,
    engage_rising=True, now=0.0,
  )
  below = 45 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold2, engaged=True, mode=MODE_FOLLOW, raw_kph=below, posted_kph=a,
    engage_rising=False, now=1.0, stalk_pressed=True,
  )
  assert dec.sticky
  assert abs(_follow_hud(dec, a) - below) < 1e-6
  dec = decide_map_cruise(
    hold2, engaged=True, mode=MODE_FOLLOW, raw_kph=below, posted_kph=a,
    engage_rising=False, now=60.0, stalk_pressed=False,
  )
  assert dec.sticky
  assert dec.seed_kph is None
  assert abs(_follow_hud(dec, a) - below) < 1e-6
  dec = decide_map_cruise(
    hold2, engaged=True, mode=MODE_FOLLOW, raw_kph=below, posted_kph=b,
    engage_rising=False, now=61.0, stalk_pressed=False,
  )
  assert not dec.sticky
  assert dec.seed_kph is None
  assert abs(_follow_hud(dec, b) - b) < 1e-6


def test_cap_does_not_exceed_limit_when_raising():
  hold = MapCruiseHold()
  a = 45 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_CAP, raw_kph=a, posted_kph=a,
    engage_rising=True, now=0.0,
  )
  above = a + 5 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_CAP, raw_kph=above, posted_kph=a,
    engage_rising=False, now=1.0,
  )
  assert not dec.sticky
  out = apply_map_speed_kph(
    dec.driver_kph, a, mode=MODE_CAP, engaged=True, op_long_software_cruise=True,
    driver_override=dec.follow_override,
  )
  assert abs(out - a) < 1e-6


def test_slew_rate_limits_map_max_steps():
  # 0.80 m/s² × 0.1 s = 0.08 m/s max step
  out = slew_map_speed_ms(30.0, 20.0, 0.1, 0.80)
  assert abs(out - 29.92) < 1e-9
  done = slew_map_speed_ms(20.05, 20.0, 0.1, 0.80)
  assert done == 20.0


def test_should_write_preap_pedal_on_raise_not_every_frame():
  a = 45 * CV.MPH_TO_KPH
  b = 50 * CV.MPH_TO_KPH
  # Engage / posted / stalk-step seed writes once.
  assert should_write_preap_pedal(a, a, a)
  assert should_write_preap_pedal(a, a, None)
  # Sticky hold / same MAX every Follow frame: do not write (that ate stalk).
  assert not should_write_preap_pedal(None, a, a)
  # Stalk up / Follow posted raise: HUD rose vs last pedal write.
  assert should_write_preap_pedal(None, b, a)
  # No last write yet and no seed: leave CI pedal alone.
  assert not should_write_preap_pedal(None, b, None)
  # Decrease without seed: planner brakes from HUD MAX; do not clobber stalk down.
  assert not should_write_preap_pedal(None, a, b)


def test_sticky_hold_does_not_overwrite_ci_stalk_step():
  """After CI.update steps pedal, card must not write the old sticky hold.

  Order in card.py: CI.update (stalk +/-) then overlay. Writing seed_kph
  every sticky frame undoes that 1/5 mph step.
  """
  hold = MapCruiseHold()
  a = 50 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=a,
    engage_rising=True, now=0.0,
  )
  last_pedal = a
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=a,
    engage_rising=False, now=1.0, stalk_pressed=False,
  )
  hud = _follow_hud(dec, a)
  assert not dec.sticky
  assert dec.seed_kph is None
  assert not should_write_preap_pedal(dec.seed_kph, hud, last_pedal)

  above = a + 5 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=above, posted_kph=a,
    engage_rising=False, now=2.0, stalk_pressed=False,
  )
  hud = _follow_hud(dec, a)
  assert dec.sticky
  assert abs(hud - above) < 1e-6
  assert should_write_preap_pedal(dec.seed_kph, hud, last_pedal)
  write_val = dec.seed_kph if dec.seed_kph is not None else hud
  assert abs(write_val - above) < 1e-6

  last_pedal = above
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=above, posted_kph=a,
    engage_rising=False, now=3.0, stalk_pressed=False,
  )
  hud = _follow_hud(dec, a)
  assert dec.sticky
  assert dec.seed_kph is None
  assert abs(hud - above) < 1e-6
  assert not should_write_preap_pedal(dec.seed_kph, hud, last_pedal)

  down = above - 5 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=down, posted_kph=a,
    engage_rising=False, now=4.0, stalk_pressed=False,
  )
  hud = _follow_hud(dec, a)
  assert dec.sticky
  assert abs(hud - down) < 1e-6
  write_val = dec.seed_kph if dec.seed_kph is not None else hud
  assert abs(write_val - down) < 1e-6

  # Button event this frame, CS.speed still the old hold: do not write `a`
  # over CI's in-flight +5.
  hold2 = MapCruiseHold()
  decide_map_cruise(
    hold2, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=a,
    engage_rising=True, now=0.0,
  )
  dec = decide_map_cruise(
    hold2, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=a,
    engage_rising=False, now=2.0, stalk_pressed=True,
  )
  hud = _follow_hud(dec, a)
  assert not should_write_preap_pedal(dec.seed_kph, hud, a)
  assert dec.seed_kph is None


def test_hud_current_speed_is_wheel_ego_not_cluster_or_max():
  """Top-middle 3X speed was ~56 mph vs ~45 actual — that is 90 kph MAX.

  vEgoCluster is DI_digitalSpeed, which pre-AP also uses as cruiseState.speed.
  Map-speed writes MAX into vCruise / pedal_speed / cruiseState.speed. Live
  speed must be ESP/wheel vEgo only.
  """
  from pathlib import Path
  root = Path(__file__).resolve().parents[3]
  for rel in (
    "selfdrive/ui/onroad/hud_renderer.py",
    "selfdrive/ui/mici/onroad/hud_renderer.py",
  ):
    src = (root / rel).read_text()
    assert "self.speed = max(0.0, float(car_state.vEgo) * speed_conversion)" in src
    assert "v_ego = v_ego_cluster if" not in src
    assert "self.speed = md.speedLimit" not in src
    assert "self.speed = self.map_speed_limit" not in src
    assert "self.speed = self.set_speed" not in src
    # MAX box is vCruiseCluster; LIMIT sign is OSM. Do not paint either as live.
  card = (root / "selfdrive/car/card.py").read_text()
  assert "vEgoCluster" not in card
  assert "CS.vEgo =" not in card
  assert "CS.vCruise =" in card


def test_map_speed_submenu_wires_params():
  """Menu wiring without importing raylib / cereal UI."""
  from pathlib import Path
  root = Path(__file__).resolve().parents[3]
  tici = (root / "selfdrive/ui/layouts/settings/map_speed.py").read_text()
  mici = (root / "selfdrive/ui/mici/layouts/settings/map_speed.py").read_text()
  nap = (root / "selfdrive/ui/layouts/settings/nap.py").read_text()
  nap_mici = (root / "selfdrive/ui/mici/layouts/settings/nap.py").read_text()
  for src in (tici, mici):
    for key in ("NAPMapSpeedMode", "NAPMapSpeedOffsetMph", "NAPMapSpeedLookahead"):
      assert key in src
    assert "NAPMapSpeedAccel" not in src
  content = (root / "selfdrive/ui/layouts/settings/nap_content.py").read_text()
  docs = (root / "docs-nap/map-speed.md").read_text()
  assert "self._scroller.add_widgets" in mici
  assert "Acceleration" not in tici
  assert "acceleration" not in mici
  assert "1.20 m/s² at Normal" not in tici
  assert "pauses Follow for 10s" not in tici
  assert "holds until the posted limit changes" in content
  assert "lookahead pauses while that set is active" in content
  assert "decreases still ease with lookahead" not in tici
  assert "decreases still ease with lookahead" not in content
  assert "kin + 110 m" in docs
  assert "1.5 s GPS lag" not in tici
  assert "A higher limit far ahead never raises MAX" in content
  assert "A higher limit ahead never raises MAX early" not in tici
  assert "A higher limit ahead never raises MAX early" not in content
  assert tici.index("_all_items.append(self._refresh_btn)") < tici.index("_all_items.append(self._db_status)")
  assert "Map Speed Limit" in nap
  assert "Radar Settings" in nap
  assert "Driving Mannerisms" in nap
  assert "map speed limit" in nap_mici
  assert "radar settings" in nap_mici
  assert "driving mannerisms" in nap_mici
  manner = (root / "selfdrive/ui/layouts/settings/driving_mannerisms.py").read_text()
  manner_mici = (root / "selfdrive/ui/mici/layouts/settings/driving_mannerisms.py").read_text()
  assert "FOLLOW_DISTANCE_CITY_DESCRIPTION" in manner
  assert "FOLLOW_DISTANCE_HWY_DESCRIPTION" in manner
  assert "MAP_SPEED_ACCEL_DESCRIPTION" in manner
  assert "ADAPTIVE_ACCEL_DESCRIPTION" in manner
  assert "MAP_SPEED_ACCEL_DESCRIPTION" in content
  assert "1 lazy" in content
  assert "same Accel 1–10 gradient" in content
  assert "last-mph taper" in content
  assert "0.20 / 0.30 / 0.50" not in content
  assert "not a harder brake" in content
  assert "steps the active 1–7" in content
  assert "No lead:" in content
  assert "follow distance" in manner_mici
  assert "Adaptive Accel Limits" not in nap
  assert "Follow Distance" not in nap
  assert "Soft Lateral Handoff" not in nap
  assert "adaptive accel limits" not in nap_mici
  assert "follow distance" not in nap_mici
  assert "soft lateral handoff" not in nap_mici
  assert "Refresh maps" in tici
  assert '"Back To"' in tici
  assert "←" not in tici
  assert "Check for map updates" not in tici
  assert "refresh maps" in mici
  assert "check for map updates" not in mici
  assert "scripts.nap.refresh_osm_maps" in nap
  assert "scripts.nap.refresh_osm_maps" in mici
  assert tici.index("_all_items.append(self._refresh_btn)") < tici.index("_all_items.append(self._download_btn)")
  mici_widgets = mici.split("self._scroller.add_widgets", 1)[1]
  assert mici_widgets.index("refresh_maps_btn") < mici_widgets.index("download_maps_btn")
  assert "NAPMapSpeedAccel" in (root / "common/params_keys.h").read_text()
  assert "NAPMapSpeedDbRevision" in (root / "common/params_keys.h").read_text()
  assert "NAPMapSpeedDbSha256" in (root / "common/params_keys.h").read_text()


def test_driving_mannerisms_submenu_wires_params():
  """Driving Mannerisms submenu mirrors Map Speed Limit open/close/render."""
  from pathlib import Path
  root = Path(__file__).resolve().parents[3]
  tici = (root / "selfdrive/ui/layouts/settings/driving_mannerisms.py").read_text()
  mici = (root / "selfdrive/ui/mici/layouts/settings/driving_mannerisms.py").read_text()
  nap = (root / "selfdrive/ui/layouts/settings/nap.py").read_text()
  nap_mici = (root / "selfdrive/ui/mici/layouts/settings/nap.py").read_text()
  for src in (tici, mici):
    for key in ("ADAPTIVE_ACCEL", "FOLLOW_DISTANCE", "NAP_DRIVER_LAT_HANDOFF", "NAP_ONE_PEDAL_LONG", "NAP_HYPERMILE", "NAP_HYPERMILE_STEP_DOWN", "NAPMapSpeedAccel"):
      assert key in src
  content = (root / "selfdrive/ui/layouts/settings/nap_content.py").read_text()
  assert "Hypermile" in tici
  assert "One-Pedal Long" in tici
  assert "Adaptive Accel" in tici
  assert "Acceleration" in tici
  assert "MAP_SPEED_ACCEL_DESCRIPTION" in tici
  assert "ADAPTIVE_ACCEL_DESCRIPTION" in tici
  assert "ONE_PEDAL_LONG_DESCRIPTION" in tici
  assert "stays Accel 5" in content
  assert "Lead still owns follow" in content
  assert "1 lazy" in content
  assert "same Accel 1–10 gradient" in content
  assert "0.20 / 0.30 / 0.50" not in content
  assert tici.index('"Acceleration"') < tici.index('"Adaptive Accel"')
  assert tici.index("self._accel_buttons") < tici.index("self._adaptive_accel")
  assert tici.index("self._adaptive_accel") < tici.index("self._follow_city_buttons")
  assert tici.index("self._follow_city_buttons") < tici.index("self._follow_hwy_buttons")
  assert tici.index("self._follow_hwy_buttons") < tici.index("self._lat_handoff")
  assert tici.index("self._lat_handoff") < tici.index("self._one_pedal")
  assert tici.index("self._one_pedal") < tici.index("self._hypermile")
  assert "City Follow Distance" in tici
  assert "Highway Follow Distance" in tici
  assert "Soft Lateral Handoff" in tici
  assert "Simulate Look" not in tici
  assert "False Alert Ignore" not in tici
  assert "NAP_DM_SIMULATE_LOOKING" not in tici
  assert "NAP_DM_FALSE_ALERT_IGNORE" not in tici
  assert '"Back To"' in tici
  assert "Return to NAP settings." in tici
  assert "hypermile" in mici
  assert "one-pedal long" in mici
  assert "adaptive accel" in mici
  assert '"acceleration"' in mici
  assert "NAPMapSpeedAccel" in mici
  mici_widgets = mici.split("self._scroller.add_widgets", 1)[1]
  assert mici_widgets.index("self._accel") < mici_widgets.index("adaptive_accel")
  assert mici_widgets.index("adaptive_accel") < mici_widgets.index("self._follow_distance_city")
  assert mici_widgets.index("self._follow_distance_city") < mici_widgets.index("self._follow_distance_hwy")
  assert mici_widgets.index("self._follow_distance_hwy") < mici_widgets.index("lat_handoff")
  assert mici_widgets.index("lat_handoff") < mici_widgets.index("one_pedal")
  assert mici_widgets.index("one_pedal") < mici_widgets.index("hypermile")
  assert "city follow distance" in mici
  assert "highway follow distance" in mici
  assert "soft lateral handoff" in mici
  assert "simulate look" not in mici
  assert "false alert ignore" not in mici
  assert "self._scroller.add_widgets" in mici
  assert "self._accel.refresh()" in mici
  assert "DrivingMannerismsLayout" in nap
  assert "Accel feel, follow, soft lat" in nap
  assert "one-pedal" in nap
  assert "lookahead, acceleration" not in nap
  assert "_open_driving_mannerisms" in nap
  assert "_close_driving_mannerisms" in nap
  assert 'self._page = "driving_mannerisms"' in nap
  assert "self._driving_mannerisms_page.render" in nap
  assert "NAP_DM_SIMULATE_LOOKING" in nap
  assert "put_bool(NAP_DM_SIMULATE_LOOKING, True)" in nap
  assert "NAP_DM_FALSE_ALERT_IGNORE" in nap
  assert "put_bool(NAP_DM_FALSE_ALERT_IGNORE, False)" in nap
  keys = (root / "common/params_keys.h").read_text()
  assert "NAPDmSimulateLooking" in keys
  assert "NAPDmFalseAlertIgnore" in keys
  assert "DrivingMannerismsLayoutMici" in nap_mici
  assert "driving mannerisms" in nap_mici
  # Map Speed Limit submenu must stay on the main NAP list.
  assert "Map Speed Limit" in nap
  assert "map speed limit" in nap_mici


def test_planner_and_mpc_keep_radar_after_map_cap():
  """Regression: map cap runs before mpc.update(radarState, v_cruise)."""
  from pathlib import Path
  root = Path(__file__).resolve().parents[3]
  planner = (root / "selfdrive/controls/lib/longitudinal_planner.py").read_text()
  mpc = (root / "selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py").read_text()
  cap_at = planner.find("cap_planner_v_cruise_ms")
  mpc_at = planner.find("self.mpc.update(sm['radarState'], v_cruise")
  hold_at = planner.find("map_in_track_deadband(v_ego, v_hud_ms)")
  track_at = planner.find("a_brake = map_track_decel_ms2")
  assert 0 <= cap_at < mpc_at
  assert 0 <= mpc_at < hold_at < track_at
  assert "map_brake_a_ms2" in planner
  assert "map_track_accel_ms2" in planner
  assert "resolve_lead_close_hold" in planner
  assert "_apply_lead_follow" in planner
  assert "min(float(output_a_target), a_brake)" in planner
  assert "if float(output_a_target) >= 0.0:" in planner
  assert "output_a_target = 0.0" in planner
  assert "np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle])" in mpc
  assert "self.params[:,2] = np.min(x_obstacles, axis=1)" in mpc
  card = (root / "selfdrive/car/card.py").read_text()
  assert "pedalLongActive" in card
  assert "stalk_pressed=stalk_pressed" in card
  assert "take_speed_now=take_speed_now" in card
  assert "resume_held=resume_held" in card
  assert "_write_preap_pedal_speed" in card
  assert "should_write_preap_pedal" in card
  assert "_adopt_preap_fsm_held_max" in card
  assert "long_active=soft_long" in card
  # Must not seed/overlay on lateral-only first pull (CC.enabled).
  assert "engage_rising = long_active and not long_active_prev" in card
  # Must not clobber stalk by writing Follow HUD onto pedal every frame.
  assert "should_write_preap_pedal(seed_kph, preap_v_cruise_kph, self._last_pedal_kph)" in card
  # Curve snapshot/restore: freeze posted flicker, restore pre-curve MAX.
  assert "CurveMaxHold" in card
  assert "begin_cycle" in card
  assert "restore_seed_kph" in card
  assert "if long_active and dec.seed_kph is not None:" not in card
  # Sticky hold must not slew toward nextSpeedLimit.
  assert "not dec.sticky:" in card
  assert "sticky_hold = self._map_hold.sticky_set_kph is not None" not in card
  planner_src = planner
  assert "output_a_target = a_up" in planner_src
  assert "min(float(output_a_target), a_up)" not in planner_src
  assert "map_climb_replaces_mpc" in planner_src
  assert "has_valid_lead" in planner_src
  assert "leadOne.status" in planner_src
  mapd = (root / "selfdrive/mapd/mapd.py").read_text()
  osm = (root / "selfdrive/mapd/osm_db.py").read_text()
  constants = (root / "selfdrive/mapd/constants.py").read_text()
  assert "v_ego_ms=v_ego_ms" in mapd
  assert "osm_sign_lead_m(v_ego_ms)" in osm
  assert "OSM_SIGN_LEAD_S = 1.5" in constants
  assert "OSM_SIGN_LEAD_M" not in constants
  assert "DECREASE_START_MARGIN_M = 110.0" in constants
  assert "Do not snap posted down" in osm
  policy = (root / "selfdrive/mapd/map_speed_policy.py").read_text()
  assert "float(posted_kph) if raised else None" in policy
  # Delayed pedalLongActive rising after one SET must not take traveled.
  assert "not bool(resume_held) and hold.held_max_kph is None" in policy
  # Sticky / stalk path unchanged.
  assert "should_write_preap_pedal(seed_kph, preap_v_cruise_kph, self._last_pedal_kph)" in card


def _decision_fields(dec, hold):
  seed = None if dec.seed_kph is None else round(float(dec.seed_kph), 4)
  return (
    bool(dec.sticky),
    seed,
    round(float(hold.held_max_kph), 4),
    hold.sticky_set_kph is None,
  )


def test_none_to_valid_posted_matches_a_normal_limit_change():
  """A map dropout that gains a posted limit is a normal limit change.

  Engage while the sign is missing, then the posted limit returns at 3 s
  and at 30 s: both adopt it. A valid → invalid → same valid flicker does
  not move MAX. A stalk during the dropout is treated the same way a stalk
  is treated before a 55 → 70 change.
  """
  posted = 70 * CV.MPH_TO_KPH
  ego = 42.4 * CV.MPH_TO_KPH

  def engage_dropout():
    hold = MapCruiseHold()
    dec = decide_map_cruise(
      hold, engaged=True, mode=MODE_FOLLOW, raw_kph=ego, posted_kph=None,
      engage_rising=True, now=0.0, take_speed_now=True, traveled_kph=ego,
    )
    assert dec.sticky
    assert abs(dec.seed_kph - ego) < 1e-6
    assert hold.last_posted_kph is None
    return hold

  for t_back in (3.0, 30.0):
    hold = engage_dropout()
    decide_map_cruise(
      hold, engaged=True, mode=MODE_FOLLOW, raw_kph=ego, posted_kph=None,
      engage_rising=False, now=t_back - 0.05, traveled_kph=ego,
    )
    assert abs(hold.held_max_kph - ego) < 1e-6
    dec = decide_map_cruise(
      hold, engaged=True, mode=MODE_FOLLOW, raw_kph=ego, posted_kph=posted,
      engage_rising=False, now=t_back, traveled_kph=ego,
    )
    assert dec.seed_kph is not None
    assert abs(dec.seed_kph - posted) < 1e-6
    assert not dec.sticky
    assert abs(hold.held_max_kph - posted) < 1e-6
    assert abs(_follow_hud(dec, posted) - posted) < 1e-6
    # The next frame must not treat the old traveled speed as a stalk.
    dec = decide_map_cruise(
      hold, engaged=True, mode=MODE_FOLLOW, raw_kph=ego, posted_kph=posted,
      engage_rising=False, now=t_back + 0.05, traveled_kph=ego,
    )
    assert not dec.sticky
    assert abs(_follow_hud(dec, posted) - posted) < 1e-6

  # Same limit returns and MAX already equals it: no pedal write, no rebase.
  hold = MapCruiseHold()
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=posted, posted_kph=posted,
    engage_rising=True, now=0.0, take_speed_now=True, traveled_kph=posted,
  )
  held_before = hold.held_max_kph
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=posted, posted_kph=None,
    engage_rising=False, now=1.0, traveled_kph=posted,
  )
  assert hold.last_posted_kph is not None
  assert abs(hold.held_max_kph - posted) < 1e-6
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=posted, posted_kph=posted,
    engage_rising=False, now=1.5, traveled_kph=posted,
  )
  assert dec.seed_kph is None
  assert abs(hold.held_max_kph - held_before) < 1e-6
  assert abs(_follow_hud(dec, posted) - posted) < 1e-6
  assert not should_write_preap_pedal(dec.seed_kph, _follow_hud(dec, posted), posted)

  # Stalk, then the limit changes: dropout and a normal 55 → 70 match.
  normal = MapCruiseHold()
  start = 55 * CV.MPH_TO_KPH
  stalk = start + 5 * CV.MPH_TO_KPH
  decide_map_cruise(
    normal, engaged=True, mode=MODE_FOLLOW, raw_kph=start, posted_kph=start,
    engage_rising=True, now=0.0,
  )
  decide_map_cruise(
    normal, engaged=True, mode=MODE_FOLLOW, raw_kph=stalk, posted_kph=start,
    engage_rising=False, now=1.0,
  )
  assert normal.sticky_set_kph is not None
  dec_normal = decide_map_cruise(
    normal, engaged=True, mode=MODE_FOLLOW, raw_kph=stalk, posted_kph=posted,
    engage_rising=False, now=2.0,
  )

  dropped = engage_dropout()
  stalk_drop = ego + 5 * CV.MPH_TO_KPH
  decide_map_cruise(
    dropped, engaged=True, mode=MODE_FOLLOW, raw_kph=stalk_drop, posted_kph=None,
    engage_rising=False, now=1.0, traveled_kph=ego,
  )
  assert dropped.sticky_set_kph is not None
  dec_drop = decide_map_cruise(
    dropped, engaged=True, mode=MODE_FOLLOW, raw_kph=stalk_drop, posted_kph=posted,
    engage_rising=False, now=30.0, traveled_kph=ego,
  )
  assert _decision_fields(dec_normal, normal) == _decision_fields(dec_drop, dropped)
  assert dec_drop.seed_kph is not None
  assert abs(dec_drop.seed_kph - posted) < 1e-6
  assert not dec_drop.sticky


def test_valid_to_invalid_posted_leaves_max_unchanged():
  """Posted → no limit holds MAX. No drop, and no reseed onto ego speed."""
  posted = 70 * CV.MPH_TO_KPH
  # 5 mph under MAX: the same delta as a stalk down, if raw falls back to ego.
  ego_near = posted - 5 * CV.MPH_TO_KPH
  ego_far = 42.4 * CV.MPH_TO_KPH

  def engage():
    hold = MapCruiseHold()
    dec = decide_map_cruise(
      hold, engaged=True, mode=MODE_FOLLOW, raw_kph=posted, posted_kph=posted,
      engage_rising=True, now=0.0, take_speed_now=True, traveled_kph=ego_far,
    )
    assert dec.seed_kph is not None
    assert abs(dec.seed_kph - posted) < 1e-6
    assert abs(hold.held_max_kph - posted) < 1e-6
    return hold

  for ego in (ego_near, ego_far):
    hold = engage()
    dec = decide_map_cruise(
      hold, engaged=True, mode=MODE_FOLLOW, raw_kph=ego, posted_kph=None,
      engage_rising=False, now=1.0, traveled_kph=ego,
    )
    assert dec.seed_kph is None
    assert not dec.sticky
    assert hold.sticky_set_kph is None
    assert abs(hold.held_max_kph - posted) < 1e-6
    assert abs(hold.policy_kph - posted) < 1e-6
    assert abs(dec.driver_kph - posted) < 1e-6
    assert abs(_follow_hud(dec, None) - posted) < 1e-6
    # Still invalid. Ego must not become MAX on a later frame either.
    dec = decide_map_cruise(
      hold, engaged=True, mode=MODE_FOLLOW, raw_kph=ego, posted_kph=None,
      engage_rising=False, now=5.0, traveled_kph=ego,
    )
    assert dec.seed_kph is None
    assert not dec.sticky
    assert abs(hold.held_max_kph - posted) < 1e-6
    assert abs(_follow_hud(dec, None) - posted) < 1e-6

  # Set speed itself did not move; only the sign went away.
  hold = engage()
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=posted, posted_kph=None,
    engage_rising=False, now=1.0, traveled_kph=ego_far,
  )
  assert dec.seed_kph is None
  assert abs(hold.held_max_kph - posted) < 1e-6
  assert abs(_follow_hud(dec, None) - posted) < 1e-6


# --- Oct 13: untagged state defaults --------------------------------------------------------------------------
from openpilot.selfdrive.mapd.map_speed_policy import (
  DEFAULT_DROP_A_MS2, RURAL_KEEP_MS, default_drop_cap_ms, keep_rural_default_ms, map_source_for,
)

_MPH = 0.44704


def test_source_tagged_vs_default():
  assert map_source_for(False) == "osm"
  assert map_source_for(True) == "osm_default"
  assert map_source_for(True, sign=True) == "sign"


def test_ridgewood_untagged_tertiary_keeps_55():
  prev = ("Ridgewood Road", "tertiary", 55 * _MPH)
  assert keep_rural_default_ms(30 * _MPH, estimated=True, highway="tertiary",
                               road_name="Ridgewood Road", prev=prev) == RURAL_KEEP_MS


def test_tagged_or_other_road_not_kept():
  prev = ("Ridgewood Road", "tertiary", 55 * _MPH)
  assert keep_rural_default_ms(30 * _MPH, estimated=False, highway="tertiary",
                               road_name="Ridgewood Road", prev=prev) == 30 * _MPH
  assert keep_rural_default_ms(30 * _MPH, estimated=True, highway="tertiary",
                               road_name="Main St", prev=prev) == 30 * _MPH
  assert keep_rural_default_ms(30 * _MPH, estimated=True, highway="residential",
                               road_name="Ridgewood Road", prev=prev) == 30 * _MPH


def test_default_drop_no_harder_than_half():
  cap = None
  v = 45 * _MPH
  for _ in range(100):
    new = default_drop_cap_ms(30 * _MPH, cap, v, 0.05)
    if cap is not None:
      assert (cap - new) / 0.05 <= DEFAULT_DROP_A_MS2 + 1e-9
    cap = new
  assert cap > 30 * _MPH
