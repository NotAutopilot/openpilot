from openpilot.selfdrive.selfdrived.preap_regen import (
  PEDAL_CHIME_DISABLED,
  PEDAL_CHIME_ENABLED,
  REGEN_DEMAND_EVIDENCE_COUNT,
  RegenDemandCheck,
  pedal_long_chime,
)

# get_preap_accel_limits floor is -1.5 m/s²; -2.0 clears the trigger margin.
OVERFLOW_TARGET = -2.0


def _update(check, *, a_target=OVERFLOW_TARGET, v_ego=15.0,
            pedal_long_active=True, brake_pressed=False):
  return check.update(
    pedal_long_active=pedal_long_active,
    brake_pressed=brake_pressed,
    a_target=a_target,
    v_ego=v_ego,
  )


def test_demand_prompt_requires_sustained_overflow():
  check = RegenDemandCheck()
  for _ in range(REGEN_DEMAND_EVIDENCE_COUNT - 1):
    assert not _update(check)
  assert _update(check)


def test_demand_prompt_silent_when_plan_fits_envelope():
  check = RegenDemandCheck()
  for _ in range(3 * REGEN_DEMAND_EVIDENCE_COUNT):
    assert not _update(check, a_target=-1.5)


def test_demand_prompt_survives_single_sample_dropouts():
  check = RegenDemandCheck()
  fired = False
  for _ in range(3 * REGEN_DEMAND_EVIDENCE_COUNT):
    for _ in range(9):
      fired = _update(check) or fired
    # -1.55 is inside the 0.10 trigger margin, outside the 0.03 clear margin.
    fired = _update(check, a_target=-1.55) or fired
    if fired:
      break
  assert fired


def test_demand_prompt_does_not_fire_at_standstill():
  check = RegenDemandCheck()
  for _ in range(2 * REGEN_DEMAND_EVIDENCE_COUNT):
    assert not _update(check, v_ego=0.0)


def test_demand_prompt_clears_when_driver_brakes():
  check = RegenDemandCheck()
  for _ in range(REGEN_DEMAND_EVIDENCE_COUNT):
    _update(check)
  assert check.active

  assert not _update(check, brake_pressed=True)
  assert not check.active


def test_demand_prompt_uses_hysteresis_before_clearing():
  check = RegenDemandCheck()
  for _ in range(REGEN_DEMAND_EVIDENCE_COUNT):
    _update(check)
  assert check.active

  # Back inside the trigger margin but still beyond the clear margin.
  assert _update(check, a_target=-1.55, v_ego=1.5)

  # Demand returns to the envelope: prompt clears.
  assert not _update(check, a_target=-1.5)
  assert not check.active


def test_demand_prompt_resets_when_pedal_long_inactive():
  check = RegenDemandCheck()
  for _ in range(REGEN_DEMAND_EVIDENCE_COUNT):
    _update(check)
  assert check.active

  assert not _update(check, pedal_long_active=False)
  assert not check.active


def _chimes(enable_long_frames):
  prev = False
  events = []
  for enable_long in enable_long_frames:
    chime = pedal_long_chime(
      enable_long_control=enable_long,
      prev_enable_long_control=prev,
    )
    if chime is not None:
      events.append(chime)
    prev = enable_long
  return events


def test_brake_or_single_pull_to_lat_only_fires_disengage():
  # enableLongControl 1→0 while cruise stays (brake or first-pull).
  # Interceptor authority also drops, but the chime is the FSM edge.
  assert _chimes([True, False]) == [PEDAL_CHIME_ENABLED, PEDAL_CHIME_DISABLED]


def test_double_pull_long_on_fires_engage():
  # Lat-only (cruise on, long off) then double-pull: enableLongControl 0→1.
  assert _chimes([False, True]) == [PEDAL_CHIME_ENABLED]


def test_pedal_override_does_not_use_pedal_up_as_engage():
  # Gas override drops pedalLongActive while enableLongControl stays true.
  # Pedal-up re-accepts authority on the same still-true flag — not a 0→1.
  assert pedal_long_chime(enable_long_control=True, prev_enable_long_control=True) is None
  assert _chimes([True, True, True]) == [PEDAL_CHIME_ENABLED]


def test_full_off_then_on_still_chimes():
  assert _chimes([True, False, True]) == [
    PEDAL_CHIME_ENABLED, PEDAL_CHIME_DISABLED, PEDAL_CHIME_ENABLED,
  ]


def test_authority_drop_is_not_a_disengage_when_long_stays_on():
  # The 55704413 failure mode: chiming pedalLongActive 1→0. Long stays on.
  assert pedal_long_chime(enable_long_control=True, prev_enable_long_control=True) is None
