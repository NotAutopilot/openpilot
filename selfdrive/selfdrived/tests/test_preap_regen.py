from openpilot.selfdrive.selfdrived.preap_regen import (
  REGEN_DEMAND_EVIDENCE_COUNT,
  PreAPChimeState,
  RegenDemandCheck,
  update_preap_chimes,
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
    fired = _update(check, a_target=-1.6) or fired
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
  assert _update(check, a_target=-1.6, v_ego=1.5)

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


def _chime(prev, *, lat=False, long_on=False, gas=False):
  return update_preap_chimes(
    lat_engaged=lat, long_engaged=long_on, gas_pressed=gas, prev=prev,
  )


def test_lat_engage_and_disengage_chime_on_cruise_edges():
  chimes, state = _chime(PreAPChimeState(), lat=True)
  assert chimes.lat_engage
  assert not chimes.lat_disengage
  assert not chimes.long_engage

  chimes, state = _chime(state, lat=True)
  assert not chimes.lat_engage
  assert not chimes.lat_disengage

  chimes, _ = _chime(state, lat=False)
  assert chimes.lat_disengage
  assert not chimes.lat_engage


def test_long_engage_chimes_on_fsm_intent_not_pedal_authority():
  # Stalk long engage must chime even before interceptor handshake.
  chimes, state = _chime(PreAPChimeState(), lat=True, long_on=True)
  assert chimes.lat_engage
  assert chimes.long_engage

  chimes, _ = _chime(state, lat=True, long_on=True)
  assert not chimes.long_engage


def test_long_engage_while_gas_pressed_is_not_deferred():
  chimes, state = _chime(PreAPChimeState(gas_pressed=True), lat=True, long_on=True, gas=True)
  assert chimes.long_engage
  assert not chimes.long_disengage

  # Release back to control: already engaged, no second engage chime.
  chimes, _ = _chime(state, lat=True, long_on=True, gas=False)
  assert not chimes.long_engage
  assert not chimes.long_disengage


def test_gas_override_is_long_disengage():
  chimes, state = _chime(PreAPChimeState(), lat=True, long_on=True)
  assert chimes.long_engage

  chimes, state = _chime(state, lat=True, long_on=True, gas=True)
  assert chimes.long_disengage
  assert not chimes.long_engage
  assert not chimes.lat_disengage

  chimes, _ = _chime(state, lat=True, long_on=True, gas=False)
  assert not chimes.long_engage
  assert not chimes.long_disengage


def test_brake_or_cancel_long_chimes_disengage():
  chimes, state = _chime(PreAPChimeState(), lat=True, long_on=True)
  assert chimes.long_engage

  chimes, state = _chime(state, lat=True, long_on=False)
  assert chimes.long_disengage
  assert not chimes.lat_disengage

  chimes, _ = _chime(state, lat=False, long_on=False)
  assert chimes.lat_disengage
  assert not chimes.long_disengage


def test_reengage_after_disengage_chimes_again():
  chimes, state = _chime(PreAPChimeState(), lat=True, long_on=True)
  assert chimes.long_engage
  chimes, state = _chime(state, lat=False, long_on=False)
  assert chimes.lat_disengage
  assert chimes.long_disengage

  chimes, _ = _chime(state, lat=True, long_on=True)
  assert chimes.lat_engage
  assert chimes.long_engage


def test_lateral_only_gas_does_not_chime_long():
  chimes, state = _chime(PreAPChimeState(), lat=True)
  assert chimes.lat_engage
  assert not chimes.long_engage

  chimes, _ = _chime(state, lat=True, gas=True)
  assert not chimes.long_engage
  assert not chimes.long_disengage
  assert not chimes.lat_disengage
