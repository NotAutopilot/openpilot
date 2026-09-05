import pytest

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


def _chime(prev, *, lat=False, long_on=False):
  return update_preap_chimes(lat_engaged=lat, long_engaged=long_on, prev=prev)


# Complete (prev_lat, prev_long) × (lat, long) table. Gas override is a hold of
# enableLongControl, so it is the (T,T)→(T,T) row, not a disengage edge.
CHIME_EDGES = [
  # prev_lat, prev_long, lat, long, lat_eng, lat_dis, long_eng, long_dis, note
  (False, False, False, False, False, False, False, False, "idle"),
  (False, False, True,  False, True,  False, False, False, "lat-only engage"),
  (False, False, False, True,  False, False, True,  False, "long-only engage"),
  (False, False, True,  True,  True,  False, True,  False, "stalk lat+long engage"),
  (True,  False, False, False, False, True,  False, False, "lat cancel"),
  (True,  False, True,  False, False, False, False, False, "hold lat"),
  (True,  False, False, True,  False, True,  True,  False, "lat cancel + long engage"),
  (True,  False, True,  True,  False, False, True,  False, "second pull adds long"),
  (False, True,  False, False, False, False, False, True,  "long-only cancel"),
  (False, True,  True,  False, True,  False, False, True,  "lat engage + long cancel"),
  (False, True,  False, True,  False, False, False, False, "hold long-only"),
  (False, True,  True,  True,  True,  False, False, False, "add lat to long"),
  (True,  True,  False, False, False, True,  False, True,  "stalk cancel both"),
  (True,  True,  True,  False, False, False, False, True,  "brake drops long"),
  (True,  True,  False, True,  False, True,  False, False, "lat cancel keeps long"),
  (True,  True,  True,  True,  False, False, False, False, "hold both / gas override"),
]


@pytest.mark.parametrize(
  "prev_lat,prev_long,lat,long_on,lat_eng,lat_dis,long_eng,long_dis,note",
  CHIME_EDGES,
  ids=[row[-1] for row in CHIME_EDGES],
)
def test_chime_edge_table(prev_lat, prev_long, lat, long_on,
                          lat_eng, lat_dis, long_eng, long_dis, note):
  prev = PreAPChimeState(lat_engaged=prev_lat, long_engaged=prev_long)
  chimes, state = _chime(prev, lat=lat, long_on=long_on)
  assert chimes.lat_engage is lat_eng, note
  assert chimes.lat_disengage is lat_dis, note
  assert chimes.long_engage is long_eng, note
  assert chimes.long_disengage is long_dis, note
  assert state == PreAPChimeState(lat_engaged=lat, long_engaged=long_on)


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


def test_gas_override_does_not_chime():
  # enableLongControl stays true during pedal override. Press and release
  # are holds, not engage/disengage edges.
  chimes, state = _chime(PreAPChimeState(), lat=True, long_on=True)
  assert chimes.long_engage

  chimes, state = _chime(state, lat=True, long_on=True)
  assert not chimes.long_disengage
  assert not chimes.long_engage
  assert not chimes.lat_disengage

  chimes, _ = _chime(state, lat=True, long_on=True)
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


def test_lateral_only_does_not_chime_long_on_hold():
  chimes, state = _chime(PreAPChimeState(), lat=True)
  assert chimes.lat_engage
  assert not chimes.long_engage

  chimes, _ = _chime(state, lat=True)
  assert not chimes.long_engage
  assert not chimes.long_disengage
  assert not chimes.lat_disengage
