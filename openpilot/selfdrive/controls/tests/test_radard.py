from types import SimpleNamespace

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.cereal import messaging
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP
from openpilot.selfdrive.controls.radard import (
  ASSOCIATION_SWITCH_MARGIN,
  KalmanParams,
  RADAR_MEASUREMENT_TIMEOUT,
  RADAR_TO_CAMERA,
  RadarD,
  Track,
)


def _cp(*, fingerprint: str = "TESLA_MODEL_S_PREAP", brand: str = "tesla", flags: int = 0):
  return SimpleNamespace(brand=brand, carFingerprint=fingerprint, flags=flags)


def _cp_sp(*, preap_radar: bool = True, flags: int | None = None):
  if flags is None:
    flags = int(TeslaFlagsSP.PREAP_RADAR_PRESENT) if preap_radar else 0
  return SimpleNamespace(flags=flags)


class RadarScenario:
  def __init__(self, v_ego: float = 20.0, *, preap_radar: bool = True,
               fingerprint: str = "TESLA_MODEL_S_PREAP", brand: str = "tesla",
               flags: int | None = None):
    services = ["modelV2", "carState", "liveTracks"]
    self.sm = messaging.SubMaster(services, ignore_alive=services, ignore_avg_freq=services)
    self.radar = RadarD(
      _cp(fingerprint=fingerprint, brand=brand),
      _cp_sp(preap_radar=preap_radar, flags=flags),
    )
    self.v_ego = v_ego
    self.frame = 0

  def step(self, time_s: float, vision_d_rel: float, radar_points: list[tuple[int, float, float, float]] | None = None,
           vision_v: float | None = None, live_tracks_valid: bool = True, measured: bool = True):
    messages = [self._model_message(time_s, vision_d_rel, vision_v)]
    if self.frame == 0:
      messages.append(self._car_state_message(time_s))
    if radar_points is not None:
      messages.append(self._radar_message(time_s, radar_points, valid=live_tracks_valid, measured=measured))

    self.sm.update_msgs(time_s, [message.as_reader() for message in messages])
    self.radar.update(self.sm, self.sm["liveTracks"])
    self.frame += 1
    return self.radar.radar_state.leadOne

  def _model_message(self, time_s: float, vision_d_rel: float, vision_v: float | None):
    message = messaging.new_message("modelV2", valid=True)
    message.logMonoTime = int(time_s * 1e9)
    message.modelV2.velocity.x = [self.v_ego]
    leads = message.modelV2.init("leadsV3", 2)
    for lead in leads:
      lead.prob = 0.9
      lead.x = [vision_d_rel + RADAR_TO_CAMERA]
      lead.xStd = [3.0]
      lead.y = [0.0]
      lead.yStd = [1.0]
      lead.v = [self.v_ego if vision_v is None else vision_v]
      lead.vStd = [2.0]
      lead.a = [0.0]
    return message

  def _car_state_message(self, time_s: float):
    message = messaging.new_message("carState", valid=True)
    message.logMonoTime = int(time_s * 1e9)
    message.carState.vEgo = self.v_ego
    return message

  @staticmethod
  def _radar_message(time_s: float, radar_points: list[tuple[int, float, float, float]],
                     valid: bool = True, measured: bool = True):
    message = messaging.new_message("liveTracks", valid=valid)
    message.logMonoTime = int(time_s * 1e9)
    points = message.liveTracks.init("points", len(radar_points))
    for point, (track_id, d_rel, y_rel, v_rel) in zip(points, radar_points, strict=True):
      point.trackId = track_id
      point.dRel = d_rel
      point.yRel = y_rel
      point.vRel = v_rel
      point.deprecated.measured = measured
    return message


def test_radar_silence_falls_back_to_current_vision_lead():
  scenario = RadarScenario()
  lead = scenario.step(1.0, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)])
  assert lead.radar
  assert lead.present

  model_dt = 0.05
  expiration_frame = int(RADAR_MEASUREMENT_TIMEOUT / model_dt) + 1
  for frame in range(1, expiration_frame):
    lead = scenario.step(1.0 + frame * model_dt, vision_d_rel=36.0)
  assert lead.radar

  lead = scenario.step(1.0 + expiration_frame * model_dt, vision_d_rel=36.0)

  assert not lead.radar
  assert lead.present
  assert lead.dRel == pytest.approx(36.0)
  assert lead.radarTrackId == -1


def test_association_rejects_distance_outlier():
  scenario = RadarScenario()
  lead = scenario.step(1.0, vision_d_rel=100.0, radar_points=[(7, 120.0, 0.0, 0.0)])

  assert not lead.radar
  assert lead.dRel == pytest.approx(100.0)


def test_association_rejects_velocity_outlier():
  scenario = RadarScenario()
  lead = scenario.step(1.0, vision_d_rel=50.0, radar_points=[(7, 50.0, 0.0, 15.0)])

  assert not lead.radar
  assert lead.vLead == pytest.approx(20.0)


def test_association_requires_minimum_score():
  scenario = RadarScenario()
  lead = scenario.step(1.0, vision_d_rel=50.0, radar_points=[(7, 58.0, 2.9, 5.5)])

  assert not lead.radar


def test_association_retains_incumbent_until_challenger_wins():
  scenario = RadarScenario()
  two_tracks = [(81, 70.0, 0.0, 0.0), (82, 72.0, 0.0, 0.0)]
  vision_distances = [70.8, 71.2, 70.8, 71.2, 70.8, 71.2, 72.0, 70.8]

  selected_ids = [
    scenario.step(1.0 + frame * 0.1, vision_d_rel=vision_d_rel, radar_points=two_tracks).radarTrackId
    for frame, vision_d_rel in enumerate(vision_distances)
  ]
  selected_ids.append(scenario.step(1.8, vision_d_rel=70.0, radar_points=[two_tracks[0]]).radarTrackId)

  assert selected_ids == [81, 81, 81, 81, 81, 81, 82, 82, 81]
  assert ASSOCIATION_SWITCH_MARGIN == 1.5


def test_raw_id_reuse_clears_incumbent():
  scenario = RadarScenario()
  first = scenario.step(1.0, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)])
  assert first.radarTrackId == 7

  reused = scenario.step(1.1, vision_d_rel=30.0, radar_points=[(8, 30.0, 0.0, 0.0)])
  assert reused.radar
  assert reused.radarTrackId == 8
  assert reused.radarTrackId != first.radarTrackId


def test_unmeasured_track_does_not_update_kalman_state():
  track = Track(identifier=7, v_lead=20.0, kalman_params=KalmanParams(0.1))
  track.update(d_rel=30.0, y_rel=0.0, v_rel=0.0, v_lead=20.0, measured=True)
  track.update(d_rel=30.0, y_rel=0.0, v_rel=1.0, v_lead=21.0, measured=True)
  measured_state = (track.vLeadK, track.aLeadK)

  track.update(d_rel=30.0, y_rel=0.0, v_rel=10.0, v_lead=30.0, measured=False)

  assert (track.vLeadK, track.aLeadK) == pytest.approx(measured_state)


def test_kalman_uses_observed_radar_interval():
  scenario = RadarScenario()
  scenario.step(1.0, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)])

  scenario.step(1.08, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)])

  assert scenario.radar.tracks[7].K_A[0][1] == pytest.approx(0.08)


def test_publication_uses_present_not_status():
  scenario = RadarScenario()
  lead = scenario.step(1.0, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)])
  assert lead.present is True
  assert lead.radar is True
  assert lead.radarTrackId == 7


def test_invalid_live_tracks_clears_tracks_and_invalidates():
  scenario = RadarScenario()
  lead = scenario.step(1.0, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)])
  assert lead.radar
  assert scenario.radar.radar_state_valid
  accepted = scenario.radar.last_radar_update_time
  assert accepted is not None
  assert scenario.radar.lead_one_association.incumbent_track_id == 7

  lead = scenario.step(1.1, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)], live_tracks_valid=False)
  assert scenario.radar.tracks == {}
  assert scenario.radar.lead_one_association.incumbent_track_id is None
  assert scenario.radar.lead_two_association.incumbent_track_id is None
  assert scenario.radar.last_radar_update_time == accepted
  assert not scenario.radar.last_live_tracks_healthy
  assert not scenario.radar.radar_state_valid
  assert not lead.radar


def test_invalid_live_tracks_recovers_only_on_valid_fresh_input():
  scenario = RadarScenario()
  scenario.step(1.0, vision_d_rel=30.0, radar_points=[(8, 30.0, 0.0, 0.0)], live_tracks_valid=False)
  assert scenario.radar.tracks == {}
  assert not scenario.radar.radar_state_valid
  assert scenario.radar.last_radar_update_time is None

  lead = scenario.step(1.2, vision_d_rel=30.0, radar_points=[(8, 30.0, 0.0, 0.0)], live_tracks_valid=True)
  assert lead.radar
  assert lead.radarTrackId == 8
  assert scenario.radar.radar_state_valid
  assert scenario.radar.last_live_tracks_healthy
  assert scenario.radar.last_radar_update_time == pytest.approx(1.2)


def test_modern_tesla_does_not_enable_preap_hardening():
  scenario = RadarScenario(preap_radar=False, fingerprint="TESLA_MODEL_Y")
  assert scenario.radar.preap_radar is False


def test_modern_tesla_keeps_tracks_without_preap_timeout():
  scenario = RadarScenario(preap_radar=False, fingerprint="TESLA_MODEL_Y")
  lead = scenario.step(1.0, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)])
  assert lead.radar
  assert lead.radarTrackId == 7

  model_dt = 0.05
  expiration_frame = int(RADAR_MEASUREMENT_TIMEOUT / model_dt) + 1
  for frame in range(1, expiration_frame + 1):
    lead = scenario.step(1.0 + frame * model_dt, vision_d_rel=36.0)

  assert lead.radar
  assert lead.radarTrackId == 7
  assert lead.dRel == pytest.approx(30.0)


def test_modern_tesla_uses_target_association_not_preap_gates():
  scenario = RadarScenario(preap_radar=False, fingerprint="TESLA_MODEL_Y")
  lead = scenario.step(1.0, vision_d_rel=100.0, radar_points=[(7, 120.0, 0.0, 0.0)])

  assert lead.radar
  assert lead.radarTrackId == 7
  assert lead.dRel == pytest.approx(120.0)


def test_modern_tesla_kalman_uses_dt_mdl():
  scenario = RadarScenario(preap_radar=False, fingerprint="TESLA_MODEL_Y")
  scenario.step(1.0, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)])
  scenario.step(1.08, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)])

  assert scenario.radar.tracks[7].K_A[0][1] == pytest.approx(DT_MDL)


def test_modern_tesla_invalid_live_tracks_does_not_preap_clear():
  scenario = RadarScenario(preap_radar=False, fingerprint="TESLA_MODEL_Y")
  lead = scenario.step(1.0, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)])
  assert lead.radar

  lead = scenario.step(1.1, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)], live_tracks_valid=False)
  assert 7 in scenario.radar.tracks
  assert lead.radar
  assert lead.radarTrackId == 7
  assert not scenario.radar.radar_state_valid


def test_preap_unmeasured_live_track_does_not_update_kalman():
  scenario = RadarScenario()
  scenario.step(1.0, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)])
  scenario.step(1.1, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 1.0)])
  measured_state = (scenario.radar.tracks[7].vLeadK, scenario.radar.tracks[7].aLeadK)

  scenario.step(1.2, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 10.0)], measured=False)
  track = scenario.radar.tracks[7]

  assert track.measured is False
  assert track.vRel == pytest.approx(10.0)
  assert (track.vLeadK, track.aLeadK) == pytest.approx(measured_state)


def test_modern_tesla_updates_kalman_when_unmeasured():
  scenario = RadarScenario(preap_radar=False, fingerprint="TESLA_MODEL_Y")
  scenario.step(1.0, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)])
  scenario.step(1.05, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 1.0)])
  prior = (scenario.radar.tracks[7].vLeadK, scenario.radar.tracks[7].aLeadK)

  scenario.step(1.10, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 10.0)], measured=False)
  track = scenario.radar.tracks[7]

  assert (track.vLeadK, track.aLeadK) != pytest.approx(prior)


def test_hyundai_non_scc_overlapping_flag_does_not_enable_preap_hardening():
  from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

  assert int(HyundaiFlagsSP.NON_SCC) == int(TeslaFlagsSP.PREAP_RADAR_PRESENT)
  scenario = RadarScenario(
    fingerprint="HYUNDAI_KONA_NON_SCC",
    brand="hyundai",
    flags=int(HyundaiFlagsSP.NON_SCC),
  )
  assert scenario.radar.preap_radar is False
  assert scenario.radar.kalman_params.A[0][1] == pytest.approx(DT_MDL)

  lead = scenario.step(1.0, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)])
  assert lead.radar
  model_dt = 0.05
  expiration_frame = int(RADAR_MEASUREMENT_TIMEOUT / model_dt) + 1
  for frame in range(1, expiration_frame + 1):
    lead = scenario.step(1.0 + frame * model_dt, vision_d_rel=36.0)
  assert lead.radar
  assert lead.radarTrackId == 7
  assert lead.dRel == pytest.approx(30.0)


def test_modern_tesla_overlapping_flag_does_not_enable_preap_hardening():
  scenario = RadarScenario(
    fingerprint="TESLA_MODEL_Y",
    flags=int(TeslaFlagsSP.PREAP_RADAR_PRESENT),
  )
  assert scenario.radar.preap_radar is False

  lead = scenario.step(1.0, vision_d_rel=100.0, radar_points=[(7, 120.0, 0.0, 0.0)])
  assert lead.radar
  assert lead.dRel == pytest.approx(120.0)


def test_preap_platform_requires_flag_for_hardening():
  without_flag = RadarScenario(preap_radar=False, fingerprint="TESLA_MODEL_S_PREAP")
  assert without_flag.radar.preap_radar is False
  with_flag = RadarScenario(preap_radar=True, fingerprint="TESLA_MODEL_S_PREAP")
  assert with_flag.radar.preap_radar is True

