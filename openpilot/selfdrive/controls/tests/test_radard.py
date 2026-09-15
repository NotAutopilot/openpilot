from types import SimpleNamespace

import pytest

from openpilot.cereal import messaging
from openpilot.selfdrive.controls.radard import KalmanParams, RADAR_TO_CAMERA, RadarD, Track


MAX_RADAR_MEASUREMENT_AGE = 0.5
CLIPPED_VISION_STD = 59874.140625


def _cp(*, fingerprint: str = "TESLA_MODEL_S_PREAP", brand: str = "tesla"):
  return SimpleNamespace(brand=brand, carFingerprint=fingerprint, flags=0)


def _cp_sp():
  return SimpleNamespace(flags=0)


class RadarScenario:
  def __init__(self, v_ego: float = 20.0, *, fingerprint: str = "TESLA_MODEL_S_PREAP", brand: str = "tesla"):
    services = ["modelV2", "carState", "liveTracks"]
    self.sm = messaging.SubMaster(services, ignore_alive=services, ignore_avg_freq=services)
    self.radar = RadarD(_cp(fingerprint=fingerprint, brand=brand), _cp_sp())
    self.v_ego = v_ego
    self.frame = 0

  def step(self, time_s: float, vision_d_rel: float, radar_points: list[tuple[int, float, float, float]] | None = None,
           vision_v: float | None = None, *, x_std: float = 3.0, v_std: float = 2.0, y_std: float = 1.0,
           vision_y: float = 0.0, vision_prob: float = 0.9):
    messages = [self._model_message(time_s, vision_d_rel, vision_v, x_std=x_std, v_std=v_std, y_std=y_std,
                                    vision_y=vision_y, vision_prob=vision_prob)]
    if self.frame == 0:
      messages.append(self._car_state_message(time_s))
    if radar_points is not None:
      messages.append(self._radar_message(time_s, radar_points))

    self.sm.update_msgs(time_s, [message.as_reader() for message in messages])
    self.radar.update(self.sm, self.sm["liveTracks"])
    self.frame += 1
    return self.radar.radar_state.leadOne

  def _model_message(self, time_s: float, vision_d_rel: float, vision_v: float | None, *,
                     x_std: float = 3.0, v_std: float = 2.0, y_std: float = 1.0,
                     vision_y: float = 0.0, vision_prob: float = 0.9):
    message = messaging.new_message("modelV2")
    message.logMonoTime = int(time_s * 1e9)
    message.modelV2.velocity.x = [self.v_ego]
    leads = message.modelV2.init("leadsV3", 2)
    for lead in leads:
      lead.prob = vision_prob
      lead.x = [vision_d_rel + RADAR_TO_CAMERA]
      lead.xStd = [x_std]
      lead.y = [vision_y]
      lead.yStd = [y_std]
      lead.v = [self.v_ego if vision_v is None else vision_v]
      lead.vStd = [v_std]
      lead.a = [0.0]
    return message

  def _car_state_message(self, time_s: float):
    message = messaging.new_message("carState")
    message.logMonoTime = int(time_s * 1e9)
    message.carState.vEgo = self.v_ego
    return message

  @staticmethod
  def _radar_message(time_s: float, radar_points: list[tuple[int, float, float, float]]):
    message = messaging.new_message("liveTracks")
    message.logMonoTime = int(time_s * 1e9)
    points = message.liveTracks.init("points", len(radar_points))
    for point, (track_id, d_rel, y_rel, v_rel) in zip(points, radar_points, strict=True):
      point.trackId = track_id
      point.dRel = d_rel
      point.yRel = y_rel
      point.vRel = v_rel
      point.measured = True
    return message


def test_radar_silence_falls_back_to_current_vision_lead():
  scenario = RadarScenario()
  lead = scenario.step(1.0, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)])
  assert lead.radar

  model_dt = 0.05
  expiration_frame = int(MAX_RADAR_MEASUREMENT_AGE / model_dt) + 1
  for frame in range(1, expiration_frame):
    lead = scenario.step(1.0 + frame * model_dt, vision_d_rel=36.0)
  assert lead.radar

  lead = scenario.step(1.0 + expiration_frame * model_dt, vision_d_rel=36.0)

  assert not lead.radar
  assert lead.dRel == pytest.approx(36.0)


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
  # Each residual remains inside its independent 3-sigma gate, while their
  # combined likelihood is below the score floor.
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


def test_fingerprint_gate_skips_preap_association_for_other_cars():
  scenario = RadarScenario(fingerprint="TESLA_MODEL_Y")
  assert scenario.radar.preap_radar is False
  lead = scenario.step(1.0, vision_d_rel=100.0, radar_points=[(7, 120.0, 0.0, 0.0)])
  assert lead.radar
  assert lead.dRel == pytest.approx(120.0)


def test_association_rejects_stationary_speed_mismatch_when_distance_fits_cap():
  vision_d_rel = 97.1520767 - RADAR_TO_CAMERA
  scenario = RadarScenario(v_ego=22.33053)
  lead = scenario.step(
    1.0,
    vision_d_rel=vision_d_rel,
    radar_points=[(438, 75.8125, 1.375, -22.0)],
    vision_v=22.43566,
    x_std=CLIPPED_VISION_STD,
    v_std=CLIPPED_VISION_STD,
    y_std=0.23918359,
    vision_y=-1.4312779,
  )

  assert not lead.radar
  assert lead.dRel == pytest.approx(vision_d_rel)


def test_association_rejects_distant_mismatch_when_velocity_fits():
  v_ego = 17.41916
  vision_v = 12.84962
  vision_d_rel = 89.7781 - RADAR_TO_CAMERA
  scenario = RadarScenario(v_ego=v_ego)
  lead = scenario.step(
    1.0,
    vision_d_rel=vision_d_rel,
    radar_points=[(478, 15.875, 0.0, vision_v - v_ego)],
    vision_v=vision_v,
    x_std=CLIPPED_VISION_STD,
    v_std=CLIPPED_VISION_STD,
  )

  assert not lead.radar
  assert lead.dRel == pytest.approx(vision_d_rel)


def test_association_does_not_keep_ineligible_incumbent_via_hysteresis():
  scenario = RadarScenario()
  clipped = {"x_std": CLIPPED_VISION_STD, "v_std": CLIPPED_VISION_STD}
  lead = scenario.step(1.0, vision_d_rel=40.0, radar_points=[(81, 40.0, 0.0, 0.0)], **clipped)
  assert lead.radarTrackId == 81

  lead = scenario.step(
    1.1,
    vision_d_rel=40.0,
    radar_points=[(81, 15.0, 0.0, 0.0), (82, 40.0, 0.0, 0.0)],
    **clipped,
  )

  assert lead.radar
  assert lead.radarTrackId == 82


@pytest.mark.parametrize("v_ego, vision_v, radar_v_rel", [
  (20.0, 20.0, 0.0),
  (22.33053, 0.0, -22.33053),
])
def test_association_keeps_agreeing_tracks_at_clipped_std(v_ego, vision_v, radar_v_rel):
  scenario = RadarScenario(v_ego=v_ego)
  lead = scenario.step(
    1.0,
    vision_d_rel=40.0,
    radar_points=[(7, 40.0, 0.0, radar_v_rel)],
    vision_v=vision_v,
    x_std=CLIPPED_VISION_STD,
    v_std=CLIPPED_VISION_STD,
  )

  assert lead.radar
  assert lead.radarTrackId == 7
  assert lead.dRel == pytest.approx(40.0)


def test_low_speed_override_selects_in_path_stop_without_vision():
  scenario = RadarScenario(v_ego=3.0)
  lead = scenario.step(1.0, vision_d_rel=30.0, radar_points=[(7, 10.0, 0.0, -3.0)], vision_prob=0.0)

  assert lead.radar
  assert lead.radarTrackId == 7
  assert lead.dRel == pytest.approx(10.0)
