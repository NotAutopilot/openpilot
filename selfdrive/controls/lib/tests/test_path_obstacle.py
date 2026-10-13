"""Radar obstacle trigger, projection, fusion, and the animal/person chime."""
from __future__ import annotations

import math
import time
import wave
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.controls.lib.path_obstacle import (
  _Hit,
  chime_policy_block,
  LIVING_CLASSES,
  PARAM_OBSTACLE_CHIME,
  PARAM_OBSTACLE_LOG,
  SPEED_DISABLE_MPS,
  SPEED_ENABLE_MPS,
  BrakeIntent,
  ChimeGate,
  ConeHint,
  ObstacleSample,
  ObstacleStage,
  PathObstacleDetector,
  VisionScore,
  chime_banner,
  crop_light_stats,
  fuse_scores,
  fusion_weights,
  future_brake_intent,
  is_night_lighting,
  light_dominated,
  _fuse_class,
  lively_from_history,
  patch_change_score,
  project_road_point,
  scale_living_vision,
  score_row_patches,
  prune_thumbs,
  save_ppm,
  should_raise_chime,
  vehicle_exclusion_points,
)
from openpilot.selfdrive.ui.obstacle_chime import (
  OBSTACLE_CHIME_ID,
  OBSTACLE_CHIME_MIN_VOLUME,
  alert_sound_id,
  floor_volume,
)

ROOT = Path(__file__).resolve().parents[4]
V_EGO = 12.0
PATH_X = [0.0, 100.0]
PATH_Y = [0.0, 0.0]
ANIMAL = VisionScore(conf=0.70, human=0.10, animal=0.82, obstacle=0.18)
HUMAN = VisionScore(conf=0.72, human=0.64, animal=0.10, obstacle=0.14)
DEBRIS = VisionScore(conf=0.80, human=0.08, animal=0.10, obstacle=0.70)
WEAK = VisionScore(conf=0.20, human=0.90, animal=0.05, obstacle=0.05)


def _pt(x, y_rel, v_rel, tid, yv_rel=0.0, rcs=5.0):
  return SimpleNamespace(
    dRel=x - 1.52, yRel=y_rel, vRel=v_rel, measured=True,
    trackId=tid, yvRel=yv_rel, rcs=rcs,
  )


def _run(det, points, n=5, dt=0.1, vision=None, chime_enabled=True, now0=100.0, **kw):
  sample = None
  for i in range(n):
    sample = det.update(
      points, V_EGO, PATH_X, PATH_Y, dt,
      vision=vision, chime_enabled=chime_enabled, now=now0 + i * dt, **kw,
    )
  return sample


def _agreed(cls, zone, tid):
  return ObstacleSample(
    active=True, track_id=tid, object_class=cls, zone=zone, agree=True,
    member_ids=(tid,), reject_reason="none",
  )


def test_projection_matches_identity_and_camera():
  ground = project_road_point(20, 0, 0)
  assert ground is not None
  assert abs(ground[0] - 672.0) < 1e-6
  assert abs(ground[1] - (380.0 + 1141.5 * 1.22 / 20.0)) < 1e-4
  top = project_road_point(20, 0, 1.0)
  assert top is not None
  assert abs(top[1] - (380.0 + 1141.5 * 0.22 / 20.0)) < 1e-4
  left = project_road_point(20, 2.0, 0)
  assert left is not None
  assert abs(left[0] - (672.0 - 1141.5 * 2.0 / 20.0)) < 1e-4

  from openpilot.common.transformations.camera import get_view_frame_from_road_frame
  view = get_view_frame_from_road_frame(0.0, 0.0, 0.0, 1.22).dot(np.array([20.0, 0.0, 0.0, 1.0]))
  u = 672.0 + 1141.5 * (view[0] / view[2])
  v = 380.0 + 1141.5 * (view[1] / view[2])
  assert abs(u - ground[0]) < 1e-4
  assert abs(v - ground[1]) < 1e-4


def test_fusion_weights_are_smooth_and_nan_vision_does_not_agree():
  good = fusion_weights(1.0)
  mid = fusion_weights(0.5)
  poor = fusion_weights(0.0)
  assert abs(good[0] - 0.50) < 1e-9 and abs(good[1] - 0.50) < 1e-9
  assert abs(mid[0] - 0.64) < 1e-9 and abs(mid[1] - 0.36) < 1e-9
  assert abs(poor[0] - 0.78) < 1e-9 and abs(poor[1] - 0.22) < 1e-9
  assert poor[0] > mid[0] > good[0]
  fused, _wr, _wv, agree = fuse_scores(0.80, None, 1.0)
  assert agree is False and abs(fused - 0.80) < 1e-9
  _f, _wr, _wv, agree = fuse_scores(0.80, float("nan"), 0.0)
  assert agree is False
  _f, _wr, _wv, agree = fuse_scores(0.60, 0.50, 1.0)
  assert agree is True
  assert fuse_scores(0.50, 0.50, 1.0)[3] is False
  assert fuse_scores(0.60, 0.40, 1.0)[3] is False


def test_stationary_in_path_deer_chimes_only_when_vision_says_animal():
  deer = [_pt(30.0, 0.0, -V_EGO, 3)]
  bare = _run(PathObstacleDetector(), deer, vision=None)
  assert bare.vision_evaluated is False
  assert bare.chimed is False
  assert bare.object_class == "unknown"

  seen = _run(PathObstacleDetector(), deer, vision=ANIMAL)
  assert seen.active and seen.zone == "inPath"
  assert seen.object_class == "animal"
  assert seen.lively_score < 0.2
  assert 0.25 < seen.vision_conf_animal < 0.55
  assert seen.agree and seen.vision_evaluated
  assert seen.chimed and seen.chime_reason == "chimed"
  assert seen.brake_gate is True
  assert future_brake_intent(seen) is not None

  debris = _run(PathObstacleDetector(), deer, vision=DEBRIS)
  assert debris.object_class == "obstacle"
  assert not debris.chimed and debris.chime_reason == "log_only_class"
  assert debris.brake_gate is True


def test_human_entering_chimes_and_can_brake_later():
  walker = [_pt(25.0, -2.3, -V_EGO, 8, yv_rel=1.0)]
  person = _run(PathObstacleDetector(), walker, vision=HUMAN)
  assert person.object_class == "human"
  assert person.zone == "entering"
  assert person.agree and person.chimed
  assert person.brake_gate is True
  intent = future_brake_intent(person)
  assert isinstance(intent, BrakeIntent) and intent.allowed and intent.decel == 0.0
  assert chime_banner(person.object_class, person.zone) == "Object ahead"

  weak = _run(PathObstacleDetector(), walker, vision=WEAK)
  assert weak.object_class == "human"
  assert weak.chimed is False and weak.chime_reason == "no_agree"

  # Below the traffic cutoff a narrow in-path rider can still chime.
  slow_bike = [_pt(22.0, 0.0, 4.0 - V_EGO, 9, yv_rel=0.1)]
  rider = _run(PathObstacleDetector(), slow_bike, vision=HUMAN)
  assert rider.object_class == "human" and rider.zone == "inPath" and rider.chimed


def test_excludes_traffic_motorcycles_and_parked_cars():
  # Motorcycle-like: one narrow return, with the flow, almost no lateral speed.
  bike = [_pt(22.0, 0.15, 7.5 - V_EGO, 9, yv_rel=0.1)]
  moto = _run(PathObstacleDetector(), bike, vision=HUMAN)
  assert moto.reject_reason == "exclVehicle" and moto.chimed is False

  # A dart across the lane is not traffic, even above 5 m/s along the road.
  dart = [_pt(22.0, 0.2, 6.5 - V_EGO, 19, yv_rel=-2.6)]
  crossing = _run(PathObstacleDetector(), dart, vision=ANIMAL)
  assert crossing.reject_reason != "exclVehicle"
  assert crossing.zone == "inPath" and crossing.chimed

  # Adjacent-lane car: ~3.7 m to the right, ground speed ~11 m/s, small v_lat.
  traffic = [_pt(35.0, 3.7, 11.0 - V_EGO, 44, yv_rel=-0.15)]
  lane = _run(PathObstacleDetector(), traffic, vision=DEBRIS)
  assert lane.reject_reason == "exclVehicle" and lane.chimed is False

  # coneLineNAP parked-car cluster on the shoulder.
  parked = ConeHint(parked=True)
  shoulder = _run(PathObstacleDetector(), [_pt(18.0, -3.2, -V_EGO, 7)], cone=parked, vision=DEBRIS)
  assert shoulder.reject_reason == "exclVehicle" and shoulder.chimed is False
  cluster = [
    _pt(22.0, -2.4, -V_EGO, 71),
    _pt(22.0, -3.2, -V_EGO, 72),
    _pt(23.0, -4.0, -V_EGO, 73),
  ]
  cars = _run(PathObstacleDetector(), cluster, vision=DEBRIS)
  assert cars.reject_reason == "exclVehicle" and cars.chimed is False

  # Stopped in the path, and the model has not tagged it: still an object.
  stopped = [_pt(30.0, 0.0, -V_EGO, 4)]
  bare = _run(PathObstacleDetector(), stopped, vision=DEBRIS)
  assert bare.zone == "inPath" and bare.reject_reason == "none"
  assert not bare.chimed and bare.chime_reason == "log_only_class"
  # lead0 is a far car. lead1 is this one. Both count, not only the current lead.
  tagged = _run(
    PathObstacleDetector(), stopped, vision=DEBRIS,
    model_leads=[(90.0, 0.0, 0.95), (30.0, 0.0, 0.80)],
  )
  assert tagged.reject_reason == "exclVehicle" and tagged.chimed is False
  # The lead does not take the slot from a different return.
  both = [_pt(30.0, 0.0, -V_EGO, 4), _pt(55.0, 0.2, -V_EGO, 5)]
  mixed = _run(PathObstacleDetector(), both, vision=DEBRIS, model_leads=[(30.0, 0.0, 0.80)])
  assert mixed.track_id == 5 and mixed.reject_reason == "none" and mixed.chime_reason == "log_only_class"
  far_only = _run(
    PathObstacleDetector(), stopped, vision=DEBRIS,
    model_leads=[(90.0, 0.0, 0.95)],
  )
  assert far_only.reject_reason == "none" and far_only.chime_reason == "log_only_class"
  # The second radard lead, not only leadOne.
  other = _run(PathObstacleDetector(), stopped, vision=DEBRIS, lead_ids=(11, 4))
  assert other.reject_reason == "exclVehicle" and other.chimed is False
  assert _run(PathObstacleDetector(), stopped, vision=DEBRIS, lead_ids=(11,)).chime_reason == "log_only_class"
  # A weak model probability does not count as a tracked vehicle.
  weak_lead = _run(
    PathObstacleDetector(), stopped, vision=DEBRIS,
    model_leads=[(30.0, 0.0, 0.20)],
  )
  assert weak_lead.reject_reason == "none" and weak_lead.chime_reason == "log_only_class"


def test_vehicle_points_cover_both_model_leads_and_both_radar_leads():
  leads_v3 = [
    SimpleNamespace(x=[90.0], y=[0.0], prob=0.95),
    SimpleNamespace(x=[22.0], y=[0.4], prob=0.70),
    SimpleNamespace(x=[40.0], y=[1.0], prob=0.10),
  ]
  leads_v2 = [SimpleNamespace(prob=0.80, xyva=[50.0, -1.0, 12.0, 0.0])]
  radar = [
    SimpleNamespace(status=True, dRel=22.0 - 1.52, yRel=-0.4, radarTrackId=9, modelProb=0.7),
    SimpleNamespace(status=False, dRel=10.0, yRel=0.0, radarTrackId=3, modelProb=0.9),
    SimpleNamespace(status=True, dRel=60.0 - 1.52, yRel=1.0, radarTrackId=-1, modelProb=0.0),
  ]
  ids, points = vehicle_exclusion_points(leads_v3, leads_v2, radar)
  assert ids == [9]
  assert (90.0, 0.0, 0.95) in points
  assert (22.0, 0.4, 0.70) in points
  assert (50.0, -1.0, 0.80) in points
  assert (22.0, 0.4, 0.7, 9) in points
  assert (60.0, -1.0, 1.0) in points
  assert all(p[2] >= 0.40 for p in points)
  assert not any(len(p) > 3 and p[3] == 3 for p in points)

  # Position from the helper excludes a second return on lead1, y in device frame.
  _ids, pts = vehicle_exclusion_points(leads_v3, (), ())
  stopped = [_pt(22.0, -0.4, -V_EGO, 80)]
  hit = _run(PathObstacleDetector(), stopped, vision=DEBRIS, model_leads=pts)
  assert hit.reject_reason == "exclVehicle"
  other_side = [_pt(22.0, 1.5, -V_EGO, 81)]
  kept = _run(PathObstacleDetector(), other_side, vision=DEBRIS, model_leads=pts)
  assert kept.reject_reason != "exclVehicle" and kept.chime_reason == "log_only_class"


def test_rejects_clutter_and_a_lane_spanning_tree():
  det = PathObstacleDetector()
  assert _run(det, [_pt(30.0, 0.0, -V_EGO, 4)], lead_ids=(4,)).reject_reason == "exclVehicle"
  # yRel is +left, PathObstacle lat is +right. yRel -2 is 2 m to the right.
  cones = ConeHint(active=True, side=-1, lat_near=2.0, lat_mid=2.0, lat_far=2.0)
  assert _run(PathObstacleDetector(), [_pt(20.0, -2.0, -V_EGO, 5)], cone=cones).reject_reason == "cone_line"
  # Oct 7 posts: left side, side +1, lat negative. The opposite side stays.
  left = ConeHint(active=True, side=1, lat_near=-2.0, lat_mid=-2.0, lat_far=-2.0)
  assert _run(PathObstacleDetector(), [_pt(20.0, 2.0, -V_EGO, 15)], cone=left).reject_reason == "cone_line"
  other = _run(PathObstacleDetector(), [_pt(20.0, -2.0, -V_EGO, 16)], cone=left)
  assert other.reject_reason != "cone_line"
  # Left rail. lat is +right, so the rail sits at negative y. Side is +1.
  rail = ConeHint(barrier=True, side=1, lat_near=-3.0, lat_mid=-3.0, lat_far=-3.0)
  assert _run(PathObstacleDetector(), [_pt(20.0, 3.0, -V_EGO, 6)], cone=rail).reject_reason == "barrier"
  assert _run(PathObstacleDetector(), [_pt(40.0, 0.0, 18.0 - V_EGO, 11)]).reject_reason == "exclVehicle"
  assert _run(PathObstacleDetector(), [_pt(10.0, 0.0, -V_EGO, 12)]).reject_reason == "road_surface"
  assert _run(PathObstacleDetector(), [_pt(40.0, -15.0, -V_EGO, 13)]).reject_reason == "off_path"
  assert _run(PathObstacleDetector(), [_pt(8.0, -12.0, -V_EGO, 14)]).reject_reason == "fov"

  tree = [
    _pt(28.0, 1.2, -V_EGO, 21),
    _pt(28.0, 0.0, -V_EGO, 22),
    _pt(28.0, -1.2, -V_EGO, 23),
  ]
  # Span forces obstacle even when the camera's animal score is higher.
  # The obstacle score still has to clear the class bar or the chime waits.
  span_view = VisionScore(conf=0.80, human=0.08, animal=0.82, obstacle=0.55)
  fallen = _run(PathObstacleDetector(), tree, vision=span_view)
  assert fallen.object_class == "obstacle"
  assert fallen.cluster_count == 3 and fallen.span_m >= 1.8
  assert fallen.agree and not fallen.chimed and fallen.chime_reason == "log_only_class"
  assert fallen.brake_gate is True
  assert future_brake_intent(fallen) is not None

  curtain = [
    _pt(40.0, 5.0, -V_EGO, 31),
    _pt(40.0, 2.0, -V_EGO, 32),
    _pt(40.0, -1.0, -V_EGO, 33),
    _pt(40.0, -4.0, -V_EGO, 34),
  ]
  assert _run(PathObstacleDetector(), curtain).reject_reason == "overhead"


def test_still_mailbox_stays_quiet_and_a_moving_person_chimes():
  mailbox = VisionScore(conf=0.70, human=0.64, animal=0.10, obstacle=0.16)
  shoulder = [_pt(24.0, -4.5, -V_EGO, 41)]
  still = _run(PathObstacleDetector(), shoulder, n=16, vision=mailbox)
  assert still.zone == "roadside"
  assert still.lively_score < 0.15
  assert still.object_class != "human"
  assert still.chimed is False
  assert still.vision_conf_human > 0.15

  # 4.5 m is about 2.8 m outside the corridor, past the 0.9 m human band.
  walking = [_pt(24.0, -4.5, 1.6 - V_EGO, 42)]
  person = _run(PathObstacleDetector(), walking, n=8, vision=mailbox)
  assert person.chimed is False and person.chime_reason == "off_zone"

  near = [_pt(24.0, -2.2, 1.6 - V_EGO, 45)]
  close = _run(PathObstacleDetector(), near, n=8, vision=mailbox)
  assert close.zone == "roadside" and close.object_class == "human"
  assert close.lively_score > 0.8
  assert close.agree and close.chimed and close.chime_reason == "chimed"
  assert close.brake_gate is False
  assert future_brake_intent(close) is None

  deer_side = [_pt(24.0, -4.5, -V_EGO, 46)]
  animal = _run(PathObstacleDetector(), deer_side, n=8, vision=ANIMAL)
  assert animal.zone == "roadside" and animal.object_class == "animal"
  assert animal.agree and animal.chimed and animal.brake_gate is False
  assert chime_banner("human", "roadside") == "Object near road"
  assert chime_banner(1, 3) == "Object near road"
  assert chime_banner("obstacle", "inPath") == "Object ahead"
  assert chime_banner("unknown", "entering") == "Object ahead"

  quiet = _run(PathObstacleDetector(), shoulder, n=8, vision=WEAK)
  assert quiet.chimed is False
  assert quiet.chime_reason == "inanimate"


def test_wander_and_patch_change_raise_lively_without_zeroing_a_freeze():
  frozen = scale_living_vision(ANIMAL, 0.0)
  assert frozen.animal > 0.30
  assert frozen.animal < ANIMAL.animal
  moving = scale_living_vision(ANIMAL, 1.0)
  assert abs(moving.animal - ANIMAL.animal) < 1e-9

  assert lively_from_history([(0.0, 20.0, 0.0, 0.0, 0.0), (1.5, 20.1, 0.05, 0.0, 0.0)]) < 0.05
  # One fast frame is not sustained motion. 0.5 s of it is.
  assert lively_from_history([(0.0, 20.0, 0.0, 1.6, 0.0)]) == 0.0
  assert lively_from_history([(i * 0.1, 20.0, 0.0, 1.6, 0.0) for i in range(6)]) > 0.8
  assert lively_from_history([(i * 0.1, 20.0, 0.0, 0.0, 1.2) for i in range(6)]) >= 0.75
  # Position wander alone (ego approach, radar jitter) no longer counts.
  wandered = [(i * 0.1, 24.0, 4.0 + 0.12 * i, 0.0, 0.0) for i in range(16)]
  assert lively_from_history(wandered) == 0.0
  approach = [(i * 0.1, 47.0 - 2.7 * i, 4.3, 0.1, 0.05) for i in range(20)]
  assert lively_from_history(approach) == 0.0

  det = PathObstacleDetector()
  sample = None
  chimed = False
  for i in range(16):
    # Stay inside the 0.9 m human band while the return is clearly moving.
    y_rel = -2.05 - 0.02 * i
    sample = det.update(
      [_pt(24.0, y_rel, 1.6 - V_EGO, 43)], V_EGO, PATH_X, PATH_Y, 0.1,
      vision=HUMAN, now=200.0 + i * 0.1,
    )
    chimed = chimed or bool(sample.chimed)
  assert sample is not None and chimed
  assert sample.lively_score > 0.4 and sample.object_class == "human"

  still_patch = [[40, 40, 40], [40, 42, 40], [40, 40, 40]]
  moved_patch = [[40, 180, 40], [180, 40, 180], [40, 180, 40]]
  assert patch_change_score(still_patch, still_patch) == 0.0
  assert patch_change_score(still_patch, moved_patch) > 0.5
  boosted = _run(PathObstacleDetector(), [_pt(24.0, -2.2, -V_EGO, 44)], n=8, vision=HUMAN, vision_motion=1.0)
  assert boosted.lively_score > 0.8 and boosted.object_class == "human" and boosted.chimed


def test_chime_hold_cooldown_and_shared_classes():
  det = PathObstacleDetector()
  deer = [_pt(30.0, 0.0, -V_EGO, 3)]
  first = _run(det, deer, n=5, vision=ANIMAL, now0=100.0)
  assert first.chimed
  held = _run(det, deer, n=1, vision=ANIMAL, now0=100.4)
  assert held.chimed and held.chime_reason == "chimed"
  later = _run(det, deer, n=1, vision=ANIMAL, now0=100.8)
  assert later.chimed is False and later.chime_reason == "already_tracked"

  gate = ChimeGate()
  assert gate.consider(_agreed("animal", "inPath", 1), True, 10.0) == (True, "chimed")
  assert gate.consider(_agreed("animal", "inPath", 1), True, 10.2) == (True, "chimed")
  assert gate.consider(_agreed("animal", "inPath", 1), True, 10.5) == (False, "already_tracked")
  assert gate.consider(_agreed("human", "entering", 2), True, 12.0) == (False, "cooldown")
  assert gate.consider(_agreed("human", "roadside", 2), True, 19.0) == (True, "chimed")
  assert gate.consider(_agreed("animal", "inPath", 1), True, 40.0) == (False, "already_tracked")

  shared = ChimeGate()
  assert shared.consider(_agreed("human", "inPath", 5), True, 0.0)[0] is True
  assert shared.consider(_agreed("animal", "roadside", 6), True, 1.0) == (False, "cooldown")

  muted = ChimeGate()
  assert muted.consider(_agreed("human", "inPath", 7), False, 0.0) == (False, "disabled")
  assert muted.consider(_agreed("human", "inPath", 7), True, 0.0) == (True, "chimed")

  off = _run(PathObstacleDetector(), deer, vision=ANIMAL, chime_enabled=False)
  assert off.chime_reason == "disabled" and off.chimed is False
  assert should_raise_chime(True, "human") and should_raise_chime(True, "obstacle")
  assert should_raise_chime(True, "unknown")
  assert not should_raise_chime(False, "human")
  assert LIVING_CLASSES == frozenset({"animal", "human"})
  gate_still = ChimeGate()
  assert gate_still.consider(_agreed("obstacle", "roadside", 8), True, 0.0) == (False, "inanimate")
  assert gate_still.consider(_agreed("unknown", "inPath", 8), True, 0.0) == (True, "chimed")
  lively_post = ObstacleSample(
    active=True, track_id=9, object_class="obstacle", zone="roadside", agree=True,
    member_ids=(9,), reject_reason="none", lively_score=0.8, lateral=4.5,
  )
  assert ChimeGate().consider(lively_post, True, 0.0) == (False, "inanimate")


def test_stationary_human_guess_outside_the_band_is_dropped():
  # toward ~0.5 m/s makes the radar class "human", but the return has not moved.
  post = [_pt(24.0, -4.0, -V_EGO, 51, yv_rel=0.5)]
  got = _run(PathObstacleDetector(), post, n=8, vision=HUMAN)
  assert got.chimed is False
  assert "off_zone" in (got.reject_reason, got.chime_reason) or got.chime_reason == "human_off_3ft"

  far_animal = _run(PathObstacleDetector(), [_pt(30.0, -9.0, -V_EGO, 52)], vision=ANIMAL)
  assert far_animal.reject_reason == "off_path" and far_animal.chimed is False


def test_highway_stationary_in_path_needs_a_real_approach_and_a_vision_class():
  stuck = [_pt(40.0, 0.0, -25.0, 61)]
  det = PathObstacleDetector()
  chimed = False
  sample = None
  for i in range(25):
    sample = det.update(stuck, 25.0, PATH_X, PATH_Y, 0.1, vision=DEBRIS, now=10.0 + i * 0.1)
    chimed = chimed or bool(sample.chimed)
  assert sample is not None
  assert chimed is False
  assert sample.reject_reason == "not_closing"
  assert sample.active is False

  weak = VisionScore(conf=0.55, human=0.10, animal=0.10, obstacle=0.10)
  det = PathObstacleDetector()
  sample = None
  for i in range(10):
    x = 50.0 - i * 2.5
    sample = det.update([_pt(x, 0.0, -25.0, 62)], 25.0, PATH_X, PATH_Y, 0.1, vision=weak, now=30.0 + i * 0.1)
  assert sample is not None and sample.active and sample.agree
  assert sample.object_class == "unknown"
  assert sample.chimed is False and sample.chime_reason == "no_agree"

  det = PathObstacleDetector()
  chimed = False
  for i in range(10):
    x = 50.0 - i * 2.5
    sample = det.update([_pt(x, 0.0, -25.0, 63)], 25.0, PATH_X, PATH_Y, 0.1, vision=None, now=40.0 + i * 0.1)
    chimed = chimed or bool(sample.chimed)
  assert chimed is False

  det = PathObstacleDetector()
  chimed = False
  for i in range(10):
    x = 50.0 - i * 2.5
    sample = det.update([_pt(x, 0.0, -25.0, 64)], 25.0, PATH_X, PATH_Y, 0.1, vision=DEBRIS, now=50.0 + i * 0.1)
    chimed = chimed or bool(sample.chimed)
  # Oct 13: obstacle class is log only.
  assert chimed is False and sample.chime_reason == "log_only_class"


def test_speed_gate_skips_the_scan_below_15_mph():
  stage = ObstacleStage()
  stage._log_override = True
  calls = {"n": 0}
  real_begin = stage.det.begin

  def wrapped(*args, **kwargs):
    calls["n"] += 1
    return real_begin(*args, **kwargs)

  stage.det.begin = wrapped
  points = [_pt(30.0, 0.0, -20.0, 1)]
  path = [0.0, 100.0]
  t = 1000.0

  def step(speed, at):
    return stage.step(points, speed, path, path, 0.05, [], None, [], at)

  below = step(SPEED_ENABLE_MPS - 0.2, t)
  assert below is not None and below.reject_reason == "lowSpeed" and below.active is False
  assert calls["n"] == 0
  # 14 mph is under 15 and above 13, but the gate has never armed.
  assert step(6.26, t + 2.0).reject_reason == "lowSpeed"
  assert calls["n"] == 0

  armed = step(SPEED_ENABLE_MPS, t + 4.0)
  assert armed is None or armed.reject_reason != "lowSpeed"
  assert calls["n"] == 1
  # Hysteresis holds through 14 mph.
  step(SPEED_DISABLE_MPS + 0.4, t + 6.0)
  assert calls["n"] == 2
  dropped = step(SPEED_DISABLE_MPS - 0.2, t + 8.0)
  assert dropped is not None and dropped.reject_reason == "lowSpeed" and dropped.active is False
  assert calls["n"] == 2


def test_future_brake_is_any_agreed_object_in_or_entering():
  assert future_brake_intent(_agreed("animal", "inPath", 1)) is not None
  assert future_brake_intent(_agreed("obstacle", "inPath", 1)) is not None
  assert future_brake_intent(_agreed("unknown", "entering", 1)) is not None
  assert future_brake_intent(_agreed("human", "roadside", 1)) is None
  quiet = ObstacleSample(active=True, agree=False, zone="inPath", object_class="obstacle", member_ids=(1,))
  assert future_brake_intent(quiet) is None


def test_alert_sound_id_and_unique_wav():
  assert alert_sound_id(2, "obstacleChime/permanent") == OBSTACLE_CHIME_ID
  assert alert_sound_id(2, "") == 2
  assert OBSTACLE_CHIME_ID < 0
  path = ROOT / "selfdrive/assets/sounds/animal_chime.wav"
  with wave.open(str(path), "rb") as handle:
    assert handle.getnchannels() == 1
    assert handle.getsampwidth() == 2
    assert handle.getframerate() == 48000
    frames = handle.getnframes()
    duration = frames / 48000.0
    pcm = np.frombuffer(handle.readframes(frames), dtype=np.int16)
  assert 0.90 <= duration <= 1.15
  peak_level = float(np.max(np.abs(pcm))) / 32767.0
  assert peak_level >= 0.90
  spec = np.abs(np.fft.rfft(pcm.astype(np.float64)))
  freqs = np.fft.rfftfreq(len(pcm), 1.0 / 48000.0)
  peak = float(freqs[int(spec.argmax())])
  # G6 then C7, twice. Not engage's falling 1661 Hz note, and not the old low boop.
  assert peak > 1400.0
  assert not 1620.0 < peak < 1720.0
  g6 = spec[(freqs > 1500.0) & (freqs < 1640.0)].sum()
  c7 = spec[(freqs > 2000.0) & (freqs < 2200.0)].sum()
  low = spec[freqs < 800.0].sum()
  assert g6 > 0.04 * spec.sum()
  assert c7 > 0.02 * spec.sum()
  assert low < 0.15 * spec.sum()
  assert floor_volume(OBSTACLE_CHIME_ID, 0.355) == OBSTACLE_CHIME_MIN_VOLUME
  assert floor_volume(OBSTACLE_CHIME_ID, 0.91) == 0.91
  assert floor_volume(2, 0.355) == 0.355
  assert OBSTACLE_CHIME_MIN_VOLUME == 0.7


def test_thumbnail_prune_and_scan_cost():
  folder = ROOT / "selfdrive/controls/lib/tests/_thumb_tmp"
  folder.mkdir(exist_ok=True)
  try:
    for i in range(30):
      assert save_ppm(str(folder / f"{i:02d}.ppm"), [[i, 255 - i], [10, 20]])
    prune_thumbs(str(folder), max_files=24, max_bytes=2_000_000)
    assert len(list(folder.glob("*.ppm"))) <= 24
  finally:
    for path in folder.glob("*.ppm"):
      path.unlink()
    folder.rmdir()

  det = PathObstacleDetector()
  points = [
    _pt(8.0 + (i % 20) * 4.0, ((i % 9) - 4) * 0.4, -V_EGO, i + 1, yv_rel=0.05 * ((i % 5) - 2))
    for i in range(40)
  ]
  n = 50
  t0 = time.perf_counter()
  for i in range(n):
    det.update(points, V_EGO, PATH_X, PATH_Y, 0.05, now=float(i))
  mean_ms = (time.perf_counter() - t0) / n * 1000.0
  print(f"path obstacle scan mean {mean_ms:.3f} ms over {n} frames of 40 tracks")
  assert mean_ms < 5.0


def test_wiring_stays_off_the_control_core_and_off_longitudinal():
  def text(rel):
    return (ROOT / rel).read_text(encoding="utf-8")

  for rel in (
    "selfdrive/controls/controlsd.py",
    "selfdrive/controls/lib/longcontrol.py",
    "selfdrive/controls/lib/longitudinal_planner.py",
  ):
    src = text(rel)
    assert "pathObstacle" not in src
    assert "obstacleChime" not in src
    assert "future_brake_intent" not in src
  lib = text("selfdrive/controls/lib/path_obstacle.py")
  assert "actuators" not in lib
  assert PARAM_OBSTACLE_CHIME == "NAPObstacleChime"
  assert PARAM_OBSTACLE_LOG == "NAPObstacleLog"

  events = text("selfdrive/selfdrived/events.py")
  block = events.split("_obstacle_chime", 1)[1][:600]
  assert "ET.PERMANENT" in block
  assert "ET.WARNING" not in block
  assert "NO_ENTRY" not in block
  assert "SOFT_DISABLE" not in block
  assert "IMMEDIATE_DISABLE" not in block
  assert "Person ahead" not in events
  assert "Object ahead" in events
  assert "obstacle_chime_alert" in events

  selfd = text("selfdrive/selfdrived/selfdrived.py")
  assert "'pathObstacleVisionNAP'" in selfd
  assert "'pathObstacleNAP'" in selfd
  assert "should_raise_chime" in selfd
  assert "path_obstacle" not in selfd.split("class SelfdriveD", 1)[0]
  assert "a_cmd=float(self.sm['carControl'].actuators.accel),\n" in selfd

  keys = text("common/params_keys.h")
  assert '{"NAPObstacleChime", {PERSISTENT, BOOL, "1"}}' in keys
  assert '{"NAPObstacleLog", {PERSISTENT, BOOL, "1"}}' in keys
  services = text("cereal/services.py")
  assert '"pathObstacleNAP": (True, 0., 1)' in services
  assert '"pathObstacleVisionNAP": (True, 0., 1)' in services
  manner = text("selfdrive/ui/layouts/settings/driving_mannerisms.py")
  assert "Live object detection chime" in manner
  assert "livelyScore" in text("cereal/custom.capnp")
  assert "visionFailReason" in text("cereal/custom.capnp")
  assert "driveOverReason" in text("cereal/custom.capnp")
  assert "modelLeadAgree" in text("cereal/custom.capnp")
  assert "lowSpeed" in lib
  assert "self._all_items.append(self._turn_in_buttons)" in manner

  proc = text("selfdrive/pathobstacled/pathobstacled.py")
  radar = text("selfdrive/controls/radard.py")
  assert "vehicle_exclusion_points" in radar
  assert "leadsV3" in radar and ".leads" in radar
  assert "leadOne" in radar and "leadTwo" in radar
  assert "run_obstacle" in radar
  assert '"yaw_rate"' in radar
  assert "model_yaw_rate_right(sm['modelV2'])" in radar
  assert radar.count("SubMaster(") == 1
  sound = text("selfdrive/ui/soundd.py")
  thread = sound.split("def soundd_thread", 1)[1]
  assert thread.find("warningImmediate") < thread.find("floor_volume")
  assert "carState" not in thread
  assert "sub_sock('carState'" not in proc and 'sub_sock("carState"' not in proc
  assert "get_gps_location_service" in proc
  assert 'sub_sock("gpsLocationExternal"' not in proc and "sub_sock('gpsLocationExternal'" not in proc
  assert 'sub_sock("liveCalibration"' not in proc and "sub_sock('liveCalibration'" not in proc
  assert "CalibrationParams" in proc
  assert "pathObstacleVisionNAP" in proc
  assert ".copy()" in proc
  assert "exclVehicle" in text("selfdrive/controls/lib/path_obstacle.py")
  assert "CORES = [0, 1, 2, 3]" in proc
  assert "set_core_affinity(CORES)" in proc
  assert "os.nice(NICE)" in proc
  assert "NICE = 19" in proc
  assert "config_realtime_process(4" not in proc
  cfg = text("system/manager/process_config.py")
  assert 'PythonProcess("pathobstacled"' in cfg
  assert "optional=True" in cfg.split('PythonProcess("pathobstacled"', 1)[1][:400]
  optional = text("system/manager/optional_procs.py")
  assert "pathobstacled" in optional
  assert '"card"' not in optional or "card" not in optional.split("OPTIONAL_PROCESS_NAMES", 1)[1][:200]
  onroad = text("selfdrive/test/test_onroad.py")
  assert '"selfdrive.pathobstacled.pathobstacled": 1.0' in onroad


def test_oct7_yaw_curve_does_not_chime_and_stays_roadside():
  """20:02:12 route 174. Radar vLat was the car's own right turn at 119 m."""
  det = PathObstacleDetector()
  # device y +right. yRel is +left, so a target 5.91 m right is yRel -5.91.
  vision = VisionScore(conf=0.76, human=0.15, animal=0.06, obstacle=0.65)
  v_ego = 26.7
  yaw = 0.0126
  sample = None
  chimed = False
  for i in range(8):
    sample = det.update(
      [_pt(118.6, -5.91, 0.05 - v_ego, 952, yv_rel=1.62)],
      v_ego, PATH_X, PATH_Y, 0.1,
      vision=vision, lighting=0.43, yaw_rate=yaw, now=200.0 + i * 0.1,
    )
    chimed = chimed or bool(sample.chimed)
  assert sample is not None
  assert chimed is False
  assert sample.zone == "roadside"
  assert sample.object_class not in ("animal", "human")
  assert -0.40 < sample.v_lat < 0.20
  assert sample.vision_evaluated and sample.vision_conf > 0.70
  assert sample.agree is False
  assert sample.range_m > 85.0

  # The same turn must not look like the target wandered into the lane.
  curve = PathObstacleDetector()
  x = 80.0
  y0 = 6.0
  yaw_fast = 0.02
  dt = 0.1
  lively = None
  for i in range(16):
    psi = yaw_fast * dt * (i + 1)
    device_y = -math.sin(psi) * x + math.cos(psi) * y0
    lively = curve.update(
      [_pt(x, -device_y, -20.0, 951, yv_rel=yaw_fast * x)],
      20.0, PATH_X, PATH_Y, dt, yaw_rate=yaw_fast, now=300.0 + i * dt,
    )
  assert lively is not None
  assert abs(lively.v_lat) < 0.35
  assert lively.lively_score < 0.2
  assert lively.chimed is False


def test_oct7_yaw_animal_mislabel_does_not_chime():
  """20:02:31. A left curve made a roadside return look like a 4 m/s animal."""
  low = VisionScore(conf=0.58, human=0.10, animal=0.05, obstacle=0.20)
  assert _fuse_class("animal", 0.0, 1, low, raw=low) == "obstacle"
  assert _fuse_class("human", 0.0, 1, VisionScore(conf=0.70, human=0.22, animal=0.10, obstacle=0.15)) == "obstacle"
  kept = VisionScore(conf=0.70, human=0.10, animal=0.55, obstacle=0.10)
  assert _fuse_class("animal", 0.0, 1, kept, raw=kept) == "animal"

  det = PathObstacleDetector()
  v_ego = 26.0
  yaw = -0.058
  sample = None
  chimed = False
  for i in range(8):
    sample = det.update(
      [_pt(69.0, -7.7, -v_ego, 991, yv_rel=-4.4)],
      v_ego, PATH_X, PATH_Y, 0.1,
      vision=low, lighting=0.43, yaw_rate=yaw, now=400.0 + i * 0.1,
    )
    chimed = chimed or bool(sample.chimed)
  assert sample is not None
  assert chimed is False
  assert sample.zone == "roadside"
  assert sample.object_class != "animal"
  assert abs(sample.v_lat) < 1.0
  assert sample.chime_reason != "chimed"


def test_oct7_single_look_does_not_chime_and_a_real_obstacle_does():
  """20:09:02. One 0.70 look, then 0.44. A stopped in-path obstacle still chimes."""
  det = PathObstacleDetector()
  v_ego = 18.1
  good = VisionScore(conf=0.70, human=0.10, animal=0.08, obstacle=0.59)
  bad = VisionScore(conf=0.44, human=0.10, animal=0.08, obstacle=0.30)
  good_frame = None
  chimed = False
  for i in range(8):
    vision = good if i == 5 else bad if i == 6 else None
    sample = det.update(
      [_pt(46.4, -1.96, 0.25 - v_ego, 1520, yv_rel=0.62)],
      v_ego, PATH_X, PATH_Y, 0.1,
      vision=vision, lighting=0.40, yaw_rate=0.0003, now=500.0 + i * 0.1,
    )
    chimed = chimed or bool(sample.chimed)
    if i == 5:
      good_frame = sample
  assert good_frame is not None
  assert good_frame.agree is True
  assert good_frame.object_class == "obstacle"
  assert good_frame.chimed is False
  assert good_frame.chime_reason == "no_agree"
  assert chimed is False

  real = PathObstacleDetector()
  solid = VisionScore(conf=0.82, human=0.05, animal=0.06, obstacle=0.74)
  chimed_at = None
  last = None
  for i in range(8):
    vision = solid if i >= 4 else None
    last = real.update(
      [_pt(28.0, 0.0, -12.0, 77)], 12.0, PATH_X, PATH_Y, 0.1,
      vision=vision, now=600.0 + i * 0.1,
    )
    if last.chimed and chimed_at is None:
      chimed_at = i
  assert last is not None
  assert last.zone == "inPath" and last.object_class == "obstacle"
  # Agreed after the second look, but obstacle class is log only (Oct 13).
  assert not last.chimed and last.chime_reason == "log_only_class"
  assert chimed_at is None


def _solid_patch(lo, hi, size=28):
  """Mid-gray structure. No pixel near saturation."""
  span = max(hi - lo, 1)
  return [[lo + ((x * 3 + y * 5) % span) for x in range(size)] for y in range(size)]


def _blob_patch(background, blob, y0, y1, x0, x1, size=28):
  img = [[background for _ in range(size)] for _ in range(size)]
  for y in range(y0, y1):
    for x in range(x0, x1):
      img[y][x] = blob
  return img


def _night_run(points, vision, *, lighting=0.42, model_leads=(), v_ego=12.0, n=8):
  det = PathObstacleDetector()
  last = None
  chimed = False
  for i in range(n):
    last = det.update(
      points, v_ego, PATH_X, PATH_Y, 0.1,
      vision=vision, lighting=lighting, model_leads=model_leads,
      now=800.0 + i * 0.1,
    )
    chimed = chimed or bool(last.chimed)
  return last, chimed


def test_night_drive_over_suppresses_lights_and_keeps_real_obstacles():
  """Stationary in-lane returns at night. The Oct 7 leftovers were rails
  and road hardware whose crop was headlights or taillights.
  """
  assert is_night_lighting(0.43) and is_night_lighting(0.40)
  assert is_night_lighting(1.0) is False
  assert is_night_lighting(float("nan")) is False

  road = [[25] * 28 for _ in range(10)]
  headlights = _blob_patch(12, 252, 6, 20, 8, 22)
  head_score = score_row_patches(headlights, road)
  assert head_score.light_dominated
  assert head_score.light_frac >= 0.12
  assert head_score.conf >= 0.60 and head_score.obstacle >= 0.40

  tree = _solid_patch(70, 140)
  tree_score = score_row_patches(tree, road)
  assert tree_score.light_dominated is False
  assert tree_score.conf >= 0.60 and tree_score.obstacle >= 0.40

  rails = [[40] * 28 for _ in range(28)]
  rail_score = score_row_patches(rails, [[40] * 28 for _ in range(8)])
  assert rail_score.light_dominated is False
  assert rail_score.conf < 0.45

  # Red taillight blob, and the same pattern as a bright Y plus high V.
  red = [[(12, 12, 12) for _ in range(24)] for _ in range(24)]
  for y in range(8, 16):
    for x in range(4, 16):
      red[y][x] = (255, 8, 8)
  _near, _sat, red_frac, _blob = crop_light_stats(red)
  assert red_frac >= 0.06
  assert light_dominated(_near, _sat, red_frac, _blob)
  y_plane = _blob_patch(20, 210, 8, 16, 4, 16, size=24)
  v_plane = _blob_patch(128, 200, 4, 8, 2, 8, size=12)
  _near_v, _sat_v, red_v, _blob_v = crop_light_stats(y_plane, v_plane)
  assert red_v >= 0.06

  # (1) Headlight-saturated crop at night, in the lane: no chime.
  night_light, light_chimed = _night_run(
    [_pt(45.0, 0.0, -18.0, 1741)], head_score, v_ego=18.0,
  )
  assert light_chimed is False
  assert night_light.chime_reason == "drive_over"
  assert night_light.drive_over_reason == "light"
  assert night_light.zone == "inPath"

  # (2) Railroad crossing: no model lead, crop is not a solid object.
  rails_hit, rails_chimed = _night_run(
    [_pt(48.0, 0.0, -18.0, 976)], rail_score, v_ego=18.0,
  )
  assert rails_chimed is False
  assert rails_hit.chime_reason == "drive_over"
  assert rails_hit.drive_over_reason == "no_solid"

  # (3) Stalled car at night. The model lead is at the same spot, so it
  # chimes even when the crop is the car's own taillights.
  stalled, stalled_chimed = _night_run(
    [_pt(32.0, 0.0, -12.0, 501)], head_score,
    model_leads=[(32.0, 0.0, 0.80)],
  )
  assert not stalled_chimed
  assert stalled.chime_reason == "log_only_class"
  assert stalled.object_class == "obstacle"
  assert stalled.drive_over_reason == ""

  # (4) Fallen tree: solid, not a light, inside 60 m.
  fallen, fallen_chimed = _night_run([_pt(28.0, 0.0, -12.0, 880)], tree_score)
  assert not fallen_chimed and fallen.chime_reason == "log_only_class"
  assert fallen.drive_over_reason == ""
  far_tree, far_chimed = _night_run([_pt(75.0, 0.0, -12.0, 881)], tree_score)
  assert far_chimed is False
  assert far_tree.drive_over_reason == "beyond_60"

  # (5) The same headlight crop in daylight still chimes.
  day, day_chimed = _night_run(
    [_pt(45.0, 0.0, -18.0, 1742)], head_score, lighting=1.0, v_ego=18.0,
  )
  assert not day_chimed and day.chime_reason == "log_only_class"
  assert day.drive_over_reason == ""

  # Living things and anything crossing the lane are outside the rule.
  # Oct 13: a single radar point in the dark is never an animal chime.
  deer, deer_chimed = _night_run([_pt(30.0, 0.0, -12.0, 77)], ANIMAL)
  assert not deer_chimed and deer.object_class == "animal" and deer.chime_reason == "dark_single_point"
  # Two members (a real body) still chimes at night.
  pair = [_pt(30.0, 0.0, -12.0, 77), _pt(30.4, 0.5, -12.0, 177)]
  deer2, deer2_chimed = _night_run(pair, ANIMAL)
  assert deer2_chimed and deer2.object_class == "animal" and deer2.chime_reason == "chimed"
  lit_deer = replace_light(ANIMAL)
  still_deer, still_chimed = _night_run([_pt(30.0, 0.0, -12.0, 78)], lit_deer)
  assert not still_chimed and still_deer.object_class == "animal"
  walker, walker_chimed = _night_run(
    [_pt(25.0, -2.3, -12.0, 79, yv_rel=1.2)], replace_light(HUMAN),
  )
  assert walker_chimed and walker.zone == "entering" and walker.object_class == "human"


def replace_light(score: VisionScore) -> VisionScore:
  return VisionScore(
    conf=score.conf, human=score.human, animal=score.animal, obstacle=score.obstacle,
    light_dominated=True, light_frac=0.40, red_frac=0.10,
  )


# ---- Oct 13 2026: the nine chime events (8 false, 1 duplicate crossing SUV) ----

def _oct13(cls, zone, x, lat, *, light, members=1, span=0.0, along=0.0, v_lat=0.0,
           lively=0.0, vision=None, radar_class=None, model_lead=False):
  hit = _Hit(
    active=True, track_id=1, member_ids=tuple(range(1, members + 1)), x=x, y=lat, lateral=lat,
    v_rel=-10.0, v_lat=v_lat, along=along, radar_conf=0.9, radar_class=radar_class or cls,
    zone=zone, time_to_reach=3.0, time_to_enter=0.0, cluster_count=members, span_m=span,
    reject_reason="none", lighting=light, lively=lively, v_ego=15.0, model_lead=model_lead,
  )
  sample = ObstacleSample(
    active=True, track_id=1, range_m=x, lateral=lat, v_lat=v_lat, object_class=cls, zone=zone,
    agree=True, lighting_score=light, lively_score=lively, cluster_count=members, span_m=span,
    member_ids=hit.member_ids, vision_evaluated=vision is not None, reject_reason="none",
  )
  return chime_policy_block(sample, hit, vision)


def _vs(conf=0.6, human=0.0, animal=0.0, obstacle=0.0):
  return VisionScore(conf=conf, human=human, animal=animal, obstacle=obstacle)


def test_oct13_morning_animal_chimes_are_blocked():
  # 07:01:43 60 mph, sign post +4.3 m, single still point, dark, vision animal 0.52.
  assert _oct13("animal", "roadside", 47.0, 4.3, light=0.40, vision=_vs(animal=0.52)) == "still_animal"
  # 07:09:22 34 mph, tree line/sign +3.8 m, dark.
  assert _oct13("animal", "roadside", 30.0, 3.8, light=0.36, vision=_vs(animal=0.58)) == "still_animal"
  # 07:35:49 62 mph, +6.5 m: past the 20 ft cap from path centre.
  assert _oct13("animal", "roadside", 40.0, 6.5, light=0.46, vision=_vs(animal=0.80)) == "beyond_20ft"
  # 08:00:09 19 mph, parked box truck +4.5 m, along -0.6 m/s jitter but not sustained (lively 0).
  assert _oct13("animal", "roadside", 25.0, 4.5, light=0.44, along=-0.6, vision=_vs(animal=0.66)) == "still_animal"
  # Even with the old broken lively=1.0, a dark single point never chimes as an animal.
  assert _oct13("animal", "roadside", 47.0, 4.3, light=0.40, lively=1.0, vision=_vs(animal=0.52)) == "dark_single_point"


def test_oct13_human_and_obstacle_chimes_are_blocked():
  # 07:46:54 19 mph, pole -2.7…-3.2 m, radar class human, vision NaN.
  assert _oct13("human", "inPath", 20.0, -3.19, light=0.9, members=2, span=1.5) == "human_no_camera"
  # Same pole even if the camera had scored it: outside 3 ft of the path.
  assert _oct13("human", "inPath", 20.0, -3.19, light=0.9, members=2, span=1.5, vision=_vs(human=0.7)) == "human_off_3ft"
  # 11:12:31 29 mph, hydrant -2.7 m, "human roadside", vision never evaluated.
  assert _oct13("human", "roadside", 22.0, -2.7, light=1.0) == "human_no_camera"
  # 11:14:07 23 mph, crosswalk paint, in path at 42 m, radar human, no camera.
  assert _oct13("human", "inPath", 42.0, 0.1, light=1.0) == "human_no_camera"
  # 12:03:33 40 mph, pavement markings, obstacle 0.99: obstacle class is log only.
  assert _oct13("obstacle", "inPath", 61.0, 1.6, light=1.0, vision=_vs(conf=0.99, obstacle=0.99)) == "log_only_class"
  assert _oct13("unknown", "inPath", 43.0, 0.5, light=1.0) == "log_only_class"


def test_oct13_crossing_suv_belongs_to_crossing_vehicle_handling():
  # 11:13:08 16 mph, SUV pulling out across the lane, +0.5 m at 43 m.
  assert _oct13("human", "inPath", 43.0, 0.5, light=1.0, members=2, span=1.4, v_lat=1.5, along=0.8,
                vision=_vs(human=0.6)) == "crossing_vehicle"
  # A walker crossing (single point) is not a crossing vehicle and still chimes.
  assert _oct13("human", "inPath", 20.0, 0.5, light=1.0, v_lat=1.2, along=0.2, lively=1.0,
                vision=_vs(human=0.6)) is None


def test_oct13_road_surface_and_real_animals():
  # Still, single in-path point at 40–60 m with only a weak camera look: road surface.
  assert _oct13("animal", "inPath", 50.0, 0.2, light=1.0, vision=_vs(conf=0.3, animal=0.8)) == "road_surface_single"
  # A second member, or a solid camera hit, lets it through.
  assert _oct13("animal", "inPath", 50.0, 0.2, light=1.0, members=2, span=0.6, vision=_vs(conf=0.3, animal=0.8)) is None
  assert _oct13("animal", "inPath", 50.0, 0.2, light=1.0, vision=_vs(conf=0.8, animal=0.8)) is None
  # Real moving deer on the shoulder in daylight chimes; a strong still deer chimes.
  assert _oct13("animal", "roadside", 30.0, 4.0, light=0.9, lively=1.0, vision=_vs(animal=0.5)) is None
  assert _oct13("animal", "roadside", 30.0, 4.0, light=0.9, vision=_vs(animal=0.76)) is None
  # Person within 3 ft of the path edge, camera scored: chimes.
  assert _oct13("human", "roadside", 15.0, 2.4, light=1.0, vision=_vs(human=0.6)) is None


def test_oct13_speed_gate_unchanged():
  assert abs(SPEED_ENABLE_MPS - 6.7) < 1e-6
