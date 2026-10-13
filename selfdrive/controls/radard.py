#!/usr/bin/env python3
import math
import time
import numpy as np
from collections import deque
from typing import Any

import capnp
from cereal import messaging, log, car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D
from openpilot.selfdrive.controls.lib.path_obstacle import (
  ConeHint,
  ObstacleStage,
  hit_to_msg,
  vehicle_exclusion_points,
)
from openpilot.selfdrive.controls.lib.cone_line import (
  ConeLineDetector,
  ConeLineSample,
  lane_room_m,
  model_yaw_rate_right,
  publish_cone_line,
  road_edges_xy,
)
from openpilot.selfdrive.controls.lib.crossing_vehicle import GroundLateral
from openpilot.selfdrive.controls.lib.cone_line_hold import PARAM_CONE_LINE_LOG
from openpilot.selfdrive.controls.lib.radar_path_gate import (
  PATH_INCUMBENT_HALF_WIDTH_M,
  RADAR_TO_CAMERA_M,
  collapse_blocks_new_lead,
  model_path_xy,
  path_lateral_m,
  path_model_collapsed,
  radar_follow_ok,
  vision_lead_follow_ok,
)
from openpilot.selfdrive.controls.lib.radar_sensor_dirty import (
  SensorDirtyPolicy,
  reason_token,
  sensor_dirty_ignore_enabled,
)
from openpilot.selfdrive.controls.lib.rain_radar_hold import (
  RAIN_RADAR_LOST_HOLD_FRAMES,
  RAIN_VISION_ONLY_MIN_PROB,
  RadarReliability,
  RainRadarGate,
  pick_rain_radar_track,
  rain_far_hold_ok,
)


# Default lead acceleration decay set to 50% at 1s
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1     # Kalman filter states enum

# stationary qualification parameters
V_EGO_STATIONARY = 4.   # no stationary object flag below this speed

RADAR_TO_CENTER = 2.7   # (deprecated) RADAR is ~ 2.7m ahead from center of car
RADAR_TO_CAMERA = 1.52  # RADAR is ~ 1.5m ahead from center of mesh frame

# Bosch radar updates at 8 Hz; use actual measurement interval for KF
RADAR_DT = 1.0 / 8
RADAR_MEASUREMENT_TIMEOUT = 0.5

ASSOCIATION_DISTANCE_GATE = 3.0
ASSOCIATION_MIN_DISTANCE_STD = 1.0
ASSOCIATION_LATERAL_GATE = 3.0
ASSOCIATION_MIN_LATERAL_STD = 0.5
ASSOCIATION_VELOCITY_GATE = 3.0
ASSOCIATION_MIN_VELOCITY_STD = 1.0
# The independent 3-sigma gates below reject single-axis outliers. Keep this
# combined likelihood floor lower so ordinary multi-axis noise does not cause
# repeated radar/vision fallback transitions.
ASSOCIATION_MIN_SCORE = 0.001
ASSOCIATION_SWITCH_MARGIN = 1.5


class KalmanParams:
  def __init__(self, dt: float):
    # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
    # hardcoding a lookup table to compute K for values of radar_ts between 0.01s and 0.2s
    assert dt > .01 and dt < .2, "Radar time step must be between .01s and 0.2s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    #Q = np.matrix([[10., 0.0], [0.0, 100.]])
    #R = 1e3
    #K = np.matrix([[ 0.05705578], [ 0.03073241]])
    dts = [i * 0.01 for i in range(1, 21)]
    K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,  0.21372394,
          0.22761098, 0.24069424, 0.253096,   0.26491023, 0.27621103, 0.28705801,
          0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
          0.35353899, 0.36200124]
    K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
          0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
          0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
          0.26393339, 0.26278425]
    self.K = [[np.interp(dt, dts, K0)], [np.interp(dt, dts, K1)]]


class Track:
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    self.identifier = identifier
    self.cnt = 0
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)
    # Ground-frame lateral speed, +left. 0 until the yaw-corrected fit has a window.
    self.vLat = 0.0

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float, measured: float,
             kalman_params: KalmanParams | None = None):
    # relative values, copy
    self.dRel = d_rel   # LONG_DIST
    self.yRel = y_rel   # -LAT_DIST
    self.vRel = v_rel   # REL_SPEED
    self.vLead = v_lead
    self.measured = measured   # measured or estimate

    # computed velocity and accelerations
    if measured and kalman_params is not None:
      self._set_kalman_params(kalman_params)
    if self.cnt > 0 and measured:
      self.kf.update(self.vLead)

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

    # Learn if constant acceleration
    if abs(self.aLeadK) < 0.5:
      self.aLeadTau.x = _LEAD_ACCEL_TAU
    else:
      self.aLeadTau.update(0.0)

    self.cnt += 1

  def _set_kalman_params(self, kalman_params: KalmanParams):
    state = self.kf.x
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D(state, self.K_A, self.K_C, self.K_K)

  def get_RadarState(self, model_prob: float = 0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "status": True,
      "fcw": self.is_potential_fcw(model_prob),
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
      "vLat": float(self.vLat),
    }

  def potential_low_speed_lead(self, v_ego: float):
    # stop for stuff in front of you and low speed, even without model confirmation
    # Radar points closer than 0.75, are almost always glitches on toyota radars
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)

  def is_potential_fcw(self, model_prob: float):
    return model_prob > .9

  def __str__(self):
    ret = f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"
    return ret


def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


def association_score(v_ego: float, vision_d_rel: float, lead: capnp._DynamicStructReader, track: Track) -> float:
  distance_probability = laplacian_pdf(track.dRel, vision_d_rel, lead.xStd[0])
  lateral_probability = laplacian_pdf(track.yRel, -lead.y[0], lead.yStd[0])
  velocity_probability = laplacian_pdf(track.vRel + v_ego, lead.v[0], lead.vStd[0])
  return distance_probability * lateral_probability * velocity_probability


def is_association_candidate(v_ego: float, vision_d_rel: float, lead: capnp._DynamicStructReader,
                             track: Track, score: float,
                             path_x=None, path_y=None, collapsed=False,
                             incumbent=False) -> bool:
  # Vision already nominated this lead. Allow lane-edge / early cut-in
  # (incumbent half-width). Unassociated prefer pick stays on the tighter
  # acquire gate and still requires a path association.
  # A collapsed path raises the bar for a *new* |yRel| ≳ 2 association
  # (construction right-side blip). The incumbent track is unchanged.
  if collapse_blocks_new_lead(track.yRel, collapsed, incumbent=incumbent):
    return False
  if not radar_follow_ok(track, v_ego, path_x, path_y,
                         max_lat=PATH_INCUMBENT_HALF_WIDTH_M):
    return False
  distance_limit = ASSOCIATION_DISTANCE_GATE * max(lead.xStd[0], ASSOCIATION_MIN_DISTANCE_STD)
  lateral_limit = ASSOCIATION_LATERAL_GATE * max(lead.yStd[0], ASSOCIATION_MIN_LATERAL_STD)
  velocity_limit = ASSOCIATION_VELOCITY_GATE * max(lead.vStd[0], ASSOCIATION_MIN_VELOCITY_STD)

  distance_compatible = abs(track.dRel - vision_d_rel) <= distance_limit
  lateral_compatible = abs(track.yRel + lead.y[0]) <= lateral_limit
  velocity_compatible = abs(track.vRel + v_ego - lead.v[0]) <= velocity_limit
  return distance_compatible and lateral_compatible and velocity_compatible and score >= ASSOCIATION_MIN_SCORE


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, tracks: dict[int, Track],
                          incumbent_track_id: int | None = None,
                          path_x=None, path_y=None, collapsed=False) -> Track | None:
  vision_d_rel = lead.x[0] - RADAR_TO_CAMERA
  scores = {track_id: association_score(v_ego, vision_d_rel, lead, track) for track_id, track in tracks.items()}
  eligible_track_ids = [
    track_id for track_id, track in tracks.items()
    if is_association_candidate(
      v_ego, vision_d_rel, lead, track, scores[track_id],
      path_x=path_x, path_y=path_y, collapsed=collapsed,
      incumbent=track_id == incumbent_track_id,
    )
  ]
  if not eligible_track_ids:
    return None

  challenger_track_id = max(eligible_track_ids, key=scores.__getitem__)
  if incumbent_track_id in eligible_track_ids and challenger_track_id != incumbent_track_id:
    challenger_wins = scores[challenger_track_id] > scores[incumbent_track_id] * ASSOCIATION_SWITCH_MARGIN
    if not challenger_wins:
      return tracks[incumbent_track_id]

  return tracks[challenger_track_id]


def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float):
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  return {
    "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLeadK": float(lead_msg.a[0]),
    "aLeadTau": 0.3,
    "fcw": False,
    "modelProb": float(lead_msg.prob),
    "status": True,
    "radar": False,
    "radarTrackId": -1,
  }


class LeadTrackAssociation:
  def __init__(self, low_speed_override: bool):
    self.low_speed_override = low_speed_override
    self.incumbent_track_id: int | None = None
    self._rain_lost_frames = 0
    self._held_radar_lead: dict[str, Any] | None = None

  def update(self, v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float, rain_hold: bool = False, radar_prefer: bool | None = None,
             path_x=None, path_y=None, collapsed=False) -> dict[str, Any]:
    if radar_prefer is None:
      radar_prefer = rain_hold
    if tracks and ready and lead_msg.prob > .5:
      track = match_vision_to_track(v_ego, lead_msg, tracks, self.incumbent_track_id,
                                    path_x=path_x, path_y=path_y, collapsed=collapsed)
    else:
      track = None

    if radar_prefer:
      track = pick_rain_radar_track(track, tracks, self.incumbent_track_id, v_ego,
                                    path_x, path_y, vision_prob=float(lead_msg.prob))
      if track is not None:
        self._rain_lost_frames = 0
      elif (self.incumbent_track_id is not None and
            self.incumbent_track_id in tracks):
        # Live track failed path/oncoming / far-low-prob — do not keep the cache.
        self._held_radar_lead = None
        self._rain_lost_frames = 0
      elif tracks:
        # Incumbent gone; live table has other tracks that did not qualify.
        # Do not ghost-hold a disappeared lead over oncoming / off-path clutter.
        self._held_radar_lead = None
        self._rain_lost_frames = 0
      elif self._held_radar_lead is not None:
        self._rain_lost_frames += 1
      else:
        self._rain_lost_frames = 0
    else:
      self._rain_lost_frames = 0
      self._held_radar_lead = None

    vision_prob_min = RAIN_VISION_ONLY_MIN_PROB if radar_prefer else .5

    lead_dict = {'status': False}
    if track is not None:
      lead_dict = track.get_RadarState(lead_msg.prob)
    elif (radar_prefer and self._held_radar_lead is not None and
          self._rain_lost_frames <= RAIN_RADAR_LOST_HOLD_FRAMES and
          radar_follow_ok(self._held_radar_lead, v_ego, path_x, path_y,
                          max_lat=PATH_INCUMBENT_HALF_WIDTH_M) and
          rain_far_hold_ok(self._held_radar_lead, None, float(lead_msg.prob))):
      lead_dict = dict(self._held_radar_lead)
      lead_dict["modelProb"] = float(lead_msg.prob)
    elif (ready and lead_msg.prob >= vision_prob_min and
          vision_lead_follow_ok(lead_msg, v_ego, path_x, path_y)):
      lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)
      # Vision-only at |yRel| ≳ 2 is a new association. Do not start one
      # while the path model is collapsed (right-side construction blip).
      if collapse_blocks_new_lead(lead_dict.get("yRel"), collapsed, incumbent=False):
        lead_dict = {'status': False}

    if self.low_speed_override:
      low_speed_tracks = [candidate for candidate in tracks.values() if candidate.potential_low_speed_lead(v_ego)]
      if low_speed_tracks:
        closest_track = min(low_speed_tracks, key=lambda candidate: candidate.dRel)
        if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
          lead_dict = closest_track.get_RadarState()

    if lead_dict.get('status'):
      lat = path_lateral_m(lead_dict, path_x, path_y)
      if lat is not None:
        lead_dict["dPath"] = float(lat)

    if lead_dict.get('radar', False):
      self.incumbent_track_id = lead_dict['radarTrackId']
      self._held_radar_lead = dict(lead_dict)
    else:
      self.incumbent_track_id = None
    return lead_dict


def _capnp_attr(obj, name):
  try:
    return getattr(obj, name)
  except Exception:
    return None


def _radar_point(pt) -> dict:
  """Plain snapshot so the scan never writes back into the live radar message."""
  measured = _capnp_attr(pt, "measured")
  item = {
    "dRel": _capnp_attr(pt, "dRel"),
    "yRel": _capnp_attr(pt, "yRel"),
    "vRel": _capnp_attr(pt, "vRel"),
    "trackId": _capnp_attr(pt, "trackId"),
    "measured": True if measured is None else bool(measured),
  }
  yv = _capnp_attr(pt, "yvRel")
  if yv is not None:
    item["yvRel"] = yv
  rcs = _capnp_attr(pt, "rcs")
  if rcs is None:
    rcs = _capnp_attr(pt, "rcsDb")
  if rcs is not None:
    item["rcs"] = rcs
  return item


def _radar_lead(lead) -> dict | None:
  try:
    if lead is None or not bool(lead.status):
      return None
    track = _capnp_attr(lead, "radarTrackId")
    prob = _capnp_attr(lead, "modelProb")
    return {
      "status": True,
      "dRel": float(lead.dRel),
      "yRel": float(lead.yRel),
      "modelProb": 1.0 if prob is None else float(prob),
      "radarTrackId": -1 if track is None else int(track),
    }
  except Exception:
    return None


def _model_lead_tuples(leads, xyva: bool) -> list:
  out = []
  try:
    seq = list(leads) if leads is not None else []
  except TypeError:
    return out
  for item in seq:
    try:
      prob = float(item.prob)
      if xyva:
        xy = item.xyva
        out.append((float(xy[0]), float(xy[1]), prob))
      else:
        out.append((float(item.x[0]), float(item.y[0]), prob))
    except Exception:
      continue
  return out


def _cone_hint(sample) -> ConeHint | None:
  if sample is None:
    return None
  try:
    return ConeHint(
      active=bool(sample.active),
      side=int(sample.side),
      lat_near=float(sample.lat_near),
      lat_mid=float(sample.lat_mid),
      lat_far=float(sample.lat_far),
      barrier=bool(sample.barrier),
      parked=bool(sample.parked),
    )
  except Exception:
    return None


def _lane_prob_min(model) -> float:
  try:
    probs = list(model.laneLineProbs)
    if len(probs) >= 3:
      return float(min(float(probs[1]), float(probs[2])))
  except Exception:
    pass
  return 1.0


def _path_y_std_3s(model):
  try:
    ts = [float(v) for v in model.position.t]
    stds = [float(v) for v in model.position.yStd]
  except Exception:
    return None
  if not ts or not stds:
    return None
  n = min(len(ts), len(stds))
  idx = min(range(n), key=lambda i: abs(ts[i] - 3.0))
  return stds[idx]


class RadarD:
  def __init__(self, delay: float = 0.0, rain_gate: RainRadarGate | None = None):
    self.current_time = 0.0

    self.tracks: dict[int, Track] = {}
    self.kalman_params = KalmanParams(RADAR_DT)
    self.last_radar_update_time: float | None = None

    self.v_ego = 0.0
    self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL))+1)
    self.last_v_ego_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False

    self.ready = False
    self.lead_one_association = LeadTrackAssociation(low_speed_override=True)
    self.lead_two_association = LeadTrackAssociation(low_speed_override=False)
    self.rain_gate = rain_gate if rain_gate is not None else RainRadarGate()
    self.reliability = RadarReliability()
    self.engaged = False
    # Bosch SensorDirty: degrade (keep the radar lead) instead of soft-disable.
    self.sensor_dirty = SensorDirtyPolicy()
    self._sensor_dirty_ignore_override: bool | None = None
    self._live_measured = False
    # Cone line is log-only. It does not select a lead or touch longitudinal.
    self._cone = ConeLineDetector()
    self._cone_sample = None
    self._cone_dirty = False
    self._cone_log_on = True
    self._cone_log_check_t = -1.0
    # Radar half of the path-obstacle log. The scan runs after radarState
    # is published, so a slow or failing scan cannot change lead selection.
    self._obstacle = ObstacleStage()
    self._obstacle_pending = False
    self._obstacle_dt = RADAR_DT
    self._obstacle_inputs = None
    # Yaw-corrected lateral speed for the lead. Same modelV2 radard already
    # reads; no extra carState subscriber.
    self._ground = GroundLateral()

  def set_sensor_dirty_ignore_override(self, ignore: bool | None) -> None:
    """Test hook. None reads NAPRadarIgnoreSensorDirty (default on)."""
    self._sensor_dirty_ignore_override = None if ignore is None else bool(ignore)

  def read_sensor_dirty_ignore(self) -> bool:
    if self._sensor_dirty_ignore_override is not None:
      return self._sensor_dirty_ignore_override
    return sensor_dirty_ignore_enabled(self.rain_gate._get_params())

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self._obstacle_pending = False
    self.ready = sm.seen['modelV2']
    self.current_time = 1e-9*max(sm.logMonoTime.values())

    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']
      cruise = getattr(sm['carState'], 'cruiseState', None)
      self.engaged = bool(getattr(cruise, 'enabled', False))

    radar_timed_out = False
    if sm.updated['liveTracks']:
      radar_update_time = 1e-9 * sm.logMonoTime['liveTracks']
      radar_dt = RADAR_DT
      if self.last_radar_update_time is not None:
        observed_radar_dt = radar_update_time - self.last_radar_update_time
        if 0.01 < observed_radar_dt < 0.2:
          radar_dt = observed_radar_dt
      self.last_radar_update_time = radar_update_time
      self.kalman_params = KalmanParams(radar_dt)
      ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel, pt.measured] for pt in rr.points}
      self._live_measured = any(rpt[3] for rpt in ar_pts.values())

      # *** remove missing points from meta data ***
      for ids in list(self.tracks.keys()):
        if ids not in ar_pts:
          self.tracks.pop(ids, None)

      # *** compute the tracks ***
      for ids in ar_pts:
        rpt = ar_pts[ids]

        # align v_ego by a fixed time to align it with the radar measurement
        v_lead = rpt[2] + self.v_ego_hist[0]

        # create the track if it doesn't exist or it's a new track
        if ids not in self.tracks:
          self.tracks[ids] = Track(ids, v_lead, self.kalman_params)
        self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, rpt[3], self.kalman_params)
      self._update_ground(radar_update_time, sm)
      self._update_cone_line(sm, rr.points, radar_dt)
      self._obstacle_dt = radar_dt
      self._obstacle_pending = True
    elif self.last_radar_update_time is None or self.current_time - self.last_radar_update_time > RADAR_MEASUREMENT_TIMEOUT:
      self.tracks.clear()
      self._ground.reset()
      radar_timed_out = self.last_radar_update_time is not None
      self._live_measured = False
      self._update_cone_line(sm, (), RADAR_DT)

    # *** publish radarState ***
    # Exclude liveTracks from validity check: it arrives at radar rate (8Hz for
    # Bosch) not the 20Hz SubMaster expects, so all_checks() would mark it stale
    # between radar updates and cascade valid=False through the whole pipeline.
    self.radar_state_valid = sm.all_checks(service_list=['modelV2', 'carState'])
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors
    self.radar_state.carStateMonoTime = sm.logMonoTime['carState']
    # SensorDirty is the radar's own obstruction self-report. With live
    # measured tracks it must not soft-disable or lock out re-engage
    # (I-40 gorge, Oct 4 03:03:59 CT). Only that one flag is masked, and only
    # until it persists with no live tracks (SENSOR_DIRTY_PERSIST_S).
    sensor_dirty_flag = bool(getattr(rr.errors, 'radarUnavailableTemporary', False))
    sensor_dirty_mask = self.sensor_dirty.update(
      sensor_dirty_flag, self._live_measured and not radar_timed_out, DT_MDL,
      ignore=self.read_sensor_dirty_ignore())
    if sensor_dirty_mask:
      self.radar_state.radarErrors.radarUnavailableTemporary = False

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego
    leads_v3 = sm['modelV2'].leadsV3
    # Prefer is default (Radar Enabled + healthy). Auto wiper is not required.
    # Unhealthy / disabled → stock fusion. Path/oncoming gates still apply.
    # Health first so association uses the new prefer; HUD after so a
    # path-associated lead can suppress clutter / timeout flashes.
    healthy = self.reliability.update(
      tracks=self.tracks, errors=self.radar_state.radarErrors, v_ego=self.v_ego,
      timed_out=radar_timed_out,
      ignore_hw_fail=self.rain_gate.read_ignore_hw_fail(),
      engaged=self.engaged)
    self.rain_gate.set_reliable(healthy, alert=False)
    radar_prefer = bool(self.rain_gate.update())
    path_x, path_y = model_path_xy(sm['modelV2'])
    try:
      lane_probs = list(sm['modelV2'].laneLineProbs)
    except (TypeError, AttributeError):
      lane_probs = None
    collapsed = path_model_collapsed(lane_probs, path_x, path_y)
    lead_one: dict[str, Any] = {'status': False}
    if len(leads_v3) > 1:
      lead_one = self.lead_one_association.update(
        self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego,
        rain_hold=radar_prefer, radar_prefer=radar_prefer,
        path_x=path_x, path_y=path_y, collapsed=collapsed)
      self.radar_state.leadOne = lead_one
      self.radar_state.leadTwo = self.lead_two_association.update(
        self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego,
        rain_hold=radar_prefer, radar_prefer=radar_prefer,
        path_x=path_x, path_y=path_y, collapsed=collapsed)
    path_lead = bool(lead_one.get('status') and lead_one.get('radar'))
    self.reliability.set_path_lead(path_lead)
    self.rain_gate.set_reliable(healthy, alert=self.reliability.should_alert)
    if hasattr(self.radar_state, "radarPreferFallback"):
      # Degraded SensorDirty reuses the existing fallback event (selfdrived maps it to a
      # WARNING, never a disable); events.py words it from radarPreferReason.
      self.radar_state.radarPreferFallback = bool(self.rain_gate.fallback_alert) or sensor_dirty_mask
    if hasattr(self.radar_state, "radarPreferReason"):
      self.radar_state.radarPreferReason = reason_token(self.reliability.log_reason, sensor_dirty_mask)
    if self._obstacle_pending:
      self._stash_obstacle(sm, rr, path_x, path_y, leads_v3)

  def _update_ground(self, t: float, sm) -> None:
    """Publish each track's ground-frame lateral speed. Failures leave vLat at 0."""
    try:
      # model orientationRate.z is +right. yRel and vLat are +left.
      yaw_left = -model_yaw_rate_right(sm['modelV2'])
      points = [(int(tr.identifier), float(tr.dRel), float(tr.yRel)) for tr in self.tracks.values()]
      speeds = self._ground.update(float(t), float(self.v_ego_hist[0]), yaw_left, points)
    except Exception:
      cloudlog.exception("ground lateral speed failed")
      return
    for tid, speed in speeds.items():
      tr = self.tracks.get(tid)
      if tr is not None:
        tr.vLat = float(speed)

  def _cone_logging(self) -> bool:
    """NAPConeLineLog, cached ~1 s. Default On. Missing params stay On."""
    now = self.current_time if self.current_time else 0.0
    if self._cone_log_check_t >= 0.0 and now - self._cone_log_check_t < 1.0:
      return self._cone_log_on
    self._cone_log_check_t = now
    try:
      self._cone_log_on = bool(Params().get_bool(PARAM_CONE_LINE_LOG))
    except Exception:
      self._cone_log_on = True
    return self._cone_log_on

  def _update_cone_line(self, sm, points, dt: float) -> None:
    """One O(n) scan at radar rate. Failures stay off the lead path."""
    if not self._cone_logging():
      # One inactive sample so a held line starts its clear timer, then silence.
      was_active = self._cone_sample is not None and self._cone_sample.active
      self._cone.reset()
      self._cone_sample = ConeLineSample() if was_active else None
      self._cone_dirty = self._cone_sample is not None
      return
    try:
      path_x, path_y = model_path_xy(sm['modelV2'])
      prev_side = self._cone_sample.side if self._cone_sample is not None else 0
      self._cone_sample = self._cone.update(
        points, self.v_ego, path_x, path_y, road_edges_xy(sm['modelV2']), dt,
        model_yaw_rate_right(sm['modelV2']),
        lead_xs=_cone_lead_xs(sm['modelV2'], self.radar_state),
        lane_room=lane_room_m(sm['modelV2'], prev_side) if prev_side else None)
      self._cone_dirty = True
    except Exception:
      cloudlog.exception("cone line detector failed")
      self._cone_sample = None
      self._cone_dirty = False

  def _stash_obstacle(self, sm, rr, path_x, path_y, leads_v3) -> None:
    """Copy the inputs the scan needs. Failures stay off the lead path."""
    try:
      points = [_radar_point(pt) for pt in rr.points]
    except Exception:
      cloudlog.exception("path obstacle stash failed")
      self._obstacle_inputs = None
      self._obstacle_pending = False
      return
    lead_ids: list = []
    model_leads: list = []
    try:
      leads_v2 = sm['modelV2'].leads
      radar_leads = []
      if self.radar_state is not None:
        radar_leads = [_radar_lead(self.radar_state.leadOne), _radar_lead(self.radar_state.leadTwo)]
      lead_ids, model_leads = vehicle_exclusion_points(
        _model_lead_tuples(leads_v3, xyva=False),
        _model_lead_tuples(leads_v2, xyva=True),
        [lead for lead in radar_leads if lead is not None],
      )
    except Exception:
      lead_ids, model_leads = [], []
    try:
      lane = _lane_prob_min(sm['modelV2'])
      path_std = _path_y_std_3s(sm['modelV2'])
    except Exception:
      lane, path_std = 1.0, None
    self._obstacle_inputs = {
      "points": points,
      "dt": float(self._obstacle_dt),
      "v_ego": float(self.v_ego),
      "path_x": list(path_x) if path_x is not None else None,
      "path_y": list(path_y) if path_y is not None else None,
      "lead_ids": lead_ids,
      "model_leads": model_leads,
      "cone": _cone_hint(self._cone_sample),
      "lane_prob_min": lane,
      "path_y_std": path_std,
      # Same model yaw the cone-line scan uses. No extra carState reader.
      "yaw_rate": model_yaw_rate_right(sm['modelV2']),
    }

  def run_obstacle(self, pm) -> None:
    """Radar scan and pathObstacleNAP publish. Call after radarState is sent."""
    if not self._obstacle_pending:
      return
    self._obstacle_pending = False
    inputs = self._obstacle_inputs
    self._obstacle_inputs = None
    if not inputs:
      return
    try:
      hit = self._obstacle.step(
        inputs["points"], inputs["v_ego"], inputs["path_x"], inputs["path_y"], inputs["dt"],
        inputs["lead_ids"], inputs["cone"], inputs["model_leads"], time.monotonic(),
        inputs.get("yaw_rate", 0.0),
      )
      if hit is None or pm is None:
        return
      self._publish_obstacle(pm, hit, inputs)
    except Exception:
      cloudlog.exception("path obstacle radar stage failed")
      self._obstacle.note_failure()

  def _publish_obstacle(self, pm, hit, inputs) -> None:
    msg = messaging.new_message("pathObstacleNAP", valid=True)
    dest = msg.pathObstacleNAP
    hit_to_msg(hit, dest)
    dest.laneProbMin = float(inputs.get("lane_prob_min", 1.0))
    std = inputs.get("path_y_std")
    dest.pathYStd3s = float("nan") if std is None else float(std)
    dest.overBudget = hit.reject_reason == "over_budget"
    dest.heartbeat = bool(self._obstacle.heartbeat)
    pm.send("pathObstacleNAP", msg)

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)
    if self._cone_dirty:
      self._cone_dirty = False
      try:
        publish_cone_line(pm, self._cone_sample)
      except Exception:
        cloudlog.exception("coneLineNAP publish failed")


def _cone_lead_xs(model, radar_state) -> list[float]:
  """Camera-x ranges of leads in our lane, for the cone-shift lead-in-span gate."""
  out: list[float] = []
  try:
    for lead in list(model.leadsV3)[:2]:
      if float(lead.prob) >= 0.5 and len(lead.x):
        out.append(float(lead.x[0]))
  except Exception:
    pass
  for name in ("leadOne", "leadTwo"):
    try:
      lead = getattr(radar_state, name)
      if bool(lead.status) and abs(float(lead.yRel)) < 1.8:
        out.append(float(lead.dRel) + RADAR_TO_CAMERA_M)
    except Exception:
      continue
  return out


# fuses camera and radar data for best lead detection
def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  # *** setup messaging
  sm = messaging.SubMaster(['modelV2', 'carState', 'liveTracks'], poll='modelV2')
  pm = messaging.PubMaster(['radarState', 'coneLineNAP', 'pathObstacleNAP'])

  RD = RadarD(CP.radarDelay)

  while 1:
    sm.update()

    if sm.updated['modelV2']:
      RD.update(sm, sm['liveTracks'])
      RD.publish(pm)
      RD.run_obstacle(pm)


if __name__ == "__main__":
  main()
