#!/usr/bin/env python3
import math
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
                             track: Track, score: float) -> bool:
  distance_limit = ASSOCIATION_DISTANCE_GATE * max(lead.xStd[0], ASSOCIATION_MIN_DISTANCE_STD)
  lateral_limit = ASSOCIATION_LATERAL_GATE * max(lead.yStd[0], ASSOCIATION_MIN_LATERAL_STD)
  velocity_limit = ASSOCIATION_VELOCITY_GATE * max(lead.vStd[0], ASSOCIATION_MIN_VELOCITY_STD)

  distance_compatible = abs(track.dRel - vision_d_rel) <= distance_limit
  lateral_compatible = abs(track.yRel + lead.y[0]) <= lateral_limit
  velocity_compatible = abs(track.vRel + v_ego - lead.v[0]) <= velocity_limit
  return distance_compatible and lateral_compatible and velocity_compatible and score >= ASSOCIATION_MIN_SCORE


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, tracks: dict[int, Track],
                          incumbent_track_id: int | None = None) -> Track | None:
  vision_d_rel = lead.x[0] - RADAR_TO_CAMERA
  scores = {track_id: association_score(v_ego, vision_d_rel, lead, track) for track_id, track in tracks.items()}
  eligible_track_ids = [
    track_id for track_id, track in tracks.items()
    if is_association_candidate(v_ego, vision_d_rel, lead, track, scores[track_id])
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

  def update(self, v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float) -> dict[str, Any]:
    if tracks and ready and lead_msg.prob > .5:
      track = match_vision_to_track(v_ego, lead_msg, tracks, self.incumbent_track_id)
    else:
      track = None

    lead_dict = {'status': False}
    if track is not None:
      lead_dict = track.get_RadarState(lead_msg.prob)
    elif ready and lead_msg.prob > .5:
      lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)

    if self.low_speed_override:
      low_speed_tracks = [candidate for candidate in tracks.values() if candidate.potential_low_speed_lead(v_ego)]
      if low_speed_tracks:
        closest_track = min(low_speed_tracks, key=lambda candidate: candidate.dRel)
        if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
          lead_dict = closest_track.get_RadarState()

    self.incumbent_track_id = lead_dict['radarTrackId'] if lead_dict.get('radar', False) else None
    return lead_dict


class RadarD:
  def __init__(self, delay: float = 0.0):
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

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen['modelV2']
    self.current_time = 1e-9*max(sm.logMonoTime.values())

    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

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
    elif self.last_radar_update_time is None or self.current_time - self.last_radar_update_time > RADAR_MEASUREMENT_TIMEOUT:
      self.tracks.clear()

    # *** publish radarState ***
    # Exclude liveTracks from validity check: it arrives at radar rate (8Hz for
    # Bosch) not the 20Hz SubMaster expects, so all_checks() would mark it stale
    # between radar updates and cascade valid=False through the whole pipeline.
    self.radar_state_valid = sm.all_checks(service_list=['modelV2', 'carState'])
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors
    self.radar_state.carStateMonoTime = sm.logMonoTime['carState']

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego
    leads_v3 = sm['modelV2'].leadsV3
    if len(leads_v3) > 1:
      self.radar_state.leadOne = self.lead_one_association.update(self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego)
      self.radar_state.leadTwo = self.lead_two_association.update(self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego)

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)


# fuses camera and radar data for best lead detection
def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  # *** setup messaging
  sm = messaging.SubMaster(['modelV2', 'carState', 'liveTracks'], poll='modelV2')
  pm = messaging.PubMaster(['radarState'])

  RD = RadarD(CP.radarDelay)

  while 1:
    sm.update()

    if sm.updated['modelV2']:
      RD.update(sm, sm['liveTracks'])
      RD.publish(pm)


if __name__ == "__main__":
  main()
