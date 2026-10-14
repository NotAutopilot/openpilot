#!/usr/bin/env python3
import math
from numbers import Number

from cereal import car, log
import cereal.messaging as messaging
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_CTRL, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog

from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.controls.lib.blinker_lateral_pause import BlinkerLateralHold, lat_active_with_blinker_pause
from openpilot.selfdrive.controls.lib.driver_lateral_handoff import (
  PARAM_DRIVER_LAT_HANDOFF, DriverLateralHandoff, apply_lat_authority,
  cs_hands_on_level, cs_real_brake_pressed, handoff_enabled,
  handoff_new_desired_curvature, lat_active_after_handoff,
  pin_desired_curvature_to_measured)
from opendbc.car.tesla.preap.lat_yield import UNDERTRACK_CURVATURE, roundabout_yield_context
from openpilot.selfdrive.controls.lib.cone_line_hold import PARAM_CONE_LINE_HOLD, ConeLineHold, lane_offset_from_model, lookahead_m
from openpilot.selfdrive.controls.lib.lat_low_visibility import (
  PARAM_LOW_VIS_BACKOFF, LowVisibility, fade_curvature, sun_ahead_from_fix)
from openpilot.selfdrive.controls.lib.radar_path_gate import model_path_xy
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.controls.lib.lane_change_nudge import TippedLaneChangeTorque
from openpilot.selfdrive.controls.lib.lane_change_turn import LaneChangeTurnHold, blinker_with_turn_hold
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
from openpilot.selfdrive.controls.lib.lat_turn_geometry import (
  PARAM_TURN_GEOMETRY, is_preap_car, turn_geometry_active,
)
from openpilot.selfdrive.mapd.roundabout import (
  live_map_roundabout_hint, roundabout_lateral_curvature_bias,
)
from openpilot.selfdrive.controls.lib.roundabout_guide import RoundaboutAssist
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.gap_lock import slack_for_guard
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.controls.lib.preap_driver_brake import (
  driver_brake_applied,
  preap_longitudinal_active,
  preap_pedal_long,
  published_long_accel,
)
from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose

State = log.SelfdriveState.OpenpilotState
LaneChangeState = log.LaneChangeState

ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())


class Controls:
  def __init__(self) -> None:
    self.params = Params()
    cloudlog.info("controlsd is waiting for CarParams")
    self.CP = messaging.log_from_bytes(self.params.get("CarParams", block=True), car.CarParams)
    cloudlog.info("controlsd got CarParams")

    self.CI = interfaces[self.CP.carFingerprint](self.CP)

    self.sm = messaging.SubMaster(['liveDelay', 'liveParameters', 'liveTorqueParameters', 'modelV2', 'selfdriveState',
                                   'liveCalibration', 'livePose', 'longitudinalPlan', 'lateralManeuverPlan', 'carState', 'carOutput',
                                   'driverMonitoringState', 'onroadEvents', 'driverAssistance', 'liveMapDataNAP', 'radarState',
                                   'gpsLocation', 'gpsLocationExternal', 'roadCameraState', 'coneLineNAP'], poll='selfdriveState',
                                  ignore_alive=['roadCameraState', 'coneLineNAP'],
                                  ignore_avg_freq=['roadCameraState', 'coneLineNAP'],
                                  ignore_valid=['roadCameraState', 'coneLineNAP'])
    self.pm = messaging.PubMaster(['carControl', 'controlsState'])

    self.steer_limited_by_safety = False
    self._turn_geom_preap = is_preap_car(self.CP)
    self._turn_geom_active = False
    self._turn_geom_param_frame = -1
    # Roundabout Steering Assist (NAPRoundaboutAssist, Pre-AP only, default Off).
    self.rb_assist = RoundaboutAssist(self._turn_geom_preap, self.params)
    self.curvature = 0.0
    self.desired_curvature = 0.0
    self.blinker_lat_hold = BlinkerLateralHold()
    self._lane_change_torque = TippedLaneChangeTorque()
    # Lane change -> driver turn: blinker hold + manual-turn lat pause.
    self._lane_change_turn = LaneChangeTurnHold()
    self._lat_active_prev = False
    # Default On (NAPDriverLatHandoff=1) for Pre-AP. Re-read each cycle so
    # Settings → NAP can turn it Off immediately if gravel/wind misbehave.
    self.lat_handoff = DriverLateralHandoff(
      enabled=handoff_enabled(
        fingerprint=self.CP.carFingerprint,
        param_on=bool(self.params.get_bool(PARAM_DRIVER_LAT_HANDOFF))))
    self._lat_handoff = self.lat_handoff.update(
      engaged=False, lat_would_be_active=False,
      steering_torque=0.0, steering_rate_deg=0.0)
    # napStalkSeq rides the carState subscription above. Do not add another.
    self._nap_stalk_seq = 0
    self._nap_stalk_seen = False
    # Low-visibility fade. Default On. Does not touch longitudinal.
    self.low_vis = LowVisibility()
    self._low_vis = self.low_vis.update(enabled=False)
    self._raw_model_curvature = 0.0
    # Previous cycle's yield context. Card sends the same bits to panda
    # after this loop, so the next hands edge sees what panda already has.
    self._cmd_angle_prev = None
    self._under_prev = False
    self._rb_yield_prev = False
    # Cone-line offset hold. Default On. Identity until a real push.
    self.cone_hold = ConeLineHold()
    self._cone_zone = False
    self._cone_out = self.cone_hold.update(
      enabled=False, engaged=False, cone=None, torque_nm=0.0, measured_k=0.0,
      model_k=0.0, path_x=None, path_y=None, v_ego=0.0, yielded=False,
      lat_active=False, dt=DT_CTRL)

    self.pose_calibrator = PoseCalibrator()
    self.calibrated_pose: Pose | None = None

    self.LoC = LongControl(self.CP)
    self.VM = VehicleModel(self.CP)
    self.LaC: LatControl
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CI, DT_CTRL)

  def update(self):
    self.sm.update(15)
    if self.sm.updated["liveCalibration"]:
      self.pose_calibrator.feed_live_calib(self.sm['liveCalibration'])
    if self.sm.updated["livePose"]:
      device_pose = Pose.from_live_pose(self.sm['livePose'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_pose)

  def _update_low_visibility(self, engaged: bool):
    """Scale lateral authority when the camera or the model cannot see the road.

    Longitudinal is not read or written here. Off (or disengaged) is identity.
    """
    try:
      param_on = bool(self.params.get_bool(PARAM_LOW_VIS_BACKOFF))
    except Exception:
      param_on = True
    model = self.sm['modelV2']
    try:
      lane_probs = [float(p) for p in model.laneLineProbs]
    except Exception:
      lane_probs = []
    try:
      edge_stds = [float(s) for s in model.roadEdgeStds]
    except Exception:
      edge_stds = []
    try:
      path_t = [float(t) for t in model.position.t]
      path_y = [float(y) for y in model.position.yStd]
    except Exception:
      path_t, path_y = [], []
    integ = None
    if self.sm.recv_frame.get('roadCameraState', 0) > 0:
      try:
        integ = int(self.sm['roadCameraState'].integLines)
      except Exception:
        integ = None
    sun_ahead = False
    for key in ('gpsLocationExternal', 'gpsLocation'):
      if self.sm.recv_frame.get(key, 0) <= 0:
        continue
      fix = self.sm[key]
      ahead = sun_ahead_from_fix(
        latitude=fix.latitude, longitude=fix.longitude,
        unix_timestamp_millis=fix.unixTimestampMillis,
        bearing_deg=fix.bearingDeg,
        horizontal_accuracy_m=fix.horizontalAccuracy)
      if ahead is not None:
        sun_ahead = bool(ahead)
        break
    # Same carState subscription controlsd already has. No new socket.
    # Measured curvature is self.curvature (steered). Model curvature is
    # the desired curvature this loop already reads for the lateral target.
    try:
      cs = self.sm['carState']
      v_ego = float(cs.vEgo)
      blinker = bool(cs.leftBlinker or cs.rightBlinker)
    except Exception:
      v_ego = 0.0
      blinker = False
    try:
      if self.sm.valid['lateralManeuverPlan']:
        model_k = float(self.sm['lateralManeuverPlan'].desiredCurvature)
      else:
        model_k = float(model.action.desiredCurvature)
    except Exception:
      model_k = 0.0
    # Same liveMapDataNAP read the lateral-yield roundabout flag uses.
    # In or within 40 m: roundabout_yield_context. No new socket.
    try:
      md = self.sm['liveMapDataNAP'] if self.sm.alive.get('liveMapDataNAP', False) else None
      hint = live_map_roundabout_hint(md)
      near_roundabout = bool(
        hint is not None and roundabout_yield_context(
          hint.on_roundabout, hint.approaching, hint.distance_m))
    except Exception:
      near_roundabout = False
    return self.low_vis.update(
      enabled=bool(engaged) and param_on,
      lane_probs=lane_probs, edge_stds=edge_stds,
      path_t=path_t, path_y_std=path_y, integ_lines=integ,
      sun_ahead=sun_ahead, dt=DT_CTRL,
      v_ego=v_ego, blinker=blinker,
      yielded=bool(self._lat_handoff.yielded),
      measured_curvature=float(self.curvature),
      model_curvature=model_k,
      near_roundabout=near_roundabout)

  def _cone_curvature(self, model_k: float, CS) -> float:
    """Shift the lateral target onto the driver's line after a cone-line push.

    Returns model_k unchanged until that push has already yielded lateral.
    Does not read or write longitudinal.
    """
    try:
      enabled = bool(self.params.get_bool(PARAM_CONE_LINE_HOLD))
    except Exception:
      enabled = True
    cone = None
    if self.sm.seen.get("coneLineNAP", False) and self.sm.alive.get("coneLineNAP", False):
      cone = self.sm["coneLineNAP"]
    cone_active = bool(getattr(cone, "active", False)) if cone is not None else False
    cone_releasing = bool(getattr(cone, "releasing", False)) if cone is not None else False
    # Inside a cone corridor the low-visibility fade must not undo the cone shift.
    self._cone_zone = cone_active or cone_releasing or self.cone_hold.busy
    path_x = path_y = None
    lane_offset = lane_centre = None
    cone_shift = 0.0
    if self._cone_zone:
      path_x, path_y = model_path_xy(self.sm["modelV2"])
      try:
        lane_offset, lane_centre = lane_offset_from_model(self.sm["modelV2"], lookahead_m(float(CS.vEgo)))
      except Exception:
        lane_offset = lane_centre = None
      try:
        cone_shift = float(getattr(cone, "wouldSteer", 0.0)) if cone is not None else 0.0
      except (TypeError, ValueError):
        cone_shift = 0.0
    self._cone_out = self.cone_hold.update(
      enabled=enabled,
      engaged=bool(self.sm["selfdriveState"].enabled),
      cone=cone,
      torque_nm=float(CS.steeringTorque),
      measured_k=float(self.curvature),
      model_k=float(model_k),
      path_x=path_x,
      path_y=path_y,
      v_ego=float(CS.vEgo),
      yielded=bool(self._lat_handoff.yielded),
      lat_active=bool(self._lat_active_prev),
      dt=DT_CTRL,
      lane_offset=lane_offset,
      lane_center_la=lane_centre,
      cone_shift=cone_shift,
    )
    return float(self._cone_out.curvature)

  def state_control(self):
    CS = self.sm['carState']

    # Update VehicleModel
    lp = self.sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    self.curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)

    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['liveTorqueParameters']
      if self.sm.all_checks(['liveTorqueParameters']) and torque_params.useParams:
        self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']

    CC = car.CarControl.new_message()
    CC.enabled = self.sm['selfdriveState'].enabled

    # Check which actuators can be enabled
    standstill = abs(CS.vEgo) <= max(self.CP.minSteerSpeed, 0.3) or CS.standstill
    # Driver-turn lamps latch through flash gaps. Soft-lat Off still pauses
    # lat on that latch; Soft-lat On keeps latActive and lets handoff
    # inhibit re-enable. ALC keep-alive flashes must not latch a driver
    # turn: gate while laneChangeState != off, and on a stalk tip
    # (LEFT/RIGHT then IDLE within 0.40s) so latActive stays up for
    # DesireHelper. A held stalk past that window is a turn at any speed
    # when ALC is not latched. During ALC / leftover keep-alive,
    # same-direction stalk must stay on >1.0s to steal ALC as a turn.
    alc_active = model_v2.meta.laneChangeState != LaneChangeState.off
    # Tipped ALC stays armed through lamp flash-dark gaps. Same-direction
    # torque is the confirm, not a soft-lat release. An emergency yank
    # still frees the EPS immediately.
    lc_state = model_v2.meta.laneChangeState
    tipped_alc = lc_state in (
      LaneChangeState.preLaneChange,
      LaneChangeState.laneChangeStarting,
      LaneChangeState.laneChangeFinishing,
    )
    if model_v2.meta.laneChangeDirection == log.LaneChangeDirection.left:
      nudge_dir = 1
    elif model_v2.meta.laneChangeDirection == log.LaneChangeDirection.right:
      nudge_dir = 2
    else:
      nudge_dir = 0
    # Emergency (spike / opposite) or a sustained same-direction takeover
    # (held until the hands come off) yields lateral now; only the
    # emergency also disengages (car_specific steerDisengage).
    # Resting EPAS offset (~+0.25 Nm on this car). Relative torque keeps a
    # hands-off reading from looking like a left-lane nudge.
    driver_tq = float(CS.steeringTorque) - float(self.lat_handoff.rest_bias)
    self._lane_change_torque.update(
      torque_nm=driver_tq, hands_on_level=cs_hands_on_level(CS),
      direction=nudge_dir, tipped=tipped_alc, dt=DT_CTRL)
    lane_change_confirm = self._lane_change_torque.confirm
    # Turning the wheel well past a lane change in its direction ends the
    # change (DesireHelper) and becomes a manual driver turn here: keep the
    # blinker until the turn completes, pause lat like a latched stalk.
    turn_hold_dir = self._lane_change_turn.update(
      lc_state=lc_state, lc_direction=nudge_dir, steering_angle_deg=float(CS.steeringAngleDeg),
      torque_nm=driver_tq, lat_active=self._lat_active_prev,
      v_ego=float(CS.vEgo), stalk_state=getattr(CS, 'turnSignalStalkState', 0),
      engaged=bool(CC.enabled), dt=DT_CTRL)
    # Soft yield frees the EPS the same way blinker pause does (latActive
    # false → DAS_steeringControlType=0). Do not keep latActive and track
    # measured angle — that is follow-the-rim holding, not a free wheel.
    # Handoff still sees the pre-yield bit so it does not reset itself.
    # Longitudinal / enabled are untouched. ALC is not this path.
    # Soft-lat On: lamp latch must not clear latActive. Soft-lat Off:
    # keep today's blinker lat-pause so a held turn still frees the wheel.
    self.lat_handoff.enabled = handoff_enabled(
      fingerprint=self.CP.carFingerprint,
      param_on=bool(self.params.get_bool(PARAM_DRIVER_LAT_HANDOFF)))
    lat_would_be_active = lat_active_with_blinker_pause(
      active=self.sm['selfdriveState'].active,
      steer_fault_temporary=CS.steerFaultTemporary,
      steer_fault_permanent=CS.steerFaultPermanent,
      standstill=standstill,
      steer_at_standstill=self.CP.steerAtStandstill,
      left_blinker=CS.leftBlinker,
      right_blinker=CS.rightBlinker,
      steering_pressed=CS.steeringPressed,
      steering_disengage=bool(getattr(CS, 'steeringDisengage', False)),
      engaged=bool(self.sm['selfdriveState'].enabled),
      hold=self.blinker_lat_hold,
      alc_active=alc_active,
      v_ego=CS.vEgo,
      stalk_state=getattr(CS, 'turnSignalStalkState', 0),
      soft_lat_on=self.lat_handoff.enabled,
      driver_turn=self._lane_change_turn.turning,
    )
    # Pre-AP: brake and a long pause keep lateral, but must not log longActive
    # or a decel the interceptor is not executing. Other cars keep the stock
    # formula (enabled, no longitudinal override, openpilot longitudinal).
    preap_pedal = preap_pedal_long(self.CP)
    CC.longActive = preap_longitudinal_active(
      enabled=CC.enabled,
      longitudinal_override=any(e.overrideLongitudinal for e in self.sm['onroadEvents']),
      openpilot_longitudinal=self.CP.openpilotLongitudinalControl,
      preap_pedal=preap_pedal,
      enable_long_control=bool(getattr(CS, "enableLongControl", False)),
      driver_brake=driver_brake_applied(CS),
    )
    # turn_active is the latched driver-turn blinker (not ALC, not the
    # post-turn hand-on hold). Soft-lat ORs that with v_ego < 10 mph
    # as re-enable inhibit + falling-edge enter-yield. Soft-lat Off
    # ignores it (identity).
    # In-session stalk pull: clear a lateral yield. Seq lives on carState.
    stalk_seq = int(getattr(CS, "napStalkSeq", 0) or 0) & 0xFF
    if self._nap_stalk_seen and stalk_seq != self._nap_stalk_seq:
      self.lat_handoff.driver_resume_request()
    self._nap_stalk_seen = True
    self._nap_stalk_seq = stalk_seq
    self._lat_handoff = self.lat_handoff.update(
      engaged=bool(CC.enabled),
      lat_would_be_active=bool(lat_would_be_active),
      steering_torque=float(CS.steeringTorque),
      steering_rate_deg=float(CS.steeringRateDeg),
      alc_active=alc_active,
      blinker_paused=bool(self.blinker_lat_hold.turn_active),
      hands_on_level=cs_hands_on_level(CS),
      tracking_error=float(self.desired_curvature - self.curvature),
      brake_applied=cs_real_brake_pressed(CS),
      a_ego=float(CS.aEgo),
      v_ego=float(CS.vEgo),
      emergency_yank=bool(self._lane_change_torque.release),
      lane_change_confirm=bool(lane_change_confirm),
      commanded_angle_deg=self._cmd_angle_prev,
      roundabout_yield=bool(self._rb_yield_prev),
      undertrack=bool(self._under_prev),
      steering_angle_deg=float(CS.steeringAngleDeg),
      steering_pressed=bool(CS.steeringPressed),
      model_curvature=float(self._raw_model_curvature),
      measured_curvature=float(self.curvature),
    )
    self._low_vis = self._update_low_visibility(bool(CC.enabled))
    CC.latActive = lat_active_after_handoff(
      lat_would_be_active, self._lat_handoff.yielded)
    self._lat_active_prev = bool(CC.latActive)

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Keep the indicator flashing while ALC is armed or in progress. Pre-AP
    # carcontroller TXes DAS_bodyControls from CC.leftBlinker / rightBlinker.
    CC.leftBlinker, CC.rightBlinker = blinker_with_turn_hold(
      DesireHelper.lane_change_keep_blinker(
        model_v2.meta.laneChangeState, model_v2.meta.laneChangeDirection),
      turn_hold_dir)

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset()

    # accel PID loop
    pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)
    lead_v_rel = lead_d_rel = lead_slack = lead_a_lead = None
    if self.sm.valid['radarState'] and self.sm['radarState'].leadOne.status:
      lead = self.sm['radarState'].leadOne
      lead_d_rel = float(lead.dRel)
      lead_v_rel = float(CS.vEgo) - float(lead.vLead)
      gap_lock_m = getattr(long_plan, "gapLockM", 0.0)
      lead_slack = slack_for_guard(lead_d_rel, float(lead.vLead), float(long_plan.tFollow), gap_lock_m)
      lead_a_lead = float(lead.aLeadK)
    actuators.accel = published_long_accel(
      long_active=bool(CC.longActive),
      preap_pedal=preap_pedal,
      loc_accel=self.LoC.update(
        CC.longActive, CS, long_plan.aTarget, long_plan.shouldStop, pid_accel_limits,
        lead_v_rel=lead_v_rel, lead_d_rel=lead_d_rel, lead_slack=lead_slack,
        lead_fcw=bool(long_plan.fcw),
        lead_v_ego=float(CS.vEgo), lead_v_cruise=float(CS.vCruise) * CV.KPH_TO_MS,
        lead_a_lead=lead_a_lead,
      ),
    )

    # Steering PID loop and lateral MPC
    # Reset desired curvature to current to avoid violating the limits on engage.
    # Yield (and soft-lat Off blinker pause) clear latActive (EPS free).
    # Pin to the wheel while lat is down so resume clips from there.
    # Soft-lat On does not drop lat on lamp latch; after a yielded turn
    # the falling blinker enters yield and the normal 0.15 s confirm +
    # 1 s blend owns take-back. Hands still on stay yielded.
    if self.sm.valid['lateralManeuverPlan']:
      model_or_plan_curvature = self.sm['lateralManeuverPlan'].desiredCurvature
    else:
      model_or_plan_curvature = model_v2.action.desiredCurvature
    rb_hint = live_map_roundabout_hint(self.sm['liveMapDataNAP'] if self.sm.valid.get('liveMapDataNAP', False) else None)
    is_rhd = bool(self.sm['driverMonitoringState'].isRHD) if self.sm.valid.get('driverMonitoringState', False) else False
    # Turn geometry ON: no roundabout outer bias (modeld owns turn-in timing).
    if self._turn_geom_param_frame < 0 or self._turn_geom_param_frame >= 100:
      self._turn_geom_param_frame = 0
      self._turn_geom_active = turn_geometry_active(
        self._turn_geom_preap, bool(self.params.get_bool(PARAM_TURN_GEOMETRY)))
    self._turn_geom_param_frame += 1
    # Map-guided ring curvature blend (identity unless the toggle is On near a mapped ring).
    # The hint is read by alive, not valid: liveMapDataNAP is invalid without a speed match.
    rb_assist_hint = rb_hint
    if rb_assist_hint is None and self.sm.alive.get('liveMapDataNAP', False):
      rb_assist_hint = live_map_roundabout_hint(self.sm['liveMapDataNAP'])
    yaw_rate = float(self.calibrated_pose.angular_velocity.z) if self.calibrated_pose is not None else 0.0
    model_or_plan_curvature = self.rb_assist.update(
      self.sm, t=float(self.sm.logMonoTime['carState']) * 1e-9, v_ego=float(CS.vEgo), yaw_rate=yaw_rate,
      model_k=float(model_or_plan_curvature), lat_active=bool(CC.latActive) and not bool(self._lat_handoff.yielded),
      maneuver_active=bool(self.sm.valid['lateralManeuverPlan']), lane_change_active=bool(alc_active),
      hint=rb_assist_hint, model_v2=model_v2, stalk_state=int(getattr(CS, 'turnSignalStalkState', 0) or 0),
      steering_pressed=bool(CS.steeringPressed), steering_torque=float(CS.steeringTorque))
    rb_bias = roundabout_lateral_curvature_bias(
      rb_hint, is_rhd=is_rhd, turn_geometry_active=self._turn_geom_active,
    )
    # The legacy outer bias never stacks on the assist.
    model_or_plan_curvature = float(model_or_plan_curvature) + (0.0 if self.rb_assist.active else rb_bias)
    # Raw model curvature feeds the fight-hold resume (not the faded command).
    self._raw_model_curvature = float(model_or_plan_curvature)
    # Cone-line hold, after a real push, aims the re-take at the driver's
    # line. Identity otherwise. Longitudinal accel above is unchanged.
    model_or_plan_curvature = self._cone_curvature(float(model_or_plan_curvature), CS)
    # Low visibility eases the lateral target toward the wheel. Longitudinal
    # accel above is unchanged, and latActive / enabled are unchanged.
    # Not inside a cone corridor (Oct 14 seg 25: the alert was on while the cones closed in).
    low_vis_auth = 1.0 if self._cone_zone else float(self._low_vis.authority)
    model_or_plan_curvature = fade_curvature(
      model_or_plan_curvature, self.curvature, low_vis_auth)
    new_desired_curvature = handoff_new_desired_curvature(
      yielded=bool(self._lat_handoff.yielded),
      lat_active=bool(CC.latActive),
      model_curvature=model_or_plan_curvature,
      measured_curvature=self.curvature,
    )
    if pin_desired_curvature_to_measured(
        self._lat_handoff.yielded, bool(CC.latActive)):
      self.desired_curvature = self.curvature
      curvature_limited = False
    else:
      self.desired_curvature, curvature_limited = clip_curvature(
        CS.vEgo, self.desired_curvature, new_desired_curvature, lp.roll)
    lat_delay = self.sm["liveDelay"].lateralDelay + LAT_SMOOTH_SECONDS

    actuators.curvature = self.desired_curvature
    steer, steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                       self.steer_limited_by_safety, self.desired_curvature,
                                                       curvature_limited, lat_delay)
    # Blend only while lat is actually requesting and authority < 1.
    # While yielded, latActive is false — do not follow-measured-angle.
    # Do not write blended values back into desired_curvature.
    if CC.latActive and self._lat_handoff.authority < 1.0:
      steer, steeringAngleDeg, blended_curvature = apply_lat_authority(
        self._lat_handoff.authority, steer, steeringAngleDeg, CS.steeringAngleDeg,
        self.desired_curvature, self.curvature)
      actuators.torque = float(steer)
      actuators.steeringAngleDeg = float(steeringAngleDeg)
      actuators.curvature = float(blended_curvature)
    else:
      actuators.torque = float(steer)
      actuators.steeringAngleDeg = float(steeringAngleDeg)
    if low_vis_auth < 1.0:
      actuators.torque = float(actuators.torque) * low_vis_auth
    # Latch what this cycle will tell panda (0x561) for the next hands edge.
    self._cmd_angle_prev = float(actuators.steeringAngleDeg)
    self._under_prev = abs(float(actuators.curvature) - float(self.curvature)) > UNDERTRACK_CURVATURE
    _md = self.sm['liveMapDataNAP'] if self.sm.alive.get('liveMapDataNAP', False) else None
    _hint = live_map_roundabout_hint(_md)
    self._rb_yield_prev = bool(
      _hint is not None and roundabout_yield_context(
        _hint.on_roundabout, _hint.approaching, _hint.distance_m))
    # Ensure no NaNs/Infs
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, Number):
        continue

      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)

    return CC, lac_log

  def publish(self, CC, lac_log):
    CS = self.sm['carState']

    # Orientation and angle rates can be useful for carcontroller
    # Only calibrated (car) frame is relevant for the carcontroller
    CC.currentCurvature = self.curvature
    if self.calibrated_pose is not None:
      CC.orientationNED = self.calibrated_pose.orientation.xyz.tolist()
      CC.angularVelocity = self.calibrated_pose.angular_velocity.xyz.tolist()

    CC.cruiseControl.override = CC.enabled and not CC.longActive and self.CP.openpilotLongitudinalControl
    # Soft-lat emergency hard-brake: request the normal cancel path (stock
    # CC spoof / session teardown). Distinct from silent long pause.
    emergency_cancel = bool(
      self.lat_handoff.enabled and self._lat_handoff.emergency_cancel)
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (
      not CC.enabled or not self.CP.pcmCruise or emergency_cancel)
    CC.cruiseControl.resume = CC.enabled and CS.cruiseState.standstill and not self.sm['longitudinalPlan'].shouldStop

    hudControl = CC.hudControl
    hudControl.setSpeed = float(CS.vCruiseCluster * CV.KPH_TO_MS)
    hudControl.speedVisible = CC.enabled
    hudControl.lanesVisible = CC.enabled
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead
    hudControl.leadDistanceBars = self.sm['selfdriveState'].personality.raw + 1
    hudControl.visualAlert = self.sm['selfdriveState'].alertHudVisual

    hudControl.rightLaneVisible = True
    hudControl.leftLaneVisible = True
    if self.sm.valid['driverAssistance']:
      hudControl.leftLaneDepart = self.sm['driverAssistance'].leftLaneDeparture
      hudControl.rightLaneDepart = self.sm['driverAssistance'].rightLaneDeparture

    if self.sm['selfdriveState'].active:
      CO = self.sm['carOutput']
      if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
        self.steer_limited_by_safety = abs(CC.actuators.steeringAngleDeg - CO.actuatorsOutput.steeringAngleDeg) > \
                                              STEER_ANGLE_SATURATION_THRESHOLD
      else:
        self.steer_limited_by_safety = abs(CC.actuators.torque - CO.actuatorsOutput.torque) > 1e-2

    # TODO: both controlsState and carControl valids should be set by
    #       sm.all_checks(), but this creates a circular dependency

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    cs = dat.controlsState

    cs.curvature = self.curvature
    cs.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    cs.lateralPlanMonoTime = self.sm.logMonoTime['modelV2']
    cs.desiredCurvature = self.desired_curvature
    if self.lat_handoff.enabled:
      cs.latAuthority = float(self._lat_handoff.authority)
      cs.latHandoffPaused = bool(self._lat_handoff.ui_paused)
    cs.lowVisibility = bool(self._low_vis.alert)
    cs.coneLineHold = bool(self._cone_out.active)
    cs.coneLineOffset = float(self._cone_out.offset_m)
    cs.longControlState = self.LoC.long_control_state
    cs.upAccelCmd = float(self.LoC.pid.p)
    cs.uiAccelCmd = float(self.LoC.pid.i)
    cs.ufAccelCmd = float(self.LoC.pid.f)
    cs.forceDecel = bool((self.sm['driverMonitoringState'].alertLevel == log.DriverMonitoringState.AlertLevel.three) or
                         (self.sm['selfdriveState'].state == State.softDisabling))

    lat_tuning = self.CP.lateralTuning.which()
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      cs.lateralControlState.angleState = lac_log
    elif lat_tuning == 'pid':
      cs.lateralControlState.pidState = lac_log
    elif lat_tuning == 'torque':
      cs.lateralControlState.torqueState = lac_log

    self.pm.send('controlsState', dat)

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

  def run(self):
    rk = Ratekeeper(100, print_delay_threshold=None)
    while True:
      self.update()
      CC, lac_log = self.state_control()
      self.publish(CC, lac_log)
      rk.monitor_time()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  controls = Controls()
  controls.run()


if __name__ == "__main__":
  main()
