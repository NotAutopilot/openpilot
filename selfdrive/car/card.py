#!/usr/bin/env python3
import json
import os
import time
import threading

import cereal.messaging as messaging

from cereal import car, log

from opendbc.car.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog, ForwardingHandler

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData, CanRecvCallable, CanSendCallable
from opendbc.car.carlog import carlog
from opendbc.car.fw_versions import ObdCallback
from opendbc.car.car_helpers import get_car, interfaces
from opendbc.car.interfaces import CarInterfaceBase, RadarInterfaceBase
from openpilot.selfdrive.pandad import can_capnp_to_list, can_list_to_can_capnp
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET, VCruiseHelper
from openpilot.selfdrive.mapd.constants import MANUAL_SET_EPS_KPH, map_accel_a_ms2
from openpilot.selfdrive.mapd.map_speed_policy import (
  MapCruiseHold, apply_map_speed_kph, decide_map_cruise, effective_map_limit_ms,
  map_slew_a_ms2, read_map_speed_params, should_write_preap_pedal, slew_map_speed_ms,
)
from openpilot.selfdrive.mapd.roundabout import live_map_roundabout_hint, roundabout_ease_v_ms, roundabout_target_ms
from openpilot.selfdrive.controls.lib.curve_follow import MODE_ACTIVE, MODE_SHADOW, read_curve_follow_mode
from openpilot.selfdrive.controls.lib.curve_max_hold import CurveMaxHold
from openpilot.selfdrive.controls.lib.radar_status_log import RadarStatusLogger
from openpilot.selfdrive.controls.lib.follow_distance import published_cruise_ms
from openpilot.selfdrive.controls.lib.hypermile import (
  FollowStalkGesture, button_event_closer, button_event_released,
  map_target_offset_kph, persist_follow_distance, read_hypermile_params,
  read_hypermile_step_down,
)
from openpilot.selfdrive.car.tesla.preap_force_offroad_handoff import (
  install_force_offroad_handoff,
  update_force_offroad_handoff,
)

REPLAY = "REPLAY" in os.environ

EventName = log.OnroadEvent.EventName


def map_slew_from_displayed_kph(displayed_kph: float, offset_kph: float) -> float:
  """Slew state tracks the raw OSM limit. HUD / pedal MAX already includes offset."""
  return (float(displayed_kph) - float(offset_kph)) * CV.KPH_TO_MS


def _kph_near(a, b, eps: float = 1.0) -> bool:
  if a is None or b is None:
    return False
  try:
    return abs(float(a) - float(b)) <= eps
  except (TypeError, ValueError):
    return False


def _snapshot_cruise_hold(hold) -> dict:
  return {
    "sticky_set_kph": hold.sticky_set_kph,
    "held_max_kph": hold.held_max_kph,
    "policy_kph": hold.policy_kph,
    "last_raw": hold.last_raw_kph,
  }


def _refresh_ring_snapshot(pre: dict, hold, ring_kph: float) -> None:
  """Posted / stalk changes that are not the ring speed become the resume target."""
  if pre is None or _kph_near(hold.held_max_kph, ring_kph):
    return
  pre["held_max_kph"] = hold.held_max_kph
  pre["policy_kph"] = hold.policy_kph
  pre["sticky_set_kph"] = hold.sticky_set_kph
  pre["last_raw"] = hold.last_raw_kph


def _unwind_ring_speed_set(hold, pre, ring_kph: float) -> bool:
  """Drop a MAX that was stored as the ring speed. Keep a pre-ring set.

  Returns True when a ring-speed latch was removed. Gap lock is a lead
  setpoint and is not touched; MAX stays the cruise ceiling.
  """
  if pre is None:
    return False
  changed = False
  for field in ("sticky_set_kph", "held_max_kph", "policy_kph"):
    cur = getattr(hold, field)
    old = pre.get(field)
    if _kph_near(cur, ring_kph) and not _kph_near(old, ring_kph):
      setattr(hold, field, old)
      changed = True
  if changed and _kph_near(hold.last_raw_kph, ring_kph):
    old_raw = pre.get("last_raw")
    if old_raw is not None:
      hold.last_raw_kph = old_raw
  return changed


def _ring_exit_target_ms(hold, lim, posted_kph, detect_v_ms: float | None = None) -> float | None:
  """Posted limit, else the pre-ring MAX. None when nothing is known.

  MAX is a hard ceiling: a driver-set pre-ring MAX (stalk sticky / held)
  is restored exactly, even if the car was faster when the ring was first
  detected. Only a map-auto MAX (policy_kph, no driver set) with no posted
  limit now may rise to the detection speed, since that MAX was never a
  driver choice and the map no longer has a limit to restore toward.
  """
  if hold.sticky_set_kph is None and lim is not None and float(lim) > 0.0:
    return float(lim)
  for val in (hold.sticky_set_kph, hold.held_max_kph):
    if val is not None and float(val) > 0.0:
      return float(val) * CV.KPH_TO_MS
  if hold.policy_kph is not None and float(hold.policy_kph) > 0.0:
    pol = float(hold.policy_kph) * CV.KPH_TO_MS
    if detect_v_ms is not None and float(detect_v_ms) > pol:
      return float(detect_v_ms)
    return pol
  if posted_kph is not None and float(posted_kph) > 0.0:
    return float(posted_kph) * CV.KPH_TO_MS
  return None


def _apply_ring_exit_max(target_ms, floor_ms, lookahead, accel_level):
  """Start the ramp at least at current speed, then ease toward the target.

  One map-rate step, so a ring exit does not publish the ring speed and
  does not step MAX down in a single frame.
  """
  start = float(floor_ms)
  if target_ms is None or float(target_ms) <= 0.0:
    return start * CV.MS_TO_KPH, start
  a = map_slew_a_ms2(start, float(target_ms), lookahead, accel_level)
  nxt = slew_map_speed_ms(start, float(target_ms), DT_CTRL, a)
  return nxt * CV.MS_TO_KPH, nxt

# forward
carlog.addHandler(ForwardingHandler(cloudlog))


def log_curve_max(owner, curve_out, hud_in_kph, posted_kph, CS) -> None:
  """2 Hz `curvemax` line (errorLogMessage -> qlog) while the MAX cap is in play. Never touches MAX."""
  try:
    owner._curve_log_n = getattr(owner, "_curve_log_n", 0) + 1
    cap = getattr(owner._curve_max, "_cap_kph", None)
    if owner._curve_log_n % 50 != 0 or not (curve_out.active or cap is not None):
      return
    rec = {
      "m": int(getattr(owner, "_curve_follow_mode", MODE_SHADOW)), "act": int(bool(curve_out.active)),
      "cap": None if cap is None else round(float(cap), 1), "hud_in": round(float(hud_in_kph), 1),
      "hud": round(float(curve_out.hud_kph), 1),
      "seed": None if curve_out.restore_seed_kph is None else round(float(curve_out.restore_seed_kph), 1),
      "frz": int(bool(curve_out.freeze_posted)), "posted": None if posted_kph is None else round(float(posted_kph), 1),
      "v": round(float(CS.vEgo), 2),
    }
    cloudlog.error("curvemax " + json.dumps(rec, separators=(",", ":")))
  except Exception:
    pass


def obd_callback(params: Params) -> ObdCallback:
  def set_obd_multiplexing(obd_multiplexing: bool):
    if params.get_bool("ObdMultiplexingEnabled") != obd_multiplexing:
      cloudlog.warning(f"Setting OBD multiplexing to {obd_multiplexing}")
      params.remove("ObdMultiplexingChanged")
      params.put_bool("ObdMultiplexingEnabled", obd_multiplexing, block=True)
      params.get_bool("ObdMultiplexingChanged", block=True)
      cloudlog.warning("OBD multiplexing set successfully")
  return set_obd_multiplexing


def can_comm_callbacks(logcan: messaging.SubSocket, sendcan: messaging.PubSocket) -> tuple[CanRecvCallable, CanSendCallable]:
  def can_recv(wait_for_one: bool = False) -> list[list[CanData]]:
    """
    wait_for_one: wait the normal logcan socket timeout for a CAN packet, may return empty list if nothing comes

    Returns: CAN packets comprised of CanData objects for easy access
    """
    ret = []
    for can in messaging.drain_sock(logcan, wait_for_one=wait_for_one):
      ret.append([CanData(msg.address, msg.dat, msg.src) for msg in can.can])
    return ret

  def can_send(msgs: list[CanData]) -> None:
    sendcan.send(can_list_to_can_capnp(msgs, msgtype='sendcan'))

  return can_recv, can_send


class Car:
  CI: CarInterfaceBase
  RI: RadarInterfaceBase
  CP: car.CarParams

  def __init__(self, CI=None, RI=None) -> None:
    self.can_sock = messaging.sub_sock('can', timeout=20)
    # Curve MAX reads vehicle-model curvature off carControl (already
    # polled here). Do not subscribe to controlsState: that 100 Hz socket
    # pinned card/controlsd/selfdrived and dropped controlsd below 100 Hz.
    self.sm = messaging.SubMaster(['pandaStates', 'carControl', 'onroadEvents', 'liveMapDataNAP', 'radarState',
                                   'livePose'])
    self.pm = messaging.PubMaster(['sendcan', 'carState', 'carParams', 'carOutput', 'liveTracks'])

    self.can_rcv_cum_timeout_counter = 0

    self.CC_prev = car.CarControl.new_message()
    self.CS_prev = car.CarState.new_message()
    self.initialized_prev = False

    self.last_actuators_output = structs.CarControl.Actuators()

    self.params = Params()

    self.can_callbacks = can_comm_callbacks(self.can_sock, self.pm.sock['sendcan'])

    is_release = self.params.get_bool("IsReleaseBranch")

    if CI is None:
      # wait for one pandaState and one CAN packet
      print("Waiting for CAN messages...")
      while True:
        can = messaging.recv_one_retry(self.can_sock)
        if len(can.can) > 0:
          break

      alpha_long_allowed = self.params.get_bool("AlphaLongitudinalEnabled")

      cached_params = None
      cached_params_raw = self.params.get("CarParamsCache")
      if cached_params_raw is not None:
        with car.CarParams.from_bytes(cached_params_raw) as _cached_params:
          cached_params = _cached_params

      self.CI = get_car(*self.can_callbacks, obd_callback(self.params), alpha_long_allowed, is_release, cached_params)
      self.RI = interfaces[self.CI.CP.carFingerprint].RadarInterface(self.CI.CP)
      self.CP = self.CI.CP

      # continue onto next fingerprinting step in pandad
      self.params.put_bool("FirmwareQueryDone", True, block=True)
    else:
      self.CI, self.CP = CI, CI.CP
      self.RI = RI

    self.CP.alternativeExperience = 0
    openpilot_enabled_toggle = self.params.get_bool("OpenpilotEnabledToggle")
    controller_available = self.CI.CC is not None and openpilot_enabled_toggle and not self.CP.dashcamOnly
    self.CP.passive = not controller_available or self.CP.dashcamOnly
    if self.CP.passive:
      safety_config = structs.CarParams.SafetyConfig()
      safety_config.safetyModel = structs.CarParams.SafetyModel.noOutput
      self.CP.safetyConfigs = [safety_config]

    if self.CP.secOcRequired:
      # Copy user key if available
      try:
        with open("/cache/params/SecOCKey") as f:
          user_key = f.readline().strip()
          if len(user_key) == 32:
            self.params.put("SecOCKey", user_key, block=True)
      except Exception:
        pass

      secoc_key = self.params.get("SecOCKey")
      if secoc_key is not None:
        saved_secoc_key = bytes.fromhex(secoc_key.strip())
        if len(saved_secoc_key) == 16:
          self.CP.secOcKeyAvailable = True
          self.CI.CS.secoc_key = saved_secoc_key
          if controller_available:
            self.CI.CC.secoc_key = saved_secoc_key
        else:
          cloudlog.warning("Saved SecOC key is invalid")

    # Write previous route's CarParams
    prev_cp = self.params.get("CarParamsPersistent")
    if prev_cp is not None:
      self.params.put("CarParamsPrevRoute", prev_cp, block=True)

    # Write CarParams for controls and radard
    cp_bytes = self.CP.to_bytes()
    self.params.put("CarParams", cp_bytes, block=True)
    self.params.put("CarParamsCache", cp_bytes)
    self.params.put("CarParamsPersistent", cp_bytes)

    self.v_cruise_helper = VCruiseHelper(self.CP)
    self._map_hold = MapCruiseHold()
    self._curve_max = CurveMaxHold()
    self._map_slew_ms: float | None = None
    self._last_pedal_kph: float | None = None
    self._pedal_self_write_kph: float | None = None
    self._follow_stalk_mono: float = 0.0
    self._follow_gesture = FollowStalkGesture()
    self._refresh_map_speed_params()

    self.is_metric = self.params.get_bool("IsMetric")
    self.experimental_mode = self.params.get_bool("ExperimentalMode")

    # card is driven by can recv, expected at 100Hz
    self.rk = Ratekeeper(100, print_delay_threshold=None)

    self._can_packets: list[CanData] = []
    self.radar_donor_vin = None
    tesla_preap = any(cfg.safetyModel == car.CarParams.SafetyModel.teslaPreap for cfg in self.CP.safetyConfigs)
    self._tesla_preap = tesla_preap
    self._nap_orphan_s = 0.0
    # 2 Hz radarstat line (radar status bits + track summary) for Pre-AP Bosch digs.
    self._radar_stat = RadarStatusLogger() if tesla_preap else None
    if tesla_preap:
      from openpilot.selfdrive.car.tesla.preap_blinker_lat_pause import install_blinker_lat_pause
      from openpilot.selfdrive.car.tesla.preap_body_controls import install_body_controls_test
      from opendbc.car.tesla.preap.nap_conf import nap_conf
      from opendbc.car.tesla.preap.radar_donor_vin import RadarDonorVinCommissioner

      from openpilot.selfdrive.controls.lib.lead_approach import install_preap_plant_regen_guard

      install_blinker_lat_pause()
      install_body_controls_test()
      install_force_offroad_handoff()
      install_preap_plant_regen_guard()

      def store_donor_vin(vin: str) -> None:
        nap_conf.radar_donor_vin = vin
        self.params.put("NAPRadarVinReadStatus", f"saved {vin}")
        cloudlog.info("preap radar donor vin stored")

      self._nap_conf = nap_conf
      self.radar_donor_vin = RadarDonorVinCommissioner(store_donor_vin)

  def state_update(self) -> tuple[car.CarState, structs.RadarDataT | None]:
    """carState update loop, driven by can"""

    can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
    can_list = can_capnp_to_list(can_strs)
    self._can_packets = [CanData(addr, dat, src) for _ts, frames in can_list for addr, dat, src in frames]

    # Update carState from CAN
    CS = self.CI.update(can_list)

    # Pre-AP Force Offroad stock-CC handoff (or immediate ready if not
    # software-long). Must run while onroad so CANCEL/SET can still TX.
    update_force_offroad_handoff(
      getattr(self.CI, "CS", None) if getattr(self, "_tesla_preap", False) else None,
      CS,
    )

    # Update radar tracks from CAN
    RD: structs.RadarDataT | None = self.RI.update(can_list)
    radar_stat = getattr(self, "_radar_stat", None)
    if radar_stat is not None:
      stat_line = radar_stat.update(self._can_packets, getattr(RD, "points", None), CS.vEgo)
      if stat_line is not None:
        cloudlog.error(stat_line)

    self.sm.update(0)

    can_rcv_valid = len(can_strs) > 0

    # Check for CAN timeout
    if not can_rcv_valid:
      self.can_rcv_cum_timeout_counter += 1

    if can_rcv_valid and REPLAY:
      self.can_log_mono_time = messaging.log_from_bytes(can_strs[0]).logMonoTime

    if getattr(self, "_tesla_preap", False):
      # Cruise latched on the car side, openpilot not actually engaged
      # (startup, noEntry, missed rising edge). Quiet reset after 0.75 s.
      from openpilot.selfdrive.car.tesla.preap_blinker_lat_pause import reconcile_orphan_session
      self._nap_orphan_s, _canceled = reconcile_orphan_session(
        getattr(getattr(self.CI, "CS", None), "engagement", None),
        op_enabled=bool(self.sm['carControl'].enabled),
        orphan_s=float(getattr(self, "_nap_orphan_s", 0.0)),
        dt=DT_CTRL,
        cereal_cs=CS,
        interface_cs=getattr(self.CI, "CS", None),
      )

    try:
      preap_software_cruise = (
        self.CP.brand == "tesla"
        and self.CP.carFingerprint == "TESLA_MODEL_S_PREAP"
        and self.CP.openpilotLongitudinalControl
        and not self.CP.pcmCruise
      )

      if not preap_software_cruise:
        self.v_cruise_helper.update_v_cruise(CS, self.sm['carControl'].enabled, self.is_metric)
        if self.sm['carControl'].enabled and not self.CC_prev.enabled:
          # Use CarState w/ buttons from the step selfdrived enables on
          self.v_cruise_helper.initialize_v_cruise(self.CS_prev, self.experimental_mode)
      else:
        self._update_preap_map_cruise(CS)
    except Exception:
      # Fail-safe: never crash card due cruise-target selection logic.
      cloudlog.exception("Pre-AP software cruise target update failed, falling back to VCruiseHelper default")
      self.v_cruise_helper.update_v_cruise(CS, self.sm['carControl'].enabled, self.is_metric)
      if self.sm['carControl'].enabled and not self.CC_prev.enabled:
        self.v_cruise_helper.initialize_v_cruise(self.CS_prev, self.experimental_mode)

    # TODO: mirror the carState.cruiseState struct?
    CS.vCruise = float(self.v_cruise_helper.v_cruise_kph)
    CS.vCruiseCluster = float(self.v_cruise_helper.v_cruise_cluster_kph)

    return CS, RD

  def _update_preap_map_cruise(self, CS) -> None:
    """OSM overlay for pre-AP pedal MAX. Slew state stays in raw-limit units."""
    # Pre-AP pedal mode owns set-speed via pedal_speed_kph. Overlay OSM
    # onto vCruise/MAX for the OP session (cruiseEnabled), including a
    # brake long pause so sticky MAX can rebase. The first
    # pull is lateral-only and must not seed or arm a sticky hold.
    raw_kph = float(CS.cruiseState.speed * CV.MS_TO_KPH)
    raw_kph, follow_stalk = self._maybe_follow_stalk(CS, raw_kph)
    long_active = bool(getattr(CS, 'pedalLongActive', False))
    long_active_prev = bool(getattr(self.CS_prev, 'pedalLongActive', False))
    engage_rising = long_active and not long_active_prev
    session_enabled = bool(getattr(CS.cruiseState, 'enabled', False))
    resume_held, take_speed_now = self._preap_set_events()
    # Overlay held can be None (maps off never stalk-latched). The FSM
    # still has the brake-pause MAX — adopt it before pause substitution
    # so we never treat ego as held, and so session_engaged stays up.
    self._adopt_preap_fsm_held_max()
    # Software long (enableLongControl), not pedal authority. Pause and
    # the authority-acquisition gap both have pedalLongActive False;
    # only a true long pause publishes cruiseState.speed as ego.
    soft_long = bool(getattr(CS, 'enableLongControl', False))
    # Pause publishes cruiseState.speed as ego. Keep the held MAX instead.
    if not soft_long and self._map_hold.held_max_kph is not None:
      raw_kph = float(self._map_hold.held_max_kph)
    traveled_kph = float(CS.vEgo) * CV.MS_TO_KPH
    has_held = self._map_hold.held_max_kph is not None
    session_engaged = bool(session_enabled) and (
      soft_long or has_held or resume_held or take_speed_now
    )
    stalk_pressed = (not follow_stalk) and self._preap_stalk_set_pressed(CS)
    self_wrote_pedal = (
      self._pedal_self_write_kph is not None
      and abs(float(raw_kph) - float(self._pedal_self_write_kph)) <= MANUAL_SET_EPS_KPH
    )
    self._pedal_self_write_kph = None
    posted_kph = None
    map_kph = None
    map_valid = bool(self.sm.valid.get('liveMapDataNAP', False) and self.sm['liveMapDataNAP'].speedLimitValid)
    md = self.sm['liveMapDataNAP'] if map_valid else None
    raw_posted_kph = None
    if md is not None and md.speedLimit > 0:
      raw_posted_kph = float(md.speedLimit) * CV.MS_TO_KPH
    # Offset from raw OSM posted (mph). Hypermile eco / Step Down apply
    # only with a known map posted — never invent a drop when maps are
    # off, unmatched, or speedLimit unknown (same as sticky MAX).
    maps_posted = raw_posted_kph is not None
    map_offset_kph = self._live_map_offset_kph(raw_posted_kph, maps_posted=maps_posted)
    if raw_posted_kph is not None:
      posted_kph = raw_posted_kph + map_offset_kph
    # Snapshot pre-curve MAX before decide so OSM flicker cannot wipe sticky.
    last_hud_kph = float(self.v_cruise_helper.v_cruise_kph)
    steer_deg = float(getattr(CS, 'steeringAngleDeg', 0.0) or 0.0)
    curve_kappa, curve_yaw = self._curve_cornering()
    # NAPCurveFollow = 2: the planner's continuous curve term owns bends and MAX
    # is never touched. CurveMaxHold is fed engaged=False, which resets it and
    # passes posted / HUD MAX straight through (no cap, snapshot, restore or
    # posted freeze). 0 / 1 (shadow): CurveMaxHold runs exactly as before.
    curve_follow_on = getattr(self, "_curve_follow_mode", MODE_SHADOW) == MODE_ACTIVE
    policy_posted_kph, _ = self._curve_max.begin_cycle(
      self._map_hold,
      last_hud_kph=last_hud_kph,
      posted_kph=posted_kph,
      v_ego_ms=float(CS.vEgo),
      angle_steers_deg=steer_deg,
      steer_ratio=float(getattr(self.CP, 'steerRatio', 0.0) or 0.0),
      wheelbase=float(getattr(self.CP, 'wheelbase', 0.0) or 0.0),
      engaged=session_engaged and not curve_follow_on,
      take_speed_now=take_speed_now,
      dt=DT_CTRL,
      curvature=curve_kappa,
      yaw_rate=curve_yaw,
    )
    dec = decide_map_cruise(
      self._map_hold,
      engaged=session_engaged,
      mode=self._map_speed_mode,
      raw_kph=raw_kph,
      posted_kph=policy_posted_kph,
      engage_rising=engage_rising,
      now=time.monotonic(),
      stalk_pressed=stalk_pressed,
      take_speed_now=take_speed_now,
      resume_held=resume_held,
      traveled_kph=traveled_kph,
      long_active=soft_long,
      self_wrote_pedal=self_wrote_pedal,
    )
    # Stalk +/- while maps off updates overlay held; keep the FSM latch
    # aligned so the next brake pause resumes that MAX.
    self._publish_preap_held_max(long_active=soft_long)
    if map_valid and md is not None and not dec.sticky:
      # Any posted decrease: kin+110 m ease, never assign the new limit in one shot.
      lim = effective_map_limit_ms(
        float(md.speedLimit),
        float(md.nextSpeedLimit),
        float(md.nextSpeedLimitDistance),
        float(CS.vEgo),
        self._map_speed_lookahead,
        self._map_speed_accel,
        sticky=False,
      )
    else:
      lim = None
    # RB funnel is geometry, not a posted rebase. Ease even on a sticky
    # hold so aTarget cannot stay +a into the ring (Willmar 49→17).
    # The ring hint does not need a speed-limit match: a ring often has no
    # maxspeed. Read it whenever mapd publishes it, then use the ring speed.
    rb_lim = None
    rb_md = self.sm['liveMapDataNAP'] if self.sm.valid.get('liveMapDataNAP', False) else None
    rb_hint = live_map_roundabout_hint(rb_md)
    if rb_hint is not None:
      posted_ms = float(md.speedLimit) if md is not None and md.speedLimit > 0 else float(CS.vEgo)
      rb_lim = roundabout_ease_v_ms(rb_hint, float(CS.vEgo), posted_ms, self._map_speed_lookahead)
      if rb_lim is not None and map_valid and md is not None:
        lim = rb_lim if lim is None else min(float(lim), rb_lim)
    was_on_ring = bool(getattr(self, "_rb_active", False))
    exiting_ring = was_on_ring and rb_lim is None
    if exiting_ring:
      ring_kph = float(getattr(self, "_rb_target_ms", 0.0) or 0.0) * CV.MS_TO_KPH
      if _unwind_ring_speed_set(self._map_hold, getattr(self, "_rb_pre", None), ring_kph):
        dec.sticky = self._map_hold.sticky_set_kph is not None
        dec.follow_override = bool(dec.sticky)
        if not dec.sticky and map_valid and md is not None:
          lim = effective_map_limit_ms(
            float(md.speedLimit),
            float(md.nextSpeedLimit),
            float(md.nextSpeedLimitDistance),
            float(CS.vEgo),
            self._map_speed_lookahead,
            self._map_speed_accel,
            sticky=False,
          )
    if map_valid and md is not None and lim is not None and lim > 0:
      if rb_lim is not None:
        # Instant RB latch: MAX is the ring target now. Do not Accel-5
        # slew over another 100 m after long enables in the funnel.
        self._map_slew_ms = float(lim)
        map_kph = float(lim) * CV.MS_TO_KPH
      else:
        if self._map_slew_ms is None:
          hud_kph = float(self.v_cruise_helper.v_cruise_kph)
          prev_ms = map_slew_from_displayed_kph(hud_kph, map_offset_kph)
          if 0.0 < hud_kph < V_CRUISE_UNSET and prev_ms > lim + 0.3:
            self._map_slew_ms = prev_ms
          else:
            self._map_slew_ms = lim
        a = map_slew_a_ms2(
          self._map_slew_ms, lim, self._map_speed_lookahead, self._map_speed_accel,
        )
        self._map_slew_ms = slew_map_speed_ms(self._map_slew_ms, lim, DT_CTRL, a)
        map_kph = self._map_slew_ms * CV.MS_TO_KPH
    elif map_valid and md is not None:
      self._map_slew_ms = None
    elif not map_valid:
      self._map_slew_ms = None
    # seed_kph is a one-shot write (engage, posted raise, stalk step).
    # Sticky hold uses driver_kph / follow_override — do not write pedal
    # every frame or CI.update's stalk step is undone.
    if dec.seed_kph is not None:
      preap_v_cruise_kph = dec.seed_kph
      self._map_slew_ms = map_slew_from_displayed_kph(dec.seed_kph, map_offset_kph)
    else:
      preap_v_cruise_kph = apply_map_speed_kph(
        dec.driver_kph,
        map_kph,
        mode=self._map_speed_mode,
        offset_kph=map_offset_kph,
        engaged=session_engaged,
        op_long_software_cruise=True,
        driver_override=dec.follow_override,
      )
    ring_capped = False
    if rb_lim is not None:
      rb_kph = float(rb_lim) * CV.MS_TO_KPH
      # The ring ceiling is the HUD MAX only. A higher set stays the
      # resume target so a SET in the ring cannot latch the ring speed.
      if float(preap_v_cruise_kph) > rb_kph + MANUAL_SET_EPS_KPH:
        ring_capped = True
      preap_v_cruise_kph = min(float(preap_v_cruise_kph), rb_kph)
      self._map_slew_ms = float(rb_lim)
    # Temporary curve cap may lower HUD MAX. Restore seed puts pre-curve
    # MAX back after the bend. Do not let that cap rebase sticky / held.
    curve_out = self._curve_max.finish(
      hud_kph=preap_v_cruise_kph,
      hold=self._map_hold,
      posted_kph=posted_kph,
      v_ego_ms=float(CS.vEgo),
      angle_steers_deg=steer_deg,
      steer_ratio=float(getattr(self.CP, 'steerRatio', 0.0) or 0.0),
      wheelbase=float(getattr(self.CP, 'wheelbase', 0.0) or 0.0),
      engaged=session_engaged and not curve_follow_on,
      stalk_pressed=stalk_pressed,
      take_speed_now=take_speed_now,
      dt=DT_CTRL,
      curvature=curve_kappa,
      yaw_rate=curve_yaw,
      restore_a_ms2=map_accel_a_ms2(self._map_speed_lookahead, self._map_speed_accel),
      long_active=soft_long,
    )
    log_curve_max(self, curve_out, preap_v_cruise_kph, posted_kph, CS)
    preap_v_cruise_kph = float(curve_out.hud_kph)
    restore_seed_kph = curve_out.restore_seed_kph
    seed_kph = dec.seed_kph if restore_seed_kph is None else float(restore_seed_kph)
    if restore_seed_kph is not None:
      self._map_slew_ms = map_slew_from_displayed_kph(float(restore_seed_kph), map_offset_kph)
    if exiting_ring:
      # Leave the ring at least at current speed and ramp toward the
      # posted / pre-ring MAX. MAX stays a hard ceiling (the slew target).
      # Gap lock still owns the lead gap; this does not raise it.
      floor_ms = max(float(getattr(self, "_rb_target_ms", 0.0) or 0.0), float(CS.vEgo))
      target_ms = _ring_exit_target_ms(self._map_hold, None if dec.sticky else lim, posted_kph,
                                       getattr(self, "_rb_detect_v_ms", None))
      preap_v_cruise_kph, self._map_slew_ms = _apply_ring_exit_max(
        target_ms, floor_ms, self._map_speed_lookahead, self._map_speed_accel,
      )
    # Write engage/posted/stalk seed, or when HUD MAX rose. Never write
    # the same sticky MAX every frame. Pause still writes a rebase /
    # resume seed onto pedal_speed so one SET keeps the held MAX.
    # Curve restore is a one-shot seed so MAX returns to the pre-bend set.
    write_max = long_active or soft_long or resume_held or take_speed_now or (
      session_engaged and seed_kph is not None
    )
    if write_max and should_write_preap_pedal(seed_kph, preap_v_cruise_kph, self._last_pedal_kph) and not ring_capped:
      self._write_preap_pedal_speed(CS, preap_v_cruise_kph)
      self._last_pedal_kph = float(preap_v_cruise_kph)
      self._pedal_self_write_kph = float(preap_v_cruise_kph)
    elif not session_enabled:
      self._last_pedal_kph = None
      self._pedal_self_write_kph = None
    if rb_lim is not None:
      # Latch / unwind key on the ring target, not the approach profile
      # value (rb_lim ramps down along the 0.7 m/s² profile).
      rb_target_ms = roundabout_target_ms(rb_hint.speed_limit_ms)
      ring_kph = rb_target_ms * CV.MS_TO_KPH
      if not was_on_ring or getattr(self, "_rb_pre", None) is None:
        self._rb_pre = _snapshot_cruise_hold(self._map_hold)
        self._rb_detect_v_ms = float(CS.vEgo)
      else:
        _refresh_ring_snapshot(self._rb_pre, self._map_hold, ring_kph)
      _unwind_ring_speed_set(self._map_hold, self._rb_pre, ring_kph)
      eng = self._preap_engagement()
      if eng is not None and _kph_near(getattr(eng, "_nap_held_max_kph", None), ring_kph):
        restore = None if self._rb_pre is None else self._rb_pre.get("held_max_kph")
        if restore is not None and not _kph_near(restore, ring_kph):
          eng._nap_held_max_kph = restore
      self._rb_target_ms = rb_target_ms
      self._rb_active = True
    else:
      self._rb_active = False
    self.v_cruise_helper.v_cruise_kph_last = self.v_cruise_helper.v_cruise_kph
    self.v_cruise_helper.v_cruise_kph = preap_v_cruise_kph
    self.v_cruise_helper.v_cruise_cluster_kph = preap_v_cruise_kph

  def _curve_cornering(self) -> tuple[float | None, float | None]:
    """Vehicle-model curvature, else livePose yaw rate. None if not yet valid.

    controlsd publishes the learned-ratio / angle-offset curvature on
    carControl.currentCurvature (the same value as controlsState.curvature).
    card already receives carControl, so this is a field read, not another
    100 Hz poll. livePose stays the low-rate yaw fallback only.
    """
    curvature = None
    yaw_rate = None
    if self.sm.valid.get('carControl', False):
      curvature = float(self.sm['carControl'].currentCurvature)
    if self.sm.valid.get('livePose', False):
      av = self.sm['livePose'].angularVelocityDevice
      if bool(getattr(av, 'valid', False)):
        yaw_rate = float(av.z)
    return curvature, yaw_rate

  def _preap_engagement(self):
    inner = getattr(self.CI, 'CS', None)
    return getattr(inner, 'engagement', None) if inner is not None else None

  def _preap_set_events(self) -> tuple[bool, bool]:
    """Consume one-shot SET flags from the Pre-AP engagement patch.

    resume_held: one SET after a brake long pause.
    take_speed_now: double SET / initial engage (forget sticky).
    """
    eng = self._preap_engagement()
    resume = bool(getattr(eng, '_nap_set_resume_long', False))
    take = bool(getattr(eng, '_nap_set_take_speed_now', False))
    if eng is not None:
      eng._nap_set_resume_long = False
      eng._nap_set_take_speed_now = False
    return resume, take

  def _adopt_preap_fsm_held_max(self) -> None:
    """If overlay never latched MAX, take the brake/turn FSM hold.

    Maps off does not invent posted. A stalk-set pedal MAX can exist on
    the FSM latch while MapCruiseHold.held_max_kph is still None. Adopting
    it before pause substitution prevents writing ego into held.
    """
    if self._map_hold.held_max_kph is not None:
      return
    eng = self._preap_engagement()
    held = getattr(eng, '_nap_held_max_kph', None) if eng is not None else None
    if held is not None and 0.0 < float(held) < V_CRUISE_UNSET:
      self._map_hold.held_max_kph = float(held)

  def _publish_preap_held_max(self, *, long_active: bool) -> None:
    """Keep the FSM latch in sync with overlay held while software long is on."""
    if not long_active or self._map_hold.held_max_kph is None:
      return
    eng = self._preap_engagement()
    if eng is not None:
      eng._nap_held_max_kph = float(self._map_hold.held_max_kph)

  def _preap_cruise_detent(self) -> int | None:
    """Raw SpdCtrlLvr_Stat / CruiseButtons. Distinguishes tip vs 2nd detent."""
    inner = getattr(self.CI, "CS", None)
    if inner is None:
      return None
    if hasattr(inner, "cruise_buttons"):
      try:
        return int(inner.cruise_buttons)
      except (TypeError, ValueError):
        pass
    msg = getattr(inner, "msg_stw_actn_req", None)
    if isinstance(msg, dict):
      try:
        return int(msg.get("SpdCtrlLvr_Stat") or 0)
      except (TypeError, ValueError):
        return None
    return None

  def _maybe_follow_stalk(self, CS, raw_kph: float) -> tuple[float, bool]:
    """Route Pre-AP stalk tip to City or Highway Follow Distance with a lead.

    Same whether Hypermile is On or Off. A first-detent tip undoes that
    frame's 1 mph MAX and, after the lever returns to IDLE without a
    2nd detent, steps the matching 1–7: Highway when MAX is 50 mph or
    higher (including while ego is still coming up to that set speed),
    City when MAX is under 50 even if ego is already faster. Traveled
    speed picks the band only when MAX is unset. HUD NAPFollowDistance
    tracks the band just stepped. A full press (2nd detent / 5 mph)
    keeps MAX +5/−5 and does not remap Follow, even if the lever passed
    through first detent.
    No lead: leave stalk as MAX adjust. During a long pause,
    cruiseState.speed is ego — detent still distinguishes tip vs hold;
    buttonEvents alone do not.
    """
    has_lead = bool(
      self.sm.valid.get("radarState", False)
      and getattr(self.sm["radarState"].leadOne, "status", False)
    )
    soft_long = bool(getattr(CS, "enableLongControl", False))
    prev_raw = None
    try:
      prev_raw = float(self.CS_prev.cruiseState.speed) * CV.MS_TO_KPH
    except Exception:
      prev_raw = None
    events = getattr(CS, "buttonEvents", None)
    commit, closer, undo = self._follow_gesture.update(
      has_lead=has_lead,
      detent=self._preap_cruise_detent(),
      button_closer=button_event_closer(events),
      button_released=button_event_released(events),
      raw_kph=raw_kph if soft_long else None,
      prev_raw_kph=prev_raw if soft_long else None,
    )
    routed = commit or self._follow_gesture.is_pending or undo is not None
    if commit and closer is not None and (time.monotonic() - self._follow_stalk_mono) >= 0.25:
      # CarState.vCruise is still 0 here — card publishes it at the end of
      # the frame. Read the HUD MAX already on the helper.
      v_cruise_ms = published_cruise_ms(self.v_cruise_helper.v_cruise_kph, V_CRUISE_UNSET)
      persist_follow_distance(
        self.params, bool(closer),
        v_ego=getattr(CS, "vEgo", None),
        v_cruise=v_cruise_ms,
        has_lead=has_lead,
        engaged=soft_long,
      )
      self._follow_stalk_mono = time.monotonic()
    if undo is not None and soft_long:
      self._write_preap_pedal_speed(CS, undo)
      self._last_pedal_kph = float(undo)
      self._pedal_self_write_kph = float(undo)
      return float(undo), True
    return raw_kph, routed

  @staticmethod
  def _preap_stalk_set_pressed(CS) -> bool:
    """True on stalk +/-. Extra signal; pedal_speed 1/5 mph steps also count."""
    for be in getattr(CS, 'buttonEvents', None) or []:
      try:
        typ = getattr(be, 'type', None)
        name = str(getattr(typ, 'name', typ)).lower().replace('_', '')
        if 'accelcruise' in name or 'decelcruise' in name or name in ('accel', 'decel'):
          return True
      except Exception:
        continue
    return False

  def _write_preap_pedal_speed(self, CS, kph: float) -> None:
    """Write HUD MAX into pre-AP pedal_speed so long / stalk +/- track that set."""
    kph = float(kph)
    try:
      CS.cruiseState.speed = kph * CV.KPH_TO_MS
    except Exception:
      pass
    inner = getattr(self.CI, 'CS', None)
    if inner is None:
      return
    if hasattr(inner, 'pedal_speed_kph'):
      inner.pedal_speed_kph = kph
    eng = getattr(inner, 'engagement', None)
    if eng is not None and hasattr(eng, 'pedal_speed_kph'):
      eng.pedal_speed_kph = kph
      eng._nap_held_max_kph = kph

  def state_publish(self, CS: car.CarState, RD: structs.RadarDataT | None):
    """carState and carParams publish loop"""

    # carParams - logged every 50 seconds (> 1 per segment)
    if self.sm.frame % int(50. / DT_CTRL) == 0:
      cp_send = messaging.new_message('carParams')
      cp_send.valid = True
      cp_send.carParams = self.CP
      self.pm.send('carParams', cp_send)

    # publish new carOutput
    co_send = messaging.new_message('carOutput')
    co_send.valid = self.sm.all_checks(['carControl'])
    co_send.carOutput.actuatorsOutput = self.last_actuators_output
    self.pm.send('carOutput', co_send)

    # kick off controlsd step while we actuate the latest carControl packet
    cs_send = messaging.new_message('carState')
    cs_send.valid = CS.canValid
    cs_send.carState = CS
    cs_send.carState.canErrorCounter = self.can_rcv_cum_timeout_counter
    cs_send.carState.cumLagMs = -self.rk.remaining * 1000.
    self.pm.send('carState', cs_send)
    if self._tesla_preap:
      from openpilot.selfdrive.car.tesla.preap_body_controls import note_published_car_state
      note_published_car_state(cs_send.carState)

    if RD is not None:
      tracks_msg = messaging.new_message('liveTracks')
      tracks_msg.valid = not any(RD.errors.to_dict().values())
      tracks_msg.liveTracks = RD
      self.pm.send('liveTracks', tracks_msg)

  def _preap_lat_yield_flag(self, CC: car.CarControl):
    """Host-only 0x561. Widens yield-not-disengage. Never an actuation bit.

    Panda drops the frame (tx hook returns false). Stuck true means a
    steering yank yields lateral instead of ending the session, which is
    the existing yielded-lateral rule. Stuck false is today's disengage
    plus the local angle-error help check. Doors, gear, stalk, brake,
    and an EPAS reject with hands off still disengage.
    """
    from opendbc.car.tesla.preap.lat_yield import (
      UNDERTRACK_CURVATURE, YIELD_FLAG_ADDR, encode_yield_flag, roundabout_yield_context)

    md = self.sm['liveMapDataNAP'] if self.sm.alive.get('liveMapDataNAP', False) else None
    hint = live_map_roundabout_hint(md)
    on_rb = bool(hint is not None and roundabout_yield_context(
      hint.on_roundabout, hint.approaching, hint.distance_m))
    try:
      under = abs(float(CC.actuators.curvature) - float(CC.currentCurvature)) > UNDERTRACK_CURVATURE
    except (TypeError, ValueError):
      under = False
    eng = getattr(getattr(self.CI, "CS", None), "engagement", None)
    if eng is not None:
      eng._nap_roundabout_yield = on_rb
      eng._nap_undertrack = bool(under)
    return (YIELD_FLAG_ADDR, encode_yield_flag(on_rb, under), 0)

  def controls_update(self, CS: car.CarState, CC: car.CarControl):
    """control update loop, driven by carControl"""

    if not self.initialized_prev:
      # Initialize CarInterface, once controls are ready
      # TODO: this can make us miss at least a few cycles when doing an ECU knockout
      self.CI.init(self.CP, *self.can_callbacks)
      # signal pandad to switch to car safety mode
      self.params.put_bool("ControlsReady", True)

    if self.sm.all_alive(['carControl']):
      # send car controls over can
      now_nanos = self.can_log_mono_time if REPLAY else int(time.monotonic() * 1e9)
      self.last_actuators_output, can_sends = self.CI.apply(CC, now_nanos)
      can_sends = list(can_sends)
      if getattr(self, "_tesla_preap", False):
        can_sends.append(self._preap_lat_yield_flag(CC))
      if self.radar_donor_vin is not None:
        controls_allowed = False
        if self.sm.valid['pandaStates']:
          controls_allowed = any(ps.controlsAllowed for ps in self.sm['pandaStates'])
        force_read = self.params.get_bool("NAPRadarReadVin")
        if force_read and not self._nap_conf.radar_enabled:
          self.params.put("NAPRadarVinReadStatus", "enable radar first")
          self.params.put_bool("NAPRadarReadVin", False)
        else:
          can_sends.extend(self.radar_donor_vin.update(
            self._can_packets,
            time.monotonic(),
            radar_enabled=self._nap_conf.radar_enabled,
            stored_vin=self._nap_conf.radar_donor_vin,
            controls_allowed=controls_allowed,
            enabled=bool(CC.enabled),
            force_read=force_read,
          ))
          if force_read and self.radar_donor_vin.read_finished:
            if not self.radar_donor_vin.reader.vin and self.radar_donor_vin.reader.failure:
              self.params.put(
                "NAPRadarVinReadStatus",
                self.radar_donor_vin.reader.failure.name.lower().replace("_", " "),
              )
            self.params.put_bool("NAPRadarReadVin", False)
      self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))

      self.CC_prev = CC

  def step(self):
    CS, RD = self.state_update()

    self.state_publish(CS, RD)

    initialized = (not any(e.name == EventName.selfdriveInitializing for e in self.sm['onroadEvents']) and
                   self.sm.seen['onroadEvents'])
    if not self.CP.passive and initialized:
      self.controls_update(CS, self.sm['carControl'])

    self.initialized_prev = initialized
    self.CS_prev = CS

  def _refresh_map_speed_params(self):
    self._curve_follow_mode = read_curve_follow_mode(self.params)
    mode, offset, lookahead, accel = read_map_speed_params(self.params)
    hm_on = read_hypermile_params(self.params)
    self._map_speed_mode = mode
    # User / settings offset only. Hypermile eco is computed live from posted.
    self._map_speed_user_offset_kph = offset
    self._map_hypermile_on = hm_on
    self._map_step_down_on = read_hypermile_step_down(self.params)
    self._map_speed_lookahead = lookahead
    self._map_speed_accel = accel

  def _live_map_offset_kph(self, raw_posted_kph: float | None,
                           maps_posted: bool | None = None) -> float:
    return map_target_offset_kph(
      self._map_speed_user_offset_kph,
      hypermile_on=self._map_hypermile_on,
      step_down_on=self._map_step_down_on,
      posted_kph=raw_posted_kph,
      maps_posted=maps_posted,
    )

  def params_thread(self, evt):
    while not evt.is_set():
      self.is_metric = self.params.get_bool("IsMetric")
      self.experimental_mode = self.params.get_bool("ExperimentalMode") and self.CP.openpilotLongitudinalControl
      self._refresh_map_speed_params()
      time.sleep(0.1)

  def card_thread(self):
    e = threading.Event()
    t = threading.Thread(target=self.params_thread, args=(e, ))
    try:
      t.start()
      while True:
        self.step()
        self.rk.monitor_time()
    finally:
      e.set()
      t.join()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  car = Car()
  car.card_thread()


if __name__ == "__main__":
  main()
