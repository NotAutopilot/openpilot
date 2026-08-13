"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from dataclasses import dataclass

from openpilot.common.params import Params
from opendbc.car import structs
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP, HyundaiSafetyFlagsSP
from opendbc.sunnypilot.car.tesla.values import MadsScreenButtonType, TeslaFlagsSP


MADS_NO_ACC_MAIN_BUTTON = ("rivian", "tesla")

MadsMainCruiseInputKind = structs.CarParamsSP.MadsMainCruiseInputKind
MadsSteeringMode = structs.CarParamsSP.MadsSteeringMode


class MadsSteeringModeOnBrake:
  REMAIN_ACTIVE = 0
  PAUSE = 1
  DISENGAGE = 2


STEERING_MODE_TO_INT = {
  MadsSteeringMode.remainActive: MadsSteeringModeOnBrake.REMAIN_ACTIVE,
  MadsSteeringMode.pause: MadsSteeringModeOnBrake.PAUSE,
  MadsSteeringMode.disengage: MadsSteeringModeOnBrake.DISENGAGE,
}


@dataclass(frozen=True)
class MadsCapabilities:
  full_settings_available: bool
  main_cruise_input_kind: structs.CarParamsSP.MadsMainCruiseInputKind
  main_cruise_allowed: bool
  mads_required: bool
  tesla_coop_steering_available: bool
  unified_engagement_mode: bool
  steering_mode: int
  hands_on_pause_available: bool
  no_main_cruise: bool


def _truthy_mads(value) -> bool:
  return value in (True, 1, "1", b"1", "true", "True")


def is_mads_required(CP_SP=None, params: Params | None = None) -> bool:
  if CP_SP is not None:
    return bool(getattr(CP_SP, "madsRequired", False)) and int(getattr(CP_SP, "madsCapabilityContractVersion", 0) or 0) >= 1
  if params is None:
    params = Params()
  try:
    from openpilot.cereal import custom
    import openpilot.cereal.messaging as messaging
    for key in ("CarParamsSP", "CarParamsSPPersistent", "CarParamsSPCache"):
      raw = params.get(key)
      if not raw:
        continue
      msg = messaging.log_from_bytes(raw, custom.CarParamsSP)
      return bool(msg.madsRequired) and int(msg.madsCapabilityContractVersion) >= 1
  except Exception:
    return False
  return False


def persist_required_mads(params: Params, CP_SP=None) -> bool:
  """Force and persist Mads=true on required platforms. Returns True if required."""
  if not is_mads_required(CP_SP, params):
    return False
  if not params.get_bool("Mads"):
    params.put_bool("Mads", True, block=True)
  return True


def coerce_mads_write(params: Params, key: str, value, CP_SP=None):
  """Reject runtime false writes of Mads on required platforms."""
  if key != "Mads":
    return value
  if persist_required_mads(params, CP_SP) and not _truthy_mads(value):
    return True
  return value


def _version0_limited_tesla(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params: Params) -> bool:
  if CP.brand != "tesla":
    return False
  if not CP_SP.flags & TeslaFlagsSP.HAS_VEHICLE_BUS:
    return True
  screen_button = int(params.get("TeslaMadsScreenButton", return_default=True) or 0)
  return screen_button == MadsScreenButtonType.OFF


def get_mads_limited_brands(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params: Params) -> bool:
  caps = resolve_mads_capabilities(CP, CP_SP, params)
  if caps.mads_required:
    return False
  if CP.brand == "rivian":
    return True
  if CP.brand == "tesla":
    return not caps.full_settings_available
  return False


def resolve_mads_capabilities(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params: Params | None = None) -> MadsCapabilities:
  if getattr(CP_SP, "madsCapabilityContractVersion", 0) >= 1:
    kind = CP_SP.madsMainCruiseInputKind
    return MadsCapabilities(
      full_settings_available=bool(CP_SP.madsFullSettingsAvailable),
      main_cruise_input_kind=kind,
      main_cruise_allowed=bool(CP_SP.madsMainCruiseAllowed),
      mads_required=bool(CP_SP.madsRequired),
      tesla_coop_steering_available=bool(CP_SP.teslaCoopSteeringAvailable),
      unified_engagement_mode=bool(CP_SP.madsUnifiedEngagementMode),
      steering_mode=STEERING_MODE_TO_INT.get(CP_SP.madsSteeringMode, MadsSteeringModeOnBrake.REMAIN_ACTIVE),
      hands_on_pause_available=bool(CP_SP.madsHandsOnPauseAvailable),
      no_main_cruise=kind in (MadsMainCruiseInputKind.none, MadsMainCruiseInputKind.momentary),
    )

  if params is None:
    params = Params()
  limited = CP.brand == "rivian" or _version0_limited_tesla(CP, CP_SP, params)
  kind = MadsMainCruiseInputKind.none if CP.brand in MADS_NO_ACC_MAIN_BUTTON else MadsMainCruiseInputKind.stateful
  steering = MadsSteeringModeOnBrake.DISENGAGE if limited else int(params.get("MadsSteeringMode", return_default=True) or 0)
  return MadsCapabilities(
    full_settings_available=not limited,
    main_cruise_input_kind=kind,
    main_cruise_allowed=bool(params.get_bool("MadsMainCruiseAllowed")) if kind == MadsMainCruiseInputKind.stateful else False,
    mads_required=False,
    tesla_coop_steering_available=CP.brand == "tesla",
    unified_engagement_mode=bool(params.get_bool("MadsUnifiedEngagementMode")),
    steering_mode=steering,
    hands_on_pause_available=False,
    no_main_cruise=kind in (MadsMainCruiseInputKind.none, MadsMainCruiseInputKind.momentary),
  )


def read_steering_mode_param(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params: Params):
  return resolve_mads_capabilities(CP, CP_SP, params).steering_mode


def set_alternative_experience(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params: Params):
  caps = resolve_mads_capabilities(CP, CP_SP, params)
  enabled = True if persist_required_mads(params, CP_SP) else params.get_bool("Mads")
  steering_mode = caps.steering_mode

  if enabled:
    CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.ENABLE_MADS

    if steering_mode == MadsSteeringModeOnBrake.DISENGAGE:
      CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.MADS_DISENGAGE_LATERAL_ON_BRAKE
    elif steering_mode == MadsSteeringModeOnBrake.PAUSE:
      CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.MADS_PAUSE_LATERAL_ON_BRAKE


def set_car_specific_params(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params: Params):
  if CP.brand == "hyundai":
    # TODO-SP: This should be separated from MADS module for future implementations
    #          Use "HyundaiLongitudinalMainCruiseToggleable" param
    hyundai_cruise_main_toggleable = True
    if hyundai_cruise_main_toggleable:
      CP_SP.flags |= HyundaiFlagsSP.LONGITUDINAL_MAIN_CRUISE_TOGGLEABLE.value
      CP_SP.safetyParam |= HyundaiSafetyFlagsSP.LONG_MAIN_CRUISE_TOGGLEABLE

  caps = resolve_mads_capabilities(CP, CP_SP, params)
  if persist_required_mads(params, CP_SP):
    return

  # MADS Partial Support
  # MADS is currently partially supported for these platforms due to lack of consistent states to engage controls
  # Only MadsSteeringModeOnBrake.DISENGAGE is supported for these platforms
  # TODO-SP: To enable MADS full support for Rivian and most Tesla, identify consistent signals for MADS toggling
  mads_partial_support = get_mads_limited_brands(CP, CP_SP, params)
  if mads_partial_support:
    params.put("MadsSteeringMode", 2, block=True)
    params.put_bool("MadsUnifiedEngagementMode", True, block=True)
    CP_SP.madsUnifiedEngagementMode = True
    CP_SP.madsSteeringMode = structs.CarParamsSP.MadsSteeringMode.disengage

  # no ACC MAIN button for these brands
  if caps.no_main_cruise:
    params.remove("MadsMainCruiseAllowed")
