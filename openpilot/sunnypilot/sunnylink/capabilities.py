"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json

from openpilot.cereal import custom, messaging
from opendbc.car.structs import car
from opendbc.car.hyundai.values import CAR as HYUNDAI_CAR, UNSUPPORTED_LONGITUDINAL_CAR
from opendbc.car.subaru.values import CAR as SUBARU_CAR, SubaruFlags
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.common.hardware import HARDWARE
from opendbc.car.tesla.preap.sp.platform import is_preap_ui_platform


# Wire-protocol version for the capabilities payload. Bump on breaking changes
# only; additive fields are backward-compatible and do not require a bump.
PROTOCOL_VERSION = 1

# All capability fields that rules may reference.
# Non-boolean fields must have defaults in CAPABILITY_DEFAULTS.
CAPABILITY_FIELDS = (
  "protocol_version",
  "has_longitudinal_control",
  "has_icbm",
  "icbm_available",
  "torque_allowed",
  "brand",
  "pcm_cruise",
  "alpha_long_available",
  "steer_control_type",
  "enable_bsm",
  "is_release",
  "is_sp_release",
  "is_development",
  "tesla_has_vehicle_bus",
  "tesla_preap",
  "tesla_preap_pedal",
  "tesla_preap_radar",
  "tesla_preap_pedal_calib",
  "tesla_preap_independent_brake",
  "tesla_preap_active_mode",
  "tesla_preap_longitudinal_path",
  "tesla_preap_pedal_health",
  "tesla_preap_radar_health",
  "has_stop_and_go",
  "stock_longitudinal",
  "device_type",
  "subaru_has_sng",
  "hyundai_alpha_long_available",
)

CAPABILITY_LABELS: dict[str, str] = {
  "protocol_version": "Capabilities protocol version",
  "has_longitudinal_control": "sunnypilot longitudinal control",
  "has_icbm": "ICBM enabled",
  "icbm_available": "ICBM available",
  "torque_allowed": "torque steering (not available for angle steering vehicles)",
  "brand": "Vehicle brand",
  "pcm_cruise": "PCM cruise",
  "alpha_long_available": "Alpha Longitudinal available",
  "steer_control_type": "Steer control type",
  "enable_bsm": "BSM available",
  "is_release": "Release branch",
  "is_sp_release": "SP release branch",
  "is_development": "Development branch",
  "tesla_has_vehicle_bus": "Tesla vehicle bus",
  "tesla_preap": "Tesla Pre-AP platform",
  "tesla_preap_pedal": "Tesla Pre-AP pedal interceptor",
  "tesla_preap_radar": "Tesla Pre-AP Bosch radar",
  "tesla_preap_pedal_calib": "Tesla Pre-AP pedal calibration available",
  "tesla_preap_independent_brake": "Tesla Pre-AP independent brake-mode settings",
  "tesla_preap_active_mode": "Tesla Pre-AP active engagement mode",
  "tesla_preap_longitudinal_path": "Tesla Pre-AP physical longitudinal path",
  "tesla_preap_pedal_health": "Tesla Pre-AP pedal health",
  "tesla_preap_radar_health": "Tesla Pre-AP radar health",
  "has_stop_and_go": "Stop and Go",
  "stock_longitudinal": "stock longitudinal",
  "device_type": "Device type",
  "subaru_has_sng": "Subaru Stop-and-Go available",
  "hyundai_alpha_long_available": "Hyundai Alpha Longitudinal available",
}

# Explicit defaults for non-boolean capability fields
CAPABILITY_DEFAULTS: dict[str, bool | str | int] = {
  "brand": "",
  "steer_control_type": "",
  "device_type": "",
  "protocol_version": PROTOCOL_VERSION,
  "tesla_preap_active_mode": "",
  "tesla_preap_longitudinal_path": "",
  "tesla_preap_pedal_health": "none",
  "tesla_preap_radar_health": "none",
}


def _bundle_field(bundle: dict | None, key: str) -> str:
  return bundle.get(key, "") if isinstance(bundle, dict) else ""


def _resolve_brand_capabilities(caps: dict, bundle_platform: str, CP) -> None:
  """Set brand-specific capabilities from bundle platform or CarParams fallback.

  Bundle (manual car selection) is a pre-fingerprint approximation.
  CarParams (auto-fingerprint) is the authoritative post-fingerprint source.
  Mirrors the per-brand update_settings() logic in device UI layouts.
  """
  brand = caps["brand"]

  if brand == "hyundai":
    if bundle_platform:
      try:
        unsupported = set().union(*UNSUPPORTED_LONGITUDINAL_CAR.values())
        caps["hyundai_alpha_long_available"] = HYUNDAI_CAR[bundle_platform] not in unsupported
      except KeyError:
        cloudlog.exception(f"capabilities: unknown hyundai platform {bundle_platform!r}")
    elif CP is not None:
      caps["hyundai_alpha_long_available"] = bool(CP.alphaLongitudinalAvailable)

  elif brand == "subaru":
    if bundle_platform:
      try:
        flags = SUBARU_CAR[bundle_platform].config.flags
        caps["subaru_has_sng"] = not bool(flags & (SubaruFlags.GLOBAL_GEN2 | SubaruFlags.HYBRID))
        caps["has_stop_and_go"] = caps["subaru_has_sng"]
      except KeyError:
        cloudlog.exception(f"capabilities: unknown subaru platform {bundle_platform!r}")
    elif CP is not None:
      caps["subaru_has_sng"] = not bool(CP.flags & (SubaruFlags.GLOBAL_GEN2 | SubaruFlags.HYBRID))
      caps["has_stop_and_go"] = caps["subaru_has_sng"]


def _is_tesla_preap(CP, bundle_platform: str) -> bool:
  """Typed Pre-AP discriminator from PREAP_PLATFORM bundle/fingerprint. Never HAS_VEHICLE_BUS."""
  return is_preap_ui_platform(bundle_platform, CP)


def _resolve_tesla_preap_capabilities(caps: dict, CP, CP_SP, params=None) -> None:
  del params  # boot snapshot only; staged Params do not change active brake visibility
  if not caps["tesla_preap"]:
    return

  flags = int(getattr(CP_SP, "flags", 0) or 0) if CP_SP is not None else 0
  caps["tesla_preap_pedal"] = bool(flags & TeslaFlagsSP.PREAP_PEDAL_PRESENT)
  caps["tesla_preap_radar"] = bool(flags & TeslaFlagsSP.PREAP_RADAR_PRESENT)
  caps["tesla_preap_pedal_calib"] = bool(flags & TeslaFlagsSP.PREAP_PEDAL_CALIB_AVAILABLE)

  # Stalk pull is the only Pre-AP lateral request. Brake-mode settings stay applicable.
  caps["tesla_preap_active_mode"] = "independent"
  caps["tesla_preap_independent_brake"] = True

  if caps["tesla_preap_pedal"] and CP is not None and bool(CP.openpilotLongitudinalControl) and not bool(CP.pcmCruise):
    caps["tesla_preap_longitudinal_path"] = "pedal"
  else:
    caps["tesla_preap_longitudinal_path"] = "stock_di"

  if not caps["tesla_preap_pedal"]:
    caps["tesla_preap_pedal_health"] = "none"
  elif not caps["tesla_preap_pedal_calib"]:
    caps["tesla_preap_pedal_health"] = "uncalibrated"
  else:
    caps["tesla_preap_pedal_health"] = "ok"

  if not caps["tesla_preap_radar"]:
    caps["tesla_preap_radar_health"] = "none"
  elif CP is not None and bool(CP.radarUnavailable):
    caps["tesla_preap_radar_health"] = "unconfigured"
  else:
    caps["tesla_preap_radar_health"] = "ok"


def generate_capabilities(params: Params | None = None) -> dict:
  """Generate a SettingsCapabilities dict from CarParams + boolean params.

  When CarPlatformBundle is present, brand and platform come from the bundle
  (mirrors Raylib). CarParams* deserialization is the fallback before the bundle
  is written (early after first pairing).
  """
  params = params or Params()

  caps: dict = {field: CAPABILITY_DEFAULTS.get(field, False) for field in CAPABILITY_FIELDS}

  # Wire-protocol version is always set explicitly.
  caps["protocol_version"] = PROTOCOL_VERSION

  # Hardware + boolean params (no CarParams dependency)
  caps["device_type"] = HARDWARE.get_device_type()
  caps["is_release"] = False  # params.get_bool("IsReleaseBranch")
  caps["is_sp_release"] = params.get_bool("IsReleaseSpBranch")
  caps["is_development"] = params.get_bool("IsDevelopmentBranch")
  caps["stock_longitudinal"] = params.get_bool("ToyotaEnforceStockLongitudinal")

  bundle = params.get("CarPlatformBundle")
  bundle_brand = _bundle_field(bundle, "brand")
  bundle_platform = _bundle_field(bundle, "platform")

  # Bundle-first brand resolution; CP is fallback only.
  if bundle_brand:
    caps["brand"] = bundle_brand

  # CarParams-derived capabilities
  CP = None
  CP_bytes = params.get("CarParamsPersistent")
  if CP_bytes is not None:
    try:
      CP = messaging.log_from_bytes(CP_bytes, car.CarParams)
      caps["alpha_long_available"] = bool(CP.alphaLongitudinalAvailable)
      if CP.alphaLongitudinalAvailable:
        caps["has_longitudinal_control"] = params.get_bool("AlphaLongitudinalEnabled")
      else:
        caps["has_longitudinal_control"] = bool(CP.openpilotLongitudinalControl)
      # CP.steerControlType is the physical control mode (angle / torque).
      # CP.lateralTuning.which() returns the tuning class (pid / torque / indi)
      # which is a separate concept and is not interchangeable.
      caps["steer_control_type"] = str(CP.steerControlType)
      caps["torque_allowed"] = CP.steerControlType != car.CarParams.SteerControlType.angle
      if not caps["brand"] and CP.brand:
        caps["brand"] = str(CP.brand)
      caps["pcm_cruise"] = bool(CP.pcmCruise)
      caps["enable_bsm"] = bool(CP.enableBsm)
      # Generic SnG fallback. Brand-specific opaque flags below override.
      caps["has_stop_and_go"] = bool(CP.openpilotLongitudinalControl)
    except Exception:
      CP = None
      cloudlog.exception("capabilities: failed to deserialize CarParamsPersistent")

  # CarParamsSP-derived capabilities
  CP_SP = None
  CP_SP_bytes = params.get("CarParamsSPPersistent")
  if CP_SP_bytes is not None:
    try:
      CP_SP = messaging.log_from_bytes(CP_SP_bytes, custom.CarParamsSP)
      caps["icbm_available"] = bool(CP_SP.intelligentCruiseButtonManagementAvailable)
      caps["has_icbm"] = bool(CP_SP.intelligentCruiseButtonManagementAvailable) and params.get_bool("IntelligentCruiseButtonManagement")
      caps["tesla_has_vehicle_bus"] = bool(CP_SP.flags & TeslaFlagsSP.HAS_VEHICLE_BUS)
    except Exception:
      CP_SP = None
      cloudlog.exception("capabilities: failed to deserialize CarParamsSPPersistent")

  caps["tesla_preap"] = _is_tesla_preap(CP, bundle_platform)
  _resolve_tesla_preap_capabilities(caps, CP, CP_SP, params)

  _resolve_brand_capabilities(caps, bundle_platform, CP)

  return caps


def generate_capabilities_json(params: Params | None = None) -> str:
  """Generate SettingsCapabilities as a JSON string."""
  return json.dumps(generate_capabilities(params), separators=(",", ":"))
