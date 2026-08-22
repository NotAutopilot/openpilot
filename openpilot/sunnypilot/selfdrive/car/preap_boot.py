"""Pre-AP boot selection, installer seeding, and one-time mode migration.

Card-boundary helpers. Opendbc vehicle modules must not import Params.
"""
from __future__ import annotations

from dataclasses import dataclass
import os
from typing import Any

from opendbc.car.tesla.preap.boot import PREAP_PLATFORM, is_preap_platform, parse_engagement_mode
from opendbc.car.tesla.preap.constants import STALK_DOUBLE_PULL_MS

assert STALK_DOUBLE_PULL_MS == 400


def is_preap_ui_platform(bundle_platform: str = "", CP=None) -> bool:
  """Settings visibility. Bundle platform wins when present; never HAS_VEHICLE_BUS."""
  if bundle_platform:
    return bundle_platform == PREAP_PLATFORM
  return bool(CP is not None and is_preap_platform(CP))



@dataclass(frozen=True)
class PreAPBootSelection:
  candidate: str | None
  skip_fw_query: bool | None
  source: str
  lock_preap: bool


def _truthy_force_preap(value) -> bool:
  return value in (True, 1, "1", b"1")


def resolve_preap_boot_selection(
  *,
  bundle_platform: str | None = None,
  env_fingerprint: str | None = None,
  nap_force_preap=None,
) -> PreAPBootSelection:
  """Card-boundary precedence before firmware query.

  1. explicit CarPlatformBundle / FINGERPRINT environment override
  2. persisted NAPForcePreAP = true
  3. normal fingerprint / FW flow

  Locked Pre-AP skips FW. Every other path leaves skip_fw_query unspecified so
  fingerprint retains the existing SKIP_FW_QUERY environment fallback.
  """
  explicit = (bundle_platform or None) or (env_fingerprint or None) or None
  if explicit:
    lock = explicit == PREAP_PLATFORM
    return PreAPBootSelection(candidate=explicit, skip_fw_query=True if lock else None, source="explicit", lock_preap=lock)
  if _truthy_force_preap(nap_force_preap):
    return PreAPBootSelection(candidate=PREAP_PLATFORM, skip_fw_query=True, source="nap_force_preap", lock_preap=True)
  return PreAPBootSelection(candidate=None, skip_fw_query=None, source="normal", lock_preap=False)


def seed_preap_installer(params, bundle_platform: str | None) -> bool:
  """Fresh Pre-AP installer/platform bundle seeds NAPForcePreAP=true. Shared SP stays absent/false."""
  if bundle_platform != PREAP_PLATFORM:
    return False
  if params.get("NAPForcePreAP") is not None:
    return False
  params.put_bool("NAPForcePreAP", True, block=True)
  return True


def _canonical_mode_readback_matches(written, mode: int) -> bool:
  """Require an actually present exact raw readback. Enum-zero parsing must not mask absence."""
  if written is None or written == "" or written == b"":
    return False
  try:
    return int(written) == int(mode)
  except (TypeError, ValueError):
    return parse_engagement_mode(written) == mode and written not in (None, "", b"")


def migrate_preap_engagement_mode(params) -> int:
  """Migrate Main/UEM once. Canonical write must succeed before the pair is considered retired."""
  canonical = params.get("NAPLateralEngagementMode")
  if canonical is not None:
    return parse_engagement_mode(canonical)

  if _truthy_force_preap(params.get("NAPLateralEngagementModeMigrated")):
    mode = 0
  else:
    main = params.get("MadsMainCruiseAllowed")
    uem = params.get("MadsUnifiedEngagementMode")
    # Absent legacy pair is not an explicit "both off" — default independent.
    if main is None and uem is None:
      mode = 0
    else:
      main_on = main not in (None, False, 0, "0", b"0", "")
      uem_on = uem not in (None, False, 0, "0", b"0", "")
      if main_on:
        mode = 0  # independent
      elif uem_on:
        mode = 1  # cruiseCoupled
      else:
        mode = 2  # longitudinalOnly

  params.put("NAPLateralEngagementMode", int(mode), block=True)
  written = params.get("NAPLateralEngagementMode")
  if not _canonical_mode_readback_matches(written, mode):
    return 0
  params.put_bool("NAPLateralEngagementModeMigrated", True, block=True)
  return mode


def force_mads_required(params) -> None:
  params.put_bool("Mads", True, block=True)

def reject_unsupported_cooperative_steering(params) -> None:
  params.put_bool("TeslaCoopSteering", False, block=True)


def snapshot_param_list(params) -> list[dict[str, Any]]:
  keys = [
    "NAPForcePreAP",
    "NAPLateralEngagementMode",
    "NAPPedalEnabled",
    "NAPPedalCanBus",
    "NAPPedalCalibDone",
    "NAPPedalCalibFactor",
    "NAPPedalCalibMin",
    "NAPPedalCalibMax",
    "NAPPedalCalibZero",
    "NAPPedalProfile",
    "NAPRadarEnabled",
    "NAPRadarBehindNosecone",
    "NAPRadarOffset",
    "NAPFollowDistance",
    "Mads",
    "MadsMainCruiseAllowed",
    "MadsUnifiedEngagementMode",
    "MadsSteeringMode",
    "TeslaMadsScreenButton",
    "TeslaCoopSteering",
  ]
  # Absent-only defaults for the unsaved Param schema. Present invalid values
  # (bus 7, min>=max) and present-but-unreadable files must pass through as
  # None so hardware snapshot can fail closed.
  snapshot_defaults = {
    "NAPPedalCanBus": 2,
    "NAPPedalCalibMin": -3.0,
    "NAPPedalCalibMax": 99.6,
  }
  rows: list[dict[str, Any]] = []
  for k in keys:
    value = params.get(k)
    if value is None and k in snapshot_defaults:
      get_param_path = getattr(params, "get_param_path", None)
      if not (callable(get_param_path) and os.path.isfile(get_param_path(k))):
        value = snapshot_defaults[k]
    rows.append({k: value})
  return rows


def resolve_card_boot(params, environ=None) -> tuple[PreAPBootSelection, str | None]:
  """Production card fingerprinting inputs. Does not inject a final get_car candidate."""
  if environ is None:
    environ = os.environ
  bundle = params.get("CarPlatformBundle") or {}
  if not isinstance(bundle, dict):
    bundle = {}
  bundle_platform = bundle.get("platform", None)
  if bundle_platform and bundle_platform != PREAP_PLATFORM:
    if _truthy_force_preap(params.get("NAPForcePreAP")):
      params.put_bool("NAPForcePreAP", False, block=True)
  seed_preap_installer(params, bundle_platform)
  selection = resolve_preap_boot_selection(
    bundle_platform=bundle_platform,
    env_fingerprint=environ.get("FINGERPRINT") or None,
    nap_force_preap=params.get("NAPForcePreAP"),
  )
  if selection.lock_preap:
    migrate_preap_engagement_mode(params)
    force_mads_required(params)
    reject_unsupported_cooperative_steering(params)
  fixed_fingerprint = selection.candidate if selection.candidate is not None else bundle_platform
  return selection, fixed_fingerprint


def new_preap_intent_epoch() -> int:
  value = int.from_bytes(os.urandom(8), "little")
  return value or 1
