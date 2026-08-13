"""Pre-AP boot selection, installer seeding, and one-time mode migration.

Params live here (openpilot), never in opendbc vehicle modules.
"""
from __future__ import annotations

from dataclasses import dataclass
import os
from typing import Any

from opendbc.car.tesla.preap.boot import PREAP_PLATFORM, parse_engagement_mode
from opendbc.car.tesla.preap.constants import STALK_DOUBLE_PULL_MS

assert STALK_DOUBLE_PULL_MS == 400


@dataclass(frozen=True)
class PreAPBootSelection:
  candidate: str | None
  skip_fw_query: bool
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
  """
  explicit = (bundle_platform or None) or (env_fingerprint or None) or None
  if explicit:
    lock = explicit == PREAP_PLATFORM
    return PreAPBootSelection(candidate=explicit, skip_fw_query=lock, source="explicit", lock_preap=lock)
  if _truthy_force_preap(nap_force_preap):
    return PreAPBootSelection(candidate=PREAP_PLATFORM, skip_fw_query=True, source="nap_force_preap", lock_preap=True)
  return PreAPBootSelection(candidate=None, skip_fw_query=False, source="normal", lock_preap=False)


def seed_preap_installer(params, bundle_platform: str | None) -> bool:
  """Fresh Pre-AP installer/platform bundle seeds NAPForcePreAP=true. Shared SP stays absent/false."""
  if bundle_platform != PREAP_PLATFORM:
    return False
  if params.get("NAPForcePreAP") is not None:
    return False
  params.put_bool("NAPForcePreAP", True, block=True)
  return True


def migrate_preap_engagement_mode(params) -> int:
  """Migrate Main/UEM once. Canonical write must succeed before the pair is considered retired."""
  canonical = params.get("NAPLateralEngagementMode")
  if canonical is not None:
    return parse_engagement_mode(canonical)

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
  if parse_engagement_mode(written) != mode:
    return 0
  params.put_bool("NAPLateralEngagementModeMigrated", True, block=True)
  return mode


def force_mads_required(params) -> None:
  params.put_bool("Mads", True, block=True)


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
  ]
  return [{k: params.get(k)} for k in keys]


def new_preap_intent_epoch() -> int:
  value = int.from_bytes(os.urandom(8), "little")
  return value or 1
