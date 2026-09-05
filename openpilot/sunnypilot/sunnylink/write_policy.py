"""Remote write policy for sunnylink Param updates.

Local UI remains the only writer for hardware/calibration. Sunnylink may write
follow distance live and stage next-drive brake values. Conflicting or
unsupported Pre-AP writes are rejected. The active boot snapshot is not
mutated here; card/interface freeze it at boot.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any
from collections.abc import Iterable

from openpilot.sunnypilot.mads.helpers import is_mads_required


FOLLOW_DISTANCE_MIN = 1
FOLLOW_DISTANCE_MAX = 7
STEERING_MODE_MIN = 0
STEERING_MODE_MAX = 2
PREAP_MODE_INDEPENDENT = 0

# Hardware/calibration/radar and boot-lock keys stay local.
# NAPBrakeFactor is not registered; keep the name here so hostile remote
# writes still fail closed.
PREAP_LOCAL_ONLY_KEYS = frozenset({
  "NAPPedalEnabled",
  "NAPPedalCanBus",
  "NAPPedalProfile",
  "NAPPedalCalibDone",
  "NAPPedalCalibFactor",
  "NAPPedalCalibMin",
  "NAPPedalCalibMax",
  "NAPPedalCalibZero",
  "NAPRadarEnabled",
  "NAPRadarBehindNosecone",
  "NAPRadarOffset",
  "NAPForcePreAP",
  "NAPEpasRiskAccepted",
  "NAPScriptRunning",
  "NAPBrakeFactor",
  "NAPiBoosterEnabled",
  "NAPAdaptiveAccel",
  "tesla_preap_active_mode",
  "tesla_preap_longitudinal_path",
  "tesla_preap_pedal_health",
  "tesla_preap_radar_health",
})

# Device-owned identity/cache snapshots. Remote writes would spoof capabilities.
DEVICE_OWNED_IDENTITY_KEYS = frozenset({
  "CarParams",
  "CarParamsCache",
  "CarParamsPersistent",
  "CarParamsPrevRoute",
  "CarParamsSP",
  "CarParamsSPCache",
  "CarParamsSPPersistent",
  "CarPlatformBundle",
  "DongleId",
  "SunnylinkDongleId",
})

# Pre-AP hides these controls; remote writes are capability-conflicting.
PREAP_UNSUPPORTED_KEYS = frozenset({
  "TeslaCoopSteering",
  "TeslaMadsScreenButton",
  "Mads",
  "MadsMainCruiseAllowed",
  "MadsUnifiedEngagementMode",
})

# Dropped with the three-mode Pre-AP lateral picker. Nothing reads these.
RETIRED_KEYS = frozenset({
  "NAPLateralEngagementMode",
  "NAPLateralEngagementModeMigrated",
})

PREAP_LIVE_KEYS = frozenset({
  "NAPFollowDistance",
})

PREAP_NEXT_DRIVE_KEYS = frozenset({
  "MadsSteeringMode",
})

# Pre-AP-only remote keys. MadsSteeringMode remains a modern Tesla setting.
PREAP_ONLY_REMOTE_KEYS = frozenset({
  "NAPFollowDistance",
})


@dataclass(frozen=True)
class WriteDecision:
  allow: bool
  reason: str = ""


def _as_int(value) -> int | None:
  if isinstance(value, bool):
    return int(value)
  try:
    return int(value)
  except (TypeError, ValueError):
    return None


def _as_bool(value) -> bool:
  return value in (True, 1, "1", b"1", "true", "True")


def _staged_independent(caps: dict, staged_mode: int | None) -> bool:
  """Brake writes follow the evolving staged next-drive mode when present."""
  if staged_mode is not None:
    return int(staged_mode) == PREAP_MODE_INDEPENDENT
  return bool(caps.get("tesla_preap_independent_brake"))


def evaluate_param_write(key: str, value, caps: dict | None = None, *,
                         mads_required: bool | None = None,
                         staged_mode: int | None = None) -> WriteDecision:
  """Return whether a sunnylink saveParams write may proceed."""
  caps = caps or {}
  tesla_preap = bool(caps.get("tesla_preap"))

  if key in DEVICE_OWNED_IDENTITY_KEYS:
    return WriteDecision(False, "device_owned_identity")

  if key in PREAP_LOCAL_ONLY_KEYS:
    return WriteDecision(False, "local_only")

  if key in RETIRED_KEYS:
    return WriteDecision(False, "retired_key")

  if tesla_preap and key in PREAP_UNSUPPORTED_KEYS:
    return WriteDecision(False, "unsupported_on_preap")

  if key == "Mads" and (mads_required if mads_required is not None else is_mads_required()):
    if not _as_bool(value):
      return WriteDecision(False, "mads_required")

  if not tesla_preap:
    if key in PREAP_ONLY_REMOTE_KEYS:
      return WriteDecision(False, "unsupported_on_modern")
    return WriteDecision(True)

  if key == "NAPFollowDistance":
    parsed = _as_int(value)
    if parsed is None or not (FOLLOW_DISTANCE_MIN <= parsed <= FOLLOW_DISTANCE_MAX):
      return WriteDecision(False, "invalid_follow_distance")
    return WriteDecision(True)

  if key == "MadsSteeringMode":
    if not _staged_independent(caps, staged_mode):
      return WriteDecision(False, "brake_mode_not_independent")
    parsed = _as_int(value)
    if parsed is None or not (STEERING_MODE_MIN <= parsed <= STEERING_MODE_MAX):
      return WriteDecision(False, "invalid_steering_mode")
    return WriteDecision(True)

  if key in PREAP_LIVE_KEYS or key in PREAP_NEXT_DRIVE_KEYS:
    return WriteDecision(True)

  return WriteDecision(True)


def evaluate_ordered_writes(items: Iterable[tuple[str, Any]], caps: dict | None = None, *,
                            mads_required: bool | None = None,
                            staged_mode: int | None = None) -> list[tuple[str, Any, WriteDecision]]:
  """Evaluate a batch. Brake-mode authorization uses the caller's staged mode."""
  caps = caps or {}
  return [
    (
      key,
      value,
      evaluate_param_write(
        key, value, caps, mads_required=mads_required, staged_mode=staged_mode,
      ),
    )
    for key, value in items
  ]
