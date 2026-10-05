"""NAP per-car profiles: each car keeps its own learned state and driving prefs.

One comma moves between a Pre-AP and an AP1 Model S. card calls switch_profile()
right after fingerprinting and before it writes CarParams. calibrationd, paramsd,
torqued, and lagd all block on CarParams before loading their saved values, so
they always load the incoming car's.
"""
from cereal import car
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

PROFILE_ROOT = "/data/nap_profiles"
ACTIVE_PROFILE_KEY = "NAPActiveProfile"

PROFILE_FOR_PLATFORM = {
  "TESLA_MODEL_S_PREAP": "preap",
  "TESLA_MODEL_S_HW1": "ap1",
}
PROFILE_LABELS = {
  "preap": "Pre-AP Model S",
  "ap1": "AP1 Model S",
}

# Learned per car. Removed when the incoming car has none saved, so its learners start fresh.
# CarParamsPersistent becomes CarParamsPrevRoute in card; restoring it keeps paramsd/torqued/lagd
# from discarding the restored values as a car change.
LEARNED_KEYS = ("CalibrationParams", "LiveParametersV2", "LiveParameters",
                "LiveTorqueParameters", "LiveDelay", "CarParamsPersistent")
# Chosen per car. Kept as-is when the incoming car has none saved.
PREF_KEYS = ("ExperimentalMode", "LongitudinalPersonality", "IsLdwEnabled", "IsMetric")


def profile_for_platform(platform: str) -> str | None:
  return PROFILE_FOR_PLATFORM.get(platform)


def car_label(CP) -> str:
  """'AP1 Model S (auto)' for a NAP platform, '' otherwise."""
  profile = profile_for_platform(CP.carFingerprint)
  if profile is None:
    return ""
  mode = "auto" if CP.fingerprintSource == "can" else "forced"
  return f"{PROFILE_LABELS[profile]} ({mode})"


def _profile_store(root: str, profile: str) -> Params:
  return Params(f"{root}/{profile}")


def _last_route_profile(live: Params) -> str | None:
  raw = live.get("CarParamsPersistent")
  if raw is None:
    return None
  try:
    with car.CarParams.from_bytes(raw) as last_CP:
      return profile_for_platform(last_CP.carFingerprint)
  except Exception:
    return None


def _copy(src: Params, dst: Params, key: str) -> None:
  value = src.get(key)
  if value is None:
    dst.remove(key)
  else:
    dst.put(key, value, block=True)


def switch_profile(live: Params, platform: str, root: str = PROFILE_ROOT) -> str | None:
  """Make the live params hold this car's profile. Returns the profile name, or None
  for a non-NAP platform or on error (live params are then left as they were)."""
  new = profile_for_platform(platform)
  if new is None:
    return None

  try:
    # Before this feature existed there was no active profile: the last route's car owns the live state
    active = live.get(ACTIVE_PROFILE_KEY) or _last_route_profile(live)
    switching = active is not None and active != new

    if switching:
      outgoing = _profile_store(root, active)
      for key in LEARNED_KEYS + PREF_KEYS:
        _copy(live, outgoing, key)

    # Before restoring: a crash mid-restore must not stash mixed state into the outgoing profile next boot
    live.put(ACTIVE_PROFILE_KEY, new, block=True)

    if switching:
      incoming = _profile_store(root, new)
      for key in LEARNED_KEYS:
        _copy(incoming, live, key)
      for key in PREF_KEYS:
        value = incoming.get(key)
        if value is not None:
          live.put(key, value, block=True)
      cloudlog.warning(f"nap_profiles: switched {active} -> {new}")

    return new
  except Exception:
    cloudlog.exception("nap_profiles: profile switch failed")
    return None
