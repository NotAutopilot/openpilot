"""
NAP (NotAutopilot) Parameter Keys

Single source of truth for all NAP param key names used by
the UI settings panel and Tesla Pre-AP car code.

Storage: openpilot Params system (params_keys.h)
"""


class NAPParamKeys:
  # Longitudinal Control
  ADAPTIVE_ACCEL = "NAPAdaptiveAccel"
  PEDAL_ENABLED = "NAPPedalEnabled"
  FOLLOW_DISTANCE = "NAPFollowDistance"
  PEDAL_PASSTHROUGH = "NAPPedalPassthrough"

  # Pedal Hardware
  PEDAL_PROFILE = "NAPPedalProfile"
  PEDAL_CAN_BUS = "NAPPedalCanBus"
  PEDAL_CALIB_DONE = "NAPPedalCalibDone"
  PEDAL_CALIB_MIN = "NAPPedalCalibMin"
  PEDAL_CALIB_MAX = "NAPPedalCalibMax"
  PEDAL_CALIB_FACTOR = "NAPPedalCalibFactor"
  PEDAL_CALIB_ZERO = "NAPPedalCalibZero"

  # Radar
  RADAR_ENABLED = "NAPRadarEnabled"
  RADAR_BEHIND_NOSECONE = "NAPRadarBehindNosecone"

  # iBooster / Braking
  IBOOSTER_ENABLED = "NAPiBoosterEnabled"
  BRAKE_FACTOR = "NAPBrakeFactor"

  # Advanced
  FORCE_PRE_AP = "NAPForcePreAP"


# Default values matching params_keys.h declarations
DEFAULTS = {
  NAPParamKeys.ADAPTIVE_ACCEL: True,
  NAPParamKeys.PEDAL_ENABLED: False,
  NAPParamKeys.PEDAL_PASSTHROUGH: False,
  NAPParamKeys.FOLLOW_DISTANCE: 4,
  NAPParamKeys.PEDAL_PROFILE: 4,
  NAPParamKeys.PEDAL_CAN_BUS: 2,
  NAPParamKeys.PEDAL_CALIB_DONE: False,
  NAPParamKeys.PEDAL_CALIB_MIN: -3.0,
  NAPParamKeys.PEDAL_CALIB_MAX: 99.6,
  NAPParamKeys.PEDAL_CALIB_FACTOR: 1.0,
  NAPParamKeys.PEDAL_CALIB_ZERO: 0.0,
  NAPParamKeys.RADAR_ENABLED: False,
  NAPParamKeys.RADAR_BEHIND_NOSECONE: False,
  NAPParamKeys.IBOOSTER_ENABLED: False,
  NAPParamKeys.BRAKE_FACTOR: 1.0,
  NAPParamKeys.FORCE_PRE_AP: False,
}
