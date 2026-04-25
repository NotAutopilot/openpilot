import numpy as np

from opendbc.car import get_safety_config, structs, STD_CARGO_KG
from opendbc.car.carlog import carlog
from opendbc.car.tesla.preap.nap_conf import nap_conf

# Safety param flags matching tesla_preap.h (renumbered — LONG_CONTROL removed as dead code)
PREAP_FLAG_ENABLE_PEDAL = 1
PREAP_FLAG_RADAR_EMULATION = 2
PREAP_FLAG_RADAR_BEHIND_NOSECONE = 4
from opendbc.car.tesla.preap.constants import (
  ACCEL_PREAP_BP, ACCEL_PREAP_PROFILES,
  PEDAL_LONG_K_BP, PEDAL_LONG_KP_V, PEDAL_LONG_KI_V,
)

try:
  from openpilot.common.params import Params as _Params
  _params = _Params()
except ImportError:
  _params = None


def get_preap_accel_limits(current_speed):
  personality = 1
  if _params is not None:
    try:
      personality = int(_params.get("LongitudinalPersonality", return_default=True))
    except (TypeError, ValueError):
      pass
  profile = ACCEL_PREAP_PROFILES.get(personality, ACCEL_PREAP_PROFILES[1])
  a_max = float(np.interp(current_speed, ACCEL_PREAP_BP, profile))
  return -1.5, a_max


def get_preap_params(ret, fingerprint):
  # Build safety param flags for the standalone Pre-AP safety mode
  flags = 0
  use_pedal = nap_conf.use_pedal
  radar_enabled = nap_conf.radar_enabled
  radar_behind_nosecone = nap_conf.radar_behind_nosecone
  carlog.info("Pre-AP fingerprint: use_pedal=%s radar_enabled=%s behind_nosecone=%s",
              use_pedal, radar_enabled, radar_behind_nosecone)

  if use_pedal:
    flags |= PREAP_FLAG_ENABLE_PEDAL
  if radar_enabled:
    flags |= PREAP_FLAG_RADAR_EMULATION
  if radar_behind_nosecone:
    flags |= PREAP_FLAG_RADAR_BEHIND_NOSECONE

  ret.safetyConfigs = [
    get_safety_config(structs.CarParams.SafetyModel.teslaPreap, int(flags)),
  ]
  ret.radarUnavailable = not radar_enabled
  ret.steerControlType = structs.CarParams.SteerControlType.angle
  # Only claim longitudinal control when a Comma Pedal is installed. Without
  # the pedal NAP can't actuate throttle — stock Tesla CC handles speed via
  # the stalk-spoof engage/cancel in carcontroller.py, so op-long would just
  # run a planner whose output is silently discarded.
  ret.openpilotLongitudinalControl = use_pedal
  ret.pcmCruise = not use_pedal

  if use_pedal:
    ret.longitudinalTuning.kpBP = PEDAL_LONG_K_BP
    ret.longitudinalTuning.kpV = PEDAL_LONG_KP_V
    ret.longitudinalTuning.kiBP = PEDAL_LONG_K_BP
    ret.longitudinalTuning.kiV = PEDAL_LONG_KI_V
    # kf=1.0 feedforward: a_target passes through 1:1 to pedal mapping
    try:
      ret.longitudinalTuning.kf = 1.0
    except AttributeError:
      pass  # kf not available in all capnp schema versions
    ret.longitudinalActuatorDelay = 0.4

  # Legacy Model S steering and physical params
  ret.steerLimitTimer = 0.4
  ret.steerActuatorDelay = 0.1
  ret.steerAtStandstill = True
  ret.alphaLongitudinalAvailable = False
  ret.vEgoStopping = 0.1
  ret.vEgoStarting = 0.1
  ret.stoppingDecelRate = 1.0

  # Pre-AP Model S is physically the same platform as HW1/HW2/HW3 Model S.
  # Vehicle params (mass=2100, wheelbase=2.960, steerRatio=15.0) come from
  # CarSpecs in values.py — do NOT override them here to avoid double-counting
  # STD_CARGO_KG (the framework adds it automatically).
  # Confirmed by Lukas (xnor-tech): identical to HW3.
  ret.centerToFront = ret.wheelbase * 0.5

  return ret
