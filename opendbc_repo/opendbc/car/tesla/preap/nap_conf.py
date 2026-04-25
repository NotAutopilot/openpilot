"""Tesla Pre-AP configuration: pedal calibration, radar, and control mode settings."""

import json
import os
import tempfile

from opendbc.car.carlog import carlog
from opendbc.car.tesla.preap.nap_params import NAPParamKeys

try:
  from openpilot.common.params import Params
  _params = Params()
  _PARAMS_AVAILABLE = True
except ImportError:
  _PARAMS_AVAILABLE = False

carlog.info("nap_conf: _PARAMS_AVAILABLE=%s", _PARAMS_AVAILABLE)


CONFIG_FILE = "/data/nap_params.json"

DEFAULT_CONFIG = {
  'double_pull_window_ms': 400,
  'use_pedal': False,
  'pedal_calibrated': False,
  'accel_profile': 'Chill',
  'pedal_can_zero': False,
  'pedal_profile': 'P85+',
  'pedal_min': 0,
  'pedal_max': 1023,
  'pedal_calib_min': -3.0,
  'pedal_calib_max': 99.6,
  'pedal_calib_zero': 0.0,
  'pedal_calib_factor': 1.0,
  'radar_enabled': False,
  'radar_behind_nosecone': False,
  'radar_offset': 0.0,
}

# Pedal DI (Driver Intent) constants — internal representation before calibration
PEDAL_DI_MIN = -5       # Max regen (coasting hard)
PEDAL_DI_ZERO = 0       # Neutral
PEDAL_DI_PRESSED = 2    # "pedal pressed" threshold

ACCEL_MAX = 2.5         # m/s^2
REGEN_MAX = -1.5        # m/s^2
PEDAL_HYST_GAP = 1.0

# Speed-dependent max pedal (m/s breakpoints)
# mph:   0   11   27   44   67   90
PEDAL_BP = [0., 5., 12., 20., 30., 40.]

# Fixed pedal gain curve. Based on P85 with ~12% bump for more punch,
# well below S85 where oscillation starts. Same gain for all trims —
# the motor's torque curve handles the rest.
PEDAL_MAX_VALUES = [50., 58., 66., 74., 82., 90.]

# Planner acceleration envelopes by personality
ACCEL_LOOKUP_BP = [0.0, 1.3, 7.5, 15.0, 25.0, 40.0]  # m/s
ACCEL_MAX_PROFILES = {
  'Chill': [0.3, 0.7, 0.9, 0.7, 0.6, 0.5],
  'Standard': [0.3, 0.9, 1.2, 1.0, 0.8, 0.6],
  'MadMax': [0.3, 1.6, 1.9, 1.5, 1.2, 1.0],
}
ACCEL_MAX_DEFAULT = ACCEL_MAX_PROFILES['Chill']


def transform_di_to_pedal(val, pedal_zero, pedal_factor):
  """DI units -> pedal voltage. pedal_zero + (val - DI_ZERO) / factor"""
  if pedal_factor == 0:
    pedal_factor = 1.0
  return pedal_zero + (val - PEDAL_DI_ZERO) / pedal_factor


def transform_pedal_to_di(val, pedal_zero, pedal_factor):
  """Pedal voltage -> DI units. DI_ZERO + (val - pedal_zero) * factor"""
  return PEDAL_DI_ZERO + (val - pedal_zero) * pedal_factor


class NAPConf:
  """Pre-AP config backed by JSON file at /data/nap_params.json.
  Uses openpilot Params when available, falls back to JSON."""

  def __init__(self):
    self._cache = {}
    self._load()

  # Storage

  def _load(self):
    try:
      if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE) as f:
          loaded = json.load(f)
        self._cache = {**DEFAULT_CONFIG, **loaded}
      else:
        self._cache = DEFAULT_CONFIG.copy()
        self._save()
    except Exception:
      self._cache = DEFAULT_CONFIG.copy()

  def _save(self):
    """Atomic write: temp file + rename to prevent corruption on power loss."""
    try:
      os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)
      fd, tmp_path = tempfile.mkstemp(
        dir=os.path.dirname(CONFIG_FILE),
        prefix='.nap_params_', suffix='.tmp')
      try:
        with os.fdopen(fd, 'w') as f:
          json.dump(self._cache, f, indent=2)
        os.replace(tmp_path, CONFIG_FILE)
      except Exception:
        try:
          os.unlink(tmp_path)
        except Exception:
          pass
        raise
    except Exception:
      pass

  def _get(self, key, default):
    return self._cache.get(key, default)

  def _put(self, key, value):
    self._cache[key] = value
    self._save()

  # Params helpers

  def _get_param_bool(self, param_key, json_key, default=False):
    if _PARAMS_AVAILABLE:
      return _params.get_bool(param_key)
    return self._get(json_key, default)

  def _put_param_bool(self, param_key, json_key, value):
    if _PARAMS_AVAILABLE:
      _params.put_bool_nonblocking(param_key, bool(value))
    self._put(json_key, bool(value))

  def _get_param_float(self, param_key, json_key, default):
    if _PARAMS_AVAILABLE:
      val = _params.get(param_key, return_default=True)
      return float(val) if val is not None else default
    return float(self._get(json_key, default))

  def _put_param_float(self, param_key, json_key, value):
    if _PARAMS_AVAILABLE:
      _params.put(param_key, float(value))
    self._put(json_key, float(value))

  # Bool properties

  @property
  def use_pedal(self):
    return self._get_param_bool(NAPParamKeys.PEDAL_ENABLED, 'use_pedal')

  @use_pedal.setter
  def use_pedal(self, value):
    self._put_param_bool(NAPParamKeys.PEDAL_ENABLED, 'use_pedal', value)

  @property
  def radar_enabled(self):
    return self._get_param_bool(NAPParamKeys.RADAR_ENABLED, 'radar_enabled')

  @radar_enabled.setter
  def radar_enabled(self, value):
    self._put_param_bool(NAPParamKeys.RADAR_ENABLED, 'radar_enabled', value)

  @property
  def radar_behind_nosecone(self):
    return self._get_param_bool(NAPParamKeys.RADAR_BEHIND_NOSECONE, 'radar_behind_nosecone')

  @radar_behind_nosecone.setter
  def radar_behind_nosecone(self, value):
    self._put_param_bool(NAPParamKeys.RADAR_BEHIND_NOSECONE, 'radar_behind_nosecone', value)

  @property
  def pedal_calibrated(self):
    if not self._get_param_bool(NAPParamKeys.PEDAL_CALIB_DONE, 'pedal_calibrated'):
      return False
    # Defaults-sanity: the done-flag is only trustworthy if at least one calibration
    # value has moved off its DEFAULT_CONFIG value. Guards against a write-failure
    # that leaves the flag True but the values unpersisted (see 2026-04 drive-1).
    if self._calib_zero_raw() == DEFAULT_CONFIG['pedal_calib_zero'] and \
       self.pedal_factor == DEFAULT_CONFIG['pedal_calib_factor']:
      return False
    return True

  @pedal_calibrated.setter
  def pedal_calibrated(self, value):
    self._put_param_bool(NAPParamKeys.PEDAL_CALIB_DONE, 'pedal_calibrated', value)

  def _calib_zero_raw(self):
    """Raw stored pedal_calib_zero (no coast-position transform applied)."""
    return self._get_param_float(NAPParamKeys.PEDAL_CALIB_ZERO, 'pedal_calib_zero', 0.0)

  @property
  def pedal_can_zero(self):
    if _PARAMS_AVAILABLE:
      bus = _params.get(NAPParamKeys.PEDAL_CAN_BUS, return_default=True)
      return bus == 0
    return self._get('pedal_can_zero', False)

  @pedal_can_zero.setter
  def pedal_can_zero(self, value):
    if _PARAMS_AVAILABLE:
      _params.put(NAPParamKeys.PEDAL_CAN_BUS, 0 if value else 2)
    self._put('pedal_can_zero', bool(value))

  # Engagement

  @property
  def double_pull_enabled(self):
    return True  # always on — safety requirement

  @property
  def double_pull_window_ms(self):
    # Hard cap at 400ms in code — overrides any stored value (e.g. an old
    # 750 left over in /data/nap_params.json before this change). 400 is
    # the longest cancel-on-single-pull delay that still feels prompt;
    # natural human double-pulls land in 250–400ms.
    return min(int(self._get('double_pull_window_ms', 400)), 400)

  @double_pull_window_ms.setter
  def double_pull_window_ms(self, value):
    self._put('double_pull_window_ms', max(300, min(1500, int(value))))

  @property
  def accel_profile(self):
    val = self._get('accel_profile', 'Chill')
    return val if val in ACCEL_MAX_PROFILES else 'Chill'

  @accel_profile.setter
  def accel_profile(self, value):
    if value in ACCEL_MAX_PROFILES:
      self._put('accel_profile', value)

  # Pedal calibration

  @property
  def pedal_min(self):
    return int(self._get('pedal_min', 0))

  @pedal_min.setter
  def pedal_min(self, value):
    self._put('pedal_min', int(value))

  @property
  def pedal_max(self):
    return int(self._get('pedal_max', 1023))

  @pedal_max.setter
  def pedal_max(self, value):
    self._put('pedal_max', int(value))

  @property
  def radar_offset(self):
    return self._get_param_float(NAPParamKeys.RADAR_OFFSET, 'radar_offset', 0.0)

  @radar_offset.setter
  def radar_offset(self, value):
    self._put_param_float(NAPParamKeys.RADAR_OFFSET, 'radar_offset', value)

  @property
  def pedal_calib_min(self):
    return self._get_param_float(NAPParamKeys.PEDAL_CALIB_MIN, 'pedal_calib_min', -3.0)

  @pedal_calib_min.setter
  def pedal_calib_min(self, value):
    self._put_param_float(NAPParamKeys.PEDAL_CALIB_MIN, 'pedal_calib_min', value)

  @property
  def pedal_calib_max(self):
    return self._get_param_float(NAPParamKeys.PEDAL_CALIB_MAX, 'pedal_calib_max', 99.6)

  @pedal_calib_max.setter
  def pedal_calib_max(self, value):
    self._put_param_float(NAPParamKeys.PEDAL_CALIB_MAX, 'pedal_calib_max', value)

  @property
  def pedal_zero(self):
    """Coast position: calib_zero - 1/factor"""
    if _PARAMS_AVAILABLE:
      calib_zero_val = _params.get(NAPParamKeys.PEDAL_CALIB_ZERO, return_default=True)
      calib_zero = float(calib_zero_val) if calib_zero_val is not None else 0.0
    else:
      calib_zero = float(self._get('pedal_calib_zero', 0.0))
    factor = self.pedal_factor
    if factor == 0:
      factor = 1.0
    return calib_zero - 1.0 / factor

  @pedal_zero.setter
  def pedal_zero(self, value):
    if _PARAMS_AVAILABLE:
      _params.put(NAPParamKeys.PEDAL_CALIB_ZERO, float(value))
    self._put('pedal_calib_zero', float(value))

  @property
  def pedal_factor(self):
    """Calibration scaling: 100.0 / (pedal_max - pedal_pressed)"""
    return self._get_param_float(NAPParamKeys.PEDAL_CALIB_FACTOR, 'pedal_calib_factor', 1.0)

  @pedal_factor.setter
  def pedal_factor(self, value):
    self._put_param_float(NAPParamKeys.PEDAL_CALIB_FACTOR, 'pedal_calib_factor', value)

  # Utilities

  @property
  def pedal_can_bus(self):
    return 0 if self.pedal_can_zero else 2

  def get_pedal_profile_values(self):
    return PEDAL_MAX_VALUES

  def get_accel_profile_values(self):
    return ACCEL_MAX_PROFILES.get(self.accel_profile, ACCEL_MAX_DEFAULT)

  def di_to_pedal(self, val):
    return transform_di_to_pedal(val, self.pedal_zero, self.pedal_factor)

  def pedal_to_di(self, val):
    return transform_pedal_to_di(val, self.pedal_zero, self.pedal_factor)

  def print_config(self):
    """Diagnostic dump to stdout (for CLI tools)."""
    print("=== Tesla Pre-AP Configuration ===")
    storage = "openpilot Params" if _PARAMS_AVAILABLE else CONFIG_FILE
    print(f"    Storage: {storage}")
    print("")
    print("  [CONTROL MODES]")
    print("    Double-Pull Mode:     ON (always enabled)")
    print("")
    print("  [LONGITUDINAL]")
    print(f"    Pedal Enabled:        {'ON' if self.use_pedal else 'OFF'}")
    print(f"    Pedal Calibrated:     {'YES' if self.pedal_calibrated else 'NO'}")
    print(f"    Accel Profile:        {self.accel_profile}")
    print("")
    print("  [PEDAL CALIBRATION]")
    print(f"    Pedal Min (raw):      {self.pedal_min}")
    print(f"    Pedal Max (raw):      {self.pedal_max}")
    print(f"    Pedal Calib Min:      {self.pedal_calib_min:.2f}")
    print(f"    Pedal Calib Max:      {self.pedal_calib_max:.2f}")
    print(f"    Pedal Zero:           {self.pedal_zero:.3f}")
    print(f"    Pedal Factor:         {self.pedal_factor:.3f}")
    print(f"    Pedal CAN Bus:        {self.pedal_can_bus}")
    print("")
    print("  [RADAR]")
    print(f"    Radar Enabled:        {'ON' if self.radar_enabled else 'OFF'}")
    print(f"    Behind Nosecone:      {'YES' if self.radar_behind_nosecone else 'NO'}")
    print(f"    Radar Offset:         {self.radar_offset}m")
    print("")
    print("==================================")

  def get_all_params(self):
    return {
      'double_pull_window_ms': self.double_pull_window_ms,
      'use_pedal': self.use_pedal,
      'pedal_calibrated': self.pedal_calibrated,
      'accel_profile': self.accel_profile,
      'pedal_min': self.pedal_min,
      'pedal_max': self.pedal_max,
      'pedal_calib_min': self.pedal_calib_min,
      'pedal_calib_max': self.pedal_calib_max,
      'pedal_zero': self.pedal_zero,
      'pedal_factor': self.pedal_factor,
      'pedal_can_bus': self.pedal_can_bus,
      'pedal_can_zero': self.pedal_can_zero,
      'radar_enabled': self.radar_enabled,
      'radar_behind_nosecone': self.radar_behind_nosecone,
      'radar_offset': self.radar_offset,
    }

  def reset_to_defaults(self):
    self._cache = DEFAULT_CONFIG.copy()
    self._save()

  def reload(self):
    self._load()


nap_conf = NAPConf()
