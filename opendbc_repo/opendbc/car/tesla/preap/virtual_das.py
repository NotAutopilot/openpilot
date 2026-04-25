"""Virtual DAS — feedforward-dominant cascaded longitudinal controller.

Replaces the open-loop compute_pedal_command() path with a jerk-limited,
feedback-corrected controller for smooth pedal actuation without DAS hardware.
"""

import json
import math
import os

import numpy as np
from numpy import clip, interp

from opendbc.car.common.filter_simple import FirstOrderFilter, HighPassFilter
from opendbc.car.common.pid import PIDController
from opendbc.car.tesla.preap.constants import (
  VDAS_INNER_K_BP, VDAS_INNER_KP_V, VDAS_INNER_KI_V,
  VDAS_FUTURE_T_BP, VDAS_FUTURE_T_V,
  VDAS_AEGO_FILTER_RC,
)
from opendbc.car.tesla.preap.ff_table_default import (
  SPEED_BP as FF_SPEED_BP,
  ACCEL_BP as FF_ACCEL_BP,
  DEFAULT_TABLE as FF_DEFAULT_TABLE,
)
from opendbc.car.tesla.preap.nap_conf import (
  nap_conf,
  PEDAL_DI_MIN, PEDAL_DI_ZERO,
  PEDAL_BP, PEDAL_MAX_VALUES,
  ACCEL_MAX, REGEN_MAX,
)
from opendbc.car.tesla.pedal.controller import (
  get_zero_torque,
  PEDAL_RAMP_RATE_UP, PEDAL_RAMP_RATE_DOWN,
)


FF_TABLE_PATH = "/data/vdas_ff_table.json"

# Inner PID error deadband: errors below this threshold are zeroed before
# entering the PID. Prevents integral accumulation from sensor noise and
# MPC jitter near steady-state. Applied to the error input, not the output,
# so there's no discontinuity in the correction signal.
PID_ERROR_DEADBAND = 0.1  # m/s²

GRAVITY = 9.81  # m/s²
PITCH_LP_RC = 0.5   # low-pass filter RC for steady-state grade (seconds)
PITCH_HP_RC1 = 0.1  # high-pass inner RC for transient grade detection
PITCH_HP_RC2 = 1.0  # high-pass outer RC
MAX_PITCH_COMPENSATION = 1.5  # m/s² — clamp transient compensation


class GradeEstimator:
  """Estimates road grade from IMU pitch and compensates the controller.

  Uses a low-pass filter on pitch for the steady-state grade component
  (subtracted from a_ego so the inner PID doesn't fight gravity) and
  a high-pass filter for transient grade changes (added to feedforward
  so the controller anticipates crests and dips).

  Follows the same pattern as Toyota's carcontroller.py lines 68-69, 204-235.
  """

  def __init__(self, dt: float = 0.02):
    self.pitch_lp = FirstOrderFilter(0.0, PITCH_LP_RC, dt)
    self.pitch_hp = HighPassFilter(0.0, PITCH_HP_RC1, PITCH_HP_RC2, dt)

  def update(self, orientation_ned: list) -> tuple:
    """Update filters with current pitch.

    Args:
      orientation_ned: [roll, pitch, yaw] from CC.orientationNED.
                       Empty list if not yet calibrated.

    Returns:
      (grade_accel, pitch_compensation):
        grade_accel: steady-state gravitational component along road (m/s²).
                     Positive = downhill (gravity accelerates the car).
        pitch_compensation: transient feedforward bump for grade changes (m/s²).
    """
    if len(orientation_ned) < 2:
      return 0.0, 0.0

    pitch = orientation_ned[1]
    self.pitch_lp.update(pitch)
    self.pitch_hp.update(pitch)

    grade_accel = math.sin(self.pitch_lp.x) * GRAVITY
    pitch_compensation = float(clip(
      math.sin(self.pitch_hp.x) * GRAVITY,
      -MAX_PITCH_COMPENSATION, MAX_PITCH_COMPENSATION))

    return grade_accel, pitch_compensation

  def reset(self):
    self.pitch_lp.x = 0.0
    self.pitch_hp.x = 0.0
    self.pitch_hp._f1.x = 0.0
    self.pitch_hp._f2.x = 0.0


class JerkLimiter:
  """S-curve rate limiter on acceleration commands.

  Bounds the rate of change of acceleration (jerk) to j_max,
  preventing discontinuous inputs from reaching the pedal controller.
  """

  def __init__(self, j_max: float = 2.5, dt: float = 0.02):
    self.j_max = j_max
    self.dt = dt
    self.a_limited = 0.0

  def update(self, a_cmd: float) -> float:
    da_max = self.j_max * self.dt
    self.a_limited += float(clip(a_cmd - self.a_limited, -da_max, da_max))
    return self.a_limited

  def reset(self, a_init: float = 0.0):
    self.a_limited = a_init


class FeedforwardModel:
  """2D lookup table mapping (speed, accel) → pedal_di.

  Loads from a JSON file if available (produced by generate_ff_table.py),
  otherwise falls back to the default table computed from the legacy
  3-breakpoint interpolation. Zero-torque offset is applied at runtime.
  """

  def __init__(self, table_path: str = FF_TABLE_PATH):
    self.speed_bp = list(FF_SPEED_BP)
    self.accel_bp = list(FF_ACCEL_BP)
    self.table = [list(row) for row in FF_DEFAULT_TABLE]
    self._load_override(table_path)

  def _load_override(self, path: str):
    if not os.path.isfile(path):
      return
    try:
      with open(path) as f:
        data = json.load(f)
      self.speed_bp = data['speed_bp']
      self.accel_bp = data['accel_bp']
      self.table = data['table']
    except (json.JSONDecodeError, KeyError, TypeError):
      from opendbc.car.carlog import carlog
      carlog.warning("vdas: failed to load FF table from %s, using defaults", path)

  def get(self, a_cmd: float, v_ego: float, zero_torque_di: float) -> float:
    """Look up pedal_di for a given (speed, accel) pair.

    The table is stored with zero_torque_di=0. At runtime the learned
    zero-torque offset shifts the result: fully applied for positive
    accel (gas side), linearly blended to zero at max regen.
    """
    # Bilinear interpolation: speed (outer), accel (inner)
    si = float(interp(v_ego, self.speed_bp, range(len(self.speed_bp))))
    si_lo = int(si)
    si_hi = min(si_lo + 1, len(self.speed_bp) - 1)
    sf = si - si_lo

    di_lo = float(interp(a_cmd, self.accel_bp, self.table[si_lo]))
    di_hi = float(interp(a_cmd, self.accel_bp, self.table[si_hi]))
    base_di = di_lo + sf * (di_hi - di_lo)

    # Zero-torque shift: full at accel=0, fades to zero at both extremes.
    # Reproduces the legacy interp where zt is the midpoint, not an additive offset.
    if a_cmd < 0:
      blend = float(clip((a_cmd - REGEN_MAX) / (0.0 - REGEN_MAX), 0.0, 1.0))
    else:
      blend = float(1.0 - a_cmd / ACCEL_MAX)
    base_di += zero_torque_di * blend

    return base_di


class VirtualDAS:
  """Cascaded longitudinal controller for Pre-AP Tesla pedal control.

  Jerk limiter smooths the input, feedforward maps accel→DI, inner PID
  corrects residual error using predicted future acceleration.
  """

  def __init__(self, dt: float = 0.02):
    self.dt = dt
    self.jerk_limiter = JerkLimiter(j_max=2.5, dt=dt)
    self.ff_model = FeedforwardModel()
    self.grade_estimator = GradeEstimator(dt=dt)
    self.prev_pedal_di = 0.0

    self.inner_pid = PIDController(
      k_p=(VDAS_INNER_K_BP, VDAS_INNER_KP_V),
      k_i=(VDAS_INNER_K_BP, VDAS_INNER_KI_V),
      k_f=0.0,
      pos_limit=PEDAL_RAMP_RATE_UP,
      neg_limit=-PEDAL_RAMP_RATE_DOWN,
      rate=1.0 / dt,
    )
    self.a_ego_filter = FirstOrderFilter(0.0, VDAS_AEGO_FILTER_RC, dt)
    self.prev_a_ego_filtered = 0.0

  def update(self, a_cmd: float, v_ego: float, prev_pedal_di: float,
             a_ego: float = 0.0, freeze_integrator: bool = False,
             orientation_ned: list | None = None) -> float:
    """Compute pedal DI from acceleration command.

    Args:
      a_cmd: desired acceleration in m/s² (from longcontrol.py)
      v_ego: current vehicle speed in m/s
      prev_pedal_di: previous output DI (for rate limiting backstop)
      a_ego: measured longitudinal acceleration in m/s²
      freeze_integrator: True during engage grace period
      orientation_ned: [roll, pitch, yaw] from CC.orientationNED, or None

    Returns:
      pedal_di: output in DI units (caller converts to voltage via di_to_pedal)
    """
    a_limited = self.jerk_limiter.update(a_cmd)

    grade_accel, pitch_compensation = self.grade_estimator.update(
      orientation_ned if orientation_ned is not None else [])

    ff_di = self._feedforward(a_limited, v_ego)
    ff_di += pitch_compensation

    # Subtract grade from a_ego so the PID doesn't fight gravity
    a_ego_corrected = a_ego - grade_accel

    a_ego_filtered = self.a_ego_filter.update(a_ego_corrected)
    j_ego = (a_ego_filtered - self.prev_a_ego_filtered) / self.dt
    self.prev_a_ego_filtered = a_ego_filtered

    future_t = float(interp(v_ego, VDAS_FUTURE_T_BP, VDAS_FUTURE_T_V))
    a_ego_future = a_ego_filtered + j_ego * future_t

    error = a_limited - a_ego_future
    if abs(error) < PID_ERROR_DEADBAND:
      error = 0.0

    pid_correction = float(self.inner_pid.update(
      error, speed=v_ego, freeze_integrator=freeze_integrator))

    pedal_di = ff_di + pid_correction

    pedal_profile = nap_conf.get_pedal_profile_values()
    max_pedal_value = float(interp(v_ego, PEDAL_BP, pedal_profile))
    pedal_di = float(clip(pedal_di, PEDAL_DI_MIN, max_pedal_value))

    pedal_di = self._rate_limit(pedal_di, prev_pedal_di)

    self.prev_pedal_di = pedal_di
    return pedal_di

  def reset(self, a_init: float = 0.0, pedal_di_init: float = 0.0):
    """Reset all internal state on engage transition."""
    self.jerk_limiter.reset(a_init)
    self.inner_pid.reset()
    self.grade_estimator.reset()
    self.a_ego_filter.x = 0.0
    self.prev_a_ego_filtered = 0.0
    self.prev_pedal_di = pedal_di_init

  def _feedforward(self, a_cmd: float, v_ego: float) -> float:
    """Map acceleration to pedal DI via 2D lookup table."""
    zero_torque_di = get_zero_torque().get(v_ego)
    pedal_profile = nap_conf.get_pedal_profile_values()
    max_pedal_value = float(interp(v_ego, PEDAL_BP, pedal_profile))

    pedal_di = self.ff_model.get(a_cmd, v_ego, zero_torque_di)

    return float(clip(pedal_di, PEDAL_DI_MIN, max_pedal_value))

  def _rate_limit(self, pedal_di: float, prev_pedal_di: float) -> float:
    """Safety backstop: asymmetric DI rate limit."""
    return float(clip(
      pedal_di,
      prev_pedal_di - PEDAL_RAMP_RATE_DOWN,
      prev_pedal_di + PEDAL_RAMP_RATE_UP,
    ))
