#!/usr/bin/env python3
"""Measure the actuator delay between pedal command and measured acceleration.

Cross-correlates actuators.accel (commanded) with carState.aEgo (measured)
to find the time offset that best aligns the two signals. This gives the
true end-to-end delay: CAN TX → pedal firmware → inverter → motor → IMU.

Usage:
  # From a local rlog file
  python scripts/measure_actuator_delay.py /path/to/rlog.zst

  # From a route segment
  python scripts/measure_actuator_delay.py "dongle_id/date--time/segment"

  # Multiple segments for better statistics
  python scripts/measure_actuator_delay.py /path/to/rlog1.zst /path/to/rlog2.zst

The script filters for periods where longitudinal control is active and
the car is moving (v_ego > 3 m/s), since delay measurement is meaningless
at standstill or when the system is disengaged.
"""

import sys
import numpy as np

from openpilot.tools.lib.logreader import LogReader


MIN_SPEED = 3.0        # m/s — ignore standstill
MIN_ACTIVE_SECONDS = 5  # need at least this much active data
MAX_DELAY_S = 1.0       # search window for cross-correlation


def extract_signals(route_or_file: str):
  """Extract time-aligned accel command and measured accel from a log."""
  lr = LogReader(route_or_file)

  cmd_times, cmd_vals = [], []
  ego_times, ego_vals, ego_speeds = [], [], []

  for msg in lr:
    w = msg.which()
    t = msg.logMonoTime / 1e9

    if w == 'carControl':
      if msg.carControl.longActive:
        cmd_times.append(t)
        cmd_vals.append(float(msg.carControl.actuators.accel))

    elif w == 'carState':
      ego_times.append(t)
      ego_vals.append(float(msg.carState.aEgo))
      ego_speeds.append(float(msg.carState.vEgo))

  return (np.array(cmd_times), np.array(cmd_vals),
          np.array(ego_times), np.array(ego_vals), np.array(ego_speeds))


def resample_to_uniform(times, values, dt=0.01):
  """Resample irregular time series to uniform grid via linear interpolation."""
  if len(times) < 2:
    return np.array([]), np.array([])
  t_uniform = np.arange(times[0], times[-1], dt)
  v_uniform = np.interp(t_uniform, times, values)
  return t_uniform, v_uniform


def measure_delay(cmd_t, cmd_v, ego_t, ego_v, ego_speed, dt=0.01):
  """Cross-correlate command and measurement to find actuator delay.

  Returns (delay_seconds, correlation_peak, n_samples_used).
  """
  # Resample both to the same uniform grid
  t0 = max(cmd_t[0], ego_t[0])
  t1 = min(cmd_t[-1], ego_t[-1])
  if t1 - t0 < MIN_ACTIVE_SECONDS:
    return None, None, 0

  t_grid = np.arange(t0, t1, dt)
  cmd_resampled = np.interp(t_grid, cmd_t, cmd_v)
  ego_resampled = np.interp(t_grid, ego_t, ego_v)
  speed_resampled = np.interp(t_grid, ego_t, ego_speed)

  # Filter: only use samples where car is moving
  mask = speed_resampled > MIN_SPEED
  if mask.sum() < MIN_ACTIVE_SECONDS / dt:
    return None, None, int(mask.sum())

  cmd_masked = cmd_resampled.copy()
  ego_masked = ego_resampled.copy()
  cmd_masked[~mask] = 0.0
  ego_masked[~mask] = 0.0

  # Remove mean (cross-correlation needs zero-mean signals)
  cmd_masked -= cmd_masked[mask].mean()
  ego_masked -= ego_masked[mask].mean()

  # Normalize
  cmd_std = cmd_masked[mask].std()
  ego_std = ego_masked[mask].std()
  if cmd_std < 1e-6 or ego_std < 1e-6:
    return None, None, int(mask.sum())

  cmd_masked /= cmd_std
  ego_masked /= ego_std

  # Cross-correlate: shift ego relative to cmd
  max_lag_samples = int(MAX_DELAY_S / dt)
  lags = np.arange(0, max_lag_samples)
  correlations = np.zeros(len(lags))

  n = len(cmd_masked)
  for i, lag in enumerate(lags):
    if lag >= n:
      break
    overlap = min(n - lag, n)
    correlations[i] = np.mean(cmd_masked[:overlap] * ego_masked[lag:lag + overlap])

  best_idx = np.argmax(correlations)
  delay = lags[best_idx] * dt
  peak_corr = correlations[best_idx]

  return delay, peak_corr, int(mask.sum())


def main():
  if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(1)

  routes = sys.argv[1:]
  delays = []

  for route in routes:
    print(f"\nProcessing: {route}")
    try:
      cmd_t, cmd_v, ego_t, ego_v, ego_speed = extract_signals(route)
    except Exception as e:
      print(f"  Error reading: {e}")
      continue

    print(f"  Command samples: {len(cmd_t)}, Ego samples: {len(ego_t)}")

    if len(cmd_t) < 10 or len(ego_t) < 10:
      print("  Not enough data (need active longitudinal control)")
      continue

    delay, corr, n_used = measure_delay(cmd_t, cmd_v, ego_t, ego_v, ego_speed)

    if delay is None:
      print(f"  Could not measure delay (n_samples={n_used})")
      continue

    print(f"  Delay: {delay:.3f}s  (correlation: {corr:.3f}, samples: {n_used})")
    delays.append(delay)

  if delays:
    delays = np.array(delays)
    print(f"\n{'='*50}")
    print(f"Results across {len(delays)} segment(s):")
    print(f"  Mean delay:   {delays.mean():.3f}s")
    print(f"  Std dev:      {delays.std():.3f}s")
    print(f"  Min:          {delays.min():.3f}s")
    print(f"  Max:          {delays.max():.3f}s")
    print("\nCurrent setting: longitudinalActuatorDelay = 0.4s")
    print(f"Measured:        {delays.mean():.3f}s")
    if abs(delays.mean() - 0.4) > 0.05:
      print(f"  --> Consider updating to {delays.mean():.2f}s")
    else:
      print("  --> Current 0.4s is within tolerance")
  else:
    print("\nNo valid measurements. Need drive logs with active pedal control.")


if __name__ == '__main__':
  main()
