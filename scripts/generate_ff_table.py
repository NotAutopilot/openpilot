#!/usr/bin/env python3
"""Generate a data-driven feedforward table for VirtualDAS.

Reads drive logs and builds a 2D lookup table mapping (speed, accel) → pedal_di
by observing what pedal positions produced what accelerations at each speed.

Usage:
  python scripts/generate_ff_table.py /path/to/*.rlog.zst

Output: /data/vdas_ff_table.json (or --output path)

The script:
  1. Extracts (v_ego, actuators.accel, carState.aEgo) from logs where
     longitudinal control was active and the car was moving
  2. Bins observations by speed and commanded accel
  3. For each bin, finds the median pedal_di that was being sent
  4. Builds the inverse: (speed, desired_accel) → pedal_di
  5. Writes JSON in the format FeedforwardModel expects
"""

import argparse
import json
import sys

import numpy as np

from openpilot.tools.lib.logreader import LogReader


MIN_SPEED = 3.0  # m/s

SPEED_BP = [0.0, 5.0, 12.0, 20.0, 30.0, 40.0]
ACCEL_BP = [-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5]

# Delay between command and response — shift a_ego backward by this many seconds
ACTUATOR_DELAY = 0.4


def extract_data(route_or_file: str):
  """Extract time-aligned longitudinal data from a log."""
  lr = LogReader(route_or_file)

  cmd_data = []  # (time, accel_cmd)
  ego_data = []  # (time, a_ego, v_ego)

  for msg in lr:
    t = msg.logMonoTime / 1e9

    if msg.which() == 'carControl' and msg.carControl.longActive:
      cmd_data.append((t, float(msg.carControl.actuators.accel)))

    elif msg.which() == 'carState':
      v = float(msg.carState.vEgo)
      if v > MIN_SPEED:
        ego_data.append((t, float(msg.carState.aEgo), v))

  return cmd_data, ego_data


def build_table(all_cmd_data, all_ego_data):
  """Build the 2D feedforward table from collected data.

  Strategy: for each (speed_bin, accel_cmd_bin), find what a_ego resulted
  after the actuator delay. Then invert: the accel_cmd that produced a
  given a_ego at a given speed tells us what pedal position to use for
  that desired acceleration.

  Since we don't have direct pedal_di in the logs, we use accel_cmd as
  a proxy for the feedforward output. The table then maps
  (speed, desired_accel) → accel_cmd_that_achieved_it, which is what the
  feedforward should output (the current linear interp's job).
  """
  if not all_cmd_data or not all_ego_data:
    return None

  cmd_t = np.array([d[0] for d in all_cmd_data])
  cmd_a = np.array([d[1] for d in all_cmd_data])

  ego_t = np.array([d[0] for d in all_ego_data])
  ego_a = np.array([d[1] for d in all_ego_data])
  ego_v = np.array([d[2] for d in all_ego_data])

  # Resample cmd to ego timestamps (shifted by delay)
  cmd_at_ego = np.interp(ego_t - ACTUATOR_DELAY, cmd_t, cmd_a)

  # Bin by speed and commanded accel, collect achieved accel
  speed_edges = [(SPEED_BP[i] + SPEED_BP[i+1]) / 2 for i in range(len(SPEED_BP) - 1)]
  accel_edges = [(ACCEL_BP[i] + ACCEL_BP[i+1]) / 2 for i in range(len(ACCEL_BP) - 1)]

  from opendbc.car.tesla.preap.ff_table_default import DEFAULT_TABLE

  # For each (speed, accel_cmd) bin, compute a correction factor from
  # observed (commanded vs achieved) accel, then scale the default DI.
  table = []
  for si, s_target in enumerate(SPEED_BP):
    if si == 0:
      s_lo, s_hi = 0.0, speed_edges[0] if speed_edges else 100.0
    elif si == len(SPEED_BP) - 1:
      s_lo, s_hi = speed_edges[-1], 100.0
    else:
      s_lo, s_hi = speed_edges[si-1], speed_edges[si]

    s_mask = (ego_v >= s_lo) & (ego_v < s_hi)

    row = []
    for ai, a_target in enumerate(ACCEL_BP):
      if ai == 0:
        a_lo, a_hi = -10.0, accel_edges[0] if accel_edges else 10.0
      elif ai == len(ACCEL_BP) - 1:
        a_lo, a_hi = accel_edges[-1], 10.0
      else:
        a_lo, a_hi = accel_edges[ai-1], accel_edges[ai]

      a_mask = (cmd_at_ego >= a_lo) & (cmd_at_ego < a_hi)
      combined = s_mask & a_mask

      default_di = DEFAULT_TABLE[si][ai]
      if combined.sum() > 5:
        achieved = np.median(ego_a[combined])
        cmd_median = np.median(cmd_at_ego[combined])
        if abs(achieved) > 0.05 and abs(cmd_median) > 0.05:
          correction = cmd_median / achieved
        else:
          correction = 1.0
        row.append(round(float(default_di * correction), 2))
      else:
        row.append(default_di)

    table.append(row)

  return {
    'speed_bp': SPEED_BP,
    'accel_bp': ACCEL_BP,
    'table': table,
  }


def main():
  parser = argparse.ArgumentParser(description=__doc__,
                                   formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument('logs', nargs='+', help='rlog files or route identifiers')
  parser.add_argument('--output', default=FF_TABLE_PATH,
                      help=f'output JSON path (default: {FF_TABLE_PATH})')
  args = parser.parse_args()

  all_cmd, all_ego = [], []

  for log_path in args.logs:
    print(f"Reading: {log_path}")
    try:
      cmd, ego = extract_data(log_path)
      all_cmd.extend(cmd)
      all_ego.extend(ego)
      print(f"  cmd={len(cmd)}, ego={len(ego)}")
    except Exception as e:
      print(f"  Error: {e}")

  print(f"\nTotal: cmd={len(all_cmd)}, ego={len(all_ego)}")

  if not all_cmd or not all_ego:
    print("Not enough data to build table.")
    sys.exit(1)

  result = build_table(all_cmd, all_ego)
  if result is None:
    print("Failed to build table.")
    sys.exit(1)

  output = args.output
  with open(output, 'w') as f:
    json.dump(result, f, indent=2)

  print(f"\nTable written to: {output}")
  print(f"Speed breakpoints: {result['speed_bp']}")
  print(f"Accel breakpoints: {result['accel_bp']}")
  for i, row in enumerate(result['table']):
    print(f"  {result['speed_bp'][i]:5.1f} m/s: {row}")


FF_TABLE_PATH = "/data/vdas_ff_table.json"

if __name__ == '__main__':
  main()
