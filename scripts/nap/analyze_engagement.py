#!/usr/bin/env python3
"""Analyze pedal engagement transitions from openpilot route logs.

Usage:
  python scripts/nap/analyze_engagement.py /path/to/logs/

Shows engagement events, gas release transitions, and pedal state
to diagnose regen spikes and engagement failures.
"""
import glob
import sys
import bisect

from openpilot.tools.lib.logreader import LogReader


def analyze(log_dir):
  files = sorted(glob.glob(f'{log_dir}/*--rlog.zst'))
  if not files:
    print(f'No rlog.zst files found in {log_dir}')
    return

  print(f'Reading {len(files)} segments from {log_dir}...')

  cs_list = []
  cc_list = []

  for f in files:
    for msg in LogReader(f):
      t = msg.logMonoTime / 1e9
      w = msg.which()
      if w == 'carState':
        cs = msg.carState
        cs_list.append({
          't': t,
          'enabled': cs.cruiseState.enabled,
          'gas': cs.gasPressed,
          'vEgo': cs.vEgo,
          'aEgo': cs.aEgo,
        })
      elif w == 'carControl':
        cc = msg.carControl
        cc_list.append({
          't': t,
          'enabled': cc.enabled,
          'longActive': cc.longActive,
          'latActive': cc.latActive,
          'accel': cc.actuators.accel,
        })

  cs_list.sort(key=lambda x: x['t'])
  cc_list.sort(key=lambda x: x['t'])
  cc_times = [c['t'] for c in cc_list]

  print(f'carStates: {len(cs_list)} | carControls: {len(cc_list)}')
  print(f'Duration: {(cs_list[-1]["t"] - cs_list[0]["t"]) / 60:.1f} min')

  def get_cc(t):
    idx = bisect.bisect_left(cc_times, t)
    if idx < len(cc_list):
      return cc_list[idx]
    return None

  def print_window(center, before=3, after=20, label=""):
    start = max(0, center - before)
    end = min(center + after, len(cs_list))
    for j in range(start, end):
      c = cs_list[j]
      cc = get_cc(c['t'])
      la = f'{cc["longActive"]}' if cc else '--'
      ca = f'{cc["accel"]:+6.2f}' if cc else '  --  '
      marker = f' <-- {label}' if j == center else ''
      print(f'  t={c["t"]:.2f} en={c["enabled"]} gas={c["gas"]} '
            f'v={c["vEgo"]:5.1f} a={c["aEgo"]:+6.2f} '
            f'longAct={la:5s} ccAccel={ca}{marker}')

  prev_en = False
  prev_gas = False
  engage_num = 0
  release_num = 0

  for i, cs in enumerate(cs_list):
    if cs['enabled'] and not prev_en:
      engage_num += 1
      print(f'\n=== ENGAGE #{engage_num} at t={cs["t"]:.2f}s ===')
      print_window(i, label="ENGAGE")

    if prev_gas and not cs['gas'] and cs['enabled']:
      release_num += 1
      if release_num <= 20:
        print(f'\n=== GAS RELEASE #{release_num} at t={cs["t"]:.2f}s ===')
        print_window(i, label="RELEASE")

    prev_en = cs['enabled']
    prev_gas = cs['gas']

  print(f'\nTotal: {engage_num} engagements, {release_num} gas releases while engaged')


if __name__ == '__main__':
  if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(1)
  analyze(sys.argv[1])
