#!/usr/bin/env python3
"""
Tesla Pre-AP Radar Test Tool (GUI version)
Shows all detected radar points to verify radar connectivity and operation.

Displays every tracked radar point from the Bosch radar with distance,
lateral offset, speed, and probability. Use this to verify the radar
is working and communicating properly.

Uses direct Panda access (stops pandad via ScriptRunner).
"""

import time
import sys

# ============================================
# Radar CAN Constants (Bosch, 32 points)
# ============================================
RADAR_BUS = 1           # CANBUS.radar
NUM_RADAR_POINTS = 32
RADAR_ADDR_START = 0x310
RADAR_ADDR_END = 0x36F

# ============================================
# Display
# ============================================
DISPLAY_INTERVAL = 0.5
PANDA_CONNECT_RETRIES = 5
PANDA_CONNECT_DELAY = 2.0

# Moving state names
MOVING_STATES = {
  0: "Unknown",
  1: "Moving",
  2: "Stopped",
  3: "Standing",
}


def p(msg):
  """Print with flush for real-time ScriptRunner display."""
  print(msg, flush=True)


def extract_signal(data, start_bit, length):
  """Extract a little-endian CAN signal from raw bytes."""
  value = 0
  for i in range(length):
    bit_pos = start_bit + i
    byte_idx = bit_pos // 8
    bit_idx = bit_pos % 8
    if byte_idx < len(data) and (data[byte_idx] & (1 << bit_idx)):
      value |= (1 << i)
  return value


def parse_radar_a(data):
  """Parse RadarPoint_A message."""
  return {
    'long_dist': extract_signal(data, 0, 12) * 0.0625,
    'long_speed': extract_signal(data, 12, 12) * 0.0625 - 128,
    'lat_dist': extract_signal(data, 24, 11) * 0.125 - 128,
    'prob_exist': extract_signal(data, 35, 5) * 3.125,
    'long_accel': extract_signal(data, 40, 10) * 0.03125 - 16,
    'valid': extract_signal(data, 55, 1),
    'meas': extract_signal(data, 61, 1),
    'tracked': extract_signal(data, 62, 1),
    'index': extract_signal(data, 63, 1),
  }


def parse_radar_b(data):
  """Parse RadarPoint_B message."""
  return {
    'lat_speed': extract_signal(data, 0, 10) * 0.125 - 64,
    'length': extract_signal(data, 10, 6) * 0.125,
    'dz': extract_signal(data, 16, 6) * 0.25 - 5,
    'moving_state': extract_signal(data, 22, 2),
    'index2': extract_signal(data, 63, 1),
  }


def connect_panda():
  """Connect to Panda with retries."""
  from panda import Panda

  for attempt in range(PANDA_CONNECT_RETRIES):
    try:
      panda = Panda()
      return panda
    except Exception as e:
      if attempt < PANDA_CONNECT_RETRIES - 1:
        p(f"  Panda not ready ({e}), retrying in {PANDA_CONNECT_DELAY}s...")
        time.sleep(PANDA_CONNECT_DELAY)
      else:
        raise


def main():
  p("=" * 60)
  p("RADAR TEST TOOL")
  p("=" * 60)
  p("")
  p("Showing ALL tracked radar points.")
  p("Use this to verify radar connectivity and operation.")
  p("Press Cancel when done.")
  p("=" * 60)

  panda = None
  try:
    p("\nConnecting to Panda...")
    panda = connect_panda()
    p("  Connected")

    panda.set_safety_mode(17)  # SAFETY_ALLOUTPUT for CAN forwarding
    p("  Safety mode set")

    points_a = {}
    points_b = {}

    start_time = time.time()
    last_display = 0
    msg_count = 0
    radar_msgs_seen = False

    p("\nListening for radar data on CAN bus 1...")

    while True:
      try:
        for msg in panda.can_recv():
          if len(msg) == 4:
            addr, _, dat, src = msg
          elif len(msg) == 3:
            addr, dat, src = msg
          else:
            continue

          if src != RADAR_BUS:
            continue

          if RADAR_ADDR_START <= addr <= RADAR_ADDR_END:
            radar_msgs_seen = True
            point_idx = (addr - RADAR_ADDR_START) // 2
            is_b = (addr - RADAR_ADDR_START) % 2 == 1
            data = bytes(dat) if not isinstance(dat, (bytes, bytearray)) else dat

            if is_b:
              points_b[point_idx] = parse_radar_b(data)
            else:
              points_a[point_idx] = parse_radar_a(data)
            msg_count += 1
      except Exception as e:
        if "not enough values" not in str(e):
          p(f"  CAN recv error: {e}")

      now = time.time()
      if now - last_display >= DISPLAY_INTERVAL:
        last_display = now
        elapsed = now - start_time

        if not radar_msgs_seen:
          p(f"  [{elapsed:.0f}s] No radar messages on bus {RADAR_BUS}...")
          time.sleep(0.5)
          continue

        # Collect all tracked points
        tracked = []
        total_with_data = 0
        for i in range(NUM_RADAR_POINTS):
          if i not in points_a:
            continue
          pa = points_a[i]
          total_with_data += 1
          if not pa['tracked']:
            continue
          if pa['long_dist'] <= 0 or pa['long_dist'] > 250:
            continue
          if pa['prob_exist'] < 50:
            continue
          pb = points_b.get(i, {})
          tracked.append((i, pa, pb))

        # Sort by distance
        tracked.sort(key=lambda x: x[1]['long_dist'])

        p(f"\n--- [{elapsed:.0f}s] {len(tracked)} tracked / {total_with_data} active ({msg_count} msgs) ---")
        if tracked:
          p(f"  {'#':>3s}  {'Dist(m)':>8s}  {'Lat(m)':>8s}  {'Spd(m/s)':>9s}  {'Accel':>7s}  {'Prob':>6s}  {'State':>8s}")
          p(f"  {'---':>3s}  {'-------':>8s}  {'------':>8s}  {'--------':>9s}  {'-----':>7s}  {'----':>6s}  {'-----':>8s}")
          for idx, pa, pb in tracked:
            state = MOVING_STATES.get(pb.get('moving_state', 0), "?")
            p(f"  {idx:3d}  {pa['long_dist']:8.2f}  {pa['lat_dist']:8.2f}  {pa['long_speed']:9.2f}  {pa['long_accel']:7.2f}  {pa['prob_exist']:5.1f}%  {state:>8s}")
        else:
          p("  (no tracked points)")

      time.sleep(0.01)

  except KeyboardInterrupt:
    p("\nStopped.")
  except Exception as e:
    p(f"\nERROR: {e}")
    import traceback
    traceback.print_exc()
    return 1
  finally:
    if panda is not None:
      try:
        panda.set_safety_mode(0)  # SAFETY_SILENT
        panda.close()
        p("\nPanda connection closed.")
      except Exception:
        pass

  p("\nRadar test ended.")
  return 0


if __name__ == "__main__":
  sys.exit(main())
