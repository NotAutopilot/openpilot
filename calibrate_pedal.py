#!/usr/bin/env python3
"""
Tesla Pre-AP Comma Pedal Calibration Tool
Ported from Tinkla's pedal_calibrator/calibratePedal.py

This simplified version reads raw sensor values for min/max calibration.
For full automated calibration with pedal control, see Tinkla's original.

Usage:
  SSH into comma device and run:
    cd /data/openpilot
    python calibrate_pedal.py
"""

import time
import sys

# ============================================
# Safety Banner
# ============================================
SAFETY_BANNER = """
╔══════════════════════════════════════════════════════════════════╗
║                    ⚠️  SAFETY WARNING ⚠️                          ║
╠══════════════════════════════════════════════════════════════════╣
║                                                                  ║
║  Before proceeding, ensure:                                      ║
║                                                                  ║
║    1. Vehicle is in PARK                                         ║
║    2. Parking brake is ENGAGED                                   ║
║    3. Engine/Motor is OFF (or car is not in Drive)               ║
║    4. You are in a safe, stationary location                     ║
║                                                                  ║
║  This tool will read the Comma Pedal sensor values.              ║
║  The car should NOT move during calibration.                     ║
║                                                                  ║
╚══════════════════════════════════════════════════════════════════╝
"""

# CAN Constants
GAS_SENSOR_ID = 0x552  # 1362 decimal - GAS_SENSOR from comma_pedal.dbc

# Calibration Constants
SAMPLE_COUNT = 50  # Number of samples to average
SAMPLE_TIMEOUT = 0.5  # Timeout per sample in seconds
DETECTION_TIMEOUT = 10.0  # Initial detection timeout


def get_pedal_value(can_sock, timeout=1.0, pedal_bus=None):
  """
  Read pedal sensor value from CAN.
  
  GAS_SENSOR message format (comma_pedal.dbc):
    SG_ INTERCEPTOR_GAS : 7|16@0+ (1,0) [0|1] "" EON
    
  This is big-endian: (dat[0] << 8) | dat[1]
  """
  import cereal.messaging as messaging
  
  start_time = time.time()
  while time.time() - start_time < timeout:
    messages = messaging.drain_sock(can_sock, wait_for_one=False)
    for m in messages:
      if m.which() == 'can':
        for c in m.can:
          if c.address == GAS_SENSOR_ID:
            # Filter by bus if specified
            if pedal_bus is not None and c.src not in (pedal_bus,):
              continue
            # Big-endian 16-bit value
            if len(c.dat) >= 2:
              return (c.dat[0] << 8) | c.dat[1]
    time.sleep(0.01)
  return None


def detect_pedal_bus(can_sock):
  """Detect which CAN bus the pedal is on (0 or 2)."""
  import cereal.messaging as messaging
  
  print("Detecting pedal CAN bus...")
  start_time = time.time()
  
  while time.time() - start_time < DETECTION_TIMEOUT:
    messages = messaging.drain_sock(can_sock, wait_for_one=False)
    for m in messages:
      if m.which() == 'can':
        for c in m.can:
          if c.address == GAS_SENSOR_ID:
            print(f"  Found pedal on CAN bus {c.src}")
            return c.src
    time.sleep(0.01)
  
  return None


def read_samples(can_sock, pedal_bus, count=SAMPLE_COUNT):
  """Read multiple samples and return list of values."""
  values = []
  errors = 0
  
  for i in range(count):
    val = get_pedal_value(can_sock, timeout=SAMPLE_TIMEOUT, pedal_bus=pedal_bus)
    if val is not None:
      values.append(val)
      print(".", end="", flush=True)
    else:
      errors += 1
      print("x", end="", flush=True)
    time.sleep(0.02)
  
  print("")
  return values, errors


def main():
  import cereal.messaging as messaging
  
  # Import here to avoid issues when running outside comma device
  try:
    from openpilot.common.params import Params
    params = Params()
  except ImportError:
    print("Error: Must run on comma device with openpilot installed.")
    return 1
  
  # Show safety warning
  print(SAFETY_BANNER)
  response = input("Type 'YES' to acknowledge safety warning and continue: ")
  if response.strip().upper() != "YES":
    print("Calibration cancelled.")
    return 1
  
  print("\n" + "=" * 50)
  print("Comma Pedal Calibration Tool")
  print("=" * 50)
  
  # Connect to CAN
  print("\nConnecting to CAN bus...")
  try:
    can_sock = messaging.sub_sock('can')
    # Give it a moment to connect
    time.sleep(0.5)
  except Exception as e:
    print(f"Error: Failed to subscribe to CAN: {e}")
    return 1
  
  # Detect pedal bus
  pedal_bus = detect_pedal_bus(can_sock)
  if pedal_bus is None:
    print("\nError: Could not detect Comma Pedal!")
    print("  - Check that the pedal is connected")
    print("  - Check CAN wiring")
    print("  - Verify pedal is powered (car ignition on)")
    return 1
  
  # Store pedal bus preference
  if pedal_bus == 0:
    params.put_bool("TeslaPedalCanZero", True)
    print("  Saved: TeslaPedalCanZero = True")
  else:
    params.put_bool("TeslaPedalCanZero", False)
    print("  Saved: TeslaPedalCanZero = False")
  
  # Initial reading
  val = get_pedal_value(can_sock, timeout=2.0, pedal_bus=pedal_bus)
  if val is None:
    print("\nError: Lost connection to pedal!")
    return 1
  print(f"\nCurrent pedal sensor value: {val}")
  
  # ============================================
  # Step 1: Calibrate Minimum (Released)
  # ============================================
  print("\n" + "-" * 50)
  print("STEP 1: Calibrate MINIMUM (Pedal Released)")
  print("-" * 50)
  print("  Completely release the accelerator pedal.")
  print("  Keep your foot OFF the pedal.")
  input("\n  Press Enter when ready...")
  
  print("\n  Reading samples", end="", flush=True)
  min_vals, min_errors = read_samples(can_sock, pedal_bus)
  
  if len(min_vals) < 10:
    print(f"\nError: Only got {len(min_vals)} valid samples (need at least 10)")
    return 1
  
  avg_min = int(sum(min_vals) / len(min_vals))
  print(f"\n  Samples: {len(min_vals)}, Errors: {min_errors}")
  print(f"  Min value: {min(min_vals)}")
  print(f"  Max value: {max(min_vals)}")
  print(f"  Average:   {avg_min}")
  print(f"\n  ✓ Recorded MINIMUM: {avg_min}")
  
  # ============================================
  # Step 2: Calibrate Maximum (Floored)
  # ============================================
  print("\n" + "-" * 50)
  print("STEP 2: Calibrate MAXIMUM (Pedal Floored)")
  print("-" * 50)
  print("  Press the accelerator pedal ALL THE WAY to the floor.")
  print("  Hold it there firmly.")
  input("\n  Press Enter when ready...")
  
  print("\n  Reading samples", end="", flush=True)
  max_vals, max_errors = read_samples(can_sock, pedal_bus)
  
  if len(max_vals) < 10:
    print(f"\nError: Only got {len(max_vals)} valid samples (need at least 10)")
    return 1
  
  avg_max = int(sum(max_vals) / len(max_vals))
  print(f"\n  Samples: {len(max_vals)}, Errors: {max_errors}")
  print(f"  Min value: {min(max_vals)}")
  print(f"  Max value: {max(max_vals)}")
  print(f"  Average:   {avg_max}")
  print(f"\n  ✓ Recorded MAXIMUM: {avg_max}")
  
  # ============================================
  # Validation
  # ============================================
  print("\n" + "-" * 50)
  print("VALIDATION")
  print("-" * 50)
  
  if avg_max <= avg_min:
    print("\n  ✗ ERROR: Maximum value is not greater than minimum!")
    print(f"    Min: {avg_min}, Max: {avg_max}")
    print("\n  Calibration FAILED. Please try again.")
    return 1
  
  # Check for reasonable range
  pedal_range = avg_max - avg_min
  if pedal_range < 100:
    print(f"\n  ⚠ WARNING: Pedal range seems small ({pedal_range})")
    print("    Expected range is typically 500-1000")
    response = input("    Continue anyway? (y/N): ")
    if response.strip().lower() != 'y':
      print("\n  Calibration cancelled.")
      return 1
  
  print(f"\n  ✓ Pedal range: {pedal_range} (looks reasonable)")
  
  # ============================================
  # Save Parameters
  # ============================================
  print("\n" + "-" * 50)
  print("SAVING CALIBRATION")
  print("-" * 50)
  
  params.put("TeslaPedalMin", str(avg_min))
  params.put("TeslaPedalMax", str(avg_max))
  params.put_bool("TeslaPedalCalibrated", True)
  params.put_bool("TeslaUsePedal", True)
  
  print(f"  TeslaPedalMin:        {avg_min}")
  print(f"  TeslaPedalMax:        {avg_max}")
  print(f"  TeslaPedalCalibrated: True")
  print(f"  TeslaUsePedal:        True")
  
  # ============================================
  # Summary
  # ============================================
  print("\n" + "=" * 50)
  print("CALIBRATION COMPLETE!")
  print("=" * 50)
  print(f"\n  Pedal Min:   {avg_min}")
  print(f"  Pedal Max:   {avg_max}")
  print(f"  Range:       {pedal_range}")
  print(f"  CAN Bus:     {pedal_bus}")
  print("\n  The Comma Pedal is now enabled and calibrated.")
  print("  Reboot your comma device for changes to take effect.")
  print("\n  To disable the pedal later, run:")
  print("    echo 0 > /data/params/d/TeslaUsePedal")
  
  return 0


if __name__ == "__main__":
  sys.exit(main())
