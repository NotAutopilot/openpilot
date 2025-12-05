#!/usr/bin/env python3
"""
Tesla Pre-AP Comma Pedal Calibration Tool
Ported from Tinkla's pedal_calibrator/calibratePedal.py

CRITICAL: This script uses DIRECT PANDA ACCESS with SAFETY_ALLOUTPUT (mode 17).
This bypasses the normal safety layer to allow sending pedal commands while
the car is in Neutral (not engaged).

Usage:
  SSH into comma device and run:
    cd /data/openpilot
    python calibrate_pedal.py
"""

import time
import sys
import struct
import os
from ctypes import create_string_buffer

# ============================================
# Safety Mode Constants (Raw Integers)
# Using raw integers because Panda class attributes vary by version
# ============================================
SAFETY_SILENT = 0       # No TX allowed
SAFETY_ALLOUTPUT = 17   # All TX allowed (bypasses all safety checks)

# ============================================
# Pedal Constants from Tinkla
# ============================================
M1 = 0.050796813
M2 = 0.101593626
D = -22.85856576

# CAN Message IDs
GAS_SENSOR_ID = 0x552       # 1362 - Pedal sensor feedback (RX)
GAS_COMMAND_ID = 0x551      # 1361 - Pedal command (TX)
GTW_STATUS_ID = 0x348       # 840 - Car status
BRAKE_MESSAGE_ID = 0x20A    # 522 - Brake status
DI_TORQUE1_ID = 0x108       # 264 - Accelerator position
DI_TORQUE2_ID = 0x118       # 280 - Gear status

# Timing
PEDAL_TIMEOUT_MS = 500
MAX_PEDAL_ERRORS = 10
SEND_RATE_MS = 20  # Send pedal command every 20ms (50Hz)

# Gear values
GEAR_NEUTRAL = 0x30

# Config file
CONFIG_FILE = "/data/tinkla_params.json"

# ============================================
# Status Messages
# ============================================
CALIBRATOR_STATUSES = [
  "Initializing...",
  "Configuring Panda...",
  "Reading Pedal Zero...",
  "Detecting Pedal Max...",
  "Fine-tuning calibration...",
  "Validating calibration...",
  "Saving values...",
  "Calibration Complete!",
  "Calibration Error: ",
]

CALIBRATION_ERRORS = [
  "Pedal communication timeout!",
  "Car is not ON! Turn ignition on.",
  "Car is not in NEUTRAL! Shift to N.",
  "Accelerator pedal is pressed! Release it.",
  "Brake pedal not pressed! Press and hold brake.",
  "Pedal sensor error!",
  "Panda not found!",
]

SAFETY_BANNER = """
╔══════════════════════════════════════════════════════════════════╗
║              ⚠️  PEDAL CALIBRATION - SAFETY FIRST ⚠️              ║
╠══════════════════════════════════════════════════════════════════╣
║                                                                  ║
║  REQUIREMENTS:                                                   ║
║    1. Park in a SAFE location                                    ║
║    2. Turn the car ON (ready to drive)                           ║
║    3. Shift into NEUTRAL (N)                                     ║
║    4. Press and HOLD the BRAKE pedal                             ║
║                                                                  ║
║  ⚠️  SAFETY_ALLOUTPUT mode will be used for TX.                   ║
║     Safety is restored automatically on exit.                    ║
║                                                                  ║
╚══════════════════════════════════════════════════════════════════╝
"""


def current_time_ms():
  return int(round(time.time() * 1000))


def load_config():
  """Load config from JSON file."""
  import json
  try:
    if os.path.exists(CONFIG_FILE):
      with open(CONFIG_FILE, 'r') as f:
        return json.load(f)
  except Exception:
    pass
  return {}


def save_config(config):
  """Atomically save config to JSON file."""
  import json
  import tempfile
  try:
    os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(
      dir=os.path.dirname(CONFIG_FILE),
      prefix='.tinkla_params_',
      suffix='.tmp'
    )
    try:
      with os.fdopen(fd, 'w') as f:
        json.dump(config, f, indent=2)
      os.replace(tmp_path, CONFIG_FILE)
      return True
    except Exception:
      try:
        os.unlink(tmp_path)
      except Exception:
        pass
  except Exception:
    pass
  return False


class PedalCalibrator:
  """
  Comma Pedal Calibrator using Direct Panda Access.
  """
  
  @staticmethod
  def checksum(msg_id, dat):
    """Calculate Tesla-style checksum for pedal message."""
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF
  
  def __init__(self):
    from panda import Panda
    
    print(CALIBRATOR_STATUSES[0])
    print(CALIBRATOR_STATUSES[1])
    
    # Connect to Panda
    try:
      self.panda = Panda()
      print(f"  ✓ Connected to Panda")
    except Exception as e:
      print(f"  ❌ {CALIBRATION_ERRORS[6]}: {e}")
      raise
    
    # ============================================
    # CRITICAL: Set SAFETY_ALLOUTPUT (mode 17)
    # This is the ONLY way to TX pedal commands when not engaged
    # ============================================
    print(f"  Setting safety mode to ALLOUTPUT ({SAFETY_ALLOUTPUT})...")
    self.panda.set_safety_mode(SAFETY_ALLOUTPUT)
    print(f"  ✓ Safety mode set")
    
    # Determine pedal CAN bus
    self.pedal_can = 2  # Default: Bus 2
    config = load_config()
    if config.get('pedal_can_zero', False):
      self.pedal_can = 0
    print(f"  Pedal CAN bus: {self.pedal_can}")
    
    # Rolling counter (0-15) - CRITICAL for interceptor watchdog
    self.pedal_idx = 0
    
    # Pedal state
    self.rcv_pedal_idx = -1
    self.last_rcv_pedal_idx = -1
    self.last_pedal_seen_ms = 0
    self.last_pedal_sent_ms = 0
    self.pedal_interceptor_state = 0
    self.pedal_interceptor_value = 1000.0
    self.pedal_interceptor_value2 = 1000.0
    self.pedal_available = False
    self.pedal_timeout = True
    self.pedal_error_count = 0
    self.pedal_enabled = 0
    
    # TX debug counters
    self.tx_count = 0
    
    # Car state
    self.car_on = False
    self.brake_pressed = False
    self.gear_shifter = None
    self.gear_neutral = False
    self.di_gas = 0.0
    
    # Calibration state
    self.status = 0
    self.prev_status = -1
    self.frame = 0
    
    # Stage 2: Pedal zero
    self.pedal_zero_count = 0
    self.pedal_zero_values_to_read = 100
    self.pedal_zero_sum = 0.0
    
    # Stage 3: Pedal max detection
    self.pedal_last_value_sent = 0.0
    self.pedal_pressed_value = -1000.0
    self.pedal_max_value = -1000.0
    self.pedal_step = 0
    
    # Stage 4: Fine-tuning
    self.finetuning_stage = 0
    self.finetuning_target = 99.6
    self.finetuning_step = 0.1
    self.finetuning_sum = 0.0
    self.finetuning_count = 0
    self.finetuning_steps = 10
    self.finetuning_best_val = 0.0
    self.finetuning_best_delta = 1000.0
    self.finetuning_best_result = -1000.0
    self.finetuning_start = 0.0
    
    # Stage 5: Validation
    self.validation_stage = 0
    self.validation_target = 0.0
    self.validation_count = 0
    self.validation_sum = 0.0
    self.validation_steps = 10
    self.validation_value = 0.0
    
    # Final values
    self.pedal_min = -1000.0
    self.pedal_max = -1000.0
    self.pedal_pressed = -1000.0
    self.pedal_factor = -1000.0
  
  def cleanup(self):
    """Restore Panda to safe state."""
    if hasattr(self, 'panda') and self.panda is not None:
      print("\n" + "=" * 50)
      print("RESTORING PANDA SAFETY MODE")
      print("=" * 50)
      try:
        # Disable pedal
        print("  Disabling pedal...")
        self.send_pedal_command(0, enable=0)
        time.sleep(0.1)
        
        # Restore safety
        print(f"  Setting safety mode to SILENT ({SAFETY_SILENT})...")
        self.panda.set_safety_mode(SAFETY_SILENT)
        print("  ✓ Panda safety restored")
      except Exception as e:
        print(f"  ⚠️  Warning: {e}")
      
      try:
        self.panda.close()
      except Exception:
        pass
  
  def send_pedal_command(self, accel_command, enable=1):
    """
    Create and send GAS_COMMAND (0x551) to Comma Pedal.
    
    Message format (6 bytes):
      Byte 0-1: accel_command (big-endian, scaled by M1)
      Byte 2-3: accel_command2 (big-endian, scaled by M2) 
      Byte 4: (enable << 7) | (idx & 0x0F)  -- ENABLE BIT IS CRITICAL!
      Byte 5: checksum
    
    The enable bit (0x80) in byte 4 MUST be set for the pedal to respond!
    The rolling counter (idx) MUST increment for the watchdog!
    """
    msg_id = GAS_COMMAND_ID
    msg_len = 6
    
    if enable == 1:
      int_accel = int((accel_command - D) / M1)
      int_accel2 = int((accel_command - D) / M2)
    else:
      int_accel = 0
      int_accel2 = 0
    
    # Clamp to valid range
    int_accel = max(0, min(65534, int_accel))
    int_accel2 = max(0, min(65534, int_accel2))
    
    # Build message
    msg = create_string_buffer(msg_len)
    struct.pack_into(
      "BBBBB",
      msg,
      0,
      (int_accel >> 8) & 0xFF,      # Byte 0: accel high
      int_accel & 0xFF,              # Byte 1: accel low
      (int_accel2 >> 8) & 0xFF,      # Byte 2: accel2 high
      int_accel2 & 0xFF,             # Byte 3: accel2 low
      ((enable << 7) | (self.pedal_idx & 0x0F)) & 0xFF,  # Byte 4: enable + idx
    )
    struct.pack_into("B", msg, msg_len - 1, self.checksum(msg_id, msg.raw))
    
    # Send via direct Panda access
    # panda.can_send(arbitration_id, data, bus)
    self.panda.can_send(msg_id, msg.raw, self.pedal_can)
    
    # Increment rolling counter (wraps 0-15)
    self.pedal_idx = (self.pedal_idx + 1) % 16
    self.last_pedal_sent_ms = current_time_ms()
    self.tx_count += 1
  
  def process_can(self):
    """Process incoming CAN messages from Panda."""
    try:
      for msg in self.panda.can_recv():
        # Handle both old (3-tuple) and new (4-tuple) formats
        if len(msg) == 4:
          addr, _, dat, src = msg
        elif len(msg) == 3:
          addr, dat, src = msg
        else:
          continue
        
        # GTW_status - Car on state
        if addr == GTW_STATUS_ID:
          self.car_on = (dat[0] & 0x01) == 1
        
        # BrakeMessage - Brake pressed
        elif addr == BRAKE_MESSAGE_ID:
          self.brake_pressed = ((dat[0] >> 2) & 0x03) != 1
        
        # DI_torque2 - Gear position
        elif addr == DI_TORQUE2_ID:
          self.gear_shifter = dat[1] & 0x70
          self.gear_neutral = self.gear_shifter == GEAR_NEUTRAL
        
        # DI_torque1 - Accelerator position (car's view)
        elif addr == DI_TORQUE1_ID:
          self.di_gas = dat[6] * 0.4
        
        # GAS_SENSOR - Pedal feedback
        elif addr == GAS_SENSOR_ID:
          self.pedal_interceptor_state = (dat[4] >> 7) & 0x01
          self.pedal_interceptor_value = ((dat[0] << 8) + dat[1]) * M1 + D
          self.pedal_interceptor_value2 = ((dat[2] << 8) + dat[3]) * M2 + D
          self.rcv_pedal_idx = dat[4] & 0x0F
          self.last_pedal_seen_ms = current_time_ms()
          
    except Exception as e:
      if "not enough values" not in str(e):
        print(f"  CAN recv error: {e}")
  
  def check_safety(self):
    """Check safety conditions."""
    if not self.car_on:
      if self.frame % 100 == 0:
        print(f"  ⚠️  {CALIBRATION_ERRORS[1]}", flush=True)
      return False
    
    if not self.brake_pressed:
      if self.frame % 100 == 0:
        print(f"  ⚠️  {CALIBRATION_ERRORS[4]}", flush=True)
      if self.pedal_enabled:
        self.send_pedal_command(0, enable=0)
        self.pedal_enabled = 0
      return False
    
    if not self.gear_neutral:
      if self.frame % 100 == 0:
        print(f"  ⚠️  {CALIBRATION_ERRORS[2]}", flush=True)
      if self.pedal_enabled:
        self.send_pedal_command(0, enable=0)
        self.pedal_enabled = 0
      return False
    
    if self.di_gas > 0 and self.status < 3:
      if self.frame % 100 == 0:
        print(f"  ⚠️  {CALIBRATION_ERRORS[3]}", flush=True)
      if self.pedal_enabled:
        self.send_pedal_command(0, enable=0)
        self.pedal_enabled = 0
      return False
    
    return True
  
  def run(self, rate=100):
    """Main calibration loop."""
    self.status = 2  # Start at "Reading Pedal Zero"
    
    print("\nWaiting for safety conditions...")
    print("  - Car ON")
    print("  - Gear in NEUTRAL")
    print("  - Brake PRESSED")
    print("  - Accelerator RELEASED")
    print(f"\n  [TX Debug: Sending to Bus {self.pedal_can}, ID 0x{GAS_COMMAND_ID:03X}]")
    
    loop_period = 1.0 / rate
    
    while True:
      loop_start = time.time()
      self.frame += 1
      curr_time_ms = current_time_ms()
      
      # Process incoming CAN
      self.process_can()
      
      # Show status change
      if self.status != self.prev_status:
        print(f"\n{CALIBRATOR_STATUSES[self.status]}")
        self.prev_status = self.status
      
      # Check for completion
      if self.status == 7:
        print(f"\n✅ {CALIBRATOR_STATUSES[-2]}")
        return 0
      
      # Check safety
      if not self.check_safety():
        time.sleep(loop_period)
        continue
      
      # ============================================
      # ALWAYS send pedal command at 50Hz to keep watchdog happy
      # Even when waiting for pedal response
      # ============================================
      if curr_time_ms - self.last_pedal_sent_ms >= SEND_RATE_MS:
        self.send_pedal_command(self.pedal_last_value_sent, self.pedal_enabled)
        
        # Debug output every 2 seconds
        if self.tx_count % 100 == 0:
          print(f"  [TX #{self.tx_count}: val={self.pedal_last_value_sent:.1f}, en={self.pedal_enabled}, idx={self.pedal_idx}]", flush=True)
      
      # Check for new pedal message
      if self.rcv_pedal_idx != self.last_rcv_pedal_idx:
        self.last_rcv_pedal_idx = self.rcv_pedal_idx
        
        if self.pedal_interceptor_state == 0:
          self.pedal_error_count = 0
          
          # Stage 2: Reading pedal zero
          if self.status == 2:
            self.pedal_zero_sum += self.pedal_interceptor_value
            self.pedal_zero_count += 1
            
            if self.pedal_zero_count >= self.pedal_zero_values_to_read:
              self.pedal_min = self.pedal_zero_sum / self.pedal_zero_count
              print(f"\n  ✓ Pedal MIN: {self.pedal_min:.2f}")
              self.pedal_last_value_sent = self.pedal_min
              self.pedal_enabled = 1
              self.status = 3
            elif self.frame % 20 == 0:
              print(f"  Reading zero: {self.pedal_zero_count}/{self.pedal_zero_values_to_read} (val={self.pedal_interceptor_value:.2f})", flush=True)
          
          # Stage 3: Detecting pedal max
          elif self.status == 3:
            if self.di_gas >= 99.6 and self.pedal_max_value == -1000 and self.pedal_step == 1:
              print(f"\n  ✓ Pedal MAX: {self.pedal_last_value_sent:.2f}")
              self.pedal_max_value = self.pedal_last_value_sent
              self.pedal_last_value_sent = self.pedal_max_value - 0.5
              self.finetuning_start = self.pedal_max_value - 0.5
              self.pedal_step = 0
              self.status = 4
            
            if self.di_gas > 0 and self.pedal_pressed_value == -1000 and self.pedal_step == 1:
              print(f"\n  ✓ Pedal PRESSED threshold: {self.pedal_last_value_sent:.2f}")
              self.pedal_pressed_value = self.pedal_last_value_sent
            
            if self.di_gas < 100:
              if self.frame % 50 == 0:
                print(f"  Detecting: sent {self.pedal_last_value_sent:.1f} -> car sees {self.di_gas:.1f}%", flush=True)
              if self.pedal_step == 1:
                self.pedal_last_value_sent += 1
                self.pedal_step = 0
              else:
                self.pedal_step = 1
          
          # Stage 4: Fine-tuning
          elif self.status == 4:
            if self.finetuning_count < self.finetuning_steps:
              self.finetuning_sum += self.di_gas
              self.finetuning_count += 1
            
            if self.finetuning_count == self.finetuning_steps:
              avg = self.finetuning_sum / self.finetuning_count
              delta = abs(self.finetuning_target - avg)
              
              if delta < self.finetuning_best_delta:
                self.finetuning_best_val = self.pedal_last_value_sent
                self.finetuning_best_delta = delta
              
              self.pedal_last_value_sent += self.finetuning_step
              print(f"  Fine-tuning: target {self.finetuning_target:.1f}%, got {avg:.1f}%", flush=True)
              self.finetuning_count = 0
              self.finetuning_sum = 0
              
              if self.pedal_last_value_sent > self.finetuning_start + 0.5:
                if self.finetuning_stage == 0:
                  self.pedal_max = self.finetuning_best_val
                  self.finetuning_best_val = 0
                  self.finetuning_best_delta = 1000.0
                  self.finetuning_stage = 1
                  self.pedal_last_value_sent = self.pedal_pressed_value - 0.5
                  self.finetuning_start = self.pedal_pressed_value - 0.5
                  self.finetuning_target = 0.4
                  print("\n  ✓ Done fine-tuning MAX")
                else:
                  self.pedal_pressed = self.finetuning_best_val
                  self.pedal_last_value_sent = self.pedal_min
                  self.pedal_factor = 100.0 / (self.pedal_max - self.pedal_pressed)
                  self.status = 5
                  print("\n  ✓ Done fine-tuning ZERO")
          
          # Stage 5: Validation
          elif self.status == 5:
            if self.validation_stage == 0:
              self.validation_stage = 1
              self.validation_target = self.validation_stage * 10
              self.validation_count = 0
              self.validation_sum = 0
              self.validation_value = self.pedal_pressed + self.validation_target * (self.pedal_max - self.pedal_pressed) / 100.0
              self.pedal_last_value_sent = self.validation_value
            else:
              if self.validation_count < self.validation_steps:
                self.validation_sum += self.di_gas
                self.validation_count += 1
                
                if self.validation_count == self.validation_steps:
                  avg = self.validation_sum / self.validation_count
                  print(f"  Validating {self.validation_target:.0f}%: got {avg:.1f}%", flush=True)
                  self.validation_stage += 1
                  
                  if self.validation_stage == 10:
                    print("\n  ✓ Validation complete")
                    self.status = 6
                  else:
                    self.validation_target = self.validation_stage * 10
                    self.validation_count = 0
                    self.validation_sum = 0
                    self.validation_value = self.pedal_pressed + self.validation_target * (self.pedal_max - self.pedal_pressed) / 100.0
                    self.pedal_last_value_sent = self.validation_value
          
          # Stage 6: Save values
          elif self.status == 6:
            config = load_config()
            
            print("\n" + "=" * 50)
            print("CALIBRATION RESULTS")
            print("=" * 50)
            
            config['pedal_calib_min'] = self.pedal_min
            config['pedal_min'] = int(self.pedal_min)
            print(f"  Pedal Min:    {self.pedal_min:.2f}")
            
            config['pedal_calib_max'] = self.pedal_max
            config['pedal_max'] = int(self.pedal_max)
            print(f"  Pedal Max:    {self.pedal_max:.2f}")
            
            config['pedal_calib_zero'] = self.pedal_pressed
            print(f"  Pedal Zero:   {self.pedal_pressed:.2f}")
            
            config['pedal_calib_factor'] = self.pedal_factor
            print(f"  Pedal Factor: {self.pedal_factor:.4f}")
            
            config['pedal_calibrated'] = True
            config['use_pedal'] = True
            
            if save_config(config):
              print(f"\n  ✓ Saved to {CONFIG_FILE}")
            else:
              print(f"\n  ⚠️  Could not save config!")
            
            self.status = 7
      
      # Check pedal timeout
      self.pedal_timeout = curr_time_ms - self.last_pedal_seen_ms > PEDAL_TIMEOUT_MS
      
      if self.pedal_timeout:
        self.pedal_error_count += 1
        if self.pedal_error_count > MAX_PEDAL_ERRORS * 10:  # More tolerance
          print(f"\n❌ {CALIBRATION_ERRORS[0]}")
          return 1
      
      # Rate limiting
      elapsed = time.time() - loop_start
      if elapsed < loop_period:
        time.sleep(loop_period - elapsed)
    
    return 0


def main():
  print(SAFETY_BANNER)
  
  response = input("Type 'YES' to continue: ")
  if response.strip().upper() != "YES":
    print("\nCancelled.")
    return 1
  
  print("\n" + "=" * 50)
  print("COMMA PEDAL CALIBRATION")
  print("=" * 50)
  
  calibrator = None
  exit_code = 0
  
  try:
    calibrator = PedalCalibrator()
    exit_code = calibrator.run()
  except KeyboardInterrupt:
    print("\n\n⚠️  Interrupted by user.")
    exit_code = 1
  except Exception as e:
    print(f"\n❌ Error: {e}")
    import traceback
    traceback.print_exc()
    exit_code = 1
  finally:
    # CRITICAL: Always restore safety mode
    if calibrator is not None:
      calibrator.cleanup()
  
  return exit_code


if __name__ == "__main__":
  sys.exit(main())
