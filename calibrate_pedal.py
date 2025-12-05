#!/usr/bin/env python3
"""
Tesla Pre-AP Comma Pedal Calibration Tool
Ported from Tinkla's pedal_calibrator/calibratePedal.py

CRITICAL: This script uses DIRECT PANDA ACCESS with SAFETY_ALLOUTPUT mode.
This bypasses the normal safety layer to allow sending pedal commands while
the car is in Neutral (not engaged).

Safety is ALWAYS restored via try/finally block when the script exits.

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
# Constants from Tinkla
# ============================================

# Pedal conversion constants (from comma_pedal.dbc)
M1 = 0.050796813
M2 = 0.101593626
D = -22.85856576

# CAN Message IDs
GAS_SENSOR_ID = 0x552       # 1362 - Pedal sensor feedback
GAS_COMMAND_ID = 0x551      # 1361 - Pedal command
GTW_STATUS_ID = 0x348       # 840 - Car status
BRAKE_MESSAGE_ID = 0x20A    # 522 - Brake status
DI_TORQUE1_ID = 0x108       # 264 - Accelerator position
DI_TORQUE2_ID = 0x118       # 280 - Gear status

# Timing
PEDAL_TIMEOUT_MS = 500
MAX_PEDAL_ERRORS = 10

# Gear values (from DBC)
GEAR_NEUTRAL = 0x30  # DI_gear = 3 (Neutral) shifted to position

# JSON config file path
CONFIG_FILE = "/data/tinkla_params.json"

# ============================================
# Status Messages
# ============================================

CALIBRATOR_STATUSES = [
  "Initializing...",
  "Configuring Panda (SAFETY_ALLOUTPUT)...",
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
  "Panda not found! Is USB connected?",
]

# ============================================
# Safety Banner
# ============================================
SAFETY_BANNER = """
╔══════════════════════════════════════════════════════════════════╗
║              ⚠️  PEDAL CALIBRATION - SAFETY FIRST ⚠️              ║
╠══════════════════════════════════════════════════════════════════╣
║                                                                  ║
║  BEFORE PROCEEDING, YOU MUST:                                    ║
║                                                                  ║
║    1. Park in a SAFE location (flat, away from traffic)          ║
║    2. Turn the car ON (ignition on, ready to drive)              ║
║    3. Shift into NEUTRAL (N)                                     ║
║    4. Press and HOLD the BRAKE pedal                             ║
║    5. Keep hands ready on the gear selector                      ║
║                                                                  ║
║  During calibration:                                             ║
║    - The pedal will be tested automatically                      ║
║    - Keep your foot on the brake at ALL times                    ║
║    - If anything feels wrong, shift to PARK immediately          ║
║                                                                  ║
║  The calibration will ABORT if:                                  ║
║    - Car is not on                                               ║
║    - Gear is not in Neutral                                      ║
║    - Brake is not pressed                                        ║
║                                                                  ║
║  ⚠️  This script uses DIRECT PANDA ACCESS.                        ║
║     Safety mode is restored automatically on exit.               ║
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
  Full port of Tinkla's PedalCalibrator class.
  
  Uses DIRECT PANDA ACCESS with SAFETY_ALLOUTPUT to bypass
  the safety layer that blocks pedal commands when not engaged.
  """
  
  @staticmethod
  def checksum(msg_id, dat):
    """Calculate Tesla-style checksum."""
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF
  
  def __init__(self):
    from panda import Panda
    
    print(CALIBRATOR_STATUSES[0])
    
    # ============================================
    # CRITICAL: Direct Panda Access
    # ============================================
    print(CALIBRATOR_STATUSES[1])
    
    try:
      self.panda = Panda()
    except Exception as e:
      print(f"\n❌ {CALIBRATION_ERRORS[6]}")
      print(f"   Error: {e}")
      raise
    
    # Store original safety mode to restore later
    self.original_safety_mode = None
    
    # Set SAFETY_ALLOUTPUT to allow sending any CAN message
    # This is required because the Tesla safety blocks 0x551 when not engaged
    # Using raw integer 17 because Panda class may not have SAFETY_ALLOUTPUT attribute
    print("  Setting SAFETY_ALLOUTPUT mode...")
    SAFETY_ALLOUTPUT = 17  # Raw firmware ID for all-output mode
    self.panda.set_safety_mode(SAFETY_ALLOUTPUT)
    print("  ✓ Panda configured for calibration")
    
    # Pedal state
    self.pedal_idx = 0
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
    
    # Determine pedal CAN bus from config
    self.pedal_can = 2
    config = load_config()
    if config.get('pedal_can_zero', False):
      self.pedal_can = 0
    print(f"  Pedal CAN bus: {self.pedal_can}")
    
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
    """
    Restore Panda to safe state.
    
    CRITICAL: This MUST be called when exiting, even on error.
    The main() function uses try/finally to ensure this.
    """
    if hasattr(self, 'panda') and self.panda is not None:
      print("\n" + "=" * 50)
      print("RESTORING PANDA SAFETY MODE")
      print("=" * 50)
      try:
        # Send disable command to pedal first
        print("  Disabling pedal...")
        self.create_pedal_command_msg(0, 0)
        time.sleep(0.1)
        
        # Restore to SAFETY_SILENT (no output) for safety
        # The car software will set the proper mode when it starts
        # Using raw integer 0 because Panda class may not have SAFETY_SILENT attribute
        SAFETY_SILENT = 0  # Raw firmware ID for silent mode (no TX allowed)
        print("  Setting SAFETY_SILENT mode...")
        self.panda.set_safety_mode(SAFETY_SILENT)
        print("  ✓ Panda safety restored")
      except Exception as e:
        print(f"  ⚠️  Warning: Could not restore safety: {e}")
      
      try:
        self.panda.close()
      except Exception:
        pass
  
  def show_status(self, n):
    """Print status message."""
    if self.status != self.prev_status:
      print(f"\n{CALIBRATOR_STATUSES[n]}")
      self.prev_status = self.status
  
  def show_error(self, n):
    """Print error message (rate limited)."""
    if self.frame % 100 == 0:
      print(f"  ⚠️  {CALIBRATION_ERRORS[n]}", flush=True)
  
  def finish_with_error(self, n):
    """Exit with error."""
    print(f"\n❌ {CALIBRATOR_STATUSES[-1]}{CALIBRATION_ERRORS[n]}")
    raise SystemExit(1)
  
  def finish(self):
    """Exit successfully."""
    print(f"\n✅ {CALIBRATOR_STATUSES[-2]}")
    raise SystemExit(0)
  
  def create_pedal_command_msg(self, accel_command, enable):
    """
    Create and send GAS_COMMAND (0x551) message to Comma Pedal.
    
    Uses DIRECT PANDA CAN SEND (not sendcan/boardd).
    The enable bit (bit 7 of byte 4) MUST be set for the pedal to respond.
    """
    msg_id = GAS_COMMAND_ID
    msg_len = 6
    
    if enable == 1:
      int_accel_command = int((accel_command - D) / M1)
      int_accel_command2 = int((accel_command - D) / M2)
    else:
      int_accel_command = 0
      int_accel_command2 = 0
    
    # Clamp values
    int_accel_command = max(0, min(65534, int_accel_command))
    int_accel_command2 = max(0, min(65534, int_accel_command2))
    
    msg = create_string_buffer(msg_len)
    struct.pack_into(
      "BBBBB",
      msg,
      0,
      int((int_accel_command >> 8) & 0xFF),
      int_accel_command & 0xFF,
      int((int_accel_command2 >> 8) & 0xFF),
      int_accel_command2 & 0xFF,
      ((enable << 7) + self.pedal_idx) & 0xFF,  # Enable bit is bit 7!
    )
    struct.pack_into("B", msg, msg_len - 1, self.checksum(msg_id, msg.raw))
    
    # DIRECT PANDA SEND (bypasses boardd/safety)
    self.panda.can_send(msg_id, msg.raw, self.pedal_can)
    
    self.last_pedal_sent_ms = current_time_ms()
    self.pedal_idx = (self.pedal_idx + 1) % 16
  
  def process_can(self):
    """Process incoming CAN messages from Panda."""
    try:
      # Direct Panda receive
      for addr, _, dat, src in self.panda.can_recv():
        # GTW_status - Car on state
        if addr == GTW_STATUS_ID:
          self.car_on = (dat[0] & 0x01) == 1
        
        # BrakeMessage - Brake pressed
        if addr == BRAKE_MESSAGE_ID:
          self.brake_pressed = ((dat[0] >> 2) & 0x03) != 1
        
        # DI_torque2 - Gear position
        if addr == DI_TORQUE2_ID:
          self.gear_shifter = dat[1] & 0x70
          self.gear_neutral = self.gear_shifter == GEAR_NEUTRAL
        
        # DI_torque1 - Accelerator position
        if addr == DI_TORQUE1_ID:
          self.di_gas = dat[6] * 0.4
        
        # GAS_SENSOR - Pedal feedback
        if addr == GAS_SENSOR_ID:
          self.pedal_interceptor_state = (dat[4] >> 7) & 0x01
          self.pedal_interceptor_value = ((dat[0] << 8) + dat[1]) * M1 + D
          self.pedal_interceptor_value2 = ((dat[2] << 8) + dat[3]) * M2 + D
          self.rcv_pedal_idx = dat[4] & 0x0F
    except Exception as e:
      print(f"  CAN recv error: {e}")
  
  def check_safety_conditions(self):
    """Check all safety conditions. Returns True if safe to proceed."""
    # Car must be on
    if not self.car_on:
      self.show_error(1)
      return False
    
    # Brake must be pressed
    if not self.brake_pressed:
      self.show_error(4)
      if self.pedal_enabled == 1:
        self.create_pedal_command_msg(0, 0)
        self.pedal_enabled = 0
      return False
    
    # Must be in neutral
    if not self.gear_neutral:
      self.show_error(2)
      if self.pedal_enabled == 1:
        self.create_pedal_command_msg(0, 0)
        self.pedal_enabled = 0
      return False
    
    # Accelerator not pressed (only for early stages)
    if self.di_gas > 0 and self.status < 3:
      self.show_error(3)
      if self.pedal_enabled == 1:
        self.create_pedal_command_msg(0, 0)
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
    
    loop_period = 1.0 / rate
    
    while True:
      loop_start = time.time()
      self.frame += 1
      curr_time_ms = current_time_ms()
      
      # Process CAN
      self.process_can()
      
      # Show status change
      self.show_status(self.status)
      
      # Check for completion
      if self.status == 7:
        self.finish()
      
      # Check safety conditions
      if not self.check_safety_conditions():
        # Sleep and continue
        elapsed = time.time() - loop_start
        if elapsed < loop_period:
          time.sleep(loop_period - elapsed)
        continue
      
      # Process pedal message
      if self.rcv_pedal_idx != self.last_rcv_pedal_idx:
        self.last_pedal_seen_ms = curr_time_ms
        self.last_rcv_pedal_idx = self.rcv_pedal_idx
        
        if self.pedal_interceptor_state == 0:
          self.pedal_error_count = 0
          
          # Stage 2: Reading pedal zero (released position)
          if self.status == 2:
            self.pedal_zero_sum += self.pedal_interceptor_value
            self.pedal_zero_count += 1
            
            if self.pedal_zero_count >= self.pedal_zero_values_to_read:
              self.pedal_min = self.pedal_zero_sum / self.pedal_zero_count
              print(f"\n  ✓ Pedal MIN detected: {self.pedal_min:.2f}")
              self.pedal_last_value_sent = self.pedal_min
              self.pedal_enabled = 1
              self.status = 3
            elif self.frame % 20 == 0:
              print(f"  Reading zero: {self.pedal_zero_count}/{self.pedal_zero_values_to_read}", flush=True)
          
          # Stage 3: Detecting pedal max
          if self.status == 3:
            if self.di_gas >= 99.6 and self.pedal_max_value == -1000 and self.pedal_step == 1:
              print(f"\n  ✓ Pedal MAX detected: {self.pedal_last_value_sent:.2f}")
              self.pedal_max_value = self.pedal_last_value_sent
              self.pedal_last_value_sent = self.pedal_max_value - 0.5
              self.finetuning_start = self.pedal_max_value - 0.5
              self.pedal_enabled = 1
              self.pedal_step = 0
              self.status = 4
            
            if self.di_gas > 0 and self.pedal_pressed_value == -1000 and self.pedal_step == 1:
              print(f"\n  ✓ Pedal PRESSED threshold: {self.pedal_last_value_sent:.2f}")
              self.pedal_pressed_value = self.pedal_last_value_sent
            
            if self.di_gas < 100:
              if self.frame % 50 == 0:
                print(f"  Detecting max: sent {self.pedal_last_value_sent:.1f} -> car sees {self.di_gas:.1f}%", flush=True)
              if self.pedal_step == 1:
                self.pedal_last_value_sent += 1
                self.pedal_step = 0
              else:
                self.pedal_step = 1
          
          # Stage 4: Fine-tuning
          if self.status == 4:
            if self.finetuning_count < self.finetuning_steps:
              self.finetuning_sum += self.di_gas
              self.finetuning_count += 1
            
            if self.finetuning_count == self.finetuning_steps:
              avg = self.finetuning_sum / self.finetuning_count
              delta = abs(self.finetuning_target - avg)
              
              if delta < self.finetuning_best_delta:
                self.finetuning_best_val = self.pedal_last_value_sent
                self.finetuning_best_delta = delta
                self.finetuning_best_result = avg
              
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
                  print("\n  ✓ Done fine-tuning pedal MAX")
                else:
                  self.pedal_pressed = self.finetuning_best_val
                  self.pedal_last_value_sent = self.pedal_min
                  self.pedal_factor = 100.0 / (self.pedal_max - self.pedal_pressed)
                  self.status = 5
                  print("\n  ✓ Done fine-tuning pedal ZERO")
          
          # Stage 5: Validation
          if self.status == 5:
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
          
          # Stage 6: Save values to JSON config
          if self.status == 6:
            config = load_config()
            saved_count = 0
            
            print("\n" + "=" * 50)
            print("CALIBRATION RESULTS")
            print("=" * 50)
            
            if self.pedal_min != -1000:
              config['pedal_calib_min'] = self.pedal_min
              config['pedal_min'] = int(self.pedal_min)
              print(f"  Pedal Min:    {self.pedal_min:.2f}")
              saved_count += 1
            
            if self.pedal_max != -1000:
              config['pedal_calib_max'] = self.pedal_max
              config['pedal_max'] = int(self.pedal_max)
              print(f"  Pedal Max:    {self.pedal_max:.2f}")
              saved_count += 1
            
            if self.pedal_pressed != -1000:
              config['pedal_calib_zero'] = self.pedal_pressed
              print(f"  Pedal Zero:   {self.pedal_pressed:.2f}")
              saved_count += 1
            
            if self.pedal_factor != -1000:
              config['pedal_calib_factor'] = self.pedal_factor
              print(f"  Pedal Factor: {self.pedal_factor:.4f}")
              saved_count += 1
            
            if saved_count == 4:
              config['pedal_calibrated'] = True
              config['use_pedal'] = True
              
              if save_config(config):
                print("\n  ✓ All parameters saved to JSON!")
                print(f"  ✓ Config file: {CONFIG_FILE}")
                print("  ✓ Pedal enabled (use_pedal = True)")
              else:
                print("\n  ⚠️  Warning: Could not save config file!")
            
            self.status = 7
          
          # Keep sending pedal command
          self.create_pedal_command_msg(self.pedal_last_value_sent, self.pedal_enabled)
      
      # Check pedal timeout
      self.pedal_timeout = curr_time_ms - self.last_pedal_seen_ms > PEDAL_TIMEOUT_MS
      self.pedal_available = not self.pedal_timeout and self.pedal_interceptor_state == 0
      
      if self.pedal_timeout or self.pedal_interceptor_state > 0:
        if curr_time_ms - self.last_pedal_sent_ms > PEDAL_TIMEOUT_MS:
          self.pedal_error_count += 1
          if self.pedal_error_count > MAX_PEDAL_ERRORS:
            self.finish_with_error(0)
          # Send reset command
          self.create_pedal_command_msg(0, 0)
      
      # Rate limiting
      elapsed = time.time() - loop_start
      if elapsed < loop_period:
        time.sleep(loop_period - elapsed)


def main():
  print(SAFETY_BANNER)
  
  response = input("Type 'YES' to acknowledge safety requirements and continue: ")
  if response.strip().upper() != "YES":
    print("\nCalibration cancelled.")
    return 1
  
  print("\n" + "=" * 50)
  print("COMMA PEDAL CALIBRATION")
  print("Ported from Tinkla")
  print("=" * 50)
  
  calibrator = None
  exit_code = 0
  
  try:
    calibrator = PedalCalibrator()
    calibrator.run()
  except SystemExit as e:
    exit_code = e.code if e.code is not None else 0
  except KeyboardInterrupt:
    print("\n\n⚠️  Calibration interrupted by user.")
    exit_code = 1
  except Exception as e:
    print(f"\n\n❌ Error: {e}")
    import traceback
    traceback.print_exc()
    exit_code = 1
  finally:
    # ============================================
    # CRITICAL: ALWAYS restore safety mode
    # ============================================
    if calibrator is not None:
      calibrator.cleanup()
  
  return exit_code


if __name__ == "__main__":
  sys.exit(main())
