#!/usr/bin/env python3
"""
Tesla Pre-AP Comma Pedal Calibration Tool
Ported from Tinkla's pedal_calibrator/calibratePedal.py

This is a full port of the original Tinkla calibration tool that:
- Verifies safety conditions (car on, brake pressed, gear in neutral)
- Sends commands to the pedal to detect min/max values
- Fine-tunes and validates the calibration
- Saves all calibration parameters

Usage:
  SSH into comma device and run:
    cd /data/openpilot
    python calibrate_pedal.py
"""

import time
import sys
import struct
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

# ============================================
# Status Messages
# ============================================

CALIBRATOR_STATUSES = [
  "Initializing...",
  "Checking safety conditions...",
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
╚══════════════════════════════════════════════════════════════════╝
"""


def current_time_ms():
  return int(round(time.time() * 1000))


class PedalCalibrator:
  """
  Full port of Tinkla's PedalCalibrator class.
  """
  
  @staticmethod
  def checksum(msg_id, dat):
    """Calculate Tesla-style checksum."""
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF
  
  def __init__(self):
    import cereal.messaging as messaging
    from openpilot.common.params import Params
    
    self.messaging = messaging
    self.params = Params()
    
    print(CALIBRATOR_STATUSES[0])
    
    # Messaging setup
    self.can_sock = messaging.sub_sock('can')
    self.pm = messaging.PubMaster(['sendcan'])
    
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
    
    # Determine pedal CAN bus
    self.pedal_can = 2
    try:
      if self.params.get_bool("TeslaPedalCanZero"):
        self.pedal_can = 0
    except:
      pass
    
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
    sys.exit(1)
  
  def finish(self):
    """Exit successfully."""
    print(f"\n✅ {CALIBRATOR_STATUSES[-2]}")
    sys.exit(0)
  
  def create_pedal_command_msg(self, accel_command, enable):
    """Create and send GAS_COMMAND (0x551) message to Comma Pedal."""
    msg_id = GAS_COMMAND_ID
    msg_len = 6
    
    if enable == 1:
      int_accel_command = int((accel_command - D) / M1)
      int_accel_command2 = int((accel_command - D) / M2)
    else:
      int_accel_command = 0
      int_accel_command2 = 0
    
    msg = create_string_buffer(msg_len)
    struct.pack_into(
      "BBBBB",
      msg,
      0,
      int((int_accel_command >> 8) & 0xFF),
      int_accel_command & 0xFF,
      int((int_accel_command2 >> 8) & 0xFF),
      int_accel_command2 & 0xFF,
      ((enable << 7) + self.pedal_idx) & 0xFF,
    )
    struct.pack_into("B", msg, msg_len - 1, self.checksum(msg_id, msg.raw))
    
    # Send via sendcan
    can_msg = self.messaging.new_message('sendcan', 1)
    can_msg.sendcan[0].address = msg_id
    can_msg.sendcan[0].dat = msg.raw
    can_msg.sendcan[0].src = self.pedal_can
    self.pm.send('sendcan', can_msg)
    
    self.last_pedal_sent_ms = current_time_ms()
    self.pedal_idx = (self.pedal_idx + 1) % 16
  
  def process_can(self):
    """Process incoming CAN messages."""
    messages = self.messaging.drain_sock(self.can_sock, wait_for_one=False)
    
    for m in messages:
      if m.which() != 'can':
        continue
      
      for c in m.can:
        addr = c.address
        dat = c.dat
        
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
    from openpilot.common.realtime import Ratekeeper
    
    self.status = 1  # Checking safety
    rk = Ratekeeper(rate)
    
    print("\nWaiting for safety conditions...")
    print("  - Car ON")
    print("  - Gear in NEUTRAL")
    print("  - Brake PRESSED")
    print("  - Accelerator RELEASED")
    
    while True:
      rk.keep_time()
      self.frame = rk.frame
      curr_time_ms = current_time_ms()
      
      # Process CAN
      self.process_can()
      
      # Check for completion
      if self.status == 7:
        self.finish()
      
      # Show status change
      self.show_status(self.status)
      
      # Check safety conditions
      if not self.check_safety_conditions():
        continue
      
      # Safety conditions met - advance from checking stage
      if self.status == 1:
        print("\n✓ All safety conditions met!")
        self.status = 2
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
          
          # Stage 6: Save values
          if self.status == 6:
            saved_count = 0
            
            print("\n" + "=" * 50)
            print("CALIBRATION RESULTS")
            print("=" * 50)
            
            if self.pedal_min != -1000:
              self.params.put("TeslaPedalCalibMin", str(self.pedal_min))
              print(f"  Pedal Min:    {self.pedal_min:.2f}")
              saved_count += 1
            
            if self.pedal_max != -1000:
              self.params.put("TeslaPedalCalibMax", str(self.pedal_max))
              print(f"  Pedal Max:    {self.pedal_max:.2f}")
              saved_count += 1
            
            if self.pedal_pressed != -1000:
              self.params.put("TeslaPedalCalibZero", str(self.pedal_pressed))
              print(f"  Pedal Zero:   {self.pedal_pressed:.2f}")
              saved_count += 1
            
            if self.pedal_factor != -1000:
              self.params.put("TeslaPedalCalibFactor", str(self.pedal_factor))
              print(f"  Pedal Factor: {self.pedal_factor:.4f}")
              saved_count += 1
            
            # Also save simplified min/max for compatibility
            if self.pedal_min != -1000:
              self.params.put("TeslaPedalMin", str(int(self.pedal_min)))
            if self.pedal_max != -1000:
              self.params.put("TeslaPedalMax", str(int(self.pedal_max)))
            
            if saved_count == 4:
              self.params.put_bool("TeslaPedalCalibDone", True)
              self.params.put_bool("TeslaPedalCalibrated", True)
              self.params.put_bool("TeslaUsePedal", True)
              print("\n  ✓ All parameters saved!")
              print("  ✓ Pedal enabled (TeslaUsePedal = True)")
            
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
  
  try:
    calibrator = PedalCalibrator()
    calibrator.run()
  except KeyboardInterrupt:
    print("\n\nCalibration interrupted by user.")
    return 1
  except Exception as e:
    print(f"\n\nError: {e}")
    import traceback
    traceback.print_exc()
    return 1
  
  return 0


if __name__ == "__main__":
  sys.exit(main())
