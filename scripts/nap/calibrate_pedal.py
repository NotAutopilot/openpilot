#!/usr/bin/env python3
"""
Tesla Pre-AP Comma Pedal Calibration Tool (GUI version)
Adapted from calibrate_pedal.py for use with ScriptRunner (no keyboard input).

CRITICAL: This script uses DIRECT PANDA ACCESS with SAFETY_ALLOUTPUT (mode 17).
This bypasses the normal safety layer to allow sending pedal commands while
the car is in Neutral (not engaged).

Reads configuration from openpilot Params and saves calibration results back to Params.
"""

import time
import sys
import struct
from ctypes import create_string_buffer

from openpilot.common.params import Params
from opendbc.car.tesla.nap_params import NAPParamKeys

# ============================================
# Safety Mode Constants
# ============================================
SAFETY_SILENT = 0
SAFETY_ALLOUTPUT = 17

# ============================================
# Pedal Constants from Tinkla
# ============================================
M1 = 0.050796813
M2 = 0.101593626
D = -22.85856576

# CAN Message IDs
GAS_SENSOR_ID = 0x552
GAS_COMMAND_ID = 0x551
GTW_STATUS_ID = 0x348
BRAKE_MESSAGE_ID = 0x20A
DI_TORQUE1_ID = 0x108
DI_TORQUE2_ID = 0x118

# Timing
PEDAL_TIMEOUT_MS = 500
MAX_PEDAL_ERRORS = 10
SEND_RATE_MS = 20

# Gear values
GEAR_NEUTRAL = 0x30


def current_time_ms():
  return int(round(time.time() * 1000))


def p(msg):
  """Print with flush for real-time ScriptRunner display."""
  print(msg, flush=True)


class PedalCalibrator:
  """Comma Pedal Calibrator using Direct Panda Access."""

  @staticmethod
  def checksum(msg_id, dat):
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF

  def __init__(self, params: Params):
    from panda import Panda

    self.params = params

    p("Initializing...")
    p("Configuring Panda...")

    try:
      self.panda = Panda()
      p("  Connected to Panda")
    except Exception as e:
      p(f"  ERROR: Panda not found: {e}")
      raise

    p(f"  Setting safety mode to ALLOUTPUT ({SAFETY_ALLOUTPUT})...")
    self.panda.set_safety_mode(SAFETY_ALLOUTPUT)
    p("  Safety mode set")

    # Read pedal CAN bus from Params
    pedal_bus = params.get(NAPParamKeys.PEDAL_CAN_BUS, return_default=True)
    self.pedal_can = pedal_bus if pedal_bus in (0, 2) else 2
    p(f"  Pedal CAN bus: {self.pedal_can}")

    # Rolling counter (0-15)
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
      p("")
      p("=" * 50)
      p("RESTORING PANDA SAFETY MODE")
      p("=" * 50)
      try:
        p("  Disabling pedal...")
        self.send_pedal_command(0, enable=0)
        time.sleep(0.1)
        p(f"  Setting safety mode to SILENT ({SAFETY_SILENT})...")
        self.panda.set_safety_mode(SAFETY_SILENT)
        p("  Panda safety restored")
      except Exception as e:
        p(f"  Warning: {e}")
      try:
        self.panda.close()
      except Exception:
        pass

  def send_pedal_command(self, accel_command, enable=1):
    msg_id = GAS_COMMAND_ID
    msg_len = 6

    if enable == 1:
      int_accel = int((accel_command - D) / M1)
      int_accel2 = int((accel_command - D) / M2)
    else:
      int_accel = 0
      int_accel2 = 0

    int_accel = max(0, min(65534, int_accel))
    int_accel2 = max(0, min(65534, int_accel2))

    msg = create_string_buffer(msg_len)
    struct.pack_into(
      "BBBBB", msg, 0,
      (int_accel >> 8) & 0xFF,
      int_accel & 0xFF,
      (int_accel2 >> 8) & 0xFF,
      int_accel2 & 0xFF,
      ((enable << 7) | (self.pedal_idx & 0x0F)) & 0xFF,
    )
    struct.pack_into("B", msg, msg_len - 1, self.checksum(msg_id, msg.raw))

    self.panda.can_send(msg_id, msg.raw, self.pedal_can)
    self.pedal_idx = (self.pedal_idx + 1) % 16
    self.last_pedal_sent_ms = current_time_ms()
    self.tx_count += 1

  def process_can(self):
    try:
      for msg in self.panda.can_recv():
        if len(msg) == 4:
          addr, _, dat, src = msg
        elif len(msg) == 3:
          addr, dat, src = msg
        else:
          continue

        if addr == GTW_STATUS_ID:
          self.car_on = (dat[0] & 0x01) == 1
        elif addr == BRAKE_MESSAGE_ID:
          self.brake_pressed = ((dat[0] >> 2) & 0x03) != 1
        elif addr == DI_TORQUE2_ID:
          self.gear_shifter = dat[1] & 0x70
          self.gear_neutral = self.gear_shifter == GEAR_NEUTRAL
        elif addr == DI_TORQUE1_ID:
          self.di_gas = dat[6] * 0.4
        elif addr == GAS_SENSOR_ID:
          self.pedal_interceptor_state = (dat[4] >> 7) & 0x01
          self.pedal_interceptor_value = ((dat[0] << 8) + dat[1]) * M1 + D
          self.pedal_interceptor_value2 = ((dat[2] << 8) + dat[3]) * M2 + D
          self.rcv_pedal_idx = dat[4] & 0x0F
          self.last_pedal_seen_ms = current_time_ms()
    except Exception as e:
      if "not enough values" not in str(e):
        p(f"  CAN recv error: {e}")

  def check_safety(self):
    if not self.car_on:
      if self.frame % 100 == 0:
        p("  Waiting: Car is not ON! Turn ignition on.")
      return False

    if not self.brake_pressed:
      if self.frame % 100 == 0:
        p("  Waiting: Brake not pressed! Press and hold brake.")
      if self.pedal_enabled:
        self.send_pedal_command(0, enable=0)
        self.pedal_enabled = 0
      return False

    if not self.gear_neutral:
      if self.frame % 100 == 0:
        p("  Waiting: Car is not in NEUTRAL! Shift to N.")
      if self.pedal_enabled:
        self.send_pedal_command(0, enable=0)
        self.pedal_enabled = 0
      return False

    if self.di_gas > 0 and self.status < 3:
      if self.frame % 100 == 0:
        p("  Waiting: Accelerator pedal is pressed! Release it.")
      if self.pedal_enabled:
        self.send_pedal_command(0, enable=0)
        self.pedal_enabled = 0
      return False

    return True

  STATUS_MESSAGES = [
    "Initializing...",
    "Configuring Panda...",
    "Reading Pedal Zero...",
    "Detecting Pedal Max...",
    "Fine-tuning calibration...",
    "Validating calibration...",
    "Saving values...",
    "Calibration Complete!",
  ]

  def run(self, rate=100):
    self.status = 2  # Start at "Reading Pedal Zero"

    p("")
    p("Waiting for safety conditions...")
    p("  - Car ON")
    p("  - Gear in NEUTRAL")
    p("  - Brake PRESSED")
    p("  - Accelerator RELEASED")
    p(f"  [TX: Bus {self.pedal_can}, ID 0x{GAS_COMMAND_ID:03X}]")

    loop_period = 1.0 / rate

    while True:
      loop_start = time.time()
      self.frame += 1
      curr_time_ms = current_time_ms()

      self.process_can()

      if self.status != self.prev_status:
        p(f"\n{self.STATUS_MESSAGES[self.status]}")
        self.prev_status = self.status

      if self.status == 7:
        p("\nCalibration Complete!")
        return 0

      if not self.check_safety():
        time.sleep(loop_period)
        continue

      # Send pedal command at 50Hz to keep watchdog happy
      if curr_time_ms - self.last_pedal_sent_ms >= SEND_RATE_MS:
        self.send_pedal_command(self.pedal_last_value_sent, self.pedal_enabled)
        if self.tx_count % 100 == 0:
          p(f"  [TX #{self.tx_count}: val={self.pedal_last_value_sent:.1f}, en={self.pedal_enabled}]")

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
              p(f"\n  Pedal MIN: {self.pedal_min:.2f}")
              self.pedal_last_value_sent = self.pedal_min
              self.pedal_enabled = 1
              self.status = 3
            elif self.frame % 20 == 0:
              p(f"  Reading zero: {self.pedal_zero_count}/{self.pedal_zero_values_to_read} (val={self.pedal_interceptor_value:.2f})")

          # Stage 3: Detecting pedal max
          elif self.status == 3:
            if self.di_gas >= 99.6 and self.pedal_max_value == -1000 and self.pedal_step == 1:
              p(f"\n  Pedal MAX: {self.pedal_last_value_sent:.2f}")
              self.pedal_max_value = self.pedal_last_value_sent
              self.pedal_last_value_sent = self.pedal_max_value - 0.5
              self.finetuning_start = self.pedal_max_value - 0.5
              self.pedal_step = 0
              self.status = 4

            if self.di_gas > 0 and self.pedal_pressed_value == -1000 and self.pedal_step == 1:
              p(f"\n  Pedal PRESSED threshold: {self.pedal_last_value_sent:.2f}")
              self.pedal_pressed_value = self.pedal_last_value_sent

            if self.di_gas < 100:
              if self.frame % 50 == 0:
                p(f"  Detecting: sent {self.pedal_last_value_sent:.1f} -> car sees {self.di_gas:.1f}%")
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
              p(f"  Fine-tuning: target {self.finetuning_target:.1f}%, got {avg:.1f}%")
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
                  p("\n  Done fine-tuning MAX")
                else:
                  self.pedal_pressed = self.finetuning_best_val
                  self.pedal_last_value_sent = self.pedal_min
                  self.pedal_factor = 100.0 / (self.pedal_max - self.pedal_pressed)
                  self.status = 5
                  p("\n  Done fine-tuning ZERO")

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
                  p(f"  Validating {self.validation_target:.0f}%: got {avg:.1f}%")
                  self.validation_stage += 1
                  if self.validation_stage == 10:
                    p("\n  Validation complete")
                    self.status = 6
                  else:
                    self.validation_target = self.validation_stage * 10
                    self.validation_count = 0
                    self.validation_sum = 0
                    self.validation_value = self.pedal_pressed + self.validation_target * (self.pedal_max - self.pedal_pressed) / 100.0
                    self.pedal_last_value_sent = self.validation_value

          # Stage 6: Save values to Params
          elif self.status == 6:
            p("")
            p("=" * 50)
            p("CALIBRATION RESULTS")
            p("=" * 50)

            p(f"  Pedal Min:    {self.pedal_min:.2f}")
            self.params.put(NAPParamKeys.PEDAL_CALIB_MIN, self.pedal_min)

            p(f"  Pedal Max:    {self.pedal_max:.2f}")
            self.params.put(NAPParamKeys.PEDAL_CALIB_MAX, self.pedal_max)

            p(f"  Pedal Zero:   {self.pedal_pressed:.2f}")
            self.params.put(NAPParamKeys.PEDAL_CALIB_ZERO, self.pedal_pressed)

            p(f"  Pedal Factor: {self.pedal_factor:.4f}")
            self.params.put(NAPParamKeys.PEDAL_CALIB_FACTOR, self.pedal_factor)

            self.params.put_bool(NAPParamKeys.PEDAL_CALIB_DONE, True)
            self.params.put_bool(NAPParamKeys.PEDAL_ENABLED, True)
            p("\n  Saved to openpilot Params")

            self.status = 7

      # Check pedal timeout
      self.pedal_timeout = curr_time_ms - self.last_pedal_seen_ms > PEDAL_TIMEOUT_MS
      if self.pedal_timeout:
        self.pedal_error_count += 1
        if self.pedal_error_count > MAX_PEDAL_ERRORS * 10:
          p("\nERROR: Pedal communication timeout!")
          return 1

      elapsed = time.time() - loop_start
      if elapsed < loop_period:
        time.sleep(loop_period - elapsed)

    return 0


def main():
  p("=" * 60)
  p("COMMA PEDAL CALIBRATION")
  p("=" * 60)
  p("")
  p("REQUIREMENTS:")
  p("  1. Park in a SAFE location")
  p("  2. Turn the car ON (ready to drive)")
  p("  3. Shift into NEUTRAL (N)")
  p("  4. Press and HOLD the BRAKE pedal")
  p("")
  p("SAFETY_ALLOUTPUT mode will be used for TX.")
  p("Safety is restored automatically on exit.")
  p("=" * 60)

  params = Params()
  calibrator = None
  exit_code = 0

  try:
    calibrator = PedalCalibrator(params)
    exit_code = calibrator.run()
  except KeyboardInterrupt:
    p("\nInterrupted by user.")
    exit_code = 1
  except Exception as e:
    p(f"\nERROR: {e}")
    import traceback
    traceback.print_exc()
    exit_code = 1
  finally:
    if calibrator is not None:
      calibrator.cleanup()

  return exit_code


if __name__ == "__main__":
  sys.exit(main())
