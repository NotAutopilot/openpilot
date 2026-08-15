#!/usr/bin/env python3
"""Production Pre-AP pedal calibration adapted from NAP calibrate_pedal.

Pedal frames go through Tesla Pre-AP safety with the calibration-only flag.
ELM327/allOutput are rejected for 0x551. Command bytes come from
TeslaCANPreAP.create_pedal_command.
"""
from __future__ import annotations

import time

from openpilot.common.params import Params
from opendbc.car.tesla.preap.boot import hardware_snapshot_from_values
from opendbc.car.tesla.preap.constants import (
  GAS_COMMAND_ID,
  GAS_SENSOR_ID,
  PEDAL_D,
  PEDAL_M1,
  PEDAL_TIMEOUT_MS,
)
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.safety import parse_explicit_confirmation, require_preap_tool_start
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.transport import DiagnosticTransport, TransportError

GTW_STATUS_ID = 0x348
BRAKE_MESSAGE_ID = 0x20A
DI_TORQUE1_ID = 0x108
DI_TORQUE2_ID = 0x118
GEAR_NEUTRAL = 0x30
MAX_PEDAL_ERRORS = 10
SEND_RATE_MS = 20


def current_time_ms() -> int:
  return int(round(time.monotonic() * 1000))


def p(msg=""):
  print(msg, flush=True)


def unpack_can(msg):
  if len(msg) == 4:
    addr, _bus, dat, src = msg
    return int(addr), bytes(dat), int(src)
  if len(msg) == 3:
    addr, dat, src = msg
    return int(addr), bytes(dat), int(src)
  return None


class PedalCalibrationError(Exception):
  pass


def parse_configured_pedal_bus(value) -> int:
  """Preserve configured bus 0. Absent or empty defaults to 2."""
  if value is None or value == "" or value == b"":
    return 2
  return int(value)


def build_pedal_command(accel_command: float, enable: int, bus: int, can: TeslaCANPreAP | None = None):
  can = can or TeslaCANPreAP(None)
  addr, dat, out_bus = can.create_pedal_command(accel_command, enable=enable, pedal_can_bus=bus)
  if enable not in (0, 1):
    raise PedalCalibrationError("invalid enable flag")
  if bus not in (0, 2):
    raise PedalCalibrationError("invalid pedal bus")
  if out_bus != bus:
    raise PedalCalibrationError("command bus mismatch")
  if addr != GAS_COMMAND_ID:
    raise PedalCalibrationError("invalid command address")
  if len(dat) != 6:
    raise PedalCalibrationError("invalid command length")
  return addr, dat, out_bus


def validate_calibration(min_v: float, max_v: float, factor: float, zero: float) -> None:
  if not (max_v > min_v):
    raise PedalCalibrationError("calibration min/max invalid")
  if factor <= 1e-6:
    raise PedalCalibrationError("calibration factor invalid")
  if factor == 1.0 and zero == 0.0:
    raise PedalCalibrationError("default calibration is not a completed calibration")


def persist_calibration(params: Params, *, min_v: float, max_v: float, factor: float, zero: float) -> None:
  validate_calibration(min_v, max_v, factor, zero)
  snapshot = hardware_snapshot_from_values(
    pedal_enabled=True,
    pedal_bus=params.get("NAPPedalCanBus"),
    pedal_calib_done=True,
    pedal_calib_factor=factor,
    pedal_calib_zero=zero,
    pedal_calib_min=min_v,
    pedal_calib_max=max_v,
  )
  if not snapshot.pedal_calib_available:
    raise PedalCalibrationError("calibration rejected by boot snapshot")
  params.put("NAPPedalCalibMin", float(min_v), block=True)
  params.put("NAPPedalCalibMax", float(max_v), block=True)
  params.put("NAPPedalCalibFactor", float(factor), block=True)
  params.put("NAPPedalCalibZero", float(zero), block=True)
  params.put_bool("NAPPedalCalibDone", True, block=True)
  params.put_bool("NAPPedalEnabled", True, block=True)


def send_safe_release(transport: DiagnosticTransport, bus: int, can: TeslaCANPreAP | None = None) -> None:
  addr, dat, out_bus = build_pedal_command(0.0, enable=0, bus=bus, can=can)
  transport.can_send(addr, dat, out_bus)


class PedalCalibrator:
  """NAP pedal calibration stages, sent only through Tesla Pre-AP calibration safety."""

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

  def __init__(self, params: Params, transport: DiagnosticTransport, bus: int):
    self.params = params
    self.transport = transport
    self.pedal_can = bus
    self.can = TeslaCANPreAP(None)
    self.can.pedal_can_bus = bus

    self.rcv_pedal_idx = -1
    self.last_rcv_pedal_idx = -1
    self.last_pedal_seen_ms = 0
    self.last_pedal_sent_ms = 0
    self.pedal_interceptor_state = 0
    self.pedal_interceptor_value = 1000.0
    self.pedal_enabled = 0
    self.tx_count = 0
    self.pedal_error_count = 0
    self.pedal_timeout = True

    self.car_on = False
    self.brake_pressed = False
    self.gear_neutral = False
    self.di_gas = 0.0

    self.status = 0
    self.prev_status = -1
    self.frame = 0

    self.pedal_zero_count = 0
    self.pedal_zero_values_to_read = 100
    self.pedal_zero_sum = 0.0

    self.pedal_last_value_sent = 0.0
    self.pedal_pressed_value = -1000.0
    self.pedal_max_value = -1000.0
    self.pedal_step = 0

    self.finetuning_stage = 0
    self.finetuning_target = 99.6
    self.finetuning_step = 0.1
    self.finetuning_sum = 0.0
    self.finetuning_count = 0
    self.finetuning_steps = 10
    self.finetuning_best_val = 0.0
    self.finetuning_best_delta = 1000.0
    self.finetuning_start = 0.0

    self.validation_stage = 0
    self.validation_target = 0.0
    self.validation_count = 0
    self.validation_sum = 0.0
    self.validation_steps = 10
    self.validation_value = 0.0

    self.pedal_min = -1000.0
    self.pedal_max = -1000.0
    self.pedal_pressed = -1000.0
    self.pedal_factor = -1000.0

  def cleanup(self) -> None:
    p("")
    p("RESTORING PEDAL / PANDA SAFETY")
    try:
      send_safe_release(self.transport, self.pedal_can, can=self.can)
      time.sleep(0.1)
    except Exception as exc:
      p(f"  Warning: safe release failed: {exc}")
    try:
      self.transport.set_silent()
    except Exception as exc:
      p(f"  Warning: {exc}")

  def send_pedal_command(self, accel_command, enable=1) -> None:
    addr, dat, out_bus = build_pedal_command(accel_command, enable=enable, bus=self.pedal_can, can=self.can)
    self.transport.can_send(addr, dat, out_bus)
    self.last_pedal_sent_ms = current_time_ms()
    self.tx_count += 1

  def process_can(self) -> None:
    try:
      for msg in self.transport.can_recv():
        unpacked = unpack_can(msg)
        if unpacked is None:
          continue
        addr, dat, _src = unpacked
        if addr == GTW_STATUS_ID and dat:
          self.car_on = (dat[0] & 0x01) == 1
        elif addr == BRAKE_MESSAGE_ID and dat:
          self.brake_pressed = ((dat[0] >> 2) & 0x03) != 1
        elif addr == DI_TORQUE2_ID and len(dat) > 1:
          self.gear_neutral = (dat[1] & 0x70) == GEAR_NEUTRAL
        elif addr == DI_TORQUE1_ID and len(dat) > 6:
          self.di_gas = dat[6] * 0.4
        elif addr == GAS_SENSOR_ID and len(dat) > 4:
          self.pedal_interceptor_state = (dat[4] >> 7) & 0x01
          self.pedal_interceptor_value = ((dat[0] << 8) + dat[1]) * PEDAL_M1 + PEDAL_D
          self.rcv_pedal_idx = dat[4] & 0x0F
          self.last_pedal_seen_ms = current_time_ms()
    except TransportError as exc:
      p(f"  CAN recv error: {exc}")

  def check_safety(self) -> bool:
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

  def _advance_on_pedal_message(self) -> None:
    if self.pedal_interceptor_state != 0:
      return
    self.pedal_error_count = 0

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
            span = self.pedal_max - self.pedal_pressed
            if span <= 1e-6:
              raise PedalCalibrationError("calibration span invalid")
            self.pedal_factor = 100.0 / span
            self.status = 5
            p("\n  Done fine-tuning ZERO")

    elif self.status == 5:
      if self.validation_stage == 0:
        self.validation_stage = 1
        self.validation_target = self.validation_stage * 10
        self.validation_count = 0
        self.validation_sum = 0
        self.validation_value = self.pedal_pressed + self.validation_target * (self.pedal_max - self.pedal_pressed) / 100.0
        self.pedal_last_value_sent = self.validation_value
      elif self.validation_count < self.validation_steps:
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

    elif self.status == 6:
      p("")
      p("CALIBRATION RESULTS")
      p(f"  Pedal Min:    {self.pedal_min:.2f}")
      p(f"  Pedal Max:    {self.pedal_max:.2f}")
      p(f"  Pedal Zero:   {self.pedal_pressed:.2f}")
      p(f"  Pedal Factor: {self.pedal_factor:.4f}")
      persist_calibration(
        self.params,
        min_v=self.pedal_min,
        max_v=self.pedal_max,
        factor=self.pedal_factor,
        zero=self.pedal_pressed,
      )
      p("  Saved to openpilot Params")
      self.status = 7

  def run(self, rate=100) -> int:
    self.status = 2
    p("")
    p("Waiting for safety conditions...")
    p("  - Car ON")
    p("  - Gear in NEUTRAL")
    p("  - Brake PRESSED")
    p("  - Accelerator RELEASED")
    p(f"  [TX: Bus {self.pedal_can}, ID 0x{GAS_COMMAND_ID:03X}, teslaPreap calibration]")
    loop_period = 1.0 / rate
    while True:
      loop_start = time.monotonic()
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
      if curr_time_ms - self.last_pedal_sent_ms >= SEND_RATE_MS:
        self.send_pedal_command(self.pedal_last_value_sent, self.pedal_enabled)
        if self.tx_count % 100 == 0:
          p(f"  [TX #{self.tx_count}: val={self.pedal_last_value_sent:.1f}, en={self.pedal_enabled}]")
      if self.rcv_pedal_idx != self.last_rcv_pedal_idx:
        self.last_rcv_pedal_idx = self.rcv_pedal_idx
        self._advance_on_pedal_message()
      self.pedal_timeout = curr_time_ms - self.last_pedal_seen_ms > PEDAL_TIMEOUT_MS
      if self.pedal_timeout:
        self.pedal_error_count += 1
        if self.pedal_error_count > MAX_PEDAL_ERRORS * 10:
          p("\nERROR: Pedal communication timeout!")
          return 1
      elapsed = time.monotonic() - loop_start
      if elapsed < loop_period:
        time.sleep(loop_period - elapsed)


def run(*, confirmed: bool, params: Params | None = None, transport: DiagnosticTransport | None = None) -> int:
  params = params or Params()
  require_preap_tool_start(params, tool="calibrate_pedal", confirmed=confirmed)
  if not params.get_bool("NAPPedalEnabled"):
    raise PedalCalibrationError("no pedal configured")
  bus = parse_configured_pedal_bus(params.get("NAPPedalCanBus"))
  if bus not in (0, 2):
    raise PedalCalibrationError("invalid pedal bus")

  owned = transport is None
  transport = transport or DiagnosticTransport()
  calibrator = None
  try:
    transport.connect()
    transport.set_pedal_calibration_session(bus)
    calibrator = PedalCalibrator(params, transport, bus)
    return calibrator.run()
  except TransportError as exc:
    raise PedalCalibrationError(str(exc)) from exc
  finally:
    if calibrator is not None:
      calibrator.cleanup()
    elif transport is not None:
      try:
        send_safe_release(transport, bus)
      except Exception:
        pass
    if owned and transport is not None:
      transport.close()


def main(argv=None) -> int:
  p("=" * 60)
  p("COMMA PEDAL CALIBRATION")
  p("=" * 60)
  p("Tesla Pre-AP calibration safety only. ELM327/ALLOUTPUT are rejected.")
  try:
    return run(confirmed=parse_explicit_confirmation(argv))
  except KeyboardInterrupt:
    p("\nInterrupted by user.")
    return 1
  except Exception as exc:
    p(f"ERROR: {exc}")
    return 1


if __name__ == "__main__":
  raise SystemExit(main())
