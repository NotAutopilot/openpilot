"""Bosch radar point decode used by production Pre-AP radar tools."""
from __future__ import annotations

RADAR_BUS = 1
NUM_RADAR_POINTS = 32
RADAR_ADDR_START = 0x310
RADAR_ADDR_END = 0x36F

MIN_LONG = 2.5
MAX_LONG = 14.5
MIN_LAT = -1.0
MAX_LAT = 1.0

MOVING_STATES = {
  0: "Unknown",
  1: "Moving",
  2: "Stopped",
  3: "Standing",
}


def extract_signal(data, start_bit: int, length: int) -> int:
  value = 0
  for i in range(length):
    bit_pos = start_bit + i
    byte_idx = bit_pos // 8
    bit_idx = bit_pos % 8
    if byte_idx < len(data) and (data[byte_idx] & (1 << bit_idx)):
      value |= (1 << i)
  return value


def parse_radar_a(data: bytes) -> dict:
  return {
    "long_dist": extract_signal(data, 0, 12) * 0.0625,
    "long_speed": extract_signal(data, 12, 12) * 0.0625 - 128,
    "lat_dist": extract_signal(data, 24, 11) * 0.125 - 128,
    "prob_exist": extract_signal(data, 35, 5) * 3.125,
    "long_accel": extract_signal(data, 40, 10) * 0.03125 - 16,
    "valid": extract_signal(data, 55, 1),
    "meas": extract_signal(data, 61, 1),
    "tracked": extract_signal(data, 62, 1),
    "index": extract_signal(data, 63, 1),
  }


def parse_radar_b(data: bytes) -> dict:
  return {
    "lat_speed": extract_signal(data, 0, 10) * 0.125 - 64,
    "length": extract_signal(data, 10, 6) * 0.125,
    "dz": extract_signal(data, 16, 6) * 0.25 - 5,
    "moving_state": extract_signal(data, 22, 2),
    "index2": extract_signal(data, 63, 1),
  }


def unpack_can(msg) -> tuple[int, bytes, int] | None:
  if len(msg) == 4:
    addr, _bus, dat, src = msg
    return int(addr), bytes(dat), int(src)
  if len(msg) == 3:
    addr, dat, src = msg
    return int(addr), bytes(dat), int(src)
  return None


def update_tracks(tracks: dict[int, dict], addr: int, dat: bytes) -> None:
  if not (RADAR_ADDR_START <= addr <= RADAR_ADDR_END):
    return
  idx = (addr - RADAR_ADDR_START) // 2
  if idx not in tracks:
    tracks[idx] = {}
  if addr % 2 == 0:
    tracks[idx].update(parse_radar_a(dat))
  else:
    tracks[idx].update(parse_radar_b(dat))


def in_calibration_window(point: dict) -> bool:
  if not point.get("tracked") and not point.get("valid"):
    return False
  long_dist = point.get("long_dist")
  lat_dist = point.get("lat_dist")
  if long_dist is None or lat_dist is None:
    return False
  return MIN_LONG <= long_dist <= MAX_LONG and MIN_LAT <= lat_dist <= MAX_LAT
