from types import SimpleNamespace

from openpilot.selfdrive.ui.radar.bosch_status import (
  ADDR_ALERT,
  ADDR_CAR_CONFIG,
  ADDR_SGU,
  ADDR_VIN_FEED,
  ADDR_VIN_HOST,
  BoschRadarMonitor,
  decode_car_config,
  vin_mux_chars,
)


def _sgu(*, hw=0, sgu=0, dirty=0) -> bytes:
  dat = bytearray(6)
  if dirty:
    dat[5] |= 1 << 4
  if hw:
    dat[5] |= 1 << 5
  if sgu:
    dat[5] |= 1 << 6
  return bytes(dat)


def _alert(*bits: int) -> bytes:
  dat = bytearray(8)
  for bit in bits:
    dat[bit // 8] |= 1 << (bit % 8)
  return bytes(dat)


def _tracks(*points):
  pts = [
    SimpleNamespace(trackId=i, dRel=d, yRel=y, vRel=v, measured=True)
    for i, (d, y, v) in enumerate(points)
  ]
  errors = SimpleNamespace(canError=False, radarFault=False, radarUnavailableTemporary=False)
  return SimpleNamespace(points=pts, errors=errors)


def test_sgu_and_alert_bits():
  mon = BoschRadarMonitor()
  status = mon.update(0.0, [
    (ADDR_CAR_CONFIG, bytes.fromhex("4a85555300200010"), 129),
    (ADDR_SGU, _sgu(sgu=1), 1),
    (ADDR_ALERT, _alert(36, 41, 60), 1),
  ])
  assert status.sgu_fail is True
  assert status.hw_fail is False
  assert status.alerts == ("vinValidity", "xwdValidity", "radPositionMismatch")
  assert status.health_label == "REJECT"


def test_chassis_bus_is_ignored():
  mon = BoschRadarMonitor()
  status = mon.update(0.0, [
    (ADDR_SGU, _sgu(hw=1, sgu=1), 0),
    (ADDR_ALERT, _alert(36), 0),
  ])
  assert status.hw_fail is False
  assert status.alerts == ()


def test_returned_tx_still_counts():
  dat = bytes.fromhex("4a85555300200010")
  assert decode_car_config(dat) == (True, 0, 2)
  mon = BoschRadarMonitor()
  status = mon.update(0.0, [(ADDR_CAR_CONFIG, dat, 129)])
  assert status.awd is True
  assert status.position == 0
  assert status.epas_type == 2


def test_vin_mux():
  assert vin_mux_chars(bytes([0x10, 0, 0, 0, 0, 0x35, 0x59, 0x4A])) == (0, ("5", "Y", "J"))
  mon = BoschRadarMonitor()
  mon.update(0.0, [
    (ADDR_VIN_FEED, bytes([0x10, 0, 0, 0, 0, 0x35, 0x59, 0x4A]), 129),
    (ADDR_VIN_FEED, bytes([0x11, 0x53, 0x41, 0x31, 0x45, 0x34, 0x35, 0x46]), 129),
    (ADDR_VIN_FEED, bytes([0x12, 0x46, 0x31, 0x30, 0x38, 0x34, 0x38, 0x35]), 129),
  ])
  assert mon.status.vin == "5YJSA1E45FF108485"


def test_frozen_table():
  mon = BoschRadarMonitor()
  frozen = _tracks((20.0, 0.1, -2.0), (30.0, -0.4, -1.5), (40.0, 0.2, 0.0), (50.0, 1.0, -0.5))
  status = None
  for i in range(12):
    status = mon.update(i * 0.5, [], frozen)
  assert status is not None
  assert status.table_frozen is True
  assert status.health_label == "FROZEN"


def test_moving_table_is_live():
  mon = BoschRadarMonitor()
  status = None
  for i in range(12):
    tracks = _tracks((20.0 + i, 0.1, -2.0), (30.0, -0.4, -1.5), (40.0, 0.2, 0.0), (50.0, 1.0, -0.5))
    status = mon.update(i * 0.5, [(ADDR_SGU, _sgu(), 1)], tracks)
  assert status is not None
  assert status.table_frozen is False
  assert status.health_label == "LIVE"


def test_hw_fail_is_fault():
  mon = BoschRadarMonitor()
  status = mon.update(0.0, [
    (ADDR_CAR_CONFIG, bytes.fromhex("4295555310001710"), 129),
    (ADDR_SGU, _sgu(hw=1), 1),
  ], None)
  assert status.health_label == "FAULT"


def test_wait_vin_until_host_stream_and_gtw():
  mon = BoschRadarMonitor()
  status = mon.update(0.0, [], None)
  assert status.health_label == "WAIT VIN"

  status = mon.update(0.1, [
    (ADDR_VIN_HOST, bytes([0, 1, 1, 0, 0, 0x20, 0x20, 0x20]), 192),
    (ADDR_VIN_HOST, bytes([1, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20]), 192),
    (ADDR_VIN_HOST, bytes([2, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20]), 192),
  ], None)
  assert status.vin_stream_complete is True
  assert status.health_label == "WAIT GTW"

  status = mon.update(0.2, [(ADDR_CAR_CONFIG, bytes.fromhex("4295555310001710"), 129)], None)
  assert status.gtw_live is True
  assert status.health_label == "NO TRACKS"
