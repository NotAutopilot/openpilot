#!/usr/bin/env python3
"""Tests for the VIN-learn runner's CAN plumbing.

Not under pyproject's `testpaths`, so run it explicitly:
    pytest scripts/nap/tests/test_vin_learn_radar.py
"""
import unittest

from opendbc.car.can_definitions import CanData
from opendbc.car.tesla.preap.radar_vin import RADAR_BUS, RADAR_RX_ADDRESS, RADAR_TX_ADDRESS
from scripts.nap.vin_learn_radar import recv


class FakePanda:
  def __init__(self, msgs):
    self._msgs = msgs

  def can_recv(self):
    return self._msgs


class TestRecv(unittest.TestCase):
  def test_passes_through_received_traffic(self):
    panda = FakePanda([
      (0x405, b"\x10\x00\x00\x00\x00ABC", 0),
      (RADAR_RX_ADDRESS, b"\x03\x62\xf1\x90\x00\x00\x00\x00", RADAR_BUS),
    ])
    packets, rejected = recv(panda)
    self.assertFalse(rejected)
    self.assertEqual(packets, [
      CanData(0x405, b"\x10\x00\x00\x00\x00ABC", 0),
      CanData(RADAR_RX_ADDRESS, b"\x03\x62\xf1\x90\x00\x00\x00\x00", RADAR_BUS),
    ])

  def test_drops_loopback_echo_of_our_own_sends(self):
    # The panda returns what we transmitted with the bus offset by 128. Feeding
    # that to the learner would look like the radar answering its own request.
    panda = FakePanda([(RADAR_TX_ADDRESS, b"\x02\x10\x03\x00\x00\x00\x00\x00", RADAR_BUS + 128)])
    packets, rejected = recv(panda)
    self.assertEqual(packets, [])
    self.assertFalse(rejected)

  def test_flags_safety_rejected_sends(self):
    # 192 offset = the safety layer refused the TX, i.e. panda firmware without
    # PREAP_FLAG_RADAR_VIN_LEARN.
    panda = FakePanda([(RADAR_TX_ADDRESS, b"\x02\x10\x03\x00\x00\x00\x00\x00", RADAR_BUS + 192)])
    packets, rejected = recv(panda)
    self.assertEqual(packets, [])
    self.assertTrue(rejected)

  def test_flags_rejected_when_also_returned(self):
    panda = FakePanda([(RADAR_TX_ADDRESS, b"\x02\x10\x03\x00\x00\x00\x00\x00", RADAR_BUS + 128 + 192)])
    packets, rejected = recv(panda)
    self.assertEqual(packets, [])
    self.assertTrue(rejected)

  def test_accepts_four_tuple_shape(self):
    panda = FakePanda([(0x405, None, b"\x11\x00\x00\x00\x00\x00\x00\x00", 0)])
    packets, _ = recv(panda)
    self.assertEqual(packets, [CanData(0x405, b"\x11\x00\x00\x00\x00\x00\x00\x00", 0)])

  def test_skips_unknown_tuple_shapes(self):
    panda = FakePanda([(0x405, 0), (0x405, 0, b"\x00", 0, 0)])
    packets, rejected = recv(panda)
    self.assertEqual(packets, [])
    self.assertFalse(rejected)


if __name__ == "__main__":
  unittest.main()
