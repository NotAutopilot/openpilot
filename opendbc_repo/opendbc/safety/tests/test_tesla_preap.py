#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.car.tesla.values import CruiseButtons
from opendbc.safety.tests.common import CANPackerPanda as CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py

# Safety param flags matching tesla_preap.h
PREAP_FLAG_LONG_CONTROL = 1
PREAP_FLAG_ENABLE_PEDAL = 2


def _fix_epas_checksum(msg):
  """Compute Tesla byte-sum checksum for EPAS_sysStatus (checksum at byte 7)."""
  addr, data, bus = msg
  data = bytearray(data)
  chk = (addr & 0xFF) + ((addr >> 8) & 0xFF)
  for i in range(len(data)):
    if i != 7:
      chk += data[i]
  data[7] = chk & 0xFF
  return addr, bytes(data), bus


class TestTeslaPreAPStalkRearm(unittest.TestCase):
  TX_MSGS = [[0x488, 0], [0x2B9, 0], [0x214, 0], [0x551, 0], [0x551, 2], [0x45, 0], [0x659, 0]]
  cnt_epas = 0

  def setUp(self):
    self.safety = libsafety_py.libsafety
    flags = PREAP_FLAG_LONG_CONTROL | PREAP_FLAG_ENABLE_PEDAL
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, flags)
    self.safety.init_tests()
    self.packer = CANPackerSafety("tesla_preap")

  def _rx(self, msg):
    return self.safety.safety_rx_hook(msg)

  def _stalk_msg(self, lever_position):
    return self.packer.make_can_msg_panda("STW_ACTN_RQ", 0, {"SpdCtrlLvr_Stat": lever_position})

  def _epas_msg(self, hands_on_level, eac_status=1, eac_error_code=0):
    counter = self.__class__.cnt_epas % 16
    self.__class__.cnt_epas += 1
    values = {
      "EPAS_handsOnLevel": hands_on_level,
      "EPAS_eacStatus": eac_status,
      "EPAS_eacErrorCode": eac_error_code,
      "EPAS_internalSAS": 8192,
      "EPAS_sysStatusCounter": counter,
    }
    return self.packer.make_can_msg_panda("EPAS_sysStatus", 0, values,
                                           fix_checksum=_fix_epas_checksum)

  def _gear_msg(self, gear):
    return self.packer.make_can_msg_panda("DI_torque2", 0, {"DI_gear": gear})

  def _door_msg_closed(self):
    return self.packer.make_can_msg_panda("GTW_carState", 0, {})

  def test_stalk_reengages_after_steering_disengage(self):
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_cruise_engaged_prev())

    # Engage preconditions: gear=drive, doors closed
    self.assertTrue(self._rx(self._gear_msg(4)))
    self.assertTrue(self._rx(self._door_msg_closed()))

    # Initial stalk pull engages controls
    self.assertTrue(self._rx(self._stalk_msg(CruiseButtons.MAIN)))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_cruise_engaged_prev())

    # Steering disengage drops controls and cruise edge state
    self.assertTrue(self._rx(self._epas_msg(hands_on_level=3)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_cruise_engaged_prev())

    # Clear EPAS override
    self.assertTrue(self._rx(self._epas_msg(hands_on_level=0)))
    self.assertFalse(self.safety.get_controls_allowed())

    # Re-engage via stalk pull
    self.assertTrue(self._rx(self._stalk_msg(CruiseButtons.MAIN)))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_cruise_engaged_prev())


if __name__ == "__main__":
  unittest.main()
