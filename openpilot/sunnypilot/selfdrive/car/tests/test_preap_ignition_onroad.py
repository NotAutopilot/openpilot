import unittest

from panda import Panda

from openpilot.cereal import log
from opendbc.car import structs
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.system.manager.process_config import only_onroad, procs
from openpilot.system.hardware.hardwared import ignition_from_panda_states


HEALTH_STRUCT = Panda.HEALTH_STRUCT
IGNITION_CAN_INDEX = 9


def _panda_state_from_health(ignition_can, *, panda_type=None):
  health_fields = [0] * len(HEALTH_STRUCT.unpack(bytes(HEALTH_STRUCT.size)))
  health_fields[IGNITION_CAN_INDEX] = int(ignition_can)
  health = HEALTH_STRUCT.unpack(HEALTH_STRUCT.pack(*health_fields))

  ps = log.PandaState.new_message()
  ps.ignitionCan = bool(health[IGNITION_CAN_INDEX])
  ps.ignitionLine = False
  ps.pandaType = log.PandaState.PandaType.uno if panda_type is None else panda_type
  return ps




class TestPreAPIgnitionOnroadContract(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()
    for _ in range(4):
      self.safety.ignition_can_1hz_tick()
    self.safety.init_tests()
    self.packer = CANPackerSafety("tesla_preap")

  def _msg(self, counter, drive_rail, bus=0):
    return self.packer.make_can_msg_safety(
      "GTW_status", bus, {"GTW_statusCounter": counter, "GTW_driveRailReq": int(drive_rail)},
    )

  def test_valid_0x348_sets_pandastate_and_starts_card(self):
    self.safety.ignition_can_hook(self._msg(0, 1))
    self.safety.ignition_can_hook(self._msg(1, 1))
    self.assertTrue(self.safety.get_ignition_can())

    ps = _panda_state_from_health(self.safety.get_ignition_can())
    started = ignition_from_panda_states([ps])
    self.assertTrue(started)

    card = next(proc for proc in procs if proc.name == "card")
    self.assertIs(card.should_run, only_onroad)
    self.assertTrue(only_onroad(started, None, structs.CarParams()))

  def test_ignition_off_stops_card(self):
    self.safety.ignition_can_hook(self._msg(0, 1))
    self.safety.ignition_can_hook(self._msg(1, 1))
    self.safety.ignition_can_hook(self._msg(2, 0))
    self.safety.ignition_can_hook(self._msg(3, 0))
    self.assertFalse(self.safety.get_ignition_can())
    ps = _panda_state_from_health(self.safety.get_ignition_can())
    started = ignition_from_panda_states([ps])
    card = next(proc for proc in procs if proc.name == "card")
    self.assertFalse(only_onroad(started, None, structs.CarParams()))
    self.assertIs(card.should_run, only_onroad)

  def test_unknown_panda_cannot_start_card(self):
    ps = _panda_state_from_health(True, panda_type=log.PandaState.PandaType.unknown)
    started = ignition_from_panda_states([ps])
    self.assertFalse(started)
    self.assertFalse(only_onroad(started, None, structs.CarParams()))


if __name__ == "__main__":
  unittest.main()
