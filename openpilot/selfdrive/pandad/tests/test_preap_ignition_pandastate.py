import unittest

from openpilot.cereal import log
from opendbc.car import structs
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.system.manager.process_config import only_onroad, procs


def _panda_state(ignition_can):
  ps = log.PandaState.new_message()
  ps.ignitionCan = ignition_can
  ps.ignitionLine = False
  ps.pandaType = log.PandaState.PandaType.uno
  return ps


class TestPreAPIgnitionPandaState(unittest.TestCase):
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

  def test_ignition_can_pkt_feeds_pandastate_ignitionCan(self):
    self.safety.ignition_can_hook(self._msg(0, 1))
    self.safety.ignition_can_hook(self._msg(1, 1))
    ps = _panda_state(bool(self.safety.get_ignition_can()))
    self.assertTrue(ps.ignitionCan)
    started = any(p.ignitionLine or p.ignitionCan for p in [ps] if p.pandaType != log.PandaState.PandaType.unknown)
    card = next(proc for proc in procs if proc.name == "card")
    self.assertTrue(only_onroad(started, None, structs.CarParams()))
    self.assertIs(card.should_run, only_onroad)


if __name__ == "__main__":
  unittest.main()
