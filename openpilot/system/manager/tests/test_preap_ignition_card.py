import unittest

from openpilot.cereal import log
from opendbc.car import structs
from openpilot.system.manager.process_config import only_onroad, procs


class TestPreAPIgnitionCardStart(unittest.TestCase):
  def test_card_is_onroad_only(self):
    card = next(proc for proc in procs if proc.name == "card")
    self.assertEqual(card.module, "openpilot.selfdrive.car.card")
    self.assertIs(card.should_run, only_onroad)
    self.assertTrue(only_onroad(True, None, structs.CarParams()))
    self.assertFalse(only_onroad(False, None, structs.CarParams()))

  def test_manager_ignition_uses_pandaState_ignitionCan(self):
    ps = log.PandaState.new_message()
    ps.ignitionCan = True
    ps.ignitionLine = False
    ps.pandaType = log.PandaState.PandaType.uno
    ignition = any(p.ignitionLine or p.ignitionCan for p in [ps] if p.pandaType != log.PandaState.PandaType.unknown)
    self.assertTrue(ignition)
    ps.ignitionCan = False
    ignition = any(p.ignitionLine or p.ignitionCan for p in [ps] if p.pandaType != log.PandaState.PandaType.unknown)
    self.assertFalse(ignition)


if __name__ == "__main__":
  unittest.main()
