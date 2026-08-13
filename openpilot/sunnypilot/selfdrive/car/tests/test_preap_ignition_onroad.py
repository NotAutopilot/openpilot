import unittest

from pathlib import Path


class TestPreAPIgnitionOnroadContract(unittest.TestCase):
  def test_pandad_and_manager_treat_ignition_can_as_onroad(self):
    root = Path("/home/jack/projects/personal/notautopilot/.worktrees/naponsp-port/openpilot")
    pandad = (root / "selfdrive/pandad/pandad.cc").read_text()
    manager = (root / "system/manager/manager.py").read_text()
    self.assertIn("health.ignition_can_pkt", pandad)
    self.assertIn("ignition_line_pkt", pandad)
    self.assertIn("ps.ignitionLine or ps.ignitionCan", manager)

    def harness_onroad(ignition_line: bool, ignition_can: bool, always_offroad: bool = False) -> bool:
      return ((ignition_line or ignition_can) and not always_offroad)

    self.assertTrue(harness_onroad(False, True))
    self.assertFalse(harness_onroad(False, False))
    self.assertTrue(harness_onroad(True, False))
    self.assertFalse(harness_onroad(True, True, always_offroad=True))


if __name__ == "__main__":
  unittest.main()
