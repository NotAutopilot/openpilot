"""Native MADS settings own Pre-AP engagement. Does not import the UI module (raylib)."""
import unittest
from pathlib import Path

STEERING_PY = (
  Path(__file__).resolve().parents[4]
  / "selfdrive/ui/sunnypilot/layouts/settings/steering.py"
)
MADS_SETTINGS_PY = (
  Path(__file__).resolve().parents[4]
  / "selfdrive/ui/sunnypilot/layouts/settings/steering_sub_layouts/mads_settings.py"
)


class TestNativeMadsPreapSettings(unittest.TestCase):
  def test_required_mads_copy_on_steering(self):
    src = STEERING_PY.read_text()
    assert "This platform requires MADS." in src
    assert "Customize MADS" in src

  def test_preap_mads_settings_copy(self):
    src = MADS_SETTINGS_PY.read_text()
    assert "This platform requires MADS." in src
    assert "Stalk pull engages steering" in src
    assert "Cruise Coupled" not in src
    assert "Longitudinal Only" not in src


if __name__ == "__main__":
  unittest.main()
