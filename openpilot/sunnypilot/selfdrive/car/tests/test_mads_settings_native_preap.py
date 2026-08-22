"""Native MADS settings own Pre-AP engagement. Does not import the UI module (raylib)."""
import unittest
from pathlib import Path

MADS_PY = (
  Path(__file__).resolve().parents[4]
  / "selfdrive/ui/sunnypilot/layouts/settings/steering_sub_layouts/mads_settings.py"
)
STEERING_PY = (
  Path(__file__).resolve().parents[4]
  / "selfdrive/ui/sunnypilot/layouts/settings/steering.py"
)


class TestNativeMadsPreapSettings(unittest.TestCase):
  def test_engagement_mode_lives_in_mads_settings(self):
    src = MADS_PY.read_text()
    self.assertIn('param="NAPLateralEngagementMode"', src)
    self.assertIn('title=lambda: tr("Lateral Engagement Mode")', src)
    self.assertIn('tr("Independent")', src)
    self.assertIn('tr("Cruise Coupled")', src)
    self.assertIn('tr("Longitudinal Only")', src)
    self.assertIn("self._engagement_mode.set_visible(is_preap)", src)

  def test_required_mads_copy_on_steering(self):
    src = STEERING_PY.read_text()
    self.assertIn("This platform requires MADS.", src)
    self.assertIn("Customize MADS", src)


if __name__ == "__main__":
  unittest.main()
