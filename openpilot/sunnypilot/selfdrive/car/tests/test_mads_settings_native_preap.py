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
    assert 'param="NAPLateralEngagementMode"' in src
    assert 'title=lambda: tr("Lateral Engagement Mode")' in src
    assert 'tr("Independent")' in src
    assert 'tr("Cruise Coupled")' in src
    assert 'tr("Longitudinal Only")' in src
    assert "self._engagement_mode.set_visible(is_preap)" in src

  def test_required_mads_copy_on_steering(self):
    src = STEERING_PY.read_text()
    assert "This platform requires MADS." in src
    assert "Customize MADS" in src


if __name__ == "__main__":
  unittest.main()
