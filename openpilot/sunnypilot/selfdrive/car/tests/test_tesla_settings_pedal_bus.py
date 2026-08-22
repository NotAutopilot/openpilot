"""Tesla native settings pedal-bus selector. Does not import the UI module (raylib)."""
import ast
import unittest
from pathlib import Path

TESLA_PY = (
  Path(__file__).resolve().parents[4]
  / "selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/tesla.py"
)


def _load_tesla_bus_helpers():
  src = TESLA_PY.read_text()
  tree = ast.parse(src)
  wanted = {"parse_configured_pedal_bus", "pedal_bus_selector_index"}
  body: list[ast.stmt] = [
    node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in wanted
  ]
  names = {node.name for node in body}
  if wanted - names:
    raise AssertionError(f"tesla.py missing helpers {wanted - names}")
  mod = ast.Module(body=body, type_ignores=[])
  ast.fix_missing_locations(mod)
  ns = {}
  exec(compile(mod, str(TESLA_PY), "exec"), ns)
  return ns


class TestTeslaSettingsPedalBus(unittest.TestCase):
  def test_parse_preserves_zero_and_defaults_empty(self):
    parse = _load_tesla_bus_helpers()["parse_configured_pedal_bus"]
    self.assertEqual(parse(0), 0)
    self.assertEqual(parse("0"), 0)
    self.assertEqual(parse(b"0"), 0)
    self.assertEqual(parse(None), 2)
    self.assertEqual(parse(""), 2)
    self.assertEqual(parse(b""), 2)
    self.assertEqual(parse(2), 2)

  def test_selector_index_maps_bus_zero_to_first_button(self):
    index = _load_tesla_bus_helpers()["pedal_bus_selector_index"]
    self.assertEqual(index(0), 0)
    self.assertEqual(index("0"), 0)
    self.assertEqual(index(b"0"), 0)
    self.assertEqual(index(None), 1)
    self.assertEqual(index(""), 1)
    self.assertEqual(index(b""), 1)
    self.assertEqual(index(2), 1)

  def test_status_refresh_uses_named_helper_not_falsy_or(self):
    src = TESLA_PY.read_text()
    self.assertNotIn('params.get("NAPPedalCanBus") or', src)
    self.assertTrue("pedal_bus_selector_index(ui_state.params.get(\"NAPPedalCanBus\"))" in src)
    self.assertTrue("ui_state.params.put(\"NAPPedalCanBus\", 0 if index == 0 else 2)" in src)

  def test_tesla_menu_does_not_own_mads_engagement(self):
    src = TESLA_PY.read_text()
    self.assertNotIn("NAPLateralEngagementMode", src)
    self.assertNotIn("Lateral Engagement Mode", src)
    self.assertNotIn("Active Engagement Mode", src)

  def test_preap_uses_native_status_and_steppers(self):
    src = TESLA_PY.read_text()
    self.assertIn("option_item_sp(", src)
    self.assertIn("text_item_sp(", src)
    self.assertIn('param="NAPFollowDistance"', src)
    self.assertIn('param="NAPRadarOffset"', src)
    self.assertNotIn("button_width=90", src)
    self.assertNotIn("Keyboard(", src)
    self.assertNotIn("yRel", src)
    self.assertIn("label_width=style.BUTTON_ACTION_WIDTH", src)
    self.assertIn("button_width=364", src)
    self.assertIn('lambda: tr("CALIBRATE")', src)
    self.assertIn('lambda: tr("FLASH")', src)
    self.assertNotIn("self.follow_distance.action_item.set_enabled(offroad)", src)


if __name__ == "__main__":
  unittest.main()
