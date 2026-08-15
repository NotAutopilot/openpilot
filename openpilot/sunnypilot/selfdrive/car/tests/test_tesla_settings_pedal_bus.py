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
  body = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in wanted]
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
    self.assertIn("pedal_bus_selector_index(ui_state.params.get(\"NAPPedalCanBus\"))", src)
    self.assertIn("ui_state.params.put(\"NAPPedalCanBus\", 0 if index == 0 else 2)", src)


if __name__ == "__main__":
  unittest.main()
