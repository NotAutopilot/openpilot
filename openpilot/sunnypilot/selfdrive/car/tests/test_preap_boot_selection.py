import unittest
from pathlib import Path
from unittest.mock import MagicMock

from openpilot.sunnypilot.selfdrive.car.preap_boot import (
  PREAP_PLATFORM,
  migrate_preap_engagement_mode,
  resolve_preap_boot_selection,
  seed_preap_installer,
  snapshot_param_list,
)


class TestPreAPBootSelection(unittest.TestCase):
  def test_explicit_bundle_wins_over_force(self):
    sel = resolve_preap_boot_selection(
      bundle_platform="TESLA_MODEL_3",
      env_fingerprint=None,
      nap_force_preap=True,
    )
    self.assertEqual(sel.candidate, "TESLA_MODEL_3")
    self.assertEqual(sel.source, "explicit")
    self.assertFalse(sel.lock_preap)
    self.assertFalse(sel.skip_fw_query)

  def test_env_override_locks_preap_and_skips_fw(self):
    sel = resolve_preap_boot_selection(
      bundle_platform=None,
      env_fingerprint=PREAP_PLATFORM,
      nap_force_preap=False,
    )
    self.assertEqual(sel.candidate, PREAP_PLATFORM)
    self.assertTrue(sel.lock_preap)
    self.assertTrue(sel.skip_fw_query)

  def test_persisted_force_true(self):
    sel = resolve_preap_boot_selection(nap_force_preap=True)
    self.assertEqual(sel.candidate, PREAP_PLATFORM)
    self.assertEqual(sel.source, "nap_force_preap")
    self.assertTrue(sel.skip_fw_query)

  def test_absent_or_false_uses_normal_flow(self):
    for value in (None, False, 0, "0", ""):
      sel = resolve_preap_boot_selection(nap_force_preap=value)
      self.assertIsNone(sel.candidate)
      self.assertEqual(sel.source, "normal")
      self.assertFalse(sel.lock_preap)
      self.assertFalse(sel.skip_fw_query)

  def test_installer_seeds_only_when_absent(self):
    params = MagicMock()
    params.get.return_value = None
    self.assertTrue(seed_preap_installer(params, PREAP_PLATFORM))
    params.put_bool.assert_called_once_with("NAPForcePreAP", True, block=True)

    params = MagicMock()
    params.get.return_value = False
    self.assertFalse(seed_preap_installer(params, PREAP_PLATFORM))

    params = MagicMock()
    params.get.return_value = None
    self.assertFalse(seed_preap_installer(params, "TESLA_MODEL_3"))

  def test_migrate_main_uem_once(self):
    stored = {}

    def get(key):
      return stored.get(key)

    def put(key, value, block=True):
      stored[key] = value

    params = MagicMock()
    params.get.side_effect = get
    params.put.side_effect = put
    params.put_bool.side_effect = lambda k, v, block=True: stored.__setitem__(k, v)

    stored["MadsMainCruiseAllowed"] = True
    stored["MadsUnifiedEngagementMode"] = False
    self.assertEqual(migrate_preap_engagement_mode(params), 0)
    self.assertEqual(stored["NAPLateralEngagementMode"], 0)
    self.assertTrue(stored["NAPLateralEngagementModeMigrated"])

    stored["MadsMainCruiseAllowed"] = False
    stored["MadsUnifiedEngagementMode"] = True
    # canonical already present: do not reread pair
    self.assertEqual(migrate_preap_engagement_mode(params), 0)

    stored.pop("NAPLateralEngagementMode")
    stored.pop("NAPLateralEngagementModeMigrated")
    self.assertEqual(migrate_preap_engagement_mode(params), 1)

    stored.pop("NAPLateralEngagementMode")
    stored["MadsMainCruiseAllowed"] = False
    stored["MadsUnifiedEngagementMode"] = False
    self.assertEqual(migrate_preap_engagement_mode(params), 2)

    stored.clear()
    self.assertEqual(migrate_preap_engagement_mode(params), 0)

  def test_napadaptiveaccel_not_registered(self):
    keys = Path("openpilot/common/params_keys.h").read_text() if Path("openpilot/common/params_keys.h").exists() else \
      Path("/home/jack/projects/personal/notautopilot/.worktrees/naponsp-port/openpilot/common/params_keys.h").read_text()
    self.assertNotIn('{"NAPAdaptiveAccel"', keys)
    self.assertIn('{"NAPForcePreAP"', keys)
    self.assertIn('{"NAPLateralEngagementMode"', keys)


class TestPreAPCardWiring(unittest.TestCase):
  def test_card_wires_selection_before_fw_query(self):
    card = Path("/home/jack/projects/personal/notautopilot/.worktrees/naponsp-port/openpilot/selfdrive/car/card.py").read_text()
    helpers = Path("/home/jack/projects/personal/notautopilot/.worktrees/naponsp-port/opendbc_repo/opendbc/car/car_helpers.py").read_text()
    self.assertIn("preap_boot.seed_preap_installer", card)
    self.assertIn("preap_boot.resolve_preap_boot_selection", card)
    self.assertIn("selection.skip_fw_query", card)
    self.assertLess(card.index("preap_boot.resolve_preap_boot_selection"), card.index("get_car("))
    self.assertLess(card.index("seed_preap_installer"), card.index("resolve_preap_boot_selection"))
    self.assertIn("skip_fw_query: bool | None = None", helpers)
    self.assertNotIn("candidate=PREAP_PLATFORM", card.split("get_car")[1][:400])


class TestPreAPSnapshotKeys(unittest.TestCase):
  def test_snapshot_does_not_include_adaptive_accel(self):
    params = MagicMock()
    params.get.return_value = None
    blob = snapshot_param_list(params)
    keys = {k for d in blob for k in d}
    self.assertNotIn("NAPAdaptiveAccel", keys)
    self.assertIn("NAPForcePreAP", keys)
    self.assertIn("NAPLateralEngagementMode", keys)


if __name__ == "__main__":
  unittest.main()
