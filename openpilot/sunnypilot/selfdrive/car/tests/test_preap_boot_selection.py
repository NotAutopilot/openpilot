import os
import unittest
from unittest.mock import MagicMock, patch

from opendbc.car import structs
from opendbc.car.car_helpers import get_car
from opendbc.car.tesla.preap.boot import PREAP_PLATFORM
from openpilot.sunnypilot.selfdrive.car.preap_boot import (
  migrate_preap_engagement_mode,
  resolve_card_boot,
  snapshot_param_list,
)


class FakeParams:
  def __init__(self, initial=None):
    self.store = dict(initial or {})

  def get(self, key, return_default=False):
    return self.store.get(key)

  def put(self, key, value, block=True):
    self.store[key] = value

  def put_bool(self, key, value, block=True):
    self.store[key] = bool(value)


def _idle_can(wait_for_one=False):
  return [[]]


def _send(_msgs):
  return None


def _obd(_enabled):
  return None


def _get_car(selection, params, extra_params=None):
  init = snapshot_param_list(params)
  if extra_params:
    init.extend(extra_params)
  return get_car(_idle_can, _send, _obd, False, False, None,
                 selection.candidate if selection.candidate is not None else None,
                 init, False, selection.skip_fw_query)


class TestPreAPBootSelection(unittest.TestCase):
  def test_explicit_bundle_wins_over_force(self):
    params = FakeParams({"CarPlatformBundle": {"platform": "TESLA_MODEL_3"}, "NAPForcePreAP": True})
    sel, fingerprint = resolve_card_boot(params, environ={})
    self.assertEqual(sel.candidate, "TESLA_MODEL_3")
    self.assertEqual(fingerprint, "TESLA_MODEL_3")
    self.assertEqual(sel.source, "explicit")
    self.assertFalse(sel.lock_preap)
    self.assertIsNone(sel.skip_fw_query)

  def test_env_override_locks_preap_and_skips_fw(self):
    params = FakeParams()
    sel, fingerprint = resolve_card_boot(params, environ={"FINGERPRINT": PREAP_PLATFORM})
    self.assertEqual(sel.candidate, PREAP_PLATFORM)
    self.assertEqual(fingerprint, PREAP_PLATFORM)
    self.assertTrue(sel.lock_preap)
    self.assertTrue(sel.skip_fw_query)

  def test_persisted_force_true(self):
    params = FakeParams({"NAPForcePreAP": True})
    sel, fingerprint = resolve_card_boot(params, environ={})
    self.assertEqual(sel.candidate, PREAP_PLATFORM)
    self.assertEqual(sel.source, "nap_force_preap")
    self.assertTrue(sel.skip_fw_query)

  def test_absent_or_false_uses_normal_flow(self):
    for value in (None, False, 0, "0", ""):
      params = FakeParams({} if value is None else {"NAPForcePreAP": value})
      sel, fingerprint = resolve_card_boot(params, environ={})
      self.assertIsNone(sel.candidate)
      self.assertIsNone(fingerprint)
      self.assertEqual(sel.source, "normal")
      self.assertFalse(sel.lock_preap)
      self.assertIsNone(sel.skip_fw_query)

  def test_installer_seeds_only_when_absent(self):
    params = FakeParams({"CarPlatformBundle": {"platform": PREAP_PLATFORM}})
    sel, fingerprint = resolve_card_boot(params, environ={})
    self.assertTrue(params.get("NAPForcePreAP"))
    self.assertEqual(sel.candidate, PREAP_PLATFORM)
    self.assertTrue(sel.lock_preap)

    params = FakeParams({"CarPlatformBundle": {"platform": PREAP_PLATFORM}, "NAPForcePreAP": False})
    sel, fingerprint = resolve_card_boot(params, environ={})
    self.assertFalse(params.get("NAPForcePreAP"))
    # Explicit Pre-AP bundle still locks before FW; False only blocks installer reseeding.
    self.assertEqual(sel.candidate, PREAP_PLATFORM)
    self.assertEqual(sel.source, "explicit")
    self.assertTrue(sel.lock_preap)

    params = FakeParams({"CarPlatformBundle": {"platform": "TESLA_MODEL_3"}})
    sel, fingerprint = resolve_card_boot(params, environ={})
    self.assertIsNone(params.get("NAPForcePreAP"))
    self.assertEqual(sel.candidate, "TESLA_MODEL_3")
    self.assertFalse(sel.lock_preap)

  def test_migrate_main_uem_once(self):
    params = FakeParams({"MadsMainCruiseAllowed": True, "MadsUnifiedEngagementMode": False})
    self.assertEqual(migrate_preap_engagement_mode(params), 0)
    self.assertEqual(params.get("NAPLateralEngagementMode"), 0)
    self.assertTrue(params.get("NAPLateralEngagementModeMigrated"))

    params.store["MadsMainCruiseAllowed"] = False
    params.store["MadsUnifiedEngagementMode"] = True
    self.assertEqual(migrate_preap_engagement_mode(params), 0)

    params.store.pop("NAPLateralEngagementMode")
    params.store.pop("NAPLateralEngagementModeMigrated", None)
    self.assertEqual(migrate_preap_engagement_mode(params), 1)

    params.store.pop("NAPLateralEngagementMode")
    params.store["MadsMainCruiseAllowed"] = False
    params.store["MadsUnifiedEngagementMode"] = False
    self.assertEqual(migrate_preap_engagement_mode(params), 2)

    params.store.pop("NAPLateralEngagementMode")
    params.store.pop("NAPLateralEngagementModeMigrated", None)
    params.store["MadsMainCruiseAllowed"] = True
    params.store["MadsUnifiedEngagementMode"] = True
    self.assertEqual(migrate_preap_engagement_mode(params), 0)

    params = FakeParams()
    self.assertEqual(migrate_preap_engagement_mode(params), 0)
    self.assertTrue(params.get("NAPLateralEngagementModeMigrated"))

  def test_mode0_migration_requires_present_readback(self):
    stored = {}

    def get(key):
      return stored.get(key)

    def put(key, value, block=True):
      # Simulate a failed canonical persist: write is attempted but not readable.
      if key == "NAPLateralEngagementMode":
        return
      stored[key] = value

    params = MagicMock()
    params.get.side_effect = get
    params.put.side_effect = put
    params.put_bool.side_effect = lambda k, v, block=True: stored.__setitem__(k, v)
    self.assertEqual(migrate_preap_engagement_mode(params), 0)
    self.assertNotIn("NAPLateralEngagementModeMigrated", stored)
    self.assertNotIn("NAPLateralEngagementMode", stored)

  def test_napadaptiveaccel_not_in_boot_snapshot(self):
    keys = {k for d in snapshot_param_list(FakeParams()) for k in d}
    self.assertNotIn("NAPAdaptiveAccel", keys)
    self.assertIn("NAPLateralEngagementMode", keys)
    self.assertIn("NAPForcePreAP", keys)

  def test_get_car_from_raw_bundle_and_params(self):
    params = FakeParams({"CarPlatformBundle": {"platform": PREAP_PLATFORM}})
    sel, fingerprint = resolve_card_boot(params, environ={})
    CI = _get_car(sel, params)
    self.assertEqual(CI.CP.carFingerprint, PREAP_PLATFORM)
    self.assertEqual(fingerprint, PREAP_PLATFORM)
    self.assertEqual(CI.CP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.noOutput)

    params = FakeParams({"CarPlatformBundle": {"platform": "TESLA_MODEL_3"}})
    sel, fingerprint = resolve_card_boot(params, environ={})
    CI = _get_car(sel, params)
    self.assertEqual(CI.CP.carFingerprint, "TESLA_MODEL_3")
    self.assertNotEqual(CI.CP.carFingerprint, PREAP_PLATFORM)

  def test_normal_skip_fw_query_environment_fallback(self):
    params = FakeParams()
    sel, _fingerprint = resolve_card_boot(params, environ={})
    self.assertIsNone(sel.skip_fw_query)
    with patch.dict(os.environ, {"SKIP_FW_QUERY": "1"}, clear=False):
      with patch("opendbc.car.car_helpers.get_vin") as get_vin:
        CI = get_car(_idle_can, _send, _obd, False, False, None, "TESLA_MODEL_3",
                     snapshot_param_list(params), False, sel.skip_fw_query)
        get_vin.assert_not_called()
        self.assertEqual(CI.CP.carFingerprint, "TESLA_MODEL_3")


if __name__ == "__main__":
  unittest.main()
