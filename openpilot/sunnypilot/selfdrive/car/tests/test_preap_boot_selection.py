import json
import os
import unittest
from unittest.mock import MagicMock, patch
from openpilot.common.params import Params

from opendbc.car import structs
from opendbc.car.car_helpers import get_car
from opendbc.car.tesla.preap.boot import PREAP_PLATFORM
from openpilot.sunnypilot.selfdrive.car.preap_boot import (
  is_preap_ui_platform,
  migrate_preap_engagement_mode,
  seed_preap_installer,
  resolve_card_boot,
  snapshot_param_list,
)
from openpilot.sunnypilot.selfdrive.car.sync_sunnylink_params import CAR_LIST_JSON_OUT, build_platform_bundle


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
  def test_selection_and_mode_have_no_manager_defaults(self):
    params = Params()
    self.assertIsNone(params.get_default_value("NAPForcePreAP"))
    self.assertIsNone(params.get_default_value("NAPLateralEngagementMode"))

    params.put_bool("MadsMainCruiseAllowed", False, block=True)
    params.put_bool("MadsUnifiedEngagementMode", True, block=True)
    self.assertEqual(migrate_preap_engagement_mode(params), 1)

    params.remove("NAPForcePreAP")
    self.assertTrue(seed_preap_installer(params, PREAP_PLATFORM))
    self.assertTrue(params.get("NAPForcePreAP"))

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

  def test_vehicle_selector_preap_bundle_routes_boot_and_ui(self):
    with open(CAR_LIST_JSON_OUT) as car_list_file:
      car_list = json.load(car_list_file)

    bundle = build_platform_bundle(car_list, "Tesla Model S (Pre-AP) 2012-14")
    assert bundle is not None
    self.assertEqual(bundle["name"], "Tesla Model S (Pre-AP) 2012-14")
    self.assertEqual(bundle["brand"], "tesla")
    params = FakeParams({"CarPlatformBundle": bundle})
    selection, fingerprint = resolve_card_boot(params, environ={})

    self.assertEqual(bundle["platform"], PREAP_PLATFORM)
    self.assertEqual(selection.candidate, PREAP_PLATFORM)
    self.assertEqual(fingerprint, PREAP_PLATFORM)
    self.assertTrue(selection.lock_preap)
    self.assertTrue(selection.skip_fw_query)
    self.assertTrue(params.get("NAPForcePreAP"))
    self.assertTrue(is_preap_ui_platform(bundle["platform"], None))

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
    params.store.pop("NAPLateralEngagementModeMigrated", None)
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

  def test_completed_migration_never_rereads_legacy_pair(self):
    params = FakeParams({
      "NAPLateralEngagementModeMigrated": True,
      "MadsMainCruiseAllowed": False,
      "MadsUnifiedEngagementMode": True,
    })
    self.assertEqual(migrate_preap_engagement_mode(params), 0)
    self.assertEqual(params.get("NAPLateralEngagementMode"), 0)
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
    assert "NAPLateralEngagementMode" in keys
    assert "NAPForcePreAP" in keys

  def test_unset_pedal_bus_and_calib_range_keep_calibrated_pedal(self):
    from opendbc.car import gen_empty_fingerprint
    from opendbc.car.tesla.interface import CarInterface
    from opendbc.car.tesla.preap.boot import apply_preap_hardware_snapshot, hardware_snapshot_from_values
    from opendbc.car.tesla.values import CAR
    from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP

    params = Params()
    for key in ("NAPPedalCanBus", "NAPPedalCalibMin", "NAPPedalCalibMax"):
      params.remove(key)
    params.put_bool("NAPPedalEnabled", True, block=True)
    params.put_bool("NAPPedalCalibDone", True, block=True)
    params.put("NAPPedalCalibFactor", 0.035, block=True)
    params.put("NAPPedalCalibZero", 0.25, block=True)

    merged = {k: v for row in snapshot_param_list(params) for k, v in row.items()}
    self.assertEqual(merged["NAPPedalCanBus"], 2)
    self.assertEqual(merged["NAPPedalCalibMin"], -3.0)
    self.assertEqual(merged["NAPPedalCalibMax"], 99.6)

    snapshot = hardware_snapshot_from_values(
      pedal_enabled=merged["NAPPedalEnabled"],
      pedal_bus=merged["NAPPedalCanBus"],
      pedal_calib_done=merged["NAPPedalCalibDone"],
      pedal_calib_factor=merged["NAPPedalCalibFactor"],
      pedal_calib_zero=merged["NAPPedalCalibZero"],
      pedal_calib_min=merged["NAPPedalCalibMin"],
      pedal_calib_max=merged["NAPPedalCalibMax"],
    )
    self.assertTrue(snapshot.pedal_present)
    self.assertTrue(snapshot.pedal_calib_available)
    self.assertEqual(snapshot.pedal_bus, 2)

    CP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
    apply_preap_hardware_snapshot(CP, CP_SP, snapshot)
    self.assertTrue(CP.openpilotLongitudinalControl)
    self.assertFalse(CP.pcmCruise)
    self.assertTrue(bool(CP_SP.flags & TeslaFlagsSP.PREAP_PEDAL_PRESENT))
    self.assertTrue(bool(CP_SP.flags & TeslaFlagsSP.PREAP_PEDAL_CALIB_AVAILABLE))

  def test_get_car_from_raw_bundle_and_params(self):
    params = FakeParams({"CarPlatformBundle": {"platform": PREAP_PLATFORM}})
    sel, fingerprint = resolve_card_boot(params, environ={})
    CI = _get_car(sel, params)
    self.assertEqual(CI.CP.carFingerprint, PREAP_PLATFORM)
    self.assertEqual(fingerprint, PREAP_PLATFORM)
    self.assertEqual(CI.CP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.teslaPreap)

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


  def test_hostile_upgrade_forces_mads_and_disables_coop(self):
    from opendbc.car.tesla.preap.constants import STALK_DOUBLE_PULL_MS
    params = FakeParams({
      "Mads": False,
      "TeslaCoopSteering": True,
      "CarPlatformBundle": {"platform": PREAP_PLATFORM},
    })
    sel, fingerprint = resolve_card_boot(params, environ={})
    self.assertTrue(params.get("Mads"))
    self.assertEqual(fingerprint, PREAP_PLATFORM)
    self.assertEqual(STALK_DOUBLE_PULL_MS, 400)
    CI = _get_car(sel, params)
    self.assertTrue(CI.CP_SP.madsRequired)
    self.assertFalse(CI.CP_SP.teslaCoopSteeringAvailable)
    snap = snapshot_param_list(params)
    merged = {k: v for row in snap for k, v in row.items()}
    self.assertTrue(merged["Mads"])
    self.assertFalse(merged["TeslaCoopSteering"])

  def test_modern_tesla_absent_force_preap_unchanged(self):
    params = FakeParams({
      "Mads": False,
      "TeslaCoopSteering": True,
      "CarPlatformBundle": {"platform": "TESLA_MODEL_3"},
    })
    sel, fingerprint = resolve_card_boot(params, environ={})
    self.assertIsNone(params.get("NAPForcePreAP"))
    self.assertFalse(params.get("Mads"))
    self.assertTrue(params.get("TeslaCoopSteering"))
    self.assertEqual(fingerprint, "TESLA_MODEL_3")
    CI = _get_car(sel, params)
    self.assertEqual(CI.CP.carFingerprint, "TESLA_MODEL_3")
    self.assertNotEqual(CI.CP.carFingerprint, PREAP_PLATFORM)


if __name__ == "__main__":
  unittest.main()
