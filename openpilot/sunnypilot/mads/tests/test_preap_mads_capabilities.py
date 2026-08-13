import unittest
from unittest.mock import MagicMock

from opendbc.car import structs
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP
from openpilot.sunnypilot.mads.helpers import (
  MadsSteeringModeOnBrake,
  resolve_mads_capabilities,
  set_alternative_experience,
  set_car_specific_params,
)
from opendbc.safety import ALTERNATIVE_EXPERIENCE


class TestPreAPMadsCapabilities(unittest.TestCase):
  def test_preap_version1_forces_mads_and_rejects_coop(self):
    CP = structs.CarParams()
    CP.brand = "tesla"
    CP.carFingerprint = "TESLA_MODEL_S_PREAP"
    CP_SP = structs.CarParamsSP()
    CP_SP.madsCapabilityContractVersion = 1
    CP_SP.madsRequired = True
    CP_SP.madsFullSettingsAvailable = True
    CP_SP.madsMainCruiseInputKind = structs.CarParamsSP.MadsMainCruiseInputKind.momentary
    CP_SP.teslaCoopSteeringAvailable = False
    CP_SP.madsSteeringMode = structs.CarParamsSP.MadsSteeringMode.pause
    CP_SP.madsHandsOnPauseAvailable = True

    params = MagicMock()
    params.get_bool.return_value = False
    caps = resolve_mads_capabilities(CP, CP_SP, params)
    self.assertTrue(caps.mads_required)
    self.assertTrue(caps.no_main_cruise)
    self.assertTrue(caps.full_settings_available)
    self.assertFalse(caps.tesla_coop_steering_available)
    self.assertEqual(caps.steering_mode, MadsSteeringModeOnBrake.PAUSE)

    set_alternative_experience(CP, CP_SP, params)
    params.put_bool.assert_any_call("Mads", True, block=True)
    self.assertTrue(CP.alternativeExperience & ALTERNATIVE_EXPERIENCE.ENABLE_MADS)
    self.assertTrue(CP.alternativeExperience & ALTERNATIVE_EXPERIENCE.MADS_PAUSE_LATERAL_ON_BRAKE)

    params.reset_mock()
    CP_SP.flags |= TeslaFlagsSP.COOP_STEERING
    set_car_specific_params(CP, CP_SP, params)
    params.put_bool.assert_any_call("Mads", True, block=True)

  def test_version0_tesla_and_rivian_unchanged(self):
    params = MagicMock()
    params.get_bool.side_effect = lambda k: {"Mads": True, "MadsMainCruiseAllowed": True, "MadsUnifiedEngagementMode": True}.get(k, False)
    params.get.return_value = 0

    tesla = structs.CarParams()
    tesla.brand = "tesla"
    tesla_sp = structs.CarParamsSP()
    caps = resolve_mads_capabilities(tesla, tesla_sp, params)
    self.assertFalse(caps.mads_required)
    self.assertTrue(caps.no_main_cruise)
    self.assertFalse(caps.full_settings_available)
    self.assertEqual(caps.steering_mode, MadsSteeringModeOnBrake.DISENGAGE)

    tesla_sp.flags |= TeslaFlagsSP.HAS_VEHICLE_BUS
    params.get.return_value = 1  # 3-finger screen button
    caps = resolve_mads_capabilities(tesla, tesla_sp, params)
    self.assertTrue(caps.full_settings_available)

    rivian = structs.CarParams()
    rivian.brand = "rivian"
    caps = resolve_mads_capabilities(rivian, structs.CarParamsSP(), params)
    self.assertTrue(caps.no_main_cruise)
    self.assertFalse(caps.mads_required)


if __name__ == "__main__":
  unittest.main()
