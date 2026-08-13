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
from openpilot.cereal import custom

EventNameSP = custom.OnroadEventSP.EventName


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


  def test_version1_preap_does_not_emit_lkas_from_cruise_available(self):
    from openpilot.sunnypilot.mads.mads import ModularAssistiveDrivingSystem
    from openpilot.selfdrive.selfdrived.events import Events
    from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

    CP = structs.CarParams()
    CP.brand = "tesla"
    CP.carFingerprint = "TESLA_MODEL_S_PREAP"
    CP_SP = structs.CarParamsSP()
    CP_SP.madsCapabilityContractVersion = 1
    CP_SP.madsRequired = True
    CP_SP.madsMainCruiseInputKind = structs.CarParamsSP.MadsMainCruiseInputKind.momentary
    CP_SP.madsMainCruiseAllowed = True
    CP_SP.madsUnifiedEngagementMode = False
    CP_SP.madsSteeringMode = structs.CarParamsSP.MadsSteeringMode.remainActive

    params = MagicMock()
    params.get_bool.side_effect = lambda k: {"Mads": False, "MadsMainCruiseAllowed": True}.get(k, False)
    sd = MagicMock()
    sd.CP = CP
    sd.CP_SP = CP_SP
    sd.params = params
    sd.events = Events()
    sd.events_sp = EventsSP()
    sd.enabled = False
    sd.enabled_prev = False
    sd.initialized = True
    prev = structs.CarState()
    prev.cruiseState.available = False
    sd.CS_prev = prev
    sd.sm = {"pandaStates": []}

    mads = ModularAssistiveDrivingSystem(sd)
    self.assertTrue(mads.no_main_cruise)
    self.assertTrue(mads.enabled_toggle)
    cs = structs.CarState()
    cs.cruiseState.available = True
    mads.update(cs)
    self.assertFalse(sd.events_sp.has(EventNameSP.lkasEnable))

    frozen_main = mads.main_enabled_toggle
    params.get_bool.side_effect = lambda k: False
    mads.read_params()
    self.assertEqual(mads.main_enabled_toggle, frozen_main)

    params.reset_mock()
    params.get_bool.side_effect = lambda k: False
    mads.read_params()
    self.assertTrue(mads.enabled_toggle)
    params.put_bool.assert_called_once_with("Mads", True, block=True)

  def test_nonrequired_version1_keeps_live_params(self):
    from openpilot.sunnypilot.mads.mads import ModularAssistiveDrivingSystem
    from openpilot.selfdrive.selfdrived.events import Events
    from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

    CP = structs.CarParams()
    CP.brand = "hyundai"
    CP_SP = structs.CarParamsSP()
    CP_SP.madsCapabilityContractVersion = 1
    CP_SP.madsRequired = False
    values = {"Mads": True, "MadsMainCruiseAllowed": True, "MadsUnifiedEngagementMode": False}
    params = MagicMock()
    params.get_bool.side_effect = lambda key: values.get(key, False)
    params.get.return_value = 0
    sd = MagicMock()
    sd.CP = CP
    sd.CP_SP = CP_SP
    sd.params = params
    sd.events = Events()
    sd.events_sp = EventsSP()
    sd.enabled = False
    sd.enabled_prev = False
    sd.initialized = True
    prev = structs.CarState()
    prev.cruiseState.available = False
    sd.CS_prev = prev
    sd.sm = {"pandaStates": []}
    mads = ModularAssistiveDrivingSystem(sd)
    self.assertTrue(mads.main_enabled_toggle)
    self.assertFalse(mads.unified_engagement_mode)

    values["MadsMainCruiseAllowed"] = False
    values["MadsUnifiedEngagementMode"] = True
    mads.read_params()
    self.assertFalse(mads.main_enabled_toggle)
    self.assertTrue(mads.unified_engagement_mode)

    cs = structs.CarState()
    cs.cruiseState.available = True
    mads.update(cs)
    self.assertFalse(sd.events_sp.has(EventNameSP.lkasEnable))


if __name__ == "__main__":
  unittest.main()
