"""Pre-AP safety configuration integrated with the host's persistent Params store."""

import pytest

from opendbc.car.car_helpers import interfaces
import opendbc.car.tesla.preap.nap_conf as nap_conf_mod
from opendbc.car.tesla.preap.constants import get_hands_on_disengage_level
from opendbc.car.tesla.preap.nap_params import NAPParamKeys
from opendbc.car.tesla.values import CAR
from openpilot.common.params import Params


@pytest.fixture
def preap_params(monkeypatch):
  # The parent test fixture creates an isolated OpenpilotPrefix for each test.
  params = Params()
  monkeypatch.setattr(nap_conf_mod, "_PARAMS_AVAILABLE", True)
  monkeypatch.setattr(nap_conf_mod, "_params", params)
  return params


def test_saved_pedal_and_radar_params_produce_safety_param_3(preap_params):
  preap_params.put_bool(NAPParamKeys.PEDAL_ENABLED, True, block=True)
  preap_params.put_bool(NAPParamKeys.RADAR_ENABLED, True, block=True)

  ret = interfaces[CAR.TESLA_MODEL_S_PREAP].get_non_essential_params(CAR.TESLA_MODEL_S_PREAP)

  assert int(ret.safetyConfigs[0].safetyParam) == 3
  assert ret.openpilotLongitudinalControl
  assert not ret.pcmCruise


def test_hands_on_pause_packs_legacy_level_two_without_level_bits(preap_params):
  preap_params.put_bool("TeslaPreapHandsOnPause", True, block=True)
  preap_params.put("TeslaPreapHandsOnLevel", 2, block=True)
  ret = interfaces[CAR.TESLA_MODEL_S_PREAP].get_non_essential_params(CAR.TESLA_MODEL_S_PREAP)
  safety_param = int(ret.safetyConfigs[0].safetyParam)
  assert safety_param & 8
  assert ((safety_param >> 8) & 3) == 0
  assert get_hands_on_disengage_level(safety_param) == 2


def test_hands_on_pause_packs_level_one_and_three(preap_params):
  preap_params.put_bool("TeslaPreapHandsOnPause", True, block=True)
  preap_params.put("TeslaPreapHandsOnLevel", 1, block=True)
  ret = interfaces[CAR.TESLA_MODEL_S_PREAP].get_non_essential_params(CAR.TESLA_MODEL_S_PREAP)
  safety_param = int(ret.safetyConfigs[0].safetyParam)
  assert safety_param & 8
  assert ((safety_param >> 8) & 3) == 1
  assert get_hands_on_disengage_level(safety_param) == 1

  preap_params.put("TeslaPreapHandsOnLevel", 3, block=True)
  ret = interfaces[CAR.TESLA_MODEL_S_PREAP].get_non_essential_params(CAR.TESLA_MODEL_S_PREAP)
  safety_param = int(ret.safetyConfigs[0].safetyParam)
  assert ((safety_param >> 8) & 3) == 3
  assert get_hands_on_disengage_level(safety_param) == 3


def test_pause_off_does_not_pack_level_bits(preap_params):
  preap_params.put_bool("TeslaPreapHandsOnPause", False, block=True)
  preap_params.put("TeslaPreapHandsOnLevel", 1, block=True)
  ret = interfaces[CAR.TESLA_MODEL_S_PREAP].get_non_essential_params(CAR.TESLA_MODEL_S_PREAP)
  safety_param = int(ret.safetyConfigs[0].safetyParam)
  assert (safety_param & 8) == 0
  assert ((safety_param >> 8) & 3) == 0
  assert get_hands_on_disengage_level(safety_param) == 2


def test_invalid_persisted_level_packs_legacy_two(preap_params):
  preap_params.put_bool("TeslaPreapHandsOnPause", True, block=True)
  preap_params.put("TeslaPreapHandsOnLevel", 0, block=True)
  ret = interfaces[CAR.TESLA_MODEL_S_PREAP].get_non_essential_params(CAR.TESLA_MODEL_S_PREAP)
  safety_param = int(ret.safetyConfigs[0].safetyParam)
  assert ((safety_param >> 8) & 3) == 0
  assert get_hands_on_disengage_level(safety_param) == 2
