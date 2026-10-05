import os

import pytest

from cereal import car
from openpilot.common.params import Params
from openpilot.selfdrive.car import nap_profiles
from openpilot.selfdrive.car.nap_profiles import car_label, switch_profile

PREAP = "TESLA_MODEL_S_PREAP"
HW1 = "TESLA_MODEL_S_HW1"


def car_params_bytes(platform: str) -> bytes:
  return car.CarParams.new_message(carFingerprint=platform).to_bytes()


@pytest.fixture
def live(tmp_path):
  return Params(str(tmp_path / "live"))


@pytest.fixture
def root(tmp_path):
  return str(tmp_path / "profiles")


def test_non_nap_platform_is_noop(live, root):
  live.put("CalibrationParams", b"calib", block=True)
  assert switch_profile(live, "HONDA_CIVIC", root) is None
  assert live.get("NAPActiveProfile") is None
  assert live.get("CalibrationParams") == b"calib"


def test_first_boot_adopts_live_state(live, root):
  live.put("CalibrationParams", b"preap-calib", block=True)
  assert switch_profile(live, PREAP, root) == "preap"
  assert live.get("NAPActiveProfile") == "preap"
  assert live.get("CalibrationParams") == b"preap-calib"


def test_first_boot_infers_previous_car_from_last_route(live, root):
  live.put("CalibrationParams", b"preap-calib", block=True)
  live.put("CarParamsPersistent", car_params_bytes(PREAP), block=True)

  assert switch_profile(live, HW1, root) == "ap1"

  assert live.get("CalibrationParams") is None  # AP1 seen for the first time: fresh learners
  assert Params(os.path.join(root, "preap")).get("CalibrationParams") == b"preap-calib"


def test_round_trip_restores_each_cars_state(live, root):
  live.put("NAPActiveProfile", "preap", block=True)
  live.put("CalibrationParams", b"preap-calib", block=True)
  live.put_bool("ExperimentalMode", True, block=True)

  switch_profile(live, HW1, root)
  assert live.get("CalibrationParams") is None
  assert live.get_bool("ExperimentalMode")  # prefs carry forward on a car's first visit

  live.put("CalibrationParams", b"ap1-calib", block=True)
  live.put_bool("ExperimentalMode", False, block=True)

  switch_profile(live, PREAP, root)
  assert live.get("CalibrationParams") == b"preap-calib"
  assert live.get_bool("ExperimentalMode")

  switch_profile(live, HW1, root)
  assert live.get("CalibrationParams") == b"ap1-calib"
  assert not live.get_bool("ExperimentalMode")


def test_same_car_is_noop(live, root):
  live.put("NAPActiveProfile", "preap", block=True)
  live.put("CalibrationParams", b"preap-calib", block=True)
  assert switch_profile(live, PREAP, root) == "preap"
  assert live.get("CalibrationParams") == b"preap-calib"
  assert not os.path.exists(os.path.join(root, "preap"))


def test_restores_last_route_car_params_for_learners(live, root):
  # card copies CarParamsPersistent into CarParamsPrevRoute; paramsd/torqued/lagd keep their
  # learned values only when that matches the current fingerprint
  live.put("NAPActiveProfile", "preap", block=True)
  live.put("CarParamsPersistent", car_params_bytes(PREAP), block=True)

  switch_profile(live, HW1, root)
  assert live.get("CarParamsPersistent") is None
  live.put("CarParamsPersistent", car_params_bytes(HW1), block=True)  # card, during the AP1 drive

  switch_profile(live, PREAP, root)
  with car.CarParams.from_bytes(live.get("CarParamsPersistent")) as cp:
    assert cp.carFingerprint == PREAP


def test_active_profile_written_before_restore(live, root, monkeypatch):
  live.put("NAPActiveProfile", "preap", block=True)
  live.put("CalibrationParams", b"preap-calib", block=True)

  real_store = nap_profiles._profile_store

  class BrokenStore:
    def get(self, key):
      raise OSError("disk error")

  monkeypatch.setattr(nap_profiles, "_profile_store", lambda r, p: BrokenStore() if p == "ap1" else real_store(r, p))

  assert switch_profile(live, HW1, root) is None
  assert live.get("NAPActiveProfile") == "ap1"
  assert Params(os.path.join(root, "preap")).get("CalibrationParams") == b"preap-calib"


@pytest.mark.parametrize("platform, source, expected", [
  (HW1, "can", "AP1 Model S (auto)"),
  (PREAP, "fixed", "Pre-AP Model S (forced)"),
  ("HONDA_CIVIC", "fw", ""),
])
def test_car_label(platform, source, expected):
  CP = car.CarParams.new_message(carFingerprint=platform, fingerprintSource=source)
  assert car_label(CP) == expected
