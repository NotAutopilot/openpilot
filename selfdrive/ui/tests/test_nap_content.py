from opendbc.car.tesla.nap_detect import FORCED_PLATFORM, NAP_CAR_TYPE_AUTO
from openpilot.selfdrive.car.nap_profiles import PROFILE_FOR_PLATFORM
from openpilot.selfdrive.ui.layouts.settings.nap_content import (CAR_TYPE_LABELS, CAR_TYPE_VALUES,
                                                                 active_car_text, car_type_index)


def test_car_type_options_line_up():
  assert len(CAR_TYPE_VALUES) == len(CAR_TYPE_LABELS)
  assert CAR_TYPE_VALUES[0] == NAP_CAR_TYPE_AUTO  # Auto is the default and first option


def test_every_car_type_is_selectable_and_has_a_profile():
  assert set(CAR_TYPE_VALUES) == {NAP_CAR_TYPE_AUTO, *FORCED_PLATFORM}
  for platform in FORCED_PLATFORM.values():
    assert platform in PROFILE_FOR_PLATFORM


def test_car_type_index_defaults_to_auto():
  assert [car_type_index(v) for v in CAR_TYPE_VALUES] == list(range(len(CAR_TYPE_VALUES)))
  assert car_type_index(None) == 0
  assert car_type_index(9) == 0


def test_active_car_text():
  assert active_car_text(None, 0) == "None yet"
  assert active_car_text("ap1", 0) == "AP1 Model S (auto-detected)"
  assert active_car_text("ap1x", 0) == "AP1 Model X (auto-detected)"
  assert active_car_text("ap1x", 3) == "AP1 Model X (forced)"
  assert active_car_text("preap", 1) == "Pre-AP Model S (forced)"
  assert active_car_text("bogus", 0) == "None yet"
