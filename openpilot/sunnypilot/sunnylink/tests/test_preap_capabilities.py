from types import SimpleNamespace

from opendbc.car.tesla.preap.sp.platform import PREAP_PLATFORM
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP
from openpilot.sunnypilot.sunnylink.capabilities import (
  CAPABILITY_FIELDS,
  PROTOCOL_VERSION,
  _is_tesla_preap,
  _resolve_tesla_preap_capabilities,
)
from opendbc.car.tesla.preap.sp.platform import is_preap_ui_platform


def _caps(**overrides):
  caps = {
    "tesla_preap": True,
    "tesla_preap_pedal": False,
    "tesla_preap_radar": False,
    "tesla_preap_pedal_calib": False,
    "tesla_preap_independent_brake": False,
    "tesla_preap_active_mode": "",
    "tesla_preap_longitudinal_path": "",
    "tesla_preap_pedal_health": "none",
    "tesla_preap_radar_health": "none",
  }
  caps.update(overrides)
  return caps


def test_protocol_version_stays_one():
  assert PROTOCOL_VERSION == 1


def test_capability_fields_include_tesla_preap():
  for field in (
    "tesla_preap",
    "tesla_preap_pedal",
    "tesla_preap_radar",
    "tesla_preap_pedal_calib",
    "tesla_preap_independent_brake",
    "tesla_preap_active_mode",
    "tesla_preap_longitudinal_path",
    "tesla_preap_pedal_health",
    "tesla_preap_radar_health",
  ):
    assert field in CAPABILITY_FIELDS


def test_preap_from_fingerprint_not_vehicle_bus():
  cp = SimpleNamespace(carFingerprint=PREAP_PLATFORM)
  assert _is_tesla_preap(cp, "")
  assert not _is_tesla_preap(cp, "TESLA_MODEL_3")
  modern = SimpleNamespace(carFingerprint="TESLA_MODEL_3")
  assert not _is_tesla_preap(modern, "")
  assert _is_tesla_preap(modern, PREAP_PLATFORM)
  no_bus = SimpleNamespace(carFingerprint="TESLA_MODEL_3")
  assert not _is_tesla_preap(no_bus, "")
  assert _is_tesla_preap(None, PREAP_PLATFORM)
  assert not _is_tesla_preap(None, "TESLA_MODEL_3")


def test_ui_helper_bundle_wins_and_ignores_vehicle_bus():
  cp = SimpleNamespace(carFingerprint="TESLA_MODEL_3")
  assert is_preap_ui_platform(PREAP_PLATFORM, cp)
  assert not is_preap_ui_platform("TESLA_MODEL_3", SimpleNamespace(carFingerprint=PREAP_PLATFORM))
  assert is_preap_ui_platform("", SimpleNamespace(carFingerprint=PREAP_PLATFORM))
  assert not is_preap_ui_platform("", SimpleNamespace(carFingerprint="TESLA_MODEL_3"))


def test_pedal_capabilities():
  caps = _caps()
  cp = SimpleNamespace(openpilotLongitudinalControl=True, pcmCruise=False, radarUnavailable=True)
  cp_sp = SimpleNamespace(
    flags=TeslaFlagsSP.PREAP_PEDAL_PRESENT | TeslaFlagsSP.PREAP_PEDAL_CALIB_AVAILABLE,
  )
  _resolve_tesla_preap_capabilities(caps, cp, cp_sp)
  assert caps["tesla_preap_pedal"] is True
  assert caps["tesla_preap_pedal_calib"] is True
  assert caps["tesla_preap_independent_brake"] is True
  assert caps["tesla_preap_active_mode"] == "independent"
  assert caps["tesla_preap_longitudinal_path"] == "pedal"
  assert caps["tesla_preap_pedal_health"] == "ok"


def test_no_pedal_capabilities():
  caps = _caps()
  cp = SimpleNamespace(openpilotLongitudinalControl=False, pcmCruise=True, radarUnavailable=True)
  cp_sp = SimpleNamespace(flags=0)
  _resolve_tesla_preap_capabilities(caps, cp, cp_sp)
  assert caps["tesla_preap_pedal"] is False
  assert caps["tesla_preap_independent_brake"] is True
  assert caps["tesla_preap_active_mode"] == "independent"
  assert caps["tesla_preap_longitudinal_path"] == "stock_di"
  assert caps["tesla_preap_pedal_health"] == "none"


def test_modern_skips_preap_fields():
  caps = _caps(tesla_preap=False)
  _resolve_tesla_preap_capabilities(caps, SimpleNamespace(), SimpleNamespace(flags=TeslaFlagsSP.HAS_VEHICLE_BUS))
  assert caps["tesla_preap_pedal"] is False
  assert caps["tesla_preap_radar"] is False


def test_brake_mode_always_applicable_on_preap():
  caps = _caps()
  cp = SimpleNamespace(openpilotLongitudinalControl=True, pcmCruise=False, radarUnavailable=False)
  cp_sp = SimpleNamespace(flags=TeslaFlagsSP.PREAP_PEDAL_PRESENT)
  _resolve_tesla_preap_capabilities(caps, cp, cp_sp)
  assert caps["tesla_preap_active_mode"] == "independent"
  assert caps["tesla_preap_independent_brake"] is True

  _resolve_tesla_preap_capabilities(caps, cp, cp_sp)
  assert caps["tesla_preap_active_mode"] == "independent"
  assert caps["tesla_preap_independent_brake"] is True


def test_radar_health_matrix():
  caps = _caps()
  cp = SimpleNamespace(openpilotLongitudinalControl=False, pcmCruise=True, radarUnavailable=True)
  cp_sp = SimpleNamespace(flags=TeslaFlagsSP.PREAP_RADAR_PRESENT)
  _resolve_tesla_preap_capabilities(caps, cp, cp_sp)
  assert caps["tesla_preap_radar"] is True
  assert caps["tesla_preap_radar_health"] == "unconfigured"
  cp.radarUnavailable = False
  _resolve_tesla_preap_capabilities(caps, cp, cp_sp)
  assert caps["tesla_preap_radar_health"] == "ok"
