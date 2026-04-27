"""Tesla Pre-AP specific events — pedalNotCalibrated gate.

CarSpecificEvents must raise pedalNotCalibrated only when:
  carFingerprint == TESLA_MODEL_S_PREAP  (pre-AP car)
  and not pcmCruise                      (pedal mode — we're the longitudinal actuator)
  and nap_conf.pedal_calibrated is False (calibration not trusted)

All three conditions together.
"""
from unittest.mock import PropertyMock, patch  # noqa: TID251

from cereal import car, log

from openpilot.selfdrive.car.car_specific import CarSpecificEvents


EventName = log.OnroadEvent.EventName
NAP_CONF_PATH = "opendbc.car.tesla.preap.nap_conf.NAPConf.pedal_calibrated"


def _make_cp(*, fingerprint="TESLA_MODEL_S_PREAP", brand="tesla", pcm_cruise=False,
             op_long=True):
  cp = car.CarParams.new_message()
  cp.carFingerprint = fingerprint
  cp.brand = brand
  cp.pcmCruise = pcm_cruise
  cp.openpilotLongitudinalControl = op_long
  return cp


def _make_cs():
  cs = car.CarState.new_message()
  cs.cruiseState.available = True
  cs.gearShifter = "drive"
  return cs


def _run(cp, calibrated):
  with patch(NAP_CONF_PATH, new_callable=PropertyMock, return_value=calibrated):
    ce = CarSpecificEvents(cp)
    events = ce.update(_make_cs(), _make_cs(), car.CarControl.new_message())
  return events.names


def test_fires_in_preap_pedal_mode_when_uncalibrated():
  cp = _make_cp(pcm_cruise=False, op_long=True)
  assert EventName.pedalNotCalibrated in _run(cp, calibrated=False)


def test_silent_in_preap_pedal_mode_when_calibrated():
  cp = _make_cp(pcm_cruise=False, op_long=True)
  assert EventName.pedalNotCalibrated not in _run(cp, calibrated=True)


def test_silent_in_preap_nopedal_mode_even_when_uncalibrated():
  # no-pedal mode doesn't use the Comma Pedal; calibration is irrelevant
  cp = _make_cp(pcm_cruise=True, op_long=False)
  assert EventName.pedalNotCalibrated not in _run(cp, calibrated=False)


def test_silent_on_non_preap_tesla():
  cp = _make_cp(fingerprint="TESLA_MODEL_S", pcm_cruise=False, op_long=True)
  assert EventName.pedalNotCalibrated not in _run(cp, calibrated=False)


def test_silent_on_non_tesla_brand():
  cp = _make_cp(fingerprint="HONDA_CIVIC_2022", brand="honda", pcm_cruise=True, op_long=False)
  assert EventName.pedalNotCalibrated not in _run(cp, calibrated=False)
