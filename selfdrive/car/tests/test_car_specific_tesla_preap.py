"""Tesla Pre-AP pedal calibration and authority-failure events.

CarSpecificEvents must raise pedalNotCalibrated only when:
  carFingerprint == TESLA_MODEL_S_PREAP  (pre-AP car)
  and not pcmCruise                      (pedal mode — we're the longitudinal actuator)
  and nap_conf.pedal_calibrated is False (calibration not trusted)

All three conditions together.

The authority-failure regression also crosses the real controller, engagement,
CarState publication, event-selection, and alert-definition seams.
"""
from unittest.mock import PropertyMock, patch  # noqa: TID251

from cereal import car, log
from opendbc.can import CANPacker
from opendbc.car import CanData
from opendbc.car.car_helpers import interfaces

from openpilot.selfdrive.car.car_specific import CarSpecificEvents
from openpilot.selfdrive.selfdrived.events import ET, EVENTS


EventName = log.OnroadEvent.EventName
NAP_CONF_PATH = "opendbc.car.tesla.preap.nap_conf.NAPConf.pedal_calibrated"
NAP_USE_PEDAL_PATH = "opendbc.car.tesla.preap.nap_conf.NAPConf.use_pedal"
NAP_PEDAL_FACTOR_PATH = "opendbc.car.tesla.preap.nap_conf.NAPConf.pedal_factor"
GAS_COMMAND_ID = 0x551


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


def _run(cp, calibrated, cs=None):
  with patch(NAP_CONF_PATH, new_callable=PropertyMock, return_value=calibrated):
    ce = CarSpecificEvents(cp)
    events = ce.update(cs or _make_cs(), _make_cs(), car.CarControl.new_message())
  return events.names


def _preap_can_packet(message, values):
  address, dat, bus = CANPacker("tesla_preap").make_can_msg(message, 0, values)
  return [(1, [CanData(address, dat, bus)])]


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


def test_pedal_authority_failure_warns_only_in_preap_pedal_mode():
  cs = _make_cs()
  cs.pedalAuthorityFailed = True

  pedal_mode = _make_cp(pcm_cruise=False, op_long=True)
  assert EventName.pedalUnavailable in _run(pedal_mode, calibrated=True, cs=cs)

  no_pedal_mode = _make_cp(pcm_cruise=True, op_long=False)
  assert EventName.pedalUnavailable not in _run(no_pedal_mode, calibrated=True, cs=cs)

  longitudinal_disabled = _make_cp(pcm_cruise=False, op_long=False)
  assert EventName.pedalUnavailable not in _run(longitudinal_disabled, calibrated=True, cs=cs)

  non_preap = _make_cp(fingerprint="TESLA_MODEL_S", pcm_cruise=False, op_long=True)
  assert EventName.pedalUnavailable not in _run(non_preap, calibrated=True, cs=cs)


def test_pedal_acquisition_failure_keeps_lateral_and_surfaces_alert():
  with (
    patch(NAP_USE_PEDAL_PATH, new_callable=PropertyMock, return_value=True),
    patch(NAP_PEDAL_FACTOR_PATH, new_callable=PropertyMock, return_value=1.0),
    patch(NAP_CONF_PATH, new_callable=PropertyMock, return_value=True),
  ):
    CarInterface = interfaces["TESLA_MODEL_S_PREAP"]
    cp = CarInterface.get_params(
      "TESLA_MODEL_S_PREAP", {bus: {} for bus in range(8)}, [],
      alpha_long=False, is_release=False, docs=False,
    )
    interface = CarInterface(cp)
    drive_packet = _preap_can_packet("DI_torque2", {"DI_gear": 4})
    interface.update(drive_packet)

    internal_state = interface.CS
    internal_state.engagement.cruiseEnabled = True
    internal_state.engagement.enableLongControl = True
    internal_state.cruiseEnabled = True
    internal_state.enableLongControl = True
    internal_state.enableJustCC = False
    internal_state.pedal.available = False
    internal_state.pedal.interceptor_state = 5
    internal_state.pedal.idx = 0

    control = car.CarControl.new_message()
    control.enabled = True
    control.latActive = True
    control.longActive = True
    control.actuators.accel = 0.5
    control = control.as_reader()

    pedal_commands = []
    for update_index in range(9):
      if interface.CC.frame % 2 == 0:
        internal_state.pedal.idx = (internal_state.pedal.idx + 1) % 16
      _, can_sends = interface.apply(control, now_nanos=update_index * 10_000_000)
      pedal_commands.extend(command for command in can_sends if command[0] == GAS_COMMAND_ID)

    assert len(pedal_commands) == 4
    assert all(not command[1][4] & 0x80 for command in pedal_commands)
    assert internal_state.engagement.cruiseEnabled
    assert not internal_state.engagement.enableLongControl
    assert internal_state.engagement.enableJustCC
    assert internal_state.engagement.pedal_unavailable

    public_state = interface.update(drive_packet)
    assert public_state.cruiseState.enabled
    assert not public_state.pedalLongActive
    assert public_state.pedalAuthorityFailed

    events = CarSpecificEvents(cp).update(public_state, _make_cs(), control)
    assert EventName.pedalUnavailable in events.names

    event_types = EVENTS[EventName.pedalUnavailable]
    alert = event_types[ET.WARNING]
    assert set(event_types) == {ET.WARNING}
    assert alert.alert_text_1 == "Pedal Control Unavailable"
    assert alert.alert_text_2 == "Speed Control Disabled"
