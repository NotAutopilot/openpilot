from openpilot.cereal import custom, log
from opendbc.car.structs import car
from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import ET
from openpilot.sunnypilot.selfdrive.selfdrived.preap_alerts import (
  preap_lkas_disable_alert,
  preap_lkas_enable_alert,
  register_preap_alerts,
  select_preap_alerts,
  PreAPAlertInputs,
)

EventNameSP = custom.OnroadEventSP.EventName
AudibleAlert = log.SelfdriveState.AudibleAlert


def test_preap_lkas_alerts_show_steering_prompt():
  register_preap_alerts()
  cp = car.CarParams.new_message()
  cp.brand = "tesla"
  cp.carFingerprint = "TESLA_MODEL_S_PREAP"
  cs = car.CarState.new_message()
  args = (cp, cs, None, False, 100, log.LongitudinalPersonality.standard)

  enable = preap_lkas_enable_alert(*args)
  assert enable.alert_text_1 == "Steering Engaged"
  assert enable.audible_alert == AudibleAlert.engage

  disable = preap_lkas_disable_alert(*args)
  assert disable.alert_text_1 == "Steering Disengaged"
  assert disable.audible_alert == AudibleAlert.disengage

  stock = car.CarParams.new_message()
  stock_enable = preap_lkas_enable_alert(stock, *args[1:])
  assert stock_enable.alert_text_1 == ""
  assert stock_enable.audible_alert == AudibleAlert.engage

  assert EventNameSP.lkasEnable in EVENTS_SP
  assert ET.ENABLE in EVENTS_SP[EventNameSP.lkasEnable]


def test_select_preap_alerts_defers_to_nap_eventname():
  inputs = PreAPAlertInputs(
    is_preap=True,
    pedal_present=True,
    pedal_calib_available=False,
    pedal_calib_done=False,
    pedal_available=False,
    pedal_timeout=True,
    pedal_authority_state=0,
    pedal_authority_failed=True,
    interceptor_state=0,
    radar_present=False,
    radar_config_invalid=False,
    radar_fault=False,
  )
  assert select_preap_alerts(inputs) == ()
