from pathlib import Path
from openpilot.cereal import custom, log
from opendbc.car.tesla.preap.carcontroller import PedalAuthorityState
from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP, EventsSP
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import ET, Priority
from openpilot.selfdrive.selfdrived.events import EVENTS as STOCK_EVENTS
from openpilot.selfdrive.selfdrived.preap_regen import PREAP_PEDAL_UNAVAILABLE_ALERT, PREAP_REGEN_ALERT
from openpilot.sunnypilot.selfdrive.selfdrived.preap_alerts import (
  ALERT_PEDAL_UNAVAILABLE,
  ALERT_PEDAL_UNAVAILABLE_SUB,
  ALERT_RADAR_FAULT,
  ALERT_REGEN,
  ALERT_REGEN_SUB,
  PedalAuthorityLossMapper,
  PreAPAlertInputs,
  preap_radar_fault,
  radar_state_has_fault,
  register_preap_alerts,
  select_preap_alerts,
)

EventNameSP = custom.OnroadEventSP.EventName


def _inputs(**overrides):
  values = {
    "is_preap": True,
    "pedal_present": True,
    "pedal_calib_available": True,
    "pedal_calib_done": True,
    "pedal_available": True,
    "pedal_timeout": False,
    "pedal_authority_state": int(PedalAuthorityState.ACTIVE),
    "pedal_authority_failed": False,
    "interceptor_state": 0,
    "radar_present": False,
    "radar_config_invalid": False,
    "radar_fault": False,
    "established_authority_lost": False,
  }
  values.update(overrides)
  return PreAPAlertInputs(**values)


def test_eventnamesp_ends_at_28():
  assert max(int(v) for v in EventNameSP.schema.enumerants.values()) == 28
  assert "pedalUnavailable" in EventNameSP.schema.enumerants
  assert "pedalMaxRegen" in EventNameSP.schema.enumerants


def test_alerts_registered_before_eventssp_keys():
  register_preap_alerts()
  assert EventNameSP.pedalUnavailable in EVENTS_SP
  assert EventNameSP.pedalMaxRegen in EVENTS_SP
  events = EventsSP()
  events.add(EventNameSP.pedalUnavailable)
  assert EventNameSP.pedalUnavailable in events.names


def test_pedal_faults_reuse_pedal_unavailable():
  for kwargs in (
    {"pedal_calib_done": False},
    {"pedal_calib_available": False},
    {"pedal_authority_failed": True},
    {"pedal_authority_state": int(PedalAuthorityState.FAILED)},
    {"pedal_timeout": True, "pedal_available": False},
    {"interceptor_state": 1},
  ):
    names = select_preap_alerts(_inputs(**kwargs))
    assert names == (EventNameSP.pedalUnavailable,)


def test_invalid_calibration_persists_until_healthy():
  assert select_preap_alerts(_inputs(pedal_calib_done=False)) == (EventNameSP.pedalUnavailable,)
  assert select_preap_alerts(_inputs(pedal_calib_available=False, pedal_calib_done=False)) == (EventNameSP.pedalUnavailable,)
  assert select_preap_alerts(_inputs()) == ()


def test_authority_loss_persists_while_failed():
  failed = _inputs(pedal_authority_failed=True, pedal_authority_state=int(PedalAuthorityState.FAILED))
  assert select_preap_alerts(failed) == (EventNameSP.pedalUnavailable,)
  assert select_preap_alerts(_inputs()) == ()


def test_no_pedal_suppresses_pedal_alerts():
  assert select_preap_alerts(_inputs(pedal_present=False, pedal_calib_done=False, pedal_authority_failed=True)) == ()


def test_modern_silent():
  assert select_preap_alerts(_inputs(is_preap=False, pedal_calib_done=False, radar_fault=True, radar_present=True)) == ()
  assert not preap_radar_fault(_inputs(is_preap=False, radar_present=True, radar_fault=True))


def test_healthy_pedal_is_silent():
  assert select_preap_alerts(_inputs()) == ()


def test_select_never_emits_regen():
  assert EventNameSP.pedalMaxRegen not in select_preap_alerts(_inputs())
  assert EventNameSP.pedalMaxRegen not in select_preap_alerts(_inputs(pedal_calib_done=False))


def test_radar_fault_uses_stock_event_path():
  assert select_preap_alerts(_inputs(radar_present=True, radar_fault=True, pedal_present=False)) == ()
  assert preap_radar_fault(_inputs(radar_present=True, radar_fault=True))
  assert preap_radar_fault(_inputs(radar_present=True, radar_config_invalid=True))
  assert not preap_radar_fault(_inputs(radar_present=False, radar_fault=True))
  assert not preap_radar_fault(_inputs(radar_present=True, radar_fault=False, radar_config_invalid=False))


def test_mapped_alert_severity_and_text():
  register_preap_alerts()
  pedal = EVENTS_SP[EventNameSP.pedalUnavailable][ET.WARNING]
  assert pedal.alert_text_1 == ALERT_PEDAL_UNAVAILABLE
  assert pedal.alert_text_2 == ALERT_PEDAL_UNAVAILABLE_SUB
  assert pedal.priority == Priority.HIGH
  regen = EVENTS_SP[EventNameSP.pedalMaxRegen][ET.WARNING]
  assert regen.alert_text_1 == ALERT_REGEN
  assert regen.alert_text_2 == ALERT_REGEN_SUB
  assert regen.priority == Priority.HIGH

def test_established_authority_loss_alerts_during_reacquisition():
  mapper = PedalAuthorityLossMapper()
  assert mapper.update(int(PedalAuthorityState.INACTIVE)) is False
  assert mapper.update(int(PedalAuthorityState.ACQUIRING)) is False
  assert select_preap_alerts(_inputs(pedal_authority_state=int(PedalAuthorityState.ACQUIRING))) == ()
  assert mapper.update(int(PedalAuthorityState.ACTIVE)) is False
  assert mapper.update(int(PedalAuthorityState.ACQUIRING)) is True
  lost = _inputs(
    pedal_authority_state=int(PedalAuthorityState.ACQUIRING),
    established_authority_lost=True,
  )
  assert select_preap_alerts(lost) == (EventNameSP.pedalUnavailable,)
  assert mapper.update(int(PedalAuthorityState.ACTIVE)) is False
  mapper.update(int(PedalAuthorityState.ACTIVE))
  assert mapper.update(int(PedalAuthorityState.FAILED)) is True
  mapper.reset()
  assert mapper.update(int(PedalAuthorityState.ACQUIRING)) is False


def test_no_pedal_suppresses_established_authority_loss():
  assert select_preap_alerts(_inputs(
    pedal_present=False,
    pedal_authority_state=int(PedalAuthorityState.ACQUIRING),
    established_authority_lost=True,
  )) == ()


def test_dispatched_alert_text_is_translation_marked():
  register_preap_alerts()
  assert PREAP_PEDAL_UNAVAILABLE_ALERT.alert_text_1 == ALERT_PEDAL_UNAVAILABLE
  assert PREAP_PEDAL_UNAVAILABLE_ALERT.alert_text_2 == ALERT_PEDAL_UNAVAILABLE_SUB
  assert PREAP_REGEN_ALERT.alert_text_1 == ALERT_REGEN
  assert PREAP_REGEN_ALERT.alert_text_2 == ALERT_REGEN_SUB
  assert ALERT_RADAR_FAULT == "Radar Error: Restart the Car"
  events_src = Path(__file__).resolve().parents[4] / "selfdrive" / "selfdrived" / "events.py"
  assert f'soft_disable_alert("{ALERT_RADAR_FAULT}")' in events_src.read_text()
  from openpilot.sunnypilot.selfdrive.selfdrived.events_base import NoEntryAlert
  radar_no_entry = STOCK_EVENTS[log.OnroadEvent.EventName.radarFault][ET.NO_ENTRY]
  assert isinstance(radar_no_entry, NoEntryAlert)
  assert ALERT_RADAR_FAULT in (radar_no_entry.alert_text_1, radar_no_entry.alert_text_2)


def test_alert_renderer_translates_dispatched_text():
  src = Path(__file__).resolve().parents[4] / "selfdrive" / "ui" / "onroad" / "alert_renderer.py"
  text = src.read_text()
  assert "text1=tr(ss.alertText1)" in text
  assert "text2=tr(ss.alertText2)" in text
  mici = Path(__file__).resolve().parents[4] / "selfdrive" / "ui" / "mici" / "onroad" / "alert_renderer.py"
  mici_text = mici.read_text()
  assert "text1=tr(ss.alertText1)" in mici_text
  assert "text2=tr(ss.alertText2)" in mici_text


def test_radar_state_has_fault_reads_real_radarerrors_struct():
  from types import SimpleNamespace

  healthy = SimpleNamespace(radarErrors=SimpleNamespace(
    canError=False, radarFault=False, wrongConfig=False, radarUnavailableTemporary=False,
  ))
  assert radar_state_has_fault(healthy) is False
  assert radar_state_has_fault(SimpleNamespace(radarErrors=SimpleNamespace(
    canError=False, radarFault=True, wrongConfig=False, radarUnavailableTemporary=False,
  ))) is True
  assert radar_state_has_fault(SimpleNamespace(radarErrors=SimpleNamespace(
    canError=False, radarFault=False, wrongConfig=True, radarUnavailableTemporary=False,
  ))) is True
  assert radar_state_has_fault(SimpleNamespace(radarErrors=SimpleNamespace(
    canError=True, radarFault=False, wrongConfig=False, radarUnavailableTemporary=False,
  ))) is True
  assert radar_state_has_fault(SimpleNamespace(radarErrors=SimpleNamespace(
    canError=False, radarFault=False, wrongConfig=False, radarUnavailableTemporary=True,
  ))) is False
  assert radar_state_has_fault(SimpleNamespace(
    radarFault=True, errors=SimpleNamespace(canError=True), radarErrors=None,
  )) is False
  assert radar_state_has_fault(SimpleNamespace(radarErrors=["fault"])) is False
  assert preap_radar_fault(_inputs(
    pedal_present=False, radar_present=True, radar_fault=True,
  )) is True



def test_from_snapshot_no_pedal_radar_fault_reaches_alerts():
  from types import SimpleNamespace
  from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP
  from openpilot.sunnypilot.selfdrive.selfdrived.preap_alerts import preap_alert_inputs_from_snapshot

  CP = SimpleNamespace(carFingerprint="TESLA_MODEL_S_PREAP", radarUnavailable=False)
  CP_SP = SimpleNamespace(flags=int(TeslaFlagsSP.PREAP_RADAR_PRESENT))
  inputs = preap_alert_inputs_from_snapshot(CP, CP_SP, None, radar_fault=True)
  assert inputs.pedal_present is False
  assert inputs.radar_present is True
  assert preap_radar_fault(inputs) is True
  assert select_preap_alerts(inputs) == ()

def test_selfdrived_uses_radar_state_helper():
  src = Path(__file__).resolve().parents[4] / "selfdrive" / "selfdrived" / "selfdrived.py"
  text = src.read_text()
  assert "radar_state_has_fault" in text
  assert "getattr(rs, 'radarFault'" not in text




def test_from_snapshot_uses_published_feedback_timeout():
  from types import SimpleNamespace
  from opendbc.car.tesla.preap.constants import PEDAL_FEEDBACK_TIMEOUT_STATE
  from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP
  from openpilot.sunnypilot.selfdrive.selfdrived.preap_alerts import preap_alert_inputs_from_snapshot

  CP = SimpleNamespace(carFingerprint="TESLA_MODEL_S_PREAP", radarUnavailable=True)
  flags = int(TeslaFlagsSP.PREAP_PEDAL_PRESENT | TeslaFlagsSP.PREAP_PEDAL_CALIB_AVAILABLE)
  CP_SP = SimpleNamespace(flags=flags)
  healthy = SimpleNamespace(pedalFeedbackState=0, pedalFeedbackCounter=3, pedalAuthorityState=0, pedalAuthorityFailed=False)
  timed_out = SimpleNamespace(
    pedalFeedbackState=PEDAL_FEEDBACK_TIMEOUT_STATE, pedalFeedbackCounter=3,
    pedalAuthorityState=0, pedalAuthorityFailed=False,
  )
  healthy_inputs = preap_alert_inputs_from_snapshot(CP, CP_SP, healthy)
  assert healthy_inputs.pedal_timeout is False
  assert healthy_inputs.pedal_available is True
  assert select_preap_alerts(healthy_inputs) == ()

  timeout_inputs = preap_alert_inputs_from_snapshot(CP, CP_SP, timed_out)
  assert timeout_inputs.pedal_timeout is True
  assert timeout_inputs.pedal_available is False
  assert select_preap_alerts(timeout_inputs) == (EventNameSP.pedalUnavailable,)


def test_from_snapshot_inactive_configured_pedal_timeout_alerts():
  from types import SimpleNamespace
  from opendbc.car.tesla.preap.constants import PEDAL_FEEDBACK_TIMEOUT_STATE
  from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP
  from openpilot.sunnypilot.selfdrive.selfdrived.preap_alerts import preap_alert_inputs_from_snapshot

  CP = SimpleNamespace(carFingerprint="TESLA_MODEL_S_PREAP", radarUnavailable=True)
  CP_SP = SimpleNamespace(flags=int(TeslaFlagsSP.PREAP_PEDAL_PRESENT))
  cs_sp = SimpleNamespace(
    pedalFeedbackState=PEDAL_FEEDBACK_TIMEOUT_STATE, pedalFeedbackCounter=0,
    pedalAuthorityState=0, pedalAuthorityFailed=False,
  )
  inputs = preap_alert_inputs_from_snapshot(CP, CP_SP, cs_sp)
  assert inputs.pedal_present is True
  assert inputs.pedal_calib_available is False
  assert inputs.pedal_timeout is True
  assert select_preap_alerts(inputs) == (EventNameSP.pedalUnavailable,)


def test_from_snapshot_active_configured_pedal_timeout_alerts():
  from types import SimpleNamespace
  from opendbc.car.tesla.preap.constants import PEDAL_FEEDBACK_TIMEOUT_STATE
  from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP
  from openpilot.sunnypilot.selfdrive.selfdrived.preap_alerts import preap_alert_inputs_from_snapshot

  CP = SimpleNamespace(carFingerprint="TESLA_MODEL_S_PREAP", radarUnavailable=False)
  flags = int(TeslaFlagsSP.PREAP_PEDAL_PRESENT | TeslaFlagsSP.PREAP_PEDAL_CALIB_AVAILABLE)
  CP_SP = SimpleNamespace(flags=flags)
  cs_sp = SimpleNamespace(
    pedalFeedbackState=PEDAL_FEEDBACK_TIMEOUT_STATE, pedalFeedbackCounter=3,
    pedalAuthorityState=int(PedalAuthorityState.ACTIVE), pedalAuthorityFailed=False,
  )
  inputs = preap_alert_inputs_from_snapshot(CP, CP_SP, cs_sp)
  assert inputs.pedal_timeout is True
  assert inputs.pedal_available is False
  assert inputs.pedal_authority_state == int(PedalAuthorityState.ACTIVE)
  assert select_preap_alerts(inputs) == (EventNameSP.pedalUnavailable,)


def test_hyundai_non_scc_does_not_take_preap_radar_error_path():
  from types import SimpleNamespace
  from opendbc.car.tesla.preap.boot import preap_radar_present
  from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
  from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP

  overlapping = int(HyundaiFlagsSP.NON_SCC)
  assert overlapping == int(TeslaFlagsSP.PREAP_RADAR_PRESENT)

  hyundai = SimpleNamespace(carFingerprint="HYUNDAI_KONA_NON_SCC", openpilotLongitudinalControl=False)
  modern = SimpleNamespace(carFingerprint="TESLA_MODEL_Y", openpilotLongitudinalControl=False)
  preap = SimpleNamespace(carFingerprint="TESLA_MODEL_S_PREAP", openpilotLongitudinalControl=False)
  flags = SimpleNamespace(flags=overlapping)

  assert preap_radar_present(hyundai, flags) is False
  assert preap_radar_present(modern, flags) is False
  assert preap_radar_present(preap, flags) is True
  assert preap_radar_present(preap, SimpleNamespace(flags=0)) is False

  assert not (hyundai.openpilotLongitudinalControl or preap_radar_present(hyundai, flags))
  assert not (modern.openpilotLongitudinalControl or preap_radar_present(modern, flags))
  assert preap.openpilotLongitudinalControl or preap_radar_present(preap, flags)


def test_selfdrived_radar_error_path_requires_preap_identity():
  src = Path(__file__).resolve().parents[4] / "selfdrive" / "selfdrived" / "selfdrived.py"
  text = src.read_text()
  assert "preap_radar_present(self.CP, self.CP_SP)" in text
  assert "TeslaFlagsSP.PREAP_RADAR_PRESENT" not in text

