from types import SimpleNamespace
from unittest.mock import Mock

import pyray as rl
import pytest

from openpilot.cereal import messaging
from openpilot.common.constants import CV


class _CruiseSM(dict):
  def __init__(self):
    super().__init__((service, getattr(messaging.new_message(service), service))
                     for service in ("carState", "carControl", "longitudinalPlanSP"))
    self.recv_frame = dict.fromkeys(self, 101)
    self.healthy = True

  def all_checks(self, service_list):
    return self.healthy


@pytest.fixture
def cruise_display(monkeypatch):
  monkeypatch.setenv("SCALE", "1")
  from openpilot.selfdrive.ui.sunnypilot.onroad import hud_renderer
  sm = _CruiseSM()
  sm["carState"].enableLongControl = True
  sm["carControl"].longActive = True
  sm["longitudinalPlanSP"].longitudinalPlanSource = "sccVision"
  sm["longitudinalPlanSP"].vTarget = 15.0
  state = SimpleNamespace(sm=sm, started_frame=100, is_metric=True,
                          CP=SimpleNamespace(brand="tesla", carFingerprint="TESLA_MODEL_S_PREAP",
                                             openpilotLongitudinalControl=True, pcmCruise=False),
                          status=hud_renderer.UIStatus.ENGAGED)
  monkeypatch.setattr(hud_renderer, "ui_state", state)
  hud = hud_renderer.HudRendererSP.__new__(hud_renderer.HudRendererSP)
  hud.set_speed = 90.0
  hud.is_cruise_set = True
  hud.effective_set_speed = None
  hud.speed_conv = CV.MS_TO_KPH
  return hud_renderer, hud, state


@pytest.mark.parametrize("source", ["sccVision", "sccMap", "speedLimitAssist"])
@pytest.mark.parametrize("metric", [True, False])
def test_display_uses_selected_native_target_preserving_manual_ceiling(cruise_display, source, metric):
  module, hud, state = cruise_display
  state.sm["longitudinalPlanSP"].longitudinalPlanSource = source
  hud.speed_conv = CV.MS_TO_KPH if metric else CV.MS_TO_MPH
  hud.set_speed = 25.0 * hud.speed_conv
  ceiling = hud.set_speed
  hud._update_cruise_target()
  assert hud.effective_set_speed == pytest.approx(15.0 * hud.speed_conv)
  assert hud.set_speed == ceiling


@pytest.mark.parametrize("case", ["stale", "invalid", "disabled", "override", "no_request", "other_car", "cruise",
                                    "above_ceiling", "nan", "negative"])
def test_ineligible_target_leaves_retained_manual_speed(cruise_display, case):
  _, hud, state = cruise_display
  sm = state.sm
  if case == "stale":
    sm.recv_frame["longitudinalPlanSP"] = 99
  elif case == "invalid":
    sm.healthy = False
  elif case == "disabled":
    sm["carControl"].longActive = False
  elif case == "override":
    sm["carControl"].cruiseControl.override = True
  elif case == "no_request":
    sm["carState"].enableLongControl = False
  elif case == "other_car":
    state.CP.brand = "honda"
  elif case == "cruise":
    sm["longitudinalPlanSP"].longitudinalPlanSource = "cruise"
  else:
    sm["longitudinalPlanSP"].vTarget = {"above_ceiling": 30.0, "nan": float("nan"), "negative": -1.0}[case]
  hud._update_cruise_target()
  assert hud.effective_set_speed is None
  assert hud.set_speed == 90.0


def test_large_number_and_max_label_show_distinct_authorities(cruise_display, monkeypatch):
  module, hud, state = cruise_display
  hud._update_cruise_target()
  hud._font_semi_bold = rl.Font()
  hud._font_bold = rl.Font()
  hud.pcm_cruise_speed = True
  hud.icbm_active_counter = 0
  monkeypatch.setattr(module, "tr", lambda text: text)
  monkeypatch.setattr(module, "measure_text_cached", lambda *args: rl.Vector2(80, 40))
  monkeypatch.setattr(module.rl, "draw_rectangle_rounded", Mock())
  monkeypatch.setattr(module.rl, "draw_rectangle_rounded_lines_ex", Mock())
  draw_text = Mock()
  monkeypatch.setattr(module.rl, "draw_text_ex", draw_text)
  hud._draw_set_speed(rl.Rectangle(0, 0, 1920, 1080))
  assert [call.args[1] for call in draw_text.call_args_list] == ["MAX 90", "54"]
  assert hud.set_speed == 90.0


@pytest.mark.parametrize("metric", [True, False])
def test_stopped_double_down_target_survives_helper_and_real_hud_update(cruise_display, monkeypatch, metric):
  from opendbc.car import structs
  from opendbc.car.tesla.preap.engagement import PreAPEngagement
  from opendbc.car.tesla.values import CruiseButtons
  from openpilot.selfdrive.car.cruise import VCruiseHelper
  from openpilot.selfdrive.ui.onroad import hud_renderer as base_hud

  _, hud, state = cruise_display
  state.is_metric = metric
  state.CP = structs.CarParams(brand="tesla", carFingerprint="TESLA_MODEL_S_PREAP",
                              openpilotLongitudinalControl=True, pcmCruise=False)
  helper = VCruiseHelper(state.CP, structs.CarParamsSP(pcmCruiseSpeed=True))
  monkeypatch.setattr(base_hud, "ui_state", state)
  hud.v_ego_cluster_seen = False
  controls = messaging.new_message("controlsState").controlsState
  controls.deprecated.vCruise = 88.0  # Must not substitute the legacy field for valid zero.
  state.sm["controlsState"] = controls
  cs = state.sm["carState"]
  cs.cruiseState.speed = -1.0
  helper.update_v_cruise(cs, False, metric)
  cs.vCruise = helper.v_cruise_kph
  cs.vCruiseCluster = helper.v_cruise_cluster_kph
  base_hud.HudRenderer._update_state(hud)
  assert not hud.is_cruise_set
  assert hud.set_speed == base_hud.SET_SPEED_NA

  engagement = PreAPEngagement(True, 750)
  for button, previous, timestamp in (
    (CruiseButtons.DECEL_SET, CruiseButtons.IDLE, 1000),
    (CruiseButtons.IDLE, CruiseButtons.DECEL_SET, 1010),
    (CruiseButtons.DECEL_SET, CruiseButtons.IDLE, 1300),
  ):
    engagement.process_buttons(button, previous, timestamp, 0.0, "KPH" if metric else "MPH",
                               True, True, True, False)
  assert engagement.target_speed_initialized
  assert not engagement.enableLongControl
  cs.cruiseState.speed = engagement.pedal_speed_kph * CV.KPH_TO_MS
  helper.update_v_cruise(cs, False, metric)
  cs.vCruise = helper.v_cruise_kph
  cs.vCruiseCluster = helper.v_cruise_cluster_kph
  base_hud.HudRenderer._update_state(hud)
  assert hud.is_cruise_set
  assert hud.is_cruise_available
  assert hud.set_speed == 0.0

  # Other cars retain the existing zero-cluster compatibility fallback.
  state.CP.brand = "honda"
  base_hud.HudRenderer._update_state(hud)
  assert hud.set_speed == pytest.approx(88.0 if metric else 88.0 * base_hud.KM_TO_MILE)
