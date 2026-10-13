from pathlib import Path

from openpilot.selfdrive.speedsignd.hud import AnswerGate, LiveSignView, point_in_confirm_buttons

YES = (100.0, 500.0, 120.0, 60.0)
NO = (240.0, 500.0, 120.0, 60.0)
ROOT = Path(__file__).resolve().parents[3]


def test_touch_only_inside_visible_buttons():
  v = LiveSignView(show_mph=True, mph=35)
  assert point_in_confirm_buttons(150, 520, v, YES, NO)
  assert point_in_confirm_buttons(300, 520, v, YES, NO)
  assert not point_in_confirm_buttons(50, 50, v, YES, NO)      # elsewhere: onroad toggles sidebar
  assert not point_in_confirm_buttons(150, 520, LiveSignView(), YES, NO)  # popup hidden
  assert not point_in_confirm_buttons(150, 520, LiveSignView(show_missing_weights=True), YES, NO)
  assert not point_in_confirm_buttons(0, 0, v, (0, 0, 0, 0), (0, 0, 0, 0))  # before layout


def test_after_answer_buttons_release_touch():
  g = AnswerGate()
  g.answer(35, True, 0.0)
  v = g.filter(LiveSignView(show_mph=True, mph=35), 0.1)
  assert not point_in_confirm_buttons(150, 520, v, YES, NO)


def test_onroad_press_checks_hud_buttons_and_hud_is_guarded():
  road = (ROOT / "selfdrive/ui/onroad/augmented_road_view.py").read_text()
  assert "touch_on_hud_button" in road
  hud = (ROOT / "selfdrive/ui/onroad/speed_sign_hud.py").read_text()
  assert "def render(self" in hud and "except Exception" in hud
  assert "threading.Thread(target=_safe_confirm" in hud
