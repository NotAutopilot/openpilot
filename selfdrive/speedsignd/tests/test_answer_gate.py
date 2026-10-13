from openpilot.selfdrive.speedsignd.hud import AnswerGate, LiveSignView, REASK_SUPPRESS_S, hud_confirm_visible


def v(mph):
  return LiveSignView(show_mph=mph > 0, mph=mph)


def test_yes_or_no_hides_popup_and_no_reask():
  for yes in (True, False):
    g = AnswerGate()
    assert hud_confirm_visible(g.filter(v(35), 0.0))
    g.answer(35, yes, 1.0)
    assert not g.filter(v(35), 1.0).show
    assert not g.filter(v(0), 5.0).show
    assert not g.filter(v(35), REASK_SUPPRESS_S - 1).show
    assert not g.filter(v(35), REASK_SUPPRESS_S + 5).show   # same mph still live
    assert not g.filter(v(0), REASK_SUPPRESS_S + 6).show
    assert hud_confirm_visible(g.filter(v(35), REASK_SUPPRESS_S + 7))


def test_different_mph_asks_again():
  g = AnswerGate()
  g.answer(40, True, 0.0)
  assert hud_confirm_visible(g.filter(v(30), 1.0))


def test_diagnostic_plates_pass():
  g = AnswerGate()
  g.answer(40, False, 0.0)
  w = LiveSignView(show_missing_weights=True)
  assert g.filter(w, 1.0) is w
