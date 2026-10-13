from openpilot.selfdrive.speedsignd.hud import AnswerGate, LiveSignView, REASK_SUPPRESS_S, hud_confirm_visible


def v(mph):
  return LiveSignView(show_mph=mph > 0, mph=mph)


def test_yes_keeps_plate_without_buttons():
  g = AnswerGate()
  assert hud_confirm_visible(g.filter(v(35), 0.0))
  g.answer(35, True, 1.0)
  out = g.filter(v(35), 1.0)
  assert out.show_mph and out.mph == 35 and not hud_confirm_visible(out)
  out = g.filter(v(0), 30.0)          # live plate expired: confirmed plate stays
  assert out.mph == 35 and not hud_confirm_visible(out)
  out = g.filter(v(35), 40.0)         # same sign again soon: no re-ask
  assert not hud_confirm_visible(out)
  out = g.filter(v(45), 50.0)         # different mph: ask again
  assert out.mph == 45 and hud_confirm_visible(out)


def test_no_clears_plate_and_suppresses_reask():
  g = AnswerGate()
  g.answer(40, False, 0.0)
  assert not g.filter(v(40), 0.0).show
  assert not g.filter(v(0), 5.0).show
  assert not g.filter(v(40), REASK_SUPPRESS_S - 1).show
  assert hud_confirm_visible(g.filter(v(40), REASK_SUPPRESS_S + 1)) is False  # still live same mph
  assert not g.filter(v(0), REASK_SUPPRESS_S + 2).show
  assert hud_confirm_visible(g.filter(v(40), REASK_SUPPRESS_S + 3))
  g.answer(40, False, 100.0)
  assert hud_confirm_visible(g.filter(v(30), 101.0))


def test_diagnostic_plates_pass():
  g = AnswerGate()
  g.answer(40, False, 0.0)
  w = LiveSignView(show_missing_weights=True)
  assert g.filter(w, 1.0) is w
