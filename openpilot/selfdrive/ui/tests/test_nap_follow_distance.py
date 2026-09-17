import time
from unittest.mock import Mock

import pyray as rl
import pytest

from openpilot.common.params import Params


@pytest.fixture
def follow_picker(monkeypatch):
  monkeypatch.setenv("SCALE", "1")
  from openpilot.system.ui.lib.application import gui_app
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.nap import NAPLayout

  # State and callback tests need handles, but no rendering resources.
  monkeypatch.setattr(gui_app, "font", lambda *_: rl.Font())
  monkeypatch.setattr(gui_app, "texture", lambda *_, **__: rl.Texture())
  params = Params()
  params.put("NAPFollowDistance", 4, block=True)
  panel = NAPLayout()
  panel.show_event()
  panel._params = Mock(wraps=params)
  return panel, params


@pytest.mark.parametrize("refresh", ["_update_state", "show_event"])
def test_stalk_return_before_refresh_corrects_tapped_selection(follow_picker, refresh):
  panel, params = follow_picker
  buttons = panel._follow_buttons.action_item
  buttons.set_rect(rl.Rectangle(0, 0, 560, 100))
  buttons._handle_mouse_release(rl.Vector2(520, 50))
  assert buttons.get_selected_button() == 6

  deadline = time.monotonic() + 5
  while params.get("NAPFollowDistance") != 7 and time.monotonic() < deadline:
    time.sleep(0.01)
  assert params.get("NAPFollowDistance") == 7
  # The stalk returns to the last displayed value before another UI frame.
  params.put("NAPFollowDistance", 4, block=True)
  panel._params.reset_mock()

  getattr(panel, refresh)()

  assert buttons.get_selected_button() == 3
  panel._params.put.assert_not_called()
  panel._params.put_bool.assert_not_called()


def test_visible_picker_tracks_all_external_distances_without_writing(follow_picker):
  panel, params = follow_picker
  for distance in (1, 7, 2, 6, 3, 5, 4):
    params.put("NAPFollowDistance", distance, block=True)
    panel._update_state()
    assert panel._follow_buttons.action_item.get_selected_button() == distance - 1

  panel._params.put.assert_not_called()
  panel._params.put_bool.assert_not_called()
