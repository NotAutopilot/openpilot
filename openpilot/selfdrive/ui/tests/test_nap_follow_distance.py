import time
from types import SimpleNamespace
from unittest.mock import Mock

import pyray as rl
import pytest

from openpilot.common.params import Params


@pytest.fixture
def feedback(monkeypatch):
  monkeypatch.setenv("SCALE", "1")
  from openpilot.selfdrive.ui.onroad import follow_distance_indicator
  follow_distance_indicator.reset_follow_distance_tap()
  return follow_distance_indicator


class _WheelSM:
  def __init__(self, distance, alive=True, valid=True):
    self.alive = {"carState": alive}
    self.valid = {"carState": valid}
    self._distance = distance

  def __getitem__(self, key):
    assert key == "carState"
    return SimpleNamespace(napStalkFollowDistance=self._distance)


def test_live_stalk_requires_valid_alive_carstate(feedback):
  live_stalk_follow_distance = feedback.live_stalk_follow_distance
  assert live_stalk_follow_distance(_WheelSM(6, alive=False)) == 0
  assert live_stalk_follow_distance(_WheelSM(6, valid=False)) == 0
  assert live_stalk_follow_distance(_WheelSM(0)) == 0
  assert live_stalk_follow_distance(_WheelSM(255)) == 0
  assert live_stalk_follow_distance(_WheelSM(6)) == 6


def test_tap_survives_unchanged_wheel_until_detent(feedback):
  feedback.note_follow_distance_tap(7, wheel=4)
  assert feedback.selected_follow_distance(4) == 7
  assert feedback.selected_follow_distance(4) == 7
  assert feedback.selected_follow_distance(5) == 5
  assert feedback.selected_follow_distance(4) == 4


def test_overlay_first_sample_does_not_show(feedback):
  overlay = feedback.FollowDistanceIndicator()
  assert overlay.note_selection(10.0, 4, onroad=True, alert=False) == 0


def test_overlay_change_shows_until_timeout_without_queue(feedback):
  overlay = feedback.FollowDistanceIndicator()
  overlay.note_selection(10.0, 4, onroad=True, alert=False)
  assert overlay.note_selection(10.1, 6, onroad=True, alert=False) == 6
  assert overlay.note_selection(10.2, 2, onroad=True, alert=False) == 2
  assert overlay.note_selection(10.2 + feedback.OVERLAY_SECONDS, 2, onroad=True, alert=False) == 0


def test_overlay_alert_hides_and_offroad_resets(feedback):
  overlay = feedback.FollowDistanceIndicator()
  overlay.note_selection(10.0, 3, onroad=True, alert=False)
  overlay.note_selection(10.1, 5, onroad=True, alert=False)
  assert overlay.note_selection(10.2, 5, onroad=True, alert=True) == 0
  overlay.note_selection(10.3, 5, onroad=False, alert=False)
  assert overlay.note_selection(10.4, 5, onroad=True, alert=False) == 0


@pytest.fixture
def follow_picker(monkeypatch, feedback):
  monkeypatch.setenv("SCALE", "1")
  from openpilot.system.ui.lib.application import gui_app
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.nap import NAPLayout

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


def test_live_picker_tracks_wheel_without_waiting_for_disk(follow_picker, monkeypatch):
  from openpilot.selfdrive.ui.ui_state import ui_state

  panel, params = follow_picker
  wheel = _WheelSM(4)
  monkeypatch.setattr(ui_state, "sm", wheel)
  monkeypatch.setattr(ui_state, "started", True)
  panel._update_state()

  wheel._distance = 6
  panel._update_state()
  assert panel._follow_buttons.action_item.get_selected_button() == 5
  assert params.get("NAPFollowDistance") == 4

  # A queued older write must not undo live selection.
  params.put("NAPFollowDistance", 5, block=True)
  panel._update_state()
  assert panel._follow_buttons.action_item.get_selected_button() == 5

  panel._on_follow_distance(6)
  panel._update_state()
  assert panel._follow_buttons.action_item.get_selected_button() == 6
  wheel.alive["carState"] = False
  panel._update_state()
  assert panel._follow_buttons.action_item.get_selected_button() == 6
  wheel.alive["carState"] = True
  wheel._distance = 3
  panel._update_state()
  assert panel._follow_buttons.action_item.get_selected_button() == 2


def test_initial_overlay_frame_does_not_undo_picker_tap(follow_picker, feedback, monkeypatch):
  from openpilot.selfdrive.ui.ui_state import ui_state

  panel, _ = follow_picker
  monkeypatch.setattr(ui_state, "sm", _WheelSM(4))
  monkeypatch.setattr(ui_state, "started", True)
  monkeypatch.setattr(ui_state, "CP", SimpleNamespace(carFingerprint="TESLA_MODEL_S_PREAP"))
  panel._on_follow_distance(6)
  feedback.FollowDistanceIndicator().render(rl.Rectangle(0, 0, 2160, 1080), alert=True)
  panel._update_state()
  assert panel._follow_buttons.action_item.get_selected_button() == 6
