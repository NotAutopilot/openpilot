import time
import pyray as rl
from dataclasses import dataclass
from enum import IntEnum
from collections.abc import Callable
from openpilot.selfdrive.ui.layouts.settings.developer import DeveloperLayout
from openpilot.selfdrive.ui.layouts.settings.device import DeviceLayout
from openpilot.selfdrive.ui.layouts.settings.firehose import FirehoseLayout
from openpilot.selfdrive.ui.layouts.settings.hidden_toggles import HiddenTogglesPopup
from openpilot.selfdrive.ui.layouts.settings.nap import NAPLayout
from openpilot.selfdrive.ui.layouts.settings.software import SoftwareLayout
from openpilot.selfdrive.ui.layouts.settings.toggles import TogglesLayout
from openpilot.selfdrive.ui.layouts.settings.triple_tap import TripleTapDetector
from openpilot.system.ui.lib.application import gui_app, FontWeight, MousePos
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.lib.wifi_manager import WifiManager
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.network import NetworkUI

# Constants
SIDEBAR_WIDTH = 500
CLOSE_BTN_SIZE = 200
CLOSE_ICON_SIZE = 70
NAV_BTN_HEIGHT = 110
PANEL_MARGIN = 50

# Colors
SIDEBAR_COLOR = rl.BLACK
PANEL_COLOR = rl.Color(41, 41, 41, 255)
CLOSE_BTN_COLOR = rl.Color(41, 41, 41, 255)
CLOSE_BTN_PRESSED = rl.Color(59, 59, 59, 255)
TEXT_NORMAL = rl.Color(128, 128, 128, 255)
TEXT_SELECTED = rl.WHITE


class PanelType(IntEnum):
  DEVICE = 0
  NETWORK = 1
  TOGGLES = 2
  SOFTWARE = 3
  NAP = 4
  FIREHOSE = 5
  DEVELOPER = 6


@dataclass
class PanelInfo:
  name: str
  instance: Widget
  button_rect: rl.Rectangle = rl.Rectangle(0, 0, 0, 0)


class SettingsLayout(Widget):
  def __init__(self):
    super().__init__()
    self._current_panel = PanelType.DEVICE

    # Panel configuration
    wifi_manager = WifiManager()
    wifi_manager.set_active(False)

    self._panels = {
      PanelType.DEVICE: PanelInfo(tr_noop("Device"), DeviceLayout()),
      PanelType.NETWORK: PanelInfo(tr_noop("Network"), NetworkUI(wifi_manager)),
      PanelType.TOGGLES: PanelInfo(tr_noop("Toggles"), TogglesLayout()),
      PanelType.SOFTWARE: PanelInfo(tr_noop("Software"), SoftwareLayout()),
      PanelType.NAP: PanelInfo(tr_noop("NAP"), NAPLayout()),
      PanelType.FIREHOSE: PanelInfo(tr_noop("Firehose"), FirehoseLayout()),
      PanelType.DEVELOPER: PanelInfo(tr_noop("Developer"), DeveloperLayout()),
    }

    self._font_medium = gui_app.font(FontWeight.MEDIUM)
    self._close_icon = gui_app.texture("icons/close2.png", CLOSE_ICON_SIZE, CLOSE_ICON_SIZE)

    # Hidden NAP toggles: 3 taps on the NAP sidebar item within 1.0 s.
    # Counted here (not only in set_current_panel) so re-taps while already
    # on NAP still count.
    self._nap_triple_tap = TripleTapDetector()
    self._hidden_popup_visible = False
    self._hidden_popup = HiddenTogglesPopup(on_dismiss=self._dismiss_hidden_popup)
    self._hidden_popup.set_visible(False)

    # Callbacks
    self._close_callback: Callable | None = None

  def set_callbacks(self, on_close: Callable):
    self._close_callback = on_close

  def _render(self, rect: rl.Rectangle):
    # Calculate layout
    sidebar_rect = rl.Rectangle(rect.x, rect.y, SIDEBAR_WIDTH, rect.height)
    panel_rect = rl.Rectangle(rect.x + SIDEBAR_WIDTH, rect.y, rect.width - SIDEBAR_WIDTH, rect.height)

    # Draw components
    self._draw_sidebar(sidebar_rect)
    self._draw_current_panel(panel_rect)

  def _draw_sidebar(self, rect: rl.Rectangle):
    rl.draw_rectangle_rec(rect, SIDEBAR_COLOR)

    # Close button
    close_btn_rect = rl.Rectangle(
      rect.x + (rect.width - CLOSE_BTN_SIZE) / 2, rect.y + 60, CLOSE_BTN_SIZE, CLOSE_BTN_SIZE
    )

    pressed = (rl.is_mouse_button_down(rl.MouseButton.MOUSE_BUTTON_LEFT) and
               rl.check_collision_point_rec(rl.get_mouse_position(), close_btn_rect))
    close_color = CLOSE_BTN_PRESSED if pressed else CLOSE_BTN_COLOR
    rl.draw_rectangle_rounded(close_btn_rect, 1.0, 20, close_color)

    icon_color = rl.Color(255, 255, 255, 255) if not pressed else rl.Color(220, 220, 220, 255)
    icon_dest = rl.Rectangle(
      close_btn_rect.x + (close_btn_rect.width - self._close_icon.width) / 2,
      close_btn_rect.y + (close_btn_rect.height - self._close_icon.height) / 2,
      self._close_icon.width,
      self._close_icon.height,
    )
    rl.draw_texture_pro(
      self._close_icon,
      rl.Rectangle(0, 0, self._close_icon.width, self._close_icon.height),
      icon_dest,
      rl.Vector2(0, 0),
      0,
      icon_color,
    )

    # Store close button rect for click detection
    self._close_btn_rect = close_btn_rect

    # Navigation buttons
    y = rect.y + 300
    for panel_type, panel_info in self._panels.items():
      button_rect = rl.Rectangle(rect.x + 50, y, rect.width - 150, NAV_BTN_HEIGHT)

      # Button styling
      is_selected = panel_type == self._current_panel
      text_color = TEXT_SELECTED if is_selected else TEXT_NORMAL
      # Draw button text (right-aligned)
      panel_name = tr(panel_info.name)
      text_size = measure_text_cached(self._font_medium, panel_name, 65)
      text_pos = rl.Vector2(
        button_rect.x + button_rect.width - text_size.x, button_rect.y + (button_rect.height - text_size.y) / 2
      )
      rl.draw_text_ex(self._font_medium, panel_name, text_pos, 65, 0, text_color)

      # Store button rect for click detection
      panel_info.button_rect = button_rect

      y += NAV_BTN_HEIGHT

  def _draw_current_panel(self, rect: rl.Rectangle):
    rl.draw_rectangle_rounded(
      rl.Rectangle(rect.x + 10, rect.y + 10, rect.width - 20, rect.height - 20), 0.04, 30, PANEL_COLOR
    )
    content_rect = rl.Rectangle(rect.x + PANEL_MARGIN, rect.y + 25, rect.width - (PANEL_MARGIN * 2), rect.height - 50)
    # rl.draw_rectangle_rounded(content_rect, 0.03, 30, PANEL_COLOR)
    if self._hidden_popup_visible:
      # Skip the live panel so click-outside cannot hit list items underneath.
      self._hidden_popup.render(rect)
      return
    panel = self._panels[self._current_panel]
    if panel.instance:
      panel.instance.render(content_rect)

  def _handle_mouse_release(self, mouse_pos: MousePos) -> None:
    # Check close button
    if rl.check_collision_point_rec(mouse_pos, self._close_btn_rect):
      self._dismiss_hidden_popup()
      if self._close_callback:
        self._close_callback()
      return

    # Check navigation buttons
    for panel_type, panel_info in self._panels.items():
      if rl.check_collision_point_rec(mouse_pos, panel_info.button_rect):
        if panel_type == PanelType.NAP:
          self._on_nap_sidebar_tap()
        else:
          self._nap_triple_tap.reset()
          self._dismiss_hidden_popup()
        self.set_current_panel(panel_type)
        return

  def _on_nap_sidebar_tap(self):
    # Tap NAP again while the popup is open: dismiss, do not immediately re-open.
    if self._hidden_popup_visible:
      self._dismiss_hidden_popup()
      self._nap_triple_tap.reset()
      return
    if self._nap_triple_tap.tap(time.monotonic()):
      self._show_hidden_popup()

  def _show_hidden_popup(self):
    self._hidden_popup_visible = True
    self._hidden_popup.set_visible(True)
    self._hidden_popup.show_event()

  def _dismiss_hidden_popup(self):
    if not self._hidden_popup_visible:
      return
    self._hidden_popup_visible = False
    self._hidden_popup.set_visible(False)

  def set_current_panel(self, panel_type: PanelType):
    if panel_type != self._current_panel:
      self._panels[self._current_panel].instance.hide_event()
      self._current_panel = panel_type
      self._panels[self._current_panel].instance.show_event()

  def show_event(self):
    super().show_event()
    self._panels[self._current_panel].instance.show_event()

  def hide_event(self):
    super().hide_event()
    self._dismiss_hidden_popup()
    self._nap_triple_tap.reset()
    self._panels[self._current_panel].instance.hide_event()
