import pyray as rl
from openpilot.common.params import Params
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.list_view import (
  toggle_item, multiple_button_item, button_item, text_item,
  ListItem, ITEM_PADDING, ITEM_TEXT_COLOR,
)
from openpilot.system.ui.widgets.scroller import Scroller
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets.html_render import HtmlRenderer, ElementType
from opendbc.car.tesla.nap_params import NAPParamKeys, DEFAULTS

# Preset values for float/int params exposed as multiple-button selectors
SPEED_OFFSET_PRESETS = [0.0, 5.0, 10.0, 15.0]
BRAKE_FACTOR_PRESETS = [0.5, 1.0, 1.5, 2.0]
PEDAL_CAN_BUS_VALUES = [0, 2]


def _find_preset_index(presets, value, default=0):
  """Find the closest matching preset index for a given value."""
  try:
    return presets.index(value)
  except ValueError:
    closest = min(range(len(presets)), key=lambda i: abs(presets[i] - value))
    return closest


class SectionHeader(Widget):
  """Lightweight section label to visually separate groups of settings."""

  HEADER_HEIGHT = 70

  def __init__(self, title: str):
    super().__init__()
    self._title = title
    self._font = gui_app.font(FontWeight.BOLD)
    self.set_rect(rl.Rectangle(0, 0, 0, self.HEADER_HEIGHT))

  def set_parent_rect(self, parent_rect: rl.Rectangle):
    super().set_parent_rect(parent_rect)
    self._rect.width = parent_rect.width

  def _render(self, rect):
    text_size = measure_text_cached(self._font, self._title, 40)
    text_y = self._rect.y + (self._rect.height - text_size.y) / 2
    rl.draw_text_ex(
      self._font, self._title,
      rl.Vector2(self._rect.x + ITEM_PADDING, text_y),
      40, 0, rl.Color(180, 180, 180, 255),
    )


class CreditsBlock(Widget):
  """Static block of HTML-rendered credits text with auto-sized height."""

  def __init__(self, html: str):
    super().__init__()
    self._html = HtmlRenderer(
      text=html,
      text_size={ElementType.P: 40},
      text_color=rl.Color(140, 140, 140, 255),
    )
    self.set_rect(rl.Rectangle(0, 0, 0, 200))  # placeholder height

  def set_parent_rect(self, parent_rect: rl.Rectangle):
    super().set_parent_rect(parent_rect)
    self._rect.width = parent_rect.width
    content_w = int(self._rect.width - ITEM_PADDING * 2)
    self._rect.height = self._html.get_total_height(content_w) + ITEM_PADDING

  def _render(self, rect):
    content_w = int(self._rect.width - ITEM_PADDING * 2)
    h = self._html.get_total_height(content_w)
    html_rect = rl.Rectangle(
      self._rect.x + ITEM_PADDING, self._rect.y,
      content_w, h,
    )
    self._html.set_rect(html_rect)
    self._html.render(html_rect)


def section_header_item(title: str) -> SectionHeader:
  return SectionHeader(title)


class NAPLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._build_items()
    self._scroller = Scroller(self._all_items, line_separator=True, spacing=0)

  def _build_items(self):
    """Build all list items organized into sections."""
    self._all_items = []
    self._toggle_map = {}  # param_key -> ListItem (for refresh)

    # ── Section 1: Longitudinal Control ──
    self._all_items.append(section_header_item("Longitudinal Control"))

    self._add_toggle(
      NAPParamKeys.PEDAL_ENABLED,
      "Pedal Interceptor",
      "Enable Comma Pedal hardware for direct throttle control.",
    )

    self._add_toggle(
      NAPParamKeys.DISABLE_CRUISE_CONTROL,
      "Disable Stock Cruise Control",
      "Send CANCEL to prevent stock cruise control from engaging over pedal cruise.",
    )

    follow_dist = self._params.get(NAPParamKeys.FOLLOW_DISTANCE, return_default=True)
    self._follow_buttons = multiple_button_item(
      "Follow Distance",
      "Set the following distance behind the lead car.",
      buttons=["Close", "Med", "Far", "Max"],
      button_width=150,
      selected_index=max(0, min(3, follow_dist - 1)),
      callback=self._on_follow_distance,
    )
    self._all_items.append(self._follow_buttons)

    self._add_toggle(
      NAPParamKeys.AUTORESUME_ACC,
      "Auto-Resume ACC",
      "Automatically resume adaptive cruise control after a stop.",
    )

    self._add_toggle(
      NAPParamKeys.ENABLE_JUST_CC,
      "Enable Just CC",
      "Enable conventional (non-adaptive) cruise control mode.",
    )

    # ── Section 3: Pedal Hardware ──
    self._all_items.append(section_header_item("Pedal Hardware"))

    pedal_profile = self._params.get(NAPParamKeys.PEDAL_PROFILE, return_default=True)
    self._pedal_profile_buttons = multiple_button_item(
      "Pedal Profile",
      "Select the pedal response curve for your vehicle model.",
      buttons=["1", "2", "3", "4"],
      button_width=100,
      selected_index=max(0, min(3, pedal_profile - 1)),
      callback=self._on_pedal_profile,
    )
    self._all_items.append(self._pedal_profile_buttons)

    pedal_bus = self._params.get(NAPParamKeys.PEDAL_CAN_BUS, return_default=True)
    self._pedal_bus_buttons = multiple_button_item(
      "Pedal CAN Bus",
      "Select which CAN bus the Comma Pedal is connected to.",
      buttons=["Bus 0", "Bus 2"],
      button_width=150,
      selected_index=0 if pedal_bus == 0 else 1,
      callback=self._on_pedal_can_bus,
    )
    self._all_items.append(self._pedal_bus_buttons)

    self._pedal_calib_status = text_item(
      "Pedal Calibration",
      lambda: "Calibrated" if self._params.get_bool(NAPParamKeys.PEDAL_CALIB_DONE) else "Not Calibrated",
      description="Shows whether the pedal interceptor has been calibrated.",
    )
    self._all_items.append(self._pedal_calib_status)

    self._calibrate_pedal_btn = button_item(
      "Calibrate Pedal",
      "Start",
      description="Run the pedal calibration routine. Vehicle must be stationary with ignition on.",
      callback=self._on_calibrate_pedal,
    )
    self._all_items.append(self._calibrate_pedal_btn)

    # ── Section 4: Radar ──
    self._all_items.append(section_header_item("Radar"))

    self._add_toggle(
      NAPParamKeys.RADAR_ENABLED,
      "Radar Enabled",
      "Enable the stock Bosch radar for lead car detection.",
    )

    self._add_toggle(
      NAPParamKeys.RADAR_BEHIND_NOSECONE,
      "Radar Behind Nosecone",
      "Apply signal attenuation adjustment for radar mounted behind the nosecone.",
    )

    self._calibrate_radar_btn = button_item(
      "Calibrate Radar",
      "Start",
      description="Run the radar calibration routine.",
      callback=self._on_calibrate_radar,
    )
    self._all_items.append(self._calibrate_radar_btn)

    self._test_radar_btn = button_item(
      "Test Radar",
      "Test",
      description="Test radar connectivity and verify signals.",
      callback=self._on_test_radar,
    )
    self._all_items.append(self._test_radar_btn)

    # ── Section 5: Speed Limit ──
    self._all_items.append(section_header_item("Speed Limit"))

    self._add_toggle(
      NAPParamKeys.ADJUST_ACC_WITH_SPEED_LIMIT,
      "Adjust Speed with Speed Limit",
      "Automatically adjust cruise speed based on detected speed limit signs.",
    )

    self._add_toggle(
      NAPParamKeys.SPEED_LIMIT_USE_RELATIVE,
      "Use Relative Offset",
      "Apply speed limit offset as a relative value instead of absolute.",
    )

    speed_offset = self._params.get(NAPParamKeys.SPEED_LIMIT_OFFSET, return_default=True)
    self._speed_offset_buttons = multiple_button_item(
      "Speed Limit Offset",
      "How much over the speed limit to set cruise speed.",
      buttons=["0", "+5", "+10", "+15"],
      button_width=120,
      selected_index=_find_preset_index(SPEED_OFFSET_PRESETS, speed_offset),
      callback=self._on_speed_offset,
    )
    self._all_items.append(self._speed_offset_buttons)

    # ── Section 6: iBooster / Braking (not yet implemented — greyed out) ──
    self._all_items.append(section_header_item("iBooster / Braking"))

    self._add_toggle(
      NAPParamKeys.IBOOSTER_ENABLED,
      "iBooster Enabled",
      "Enable the iBooster brake-by-wire system for electronic braking. (Not yet implemented)",
      enabled=False,
    )

    brake_factor = self._params.get(NAPParamKeys.BRAKE_FACTOR, return_default=True)
    self._brake_factor_buttons = multiple_button_item(
      "Brake Factor",
      "Multiplier for brake force. Higher values brake more aggressively. (Not yet implemented)",
      buttons=["0.5x", "1.0x", "1.5x", "2.0x"],
      button_width=130,
      selected_index=_find_preset_index(BRAKE_FACTOR_PRESETS, brake_factor),
      callback=self._on_brake_factor,
    )
    self._brake_factor_buttons.action_item.set_enabled(False)
    self._all_items.append(self._brake_factor_buttons)

    # ── Section 7: Advanced ──
    self._all_items.append(section_header_item("Advanced"))

    self._add_toggle(
      NAPParamKeys.FORCE_PRE_AP,
      "Force Pre-AP Mode",
      "Force the system to treat this vehicle as a Pre-Autopilot Tesla.",
    )

    self._add_toggle(
      NAPParamKeys.USE_LONG_CONTROL_DATA,
      "Use Longitudinal Control Data",
      "Enable advanced longitudinal control data path.",
    )

    # ── Section 8: Actions ──
    self._all_items.append(section_header_item("Actions"))

    self._flash_epas_btn = button_item(
      "Flash EPAS",
      "Flash",
      description="Flash the EPAS (Electric Power Assisted Steering) firmware.",
      callback=self._on_flash_epas,
    )
    self._all_items.append(self._flash_epas_btn)

    self._emergency_disable_btn = button_item(
      "Emergency Disable",
      "Disable",
      description="Immediately disable pedal interceptor and clear calibration. Restart required.",
      callback=self._on_emergency_disable,
    )
    self._all_items.append(self._emergency_disable_btn)

    self._reset_defaults_btn = button_item(
      "Reset to Defaults",
      "Reset",
      description="Reset all NAP settings to factory defaults. This cannot be undone.",
      callback=self._on_reset_defaults,
    )
    self._all_items.append(self._reset_defaults_btn)

    # ── Acknowledgments ──
    self._all_items.append(section_header_item("Acknowledgments"))
    self._all_items.append(CreditsBlock(
      "<p>Special thanks to the following members. "
      "This project wouldn't be possible without you:</p>"
      "<p><b>Boggyver</b> and the <b>Tinkla Project</b><br>"
      "<b>Lukas Loetkolben</b><br>"
      "<b>Johnmr1</b><br>"
      "<b>SeriouslySerious</b><br>"
      "<b>Pod042</b><br>"
      "<b>1FrostlySlime</b></p>"
    ))

  def _add_toggle(self, param_key: str, title: str, description: str, enabled: bool | None = None):
    """Helper to add a toggle item and register it for state refresh."""
    kwargs = {}
    if enabled is not None:
      kwargs['enabled'] = enabled
    item = toggle_item(
      title,
      description=description,
      initial_state=self._params.get_bool(param_key),
      callback=lambda state, k=param_key: self._params.put_bool(k, state),
      **kwargs,
    )
    self._toggle_map[param_key] = item
    self._all_items.append(item)

  # ── Multiple-button callbacks ──

  def _on_follow_distance(self, index: int):
    self._params.put(NAPParamKeys.FOLLOW_DISTANCE, index + 1)

  def _on_pedal_profile(self, index: int):
    self._params.put(NAPParamKeys.PEDAL_PROFILE, index + 1)

  def _on_pedal_can_bus(self, index: int):
    self._params.put(NAPParamKeys.PEDAL_CAN_BUS, PEDAL_CAN_BUS_VALUES[index])

  def _on_speed_offset(self, index: int):
    self._params.put(NAPParamKeys.SPEED_LIMIT_OFFSET, SPEED_OFFSET_PRESETS[index])

  def _on_brake_factor(self, index: int):
    self._params.put(NAPParamKeys.BRAKE_FACTOR, BRAKE_FACTOR_PRESETS[index])

  # ── Action button callbacks ──

  def _on_calibrate_pedal(self):
    # TODO: wire to ScriptRunner when available
    pass

  def _on_calibrate_radar(self):
    # TODO: wire to ScriptRunner when available
    pass

  def _on_test_radar(self):
    # TODO: wire to ScriptRunner when available
    pass

  def _on_flash_epas(self):
    # TODO: wire to ScriptRunner when available
    pass

  def _on_emergency_disable(self):
    def confirm_callback(result: int):
      if result == DialogResult.CONFIRM:
        self._params.put_bool(NAPParamKeys.PEDAL_ENABLED, False)
        self._params.put_bool(NAPParamKeys.PEDAL_CALIB_DONE, False)
        self._refresh_toggles()

    content = (
      "<h1>Emergency Disable</h1><br>"
      "<p>This will disable the pedal interceptor and clear calibration. "
      "You will need to restart the device for changes to take effect.</p>"
    )
    dlg = ConfirmDialog(content, "Disable", rich=True)
    gui_app.set_modal_overlay(dlg, callback=confirm_callback)

  def _on_reset_defaults(self):
    def confirm_callback(result: int):
      if result == DialogResult.CONFIRM:
        self._reset_all_to_defaults()
        self._refresh_toggles()

    content = (
      "<h1>Reset to Defaults</h1><br>"
      "<p>This will reset all NAP settings to their factory default values. "
      "This action cannot be undone.</p>"
    )
    dlg = ConfirmDialog(content, "Reset All", rich=True)
    gui_app.set_modal_overlay(dlg, callback=confirm_callback)

  def _reset_all_to_defaults(self):
    """Write default value for each NAP param."""
    for key, default in DEFAULTS.items():
      if isinstance(default, bool):
        self._params.put_bool(key, default)
      elif isinstance(default, (int, float)):
        self._params.put(key, default)

  # ── Render / lifecycle ──

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._scroller.show_event()
    self._refresh_toggles()

  def _refresh_toggles(self):
    """Sync all toggle states from params (handles external changes)."""
    for key, item in self._toggle_map.items():
      item.action_item.set_state(self._params.get_bool(key))

    # Refresh multiple-button selections
    follow_dist = self._params.get(NAPParamKeys.FOLLOW_DISTANCE, return_default=True)
    self._follow_buttons.action_item.set_selected_button(
      max(0, min(3, follow_dist - 1)))

    pedal_profile = self._params.get(NAPParamKeys.PEDAL_PROFILE, return_default=True)
    self._pedal_profile_buttons.action_item.set_selected_button(
      max(0, min(3, pedal_profile - 1)))

    pedal_bus = self._params.get(NAPParamKeys.PEDAL_CAN_BUS, return_default=True)
    self._pedal_bus_buttons.action_item.set_selected_button(
      0 if pedal_bus == 0 else 1)

    speed_offset = self._params.get(NAPParamKeys.SPEED_LIMIT_OFFSET, return_default=True)
    self._speed_offset_buttons.action_item.set_selected_button(
      _find_preset_index(SPEED_OFFSET_PRESETS, speed_offset))

    brake_factor = self._params.get(NAPParamKeys.BRAKE_FACTOR, return_default=True)
    self._brake_factor_buttons.action_item.set_selected_button(
      _find_preset_index(BRAKE_FACTOR_PRESETS, brake_factor))
