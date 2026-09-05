"""NAP sidebar for sunnypilot. Same groups as nap-dev, native Sunny widgets."""
from collections.abc import Callable

import pyray as rl

from opendbc.car.tesla.preap.nap_params import NAPParamKeys
from openpilot.common.params import Params
from openpilot.selfdrive.ui.layouts.settings.nap_content import (
  BACKUP_EPAS_INSTRUCTIONS, BRAKE_FACTOR_PRESETS, CALIBRATE_PEDAL_INSTRUCTIONS,
  DEFAULTS, FLASH_EPAS_INSTRUCTIONS, PEDAL_CAN_BUS_VALUES,
  RADAR_OFFSET_MAX, RADAR_OFFSET_MIN, RESTORE_EPAS_INSTRUCTIONS,
  acknowledgments_html, find_preset_index,
)
from openpilot.selfdrive.ui.radar.radar_view import RadarMonitorDialog
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools import instructions as preap_instructions
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.runner import launch_on_device_runner
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.safety import ToolSafetyError
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.sunnypilot.widgets.list_view import (
  button_item_sp, multiple_button_item_sp, text_item_sp, toggle_item_sp,
)
from openpilot.system.ui.widgets import DialogResult, Widget
from openpilot.system.ui.widgets.button import Button
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog, alert_dialog
from openpilot.system.ui.widgets.html_render import HtmlRenderer, ElementType
from openpilot.system.ui.widgets.keyboard import Keyboard
from openpilot.system.ui.widgets.list_view import ITEM_PADDING
from openpilot.system.ui.widgets.scroller_tici import Scroller


class SectionHeader(Widget):
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
  def __init__(self, html: str):
    super().__init__()
    self._html = HtmlRenderer(
      text=html,
      text_size={ElementType.P: 40},
      text_color=rl.Color(140, 140, 140, 255),
    )
    self.set_rect(rl.Rectangle(0, 0, 0, 200))

  def set_parent_rect(self, parent_rect: rl.Rectangle):
    super().set_parent_rect(parent_rect)
    self._rect.width = parent_rect.width
    content_w = int(self._rect.width - ITEM_PADDING * 2)
    self._rect.height = self._html.get_total_height(content_w) + ITEM_PADDING

  def _render(self, rect):
    content_w = int(self._rect.width - ITEM_PADDING * 2)
    h = self._html.get_total_height(content_w)
    html_rect = rl.Rectangle(self._rect.x + ITEM_PADDING, self._rect.y, content_w, h)
    self._html.set_rect(html_rect)
    self._html.render(html_rect)


def parse_configured_pedal_bus(value) -> int:
  if value in (None, "", b""):
    return 2
  return int(value)


def pedal_bus_selector_index(value) -> int:
  return 0 if parse_configured_pedal_bus(value) == 0 else 1


class NAPLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._page = "main"
    self._build_items()
    self._scroller = Scroller(self._main_items, line_separator=True, spacing=0)
    self._radar_scroller = Scroller(self._radar_items, line_separator=True, spacing=0)
    self._radar_back_btn = Button(tr("Back"), click_callback=self._close_radar)
    self._radar_back_btn.set_rect(rl.Rectangle(0, 0, 220, 90))

  def _build_items(self):
    self._main_items = []
    self._radar_items = []
    self._toggle_map = {}

    self._main_items.append(SectionHeader(tr("Longitudinal Control")))
    self._add_toggle(NAPParamKeys.PEDAL_ENABLED, tr("Pedal Interceptor"),
                     tr("Enable Comma Pedal hardware for direct throttle control. Requires reboot."),
                     enabled=ui_state.is_offroad, needs_reboot=True)
    self._add_toggle(NAPParamKeys.ADAPTIVE_ACCEL, tr("Adaptive Accel Limits"),
                     tr("Reduces acceleration authority when close to a lead car to prevent overshoot."))
    follow_dist = int(self._params.get(NAPParamKeys.FOLLOW_DISTANCE, return_default=True) or 4)
    self._follow_buttons = multiple_button_item_sp(
      title=lambda: tr("Follow Distance"),
      description=lambda: tr("Follow distance (1=closest, 7=farthest). Overridden by cruise stalk if present."),
      buttons=["1", "2", "3", "4", "5", "6", "7"],
      button_width=80,
      selected_index=max(0, min(6, follow_dist - 1)),
      callback=self._on_follow_distance,
    )
    self._main_items.append(self._follow_buttons)

    self._main_items.append(SectionHeader(tr("Pedal Hardware")))
    self._pedal_bus_buttons = multiple_button_item_sp(
      title=lambda: tr("Pedal CAN Bus"),
      description=lambda: tr("Select which CAN bus the Comma Pedal is connected to. Requires reboot."),
      buttons=[lambda: tr("Bus 0"), lambda: tr("Bus 2")],
      button_width=150,
      callback=self._on_pedal_can_bus,
    )
    self._pedal_bus_buttons.action_item.set_enabled(ui_state.is_offroad)
    self._main_items.append(self._pedal_bus_buttons)
    self._pedal_calib_status = text_item_sp(
      lambda: tr("Pedal Calibration"),
      lambda: tr("Calibrated") if self._params.get_bool(NAPParamKeys.PEDAL_CALIB_DONE) else tr("Not Calibrated"),
      description=lambda: tr("Shows whether the pedal interceptor has been calibrated."),
    )
    self._main_items.append(self._pedal_calib_status)
    self._calibrate_pedal_btn = button_item_sp(
      lambda: tr("Calibrate Pedal"), lambda: tr("Start"),
      description=lambda: tr("Run the pedal calibration routine. Vehicle must be stationary with ignition on."),
      callback=lambda: self._confirm_tool("calibrate_pedal"),
    )
    self._calibrate_pedal_btn.action_item.set_enabled(ui_state.is_offroad)
    self._main_items.append(self._calibrate_pedal_btn)

    self._main_items.append(SectionHeader(tr("Radar")))
    self._radar_settings_btn = button_item_sp(
      lambda: tr("Radar Settings"), lambda: tr("Open"),
      description=lambda: tr("Donor VIN, position, offset, test, and calibration."),
      callback=self._open_radar,
    )
    self._main_items.append(self._radar_settings_btn)
    self._build_radar_items()

    self._main_items.append(SectionHeader(tr("iBooster / Braking")))
    self._add_toggle(NAPParamKeys.IBOOSTER_ENABLED, tr("iBooster Enabled"),
                     tr("Enable the iBooster brake-by-wire system for electronic braking. (Not yet implemented)"),
                     enabled=False)
    brake_factor = self._params.get(NAPParamKeys.BRAKE_FACTOR, return_default=True)
    self._brake_factor_buttons = multiple_button_item_sp(
      title=lambda: tr("Brake Factor"),
      description=lambda: tr("Multiplier for brake force. Higher values brake more aggressively. (Not yet implemented)"),
      buttons=["0.5x", "1.0x", "1.5x", "2.0x"],
      button_width=130,
      selected_index=find_preset_index(BRAKE_FACTOR_PRESETS, brake_factor),
      callback=self._on_brake_factor,
    )
    self._brake_factor_buttons.action_item.set_enabled(False)
    self._main_items.append(self._brake_factor_buttons)

    self._main_items.append(SectionHeader(tr("Advanced")))
    self._params.put_bool(NAPParamKeys.FORCE_PRE_AP, True)
    self._add_toggle(NAPParamKeys.FORCE_PRE_AP, tr("Force Pre-AP Mode"),
                     tr("Force the system to treat this vehicle as a Pre-Autopilot Tesla."),
                     enabled=False)

    self._main_items.append(SectionHeader(tr("Actions")))
    self._backup_epas_btn = button_item_sp(
      lambda: tr("Backup EPAS"), lambda: tr("Extract"),
      description=lambda: tr("Extract and save stock EPAS firmware image without flashing."),
      callback=lambda: self._confirm_tool("extract_epas"),
    )
    self._backup_epas_btn.action_item.set_enabled(ui_state.is_offroad)
    self._main_items.append(self._backup_epas_btn)
    self._flash_epas_btn = button_item_sp(
      lambda: tr("Flash EPAS"), lambda: tr("Flash"),
      description=lambda: tr("Flash the EPAS (Electric Power Assisted Steering) firmware."),
      callback=lambda: self._confirm_tool("flash_epas"),
    )
    self._flash_epas_btn.action_item.set_enabled(ui_state.is_offroad)
    self._main_items.append(self._flash_epas_btn)
    self._restore_epas_btn = button_item_sp(
      lambda: tr("Restore EPAS"), lambda: tr("Restore"),
      description=lambda: tr("Restore stock EPAS firmware image."),
      callback=lambda: self._confirm_tool("restore_epas"),
    )
    self._restore_epas_btn.action_item.set_enabled(ui_state.is_offroad)
    self._main_items.append(self._restore_epas_btn)
    self._emergency_disable_btn = button_item_sp(
      lambda: tr("Emergency Disable"), lambda: tr("Disable"),
      description=lambda: tr("Immediately disable pedal interceptor and clear calibration. Restart required."),
      callback=self._on_emergency_disable,
    )
    self._main_items.append(self._emergency_disable_btn)
    self._reset_defaults_btn = button_item_sp(
      lambda: tr("Reset to Defaults"), lambda: tr("Reset"),
      description=lambda: tr("Reset all NAP settings to factory defaults. This cannot be undone."),
      callback=self._on_reset_defaults,
    )
    self._reset_defaults_btn.action_item.set_enabled(ui_state.is_offroad)
    self._main_items.append(self._reset_defaults_btn)

    self._main_items.append(SectionHeader(tr("Acknowledgments")))
    self._main_items.append(CreditsBlock(acknowledgments_html()))

  def _build_radar_items(self):
    self._radar_items.append(SectionHeader(tr("Radar")))
    self._add_toggle(NAPParamKeys.RADAR_ENABLED, tr("Radar Enabled"),
                     tr("Enable the stock Bosch radar for lead car detection. Requires reboot."),
                     enabled=ui_state.is_offroad, needs_reboot=True, dest=self._radar_items)
    self._add_toggle(NAPParamKeys.RADAR_IGNORE_HW_FAIL, tr("Ignore radar hardware fail"),
                     tr("Let's you engage when Bosch raises HWFail even though tracks are still live."),
                     dest=self._radar_items)

    def on_radar_hud(state):
      self._params.put_bool(NAPParamKeys.RADAR_HUD, state)
      ui_state.radar_hud = state

    hud_item = toggle_item_sp(
      tr("Radar HUD"),
      tr("Show live tracks, lamps, and alert bits over the driving view. Works while driving."),
      initial_state=self._params.get_bool(NAPParamKeys.RADAR_HUD),
      callback=on_radar_hud,
    )
    self._toggle_map[NAPParamKeys.RADAR_HUD] = hud_item
    self._radar_items.append(hud_item)

    self._radar_offset_keyboard = Keyboard(max_text_size=10)
    self._radar_offset_btn = button_item_sp(
      lambda: tr("Radar Lateral Offset"), self._get_radar_offset_text,
      description=lambda: tr(
        "Lateral offset in meters added to radar yRel. Negative shifts leads toward the left of current radar "
        + "reading; positive shifts right. Example: -0.27 for the 3D-printed factory-location mount."
      ),
      callback=self._on_radar_offset_click,
    )
    self._radar_items.append(self._radar_offset_btn)

    self._radar_vin_keyboard = Keyboard(max_text_size=17)
    self._radar_vin_btn = button_item_sp(
      lambda: tr("Donor Radar VIN"), self._get_radar_vin_text,
      description=lambda: tr(
        "VIN already programmed in a salvage radar. Leave empty to send this car. "
        + "A 17-character VIN makes panda impersonate the donor (Tinkla 0.6.6)."
      ),
      callback=self._on_radar_vin_click,
    )
    self._radar_items.append(self._radar_vin_btn)

    self._read_vin_btn = button_item_sp(
      lambda: tr("Read VIN"), self._get_read_vin_text,
      description=lambda: tr(
        "Read the VIN stored in the radar (F190). Does not write or flash. "
        + "Car must be on and not engaged. Power-cycle after it fills."
      ),
      callback=self._on_read_vin,
    )
    self._radar_items.append(self._read_vin_btn)

    radar_position = int(self._params.get(NAPParamKeys.RADAR_POSITION, return_default=True) or 0)
    self._radar_position_buttons = multiple_button_item_sp(
      title=lambda: tr("Donor Radar Position"),
      description=lambda: tr("Always sent when radar is on, including empty VIN. 0 pre-facelift S, 1 post-facelift S, 2 Model X."),
      buttons=["0", "1", "2"],
      button_width=130,
      selected_index=max(0, min(2, radar_position)),
      callback=self._on_radar_position,
    )
    self._radar_items.append(self._radar_position_buttons)

    radar_epas = int(self._params.get(NAPParamKeys.RADAR_EPAS_TYPE, return_default=True) or 0)
    self._radar_epas_buttons = multiple_button_item_sp(
      title=lambda: tr("Donor EPAS Type"),
      description=lambda: tr("Must match the donor car rack: 0 Bosch L538, 1 L405, 2 Mando FGR64, 3 VGR66, 4 VGR66 Gen3."),
      buttons=["0", "1", "2", "3", "4"],
      button_width=100,
      selected_index=max(0, min(4, radar_epas)),
      callback=self._on_radar_epas,
    )
    self._radar_items.append(self._radar_epas_buttons)

    self._live_radar_btn = button_item_sp(
      lambda: tr("Live Radar"), lambda: tr("Open"),
      description=lambda: tr("Live tracks, ECU lamps, and alert bits. Stays in this session; works while driving."),
      callback=lambda: gui_app.push_widget(RadarMonitorDialog()),
    )
    self._radar_items.append(self._live_radar_btn)

    self._tool_calibrate_radar = button_item_sp(
      lambda: tr("Calibrate Radar"), lambda: tr("CALIBRATE"),
      callback=lambda: self._confirm_tool("calibrate_radar"),
    )
    self._tool_diagnose_radar = button_item_sp(
      lambda: tr("Diagnose Radar"), lambda: tr("DIAGNOSE"),
      callback=lambda: self._confirm_tool("diagnose_radar"),
    )
    self._tool_test_radar = button_item_sp(
      lambda: tr("Test Radar"), lambda: tr("TEST"),
      callback=lambda: self._confirm_tool("test_radar"),
    )
    self._radar_items.extend([self._tool_calibrate_radar, self._tool_diagnose_radar, self._tool_test_radar])

  def _add_toggle(self, param_key, title, description, enabled: bool | Callable[[], bool] | None = None,
                  needs_reboot=False, dest=None):
    def on_toggle(state, k=param_key):
      self._params.put_bool(k, state)
      if needs_reboot:
        self._show_reboot_modal()

    item = toggle_item_sp(
      title,
      description,
      initial_state=self._params.get_bool(param_key),
      callback=on_toggle,
      enabled=True if enabled is None else enabled,
      param=param_key,
    )
    self._toggle_map[param_key] = item
    (dest if dest is not None else self._main_items).append(item)

  def _on_follow_distance(self, index: int):
    self._params.put(NAPParamKeys.FOLLOW_DISTANCE, index + 1)

  def _on_pedal_can_bus(self, index: int):
    self._params.put(NAPParamKeys.PEDAL_CAN_BUS, PEDAL_CAN_BUS_VALUES[index])
    self._show_reboot_modal()

  def _on_brake_factor(self, index: int):
    self._params.put(NAPParamKeys.BRAKE_FACTOR, BRAKE_FACTOR_PRESETS[index])

  def _get_radar_offset(self) -> float:
    raw = self._params.get(NAPParamKeys.RADAR_OFFSET, return_default=True)
    try:
      return float(raw) if raw is not None else 0.0
    except (TypeError, ValueError):
      return 0.0

  def _get_radar_offset_text(self) -> str:
    return f"{self._get_radar_offset():+.2f}m"

  def _get_radar_vin_text(self) -> str:
    raw = self._params.get(NAPParamKeys.RADAR_DONOR_VIN, return_default=True) or ""
    if isinstance(raw, bytes):
      raw = raw.decode("ascii", errors="ignore")
    vin = "".join(ch for ch in str(raw).upper() if ch.isalnum())
    return vin if len(vin) == 17 else tr("Not set")

  def _get_read_vin_text(self) -> str:
    if self._params.get_bool(NAPParamKeys.RADAR_READ_VIN):
      return tr("Reading...")
    raw = self._params.get(NAPParamKeys.RADAR_VIN_READ_STATUS, return_default=True) or ""
    if isinstance(raw, bytes):
      raw = raw.decode("ascii", errors="ignore")
    status = str(raw).strip()
    if status.startswith("saved "):
      return tr("Cycle car")
    if status:
      return status[:18]
    return tr("Read")

  def _open_radar(self):
    self._page = "radar"
    self._radar_scroller.show_event()

  def _close_radar(self):
    self._page = "main"
    self._scroller.show_event()

  def _on_read_vin(self):
    if ui_state.engaged:
      gui_app.push_widget(alert_dialog(tr("Disengage first")))
      return
    if not self._params.get_bool(NAPParamKeys.RADAR_ENABLED):
      gui_app.push_widget(alert_dialog(tr("Enable radar first")))
      return

    def confirm_callback(result: int):
      if result == DialogResult.CONFIRM:
        self._params.put_bool(NAPParamKeys.RADAR_READ_VIN, True)

    gui_app.push_widget(ConfirmDialog(tr("Reads the VIN stored in the radar. Does not write or flash."),
                                      tr("Read"), callback=confirm_callback))

  def _on_radar_vin_click(self):
    current = self._get_radar_vin_text()
    self._radar_vin_keyboard.reset(min_text_size=0)
    self._radar_vin_keyboard.set_title(tr("Donor Radar VIN"))
    self._radar_vin_keyboard.set_text("" if current == tr("Not set") else current)
    self._radar_vin_keyboard.set_callback(self._on_radar_vin_submit)
    gui_app.push_widget(self._radar_vin_keyboard)

  def _on_radar_vin_submit(self, result: DialogResult):
    if result != DialogResult.CONFIRM:
      return
    text = "".join(ch for ch in (self._radar_vin_keyboard.text or "").upper() if ch.isalnum())
    try:
      self._params.put(NAPParamKeys.RADAR_DONOR_VIN, text if len(text) == 17 else "")
    except Exception:
      pass

  def _on_radar_position(self, index: int):
    try:
      self._params.put(NAPParamKeys.RADAR_POSITION, int(index))
    except Exception:
      pass

  def _on_radar_epas(self, index: int):
    try:
      self._params.put(NAPParamKeys.RADAR_EPAS_TYPE, int(index))
    except Exception:
      pass

  def _on_radar_offset_click(self):
    self._radar_offset_keyboard.reset(min_text_size=1)
    self._radar_offset_keyboard.set_title(tr("Radar Lateral Offset (m)"))
    self._radar_offset_keyboard.set_text(f"{self._get_radar_offset():.2f}")
    self._radar_offset_keyboard.set_callback(self._on_radar_offset_submit)
    gui_app.push_widget(self._radar_offset_keyboard)

  def _on_radar_offset_submit(self, result: DialogResult):
    if result != DialogResult.CONFIRM:
      return
    try:
      value = float((self._radar_offset_keyboard.text or "").strip())
    except (TypeError, ValueError, AttributeError):
      return
    value = max(RADAR_OFFSET_MIN, min(RADAR_OFFSET_MAX, value))
    try:
      self._params.put(NAPParamKeys.RADAR_OFFSET, value)
    except Exception:
      pass

  def _confirm_tool(self, tool: str):
    if not ui_state.is_offroad():
      gui_app.push_widget(alert_dialog(tr("Tools are only available offroad.")))
      return
    instructions = {
      "calibrate_pedal": CALIBRATE_PEDAL_INSTRUCTIONS,
      "calibrate_radar": preap_instructions.CALIBRATE_RADAR_INSTRUCTIONS,
      "diagnose_radar": preap_instructions.DIAGNOSE_RADAR_INSTRUCTIONS,
      "test_radar": preap_instructions.TEST_RADAR_INSTRUCTIONS,
      "extract_epas": BACKUP_EPAS_INSTRUCTIONS,
      "flash_epas": FLASH_EPAS_INSTRUCTIONS,
      "restore_epas": RESTORE_EPAS_INSTRUCTIONS,
    }

    def callback(result: DialogResult):
      if result != DialogResult.CONFIRM:
        return
      try:
        launch_on_device_runner(tr(tool), tool, instructions[tool])
      except (ToolSafetyError, ValueError) as exc:
        gui_app.push_widget(alert_dialog(str(exc)))

    gui_app.push_widget(ConfirmDialog(tr(instructions[tool]).replace("\n", "<br>"),
                                      tr("Start"), rich=True, callback=callback))

  def _show_reboot_modal(self):
    def confirm_callback(result: int):
      if result == DialogResult.CONFIRM:
        self._params.put_bool("DoReboot", True)
    gui_app.push_widget(ConfirmDialog(
      "<h1>Reboot Required</h1><br><p>This change requires a reboot to take effect.</p>",
      tr("Reboot"), cancel_text=tr("Ignore"), rich=True, callback=confirm_callback))

  def _on_emergency_disable(self):
    def confirm_callback(result: int):
      if result == DialogResult.CONFIRM:
        self._params.put_bool(NAPParamKeys.PEDAL_ENABLED, False)
        self._params.put_bool(NAPParamKeys.PEDAL_CALIB_DONE, False)
        self._refresh_toggles()
    gui_app.push_widget(ConfirmDialog(
      "<h1>Emergency Disable</h1><br><p>This will disable the pedal interceptor and clear calibration.</p>",
      tr("Disable"), rich=True, callback=confirm_callback))

  def _on_reset_defaults(self):
    def confirm_callback(result: int):
      if result == DialogResult.CONFIRM:
        for key, default in DEFAULTS.items():
          if isinstance(default, bool):
            self._params.put_bool(key, default)
          elif isinstance(default, (int, float, str)):
            self._params.put(key, default)
        self._params.put_bool(NAPParamKeys.FORCE_PRE_AP, True)
        self._refresh_toggles()
    gui_app.push_widget(ConfirmDialog(
      "<h1>Reset to Defaults</h1><br><p>This will reset all NAP settings. This cannot be undone.</p>",
      tr("Reset All"), rich=True, callback=confirm_callback))

  def _render(self, rect):
    if self._page == "radar":
      self._radar_back_btn.set_position(rect.x, rect.y + 10)
      self._radar_back_btn.render()
      content = rl.Rectangle(
        rect.x, rect.y + self._radar_back_btn.rect.height + 20,
        rect.width, rect.height - self._radar_back_btn.rect.height - 20,
      )
      self._radar_scroller.render(content)
    else:
      self._scroller.render(rect)

  def show_event(self):
    self._page = "main"
    self._scroller.show_event()
    self._refresh_toggles()

  def _refresh_toggles(self):
    for key, item in self._toggle_map.items():
      item.action_item.set_state(self._params.get_bool(key))
    follow_dist = int(self._params.get(NAPParamKeys.FOLLOW_DISTANCE, return_default=True) or 4)
    self._follow_buttons.action_item.set_selected_button(max(0, min(6, follow_dist - 1)))
    self._pedal_bus_buttons.action_item.set_selected_button(
      pedal_bus_selector_index(self._params.get(NAPParamKeys.PEDAL_CAN_BUS)))
    brake_factor = self._params.get(NAPParamKeys.BRAKE_FACTOR, return_default=True)
    self._brake_factor_buttons.action_item.set_selected_button(
      find_preset_index(BRAKE_FACTOR_PRESETS, brake_factor))
    radar_position = int(self._params.get(NAPParamKeys.RADAR_POSITION, return_default=True) or 0)
    self._radar_position_buttons.action_item.set_selected_button(max(0, min(2, radar_position)))
    radar_epas = int(self._params.get(NAPParamKeys.RADAR_EPAS_TYPE, return_default=True) or 0)
    self._radar_epas_buttons.action_item.set_selected_button(max(0, min(4, radar_epas)))
