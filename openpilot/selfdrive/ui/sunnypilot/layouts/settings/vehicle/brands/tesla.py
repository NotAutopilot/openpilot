"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.selfdrive.car.preap_boot import is_preap_ui_platform
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools import instructions as preap_instructions
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.runner import start_tool
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.safety import ToolSafetyError
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import (
  button_item_sp, multiple_button_item_sp, toggle_item_sp,
)
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog, alert_dialog
from openpilot.system.ui.widgets.keyboard import Keyboard

COOP_STEERING_MIN_KMH = 23
OEM_STEERING_MIN_KMH = 48
KM_TO_MILE = 0.621371

FOLLOW_DISTANCE_MIN = 1
FOLLOW_DISTANCE_MAX = 7
RADAR_OFFSET_MIN = -2.0
RADAR_OFFSET_MAX = 2.0


def _mode_label(mode: int) -> str:
  labels = (tr("Independent"), tr("Cruise Coupled"), tr("Longitudinal Only"))
  return labels[mode] if 0 <= mode < len(labels) else labels[0]


def _path_label(path: str) -> str:
  return tr("Pedal") if path == "pedal" else tr("Stock DI")


def _health_label(value: str) -> str:
  return {
    "none": tr("None"),
    "ok": tr("OK"),
    "uncalibrated": tr("Uncalibrated"),
    "unconfigured": tr("Unconfigured"),
  }.get(value, value)


def _bundle_platform() -> str:
  bundle = ui_state.params.get("CarPlatformBundle")
  if isinstance(bundle, dict):
    return str(bundle.get("platform", "") or "")
  return ""


def is_tesla_preap_ui() -> bool:
  """Native Tesla Pre-AP visibility. Never inferred from HAS_VEHICLE_BUS."""
  return is_preap_ui_platform(_bundle_platform(), ui_state.CP)


def parse_configured_pedal_bus(value) -> int:
  """Preserve configured bus 0. Absent or empty defaults to 2."""
  if value is None or value == "" or value == b"":
    return 2
  return int(value)


def pedal_bus_selector_index(value) -> int:
  """Button 0 = bus 0, button 1 = bus 2 (including the empty default)."""
  return 0 if parse_configured_pedal_bus(value) == 0 else 1


class TeslaSettings(BrandSettings):
  def __init__(self):
    super().__init__()
    self.coop_steering_toggle = toggle_item_sp(tr("Cooperative Steering (Beta)"), "", param="TeslaCoopSteering")
    self.mads_screen_button = multiple_button_item_sp(
      title=lambda: tr("MADS Screen Activation"),
      description="",
      buttons=[lambda: tr("Off"), lambda: tr("3-Finger"), lambda: tr("4-Finger"), lambda: tr("5-Finger")],
      param="TeslaMadsScreenButton",
      inline=False,
    )
    self.engagement_mode = multiple_button_item_sp(
      title=lambda: tr("Lateral Engagement Mode"),
      description=lambda: tr("Pre-AP stalk engagement. Onroad writes apply on the next drive."),
      buttons=[lambda: tr("Independent"), lambda: tr("Cruise Coupled"), lambda: tr("Longitudinal Only")],
      param="NAPLateralEngagementMode",
      inline=False,
      button_width=280,
    )
    self.follow_distance = multiple_button_item_sp(
      title=lambda: tr("Follow Distance"),
      description=lambda: tr("Follow distance (1=closest, 7=farthest). Writes apply live."),
      buttons=[str(i) for i in range(FOLLOW_DISTANCE_MIN, FOLLOW_DISTANCE_MAX + 1)],
      inline=True,
      button_width=90,
      callback=self._on_follow_selected,
    )
    self.pedal_enabled = toggle_item_sp(tr("Pedal Interceptor"), tr("Enable Comma Pedal hardware. Local and offroad only."),
                                        param="NAPPedalEnabled")
    self.pedal_bus = multiple_button_item_sp(
      title=lambda: tr("Pedal CAN Bus"),
      description=lambda: tr("CAN bus for the Comma Pedal. Local and offroad only."),
      buttons=[lambda: tr("Bus 0"), lambda: tr("Bus 2")],
      inline=True,
      callback=self._on_pedal_bus_selected,
    )
    self.radar_enabled = toggle_item_sp(tr("Bosch Radar"), tr("Enable stock Bosch radar. Local and offroad only."),
                                        param="NAPRadarEnabled")
    self.radar_nosecone = toggle_item_sp(tr("Radar Behind Nosecone"),
                                         tr("Attenuate radar mounted behind the nosecone. Local and offroad only."),
                                         param="NAPRadarBehindNosecone")
    self._radar_offset_keyboard = Keyboard(max_text_size=10)
    self.radar_offset = button_item_sp(
      lambda: tr("Radar Lateral Offset"),
      self._get_radar_offset_text,
      description=lambda: tr(
        "Lateral offset in meters added to radar yRel. Negative shifts leads left of the current radar reading; "
        + "positive shifts right. Local and offroad only."
      ),
      callback=self._on_radar_offset_click,
    )
    self.status_mode = button_item_sp(lambda: tr("Active Engagement Mode"), "", enabled=False)
    self.status_path = button_item_sp(lambda: tr("Longitudinal Path"), "", enabled=False)
    self.status_pedal = button_item_sp(lambda: tr("Pedal Health"), "", enabled=False)
    self.status_radar = button_item_sp(lambda: tr("Radar Health"), "", enabled=False)

    self.tool_calibrate_pedal = button_item_sp(lambda: tr("Calibrate Pedal"), lambda: tr("Start"),
                                               callback=lambda: self._confirm_tool("calibrate_pedal"))
    self.tool_calibrate_radar = button_item_sp(lambda: tr("Calibrate Radar"), lambda: tr("Start"),
                                               callback=lambda: self._confirm_tool("calibrate_radar"))
    self.tool_diagnose_radar = button_item_sp(lambda: tr("Diagnose Radar"), lambda: tr("Start"),
                                              callback=lambda: self._confirm_tool("diagnose_radar"))
    self.tool_test_radar = button_item_sp(lambda: tr("Test Radar"), lambda: tr("Start"),
                                          callback=lambda: self._confirm_tool("test_radar"))
    self.tool_extract_epas = button_item_sp(lambda: tr("Backup EPAS"), lambda: tr("Start"),
                                            callback=lambda: self._confirm_tool("extract_epas"))
    self.tool_flash_epas = button_item_sp(lambda: tr("Flash EPAS"), lambda: tr("Start"),
                                          callback=lambda: self._confirm_tool("flash_epas"))
    self.tool_restore_epas = button_item_sp(lambda: tr("Restore EPAS"), lambda: tr("Start"),
                                            callback=lambda: self._confirm_tool("restore_epas"))

    self._preap_items = [
      self.engagement_mode,
      self.follow_distance,
      self.pedal_enabled,
      self.pedal_bus,
      self.radar_enabled,
      self.radar_nosecone,
      self.radar_offset,
      self.status_mode,
      self.status_path,
      self.status_pedal,
      self.status_radar,
      self.tool_calibrate_pedal,
      self.tool_calibrate_radar,
      self.tool_diagnose_radar,
      self.tool_test_radar,
      self.tool_extract_epas,
      self.tool_flash_epas,
      self.tool_restore_epas,
    ]
    self.items = [self.coop_steering_toggle, self.mads_screen_button, *self._preap_items]
    self._tool_instructions = {
      "calibrate_pedal": preap_instructions.CALIBRATE_PEDAL_INSTRUCTIONS,
      "calibrate_radar": preap_instructions.CALIBRATE_RADAR_INSTRUCTIONS,
      "diagnose_radar": preap_instructions.DIAGNOSE_RADAR_INSTRUCTIONS,
      "test_radar": preap_instructions.TEST_RADAR_INSTRUCTIONS,
      "extract_epas": preap_instructions.BACKUP_EPAS_INSTRUCTIONS,
      "flash_epas": preap_instructions.FLASH_EPAS_INSTRUCTIONS,
      "restore_epas": preap_instructions.RESTORE_EPAS_INSTRUCTIONS,
    }

  @staticmethod
  def _on_follow_selected(index):
    ui_state.params.put("NAPFollowDistance", index + FOLLOW_DISTANCE_MIN)

  @staticmethod
  def _on_pedal_bus_selected(index):
    ui_state.params.put("NAPPedalCanBus", 0 if index == 0 else 2)

  def _get_radar_offset(self) -> float:
    raw = ui_state.params.get("NAPRadarOffset")
    try:
      return float(raw)
    except (TypeError, ValueError):
      return 0.0

  def _get_radar_offset_text(self) -> str:
    return f"{self._get_radar_offset():+.2f} m"

  def _on_radar_offset_click(self):
    if not ui_state.is_offroad():
      gui_app.push_widget(alert_dialog(tr("Hardware settings are only available offroad.")))
      return
    self._radar_offset_keyboard.reset(min_text_size=1)
    self._radar_offset_keyboard.set_title(tr("Radar Lateral Offset (m)"))
    self._radar_offset_keyboard.set_text(f"{self._get_radar_offset():.2f}")
    self._radar_offset_keyboard.set_callback(self._on_radar_offset_submit)
    gui_app.push_widget(self._radar_offset_keyboard)

  def _on_radar_offset_submit(self, result: DialogResult):
    if result != DialogResult.CONFIRM:
      return
    try:
      text = (self._radar_offset_keyboard.text or "").strip()
      value = float(text)
    except (TypeError, ValueError, AttributeError):
      return
    value = max(RADAR_OFFSET_MIN, min(RADAR_OFFSET_MAX, value))
    try:
      ui_state.params.put("NAPRadarOffset", value)
    except Exception:
      pass

  def _confirm_tool(self, tool: str):
    if not ui_state.is_offroad():
      gui_app.push_widget(alert_dialog(tr("Tools are only available offroad.")))
      return

    def callback(result: DialogResult):
      if result != DialogResult.CONFIRM:
        return
      try:
        start_tool(tool, confirmed=True)
      except (ToolSafetyError, ValueError) as exc:
        gui_app.push_widget(alert_dialog(str(exc)))

    text = tr(self._tool_instructions[tool]).replace("\n", "<br>")
    gui_app.push_widget(ConfirmDialog(text, tr("Start"), rich=True, callback=callback))

  def _update_preap_status(self, is_preap: bool):
    flags = int(getattr(ui_state.CP_SP, "flags", 0) or 0) if ui_state.CP_SP is not None else 0
    pedal_present = bool(flags & TeslaFlagsSP.PREAP_PEDAL_PRESENT)
    pedal_calib = bool(flags & TeslaFlagsSP.PREAP_PEDAL_CALIB_AVAILABLE)
    radar_present = bool(flags & TeslaFlagsSP.PREAP_RADAR_PRESENT)

    if ui_state.CP_SP is not None:
      try:
        mode = int(ui_state.CP_SP.preapLateralEngagementMode)
      except Exception:
        mode = int(ui_state.params.get("NAPLateralEngagementMode") or 0)
    else:
      mode = int(ui_state.params.get("NAPLateralEngagementMode") or 0)
    self.status_mode.action_item.set_value(_mode_label(mode))

    if pedal_present and ui_state.CP is not None and bool(ui_state.CP.openpilotLongitudinalControl) and not bool(ui_state.CP.pcmCruise):
      path = "pedal"
    else:
      path = "stock_di"
    self.status_path.action_item.set_value(_path_label(path))

    if not pedal_present:
      pedal_health = "none"
    elif not pedal_calib:
      pedal_health = "uncalibrated"
    else:
      pedal_health = "ok"
    self.status_pedal.action_item.set_value(_health_label(pedal_health))

    if not radar_present:
      radar_health = "none"
    elif ui_state.CP is not None and bool(ui_state.CP.radarUnavailable):
      radar_health = "unconfigured"
    else:
      radar_health = "ok"
    self.status_radar.action_item.set_value(_health_label(radar_health))

    follow = int(ui_state.params.get("NAPFollowDistance") or 4)
    follow = max(FOLLOW_DISTANCE_MIN, min(FOLLOW_DISTANCE_MAX, follow))
    self.follow_distance.action_item.set_selected_button(follow - FOLLOW_DISTANCE_MIN)

    self.pedal_bus.action_item.set_selected_button(
      pedal_bus_selector_index(ui_state.params.get("NAPPedalCanBus")))

    self.pedal_bus.set_visible(is_preap and pedal_present)
    self.radar_nosecone.set_visible(is_preap and radar_present)
    self.radar_offset.set_visible(is_preap and radar_present)
    self.tool_calibrate_pedal.set_visible(is_preap and pedal_present)
    self.tool_calibrate_radar.set_visible(is_preap and radar_present)

  def update_settings(self):
    is_metric = ui_state.is_metric
    unit = "km/h" if is_metric else "mph"
    offroad = ui_state.is_offroad()
    is_preap = is_tesla_preap_ui()

    display_value_coop = COOP_STEERING_MIN_KMH if is_metric else round(COOP_STEERING_MIN_KMH * KM_TO_MILE)
    display_value_oem = OEM_STEERING_MIN_KMH if is_metric else round(OEM_STEERING_MIN_KMH * KM_TO_MILE)

    coop_steering_disabled_msg = tr("Enable \"Always Offroad\" in Device panel, or turn vehicle off to toggle.")
    coop_steering_warning = tr(f"Warning: May experience steering oscillations below {display_value_oem} {unit} during turns, " +
                               "recommend disabling this feature if you experience these.")
    coop_steering_desc = (
      f"<b>{coop_steering_warning}</b><br><br>" +
      f"{tr('Allows the driver to provide limited steering input while openpilot is engaged.')}<br>" +
      f"{tr(f'Only works above {display_value_coop} {unit}.')}"
    )
    if not offroad:
      coop_steering_desc = f"<b>{coop_steering_disabled_msg}</b><br><br>{coop_steering_desc}"

    self.coop_steering_toggle.set_description(coop_steering_desc)
    self.coop_steering_toggle.action_item.set_enabled(offroad)
    self.coop_steering_toggle.set_visible(not is_preap)

    has_vehicle_bus = ui_state.CP_SP is not None and bool(ui_state.CP_SP.flags & TeslaFlagsSP.HAS_VEHICLE_BUS)
    self.mads_screen_button.set_visible((not is_preap) and has_vehicle_bus)

    mads_screen_button_desc = (
      f"{tr('Use a multi-finger press on the infotainment screen to toggle MADS.')} " +
      f"{tr('This allows the use of full MADS functionality when enabled.')}<br><br>" +
      f"{tr('Selecting a higher finger count may reduce accidental activations.')}<br><br>" +
      f"<b>{tr('Note: Setting this to Off will reset your MADS settings to default.')}</b>"
    )
    if not offroad:
      mads_screen_button_disabled_msg = tr("Enable \"Always Offroad\" in Device panel, or turn vehicle off to change.")
      mads_screen_button_desc = f"<b>{mads_screen_button_disabled_msg}</b><br><br>{mads_screen_button_desc}"
    self.mads_screen_button.set_description(mads_screen_button_desc)
    self.mads_screen_button.action_item.set_enabled(offroad)

    for item in self._preap_items:
      item.set_visible(is_preap)

    if is_preap:
      self._update_preap_status(is_preap)
      self.pedal_enabled.action_item.set_enabled(offroad)
      self.pedal_bus.action_item.set_enabled(offroad)
      self.radar_enabled.action_item.set_enabled(offroad)
      self.radar_nosecone.action_item.set_enabled(offroad)
      self.radar_offset.action_item.set_enabled(offroad)
      for tool in (
        self.tool_calibrate_pedal, self.tool_calibrate_radar, self.tool_diagnose_radar,
        self.tool_test_radar, self.tool_extract_epas, self.tool_flash_epas, self.tool_restore_epas,
      ):
        tool.action_item.set_enabled(offroad)
