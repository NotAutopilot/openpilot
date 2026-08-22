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
from openpilot.system.ui.sunnypilot.lib.styles import style
from openpilot.system.ui.sunnypilot.widgets.list_view import (
  button_item_sp, multiple_button_item_sp, option_item_sp, text_item_sp, toggle_item_sp,
)
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog, alert_dialog

COOP_STEERING_MIN_KMH = 23
OEM_STEERING_MIN_KMH = 48
KM_TO_MILE = 0.621371

FOLLOW_DISTANCE_MIN = 1
FOLLOW_DISTANCE_MAX = 7
RADAR_OFFSET_MIN_CM = -200
RADAR_OFFSET_MAX_CM = 200
RADAR_OFFSET_STEP_CM = 5


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
  if value in (None, "", b""):
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
    self.follow_distance = option_item_sp(
      title=lambda: tr("Follow Distance"),
      param="NAPFollowDistance",
      min_value=FOLLOW_DISTANCE_MIN,
      max_value=FOLLOW_DISTANCE_MAX,
      description=lambda: tr("How far Pre-AP follows a detected lead. 1 is closest, 7 is farthest. Changes apply while driving."),
      label_callback=str,
      inline=True,
    )
    self.pedal_enabled = toggle_item_sp(
      tr("Pedal Interceptor"),
      tr("Enable the Comma Pedal interceptor."),
      param="NAPPedalEnabled",
    )
    self.pedal_bus = multiple_button_item_sp(
      title=lambda: tr("Pedal CAN Bus"),
      description=lambda: tr("CAN bus for the Comma Pedal."),
      buttons=[lambda: tr("Bus 0"), lambda: tr("Bus 2")],
      button_width=364,
      inline=True,
      callback=self._on_pedal_bus_selected,
    )
    self.radar_enabled = toggle_item_sp(
      tr("Bosch Radar"),
      tr("Enable the stock Bosch radar."),
      param="NAPRadarEnabled",
    )
    self.radar_nosecone = toggle_item_sp(
      tr("Radar Behind Nosecone"),
      tr("Attenuate a radar mounted behind the nosecone."),
      param="NAPRadarBehindNosecone",
    )
    self.radar_offset = option_item_sp(
      title=lambda: tr("Radar Lateral Offset"),
      param="NAPRadarOffset",
      min_value=RADAR_OFFSET_MIN_CM,
      max_value=RADAR_OFFSET_MAX_CM,
      value_change_step=RADAR_OFFSET_STEP_CM,
      use_float_scaling=True,
      label_width=style.BUTTON_ACTION_WIDTH,
      description=lambda: tr("Shift detected leads left (-) or right (+)."),
      label_callback=lambda value: f"{value / 100.0:+.2f} m",
    )
    self.status_path = text_item_sp(lambda: tr("Longitudinal Path"), lambda: tr("Stock DI"))
    self.status_pedal = text_item_sp(lambda: tr("Pedal Health"), lambda: tr("None"))
    self.status_radar = text_item_sp(lambda: tr("Radar Health"), lambda: tr("None"))

    self.tool_calibrate_pedal = button_item_sp(lambda: tr("Calibrate Pedal"), lambda: tr("CALIBRATE"),
                                               callback=lambda: self._confirm_tool("calibrate_pedal"))
    self.tool_calibrate_radar = button_item_sp(lambda: tr("Calibrate Radar"), lambda: tr("CALIBRATE"),
                                               callback=lambda: self._confirm_tool("calibrate_radar"))
    self.tool_diagnose_radar = button_item_sp(lambda: tr("Diagnose Radar"), lambda: tr("DIAGNOSE"),
                                              callback=lambda: self._confirm_tool("diagnose_radar"))
    self.tool_test_radar = button_item_sp(lambda: tr("Test Radar"), lambda: tr("TEST"),
                                          callback=lambda: self._confirm_tool("test_radar"))
    self.tool_extract_epas = button_item_sp(lambda: tr("Backup EPAS"), lambda: tr("BACKUP"),
                                            callback=lambda: self._confirm_tool("extract_epas"))
    self.tool_flash_epas = button_item_sp(lambda: tr("Flash EPAS"), lambda: tr("FLASH"),
                                          callback=lambda: self._confirm_tool("flash_epas"))
    self.tool_restore_epas = button_item_sp(lambda: tr("Restore EPAS"), lambda: tr("RESTORE"),
                                            callback=lambda: self._confirm_tool("restore_epas"))

    self._preap_items = [
      self.follow_distance,
      self.status_path,
      self.status_pedal,
      self.status_radar,
      self.pedal_enabled,
      self.pedal_bus,
      self.radar_enabled,
      self.radar_nosecone,
      self.radar_offset,
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
  def _on_pedal_bus_selected(index):
    ui_state.params.put("NAPPedalCanBus", 0 if index == 0 else 2)

  def _sync_follow_distance(self):
    try:
      follow = int(ui_state.params.get("NAPFollowDistance") or FOLLOW_DISTANCE_MIN + 3)
    except (TypeError, ValueError):
      follow = 4
    follow = max(FOLLOW_DISTANCE_MIN, min(FOLLOW_DISTANCE_MAX, follow))
    self.follow_distance.action_item.current_value = follow

  def _sync_radar_offset(self):
    raw = ui_state.params.get("NAPRadarOffset")
    try:
      cm = int(round(float(raw) * 100.0))
    except (TypeError, ValueError):
      cm = 0
    cm = max(RADAR_OFFSET_MIN_CM, min(RADAR_OFFSET_MAX_CM, cm))
    self.radar_offset.action_item.current_value = cm

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

    if pedal_present and ui_state.CP is not None and bool(ui_state.CP.openpilotLongitudinalControl) and not bool(ui_state.CP.pcmCruise):
      path = "pedal"
    else:
      path = "stock_di"
    self.status_path.action_item.set_text(_path_label(path))

    if not pedal_present:
      pedal_health = "none"
    elif not pedal_calib:
      pedal_health = "uncalibrated"
    else:
      pedal_health = "ok"
    self.status_pedal.action_item.set_text(_health_label(pedal_health))

    if not radar_present:
      radar_health = "none"
    elif ui_state.CP is not None and bool(ui_state.CP.radarUnavailable):
      radar_health = "unconfigured"
    else:
      radar_health = "ok"
    self.status_radar.action_item.set_text(_health_label(radar_health))

    self._sync_follow_distance()
    self._sync_radar_offset()
    self.pedal_bus.action_item.set_selected_button(
      pedal_bus_selector_index(ui_state.params.get("NAPPedalCanBus")))

    self.pedal_bus.set_visible(is_preap and pedal_present)
    self.radar_nosecone.set_visible(is_preap and radar_present)
    self.radar_offset.set_visible(is_preap and radar_present)
    self.tool_calibrate_pedal.set_visible(is_preap and pedal_present)
    self.tool_calibrate_radar.set_visible(is_preap and radar_present)
    self.tool_diagnose_radar.set_visible(is_preap and radar_present)
    self.tool_test_radar.set_visible(is_preap and radar_present)

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
      hardware_offroad_msg = tr("Hardware settings are only available offroad.")
      pedal_desc = tr("Enable the Comma Pedal interceptor.")
      pedal_bus_desc = tr("CAN bus for the Comma Pedal.")
      radar_desc = tr("Enable the stock Bosch radar.")
      nosecone_desc = tr("Attenuate a radar mounted behind the nosecone.")
      offset_desc = tr("Shift detected leads left (-) or right (+).")
      if not offroad:
        pedal_desc = f"<b>{hardware_offroad_msg}</b><br><br>{pedal_desc}"
        pedal_bus_desc = f"<b>{hardware_offroad_msg}</b><br><br>{pedal_bus_desc}"
        radar_desc = f"<b>{hardware_offroad_msg}</b><br><br>{radar_desc}"
        nosecone_desc = f"<b>{hardware_offroad_msg}</b><br><br>{nosecone_desc}"
        offset_desc = f"<b>{hardware_offroad_msg}</b><br><br>{offset_desc}"
      self.pedal_enabled.set_description(pedal_desc)
      self.pedal_bus.set_description(pedal_bus_desc)
      self.radar_enabled.set_description(radar_desc)
      self.radar_nosecone.set_description(nosecone_desc)
      self.radar_offset.set_description(offset_desc)
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
