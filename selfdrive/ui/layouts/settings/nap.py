import os
import subprocess
import pyray as rl
from openpilot.common.params import Params
from openpilot.common.basedir import BASEDIR
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.list_view import (
  toggle_item, multiple_button_item, button_item, text_item,
  ITEM_PADDING,
)
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets.html_render import HtmlRenderer, ElementType
from opendbc.car.tesla.preap.nap_params import NAPParamKeys, DEFAULTS
from openpilot.selfdrive.ui.ui_state import ui_state

# Preset values for float/int params exposed as multiple-button selectors
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
      "Enable Comma Pedal hardware for direct throttle control. Requires reboot.",
      enabled=ui_state.is_offroad,
      needs_reboot=True,
    )

    self._add_toggle(
      NAPParamKeys.PEDAL_PASSTHROUGH,
      "Pedal Passthrough",
      "When gas is pressed during long control, pass driver's pedal input through instead of disabling. The system takes over smoothly as you lift off.",
    )

    follow_dist = self._params.get(NAPParamKeys.FOLLOW_DISTANCE, return_default=True)
    self._follow_buttons = multiple_button_item(
      "Follow Distance",
      "Follow distance (1=closest, 7=farthest). Overridden by cruise stalk if present.",
      buttons=["1", "2", "3", "4", "5", "6", "7"],
      button_width=80,
      selected_index=max(0, min(6, follow_dist - 1)),
      callback=self._on_follow_distance,
    )
    self._all_items.append(self._follow_buttons)

    # ── Section 2: Pedal Hardware ──
    self._all_items.append(section_header_item("Pedal Hardware"))


    pedal_bus = self._params.get(NAPParamKeys.PEDAL_CAN_BUS, return_default=True)
    self._pedal_bus_buttons = multiple_button_item(
      "Pedal CAN Bus",
      "Select which CAN bus the Comma Pedal is connected to. Requires reboot.",
      buttons=["Bus 0", "Bus 2"],
      button_width=150,
      selected_index=0 if pedal_bus == 0 else 1,
      callback=self._on_pedal_can_bus,
    )
    self._pedal_bus_buttons.action_item.set_enabled(ui_state.is_offroad)
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
    self._calibrate_pedal_btn.action_item.set_enabled(ui_state.is_offroad)
    self._all_items.append(self._calibrate_pedal_btn)

    # ── Section 3: Radar ──
    self._all_items.append(section_header_item("Radar"))

    self._add_toggle(
      NAPParamKeys.RADAR_ENABLED,
      "Radar Enabled",
      "Enable the stock Bosch radar for lead car detection. Requires reboot.",
      enabled=ui_state.is_offroad,
      needs_reboot=True,
    )

    self._add_toggle(
      NAPParamKeys.RADAR_BEHIND_NOSECONE,
      "Radar Behind Nosecone",
      "Apply signal attenuation adjustment for radar mounted behind the nosecone. Requires reboot.",
      enabled=ui_state.is_offroad,
      needs_reboot=True,
    )

    self._calibrate_radar_btn = button_item(
      "Calibrate Radar",
      "Start",
      description="Run the radar calibration routine.",
      callback=self._on_calibrate_radar,
    )
    self._calibrate_radar_btn.action_item.set_enabled(ui_state.is_offroad)
    self._all_items.append(self._calibrate_radar_btn)

    self._test_radar_btn = button_item(
      "Test Radar",
      "Test",
      description="Test radar connectivity and verify signals.",
      callback=self._on_test_radar,
    )
    self._test_radar_btn.action_item.set_enabled(ui_state.is_offroad)
    self._all_items.append(self._test_radar_btn)

    # ── Section 4: iBooster / Braking (not yet implemented — grayed out) ──
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

    # ── Section 5: Advanced ──
    self._all_items.append(section_header_item("Advanced"))

    # Force Pre-AP is always on for now — grayed out in the ON position
    self._params.put_bool(NAPParamKeys.FORCE_PRE_AP, True)
    self._add_toggle(
      NAPParamKeys.FORCE_PRE_AP,
      "Force Pre-AP Mode",
      "Force the system to treat this vehicle as a Pre-Autopilot Tesla.",
      enabled=False,
    )

    # ── Section 6: Actions ──
    self._all_items.append(section_header_item("Actions"))

    self._backup_epas_btn = button_item(
      "Backup EPAS",
      "Extract",
      description="Extract and save stock EPAS firmware image without flashing.",
      callback=self._on_backup_epas,
    )
    self._backup_epas_btn.action_item.set_enabled(ui_state.is_offroad)
    self._all_items.append(self._backup_epas_btn)

    self._flash_epas_btn = button_item(
      "Flash EPAS",
      "Flash",
      description="Flash the EPAS (Electric Power Assisted Steering) firmware.",
      callback=self._on_flash_epas,
    )
    self._flash_epas_btn.action_item.set_enabled(ui_state.is_offroad)
    self._all_items.append(self._flash_epas_btn)

    self._restore_epas_btn = button_item(
      "Restore EPAS",
      "Restore",
      description="Restore stock EPAS firmware image.",
      callback=self._on_restore_epas,
    )
    self._restore_epas_btn.action_item.set_enabled(ui_state.is_offroad)
    self._all_items.append(self._restore_epas_btn)

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
    self._reset_defaults_btn.action_item.set_enabled(ui_state.is_offroad)
    self._all_items.append(self._reset_defaults_btn)

    # ── Acknowledgments ──
    self._all_items.append(section_header_item("Acknowledgments"))
    self._all_items.append(CreditsBlock(
      "<p>Special thanks to the following members. This project wouldn't be possible without you:</p>"
      + "<p><b>Boggyver and the Tinkla Project</b><br>"
      + "<b>Lukas Loetkolben</b><br>"
      + "<b>Johnmr1</b><br>"
      + "<b>SeriouslySerious</b><br>"
      + "<b>Pod042</b><br>"
      + "<b>1FrostlySlime</b></p>"
    ))

  def _add_toggle(self, param_key: str, title: str, description: str,
                   enabled: bool | None = None, needs_reboot: bool = False):
    """Helper to add a toggle item and register it for state refresh."""
    kwargs = {}
    if enabled is not None:
      kwargs['enabled'] = enabled

    def on_toggle(state, k=param_key):
      self._params.put_bool(k, state)
      if needs_reboot:
        self._show_reboot_modal()

    item = toggle_item(
      title,
      description=description,
      initial_state=self._params.get_bool(param_key),
      callback=on_toggle,
      **kwargs,
    )
    self._toggle_map[param_key] = item
    self._all_items.append(item)

  # ── Multiple-button callbacks ──

  def _on_follow_distance(self, index: int):
    self._params.put(NAPParamKeys.FOLLOW_DISTANCE, index + 1)

  def _on_pedal_can_bus(self, index: int):
    self._params.put(NAPParamKeys.PEDAL_CAN_BUS, PEDAL_CAN_BUS_VALUES[index])
    self._show_reboot_modal()

  def _on_brake_factor(self, index: int):
    self._params.put(NAPParamKeys.BRAKE_FACTOR, BRAKE_FACTOR_PRESETS[index])

  # ── Script runner ──

  def _show_script_runner(self, title: str, instructions: str, script_module: str):
    """Launch the script runner as a separate process that takes over the screen."""
    script_path = os.path.join(BASEDIR, "scripts", "nap", "run_script.py")
    log_path = "/tmp/nap_script_runner.log"

    # Launch as detached process — run_script.py will kill the UI and take over
    with open(log_path, "w") as log_file:
      subprocess.Popen(
        ["python", script_path, title, script_module, instructions],
        cwd=BASEDIR,
        start_new_session=True,  # Detach from parent process
        stdout=log_file,
        stderr=log_file,
      )

  # ── Action button callbacks ──

  def _on_calibrate_pedal(self):
    self._show_script_runner(
      title="Pedal Calibration",
      instructions="""\
NAP Pedal Calibrator

This script calibrates the comma pedal interceptor for your pre-AP Tesla Model S.

PRECONDITIONS:
  1. Car must be ON
  2. Gear must be in NEUTRAL
  3. Brake pedal must be PRESSED and held
  4. Do NOT press the accelerator pedal during calibration

The calibration process will:
  - Detect pedal zero position
  - Detect pedal maximum position
  - Fine-tune the scale factor
  - Validate the calibration values
  - Save calibration to params

Press START when ready to begin calibration.""",
      script_module="scripts.nap.calibrate_pedal"
    )

  def _on_calibrate_radar(self):
    self._show_script_runner(
      title="Radar Calibration",
      instructions="""\
NAP Radar Calibrator

This script displays filtered radar points to help align the Bosch radar.

PRECONDITIONS:
  1. Vehicle must be safely parked
  2. Radar must be properly mounted and connected
  3. Place a calibration target 3-10m ahead, centered on the vehicle axis

The calibration display will show:
  - Radar points within 2.5-14.5m ahead
  - Lateral offset from center
  - Adjust radar aim until target shows ~0.0m lateral

Press START when ready to begin.""",
      script_module="scripts.nap.calibrate_radar"
    )

  def _on_test_radar(self):
    self._show_script_runner(
      title="Radar Test",
      instructions="""\
NAP Radar Test

This script displays live radar data for testing and verification.

The test will show:
  - All detected radar points
  - Distance and relative velocity
  - Track status and confidence

This is useful for:
  - Verifying radar installation
  - Checking radar alignment
  - Debugging radar issues

Press START to begin the radar test.""",
      script_module="scripts.nap.test_radar"
    )

  def _on_flash_epas(self):
    self._show_script_runner(
      title="Flash EPAS Firmware",
      instructions="""\
EPAS Firmware Flash

WARNING: This will modify your steering system firmware!

This operation:
  - Requires the vehicle to be safely parked
  - May take several minutes to complete
  - Should NOT be interrupted once started

RISKS:
  - Incorrect firmware can disable power steering
  - Interrupted flash can brick the EPAS module
  - This modification may void warranties

Only proceed if you:
  - Fully understand the implications
  - Have backup EPAS firmware available
  - Are comfortable with the risks involved

Press START only if you accept these risks.""",
      script_module="scripts.nap.flash_epas"
    )

  def _on_backup_epas(self):
    self._show_script_runner(
      title="Backup EPAS Firmware",
      instructions="""\
EPAS Firmware Backup

This action only extracts and saves the stock EPAS firmware image.
No flashing or firmware modifications are performed.

Use this before any flash operation so you have a local backup.

PRECONDITIONS:
  - Vehicle safely parked
  - Stable 12V power
  - Do not power-cycle during extraction

Press START to extract the EPAS firmware backup.""",
      script_module="scripts.nap.extract_epas"
    )

  def _on_restore_epas(self):
    self._show_script_runner(
      title="Restore EPAS Firmware",
      instructions="""\
EPAS Firmware Restore

WARNING: This will reflash your steering system firmware!

This operation:
  - Uses the extracted stock EPAS firmware image
  - May take several minutes to complete
  - Should NOT be interrupted once started

RISKS:
  - Interrupted flash can brick the EPAS module
  - Incorrect image can disable power steering

Only proceed if you:
  - Need to return to stock EPAS firmware
  - Understand and accept the risks

Press START only if you accept these risks.""",
      script_module="scripts.nap.restore_epas"
    )

  def _show_reboot_modal(self):
    """Show a modal prompting the user to reboot for the change to take effect."""
    def confirm_callback(result: int):
      if result == DialogResult.CONFIRM:
        self._params.put_bool("DoReboot", True)

    content = "<h1>Reboot Required</h1><br><p>This change requires a reboot to take effect.</p>"
    dlg = ConfirmDialog(content, "Reboot", cancel_text="Ignore", rich=True, callback=confirm_callback)
    gui_app.push_widget(dlg)

  def _on_emergency_disable(self):
    def confirm_callback(result: int):
      if result == DialogResult.CONFIRM:
        self._params.put_bool(NAPParamKeys.PEDAL_ENABLED, False)
        self._params.put_bool(NAPParamKeys.PEDAL_CALIB_DONE, False)
        self._refresh_toggles()

    content = (
      "<h1>Emergency Disable</h1><br>"
      + "<p>This will disable the pedal interceptor and clear calibration. "
      + "You will need to restart the device for changes to take effect.</p>"
    )
    dlg = ConfirmDialog(content, "Disable", rich=True, callback=confirm_callback)
    gui_app.push_widget(dlg)

  def _on_reset_defaults(self):
    def confirm_callback(result: int):
      if result == DialogResult.CONFIRM:
        self._reset_all_to_defaults()
        self._refresh_toggles()

    content = (
      "<h1>Reset to Defaults</h1><br>"
      + "<p>This will reset all NAP settings to their factory default values. "
      + "This action cannot be undone.</p>"
    )
    dlg = ConfirmDialog(content, "Reset All", rich=True, callback=confirm_callback)
    gui_app.push_widget(dlg)

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
      max(0, min(6, follow_dist - 1)))

    pedal_bus = self._params.get(NAPParamKeys.PEDAL_CAN_BUS, return_default=True)
    self._pedal_bus_buttons.action_item.set_selected_button(
      0 if pedal_bus == 0 else 1)

    brake_factor = self._params.get(NAPParamKeys.BRAKE_FACTOR, return_default=True)
    self._brake_factor_buttons.action_item.set_selected_button(
      _find_preset_index(BRAKE_FACTOR_PRESETS, brake_factor))
