"""NAP settings panel for the comma 4 / mici UI tree.

Mirrors selfdrive/ui/layouts/settings/nap.py for the BigButton/NavScroller
vocabulary used on the comma 4. Phase 1 stub: three controls (pedal
interceptor toggle, radar enabled toggle, flash EPAS button).
Subsequent phases add the rest.
"""
from opendbc.car.tesla.preap.nap_params import NAPParamKeys
from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.mici.widgets.big_multi_value_param import BigMultiValueParamToggle
from openpilot.selfdrive.ui.mici.widgets.button import BigButton, BigParamControl
from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationDialog, BigDialog, BigInputDialog
from openpilot.selfdrive.ui.mici.layouts.settings.nap_script import launch_script, open_pedal_wizard
from openpilot.selfdrive.ui.layouts.settings.nap_content import (
  BACKUP_EPAS_INSTRUCTIONS,
  FLASH_EPAS_INSTRUCTIONS,
  PEDAL_CAN_BUS_VALUES,
  RADAR_OFFSET_MAX,
  RADAR_OFFSET_MIN,
  RESTORE_EPAS_INSTRUCTIONS,
  pedal_calibration_entry_enabled,
)

from openpilot.selfdrive.ui.radar.radar_view import RadarMonitorDialog
from openpilot.selfdrive.ui.ui_state import ui_state


def _reboot_dialog() -> None:
  def confirm():
    Params().put_bool("DoReboot", True)
  icon = gui_app.texture("icons_mici/settings/device/reboot.png", 64, 70)
  gui_app.push_widget(BigConfirmationDialog("slide to\nreboot now", icon, confirm))


def _reboot_on_toggle(_):
  _reboot_dialog()


def _hands_on_pause_available() -> bool:
  return ui_state.CP_SP is not None and bool(getattr(ui_state.CP_SP, "madsHandsOnPauseAvailable", False))


def _hands_on_pause_enabled() -> bool:
  return ui_state.is_offroad() and _hands_on_pause_available()


def _hands_on_level_enabled() -> bool:
  return _hands_on_pause_enabled() and ui_state.params.get_bool("TeslaPreapHandsOnPause")


def _confirm_then_flash(slider_title: str, runner_title: str, instructions: str, module: str):
  """Slide-to-confirm dialog before launching a destructive EPAS script.

  Adds a deliberate gesture before an irreversible firmware operation.
  Read-only operations (extract, calibration, test) skip this.
  """
  def confirm():
    launch_script(runner_title, instructions, module)
  icon = gui_app.texture("icons_mici/buttons/button_circle_red.png", 180, 180)
  gui_app.push_widget(BigConfirmationDialog(slider_title, icon, confirm, red=True))


class RadarSettingsLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()
    self._params = Params()

    radar_enabled = BigParamControl("radar enabled", NAPParamKeys.RADAR_ENABLED,
                                    toggle_callback=_reboot_on_toggle)
    radar_enabled.set_enabled(ui_state.is_offroad)

    ignore_hw_fail = BigParamControl("ignore radar hardware fail", NAPParamKeys.RADAR_IGNORE_HW_FAIL)

    def on_radar_hud(checked):
      ui_state.radar_hud = checked

    radar_hud = BigParamControl("radar HUD", NAPParamKeys.RADAR_HUD, toggle_callback=on_radar_hud)

    radar_offset_btn = BigButton("radar lateral offset", self._radar_offset_label())
    radar_offset_btn.set_click_callback(lambda: self._open_radar_offset_input(radar_offset_btn))

    donor_vin_btn = BigButton("donor radar vin", self._radar_vin_label())
    donor_vin_btn.set_click_callback(lambda: self._open_radar_vin_input(donor_vin_btn))

    read_vin_btn = BigButton("read vin", self._read_vin_label())
    read_vin_btn.set_click_callback(lambda: self._on_read_vin(read_vin_btn))

    radar_position = BigMultiValueParamToggle(
      "donor radar position",
      NAPParamKeys.RADAR_POSITION,
      values=[0, 1, 2],
      labels=["0", "1", "2"],
      default_value=0,
    )

    radar_epas = BigMultiValueParamToggle(
      "donor epas type",
      NAPParamKeys.RADAR_EPAS_TYPE,
      values=[0, 1, 2, 3, 4],
      labels=["0", "1", "2", "3", "4"],
      default_value=0,
    )

    live_radar_btn = BigButton("live radar", "open")
    live_radar_btn.set_click_callback(lambda: gui_app.push_widget(RadarMonitorDialog()))
    diagnose_radar_btn = BigButton("diagnose radar", "diagnose")
    diagnose_radar_btn.set_click_callback(
      lambda: launch_script("Diagnose Radar", "", "diagnose_radar"))
    diagnose_radar_btn.set_enabled(ui_state.is_offroad)
    test_radar_btn = BigButton("test radar", "test")
    test_radar_btn.set_click_callback(
      lambda: launch_script("Test Radar", "", "test_radar"))
    test_radar_btn.set_enabled(ui_state.is_offroad)

    self._scroller.add_widgets([
      radar_enabled,
      ignore_hw_fail,
      radar_hud,
      radar_offset_btn,
      donor_vin_btn,
      read_vin_btn,
      radar_position,
      radar_epas,
      live_radar_btn,
      diagnose_radar_btn,
      test_radar_btn,
    ])

  def _radar_offset_label(self) -> str:
    raw = self._params.get(NAPParamKeys.RADAR_OFFSET, return_default=True)
    try:
      return f"{float(raw or 0):+.2f}m"
    except (TypeError, ValueError):
      return "+0.00m"

  def _radar_vin_label(self) -> str:
    raw = self._params.get(NAPParamKeys.RADAR_DONOR_VIN, return_default=True) or ""
    if isinstance(raw, bytes):
      raw = raw.decode("ascii", errors="ignore")
    vin = "".join(ch for ch in str(raw).upper() if ch.isalnum())
    return vin if len(vin) == 17 else "not set"

  def _read_vin_label(self) -> str:
    return "reading..." if self._params.get_bool(NAPParamKeys.RADAR_READ_VIN) else "read"

  def _open_radar_offset_input(self, btn: BigButton) -> None:
    raw = self._params.get(NAPParamKeys.RADAR_OFFSET, return_default=True)
    try:
      default_text = f"{float(raw or 0):.2f}"
    except (TypeError, ValueError):
      default_text = "0.00"

    def on_confirm(text: str) -> None:
      try:
        v = float(text)
      except (TypeError, ValueError):
        return
      v = max(RADAR_OFFSET_MIN, min(RADAR_OFFSET_MAX, v))
      try:
        self._params.put(NAPParamKeys.RADAR_OFFSET, v)
      except Exception:
        return
      btn.set_value(self._radar_offset_label())

    gui_app.push_widget(BigInputDialog(
      f"radar offset (m, {RADAR_OFFSET_MIN} to {RADAR_OFFSET_MAX})",
      default_text=default_text,
      confirm_callback=on_confirm,
    ))

  def _open_radar_vin_input(self, btn: BigButton) -> None:
    current = self._radar_vin_label()

    def on_confirm(text: str) -> None:
      vin = "".join(ch for ch in (text or "").upper() if ch.isalnum())
      try:
        self._params.put(NAPParamKeys.RADAR_DONOR_VIN, vin if len(vin) == 17 else "")
      except Exception:
        return
      btn.set_value(self._radar_vin_label())

    gui_app.push_widget(BigInputDialog(
      "donor radar vin",
      default_text="" if current == "not set" else current,
      minimum_length=0,
      confirm_callback=on_confirm,
    ))

  def _on_read_vin(self, btn: BigButton) -> None:
    if ui_state.engaged:
      gui_app.push_widget(BigDialog("disengage first", "VIN read is blocked while engaged."))
      return

    def confirm():
      self._params.put_bool(NAPParamKeys.RADAR_READ_VIN, True)
      btn.set_value(self._read_vin_label())

    icon = gui_app.texture("icons_mici/buttons/button_circle_red.png", 180, 180)
    gui_app.push_widget(BigConfirmationDialog("slide to\nread vin", icon, confirm))


class NAPLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._params.put_bool(NAPParamKeys.FORCE_PRE_AP, True)

    # set_enabled accepts bool | Callable[[], bool]. Pass methods directly
    # (e.g. ui_state.is_offroad), NOT lambdas-wrapping-methods. The form
    # `lambda: ui_state.is_offroad` returns the bound method object —
    # always truthy — and silently leaves destructive actions clickable
    # while onroad.

    # ── Longitudinal control ─────────────────────────
    pedal_enabled = BigParamControl("pedal interceptor", NAPParamKeys.PEDAL_ENABLED,
                                     toggle_callback=_reboot_on_toggle)
    pedal_enabled.set_enabled(ui_state.is_offroad)

    adaptive_accel = BigParamControl("adaptive accel limits", NAPParamKeys.ADAPTIVE_ACCEL)

    # Next drive: frozen into safetyParam at interface init, like BIG TeslaSettings.
    hands_on_pause = BigParamControl("hands-on pause", "TeslaPreapHandsOnPause")
    hands_on_pause.set_visible(_hands_on_pause_available)
    hands_on_pause.set_enabled(_hands_on_pause_enabled)
    hands_on_level = BigMultiValueParamToggle(
      "hands-on level",
      "TeslaPreapHandsOnLevel",
      values=[1, 2, 3],
      labels=["1", "2", "3"],
      default_value=2,
    )
    hands_on_level.set_visible(_hands_on_pause_available)
    hands_on_level.set_enabled(_hands_on_level_enabled)
    # ── Pedal hardware ───────────────────────────────
    # default_value=2 matches NAPPedalCanBus declared default in params_keys.h
    # and the runtime fallback ("any nonzero is bus 2"). If the param is set
    # to an out-of-range value (1, etc.), the widget rewrites it to 2 rather
    # than render bus 0 while the runtime acts as bus 2.
    pedal_can_bus = BigMultiValueParamToggle(
      "pedal can bus",
      NAPParamKeys.PEDAL_CAN_BUS,
      values=PEDAL_CAN_BUS_VALUES,
      labels=["bus 0", "bus 2"],
      default_value=2,
      toggle_callback=_reboot_on_toggle,
    )
    pedal_can_bus.set_enabled(ui_state.is_offroad)


    pedal_calib_status = BigButton(
      "pedal calibration",
      "calibrated" if self._params.get_bool(NAPParamKeys.PEDAL_CALIB_DONE) else "not calibrated",
    )

    # Pedal calibration is hosted in this UI. Ignition-on stationary/waiting
    # may open the wizard. Other destructive tools stay offroad-gated.
    calibrate_pedal_btn = BigButton("calibrate pedal", "start")
    calibrate_pedal_btn.set_click_callback(open_pedal_wizard)
    calibrate_pedal_btn.set_enabled(pedal_calibration_entry_enabled)


    radar_settings_btn = BigButton("radar settings", "open")
    radar_settings_btn.set_click_callback(lambda: gui_app.push_widget(RadarSettingsLayoutMici()))

    # ── iBooster (locked off) ────────────────────────
    ibooster_enabled = BigParamControl("ibooster enabled", NAPParamKeys.IBOOSTER_ENABLED)
    ibooster_enabled.set_enabled(False)

    # ── Advanced (locked on) ─────────────────────────
    force_pre_ap = BigParamControl("force pre-ap mode", NAPParamKeys.FORCE_PRE_AP)
    force_pre_ap.set_enabled(False)

    # ── Actions ──────────────────────────────────────
    backup_epas_btn = BigButton("backup epas", "extract")
    backup_epas_btn.set_click_callback(
      lambda: launch_script("Backup EPAS Firmware", BACKUP_EPAS_INSTRUCTIONS,
                            "scripts.nap.extract_epas",
                            ))
    backup_epas_btn.set_enabled(ui_state.is_offroad)

    flash_epas_btn = BigButton("flash epas", "flash")
    flash_epas_btn.set_click_callback(
      lambda: _confirm_then_flash(
        "slide to\nflash epas",
        "Flash EPAS Firmware", FLASH_EPAS_INSTRUCTIONS, "scripts.nap.flash_epas",
      ))
    flash_epas_btn.set_enabled(ui_state.is_offroad)

    restore_epas_btn = BigButton("restore epas", "restore")
    restore_epas_btn.set_click_callback(
      lambda: _confirm_then_flash(
        "slide to\nrestore epas",
        "Restore EPAS Firmware", RESTORE_EPAS_INSTRUCTIONS, "scripts.nap.restore_epas",
      ))
    restore_epas_btn.set_enabled(ui_state.is_offroad)

    self._scroller.add_widgets([
      pedal_enabled,
      adaptive_accel,
      hands_on_pause,
      hands_on_level,
      pedal_can_bus,
      pedal_calib_status,
      calibrate_pedal_btn,
      radar_settings_btn,
      ibooster_enabled,
      force_pre_ap,
      backup_epas_btn,
      flash_epas_btn,
      restore_epas_btn,
    ])

