"""NAP settings panel for the comma 4 / mici UI tree.

Mirrors selfdrive/ui/layouts/settings/nap.py for the BigButton/NavScroller
vocabulary used on the comma 4. Phase 1 stub: three controls (pedal
interceptor toggle, radar enabled toggle, flash EPAS button).
Subsequent phases add the rest.
"""
from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.mici.widgets.big_multi_value_param import BigMultiValueParamToggle
from openpilot.selfdrive.ui.mici.widgets.button import BigButton, BigParamControl
from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationDialog, BigInputDialog
from openpilot.selfdrive.ui.mici.layouts.settings.nap_script import launch_script
from openpilot.selfdrive.ui.layouts.settings.nap_content import (
  BACKUP_EPAS_INSTRUCTIONS,
  CALIBRATE_PEDAL_INSTRUCTIONS,
  CALIBRATE_RADAR_INSTRUCTIONS,
  FLASH_EPAS_INSTRUCTIONS,
  PEDAL_CAN_BUS_VALUES,
  RADAR_OFFSET_MAX,
  RADAR_OFFSET_MIN,
  RESTORE_EPAS_INSTRUCTIONS,
  TEST_RADAR_INSTRUCTIONS,
)
from openpilot.selfdrive.ui.ui_state import ui_state
from opendbc.car.tesla.preap.nap_params import NAPParamKeys


def _reboot_dialog() -> None:
  def confirm():
    Params().put_bool("DoReboot", True)
  icon = gui_app.texture("icons_mici/settings/device/reboot.png", 64, 70)
  gui_app.push_widget(BigConfirmationDialog("slide to\nreboot now", icon, confirm))


def _reboot_on_toggle(_):
  _reboot_dialog()


def _confirm_then_flash(slider_title: str, runner_title: str, instructions: str, module: str):
  """Slide-to-confirm dialog before launching a destructive EPAS script.

  Adds a deliberate gesture before an irreversible firmware operation.
  Read-only operations (extract, calibration, test) skip this.
  """
  def confirm():
    launch_script(runner_title, instructions, module)
  icon = gui_app.texture("icons_mici/buttons/button_circle_red.png", 180, 180)
  gui_app.push_widget(BigConfirmationDialog(slider_title, icon, confirm, red=True))


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

    # Offroad-gate the menu tap on all action buttons. Stationary scripts
    # still need ignition on at Start press, but the user's path is
    # "tap offroad → open runner → turn car on → press Start." Gating
    # the menu tap on offroad just means the user can't open the runner
    # while actively driving; the script's own preconditions handle the
    # ignition-on state at Start.
    calibrate_pedal_btn = BigButton("calibrate pedal", "start")
    calibrate_pedal_btn.set_click_callback(
      lambda: launch_script("Pedal Calibration", CALIBRATE_PEDAL_INSTRUCTIONS,
                            "scripts.nap.calibrate_pedal",
                            ))
    calibrate_pedal_btn.set_enabled(ui_state.is_offroad)

    # ── Radar ────────────────────────────────────────
    radar_enabled = BigParamControl("radar enabled", NAPParamKeys.RADAR_ENABLED,
                                    toggle_callback=_reboot_on_toggle)
    radar_enabled.set_enabled(ui_state.is_offroad)

    radar_behind_nosecone = BigParamControl(
      "radar behind nosecone", NAPParamKeys.RADAR_BEHIND_NOSECONE,
      toggle_callback=_reboot_on_toggle,
    )
    radar_behind_nosecone.set_enabled(ui_state.is_offroad)

    radar_offset_btn = BigButton("radar lateral offset", self._radar_offset_label())
    radar_offset_btn.set_click_callback(lambda: self._open_radar_offset_input(radar_offset_btn))

    calibrate_radar_btn = BigButton("calibrate radar", "start")
    calibrate_radar_btn.set_click_callback(
      lambda: launch_script("Radar Calibration", CALIBRATE_RADAR_INSTRUCTIONS,
                            "scripts.nap.calibrate_radar",
                            ))
    calibrate_radar_btn.set_enabled(ui_state.is_offroad)

    test_radar_btn = BigButton("test radar", "test")
    test_radar_btn.set_click_callback(
      lambda: launch_script("Radar Test", TEST_RADAR_INSTRUCTIONS,
                            "scripts.nap.test_radar",
                            ))
    test_radar_btn.set_enabled(ui_state.is_offroad)

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
      pedal_can_bus,
      pedal_calib_status,
      calibrate_pedal_btn,
      radar_enabled,
      radar_behind_nosecone,
      radar_offset_btn,
      calibrate_radar_btn,
      test_radar_btn,
      ibooster_enabled,
      force_pre_ap,
      backup_epas_btn,
      flash_epas_btn,
      restore_epas_btn,
    ])

  def _radar_offset_label(self) -> str:
    raw = self._params.get(NAPParamKeys.RADAR_OFFSET, return_default=True)
    try:
      return f"{float(raw or 0):+.2f}m"
    except (TypeError, ValueError):
      return "+0.00m"

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
        # FLOAT keys raise on bad value type; swallow so the UI
        # doesn't crash on a corrupt write.
        return
      btn.set_value(self._radar_offset_label())

    gui_app.push_widget(BigInputDialog(
      f"radar offset (m, {RADAR_OFFSET_MIN} to {RADAR_OFFSET_MAX})",
      default_text=default_text,
      confirm_callback=on_confirm,
    ))
