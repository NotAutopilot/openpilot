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
from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationDialog
from openpilot.selfdrive.ui.mici.layouts.settings.nap_script import (
  SAFETY_OFFROAD_ONLY, SAFETY_STATIONARY, launch_script,
)
from openpilot.selfdrive.ui.layouts.settings.nap_content import (
  BACKUP_EPAS_INSTRUCTIONS,
  CALIBRATE_PEDAL_INSTRUCTIONS,
  CALIBRATE_RADAR_INSTRUCTIONS,
  FLASH_EPAS_INSTRUCTIONS,
  PEDAL_CAN_BUS_VALUES,
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

    # Stationary scripts (pedal/radar calibration, radar test) need
    # ignition on so the ECU is electrically active. Don't offroad-gate
    # them — the script's own preconditions (car_on, brake pressed,
    # gear in neutral, etc.) are the load-bearing check.
    calibrate_pedal_btn = BigButton("calibrate pedal", "start")
    calibrate_pedal_btn.set_click_callback(
      lambda: launch_script("Pedal Calibration", CALIBRATE_PEDAL_INSTRUCTIONS,
                            "scripts.nap.calibrate_pedal",
                            safety_class=SAFETY_STATIONARY))

    # ── Radar ────────────────────────────────────────
    radar_enabled = BigParamControl("radar enabled", NAPParamKeys.RADAR_ENABLED,
                                    toggle_callback=_reboot_on_toggle)
    radar_enabled.set_enabled(ui_state.is_offroad)

    radar_behind_nosecone = BigParamControl(
      "radar behind nosecone", NAPParamKeys.RADAR_BEHIND_NOSECONE,
      toggle_callback=_reboot_on_toggle,
    )
    radar_behind_nosecone.set_enabled(ui_state.is_offroad)

    calibrate_radar_btn = BigButton("calibrate radar", "start")
    calibrate_radar_btn.set_click_callback(
      lambda: launch_script("Radar Calibration", CALIBRATE_RADAR_INSTRUCTIONS,
                            "scripts.nap.calibrate_radar",
                            safety_class=SAFETY_STATIONARY))

    test_radar_btn = BigButton("test radar", "test")
    test_radar_btn.set_click_callback(
      lambda: launch_script("Radar Test", TEST_RADAR_INSTRUCTIONS,
                            "scripts.nap.test_radar",
                            safety_class=SAFETY_STATIONARY))

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
                            safety_class=SAFETY_OFFROAD_ONLY))
    backup_epas_btn.set_enabled(ui_state.is_offroad)

    flash_epas_btn = BigButton("flash epas", "flash")
    flash_epas_btn.set_click_callback(
      lambda: launch_script("Flash EPAS Firmware", FLASH_EPAS_INSTRUCTIONS,
                            "scripts.nap.flash_epas",
                            safety_class=SAFETY_OFFROAD_ONLY))
    flash_epas_btn.set_enabled(ui_state.is_offroad)

    restore_epas_btn = BigButton("restore epas", "restore")
    restore_epas_btn.set_click_callback(
      lambda: launch_script("Restore EPAS Firmware", RESTORE_EPAS_INSTRUCTIONS,
                            "scripts.nap.restore_epas",
                            safety_class=SAFETY_OFFROAD_ONLY))
    restore_epas_btn.set_enabled(ui_state.is_offroad)

    self._scroller.add_widgets([
      pedal_enabled,
      adaptive_accel,
      pedal_can_bus,
      pedal_calib_status,
      calibrate_pedal_btn,
      radar_enabled,
      radar_behind_nosecone,
      calibrate_radar_btn,
      test_radar_btn,
      ibooster_enabled,
      force_pre_ap,
      backup_epas_btn,
      flash_epas_btn,
      restore_epas_btn,
    ])
