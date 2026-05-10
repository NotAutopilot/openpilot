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
from openpilot.selfdrive.ui.mici.layouts.settings.nap_script import launch_script
from openpilot.selfdrive.ui.layouts.settings.nap_content import (
  FLASH_EPAS_INSTRUCTIONS,
  PEDAL_CAN_BUS_VALUES,
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

    pedal_enabled = BigParamControl("pedal interceptor", NAPParamKeys.PEDAL_ENABLED,
                                     toggle_callback=_reboot_on_toggle)
    pedal_enabled.set_enabled(ui_state.is_offroad)

    pedal_can_bus = BigMultiValueParamToggle(
      "pedal can bus",
      NAPParamKeys.PEDAL_CAN_BUS,
      values=PEDAL_CAN_BUS_VALUES,
      labels=["bus 0", "bus 2"],
      toggle_callback=_reboot_on_toggle,
    )
    pedal_can_bus.set_enabled(ui_state.is_offroad)

    radar_enabled = BigParamControl("radar enabled", NAPParamKeys.RADAR_ENABLED,
                                    toggle_callback=_reboot_on_toggle)
    radar_enabled.set_enabled(ui_state.is_offroad)

    flash_epas_btn = BigButton("flash epas", "flash")
    flash_epas_btn.set_click_callback(
      lambda: launch_script("Flash EPAS Firmware", FLASH_EPAS_INSTRUCTIONS,
                            "scripts.nap.flash_epas"))
    flash_epas_btn.set_enabled(ui_state.is_offroad)

    self._scroller.add_widgets([
      pedal_enabled,
      pedal_can_bus,
      radar_enabled,
      flash_epas_btn,
    ])
