"""BigMultiToggle bound to a param that stores the option *value*, not its index.

`BigMultiParamToggle` (in button.py) stores the option index — that's the right
shape for enums where the index *is* the persistence value (e.g. driving
personality). It's the wrong shape for params that persist a domain value:
PEDAL_CAN_BUS is 0 or 2, BRAKE_FACTOR is 0.5/1.0/1.5/2.0. This wrapper maps
index ↔ value on load and store.
"""
from collections.abc import Callable
from typing import Any

from openpilot.common.params import Params
from openpilot.selfdrive.ui.mici.widgets.button import BigMultiToggle


class BigMultiValueParamToggle(BigMultiToggle):
  def __init__(
    self,
    text: str,
    param: str,
    values: list[Any],
    labels: list[str],
    toggle_callback: Callable | None = None,
    select_callback: Callable | None = None,
  ):
    assert len(values) == len(labels) and len(values) > 0
    super().__init__(text, labels, toggle_callback, select_callback)
    self._param = param
    self._values = values
    self._params = Params()
    self._load_value()

  def _load_value(self):
    raw = self._params.get(self._param, return_default=True)
    try:
      idx = self._values.index(raw)
    except ValueError:
      idx = 0
    self.set_value(self._options[idx])

  def _handle_mouse_release(self, mouse_pos):
    super()._handle_mouse_release(mouse_pos)
    new_idx = self._options.index(self.value)
    self._params.put(self._param, self._values[new_idx])
