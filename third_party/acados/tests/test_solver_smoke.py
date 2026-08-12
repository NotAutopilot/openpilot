from __future__ import annotations

import numpy as np
import pytest

from cereal import messaging
from openpilot.selfdrive.controls.lib.drive_helpers import CAR_ROTATION_RADIUS
from openpilot.selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import LateralMpc, N as LAT_MPC_N
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc


def test_lateral_mpc_solves_zero_reference() -> None:
  mpc = LateralMpc()
  mpc.set_weights(1.0, 0.1, 0.0, 0.05, 800.0)
  mpc.run(
    np.zeros(4),
    np.column_stack((np.full(LAT_MPC_N + 1, 30.0),
                     np.full(LAT_MPC_N + 1, CAR_ROTATION_RADIUS))),
    np.zeros(LAT_MPC_N + 1),
    np.zeros(LAT_MPC_N + 1),
    np.zeros(LAT_MPC_N + 1),
  )
  assert mpc.solution_status == 0


def test_longitudinal_mpc_solves_zero_reference() -> None:
  mpc = LongitudinalMpc()

  def fail_reset() -> None:
    pytest.fail("LongitudinalMpc.reset() was invoked during update")

  mpc.reset = fail_reset
  radar_state = messaging.new_message("radarState").radarState
  mpc.update(radar_state, 0.0, 1.0)
  assert mpc.solution_status == 0
