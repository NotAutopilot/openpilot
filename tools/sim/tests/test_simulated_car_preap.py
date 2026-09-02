#!/usr/bin/env python3
"""Pack tesla_preap SimulatedCar CAN and parse it through the Pre-AP interface."""
import time

from opendbc.car import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.structs import CarState
from opendbc.car.tesla.values import CruiseButtons
from openpilot.tools.sim.lib.common import SimulatorState, vec3
from openpilot.tools.sim.lib.plant_world import CRUISE_SPEED_MS, PlantWorld
from openpilot.tools.sim.lib.preap_params import PREAP_FINGERPRINT, PREAP_VIN, configure_preap_sim
from openpilot.tools.sim.lib.simulated_car import GAS_SENSOR_ADDR, PEDAL_REST_RAW, SimulatedCar


def _state(**overrides):
  state = SimulatorState()
  state.valid = True
  state.ignition = True
  state.velocity = vec3(20.0, 0.0, 0.0)
  state.steering_angle = 5.0
  state.cruise_button = CruiseButtons.IDLE
  for key, value in overrides.items():
    setattr(state, key, value)
  return state


def _addrs(msgs):
  return {addr for addr, _dat, _bus in msgs}


def _by_addr(msgs, addr):
  return [m for m in msgs if m[0] == addr]


class TestSimulatedCarPreap:
  def setup_method(self):
    configure_preap_sim()
    self.car = SimulatedCar()

  def test_identity_and_params(self):
    assert self.car.vin == PREAP_VIN
    assert PREAP_FINGERPRINT == "TESLA_MODEL_S_PREAP"
    assert self.car.params.get_bool("DisengageOnAccelerator") is False
    assert self.car.params.get_bool("NAPPedalEnabled") is True
    assert self.car.params.get_bool("NAPRadarEnabled") is True

  def test_packs_preap_rx_and_pedal(self):
    msgs = self.car.build_can_messages(_state())
    addrs = _addrs(msgs)
    for addr in (0x155, 0x108, 0x118, 0x20A, 0x368, 0x318, 0x370, 0x45, 0xE):
      assert addr in addrs, hex(addr)
    assert GAS_SENSOR_ADDR in addrs
    rest = _by_addr(msgs, GAS_SENSOR_ADDR)[0]
    raw = (rest[1][0] << 8) | rest[1][1]
    assert raw == PEDAL_REST_RAW
    assert rest[2] == 2

  def test_stalk_uses_tesla_not_honda_values(self):
    honda_main = 1
    tesla_main = CruiseButtons.MAIN
    assert tesla_main == 2
    assert tesla_main != honda_main

    idle = _by_addr(self.car.build_can_messages(_state()), 0x45)[0]
    assert (idle[1][0] & 0x3F) == CruiseButtons.IDLE

    pulled = _by_addr(self.car.build_can_messages(_state(cruise_button=CruiseButtons.MAIN)), 0x45)[0]
    assert (pulled[1][0] & 0x3F) == CruiseButtons.MAIN

    resumed = _by_addr(self.car.build_can_messages(_state(cruise_button=CruiseButtons.RES_ACCEL)), 0x45)[0]
    assert (resumed[1][0] & 0x3F) == CruiseButtons.RES_ACCEL

    canceled = _by_addr(self.car.build_can_messages(_state(cruise_button=CruiseButtons.CANCEL)), 0x45)[0]
    assert (canceled[1][0] & 0x3F) == CruiseButtons.CANCEL

  def test_gas_press_raises_pedal_raw_above_panda_threshold(self):
    pressed = _by_addr(self.car.build_can_messages(_state(user_gas=1.0)), GAS_SENSOR_ADDR)[0]
    raw = (pressed[1][0] << 8) | pressed[1][1]
    assert raw > 650

  def test_healthy_bosch_sgu_and_empty_tracks(self):
    msgs = self.car.build_can_messages(_state())
    sgu = _by_addr(msgs, 0x301)
    assert sgu, "TeslaRadarSguInfo missing"
    assert sgu[0][2] == 1
    point_a = _by_addr(msgs, 0x310)
    assert point_a
    trigger = _by_addr(msgs, 0x36E)
    assert trigger, "RadarPoint31_B trigger missing"

  def test_radar_points_fill_slot_zero(self):
    state = _state(radar_points=[(40.0, 0.5, -2.0)])
    msgs = self.car.build_can_messages(state)
    assert _by_addr(msgs, 0x310)

  def test_carstate_update_drive_closed_doors(self):
    CarInterface = interfaces[PREAP_FINGERPRINT]
    CP = CarInterface.get_params(
      PREAP_FINGERPRINT, {i: {} for i in range(8)}, [],
      alpha_long=False, is_release=False, docs=False,
    )
    CI = CarInterface(CP)
    msgs = self.car.build_can_messages(_state(steering_angle=7.0))
    packets = [(1, [CanData(addr, dat, bus) for addr, dat, bus in msgs])]
    CS = CI.update(packets)
    assert CS.doorOpen is False
    assert CS.gearShifter == CarState.GearShifter.drive
    assert CS.seatbeltUnlatched is False
    assert abs(CS.steeringAngleDeg - 7.0) < 0.2
    assert CS.gasPressed is False
    assert CS.enableLongControl is False

  def test_plant_world_makes_can_valid(self):
    world = PlantWorld()
    state = SimulatorState()
    world.read_sensors(state)
    assert state.valid
    assert abs(state.speed - CRUISE_SPEED_MS) < 1e-6
    msgs = self.car.build_can_messages(state)
    assert msgs
    assert 0x155 in {addr for addr, _dat, _bus in msgs}

  def test_create_bridge_plant_skips_metadrive(self):
    from openpilot.tools.sim.run_bridge import create_bridge
    queue, proc, bridge = create_bridge(False, False, plant=True)
    try:
      assert type(bridge).__name__ == "PlantBridge"
      deadline = time.monotonic() + 10
      while not bridge.started.value and time.monotonic() < deadline:
        time.sleep(0.05)
      assert bridge.started.value
    finally:
      bridge.shutdown()
      proc.join(timeout=5)
      if proc.is_alive():
        proc.terminate()
      queue.close()
