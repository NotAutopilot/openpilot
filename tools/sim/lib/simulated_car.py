import traceback

import cereal.messaging as messaging
from opendbc.can.packer import CANPacker
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.preap.interface import PREAP_FLAG_ENABLE_PEDAL, PREAP_FLAG_RADAR_EMULATION
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
from opendbc.car.tesla.values import CANBUS, CruiseButtons
from openpilot.common.params import Params
from openpilot.selfdrive.pandad.pandad_api_impl import can_list_to_can_capnp
from openpilot.tools.sim.lib.common import SimulatorState
from openpilot.tools.sim.lib.preap_params import PREAP_VIN

GAS_SENSOR_ADDR = 0x552
PEDAL_REST_RAW = 0x01D6  # 470; below panda gas_pressed threshold 650
PEDAL_PRESSED_RAW = 0x0320  # 800; typical real press
BOSCH_POINT_COUNT = 32
DI_GEAR_D = 4
DI_CRUISE_OFF = 0
DI_SYSTEM_ENABLE = 4
EPAS_EAC_INHIBITED = 0
EPAS_EAC_ACTIVE = 2


class SimulatedCar:
  """Simulates a Pre-AP Model S (panda state + tesla_preap CAN) to openpilot."""
  packer = CANPacker("tesla_preap")
  radar_packer = CANPacker("tesla_radar_bosch_generated")

  def __init__(self):
    self.pm = messaging.PubMaster(['can', 'pandaStates'])
    self.sm = messaging.SubMaster(['carControl', 'controlsState', 'carParams', 'selfdriveState'])
    self.idx = 0
    self.params = Params()
    self.obd_multiplexing = False
    self.tesla_can = TeslaCANPreAP({CANBUS.party: self.packer})
    self.pedal_idx = 0
    self.vin = PREAP_VIN
    # Read Params here, not nap_conf: the nap_conf singleton snapshots Params
    # at import, which is before configure_preap_sim() in tests and launch.
    self.use_pedal = self.params.get_bool("NAPPedalEnabled")
    self.radar_enabled = self.params.get_bool("NAPRadarEnabled")
    pedal_bus = self.params.get("NAPPedalCanBus", return_default=True)
    self.pedal_bus = int(pedal_bus) if pedal_bus is not None else 2

  def _gas_sensor_msg(self, user_gas: float):
    raw1 = PEDAL_PRESSED_RAW if user_gas > 0 else PEDAL_REST_RAW
    raw2 = raw1 // 2
    idx = self.pedal_idx
    self.pedal_idx = (self.pedal_idx + 1) % 16
    dat = bytearray((raw1 >> 8, raw1 & 0xFF, raw2 >> 8, raw2 & 0xFF, idx & 0xF, 0))
    dat[5] = TeslaCANPreAP.pedal_checksum(GAS_SENSOR_ADDR, dat[:5])
    return (GAS_SENSOR_ADDR, bytes(dat), self.pedal_bus)

  def _stalk_msg(self, simulator_state: SimulatorState):
    turn = 0
    if simulator_state.left_blinker:
      turn = 1
    elif simulator_state.right_blinker:
      turn = 2
    button = int(simulator_state.cruise_button)
    if button not in (
      CruiseButtons.IDLE, CruiseButtons.CANCEL, CruiseButtons.MAIN,
      CruiseButtons.RES_ACCEL, CruiseButtons.DECEL_SET,
      CruiseButtons.RES_ACCEL_2ND, CruiseButtons.DECEL_2ND,
    ):
      button = CruiseButtons.IDLE
    return self.tesla_can.create_action_request(
      button, CANBUS.party, self.idx % 16,
      {"TurnIndLvr_Stat": turn, "VSL_Enbl_Rq": 1},
    )

  def _bosch_point_msgs(self, simulator_state: SimulatorState):
    points = list(getattr(simulator_state, "radar_points", None) or [])
    msgs = []
    for slot in range(BOSCH_POINT_COUNT):
      tracked = slot < len(points)
      if tracked:
        d_rel, y_rel, v_rel = points[slot]
        values_a = {
          "LongDist": float(d_rel),
          "LongSpeed": float(v_rel),
          "LatDist": float(y_rel),
          "ProbExist": 90.0,
          "LongAccel": 0.0,
          "ProbObstacle": 80.0,
          "Valid": 1,
          "Meas": 1,
          "Tracked": 1,
          "Index": 0,
        }
        values_b = {"LatSpeed": 0.0, "Index2": 0, "Class": 1}
      else:
        values_a = {"Tracked": 0, "Valid": 0, "Index": 0, "LongDist": 0.0}
        values_b = {"Index2": 0, "LatSpeed": 0.0}
      msgs.append(self.radar_packer.make_can_msg(f"RadarPoint{slot}_A", CANBUS.radar, values_a))
      msgs.append(self.radar_packer.make_can_msg(f"RadarPoint{slot}_B", CANBUS.radar, values_b))
    return msgs

  def build_can_messages(self, simulator_state: SimulatorState):
    if not simulator_state.valid:
      return []

    speed_ms = simulator_state.speed
    speed_kph = speed_ms * CV.MS_TO_KPH
    speed_mph = speed_ms * CV.MS_TO_MPH
    steer_deg = float(simulator_state.steering_angle)
    brake_pressed = simulator_state.user_brake > 0
    gas_pct = max(0.0, min(100.0, simulator_state.user_gas * 100.0))
    eac_status = EPAS_EAC_ACTIVE if simulator_state.is_engaged else EPAS_EAC_INHIBITED

    msg = []

    msg.append(self.packer.make_can_msg("ESP_B", CANBUS.party, {
      "ESP_vehicleSpeed": speed_kph,
      "ESP_BCounter": self.idx % 15,
    }))
    msg.append(self.packer.make_can_msg("DI_torque1", CANBUS.party, {
      "DI_pedalPos": gas_pct,
      "DI_torque1Counter": self.idx % 8,
    }))
    msg.append(self.packer.make_can_msg("DI_torque2", CANBUS.party, {
      "DI_gear": DI_GEAR_D,
      "DI_brakePedal": int(brake_pressed),
      "DI_vehicleSpeed": speed_mph,
      "DI_torque2Counter": self.idx % 16,
    }))
    msg.append(self.packer.make_can_msg("BrakeMessage", CANBUS.party, {
      "driverBrakeStatus": 2 if brake_pressed else 0,
    }))
    msg.append(self.packer.make_can_msg("DI_state", CANBUS.party, {
      "DI_systemState": DI_SYSTEM_ENABLE,
      "DI_driveReady": 1,
      "DI_cruiseState": DI_CRUISE_OFF,
      "DI_speedUnits": 0,  # MPH
      "DI_digitalSpeed": int(round(speed_mph)),
      "DI_analogSpeed": speed_mph,
      "DI_stateCounter": self.idx % 16,
    }))
    msg.append(self.packer.make_can_msg("GTW_carState", CANBUS.party, {
      "DOOR_STATE_FL": 0,
      "DOOR_STATE_FR": 0,
      "DOOR_STATE_RL": 0,
      "DOOR_STATE_RR": 0,
      "DOOR_STATE_FrontTrunk": 0,
      "BOOT_STATE": 0,
      "BC_indicatorLStatus": int(simulator_state.left_blinker),
      "BC_indicatorRStatus": int(simulator_state.right_blinker),
    }))
    msg.append(self.packer.make_can_msg("EPAS_sysStatus", CANBUS.party, {
      "EPAS_internalSAS": -steer_deg,
      "EPAS_torsionBarTorque": 0.0,
      "EPAS_handsOnLevel": 0,
      "EPAS_eacStatus": eac_status,
      "EPAS_eacErrorCode": 0,
      "EPAS_sysStatusCounter": self.idx % 16,
    }))
    msg.append(self.packer.make_can_msg("STW_ANGLHP_STAT", CANBUS.party, {
      "StW_AnglHP": steer_deg,
      "StW_AnglHP_Spd": 0.0,
    }))
    msg.append(self._stalk_msg(simulator_state))
    msg.append(self._gas_sensor_msg(simulator_state.user_gas))

    if self.radar_enabled:
      msg.append(self.radar_packer.make_can_msg("TeslaRadarSguInfo", CANBUS.radar, {
        "RADC_HWFail": 0,
        "RADC_SGUFail": 0,
        "RADC_SensorDirty": 0,
        "RADC_SGUInfoConsistBit": self.idx % 2,
      }))
      msg.extend(self._bosch_point_msgs(simulator_state))

    return msg

  def send_can_messages(self, simulator_state: SimulatorState):
    msg = self.build_can_messages(simulator_state)
    if not msg:
      return
    self.pm.send('can', can_list_to_can_capnp(msg))

  def send_panda_state(self, simulator_state):
    self.sm.update(0)

    if self.params.get_bool("ObdMultiplexingEnabled") != self.obd_multiplexing:
      self.obd_multiplexing = not self.obd_multiplexing
      self.params.put_bool("ObdMultiplexingChanged", True, block=True)

    flags = 0
    if self.use_pedal:
      flags |= PREAP_FLAG_ENABLE_PEDAL
    if self.radar_enabled:
      flags |= PREAP_FLAG_RADAR_EMULATION

    dat = messaging.new_message('pandaStates', 1)
    dat.valid = True
    dat.pandaStates[0] = {
      'ignitionLine': simulator_state.ignition,
      'pandaType': "blackPanda",
      'controlsAllowed': True,
      'safetyModel': 'teslaPreap',
      'alternativeExperience': self.sm["carParams"].alternativeExperience,
      'safetyParam': flags,
    }
    self.pm.send('pandaStates', dat)

  def update(self, simulator_state: SimulatorState):
    try:
      self.send_can_messages(simulator_state)

      if self.idx % 50 == 0:  # panda states at 2Hz
        self.send_panda_state(simulator_state)

      self.idx += 1
    except Exception:
      traceback.print_exc()
      raise
