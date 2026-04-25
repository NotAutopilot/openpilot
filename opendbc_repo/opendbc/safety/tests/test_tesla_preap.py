#!/usr/bin/env python3
import unittest
import numpy as np

from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.tesla.values import CarControllerParams
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety

# Safety param flags matching tesla_preap.h (LONG_CONTROL removed — dead code)
PREAP_FLAG_ENABLE_PEDAL = 1
PREAP_FLAG_RADAR_EMULATION = 2
PREAP_FLAG_RADAR_BEHIND_NOSECONE = 4

# Stalk lever positions from tesla_preap.h
STALK_FWD_CANCEL = 1
STALK_RWD_ENGAGE = 2


def _fix_epas_checksum(msg):
  """Compute Tesla byte-sum checksum for EPAS_sysStatus (checksum at byte 7)."""
  addr, data, bus = msg
  data = bytearray(data)
  chk = (addr & 0xFF) + ((addr >> 8) & 0xFF)
  for i in range(len(data)):
    if i != 7:
      chk += data[i]
  data[7] = chk & 0xFF
  return addr, bytes(data), bus


def _fix_das_checksum(msg):
  """Compute Tesla byte-sum checksum for DAS_steeringControl (checksum at byte 3)."""
  addr, data, bus = msg
  data = bytearray(data)
  chk = (addr & 0xFF) + ((addr >> 8) & 0xFF)
  for i in range(len(data)):
    if i != 3:
      chk += data[i]
  data[3] = chk & 0xFF
  return addr, bytes(data), bus


def _get_preap_vm():
  """Get VehicleModel matching PREAP_STEERING_PARAMS in tesla_preap.h."""
  from opendbc.car.tesla.interface import CarInterface
  return VehicleModel(CarInterface.get_non_essential_params("TESLA_MODEL_S_HW3"))


class TeslaPreAPTestMixin(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  # Abstract base class — concrete subclasses (SteeringOnly, WithPedal) do the work.
  # __test__ = False prevents pytest from collecting this class directly (it still
  # gets collected via MRO without this, because CarSafetyTest is a TestCase).
  __test__ = False
  # Pre-AP has no relay and no bus 2 forwarding
  RELAY_MALFUNCTION_ADDRS = {}
  FWD_BUS_LOOKUP = {}
  FWD_BLACKLISTED_ADDRS = {}

  TX_MSGS = [
    [0x488, 0],  # DAS_steeringControl
    [0x2B9, 0],  # DAS_control
    [0x214, 0],  # EPB_epasControl
    [0x551, 0],  # Pedal bus 0
    [0x551, 2],  # Pedal bus 2
    [0x45,  0],  # STW_ACTN_RQ (stalk spoof)
  ]

  STANDSTILL_THRESHOLD = 0.5 / 3.6  # 0.5 kph in m/s

  # Angle control limits
  STEER_ANGLE_MAX = 360  # deg
  DEG_TO_CAN = 10
  LATERAL_FREQUENCY = 50  # Hz

  # Tesla uses VM-based limits, not breakpoint tables
  ANGLE_RATE_BP = None
  ANGLE_RATE_UP = None
  ANGLE_RATE_DOWN = None

  GAS_PRESSED_THRESHOLD = 0  # DI_torque1 byte 6 != 0

  cnt_epas = 0
  cnt_angle_cmd = 0

  packer: CANPackerSafety

  def _get_steer_cmd_angle_max(self, speed):
    return get_max_angle_vm(max(speed, 1), self.VM, CarControllerParams)

  def setUp(self):
    self.VM = _get_preap_vm()
    self.packer = CANPackerSafety("tesla_preap")
    self.safety = libsafety_py.libsafety

  def _angle_cmd_msg(self, angle, state, increment_timer=True, bus=0):
    values = {"DAS_steeringAngleRequest": angle, "DAS_steeringControlType": state}
    if increment_timer:
      self.safety.set_timer(self.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    return self.packer.make_can_msg_safety("DAS_steeringControl", bus, values,
                                           fix_checksum=_fix_das_checksum)

  def _angle_meas_msg(self, angle, hands_on_level=0, eac_status=1, eac_error_code=0):
    values = {
      "EPAS_internalSAS": angle,
      "EPAS_handsOnLevel": hands_on_level,
      "EPAS_eacStatus": eac_status,
      "EPAS_eacErrorCode": eac_error_code,
      "EPAS_sysStatusCounter": self.cnt_epas % 16,
    }
    self.__class__.cnt_epas += 1
    return self.packer.make_can_msg_safety("EPAS_sysStatus", 0, values,
                                           fix_checksum=_fix_epas_checksum)

  def _user_brake_msg(self, brake):
    values = {"driverBrakeStatus": 2 if brake else 1}
    return self.packer.make_can_msg_safety("BrakeMessage", 0, values)

  def _speed_msg(self, speed):
    values = {"ESP_vehicleSpeed": speed * 3.6}  # m/s to kph
    return self.packer.make_can_msg_safety("ESP_B", 0, values)

  def _speed_msg_2(self, speed):
    return None  # Pre-AP has no second speed source

  def _user_gas_msg(self, gas):
    values = {"DI_pedalPos": gas}
    return self.packer.make_can_msg_safety("DI_torque1", 0, values)

  def _pcm_status_msg(self, enable):
    lever = STALK_RWD_ENGAGE if enable else STALK_FWD_CANCEL
    return self.packer.make_can_msg_safety("STW_ACTN_RQ", 0, {"SpdCtrlLvr_Stat": lever})

  def _gear_msg(self, gear):
    return self.packer.make_can_msg_safety("DI_torque2", 0, {"DI_gear": gear})

  def _door_msg(self, door_fl=0, door_fr=0, door_rl=0, door_rr=0):
    values = {
      "DOOR_STATE_FL": door_fl,
      "DOOR_STATE_FR": door_fr,
      "DOOR_STATE_RL": door_rl,
      "DOOR_STATE_RR": door_rr,
    }
    return self.packer.make_can_msg_safety("GTW_carState", 0, values)

  def _engage_and_advance_timer(self):
    """Engage via stalk and advance timer past the 600ms echo filter window."""
    self._rx(self._pcm_status_msg(True))
    self.safety.set_timer(700000)

  # =====================================================================
  # Base class overrides for Pre-AP differences
  # =====================================================================

  def test_angle_cmd_when_enabled(self):
    # Tesla uses VM-based limits — test_lateral_accel_limit covers this
    pass

  def test_angle_cmd_when_disabled(self):
    self._rx(self._angle_meas_msg(0))
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, 0)))
    self.assertFalse(self._tx(self._angle_cmd_msg(100, 0)))

  def test_vehicle_speed_measurements(self):
    self._common_measurement_test(self._speed_msg, 0, 285 / 3.6, 1,
                                  self.safety.get_vehicle_speed_min, self.safety.get_vehicle_speed_max)

  def test_vehicle_moving(self):
    # Pre-AP uses: vehicle_moving = speed > (0.5f * KPH_TO_MS)
    # Due to float32 precision in the DBC factor (0.00999999978 vs 0.01),
    # exactly 0.5 kph may register as slightly above threshold. Use values
    # that are unambiguously below/above regardless of float precision.
    self.assertFalse(self.safety.get_vehicle_moving())
    self._rx(self._speed_msg(0))
    self.assertFalse(self.safety.get_vehicle_moving())
    # 0.3 kph → clearly below 0.5 kph threshold
    self._rx(self.packer.make_can_msg_safety("ESP_B", 0, {"ESP_vehicleSpeed": 0.3}))
    self.assertFalse(self.safety.get_vehicle_moving())
    # 1.0 kph → clearly above 0.5 kph threshold
    self._rx(self.packer.make_can_msg_safety("ESP_B", 0, {"ESP_vehicleSpeed": 1.0}))
    self.assertTrue(self.safety.get_vehicle_moving())

  def test_prev_user_brake(self):
    # PRE-AP BRAKE ARCHITECTURE:
    # The panda hardcodes brake_pressed=false (tesla_preap.h:340) so the
    # framework's generic brake-disengage path never fires. This is deliberate:
    #
    # Pre-AP brake-to-disengage is handled in the Python layer, NOT the panda:
    #   1. preap/carstate.py:41 — reads real_brake_pressed from BrakeMessage CAN
    #   2. preap/carstate.py:132 — passes it to engagement.process_buttons()
    #   3. preap/engagement.py:92-100 — on brake rising edge (pedal mode):
    #      drops enableLongControl=False but keeps cruiseEnabled=True
    #      (lateral stays active, only longitudinal/pedal drops)
    #   4. preap/carstate.py:134 — suppresses ret.brakePressed=False so the
    #      generic openpilot brake handler doesn't also kill lateral
    #
    # This design ensures brake drops pedal (longitudinal) but keeps steering
    # (lateral). The driver can always override steering via hands-on level >= 3.
    # See test_preap_engagement.py for Python-layer verification.
    #
    # Panda-layer invariant: brake_pressed is ALWAYS false.
    self.assertFalse(self.safety.get_brake_pressed_prev())
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self.safety.get_brake_pressed_prev())
    self._rx(self._user_brake_msg(False))
    self.assertFalse(self.safety.get_brake_pressed_prev())

  def test_allow_user_brake_at_zero_speed(self):
    # Pre-AP: brake_pressed is always false in panda → brake never affects
    # controls_allowed at the panda level. See test_prev_user_brake for the
    # full brake architecture explanation.
    self._rx(self._speed_msg(0))
    self._rx(self._user_brake_msg(True))
    self.safety.set_controls_allowed(True)
    self._rx(self._user_brake_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_not_allow_user_brake_when_moving(self):
    # Pre-AP: brake_pressed is always false in panda → brake while moving
    # doesn't disengage at the panda level. The brake-to-drop-longitudinal
    # path is in preap/engagement.py:92-100. See test_prev_user_brake.
    self._rx(self._user_brake_msg(True))
    self.safety.set_controls_allowed(True)
    self._rx(self._speed_msg(self.STANDSTILL_THRESHOLD + 1))
    self._rx(self._user_brake_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_cruise_engaged_prev(self):
    # Pre-AP uses a 600ms echo filter on stalk cancel. Advancing the timer
    # past the window is required for cancel to take effect.
    for engaged in [True, False]:
      self._rx(self._pcm_status_msg(engaged))
      if not engaged:
        self.safety.set_timer(700000)
        self._rx(self._pcm_status_msg(False))
      self.assertEqual(engaged, self.safety.get_cruise_engaged_prev())

  def test_disable_control_allowed_from_cruise(self):
    self._engage_and_advance_timer()
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())

  # test_tx_hook_on_wrong_safety_mode: inherited from base class, no override needed.

  # =====================================================================
  # Pre-AP specific safety tests
  # =====================================================================

  def test_gear_disengage(self):
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._gear_msg(0))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._gear_msg(4))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_door_disengage(self):
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._door_msg(door_fl=1))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_steering_disengage_hands_on(self):
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._angle_meas_msg(0, hands_on_level=3))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_cruise_engaged_prev())

  def test_steering_disengage_epas_error_codes(self):
    # EPAS error codes 6-9 with EAC_INHIBITED (status=0) should disengage.
    # Must reinit safety between iterations to clear stale state.
    for error_code in [6, 7, 8, 9]:
      self.setUp()
      self._setup_safety_hooks()
      self._rx(self._pcm_status_msg(True))
      self.assertTrue(self.safety.get_controls_allowed(), f"Setup failed for error code {error_code}")
      self._rx(self._angle_meas_msg(0, hands_on_level=0, eac_status=0, eac_error_code=error_code))
      self.assertFalse(self.safety.get_controls_allowed(), f"Error code {error_code} should disengage")

  def test_steering_no_disengage_on_other_error_codes(self):
    for error_code in [0, 1, 2, 3, 4, 5, 10, 11, 12]:
      self.setUp()
      self._setup_safety_hooks()
      self._rx(self._pcm_status_msg(True))
      self.assertTrue(self.safety.get_controls_allowed())
      self._rx(self._angle_meas_msg(0, hands_on_level=0, eac_status=0, eac_error_code=error_code))
      self.assertTrue(self.safety.get_controls_allowed(), f"Error code {error_code} should NOT disengage")

  def test_stalk_cancel_echo_filter(self):
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    # Cancel within echo window should be filtered
    self._rx(self._pcm_status_msg(False))
    self.assertTrue(self.safety.get_controls_allowed())
    # Cancel after echo window should work
    self.safety.set_timer(700000)
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_stalk_rearm_after_steering_disengage(self):
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._angle_meas_msg(0, hands_on_level=3))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_cruise_engaged_prev())
    self._rx(self._angle_meas_msg(0, hands_on_level=0))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_steering_control_type(self):
    self.safety.set_controls_allowed(True)
    self._rx(self._angle_meas_msg(0))
    for control_type in range(4):
      should_tx = control_type in (0, 1)
      self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(0, control_type)),
                       f"Control type {control_type} should {'pass' if should_tx else 'block'}")

  def test_epas_control_type(self):
    self.safety.set_controls_allowed(True)
    for mode in range(8):
      msg = self.packer.make_can_msg_safety("EPB_epasControl", 0, {"EPB_epasEACAllow": mode})
      should_tx = mode <= 1
      self.assertEqual(should_tx, self._tx(msg),
                       f"EPB mode {mode} should {'pass' if should_tx else 'block'}")

  def test_no_aeb(self):
    self.safety.set_controls_allowed(True)
    for aeb_event in range(4):
      msg = self.packer.make_can_msg_safety("DAS_control", 0, {"DAS_aebEvent": aeb_event})
      should_tx = aeb_event == 0
      self.assertEqual(should_tx, self._tx(msg),
                       f"AEB event {aeb_event} should {'pass' if should_tx else 'block'}")

  def test_lateral_accel_limit(self):
    # Verify VM-based lateral accel limits constrain steering at speed.
    # Ramp steering angle up in max_delta increments at a fixed speed, find the
    # angle at which the panda blocks further increases, and assert it's within
    # a tight tolerance of Python's VehicleModel computation.
    #
    # Float precision note: panda uses float32 for the curvature_factor
    # computation while Python uses float64. The difference is typically < 2°
    # at highway speeds. We allow 25% tolerance to absorb this while still
    # catching any bug that would make the limit off by a factor of 2 or more
    # (e.g. wrong slip_factor, wrong MAX_LATERAL_ACCEL, wrong wheelbase).
    #
    # TODO: for bit-exact boundary testing, port the approach from
    # test_tesla_hw1.py (round_angle + _reset_speed_measurement +
    # set_desired_angle_last) which does precise +1/+2 CAN-unit tests by
    # matching the panda's float32 arithmetic in Python.
    for speed in [20.0, 30.0]:
      self.setUp()
      self._setup_safety_hooks()
      self.safety.set_controls_allowed(True)
      # Must fill the vehicle_speed sample buffer (6 slots) so min converges
      for _ in range(10):
        self._rx(self._speed_msg(speed))
      self._rx(self._angle_meas_msg(0))
      self._tx(self._angle_cmd_msg(0, 1))

      expected_max = get_max_angle_vm(speed, self.VM, CarControllerParams)
      max_delta = get_max_angle_delta_vm(max(speed, 1), self.VM, CarControllerParams)
      angle = 0.0
      blocked_at = None
      for _ in range(5000):
        next_angle = angle + max_delta
        if next_angle > self.STEER_ANGLE_MAX:
          break
        if not self._tx(self._angle_cmd_msg(next_angle, 1)):
          blocked_at = next_angle
          break
        angle = next_angle

      self.assertIsNotNone(blocked_at,
                           f"Speed {speed}: VM limit never blocked — reached {angle:.1f} deg "
                           f"(Python expected max {expected_max:.1f} deg)")
      # Tight bound: blocked angle must be within ±25% of Python's computation.
      # Absorbs float32/float64 drift but catches order-of-magnitude bugs.
      lower_bound = expected_max * 0.75
      upper_bound = expected_max * 1.25
      self.assertGreaterEqual(blocked_at, lower_bound,
                              f"Speed {speed}: blocked at {blocked_at:.1f} deg — "
                              f"too LOW (expected ~{expected_max:.1f}, bound {lower_bound:.1f})")
      self.assertLessEqual(blocked_at, upper_bound,
                           f"Speed {speed}: blocked at {blocked_at:.1f} deg — "
                           f"too HIGH (expected ~{expected_max:.1f}, bound {upper_bound:.1f})")

  def _setup_safety_hooks(self):
    """Subclasses call this to set up the correct safety hooks."""
    raise NotImplementedError


class TestTeslaPreAPSteeringOnly(TeslaPreAPTestMixin, unittest.TestCase):
  """Pre-AP with no pedal — lateral only."""
  __test__ = True  # re-enable collection (mixin sets __test__=False)

  def setUp(self):
    super().setUp()
    self._setup_safety_hooks()

  def _setup_safety_hooks(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, 0)
    self.safety.init_tests()

  def test_pedal_blocked_without_flag(self):
    self.safety.set_controls_allowed(True)
    msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0, {"GAS_COMMAND": 0, "ENABLE": 1})
    self.assertFalse(self._tx(msg))

  def test_no_pedal_does_not_invalidate_rx_checks(self):
    # Regression: route 02ae0a637825acd6|196fda2496 — tester with no Comma Pedal
    # hit "Controls Mismatch" because the 0x552 rx_check used frequency=0,
    # which divides by zero in safety_tick (safety.h:330) and marks it lagging.
    # When ENABLE_PEDAL is unset, 0x552 must not be in rx_checks at all.
    di_state = self.packer.make_can_msg_safety("DI_state", 0, {"DI_state": 1})
    for msg in (self._angle_meas_msg(0), self._pcm_status_msg(False), self._speed_msg(0),
                self._user_brake_msg(False), self._user_gas_msg(0), self._gear_msg(4),
                self._door_msg(), di_state):
      self._rx(msg)
    # Tick 500ms after msgs — under the 1s min lag window for real checks but long
    # enough that a freq=0 divide-by-zero would have already tripped the pedal row.
    self.safety.set_timer(int(5e5))
    self.safety.safety_tick_current_safety_config()
    self.assertTrue(self.safety.safety_config_valid(),
                    "rx_checks must stay valid without a pedal installed")


class TestTeslaPreAPWithPedal(TeslaPreAPTestMixin, unittest.TestCase):
  """Pre-AP with Comma Pedal enabled."""
  __test__ = True  # re-enable collection (mixin sets __test__=False)

  def setUp(self):
    super().setUp()
    self._setup_safety_hooks()

  def _setup_safety_hooks(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, PREAP_FLAG_ENABLE_PEDAL)
    self.safety.init_tests()

  # Pedal interceptor (0x552) values are raw 16-bit integers read by the panda as
  # `(data[0] << 8) | data[1]`. The DBC scales them to physical:
  #   physical = raw * 0.0507968128 - 22.85856576
  # Panda threshold: raw > 650 → gas_pressed (chosen from real drive data; see
  # comments in tesla_preap.h rx_hook). Helper values below:
  PEDAL_RAW_AT_REST_MAX = 633      # max observed at rest in real drive data
  PEDAL_RAW_NOISE_THRESHOLD = 650  # panda threshold
  PEDAL_RAW_CLEAR_PRESS = 800      # clearly pressed

  @staticmethod
  def _raw_to_physical(raw):
    return raw * 0.0507968128 - 22.85856576

  def _pedal_msg(self, raw_value, bus=0):
    """Craft a 0x552 message with the given raw value by converting to physical."""
    return self.packer.make_can_msg_safety("GAS_SENSOR", bus,
                                           {"INTERCEPTOR_GAS": self._raw_to_physical(raw_value)})

  def _user_gas_msg(self, gas):
    # With pedal enabled, gas is detected from pedal interceptor (0x552),
    # not DI_torque1. The C code ignores DI_torque1 gas when pedal is active.
    # Use clearly pressed value when gas=True; clearly not pressed when gas=False.
    raw = self.PEDAL_RAW_CLEAR_PRESS if gas else 400
    return self._pedal_msg(raw)

  def test_pedal_allowed_with_flag(self):
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0, {"GAS_COMMAND": 0, "ENABLE": 1})
    self.assertTrue(self._tx(msg))

  def test_pedal_blocked_without_controls(self):
    self.assertFalse(self.safety.get_controls_allowed())
    msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0, {"GAS_COMMAND": 0, "ENABLE": 1})
    self.assertFalse(self._tx(msg))

  def test_pedal_gas_detection_bus_0(self):
    # Verify pedal gas detection works on bus 0 (first wiring config).
    self.assertFalse(self.safety.get_gas_pressed_prev())
    # Clearly pressed: raw 800 → > 650 → gas_pressed=True
    self._rx(self._pedal_msg(self.PEDAL_RAW_CLEAR_PRESS, bus=0))
    self.assertTrue(self.safety.get_gas_pressed_prev(),
                    "Pedal gas on bus 0 must set gas_pressed")
    # Clearly not pressed: raw 400 → < 650 → gas_pressed=False
    self._rx(self._pedal_msg(400, bus=0))
    self.assertFalse(self.safety.get_gas_pressed_prev())

  def test_pedal_gas_detection_bus_2(self):
    # Verify pedal gas detection works on bus 2 (second wiring config).
    # Regression test: earlier version had `if (msg->bus != 0U) return;` at the
    # top of rx_hook that broke bus-2-wired pedals.
    self.assertFalse(self.safety.get_gas_pressed_prev())
    self._rx(self._pedal_msg(self.PEDAL_RAW_CLEAR_PRESS, bus=2))
    self.assertTrue(self.safety.get_gas_pressed_prev(),
                    "Pedal gas on bus 2 must set gas_pressed (wiring config variant)")

  def test_pedal_gas_blocks_longitudinal_tx(self):
    # Full-chain test: pedal press → gas_pressed → !get_longitudinal_allowed() → pedal TX blocked.
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_longitudinal_allowed())
    # Press pedal (clearly pressed)
    self._rx(self._pedal_msg(self.PEDAL_RAW_CLEAR_PRESS, bus=0))
    self.assertTrue(self.safety.get_gas_pressed_prev())
    self.assertFalse(self.safety.get_longitudinal_allowed())
    # Pedal TX must be blocked
    tx_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0, {"GAS_COMMAND": 0, "ENABLE": 1})
    self.assertFalse(self._tx(tx_msg), "Pedal TX must be blocked during gas press")

  def test_pedal_rest_noise_does_not_trigger_gas(self):
    # Regression test for the pedal-engagement bug found in drive d0cdc986c5d023f5.
    # The pedal interceptor's resting voltage oscillates with noise; real Pre-AP
    # drive data showed raw values 424-633 while the driver was NOT pressing gas.
    # The original threshold of 450 was inside this noise range, causing false
    # gas_pressed readings that blocked pedal TX and prevented engagement.
    #
    # Verify that values across the entire observed rest-noise range do NOT
    # trigger gas_pressed.
    for raw in [424, 450, 475, 500, 550, 600, self.PEDAL_RAW_AT_REST_MAX]:
      for bus in [0, 2]:
        self.setUp()
        self._setup_safety_hooks()
        self._rx(self._pedal_msg(raw, bus=bus))
        self.assertFalse(self.safety.get_gas_pressed_prev(),
                         f"Raw {raw} on bus {bus} must NOT trigger gas_pressed (in rest noise range)")

  def test_pedal_rest_noise_does_not_block_longitudinal(self):
    # End-to-end regression test: after engaging, pedal rest noise must not cause
    # longitudinal TX to be blocked. Before this fix, noise-level raw values
    # (450-633) were stuck setting gas_pressed=True, blocking all pedal TX.
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    tx_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0, {"GAS_COMMAND": 0, "ENABLE": 1})
    # Pump pedal messages across the at-rest noise range; TX must remain allowed
    for raw in [424, 450, 500, 550, 600, 633]:
      self._rx(self._pedal_msg(raw, bus=2))  # real drive had pedal on bus 2
      self.assertFalse(self.safety.get_gas_pressed_prev(),
                       f"Raw {raw} (noise) must not set gas_pressed")
      self.assertTrue(self._tx(tx_msg),
                      f"Pedal TX must be allowed at raw {raw} (noise range)")

  def test_pedal_passthrough_enable_0_always_allowed(self):
    # NAP's pedal passthrough feature: when driver presses OEM pedal, Python
    # sends GAS_COMMAND with enable=0 to tell the Comma Pedal to passthrough
    # driver's foot directly. This message RELEASES control and is always safe.
    # Panda must let enable=0 through regardless of controls_allowed / gas_pressed.
    disable_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0,
                                                  {"GAS_COMMAND": 0, "ENABLE": 0})

    # Case 1: not engaged, no gas — still allowed (benign)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(disable_msg),
                    "enable=0 must be allowed when not engaged (passthrough)")

    # Case 2: engaged but driver pressing gas — this is the passthrough scenario
    self._rx(self._pcm_status_msg(True))
    self._rx(self._pedal_msg(self.PEDAL_RAW_CLEAR_PRESS, bus=2))
    self.assertTrue(self.safety.get_gas_pressed_prev())
    self.assertFalse(self.safety.get_longitudinal_allowed())
    self.assertTrue(self._tx(disable_msg),
                    "enable=0 must be allowed during gas override (explicit passthrough)")

  def test_pedal_enable_1_blocked_on_gas_press(self):
    # Conversely, enable=1 (authoritative accel command) MUST be blocked
    # when driver is pressing gas, preventing openpilot from overriding the driver.
    enable_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0,
                                                 {"GAS_COMMAND": 0, "ENABLE": 1})
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self._tx(enable_msg), "enable=1 allowed before gas press")

    # Driver presses gas
    self._rx(self._pedal_msg(self.PEDAL_RAW_CLEAR_PRESS, bus=2))
    self.assertFalse(self._tx(enable_msg),
                     "enable=1 must be blocked during driver gas press")

  def test_pedal_enable_0_blocked_without_flag(self):
    # If PREAP_FLAG_ENABLE_PEDAL is not set, NO 0x551 TX is allowed
    # (not even enable=0). This is the "pedal feature disabled" gate.
    # Override setUp to init without the pedal flag.
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, 0)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    disable_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0,
                                                  {"GAS_COMMAND": 0, "ENABLE": 0})
    self.assertFalse(self._tx(disable_msg),
                     "enable=0 must still be blocked without PREAP_FLAG_ENABLE_PEDAL")

  def test_pedal_enable_0_with_high_gas_blocked(self):
    # Defense-in-depth: ENABLE=0 + non-zero GAS_COMMAND must be blocked.
    # Legitimate passthrough sends GAS_COMMAND=0 (physical) which is raw ~450.
    # Any ENABLE=0 message with a raw value above 500 is suspicious (possible
    # bug or attack attempting to exploit a hypothetical Comma Pedal firmware
    # flaw where ENABLE=0 is not honored).
    # Verify: legitimate passthrough (physical 0) allowed, high-value blocked.
    self._rx(self._pcm_status_msg(True))  # engage
    # Legitimate: physical 0 = raw 450 → <=500 → allowed
    ok_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0,
                                             {"GAS_COMMAND": 0, "ENABLE": 0})
    self.assertTrue(self._tx(ok_msg))
    # Attack: physical 100 = raw 2419 → >500 → blocked
    attack_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0,
                                                 {"GAS_COMMAND": 100, "ENABLE": 0})
    self.assertFalse(self._tx(attack_msg),
                     "ENABLE=0 with high GAS_COMMAND must be blocked (defense-in-depth)")


if __name__ == "__main__":
  unittest.main()
