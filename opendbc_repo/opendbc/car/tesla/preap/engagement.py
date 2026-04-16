from opendbc.car import structs
from opendbc.car.carlog import carlog
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CruiseButtons

ButtonType = structs.CarState.ButtonEvent.Type

# Echo filter windows: suppress auto-cancel echoes from spoofed stalk messages
CANCEL_ECHO_WINDOW_MS = 600
SPOOF_ECHO_WINDOW_MS = 300


class PreAPEngagement:
  """Pre-AP engagement FSM: double-pull detection, target speed, brake override, CC spoof flags."""

  def __init__(self, double_pull_enabled, double_pull_window_ms):
    self.enableDoublePull = double_pull_enabled
    self.double_pull_window_ms = double_pull_window_ms

    self.cruiseEnabled = False
    self.enableLongControl = False
    self.enableJustCC = False
    self.pending_enable = False

    self.stalk_pull_time_ms = 0
    self.prev_stalk_pull_time_ms = -1000

    self.pedal_speed_kph = 0.0
    self.longCtrlEvent = None

    self.preap_cc_cancel_needed = False
    self.preap_cc_engage_needed = False
    self.preap_last_cc_spoof_ms = 0

    self.preap_brake_pressed_prev = False
    self.last_stalk_non_cancel_ms = -10000
    self.prev_steering_disengage = False

  def handle_steering_disengage(self, steering_disengage):
    """Reset engagement on steering disengage rising edge."""
    if steering_disengage and not self.prev_steering_disengage:
      was_long_active = self.enableLongControl
      self.cruiseEnabled = False
      self.enableLongControl = False
      self.enableJustCC = False
      self.pending_enable = False
      self.pedal_speed_kph = 0.0
      self.stalk_pull_time_ms = 0
      self.prev_stalk_pull_time_ms = -1000
      if was_long_active:
        self.longCtrlEvent = "pccDisabled"
    self.prev_steering_disengage = steering_disengage

  def process_buttons(self, cruise_buttons, prev_cruise_buttons, curr_time_ms,
                      v_ego, speed_units, use_pedal, pedal_long_allowed,
                      long_control_allowed, real_brake_pressed):
    button_events = []

    # MAIN button: rising edge only
    if cruise_buttons == CruiseButtons.MAIN and prev_cruise_buttons != CruiseButtons.MAIN:
      carlog.debug("STALK MAIN | cruiseEnabled=%s enableLong=%s pending=%s pedal=%s doublePull=%s",
                   self.cruiseEnabled, self.enableLongControl, self.pending_enable,
                   use_pedal, self.enableDoublePull)
      if self.enableDoublePull:
        self._handle_double_pull(curr_time_ms, v_ego, speed_units,
                                 use_pedal, pedal_long_allowed, long_control_allowed)
      else:
        carlog.debug("STALK single-pull engage — full control")
        self.cruiseEnabled = True
        self.pending_enable = False
        self.enableLongControl = long_control_allowed
        self.enableJustCC = not long_control_allowed
        if pedal_long_allowed:
          self.pedal_speed_kph = self._capture_target_speed(v_ego, speed_units)
        else:
          self.pedal_speed_kph = 0.0
          if not use_pedal:
            self.preap_cc_engage_needed = True
            self.preap_last_cc_spoof_ms = curr_time_ms

    if cruise_buttons != prev_cruise_buttons:
      be = self._make_button_event(cruise_buttons, prev_cruise_buttons, curr_time_ms,
                                   v_ego, speed_units, use_pedal)
      button_events.append(be)

    # Double-pull window expired
    if self.pending_enable:
      if curr_time_ms - self.stalk_pull_time_ms > self.double_pull_window_ms:
        self.pending_enable = False

    # Brake drops longitudinal while keeping lateral (pedal mode only)
    brake_rising_edge = real_brake_pressed and not self.preap_brake_pressed_prev
    if use_pedal:
      if brake_rising_edge and self.cruiseEnabled and self.enableLongControl:
        carlog.debug("BRAKE rising edge — dropping longitudinal")
        self.enableLongControl = False
        self.enableJustCC = True
        self.pending_enable = False
        self.pedal_speed_kph = 0.0
        self.longCtrlEvent = "pccDisabled"
    self.preap_brake_pressed_prev = real_brake_pressed

    return button_events

  def check_can_engage(self, door_open, gear_shifter, seatbelt_unlatched):
    """Check engagement prerequisites. Resets state if blocked."""
    in_drive = gear_shifter == structs.CarState.GearShifter.drive
    can_engage = not door_open and in_drive and not seatbelt_unlatched
    if not can_engage and self.cruiseEnabled:
      carlog.debug("ENGAGE BLOCKED: door=%s gear=%s seatbelt=%s", door_open, gear_shifter, seatbelt_unlatched)
      self.cruiseEnabled = False
      self.enableLongControl = False
      self.enableJustCC = False
      self.pending_enable = False
    return can_engage

  def _handle_double_pull(self, curr_time_ms, v_ego, speed_units,
                          use_pedal, pedal_long_allowed, long_control_allowed):
    self.prev_stalk_pull_time_ms = self.stalk_pull_time_ms
    self.stalk_pull_time_ms = curr_time_ms
    double_pull = (self.stalk_pull_time_ms - self.prev_stalk_pull_time_ms) < self.double_pull_window_ms

    if double_pull:
      carlog.debug("STALK double-pull (dt=%dms)", self.stalk_pull_time_ms - self.prev_stalk_pull_time_ms)
      self.cruiseEnabled = True
      self.pending_enable = False
      self.enableLongControl = long_control_allowed
      self.enableJustCC = not long_control_allowed
      if pedal_long_allowed:
        self.longCtrlEvent = "pccEnabled"
        self.pedal_speed_kph = self._capture_target_speed(v_ego, speed_units)
      else:
        self.pedal_speed_kph = 0.0
        if not use_pedal:
          self.preap_cc_engage_needed = True
          self.preap_last_cc_spoof_ms = curr_time_ms
    else:
      carlog.debug("STALK first pull — lateral only (window=%dms)", self.double_pull_window_ms)
      was_long_active = self.enableLongControl
      self.cruiseEnabled = True
      self.enableLongControl = False
      self.enableJustCC = True
      self.pedal_speed_kph = 0.0
      self.pending_enable = True
      if was_long_active:
        self.longCtrlEvent = "pccDisabled"
      if not use_pedal:
        self.preap_cc_cancel_needed = True
        self.preap_last_cc_spoof_ms = curr_time_ms

  def _make_button_event(self, cruise_buttons, prev_cruise_buttons, curr_time_ms,
                         v_ego, speed_units, use_pedal):
    be = structs.CarState.ButtonEvent()
    be.pressed = cruise_buttons != CruiseButtons.IDLE
    state = cruise_buttons if be.pressed else prev_cruise_buttons

    if state == CruiseButtons.MAIN:
      be.type = ButtonType.setCruise
      if be.pressed:
        self.last_stalk_non_cancel_ms = curr_time_ms

    elif state == CruiseButtons.CANCEL:
      # Suppress auto-cancel echoes from our spoofed stalk messages
      is_echo = (
        (self.cruiseEnabled and (curr_time_ms - self.last_stalk_non_cancel_ms) < CANCEL_ECHO_WINDOW_MS)
        or ((curr_time_ms - self.preap_last_cc_spoof_ms) < SPOOF_ECHO_WINDOW_MS)
      )
      if not is_echo:
        carlog.debug("STALK CANCEL — disabling all control")
        be.type = ButtonType.cancel
        was_long_active = self.enableLongControl
        self.cruiseEnabled = False
        self.enableLongControl = False
        self.enableJustCC = False
        self.pending_enable = False
        self.pedal_speed_kph = 0.0
        self.stalk_pull_time_ms = 0
        self.prev_stalk_pull_time_ms = -1000
        if was_long_active:
          self.longCtrlEvent = "pccDisabled"
      else:
        be.type = ButtonType.unknown

    elif CruiseButtons.is_accel(state):
      be.type = ButtonType.accelCruise
      if be.pressed:
        self.last_stalk_non_cancel_ms = curr_time_ms
        if not use_pedal and self.cruiseEnabled and not self.enableLongControl:
          self.enableLongControl = True
          self.pending_enable = False
        if self.enableLongControl:
          speed_uom_kph = CV.MPH_TO_KPH if speed_units == "MPH" else 1.0
          actual_kph = int(v_ego * CV.MS_TO_KPH / speed_uom_kph + 0.5) * speed_uom_kph
          if state == CruiseButtons.RES_ACCEL:
            self.pedal_speed_kph = max(self.pedal_speed_kph, actual_kph) + speed_uom_kph
          else:
            self.pedal_speed_kph = max(self.pedal_speed_kph, actual_kph) + 5 * speed_uom_kph
          self.pedal_speed_kph = min(self.pedal_speed_kph, 270.0)

    elif CruiseButtons.is_decel(state):
      be.type = ButtonType.decelCruise
      if be.pressed:
        self.last_stalk_non_cancel_ms = curr_time_ms
        if not use_pedal and self.cruiseEnabled and not self.enableLongControl:
          self.enableLongControl = True
          self.pending_enable = False
        if self.enableLongControl:
          speed_uom_kph = CV.MPH_TO_KPH if speed_units == "MPH" else 1.0
          if state == CruiseButtons.DECEL_SET:
            self.pedal_speed_kph -= speed_uom_kph
          else:
            self.pedal_speed_kph -= 5 * speed_uom_kph
          self.pedal_speed_kph = max(self.pedal_speed_kph, 0.0)

    else:
      be.type = ButtonType.unknown

    return be

  @staticmethod
  def _capture_target_speed(v_ego, speed_units):
    speed_uom_kph = CV.MPH_TO_KPH if speed_units == "MPH" else 1.0
    current_speed_kph = int(v_ego * CV.MS_TO_KPH / speed_uom_kph + 0.5) * speed_uom_kph
    return max(current_speed_kph, 0.0)
