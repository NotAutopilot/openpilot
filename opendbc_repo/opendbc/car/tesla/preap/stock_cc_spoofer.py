"""Stock-CC stalk spoofer.

Translates engagement-FSM stalk intent (CS.preap_cc_cancel_needed,
CS.preap_cc_engage_needed) into 0x45 STW_ACTN_RQ CAN frames the Tesla DI obeys.
Owns the cancel-pending and ENGAGING state machines, the DI-cruiseState edge
events for teslaCCEngaged/Disengaged, and the cadence/timing of spoof TX.

Independent of pedal mode: runs every carcontroller tick. The pedal-control
side and this module communicate exclusively through CarState flags so
neither has to reach into the other's state.
"""
from opendbc.car.carlog import carlog
from opendbc.car.tesla.values import CruiseButtons


# Phases
_PHASE_IDLE = 0
_PHASE_ENGAGING = 1

# Delay between cancel-pending becoming true and CANCEL hitting the bus.
# Lets pedal mode (when engaging) establish control before the DI drops.
CANCEL_DELAY_FRAMES = 10  # 100 ms at 100 Hz

# Max time to keep retrying SET_ACCEL before giving up. The DI may stay in
# STANDBY (e.g. below min cruise speed) and never accept the engage.
CC_ENGAGE_TIMEOUT_FRAMES = 50  # 500 ms at 100 Hz


class StockCCSpoofer:
  """Stalk-spoof state machine. One TX slot per `frame % 10 == 0`."""

  def __init__(self):
    self.cc_engage_phase = _PHASE_IDLE
    self.cc_engage_start_frame = 0
    self.cancel_pending = False
    self.cancel_frame = -1_000_000
    self.prev_di_cc_engaged = False
    self.pcc_event = None

  def update(self, CS, frame, tesla_can, can_bus_party):
    can_sends = []

    # --- Bridge engagement-FSM intent to internal state ---
    if getattr(CS, "preap_cc_cancel_needed", False):
      self.cancel_pending = True
      self.cancel_frame = frame
      # Cancel beats engage: abort any in-flight ENGAGING.
      self.cc_engage_phase = _PHASE_IDLE
      CS.preap_cc_cancel_needed = False

    if getattr(CS, "preap_cc_engage_needed", False) and self.cc_engage_phase == _PHASE_IDLE:
      self.cc_engage_phase = _PHASE_ENGAGING
      self.cc_engage_start_frame = frame
      carlog.debug("StockCC: ENGAGING (di=%s)", getattr(CS, "di_cruise_state", "OFF"))
      CS.preap_cc_engage_needed = False

    # --- TX one frame per 10ms slot (frame % 10 == 0) ---
    cancel_ready = (frame - self.cancel_frame) >= CANCEL_DELAY_FRAMES
    if self.cancel_pending and cancel_ready and frame % 10 == 0:
      sent = self._send(CS, tesla_can, can_bus_party, CruiseButtons.CANCEL)
      if sent is not None:
        can_sends.append(sent)
        self.cancel_pending = False
    elif self.cc_engage_phase == _PHASE_ENGAGING and frame % 10 == 0:
      di_state = getattr(CS, "di_cruise_state", "OFF")
      timed_out = (frame - self.cc_engage_start_frame) >= CC_ENGAGE_TIMEOUT_FRAMES
      if timed_out:
        carlog.warning("StockCC: engage timeout after %d frames",
                       frame - self.cc_engage_start_frame)
        self.cc_engage_phase = _PHASE_IDLE
      elif di_state == "ENABLED":
        carlog.debug("StockCC: ENABLED — exiting ENGAGING")
        self.cc_engage_phase = _PHASE_IDLE
      else:
        sent = self._send(CS, tesla_can, can_bus_party, CruiseButtons.SET_ACCEL)
        if sent is not None:
          can_sends.append(sent)

    # --- Edge events for teslaCCEngaged / teslaCCDisengaged ---
    di_cc_engaged = getattr(CS, "di_cruise_state", "OFF") == "ENABLED"
    if di_cc_engaged and not self.prev_di_cc_engaged:
      self.pcc_event = "teslaCCEngaged"
    elif not di_cc_engaged and self.prev_di_cc_engaged:
      self.pcc_event = "teslaCCDisengaged"
    else:
      self.pcc_event = None
    self.prev_di_cc_engaged = di_cc_engaged

    return can_sends

  def _send(self, CS, tesla_can, can_bus_party, button):
    msg_stw = getattr(CS, "msg_stw_actn_req", None)
    if msg_stw is None:
      return None
    counter = (int(msg_stw.get("MC_STW_ACTN_RQ", 0)) + 1) % 16
    return tesla_can.create_action_request(button, can_bus_party, counter, msg_stw)
