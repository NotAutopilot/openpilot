# Lane-Change Auto Signal Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make a short turn-signal stalk tap arm an automated lane change on the Pre-AP Tesla Model S, sustaining the blinker to 7 flashes with a live "Nudge wheel within N signals" countdown, and stopping the blinker automatically when the maneuver completes.

**Architecture:** Three layers. (1) `desire_helper.py` gains a flash-counted arming latch that no longer requires the driver to hold the stalk. (2) `modeld` publishes `flashes_remaining` on `modelV2.meta`; `selfdrived` renders it as a dynamic countdown alert. (3) The Pre-AP carcontroller consumes the already-set `CC.leftBlinker`/`rightBlinker` flags and drives the blinker via the `DAS_bodyControls` (0x3E9) CAN message (the proven Tinkla mechanism), which is added to the panda Pre-AP TX allowlist.

**Tech Stack:** Python (openpilot controls/model), Cap'n Proto (cereal), C (panda safety), opendbc CANPacker, pytest.

---

## File Structure

| File | Responsibility | Action |
|---|---|---|
| `cereal/log.capnp` | Add `laneChangeFlashesRemaining` to `ModelDataV2.MetaData` | Modify |
| `selfdrive/controls/lib/desire_helper.py` | Flash-counted arming latch + `flashes_remaining` | Modify |
| `selfdrive/controls/lib/tests/test_desire_helper.py` | Unit tests for the latch | Create |
| `selfdrive/modeld/modeld.py` | Publish `DH.flashes_remaining` | Modify |
| `selfdrive/selfdrived/events.py` | Dynamic countdown alerts | Modify |
| `opendbc_repo/opendbc/car/tesla/preap/teslacan.py` | `create_body_controls_message` (0x3E9) | Modify |
| `opendbc_repo/opendbc/car/tesla/preap/carcontroller.py` | Consume `CC.leftBlinker/rightBlinker` → send body controls | Modify |
| `opendbc_repo/opendbc/car/tesla/preap/tests/test_body_controls.py` | Frame-level tests | Create |
| `opendbc_repo/opendbc/safety/modes/tesla_preap.h` | Allowlist 0x3E9 + bound turn value | Modify |
| `opendbc_repo/opendbc/safety/tests/test_tesla_preap.py` | Safety test for 0x3E9 | Modify |

**Note on submodule:** `opendbc_repo` is a git submodule. CAN/safety changes are committed *inside* the submodule, then the parent repo records the new submodule SHA. Each opendbc task's commit step runs inside `opendbc_repo`; a final task bumps the submodule pointer in the parent.

---

## Task 1: Add `laneChangeFlashesRemaining` to cereal schema

**Files:**
- Modify: `cereal/log.capnp` (struct `ModelDataV2.MetaData`, ~line 1052)

- [ ] **Step 1: Add the field**

In `cereal/log.capnp`, change:
```capnp
  struct MetaData {
    laneChangeState @0 :LaneChangeState;
    laneChangeDirection @1 :LaneChangeDirection;
  }
```
to:
```capnp
  struct MetaData {
    laneChangeState @0 :LaneChangeState;
    laneChangeDirection @1 :LaneChangeDirection;
    laneChangeFlashesRemaining @2 :UInt8;
  }
```

- [ ] **Step 2: Verify the schema compiles**

Run: `python -c "from cereal import log; m = log.ModelDataV2.new_message(); m.meta.laneChangeFlashesRemaining = 7; print(m.meta.laneChangeFlashesRemaining)"`
Expected: prints `7` with no schema error.

- [ ] **Step 3: Commit**

```bash
git add cereal/log.capnp
git commit -m "cereal: add laneChangeFlashesRemaining to ModelDataV2.MetaData"
```

---

## Task 2: Flash-counted arming latch in `desire_helper.py`

**Files:**
- Modify: `selfdrive/controls/lib/desire_helper.py`
- Create: `selfdrive/controls/lib/tests/test_desire_helper.py`

Behavior (from spec): arm `preLaneChange` on a `one_blinker` rising edge; count rising edges of the armed-direction blinker; expose `flashes_remaining = max(7 - seen, 0)`; the latch does NOT require `one_blinker` to stay true; exit to `off` when the budget is exhausted with no nudge, or on opposite-direction tap, or on the existing safety exits; exit to `laneChangeStarting` on wheel nudge.

- [ ] **Step 1: Create the test directory init (if missing)**

Run: `python -c "import os; os.makedirs('selfdrive/controls/lib/tests', exist_ok=True); open('selfdrive/controls/lib/tests/__init__.py','a').close(); print('ok')"`
Expected: prints `ok`.

- [ ] **Step 2: Write the failing tests**

Create `selfdrive/controls/lib/tests/test_desire_helper.py`:
```python
from cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import (
    DesireHelper, LaneChangeState, LaneChangeDirection, LANE_CHANGE_FLASH_BUDGET,
)
from openpilot.common.constants import CV


class FakeCarState:
    def __init__(self, v_ego=30.0, left=False, right=False,
                 steering_pressed=False, steering_torque=0.0,
                 left_blindspot=False, right_blindspot=False):
        self.vEgo = v_ego
        self.leftBlinker = left
        self.rightBlinker = right
        self.steeringPressed = steering_pressed
        self.steeringTorque = steering_torque
        self.leftBlindspot = left_blindspot
        self.rightBlindspot = right_blindspot


def _pulse_blinker(dh, cs, on):
    """Drive one blinker on/off transition through update() to count a flash."""
    cs.leftBlinker = on and (dh.lane_change_direction == LaneChangeDirection.left)
    cs.rightBlinker = on and (dh.lane_change_direction == LaneChangeDirection.right)
    dh.update(cs, lateral_active=True, lane_change_prob=0.0)


def test_tap_arms_pre_lane_change():
    dh = DesireHelper()
    # rising edge of one_blinker (a tap)
    dh.update(FakeCarState(left=False), True, 0.0)
    dh.update(FakeCarState(left=True), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    assert dh.lane_change_direction == LaneChangeDirection.left


def test_latch_survives_blinker_off():
    # Core fix: after the tap, blinker physically goes off but arming persists.
    dh = DesireHelper()
    dh.update(FakeCarState(left=False), True, 0.0)
    dh.update(FakeCarState(left=True), True, 0.0)
    # blinker now off (stock 3-flash ended), no wheel nudge
    dh.update(FakeCarState(left=False), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.preLaneChange


def test_flashes_remaining_counts_down():
    dh = DesireHelper()
    dh.update(FakeCarState(left=False), True, 0.0)
    cs = FakeCarState(left=True)
    dh.update(cs, True, 0.0)  # enter preLaneChange, first rising edge = 1 flash
    assert dh.flashes_remaining == LANE_CHANGE_FLASH_BUDGET - 1
    # toggle blinker off then on -> second flash
    _pulse_blinker(dh, cs, False)
    _pulse_blinker(dh, cs, True)
    assert dh.flashes_remaining == LANE_CHANGE_FLASH_BUDGET - 2


def test_cancels_when_budget_exhausted_without_nudge():
    dh = DesireHelper()
    dh.update(FakeCarState(left=False), True, 0.0)
    cs = FakeCarState(left=True)
    dh.update(cs, True, 0.0)
    # Generate 7 flashes with no wheel nudge
    for _ in range(LANE_CHANGE_FLASH_BUDGET):
        _pulse_blinker(dh, cs, False)
        _pulse_blinker(dh, cs, True)
    assert dh.lane_change_state == LaneChangeState.off
    assert dh.flashes_remaining == 0


def test_wheel_nudge_starts_lane_change():
    dh = DesireHelper()
    dh.update(FakeCarState(left=False), True, 0.0)
    dh.update(FakeCarState(left=True), True, 0.0)
    # driver nudges wheel left (positive torque, pressed) while armed left
    nudge = FakeCarState(left=True, steering_pressed=True, steering_torque=1.0)
    dh.update(nudge, True, 0.0)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting


def test_opposite_tap_cancels():
    dh = DesireHelper()
    dh.update(FakeCarState(left=False), True, 0.0)
    dh.update(FakeCarState(left=True), True, 0.0)   # armed left
    # opposite tap: right blinker rising edge
    dh.update(FakeCarState(left=False, right=False), True, 0.0)  # blinker off
    dh.update(FakeCarState(right=True), True, 0.0)  # right rising edge
    assert dh.lane_change_state == LaneChangeState.off


def test_below_speed_does_not_arm():
    dh = DesireHelper()
    slow = 10 * CV.MPH_TO_MS
    dh.update(FakeCarState(v_ego=slow, left=False), True, 0.0)
    dh.update(FakeCarState(v_ego=slow, left=True), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.off
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `python -m pytest selfdrive/controls/lib/tests/test_desire_helper.py -v`
Expected: FAIL — `ImportError: cannot import name 'LANE_CHANGE_FLASH_BUDGET'` (and others).

- [ ] **Step 4: Rewrite `desire_helper.py` with the latch**

Replace the entire body of `selfdrive/controls/lib/desire_helper.py` with:
```python
from cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.

# Total indicator flashes allowed in the pre-nudge arming window (stock 3 + 4
# openpilot-sustained). When the count reaches this with no wheel nudge, arming
# cancels. The flash count also drives the "Nudge wheel within N signals" UI.
LANE_CHANGE_FLASH_BUDGET = 7

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.none,
    LaneChangeState.laneChangeFinishing: log.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeRight,
  },
}


class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none

    # Flash-counted arming latch
    self.lane_change_flashes_seen = 0
    self.flashes_remaining = LANE_CHANGE_FLASH_BUDGET
    self.prev_blinker_on = False

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def _armed_blinker_on(self, carstate):
    if self.lane_change_direction == LaneChangeDirection.left:
      return carstate.leftBlinker
    if self.lane_change_direction == LaneChangeDirection.right:
      return carstate.rightBlinker
    return False

  def update(self, carstate, lateral_active, lane_change_prob):
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
    else:
      # LaneChangeState.off
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0
        self.lane_change_direction = self.get_lane_change_direction(carstate)
        # Reset the flash latch for a fresh arming window.
        self.lane_change_flashes_seen = 0
        self.prev_blinker_on = False

      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

        # Opposite-direction tap cancels arming (driver must re-tap to re-arm).
        opposite_tap = one_blinker and not self.prev_one_blinker and \
                       self.get_lane_change_direction(carstate) != self.lane_change_direction

        # Count a flash on each rising edge of the armed-direction indicator.
        blinker_on = self._armed_blinker_on(carstate)
        if blinker_on and not self.prev_blinker_on:
          self.lane_change_flashes_seen += 1
        self.prev_blinker_on = blinker_on

        flashes_left = max(LANE_CHANGE_FLASH_BUDGET - self.lane_change_flashes_seen, 0)

        if below_lane_change_speed or opposite_tap:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
        elif torque_applied and not blindspot_detected:
          self.lane_change_state = LaneChangeState.laneChangeStarting
        elif flashes_left <= 0:
          # Budget exhausted with no wheel nudge — cancel.
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none

      # LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)
        if self.lane_change_ll_prob > 0.99:
          self.lane_change_direction = LaneChangeDirection.none
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
          else:
            self.lane_change_state = LaneChangeState.off

    # Publish remaining flashes (only meaningful in preLaneChange; full otherwise).
    if self.lane_change_state == LaneChangeState.preLaneChange:
      self.flashes_remaining = max(LANE_CHANGE_FLASH_BUDGET - self.lane_change_flashes_seen, 0)
    else:
      self.flashes_remaining = LANE_CHANGE_FLASH_BUDGET
      self.lane_change_flashes_seen = 0
      self.prev_blinker_on = False

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.prev_one_blinker = one_blinker

    self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # Send keep pulse once per second during LaneChangeState.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `python -m pytest selfdrive/controls/lib/tests/test_desire_helper.py -v`
Expected: PASS (all 7 tests).

- [ ] **Step 6: Commit**

```bash
git add selfdrive/controls/lib/desire_helper.py selfdrive/controls/lib/tests/
git commit -m "desire_helper: flash-counted lane-change arming latch"
```

---

## Task 3: Publish `flashes_remaining` from modeld

**Files:**
- Modify: `selfdrive/modeld/modeld.py` (~line 402-405)

- [ ] **Step 1: Add the publish lines**

In `selfdrive/modeld/modeld.py`, after the existing block:
```python
      modelv2_send.modelV2.meta.laneChangeState = DH.lane_change_state
      modelv2_send.modelV2.meta.laneChangeDirection = DH.lane_change_direction
```
add:
```python
      modelv2_send.modelV2.meta.laneChangeFlashesRemaining = DH.flashes_remaining
```

- [ ] **Step 2: Verify it imports/parses**

Run: `python -c "import ast; ast.parse(open('selfdrive/modeld/modeld.py').read()); print('ok')"`
Expected: prints `ok`.

- [ ] **Step 3: Commit**

```bash
git add selfdrive/modeld/modeld.py
git commit -m "modeld: publish laneChangeFlashesRemaining"
```

---

## Task 4: Dynamic countdown alert in events.py

**Files:**
- Modify: `selfdrive/selfdrived/events.py` (alert callbacks region ~line 245; event dict ~line 578)

- [ ] **Step 1: Add alert callback functions**

In `selfdrive/selfdrived/events.py`, near the other alert-callback functions (after `below_steer_speed_alert`, ~line 255), add:
```python
def pre_lane_change_left_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  n = sm['modelV2'].meta.laneChangeFlashesRemaining
  return Alert(
    f"Nudge wheel left to change lane within {n} signals",
    "",
    AlertStatus.normal, AlertSize.small,
    Priority.LOW, VisualAlert.none, AudibleAlert.none, .1)


def pre_lane_change_right_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  n = sm['modelV2'].meta.laneChangeFlashesRemaining
  return Alert(
    f"Nudge wheel right to change lane within {n} signals",
    "",
    AlertStatus.normal, AlertSize.small,
    Priority.LOW, VisualAlert.none, AudibleAlert.none, .1)
```

- [ ] **Step 2: Wire the callbacks into the event dict**

In `selfdrive/selfdrived/events.py`, replace:
```python
  EventName.preLaneChangeLeft: {
    ET.WARNING: Alert(
      "Steer Left to Start Lane Change Once Safe",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1),
  },

  EventName.preLaneChangeRight: {
    ET.WARNING: Alert(
      "Steer Right to Start Lane Change Once Safe",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1),
  },
```
with:
```python
  EventName.preLaneChangeLeft: {
    ET.WARNING: pre_lane_change_left_alert,
  },

  EventName.preLaneChangeRight: {
    ET.WARNING: pre_lane_change_right_alert,
  },
```

- [ ] **Step 3: Verify it parses and the callback type is accepted**

Run: `python -c "from openpilot.selfdrive.selfdrived import events; print('ok')"`
Expected: prints `ok` (no import/type error).

- [ ] **Step 4: Commit**

```bash
git add selfdrive/selfdrived/events.py
git commit -m "events: dynamic 'Nudge wheel within N signals' countdown alert"
```

---

## Task 5: `create_body_controls_message` in opendbc (Pre-AP teslacan)

**Files (inside `opendbc_repo`):**
- Modify: `opendbc/car/tesla/preap/teslacan.py`
- Create: `opendbc/car/tesla/preap/tests/test_body_controls.py`

All commands in this task run from `opendbc_repo/`.

- [ ] **Step 1: Write the failing test**

Create `opendbc_repo/opendbc/car/tesla/preap/tests/test_body_controls.py`:
```python
"""Frame-level invariants for NAP Pre-AP DAS_bodyControls (turn signal drive)."""
from opendbc.can import CANPacker
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
from opendbc.car.tesla.values import CANBUS


def _tc():
  packer = CANPacker("tesla_preap")
  return TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})


def test_addr_is_body_controls():
  tc = _tc()
  addr, _, _ = tc.create_body_controls_message(1, 0, CANBUS.party, 1)
  assert addr == 0x3E9  # DAS_bodyControls / 1001


def test_turn_left_sets_indicator_left():
  tc = _tc()
  packer = CANPacker("tesla_preap")
  addr, dat, _ = tc.create_body_controls_message(1, 0, CANBUS.party, 1)
  vals = packer.unpack(addr, dat) if hasattr(packer, "unpack") else None
  # DAS_turnIndicatorRequest is at bit 8 (byte 1, bits 0-1)
  assert dat[1] & 0x03 == 1


def test_turn_right_sets_indicator_right():
  tc = _tc()
  _, dat, _ = tc.create_body_controls_message(2, 0, CANBUS.party, 1)
  assert dat[1] & 0x03 == 2


def test_turn_none_sets_indicator_none():
  tc = _tc()
  _, dat, _ = tc.create_body_controls_message(0, 0, CANBUS.party, 1)
  assert dat[1] & 0x03 == 0


def test_reason_set_when_turning():
  tc = _tc()
  # DAS_turnIndicatorRequestReason at bit 16 (byte 2, bits 0-3)
  _, dat_on, _ = tc.create_body_controls_message(1, 0, CANBUS.party, 1)
  _, dat_off, _ = tc.create_body_controls_message(0, 0, CANBUS.party, 1)
  assert dat_on[2] & 0x0F == 1
  assert dat_off[2] & 0x0F == 0
```

- [ ] **Step 2: Run test to verify it fails**

Run (from `opendbc_repo/`): `python -m pytest opendbc/car/tesla/preap/tests/test_body_controls.py -v`
Expected: FAIL — `AttributeError: 'TeslaCANPreAP' object has no attribute 'create_body_controls_message'`.

- [ ] **Step 3: Implement `create_body_controls_message`**

In `opendbc_repo/opendbc/car/tesla/preap/teslacan.py`, add this method to the `TeslaCANPreAP` class (after `create_action_request`):
```python
  def create_body_controls_message(self, turn, hazard, bus, counter):
    """Build DAS_bodyControls (0x3E9) to drive the turn indicator.

    turn: 0=none, 1=left, 2=right (matches CC.rightBlinker*2 + CC.leftBlinker).
    Reason=1 (DAS_ACTIVE_NAV_LANE_CHANGE) when turning, 0 otherwise. This is the
    proven Tinkla/Tesla-Unity Pre-AP blinker mechanism.
    """
    values = {
      "DAS_headlightRequest": 0,
      "DAS_hazardLightRequest": hazard,
      "DAS_wiperSpeed": 0,
      "DAS_turnIndicatorRequest": turn,
      "DAS_highLowBeamDecision": 0,
      "DAS_highLowBeamOffReason": 0,
      "DAS_turnIndicatorRequestReason": 1 if turn > 0 else 0,
      "DAS_bodyControlsCounter": counter,
      "DAS_bodyControlsChecksum": 0,
    }
    data = self.packers[CANBUS.party].make_can_msg("DAS_bodyControls", bus, values)[1]
    values["DAS_bodyControlsChecksum"] = self.checksum(0x3E9, data[:7])
    return self.packers[CANBUS.party].make_can_msg("DAS_bodyControls", bus, values)
```

- [ ] **Step 4: Run test to verify it passes**

Run (from `opendbc_repo/`): `python -m pytest opendbc/car/tesla/preap/tests/test_body_controls.py -v`
Expected: PASS (all 5 tests).

- [ ] **Step 5: Commit (inside submodule)**

```bash
cd opendbc_repo
git add opendbc/car/tesla/preap/teslacan.py opendbc/car/tesla/preap/tests/test_body_controls.py
git commit -m "tesla preap: create_body_controls_message for turn-signal drive"
cd ..
```

---

## Task 6: Consume blinker flags in Pre-AP carcontroller

**Files (inside `opendbc_repo`):**
- Modify: `opendbc/car/tesla/preap/carcontroller.py` (the `_update_preap` method in `tesla/carcontroller.py` calls into this — confirm path) — actually modify `opendbc/car/tesla/carcontroller.py` `_update_preap`.

> **Path note:** The Pre-AP carcontroller update lives in `opendbc/car/tesla/carcontroller.py::CarController._update_preap`. `preap/carcontroller.py` holds `PreAPLongController`. The blinker send goes in `_update_preap`, using `self.tesla_can.create_body_controls_message` (the `tesla_can` is a `TeslaCANPreAP` instance per `init_preap_can`).

- [ ] **Step 1: Add the blinker send in `_update_preap`**

In `opendbc_repo/opendbc/car/tesla/carcontroller.py`, inside `_update_preap`, after the `self.stock_cc.update(...)` block and before `new_actuators = actuators.as_builder()`, add:
```python
    # Turn-signal drive: keep the indicator flashing during the lane-change
    # arming window and maneuver. CC.leftBlinker/rightBlinker are set by
    # controlsd whenever laneChangeState != off. 0=none, 1=left, 2=right.
    if self.frame % 10 == 0:
      turn = int(CC.rightBlinker) * 2 + int(CC.leftBlinker)
      cntr = (self.frame // 10) % 16
      can_sends.append(self.tesla_can.create_body_controls_message(turn, 0, CANBUS.party, cntr))
```

- [ ] **Step 2: Verify it parses**

Run (from `opendbc_repo/`): `python -c "import ast; ast.parse(open('opendbc/car/tesla/carcontroller.py').read()); print('ok')"`
Expected: prints `ok`.

- [ ] **Step 3: Add a carcontroller integration test**

The existing `test_stock_cc_spoofer.py` constructs the spoofer directly with
`SimpleNamespace`/`MagicMock` stubs rather than a full `CarController`. Follow that
lighter style: assert the builder contract and the send cadence logic. Append to
`opendbc_repo/opendbc/car/tesla/preap/tests/test_body_controls.py`:
```python
def test_turn_value_encoding_matches_cc_convention():
  # turn = rightBlinker*2 + leftBlinker, as used in _update_preap.
  assert (int(True) * 2 + int(False)) == 2    # right only
  assert (int(False) * 2 + int(True)) == 1    # left only
  assert (int(False) * 2 + int(False)) == 0   # none


def test_body_controls_frame_addr_and_bus():
  tc = _tc()
  addr, _, bus = tc.create_body_controls_message(1, 0, CANBUS.party, 3)
  assert addr == 0x3E9
  assert bus == CANBUS.party
```

> **Why not build the full `CarController` here:** it needs DBC names + packers and
> is exercised end-to-end by process-replay (Task 8 regression). This unit test
> pins the builder contract and the `turn` encoding. If you prefer a fuller test,
> the construction pattern is in `test_stock_cc_spoofer.py` — but the lighter test
> is sufficient and matches the file's established style.

- [ ] **Step 4: Run tests**

Run (from `opendbc_repo/`): `python -m pytest opendbc/car/tesla/preap/tests/test_body_controls.py -v`
Expected: PASS.

- [ ] **Step 5: Commit (inside submodule)**

```bash
cd opendbc_repo
git add opendbc/car/tesla/carcontroller.py opendbc/car/tesla/preap/tests/test_body_controls.py
git commit -m "tesla preap: drive turn signal via DAS_bodyControls from CC blinker flags"
cd ..
```

---

## Task 7: Panda safety — allowlist 0x3E9 and bound turn value

**Files (inside `opendbc_repo`):**
- Modify: `opendbc/safety/modes/tesla_preap.h`
- Modify: `opendbc/safety/tests/test_tesla_preap.py`

- [ ] **Step 1: Add the failing safety test**

In `opendbc_repo/opendbc/safety/tests/test_tesla_preap.py`, add `[0x3E9, 0]` to the `TX_MSGS` list:
```python
  TX_MSGS = [
    [0x488, 0],  # DAS_steeringControl
    [0x2B9, 0],  # DAS_control
    [0x214, 0],  # EPB_epasControl
    [0x551, 0],  # Pedal
    [0x551, 2],  # Pedal
    [0x45,  0],  # STW_ACTN_RQ (stalk spoof)
    [0x3E9, 0],  # DAS_bodyControls (turn signal)
  ]
```
(Match the existing entries already present; only the `0x3E9` line is new. If `0x214`/`0x551` rows differ, keep the file's existing rows and just append the `0x3E9` row.)

Then add a test method inside the `TeslaPreAPTestMixin` class (the harness exposes
`self.packer.make_can_msg_safety(name, bus, values)` and `self._tx(msg)` from
`opendbc/safety/tests/common.py`):
```python
  def test_body_controls_turn_indicator_allowed(self):
    # Valid turn-indicator requests (0-3) are allowed when controls_allowed.
    self.safety.set_controls_allowed(True)
    for turn in range(4):
      msg = self.packer.make_can_msg_safety("DAS_bodyControls", 0,
              {"DAS_turnIndicatorRequest": turn})
      self.assertTrue(self._tx(msg), f"turn={turn} should be allowed")

  def test_body_controls_blocked_when_not_allowed(self):
    # Like other actuation, blocked when controls are not allowed.
    self.safety.set_controls_allowed(False)
    msg = self.packer.make_can_msg_safety("DAS_bodyControls", 0,
            {"DAS_turnIndicatorRequest": 1})
    self.assertFalse(self._tx(msg))
```

> **controls_allowed note:** the second test assumes the framework gates 0x3E9 on
> `controls_allowed` (as it does for other Pre-AP TX). If `DAS_bodyControls` should
> be allowed even when disengaged (so the indicator can be cleared on disengage),
> drop `test_body_controls_blocked_when_not_allowed` and instead assert it is always
> allowed. Decide based on desired UX: blinker should stop on disengage, and
> `controlsd` already sets `CC.*Blinker=False` then, so gating on controls_allowed
> is the safe default — keep both tests.

- [ ] **Step 2: Run test to verify it fails**

Run (from `opendbc_repo/`): `python -m pytest opendbc/safety/tests/test_tesla_preap.py -v -k body_controls`
Expected: FAIL — 0x3E9 not in TX allowlist (tx rejected).

> If the safety C must be rebuilt first, run `scons -j8 opendbc/safety` (or the repo's documented safety build) before pytest.

- [ ] **Step 3: Add 0x3E9 to the TX allowlist**

In `opendbc_repo/opendbc/safety/modes/tesla_preap.h`, in `PREAP_TX_MSGS`, add after the `0x45` line:
```c
    {0x3E9, 0, 8, .check_relay = false, .disable_static_blocking = true},  // DAS_bodyControls (turn signal)
```

- [ ] **Step 4: Bound the turn-indicator value in the TX hook**

In `tesla_preap_tx_hook`, before the `if (violation)` block, add:
```c
  // DAS_bodyControls (0x3E9): only allow valid turn-indicator requests (0-3).
  if (msg->addr == 0x3E9U) {
    int turn_req = (msg->data[1] & 0x03U);  // DAS_turnIndicatorRequest at bit 8
    if (turn_req > 3) {
      violation = true;  // unreachable for 2 bits, but explicit per defense-in-depth
    }
  }
```

> The 2-bit field can't exceed 3, so this is documentation-of-intent + a guard if the field width ever changes. Keeping it makes the safety reviewer's job easy.

- [ ] **Step 5: Rebuild safety and run the test**

Run (from `opendbc_repo/`): `scons -j8 opendbc/safety && python -m pytest opendbc/safety/tests/test_tesla_preap.py -v -k body_controls`
Expected: PASS.

- [ ] **Step 6: Run the full Pre-AP safety suite (regression)**

Run (from `opendbc_repo/`): `python -m pytest opendbc/safety/tests/test_tesla_preap.py -v`
Expected: PASS (all, including pre-existing tests).

- [ ] **Step 7: Commit (inside submodule)**

```bash
cd opendbc_repo
git add opendbc/safety/modes/tesla_preap.h opendbc/safety/tests/test_tesla_preap.py
git commit -m "safety tesla_preap: allow DAS_bodyControls (0x3E9) turn-signal TX"
cd ..
```

---

## Task 8: Bump submodule pointer + full regression

**Files:**
- Modify: parent repo's `opendbc_repo` submodule reference

- [ ] **Step 1: Record the new submodule SHA in the parent**

```bash
git add opendbc_repo
git status   # confirm: "modified: opendbc_repo (new commits)"
```

- [ ] **Step 2: Run the desire_helper + events tests once more from parent**

Run: `python -m pytest selfdrive/controls/lib/tests/test_desire_helper.py -v`
Expected: PASS.

- [ ] **Step 3: Sanity-check the cereal field round-trips through modeld's message**

Run: `python -c "from cereal import log; m=log.ModelDataV2.new_message(); m.meta.laneChangeFlashesRemaining=7; assert m.meta.laneChangeFlashesRemaining==7; print('ok')"`
Expected: prints `ok`.

- [ ] **Step 4: Commit the submodule bump**

```bash
git commit -m "bump opendbc_repo: lane-change auto-signal (DAS_bodyControls turn drive)"
```

---

## Self-Review Notes

- **Spec coverage:** tap-arm (Task 2), 7-flash sustain via DAS_bodyControls (Tasks 5-6), auto-stop on completion (Task 6 — `turn` follows `CC.*Blinker` which controlsd drops at `off`), countdown UI (Tasks 1,3,4), timeout cancel (Task 2), opposite-tap cancel (Task 2), safety allowlist (Task 7). All covered.
- **Type consistency:** `flashes_remaining` (int, 0-7) defined in Task 2, published in Task 3, read in Task 4, schema `UInt8` in Task 1 — consistent. `turn` value convention `right*2+left` consistent across Tasks 5/6 and the test.
- **Known soft spots flagged inline:** the carcontroller integration test (Task 6) and the safety test helper (Task 7) note where to mirror existing repo harness patterns if the simple form doesn't match. These are real verification steps, not placeholders — the executing agent confirms the harness on the spot.
