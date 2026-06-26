# Lane-Change Auto Signal — Design Spec

**Date:** 2026-06-25
**Car:** NAP Pre-AP Tesla Model S (`TESLA_MODEL_S_PREAP`)
**Branch:** `lane-change-auto-signal`

## Problem

Today, an automated lane change requires the driver to **hold** the turn-signal
stalk for the entire arming window. A short stalk tap flashes the indicator 3
times then auto-cancels (stock car behavior), which collapses the arming window
before the driver can nudge the wheel. The driver must physically hold the stalk
down, which is awkward.

## Goal

Replicate the Tesla-Unity / Tinkla behavior:

1. Driver **short-taps** the turn-signal stalk → lane change arms.
2. The stock car flashes 3×; openpilot **sustains the flashing** to a total of
   **7 flashes** while armed (the stock 3 plus 4 more).
3. If the driver **nudges the wheel** within those 7 flashes, the lane change
   proceeds and the blinker **keeps flashing until the maneuver completes**, then
   stops automatically.
4. If the driver does **not** nudge within 7 flashes, arming cancels and the
   blinker stops.
5. The UI shows a live countdown: **"Nudge wheel to engage lane change within N
   signals"**.

---

## Revision 2 (2026-06-26) — time budget, full reset, multi-change queue

On-car testing showed the original flash-counted design works to *hold* the
signal, but has three problems, all fixed here:

1. **Countdown never advances.** On this Pre-AP build, `BC_indicatorLStatus`
   reflects only the driver's physical stalk, NOT the openpilot-driven
   `DAS_bodyControls` flashes. So the flash counter only ticks when the driver
   moves the stalk — the countdown freezes at 7/6.
   **Fix:** count down a **7-second time budget** (`LANE_CHANGE_ARM_TIME = 7.0`)
   instead of flashes. The countdown is `ceil(time_remaining)` and ticks by
   `DT_MDL` every frame, independent of any blinker hardware feedback.

2. **Stuck toast after completion.** After the maneuver, `laneChangeFinishing`
   re-enters `preLaneChange` on a phantom `one_blinker` (the op-driven blinker),
   leaving the countdown toast stuck on screen until the driver cancels/disengages.
   **Fix:** on maneuver completion with no queued changes remaining, force a
   **full reset to `off`** (direction none, timers cleared, blinker dropped).

3. **No multi-lane-change.** **New feature:** during the arming window, count
   same-direction stalk taps. `queued_changes = min(taps, 3)`. Each lane change
   needs its **own** wheel nudge. After one completes with more queued, the signal
   **stays on**, the queue decrements, and a **fresh 7s window** opens for the next
   nudge (the timer counts only while waiting for a nudge — it pauses during the
   maneuver). A UI value shows "N lane changes left" (0–2). Full reset (signal off,
   toast clears) on: queue exhausted, 7s expiring while waiting for a nudge,
   opposite-direction tap, or blindspot/below-speed/disengage.

### Cereal field changes (Rev 2)
- Rename `laneChangeFlashesRemaining @10` → `laneChangeSignalsRemaining @10`
  (now the seconds-based countdown shown in the toast).
- Add `laneChangeRemaining @11 :UInt8` (queued lane changes left, 0–2 after the
  first is in progress; up to 2 while waiting).

### State machine (Rev 2) — `desire_helper.py`
- Constant: `LANE_CHANGE_ARM_TIME = 7.0` (replaces `LANE_CHANGE_FLASH_BUDGET`).
- New state: `arm_timer` (counts up while waiting for a nudge in `preLaneChange`),
  `queued_changes` (taps seen, capped 3), `signals_remaining` (ceil of time left),
  `lane_changes_remaining` (queue minus the one in progress).
- `preLaneChange` waiting: `arm_timer += DT_MDL`; cancel to full `off` when
  `arm_timer > LANE_CHANGE_ARM_TIME`. Count same-direction taps into
  `queued_changes` (cap 3). Opposite tap → full reset. Wheel nudge →
  `laneChangeStarting` (consumes one from the queue; reset `arm_timer`).
- `laneChangeFinishing` completion: if `queued_changes` (remaining) > 0 → return
  to `preLaneChange`, keep the same direction & signal on, open a fresh window
  (`arm_timer = 0`). Else → **full `off` reset**.
- "Keep signal on between maneuvers": the between-change wait runs through
  `preLaneChange` with the direction retained, so `laneChangeState != off` and
  `controlsd` keeps asserting `CC.leftBlinker/rightBlinker`.

### UI (Rev 2)
- Countdown alert reads `laneChangeSignalsRemaining`: "Nudge wheel {left/right}
  to change lane within N signals". When `laneChangeRemaining > 0`, append the
  queue count, e.g. a second line / suffix "(N more queued)".

The carcontroller / teslacan / panda-safety layer is **unchanged** in Rev 2 — it
already drives the blinker from `CC.leftBlinker/rightBlinker`, which follow
`laneChangeState != off`.

---

## Verified Mechanism (from reading the code + `origin/tesla-unity`)

### Blinker READ (already correct)
`opendbc_repo/opendbc/car/tesla/preap/carstate.py:99-100`:
```python
ret.leftBlinker  = GTW_carState.BC_indicatorLStatus == 1
ret.rightBlinker = GTW_carState.BC_indicatorRStatus == 1
```
Reads the *actual indicator light*. A stock tap reads True during the 3 flashes,
then False. Rising edges of these flags are the flash counter source.

### Blinker DRIVE (the missing piece)
Tinkla drove the Pre-AP blinker via **`DAS_bodyControls`** (CAN `0x3E9` / 1001),
field `DAS_turnIndicatorRequest` (`0=NONE, 1=LEFT, 2=RIGHT, 3=CANCEL`).
Verified in `origin/tesla-unity`:
- `selfdrive/car/tesla/HUD_module.py:259-264` sends
  `create_body_controls_message(alca_direction, ...)`; the send gate explicitly
  includes `carFingerprint == CAR.PREAP_MODELS`.
- `selfdrive/car/tesla/carcontroller.py:83`:
  `CS.alca_direction = CC.rightBlinker * 2 + CC.leftBlinker`.

`DAS_bodyControls` (BO_ 1001) exists in NAP `tesla_preap.dbc`. It is **not** in
the current NAP panda TX allowlist (`PREAP_TX_MSGS`) — must be added.

### Existing-but-dead plumbing
`controlsd.py:103-106` already sets `CC.leftBlinker`/`rightBlinker` during the
lane-change maneuver, but the NAP Tesla carcontroller (`tesla/carcontroller.py`
and `tesla/preap/carcontroller.py`) **never consumes** these flags — confirmed by
grep (every other brand's carcontroller reads them; Tesla's does not). So the
flag is set and silently dropped. This is why "blinker during lane change" appears
implemented but doesn't work. Wiring a consumer in `preap/carcontroller.py` makes
the maneuver-time flashing (and auto-stop on completion) work essentially for free.

### Short-tap vs hold (Tinkla insight)
`origin/tesla-unity selfdrive/car/tesla/carstate.py:405`:
```python
ret.leftBlinker = (BC_indicatorLStatus==1) and (turnSignalStalkState==0) and (tap_direction==1)
```
A **tap** = light flashing while the stalk lever (`STW_ACTN_RQ.TurnIndLvr_Stat`,
read-only) has already returned to idle (0). `tap_direction` latches the armed
direction. We replicate the latch in `desire_helper`, keyed on the existing
`one_blinker` rising edge (which a tap produces).

### Safety
`opendbc_repo/opendbc/safety/modes/tesla_preap.h`: the TX hook does not currently
touch `0x3E9`. We add `0x3E9` to `PREAP_TX_MSGS` and add a TX-hook bound so
`DAS_turnIndicatorRequest` is limited to valid values (0–3). All other safety
checks unchanged. `controls_allowed` gating still applies (framework-level).

## Design

### State machine (`selfdrive/controls/lib/desire_helper.py`)

New constants:
- `LANE_CHANGE_FLASH_BUDGET = 7` — total flashes allowed in the arming window.

New state on `DesireHelper`:
- `lane_change_flashes_seen: int` — rising edges of the real blinker counted
  while in `preLaneChange`.
- `flashes_remaining: int` — `max(LANE_CHANGE_FLASH_BUDGET - lane_change_flashes_seen, 0)`,
  published for the UI.
- `prev_blinker_on: bool` — for edge detection.

Behavior changes:

**Entering `preLaneChange`** (unchanged trigger: `one_blinker` rising edge, not
below speed): latch `lane_change_direction`; reset `lane_change_flashes_seen = 0`.

**While in `preLaneChange`:**
- Count a flash on each rising edge of the *armed-direction* blinker
  (`carstate.leftBlinker` if armed left, else right).
- Compute `flashes_remaining`.
- **Exit → `laneChangeStarting`** when `torque_applied and not blindspot_detected`
  (existing wheel-nudge check).
- **Exit → `off`** when `flashes_remaining <= 0` (budget exhausted, no nudge).
- **Exit → `off`** when below lane-change speed or lateral disengaged (existing).
- **Opposite-direction tap** (a fresh `one_blinker` rising edge in the opposite
  direction): cancel → `off` (driver must tap again to re-arm; no auto re-arm).
- The arming latch **no longer requires `one_blinker` to remain true** — this is
  the core fix. A momentary tap arms; the latch holds for up to 7 flashes.

**During the maneuver** (`laneChangeStarting`/`laneChangeFinishing`): blinker
keeps flashing until `laneChangeState` returns to `off`. The 7-flash budget governs
**only** the pre-nudge arming window, not the maneuver. (Falls out of existing
`controlsd` logic.)

### Cereal (`cereal/log.capnp`)
Add to `ModelDataV2.MetaData`:
```capnp
laneChangeFlashesRemaining @2 :UInt8;
```
Append-only (next index) → safe schema evolution.

### Model process (`selfdrive/modeld/modeld.py`)
After `DH.update(...)`, publish:
```python
modelv2_send.modelV2.meta.laneChangeFlashesRemaining = DH.flashes_remaining
```

### Controls (`selfdrive/controls/controlsd.py`)
**No change required.** The existing condition at `controlsd.py:104`
(`if model_v2.meta.laneChangeState != LaneChangeState.off`) already covers
`preLaneChange` (which is `!= off`), and `desire_helper` already sets
`lane_change_direction` on entering `preLaneChange` (`desire_helper.py:61`). So
`CC.leftBlinker`/`rightBlinker` is already asserted throughout the arming window
and the maneuver. The only reason it doesn't flash today is the missing
carcontroller consumer (below).

### Tesla carcontroller (`opendbc_repo/.../tesla/preap/carcontroller.py`)
New consumer in `_update_preap`, sent on a fixed cadence (e.g. every 10 frames,
matching Tinkla's body-controls rate):
```python
turn = CC.rightBlinker * 2 + CC.leftBlinker   # 0 none, 1 left, 2 right
if self.frame % 10 == 0:
    can_sends.append(self.tesla_can.create_body_controls_message(turn, 0, CANBUS.party, counter))
```

### Tesla CAN (`opendbc_repo/.../tesla/preap/teslacan.py`)
Port `create_body_controls_message` (DAS_bodyControls, 0x3E9) with the NAP
checksum convention. Set `DAS_turnIndicatorRequest = turn`,
`DAS_turnIndicatorRequestReason = 1 if turn>0 else 0`, counter, checksum.

### Panda safety (`opendbc_repo/opendbc/safety/modes/tesla_preap.h`)
- Add to `PREAP_TX_MSGS`:
  `{0x3E9, 0, 8, .check_relay = false, .disable_static_blocking = true}`.
- In `tesla_preap_tx_hook`, bound `0x3E9`: reject if
  `DAS_turnIndicatorRequest > 3` (defense-in-depth; valid values only).

### Alerts (`selfdrive/selfdrived/events.py`)
Replace the static `preLaneChangeLeft`/`preLaneChangeRight` `Alert` objects with
dynamic callbacks (`AlertCallbackType`, signature `func(CP, CS, sm, metric,
soft_disable_time, personality) -> Alert`). The callback reads
`sm['modelV2'].meta.laneChangeFlashesRemaining` and formats:
> "Nudge wheel to engage lane change within {N} signals"

Keep the existing direction-specific phrasing where helpful, e.g.
"Nudge wheel left to change lane within {N} signals".

## Testing

- **`desire_helper` unit tests:** arm on tap; count 7 flashes; cancel at budget
  exhaustion without nudge; proceed on wheel nudge; blinker persists through
  maneuver; opposite-tap cancels; below-speed and blindspot exits preserved.
- **`preap/carcontroller` + `teslacan` tests:** `CC.leftBlinker` → correct
  `DAS_bodyControls` frame (turn value, reason, checksum, counter, cadence).
- **Panda safety test:** `0x3E9` accepted when `controls_allowed` with valid
  `DAS_turnIndicatorRequest`; rejected when value > 3.
- **Process replay / regression:** existing lane-change replay still passes.

## Constants Summary

| Constant | Value | Location |
|---|---|---|
| `LANE_CHANGE_FLASH_BUDGET` | 7 | `desire_helper.py` |
| body-controls TX cadence | every 10 frames (~10 Hz) | `preap/carcontroller.py` |
| `DAS_bodyControls` addr | `0x3E9` (1001) | DBC / safety |

## Out of Scope

- Non-Pre-AP Tesla variants (AP1/AP2/HW3) — they have a DAS ECU and different
  blinker handling; this spec targets `TESLA_MODEL_S_PREAP` only.
- Hazard lights, headlight/wiper body controls (the DAS_bodyControls message
  carries them, but we send 0 / leave them stock).
