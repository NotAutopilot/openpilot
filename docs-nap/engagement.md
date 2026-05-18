# Engagement Flow

Pre-AP has two modes depending on whether a Comma Pedal is installed. Mode is selected by `nap_conf.use_pedal` (NAP settings panel, persisted via Params). Both modes share the same stalk FSM in `engagement.py`.

## State vocabulary

- `cruiseEnabled` — NAP is engaged at the FSM level (lateral on)
- `enableLongControl` — NAP is actively modulating longitudinal (pedal mode only)
- `enableJustCC` — NAP is lateral-only, stock CC is (or will be) driving speed
- `pedal_long_allowed = use_pedal AND pedal_transform_valid` — pedal hardware is usable
- `long_control_allowed = (not use_pedal) OR pedal_transform_valid` — NAP's FSM may transition into the "cruise" state (pedal mode requires a valid pedal transform)

## Pedal mode

`use_pedal=True`, pedal hardware present:

1. Driver double-pulls stalk → `cruiseEnabled=True`, `enableLongControl=True`
2. `carcontroller.py` sends `GAS_COMMAND` (0x551) based on `actuators.accel` from the long planner
3. Zero-torque learning tracks the resting pedal position that produces zero torque at the current speed. On engage, we seed `prev_pedal_di` to this value so there is no regen spike
4. Brake rising edge → drops `enableLongControl=False`, keeps `cruiseEnabled=True` (steering stays on, pedal drops). `enableJustCC` flips true so the carcontroller knows to spoof stock CC cancel
5. Gas press → `OVERRIDE_LONGITUDINAL`. Pedal command passes through with `enable=0` — driver's foot controls throttle directly, NAP tracks position for smooth resume
6. Stalk cancel (real, not spoof echo) → full disengage

Interface flags for this mode: `openpilotLongitudinalControl=True`, `pcmCruise=False`. Long planner runs; accel goes to pedal.

## No-pedal mode

`use_pedal=False`, no pedal hardware:

1. Driver double-pulls stalk → `cruiseEnabled=True`, `preap_cc_engage_needed=True`
2. `carstate` publishes `ret.cruiseState.enabled=True`
3. selfdrived sees this with `pcmCruise=True` → engages op lateral
4. `carcontroller` sends `RES_ACCEL` stalk spoof via `0x45` → stock Tesla CC engages and holds current speed
5. `CC.longActive=False` always (`openpilotLongitudinalControl=False`) — no planner accel is produced or consumed
6. Stalk cancel → `preap_cc_cancel_needed=True` → carcontroller spoofs `CANCEL` stalk → stock CC drops
7. Driver's own stalk +/- buttons adjust stock CC speed directly (stock CC is on the same CAN bus)

Interface flags for this mode: `openpilotLongitudinalControl=False`, `pcmCruise=True`. Mirrors the honda/hyundai/volkswagen pattern.

## Stalk spoofing and echo filters

NAP both reads the real stalk and sends spoofed stalk messages. This creates an echo risk: our own spoofed message could be read back as a real driver press. Two windows suppress this:

- `CANCEL_ECHO_WINDOW_MS = 600` — ignore `CANCEL` events within 600 ms of our last non-cancel stalk activity (so auto-cancel echoes from engage spoofs don't look like the driver pressing cancel)
- `SPOOF_ECHO_WINDOW_MS = 300` — ignore `CANCEL` events within 300 ms of our last explicit spoof send

Both tuned on real drive logs. If you see stale "driver canceled" events in logs, these are the knobs.

## Double-pull vs single-pull

Single stalk pull only engages lateral if `enableDoublePull` is off. In the default double-pull mode:

- First pull → pending engage state, lateral-only, window open
- Second pull within `double_pull_window_ms` → full engage (pedal or stock CC depending on mode)
- Window expires → stays lateral-only

This is a driver-training feature: prevents accidental full engage from a bump of the stalk.

## Steering disengage

`handle_steering_disengage()` runs on every frame. Rising edge of `steeringDisengage` (hands-on level ≥ 3 or EPAS rejecting) fully tears down FSM state: `cruiseEnabled=False`, `enableLongControl=False`, `pedal_speed_kph=0`, pull timers cleared.

This is the primary user override path. The panda also enforces it independently via EPAS error codes 6–9 and hands-on level ≥ 3.

## Where to look

- `opendbc_repo/opendbc/car/tesla/preap/engagement.py` — stalk FSM
- `opendbc_repo/opendbc/car/tesla/preap/carstate.py:120+` — button event pump + cruise state publish
- `opendbc_repo/opendbc/car/tesla/preap/carcontroller.py:40+` — pedal TX + stalk spoof scheduling
- `opendbc_repo/opendbc/car/tesla/preap/tests/test_preap_engagement.py` — FSM tests (brake drop, steering disengage, double-pull)
