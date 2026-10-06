# Pre-AP drive-control corrections and validation

This sunnypilot change set targets `naponsp-dev` and is based on openpilot `b688860729d5c9c1510f4902963ee119d6082040` and opendbc `f9c9d0e3bc6f5f0c44aabefa056772146e375537`. It has not been deployed or road-tested. Bench results below are not a safety certification or proof of improved road comfort.

## Behavior changes

### Grade compensation

The existing controller already compares requested and measured acceleration in the same net-acceleration domain. That part is unchanged.

The old estimator added a band-pass correction to a slower steady-grade estimate. As the steady estimate caught up, the transient could continue adding the same gravitational load. A simulated 0 → 0.8 m/s² load step reached 0.83113 m/s²; a +0.8 → −0.8 crest reached −0.86224 m/s².

The replacement uses two filtered estimates of the same gravitational load:

```
total grade compensation = 0.1 × steady estimate + 0.9 × fast estimate
```

The total stays between those estimates. The weights preserve the old small-signal response area / low-frequency delay of 0.14 seconds; they do not preserve every transient or noise characteristic. Missing orientation holds and then decays the estimate using the existing policy, with both filters aligned so reacquisition cannot revive a stale transient. Acceleration feedback, pedal authority, and output limits are unchanged.

Signed step, crest, dropout, and full closed-loop simulation checks pass. Recorded-input replay also runs the estimator successfully. Whether this resolves the reported uphill/downhill driving feel still requires a controlled comparison on the vehicle.

### Following distance

A valid live stalk detent, 1–7, reaches the planner on its next model update rather than waiting for the persisted `NAPFollowDistance` value. The observed old delay was 67.685 seconds. Replay of the corresponding full-log transitions applied the changes to the actual MPC headway in 35 ms and 29 ms, even while persistence remained at 4.

The on-screen picker still works:

- A tap submits an explicit `NAPFollowDistanceRequest` containing the selected distance, current wheel value, monotonic timestamp, and current route identity. Persistence is a separate preference write.
- The planner reads requests at its existing 20-frame cadence, approximately once a second. Asynchronous delivery can add delay; this is not a hard one-second guarantee.
- An accepted tap overrides an unchanged dial until a subsequent physical detent change.
- Old-route, pre-start, stale, or mismatched-baseline requests cannot override a newer detent.
- A tap made while the dial is unavailable can take effect; any returning valid dial then wins. An override made against a valid dial survives a temporary outage if that same dial value returns.
- Offroad changes store the next-drive fallback only. They do not create an onroad request without a route identity.

The follow-distance graphic is higher on the driving view.

### Cruise speed and native dynamic targets

The Pre-AP engagement state machine owns the manual cruise ceiling. Once initialized, it retains that ceiling through disengagement, brake-related longitudinal cancellation, and the first pull of a new engagement gesture. This is in-process retention, not a saved target across a restart or a new drive.

- Double pull resumes the retained ceiling, subject to the existing engagement and actuator-health gates.
- Two distinct first-detent down taps within the configured gesture window set the ceiling to current speed, rounded in the selected speed unit. The sunnypilot window is 750 ms.
- Setting speed with down taps does not acquire lateral or longitudinal authority.
- A held down input or deeper-detent input is not the new two-tap gesture.
- An uninitialized target remains distinct from a deliberately selected zero target.

Sunnypilot's existing Smart Cruise Control / Speed Limit Assist arbitration already supplies the effective target to MPC; no second acceleration target selector was added. During valid active dynamic control, the main set-speed display shows that effective target and the smaller `MAX` label retains the manual ceiling. Disengagement, override, stale/invalid data, or an inactive dynamic source returns the display to the manual ceiling. The actual vehicle-speed display remains actual speed.

### Hands-on pause and lane-change nudging

A deliberate double pull can be admitted while hands-on steering override is detected. Admission is not permission to steer against the driver: lateral control enters the hands-on paused state, and steering output remains inhibited until the existing fresh-clear interval is satisfied. EPAS faults, driver monitoring, brake/gear/seatbelt constraints, and cancellation are not bypassed.

A lane-change nudge can be remembered during a specifically identified hands-on pause. Lane-change desire and fade do not advance while steering is paused. Resume requires the normal clear conditions. A target-side blindspot during pause or on the first resume frame cancels the maneuver rather than reviving it later; timeout, cancellation, and conflicting input also invalidate it.

The captured configuration had hands-on pause enabled and hands-on threshold 2. This change does not lower that threshold or disable driver monitoring.

### Inverted radar and display placement

`Radar Mounted Upside Down` is off by default. It is available only offroad and requires a reboot. The radar interface snapshots it at initialization.

For an inverted mount, radar lateral position and lateral velocity are negated before lead association. Longitudinal values are unchanged. The configured lateral offset remains in vehicle coordinates: positive is left, negative is right. Do not negate that offset merely because the radar is inverted. The calibrated numbers and calibration plot are unchanged.

Native Params is authoritative for the mounting flag, including explicit false. The persisted parameter file and then legacy configuration are fallbacks only when the preceding reader is unavailable.

The radar overlay is farther right and reserves space for driver monitoring. Large layout, sidebar, left/right driver-monitoring placement, and compact layout were rendered for inspection.

## Verification performed

- Native `scons -j8` build, including schema and parameter changes: passed.
- Publication check of the Pre-AP CI suites, cruise-speed helpers, and UI state: **1,060 passed, 39 skipped, 7 deselected, 114 subtests passed**. The opendbc and parent suites ran separately with their respective pytest configurations. The earlier integrated run passed 851 tests and 123 subtests. Coverage includes Pre-AP controller and engagement, panda safety and radar contracts, host/MADS/desire transitions, planner/MPC following, cruise-speed helpers, radar processing, UI state, and remote-write policy.
- Actual state-machine smoke: initial set, cancel with retained target, resume at lower vehicle speed, two-down current-speed selection without engagement, and resume of the new target.
- Baseline/candidate grade-estimator smoke: both slope signs, return to flat, and full crest reversals; the candidate stayed within the physical endpoint bounds.
- Full-log replay through the production planner, native MPC, and cereal publication: 3,596 model updates across three recorded segments. Published following distance and actual MPC headway agreed on every update. Native dynamic target publication agreed with planner output.
- Production drawing paths rendered to images with synthetic UI inputs, including active dynamic target and retained manual ceiling.
- Ruff checks passed for all 41 changed Python files.

Not performed: deployment, road driving, a Docker CI build, a full-repository test run, or an instrument-cluster hardware test. Skipped tests are not counted as passing.

## Morning check sequence

Use only after a separately authorized installation of this exact candidate, including its opendbc and panda changes. An older panda build does not contain the new hands-on admission behavior. Keep the known-good build available for recovery. Do not combine this check with VIN programming, radar-learning changes, or instrument-cluster changes.

1. **Parked configuration:** confirm expected software versions, pedal/radar health, mounting orientation, and hands-on threshold. Leave the new inverted-mount setting off unless the hardware is actually inverted. If changed, reboot before checking tracks.
2. **Initial controls:** in a suitable controlled environment, establish a modest target. Cancel, change vehicle speed, and double pull; verify the retained target rather than recapture. While disengaged, two distinct down taps should select current speed without enabling control. Verify cancel and brake remain effective.
3. **Following selector:** change one dial detent at a time, then two quickly. Verify displayed selection and logged `longitudinalPlan.napFollowDistance` / `tFollow`. Try a picker change with the dial stationary, then move the dial and confirm the detent takes precedence. Never use a real close-following situation to test this behavior.
4. **Hands-on pause:** confirm deliberate engagement can remain visibly paused with hands detected, with no active steering request. Confirm clear/resume behavior and cancellation before evaluating lane-change nudges. Evaluate any lane change only where legal and safe; do not deliberately approach another vehicle or an occupied blindspot to test cancellation.
5. **Radar and overlays:** check stable objects on both sides of the car for the correct lateral sign and inspect driver-monitoring visibility. An offset correction cannot repair a reversed lateral axis. Stop if tracks or health flags are inconsistent.
6. **Grade comparison:** use a familiar, low-demand segment with no close lead and a steady manual ceiling. Mark entry, crest, and exit; compare requested acceleration, measured acceleration, calibrated pitch, dynamic target, limited VDAS target, and pedal DI. Stop at unwanted acceleration or oscillation rather than tuning around it during the drive.
7. **Native dynamic target:** where its conditions occur naturally, confirm the effective set-speed number falls below the retained `MAX` ceiling and returns appropriately on disengagement/override. Separate this from actual vehicle speed.

Record exact route/segment and approximate timestamp for every discrepancy, plus configuration and which control was active. Experimental mode needs a separate capture with the mode actually enabled; the existing October logs have it disabled.

### Additional direct runtime checks

Recorded CAN was also passed through upright and inverted production radar interfaces: 6,000 CAN events, 702 scans and 8,350 paired points. Lateral position reflected around the unchanged +0.25 m vehicle offset, lateral velocity changed sign, and longitudinal position/speed matched. Changing the parameter after construction did not alter the startup snapshot. This is numerical processing proof, not hardware mounting validation.

A standalone production MADS/desire scenario exercised paused admission, clear/resume, cancel, remembered nudge, blindspot cancellation during pause, and no revival after the blindspot cleared. Physical inhibition and clear-timing behavior remain subject to the safety tests and controlled vehicle checks described above.
