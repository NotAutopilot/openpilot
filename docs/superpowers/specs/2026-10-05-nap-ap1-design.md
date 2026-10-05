# NAP-AP1: AP1 Model S support + dual-car profiles

**Date:** 2026-10-05
**Branch:** `nap-ap1` (openpilot, cut from `origin/nap-dev` @ `604d9e01c`); `nap-ap1` (opendbc, cut from `nap-dev` @ `851e9e1c`)
**Status:** Design approved in brainstorming; awaiting written-spec review

## 1. Goal

Run one comma device in two cars without manual reconfiguration:

- **Pre-AP Model S** (current NAP target): comma pedal, flashed EPAS, retrofit Bosch radar with emulation.
- **AP1 Model S** (new): stock AP1 DAS (Mobileye) intercepted by an AP1 harness; openpilot steers via `DAS_steeringControl` and controls speed via `DAS_control`; factory Bosch radar tapped onto comma bus 1.

The device detects which car it is in at ignition, selects the matching platform and panda safety mode, and restores that car's calibration, learned vehicle parameters, and a short list of driving preferences. A manual car-type selector overrides detection.

### Non-goals (this milestone)

- Instrument-cluster integration (Tesla Unity `HUD_module`: DAS_status/lane/lead spoofing).
- Speed-limit-following set speed, Tesla GPS feed, HAO, "AP disabled → run like Pre-AP" mode.
- Model X AP1, HW2/HW3 legacy platforms (code stays as-is; not validated).
- Rebasing NAP's opendbc onto upstream / `xnor-tech/opendbc@master-xnor` wholesale.
- Stock Autosteer coexistence while openpilot is enabled (see §4.6).
- VIN-keyed profiles (profile = car type; the user owns one car of each type).

## 2. Research findings (why the design looks like this)

### 2.1 Where Lukas' AP1 code lives

| Source | What it is | Use |
|---|---|---|
| `lukasloetkolben/openpilot@tesla-master` (`6ba00e50a`) | comma master + "flatten opendbc" + "Model Y Juniper 2026". **No AP1 code.** | None |
| `lukasloetkolben/openpilot@tesla_unity_betaC3` (`994fcb2af`), mirrored in NAP as `origin/tesla-unity` (`5fc42b869`) | Tesla Unity v0.9.6 (Tinkla lineage) on comma 3; supports AP1/AP2/Pre-AP; old `selfdrive/car/tesla` architecture (`ACC_module`, `PCC_module`, `HUD_module`, `LONG_module`) | Feature reference only; not mergeable |
| `xnor-tech/opendbc@master-xnor` (`3f86206a`, 2026-08-14) + `xnor-tech/openpilot@xnor-dev` (`44be03578`) + `xnor-tech/panda@master-xnor` (`fa83a40a`) | Lukas' org (NAP's `upstream` remote). Modern "Tesla legacy" port: Model S/X HW1/HW2/HW3, `tesla_legacy` safety mode, `TeslaCANRaven`. HW1 test route `9227f2c54e175788/00000000--650ad7d3e0`. | **Primary source for the AP1 port** |

### 2.2 NAP already contains the modern AP1 port, but it is unreachable and has regressed

NAP's opendbc (`851e9e1c`) has an April-2026 snapshot of the xnor legacy port (`TESLA_MODEL_S_HW1`, `opendbc/safety/modes/tesla_legacy.h`, `opendbc/car/tesla/teslacan_legacy.py`). It is never selected because `NAPForcePreAP` (default `"1"`, forced on by `system/manager/manager.py`, greyed out in the UI) makes `car_helpers.fingerprint()` pin `TESLA_MODEL_S_PREAP` and skip the FW query.

Divergences from Lukas' current code that matter for AP1:

1. **`rx_all` resend hook (NAP-only regression).** `tesla_legacy.h` gained `.rx_all = tesla_legacy_handle_forwarding` (copied from the Pre-AP mode's GTW emulation). It manually `can_send`s *every* bus-2 frame to bus 0 with `skip_tx_hook=true`, including `0x488` `DAS_steeringControl` and `0x2B9` `DAS_control`, which `tesla_legacy_fwd_hook` is supposed to block. Consequences on an AP1 car: the AP ECU's steering/ACC commands reach the EPAS/DI alongside openpilot's, and every non-blocked frame is duplicated on bus 0. Lukas' version has no `rx_all`.
2. **AP-ECU parsers on the wrong bus (blocker).** NAP's `CarState.get_can_parsers` reads `DAS_control` / `DAS_steeringControl` for HW1 on bus 0 ("HW1: redirect AP/PT parsers to Bus 0"); Lukas reads them on bus 2 (`CANBUS.autopilot_party`). With the relay open, AP-ECU frames are received on bus 2 only. Because NAP registers these messages explicitly, `CANParser` marks a never-received message invalid (`opendbc/can/parser.py` `MessageState.valid`: no timestamps → invalid), so this is a **permanent CAN error**, not just missed `stockAeb` / `stockLkas`.
2a. **Both seatbelt messages required (blocker).** NAP's explicit legacy chassis list registers both `SDM1` (`0x201`, Model S) and `RCM_status` (`0x211`, Model X / no-SDM1). A car sends one or the other, so the missing one is another permanent CAN error. Lukas' dynamic parsers only register the one `update_legacy` reads (selected by `TeslaLegacyParams.NO_SDM1`).
3. **Missing Lukas fixes:** jerk-limit ramp to zero on gas override in `TeslaCANRaven.create_longitudinal_command` (`JERK_RAMP_RATE`); Model X HW1 blinker from `STW_ACTN_RQ`; legacy CAN ignition moved to `opendbc/safety/ignition.h` (NAP's panda `master-nap` already handles `0x348` `GTW_status` in `board/drivers/can_common.h`, so this one is **not** needed); Bosch 8 Hz radar "hold last RadarData" fix (`ddb0f717`): evaluate against NAP's `card`/`radard` (Pre-AP already runs the Bosch radar on NAP, so likely not needed).
4. **`tesla_can.dbc`** differs by ~1,150 lines even ignoring whitespace. A semantic comparison (both files parsed with `opendbc.can.dbc.DBC`; addresses, names, sizes, and every signal's start/size/sign/factor/offset/endianness) found them **identical** apart from one `VAL_` table for `UI_mapSpeedLimit`, which the HW1 path never reads. No DBC change.
5. **Safety flag numbering differs** (NAP `FLAG_HW1 = 4`, Lukas `= 8` because upstream took bit 2 for `FSD_14`). Keep NAP's numbering.

### 2.3 Profile-relevant facts

- Every `NAP*` setting is only read on the Pre-AP code path, except `NAPRadarIgnoreHwFail`, which `radar_interface.py` reads for any platform.
- `paramsd`, `torqued`, and `lagd` discard learned values when `CarParamsPrevRoute.carFingerprint != CP.carFingerprint`. `calibrationd` never checks the car, so after a swap it starts with the other car's calibration.
- `calibrationd`, `paramsd`, `torqued`, and `lagd` all block on `params.get("CarParams", block=True)` before reading their persisted keys. `CarParams` is `CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION`. `card` writes it after fingerprinting. **So anything `card` does between fingerprinting and writing `CarParams` happens-before every learner's load.**
- Moving the comma between cars is a power cycle, so `CarParamsCache` (`CLEAR_ON_MANAGER_START`) never leaks between cars.
- `Params(d=<dir>)` (`common/params_pyx.pyx`) accepts an alternate directory with the same key schema, so profile stores need no new key declarations. The first implementation task verifies that it creates the directory layout on demand on-device.
- EPAS FW cannot discriminate the cars: Pre-AP retrofits commonly run AP1 EPAS firmware. AP-ECU traffic can: an AP1 car's DAS continuously sends `0x488` (50 Hz) and `0x2B9` (~25 Hz); a Pre-AP car has no AP ECU, and nothing NAP sends (pedal on bus 2, radar emulation, `0x3E9`) exists before a safety mode is set.

## 3. Repositories and branches

| Repo | Branch | Base | Notes |
|---|---|---|---|
| `NotAutopilot/openpilot` | `nap-ap1` | `origin/nap-dev` `604d9e01c` | `.gitmodules` opendbc `branch = nap-ap1` |
| `NotAutopilot/opendbc` | `nap-ap1` | `nap-dev` `851e9e1c` | All car/safety changes |
| `NotAutopilot/panda` | none planned | `master-nap` `282fae70` | Create `nap-ap1` only if a firmware change proves necessary |

- `nap-ap1` is a side branch (same policy as `nap-table`): it changes panda safety, so nothing goes to `nap-release` until validated in both cars.
- Development happens in the main checkout switched to `nap-ap1`. A separate worktree was tried and dropped: on Windows it checks out tracked symlinks as text files (breaking capnp) and lacks the WSL venv, generated radar DBCs, and built libsafety that the main checkout already has.
- Reference remotes: `lukas` (`lukasloetkolben/openpilot`) in the parent; `xnor-tech/opendbc` fetched as `xnor/master-xnor` in `opendbc_repo`.

## 4. Part A: AP1 core drive (opendbc `nap-ap1`)

Platform: `TESLA_MODEL_S_HW1` (unchanged `CarSpecs`, `tesla_can` DBCs, `tesla_radar_bosch_generated` radar DBC, safety `teslaLegacy` + `FLAG_HW1`, openpilot longitudinal always on).

Harness assumption (AP1 harness at the DAS connector + radar tap):

| Bus | Traffic |
|---|---|
| 0 | Car chassis CAN (EPAS, DI, ESP, GTW, STW) |
| 1 | Factory Bosch radar CAN (tracks `0x310+`) |
| 2 | AP1 DAS ECU (`0x488`, `0x2B9`, `0x399` DAS_status, `0x3E9` DAS_bodyControls, ...) |

### 4.1 Safety: `opendbc/safety/modes/tesla_legacy.h`

1. Remove `tesla_legacy_handle_forwarding` and `.rx_all`, plus the Tinkla byte macros and `can_send` / `can_set_checksum` forward declarations that only it used. Forwarding is then entirely `tesla_legacy_fwd_hook`:
   - block bus-2 `0x488` unless stock LKAS passthrough (`tesla_legacy_stock_lkas`);
   - block bus-2 `0x2B9` (HW1) unless stock AEB (`tesla_legacy_stock_aeb`);
   - block bus-2 `0x27D` (HW2/3 internal only; unchanged).
2. HW1 only: add `0x3E9` `DAS_bodyControls` to `TESLA_TX_LEGACY_HW1_MSGS` (`check_relay = false`, matching Pre-AP). TX hook: violation unless `controls_allowed`. `DAS_turnIndicatorRequest` is a 2-bit field (byte 1 bits 0–1), so no range check is needed (a `> 3` comparison would be dead code that cppcheck flags).
3. HW1 only: `fwd_hook` blocks bus-2 `0x3E9` while `controls_allowed`, forwards it otherwise.
4. Keep NAP flag values (`FLAG_EXTERNAL_PANDA = 2`, `FLAG_HW1 = 4`, `FLAG_HW2 = 8`, `FLAG_HW3 = 16`); adopt Lukas' MISRA/type cleanups only where they don't renumber flags.
5. Tests (`opendbc/safety/tests/test_tesla_hw1.py`): bus-2 `0x488` / `0x2B9` blocked when no stock LKAS/AEB and forwarded when present; `0x3E9` TX rejected when not `controls_allowed`; bus-2 `0x3E9` blocked only while `controls_allowed`; no frame is emitted on bus 0 by the RX path (regression test for the removed hook). Rebuild libsafety before running (`scons -C opendbc_repo -D opendbc/safety/tests/libsafety`).

### 4.2 CAN parsers and car state: `opendbc/car/tesla/carstate.py`

1. HW1 parsers: AP-ECU messages on bus 2 (`ap_party` and `ap_pt` parsers on `CANBUS.autopilot_party`, which is never mutated at runtime), chassis on bus 0, as in Lukas' version. Keep NAP's explicit per-DBC message lists (they fixed CI) but correct the bus.
1a. All legacy platforms: register `SDM1` or `RCM_status`, not both, following `TeslaLegacyParams.NO_SDM1`.
1b. Register `DAS_bodyControls` with frequency `nan` (`ignore_alive`): it is only read to carry its light/wiper fields, so its absence must not raise a CAN error.
2. Add `STW_ACTN_RQ` to the HW1 chassis parser (HW1 only; HW2/HW3 untouched). Set `ret.turnSignalStalkState` from `TurnIndLvr_Stat` (`3`/SNA → `0`), mirroring `preap/carstate.py`, so `desire_helper.py`'s lever-based tap arming works unchanged.
3. Port the Model X HW1 blinker source (`STW_ACTN_RQ.TurnIndLvr_Stat`) for parity with Lukas (not validated).
4. Keep `self.das_body_controls = copy.copy(cp_ap_party.vl["DAS_bodyControls"])` (bus 2) for §4.4.

### 4.3 CAN builders: `opendbc/car/tesla/teslacan_legacy.py` and `values.py`

1. Port Lukas' jerk ramp: `create_longitudinal_command(..., gas_pressed)`; jerk limits drop to 0 while gas is pressed and ramp back by `CarControllerParams.JERK_RAMP_RATE = JERK_LIMIT_MAX * 0.002` m/s³ per call (Lukas' value). Update the call sites in `carcontroller.py`. `TeslaCANRaven` is shared with HW2/HW3, which pick up the same change (it is Lukas' current behavior for them too).
2. Add `create_body_controls(src, turn, counter)`: start from the AP ECU's latest `DAS_bodyControls` values (`src`), override `DAS_turnIndicatorRequest = turn` and `DAS_turnIndicatorRequestReason = 1 if turn else 0` (same as the Pre-AP builder), set the counter, recompute the checksum (same byte-sum checksum as the Pre-AP builder). Headlight, high/low-beam decision, and wiper fields pass through untouched.
3. `tesla_can.dbc`: no change (see §2.2 item 4).

### 4.4 Controller: `opendbc/car/tesla/carcontroller.py` (HW1 branch of `update`)

- Steering, `DAS_control` cadence, and cancel logic are unchanged apart from the jerk-ramp argument.
- Every 10 frames, while `CC.enabled`: `turn = int(CC.rightBlinker) * 2 + int(CC.leftBlinker)`; send `create_body_controls(CS.das_body_controls, turn, counter)` on bus 0, where `counter` continues from the AP ECU's last counter so the body controller sees a continuous sequence. Not sent when disengaged (the AP ECU's own frames are forwarded then).

### 4.5 Radar

- AP1 radar is driven by the car; openpilot only listens on bus 1 via the existing NAP `RadarInterface` (Bosch track lifecycle, table-freeze watch). No emulation, no donor VIN.
- `NAPRadarIgnoreHwFail`, `radar_offset`, and `radar_upside_down` apply to `TESLA_MODEL_S_PREAP` only; HW1 behaves as if they were unset.
- Evaluate Lukas' `ddb0f717` (hold last `RadarData` between 8 Hz Bosch triggers) against NAP's `card`/`radard`; port only if `liveTracks` frequency checks fail on AP1.

### 4.6 Engagement and stock AP behavior

- Engagement is `pcmCruise`: one cruise-stalk pull starts stock TACC (DI `cruiseState` ENABLED), openpilot engages and replaces DAS steering and ACC commands.
- **Known limitation:** while openpilot is enabled (legacy safety mode active, relay open), stock Autosteer cannot steer because its `0x488` is blocked. Stock LKAS emergency steering and stock AEB keep their passthrough. Using stock Autosteer requires turning off "Enable openpilot" (passive → `noOutput`, relay closed). On-car validation records any Autosteer-unavailable chimes or IC faults this causes.

## 5. Part B: car detection and profiles

### 5.1 Car-type setting

- New param `NAPCarType` `{PERSISTENT, INT, "0"}`: `0 = Auto`, `1 = Pre-AP`, `2 = AP1`.
- **Unset means "not a NAP device":** `car_helpers` reads it with `Params().get("NAPCarType")` (no `return_default`), and an unset or unreadable value leaves stock fingerprinting untouched. `system/manager/manager.py` writes `0` when it is unset, so devices default to Auto, while process replay and tests (fresh params, no manager) keep fingerprinting other brands normally. Out-of-range values are treated as Auto.
- `NAPForcePreAP` is retired: remove the forced write in `system/manager/manager.py`, its use in `car_helpers.py`, and the UI writes (`nap.py` build and reset-to-defaults, `mici/.../nap.py`); keep the key declared in `common/params_keys.h` and the `NAPParamKeys.FORCE_PRE_AP` constant for one release so old values don't error, then delete them.
- `opendbc/car/tesla/preap/nap_params.py`: add `NAPParamKeys.CAR_TYPE = "NAPCarType"` (default `0` in `DEFAULTS`) and `NAPParamKeys.ACTIVE_PROFILE = "NAPActiveProfile"` (state, not a setting: not in `DEFAULTS`); drop `FORCE_PRE_AP` from `DEFAULTS`.
- Replace `NAPForcePreAP: True` with `NAPCarType: 1` in `selfdrive/test/process_replay/test_processes.py`, `test_processes_unit.py`, and `.github/ci/process_replay_staged_inventory.json`. Replay of Pre-AP routes is unaffected either way: those logs have `fingerprintSource = fixed`, so replay sets `FINGERPRINT` and the NAP block is skipped.

### 5.2 Detection: `opendbc/car/tesla/nap_detect.py` (new)

```python
AP_ECU_ADDRS = (0x488, 0x2B9)  # DAS_steeringControl, DAS_control

def detect_legacy_platform(finger: dict[int, dict[int, int]]) -> str:
  """TESLA_MODEL_S_HW1 when the AP1 DAS ECU is on the harness, else TESLA_MODEL_S_PREAP."""
  seen = set(finger.get(0, {})) | set(finger.get(2, {}))
  return "TESLA_MODEL_S_HW1" if all(a in seen for a in AP_ECU_ADDRS) else "TESLA_MODEL_S_PREAP"
```

`car_helpers.fingerprint()` (replacing the `NAPForcePreAP` block):

| `NAPCarType` | Behavior | `fingerprintSource` |
|---|---|---|
| `1` | Pin `TESLA_MODEL_S_PREAP`, skip FW query (as today) | `fixed` |
| `2` | Pin `TESLA_MODEL_S_HW1`, skip FW query | `fixed` |
| `0` | Skip FW query, run the existing `can_fingerprint()` window, then `detect_legacy_platform(finger)` | `can` |

Log `{"event": "nap_car_detect", "mode", "result", "ap_ecu_addrs_seen"}` via `carlog`. Both buses are checked because, with the relay closed during fingerprinting, AP-ECU frames can appear on bus 0 as well as bus 2.

### 5.3 Profiles: `selfdrive/car/nap_profiles.py` (new)

```python
PROFILE_ROOT = "/data/nap_profiles"
PROFILE_FOR_PLATFORM = {"TESLA_MODEL_S_PREAP": "preap", "TESLA_MODEL_S_HW1": "ap1"}

LEARNED_KEYS = ("CalibrationParams", "LiveParametersV2", "LiveParameters",
                "LiveTorqueParameters", "LiveDelay", "CarParamsPersistent")
PREF_KEYS = ("ExperimentalMode", "LongitudinalPersonality", "IsLdwEnabled", "IsMetric")

def switch_profile(live: Params, platform: str, root: str = PROFILE_ROOT) -> str | None:
  ...
```

`switch_profile` algorithm (`new = PROFILE_FOR_PLATFORM.get(platform)`; `active = live.get("NAPActiveProfile")`, or, when that is unset (first boot of this feature), the profile of the last route's `CarParamsPersistent.carFingerprint`):

0. `active` still unknown (no profile and no usable last-route CarParams): adopt the live keys as the new car's, i.e. only set `NAPActiveProfile = new`. This keeps an existing device's Pre-AP calibration on its first `nap-ap1` boot instead of wiping it.
1. `new is None` or `new == active`: no copying (set `NAPActiveProfile` if it was inferred), return `new`.
2. If `active`: for each key in `LEARNED_KEYS + PREF_KEYS`, copy live → `Params(f"{root}/{active}")`; if absent live, remove from the store.
3. `live.put("NAPActiveProfile", new)` (before the restore, so a crash mid-restore never overwrites a saved profile with mixed data).
4. Restore from `Params(f"{root}/{new}")`: `LEARNED_KEYS`: put if stored, else `live.remove(key)` (a car seen for the first time starts with fresh learners); `PREF_KEYS`: put if stored, else leave the live value (preferences carry forward).
5. Return `new`.

All writes use `put(..., block=True)` so they are on disk before `card` writes `CarParams` (the default `put` is asynchronous). Any exception: `cloudlog.exception`, leave live keys as they are, return `None`. Never blocks `card` startup.

New param `NAPActiveProfile` `{PERSISTENT, STRING}`.

**Integration in `selfdrive/car/card.py` `Car.__init__`:** call `switch_profile(self.params, self.CP.carFingerprint)` immediately after `get_car(...)` returns (only when `CI is None`, i.e. live, not replay/tests that inject `CI`), before the existing `CarParamsPersistent → CarParamsPrevRoute` copy and the `CarParams` write. The restored `CarParamsPersistent` then becomes `CarParamsPrevRoute`, so `paramsd`/`torqued`/`lagd` see a matching fingerprint and keep the restored values; `calibrationd` reads the restored `CalibrationParams` after `CarParams` appears. `card`'s own `IsMetric` / `ExperimentalMode` reads already happen after this point.

Offroad behavior: the profile changes only when a car is detected at ignition. Settings changed offroad before the first ignition in the other car are attributed to the previously driven car; the NAP panel's status line makes this visible.

### 5.4 Startup alert

`selfdrive/car/nap_profiles.py` provides `car_label(CP)` → `"Pre-AP Model S"` / `"AP1 Model S"` plus `"(auto)"` when `CP.fingerprintSource == "can"`, else `"(forced)"`; `""` for other platforms. `selfdrive/selfdrived/events.py` uses it as the second line of `EventName.startup` (`"NAP: AP1 Model S (auto)"`) and appends it to the branch line of `startup_master_alert` (`"nap-ap1 - AP1 Model S (auto)"`, ASCII separator). NAP builds are not on a comma remote, so `startup_master_alert` is the one shown on device. A car with no saved calibration is already covered by openpilot's existing "Calibration in Progress" alert.

## 6. Settings UI

Both `selfdrive/ui/layouts/settings/nap.py` and `selfdrive/ui/mici/layouts/settings/nap.py`:

- Replace the disabled "Force Pre-AP" toggle with a **Car type** multi-option control (`Auto` / `Pre-AP` / `AP1`) bound to `NAPCarType`. Changes take effect at the next ignition (show that in the description).
- Status line: `Active car: <label> (<auto-detected|forced>)` from `NAPActiveProfile` + `NAPCarType`; `"none yet"` before the first detection.
- The car-type control and status line live in a new "Vehicle" section at the top. Longitudinal Control, Pedal, Radar, and iBooster section headers get a "(Pre-AP only)" suffix (the planner already gates `NAPAdaptiveAccel` / `NAPFollowDistance` on Pre-AP). No hiding.
- "Reset to defaults" resets `NAPCarType` to `Auto` and does not touch `/data/nap_profiles`.
- The existing Device → "Reset Calibration" is unchanged; it clears live learned keys, i.e. the active car only.

## 7. Error handling summary

| Failure | Behavior |
|---|---|
| `NAPCarType` unset/unreadable in `car_helpers` | Stock fingerprinting (manager always sets it on device) |
| No AP-ECU traffic (AP ECU unplugged/faulted) in AP1 car | Detected as Pre-AP; startup alert shows "Pre-AP Model S (auto)"; driver can force AP1 |
| Profile store missing/corrupt | Treated as empty; learners start fresh |
| Exception inside `switch_profile` | Logged; live keys untouched; startup continues |
| Crash between stash and restore | `NAPActiveProfile` already names the new car; at worst one drive starts with some stale learned values, which the learners' own sanity checks / recalibration correct |

## 8. Testing

**opendbc (WSL + uv; rebuild libsafety after safety edits):**
- `opendbc/car/tesla/tests/test_nap_detect.py`: AP1 fingerprint (`0x488` + `0x2B9` on bus 2) → HW1; same on bus 0 → HW1; Pre-AP fingerprint with pedal `GAS_SENSOR` on bus 2 → PREAP; empty fingerprint → PREAP; only `0x488` → PREAP.
- `opendbc/safety/tests/test_tesla_hw1.py`: §4.1 cases.
- HW1 carstate test: `turnSignalStalkState` mapping (0/1/2/3→0); AP-ECU messages parsed from bus 2.
- `create_body_controls` test: headlight/high-beam/wiper fields copied from the source frame; turn fields overridden; checksum valid.
- Existing Pre-AP suites (`test_tesla_preap*.py`, `opendbc/car/tesla/preap/tests/`) unchanged and passing.

**openpilot:**
- `selfdrive/car/tests/test_nap_profiles.py` with temp `Params` dirs: first-ever drive (no active profile); Pre-AP → AP1 → Pre-AP restores the original `CalibrationParams` byte-for-byte; AP1 first visit removes learned keys and keeps prefs; same car twice is a no-op; non-Tesla platform is a no-op; stash/active/restore ordering; `CarParamsPersistent` restored so the `CarParamsPrevRoute` copy matches.
- Process replay: Pre-AP routes unchanged with `NAPCarType: 1`.
- If downloadable, run Lukas' AP1 route `9227f2c54e175788/00000000--650ad7d3e0` through `nap_detect` and HW1 `CarState`; otherwise use the first AP1 log from the user's car.

## 9. Rollout and on-car validation

Implementation order (each step independently testable):

1. Branch setup; opendbc `nap-ap1` branch; `.gitmodules` branch.
2. Safety (§4.1).
3. AP1 car state and parsers (§4.2), controller and Lukas port (§4.3–4.4), radar scoping (§4.5).
4. `NAPCarType` + detection (§5.1–5.2).
5. Profiles + startup alert (§5.3–5.4).
6. Settings UI (§6).
7. On-car validation:
   - **Pre-AP regression drive** (`NAPCarType = Auto`): detected Pre-AP; pedal, radar, engagement, lane-change blinker unchanged; calibration valid immediately.
   - **AP1 parked, ignition on** (`NAPCarType = AP1` forced): CAN valid, no EPAS/DI/DAS faults, radar tracks present, AP ECU frames seen on bus 2 only while the relay is open.
   - **AP1 low speed**: openpilot steering, then longitudinal; engage/disengage via stalk, brake, and steering override; braking response in the first seconds after a gas override (Lukas' jerk ramp starts from zero jerk and reaches full limits after ~20 s at 25 Hz).
   - **AP1 highway**: lane change with blinker drive; automatic high beams still operate; no Autosteer fault chimes beyond the documented limitation.
   - **AP1 with `NAPCarType = Auto`**: detected AP1.
   - **Swap back to Pre-AP**: Pre-AP calibration and learned params restored (no "Calibration in Progress").

## 10. Risks and open items

- **AP1 car model:** assumed Model S. A Model X AP1 would need `TESLA_MODEL_X_HW1` in detection (the AP-ECU signature is the same).
- **Blinker drive on AP1:** whether the AP1 body controller honors an openpilot-sourced `DAS_bodyControls` exactly as Pre-AP does, and whether copying the DAS high-beam fields with up to 100 ms extra latency is noticeable. The panda gates on `controls_allowed` while the controller gates on `CC.enabled`, so at an engage/disengage edge one 2 Hz `DAS_bodyControls` frame may be missing or duplicated.
- **Stock Autosteer interaction** (§4.6): may produce IC warnings; acceptable for this milestone.
- **Jerk ramp after gas override:** ported as Lukas runs it; if braking feels late after a gas override on AP1, revisit `JERK_RAMP_RATE`.
- **Detection window:** the existing CAN fingerprint stops after ~1 s once no candidates remain; `0x2B9` at ~25 Hz and `0x488` at 50 Hz arrive well within that.
