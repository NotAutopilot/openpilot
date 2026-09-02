# Pre-AP MetaDrive sim

Bring up `selfdrived` on this tree without the car. The plant is comma's MetaDrive bridge in `tools/sim`. It is not log replay and it is not CARLA (comma removed CARLA).

Jack's car identity: VIN `5YJSA1H13EFP20460`, fingerprint `TESLA_MODEL_S_PREAP`, Comma Pedal `0x551`/`0x552`, Bosch radar on CAN1.

## Launch

From the repo root, venv active:

```bash
./tools/sim/launch_openpilot.sh
./tools/sim/run_bridge.py --plant --no-keyboard
```

`--plant` is the Pre-AP CAN plant without MetaDrive. `run_bridge.py` falls back to plant if `metadrive` is not installed.

On this VPS MetaDrive physics runs (`create_bridge(plant=False)`). RGB cameras do not: forked panda3d dies in simplepbr (`AttributeError: 'NoneType' object has no attribute 'set_shader'`, EGL DRI3 / `/dev/dri/card1` permission denied). Cameras need `METADRIVE_CAMERAS=1` and a working GL context in the same process (e.g. `xvfb-run` of a single-process smoke test). comma-minimal traffic spawn also lacks `render_vehicle`, so density stays 0; SimulatedCar packs one ego-relative Bosch lead at 40 m.

Headless boxes have no DISPLAY, so `launch_openpilot.sh` blocks `ui`.

With MetaDrive installed and a display:

```bash
./tools/sim/launch_openpilot.sh
./tools/sim/run_bridge.py
```

`launch_openpilot.sh` sets `FINGERPRINT=TESLA_MODEL_S_PREAP`, `SKIP_FW_QUERY=1`, `NOBOARD=1`, `SIMULATION=1`, and writes:

- `DisengageOnAccelerator=False` so a gas override is not a disengage chime
- `NAPPedalEnabled=True` with Jack's pedal calib (not the defaults that chatter `gasPressed`)
- `NAPRadarEnabled=True`

First milestone is process bring-up. Radar tracks from MetaDrive objects come after `selfdrived` is alive.

## What SimulatedCar packs

`tools/sim/lib/simulated_car.py` is the plant. 100 Hz on the party bus (0), Bosch frames on the radar bus (1):

| Address | Message | Why |
|---------|---------|-----|
| `0x155` | `ESP_B` | speed |
| `0x108` | `DI_torque1` | `DI_pedalPos` |
| `0x118` | `DI_torque2` | Drive, brake, speed |
| `0x20a` | `BrakeMessage` | driver brake |
| `0x368` | `DI_state` | cruise/cluster |
| `0x318` | `GTW_carState` | doors closed |
| `0x370` | `EPAS_sysStatus` | SAS angle, EAC idle/active, hands-on 0 |
| `0x00E` | `STW_ANGLHP_STAT` | steer rate |
| `0x045` | `STW_ACTN_RQ` | Tesla stalk (`SpdCtrlLvr_Stat`) |
| `0x552` | `GAS_SENSOR` | pedal feedback; rest raw 470 so panda `gas_pressed` stays false |
| `0x301` | `TeslaRadarSguInfo` | HWFail/SGUFail/dirty = 0 |
| `0x310+` | `RadarPointN_A/B` | empty tracks until sim objects are wired |

`0x551` is openpilot TX (`GAS_COMMAND`), not plant TX. `enableLongControl` is FSM state from a MAIN double-pull, not a CAN field.

`pandaStates` uses `safetyModel=teslaPreap` with `PREAP_FLAG_ENABLE_PEDAL | PREAP_FLAG_RADAR_EMULATION`.

## Stalk keys

Honda cruise-button values (MAIN=1, SET=3, RES=4) are wrong here. Tesla `STW_ACTN_RQ.SpdCtrlLvr_Stat`:

| Key | Stalk | Value |
|-----|-------|-------|
| 1 | up (SET / RES / +) | 16 |
| 2 | pull MAIN (engage) | 2 |
| 3 | push cancel | 1 |
| 4 | down (DECEL / -) | 32 |

Engage is a double pull of MAIN within 400 ms. The bridge does that automatically once `selfdriveState.engageable` is true. Gas override must not drop `enableLongControl` and must not chime (`DisengageOnAccelerator=False`).

## Tests

```bash
python -m pytest tools/sim/tests/test_simulated_car_preap.py -q
```

That suite packs CAN and parses it through `TESLA_MODEL_S_PREAP`. It does not launch MetaDrive.

The full MetaDrive loop (`tools/sim/tests/test_metadrive_bridge.py`) is slow and needs the simulator. Use it after bring-up, not as the first gate.

## Later

- Fill `SimulatorState.radar_points` from MetaDrive objects (`d_rel`, `y_rel`, `v_rel`) into Bosch slots
- Optional traffic (`traffic_density` is 0 today because it is expensive)
- Card/0x551 decode on the replay harness is still a separate path
