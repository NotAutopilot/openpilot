# Pre-AP Architecture

High-level design of NAP's Pre-AP Model S port. For safety-layer details see [safety-model.md](safety-model.md); for the engagement flow see [engagement.md](engagement.md).

## What makes pre-AP different

2012–2014 Model S cars have no Autopilot ECU, no harness relay, and a CAN layout distinct from HW1+ Teslas. Standard openpilot assumes a relay that routes CAN between stock AP and the panda — pre-AP has neither. The port is a spiritual successor to Tinkla (`boggyver/openpilot`, `tesla_unity_betaC3`).

## Layout

```
opendbc_repo/opendbc/
├── car/tesla/preap/          # Python car port
│   ├── interface.py          # CarParams + safety flags
│   ├── carstate.py           # CAN → CarState translation
│   ├── carcontroller.py      # CarControl → CAN (steering + pedal + stalk spoof)
│   ├── engagement.py         # stalk FSM (double-pull, brake, cancel)
│   ├── pedal_feedback.py     # Comma Pedal interceptor parser + health tracking
│   ├── constants.py          # accel profiles, pedal PID tunings
│   └── nap_conf.py           # runtime config (Params-backed)
├── safety/modes/tesla_preap.h  # standalone panda safety mode
└── dbc/tesla_preap.dbc         # CAN message definitions
```

## Major subsystems

### Standalone safety mode (`tesla_preap.h`)

Independent from `tesla_legacy.h` and upstream tesla safety. Its own hooks, counter/checksum, init, and RX/TX/fwd paths. Registered as `SAFETY_TESLA_PREAP`.

Key rationale for each nonstandard setting is commented in the file header. In short:

- `check_relay=false` + `disable_static_blocking=true` on every TX: pre-AP has no harness relay
- `ignore_checksum=true` + `ignore_counter=true` on RX: the pre-AP EPAS checksum algorithm is not fully verified across firmware versions; mismatched validation caused a silent 21-second steering dropout during testing

All actual safety checks are active (steering angle/rate limits, hands-on disengage, EPAS error codes, door/gear, stalk echo-filtered cancel, AEB block).

### Radar (optional)

Bosch radar with GTW (GateWay ECU) emulation. The panda forwards chassis-bus traffic with a radar-position rewrite when `RADAR_BEHIND_NOSECONE` is set. The emulation hook is `rx_all` (not `rx`) because it must see every CAN frame, not just whitelisted ones.

### Comma Pedal (optional)

Pedal interceptor on bus 0 or bus 2 (wiring varies). Interface flag `ENABLE_PEDAL` gates both TX whitelist (`0x551` GAS_COMMAND) and RX route (`0x552` GAS_SENSOR). Without the pedal, pre-AP uses stock Tesla CC via stalk-spoof engage/cancel — see [engagement.md](engagement.md).

### EPAS firmware ownership

The NAP settings panel exposes EPAS firmware switching. Steering only engages when the EPAS variant is known-good for our compute_checksum. See the on-device settings for supported firmware.

## Vehicle model

Physical parameters (mass, wheelbase, steer ratio) come from `CarSpecs` in `opendbc_repo/opendbc/car/tesla/values.py`. Pre-AP is the same platform as HW1/HW2/HW3 Model S; we inherit the HW3 values. Do **not** override in `preap/interface.py` — the framework adds `STD_CARGO_KG` automatically, and overriding double-counts it.

Panda steering params (`PREAP_STEERING_PARAMS` in `tesla_preap.h`) must match the Python-layer `VehicleModel(TESLA_MODEL_S_PREAP)`. Verify:

```python
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.vehicle_model import VehicleModel, calc_slip_factor
VM = VehicleModel(CarInterface.get_non_essential_params("TESLA_MODEL_S_PREAP"))
print(calc_slip_factor(VM))  # must match slip_factor in tesla_preap.h
```

Any divergence causes the steering angle command limiter to disagree between layers — silent control ceiling.

## Upstream relationship

NAP tracks upstream openpilot closely. We fork `opendbc` because the pre-AP DBCs, car port, and safety mode all live there and are NAP-only. We track upstream `panda` directly — pre-AP safety changes go through the `opendbc_repo/opendbc/safety/` path that panda pulls in as a submodule.

See [Tinkla](https://github.com/boggyver/openpilot/tree/tesla_unity_betaC3) and [xnor-tech/openpilot](https://github.com/xnor-tech/openpilot) for the prior art NAP builds on.
